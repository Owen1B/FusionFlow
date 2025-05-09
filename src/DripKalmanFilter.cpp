#include "DripKalmanFilter.h" // Ensure the header is included for class definition

#if !defined(PIO_UNIT_TESTING) && !defined(UNIT_TEST) && defined(ARDUINO_ARCH_ESP32) // More specific for actual Arduino ESP32 compile
#include <Arduino.h>
#endif

#include <math.h> // For fabsf
#include <stdint.h> // Ensure uint32_t is available

// Helper for time if needed, assumes Arduino environment
// For non-Arduino, you might need to pass absolute time or manage dt differently.
#if (defined(PIO_UNIT_TESTING) || defined(UNIT_TEST)) && !defined(ARDUINO_ARCH_ESP32)
    // Basic millis() placeholder for native unit testing environment
    // This mock is very basic. If your filter's logic becomes dependent on 
    // an accurate or advancing millis(), you might need a more sophisticated mock.
    static uint32_t mock_millis_count = 0;
    uint32_t millis() { 
        // For some tests, a static time might be okay if dt is always passed in.
        // For others that might implicitly rely on advancing time, increment it.
        // Adjust increment as needed or make it controllable by tests.
        mock_millis_count += 10; // Simulate some time passing
        return mock_millis_count; 
    }
#endif

/**
 * @brief DripKalmanFilter 类的构造函数。
 *
 * @param drip_rate_sigma_a 滴速卡尔曼滤波器的过程噪声参数 (未建模的滴速加速度的标准差)。
 *                          影响滤波器对滴速真实变化的跟踪能力和估计的平滑度。
 * @param drip_rate_R_noise 滴速卡尔曼滤波器的测量噪声方差 (针对由 drop_count/time_interval 计算的原始滴速)。
 *                          反映了原始滴速计算的可靠性。
 * @param wpd_Q_noise 每滴重量(WPD)校准用卡尔曼滤波器的过程噪声方差。
 *                      假设在校准期间，真实的每滴重量也可能发生微小变化或漂移。
 * @param wpd_R_noise 每滴重量(WPD)校准用卡尔曼滤波器的测量噪声方差。
 *                      反映了 (weight_change / drop_count) 这个WPD测量计算值的可靠性。
 */
DripKalmanFilter::DripKalmanFilter(float drip_rate_sigma_a, float drip_rate_R_noise,
                                   float wpd_Q_noise, float wpd_R_noise) {
    this->Q_drip_rate_sigma_a = drip_rate_sigma_a;
    this->R_drip_rate_noise = drip_rate_R_noise;
    this->Q_wpd_process_noise = wpd_Q_noise;
    this->R_wpd_measurement_noise = wpd_R_noise;

    // Initialize Drip Rate KF states
    x_drip_rate_state[0] = 0.0f; // initial drip rate (drips/sec)
    x_drip_rate_state[1] = 0.0f; // initial drip rate change (drips/sec^2)
    P_drip_rate_cov[0][0] = 1.0f; P_drip_rate_cov[0][1] = 0.0f;
    P_drip_rate_cov[1][0] = 0.0f; P_drip_rate_cov[1][1] = 1.0f;

    // Initialize WPD states
    wpd_estimate_g_per_drip = (1.0f / DEFAULT_DROPS_PER_ML) * DEFAULT_LIQUID_DENSITY_G_PER_ML;
    P_wpd_cov = 1.0f;

    calibrating_wpd = false;
    current_default_drops_per_ml = DEFAULT_DROPS_PER_ML;
    current_liquid_density_g_per_ml = DEFAULT_LIQUID_DENSITY_G_PER_ML;
}

/**
 * @brief 初始化或重置滤波器的状态。
 *
 * @param initial_drip_rate_dps 初始滴速的最佳估计值 (drips/sec)。
 * @param initial_wpd_g_per_drip 初始每滴重量的最佳估计值 (g/drip)。
 *                               如果 <= 0, 则使用基于 drops_per_ml 和 density_g_per_ml 的默认计算值。
 * @param drops_per_ml 每毫升的滴数，用于计算默认WPD。
 * @param density_g_per_ml 液体密度 (g/mL)，用于计算默认WPD和流速单位转换。
 */
void DripKalmanFilter::init(float initial_drip_rate_dps, 
                            float initial_wpd_g_per_drip,
                            int drops_per_ml,
                            float density_g_per_ml) {
    x_drip_rate_state[0] = initial_drip_rate_dps;
    x_drip_rate_state[1] = 0.0f; // Assume no initial rate change
    P_drip_rate_cov[0][0] = 0.25f; P_drip_rate_cov[0][1] = 0.0f;
    P_drip_rate_cov[1][0] = 0.0f; P_drip_rate_cov[1][1] = 0.25f;

    current_default_drops_per_ml = (float)drops_per_ml;
    current_liquid_density_g_per_ml = density_g_per_ml;

    if (initial_wpd_g_per_drip <= 0.0f) {
        wpd_estimate_g_per_drip = (1.0f / current_default_drops_per_ml) * current_liquid_density_g_per_ml;
    } else {
        wpd_estimate_g_per_drip = initial_wpd_g_per_drip;
    }
    P_wpd_cov = 0.01f; // Reasonably confident in initial WPD if provided or calculated from defaults

    calibrating_wpd = false;
}

/**
 * @brief 使用新的传感器数据更新滤波器状态。
 *
 * @param drop_count 在 time_interval_s 时间内检测到的滴数。
 * @param time_interval_s 距离上一次调用 update() 的时间间隔 (秒)。必须大于0。
 * @param weight_sensor_change_g 从重量传感器测得的实际重量变化值 (g)。
 *                               此参数仅在 calibrating_wpd 为 true 时用于WPD校准。
 *                               通常是 (上周期重量 - 本周期重量)，消耗时为正。
 */
void DripKalmanFilter::update(int drop_count, float time_interval_s, float weight_sensor_change_g) {
    if (time_interval_s <= 1e-6f) { // Avoid division by zero or extremely small dt
        // Potentially just return or log an error, cannot update drip rate
        return;
    }

    // === 1. Update Drip Rate Kalman Filter ===
    float measured_drip_rate = (float)drop_count / time_interval_s;

    // State Transition Matrix (F_dr)
    float F_dr[2][2] = {{1.0f, time_interval_s}, {0.0f, 1.0f}};

    // Process Noise Covariance Matrix (Q_dr)
    float dt2 = time_interval_s * time_interval_s;
    float dt3 = dt2 * time_interval_s;
    float dt4 = dt3 * time_interval_s;
    float sigma_a_sq_dr = Q_drip_rate_sigma_a * Q_drip_rate_sigma_a;
    float Q_dr_matrix[2][2];
    Q_dr_matrix[0][0] = (dt4 / 4.0f) * sigma_a_sq_dr;
    Q_dr_matrix[0][1] = (dt3 / 2.0f) * sigma_a_sq_dr;
    Q_dr_matrix[1][0] = (dt3 / 2.0f) * sigma_a_sq_dr;
    Q_dr_matrix[1][1] = dt2 * sigma_a_sq_dr;

    // Predict state: x_pred_dr = F_dr * x_drip_rate_state
    float x_pred_dr[2];
    x_pred_dr[0] = F_dr[0][0] * x_drip_rate_state[0] + F_dr[0][1] * x_drip_rate_state[1];
    x_pred_dr[1] = F_dr[1][0] * x_drip_rate_state[0] + F_dr[1][1] * x_drip_rate_state[1];

    // Predict estimate covariance: P_pred_dr = F_dr * P_drip_rate_cov * F_dr_transpose + Q_dr_matrix
    float FP_dr[2][2];
    FP_dr[0][0] = F_dr[0][0]*P_drip_rate_cov[0][0] + F_dr[0][1]*P_drip_rate_cov[1][0];
    FP_dr[0][1] = F_dr[0][0]*P_drip_rate_cov[0][1] + F_dr[0][1]*P_drip_rate_cov[1][1];
    FP_dr[1][0] = F_dr[1][0]*P_drip_rate_cov[0][0] + F_dr[1][1]*P_drip_rate_cov[1][0];
    FP_dr[1][1] = F_dr[1][0]*P_drip_rate_cov[0][1] + F_dr[1][1]*P_drip_rate_cov[1][1];
    float P_pred_dr[2][2];
    P_pred_dr[0][0] = FP_dr[0][0]*F_dr[0][0] + FP_dr[0][1]*F_dr[0][1];
    P_pred_dr[0][1] = FP_dr[0][0]*F_dr[1][0] + FP_dr[0][1]*F_dr[1][1]; 
    P_pred_dr[1][0] = FP_dr[1][0]*F_dr[0][0] + FP_dr[1][1]*F_dr[0][1];
    P_pred_dr[1][1] = FP_dr[1][0]*F_dr[1][0] + FP_dr[1][1]*F_dr[1][1];
    P_pred_dr[0][0] += Q_dr_matrix[0][0]; P_pred_dr[0][1] += Q_dr_matrix[0][1];
    P_pred_dr[1][0] += Q_dr_matrix[1][0]; P_pred_dr[1][1] += Q_dr_matrix[1][1];

    // Kalman Gain (K_dr) - H_dr = [1, 0]
    float S_inv_dr = P_pred_dr[0][0] + R_drip_rate_noise;
    if (fabsf(S_inv_dr) < 1e-9f) S_inv_dr = (S_inv_dr >= 0) ? 1e-9f : -1e-9f;
    S_inv_dr = 1.0f / S_inv_dr;
    float K_dr[2];
    K_dr[0] = P_pred_dr[0][0] * S_inv_dr;
    K_dr[1] = P_pred_dr[1][0] * S_inv_dr;

    // Update state: x_drip_rate_state = x_pred_dr + K_dr * (measured_drip_rate - x_pred_dr[0])
    float innovation_dr = measured_drip_rate - x_pred_dr[0];
    x_drip_rate_state[0] = x_pred_dr[0] + K_dr[0] * innovation_dr;
    x_drip_rate_state[1] = x_pred_dr[1] + K_dr[1] * innovation_dr;

    // Update covariance: P_drip_rate_cov = (I - K_dr * H_dr) * P_pred_dr
    float I_KH_dr[2][2];
    I_KH_dr[0][0] = 1.0f - K_dr[0]; I_KH_dr[0][1] = 0.0f;
    I_KH_dr[1][0] = -K_dr[1];       I_KH_dr[1][1] = 1.0f;
    P_drip_rate_cov[0][0] = I_KH_dr[0][0]*P_pred_dr[0][0] + I_KH_dr[0][1]*P_pred_dr[1][0];
    P_drip_rate_cov[0][1] = I_KH_dr[0][0]*P_pred_dr[0][1] + I_KH_dr[0][1]*P_pred_dr[1][1];
    P_drip_rate_cov[1][0] = I_KH_dr[1][0]*P_pred_dr[0][0] + I_KH_dr[1][1]*P_pred_dr[1][0];
    P_drip_rate_cov[1][1] = I_KH_dr[1][0]*P_pred_dr[0][1] + I_KH_dr[1][1]*P_pred_dr[1][1];

    // === 2. Update Weight Per Drop (WPD) if calibrating === 
    if (calibrating_wpd && drop_count > 0 && weight_sensor_change_g > 1e-3f /* 的미있는 무게 변화 */) {
        float measured_wpd = fabsf(weight_sensor_change_g) / (float)drop_count;
        
        // 1D Kalman Filter for WPD
        // Predict (WPD is assumed constant, so prediction is current estimate)
        float wpd_pred_estimate = wpd_estimate_g_per_drip;
        float P_wpd_pred_cov = P_wpd_cov + Q_wpd_process_noise; // Add process noise

        // Update
        float S_wpd_inv = P_wpd_pred_cov + R_wpd_measurement_noise;
        if (fabsf(S_wpd_inv) < 1e-9f) S_wpd_inv = (S_wpd_inv >=0)? 1e-9f : -1e-9f;
        S_wpd_inv = 1.0f / S_wpd_inv;
        float K_wpd = P_wpd_pred_cov * S_wpd_inv;

        wpd_estimate_g_per_drip = wpd_pred_estimate + K_wpd * (measured_wpd - wpd_pred_estimate);
        P_wpd_cov = (1.0f - K_wpd) * P_wpd_pred_cov;
    }
}

/**
 * @brief 计算并返回基于当前滤波后滴速和校准后每滴重量的流速。
 *
 * @return float 流速 (g/sec)。 如果校准的每滴重量不合理（过小），则返回0。
 */
float DripKalmanFilter::getFlowRateGramsPerSecond() const {
    if (wpd_estimate_g_per_drip <= 1e-6f) return 0.0f; // Avoid issues if WPD is not sensible
    return x_drip_rate_state[0] * wpd_estimate_g_per_drip;
}

/**
 * @brief 计算并返回基于当前流速(g/s)和液体密度的流速(mL/h)。
 *
 * @return float 流速 (mL/hour)。如果每滴重量或液体密度不合理，则返回0。
 */
float DripKalmanFilter::getFlowRateMlPerHour() const {
    if (wpd_estimate_g_per_drip <= 1e-6f || current_liquid_density_g_per_ml <= 1e-6f) return 0.0f;
    float grams_per_second = getFlowRateGramsPerSecond();
    float ml_per_second = grams_per_second / current_liquid_density_g_per_ml;
    return ml_per_second * 3600.0f;
}

/** 
 * @brief 启动每滴重量(WPD)的校准过程。
 * 调用此函数会使滤波器在后续的 update() 调用中，
 * 使用传入的重量变化和滴数来更新其内部的WPD估计。
 * 它还会重置WPD估计的协方差，以表示对新校准数据的更高接纳度。
 */
void DripKalmanFilter::startWpdCalibration() {
    calibrating_wpd = true;
    // Reset WPD covariance to be more uncertain at start of calibration
    P_wpd_cov = 0.25f; 
} 