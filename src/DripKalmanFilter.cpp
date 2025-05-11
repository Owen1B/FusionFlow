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

    // 初始化新增的成员变量
    known_initial_total_weight_g = 0.0f;
    total_drops_for_volume_calc = 0;
    initial_weight_for_volume_calc_set = false;
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
    x_drip_rate_state[1] = 0.0f;
    P_drip_rate_cov[0][0] = 0.25f; P_drip_rate_cov[0][1] = 0.0f;
    P_drip_rate_cov[1][0] = 0.0f; P_drip_rate_cov[1][1] = 0.25f;

    current_default_drops_per_ml = (float)drops_per_ml;
    current_liquid_density_g_per_ml = density_g_per_ml;

    if (initial_wpd_g_per_drip <= 0.0f) {
        wpd_estimate_g_per_drip = (1.0f / current_default_drops_per_ml) * current_liquid_density_g_per_ml;
    } else {
        wpd_estimate_g_per_drip = initial_wpd_g_per_drip;
    }
    P_wpd_cov = 0.01f;

    if (wpd_estimate_g_per_drip < 0.04f) {
        wpd_estimate_g_per_drip = 0.04f;
    } else if (wpd_estimate_g_per_drip > 0.06f) {
        wpd_estimate_g_per_drip = 0.06f;
    }

    calibrating_wpd = false;
}

/**
 * @brief 使用新的传感器数据更新滤波器状态。
 *
 * @param measured_drip_rate 测量得到的滴速 (drips/sec)。
 * @param time_interval_s 距离上一次调用 update() 的时间间隔 (秒)。必须大于0。
 * @param weight_sensor_change_g 从重量传感器测得的实际重量变化值 (g)。
 *                               此参数仅在 calibrating_wpd 为 true 时用于WPD校准。
 *                               通常是 (上周期重量 - 本周期重量)，消耗时为正。
 */
void DripKalmanFilter::update(float measured_drip_rate, float time_interval_s, float weight_sensor_change_g) {
    if (time_interval_s <= 1e-6f) {
        return;
    }

    // === 1. Update Drip Rate Kalman Filter ===
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
    // if (calibrating_wpd && measured_drip_rate > 1e-3f && weight_sensor_change_g > 1e-3f) {
    //     // 计算测量得到的WPD
    //     float measured_wpd = fabsf(weight_sensor_change_g) / (measured_drip_rate * time_interval_s);
        
    //     // 1D Kalman Filter for WPD
    //     float wpd_pred_estimate = wpd_estimate_g_per_drip;
    //     float P_wpd_pred_cov = P_wpd_cov + Q_wpd_process_noise;

    //     float S_wpd_inv = P_wpd_pred_cov + R_wpd_measurement_noise;
    //     if (fabsf(S_wpd_inv) < 1e-9f) S_wpd_inv = (S_wpd_inv >=0)? 1e-9f : -1e-9f;
    //     S_wpd_inv = 1.0f / S_wpd_inv;
    //     float K_wpd = P_wpd_pred_cov * S_wpd_inv;

    //     wpd_estimate_g_per_drip = wpd_pred_estimate + K_wpd * (measured_wpd - wpd_pred_estimate);
    //     P_wpd_cov = (1.0f - K_wpd) * P_wpd_pred_cov;

    //     if (wpd_estimate_g_per_drip < 0.04f) {
    //         wpd_estimate_g_per_drip = 0.04f;
    //     } else if (wpd_estimate_g_per_drip > 0.06f) {
    //         wpd_estimate_g_per_drip = 0.06f;
    //     }
    // }
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

// --- 实现新增的基于滴数计算剩余量的方法 ---
void DripKalmanFilter::setInitialLiquidWeightForVolumeCalc(float initial_weight_g) {
    known_initial_total_weight_g = initial_weight_g;
    total_drops_for_volume_calc = 0; // 重置累计滴数
    initial_weight_for_volume_calc_set = true;
}

void DripKalmanFilter::updateTotalDropsForVolumeCalc(int drops_in_latest_period) {
    if (initial_weight_for_volume_calc_set) { // 只有在初始重量设置后才累计
        if (drops_in_latest_period > 0) { // 确保滴数为正
             total_drops_for_volume_calc += (unsigned long)drops_in_latest_period;
        }
    }
}

float DripKalmanFilter::getInfusedWeightByDropsG() const {
    if (!initial_weight_for_volume_calc_set) {
        return 0.0f; // 如果初始重量未设置，则认为未输注
    }
    // 使用一个合理的WPD下限，防止wpd_estimate_g_per_drip为0或过小导致问题
    float current_wpd = (wpd_estimate_g_per_drip > 0.001f) ? wpd_estimate_g_per_drip : (1.0f / current_default_drops_per_ml) * current_liquid_density_g_per_ml;
    if (current_wpd < 0.001f) current_wpd = 0.05f; // 最后的保障，0.05g/drip (20drip/ml)

    return (float)total_drops_for_volume_calc * current_wpd;
}

float DripKalmanFilter::getRemainingWeightByDropsG() const {
    if (!initial_weight_for_volume_calc_set) {
        return 0.0f; // 或者返回一个特殊值，例如 known_initial_total_weight_g (如果它被初始化了)
                     // 或者让调用者检查 initial_weight_for_volume_calc_set
    }
    float infused_weight = getInfusedWeightByDropsG();
    float remaining = known_initial_total_weight_g - infused_weight;
    return (remaining > 0.0f) ? remaining : 0.0f;
}

void DripKalmanFilter::calibrateWpdByTotal(float current_weight) {
    if (!calibrating_wpd || !initial_weight_for_volume_calc_set) return;
    if (total_drops_for_volume_calc < 5) return; // 滴数太少不校准，防止初期不稳定

    float delta_weight = known_initial_total_weight_g - current_weight;
    if (delta_weight < 0.01f) return; // 变化太小不校准

    float measured_wpd = delta_weight / (float)total_drops_for_volume_calc;
    if (measured_wpd < 0.01f || measured_wpd > 0.2f) return; // 异常值保护

    // 1D卡尔曼滤波
    float wpd_pred_estimate = wpd_estimate_g_per_drip;
    float P_wpd_pred_cov = P_wpd_cov + Q_wpd_process_noise;
    float S_wpd_inv = P_wpd_pred_cov + R_wpd_measurement_noise;
    if (fabsf(S_wpd_inv) < 1e-9f) S_wpd_inv = (S_wpd_inv >=0)? 1e-9f : -1e-9f;
    S_wpd_inv = 1.0f / S_wpd_inv;
    float K_wpd = P_wpd_pred_cov * S_wpd_inv;

    wpd_estimate_g_per_drip = wpd_pred_estimate + K_wpd * (measured_wpd - wpd_pred_estimate);
    P_wpd_cov = (1.0f - K_wpd) * P_wpd_pred_cov;

    if (wpd_estimate_g_per_drip < 0.04f) wpd_estimate_g_per_drip = 0.04f;
    if (wpd_estimate_g_per_drip > 0.06f) wpd_estimate_g_per_drip = 0.06f;
} 