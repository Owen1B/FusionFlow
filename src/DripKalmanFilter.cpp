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
 * @brief Constructor for the DripKalmanFilter class.
 *
 * @param drip_rate_sigma_a Process noise parameter for the drip rate Kalman filter (std dev of unmodeled drip acceleration).
 *                          This affects the filter's ability to track true changes in drip rate versus smoothing noise.
 * @param drip_rate_R_noise Measurement noise variance for the drip rate Kalman filter (for the raw drip rate calculated from drop_count/time_interval).
 *                          Reflects the reliability of the raw drip rate calculation.
 * @param wpd_Q_noise Process noise variance for the Weight-Per-Drop (WPD) calibration Kalman filter.
 *                    Assumes the true WPD might also undergo minor changes or drift during calibration.
 * @param wpd_R_noise Measurement noise variance for the WPD calibration Kalman filter.
 *                    Reflects the reliability of the WPD measurement calculated from (weight_change / drop_count).
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

    // Initialize new member variables
    known_initial_total_weight_g = 0.0f;
    total_drops_for_volume_calc = 0;
    initial_weight_for_volume_calc_set = false;
}

/**
 * @brief Initializes or resets the filter's state.
 *
 * @param initial_drip_rate_dps Best initial estimate for the drip rate (drips/sec).
 * @param initial_wpd_g_per_drip Best initial estimate for the weight per drop (g/drip).
 *                               If <= 0, a default value is calculated based on drops_per_ml and density_g_per_ml.
 * @param drops_per_ml Number of drops per mL, used for default WPD calculation.
 * @param density_g_per_ml Liquid density (g/mL), used for default WPD calculation and flow rate unit conversion.
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
 * @brief Updates the filter state with new sensor data.
 *
 * @param measured_drip_rate The measured drip rate (drips/sec).
 * @param time_interval_s Time interval since the last update() call (in seconds). Must be > 0.
 * @param weight_sensor_change_g The change in weight measured by the weight sensor (in g).
 *                               This parameter is only used for WPD calibration when calibrating_wpd is true.
 *                               It's typically (previous_weight - current_weight), so it's positive during consumption.
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


}

/**
 * @brief Calculates and returns the flow rate based on the current filtered drip rate and calibrated WPD.
 *
 * @return float Flow rate in grams per second (g/sec). Returns 0 if the calibrated WPD is not sensible (e.g., too small).
 */
float DripKalmanFilter::getFlowRateGramsPerSecond() const {
    if (wpd_estimate_g_per_drip <= 1e-6f) return 0.0f; // Avoid issues if WPD is not sensible
    return x_drip_rate_state[0] * wpd_estimate_g_per_drip;
}

/**
 * @brief Calculates and returns the flow rate in mL/hour based on the current flow rate (g/s) and liquid density.
 *
 * @return float Flow rate in milliliters per hour (mL/hour). Returns 0 if WPD or liquid density are not sensible.
 */
float DripKalmanFilter::getFlowRateMlPerHour() const {
    if (wpd_estimate_g_per_drip <= 1e-6f || current_liquid_density_g_per_ml <= 1e-6f) return 0.0f;
    float grams_per_second = getFlowRateGramsPerSecond();
    float ml_per_second = grams_per_second / current_liquid_density_g_per_ml;
    return ml_per_second * 3600.0f;
}

/** 
 * @brief Starts the Weight-Per-Drop (WPD) calibration process.
 * Calling this function enables the filter to use weight change and drip count 
 * in subsequent update() calls to refine its internal WPD estimate.
 * It also resets the WPD estimation covariance to reflect a higher uncertainty,
 * making it more receptive to new calibration data.
 */
void DripKalmanFilter::startWpdCalibration() {
    calibrating_wpd = true;
    // Reset WPD covariance to be more uncertain at start of calibration
    P_wpd_cov = 0.25f; 
}

// --- Implementation of methods for remaining volume calculation based on drip count ---
void DripKalmanFilter::setInitialLiquidWeightForVolumeCalc(float initial_weight_g) {
    known_initial_total_weight_g = initial_weight_g;
    total_drops_for_volume_calc = 0; // Reset cumulative drip count
    initial_weight_for_volume_calc_set = true;
}

void DripKalmanFilter::updateTotalDropsForVolumeCalc(int drops_in_latest_period) {
    if (initial_weight_for_volume_calc_set) { // Only accumulate after initial weight is set
        if (drops_in_latest_period > 0) { // Ensure drops are positive
             total_drops_for_volume_calc += (unsigned long)drops_in_latest_period;
        }
    }
}

float DripKalmanFilter::getInfusedWeightByDropsG() const {
    if (!initial_weight_for_volume_calc_set) {
        return 0.0f; // If initial weight is not set, assume no infusion has occurred
    }
    // Use a reasonable lower bound for WPD to prevent issues if wpd_estimate_g_per_drip is zero or too small
    float current_wpd = (wpd_estimate_g_per_drip > 0.001f) ? wpd_estimate_g_per_drip : (1.0f / current_default_drops_per_ml) * current_liquid_density_g_per_ml;
    if (current_wpd < 0.001f) current_wpd = 0.05f; // Final safeguard, 0.05g/drip (20drip/ml)

    return (float)total_drops_for_volume_calc * current_wpd;
}

float DripKalmanFilter::getRemainingWeightByDropsG() const {
    if (!initial_weight_for_volume_calc_set) {
        return 0.0f; // Or return a special value, e.g., known_initial_total_weight_g if it was initialized
                     // Or let the caller check initial_weight_for_volume_calc_set
    }
    float infused_weight = getInfusedWeightByDropsG();
    float remaining = known_initial_total_weight_g - infused_weight;
    return (remaining > 0.0f) ? remaining : 0.0f;
}

void DripKalmanFilter::calibrateWpdByTotal(float current_weight) {
    if (!calibrating_wpd || !initial_weight_for_volume_calc_set) return;
    if (total_drops_for_volume_calc < 5) return; // Don't calibrate with too few drops to avoid initial instability

    float delta_weight = known_initial_total_weight_g - current_weight;
    if (delta_weight < 0.01f) return; // Don't calibrate if change is too small

    float measured_wpd = delta_weight / (float)total_drops_for_volume_calc;
    if (measured_wpd < 0.01f || measured_wpd > 0.2f) return; // Outlier protection

    // 1D Kalman Filter
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