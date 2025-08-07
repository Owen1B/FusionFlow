#include "DataFusion.h"
#include <math.h> // 用于 fabsf (可能不需要，但包含无害)

/**
 * @brief Constructor for the DataFusion class.
 *
 * @param q_flow Process noise variance for flow rate.
 * @param r_weight_flow Measurement noise variance for flow rate from weight sensor.
 * @param r_drip_flow Measurement noise variance for flow rate from drip sensor.
 * @param q_weight Process noise variance for remaining weight.
 * @param r_weight_weight Measurement noise variance for remaining weight from weight sensor.
 * @param r_drip_weight Measurement noise variance for remaining weight from drip sensor.
 */
DataFusion::DataFusion(float q_flow, float r_weight_flow, float r_drip_flow,
                       float q_weight, float r_weight_weight, float r_drip_weight) {
    Q_flow_process_noise = q_flow;
    R_weight_sensor_flow_noise = r_weight_flow;
    R_drip_sensor_flow_noise = r_drip_flow;

    Q_weight_process_noise = q_weight;
    R_weight_sensor_weight_noise = r_weight_weight;
    R_drip_sensor_weight_noise = r_drip_weight;

    // 初始化融合后流速的状态估计和协方差
    x_fused_flow_rate_gps = 0.0f; 
    P_fused_flow_cov = 1.0f;      

    // 初始化融合后剩余重量的状态估计和协方差
    x_fused_remaining_weight_g = 0.0f; 
    P_fused_weight_cov = 10.0f; // Initial uncertainty for remaining weight can be larger
}

/**
 * @brief Initializes or resets the state of the data fusion filter.
 *
 * @param initial_fused_flow_rate_gps Initial estimate for the fused flow rate (g/s).
 * @param initial_fused_remaining_weight_g Initial estimate for the fused remaining weight (g).
 */
void DataFusion::init(float initial_fused_flow_rate_gps, float initial_fused_remaining_weight_g) {
    x_fused_flow_rate_gps = initial_fused_flow_rate_gps;
    P_fused_flow_cov = 0.1f; 

    x_fused_remaining_weight_g = initial_fused_remaining_weight_g;
    P_fused_weight_cov = 1.0f; // Uncertainty is reduced upon explicit initialization
}

/**
 * @brief Helper function for the standard 1D Kalman filter update step.
 *
 * @param x Reference to the state variable to be updated (e.g., x_fused_flow_rate_gps).
 * @param P Reference to the covariance of the state estimation error (e.g., P_fused_flow_cov).
 * @param measurement The current measurement value (e.g., flow rate from a sensor).
 * @param measurement_noise_R The noise variance of this measurement.
 */
static void kalman_update_1d(float& x, float& P, float measurement, float measurement_noise_R) {
    // Safety check: if measurement noise is extremely small (implying very high confidence 
    // in the measurement, or R is not set correctly), or P itself is very small, 
    // it might lead to numerical instability or K close to 1. This primarily prevents division by zero if R is 0.
    if (measurement_noise_R < 1e-9f) return; 

    // Calculate Kalman Gain K
    // K = P_predicted / (P_predicted + R_measurement)
    // Here, the incoming P is already the predicted P_pred (as the time update is done in the main update function)
    float K = P / (P + measurement_noise_R);
    
    // Update state estimate
    // x_new = x_predicted + K * (measurement - x_predicted)
    x = x + K * (measurement - x);
    
    // Update estimation error covariance
    // P_new = (1 - K) * P_predicted
    P = (1.0f - K) * P;
}

/**
 * @brief Updates the fused flow rate using estimates from two sensors.
 *
 * @param flow_from_weight_sensor_gps Flow rate estimate from WeightKalmanFilter (g/s).
 * @param flow_from_drip_sensor_gps Flow rate estimate from DripKalmanFilter (g/s).
 * @param weight_from_weight_sensor_g Current weight estimate from WeightKalmanFilter (g).
 * @param weight_from_drip_sensor_g Remaining weight estimate from DripKalmanFilter (g).
 * @param dt Time interval since the last update() call (in seconds), used for process noise accumulation.
 */
void DataFusion::update(float flow_from_weight_sensor_gps, 
                        float flow_from_drip_sensor_gps, 
                        float weight_from_weight_sensor_g,
                        float weight_from_drip_sensor_g,
                        float dt) {
    // Safety check: if the time interval is invalid, skip the prediction step.
    if (dt <= 1e-6f) return;

    // === 1. Prediction Step ===
    // --- Flow Rate Prediction ---
    float x_pred_flow = x_fused_flow_rate_gps;
    float P_pred_flow = P_fused_flow_cov + Q_flow_process_noise * dt;
    x_fused_flow_rate_gps = x_pred_flow;
    P_fused_flow_cov = P_pred_flow;

    // --- Remaining Weight Prediction ---
    // Simple prediction: assume the fused remaining weight decreases based on the current fused flow rate.
    // This couples the weight prediction with the flow rate prediction.
    // x_pred_weight = x_current_weight - x_current_flow * dt
    // This prediction method doesn't have a standard Kalman process noise model,
    // so we mainly rely on measurement updates. However, we can still add process noise to the covariance.
    float x_pred_weight = x_fused_remaining_weight_g - x_fused_flow_rate_gps * dt; 
    if (x_pred_weight < 0) x_pred_weight = 0; // Weight cannot be negative
    float P_pred_weight = P_fused_weight_cov + Q_weight_process_noise * dt; // Q_weight_process_noise could also include terms from flow rate uncertainty
    x_fused_remaining_weight_g = x_pred_weight;
    P_fused_weight_cov = P_pred_weight;


    // === 2. Update Step (sequentially use each sensor's estimate as a measurement) ===

    // --- Flow Rate Update ---
    if (R_weight_sensor_flow_noise > 1e-9f) {
        kalman_update_1d(x_fused_flow_rate_gps, P_fused_flow_cov, flow_from_weight_sensor_gps, R_weight_sensor_flow_noise);
    }
    if (R_drip_sensor_flow_noise > 1e-9f) {
        kalman_update_1d(x_fused_flow_rate_gps, P_fused_flow_cov, flow_from_drip_sensor_gps, R_drip_sensor_flow_noise);
    }

    // --- Remaining Weight Update ---
    if (R_weight_sensor_weight_noise > 1e-9f) {
        kalman_update_1d(x_fused_remaining_weight_g, P_fused_weight_cov, weight_from_weight_sensor_g, R_weight_sensor_weight_noise);
    }
    if (R_drip_sensor_weight_noise > 1e-9f) {
        kalman_update_1d(x_fused_remaining_weight_g, P_fused_weight_cov, weight_from_drip_sensor_g, R_drip_sensor_weight_noise);
    }

    // Ensure fused remaining weight is not negative
    if (x_fused_remaining_weight_g < 0.0f) {
        x_fused_remaining_weight_g = 0.0f;
    }
} 

// --- Implementation of new methods for fast convergence ---
void DataFusion::setFlowMeasurementNoises(float r_from_weight, float r_from_drip) {
    R_weight_sensor_flow_noise = r_from_weight;
    R_drip_sensor_flow_noise = r_from_drip;
}

void DataFusion::getFlowMeasurementNoises(float& r_from_weight, float& r_from_drip) const {
    r_from_weight = R_weight_sensor_flow_noise;
    r_from_drip = R_drip_sensor_flow_noise;
}

void DataFusion::setWeightMeasurementNoises(float r_from_weight, float r_from_drip) {
    R_weight_sensor_weight_noise = r_from_weight;
    R_drip_sensor_weight_noise = r_from_drip;
}

void DataFusion::getWeightMeasurementNoises(float& r_from_weight, float& r_from_drip) const {
    r_from_weight = R_weight_sensor_weight_noise;
    r_from_drip = R_drip_sensor_weight_noise;
}

