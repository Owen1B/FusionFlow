#include "WeightKalmanFilter.h"

/**
 * @brief Constructs a new WeightKalmanFilter object.
 *
 * @param sigma_a Standard deviation representing the unmodeled random acceleration (process noise strength).
 *                It affects the filter's ability to track true velocity changes versus smoothing noise.
 * @param sigma_j Standard deviation representing the unmodeled random jerk (rate of change of acceleration).
 * @param measurement_noise_R Variance of the weight sensor's measurement noise. It reflects the reliability of the sensor readings.
 */
WeightKalmanFilter::WeightKalmanFilter(float sigma_a, float sigma_j, float measurement_noise_R) {
    sigma_a_process_noise = sigma_a;         // Store process noise parameter (std dev of acceleration)
    sigma_j_process_noise = sigma_j;         // Store process noise parameter (std dev of jerk)
    R_measurement_noise = measurement_noise_R; // Store measurement noise variance

    // Initialize state vector
    x_state[0] = 0.0f; // Initial weight estimate (g)
    x_state[1] = 0.0f; // Initial velocity estimate (g/s)
    x_state[2] = 0.0f; // Initial acceleration estimate (g/s^2)

    // Initialize the estimation error covariance matrix P_cov
    // Initially, we have high uncertainty about the state, especially the weight.
    // Set P_cov[0][0] (weight variance) to a relatively large value.
    // Set P_cov[1][1] (velocity variance) to a medium value.
    // Set P_cov[2][2] (acceleration variance) to a smaller value.
    // Off-diagonal terms are typically initialized to 0, assuming initial estimation errors are uncorrelated.
    P_cov[0][0] = 100.0f; P_cov[0][1] = 0.0f; P_cov[0][2] = 0.0f;
    P_cov[1][0] = 0.0f;  P_cov[1][1] = 10.0f; P_cov[1][2] = 0.0f;
    P_cov[2][0] = 0.0f;  P_cov[2][1] = 0.0f;  P_cov[2][2] = 1.0f;
    
    // The process noise covariance matrix Q_process_noise_cov is calculated dynamically
    // in the update() function because it depends on the time interval dt.
}

/**
 * @brief Initializes or resets the filter state.
 *
 * @param initial_weight The best initial estimate for the weight (g).
 * @param initial_velocity The best initial estimate for the rate of weight change (g/s), defaults to 0.
 * @param initial_acceleration The best initial estimate for the weight acceleration (g/s^2), defaults to 0.
 */
void WeightKalmanFilter::init(float initial_weight, float initial_velocity, float initial_acceleration) {
    // Set initial state estimates
    x_state[0] = initial_weight;    // Weight (g)
    x_state[1] = initial_velocity;  // Velocity (g/s)
    x_state[2] = initial_acceleration;  // Acceleration (g/s^2)

    // Reset the estimation error covariance matrix P_cov
    // When we initialize the filter with a specific value, we can assume
    // the uncertainty is relatively small. However, it should not be zero
    // to allow the filter to learn from subsequent measurements.
    P_cov[0][0] = 1.0f;  P_cov[0][1] = 0.0f; P_cov[0][2] = 0.0f;
    P_cov[1][0] = 0.0f;  P_cov[1][1] = 1.0f; P_cov[1][2] = 0.0f;
    P_cov[2][0] = 0.0f;  P_cov[2][1] = 0.0f; P_cov[2][2] = 0.1f; // Slightly higher uncertainty for accel
}

/**
 * @brief Updates the filter state using a new sensor measurement and time interval.
 *
 * This is the core of the Kalman filter, containing both the prediction and update steps.
 * @param measurement The current raw weight reading from the sensor (g).
 * @param dt The time interval since the last update() call (in seconds).
 * @return float The filtered current weight estimate (g).
 */
float WeightKalmanFilter::update(float measurement, float dt) {
    // Safety check: if the time interval is invalid or too small, skip the update and return the current weight estimate.
    if (dt <= 1e-6f) return x_state[0]; // Use a small threshold like 1 microsecond

    // === 1. Prediction Step ===

    // Define the state transition matrix F
    // This model assumes constant acceleration over the small interval dt.
    // x_new = F * x_old
    // [weight_new] = [1, dt, 0.5*dt^2] [weight_old]
    // [speed_new ] = [0,  1,       dt] [speed_old ]
    // [accel_new ] = [0,  0,        1] [accel_old ]
    float dt2_div2 = dt * dt / 2.0f;
    float F_matrix[3][3] = {
        {1.0f, dt,   dt2_div2},
        {0.0f, 1.0f, dt      },
        {0.0f, 0.0f, 1.0f    }
    };

    // Calculate the process noise covariance matrix Q
    // This is derived from a random acceleration model (velocity is a random walk, acceleration is white noise).
    float dt2 = dt * dt;
    float dt3 = dt2 * dt;
    float dt4 = dt3 * dt;
    float sa_sq = sigma_a_process_noise * sigma_a_process_noise;
    float sj_sq = sigma_j_process_noise * sigma_j_process_noise;

    float Q_cov[3][3];
    Q_cov[0][0] = sa_sq * dt4 / 4.0f; Q_cov[0][1] = sa_sq * dt3 / 2.0f; Q_cov[0][2] = sa_sq * dt2 / 2.0f;
    Q_cov[1][0] = sa_sq * dt3 / 2.0f; Q_cov[1][1] = sa_sq * dt2;        Q_cov[1][2] = sa_sq * dt;
    Q_cov[2][0] = sa_sq * dt2 / 2.0f; Q_cov[2][1] = sa_sq * dt;         Q_cov[2][2] = sj_sq;

    // Predict state: x_pred = F * x_state (current state)
    float x_pred[3]; // Predicted state vector [predicted_weight, predicted_velocity, predicted_acceleration]
    x_pred[0] = F_matrix[0][0]*x_state[0] + F_matrix[0][1]*x_state[1] + F_matrix[0][2]*x_state[2];
    x_pred[1] = F_matrix[1][0]*x_state[0] + F_matrix[1][1]*x_state[1] + F_matrix[1][2]*x_state[2];
    x_pred[2] = F_matrix[2][0]*x_state[0] + F_matrix[2][1]*x_state[1] + F_matrix[2][2]*x_state[2];

    // Predict estimate error covariance: P_pred = F * P_cov * F_transpose + Q_cov
    // First, compute F * P_cov (result stored in FP)
    float FP[3][3];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            FP[i][j] = 0;
            for (int k = 0; k < 3; ++k) {
                FP[i][j] += F_matrix[i][k] * P_cov[k][j];
            }
        }
    }

    // Then, compute (F * P_cov) * F_transpose
    float P_pred_temp[3][3];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            P_pred_temp[i][j] = 0;
            for (int k = 0; k < 3; ++k) {
                P_pred_temp[i][j] += FP[i][k] * F_matrix[j][k]; // F_transpose[k][j] is F_matrix[j][k]
            }
        }
    }
    
    // Finally, P_pred = P_pred_temp + Q_cov
    float P_pred[3][3];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            P_pred[i][j] = P_pred_temp[i][j] + Q_cov[i][j];
        }
    }

    // === 2. Update Step ===

    // Define the measurement matrix H. We directly measure weight, not velocity or acceleration.
    // measurement = H * x_true_state + measurement_noise
    // H = [1, 0, 0]  (since measurement z = 1*weight + 0*velocity + 0*acceleration)

    // Calculate the innovation covariance S
    // S = H * P_pred * H_transpose + R_measurement_noise
    // Since H = [1,0,0], H * P_pred * H_transpose simplifies to P_pred[0][0]
    float S_innovation_cov = P_pred[0][0] + R_measurement_noise;
    // Prevent division by zero if S is too small
    if (S_innovation_cov == 0) S_innovation_cov = 1e-9f;

    // Calculate the Kalman Gain K
    // K = P_pred * H_transpose * S_inverse
    // Since H = [1,0,0], P_pred * H_transpose is the first column of P_pred: [P_pred[0][0], P_pred[1][0], P_pred[2][0]]^T
    float K_gain[3]; // Kalman gain vector (3x1)
    K_gain[0] = P_pred[0][0] / S_innovation_cov;
    K_gain[1] = P_pred[1][0] / S_innovation_cov;
    K_gain[2] = P_pred[2][0] / S_innovation_cov;

    // Calculate the innovation (measurement residual) y
    // y = measurement - H * x_pred
    // Since H = [1,0,0], H * x_pred simplifies to x_pred[0]
    float innovation_y = measurement - x_pred[0];

    // Update the state estimate: x_state_new = x_pred + K_gain * y
    x_state[0] = x_pred[0] + K_gain[0] * innovation_y;
    x_state[1] = x_pred[1] + K_gain[1] * innovation_y;
    x_state[2] = x_pred[2] + K_gain[2] * innovation_y;

    // Update the estimate error covariance: P_cov_new = (I - K_gain * H) * P_pred
    // Let I_KH = (I - K_gain * H)
    float I_KH[3][3] = {
        {1.0f - K_gain[0], 0.0f,          0.0f},
        {-K_gain[1],       1.0f,          0.0f},
        {-K_gain[2],       0.0f,          1.0f}
    };

    // P_cov = I_KH * P_pred
    float P_cov_temp[3][3]; // Temporary storage for the new P_cov
     for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            P_cov_temp[i][j] = 0;
            for (int k = 0; k < 3; ++k) {
                P_cov_temp[i][j] += I_KH[i][k] * P_pred[k][j];
            }
        }
    }
    // Copy P_cov_temp to P_cov
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            P_cov[i][j] = P_cov_temp[i][j];
        }
    }
    
    return x_state[0]; // Return the filtered weight estimate
} 