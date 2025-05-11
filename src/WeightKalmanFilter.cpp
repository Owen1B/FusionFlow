#include "WeightKalmanFilter.h"
// // 注意：如果要在非Arduino环境中使用且需要数学函数，可能需要包含 <cmath> 或 <math.h>
// // 但对于当前的实现，似乎没有直接使用外部数学库的函数。

/**
 * @brief 构造一个新的 WeightKalmanFilter 对象。
 *
 * @param sigma_a 标准差，表示未建模的随机加速度（过程噪声强度）。
 *                  它影响滤波器对真实速度变化的跟踪能力和速度估计的平滑度。
 * @param sigma_j 标准差，表示未建模的随机加速度变化率（过程噪声强度）。
 * @param measurement_noise_R 重量传感器测量噪声的方差。它反映了传感器读数的可靠性。
 */
WeightKalmanFilter::WeightKalmanFilter(float sigma_a, float sigma_j, float measurement_noise_R) {
    sigma_a_process_noise = sigma_a;         // 存储过程噪声参数 (加速度标准差)
    sigma_j_process_noise = sigma_j;         // 存储过程噪声参数 (加速度变化率标准差)
    R_measurement_noise = measurement_noise_R; // 存储测量噪声方差

    // 初始化状态向量
    x_state[0] = 0.0f; // 初始重量估计 (g)
    x_state[1] = 0.0f; // 初始速度估计 (g/s)
    x_state[2] = 0.0f; // 初始加速度估计 (g/s^2)

    // 初始化估计误差协方差矩阵 P_cov
    // 开始时，我们对状态的估计具有较高的不确定性，尤其是重量。
    // P_cov[0][0] (重量方差) 设一个相对较大的值。
    // P_cov[1][1] (速度方差) 可以设一个中等大小的值。
    // P_cov[2][2] (加速度方差) 可以设一个较低的值。
    // 协方差项 P_cov[0][1] 和 P_cov[1][0] 初始时通常设为0，表示初始假设重量和速度的估计误差不相关。
    P_cov[0][0] = 100.0f; P_cov[0][1] = 0.0f; P_cov[0][2] = 0.0f;
    P_cov[1][0] = 0.0f;  P_cov[1][1] = 10.0f; P_cov[1][2] = 0.0f;
    P_cov[2][0] = 0.0f;  P_cov[2][1] = 0.0f;  P_cov[2][2] = 1.0f;
    
    // 过程噪声协方差矩阵 Q_process_noise_cov 会在 update() 函数中根据 dt 动态计算，
    // 因为它依赖于时间间隔 dt。
}

/**
 * @brief 初始化或重置滤波器状态。
 *
 * @param initial_weight 初始重量的最佳估计值 (g)。
 * @param initial_velocity 初始重量变化速率的最佳估计值 (g/s)，默认为0。
 * @param initial_acceleration 初始重量变化率的加速度的最佳估计值 (g/s^2)，默认为0。
 */
void WeightKalmanFilter::init(float initial_weight, float initial_velocity, float initial_acceleration) {
    // 设置初始状态估计
    x_state[0] = initial_weight;    // 重量 (g)
    x_state[1] = initial_velocity;  // 速度 (g/s)
    x_state[2] = initial_acceleration;  // 加速度 (g/s^2)

    // 重置估计误差协方差矩阵 P_cov
    // 当我们用一个明确的初始值设定滤波器时，可以假设此时的不确定性相对较小。
    // 但也不应设为0，以允许滤波器从后续测量中学习。
    P_cov[0][0] = 1.0f;  P_cov[0][1] = 0.0f; P_cov[0][2] = 0.0f;
    P_cov[1][0] = 0.0f;  P_cov[1][1] = 1.0f; P_cov[1][2] = 0.0f;
    P_cov[2][0] = 0.0f;  P_cov[2][1] = 0.0f; P_cov[2][2] = 0.1f; // Slightly higher uncertainty for accel
}

/**
 * @brief 使用新的传感器测量值和时间间隔来更新滤波器状态。
 *
 * 这是卡尔曼滤波的核心，包含预测和更新两个步骤。
 * @param measurement 从重量传感器获取的当前原始重量读数 (g)。
 * @param dt 距离上一次调用 update() 的时间间隔 (秒)。
 * @return float 滤波后的当前重量估计值 (g)。
 */
float WeightKalmanFilter::update(float measurement, float dt) {
    // 安全检查：如果时间间隔无效或过小，则不进行更新，直接返回当前重量估计
    if (dt <= 1e-6f) return x_state[0]; // 使用一个很小的值(如1微秒)作为阈值

    // === 1. 预测步骤 ===

    // 定义状态转移矩阵 F
    // F = [[1, dt],  (重量_new = 1*重量_old + dt*速度_old)
    //      [0, 1]]  (速度_new = 0*重量_old + 1*速度_old) - 假设速度在小间隔内近似恒定
    float dt2_div2 = dt * dt / 2.0f;
    float F_matrix[3][3] = {
        {1.0f, dt,   dt2_div2},
        {0.0f, 1.0f, dt      },
        {0.0f, 0.0f, 1.0f    }
    };

    // 计算过程噪声协方差矩阵 Q_process_noise_cov
    // Q = [[dt^4/4, dt^3/2], [dt^3/2, dt^2]] * sigma_a_process_noise^2
    // 这是基于随机加速度模型（速度是随机游走，加速度是白噪声）推导出来的。
    float dt2 = dt * dt;
    float dt3 = dt2 * dt;
    float dt4 = dt3 * dt;
    float sa_sq = sigma_a_process_noise * sigma_a_process_noise;
    float sj_sq = sigma_j_process_noise * sigma_j_process_noise;

    float Q_cov[3][3];
    Q_cov[0][0] = sa_sq * dt4 / 4.0f; Q_cov[0][1] = sa_sq * dt3 / 2.0f; Q_cov[0][2] = sa_sq * dt2 / 2.0f;
    Q_cov[1][0] = sa_sq * dt3 / 2.0f; Q_cov[1][1] = sa_sq * dt2;        Q_cov[1][2] = sa_sq * dt;
    Q_cov[2][0] = sa_sq * dt2 / 2.0f; Q_cov[2][1] = sa_sq * dt;         Q_cov[2][2] = sj_sq; // Matches Python: sigma_j**2

    // 预测状态：x_pred = F * x_state (当前状态)
    float x_pred[3]; // 预测的状态向量 [预测重量, 预测速度, 预测加速度]
    x_pred[0] = F_matrix[0][0]*x_state[0] + F_matrix[0][1]*x_state[1] + F_matrix[0][2]*x_state[2];
    x_pred[1] = F_matrix[1][0]*x_state[0] + F_matrix[1][1]*x_state[1] + F_matrix[1][2]*x_state[2];
    x_pred[2] = F_matrix[2][0]*x_state[0] + F_matrix[2][1]*x_state[1] + F_matrix[2][2]*x_state[2];

    // 预测估计误差协方差：P_pred = F * P_cov * F_transpose + Q_cov
    // 首先计算 F * P_cov (结果存储在 FP)
    float FP[3][3];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            FP[i][j] = 0;
            for (int k = 0; k < 3; ++k) {
                FP[i][j] += F_matrix[i][k] * P_cov[k][j];
            }
        }
    }

    // 然后计算 (F * P_cov) * F_transpose。
    // F_transpose = [[F_matrix[0][0], F_matrix[1][0], F_matrix[2][0]],
    //                [F_matrix[0][1], F_matrix[1][1], F_matrix[2][1]],
    //                [F_matrix[0][2], F_matrix[1][2], F_matrix[2][2]]]
    float P_pred_temp[3][3];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            P_pred_temp[i][j] = 0;
            for (int k = 0; k < 3; ++k) {
                P_pred_temp[i][j] += FP[i][k] * F_matrix[j][k]; // F_transpose[k][j] = F_matrix[j][k]
            }
        }
    }
    
    // Step 3: P_pred = P_pred_temp + Q_cov
    float P_pred[3][3];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            P_pred[i][j] = P_pred_temp[i][j] + Q_cov[i][j];
        }
    }

    // === 2. 更新步骤 ===

    // 定义测量矩阵 H。我们直接测量重量，不直接测量速度和加速度。
    // measurement = H * x_true_state + measurement_noise
    // H = [1, 0, 0]  (因为测量值 z = 1*重量 + 0*速度 + 0*加速度)
    // H_matrix_transpose (H的转置) = [[1], [0], [0]] (列向量)

    // 计算新息协方差 S (Innovation Covariance)
    // S = H * P_pred * H_transpose + R_measurement_noise
    // 由于 H = [1,0,0], H * P_pred * H_transpose 简化为 P_pred[0][0]
    float S_innovation_cov = P_pred[0][0] + R_measurement_noise;
    // 防止S为0或过小导致除法错误
    if (S_innovation_cov == 0) S_innovation_cov = 1e-9f; // 用一个极小值代替

    // 计算卡尔曼增益 K (Kalman Gain)
    // K = P_pred * H_transpose * S_inverse
    // P_pred * H_transpose (由于H=[1,0,0]) 结果是 P_pred 的第一列: [P_pred[0][0], P_pred[1][0], P_pred[2][0]]^T
    float K_gain[3]; // 卡尔曼增益向量 (3x1)
    K_gain[0] = P_pred[0][0] / S_innovation_cov;
    K_gain[1] = P_pred[1][0] / S_innovation_cov;
    K_gain[2] = P_pred[2][0] / S_innovation_cov;

    // 计算新息（测量残差） y
    // y = measurement - H * x_pred
    // 由于 H = [1,0,0], H * x_pred 简化为 x_pred[0]
    float innovation_y = measurement - x_pred[0];

    // 更新状态估计：x_state_new = x_pred + K_gain * y
    x_state[0] = x_pred[0] + K_gain[0] * innovation_y;
    x_state[1] = x_pred[1] + K_gain[1] * innovation_y;
    x_state[2] = x_pred[2] + K_gain[2] * innovation_y;

    // 更新估计误差协方差：P_cov_new = (I - K_gain * H) * P_pred
    // 其中 I 是单位矩阵 [[1,0,0],[0,1,0],[0,0,1]]
    // K_gain * H = [[K_gain[0], 0, 0],
    //               [K_gain[1], 0, 0],
    //               [K_gain[2], 0, 0]]
    // (I - K_gain * H) = [[1-K_gain[0], 0, 0],
    //                     [-K_gain[1],  1, 0],
    //                     [-K_gain[2],  0, 1]]
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
    
    return x_state[0]; // 返回滤波后的重量估计值
} 