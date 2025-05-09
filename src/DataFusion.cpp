#include "DataFusion.h"
#include <math.h> // 用于 fabsf (可能不需要，但包含无害)

/**
 * @brief DataFusion 类的构造函数。
 *
 * @param process_noise_Q 融合卡尔曼滤波器的过程噪声方差 (Q_df)。
 *                        表示真实流速本身随时间可能发生的变化的不确定性。
 * @param weight_sensor_flow_R 重量传感器流速估计的测量噪声方差 (R_df_weight)。
 *                             反映了来自 WeightKalmanFilter 的流速估计的可靠性。
 * @param drip_sensor_flow_R 滴速传感器流速估计的测量噪声方差 (R_df_drip)。
 *                           反映了来自 DripKalmanFilter 的流速估计的可靠性。
 */
DataFusion::DataFusion(float process_noise_Q, float weight_sensor_flow_R, float drip_sensor_flow_R) {
    Q_df_process_noise = process_noise_Q;
    R_df_weight_sensor_flow_noise = weight_sensor_flow_R;
    R_df_drip_sensor_flow_noise = drip_sensor_flow_R;

    // 初始化融合后流速的状态估计和协方差
    x_fused_flow_rate_gps = 0.0f; // 初始融合流速估计为0 (g/s)
    P_fused_cov = 1.0f;           // 初始估计协方差，表示对初始估计值有较大不确定性
}

/**
 * @brief 初始化或重置数据融合滤波器的状态。
 *
 * @param initial_fused_flow_rate_gps 融合后流速的初始估计值 (g/s)。
 */
void DataFusion::init(float initial_fused_flow_rate_gps) {
    x_fused_flow_rate_gps = initial_fused_flow_rate_gps;
    // 当用一个明确的初始值设定时，可以假设此时的不确定性相对较小。
    P_fused_cov = 0.1f; 
}

/**
 * @brief 一维卡尔曼滤波器的标准更新步骤辅助函数。
 *
 * @param x 对要更新的状态变量的引用 (例如，流速 x_fused_flow_rate_gps)。
 * @param P 对该状态变量估计误差协方差的引用 (例如，P_fused_cov)。
 * @param measurement 当前的测量值 (例如，来自某个传感器的流速)。
 * @param measurement_noise_R 该测量值的噪声方差。
 */
static void kalman_update_1d(float& x, float& P, float measurement, float measurement_noise_R) {
    // 安全检查：如果测量噪声极小（意味着测量值非常可信，或者R未正确设置），
    // 或者P本身很小，可能会导致数值不稳定或K接近1。这里主要防止R为0。
    if (measurement_noise_R < 1e-9f) return; 

    // 计算卡尔曼增益 K
    // K = P_predicted / (P_predicted + R_measurement)
    // 在这里，传入的 P 已经是预测后的 P_pred (因为时间更新已在主update函数完成)
    float K = P / (P + measurement_noise_R);
    
    // 更新状态估计
    // x_new = x_predicted + K * (measurement - x_predicted)
    x = x + K * (measurement - x);
    
    // 更新估计误差协方差
    // P_new = (1 - K) * P_predicted
    P = (1.0f - K) * P;
}

/**
 * @brief 使用来自两个传感器的流速估计值更新融合后的流速。
 *
 * @param flow_from_weight_sensor_gps 从 WeightKalmanFilter 获取的流速估计 (g/s)。
 * @param flow_from_drip_sensor_gps 从 DripKalmanFilter 获取的流速估计 (g/s)。
 * @param dt 距离上一次调用 update() 的时间间隔 (秒)。用于过程噪声的时间累积。
 * @return float 更新后的融合流速估计值 (g/s)。
 */
float DataFusion::update(float flow_from_weight_sensor_gps, 
                         float flow_from_drip_sensor_gps, 
                         float dt) {
    // 安全检查：如果时间间隔无效，则不进行预测步骤
    if (dt <= 1e-6f) return x_fused_flow_rate_gps; 

    // === 1. 预测步骤 (针对融合后的流速 x_fused_flow_rate_gps) ===
    // 状态预测：假设真实流速在两次测量之间可能基于其当前值发生小的随机游走。
    // 对于一维情况，状态转移矩阵 F 通常为 [1]。
    // x_predicted = F * x_current = 1 * x_fused_flow_rate_gps = x_fused_flow_rate_gps
    // 所以，状态预测值就是当前的状态值。
    float x_pred = x_fused_flow_rate_gps; // 预测的流速（在没有控制输入的情况下，等于当前流速）

    // 协方差预测：P_predicted = F * P_current * F_transpose + Q
    // 对于 F=[1], 则 P_predicted = P_current + Q
    // 这里的 Q_df_process_noise 代表在 dt 时间段内过程噪声引入的方差增加量。
    // 如果 Q_df_process_noise 定义为单位时间的方差率，则应乘以 dt。
    // 假设 Q_df_process_noise 已是针对一个dt时间间隔的方差增量，或者乘以dt得到。
    float P_pred = P_fused_cov + Q_df_process_noise * dt; 
                                                    
    // 将预测结果赋回给当前状态和协方差，准备进行测量更新
    x_fused_flow_rate_gps = x_pred;
    P_fused_cov = P_pred;

    // === 2. 更新步骤 (依次使用每个传感器的流速作为测量值) ===

    // 使用来自重量传感器的流速进行第一次更新
    // 仅当该传感器的测量噪声方差有效（大于一个小阈值）时才进行更新
    if (R_df_weight_sensor_flow_noise > 1e-9f) {
        kalman_update_1d(x_fused_flow_rate_gps, P_fused_cov, flow_from_weight_sensor_gps, R_df_weight_sensor_flow_noise);
    }

    // 使用来自滴速传感器的流速进行第二次更新
    // 同样，仅当其测量噪声方差有效时才更新
    if (R_df_drip_sensor_flow_noise > 1e-9f) {
        kalman_update_1d(x_fused_flow_rate_gps, P_fused_cov, flow_from_drip_sensor_gps, R_df_drip_sensor_flow_noise);
    }

    return x_fused_flow_rate_gps; // 返回最终融合后的流速估计
} 