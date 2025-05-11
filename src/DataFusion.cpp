#include "DataFusion.h"
#include <math.h> // 用于 fabsf (可能不需要，但包含无害)

/**
 * @brief DataFusion 类的构造函数。
 *
 * @param q_flow 过程噪声方差 - 流速。
 * @param r_weight_flow 测量噪声方差 - 重量传感器的流速。
 * @param r_drip_flow 测量噪声方差 - 滴速传感器的流速。
 * @param q_weight 过程噪声方差 - 剩余重量。
 * @param r_weight_weight 测量噪声方差 - 重量传感器的剩余重量。
 * @param r_drip_weight 测量噪声方差 - 滴速传感器的剩余重量。
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
    P_fused_weight_cov = 10.0f; // 剩余重量的初始不确定性可能更大
}

/**
 * @brief 初始化或重置数据融合滤波器的状态。
 *
 * @param initial_fused_flow_rate_gps 融合后流速的初始估计值 (g/s)。
 * @param initial_fused_remaining_weight_g 融合后剩余重量的初始估计值 (g)。
 */
void DataFusion::init(float initial_fused_flow_rate_gps, float initial_fused_remaining_weight_g) {
    x_fused_flow_rate_gps = initial_fused_flow_rate_gps;
    P_fused_flow_cov = 0.1f; 

    x_fused_remaining_weight_g = initial_fused_remaining_weight_g;
    P_fused_weight_cov = 1.0f; // 明确初始化时，不确定性减小
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
 * @param weight_from_weight_sensor_g WeightKalmanFilter 估计的当前重量 (g)。
 * @param weight_from_drip_sensor_g DripKalmanFilter 估计的剩余重量 (g)。
 * @param dt 距离上一次调用 update() 的时间间隔 (秒)。用于过程噪声的时间累积。
 */
void DataFusion::update(float flow_from_weight_sensor_gps, 
                        float flow_from_drip_sensor_gps, 
                        float weight_from_weight_sensor_g,
                        float weight_from_drip_sensor_g,
                        float dt) {
    // 安全检查：如果时间间隔无效，则不进行预测步骤
    if (dt <= 1e-6f) return;

    // === 1. 预测步骤 ===
    // --- 流速预测 ---
    float x_pred_flow = x_fused_flow_rate_gps;
    float P_pred_flow = P_fused_flow_cov + Q_flow_process_noise * dt;
    x_fused_flow_rate_gps = x_pred_flow;
    P_fused_flow_cov = P_pred_flow;

    // --- 剩余重量预测 ---
    // 简单预测：假设当前融合后的剩余重量会根据当前融合后的流速减少
    // 这使得重量预测和流速预测之间产生耦合
    // x_pred_weight = x_current_weight - x_current_flow * dt
    // 这种预测方式没有直接的标准卡尔曼过程噪声模型，所以我们主要依赖测量更新
    // 但仍然可以为协方差添加过程噪声
    float x_pred_weight = x_fused_remaining_weight_g - x_fused_flow_rate_gps * dt; 
    if (x_pred_weight < 0) x_pred_weight = 0; // 重量不能为负
    float P_pred_weight = P_fused_weight_cov + Q_weight_process_noise * dt; // 也可以让 Q_weight_process_noise 包含流速不确定性引入的项
    x_fused_remaining_weight_g = x_pred_weight;
    P_fused_weight_cov = P_pred_weight;


    // === 2. 更新步骤 (依次使用每个传感器的估计值作为测量值) ===

    // --- 流速更新 ---
    if (R_weight_sensor_flow_noise > 1e-9f) {
        kalman_update_1d(x_fused_flow_rate_gps, P_fused_flow_cov, flow_from_weight_sensor_gps, R_weight_sensor_flow_noise);
    }
    if (R_drip_sensor_flow_noise > 1e-9f) {
        kalman_update_1d(x_fused_flow_rate_gps, P_fused_flow_cov, flow_from_drip_sensor_gps, R_drip_sensor_flow_noise);
    }

    // --- 剩余重量更新 ---
    if (R_weight_sensor_weight_noise > 1e-9f) {
        kalman_update_1d(x_fused_remaining_weight_g, P_fused_weight_cov, weight_from_weight_sensor_g, R_weight_sensor_weight_noise);
    }
    if (R_drip_sensor_weight_noise > 1e-9f) {
        kalman_update_1d(x_fused_remaining_weight_g, P_fused_weight_cov, weight_from_drip_sensor_g, R_drip_sensor_weight_noise);
    }

    // 确保融合后的剩余重量不为负
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

// float DataFusion::getFusedFlowRateGps() const { return x_fused_flow_rate_gps; } // Already defined in .h as inline
// float DataFusion::getFusedRemainingWeightG() const { return x_fused_remaining_weight_g; } // Already defined in .h as inline 