#ifndef DATA_FUSION_H
#define DATA_FUSION_H

/**
 * @brief 数据融合类，用于融合来自不同传感器的流速和剩余重量估计。
 *
 * 该类实现了一对简单的一维卡尔曼滤波器，分别用于：
 * 1. 融合两个独立的流速测量值 (来自重量传感器和滴速传感器)。
 * 2. 融合两个独立的剩余重量估计值 (来自重量传感器和滴速传感器)。
 *
 * 目标是得到更稳定、更准确的最终流速和剩余重量估计。
 */
class DataFusion {
private:
    // --- 卡尔曼滤波器状态 (针对融合后的流速) ---
    float x_fused_flow_rate_gps; // 状态变量：融合后的流速估计值 (g/s)
    float P_fused_flow_cov;      // 流速状态估计的协方差

    // --- 卡尔曼滤波器状态 (针对融合后的剩余重量) ---
    float x_fused_remaining_weight_g; // 状态变量：融合后的剩余重量估计值 (g)
    float P_fused_weight_cov;         // 剩余重量状态估计的协方差

    // --- 噪声参数 --- 
    // 过程噪声：表示真实的输液流速/剩余重量本身随时间可能发生的内在变化或不确定性。
    float Q_flow_process_noise;    // 流速融合卡尔曼滤波器的过程噪声方差
    float Q_weight_process_noise;  // 剩余重量融合卡尔曼滤波器的过程噪声方差

    // 测量噪声：表示输入给融合算法的各个估计值各自的不可靠程度（方差）。
    float R_weight_sensor_flow_noise;    // 重量传感器流速估计的测量噪声方差
    float R_drip_sensor_flow_noise;      // 滴速传感器流速估计的测量噪声方差
    float R_weight_sensor_weight_noise;  // 重量传感器剩余重量估计的测量噪声方差
    float R_drip_sensor_weight_noise;    // 滴速传感器剩余重量估计的测量噪声方差
    
public:
    /**
     * @brief 构造一个新的 DataFusion 对象。
     *
     * @param q_flow 过程噪声方差 - 流速。
     * @param r_weight_flow 测量噪声方差 - 重量传感器的流速。
     * @param r_drip_flow 测量噪声方差 - 滴速传感器的流速。
     * @param q_weight 过程噪声方差 - 剩余重量。
     * @param r_weight_weight 测量噪声方差 - 重量传感器的剩余重量。
     * @param r_drip_weight 测量噪声方差 - 滴速传感器的剩余重量。
     */
    DataFusion(float q_flow = 0.0001f, float r_weight_flow = 0.0025f, float r_drip_flow = 0.0025f,
               float q_weight = 0.01f,  float r_weight_weight = 1.0f,  float r_drip_weight = 2.0f);

    /**
     * @brief 初始化或重置数据融合滤波器的状态。
     *
     * @param initial_fused_flow_rate_gps 融合后流速的初始估计值 (g/s)。
     * @param initial_fused_remaining_weight_g 融合后剩余重量的初始估计值 (g)。
     */
    void init(float initial_fused_flow_rate_gps = 0.0f, float initial_fused_remaining_weight_g = 0.0f);

    /**
     * @brief 使用来自两个传感器的流速和剩余重量估计值更新融合状态。
     *
     * @param flow_from_weight_sensor_gps 从 WeightKalmanFilter 获取的流速估计 (g/s)。
     * @param flow_from_drip_sensor_gps 从 DripKalmanFilter 获取的流速估计 (g/s)。
     * @param weight_from_weight_sensor_g WeightKalmanFilter 估计的当前重量 (g)。
     * @param weight_from_drip_sensor_g DripKalmanFilter 估计的剩余重量 (g)。
     * @param dt 距离上一次调用 update() 的时间间隔 (秒)。
     */
    void update(float flow_from_weight_sensor_gps, 
                float flow_from_drip_sensor_gps, 
                float weight_from_weight_sensor_g,
                float weight_from_drip_sensor_g,
                float dt);

    /** @brief 获取当前融合后的流速估计值。 */
    float getFusedFlowRateGps() const { return x_fused_flow_rate_gps; }
    /** @brief 获取当前融合后的剩余重量估计值。 */
    float getFusedRemainingWeightG() const { return x_fused_remaining_weight_g; }

    // Methods for fast convergence mode
    void setFlowMeasurementNoises(float r_from_weight, float r_from_drip);
    void getFlowMeasurementNoises(float& r_from_weight, float& r_from_drip) const;
    void setWeightMeasurementNoises(float r_from_weight, float r_from_drip);
    void getWeightMeasurementNoises(float& r_from_weight, float& r_from_drip) const;

};

#endif // DATA_FUSION_H 