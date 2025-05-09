#ifndef DATA_FUSION_H
#define DATA_FUSION_H

/**
 * @brief 数据融合类，用于融合来自不同传感器的流速估计。
 *
 * 该类实现了一个简单的一维卡尔曼滤波器，用于融合两个独立的流速测量值：
 * 1. 来自重量传感器的流速估计 (通过 WeightKalmanFilter 计算得到)。
 * 2. 来自滴速传感器的流速估计 (通过 DripKalmanFilter 计算得到)。
 *
 * 目标是得到一个更稳定、更准确的最终流速估计。
 */
class DataFusion {
private:
    // --- 卡尔曼滤波器状态 (针对融合后的流速) ---
    float x_fused_flow_rate_gps; // 状态变量：融合后的流速估计值 (g/s)
    float P_fused_cov;           // 状态估计的协方差 (表示对融合结果的不确定性)

    // --- 噪声参数 ---
    // 过程噪声：表示真实的输液流速本身随时间可能发生的内在变化或不确定性。
    // 例如，药袋压力变化可能导致真实流速的缓慢漂移。
    float Q_df_process_noise; // 融合卡尔曼滤波器的过程噪声方差 (DF: Data Fusion)

    // 测量噪声：表示输入给融合算法的两个流速估计值各自的不可靠程度（方差）。
    float R_df_weight_sensor_flow_noise; // 重量传感器流速估计的测量噪声方差
    float R_df_drip_sensor_flow_noise;   // 滴速传感器流速估计的测量噪声方差
    
    // We don't need to store current weight here anymore, it's a system-level variable

public:
    /**
     * @brief 构造一个新的 DataFusion 对象。
     *
     * @param process_noise_Q 融合卡尔曼滤波器的过程噪声方差。
     *                        反映了真实流速本身随时间变化的预期波动程度。
     * @param weight_sensor_flow_R 重量传感器提供的流速估计值的测量噪声方差。
     *                             该值越小，表示越信任重量传感器的流速估计。
     * @param drip_sensor_flow_R 滴速传感器提供的流速估计值的测量噪声方差。
     *                           该值越小，表示越信任滴速传感器的流速估计。
     */
    DataFusion(float process_noise_Q = 0.0001f,      // 假设真实流速变化缓慢，过程噪声小
               float weight_sensor_flow_R = 0.0025f, // 示例值，需根据实际情况调整
               float drip_sensor_flow_R = 0.0025f);  // 示例值，需根据实际情况调整

    /**
     * @brief 初始化或重置数据融合滤波器的状态。
     *
     * @param initial_fused_flow_rate_gps 融合后流速的初始估计值 (g/s)，通常设为0。
     */
    void init(float initial_fused_flow_rate_gps = 0.0f);

    /**
     * @brief 使用来自两个传感器的流速估计值更新融合后的流速。
     *
     * 该方法首先对融合流速进行时间预测（加入过程噪声），
     * 然后依次使用重量传感器的流速和滴速传感器的流速作为测量值，
     * 通过卡尔曼更新步骤来修正融合流速的估计。
     *
     * @param flow_from_weight_sensor_gps 从 WeightKalmanFilter 获取的流速估计 (g/s)。
     *                                    应为正值，表示消耗速率。
     * @param flow_from_drip_sensor_gps 从 DripKalmanFilter 获取的流速估计 (g/s)。
     * @param dt 距离上一次调用 update() 的时间间隔 (秒)。用于过程噪声的累积。
     * @return float 更新后的融合流速估计值 (g/s)。
     */
    float update(float flow_from_weight_sensor_gps, 
                 float flow_from_drip_sensor_gps, 
                 float dt);

    /**
     * @brief 获取当前融合后的流速估计值。
     * @return float 融合流速 (g/s)。
     */
    float getFusedFlowRateGramsPerSecond() const { return x_fused_flow_rate_gps; }
    // getCurrentWeight() removed as it's not directly part of fusion logic anymore
};

#endif // DATA_FUSION_H 