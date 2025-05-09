#ifndef WEIGHT_KALMAN_FILTER_H
#define WEIGHT_KALMAN_FILTER_H

/**
 * @brief 重量卡尔曼滤波器类
 * 
 * 该类实现了一个二维卡尔曼滤波器，用于估计重量和重量变化速率（速度）。
 * 状态向量: [重量 (g), 速度 (g/s)]
 * 它可以平滑原始重量传感器读数，并提供对重量变化速率的估计。
 */
class WeightKalmanFilter {
private:
    // --- 状态变量 ---
    float x_state[2]; // 状态向量: x_state[0] = 重量 (g), x_state[1] = 速度 (g/s)

    // --- 卡尔曼滤波器协方差矩阵 ---
    // P_cov[0][0] = 重量估计的方差
    // P_cov[0][1] = 重量和速度估计的协方差
    // P_cov[1][0] = 速度和重量估计的协方差
    // P_cov[1][1] = 速度估计的方差
    float P_cov[2][2]; // 估计误差协方差矩阵 (2x2)

    // --- 噪声参数 ---
    // Q_process_noise_cov 由 sigma_a_process_noise 和 dt 在 update() 中动态计算
    // float Q_process_noise_cov[2][2]; // 过程噪声协方差矩阵 (2x2) - 在.cpp中定义为局部变量或动态计算
                                     
    float R_measurement_noise; // 测量噪声协方差 (标量，代表重量传感器读数的方差)

    // 过程噪声强度参数
    // 代表了未建模的随机加速度的标准差，影响滤波器对速度变化的适应速度。
    float sigma_a_process_noise; 

public:
    /**
     * @brief 构造一个新的 WeightKalmanFilter 对象。
     *
     * @param sigma_a 标准差，表示未建模的随机加速度（过程噪声强度）。
     *                  较小值使速度估计更平滑但响应慢，较大值使速度估计响应快但可能更嘈杂。
     *                  典型值范围: 0.001f (非常平滑) 到 0.5f (非常灵敏)。
     * @param measurement_noise_R 重量传感器测量噪声的方差。
     *                            例如，如果传感器读数标准差为0.5g, R可以设为0.25 (0.5*0.5)。
     */
    WeightKalmanFilter(float sigma_a = 0.05f, float measurement_noise_R = 1.0f);
    
    /**
     * @brief 初始化滤波器状态。
     *
     * @param initial_weight 初始重量的最佳估计值 (g)。
     * @param initial_velocity 初始重量变化速率的最佳估计值 (g/s)，通常可以设为0。
     */
    void init(float initial_weight, float initial_velocity = 0.0f);
    
    /**
     * @brief 使用新的传感器测量值更新滤波器状态。
     *
     * @param measurement 从重量传感器获取的原始重量读数 (g)。
     * @param dt 距离上一次调用 update() 的时间间隔 (秒)。
     * @return float 滤波后的当前重量估计值 (g)。
     */
    float update(float measurement, float dt);
    
    /**
     * @brief 获取当前滤波后的重量估计值。
     * @return float 当前重量 (g)。
     */
    float getWeight() const { return x_state[0]; }

    /**
     * @brief 获取当前估计的重量变化速率。
     * @return float 当前速度 (g/s)。如果重量在减少，此值通常为负。
     */
    float getVelocity() const { return x_state[1]; } 
};

#endif 