#ifndef WEIGHT_KALMAN_FILTER_H
#define WEIGHT_KALMAN_FILTER_H

/**
 * @brief 重量卡尔曼滤波器类
 * 
 * 该类实现了一个三维卡尔曼滤波器，用于估计重量、重量变化速率（速度）和重量加速度。
 * 状态向量: [重量 (g), 速度 (g/s), 加速度 (g/s^2)]
 * 它可以平滑原始重量传感器读数，并提供对重量变化速率和加速度的估计。
 */
class WeightKalmanFilter {
private:
    // --- 状态变量 ---
    float x_state[3]; // 状态向量: x_state[0] = 重量 (g), x_state[1] = 速度 (g/s), x_state[2] = 加速度 (g/s^2)

    // --- 卡尔曼滤波器协方差矩阵 ---
    float P_cov[3][3]; // 估计误差协方差矩阵 (3x3)

    // --- 噪声参数 ---
    float R_measurement_noise; // 测量噪声协方差 (标量，代表重量传感器读数的方差)

    // 过程噪声强度参数
    float sigma_a_process_noise; // 代表了未建模的随机加速度的标准差，影响滤波器对速度变化的适应速度。
    float sigma_j_process_noise; // 代表了未建模的随机加速度变化（jerk）的标准差，影响滤波器对加速度变化的适应速度。


public:
    /**
     * @brief 构造一个新的 WeightKalmanFilter 对象。
     *
     * @param sigma_a 标准差，表示影响速度的过程噪声强度。
     *                  典型值范围: 0.0001f 到 0.1f。
     * @param sigma_j 标准差，表示影响加速度的过程噪声强度。
     *                  典型值范围: 1e-8f 到 1e-5f。
     * @param measurement_noise_R 重量传感器测量噪声的方差。
     *                            例如，如果传感器读数标准差为0.5g, R可以设为0.25 (0.5*0.5)。
     */
    WeightKalmanFilter(float sigma_a = 0.0005f, float sigma_j = 1e-7f, float measurement_noise_R = 50.0f);
    
    /**
     * @brief 初始化滤波器状态。
     *
     * @param initial_weight 初始重量的最佳估计值 (g)。
     * @param initial_velocity 初始重量变化速率的最佳估计值 (g/s)，通常可以设为0。
     * @param initial_acceleration 初始重量加速度的最佳估计值 (g/s^2)，通常可以设为0。
     */
    void init(float initial_weight, float initial_velocity = 0.0f, float initial_acceleration = 0.0f);
    
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

    /**
     * @brief 获取当前估计的重量加速度。
     * @return float 当前加速度 (g/s^2)。
     */
    float getAcceleration() const { return x_state[2]; } 

    /**
     * @brief 设置测量噪声方差。
     * @param new_R 新的测量噪声方差。
     */
    void setMeasurementNoise(float new_R) { R_measurement_noise = new_R; }

    /**
     * @brief 获取当前设置的测量噪声方差。
     * @return float 当前的测量噪声方差。
     */
    float getMeasurementNoise() const { return R_measurement_noise; }
};

#endif 