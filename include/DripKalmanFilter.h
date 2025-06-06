#ifndef DRIP_KALMAN_FILTER_H
#define DRIP_KALMAN_FILTER_H

#include <stdint.h> // 用于 uint32_t (例如 Arduino 的 millis() 函数返回值类型)

/**
 * @brief 液滴卡尔曼滤波器类
 *
 * 该类负责处理来自液滴传感器的信号，并提供以下功能：
 * 1. 对原始滴速进行卡尔曼滤波，以获得更平滑和准确的滴速估计 (drips/second)。
 *    - 使用一个二维卡尔曼滤波器，状态为 [滴速 (dps), 滴速变化率 (dps/s^2)]。
 * 2. 在校准模式下，使用一维卡尔曼滤波器估计每滴液体的重量 (g/drip)。
 *    - 校准依赖于外部传入的、由重量传感器测得的实际重量变化。
 * 3. 基于滤波后的滴速和校准的每滴重量，计算流速 (g/s 和 mL/h)。
 * 4. 提供校准控制接口。
 */
class DripKalmanFilter {
private:
    // --- 滴速卡尔曼滤波器 (2D: 状态为 [滴速, 滴速变化率]) ---
    float x_drip_rate_state[2];   // 状态向量: [当前滴速 (drips/sec), 当前滴速变化率 (dps/s^2)]
    float P_drip_rate_cov[2][2];    // 滴速卡尔曼滤波器的估计误差协方差矩阵 (2x2)
    float Q_drip_rate_sigma_a;    // 过程噪声参数：未建模的滴速加速度的标准差。
                                  // 影响滤波器对滴速变化的跟踪灵敏度。
    float R_drip_rate_noise;     // 滴速测量的噪声方差 (可调)

    // --- 每滴重量 (WPD) 校准用的一维卡尔曼滤波器 ---
    float wpd_estimate_g_per_drip; // 当前估计的每滴重量 (g/drip)
    float P_wpd_cov;                 // WPD卡尔曼滤波器的估计误差协方差 (标量)
    float Q_wpd_process_noise;       // WPD过程噪声方差: 假设在校准期间，真实的每滴重量本身也可能发生微小变化。
    float R_wpd_measurement_noise;   // WPD测量的噪声方差 (可调)

    // --- 校准控制与默认参数 ---
    bool calibrating_wpd;              // 标志位：如果为true，则当前处于WPD校准模式
    // uint32_t last_calibration_run_ms;  // 上次执行WPD校准更新的时间戳 (目前主要由外部逻辑控制时长)
    
    static const int DEFAULT_DROPS_PER_ML = 20; // 输液器默认规格：每毫升的滴数
    static constexpr float DEFAULT_LIQUID_DENSITY_G_PER_ML = 1.0f; // 液体默认密度 (g/mL), 通常接近水的密度
    
    float current_default_drops_per_ml;   // 当前使用的每毫升滴数 (用于无校准时的WPD计算)
    float current_liquid_density_g_per_ml; // 当前使用的液体密度 (g/mL) (用于流速单位转换)

    // unsigned long last_drip_update_time_ms; // (已移除或未使用) 时间间隔dt由外部传入update函数

public:
    // --- 添加用于基于滴数计算剩余量的成员变量 ---
    float known_initial_total_weight_g;    // 由外部设置的初始总液体重量 (g)
    unsigned long total_drops_for_volume_calc; // 用于计算体积/剩余量的累计总滴数
    bool initial_weight_for_volume_calc_set; // 标记初始重量是否已设置
    /**
     * @brief 构造一个新的 DripKalmanFilter 对象。
     *
     * @param drip_rate_sigma_a 滴速卡尔曼滤波器的过程噪声参数 (未建模的滴速加速度标准差)。
     * @param drip_rate_R_noise 滴速卡尔曼滤波器的测量噪声方差 (针对原始滴速计算值)。
     * @param wpd_Q_noise 每滴重量(WPD)校准用卡尔曼滤波器的过程噪声方差。
     * @param wpd_R_noise 每滴重量(WPD)校准用卡尔曼滤波器的测量噪声方差。
     */
    DripKalmanFilter(float drip_rate_sigma_a = 0.1f, 
                     float drip_rate_R_noise = 0.5f,
                     float wpd_Q_noise = 0.0001f, // WPD本身变化很小，所以过程噪声小
                     float wpd_R_noise = 0.0025f); // WPD测量 (基于重量和滴数) 可能有一定误差

    /**
     * @brief 初始化或重置滤波器的状态。
     *
     * @param initial_drip_rate_dps 初始滴速的最佳估计值 (drips/sec)。
     * @param initial_wpd_g_per_drip 初始每滴重量的最佳估计值 (g/drip)。
     *                               如果提供的值 <= 0，则会使用基于 drops_per_ml 和 density_g_per_ml 的默认计算值。
     * @param drops_per_ml 每毫升的滴数，用于计算默认WPD（如果 initial_wpd_g_per_drip 未提供）。
     * @param density_g_per_ml 液体密度 (g/mL)，用于计算默认WPD和流速单位转换。
     */
    void init(float initial_drip_rate_dps = 0.0f, 
              float initial_wpd_g_per_drip = -1.0f, 
              int drops_per_ml = DEFAULT_DROPS_PER_ML, 
              float density_g_per_ml = DEFAULT_LIQUID_DENSITY_G_PER_ML);

    /**
     * @brief 使用新的传感器数据更新滤波器状态。
     *
     * @param measured_drip_rate 测量得到的滴速 (drips/sec)
     * @param time_interval_s 距离上一次调用 update() 的时间间隔 (秒)
     * @param weight_sensor_change_g 从重量传感器测得的实际重量变化值 (g)
     */
    void update(float measured_drip_rate, float time_interval_s, float weight_sensor_change_g = 0.0f);

    /**
     * @brief 获取当前滤波后的滴速。
     * @return float 滴速 (drips/sec)。
     */
    float getFilteredDripRate() const { return x_drip_rate_state[0]; }

    /**
     * @brief 获取当前校准或设定的每滴重量。
     * @return float 每滴重量 (g/drip)。
     */
    float getCalibratedWeightPerDrop() const { return wpd_estimate_g_per_drip; }

    /**
     * @brief 计算并返回基于当前滴速和每滴重量的流速。
     * @return float 流速 (g/sec)。
     */
    float getFlowRateGramsPerSecond() const;

    /**
     * @brief 计算并返回基于当前滴速、每滴重量和液体密度的流速。
     * @return float 流速 (mL/hour)。
     */
    float getFlowRateMlPerHour() const; 

    /** @brief 启动每滴重量(WPD)的校准过程。*/
    void startWpdCalibration();
    /** @brief 停止每滴重量(WPD)的校准过程。*/
    void stopWpdCalibration() { calibrating_wpd = false; }
    /** @brief 强制重新开始WPD校准 (等同于startWpdCalibration)。*/
    void forceWpdRecalibration() { startWpdCalibration(); }
    /** 
     * @brief 检查当前是否正在进行WPD校准 (指DripKalmanFilter内部是否在更新WPD值)。
     * @return true 如果正在校准WPD，否则为 false。
     */
    bool isWpdCalibrating() const { return calibrating_wpd; }

    /** @brief 设置默认的每毫升滴数（用于无WPD校准时的备用计算）。*/
    void setDefaultDropsPerML(int drops_per_ml) { current_default_drops_per_ml = (float)drops_per_ml; }
    /** @brief 设置默认的液体密度（用于流速单位转换和无WPD校准时的备用计算）。*/
    void setDefaultLiquidDensity(float density_g_per_ml) { current_liquid_density_g_per_ml = density_g_per_ml; }
    /** @brief 获取当前设定的液体密度。*/
    float getCurrentLiquidDensity() const { return current_liquid_density_g_per_ml; }

    // --- 添加用于基于滴数计算剩余量的方法 ---
    /**
     * @brief 设置用于基于滴数计算剩余量的初始总液体重量。
     * 调用此函数会重置内部累计的滴数。
     * @param initial_weight_g 初始总液体重量 (g)。
     */
    void setInitialLiquidWeightForVolumeCalc(float initial_weight_g);

    /**
     * @brief 更新用于计算体积/剩余量的累计总滴数。
     * @param drops_in_latest_period 最近一个周期内的新滴数。
     */
    void updateTotalDropsForVolumeCalc(int drops_in_latest_period);

    /**
     * @brief 基于累计滴数和校准的每滴重量，计算已输注的重量。
     * @return float 已输注的液体重量 (g)。如果WPD未校准或过小，可能返回不准确的值。
     */
    float getInfusedWeightByDropsG() const;
    
    /**
     * @brief 基于初始总重量和已滴注的量，计算剩余液体重量。
     * @return float 估计的剩余液体重量 (g)。如果初始重量未设置，则行为未定义或返回0。
     */
    float getRemainingWeightByDropsG() const;

    /**
     * @brief 基于累计总重量和累计滴数进行WPD校准（主循环调用）。
     * @param current_weight 当前滤波后重量（g）
     */
    void calibrateWpdByTotal(float current_weight);

    float getTotalDropsForVolumeCalc() const { return total_drops_for_volume_calc; }
    float getKnownInitialTotalWeightG() const { return known_initial_total_weight_g; }

    void setDripRateMeasurementNoise(float r_drip_rate_noise_param) { R_drip_rate_noise = r_drip_rate_noise_param; }
    float getDripRateMeasurementNoise() const { return R_drip_rate_noise; }
    void setWpdMeasurementNoise(float r_wpd_noise_param) { R_wpd_measurement_noise = r_wpd_noise_param; }
    float getWpdMeasurementNoise() const { return R_wpd_measurement_noise; }

};

#endif // DRIP_KALMAN_FILTER_H 