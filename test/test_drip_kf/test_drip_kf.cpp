#include <unity.h>
#include "DripKalmanFilter.h"
#include <math.h> // For fabsf

// 定义测试中使用的常量
const float DEFAULT_DT_DRIP = 0.5f; // 模拟时间间隔 (s)
const float DRIP_RATE_SIGMA_A = 0.1f;
const float DRIP_RATE_R_NOISE = 0.5f;
const float WPD_Q_NOISE = 0.0001f;
const float WPD_R_NOISE = 0.0025f;
const float FLOAT_PRECISION_DRIP = 0.001f;
const float FLOAT_PRECISION_WPD = 0.0001f;

DripKalmanFilter drip_kf_test(DRIP_RATE_SIGMA_A, DRIP_RATE_R_NOISE, WPD_Q_NOISE, WPD_R_NOISE);

void setUp(void) {
    // 每个测试用例开始前运行
    drip_kf_test.init(0.0f, -1.0f, 20, 1.0f); // 使用默认WPD初始化
}

void tearDown(void) {
    // 每个测试用例结束后运行
}

// 测试滴速滤波器初始化和默认WPD
void test_drip_kf_initialization_and_default_wpd(void) {
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_PRECISION_DRIP, 0.0f, drip_kf_test.getFilteredDripRate());
    // 默认 WPD = (1.0 / 20 drops/mL) * 1.0 g/mL = 0.05 g/drip
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_PRECISION_WPD, 0.05f, drip_kf_test.getCalibratedWeightPerDrop());
    TEST_ASSERT_EQUAL(false, drip_kf_test.isWpdCalibrating());
}

// 测试零时间间隔更新
void test_drip_kf_zero_dt_update(void) {
    float initial_rate = drip_kf_test.getFilteredDripRate();
    drip_kf_test.update(1, 0.0f, 0.05f); // dt = 0
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_PRECISION_DRIP, initial_rate, drip_kf_test.getFilteredDripRate());
}

// 测试恒定滴速输入
void test_drip_kf_constant_drip_rate(void) {
    int drops_per_period = 2; // 2 drops / 0.5s = 4 dps
    float expected_drip_rate = (float)drops_per_period / DEFAULT_DT_DRIP; // 4.0 dps

    for (int i = 0; i < 50; ++i) {
        drip_kf_test.update(drops_per_period, DEFAULT_DT_DRIP, 0.0f); // 不进行WPD校准
    }
    TEST_ASSERT_FLOAT_WITHIN(0.5f, expected_drip_rate, drip_kf_test.getFilteredDripRate());
}

// 测试WPD校准过程
void test_drip_kf_wpd_calibration(void) {
    drip_kf_test.init(0.0f, 0.05f); // 初始WPD设为0.05
    drip_kf_test.startWpdCalibration();
    TEST_ASSERT_EQUAL(true, drip_kf_test.isWpdCalibrating());

    float initial_wpd = drip_kf_test.getCalibratedWeightPerDrop();

    // 模拟几次WPD更新
    // 第一次：2滴，重量减少0.12g -> 测量WPD = 0.06g
    drip_kf_test.update(2, DEFAULT_DT_DRIP, 0.12f);
    float wpd_after_update1 = drip_kf_test.getCalibratedWeightPerDrop();
    TEST_ASSERT_TRUE(fabsf(wpd_after_update1 - initial_wpd) > FLOAT_PRECISION_WPD); // WPD应该变化
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.055f, wpd_after_update1); // 期望向0.06移动，但不会立刻到达

    // 第二次：3滴，重量减少0.18g -> 测量WPD = 0.06g
    drip_kf_test.update(3, DEFAULT_DT_DRIP, 0.18f);
    float wpd_after_update2 = drip_kf_test.getCalibratedWeightPerDrop();
    TEST_ASSERT_TRUE(fabsf(wpd_after_update2 - wpd_after_update1) > FLOAT_PRECISION_WPD || fabsf(wpd_after_update2 - 0.06f) < 0.005f);
    
    // 持续提供一致的WPD测量值，应逐渐收敛
    for(int i=0; i<30; ++i) {
        drip_kf_test.update(1, DEFAULT_DT_DRIP, 0.06f); // 1滴，0.06g
    }
    TEST_ASSERT_FLOAT_WITHIN(0.002f, 0.06f, drip_kf_test.getCalibratedWeightPerDrop());

    drip_kf_test.stopWpdCalibration();
    TEST_ASSERT_EQUAL(false, drip_kf_test.isWpdCalibrating());
    float final_wpd = drip_kf_test.getCalibratedWeightPerDrop();
    // 停止校准后，WPD不应再改变
    drip_kf_test.update(1, DEFAULT_DT_DRIP, 0.1f); // 即使有重量变化，因为不在校准模式，WPD不变
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_PRECISION_WPD, final_wpd, drip_kf_test.getCalibratedWeightPerDrop());
}

// 测试WPD校准中滴数为0的情况
void test_drip_kf_wpd_calibration_zero_drops(void) {
    drip_kf_test.startWpdCalibration();
    float initial_wpd = drip_kf_test.getCalibratedWeightPerDrop();
    drip_kf_test.update(0, DEFAULT_DT_DRIP, 0.1f); // 0滴，但有重量变化
    // WPD不应该改变，因为没有滴数来计算 measured_wpd
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_PRECISION_WPD, initial_wpd, drip_kf_test.getCalibratedWeightPerDrop());
    drip_kf_test.stopWpdCalibration();
}

// 测试流速计算
void test_drip_kf_flow_rate_calculation(void) {
    // 假设滴速为 2 dps, WPD 为 0.05 g/drip
    // 期望流速 = 2 * 0.05 = 0.1 g/s
    drip_kf_test.init(0.0f, 0.05f); // WPD = 0.05
    for (int i = 0; i < 30; ++i) { // 让滴速滤波器稳定到 2 dps
        drip_kf_test.update(1, DEFAULT_DT_DRIP, 0.0f); // 1 drop / 0.5s = 2 dps
    }
    TEST_ASSERT_FLOAT_WITHIN(0.2f, 2.0f, drip_kf_test.getFilteredDripRate());
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_PRECISION_WPD, 0.05f, drip_kf_test.getCalibratedWeightPerDrop());
    
    float expected_flow_gps = 2.0f * 0.05f;
    TEST_ASSERT_FLOAT_WITHIN(0.005f, expected_flow_gps, drip_kf_test.getFlowRateGramsPerSecond());

    // 测试mL/h转换 (假设密度为1.0 g/mL)
    // 0.1 g/s = 0.1 mL/s = 0.1 * 3600 mL/h = 360 mL/h
    drip_kf_test.setDefaultLiquidDensity(1.0f);
    float expected_flow_mlh = expected_flow_gps / 1.0f * 3600.0f;
    TEST_ASSERT_FLOAT_WITHIN(20.0f, expected_flow_mlh, drip_kf_test.getFlowRateMlPerHour()); 
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_drip_kf_initialization_and_default_wpd);
    RUN_TEST(test_drip_kf_zero_dt_update);
    RUN_TEST(test_drip_kf_constant_drip_rate);
    RUN_TEST(test_drip_kf_wpd_calibration);
    RUN_TEST(test_drip_kf_wpd_calibration_zero_drops);
    RUN_TEST(test_drip_kf_flow_rate_calculation);
    UNITY_END();
    return 0;
} 