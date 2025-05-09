#include <unity.h>
#include "WeightKalmanFilter.h" // 确保路径正确，根据你的项目结构调整
#include <math.h> // For fabsf for float comparisons

// 定义测试中使用的常量
const float DEFAULT_DT_WEIGHT = 0.5f; // 模拟时间间隔 (s)
const float SIGMA_A_WEIGHT = 0.02f;
const float R_MEASURE_WEIGHT = 0.5f;
const float FLOAT_PRECISION = 0.001f; // 浮点数比较的精度

WeightKalmanFilter weight_kf_test(SIGMA_A_WEIGHT, R_MEASURE_WEIGHT);

void setUp(void) {
    // 每个测试用例开始前运行
    // 可以在这里重新初始化滤波器，如果需要的话
    // weight_kf_test.init(0.0f, 0.0f); // 或者在每个测试用例内部调用 init
}

void tearDown(void) {
    // 每个测试用例结束后运行
}

// 测试初始化
void test_weight_kf_initialization(void) {
    weight_kf_test.init(100.0f, 0.5f);
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_PRECISION, 100.0f, weight_kf_test.getWeight());
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_PRECISION, 0.5f, weight_kf_test.getVelocity());
}

// 测试零输入 (dt=0)
void test_weight_kf_zero_dt_update(void) {
    weight_kf_test.init(50.0f, 0.0f);
    float initial_weight = weight_kf_test.getWeight();
    float updated_weight = weight_kf_test.update(55.0f, 0.0f); // dt = 0
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_PRECISION, initial_weight, updated_weight);
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_PRECISION, 0.0f, weight_kf_test.getVelocity()); // 速度应保持不变
}

// 测试常量输入，滤波器应收敛
void test_weight_kf_constant_input_convergence(void) {
    weight_kf_test.init(0.0f, 0.0f);
    float constant_measurement = 75.0f;
    // 多次更新以观察收敛
    for (int i = 0; i < 50; ++i) {
        weight_kf_test.update(constant_measurement, DEFAULT_DT_WEIGHT);
    }
    TEST_ASSERT_FLOAT_WITHIN(1.0f, constant_measurement, weight_kf_test.getWeight()); // 允许一定的收敛误差
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, weight_kf_test.getVelocity()); // 速度应接近0
}

// 测试带噪声的输入，滤波器应平滑
void test_weight_kf_noisy_input_smoothing(void) {
    weight_kf_test.init(100.0f, 0.0f);
    float base_weight = 100.0f;
    float noise_amplitude = 5.0f;
    float total_filtered_weight_change = 0.0f;
    float previous_filtered_weight = weight_kf_test.getWeight();

    for (int i = 0; i < 20; ++i) {
        float noisy_measurement = base_weight + ((i % 2 == 0) ? noise_amplitude : -noise_amplitude);
        weight_kf_test.update(noisy_measurement, DEFAULT_DT_WEIGHT);
        total_filtered_weight_change += fabsf(weight_kf_test.getWeight() - previous_filtered_weight);
        previous_filtered_weight = weight_kf_test.getWeight();
    }
    // 期望滤波后的总变化远小于原始噪声的总变化
    // 这是一个定性的检查，具体阈值取决于滤波器参数和噪声特性
    TEST_ASSERT_TRUE(total_filtered_weight_change < (20 * noise_amplitude * 0.5f)); 
    // 并且最终重量应接近基准重量
    TEST_ASSERT_FLOAT_WITHIN(noise_amplitude, base_weight, weight_kf_test.getWeight());
}

// 测试斜坡输入 (重量稳定减少)，滤波器应跟踪并估计速度
void test_weight_kf_ramp_input_tracking(void) {
    float initial_weight = 200.0f;
    float actual_velocity = -0.5f; // 重量每秒减少0.5g
    weight_kf_test.init(initial_weight, 0.0f); // 初始速度估计为0

    float current_actual_weight = initial_weight;
    for (int i = 0; i < 30; ++i) {
        current_actual_weight += actual_velocity * DEFAULT_DT_WEIGHT;
        weight_kf_test.update(current_actual_weight, DEFAULT_DT_WEIGHT);
    }
    
    // 经过多次更新后，滤波后的重量应接近当前实际重量
    TEST_ASSERT_FLOAT_WITHIN(5.0f, current_actual_weight, weight_kf_test.getWeight()); // 允许一定的跟踪误差
    // 估计的速度应接近实际速度
    TEST_ASSERT_FLOAT_WITHIN(0.2f, actual_velocity, weight_kf_test.getVelocity());
}


int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_weight_kf_initialization);
    RUN_TEST(test_weight_kf_zero_dt_update);
    RUN_TEST(test_weight_kf_constant_input_convergence);
    RUN_TEST(test_weight_kf_noisy_input_smoothing);
    RUN_TEST(test_weight_kf_ramp_input_tracking);
    UNITY_END();
    return 0;
}