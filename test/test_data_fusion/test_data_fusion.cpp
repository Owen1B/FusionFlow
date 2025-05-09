#include <unity.h>
#include "DataFusion.h"
#include <math.h> // For fabsf

// 定义测试中使用的常量
const float DEFAULT_DT_FUSION = 0.5f; // 模拟时间间隔 (s)
const float Q_PROCESS_FUSION = 0.0001f;
const float R_WEIGHT_FLOW_FUSION = 0.0025f;
const float R_DRIP_FLOW_FUSION = 0.0025f;
const float FLOAT_PRECISION_FUSION = 0.0001f;

DataFusion fusion_kf_test(Q_PROCESS_FUSION, R_WEIGHT_FLOW_FUSION, R_DRIP_FLOW_FUSION);

void setUp(void) {
    // 每个测试用例开始前运行
    fusion_kf_test.init(0.0f);
}

void tearDown(void) {
    // 每个测试用例结束后运行
}

// 测试初始化
void test_fusion_kf_initialization(void) {
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_PRECISION_FUSION, 0.0f, fusion_kf_test.getFusedFlowRateGramsPerSecond());
}

// 测试零时间间隔更新
void test_fusion_kf_zero_dt_update(void) {
    float initial_fused_rate = fusion_kf_test.getFusedFlowRateGramsPerSecond();
    fusion_kf_test.update(0.1f, 0.1f, 0.0f); // dt = 0
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_PRECISION_FUSION, initial_fused_rate, fusion_kf_test.getFusedFlowRateGramsPerSecond());
}

// 测试当两个传感器输入一致时
void test_fusion_kf_consistent_inputs(void) {
    float consistent_flow = 0.05f; // g/s
    for (int i = 0; i < 50; ++i) {
        fusion_kf_test.update(consistent_flow, consistent_flow, DEFAULT_DT_FUSION);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.005f, consistent_flow, fusion_kf_test.getFusedFlowRateGramsPerSecond());
}

// 测试当只有一个传感器提供有效数据时 (例如，滴速传感器流速为0)
void test_fusion_kf_one_sensor_active(void) {
    fusion_kf_test.init(0.0f); // 确保从0开始，避免之前测试状态影响
    float weight_flow = 0.08f; // g/s
    float drip_flow_zero = 0.0f; // g/s

    // 多次更新，期望融合结果主要受重量传感器影响
    for (int i = 0; i < 50; ++i) { // 增加迭代次数以更好地收敛
        fusion_kf_test.update(weight_flow, drip_flow_zero, DEFAULT_DT_FUSION);
    }
    // 当两个传感器的R值相同时，即使一个输入为0，融合结果也会趋向于两者的平均值。
    // 初始状态和Q的影响会使它不完全是平均值，但会接近。
    float expected_flow = (weight_flow + drip_flow_zero) / 2.0f; // 0.04f
    TEST_ASSERT_FLOAT_WITHIN(0.005f, expected_flow, fusion_kf_test.getFusedFlowRateGramsPerSecond());
}

// 测试当两个传感器输入不一致时，融合结果应在两者之间
void test_fusion_kf_conflicting_inputs(void) {
    float weight_flow = 0.1f;  // g/s
    float drip_flow = 0.05f; // g/s

    for (int i = 0; i < 50; ++i) {
        fusion_kf_test.update(weight_flow, drip_flow, DEFAULT_DT_FUSION);
    }
    float fused_rate = fusion_kf_test.getFusedFlowRateGramsPerSecond();
    // 期望融合结果在 drip_flow 和 weight_flow 之间
    // 由于R值相同，期望结果接近平均值 (0.075)，但会受过程噪声影响
    TEST_ASSERT_TRUE(fused_rate > drip_flow - 0.005f && fused_rate < weight_flow + 0.005f);
    TEST_ASSERT_FLOAT_WITHIN(0.015f, (weight_flow + drip_flow) / 2.0f, fused_rate);
}

// 测试不同R值对融合结果的影响
void test_fusion_kf_different_R_values(void) {
    float R_weight_high_confidence = 0.0001f; // 更信任重量传感器
    float R_drip_low_confidence = 0.01f;    // 不太信任滴速传感器
    DataFusion custom_fusion(Q_PROCESS_FUSION, R_weight_high_confidence, R_drip_low_confidence);
    custom_fusion.init(0.0f);

    float weight_flow = 0.1f;  // g/s
    float drip_flow = 0.02f; // g/s (与重量传感器差异较大)

    for (int i = 0; i < 50; ++i) {
        custom_fusion.update(weight_flow, drip_flow, DEFAULT_DT_FUSION);
    }
    float fused_rate = custom_fusion.getFusedFlowRateGramsPerSecond();
    // 期望融合结果更接近 weight_flow，因为其R值更小 (更可信)
    TEST_ASSERT_TRUE(fabsf(fused_rate - weight_flow) < fabsf(fused_rate - drip_flow));
    TEST_ASSERT_FLOAT_WITHIN(0.015f, weight_flow, fused_rate); // 应该比较接近 weight_flow
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_fusion_kf_initialization);
    RUN_TEST(test_fusion_kf_zero_dt_update);
    RUN_TEST(test_fusion_kf_consistent_inputs);
    RUN_TEST(test_fusion_kf_one_sensor_active);
    RUN_TEST(test_fusion_kf_conflicting_inputs);
    RUN_TEST(test_fusion_kf_different_R_values);
    UNITY_END();
    return 0;
} 