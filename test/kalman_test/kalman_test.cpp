#include <Arduino.h>
#include "KalmanFilter.h"
#include "HX711.h"

// HX711引脚定义
const int DT_PIN = 17;
const int SCK_PIN = 18;
float calibration_factor = 1687.0;
HX711 scale;

// 创建卡尔曼滤波器实例
// 参数说明：
// posNoise: 0.001 - 位置过程噪声，较小的值使位置估计更稳定
// velNoise: 0.001 - 速度过程噪声，较小的值使速度估计更平滑
// measurementNoise: 0.1 - 测量噪声，较大的值使滤波器对测量值更不敏感
KalmanFilter kalmanFilter(0.001, 0.001, 0.1);

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);  // 等待串口准备好，但最多等3秒
    
    Serial.println("初始化HX711...");
    // 初始化HX711
    scale.begin(DT_PIN, SCK_PIN);
    scale.set_scale(calibration_factor);
    scale.set_gain(128);
    
    // 多次读取以稳定初始值
    if(scale.is_ready()) {
        Serial.println("读取初始值...");
        float initialWeight = scale.get_units(10);
        Serial.print("初始重量: ");
        Serial.println(initialWeight);
        kalmanFilter.update(initialWeight);  // 初始化卡尔曼滤波器
    } else {
        Serial.println("错误：HX711未就绪！");
    }
    
    Serial.println("\n开始数据采集...");
    Serial.println("时间戳(ms),原始重量(g),滤波后重量(g),估计速度(g/s)");
}

void loop() {
    if (scale.is_ready()) {
        // 读取原始重量
        float rawWeight = scale.get_units(10);
        
        // 应用卡尔曼滤波
        float filteredWeight = kalmanFilter.update(rawWeight);
        float estimatedVelocity = kalmanFilter.getVelocity();
        
        // 输出数据
        Serial.print(millis());
        Serial.print(",");
        Serial.print(rawWeight, 1);
        Serial.print(",");
        Serial.print(filteredWeight, 1);
        Serial.print(",");
        Serial.println(estimatedVelocity, 3);
        
        // 如果重量变化超过阈值，打印警告
        if (abs(estimatedVelocity) > 1.0) {
            Serial.print("警告：检测到快速重量变化！速度 = ");
            Serial.print(estimatedVelocity, 3);
            Serial.println(" g/s");
        }
    } else {
        Serial.println("错误：HX711未就绪！");
    }
    
    delay(1000);  // 每秒读取一次
} 