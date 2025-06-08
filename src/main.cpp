// ==========================
// 1. 标准库和第三方库引用
// ==========================
#include <WiFi.h>                  // WiFi 连接
#include <WiFiClient.h>            // WiFi 客户端
#include <DNSServer.h>             // DNS 服务器
#include <WebSocketsServer.h>      // WebSocket 服务器
#include <Wire.h>                  // I2C 通讯
#include <Adafruit_GFX.h>          // GFX 图形库
#include <U8g2lib.h>               // OLED 驱动库
#include <Adafruit_NeoPixel.h>     // NeoPixel RGB LED 库
#include "HX711.h"                 // HX711 称重传感器库
#include <HTTPClient.h>            // HTTP 客户端
#include <ArduinoJson.h>           // JSON 处理

// ==========================
// 2. 自定义模块引用
// ==========================
#include "WeightKalmanFilter.h"    // 重量卡尔曼滤波器
#include "DripKalmanFilter.h"      // 滴速卡尔曼滤波器
#include "DataFusion.h"            // 数据融合模块
#include "webpage.h"               // 网页内容

// ==========================
// 3. 设备配置
// ==========================
const char* DEVICE_ID = "001";     // 设备ID
const char* API_BASE_URL = "http://114.66.55.73:31504"; // API基础URL

// WiFi配置
constexpr const char* WIFI_SSID  = "1503";           // WiFi SSID
constexpr const char* WIFI_PASS  = "18310007230";    // WiFi 密码
constexpr unsigned long MOTOR_RUN_DURATION_MS = 2000; // 电机转动时长

// 硬件引脚配置
constexpr int PIN_WATER_SENSOR   = 11;          // 水滴传感器引脚
constexpr int NEOPIXEL_PIN       = 48;          // NeoPixel 数据引脚
constexpr int PIN_I2C_SDA        = 36;          // I2C SDA 引脚（OLED）
constexpr int PIN_I2C_SCL        = 1;           // I2C SCL 引脚（OLED）
constexpr int PIN_HX711_DT       = 17;          // HX711 数据引脚
constexpr int PIN_HX711_SCK      = 18;          // HX711 时钟引脚
constexpr int PIN_INIT_BUTTON    = 15;          // 初始化按钮引脚
constexpr int MOTOR_PIN1         = 6;           // 电机控制引脚1
constexpr int MOTOR_PIN2         = 7;           // 电机控制引脚2

// ==========================
// 4. 函数声明
// ==========================

// 滴落中断服务函数
void IRAM_ATTR onWaterDropIsr();

// 滴落时间戳队列操作函数
bool getNextDripTimestamp(unsigned long& ts);
int getTimestampQueueSize();

// WebSocket事件回调函数
void onWebSocketEvent(uint8_t clientNum, WStype_t type, uint8_t * payload, size_t length);

// OLED显示更新函数
void updateOledDisplay();

// WiFi连接处理函数
void connectToWiFi();

// HTTP请求处理函数
void handleHttpRequests();

// 剩余时间计算函数
float calculate_specific_remaining_time(float current_liquid_weight, float target_empty_ref_weight, 
                                      float current_flow_rate_gps, float undefined_time_value = 88888.0f);

// 系统重新初始化函数
void performSystemReinitialization();

// 计算总输液量函数
void calculateTotalVolume(float initial_weight_g);

// 电机控制函数
void startMotorForward();
void startMotorReverse();
void stopMotor();
void handleMotor();
void toggleMotorState();

// 主循环相关函数声明
void handleButtonInputs(unsigned long current_time_ms);
void checkInfusionAbnormality(unsigned long current_time_ms);
void handleFastConvergenceMode(unsigned long current_time_ms);
void updateLedStatus();
void handleMainProcessing(unsigned long current_time_ms);
void handleWeightSensor();
void handleDripRate(float dt_main_loop_s);
void handleDataFusion(float dt_main_loop_s);
void handleWpdLongCalibration();
void calculateRemainingTime();
void updateDisplay();
void outputData(unsigned long current_time_ms);
void uploadDataToServer();

// ==========================
// 5. 配置参数
// ==========================

// 系统状态枚举定义
enum class SystemState {
    INITIALIZING,    // 初始化中
    INIT_ERROR,      // 初始化异常
    NORMAL,          // 正常输液
    INFUSION_ERROR,  // 输液异常
    FAST_CONVERGENCE,// 快速收敛
    COMPLETED        // 已完成
};



// 服务器配置
constexpr uint16_t WEBSOCKET_PORT = 81;              // WebSocket 服务器端口
constexpr uint16_t HTTP_PORT = 80;                   // HTTP 服务器端口

// 异常检测配置
constexpr unsigned long MAX_NO_DRIP_INTERVAL_MS = 10000;   // 无滴落最大间隔（10秒）
constexpr int PIN_ABNORMALITY_RESET_BUTTON = 0;           // 异常复位按钮 GPIO
constexpr unsigned long ABNORMALITY_RESET_BUTTON_DEBOUNCE_MS = 200; // 按钮去抖时间
constexpr unsigned long ABNORMALITY_RESET_BUTTON_LONG_PRESS_MS = 1000; // 长按时间


// NeoPixel 颜色配置（GRB 格式）
constexpr int NEOPIXEL_BRIGHTNESS = 50;        // NeoPixel 亮度（0-255）
#define NEO_COLOR_OFF     pixels.Color(0,   0,   0)     // 关闭
#define NEO_COLOR_RED     pixels.Color(255, 0,   0)     // 红色
#define NEO_COLOR_GREEN   pixels.Color(0,   255, 0)     // 绿色
#define NEO_COLOR_BLUE    pixels.Color(0,   0,   255)   // 蓝色
#define NEO_COLOR_YELLOW  pixels.Color(255, 255, 0)     // 黄色
#define NEO_COLOR_WHITE   pixels.Color(255, 255, 255)   // 白色

// 滴速时间戳队列配置
constexpr int MAX_TIMESTAMP_QUEUE_SIZE = 20;                // 时间戳队列最大容量

// HX711 重量传感器配置
float hx711_cal_factor = 1687.0f;                           // HX711 校准因子

// OLED 显示屏配置
constexpr int OLED_WIDTH  = 128;                            // OLED 宽度（像素）
constexpr int OLED_HEIGHT = 32;                             // OLED 高度（像素）

// 卡尔曼滤波器参数配置
constexpr float KF_WEIGHT_SIGMA_A  = 0.0005f;               // 重量 KF 过程噪声（速度）
constexpr float KF_WEIGHT_SIGMA_J  = 1e-6f;                 // 重量 KF 过程噪声（加速度）
constexpr float KF_WEIGHT_R_NOISE  = 50.0f;                 // 重量 KF 测量噪声

// 滴速 KF 参数
constexpr float DRIP_KF_SIGMA_A  = 0.00001f;                // 滴速 KF 过程噪声
constexpr float DRIP_KF_R        = 0.05f;                   // 滴速 KF 测量噪声
constexpr float DRIP_KF_WPD_Q    = 0.00000001f;             // 滴速 KF WPD 过程噪声
constexpr float DRIP_KF_WPD_R    = 0.0001f;                 // 滴速 KF WPD 测量噪声

// 数据融合参数
constexpr float FUSION_Q_FLOW           = 0.0000001f;       // 融合流速过程噪声
constexpr float FUSION_R_WEIGHT_FLOW    = 0.01f;            // 融合重量流速测量噪声
constexpr float FUSION_R_DRIP_FLOW      = 0.0005f;          // 融合滴速流速测量噪声
constexpr float FUSION_Q_WEIGHT         = 0.01f;            // 融合重量过程噪声
constexpr float FUSION_R_WEIGHT_WEIGHT  = 1.0f;             // 融合重量测量噪声
constexpr float FUSION_R_DRIP_WEIGHT    = 1.0f;             // 融合滴速重量测量噪声

// 主循环与定时相关配置
constexpr unsigned long MAIN_LOOP_INTERVAL_MS         = 1000;   // 主循环周期（ms）
constexpr unsigned long INIT_BUTTON_DEBOUNCE_MS      = 200;    // 初始化按钮去抖时间
constexpr unsigned long FAST_CONVERGENCE_DURATION_MS = 60000;  // 快速收敛时长

// WPD 长时间校准配置
constexpr unsigned long WPD_LONG_CAL_DURATION_MS = 60000;       // WPD 长时间校准目标时长（ms）
constexpr int WPD_LONG_CAL_MIN_DROPS   = 30;                    // WPD 长时间校准最小滴数

// 重量相关配置
constexpr float EQUIPMENT_TARE_G = 12.0f;                       // 传感器上固定器材的皮重
constexpr float EMPTY_BAG_TARE_G = 60.0f;                       // 输液袋空袋时的皮重
const float TOTAL_TARE_G = EQUIPMENT_TARE_G + EMPTY_BAG_TARE_G; // 总皮重

// 目标相关配置
float target_empty_weight_g = 0.0f;                            // 目标空袋重量（g），现在代表纯液体重量为0
constexpr unsigned long DEFAULT_TARGET_TOTAL_DROPS_VOLUME_CALC = 100; // 默认目标滴数

// 系统参数
constexpr unsigned long ABNORMALITY_CHECK_INTERVAL_MS = 10000; // 输液异常检测间隔（10秒）
constexpr unsigned long NO_DRIP_TIMEOUT_MS = 10000;           // 无滴落超时时间（10秒）

// ==========================
// 6. 全局变量
// ==========================

// 网络与连接状态
bool wifi_connected_flag = false;      // WiFi 连接标志
bool ws_client_connected_flag = false; // WebSocket 客户端连接标志
bool auto_clamp = false;              // 自动夹断状态

// 服务器对象
WiFiServer http_server(HTTP_PORT);     // HTTP 服务器对象
WebSocketsServer ws_server(WEBSOCKET_PORT); // WebSocket 服务器对象

// 外设对象
Adafruit_NeoPixel pixels(1, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800); // NeoPixel 对象
U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C oled_display(U8G2_R0, U8X8_PIN_NONE); // OLED 对象
HX711 scale_sensor;                    // HX711 对象

// 滴速时间戳队列
unsigned long drip_timestamps_ms[MAX_TIMESTAMP_QUEUE_SIZE]; // 滴落时间戳队列
volatile int timestamp_queue_head = 0;     // 队列头指针
volatile int timestamp_queue_tail = 0;     // 队列尾指针
volatile bool timestamp_queue_full = false; // 队列满标志

// 卡尔曼滤波器与数据融合对象
WeightKalmanFilter weight_kf(KF_WEIGHT_SIGMA_A, KF_WEIGHT_SIGMA_J, KF_WEIGHT_R_NOISE);
DripKalmanFilter drip_kf(DRIP_KF_SIGMA_A, DRIP_KF_R, DRIP_KF_WPD_Q, DRIP_KF_WPD_R);
DataFusion flow_fusion(FUSION_Q_FLOW, FUSION_R_WEIGHT_FLOW, FUSION_R_DRIP_FLOW,
                      FUSION_Q_WEIGHT, FUSION_R_WEIGHT_WEIGHT, FUSION_R_DRIP_WEIGHT);

// 滴速传感器中断相关
volatile unsigned long isr_drop_count_period = 0;     // ISR 周期内累计滴数
volatile unsigned long isr_last_drop_time_ms = 0;     // ISR 上次滴落时间戳
volatile unsigned long last_drip_detected_time_ms = 0; // 上次检测到滴落的时间戳
volatile unsigned long last_abnormality_check_time_ms = 0; // 上次异常检测时间戳

// 异常检测与复位按钮状态
bool infusion_abnormal = false;                       // 输液异常标志
int last_abnormality_reset_button_state = HIGH;       // 上次按钮状态
unsigned long last_abnormality_reset_button_press_time = 0; // 上次按钮按下时间

// 主循环与定时相关
unsigned long last_loop_run_ms = 0;                   // 上次主循环时间戳
unsigned long last_calc_time_ms = millis();           // 上次计算时间
unsigned long last_serial_print_time_ms = millis();   // 上次串口打印时间
unsigned long last_drip_count_update_time_ms = millis(); // 上次滴数更新时间
unsigned long last_button_check_time_ms = millis();   // 上次按钮检测时间

// 初始化按钮与快速收敛相关
unsigned long last_init_button_press_time = 0;        // 上次初始化按钮按下时间
int last_init_button_state = HIGH;                    // 上次初始化按钮状态
bool fast_convergence_mode = false;                   // 快速收敛模式标志
unsigned long fast_convergence_start_ms = 0;          // 快速收敛开始时间
float original_kf_weight_R_noise = KF_WEIGHT_R_NOISE; // KF 原始 R 值备份

// 滴速与重量相关全局变量
float accumulated_weight_change_for_wpd_g = 0.0f;     // 滴速 KF 更新以来累计重量变化
unsigned int drops_this_drip_cycle = 0;               // 当前滴速周期内滴数
float raw_weight_g = 0.0f;                           // HX711 原始重量（g）
float prev_raw_weight_g = 0.0f;                      // 上周期原始重量（g）
float filt_weight_g = 0.0f;                          // 卡尔曼滤波后重量（g）
float prev_filt_weight_g = 0.0f;                     // 上周期滤波后重量（g）
float flow_weight_gps = 0.0f;                        // 重量传感器估算流速（g/s）
float raw_flow_weight_gps = 0.0f;                    // 原始重量估算流速（g/s）
float raw_drip_rate_dps = 0.0f;                      // 原始滴速（滴/秒）
float filt_drip_rate_dps = 0.0f;                     // 卡尔曼滤波后滴速（滴/秒）
float flow_drip_gps = 0.0f;                          // 滴速传感器估算流速（g/s）
float raw_flow_drip_gps = 0.0f;                      // 原始滴速估算流速（g/s）
float fused_flow_rate_gps = 0.0f;                    // 融合后流速（g/s）
float fused_remaining_weight_g = 0.0f;               // 融合后剩余重量（g）
float remaining_weight_drip_calc_g = 0.0f;           // 滴速计算剩余重量（g）

// 剩余时间与目标相关全局变量
float remaining_time_s = 0.0f;                       // 预计剩余输液时间（s）
float remaining_time_raw_weight_s = 0.0f;            // 原始重量流速剩余时间
float remaining_time_filt_weight_s = 0.0f;           // 滤波后重量流速剩余时间
float remaining_time_raw_drip_s = 0.0f;              // 原始滴速流速剩余时间
float remaining_time_filt_drip_s = 0.0f;             // 滤波后滴速流速剩余时间

// WPD 长时间校准相关全局变量
bool wpd_long_cal_active = false;                    // WPD 长时间校准激活标志
unsigned long wpd_long_cal_start_ms = 0;             // WPD 长时间校准开始时间
int wpd_long_cal_accum_drops = 0;                    // WPD 长时间校准累计滴数

// 系统级初始重量与报警相关全局变量
bool system_initial_weight_set = false;              // 系统初始重量已设置标志
float system_initial_total_liquid_weight_g = 0.0f;   // 系统初始总液体重量
bool alert_5_percent_triggered = false;              // 5% 报警已触发标志

// 统计与计数相关全局变量
unsigned long drip_total_drops = 0;                  // 总滴数

// 滤波器参数备份全局变量
static float original_drip_kf_R_drip_rate_noise = 0.0f;  // DripKalmanFilter 原始 R 值备份
static float original_drip_kf_R_wpd_noise = 0.0f;        // DripKalmanFilter 原始 WPD 噪声备份
static float original_fusion_R_weight_flow = 0.0f;       // DataFusion 原始 R 值备份
static float original_fusion_R_drip_flow = 0.0f;
static float original_fusion_R_weight_weight = 0.0f;
static float original_fusion_R_drip_weight = 0.0f;

// OLED 显示与全局状态
float g_infusion_progress = 0.0f;                    // 输液进度（0.0~1.0）
float g_oled_infused_progress_percent = 0.0f;        // OLED 显示百分比（0~100）
float g_oled_flow_rate_mlh = 0.0f;                   // OLED 显示流速（mL/h）
long g_oled_remaining_time_min = -1;                 // OLED 显示剩余分钟数（-1 为无）

// 系统状态
SystemState current_system_state = SystemState::INITIALIZING;  // 当前系统状态

// 电机状态
enum class MotorDirection { FORWARD, REVERSE };
static MotorDirection next_motor_direction = MotorDirection::FORWARD;
static bool motor_is_running = false;
static unsigned long motor_start_time_ms = 0;

// 全局变量
float total_volume_ml = 0.0f;  // 总输液量(ml)

// ==========================
// 7. 函数实现
// ==========================

// 滴落中断服务函数
void IRAM_ATTR onWaterDropIsr() {
    unsigned long now = millis();
    // 消抖：两次滴落间隔需大于50ms
    if (now - isr_last_drop_time_ms > 50) {
        int nextTail = (timestamp_queue_tail + 1) % MAX_TIMESTAMP_QUEUE_SIZE;
        if (nextTail != timestamp_queue_head || !timestamp_queue_full) {
            drip_timestamps_ms[timestamp_queue_tail] = now;
            timestamp_queue_tail = nextTail;
            timestamp_queue_full = (timestamp_queue_tail == timestamp_queue_head);
        }
        isr_last_drop_time_ms = now;
        last_drip_detected_time_ms = now;
    }
}

// 从队列中取出下一个滴落时间戳
bool getNextDripTimestamp(unsigned long& ts) {
    if (timestamp_queue_head == timestamp_queue_tail && !timestamp_queue_full) {
        return false;
    }
    ts = drip_timestamps_ms[timestamp_queue_head];
    timestamp_queue_head = (timestamp_queue_head + 1) % MAX_TIMESTAMP_QUEUE_SIZE;
    timestamp_queue_full = false;
    return true;
}

// 获取队列中当前滴落时间戳数量
int getTimestampQueueSize() {
    if (timestamp_queue_full) {
        return MAX_TIMESTAMP_QUEUE_SIZE;
    }
    return (timestamp_queue_tail - timestamp_queue_head + MAX_TIMESTAMP_QUEUE_SIZE) % MAX_TIMESTAMP_QUEUE_SIZE;
}

// WebSocket事件处理
void onWebSocketEvent(uint8_t clientNum, WStype_t type, uint8_t * payload, size_t length) {
    switch(type) {
        case WStype_DISCONNECTED: 
            if (clientNum == 0) ws_client_connected_flag = false;
            Serial.printf("[%u] WebSocket客户端断开连接!\n", clientNum);
            break;
            
        case WStype_CONNECTED: { 
            IPAddress ip = ws_server.remoteIP(clientNum);
            Serial.printf("[%u] WebSocket客户端已连接, IP: %s\n", clientNum, ip.toString().c_str());
            if (clientNum == 0) ws_client_connected_flag = true;
            break;
        }
        
        case WStype_TEXT: 
            Serial.printf("[%u] 收到文本: %s\n", clientNum, payload);
            if (strcmp((char*)payload, "CALIBRATE_WPD_START") == 0) {
                if (wpd_long_cal_active) {
                    Serial.printf("WPD校准已在进行中。\n");
                    ws_server.sendTXT(clientNum, "EVENT:WPD_CALIBRATION_ALREADY_RUNNING");
                } else {
                    Serial.printf("收到WPD校准开始命令, 启动长时校准...\n");
                    drip_kf.startWpdCalibration();
                    wpd_long_cal_active = true;
                    wpd_long_cal_start_ms = millis();
                    wpd_long_cal_accum_drops = 0;
                    ws_server.sendTXT(clientNum, "CMD_ACK:WPD_LONG_CALIBRATION_STARTED");
                }
            } else if (strcmp((char*)payload, "CALIBRATE_WPD_STOP") == 0) {
                if (wpd_long_cal_active) {
                    Serial.printf("收到WPD校准手动停止命令...\n");
                    drip_kf.stopWpdCalibration();
                    wpd_long_cal_active = false;
                    float wpd = drip_kf.getCalibratedWeightPerDrop();
                    char msg[100];
                    snprintf(msg, sizeof(msg), "CMD_ACK:WPD_CALIBRATION_STOPPED_MANUALLY,CurrentWPD:%.4f", wpd);
                    ws_server.sendTXT(clientNum, msg);
                    Serial.printf("WPD校准手动停止。当前WPD: %.4f g/drip\n", wpd);
                } else {
                    Serial.printf("WPD校准未在进行中, 无需停止。\n");
                    ws_server.sendTXT(clientNum, "EVENT:WPD_CALIBRATION_NOT_RUNNING");
                }
            } else if (strncmp((char*)payload, "SET_TOTAL_VOLUME:", 16) == 0) {
                // 处理设置总输液量的命令
                float new_volume = atof((char*)payload + 16);
                if (new_volume > 0) {
                    total_volume_ml = new_volume;
                    Serial.printf("总输液量已设置为: %.1f ml\n", total_volume_ml);
                    char msg[100];
                    snprintf(msg, sizeof(msg), "CMD_ACK:TOTAL_VOLUME_SET,Volume:%.1f", total_volume_ml);
                    ws_server.sendTXT(clientNum, msg);
            } else {
                    ws_server.sendTXT(clientNum, "ERROR:INVALID_VOLUME");
                }
            } else {
                ws_server.sendTXT(clientNum, "CMD_UNKNOWN");
            }
            break;
            
        case WStype_BIN: 
            Serial.printf("[%u] 收到二进制数据, 长度: %u\n", clientNum, length);
            break;
            
        default:
            break;
    }
}

// OLED显示更新
void updateOledDisplay() {
    oled_display.clearBuffer();
    oled_display.setFont(u8g2_font_wqy12_t_gb2312);
    char buf[32];

    // 第一行：IP
    oled_display.setCursor(0, 12);
    if (wifi_connected_flag) {
        snprintf(buf, sizeof(buf), "%s", WiFi.localIP().toString().c_str());
    } else {
        snprintf(buf, sizeof(buf), "WiFi未连接");
    }
    oled_display.print(buf);
    
    // 第二行：重量、流速和剩余时间
    oled_display.setCursor(0, 26);
    if (g_oled_remaining_time_min >= 0) {
        snprintf(buf, sizeof(buf), "%.1fg %.2fg/s %ldmin", filt_weight_g, fused_flow_rate_gps, g_oled_remaining_time_min);
    } else {
        snprintf(buf, sizeof(buf), "%.1fg %.2fg/s 计算中", filt_weight_g, fused_flow_rate_gps);
    }
    oled_display.print(buf);

    oled_display.sendBuffer();
}

// WiFi连接处理
void connectToWiFi() {
    Serial.printf("正在连接WiFi: %s\n", WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    int tryCount = 0;
    while (WiFi.status() != WL_CONNECTED && tryCount < 20) {
        delay(500);
        Serial.print(".");
        tryCount++;
    }
    if (WiFi.status() == WL_CONNECTED) {
        wifi_connected_flag = true;
        Serial.printf("\nWiFi 已连接\n");
        Serial.printf("IP 地址: %s\n", WiFi.localIP().toString().c_str());
    } else {
        wifi_connected_flag = false;
        Serial.printf("\nWiFi 连接失败\n");
    }
}

// HTTP请求处理
void handleHttpRequests() {
    WiFiClient client = http_server.available();
    if (!client) return;
    
    Serial.printf("新的HTTP客户端已连接!\n");
    unsigned long current_http_req_time = millis();
    unsigned long previous_http_req_time = current_http_req_time;
    String current_line = "";

    while (client.connected() && (current_http_req_time - previous_http_req_time < 2000)) {
        current_http_req_time = millis();
        if (client.available()) {
            char c = client.read();
            if (c == '\n') {
                if (current_line.length() == 0) {
                    client.println("HTTP/1.1 200 OK");
                    client.println("Content-type:text/html; charset=UTF-8");
                    client.println("Connection: close");
                    client.println();
                    
                    const int chunkSize = 256;
                    int totalLen = strlen(HTML_WEBPAGE);
                    for (int i = 0; i < totalLen; i += chunkSize) {
                        client.print(String(HTML_WEBPAGE).substring(i, min(i + chunkSize, totalLen)));
                    }
                    client.println();
                    break;
        } else {
                    current_line = "";
                }
            } else if (c != '\r') {
                current_line += c;
            }
            previous_http_req_time = current_http_req_time;
        }
    }
    
    client.stop();
    Serial.printf("HTTP客户端已断开连接。\n");
}

// 计算特定条件下的剩余时间
float calculate_specific_remaining_time(float current_liquid_weight, float target_empty_ref_weight, 
                                      float current_flow_rate_gps, float undefined_time_value) {
    float weight_to_infuse = current_liquid_weight - target_empty_ref_weight;
    if (weight_to_infuse <= 0.01f) {
        return 0.0f;
    }
    if (current_flow_rate_gps > 1e-5f) {
        float time = weight_to_infuse / current_flow_rate_gps;
        return (time < 0.0f) ? 0.0f : ((time > 999999.0f) ? 999999.0f : time);
    } else {
        return undefined_time_value;
    }
}

// 计算总输液量（向上取整到100的倍数）
void calculateTotalVolume(float initial_weight_g) {
    // 将重量转换为ml（1g=1ml），然后向上取整到100的倍数
    float volume_ml = initial_weight_g;
    total_volume_ml = ceil(volume_ml / 100.0f) * 100.0f;
    Serial.printf("初始重量: %.1fg, 计算得到总输液量: %.1fml\n", initial_weight_g, total_volume_ml);
}

// 系统重新初始化函数
void performSystemReinitialization() {
    static int reinit_error_count = 0; // 记录连续异常次数
    const int REINIT_ERROR_THRESHOLD = 3; // 异常阈值，超过则锁定系统
    Serial.printf("系统重新初始化请求...\n"); // 打印初始化请求信息
    current_system_state = SystemState::INITIALIZING;  // 设置为初始化中状态

    // 重置所有状态变量
    infusion_abnormal = false; // 重置输液异常标志
    fast_convergence_mode = false; // 关闭快速收敛模式
    system_initial_weight_set = false; // 初始重量未设置
    system_initial_total_liquid_weight_g = 0.0f; // 初始总液体重量清零
    
    // 重置传感器
    scale_sensor.begin(PIN_HX711_DT, PIN_HX711_SCK); // 初始化HX711
    scale_sensor.set_scale(hx711_cal_factor); // 设置校准因子
    scale_sensor.set_gain(128); // 设置增益
    scale_sensor.set_offset(0); // 设置偏置为0

    // 检查称重传感器是否就绪
    if (!scale_sensor.is_ready()) { // 如果HX711未就绪
        Serial.printf("警告: 重新初始化时 HX711 未就绪, 系统进入异常状态。\n"); // 打印警告
        current_system_state = SystemState::INIT_ERROR; // 设置为初始化错误状态
        reinit_error_count++; // 异常计数加1
        return; // 退出函数
    }

    // 读取初始重量
    float gross_weight_reading = scale_sensor.get_units(10); // 读取总重
    float initial_weight_reading = gross_weight_reading - TOTAL_TARE_G; // 扣除总皮重，得到纯液体重量
    
    // 检查重量读数是否异常
    if (isnan(initial_weight_reading) || isinf(initial_weight_reading) ||  // 检查是否为无效数
        fabsf(initial_weight_reading) > 5000.0f ||  // 检查是否超出合理范围
        initial_weight_reading <= 10.0f) { // 检查纯液体重量是否小于10g
        Serial.printf("警告: 重新初始化时纯液体重量读数异常: %.2f, 系统进入异常状态。\n", initial_weight_reading); // 打印警告
        current_system_state = SystemState::INIT_ERROR; // 设置为初始化错误状态
        reinit_error_count++; // 异常计数加1
        return; // 退出函数
    }

    // 计算总输液量
    calculateTotalVolume(initial_weight_reading);

    // 重置异常计数
    reinit_error_count = 0; // 异常计数清零

    // 更新系统状态
    raw_weight_g = initial_weight_reading; // 原始重量赋值
    filt_weight_g = raw_weight_g; // 滤波后重量赋值
    prev_filt_weight_g = raw_weight_g; // 上一次滤波后重量赋值
    prev_raw_weight_g = raw_weight_g; // 上一次原始重量赋值

    // 设置系统初始重量
    system_initial_total_liquid_weight_g = raw_weight_g; // 设置初始总液体重量
    system_initial_weight_set = true; // 标记初始重量已设置

    // 初始化各滤波器
    weight_kf.init(raw_weight_g, 0.0f, 0.0f); // 初始化重量卡尔曼滤波器
    drip_kf.init(0.0f, -1.0f, 20, 1.0f); // 初始化滴速卡尔曼滤波器
    drip_kf.setInitialLiquidWeightForVolumeCalc(system_initial_total_liquid_weight_g); // 设置体积计算初始重量
    flow_fusion.init(0.0f, raw_weight_g); // 初始化数据融合模块

    // 设置WPD校准
    drip_kf.startWpdCalibration(); // 启动WPD校准
    drip_kf.total_drops_for_volume_calc = 0; // 清空累积总滴数

    // 停止WPD校准相关状态
    wpd_long_cal_active = false; // 长时间WPD校准标志关闭

    // 进入快速收敛模式
    current_system_state = SystemState::FAST_CONVERGENCE; // 设置为快速收敛状态
    fast_convergence_mode = true; // 启用快速收敛模式
    fast_convergence_start_ms = millis(); // 记录快速收敛开始时间

    // 设置快速收敛模式下的滤波器参数
    float fast_R_weight = original_kf_weight_R_noise / 10.0f; // 快速收敛重量测量噪声
    weight_kf.setMeasurementNoise(fast_R_weight); // 设置重量滤波器测量噪声

    float fast_R_drip_rate = original_drip_kf_R_drip_rate_noise / 10.0f; // 快速收敛滴速测量噪声
    float fast_R_wpd = original_drip_kf_R_wpd_noise / 10.0f; // 快速收敛WPD测量噪声
    drip_kf.setDripRateMeasurementNoise(fast_R_drip_rate); // 设置滴速滤波器测量噪声
    drip_kf.setWpdMeasurementNoise(fast_R_wpd); // 设置WPD滤波器测量噪声

    float fast_R_fusion_w_flow = original_fusion_R_weight_flow / 10.0f; // 快速收敛重量流量噪声
    float fast_R_fusion_d_flow = original_fusion_R_drip_flow / 10.0f; // 快速收敛滴速流量噪声
    float fast_R_fusion_w_weight = original_fusion_R_weight_weight / 10.0f; // 快速收敛重量权重噪声
    float fast_R_fusion_d_weight = original_fusion_R_drip_weight / 10.0f; // 快速收敛滴速权重噪声
    flow_fusion.setFlowMeasurementNoises(fast_R_fusion_w_flow, fast_R_fusion_d_flow); // 设置流量测量噪声
    flow_fusion.setWeightMeasurementNoises(fast_R_fusion_w_weight, fast_R_fusion_d_weight); // 设置权重测量噪声

    // 刷新OLED显示
    updateOledDisplay(); // 刷新OLED显示内容

    // 通知WebSocket客户端
    if (wifi_connected_flag && ws_client_connected_flag) { // 如果WiFi和WebSocket已连接
        char reinit_msg[200]; // 定义消息缓冲区
        snprintf(reinit_msg, sizeof(reinit_msg), 
                 "警告: 系统已重新初始化。新的初始总重量: %.1fg, 目标空重: %.1fg。快速收敛模式已开启(60秒)。", 
                 system_initial_total_liquid_weight_g, target_empty_weight_g); // 生成初始化消息
        ws_server.broadcastTXT(reinit_msg); // 广播初始化消息
        char initial_params_msg[100]; // 定义参数消息缓冲区
        snprintf(initial_params_msg, sizeof(initial_params_msg), "初始参数:%.1f,%.1f,%.1f",
                 system_initial_total_liquid_weight_g, target_empty_weight_g, (float)DEFAULT_TARGET_TOTAL_DROPS_VOLUME_CALC); // 生成初始参数消息
        ws_server.broadcastTXT(initial_params_msg); // 广播初始参数消息
    }

    Serial.printf("系统重新初始化完成。新的初始总重量: %.1fg, 当前目标空重: %.1fg\n", 
                  system_initial_total_liquid_weight_g, target_empty_weight_g); // 打印初始化完成信息
}

// ================================
//         系统初始化函数
// ================================
void setup() {
    // 串口初始化
    Serial.begin(115200);

    // =========================
    //   硬件引脚模式统一设置
    // =========================
    // 按钮引脚初始化（上电默认高电平，内部上拉）
    pinMode(PIN_INIT_BUTTON, INPUT_PULLUP);
    pinMode(PIN_ABNORMALITY_RESET_BUTTON, INPUT_PULLUP);

    // 水滴传感器
    pinMode(PIN_WATER_SENSOR, INPUT_PULLDOWN);

    // LED指示灯
    pinMode(NEOPIXEL_PIN, OUTPUT);

    // 电机引脚
    pinMode(MOTOR_PIN1, OUTPUT);
    pinMode(MOTOR_PIN2, OUTPUT);

    // =========================
    //   外设初始化
    // =========================
    // NeoPixel LED初始化
    pixels.begin();
    pixels.setBrightness(NEOPIXEL_BRIGHTNESS);
    pixels.clear();
    pixels.show();

    // OLED显示屏初始化
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);    // I2C总线初始化（用于OLED显示）
    oled_display.begin();
    oled_display.enableUTF8Print(); // 使能UTF-8中文显示
    oled_display.setFont(u8g2_font_wqy12_t_gb2312); // 设置中文字体
    oled_display.clearBuffer();
    oled_display.drawStr(0, 10, "系统启动中...");
    oled_display.sendBuffer();

    // HX711称重传感器初始化
    scale_sensor.begin(PIN_HX711_DT, PIN_HX711_SCK);
    scale_sensor.set_scale(hx711_cal_factor); // 校准因子
    scale_sensor.set_gain(128); // 设置增益
    scale_sensor.set_offset(0); // 设置偏置为0

    // =========================
    //   中断初始化
    // =========================
    attachInterrupt(digitalPinToInterrupt(PIN_WATER_SENSOR), onWaterDropIsr, RISING); // 水滴中断

    // =========================
    //   网络与服务初始化
    // =========================
        connectToWiFi();
        if (wifi_connected_flag) {
        http_server.begin();
        Serial.printf("HTTP服务器已启动: http://%s/\n", WiFi.localIP().toString().c_str());
            ws_server.begin(); 
            ws_server.onEvent(onWebSocketEvent); 
        Serial.printf("WebSocket服务器已启动。\n");
            updateOledDisplay();
        } else {
        Serial.printf("警告: WiFi未连接, HTTP/WebSocket服务器未启动。\n");
            oled_display.clearBuffer();
            oled_display.drawStr(0, 10, "WiFi连接失败!");
            oled_display.sendBuffer();
        }

    // 白灯闪烁三次，表示硬件初始化完成
        for (int i = 0; i < 3; i++) {
            pixels.setPixelColor(0, NEO_COLOR_WHITE);
            pixels.show();
            delay(150);
            pixels.setPixelColor(0, NEO_COLOR_OFF);
            pixels.show();
            delay(150);
        }
    stopMotor(); // 确保电机在启动时是停止的
    // =========================
    //   备份滤波器原始参数
    // =========================
    // 初始化滤波器以获取原始参数
    weight_kf.init(0.0f, 0.0f, 0.0f);
    drip_kf.init(0.0f, -1.0f, 20, 1.0f);
    flow_fusion.init(0.0f, 0.0f);

    // 备份重量滤波器参数
    original_kf_weight_R_noise = weight_kf.getMeasurementNoise();
    Serial.printf("原始重量滤波器R: %.4f\n", original_kf_weight_R_noise);

    // 备份滴速滤波器参数
    original_drip_kf_R_drip_rate_noise = drip_kf.getDripRateMeasurementNoise();
    original_drip_kf_R_wpd_noise = drip_kf.getWpdMeasurementNoise();
    Serial.printf("原始滴速滤波器R: %.4f, WPD: %.4f\n", original_drip_kf_R_drip_rate_noise, original_drip_kf_R_wpd_noise);

    // 备份数据融合滤波器参数
    flow_fusion.getFlowMeasurementNoises(original_fusion_R_weight_flow, original_fusion_R_drip_flow);
    flow_fusion.getWeightMeasurementNoises(original_fusion_R_weight_weight, original_fusion_R_drip_weight);
    Serial.printf("原始融合R_flow(重量,滴速): %.4f, %.4f; R_weight(重量,滴速): %.4f, %.4f\n",
                  original_fusion_R_weight_flow, original_fusion_R_drip_flow, 
                  original_fusion_R_weight_weight, original_fusion_R_drip_weight);

    // 执行系统初始化
    performSystemReinitialization();

    // 初始化时间戳
    last_loop_run_ms = millis();
    last_drip_detected_time_ms = millis();
}

// ==========================
// 8. 主循环函数
// ==========================

void loop() {
    unsigned long current_time_ms = millis();

    // 按钮检测
    handleButtonInputs(current_time_ms);
    
    // 电机处理
    handleMotor();
    
    // 异常检测（仅在正常状态下）
    if (current_system_state == SystemState::NORMAL) {
        checkInfusionAbnormality(current_time_ms);
    }
    
    // 快速收敛模式处理
    if (current_system_state == SystemState::FAST_CONVERGENCE) {
        handleFastConvergenceMode(current_time_ms);
    }

    // 网络任务 - 无论系统状态如何都处理
    if (wifi_connected_flag) {
        ws_server.loop(); 
        handleHttpRequests(); 
    }

    // LED状态更新
    updateLedStatus();

    // 主处理逻辑（仅在正常状态或快速收敛状态下）
    if (current_system_state == SystemState::NORMAL || 
        current_system_state == SystemState::FAST_CONVERGENCE) {
        handleMainProcessing(current_time_ms);
    } else {
        // 在异常状态下，以1秒的频率更新显示和数据输出
        static unsigned long last_abnormal_output_ms = 0;
        if (current_time_ms - last_abnormal_output_ms >= MAIN_LOOP_INTERVAL_MS) {
            last_abnormal_output_ms = current_time_ms;
            updateDisplay();
            outputData(current_time_ms);
        }
        // 短暂延时，避免CPU占用过高
        delay(10); 
    }
}

// 按钮输入处理
void handleButtonInputs(unsigned long current_time_ms) {
    // 初始化按钮状态检测（上电默认高电平，按下为低电平）
    static bool last_init_button_state = HIGH;
    static unsigned long last_init_button_press_time = 0;
    bool current_init_button_state = digitalRead(PIN_INIT_BUTTON);

    // 初始化按钮按下（低电平触发）并去抖
    if (last_init_button_state == HIGH && current_init_button_state == LOW) {
        if (current_time_ms - last_init_button_press_time > INIT_BUTTON_DEBOUNCE_MS) {
            Serial.printf("初始化按钮被按下\n");
            last_init_button_press_time = current_time_ms;
            
            // 执行系统重新初始化
            performSystemReinitialization();
            // 直接进入快速收敛模式
            current_system_state = SystemState::FAST_CONVERGENCE;
            fast_convergence_mode = true;
            fast_convergence_start_ms = current_time_ms;
        }
    }
    last_init_button_state = current_init_button_state;

    // 异常复位按钮(GPIO0)状态检测，支持单击和长按
    static unsigned long abnormality_button_press_time = 0;
    static bool abnormality_button_is_pressed = false;
    static bool long_press_action_done = false;

    bool current_abnormality_reset_button_state = digitalRead(PIN_ABNORMALITY_RESET_BUTTON);

    if (current_abnormality_reset_button_state == LOW) { // 按钮被按下或按住
        if (!abnormality_button_is_pressed) { // 按钮刚被按下
            abnormality_button_is_pressed = true;
            abnormality_button_press_time = current_time_ms;
            long_press_action_done = false;
        } else { // 按钮被持续按住
            if (!long_press_action_done && (current_time_ms - abnormality_button_press_time > ABNORMALITY_RESET_BUTTON_LONG_PRESS_MS)) {
                toggleMotorState();
                long_press_action_done = true; // 标记长按动作已完成，避免重复触发
            }
        }
    } else { // 按钮未被按下 (高电平)
        if (abnormality_button_is_pressed) { // 按钮刚被释放
            // 检查是否为短按 (单击)
            if (!long_press_action_done) {
                Serial.printf("异常复位按钮被按下 (单击)\n");
                
                // 根据当前状态决定复位行为 (原单击逻辑)
                switch (current_system_state) {
                    case SystemState::INIT_ERROR:
                        // 如果是初始化错误，执行重新初始化
                        performSystemReinitialization();
                        break;
                    case SystemState::INFUSION_ERROR:
                    case SystemState::COMPLETED:  // 添加对完成状态的处理
                        // 如果是输液异常或完成状态，清除异常标志并恢复正常状态
                        infusion_abnormal = false;
                        current_system_state = SystemState::NORMAL;
                        auto_clamp = false;  // 关闭夹断
                        // 重置检测时间，开始新的检测周期
                        last_drip_detected_time_ms = current_time_ms;
                        Serial.printf("系统恢复正常状态, 开始新的检测周期\n");
                        break;
                    case SystemState::FAST_CONVERGENCE:
                        // 如果是快速收敛模式，恢复正常状态
                        current_system_state = SystemState::NORMAL;
                        fast_convergence_mode = false;
                        Serial.printf("退出快速收敛模式, 系统恢复正常状态\n");
                        break;
                    default:
                        // 其他状态不需要处理
                        break;
                }
            }
            abnormality_button_is_pressed = false; // 重置按钮状态
        }
    }
}

// 异常检测
void checkInfusionAbnormality(unsigned long current_time_ms) {
    // 只在正常状态下检查输液异常
    if (current_system_state != SystemState::NORMAL) {
        return;
    }

    // 检查是否到达检测间隔
    if (current_time_ms - last_abnormality_check_time_ms >= ABNORMALITY_CHECK_INTERVAL_MS) {
        // 检查是否超过无滴落超时时间
        if (current_time_ms - last_drip_detected_time_ms >= NO_DRIP_TIMEOUT_MS) {
            infusion_abnormal = true;
            current_system_state = SystemState::INFUSION_ERROR;
            auto_clamp = true;  // 开启夹断
            Serial.printf("警告：检测到输液异常（无滴落）\n");
            
            // 立即上传数据到服务器
            uploadDataToServer();
            
            if (wifi_connected_flag && ws_client_connected_flag) {
                ws_server.broadcastTXT("ALERT:INFUSION_ABNORMALITY_DETECTED");
            }
        }
        last_abnormality_check_time_ms = current_time_ms;
    }
}

// 快速收敛模式处理
void handleFastConvergenceMode(unsigned long current_time_ms) {
    static bool has_printed_warning = false;
    
    // 检查是否超过快速收敛时间（60秒）
    if (current_time_ms - fast_convergence_start_ms >= FAST_CONVERGENCE_DURATION_MS) {
        if (!has_printed_warning) {
            Serial.printf("快速收敛模式已超过60秒, 强制退出\n");
            has_printed_warning = true;
        }
        
        // 恢复原始滤波器参数
        weight_kf.setMeasurementNoise(original_kf_weight_R_noise);
            drip_kf.setDripRateMeasurementNoise(original_drip_kf_R_drip_rate_noise);
            drip_kf.setWpdMeasurementNoise(original_drip_kf_R_wpd_noise);
            flow_fusion.setFlowMeasurementNoises(original_fusion_R_weight_flow, original_fusion_R_drip_flow);
            flow_fusion.setWeightMeasurementNoises(original_fusion_R_weight_weight, original_fusion_R_drip_weight);

        // 退出快速收敛模式
            fast_convergence_mode = false;
        current_system_state = SystemState::NORMAL;
        has_printed_warning = false;  // 重置警告标志
        
        // 通知WebSocket客户端
        if (wifi_connected_flag && ws_client_connected_flag) {
            ws_server.broadcastTXT("ALERT:Fast convergence mode ended");
        }
        
        // 更新显示
        updateOledDisplay();
    }
}

// LED状态更新
void updateLedStatus() {
    // 根据系统状态更新LED颜色
    switch (current_system_state) {
        case SystemState::INITIALIZING:
            // 初始化中：黄色
            pixels.setPixelColor(0, NEO_COLOR_YELLOW);
            break;
        case SystemState::INIT_ERROR:
            // 系统异常（初始化异常）：红色常亮
            pixels.setPixelColor(0, NEO_COLOR_RED);
            break;
        case SystemState::NORMAL:
            // 正常运行：绿色
            pixels.setPixelColor(0, NEO_COLOR_GREEN);
            break;
        case SystemState::INFUSION_ERROR:
            // 输液异常：红色闪烁
            if ((millis() / 500) % 2 == 0) {
                pixels.setPixelColor(0, NEO_COLOR_RED);
    } else {
                pixels.setPixelColor(0, NEO_COLOR_OFF);
            }
            break;
        case SystemState::FAST_CONVERGENCE:
            // 快速收敛模式：蓝色
            pixels.setPixelColor(0, NEO_COLOR_BLUE);
            break;
    }
    pixels.show();
}

// 主处理逻辑
void handleMainProcessing(unsigned long current_time_ms) {
        float dt_main_loop_s = (current_time_ms - last_loop_run_ms) / 1000.0f;

        if (dt_main_loop_s >= (MAIN_LOOP_INTERVAL_MS / 1000.0f)) {
        last_loop_run_ms = current_time_ms;
        
        // 重量传感器处理
        handleWeightSensor();
        
        // 滴速处理
        handleDripRate(dt_main_loop_s);
        
        // 数据融合处理
        handleDataFusion(dt_main_loop_s);
        
        // 显示更新
        updateDisplay();
        
        // 数据输出
        outputData(current_time_ms);
    }
}

// 重量传感器处理
void handleWeightSensor() {
            float current_raw_weight_g_for_flow_calc = 0.0f; 
            float prev_filt_weight_g_this_main_loop = filt_weight_g; 
            
            if (scale_sensor.is_ready()) { 
                float gross_weight_reading_g = scale_sensor.get_units(5); 
                raw_weight_g = gross_weight_reading_g - TOTAL_TARE_G; // 得到纯液体净重
                current_raw_weight_g_for_flow_calc = raw_weight_g; 
        if ((fabsf(raw_weight_g) > 2000.0f && fabsf(prev_filt_weight_g_this_main_loop) < 1000.0f) || 
            isnan(raw_weight_g) || isinf(raw_weight_g)) {
            Serial.printf("警告: HX711读数 %.2f g 异常! 使用上一滤波值 %.2f g 代替。\n", 
                         raw_weight_g, prev_filt_weight_g_this_main_loop);
                    raw_weight_g = prev_filt_weight_g_this_main_loop; 
                }
            } else {
                raw_weight_g = prev_filt_weight_g_this_main_loop; 
                current_raw_weight_g_for_flow_calc = prev_raw_weight_g; 
                Serial.printf("警告: HX711 传感器未就绪!\n");
            }
            
    float dt_main_loop_s = (millis() - last_loop_run_ms) / 1000.0f;
            if (dt_main_loop_s > 1e-5f) {
                raw_flow_weight_gps = (prev_raw_weight_g - current_raw_weight_g_for_flow_calc) / dt_main_loop_s;
            } else {
                raw_flow_weight_gps = 0.0f;
            }
    
    if (raw_flow_weight_gps < 0) {
                raw_flow_weight_gps = 0.0f; 
            }

    prev_raw_weight_g = current_raw_weight_g_for_flow_calc; 
            filt_weight_g = weight_kf.update(raw_weight_g, dt_main_loop_s);
            flow_weight_gps = -weight_kf.getVelocity(); 
            if (flow_weight_gps < 0) flow_weight_gps = 0.0f; 

            // 确保流速相关变量不为负
            if (raw_flow_weight_gps < 0) raw_flow_weight_gps = 0.0f;
            if (flow_drip_gps < 0) flow_drip_gps = 0.0f;
            if (raw_flow_drip_gps < 0) raw_flow_drip_gps = 0.0f;
            if (fused_flow_rate_gps < 0) fused_flow_rate_gps = 0.0f;

            // 确保重量相关变量不为负
            if (fused_remaining_weight_g < 0) fused_remaining_weight_g = 0.0f;
            if (remaining_weight_drip_calc_g < 0) remaining_weight_drip_calc_g = 0.0f;

            // 确保显示相关变量不为负
            if (g_oled_flow_rate_mlh < 0) g_oled_flow_rate_mlh = 0.0f;
            if (g_infusion_progress < 0.0f) g_infusion_progress = 0.0f;
            if (g_infusion_progress > 1.0f) g_infusion_progress = 1.0f;
}

// 滴速处理
void handleDripRate(float dt_main_loop_s) {
    accumulated_weight_change_for_wpd_g += (prev_filt_weight_g - filt_weight_g);

            unsigned long timestamps[MAX_TIMESTAMP_QUEUE_SIZE];
            int timestamp_count = 0;
            while (getNextDripTimestamp(timestamps[timestamp_count]) && timestamp_count < MAX_TIMESTAMP_QUEUE_SIZE) {
                timestamp_count++;
            }

            if (timestamp_count <= 1) {
                if (timestamp_count == 1) {
                    drip_timestamps_ms[0] = timestamps[0];
                    timestamp_queue_head = 0;
                    timestamp_queue_tail = 1;
                    timestamp_queue_full = false;
                }
            } else {
                int valid_intervals = 0;
                float total_interval_ms = 0.0f;
                for (int i = 1; i < timestamp_count; i++) {
                    float interval_ms = timestamps[i] - timestamps[i-1];
                    if (interval_ms > 50.0f && interval_ms < 5000.0f) {
                        total_interval_ms += interval_ms;
                        valid_intervals++;
                    }
                }

                float measured_drip_rate = 0.0f;
                if (valid_intervals > 0) {
                    float avg_interval_ms = total_interval_ms / valid_intervals;
                    measured_drip_rate = 1000.0f / avg_interval_ms;
                }

        drip_kf.update(measured_drip_rate, dt_main_loop_s, accumulated_weight_change_for_wpd_g);
                if(system_initial_weight_set) {
                    drip_kf.updateTotalDropsForVolumeCalc(valid_intervals);
            // 在每次滴落时进行WPD校准
            if (current_system_state == SystemState::NORMAL) {
                drip_kf.calibrateWpdByTotal(filt_weight_g);
                }
        }

                drops_this_drip_cycle = valid_intervals;
                accumulated_weight_change_for_wpd_g = 0.0f;
                filt_drip_rate_dps = drip_kf.getFilteredDripRate();
                // 确保滤波后滴速不为负值
                if (filt_drip_rate_dps < 0) filt_drip_rate_dps = 0.0f;
                flow_drip_gps = drip_kf.getFlowRateGramsPerSecond();
                raw_drip_rate_dps = measured_drip_rate;
                raw_flow_drip_gps = raw_drip_rate_dps * drip_kf.getCalibratedWeightPerDrop();
                if (raw_flow_drip_gps < 0) raw_flow_drip_gps = 0.0f;

                drip_timestamps_ms[0] = timestamps[timestamp_count-1];
                timestamp_queue_head = 0;
                timestamp_queue_tail = 1;
                timestamp_queue_full = false;
    }
            }
            
// 数据融合处理
void handleDataFusion(float dt_main_loop_s) {
    // 计算输液进度
            g_infusion_progress = 0.0f; 
            if (system_initial_weight_set) {
                float total_infusable_weight = system_initial_total_liquid_weight_g - target_empty_weight_g;
                if (total_infusable_weight > 1e-3f) {
                    float infused_amount = system_initial_total_liquid_weight_g - fused_remaining_weight_g;
                    if (infused_amount < 0) infused_amount = 0;
                    if (infused_amount > total_infusable_weight) infused_amount = total_infusable_weight;
                    g_infusion_progress = infused_amount / total_infusable_weight;
                }
            }
            if (g_infusion_progress < 0.0f) g_infusion_progress = 0.0f;
            if (g_infusion_progress > 1.0f) g_infusion_progress = 1.0f;

    // WPD长时间校准处理
    handleWpdLongCalibration();

    // 计算剩余重量
    if(system_initial_weight_set) {
        remaining_weight_drip_calc_g = drip_kf.getRemainingWeightByDropsG();
    } else {
        remaining_weight_drip_calc_g = filt_weight_g; 
    }

    // 检查是否输液完成
    if (system_initial_weight_set && 
        current_system_state == SystemState::NORMAL && 
        fused_remaining_weight_g <= target_empty_weight_g + 1.0f) {
        current_system_state = SystemState::COMPLETED;
        auto_clamp = true;  // 输液完成时开启夹断
        Serial.printf("输液已完成, 系统进入完成状态\n");
        if (wifi_connected_flag && ws_client_connected_flag) {
            ws_server.broadcastTXT("ALERT:INFUSION_COMPLETED");
        }
    }

    // 数据融合更新
    flow_fusion.update(flow_weight_gps, flow_drip_gps, filt_weight_g, remaining_weight_drip_calc_g, dt_main_loop_s);
    fused_flow_rate_gps = flow_fusion.getFusedFlowRateGps();
    if (fused_flow_rate_gps < 0) fused_flow_rate_gps = 0.0f; 
    fused_remaining_weight_g = flow_fusion.getFusedRemainingWeightG();
    if (fused_remaining_weight_g < 0) fused_remaining_weight_g = 0.0f;

    // 计算剩余时间
    calculateRemainingTime();
}

// WPD长时间校准处理
void handleWpdLongCalibration() {
            if (wpd_long_cal_active) {
        unsigned long elapsed_cal_time_ms = millis() - wpd_long_cal_start_ms;
                bool duration_met = elapsed_cal_time_ms >= WPD_LONG_CAL_DURATION_MS;
                bool drops_met = wpd_long_cal_accum_drops >= WPD_LONG_CAL_MIN_DROPS;

                if (duration_met && drops_met) {
                    Serial.printf("WPD长时校准自动完成。时长: %.1fs, 总滴数: %d, 最终WPD: %.4f g/drip\n",
                                  elapsed_cal_time_ms / 1000.0f,
                                  wpd_long_cal_accum_drops,
                                  drip_kf.getCalibratedWeightPerDrop());
                    drip_kf.stopWpdCalibration(); 
                    wpd_long_cal_active = false; 
                    
                    char cal_done_msg[120];
            snprintf(cal_done_msg, sizeof(cal_done_msg), 
                     "EVENT:WPD_CALIBRATION_COMPLETED,WPD:%.4f,Drops:%d,DurationSec:%.1f",
                     drip_kf.getCalibratedWeightPerDrop(), wpd_long_cal_accum_drops, 
                     elapsed_cal_time_ms / 1000.0f);
                    if(ws_client_connected_flag) ws_server.broadcastTXT(cal_done_msg); 
                } else if (duration_met && !drops_met) {
            Serial.printf("WPD长时校准时间已到 (%.1fs), 但累计滴数 (%d) 未达目标 (%d)。\n",
                          elapsed_cal_time_ms / 1000.0f, wpd_long_cal_accum_drops, 
                          WPD_LONG_CAL_MIN_DROPS);
        }
    }
}

// 计算剩余时间
void calculateRemainingTime() {
            float remaining_weight_to_infuse_fused = fused_remaining_weight_g - target_empty_weight_g;
            if (remaining_weight_to_infuse_fused < 0) remaining_weight_to_infuse_fused = 0.0f; 

            float base_remaining_time_s = 0.0f;
            if (fused_flow_rate_gps > 1e-5f) {
                base_remaining_time_s = remaining_weight_to_infuse_fused / fused_flow_rate_gps;
            } else {
                base_remaining_time_s = (remaining_weight_to_infuse_fused <= 0.01f ? 0.0f : 999999.0f);
            }
    remaining_time_s = base_remaining_time_s;

            if (remaining_time_s < 0) remaining_time_s = 0.0f;
            if (remaining_time_s > 999999.0f) remaining_time_s = 999999.0f; 

            remaining_time_raw_weight_s = calculate_specific_remaining_time(raw_weight_g, target_empty_weight_g, raw_flow_weight_gps);
            remaining_time_filt_weight_s = calculate_specific_remaining_time(filt_weight_g, target_empty_weight_g, flow_weight_gps);
            remaining_time_raw_drip_s = calculate_specific_remaining_time(remaining_weight_drip_calc_g, target_empty_weight_g, raw_flow_drip_gps);
            remaining_time_filt_drip_s = calculate_specific_remaining_time(remaining_weight_drip_calc_g, target_empty_weight_g, flow_drip_gps);
}

// 显示更新
void updateDisplay() {
            if (system_initial_weight_set) {
                g_oled_infused_progress_percent = g_infusion_progress * 100.0f;
            } else {
                g_oled_infused_progress_percent = 0.0f; 
            }

            g_oled_flow_rate_mlh = 0.0f;
            float current_density = drip_kf.getCurrentLiquidDensity(); 
            if (current_density > 1e-6f) { 
                 g_oled_flow_rate_mlh = (fused_flow_rate_gps / current_density) * 3600.0f;
            }
            if (g_oled_flow_rate_mlh < 0) g_oled_flow_rate_mlh = 0;

            g_oled_remaining_time_min = 0;
            if (remaining_time_s > 0 && remaining_time_s < (3600*99)) { 
                g_oled_remaining_time_min = (long)(remaining_time_s / 60.0f);
    } else if (fused_remaining_weight_g <= target_empty_weight_g + 1.0f && fabsf(fused_flow_rate_gps) < 0.001f) {
                g_oled_remaining_time_min = 0; 
            } else {
                g_oled_remaining_time_min = -1; 
            }
            updateOledDisplay(); 
}
            
// 数据输出
void outputData(unsigned long current_time_ms) {
    // 串口输出
            char serial_buf_debug[450]; 
            snprintf(serial_buf_debug, sizeof(serial_buf_debug),
                     "%.2f,%.2f,%.2f,%.4f,%.4f,%u,%.2f,%.2f,%.4f,%.4f,%.4f,%d,%.2f,%lu,%.5f,%.4f,%.2f,%.0f,%.0f,%.0f,%.0f",
             current_time_ms / 1000.0f, raw_weight_g, filt_weight_g, raw_flow_weight_gps, flow_weight_gps,
             drops_this_drip_cycle, raw_drip_rate_dps, filt_drip_rate_dps, raw_flow_drip_gps, flow_drip_gps,
             remaining_weight_drip_calc_g, drip_kf.isWpdCalibrating() ? 1 : 0, drip_kf.known_initial_total_weight_g,
             drip_kf.total_drops_for_volume_calc, drip_kf.getCalibratedWeightPerDrop(), fused_flow_rate_gps,
             fused_remaining_weight_g, remaining_time_s, remaining_time_raw_weight_s,
             remaining_time_filt_weight_s, remaining_time_raw_drip_s);
            Serial.println(serial_buf_debug);

    // WebSocket输出
            if (wifi_connected_flag && ws_client_connected_flag) {
                float ws_progress_percent_to_send = -1.0f; 
                if (system_initial_weight_set) {
                    ws_progress_percent_to_send = g_infusion_progress * 100.0f; 
                }
                char serial_buf_ws[450];
                snprintf(serial_buf_ws, sizeof(serial_buf_ws), 
                 "%lu,%.2f,%.2f,%.4f,%.4f,%u,%.2f,%.2f,%.4f,%.4f,%.4f,%d,%d,%.2f,%.4f,%.2f,%.0f,%.0f,%.0f,%.0f,%.0f,%lu,%.2f,%.5f,%.1f,%d",
                 current_time_ms, raw_weight_g, filt_weight_g, raw_flow_weight_gps, flow_weight_gps,
                 drops_this_drip_cycle, raw_drip_rate_dps, filt_drip_rate_dps, raw_flow_drip_gps, flow_drip_gps,
                 drip_kf.getCalibratedWeightPerDrop(), drip_kf.isWpdCalibrating() ? 1 : 0, wpd_long_cal_active ? 1 : 0,
                 remaining_weight_drip_calc_g, fused_flow_rate_gps, fused_remaining_weight_g,
                 remaining_time_raw_weight_s, remaining_time_filt_weight_s, remaining_time_raw_drip_s,
                 remaining_time_filt_drip_s, remaining_time_s, drip_kf.total_drops_for_volume_calc,
                 drip_kf.known_initial_total_weight_g, drip_kf.getCalibratedWeightPerDrop(), ws_progress_percent_to_send,
                 static_cast<int>(current_system_state));
                ws_server.broadcastTXT(serial_buf_ws); 
            }

    // 每10秒上传一次数据
    static unsigned long last_upload_time = 0;
    if (current_time_ms - last_upload_time >= 10000) {  // 10000毫秒=10秒
        uploadDataToServer();
        last_upload_time = current_time_ms;
    }
}

// 获取系统状态的中文描述
String getSystemStateText(const SystemState& state) {
    switch (state) {
        case SystemState::NORMAL:
            return "输液正常";
        case SystemState::FAST_CONVERGENCE:
            return "快速收敛";
        case SystemState::INFUSION_ERROR:
            return "输液异常";
        case SystemState::INITIALIZING:
            return "初始化中";
        case SystemState::INIT_ERROR:
            return "初始化异常";
        case SystemState::COMPLETED:
            return "已完成";
        default:
            return "未知状态";
    }
}

// 数据上传相关函数
void uploadDataToServer() {
    if (!wifi_connected_flag) {
        Serial.printf("WiFi未连接, 无法上传数据\n");
        return;
    }

    HTTPClient http;
    String url = String(API_BASE_URL) + "/api/patients";
    
    // 创建JSON文档
    StaticJsonDocument<512> doc;
    
    // 设置基本数据
    doc["deviceId"] = DEVICE_ID;
    
    // 设置体积相关数据（转换为整数）
    doc["totalVolume"] = (int)total_volume_ml;
    doc["remainingVolume"] = (int)fused_remaining_weight_g;  // 1g = 1ml，转换为整数
    
    // 设置滴速（转换为整数，滴/分钟）
    doc["currentRate"] = (int)(filt_drip_rate_dps * 60.0f);
    
    // 设置剩余时间（分钟，转换为整数）
    doc["estimatedTime"] = (int)ceil(remaining_time_s / 60.0f);
    
    // 设置系统状态
    String systemState;
    switch (current_system_state) {
        case SystemState::INITIALIZING:
            systemState = "INITIALIZING";
            break;
        case SystemState::INIT_ERROR:
            systemState = "INIT_ERROR";
            break;
        case SystemState::NORMAL:
            systemState = "NORMAL";
            break;
        case SystemState::INFUSION_ERROR:
            systemState = "INFUSION_ERROR";
            break;
        case SystemState::FAST_CONVERGENCE:
            systemState = "FAST_CONVERGENCE";
            break;
        case SystemState::COMPLETED:
            systemState = "COMPLETED";
            break;
        default:
            systemState = "UNKNOWN";
    }
    doc["systemState"] = systemState;
    
    // 设置自动夹断状态（转换为整数：0或1）
    doc["autoClamp"] = auto_clamp ? 1 : 0;

    // 序列化JSON
    String jsonString;
    serializeJson(doc, jsonString);

    // 打印要发送的JSON数据
    Serial.printf("准备发送的数据：\n");
    Serial.println(jsonString);

    // 发送HTTP请求
    http.begin(url);
    http.addHeader("Content-Type", "application/json");
    
    int httpCode = http.POST(jsonString);  // 改用POST方法
    
    if (httpCode > 0) {
        if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_CREATED) {
            String response = http.getString();
            Serial.printf("数据上传成功: %s\n", response.c_str());
        } else {
            String response = http.getString();
            Serial.printf("HTTP请求失败, 错误码: %d\n", httpCode);
            Serial.printf("服务器响应: %s\n", response.c_str());
        }
    } else {
        Serial.printf("HTTP请求失败: %s\n", http.errorToString(httpCode).c_str());
    }
    
    http.end();
} 

// ==========================
// 9. 电机控制函数
// ==========================
void startMotorForward() {
    digitalWrite(MOTOR_PIN1, HIGH);
    digitalWrite(MOTOR_PIN2, LOW);
}

void startMotorReverse() {
    digitalWrite(MOTOR_PIN1, LOW);
    digitalWrite(MOTOR_PIN2, HIGH);
}

void stopMotor() {
    digitalWrite(MOTOR_PIN1, LOW);
    digitalWrite(MOTOR_PIN2, LOW);
}

void handleMotor() {
    if (motor_is_running) {
        if (millis() - motor_start_time_ms >= MOTOR_RUN_DURATION_MS) {
            stopMotor();
            motor_is_running = false;
            Serial.printf("电机停止。\n");
        }
    }
}

void toggleMotorState() {
    if (motor_is_running) {
        Serial.printf("电机正在运行中，请等待转动完成。\n");
        return;
    }

    if (next_motor_direction == MotorDirection::FORWARD) {
        Serial.printf("电机启动：正转。\n");
        startMotorForward();
        next_motor_direction = MotorDirection::REVERSE;
    } else {
        Serial.printf("电机启动：反转。\n");
        startMotorReverse();
        next_motor_direction = MotorDirection::FORWARD;
    }
    motor_is_running = true;
    motor_start_time_ms = millis();
}