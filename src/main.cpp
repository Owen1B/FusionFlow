// 此文件是ESP32智能输液监控系统的主程序文件。
// 包含所有必要的库文件，定义全局变量、硬件配置、
// WiFi与WebSocket设置、传感器数据处理、卡尔曼滤波、数据融合、
// OLED显示、以及通过WebSocket的远程监控和控制逻辑。

// --- 标准库和第三方库引用 ---
#include <WiFi.h>             // 用于WiFi连接
#include <WiFiClient.h>       // 用于创建WiFi客户端 (HTTP Server会用到)
#include <DNSServer.h>        // (WiFiServer有时可能间接需要，但此处不直接使用DNS功能)
#include <WebSocketsServer.h> // 用于创建WebSocket服务器

#include <Wire.h>             // 用于I2C通讯 (OLED显示屏)
#include <Adafruit_GFX.h>     // GFX图形库 (U8g2可能间接依赖或共享某些概念)
#include <U8g2lib.h>          // OLED显示屏驱动库

#include "HX711.h"             // HX711称重传感器库

// --- 自定义模块引用 ---
#include "WeightKalmanFilter.h" // 重量卡尔曼滤波器模块
#include "DripKalmanFilter.h"   // 滴速卡尔曼滤波器模块
#include "DataFusion.h"         // 数据融合模块

// --- HTML 页面内容 ---
// 注意：这是一个非常基础的HTML页面，直接嵌入到C++字符串中。
// 为了节省ESP32内存，它非常精简，没有外部CSS或复杂的JS库。
const char HTML_WEBPAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="zh-CN">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>智能输液监控</title>
    <style>
        body { font-family: Arial, sans-serif; margin: 10px; background-color: #f4f4f4; color: #333; }
        .container { background-color: #fff; padding: 15px; border-radius: 8px; box-shadow: 0 0 10px rgba(0,0,0,0.1); }
        h1 { color: #0056b3; text-align: center; }
        table { width: 100%; border-collapse: collapse; margin-top: 15px; }
        th, td { text-align: left; padding: 8px; border-bottom: 1px solid #ddd; }
        th { background-color: #007bff; color: white; }
        .label { font-weight: bold; color: #555; }
        .value { color: #000; }
        .button-container { margin-top: 20px; text-align: center; }
        button { background-color: #007bff; color: white; padding: 10px 15px; border: none; border-radius: 5px; cursor: pointer; font-size: 1em; margin: 5px; }
        button:hover { background-color: #0056b3; }
        #wsState { margin-top:10px; font-style:italic; text-align: center; }
    </style>
</head>
<body>
    <div class="container">
        <h1>智能输液监控系统</h1>
        
        <div id="wsState">正在连接WebSocket...</div>

        <table>
            <tr><td class="label">系统时间戳 (ms):</td><td class="value" id="timestamp_ms">-</td></tr>
            <tr><th colspan="2">重量传感器</th></tr>
            <tr><td class="label">原始重量 (g):</td><td class="value" id="raw_weight_g">-</td></tr>
            <tr><td class="label">滤波后重量 (g):</td><td class="value" id="filt_weight_g">-</td></tr>
            <tr><td class="label">重量流速 (g/s):</td><td class="value" id="flow_weight_gps">-</td></tr>
            <tr><th colspan="2">滴速传感器</th></tr>
            <tr><td class="label">本周期滴数:</td><td class="value" id="drops_period">-</td></tr>
            <tr><td class="label">原始滴速 (dps):</td><td class="value" id="raw_drip_rate_dps">-</td></tr>
            <tr><td class="label">滤波后滴速 (dps):</td><td class="value" id="filt_drip_rate_dps">-</td></tr>
            <tr><td class="label">每滴重量 (g/drip):</td><td class="value" id="wpd_g">-</td></tr>
            <tr><td class="label">WPD校准中:</td><td class="value" id="wpd_calibrating">-</td></tr>
            <tr><td class="label">滴速流速 (g/s):</td><td class="value" id="flow_drip_gps">-</td></tr>
            <tr><th colspan="2">融合数据 & 预测</th></tr>
            <tr><td class="label">融合后流速 (g/s):</td><td class="value" id="fused_flow_gps">-</td></tr>
            <tr><td class="label">预计剩余时间 (s):</td><td class="value" id="remaining_time_s">-</td></tr>
        </table>

        <div class="button-container">
            <button onclick="sendWSCmd('CALIBRATE_WPD_START')">开始WPD校准</button>
            <button onclick="sendWSCmd('CALIBRATE_WPD_STOP')">停止WPD校准</button>
        </div>
    </div>

    <script>
        let ws;
        let lastData = {};
        const displayInterval = 1000; // 页面数据更新间隔 (ms)，例如1秒
        let displayTimer;

        function connectWebSocket() {
            const wsUrl = "ws://" + window.location.hostname + ":81/";
            ws = new WebSocket(wsUrl);
            document.getElementById("wsState").textContent = "正在连接到: " + wsUrl;

            ws.onopen = function(event) {
                document.getElementById("wsState").textContent = "WebSocket 已连接";
                console.log("WebSocket connection opened");
                // 清除旧的定时器（如果存在）并启动新的
                if(displayTimer) clearInterval(displayTimer);
                displayTimer = setInterval(updateDisplay, displayInterval);
            };

            ws.onmessage = function(event) {
                // console.log("Raw data: " + event.data);
                const data = event.data.split(',');
                // 期望的顺序 (来自ESP32的CSV表头和实际发送顺序):
                // 0:timestamp_ms, 1:raw_w, 2:filt_w, 3:flow_w_gps,
                // 4:drops_p, 5:raw_dps, 6:filt_dps, 7:flow_d_gps,
                // 8:wpd, 9:wpd_cal, 10:fused_gps, 11:rem_s
                if (data.length >= 12) { // 确保数据完整
                    if (data[0].toLowerCase().includes("timestamp_ms")) return; // 跳过表头行
                    
                    lastData.timestamp_ms = data[0];
                    lastData.raw_weight_g = parseFloat(data[1]).toFixed(2);
                    lastData.filt_weight_g = parseFloat(data[2]).toFixed(2);
                    lastData.flow_weight_gps = parseFloat(data[3]).toFixed(4);
                    lastData.drops_period = data[4];
                    lastData.raw_drip_rate_dps = parseFloat(data[5]).toFixed(2);
                    lastData.filt_drip_rate_dps = parseFloat(data[6]).toFixed(2);
                    lastData.flow_drip_gps = parseFloat(data[7]).toFixed(4);
                    lastData.wpd_g = parseFloat(data[8]).toFixed(4);
                    lastData.wpd_calibrating = (data[9] === "1") ? "是" : "否";
                    lastData.fused_flow_gps = parseFloat(data[10]).toFixed(4);
                    lastData.remaining_time_s = parseFloat(data[11]).toFixed(0);
                } else {
                    // console.log("Received incomplete data: " + event.data);
                }
            };

            function updateDisplay() {
                 if (!lastData.timestamp_ms) return; // 没有数据则不更新

                 document.getElementById("timestamp_ms").textContent = lastData.timestamp_ms;
                 document.getElementById("raw_weight_g").textContent = lastData.raw_weight_g;
                 document.getElementById("filt_weight_g").textContent = lastData.filt_weight_g;
                 document.getElementById("flow_weight_gps").textContent = lastData.flow_weight_gps;
                 document.getElementById("drops_period").textContent = lastData.drops_period;
                 document.getElementById("raw_drip_rate_dps").textContent = lastData.raw_drip_rate_dps;
                 document.getElementById("filt_drip_rate_dps").textContent = lastData.filt_drip_rate_dps;
                 document.getElementById("wpd_g").textContent = lastData.wpd_g;
                 document.getElementById("wpd_calibrating").textContent = lastData.wpd_calibrating;
                 document.getElementById("flow_drip_gps").textContent = lastData.flow_drip_gps;
                 document.getElementById("fused_flow_gps").textContent = lastData.fused_flow_gps;
                 document.getElementById("remaining_time_s").textContent = lastData.remaining_time_s + " 秒";
            }

            ws.onerror = function(error) {
                document.getElementById("wsState").textContent = "WebSocket 错误!";
                console.error("WebSocket Error: ", error);
            };

            ws.onclose = function(event) {
                document.getElementById("wsState").textContent = "WebSocket 已断开. 正在尝试重连...";
                console.log("WebSocket connection closed. Code: " + event.code + ", Reason: " + event.reason);
                if(displayTimer) clearInterval(displayTimer);
                // 尝试5秒后重连
                setTimeout(connectWebSocket, 5000);
            };
        }

        function sendWSCmd(cmd) {
            if (ws && ws.readyState === WebSocket.OPEN) {
                ws.send(cmd);
                console.log("Sent WS command: " + cmd);
            } else {
                console.log("WebSocket not connected. Cannot send command: " + cmd);
                document.getElementById("wsState").textContent = "WebSocket未连接，无法发送命令!";
            }
        }

        // 页面加载完成后开始连接WebSocket
        window.onload = connectWebSocket;
    </script>
</body>
</html>
)rawliteral";

// --- WiFi 配置 ---
const char* WIFI_SSID = "1503"; // 您的WiFi SSID
const char* WIFI_PASS = "18310007230"; // 您的WiFi密码
bool wifi_connected_flag = false;

// --- WebSocket 服务器配置 ---
WebSocketsServer ws_server = WebSocketsServer(81); // WebSocket 服务器监听81端口
bool ws_client_connected_flag = false; // 标记是否有WebSocket客户端连接

// --- HTTP 服务器配置 ---
WiFiServer http_server(80); // HTTP服务器监听80端口

// --- 硬件引脚定义 ---
const int PIN_WATER_SENSOR = 11; // 水滴传感器引脚
const int PIN_LED_STATUS = 2;   // LED状态指示灯引脚
const int PIN_I2C_SDA = 41;     // I2C SDA引脚 (OLED)
const int PIN_I2C_SCL = 42;     // I2C SCL引脚 (OLED)
const int PIN_HX711_DT = 17;    // HX711 数据引脚
const int PIN_HX711_SCK = 18;   // HX711 时钟引脚

// --- HX711 配置 ---
// !! 重要: 此校准因子必须根据您的HX711模块和称重传感器进行实际校准 !!
// !! 它直接影响重量读数的准确性。常见的校准方法是使用已知重量的物体。!!
float hx711_cal_factor = 1670.0f; // 示例校准因子，请务必重新校准!
HX711 scale_sensor; // HX711传感器对象

// --- OLED 显示屏配置 ---
#define OLED_WIDTH 128 // OLED宽度（像素）
#define OLED_HEIGHT 32 // OLED高度（像素）
// U8g2库的OLED驱动对象 (SSD1306, 128x32, I2C通讯)
U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C oled_display(U8G2_R0, U8X8_PIN_NONE);

// --- 卡尔曼滤波器和数据融合对象实例化 ---
// 参数调整指南:
// WeightKalmanFilter(sigma_a, R_sensor_noise)
//   sigma_a (过程噪声标准差 for 速度): 0.001-0.05. 越小则速度估计越平滑，但对真实速度变化响应慢。
//   R_sensor_noise (测量噪声方差 for 重量): 若原始重量读数标准差为s, 则R约为s*s. 传感器噪声大或尖峰多则调大R。
WeightKalmanFilter weight_kf(0.02f, 0.5f);

// DripKalmanFilter(drip_rate_sigma_a, drip_rate_R, wpd_Q, wpd_R)
//   drip_rate_sigma_a (滴速变化加速度标准差): 0.01-0.2.
//   drip_rate_R (滴速测量方差): (drops_period / dt_loop) 计算值的方差。
//   wpd_Q (WPD过程噪声方差): 校准时，每滴真实重量本身变化的可能性（应设小，如0.00001f）。
//   wpd_R (WPD测量方差): 校准时 (weight_change / drops_period) 计算值的方差。
DripKalmanFilter drip_kf(0.05f, 0.2f, 0.00001f, 0.001f);

// DataFusion(Q_true_flow_process, R_weight_flow, R_drip_flow)
//   Q_true_flow_process (真实流速本身变化方差): 例如药袋压力变化导致的流速固有波动。
//   R_weight_flow (重量传感器流速测量方差): weight_kf.getVelocity() 的不确定性。
//   R_drip_flow (滴速传感器流速测量方差): drip_kf.getFlowRateGramsPerSecond() 的不确定性。
DataFusion flow_fusion(0.0001f, 0.01f, 0.01f);

// --- 全局状态变量 ---
// ISR (中断服务程序) 中使用的变量必须声明为 volatile
volatile unsigned long isr_drop_count_period = 0; // ISR 在一个主循环周期内累计的滴数
volatile unsigned long isr_last_drop_time_ms = 0; // ISR 中记录的上一滴发生的时间戳 (ms)

// 主循环定时
const unsigned long MAIN_LOOP_INTERVAL_MS = 500; // 主循环更新周期 (0.5 秒)
unsigned long last_loop_run_ms = 0; // 上次主循环运行的时间戳 (ms)

// 数据变量 (g: 克, s: 秒, dps: 滴/秒, gps: 克/秒, ms: 毫秒, mlh: 毫升/小时)
float raw_weight_g = 0.0f;            // HX711原始重量读数 (g)
float filt_weight_g = 0.0f;           // 卡尔曼滤波后的重量 (g)
float prev_filt_weight_g = 0.0f;      // 上一周期滤波后的重量 (g)
float flow_weight_gps = 0.0f;         // 从重量传感器估算的流速 (g/s, 正值表示消耗)

unsigned int drops_in_loop_period = 0;  // 当前主循环周期内检测到的总滴数
float raw_drip_rate_dps = 0.0f;     // 由原始滴数计算的滴速 (dps)
float filt_drip_rate_dps = 0.0f;    // 滴速卡尔曼滤波后的滴速 (dps)
float flow_drip_gps = 0.0f;           // 从滴速传感器估算的流速 (g/s)

float fused_flow_rate_gps = 0.0f;     // 融合后的最终流速 (g/s)
float remaining_time_s = 0.0f;        // 预计剩余输液时间 (秒)
float target_empty_weight_g = 5.0f; // 目标空袋重量 (g), 用于判断输液接近完成
                                      // 可以设为一个略大于0的值，以考虑空袋自身重量和少量残留液体

// --- WPD 长时间校准相关变量 ---
bool wpd_long_cal_active = false;          // 标记是否正在进行WPD的长时间校准
unsigned long wpd_long_cal_start_ms = 0;   // WPD长时间校准的开始时间戳 (ms)
const unsigned long WPD_LONG_CAL_DURATION_MS = 60000; // WPD长时间校准的目标时长 (例如: 60秒)
int wpd_long_cal_accum_drops = 0;       // WPD长时间校准期间累计的总滴数
const int WPD_LONG_CAL_MIN_DROPS = 30;   // WPD长时间校准期间要求的最小累计滴数 (确保数据量)
                                      

// --- 中断服务函数 (ISR) ---
// 当水滴传感器检测到滴落 (引脚电平上升) 时触发此函数
// ISR 中的代码应尽可能简短快速，避免复杂计算和阻塞操作
void IRAM_ATTR onWaterDropIsr() {
    unsigned long now_ms = millis();
    // 简单的消抖逻辑：确保两次中断之间有最小间隔，防止传感器抖动或噪声误触发
    // 50ms 是一个经验值，可能需要根据实际传感器和环境进行调整
    if (now_ms - isr_last_drop_time_ms > 50) { 
        isr_drop_count_period++; // 周期内滴数累加
        isr_last_drop_time_ms = now_ms; // 更新上一滴的时间戳
    }
}

// --- WebSocket 事件处理回调函数 ---
void onWebSocketEvent(uint8_t client_num, WStype_t type, uint8_t * payload, size_t length) {
    switch(type) {
        case WStype_DISCONNECTED: 
            if (client_num == 0) ws_client_connected_flag = false; 
            Serial.printf("[%u] WebSocket客户端断开连接!\n", client_num);
            break;
        case WStype_CONNECTED: { 
            IPAddress client_ip = ws_server.remoteIP(client_num);
            Serial.printf("[%u] WebSocket客户端已连接，IP: %s\n", client_num, client_ip.toString().c_str());
            if (client_num == 0) ws_client_connected_flag = true;
            ws_server.sendTXT(client_num, HTML_WEBPAGE);
            break;
        }
        case WStype_TEXT: 
            Serial.printf("[%u] 收到文本: %s\n", client_num, payload);
            if (strcmp((char*)payload, "CALIBRATE_WPD_START") == 0) {
                if (wpd_long_cal_active) {
                    Serial.println("WPD校准已在进行中。");
                    ws_server.sendTXT(client_num, "EVENT:WPD_CALIBRATION_ALREADY_RUNNING");
                } else {
                    Serial.println("收到WPD校准开始命令，启动长时校准...");
                    drip_kf.startWpdCalibration(); // 通知DripKalmanFilter开始接收WPD更新
                    wpd_long_cal_active = true;
                    wpd_long_cal_start_ms = millis();
                    wpd_long_cal_accum_drops = 0;
                    ws_server.sendTXT(client_num, "CMD_ACK:WPD_LONG_CALIBRATION_STARTED");
                }
            } else if (strcmp((char*)payload, "CALIBRATE_WPD_STOP") == 0) {
                if (wpd_long_cal_active) {
                    Serial.println("收到WPD校准手动停止命令...");
                    drip_kf.stopWpdCalibration(); // DripKalmanFilter停止WPD更新模式
                    wpd_long_cal_active = false;
                    float current_wpd = drip_kf.getCalibratedWeightPerDrop();
                    char stop_msg[100];
                    snprintf(stop_msg, sizeof(stop_msg), "CMD_ACK:WPD_CALIBRATION_STOPPED_MANUALLY,CurrentWPD:%.4f", current_wpd);
                    ws_server.sendTXT(client_num, stop_msg);
                    Serial.printf("WPD校准手动停止。当前WPD: %.4f g/drip\n", current_wpd);
                } else {
                    Serial.println("WPD校准未在进行中，无需停止。");
                    ws_server.sendTXT(client_num, "EVENT:WPD_CALIBRATION_NOT_RUNNING");
                }
            } else {
                ws_server.sendTXT(client_num, "CMD_UNKNOWN");
            }
            break;
        case WStype_BIN: 
            Serial.printf("[%u] 收到二进制数据，长度: %u\n", client_num, length);
            break;
        default:
            break;
    }
}

// --- OLED 显示更新函数 ---
void updateOledDisplay() {
    oled_display.clearBuffer(); // 清空显示缓冲区
    oled_display.setFont(u8g2_font_wqy12_t_gb2312); // 设置中文字体

    char oled_buf[40]; // 用于格式化输出到OLED的字符缓冲区，IP地址可能需要更长

    // 第一行：显示IP地址 (如果已连接WiFi)
    if (wifi_connected_flag) {
        snprintf(oled_buf, sizeof(oled_buf), "IP:%s", WiFi.localIP().toString().c_str());
    } else {
        snprintf(oled_buf, sizeof(oled_buf), "WiFi未连接");
    }
    oled_display.setCursor(0, 10); // 调整Y坐标以适应两行显示
    oled_display.print(oled_buf);

    // 第二行：显示融合后的流速 (单位: mL/h)
    float flow_rate_mlh = 0.0f;
    float current_density = drip_kf.getCurrentLiquidDensity(); // 获取当前液体密度
    if (drip_kf.getCalibratedWeightPerDrop() > 1e-6f && current_density > 1e-6f) { 
         flow_rate_mlh = (fused_flow_rate_gps / current_density) * 3600.0f;
    }
    snprintf(oled_buf, sizeof(oled_buf), "速率:%.1f mL/h", flow_rate_mlh);
    oled_display.setCursor(0, 22); // 调整Y坐标
    oled_display.print(oled_buf);
    
    // 第三行 (如果屏幕够大，或者滚动显示): 显示预计剩余时间 (单位: 分钟)
    // 当前128x32屏幕可能放不下三行清晰的中文，或者会很挤。
    // 暂时只显示两行核心信息。如果需要显示更多，可以考虑更小的字体或翻页。
    /*
    if (remaining_time_s >= 0 && remaining_time_s < (3600 * 24)) { 
        snprintf(oled_buf, sizeof(oled_buf), "剩余:%ld分", (long)(remaining_time_s / 60.0f));
    } else if (filt_weight_g <= target_empty_weight_g + 2.0f && fused_flow_rate_gps < 0.001f) {
        snprintf(oled_buf, sizeof(oled_buf), "剩余:已完成");
    } else {
        snprintf(oled_buf, sizeof(oled_buf), "剩余:计算中"); 
    }
    oled_display.setCursor(0, 30); // 调整Y坐标 (示例)
    oled_display.print(oled_buf);
    */

    oled_display.sendBuffer(); // 将缓冲区内容发送到OLED显示
}

// --- WiFi连接函数 ---
void connectToWiFi() {
    Serial.print("正在连接WiFi: ");
    Serial.println(WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) { // 最多尝试20次 (10秒)
        delay(500);
        Serial.print(".");
        attempts++;
    }
    if (WiFi.status() == WL_CONNECTED) {
        wifi_connected_flag = true;
        Serial.println("\nWiFi 已连接");
        Serial.print("IP 地址: ");
        Serial.println(WiFi.localIP());
    } else {
        wifi_connected_flag = false;
        Serial.println("\nWiFi 连接失败");
    }
}

// --- 系统初始化函数 --- 
void setup() {
    Serial.begin(115200); // 初始化串口通讯，波特率115200
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL); // 初始化I2C总线 (用于OLED)
    
    // 初始化OLED显示屏
    if (!oled_display.begin()) {
        Serial.println(F("错误: U8g2 OLED 初始化失败!"));
        // 考虑添加错误指示，例如LED快速闪烁
    }
    oled_display.enableUTF8Print(); // 使能UTF-8字符打印 (支持中文)
    oled_display.setFont(u8g2_font_wqy12_t_gb2312); // 设置默认字体
    oled_display.clearBuffer();
    oled_display.drawStr(0, 10, "系统启动中...");
    oled_display.sendBuffer();

    // 初始化HX711称重传感器
    scale_sensor.begin(PIN_HX711_DT, PIN_HX711_SCK); // 初始化HX711接口引脚
    scale_sensor.set_scale(hx711_cal_factor);       // 设置校准因子
    scale_sensor.set_gain(128);                     // 设置增益 (通常为128或64)
    Serial.println("HX711 去皮操作中，请勿在秤上放置物品...");
    oled_display.clearBuffer();
    oled_display.drawStr(0,10,"传感器去皮中...");
    oled_display.sendBuffer();
    scale_sensor.tare(10); // 执行去皮操作，读取10次数据取平均以提高稳定性
    Serial.println("HX711 去皮完成。");
    oled_display.clearBuffer();
    oled_display.drawStr(0,10,"去皮完成");
    oled_display.sendBuffer();
    delay(500);

    // 初始化卡尔曼滤波器状态和相关变量
    float initial_raw_w = scale_sensor.get_units(10); // 读取初始重量，多次读数取平均
    // 对非常离谱的初始读数进行简单保护
    if (fabsf(initial_raw_w) > 2000.0f || isnan(initial_raw_w) || isinf(initial_raw_w)) { 
        Serial.printf("警告: HX711初始读数异常 (%.2f g)，设为0。请检查传感器和校准因子。\n", initial_raw_w);
        initial_raw_w = 0.0f; 
    }
    
    raw_weight_g = initial_raw_w;
    filt_weight_g = initial_raw_w;
    prev_filt_weight_g = initial_raw_w;

    weight_kf.init(initial_raw_w, 0.0f); // 重量滤波器初始化 (初始速度为0)
    
    // 滴速滤波器初始化：初始滴速为0，使用默认参数计算初始每滴重量
    // initial_wpd_g_per_drip = -1.0f 表示使用内部基于 drops_per_ml 和 density 的默认计算
    drip_kf.init(0.0f, -1.0f, 20, 1.0f); 
    // drip_kf.startWpdCalibration(); // 系统启动时自动开始一次WPD校准。后续可通过WebSocket控制。
    // Serial.println("滴水传感器每滴重量(WPD)校准已在启动时自动开始。");

    flow_fusion.init(0.0f); // 数据融合模块初始化 (初始融合流速为0)

    // 初始化硬件引脚模式
    pinMode(PIN_WATER_SENSOR, INPUT_PULLDOWN); // 水滴传感器引脚设为输入，内部下拉
    pinMode(PIN_LED_STATUS, OUTPUT);          // LED状态指示灯引脚设为输出
    // 附加中断：当水滴传感器引脚电平从低到高 (RISING) 时，调用 onWaterDropIsr 函数
    attachInterrupt(digitalPinToInterrupt(PIN_WATER_SENSOR), onWaterDropIsr, RISING);

    // 连接WiFi网络
    connectToWiFi();
    if (wifi_connected_flag) {
        http_server.begin(); // 启动HTTP服务器
        Serial.printf("HTTP 服务器已启动，请访问: http://%s/\n", WiFi.localIP().toString().c_str());
        
        ws_server.begin(); 
        ws_server.onEvent(onWebSocketEvent); 
        Serial.println("WebSocket 服务器已成功启动。");

        // 在OLED上显示IP地址和速率 (初始速率为0)
        updateOledDisplay(); // 调用一次以显示IP

    } else {
        Serial.println("警告: 未连接到WiFi，HTTP和WebSocket服务器无法启动。");
        oled_display.clearBuffer();
        oled_display.drawStr(0, 10, "WiFi连接失败!");
        oled_display.sendBuffer();
    }

    // 闪烁LED三次，表示系统设置完成，准备进入主循环
    for (int i = 0; i < 3; i++) {
        digitalWrite(PIN_LED_STATUS, HIGH); delay(100);
        digitalWrite(PIN_LED_STATUS, LOW); delay(100);
    }
    Serial.println("系统初始化完成，进入主循环。");
    last_loop_run_ms = millis(); // 初始化主循环的计时器
}

// --- HTTP 请求处理函数 ---
void handleHttpRequests() {
    WiFiClient client = http_server.available(); // 尝试获取连接的客户端
    if (!client) {
        return; // 没有客户端连接
    }

    Serial.println("新的HTTP客户端已连接!");
    unsigned long current_http_req_time = millis();
    unsigned long previous_http_req_time = current_http_req_time;
    String current_line = ""; // 用于存储客户端请求的当前行

    while (client.connected() && (current_http_req_time - previous_http_req_time < 2000)) { // 2秒超时
        current_http_req_time = millis();
        if (client.available()) {
            char c = client.read();
            // Serial.write(c); // 调试：打印客户端请求原文
            if (c == '\n') { // 如果是换行符，表示一行的结束
                if (current_line.length() == 0) { // 如果连续收到两个换行符，表示请求头结束
                    // 发送HTTP响应
                    client.println("HTTP/1.1 200 OK");
                    client.println("Content-type:text/html; charset=UTF-8");
                    client.println("Connection: close"); // 告知客户端完成后关闭连接
                    client.println(); // HTTP头结束后的空行
                    
                    // 发送HTML页面内容
                    // 由于HTML_WEBPAGE可能很大，分块发送以避免缓冲区问题
                    const int chunkSize = 256; // 每次发送的块大小
                    int totalLen = strlen(HTML_WEBPAGE);
                    for (int i = 0; i < totalLen; i += chunkSize) {
                        client.print(String(HTML_WEBPAGE).substring(i, min(i + chunkSize, totalLen)));
                    }
                    client.println(); // 确保内容发送完毕
                    break; // 跳出while循环，因为已发送响应
                } else {
                    current_line = ""; // 清空current_line准备接收下一行
                }
            } else if (c != '\r') { // 如果不是回车符
                current_line += c; // 将字符附加到current_line
            }
            previous_http_req_time = current_http_req_time; // 重置超时计时器
        }
    }
    
    client.stop(); // 关闭连接
    Serial.println("HTTP客户端已断开连接。");
}

// --- 主循环函数 --- 
void loop() {
    // 如果WiFi已连接，处理WebSocket和HTTP请求
    if (wifi_connected_flag) {
        ws_server.loop(); // 处理WebSocket服务器事件
        handleHttpRequests(); // 处理HTTP请求
    }

    // 计算时间间隔，控制主循环执行频率
    unsigned long now_ms = millis(); // 获取当前时间戳（毫秒）
    if (now_ms - last_loop_run_ms >= MAIN_LOOP_INTERVAL_MS) {
        // 计算距离上次循环的时间间隔（秒）
        float dt_loop_s = (now_ms - last_loop_run_ms) / 1000.0f; 
        last_loop_run_ms = now_ms; // 更新上次循环时间戳

        // 安全地获取中断中累计的滴数
        noInterrupts(); // 禁用中断，防止数据竞争
        drops_in_loop_period = isr_drop_count_period; // 获取本周期内的滴数
        isr_drop_count_period = 0; // 重置中断计数器
        interrupts(); // 重新启用中断

        // 重量传感器数据处理
        prev_filt_weight_g = weight_kf.getWeight(); // 保存上一次滤波后的重量值
        if (scale_sensor.is_ready()) { // 检查HX711传感器是否就绪
            raw_weight_g = scale_sensor.get_units(5); // 读取原始重量（5次平均）
            // 异常值检测：如果读数异常（过大、NaN或Inf），使用上一次的滤波值代替
            if ((fabsf(raw_weight_g) > 2000.0f && fabsf(prev_filt_weight_g) < 1000.0f) || isnan(raw_weight_g) || isinf(raw_weight_g)) {
                Serial.printf("警告: HX711读数 %.2f g 异常! 使用上一滤波值 %.2f g 代替。\n", raw_weight_g, prev_filt_weight_g);
                raw_weight_g = prev_filt_weight_g; 
            }
        } else {
            // 传感器未就绪，使用上一次的滤波值
            raw_weight_g = prev_filt_weight_g; 
            Serial.println("警告: HX711 传感器未就绪!");
        }
        
        // 使用卡尔曼滤波器更新重量数据
        filt_weight_g = weight_kf.update(raw_weight_g, dt_loop_s); 
        // 计算重量变化（用于每滴重量校准）
        float weight_change_for_wpd_g = prev_filt_weight_g - filt_weight_g; 
        // 计算基于重量的流速（克/秒），负值表示重量增加，设为0
        flow_weight_gps = -weight_kf.getVelocity(); 
        if (flow_weight_gps < 0) flow_weight_gps = 0.0f; 
        
        // 长时间WPD校准处理
        // 在调用 drip_kf.update() 之前，如果正在进行长时间校准，则累加滴数
        if (wpd_long_cal_active) {
            wpd_long_cal_accum_drops += drops_in_loop_period;
        }
        // 即使不在长时间校准的计时/计数阶段，如果 drip_kf.isWpdCalibrating() 内部为true，
        // DripKalmanFilter 仍会使用 weight_change_for_wpd_g 来更新其WPD估计
        drip_kf.update(drops_in_loop_period, dt_loop_s, weight_change_for_wpd_g);
        
        // 更新滴速和流速相关变量
        filt_drip_rate_dps = drip_kf.getFilteredDripRate(); // 获取滤波后的滴速（滴/秒）
        flow_drip_gps = drip_kf.getFlowRateGramsPerSecond(); // 获取基于滴速的流速（克/秒）
        // 计算原始滴速（未经滤波）
        raw_drip_rate_dps = (dt_loop_s > 1e-3f) ? ((float)drops_in_loop_period / dt_loop_s) : 0.0f;
        // 融合重量和滴速两种方式计算的流速
        fused_flow_rate_gps = flow_fusion.update(flow_weight_gps, flow_drip_gps, dt_loop_s);
        if (fused_flow_rate_gps < 0) fused_flow_rate_gps = 0.0f; 
        
        // 计算剩余输液时间
        float remaining_weight_to_infuse = filt_weight_g - target_empty_weight_g;
        if (fused_flow_rate_gps > 1e-5f) { // 如果流速足够大
            remaining_time_s = remaining_weight_to_infuse / fused_flow_rate_gps;
            if (remaining_time_s < 0) remaining_time_s = 0.0f; // 防止负值
        } else if (remaining_weight_to_infuse <= 0.1f) { // 如果几乎没有剩余液体
             remaining_time_s = 0.0f; // 设置为0
        }
        
        // 检查并处理长时间WPD校准的完成条件
        if (wpd_long_cal_active) {
            // 计算校准已进行的时间
            unsigned long elapsed_cal_time_ms = now_ms - wpd_long_cal_start_ms;
            // 检查是否满足校准时长和最小滴数要求
            bool duration_met = elapsed_cal_time_ms >= WPD_LONG_CAL_DURATION_MS;
            bool drops_met = wpd_long_cal_accum_drops >= WPD_LONG_CAL_MIN_DROPS;

            if (duration_met && drops_met) {
                // 校准成功完成：时间和滴数都满足要求
                Serial.printf("WPD长时校准自动完成。时长: %.1fs, 总滴数: %d, 最终WPD: %.4f g/drip\n",
                              elapsed_cal_time_ms / 1000.0f,
                              wpd_long_cal_accum_drops,
                              drip_kf.getCalibratedWeightPerDrop());
                drip_kf.stopWpdCalibration(); // 通知DripKalmanFilter停止WPD更新模式
                wpd_long_cal_active = false; // 结束长时间校准状态
                
                // 向WebSocket客户端发送校准完成事件
                char cal_done_msg[120];
                snprintf(cal_done_msg, sizeof(cal_done_msg), "EVENT:WPD_CALIBRATION_COMPLETED,WPD:%.4f,Drops:%d,DurationSec:%.1f",
                         drip_kf.getCalibratedWeightPerDrop(), wpd_long_cal_accum_drops, elapsed_cal_time_ms / 1000.0f);
                if(ws_client_connected_flag) ws_server.broadcastTXT(cal_done_msg); // 广播给所有客户端
            } else if (duration_met && !drops_met) {
                // 时间到了但滴数不足的情况
                Serial.printf("WPD长时校准时间已到 (%.1fs)，但累计滴数 (%d) 未达目标 (%d)。可选择手动停止或等待更多滴数。\n",
                              elapsed_cal_time_ms / 1000.0f, wpd_long_cal_accum_drops, WPD_LONG_CAL_MIN_DROPS);
                // 也可以在这里自动停止，并通知用户滴数不足
                // drip_kf.stopWpdCalibration();
                // wpd_long_cal_active = false;
                // ws_server.broadcastTXT("EVENT:WPD_CALIBRATION_TIMED_OUT_LOW_DROPS"); 
            }
        }

        // 更新OLED显示
        updateOledDisplay();
        
        // 准备并输出调试信息到串口
        char serial_buf[280]; 
        snprintf(serial_buf, sizeof(serial_buf),
                 "T:%.2f,Rw:%.2f,Fw:%.2f,Vw:%.4f,Drps:%u,RawDPS:%.2f,FiltDPS:%.2f,VdGPS:%.4f,WPD:%.4f,Cal:%d,LCal:%d,FusedGPS:%.4f,RemS:%.0f",
                 now_ms / 1000.0f,            // 当前时间（秒）
                 raw_weight_g,                // 原始重量（克）
                 filt_weight_g,               // 滤波后重量（克）
                 flow_weight_gps,             // 基于重量的流速（克/秒）
                 drops_in_loop_period,        // 本周期滴数
                 raw_drip_rate_dps,           // 原始滴速（滴/秒）
                 filt_drip_rate_dps,          // 滤波后滴速（滴/秒）
                 flow_drip_gps,               // 基于滴速的流速（克/秒）
                 drip_kf.getCalibratedWeightPerDrop(), // 校准后的每滴重量（克/滴）
                 drip_kf.isWpdCalibrating() ? 1 : 0, // 底层KF是否在WPD更新模式
                 wpd_long_cal_active ? 1 : 0,        // 是否在长时校准流程中
                 fused_flow_rate_gps,         // 融合后的流速（克/秒）
                 remaining_time_s);           // 剩余时间（秒）
        Serial.println(serial_buf);
        
        // 如果WiFi已连接且有WebSocket客户端，发送数据更新
        if (wifi_connected_flag && ws_client_connected_flag) {
            // 更新WebSocket发送的数据，使其与前端JS解析逻辑匹配 (12个字段)
            // 格式: timestamp_ms,raw_w,filt_w,flow_w_gps,drops_p,raw_dps,filt_dps,flow_d_gps,wpd,wpd_cal_status,fused_gps,rem_s
            snprintf(serial_buf, sizeof(serial_buf),
                     "%lu,%.2f,%.2f,%.4f,%u,%.2f,%.2f,%.4f,%.4f,%d,%.4f,%.0f",
                     now_ms,                                 // 0: timestamp_ms
                     raw_weight_g,                           // 1: raw_w
                     filt_weight_g,                          // 2: filt_w
                     flow_weight_gps,                        // 3: flow_w_gps
                     drops_in_loop_period,                   // 4: drops_p
                     raw_drip_rate_dps,                      // 5: raw_dps
                     filt_drip_rate_dps,                     // 6: filt_dps
                     flow_drip_gps,                          // 7: flow_d_gps
                     drip_kf.getCalibratedWeightPerDrop(),   // 8: wpd
                     wpd_long_cal_active ? 1 : 0,            // 9: wpd_cal_status (来自长时间校准状态)
                     fused_flow_rate_gps,                    // 10: fused_gps
                     remaining_time_s);                      // 11: rem_s
            ws_server.broadcastTXT(serial_buf); // 广播数据给所有WebSocket客户端
        }
        
        // 切换状态LED，提供视觉反馈表明系统正在运行
        digitalWrite(PIN_LED_STATUS, !digitalRead(PIN_LED_STATUS)); 
    }
} 