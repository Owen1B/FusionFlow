// --- 标准库和第三方库引用 ---
#include <WiFi.h>             // 用于WiFi连接
#include <WiFiClient.h>       // 用于创建WiFi客户端 (HTTP Server会用到)
#include <DNSServer.h>        // (WiFiServer有时可能间接需要，但此处不直接使用DNS功能)
#include <WebSocketsServer.h> // 用于创建WebSocket服务器

#include <Wire.h>             // 用于I2C通讯 (OLED显示屏)
#include <Adafruit_GFX.h>     // GFX图形库 (U8g2可能间接依赖或共享某些概念)
#include <U8g2lib.h>          // OLED显示屏驱动库
#include <Adafruit_NeoPixel.h> // NeoPixel RGB LED 库

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
            <tr><th colspan="2">重量传感器 (Weight Sensor)</th></tr>
            <tr><td class="label">原始重量 (g):</td><td class="value" id="raw_weight_g">-</td></tr>
            <tr><td class="label">滤波后重量 (g):</td><td class="value" id="filt_weight_g">-</td></tr>
            <tr><td class="label">重量流速 (g/s):</td><td class="value" id="flow_weight_gps">-</td></tr>
            <tr><th colspan="2">滴速传感器 (Drip Sensor)</th></tr>
            <tr><td class="label">本周期滴数:</td><td class="value" id="drops_period">-</td></tr>
            <tr><td class="label">原始滴速 (dps):</td><td class="value" id="raw_drip_rate_dps">-</td></tr>
            <tr><td class="label">滤波后滴速 (dps):</td><td class="value" id="filt_drip_rate_dps">-</td></tr>
            <tr><td class="label">每滴重量 (g/drip):</td><td class="value" id="wpd_g">-</td></tr>
            <tr><td class="label">WPD校准中:</td><td class="value" id="wpd_calibrating">-</td></tr>
            <tr><td class="label">剩余重量 (滴数法) (g):</td><td class="value" id="rem_w_drip_calc_g">-</td></tr>
            <tr><td class="label">滴速流速 (g/s):</td><td class="value" id="flow_drip_gps">-</td></tr>
            <tr><td class="label">累计总滴数:</td><td class="value" id="total_drops_count">-</td></tr>
            <tr><td class="label">滴速传感器初始重量 (g):</td><td class="value" id="drip_initial_weight_g">-</td></tr>
            <tr><th colspan="2">融合数据 & 预测 (Fused Data & Prediction)</th></tr>
            <tr><td class="label">融合后流速 (g/s):</td><td class="value" id="fused_flow_gps">-</td></tr>
            <tr><td class="label">融合后剩余重量 (g):</td><td class="value" id="fused_rem_w_g">-</td></tr>
            <tr><td class="label">预计剩余时间:</td><td class="value" id="remaining_time_s">- 秒</td></tr>
            <tr><td class="label">输液进度 (%):</td><td class="value" id="infusion_progress_percent">-</td></tr>
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
        let systemInitialWeight = 0; // Still needed for context if progress is not available from WS
        let targetEmptyWeight = 60.0; // Still needed for context if progress is not available from WS

        function connectWebSocket() {
            const wsUrl = "ws://" + window.location.hostname + ":81/";
            ws = new WebSocket(wsUrl);
            document.getElementById("wsState").textContent = "正在连接到: " + wsUrl;

            ws.onopen = function(event) {
                document.getElementById("wsState").textContent = "WebSocket 已连接";
                console.log("WebSocket connection opened");
                if(displayTimer) clearInterval(displayTimer);
                displayTimer = setInterval(updateDisplay, displayInterval);
                 // Request initial parameters or wait for them
            };

            ws.onmessage = function(event) {
                const messageText = event.data;
                if (messageText.startsWith("ALERT:")) {
                    alert(messageText);
                    console.log("Received ESP32 Alert: " + messageText);
                    return;
                }
                if (messageText.startsWith("INITIAL_PARAMS:")) {
                    const params = messageText.split(':')[1].split(',');
                    systemInitialWeight = parseFloat(params[0]); // Keep these updated for any fallback or other JS logic
                    targetEmptyWeight = parseFloat(params[1]);
                    console.log("Received initial params: Initial Weight=" + systemInitialWeight + "g, Target Empty=" + targetEmptyWeight + "g");
                    return;
                }

                const data = messageText.split(',');
                // Expecting 25 columns now
                // 0:timestamp_ms, 1:raw_w, 2:filt_w, 3:raw_flow_w_gps, 4:filt_flow_w_gps,
                // 5:drops_p, 6:raw_dps, 7:filt_dps, 8:raw_flow_d_gps, 9:filt_flow_d_gps,
                // 10:wpd, 11:wpd_cal_status, 12:wpd_l_cal_active, 
                // 13:rem_w_drip_calc_g,
                // 14:fused_gps, 15:fused_rem_w_g,
                // 16:rem_t_raw_w, 17:rem_t_filt_w, 18:rem_t_raw_d, 19:rem_t_filt_d, 
                // 20:rem_t_fused (primary remaining time in seconds)
                // 21:drip_total_drops
                // 22:drip_initial_weight
                // 23:wpd_cumulative
                // 24:esp_calculated_progress_percent (-1 if not set)
                if (data.length >= 25) { // Check for 25 data points
                    if (data[0].toLowerCase().includes("timestamp_ms")) return; 
                    
                    lastData.timestamp_ms = data[0];
                    lastData.raw_weight_g = parseFloat(data[1]).toFixed(2);
                    lastData.filt_weight_g = parseFloat(data[2]).toFixed(2);
                    lastData.raw_flow_weight_gps = parseFloat(data[3]).toFixed(4); 
                    lastData.flow_weight_gps = parseFloat(data[4]).toFixed(4); 
                    lastData.drops_period = data[5];
                    lastData.raw_drip_rate_dps = parseFloat(data[6]).toFixed(2);
                    lastData.filt_drip_rate_dps = parseFloat(data[7]).toFixed(2);
                    lastData.raw_flow_drip_gps = parseFloat(data[8]).toFixed(4); 
                    lastData.flow_drip_gps = parseFloat(data[9]).toFixed(4); 
                    lastData.wpd_g = parseFloat(data[10]).toFixed(4);
                    lastData.wpd_calibrating = (data[11] === "1" || data[12] === "1") ? "是" : "否"; // Combine WPD cal status
                    // lastData.wpd_long_calibrating = (data[12] === "1") ? "是" : "否"; // Not displayed separately currently
                    lastData.rem_w_drip_calc_g = parseFloat(data[13]).toFixed(2);
                    lastData.fused_flow_gps = parseFloat(data[14]).toFixed(4); 
                    lastData.fused_rem_w_g = parseFloat(data[15]).toFixed(2);
                    lastData.remaining_time_s = parseFloat(data[20]).toFixed(0); 
                    
                    lastData.drip_total_drops = data[21];
                    lastData.drip_initial_weight_g = parseFloat(data[22]).toFixed(2);
                    // lastData.wpd_cumulative = parseFloat(data[23]).toFixed(5); // Not displayed currently

                    // Use progress directly from ESP32
                    let esp_progress = parseFloat(data[24]);
                    if (esp_progress >= 0 && esp_progress <= 100) {
                        lastData.infusion_progress_percent = esp_progress.toFixed(1);
                    } else {
                        lastData.infusion_progress_percent = "-"; // Show "-" if ESP sends -1 or invalid
                    }
                    // Removed old JS-side progress calculation
                } else {
                    // console.log("Received incomplete data (expected 25 fields): " + event.data);
                }
            };

            function updateDisplay() {
                 if (!lastData.timestamp_ms) return; 

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
                 document.getElementById("rem_w_drip_calc_g").textContent = lastData.rem_w_drip_calc_g;
                 
                 document.getElementById("total_drops_count").textContent = lastData.drip_total_drops || "-";
                 document.getElementById("drip_initial_weight_g").textContent = lastData.drip_initial_weight_g || "-";

                 document.getElementById("fused_flow_gps").textContent = lastData.fused_flow_gps;
                 document.getElementById("fused_rem_w_g").textContent = lastData.fused_rem_w_g;
                 
                 // Update remaining time display
                 let rt_s_val = parseFloat(lastData.remaining_time_s);
                 let rt_min_str = "---";
                 if (!isNaN(rt_s_val)) {
                     if (rt_s_val > 0 && rt_s_val < (3600*99)) {
                         rt_min_str = Math.floor(rt_s_val / 60).toString();
                     } else {
                         // Check for completion/near empty and low flow for 0 min display
                         // Need fused_rem_w_g (already in lastData), targetEmptyWeight (global JS), fused_flow_gps (already in lastData)
                         let current_fused_rem_w = parseFloat(lastData.fused_rem_w_g);
                         let current_fused_flow = parseFloat(lastData.fused_flow_gps);
                         if (!isNaN(current_fused_rem_w) && !isNaN(targetEmptyWeight) && !isNaN(current_fused_flow) &&
                             current_fused_rem_w <= targetEmptyWeight + 1.0 && Math.abs(current_fused_flow) < 0.001) {
                             rt_min_str = "0";
                         } else if (rt_s_val == 0) { // If remaining time is exactly 0 seconds, minutes should be 0.
                            rt_min_str = "0";
                         }
                     }
                     document.getElementById("remaining_time_s").textContent = rt_s_val.toFixed(0) + " 秒 (" + rt_min_str + " 分)";
                 } else {
                     document.getElementById("remaining_time_s").textContent = "- 秒 (--- 分)";
                 }

                 // Update progress display
                 if (lastData.infusion_progress_percent && lastData.infusion_progress_percent !== "-") {
                     document.getElementById("infusion_progress_percent").textContent = lastData.infusion_progress_percent + "%";
                 } else {
                     document.getElementById("infusion_progress_percent").textContent = "-%";
                 }
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
// const char* WIFI_SSID = "1503"; // 您的WiFi SSID
// const char* WIFI_PASS = "18310007230"; // 您的WiFi密码
const char* WIFI_SSID = "三井寿的iPhone13"; // 您的WiFi SSID
const char* WIFI_PASS = "12345678"; // 您的WiFi密码
bool wifi_connected_flag = false;

// --- WebSocket 服务器配置 ---
WebSocketsServer ws_server = WebSocketsServer(81); // WebSocket 服务器监听81端口
bool ws_client_connected_flag = false; // 标记是否有WebSocket客户端连接
// --- Infusion Abnormality Detection ---
bool infusion_abnormal = false;
volatile unsigned long last_drip_detected_time_ms = 0; // Updated in ISR, stores millis() of last detected drip
const unsigned long MAX_NO_DRIP_INTERVAL_MS = 10000; // 10 seconds threshold for no drip
const int PIN_ABNORMALITY_RESET_BUTTON = 39; // GPIO pin for abnormality reset button
int last_abnormality_reset_button_state = HIGH; // Assuming INPUT_PULLUP
unsigned long last_abnormality_reset_button_press_time = 0;
const unsigned long ABNORMALITY_RESET_BUTTON_DEBOUNCE_MS = 200; // Debounce time for reset button

// --- HTTP 服务器配置 ---
WiFiServer http_server(80); // HTTP服务器监听80端口

// --- 硬件引脚定义 ---
const int PIN_WATER_SENSOR = 11; // 水滴传感器引脚
#define NEOPIXEL_PIN 47        // NeoPixel LED 数据引脚
#define NEOPIXEL_BRIGHTNESS 50 // NeoPixel 亮度 (0-255)
Adafruit_NeoPixel pixels(1, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800); // 单个 NeoPixel 对象

// NeoPixel 颜色定义 (GRB format)
const uint32_t NEO_COLOR_OFF    = pixels.Color(0, 0, 0);
const uint32_t NEO_COLOR_RED    = pixels.Color(0, 255, 0); // Red for NeoPixel GRB
const uint32_t NEO_COLOR_GREEN  = pixels.Color(255, 0, 0); // Green for NeoPixel GRB
const uint32_t NEO_COLOR_BLUE   = pixels.Color(0, 0, 255); // Blue for NeoPixel GRB
const uint32_t NEO_COLOR_YELLOW = pixels.Color(255, 255, 0); // Yellow for NeoPixel GRB
const uint32_t NEO_COLOR_WHITE  = pixels.Color(255, 255, 255); // White

static bool neo_led_state_is_on = false; // For blinking logic in normal operation

const int PIN_I2C_SDA = 41;     // I2C SDA引脚 (OLED)
const int PIN_I2C_SCL = 42;     // I2C SCL引脚 (OLED)
const int PIN_HX711_DT = 17;    // HX711 数据引脚
const int PIN_HX711_SCK = 18;   // HX711 时钟引脚
const int PIN_INIT_BUTTON = 40;  // 初始化按钮引脚 (GPIO40)

// --- 新增：时间戳队列相关定义 ---
const int MAX_TIMESTAMP_QUEUE_SIZE = 20; // 队列最大容量改为3，保留最近3个时间戳
unsigned long drip_timestamps_ms[MAX_TIMESTAMP_QUEUE_SIZE]; // 时间戳队列
volatile int timestamp_queue_head = 0; // 队列头指针
volatile int timestamp_queue_tail = 0; // 队列尾指针
volatile bool timestamp_queue_full = false; // 队列满标志

// --- HX711 配置 ---
// !! 重要: 此校准因子必须根据您的HX711模块和称重传感器进行实际校准 !!
// !! 它直接影响重量读数的准确性。常见的校准方法是使用已知重量的物体。!!
float hx711_cal_factor = 1687.0f; // 示例校准因子，请务必重新校准!
HX711 scale_sensor; // HX711传感器对象

// --- OLED 显示屏配置 ---
#define OLED_WIDTH 128 // OLED宽度（像素）
#define OLED_HEIGHT 32 // OLED高度（像素）
// U8g2库的OLED驱动对象 (SSD1306, 128x32, I2C通讯)
U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C oled_display(U8G2_R0, U8X8_PIN_NONE);

// -----------------------------------
// --- 卡尔曼滤波器和数据融合对象实例化 ---
// -----------------------------------

// 参数调整指南:
// WeightKalmanFilter(sigma_a, R_sensor_noise)
//   sigma_a (过程噪声标准差 for 速度): 0.001-0.05 范围内调整。
//     - 较小值(0.001-0.005): 速度估计非常平滑，但对真实速度变化响应较慢，适合稳定流速场景
//     - 中等值(0.005-0.02): 平衡了平滑性和响应性，适合大多数场景
//     - 较大值(0.02-0.05): 对速度变化响应快，但可能引入更多噪声，适合流速频繁变化场景
//   R_sensor_noise (测量噪声方差 for 重量): 若原始重量读数标准差为s，则R约为s*s。
//     - 较小值(0.1-0.5): 更信任传感器读数，适合高精度传感器或稳定环境
//     - 中等值(0.5-2.0): 适合一般场景下的传感器
//     - 较大值(2.0-10.0): 传感器噪声大或存在频繁尖峰时使用，减少异常值影响
const float KF_WEIGHT_SIGMA_A = 0.0005f; // 过程噪声标准差 (影响速度估计) - 对应Python脚本中的 weight_sigma_a
const float KF_WEIGHT_SIGMA_J = 1e-6f;   // 过程噪声标准差 (影响加速度估计) - 对应Python脚本中的 weight_sigma_j
const float KF_WEIGHT_R_NOISE = 50.0f;  // 重量测量噪声方差 - 对应Python脚本中的 weight_R
WeightKalmanFilter weight_kf(KF_WEIGHT_SIGMA_A, KF_WEIGHT_SIGMA_J, KF_WEIGHT_R_NOISE); // 旧的构造函数调用

// DripKalmanFilter(drip_rate_sigma_a, drip_rate_R, wpd_Q, wpd_R)
//   drip_rate_sigma_a (滴速变化加速度标准差): 0.01-0.2 范围内调整。
//     - 较小值(0.01-0.05): 滴速估计更平滑，适合稳定滴速场景
//     - 中等值(0.05-0.1): 平衡了平滑性和响应性
//     - 较大值(0.1-0.2): 对滴速变化响应更快，适合滴速频繁变化场景
//   drip_rate_R (滴速测量方差): 1.0-10.0 范围内调整。
//     - 较小值(1.0-2.0): 更信任原始滴速计算，适合稳定的滴速传感器
//     - 中等值(2.0-5.0): 适合一般场景
//     - 较大值(5.0-10.0): 滴速传感器不稳定或存在误触发时使用
//   wpd_Q (WPD过程噪声方差): 0.000001-0.0001 范围内调整。
//     - 通常应设置很小(0.000001-0.00001)，因为每滴重量理论上应相对稳定
//     - 中等值(0.00001-0.00005): 适合大多数场景
//     - 较大值(0.00005-0.0001): 使WPD校准更快适应变化，但可能引入不稳定性
//   wpd_R (WPD测量噪声方差): 0.00001-0.001 范围内调整。
//     - 较小值(0.00001-0.0001): 更信任单次校准测量
//     - 中等值(0.0001-0.0005): 适合大多数场景
//     - 较大值(0.0005-0.001): 减少单次异常测量的影响，校准过程更平滑
DripKalmanFilter drip_kf(0.00001f, 0.05, 0.00000001f, 0.0001f); // 修改 WPD_R_noise

// DataFusion(q_flow, r_weight_flow, r_drip_flow, q_weight, r_weight_weight, r_drip_weight)
//   q_flow (流速过程噪声方差): 0.00001-0.001 范围内调整。
//     - 较小值(0.00001-0.0001): 融合后流速更平滑，但对真实变化响应慢
//     - 中等值(0.0001-0.0005): 适合大多数场景
//     - 较大值(0.0005-0.001): 对流速变化响应更快，但可能保留更多噪声
//   r_weight_flow/r_drip_flow (流速测量噪声方差): 0.001-0.01 范围内调整。
//     - 这两个参数的相对大小决定了对两种传感器的信任度
//     - 若r_weight_flow < r_drip_flow: 更信任重量传感器的流速测量
//     - 若r_weight_flow > r_drip_flow: 更信任滴速传感器的流速测量
//     - 若两者相等: 平等对待两种传感器的测量
//   q_weight (重量过程噪声方差): 0.001-0.1 范围内调整。
//     - 较小值(0.001-0.01): 融合后重量估计更平滑，适合稳定场景
//     - 中等值(0.01-0.05): 适合大多数场景
//     - 较大值(0.05-0.1): 对重量变化响应更快，适合重量快速变化场景
//   r_weight_weight/r_drip_weight (重量测量噪声方差): 0.1-10.0 范围内调整。
//     - 这两个参数的相对大小决定了对两种传感器重量估计的信任度
//     - 通常r_weight_weight(0.5-2.0) < r_drip_weight(1.0-5.0)，因为重量传感器直接测量重量，而滴速传感器是间接估计
DataFusion flow_fusion(0.0000001f, 0.01f, 0.0005f, 0.01f, 1.0f, 1.0f); // Updated constructor

// -----------------------------------
// ----------- 全局状态变量 ------------
// -----------------------------------

// ISR (中断服务程序) 中使用的变量必须声明为 volatile
volatile unsigned long isr_drop_count_period = 0; // ISR 在一个主循环周期内累计的滴数
volatile unsigned long isr_last_drop_time_ms = 0; // ISR 中记录的上一滴发生的时间戳 (ms)

// 主循环定时
const unsigned long MAIN_LOOP_INTERVAL_MS = 1000; // 主循环更新周期 (10 秒)
unsigned long last_loop_run_ms = 0; // 上次主循环运行的时间戳 (ms)

// 滴速传感器更新定时
// const unsigned long DRIP_SENSOR_UPDATE_INTERVAL_MS = 10000; // 滴速传感器更新周期 (例如: 10 秒)
// unsigned long last_drip_sensor_update_ms = 0; // 上次滴速传感器更新的时间戳 (ms)
float accumulated_weight_change_for_wpd_g = 0.0f; // 自上次滴速KF更新以来累计的重量变化 (g)

// 数据变量 (g: 克, s: 秒, dps: 滴/秒, gps: 克/秒, ms: 毫秒, mlh: 毫升/小时)
float raw_weight_g = 0.0f;            // HX711原始重量读数 (g)
float prev_raw_weight_g = 0.0f;       // 上一周期原始重量 (g) (用于计算原始重量流速)
float filt_weight_g = 0.0f;           // 卡尔曼滤波后的重量 (g)
float prev_filt_weight_g = 0.0f;      // 上一周期滤波后的重量 (g) (主循环周期)
float flow_weight_gps = 0.0f;         // 从重量传感器估算的流速 (g/s, 正值表示消耗)
float raw_flow_weight_gps = 0.0f;     // 从原始重量传感器估算的流速 (g/s)

unsigned int drops_this_drip_cycle = 0;  // 当前滴速更新周期内检测到的总滴数
float raw_drip_rate_dps = 0.0f;     // 由原始滴数计算的滴速 (dps) (基于长周期)
float filt_drip_rate_dps = 0.0f;    // 滴速卡尔曼滤波后的滴速 (dps)
float flow_drip_gps = 0.0f;           // 从滴速传感器估算的流速 (g/s)
float raw_flow_drip_gps = 0.0f;       // 从原始滴速传感器估算的流速 (g/s)

float fused_flow_rate_gps = 0.0f;     // 融合后的最终流速 (g/s)
float fused_remaining_weight_g = 0.0f; // 新增: 融合后的剩余重量 (g)
float remaining_weight_drip_calc_g = 0.0f; // 新增: 通过滴速计算的剩余重量 (g)

float remaining_time_s = 0.0f;        // 预计剩余输液时间 (秒) - 基于融合后的值和目标空重
float target_empty_weight_g = 70.0f; // 目标空袋重量 (g), 用于判断输液接近完成
                                
float remaining_time_raw_weight_s = 0.0f;  // 基于原始重量流速的剩余时间
float remaining_time_filt_weight_s = 0.0f; // 基于滤波后重量流速的剩余时间
float remaining_time_raw_drip_s = 0.0f;    // 基于原始滴速流速的剩余时间
float remaining_time_filt_drip_s = 0.0f;   // 基于滤波后滴速流速的剩余时间

// --- WPD 长时间校准相关变量 ---
bool wpd_long_cal_active = false;          // 标记是否正在进行WPD的长时间校准
unsigned long wpd_long_cal_start_ms = 0;   // WPD长时间校准的开始时间戳 (ms)
const unsigned long WPD_LONG_CAL_DURATION_MS = 60000; // WPD长时间校准的目标时长 (例如: 60秒)
int wpd_long_cal_accum_drops = 0;       // WPD长时间校准期间累计的总滴数
const int WPD_LONG_CAL_MIN_DROPS = 30;   // WPD长时间校准期间要求的最小累计滴数 (确保数据量)
                                      
// --- 新增系统级初始重量相关变量 ---
bool system_initial_weight_set = false;
float system_initial_total_liquid_weight_g = 0.0f;
bool alert_5_percent_triggered = false;

// --- 初始化按钮和快速收敛相关变量 ---
unsigned long last_init_button_press_time = 0;
const unsigned long INIT_BUTTON_DEBOUNCE_MS = 200; // 按钮去抖时间
int last_init_button_state = HIGH;             // 假设上拉，按下为LOW
bool fast_convergence_mode = false;
unsigned long fast_convergence_start_ms = 0;
float original_kf_weight_R_noise = KF_WEIGHT_R_NOISE; // 在setup中会被正确初始化
const unsigned long FAST_CONVERGENCE_DURATION_MS = 60000;

unsigned long drip_total_drops = 0;
unsigned long last_calc_time_ms = millis();
unsigned long last_serial_print_time_ms = millis();
unsigned long last_drip_count_update_time_ms = millis();
unsigned long last_button_check_time_ms = millis();

// 新增: DripKalmanFilter 原始R值备份
static float original_drip_kf_R_drip_rate_noise = 0.0f;
static float original_drip_kf_R_wpd_noise = 0.0f;

// 新增: DataFusion 原始R值备份
static float original_fusion_R_weight_flow = 0.0f;
static float original_fusion_R_drip_flow = 0.0f;
static float original_fusion_R_weight_weight = 0.0f;
static float original_fusion_R_drip_weight = 0.0f;

// 新增: WebSocket JS 使用的常量
const unsigned long DEFAULT_TARGET_TOTAL_DROPS_VOLUME_CALC = 100; // 用于JS客户端的默认目标滴数参考值

// 新增: 用于OLED显示和内部逻辑的全局状态值
float g_infusion_progress = 0.0f;                // 输液进度 (范围 0.0 到 1.0)
float g_oled_infused_progress_percent = 0.0f;    // OLED显示的输液百分比 (范围 0.0 到 100.0)
float g_oled_flow_rate_mlh = 0.0f;               // OLED显示的流速 (mL/h)
long  g_oled_remaining_time_min = -1;            // OLED显示的剩余分钟数 (-1 代表 "---")
// filt_weight_g 已经是全局变量，并将直接用于OLED显示

// -----------------------------------
// ------ 中断服务函数 (ISR) -----------
// -----------------------------------
// 当水滴传感器检测到滴落 (引脚电平上升) 时触发此函数
void IRAM_ATTR onWaterDropIsr() {
    unsigned long now_ms = millis();
    // 简单的消抖逻辑：确保两次中断之间有最小间隔，防止传感器抖动或噪声误触发
    if (now_ms - isr_last_drop_time_ms > 50) { 
        // 将时间戳加入队列
        int next_tail = (timestamp_queue_tail + 1) % MAX_TIMESTAMP_QUEUE_SIZE;
        if (next_tail != timestamp_queue_head || !timestamp_queue_full) {
            drip_timestamps_ms[timestamp_queue_tail] = now_ms;
            timestamp_queue_tail = next_tail;
            timestamp_queue_full = (timestamp_queue_tail == timestamp_queue_head);
        }
        isr_last_drop_time_ms = now_ms;
        last_drip_detected_time_ms = now_ms; // Update global last drip detected time
    }
}

// --- 新增：从队列中获取时间戳的函数 ---
bool getNextDripTimestamp(unsigned long& timestamp) {
    if (timestamp_queue_head == timestamp_queue_tail && !timestamp_queue_full) {
        return false; // 队列为空
    }
    
    timestamp = drip_timestamps_ms[timestamp_queue_head];
    timestamp_queue_head = (timestamp_queue_head + 1) % MAX_TIMESTAMP_QUEUE_SIZE;
    timestamp_queue_full = false;
    return true;
}

// --- 新增：获取队列中时间戳数量的函数 ---
int getTimestampQueueSize() {
    if (timestamp_queue_full) {
        return MAX_TIMESTAMP_QUEUE_SIZE;
    }
    return (timestamp_queue_tail - timestamp_queue_head + MAX_TIMESTAMP_QUEUE_SIZE) % MAX_TIMESTAMP_QUEUE_SIZE;
}

// -----------------------------------
// --- WebSocket 事件处理回调函数--------
// -----------------------------------
void onWebSocketEvent(uint8_t client_num, WStype_t type, uint8_t * payload, size_t length) {
    switch(type) {
        case WStype_DISCONNECTED: 
            if (client_num == 0) ws_client_connected_flag = false; // Assuming client_num 0 is the primary one we track
            Serial.printf("[%u] WebSocket客户端断开连接!\n", client_num);
            break;
        case WStype_CONNECTED: { 
            IPAddress client_ip = ws_server.remoteIP(client_num);
            Serial.printf("[%u] WebSocket客户端已连接，IP: %s\n", client_num, client_ip.toString().c_str());
            if (client_num == 0) ws_client_connected_flag = true;
            // Don't send HTML_WEBPAGE here directly, client should request it via HTTP GET
            // ws_server.sendTXT(client_num, "Connected to ESP32 WebSocket"); // Optional: send a welcome
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

// -----------------------------------
// -------- OLED 显示更新函数 ----------
// -----------------------------------
// void updateOledDisplay(float infused_progress_percent, float current_display_weight_g, float current_flow_rate_mlh, long current_remaining_time_min) { // OLD Signature
void updateOledDisplay() { // NEW Signature - no parameters
    oled_display.clearBuffer();
    oled_display.setFont(u8g2_font_wqy12_t_gb2312);

    char oled_buf[40]; 

    if (wifi_connected_flag) {
        snprintf(oled_buf, sizeof(oled_buf), "IP:%s", WiFi.localIP().toString().c_str());
    } else {
        snprintf(oled_buf, sizeof(oled_buf), "WiFi未连接");
    }
    oled_display.setCursor(0, 10); 
    oled_display.print(oled_buf);

    // 使用全局变量 g_oled_infused_progress_percent 和 filt_weight_g
    if (system_initial_weight_set) {
        snprintf(oled_buf, sizeof(oled_buf), "进度:%.0f%% %.0fg", g_oled_infused_progress_percent, filt_weight_g);
    } else {
        snprintf(oled_buf, sizeof(oled_buf), "进度:--%% %.0fg", filt_weight_g);
    }
    oled_display.setCursor(0, 22); 
    oled_display.print(oled_buf);
    
    // 使用全局变量 g_oled_flow_rate_mlh 和 g_oled_remaining_time_min
    if (g_oled_remaining_time_min != -1) {
        snprintf(oled_buf, sizeof(oled_buf), "%.0fmL/h %ldmin", g_oled_flow_rate_mlh, g_oled_remaining_time_min);
    } else {
        snprintf(oled_buf, sizeof(oled_buf), "%.0fmL/h ---min", g_oled_flow_rate_mlh);
    }
    oled_display.setCursor(0, 32);
    oled_display.print(oled_buf);

    oled_display.sendBuffer();
}

// -----------------------------------
// ------- WiFi连接函数 ---------------
// -----------------------------------
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

// -----------------------------------
// -------- 系统初始化函数 -------------
// -----------------------------------
void setup() {
    Serial.begin(115200); // 初始化串口通讯，波特率115200
    pinMode(PIN_INIT_BUTTON, INPUT_PULLUP); // 初始化按钮引脚，使用内部上拉
    pinMode(PIN_ABNORMALITY_RESET_BUTTON, INPUT_PULLUP); // Initialize abnormality reset button pin
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL); // 初始化I2C总线 (用于OLED)
    
    pixels.begin(); // 初始化 NeoPixel
    pixels.setBrightness(NEOPIXEL_BRIGHTNESS); // 设置亮度
    pixels.clear(); // 关闭所有像素
    pixels.show();  // 更新LED显示

    // 初始化OLED显示屏
    if (!oled_display.begin()) {
        Serial.println(F("错误: U8g2 OLED 初始化失败!"));
        // 考虑添加错误指示，例如LED快速闪烁
    }
    oled_display.enableUTF8Print(); // 使能UTF-8字符打印 (支持中文)
    oled_display.setFont(u8g2_font_wqy12_t_gb2312); // 设置中文字体
    oled_display.clearBuffer();
    oled_display.drawStr(0, 10, "系统启动中...");
    oled_display.sendBuffer();

    // 初始化HX711称重传感器
    scale_sensor.begin(PIN_HX711_DT, PIN_HX711_SCK); // 初始化HX711接口引脚
    scale_sensor.set_scale(hx711_cal_factor);       // 设置校准因子
    scale_sensor.set_gain(128);                     // 设置增益 (通常为128或64)
    
    // 设置真正的空秤偏移量
    const long TRUE_EMPTY_SCALE_OFFSET_ADC = 0; // 这是一个示例值，您需要获取真正的值
    scale_sensor.set_offset(TRUE_EMPTY_SCALE_OFFSET_ADC);

    // 初始化卡尔曼滤波器状态和相关变量
    if (scale_sensor.is_ready()) {
        float initial_weight_reading = scale_sensor.get_units(5); // 读取5次平均值作为初始重量
        // weight_kf.init(initial_weight_reading); // 旧的 init 调用 - 将被下方更完整的init取代
        Serial.print("Initial raw weight reading for KF: "); Serial.println(initial_weight_reading);
        raw_weight_g = initial_weight_reading - 12.0f;          // 然后减去12g //TODO: 这个12g皮重需要校准或配置
        filt_weight_g = raw_weight_g;
        prev_filt_weight_g = raw_weight_g;
        prev_raw_weight_g = raw_weight_g; // 初始化 prev_raw_weight_g

        weight_kf.init(raw_weight_g, 0.0f, 0.0f); // 重量滤波器初始化 (初始速度和加速度为0)
        Serial.print("Weight KF initialized with (after tare offset): "); Serial.println(raw_weight_g);
        
        // 滴速滤波器初始化：初始滴速为0，使用默认参数计算初始每滴重量
        // initial_wpd_g_per_drip = -1.0f 表示使用内部基于 drops_per_ml 和 density 的默认计算
        // WPD 初始值为 0.05g/drip (1ml/20drip * 1g/ml)
        // 将WPD测量噪声R从0.001f改为0.0001f
        drip_kf.init(0.0f, -1.0f, 20, 1.0f); 

        flow_fusion.init(0.0f, raw_weight_g); // 数据融合模块初始化 (初始融合流速为0, 初始融合重量为当前读数)

        // 初始化硬件引脚模式
        pinMode(PIN_WATER_SENSOR, INPUT_PULLDOWN); // 水滴传感器引脚设为输入，内部下拉
        pinMode(NEOPIXEL_PIN, OUTPUT);          // LED状态指示灯引脚设为输出
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

            updateOledDisplay();

        } else {
            Serial.println("警告: 未连接到WiFi，HTTP和WebSocket服务器无法启动。");
            oled_display.clearBuffer();
            oled_display.drawStr(0, 10, "WiFi连接失败!");
            oled_display.sendBuffer();
        }

        // NeoPixel 闪烁三次白色，表示系统设置完成
        for (int i = 0; i < 3; i++) {
            pixels.setPixelColor(0, NEO_COLOR_WHITE);
            pixels.show();
            delay(150); // 稍作调整闪烁时间
            pixels.setPixelColor(0, NEO_COLOR_OFF);
            pixels.show();
            delay(150);
        }
        Serial.println("系统初始化完成，进入主循环。");
        last_loop_run_ms = millis(); // 初始化主循环的计时器
        last_drip_detected_time_ms = millis(); // Initialize last drip time
        
        // 初始化系统总液体重量
        if (raw_weight_g > target_empty_weight_g + 10.0f) {
            system_initial_total_liquid_weight_g = raw_weight_g;
            system_initial_weight_set = true;
            drip_kf.setInitialLiquidWeightForVolumeCalc(system_initial_total_liquid_weight_g);
            flow_fusion.init(0.0f, system_initial_total_liquid_weight_g);
            Serial.printf("系统初始总液体重量已设定: %.2f g\n", system_initial_total_liquid_weight_g);
            if (ws_client_connected_flag) {
                char initial_params_msg[50];
                snprintf(initial_params_msg, sizeof(initial_params_msg), "INITIAL_PARAMS:%.1f,%.1f", 
                         system_initial_total_liquid_weight_g, target_empty_weight_g);
                ws_server.broadcastTXT(initial_params_msg);
            }
        }
    } else {
        Serial.println("HX711 not found.");
        // 可以选择在这里进入错误状态或使用默认值初始化滤波器
        // weight_kf.init(500.0f); // 旧的 init 调用 - 将被下方更完整的init取代
        raw_weight_g = 500.0f; // 默认重量
        filt_weight_g = raw_weight_g;
        prev_filt_weight_g = raw_weight_g;
        prev_raw_weight_g = raw_weight_g; // 初始化 prev_raw_weight_g

        weight_kf.init(raw_weight_g, 0.0f, 0.0f); // 重量滤波器初始化 (初始速度和加速度为0)
        Serial.print("Weight KF initialized with default weight: "); Serial.println(raw_weight_g);
        
        // 滴速滤波器初始化：初始滴速为0，使用默认参数计算初始每滴重量
        // initial_wpd_g_per_drip = -1.0f 表示使用内部基于 drops_per_ml 和 density 的默认计算
        // WPD 初始值为 0.05g/drip (1ml/20drip * 1g/ml)
        // 将WPD测量噪声R从0.001f改为0.0001f
        drip_kf.init(0.0f, -1.0f, 20, 1.0f); 

        flow_fusion.init(0.0f, raw_weight_g); // 数据融合模块初始化 (初始融合流速为0, 初始融合重量为当前读数)

        // 初始化硬件引脚模式
        pinMode(PIN_WATER_SENSOR, INPUT_PULLDOWN); // 水滴传感器引脚设为输入，内部下拉
        pinMode(NEOPIXEL_PIN, OUTPUT);          // LED状态指示灯引脚设为输出
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

            updateOledDisplay();

        } else {
            Serial.println("警告: 未连接到WiFi，HTTP和WebSocket服务器无法启动。");
            oled_display.clearBuffer();
            oled_display.drawStr(0, 10, "WiFi连接失败!");
            oled_display.sendBuffer();
        }

        // NeoPixel 闪烁三次白色，表示系统设置完成
        for (int i = 0; i < 3; i++) {
            pixels.setPixelColor(0, NEO_COLOR_WHITE);
            pixels.show();
            delay(150); // 稍作调整闪烁时间
            pixels.setPixelColor(0, NEO_COLOR_OFF);
            pixels.show();
            delay(150);
        }
        Serial.println("系统初始化完成，进入主循环。");
        last_loop_run_ms = millis(); // 初始化主循环的计时器
        last_drip_detected_time_ms = millis(); // Initialize last drip time
        
        // 初始化系统总液体重量
        if (raw_weight_g > target_empty_weight_g + 10.0f) {
            system_initial_total_liquid_weight_g = raw_weight_g;
            system_initial_weight_set = true;
            drip_kf.setInitialLiquidWeightForVolumeCalc(system_initial_total_liquid_weight_g);
            flow_fusion.init(0.0f, system_initial_total_liquid_weight_g);
            Serial.printf("系统初始总液体重量已设定: %.2f g\n", system_initial_total_liquid_weight_g);
            if (ws_client_connected_flag) {
                char initial_params_msg[50];
                snprintf(initial_params_msg, sizeof(initial_params_msg), "INITIAL_PARAMS:%.1f,%.1f", 
                         system_initial_total_liquid_weight_g, target_empty_weight_g);
                ws_server.broadcastTXT(initial_params_msg);
            }
        }
    }

    // 获取并保存原始的 KF_WEIGHT_R_NOISE 值
    // WeightKalmanFilter 构造时已经使用了 KF_WEIGHT_R_NOISE
    // 如果需要从对象获取，可以调用 weight_kf.getMeasurementNoise();
    // 但由于我们有 KF_WEIGHT_R_NOISE 常量，直接使用它作为原始值是可靠的。
    original_kf_weight_R_noise = KF_WEIGHT_R_NOISE; 

    // 初始化卡尔曼滤波器状态和相关变量
    if (scale_sensor.is_ready()) {
        float initial_weight_reading = scale_sensor.get_units(5); // 读取5次平均值作为初始重量
        // weight_kf.init(initial_weight_reading); // 旧的 init 调用 - 将被下方更完整的init取代
        Serial.print("Initial raw weight reading for KF: "); Serial.println(initial_weight_reading);
        raw_weight_g = initial_weight_reading - 12.0f;          // 然后减去12g //TODO: 这个12g皮重需要校准或配置
        filt_weight_g = raw_weight_g;
        prev_filt_weight_g = raw_weight_g;
        prev_raw_weight_g = raw_weight_g; // 初始化 prev_raw_weight_g

        weight_kf.init(raw_weight_g, 0.0f, 0.0f); // 重量滤波器初始化 (初始速度和加速度为0)
        Serial.print("Weight KF initialized with (after tare offset): "); Serial.println(raw_weight_g);
        
        // 滴速滤波器初始化：初始滴速为0，使用默认参数计算初始每滴重量
        // initial_wpd_g_per_drip = -1.0f 表示使用内部基于 drops_per_ml 和 density 的默认计算
        // WPD 初始值为 0.05g/drip (1ml/20drip * 1g/ml)
        // 将WPD测量噪声R从0.001f改为0.0001f
        drip_kf.init(0.0f, -1.0f, 20, 1.0f); 

        flow_fusion.init(0.0f, raw_weight_g); // 数据融合模块初始化 (初始融合流速为0, 初始融合重量为当前读数)

        // 初始化硬件引脚模式
        pinMode(PIN_WATER_SENSOR, INPUT_PULLDOWN); // 水滴传感器引脚设为输入，内部下拉
        pinMode(NEOPIXEL_PIN, OUTPUT);          // LED状态指示灯引脚设为输出
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

            updateOledDisplay();

        } else {
            Serial.println("警告: 未连接到WiFi，HTTP和WebSocket服务器无法启动。");
            oled_display.clearBuffer();
            oled_display.drawStr(0, 10, "WiFi连接失败!");
            oled_display.sendBuffer();
        }

        // NeoPixel 闪烁三次白色，表示系统设置完成
        for (int i = 0; i < 3; i++) {
            pixels.setPixelColor(0, NEO_COLOR_WHITE);
            pixels.show();
            delay(150); // 稍作调整闪烁时间
            pixels.setPixelColor(0, NEO_COLOR_OFF);
            pixels.show();
            delay(150);
        }
        Serial.println("系统初始化完成，进入主循环。");
        last_loop_run_ms = millis(); // 初始化主循环的计时器
        last_drip_detected_time_ms = millis(); // Initialize last drip time
        
        // 初始化系统总液体重量
        if (raw_weight_g > target_empty_weight_g + 10.0f) {
            system_initial_total_liquid_weight_g = raw_weight_g;
            system_initial_weight_set = true;
            drip_kf.setInitialLiquidWeightForVolumeCalc(system_initial_total_liquid_weight_g);
            flow_fusion.init(0.0f, system_initial_total_liquid_weight_g);
            Serial.printf("系统初始总液体重量已设定: %.2f g\n", system_initial_total_liquid_weight_g);
            if (ws_client_connected_flag) {
                char initial_params_msg[50];
                snprintf(initial_params_msg, sizeof(initial_params_msg), "INITIAL_PARAMS:%.1f,%.1f", 
                         system_initial_total_liquid_weight_g, target_empty_weight_g);
                ws_server.broadcastTXT(initial_params_msg);
            }
        }
    } else {
        Serial.println("HX711 not found.");
        // 可以选择在这里进入错误状态或使用默认值初始化滤波器
        // weight_kf.init(500.0f); // 旧的 init 调用 - 将被下方更完整的init取代
        raw_weight_g = 500.0f; // 默认重量
        filt_weight_g = raw_weight_g;
        prev_filt_weight_g = raw_weight_g;
        prev_raw_weight_g = raw_weight_g; // 初始化 prev_raw_weight_g

        weight_kf.init(raw_weight_g, 0.0f, 0.0f); // 重量滤波器初始化 (初始速度和加速度为0)
        Serial.print("Weight KF initialized with default weight: "); Serial.println(raw_weight_g);
        
        // 滴速滤波器初始化：初始滴速为0，使用默认参数计算初始每滴重量
        // initial_wpd_g_per_drip = -1.0f 表示使用内部基于 drops_per_ml 和 density 的默认计算
        // WPD 初始值为 0.05g/drip (1ml/20drip * 1g/ml)
        // 将WPD测量噪声R从0.001f改为0.0001f
        drip_kf.init(0.0f, -1.0f, 20, 1.0f); 

        flow_fusion.init(0.0f, raw_weight_g); // 数据融合模块初始化 (初始融合流速为0, 初始融合重量为当前读数)

        // 初始化硬件引脚模式
        pinMode(PIN_WATER_SENSOR, INPUT_PULLDOWN); // 水滴传感器引脚设为输入，内部下拉
        pinMode(NEOPIXEL_PIN, OUTPUT);          // LED状态指示灯引脚设为输出
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

            updateOledDisplay();

        } else {
            Serial.println("警告: 未连接到WiFi，HTTP和WebSocket服务器无法启动。");
            oled_display.clearBuffer();
            oled_display.drawStr(0, 10, "WiFi连接失败!");
            oled_display.sendBuffer();
        }

        // NeoPixel 闪烁三次白色，表示系统设置完成
        for (int i = 0; i < 3; i++) {
            pixels.setPixelColor(0, NEO_COLOR_WHITE);
            pixels.show();
            delay(150);
            pixels.setPixelColor(0, NEO_COLOR_OFF);
            pixels.show();
            delay(150);
        }
        Serial.println("系统初始化完成，进入主循环。");
        last_loop_run_ms = millis(); // 初始化主循环的计时器
        last_drip_detected_time_ms = millis(); // Initialize last drip time
        
        // 初始化系统总液体重量
        if (raw_weight_g > target_empty_weight_g + 10.0f) {
            system_initial_total_liquid_weight_g = raw_weight_g;
            system_initial_weight_set = true;
            drip_kf.setInitialLiquidWeightForVolumeCalc(system_initial_total_liquid_weight_g);
            flow_fusion.init(0.0f, system_initial_total_liquid_weight_g);
            Serial.printf("系统初始总液体重量已设定: %.2f g\n", system_initial_total_liquid_weight_g);
            if (ws_client_connected_flag) {
                char initial_params_msg[50];
                snprintf(initial_params_msg, sizeof(initial_params_msg), "INITIAL_PARAMS:%.1f,%.1f", 
                         system_initial_total_liquid_weight_g, target_empty_weight_g);
                ws_server.broadcastTXT(initial_params_msg);
            }
        }
    }

    // 备份原始的重量滤波器R值
    original_kf_weight_R_noise = weight_kf.getMeasurementNoise();
    Serial.printf("原始重量 KF R: %.4f\n", original_kf_weight_R_noise);

    // 新增: 备份 DripKalmanFilter 的原始R值
    original_drip_kf_R_drip_rate_noise = drip_kf.getDripRateMeasurementNoise();
    original_drip_kf_R_wpd_noise = drip_kf.getWpdMeasurementNoise();
    Serial.printf("原始Drip KF R_drip_rate: %.4f, R_wpd: %.4f\n", original_drip_kf_R_drip_rate_noise, original_drip_kf_R_wpd_noise);

    // 新增: 备份 DataFusion 的原始R值
    flow_fusion.getFlowMeasurementNoises(original_fusion_R_weight_flow, original_fusion_R_drip_flow);
    flow_fusion.getWeightMeasurementNoises(original_fusion_R_weight_weight, original_fusion_R_drip_weight);
    Serial.printf("原始Fusion R_flow (w,d): %.4f, %.4f; R_weight (w,d): %.4f, %.4f\n", 
                  original_fusion_R_weight_flow, original_fusion_R_drip_flow, 
                  original_fusion_R_weight_weight, original_fusion_R_drip_weight);

    // 初始化看门狗 (如果需要)
    // ... existing code ...
}

// -----------------------------------
// ---- HTTP 请求处理函数 --------------
// -----------------------------------
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

// --- 辅助函数：计算特定条件下的剩余时间 ---
float calculate_specific_remaining_time(float current_liquid_weight, float target_empty_ref_weight, float current_flow_rate_gps, float undefined_time_value = 88888.0f) {
    float weight_to_infuse = current_liquid_weight - target_empty_ref_weight;
    if (weight_to_infuse <= 0.01f) { // 已达到或低于目标空重 (带小容差)
        return 0.0f;
    }
    if (current_flow_rate_gps > 1e-5f) { // 有效的正流速
        float time = weight_to_infuse / current_flow_rate_gps;
        // 确保时间不为负 (如果 weight_to_infuse 为正，则不应发生)
        return (time < 0.0f) ? 0.0f : ((time > 999999.0f) ? 999999.0f : time);
    } else { // 流速为零或可忽略
        return undefined_time_value; // 表示非常长或未定义的时间
    }
}

// --- 新增：系统重新初始化函数 ---
void performSystemReinitialization() {
    Serial.println("系统重新初始化请求...");

    float new_initial_weight = 0.0f;
    if (scale_sensor.is_ready()) {
        new_initial_weight = scale_sensor.get_units(10);
        if (isnan(new_initial_weight) || isinf(new_initial_weight) || fabsf(new_initial_weight) > 5000.0f) {
            Serial.printf("警告: 重新初始化时重量读数异常: %.2f, 使用500g作为替代.\n", new_initial_weight);
            new_initial_weight = 500.0f; 
        }
    } else {
        Serial.println("警告: 重新初始化时 HX711 未就绪，使用500g作为初始重量。");
        new_initial_weight = 500.0f;
    }
    if (new_initial_weight < 1.0f && new_initial_weight > -1.0f) { // If reading is very close to zero (e.g. after faulty read returning 0)
        Serial.println("警告: 重新初始化重量读数接近零，可能不准确，强制使用500g。");
        new_initial_weight = 500.0f;
    }
    new_initial_weight = new_initial_weight - 12.0f;
    system_initial_weight_set = true;
    system_initial_total_liquid_weight_g = new_initial_weight;
    
    Serial.printf("系统重新初始化完成。新的初始总重量: %.1fg, 当前目标空重: %.1fg\n", 
                  system_initial_total_liquid_weight_g, target_empty_weight_g); // Adjusted message slightly

    weight_kf.init(new_initial_weight, 0.0f, 0.0f);
    drip_kf.init(); 
    drip_kf.setInitialLiquidWeightForVolumeCalc(system_initial_total_liquid_weight_g);
    flow_fusion.init(0.0f, new_initial_weight);

    drip_kf.stopWpdCalibration();
    wpd_long_cal_active = false;
    // last_wpd_cal_check_time = millis(); // Commented out as it seems no longer actively used for WPD cal triggering



    Serial.println("所有滤波器进入快速收敛模式 (60秒).");
    fast_convergence_mode = true;
    fast_convergence_start_ms = millis(); // Corrected variable name

    float fast_R_weight = original_kf_weight_R_noise / 10.0f;
    if (fast_R_weight < 1e-7f) fast_R_weight = 1e-7f; // Ensure not too small
    weight_kf.setMeasurementNoise(fast_R_weight);
    Serial.printf("  WeightKF R: %.4f -> %.4f\n", original_kf_weight_R_noise, fast_R_weight);

    float fast_R_drip_rate = original_drip_kf_R_drip_rate_noise / 10.0f;
    float fast_R_wpd = original_drip_kf_R_wpd_noise / 10.0f;
    if (fast_R_drip_rate < 1e-7f) fast_R_drip_rate = 1e-7f;
    if (fast_R_wpd < 1e-7f) fast_R_wpd = 1e-7f; 
    drip_kf.setDripRateMeasurementNoise(fast_R_drip_rate);
    drip_kf.setWpdMeasurementNoise(fast_R_wpd);
    Serial.printf("  DripKF R_drip_rate: %.4f -> %.4f, R_wpd: %.4f -> %.4f\n", 
                  original_drip_kf_R_drip_rate_noise, fast_R_drip_rate, 
                  original_drip_kf_R_wpd_noise, fast_R_wpd);

    float fast_R_fusion_w_flow = original_fusion_R_weight_flow / 10.0f;
    float fast_R_fusion_d_flow = original_fusion_R_drip_flow / 10.0f;
    float fast_R_fusion_w_weight = original_fusion_R_weight_weight / 10.0f;
    float fast_R_fusion_d_weight = original_fusion_R_drip_weight / 10.0f;
    if (fast_R_fusion_w_flow < 1e-7f) fast_R_fusion_w_flow = 1e-7f;
    if (fast_R_fusion_d_flow < 1e-7f) fast_R_fusion_d_flow = 1e-7f;
    if (fast_R_fusion_w_weight < 1e-7f) fast_R_fusion_w_weight = 1e-7f;
    if (fast_R_fusion_d_weight < 1e-7f) fast_R_fusion_d_weight = 1e-7f;
    flow_fusion.setFlowMeasurementNoises(fast_R_fusion_w_flow, fast_R_fusion_d_flow);
    flow_fusion.setWeightMeasurementNoises(fast_R_fusion_w_weight, fast_R_fusion_d_weight);
    Serial.printf("  Fusion R_flow(w,d): (%.4f,%.4f)->(%.4f,%.4f)\n",
                  original_fusion_R_weight_flow, original_fusion_R_drip_flow,
                  fast_R_fusion_w_flow, fast_R_fusion_d_flow);
    Serial.printf("  Fusion R_weight(w,d): (%.4f,%.4f)->(%.4f,%.4f)\n",
                  original_fusion_R_weight_weight, original_fusion_R_drip_weight,
                  fast_R_fusion_w_weight, fast_R_fusion_d_weight);

    // NeoPixel feedback for reinitialization
    pixels.setPixelColor(0, NEO_COLOR_WHITE);
    pixels.show();
    delay(250); // Show white briefly
    // The LED will then be updated by the main loop logic based on the new state (e.g., green or blue if calibration starts)
    // For blinking, ensure the static blink state variable is reset or starts appropriately
    neo_led_state_is_on = false; // Start with LED off for the next blink cycle in normal mode

    updateOledDisplay();
    if (ws_client_connected_flag) {
        char reinit_msg[150];
        snprintf(reinit_msg, sizeof(reinit_msg), 
                 "ALERT:System Re-initialized. New initial weight: %.1fg. Target empty: %.1fg. Fast convergence mode ON (60s).", 
                 system_initial_total_liquid_weight_g, target_empty_weight_g);
        ws_server.broadcastTXT(reinit_msg);
        char initial_params_msg[100];
        snprintf(initial_params_msg, sizeof(initial_params_msg), "INITIAL_PARAMS:%.1f,%.1f,%.1f",
                 system_initial_total_liquid_weight_g, target_empty_weight_g, (float)DEFAULT_TARGET_TOTAL_DROPS_VOLUME_CALC);
        ws_server.broadcastTXT(initial_params_msg);
    }
}

// -----------------------------------
// -------- 主循环函数 ----------------
// -----------------------------------
void loop() {
    unsigned long current_time_ms = millis(); // Use this for most checks in the loop

    // --- Button Checks ---
    // Abnormality Reset Button
    bool current_reset_button_state = digitalRead(PIN_ABNORMALITY_RESET_BUTTON);
    if (current_reset_button_state == LOW && last_abnormality_reset_button_state == HIGH &&
        (current_time_ms - last_abnormality_reset_button_press_time > ABNORMALITY_RESET_BUTTON_DEBOUNCE_MS)) {
        last_abnormality_reset_button_press_time = current_time_ms;
        if (infusion_abnormal) {
            Serial.println("输液异常状态已通过按钮解除。恢复正常监控。");
            infusion_abnormal = false;
            last_drip_detected_time_ms = current_time_ms; // Reset drip timer
            if (wifi_connected_flag && ws_client_connected_flag) { // Notify WS
                ws_server.broadcastTXT("ALERT:INFUSION_ABNORMALITY_CLEARED_BY_BUTTON");
            }
        }
    }
    last_abnormality_reset_button_state = current_reset_button_state;

    // System Re-initialization Button (existing, with additions for abnormality reset)
    bool current_init_button_state = digitalRead(PIN_INIT_BUTTON);
    if (current_init_button_state == LOW && last_init_button_state == HIGH && (current_time_ms - last_init_button_press_time > INIT_BUTTON_DEBOUNCE_MS)) {
        last_init_button_press_time = current_time_ms;
        performSystemReinitialization();
        infusion_abnormal = false; // Re-init should clear abnormal state
        last_drip_detected_time_ms = current_time_ms; // Reset drip timer after re-init
    }
    last_init_button_state = current_init_button_state;

    // --- Abnormality Detection ---
    // Check only if system is initialized (initial weight set) and not already in abnormal state
    if (system_initial_weight_set && !infusion_abnormal) {
        if (current_time_ms - last_drip_detected_time_ms > MAX_NO_DRIP_INTERVAL_MS) {
            Serial.println("警告: 输液异常！超过10秒未检测到液滴。");
            infusion_abnormal = true;
            if (wifi_connected_flag && ws_client_connected_flag) {
                ws_server.broadcastTXT("ALERT:INFUSION_ABNORMALITY_NO_DRIPS_DETECTED");
            }
        }
    }
    
    // --- Fast convergence mode processing (existing, with addition for drip timer reset) ---
    if (fast_convergence_mode) {
        if (millis() - fast_convergence_start_ms >= FAST_CONVERGENCE_DURATION_MS) { 
            Serial.println("快速收敛模式结束，恢复所有滤波器原始R值。");
            
            weight_kf.setMeasurementNoise(original_kf_weight_R_noise);
            Serial.printf("  WeightKF R restored: %.4f\n", original_kf_weight_R_noise);
            
            drip_kf.setDripRateMeasurementNoise(original_drip_kf_R_drip_rate_noise);
            drip_kf.setWpdMeasurementNoise(original_drip_kf_R_wpd_noise);
            Serial.printf("  DripKF R_drip_rate restored: %.4f, R_wpd restored: %.4f\n", 
                          original_drip_kf_R_drip_rate_noise, original_drip_kf_R_wpd_noise);

            flow_fusion.setFlowMeasurementNoises(original_fusion_R_weight_flow, original_fusion_R_drip_flow);
            flow_fusion.setWeightMeasurementNoises(original_fusion_R_weight_weight, original_fusion_R_drip_weight);
            Serial.printf("  Fusion R_flow(w,d) restored: (%.4f,%.4f)\n", original_fusion_R_weight_flow, original_fusion_R_drip_flow);
            Serial.printf("  Fusion R_weight(w,d) restored: (%.4f,%.4f)\n", original_fusion_R_weight_weight, original_fusion_R_drip_weight);

            fast_convergence_mode = false;
            last_drip_detected_time_ms = current_time_ms; // Reset drip timer when fast convergence ends

            if (system_initial_weight_set && !drip_kf.isWpdCalibrating()) {
                drip_kf.startWpdCalibration();
                Serial.println("WPD校准已在快速收garan哦模式结束后自动启动。");
                if (ws_client_connected_flag) {
                    ws_server.broadcastTXT("ALERT:Fast convergence OFF. WPD calibration auto-started. Drip monitoring active.");
                }
            } else if (ws_client_connected_flag) { 
                 ws_server.broadcastTXT("ALERT:Fast convergence mode OFF. Filters restored. Drip monitoring active.");
            }
        }
    }

    // --- Network tasks (always run) ---
    if (wifi_connected_flag) {
        ws_server.loop(); 
        handleHttpRequests(); 
    }

    // --- Main processing logic OR Abnormal state handling ---
    if (infusion_abnormal) {
        pixels.setPixelColor(0, NEO_COLOR_RED); // Solid Red for abnormal state
        pixels.show();
        // Main processing is skipped.
        // If no network tasks are running (e.g. WiFi down), add a small delay to prevent tight loop.
        if (!wifi_connected_flag) {
            delay(100); 
        }
    } else { // Normal operation: run main sensor processing loop
        // Calculate time delta for main loop
        float dt_main_loop_s = (current_time_ms - last_loop_run_ms) / 1000.0f;

        if (dt_main_loop_s >= (MAIN_LOOP_INTERVAL_MS / 1000.0f)) {
            last_loop_run_ms = current_time_ms; // Update last run time for the main processing block
            
            // --- START OF EXISTING MAIN PROCESSING LOGIC ---
            float current_raw_weight_g_for_flow_calc = 0.0f; 
            float prev_filt_weight_g_this_main_loop = filt_weight_g; 
            
            if (scale_sensor.is_ready()) { 
                float gross_weight_reading_g = scale_sensor.get_units(5); 
                raw_weight_g = gross_weight_reading_g - 12.0f;    
                current_raw_weight_g_for_flow_calc = raw_weight_g; 
                if ((fabsf(raw_weight_g) > 2000.0f && fabsf(prev_filt_weight_g_this_main_loop) < 1000.0f) || isnan(raw_weight_g) || isinf(raw_weight_g)) {
                    Serial.printf("警告: HX711读数 %.2f g 异常! 使用上一滤波值 %.2f g 代替。\n", raw_weight_g, prev_filt_weight_g_this_main_loop);
                    raw_weight_g = prev_filt_weight_g_this_main_loop; 
                }
            } else {
                raw_weight_g = prev_filt_weight_g_this_main_loop; 
                current_raw_weight_g_for_flow_calc = prev_raw_weight_g; 
                Serial.println("警告: HX711 传感器未就绪!");
            }
            
            if (dt_main_loop_s > 1e-5f) {
                raw_flow_weight_gps = (prev_raw_weight_g - current_raw_weight_g_for_flow_calc) / dt_main_loop_s;
            } else {
                raw_flow_weight_gps = 0.0f;
            }
            if (raw_flow_weight_gps < 0 && fabsf(raw_flow_weight_gps) > 0.0001f) { 
                 raw_flow_weight_gps = 0.0f;
            } else if (raw_flow_weight_gps < 0) {
                raw_flow_weight_gps = 0.0f; 
            }
            prev_raw_weight_g = current_raw_weight_g_for_flow_calc; 

            filt_weight_g = weight_kf.update(raw_weight_g, dt_main_loop_s);
            flow_weight_gps = -weight_kf.getVelocity(); 
            if (flow_weight_gps < 0) flow_weight_gps = 0.0f; 

            accumulated_weight_change_for_wpd_g += (prev_filt_weight_g_this_main_loop - filt_weight_g);

            float dt_drip_s = dt_main_loop_s;
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
                drip_kf.update(measured_drip_rate, dt_drip_s, accumulated_weight_change_for_wpd_g);
                if(system_initial_weight_set) {
                    drip_kf.updateTotalDropsForVolumeCalc(valid_intervals);
                }
                drops_this_drip_cycle = valid_intervals;
                accumulated_weight_change_for_wpd_g = 0.0f;
                filt_drip_rate_dps = drip_kf.getFilteredDripRate();
                flow_drip_gps = drip_kf.getFlowRateGramsPerSecond();
                raw_drip_rate_dps = measured_drip_rate;
                raw_flow_drip_gps = raw_drip_rate_dps * drip_kf.getCalibratedWeightPerDrop();
                if (raw_flow_drip_gps < 0) raw_flow_drip_gps = 0.0f;
                drip_timestamps_ms[0] = timestamps[timestamp_count-1];
                timestamp_queue_head = 0;
                timestamp_queue_tail = 1;
                timestamp_queue_full = false;
            }
            
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

            if (drip_kf.isWpdCalibrating() && system_initial_weight_set) { 
                drip_kf.calibrateWpdByTotal(filt_weight_g); 
            }

            if (wpd_long_cal_active) {
                unsigned long elapsed_cal_time_ms = current_time_ms - wpd_long_cal_start_ms; // Use current_time_ms from loop start
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
                    snprintf(cal_done_msg, sizeof(cal_done_msg), "EVENT:WPD_CALIBRATION_COMPLETED,WPD:%.4f,Drops:%d,DurationSec:%.1f",
                             drip_kf.getCalibratedWeightPerDrop(), wpd_long_cal_accum_drops, elapsed_cal_time_ms / 1000.0f);
                    if(ws_client_connected_flag) ws_server.broadcastTXT(cal_done_msg); 
                } else if (duration_met && !drops_met) {
                    Serial.printf("WPD长时校准时间已到 (%.1fs)，但累计滴数 (%d) 未达目标 (%d)。可选择手动停止或等待更多滴数。\n",
                                  elapsed_cal_time_ms / 1000.0f, wpd_long_cal_accum_drops, WPD_LONG_CAL_MIN_DROPS);
                }
            }

            if(system_initial_weight_set){
                remaining_weight_drip_calc_g = drip_kf.getRemainingWeightByDropsG();
            } else {
                remaining_weight_drip_calc_g = filt_weight_g; 
            }

            unsigned long drip_total_drops_val = drip_kf.total_drops_for_volume_calc;
            float drip_initial_weight_val = drip_kf.known_initial_total_weight_g;
            float wpd_cumulative_val = 0.0f;
            if (drip_total_drops_val > 0) {
                wpd_cumulative_val = (drip_initial_weight_val - filt_weight_g) / drip_total_drops_val;
            }

            flow_fusion.update(flow_weight_gps, flow_drip_gps, filt_weight_g, remaining_weight_drip_calc_g, dt_main_loop_s);
            fused_flow_rate_gps = flow_fusion.getFusedFlowRateGps();
            if (fused_flow_rate_gps < 0) fused_flow_rate_gps = 0.0f; 
            fused_remaining_weight_g = flow_fusion.getFusedRemainingWeightG();
            if (fused_remaining_weight_g < 0) fused_remaining_weight_g = 0.0f;

            float remaining_weight_to_infuse_fused = fused_remaining_weight_g - target_empty_weight_g;
            if (remaining_weight_to_infuse_fused < 0) remaining_weight_to_infuse_fused = 0.0f; 

            float base_remaining_time_s = 0.0f;
            if (fused_flow_rate_gps > 1e-5f) {
                base_remaining_time_s = remaining_weight_to_infuse_fused / fused_flow_rate_gps;
            } else {
                base_remaining_time_s = (remaining_weight_to_infuse_fused <= 0.01f ? 0.0f : 999999.0f);
            }
            remaining_time_s = base_remaining_time_s; // Simplified: removed coef adjustment for now

            if (remaining_time_s < 0) remaining_time_s = 0.0f;
            if (remaining_time_s > 999999.0f) remaining_time_s = 999999.0f; 

            remaining_time_raw_weight_s = calculate_specific_remaining_time(raw_weight_g, target_empty_weight_g, raw_flow_weight_gps);
            remaining_time_filt_weight_s = calculate_specific_remaining_time(filt_weight_g, target_empty_weight_g, flow_weight_gps);
            remaining_time_raw_drip_s = calculate_specific_remaining_time(remaining_weight_drip_calc_g, target_empty_weight_g, raw_flow_drip_gps);
            remaining_time_filt_drip_s = calculate_specific_remaining_time(remaining_weight_drip_calc_g, target_empty_weight_g, flow_drip_gps);

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
            } else if (fused_remaining_weight_g <= target_empty_weight_g + 1.0f && fabsf(fused_flow_rate_gps) < 0.001f){
                g_oled_remaining_time_min = 0; 
            } else {
                g_oled_remaining_time_min = -1; 
            }
            updateOledDisplay(); 
            
            char serial_buf_debug[450]; 
            snprintf(serial_buf_debug, sizeof(serial_buf_debug),
                     "%.2f,%.2f,%.2f,%.4f,%.4f,%u,%.2f,%.2f,%.4f,%.4f,%.4f,%d,%.2f,%lu,%.5f,%.4f,%.2f,%.0f,%.0f,%.0f,%.0f",
                     current_time_ms / 1000.0f,raw_weight_g,filt_weight_g,raw_flow_weight_gps,flow_weight_gps,
                     drops_this_drip_cycle,raw_drip_rate_dps,filt_drip_rate_dps,raw_flow_drip_gps,flow_drip_gps,
                     remaining_weight_drip_calc_g,drip_kf.isWpdCalibrating() ? 1 : 0,drip_initial_weight_val,
                     drip_total_drops_val,wpd_cumulative_val,fused_flow_rate_gps,fused_remaining_weight_g,remaining_time_s,
                     remaining_time_raw_weight_s,remaining_time_filt_weight_s,remaining_time_raw_drip_s);
            Serial.println(serial_buf_debug);

            if (wifi_connected_flag && ws_client_connected_flag) {
                float ws_progress_percent_to_send = -1.0f; 
                if (system_initial_weight_set) {
                    ws_progress_percent_to_send = g_infusion_progress * 100.0f; 
                }
                char serial_buf_ws[450];
                snprintf(serial_buf_ws, sizeof(serial_buf_ws), 
                         "%lu,%.2f,%.2f,%.4f,%.4f,%u,%.2f,%.2f,%.4f,%.4f,%.4f,%d,%d,%.2f,%.4f,%.2f,%.0f,%.0f,%.0f,%.0f,%.0f,%lu,%.2f,%.5f,%.1f",
                         current_time_ms,raw_weight_g,filt_weight_g,raw_flow_weight_gps,flow_weight_gps,
                         drops_this_drip_cycle,raw_drip_rate_dps,filt_drip_rate_dps,raw_flow_drip_gps,flow_drip_gps,
                         drip_kf.getCalibratedWeightPerDrop(),drip_kf.isWpdCalibrating() ? 1 : 0,wpd_long_cal_active ? 1 : 0,
                         remaining_weight_drip_calc_g,fused_flow_rate_gps,fused_remaining_weight_g,
                         remaining_time_raw_weight_s,remaining_time_filt_weight_s,remaining_time_raw_drip_s,remaining_time_filt_drip_s,
                         remaining_time_s,drip_total_drops_val,drip_initial_weight_val,wpd_cumulative_val,ws_progress_percent_to_send);
                ws_server.broadcastTXT(serial_buf_ws); 
            }
            // --- END OF EXISTING MAIN PROCESSING LOGIC ---
            
            // --- NeoPixel LED Update for Normal Operation (blinking) ---
            uint32_t base_color_for_blink;
            if (system_initial_weight_set && g_infusion_progress >= 0.9f) { // Ensure progress is valid before using it
                base_color_for_blink = NEO_COLOR_YELLOW;
            } else if (drip_kf.isWpdCalibrating() || wpd_long_cal_active) {
                base_color_for_blink = NEO_COLOR_BLUE;
            } else {
                base_color_for_blink = NEO_COLOR_GREEN;
            }

            neo_led_state_is_on = !neo_led_state_is_on; // Toggle state each MAIN_LOOP_INTERVAL
            if (neo_led_state_is_on) {
                pixels.setPixelColor(0, base_color_for_blink);
            } else {
                pixels.setPixelColor(0, NEO_COLOR_OFF); // Off part of the blink
            }
            pixels.show();
            // --- End NeoPixel LED Update ---

        } // End of if (dt_main_loop_s >= (MAIN_LOOP_INTERVAL_MS / 1000.0f))
    } // End of else (normal operation)
} 