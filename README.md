# 智能输液监护系统 (Smart Infusion PIO)

这是一个基于 PlatformIO 开发的智能输液监护系统项目。该系统通过高精度传感器实时监测输液过程，利用卡尔曼滤波和数据融合算法提供准确的流速和剩余量预测，并通过多种方式（OLED屏幕、Web界面、云端API）进行数据显示和异常报警。

## 主要功能

*   **多传感器数据采集**:
    *   **重量传感器 (HX711)**: 实时测量输液袋的重量，精确到克。
    *   **滴速传感器**: 通过光电中断方式检测液滴滴落，精确记录每一次滴落事件。
*   **高级数据处理**:
    *   **卡尔曼滤波**: 分别对重量数据和滴速数据进行滤波，去除噪声，获得平滑、稳定的流速估算。
    *   **数据融合**: 融合重量传感器和滴速传感器的估算结果，提供更精确、更可靠的流速和剩余液体重量。
*   **实时状态监控**:
    *   **本地显示 (OLED)**: 在OLED屏幕上实时显示IP地址、剩余重量(g)、融合流速(g/s)和预计剩余时间(min)。
    *   **Web界面**: 提供一个基于 WebSocket 的实时 Web 监控界面，展示详细的原始数据、滤波后数据、融合数据和系统状态。
    *   **LED 状态指示**: 使用 NeoPixel RGB LED 通过不同颜色和闪烁模式指示系统当前状态（如：正常、异常、初始化、完成等）。
*   **智能告警与控制**:
    *   **输液异常检测**: 能自动检测滴速过慢或停止等异常情况，并触发报警。
    *   **完成提醒**: 在输液接近完成时发出提醒。
    *   **远程数据上报**: 定期将设备状态和输液数据通过 HTTP POST 请求上报到云端服务器。
    *   **自动夹断 (预留)**: 在检测到异常或输液完成时，可控制执行器（如舵机）夹断输液管（`auto_clamp`标志位）。
*   **系统模式**:
    *   **正常模式**: 稳定输液监控。
    *   **快速收敛模式**: 在输液开始时，采用更激进的滤波器参数，使系统读数能更快地收敛到真实值。
    *   **WPD 校准模式**: 支持通过 Web 界面启动和停止"克/每滴"(Weight-Per-Drop)的在线校准，以适应不同粘稠度的液体。
*   **用户交互**:
    *   **物理按键**: 提供"初始化/复位"和"异常清除"两个物理按键，方便现场操作。
    *   **Web指令**: 可通过Web界面发送指令，如启动/停止WPD校准、设置液体总量等。

## 硬件清单

| 组件             | 型号/规格                      | 连接引脚 (ESP32)               |
| ---------------- | ------------------------------ | ------------------------------ |
| 微控制器         | ESP32-S3 或类似开发板          | -                              |
| 显示屏           | 0.96寸 I2C OLED (SSD1306)      | SDA: `36`, SCL: `1`            |
| 重量传感器模块   | HX711 模块 + 压力传感器        | DT: `17`, SCK: `18`            |
| 滴速传感器       | 对射式红外传感器或类似装置     | DATA: `11` (中断引脚)          |
| 状态指示灯       | NeoPixel RGB LED (WS2812B)     | DATA: `48`                     |
| 物理按键         | 瞬时轻触开关 x2                | 初始化: `15`, 异常复位: `0`    |

## 软件与库

*   **开发环境**: [PlatformIO IDE](https://platformio.org/)
*   **框架**: Arduino
*   **主要库**:
    *   `WiFi`, `WiFiClient`, `HTTPClient`: 用于WiFi连接和网络通信。
    *   `WebSocketsServer`: 用于实现与Web界面的实时双向通信。
    *   `U8g2lib`: 强大的OLED显示库。
    *   `Adafruit_NeoPixel`: NeoPixel LED驱动库。
    *   `HX711`: HX711称重传感器库。
    *   `ArduinoJson`: 高效的JSON序列化/反序列化库，用于API数据交互。

## 项目结构

```
Smart_infusion_PIO/
├── platformio.ini         # PlatformIO 配置文件 (板子定义, 库依赖)
├── include/               # 自定义库头文件 (如.h)
│   ├── DataFusion.h
│   ├── DripKalmanFilter.h
│   └── WeightKalmanFilter.h
│   └── webpage.h          # 嵌入式网页HTML代码
├── src/                   # 源代码
│   └── main.cpp           # 主程序逻辑
├── lib/                   # 项目私有库
├── test/                  # 测试代码
└── ...
```

## 工作原理

系统的工作流程如下：

1.  **初始化**:
    *   系统启动，初始化硬件（OLED, NeoPixel, HX711）。
    *   连接到指定的 WiFi 网络，并启动 WebSocket 和 HTTP 服务器。
    *   执行一次`performSystemReinitialization()`，读取输液袋的初始重量，并将其作为基准。
    *   进入为时60秒的 **快速收敛模式**，以快速获得稳定的初始读数。

2.  **数据采集与处理**:
    *   **重量数据**: `handleWeightSensor()`函数在主循环中定期从 HX711 读取重量数据。原始数据首先经过异常值检测，然后送入 `WeightKalmanFilter` 进行滤波，输出平滑后的重量 `filt_weight_g` 和由重量变化计算出的流速 `flow_weight_gps`。
    *   **滴速数据**: `onWaterDropIsr()`中断服务函数捕捉每一次滴落，将时间戳存入一个循环队列。`handleDripRate()`函数从队列中取出时间戳，计算瞬时滴速 `raw_drip_rate_dps`，然后送入 `DripKalmanFilter` 进行滤波，得到平滑后的滴速 `filt_drip_rate_dps` 和基于WPD估算的流速 `flow_drip_gps`。

3.  **数据融合**:
    *   `handleDataFusion()`函数接收来自两个卡尔曼滤波器的流速和重量估算值。
    *   `DataFusion` 模块（一个扩展卡尔曼滤波器）将这两个来源的数据进行融合，得到最终的融合流速 `fused_flow_rate_gps` 和融合剩余重量 `fused_remaining_weight_g`。这提供了比单一数据源更准确和鲁棒的结果。

4.  **状态更新与输出**:
    *   根据融合后的数据计算预计剩余时间 `remaining_time_s`。
    *   通过 `updateDisplay()` 更新OLED屏幕上的显示内容。
    *   通过 `updateLedStatus()` 更新LED状态灯的颜色。
    *   `outputData()` 函数将详细的调试数据打印到串口，并通过WebSocket广播到所有连接的Web客户端。
    *   `uploadDataToServer()` 函数每10秒将关键数据（总容量、剩余容量、流速、剩余时间、系统状态等）打包成JSON格式，通过HTTP POST请求发送到指定的云端API。

5.  **异常处理**:
    *   `checkInfusionAbnormality()` 函数在 **正常模式** 下持续监控，如果在 `NO_DRIP_TIMEOUT_MS` (默认10秒) 内没有检测到滴落，系统状态会切换到 `INFUSION_ERROR`，LED变为红色闪烁，并立即上报异常状态。

## API与通信协议

### WebSocket

*   **端口**: 81
*   **服务器 -> 客户端**: 服务器以CSV格式的字符串形式，高频（约每秒一次）广播实时数据。格式如下：
    ```
    currentTime,rawWeight,filtWeight,rawFlowW,filtFlowW,drips,rawDps,filtDps,rawFlowD,filtFlowD,wpd,wpdCal,wpdLongCal,remWeightD,fusedFlow,fusedWeight,remTimeRawW,remTimeFiltW,remTimeRawD,remTimeFiltD,remTime,totalDrops,initWeight,wpd,progress,state
    ```
*   **客户端 -> 服务器**: 客户端可以发送以下字符串命令：
    *   `CALIBRATE_WPD_START`: 启动WPD长时校准。
    *   `CALIBRATE_WPD_STOP`: 手动停止WPD长时校准。
    *   `SET_TOTAL_VOLUME:<volume>`: 设置液体总量（例如 `SET_TOTAL_VOLUME:500`）。

### HTTP API (数据上报)

*   **方法**: `POST`
*   **URL**: `http://114.66.55.73:31504/api/patients`
*   **Content-Type**: `application/json`
*   **请求体 (Body)**:
    ```json
    {
      "deviceId": "001",
      "totalVolume": 500,
      "remainingVolume": 250,
      "currentRate": 60,
      "estimatedTime": 25,
      "systemState": "NORMAL",
      "autoClamp": 0
    }
    ```
    *   `totalVolume`: 总容量 (ml)
    *   `remainingVolume`: 剩余容量 (ml)
    *   `currentRate`: 当前滴速 (滴/分钟)
    *   `estimatedTime`: 预计剩余时间 (分钟)
    *   `systemState`: 系统状态 (字符串)
    *   `autoClamp`: 夹断状态 (0: 未夹断, 1: 已夹断)

## 安装与部署

1.  **安装 Visual Studio Code** 和 **PlatformIO IDE 扩展**。
2.  克隆本项目到本地。
3.  使用 VS Code 打开项目文件夹。
4.  修改 `src/main.cpp` 中的WiFi配置：
    ```cpp
    constexpr const char* WIFI_SSID  = "your_wifi_ssid";
    constexpr const char* WIFI_PASS  = "your_wifi_password";
    ```
5.  修改 `platformio.ini` 文件，确保 `board` 配置与你的ESP32开发板型号匹配。
6.  连接硬件设备。
7.  点击 PlatformIO 工具栏上的 **Upload** 按钮编译并上传固件。
8.  点击 **Monitor** 按钮打开串口监视器查看设备输出。

## 使用说明

1.  **准备**: 将输液袋挂在称重传感器上。
2.  **开机**: 接通电源，系统开始初始化，LED灯将显示黄色。
3.  **初始化**:
    *   WiFi连接成功后，系统会自动执行一次初始化，读取当前重量作为初始重量，并进入60秒的蓝色灯 **快速收敛模式**。
    *   也可以在任何时候按下 **初始化按键** 手动触发此过程。
4.  **监控**:
    *   输液开始后，系统进入绿色灯 **正常模式**。
    *   通过OLED屏幕或连接到设备IP地址的Web界面查看实时数据。
5.  **异常处理**:
    *   如果发生堵管等异常，LED将变为红色闪烁。
    *   排除故障后，按一下 **异常复位按键**，系统将恢复到 **正常模式**。
6.  **完成**:
    *   输液完成后，系统进入 **已完成** 状态，LED变为白色（或根据`updateLedStatus`中的配置）。
    *   若要开始新的输液，请按下 **初始化按键**。 