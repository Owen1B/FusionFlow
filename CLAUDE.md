# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概述

这是一个智能输液监护系统，基于 ESP32 和 PlatformIO 开发。系统通过双传感器数据融合（重量传感器 + 滴速传感器）和卡尔曼滤波算法实现精确的输液监控。包含嵌入式设备固件和完整的 Web 应用护士站监控系统。

## 开发命令

### PlatformIO (嵌入式设备)
```bash
# 编译嵌入式固件
pio run

# 编译并上传到 ESP32
pio run -t upload

# 打开串口监视器
pio device monitor

# 编译并运行本地测试
pio test -e native

# 清理构建文件
pio run -t clean
```

### 服务器端 Web 应用
```bash
# 启动完整服务（前后端）
cd server && bash start.sh

# 仅启动后端 (Node.js/Express)
cd server && node App.js

# 仅启动前端开发服务器 (React)
cd server/frontend && npm start

# 构建前端生产版本
cd server/frontend && npm run build

# 安装依赖
cd server && npm install
cd server/frontend && npm install
```

## 核心架构

### 嵌入式系统架构 (src/)
- **main.cpp**: 主程序逻辑，包含状态机、WiFi连接、传感器读取、Web界面服务
- **WeightKalmanFilter.h/cpp**: 重量传感器卡尔曼滤波器，用于平滑重量数据并估算流速
- **DripKalmanFilter.h/cpp**: 滴速传感器卡尔曼滤波器，处理滴速数据和 WPD (Weight-Per-Drop) 校准
- **DataFusion.h/cpp**: 数据融合模块，结合两种传感器的输出获得更精确的流速和重量估计

### 数据流架构
1. **传感器层**: HX711重量传感器 + 红外滴速传感器
2. **滤波层**: 分别对重量和滴速数据进行卡尔曼滤波
3. **融合层**: 扩展卡尔曼滤波器融合两种传感器数据
4. **显示层**: OLED屏幕本地显示 + Web界面远程监控
5. **通信层**: WebSocket实时数据 + HTTP API数据上报

### Web 应用架构 (server/)
- **后端**: Node.js + Express.js + SQLite，提供 RESTful API
- **前端**: React + Tailwind CSS，护士站监控界面
- **API 文档**: server/API.md 包含完整的 API 接口说明

## 重要配置

### 硬件引脚配置 (main.cpp:35-44)
```cpp
constexpr int PIN_WATER_SENSOR   = 11;  // 水滴传感器
constexpr int NEOPIXEL_PIN       = 48;  // NeoPixel LED
constexpr int PIN_I2C_SDA        = 36;  // OLED SDA
constexpr int PIN_I2C_SCL        = 1;   // OLED SCL  
constexpr int PIN_HX711_DT       = 17;  // HX711 数据引脚
constexpr int PIN_HX711_SCK      = 18;  // HX711 时钟引脚
constexpr int PIN_INIT_BUTTON    = 15;  // 初始化按钮
constexpr int MOTOR_PIN1         = 6;   // 电机控制引脚1
constexpr int MOTOR_PIN2         = 7;   // 电机控制引脚2
```

### WiFi 和服务器配置 (main.cpp:28-33)
在部署前需要修改：
- WIFI_SSID 和 WIFI_PASS：WiFi 连接信息
- API_BASE_URL：云端服务器地址

### 卡尔曼滤波器调参
详细参数说明请参考 `docs/kalman_filter_tuning_guide.md`

## 系统状态机

设备运行状态：
- **INITIALIZING** (黄灯): 连接WiFi，获取初始重量
- **FAST_CONVERGENCE** (蓝灯): 输液开始60秒，快速收敛模式
- **NORMAL** (绿灯): 正常输液监控
- **INFUSION_ERROR** (红灯闪烁): 输液异常（堵管、流空等）
- **COMPLETED** (白灯常亮): 输液完成
- **INIT_ERROR** (红灯常亮): 初始化失败

## 测试

### 单元测试 (test/)
使用 Unity 测试框架，测试各个模块：
```bash
# 运行所有测试
pio test -e native

# 运行特定测试
pio test -e native -f test_weight_kf
pio test -e native -f test_drip_kf  
pio test -e native -f test_data_fusion
```

### 数据分析工具 (scripts/)
- `collect_data.py`: 数据采集脚本
- `卡尔曼调参.py/.ipynb`: 参数调优工具

## 调试和监控

### 串口监控
```bash
pio device monitor --baud 115200
```

### Web 界面调试
设备成功连接WiFi后，在浏览器访问设备IP地址查看实时数据和控制界面。

### 日志分析
系统会通过串口输出详细的调试信息，包括传感器读数、滤波器状态、网络连接状态等。

## 部署注意事项

1. **硬件连接**: 确保所有传感器按照引脚配置正确连接
2. **WiFi配置**: 部署前修改 main.cpp 中的 WiFi 信息
3. **服务器部署**: 使用 server/start.sh 脚本启动完整服务
4. **防火墙配置**: 确保服务器端口（默认5000用于后端，3000用于前端开发）可访问

## 故障排除

### 常见问题
1. **WiFi连接失败**: 检查SSID和密码配置，确认网络可用
2. **传感器读数异常**: 检查硬件连接，查看串口输出确认传感器工作状态  
3. **滤波器不稳定**: 参考 kalman_filter_tuning_guide.md 调整参数
4. **Web界面无法访问**: 确认设备IP地址，检查防火墙设置

### 调试流程
1. 检查串口输出确认系统启动状态
2. 确认WiFi连接和IP地址获取
3. 测试传感器原始读数
4. 检查滤波器和融合算法输出
5. 验证Web服务和API通信