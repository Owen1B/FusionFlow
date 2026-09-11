# FusionFlow · 双传感器输液监测

[English](README.en.md) · [算法与系统设计](docs/architecture.md) · [构建说明](docs/build.md)

基于 ESP32-S3 的个人嵌入式项目，结合称重与光学滴液信号，研究流速、剩余液体质量和单滴质量的在线估计。核心算法使用 C++ 实现，包括三状态重量滤波、两状态滴速滤波、单滴质量标定，以及流速和剩余质量的序贯融合。

项目包含设备端代码、OLED 与 Web 监测接口、React 展示页面和实验分析材料。作者：郑皓文（Owen）。适用范围为传感器算法实验与台架演示。

## 项目演示

<p align="center">
  <img src="docs/images/web_interface.png" alt="FusionFlow Web 监测界面" width="860">
</p>

<p align="center">Web 监测界面：集中展示设备状态、液量、滴速和剩余时间。</p>

### 实机视频

https://github.com/user-attachments/assets/89d09ea9-4cbe-42cc-89d2-e86c948f0297

[查看仓库内的视频文件](docs/images/demo.mp4)

## 技术栈

| 层级 | 技术与用途 |
| --- | --- |
| 嵌入式开发 | C++、ESP32-S3、Arduino、PlatformIO；硬件访问、状态管理和传感器处理接口。 |
| 状态估计 | 线性 Kalman 滤波、常加速度状态模型、标量序贯观测更新。 |
| 传感器融合 | HX711 称重、光学滴液计数、单滴质量（WPD）标定、流速与剩余质量融合。 |
| 设备交互 | I2C OLED、GPIO 中断、按键输入、NeoPixel 状态指示。 |
| 数据展示 | ArduinoJson、HTTP/WebSocket 接口；React、Axios 和 Tailwind CSS 前端。 |
| 实验分析 | Python 数据采集与绘图脚本，PlatformIO native / Unity 测试目录。 |

## 算法设计

### 重量序列中的流速估计

称重传感器直接测量液体与容器的质量，而流速对应质量随时间的变化。`WeightKalmanFilter` 使用状态向量 `[m, v, a]`，联合估计质量、质量变化率和变化率的导数。状态预测使用实际采样间隔 `dt`，观测更新由重量读数驱动。

这一设计将微分估计放入状态递推中，便于分析噪声平滑与动态响应之间的取舍。状态矩阵、符号约定和过程噪声的实现见[重量状态估计](docs/architecture.md#重量状态估计)。

### 滴速估计与单滴质量标定

`DripKalmanFilter` 对滴速及其变化率进行两状态估计。单滴质量采用独立标量滤波器，根据累计重量变化与累计滴数构造观测：

$$
z_{\mathrm{WPD}}=\frac{m_0-m_k}{N_k},\qquad q_{\mathrm{drip}}=\hat r_k\hat w_k.
$$

其中 `r` 为滴速（滴/s），`w` 为单滴质量（g/滴），`q` 为质量流率（g/s）。实现包含最小累计滴数、重量变化门槛、观测范围筛选和估计限幅，具体条件见[滴速与标定](docs/architecture.md#滴速与单滴质量标定)。

### 流速与剩余质量的序贯融合

`DataFusion` 分别维护流速和剩余质量两个标量估计。每次先做时间预测，再依次使用称重路径与滴液路径的输出更新状态；剩余质量预测还使用前一时刻的流速估计。

各通道通过测量噪声参数配置权重。称重信号也参与滴重标定，因此两条路径存在信息共享；[融合设计](docs/architecture.md#序贯融合)进一步说明了相关性、协方差传播和参数选择的适用范围。

## 实验图表

仓库保留了流速、重量、单滴质量和剩余时间的对比图。下图为已有流速分析材料；完整图表索引及评估口径见[实验与验证](docs/validation.md)。

![流速对比图](data/fig/4.%E6%B5%81%E9%80%9F%E5%AF%B9%E6%AF%94.png)

<details>
<summary>查看单滴质量对比</summary>

![单滴质量对比图](data/fig/2.WPD%E5%AF%B9%E6%AF%94.png)

</details>

## 运行入口

```sh
git clone https://github.com/Owen1B/FusionFlow.git
cd FusionFlow
```

前端工程位于 `server/frontend/`：

```sh
cd server/frontend
npm ci
HOST=127.0.0.1 npx --no-install react-scripts start
```

页面每 3 秒请求 `/api/patients`，展示数据需要配套 API 服务。设备端使用 `platformio.ini` 中的 `esp32-s3-devkitc-1` 环境。当前源码的硬件管理与处理器接口尚需补齐实现，完整设备构建的准备项见[构建说明](docs/build.md)。

## 代码导航

| 路径 | 内容 |
| --- | --- |
| [`src/WeightKalmanFilter.cpp`](src/WeightKalmanFilter.cpp) | 三状态重量估计及协方差递推。 |
| [`src/DripKalmanFilter.cpp`](src/DripKalmanFilter.cpp) | 滴速滤波、累计滴数计算与 WPD 标定。 |
| [`src/DataFusion.cpp`](src/DataFusion.cpp) | 流速、剩余质量的预测与序贯融合。 |
| [`src/main_refactored.cpp`](src/main_refactored.cpp) | 主循环、输入处理、状态切换与上传接口调用。 |
| [`include/`](include/) | 配置、滤波器和硬件/数据处理接口。 |
| [`server/frontend/`](server/frontend/) | 请求 API 数据的 React 页面。 |
| [`test/`](test/)、[`scripts/`](scripts/) | 滤波器测试、历史代码及数据分析材料。 |

## 项目说明

项目围绕测量、估计和可视化展开。精度、响应延迟和异常处理能力的评估需结合原始数据、参数及台架条件，记录方式见[实验与验证](docs/validation.md)。本项目用于台架研究，禁止用于人体输液控制。

代码许可见 [LICENSE](LICENSE)，依赖与媒体说明见[来源说明](docs/attribution.md)。
