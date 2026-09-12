# 构建与运行说明

项目包含设备端、前端展示和算法测试三个入口。以下命令对应仓库配置；当前环境完成了源码与入口核对，设备编译和端到端联调待实际开发环境执行。

## 获取代码

```sh
git clone https://github.com/Owen1B/FusionFlow.git
cd FusionFlow
```

## 设备端

[`platformio.ini`](../platformio.ini) 默认环境为 `esp32-s3-devkitc-1`，使用 Arduino 框架；串口监视速率为 115200。平台依赖使用 `espressif32`，复现实验时应记录实际安装版本。

[`Config.h.example`](../include/Config.h.example) 提供硬件、网络、滤波器及时间配置。先备份本地 `Config.h`，再按示例逐项修改。网络字段使用自己的测试网络与设备标识；提交前检查配置差异并保留凭据在本地。

| 接口 | 示例配置 |
| --- | --- |
| 光学滴液输入 | GPIO 11，下降沿中断。 |
| HX711 | Data 17，Clock 18。 |
| OLED I2C | SDA 36，SCL 1；显示配置为 128×32。 |
| 初始化 / 复位按键 | GPIO 15 / 0。 |
| 状态指示 | NeoPixel，GPIO 48。 |
| 主循环 / 上传间隔 | 配置 1000 ms / 入口常量 5000 ms。 |

这些值对应当前示例文件，接线与传感器标定以实际装配为准。容器去皮、HX711 标定系数和 WPD 范围需要分别记录。

### 完整构建的准备项

当前 `src/` 包含 `main_refactored.cpp`、`SystemStateManager.cpp` 和三个滤波类实现。`HardwareManager.h` 与 `SensorDataProcessor.h` 声明的构造、初始化、采样和通信方法需要对应实现，才能完成固件链接。`test/original/` 保存了历史入口，可作为代码整理时的参考。

接口实现及配置齐备后，使用以下入口构建；先检查构建输出，再连接台架设备：

```sh
pio run -e esp32-s3-devkitc-1
pio run -e esp32-s3-devkitc-1 --target upload
pio device monitor --baud 115200
```

## 前端

```sh
cd server/frontend
npm ci
HOST=127.0.0.1 npx --no-install react-scripts start
```

以上为本地 POSIX shell 启动方式，直接调用已安装的 `react-scripts` 并绑定回环地址。项目现有 `npm start` 脚本关闭了开发服务器的 host 检查，本地调试使用上述入口。依赖安装与启动待本机执行验证。

前端开发代理在 `package.json` 中配置为 `http://localhost:5000`。前端通过 Axios 请求相对地址 `/api/patients`，刷新间隔 3 秒。它需要配套服务提供设备与展示字段的映射。`server/API.md` 是接口材料；`server/App.js` 为 React 示例组件，`server/package.json` 当前仅提供占位测试命令。

具体接口实现由配套服务补齐后联调。界面中的连接、电量、异常状态等字段应分别对应实际遥测与示例显示，测试时逐项核验。

## 算法测试入口

`platformio.ini` 声明了 native / Unity 环境，`test/` 保存重量滤波、滴速滤波、融合及历史测试目录。测试启动入口为：

```sh
pio test -e native
```

执行前检查 native 源文件选择与 Arduino 依赖的隔离。测试结果应记录命令、编译器、参数与输入序列；当前文档提供入口和核对项，结果以实际运行日志为准。

## 常见定位路径

| 现象 | 核对位置 |
| --- | --- |
| 构造函数或成员方法链接失败 | 对照 `src/` 与两个接口头文件，补齐实现及构建目标。 |
| 前端卡片为空 | `/api/patients` 返回值、相对请求的服务路由及字段映射。 |
| 滴数更新但估计值异常 | 中断累计量、周期新增滴数、WPD 标定调用与时间单位。 |
| 剩余时间波动 | 流速接近零的保护、容器去皮、融合权重与输入相关性。 |

评估项目见[实验与验证](validation.md)。
