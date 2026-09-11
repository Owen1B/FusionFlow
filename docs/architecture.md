# 状态估计与系统设计

本文说明三个滤波类的实际计算，以及设备入口和展示接口的衔接。源码依据为 [`9f71d0e`](https://github.com/Owen1B/FusionFlow/tree/9f71d0e52303fc42a7c8accfcb9fb3e338a3748e)。

## 问题与变量

称重路径提供质量观测，滴液路径提供事件计数。两者在换算为流速时分别依赖时间差分和单滴质量。算法层分别估计这些量，再统一为质量和质量流率。

| 符号 | 含义 | 单位 |
| --- | --- | --- |
| `m` | 质量估计 | g |
| `v` | 质量变化率；质量下降时为负 | g/s |
| `a` | 质量变化率的导数 | g/s² |
| `r` | 滴速 | 滴/s |
| `w` | 单滴质量 WPD | g/滴 |
| `q` | 消耗方向为正的质量流率 | g/s |
| `rho` | 液体密度 | g/mL |
| `dt` | 相邻更新的时间间隔 | s |

从重量变化率转换到正向消耗流率时需采用 `q_weight = -v`，并明确容器去皮与异常值处理。`WeightKalmanFilter::getVelocity()` 返回带符号的变化率，转换职责位于调用方。

## 重量状态估计

[`WeightKalmanFilter.cpp`](../src/WeightKalmanFilter.cpp) 使用三状态线性模型：

$$
x_k=[m_k,v_k,a_k]^T,\quad
F=\begin{bmatrix}1&dt&dt^2/2\\0&1&dt\\0&0&1\end{bmatrix},\quad H=[1,0,0].
$$

每次更新执行：

$$
\hat x^-_k=F\hat x_{k-1},\quad P^-_k=FP_{k-1}F^T+Q,
$$
$$
S_k=P^-_{00}+R,\quad K_k=P^-_{:,0}/S_k,
$$
$$
\hat x_k=\hat x^-_k+K_k(z_k-\hat m^-_k),\quad P_k=(I-K_kH)P^-_k.
$$

只有质量被直接观测，速度和加速度通过状态耦合及交叉协方差更新。状态维数固定为 3，矩阵运算使用固定大小数组，单次更新的存储量与算术量均为常数量级。

### 过程噪声的实际形式

记 `A = sigma_a²`、`J = sigma_j²`，代码构造：

$$
Q=\begin{bmatrix}
A dt^4/4&A dt^3/2&A dt^2/2\\
A dt^3/2&A dt^2&A dt\\
A dt^2/2&A dt&J
\end{bmatrix}.
$$

这是源码中的离散参数化形式。参数评估需要同时检查矩阵的对称性和半正定性；[验证说明](validation.md#数值模型检查)记录了示例配置下需要处理的协方差问题。调整 `R` 与 `Q` 时，应先建立有效的噪声模型，再比较估计响应。

## 滴速与单滴质量标定

### 两状态滴速滤波

[`DripKalmanFilter::update`](../src/DripKalmanFilter.cpp) 接收已计算好的滴速观测和时间间隔。状态为 `[r, dr/dt]`，其状态矩阵和过程噪声为：

$$
F_r=\begin{bmatrix}1&dt\\0&1\end{bmatrix},\quad H_r=[1,0],\quad
Q_r=\sigma_r^2\begin{bmatrix}dt^4/4&dt^3/2\\dt^3/2&dt^2\end{bmatrix}.
$$

事件计数如何变成观测滴速属于上层采样逻辑；滤波类负责递推给定的观测。主入口通过滴液下降沿中断累计事件，并采用 50 ms 去抖门槛。

### 标量 WPD 标定

WPD 由单独的 `calibrateWpdByTotal(current_weight)` 更新。设初始重量为 `m0`，累计滴数为 `N`：

$$
z_w=(m_0-m)/N,\quad P^-_w=P_w+Q_w,
$$
$$
K_w=P^-_w/(P^-_w+R_w),\quad
\hat w\leftarrow\hat w+K_w(z_w-\hat w),\quad P_w\leftarrow(1-K_w)P^-_w.
$$

| 条件 | 代码行为 |
| --- | --- |
| 已进入标定、已设置初始重量 | 接受标定调用。 |
| 累计滴数至少 5，质量变化至少 0.01 g | 开始构造观测。 |
| 观测 WPD 位于 0.01–0.2 g/滴 | 接受本次标量更新。 |
| 更新后的 WPD | 限制在 0.04–0.06 g/滴。 |

这些值是当前实现的门槛。`Config.h.example` 中还存在 30 滴的上层配置；调用层实现补齐时，需要明确两级门槛的关系。`update()` 中的重量变化参数目前未参与 WPD 更新，标定由上述独立函数完成。

滴液路径的质量流率为 `q = r × w`。体积流率换算为 `q / rho × 3600` mL/h。剩余质量采用 `max(m0 - N × w, 0)`；因此 WPD 改变时，累计消耗质量也会按当前 WPD 重新计算。累计观测窗口相互重叠，参数选择需要考虑时间相关性。

## 序贯融合

[`DataFusion.cpp`](../src/DataFusion.cpp) 维护两个标量状态：流速 `q` 和剩余质量 `m`。它们分别保存方差，质量预测使用上一轮流速：

$$
q^-_k=q_{k-1},\quad P^-_q=P_q+Q_qdt,
$$
$$
m^-_k=\max(m_{k-1}-q_{k-1}dt,0),\quad P^-_m=P_m+Q_mdt.
$$

随后对流速、质量各执行两次观测更新：先称重路径，再滴液路径。每次标量更新使用：

$$
K=P/(P+R_i),\quad x\leftarrow x+K(z_i-x),\quad P\leftarrow(1-K)P.
$$

两个滤波状态之间通过质量预测耦合；实现保存两个标量方差。测量噪声 `R_i` 由配置提供。上游估计误差需要通过独立评估来确定对应参数；当前融合接口接收估计值，协方差由融合层单独维护。

### 设计取舍

这种分层结构便于分别调试重量、滴速和标定模块，单次计算规模固定。融合层还需要考虑两点：称重数据参与 WPD 标定，因此两条观测路径存在相关性；剩余质量预测受流速误差影响，当前方差递推以独立 `Q_m` 近似吸收这部分不确定性。评价融合效果时应同时比较单通道误差、交叉相关和响应延迟。

## 设备与展示接口

[`main_refactored.cpp`](../src/main_refactored.cpp) 将工作分配给 `SystemStateManager`、`HardwareManager`、`SensorDataProcessor`。当前 `src/` 中包含状态管理和三个滤波类实现，硬件管理及数据处理类以头文件接口保留，完整实现为固件构建的准备项。

| 链路 | 当前代码接口 |
| --- | --- |
| 滴液输入 | 下降沿中断 → `updateDropCount(1)` → 最近滴液时间更新。 |
| 数据更新 | 主循环时间检查 → 称重读取 → `processSensorData` → OLED 更新。 |
| 上传 | 每 5 秒调用上传与 WebSocket 方法，负载含设备 ID、时间戳、状态、夹断标志和累计滴数。 |
| Web 页面 | `server/frontend/src/App.js` 每 3 秒请求 `/api/patients`。 |

`server/App.js` 是使用模拟记录的独立 React 页面；API 驱动的前端工程位于 `server/frontend/`。设备负载与页面字段之间的映射由配套数据服务完成，详见[构建说明](build.md)。

剩余时间的基本估算为 `T = m / q`，单位为秒。`SensorDataProcessor` 声明了对应接口；其完整实现与低流速保护需要在联调时核对。误差敏感性可由 `dT ≈ dm/q - m·dq/q²` 看出：流速接近零时，流速误差对时间估计的影响增大。
