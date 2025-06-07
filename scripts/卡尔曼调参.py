import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# ========== 1. 读取数据 ==========
# 读取data/collected_infusion_data目录下的所有csv文件
import glob
import os

csv_files = glob.glob('data/collected_infusion_data/*.csv')
csv_files.sort()  # 按文件名排序

# 打印所有可用的csv文件
print("可用的数据文件:")
for i, f in enumerate(csv_files):
    print(f"{i}: {os.path.basename(f)}")

# 选择要读取第几个文件
file_index = 0  # 可以修改这个索引来选择不同的文件
df = pd.read_csv(csv_files[file_index])
print(f"\n已选择文件: {os.path.basename(csv_files[file_index])}")

timestamp = df['timestamp_ms'].values / 1000.0
# 从CSV文件中提取原始数据
raw_weight = df['raw_w_g'].values            # 原始重量数据 (g)
raw_drip_rate = df['raw_dps'].values         # 原始滴速数据 (drops/second)
raw_flow_weight_gps = df['raw_flow_w_gps'].values  # 基于重量传感器计算的原始流速 (g/s)
raw_flow_drip_gps = df['raw_flow_d_gps'].values    # 基于滴速传感器计算的原始流速 (g/s)
rem_weight_drip = df['rem_weight_drip_g'].values   # 基于滴速计算的剩余重量估计 (g)

# 新增字段
if 'drip_total_drops' in df.columns:
    drip_total_drops = df['drip_total_drops'].values
else:
    drip_total_drops = df['total_drops_for_volume_calc'].values
if 'drip_initial_weight' in df.columns:
    drip_initial_weight = df['drip_initial_weight'].values
else:
    drip_initial_weight = df['known_initial_total_weight_g'].values

dt = np.diff(timestamp, prepend=timestamp[0])

# ========== 2. 固定参数 ==========
# 卡尔曼滤波器参数
WPD_INIT = 0.05  # 初始每滴重量(Weight Per Drop)，单位：g/滴
# 重量卡尔曼滤波器参数
weight_sigma_a = 0.0005  # 重量过程噪声标准差，表示重量变化的不确定性
weight_sigma_j = 1e-7  # 重量加速度过程噪声标准差，表示重量加速度的不确定性
weight_R = 50.0         # 重量测量噪声方差，表示重量传感器的噪声水平

# 滴速卡尔曼滤波器参数
drip_sigma_a = 1e-5    # 滴速过程噪声标准差，表示滴速变化的不确定性
drip_R = 0.05           # 滴速测量噪声方差，表示滴速传感器的噪声水平

# 流速融合卡尔曼滤波器参数
q_flow = 1e-7        # 流速过程噪声方差，表示流速变化的不确定性
r_weight_flow = 0.01 # 基于重量计算的流速测量噪声方差
r_drip_flow = 0.0005   # 基于滴速计算的流速测量噪声方差

# 剩余重量融合卡尔曼滤波器参数
q_weight = 0.01        # 剩余重量过程噪声方差，表示重量变化的不确定性
r_weight_weight = 1.0  # 直接测量的重量噪声方差
r_drip_weight = 1.0    # 基于滴速计算的重量噪声方差

# WPD卡尔曼滤波器参数
wpd_Q = 1e-8  # WPD过程噪声方差
wpd_R = 1e-4  # WPD测量噪声方差
wpd_x0 = 0.05 # WPD初始值

# ========== 3. WeightKalmanFilter ==========
def run_weight_kf(raw_weight, dt, sigma_a, sigma_j, R):
    x = np.zeros((3, len(raw_weight)))  # [位置,速度,加速度]
    P = np.zeros((3, 3, len(raw_weight)))
    x[:, 0] = [raw_weight[0], 0, 0]  # 初始状态
    P[:, :, 0] = np.eye(3) * 10
    
    for k in range(1, len(raw_weight)):
        # 状态转移矩阵
        F = np.array([[1, dt[k], dt[k]**2/2],
                     [0, 1, dt[k]],
                     [0, 0, 1]])
        
        # 过程噪声协方差矩阵
        Q = np.array([[sigma_a**2 * dt[k]**4/4, sigma_a**2 * dt[k]**3/2, sigma_a**2 * dt[k]**2/2],
                      [sigma_a**2 * dt[k]**3/2, sigma_a**2 * dt[k]**2, sigma_a**2 * dt[k]],
                      [sigma_a**2 * dt[k]**2/2, sigma_a**2 * dt[k], sigma_j**2]])
        
        # 预测
        x_pred = F @ x[:, k-1]
        P_pred = F @ P[:, :, k-1] @ F.T + Q
        
        # 更新
        H = np.array([[1, 0, 0]])  # 观测矩阵
        z = np.array([raw_weight[k]])
        y = z - H @ x_pred
        S = H @ P_pred @ H.T + R
        K = P_pred @ H.T / S
        x[:, k] = x_pred + (K.flatten() * y)
        P[:, :, k] = (np.eye(3) - K @ H) @ P_pred
        
    return x[0], x[1], x[2]  # 重量, 速度, 加速度

# ========== 4. DripKalmanFilter ==========
def run_drip_kf(raw_drip_rate, dt, sigma_a, R):
    x = np.zeros((2, len(raw_drip_rate)))
    P = np.zeros((2, 2, len(raw_drip_rate)))
    x[:, 0] = [raw_drip_rate[0], 0]
    P[:, :, 0] = np.eye(2) * 1
    for k in range(1, len(raw_drip_rate)):
        F = np.array([[1, dt[k]], [0, 1]])
        Q = sigma_a**2 * np.array([[dt[k]**4/4, dt[k]**3/2], [dt[k]**3/2, dt[k]**2]])
        x_pred = F @ x[:, k-1]
        P_pred = F @ P[:, :, k-1] @ F.T + Q
        H = np.array([[1, 0]])
        z = np.array([raw_drip_rate[k]])
        y = z - H @ x_pred
        S = H @ P_pred @ H.T + R
        K = P_pred @ H.T / S
        x[:, k] = x_pred + (K.flatten() * y)
        P[:, :, k] = (np.eye(2) - K @ H) @ P_pred
    return x[0], x[1]  # 滴速, 滴速变化率

def kalman_1d(measurements, Q=1e-6, R=1e-3, x0=0.05):
    x = np.zeros_like(measurements)
    P = 1.0
    for k in range(len(measurements)):
        if np.isnan(measurements[k]) or measurements[k] < 0.04 or measurements[k] > 0.07:
            if k > 0:
                x[k] = x[k-1]
            else:
                x[k] = x0
            continue
        P = P + Q
        K = P / (P + R)
        if k == 0:
            x[k] = measurements[k]
        else:
            x[k] = x[k-1] + K * (measurements[k] - x[k-1])
        P = (1 - K) * P
        # 限制wpd在0.04-0.07之间
        x[k] = np.clip(x[k], 0.0, 0.07)
    return x

# ========== 5. DataFusion ==========
def run_data_fusion(flow_weight, flow_drip, rem_weight_weight, rem_weight_drip, dt, 
                    q_flow, r_weight_flow, r_drip_flow, q_weight, r_weight_weight, r_drip_weight):
    n = len(flow_weight)
    fused_flow = np.zeros(n)
    fused_weight = np.zeros(n)
    P_flow = 1.0
    P_weight = 10.0
    fused_flow[0] = flow_weight[0]
    fused_weight[0] = rem_weight_weight[0]
    for k in range(1, n):
        # 预测
        P_flow += q_flow * dt[k]
        P_weight += q_weight * dt[k]
        fused_weight[k] = fused_weight[k-1] - fused_flow[k-1] * dt[k]
        if fused_weight[k] < 0: fused_weight[k] = 0
        # 流速更新
        K1 = P_flow / (P_flow + r_weight_flow)
        fused_flow[k] = fused_flow[k-1] + K1 * (flow_weight[k] - fused_flow[k-1])
        P_flow = (1 - K1) * P_flow
        K2 = P_flow / (P_flow + r_drip_flow)
        fused_flow[k] = fused_flow[k] + K2 * (flow_drip[k] - fused_flow[k])
        P_flow = (1 - K2) * P_flow
        # 剩余重量更新
        K3 = P_weight / (P_weight + r_weight_weight)
        fused_weight[k] = fused_weight[k] + K3 * (rem_weight_weight[k] - fused_weight[k])
        P_weight = (1 - K3) * P_weight
        K4 = P_weight / (P_weight + r_drip_weight)
        fused_weight[k] = fused_weight[k] + K4 * (rem_weight_drip[k] - fused_weight[k])
        P_weight = (1 - K4) * P_weight
        if fused_weight[k] < 0: fused_weight[k] = 0
    return fused_flow, fused_weight

# ========== 6. 剩余时间预测与理想线性剩余时间 ==========
def calc_remaining_time(weight, flow, target_weight):
    rem = weight - target_weight
    rem[rem < 0] = 0
    flow[flow < 1e-5] = 1e-5  # 防止除零
    remaining_time = rem / flow
    # 限制剩余时间范围在0-10000秒之间
    remaining_time[remaining_time > 10000] = 10000
    # 计算进度
    progress = 1 - rem / (weight[0] - target_weight)
    progress[progress < 0] = 0
    progress[progress > 1] = 1
    # # 应用系数
    # coef = 11.5 * (1 - progress)
    # remaining_time = remaining_time * (1 + coef)**0.16
    coef = (1 - progress)*11.5
    remaining_time  = remaining_time * (1 + coef)**0.16
    return remaining_time

def calc_ideal_remaining_time(timestamp, fused_weight, target_weight):
    end_idx = np.where(fused_weight <= target_weight + 0.1)[0]
    if len(end_idx) == 0:
        end_idx = [-1]
    end_time = timestamp[end_idx[0]]
    ideal_time = end_time - timestamp
    ideal_time[ideal_time < 0] = 0
    return ideal_time

# ========== 7. 主流程 ==========
if __name__ == '__main__':
    # 设置要显示的数据点数量
    n_points = 200
    # 计算步长
    step = max(len(timestamp) // n_points, 1)
    # 创建索引数组
    plot_idx = np.arange(0, len(timestamp), step)

    # 1. 重量卡尔曼滤波
    filt_weight, filt_weight_vel, filt_weight_acc = run_weight_kf(raw_weight, dt, weight_sigma_a, weight_sigma_j, weight_R)
    # 设置输液结束阈值为最后一个点的滤波后重量
    target_empty_weight = filt_weight[-1]
    # 2. WPD累计法与卡尔曼滤波
    wpd_cumulative = np.full_like(raw_weight, np.nan)
    for k in range(1, len(raw_weight)):
        drops_diff = drip_total_drops[k] - drip_total_drops[0]
        if drops_diff > 0:
            wpd_cumulative[k] = (raw_weight[0] - raw_weight[k]) / drops_diff
    wpd_kf = kalman_1d(wpd_cumulative, wpd_Q, wpd_R, wpd_x0)
    # 3. 滴速卡尔曼滤波（用滤波WPD）
    filt_drip_rate, _ = run_drip_kf(raw_drip_rate, dt, drip_sigma_a, drip_R)
    filt_drip_flow_gps = filt_drip_rate * wpd_kf
    # 4. 滴数法剩余重量估算
    drip_est_weight = drip_initial_weight - drip_total_drops * wpd_kf
    # 5. 数据融合
    fused_flow, fused_weight = run_data_fusion(-filt_weight_vel, filt_drip_flow_gps, 
                                              filt_weight, drip_est_weight, dt,
                                              q_flow, r_weight_flow, r_drip_flow, 
                                              q_weight, r_weight_weight, r_drip_weight)
    
    # 6. 计算各种剩余时间
    weight_rem_time = calc_remaining_time(filt_weight, -filt_weight_vel, target_empty_weight)
    drip_rem_time = calc_remaining_time(drip_est_weight, filt_drip_flow_gps, target_empty_weight)
    fused_rem_time = calc_remaining_time(fused_weight, fused_flow, target_empty_weight)
    ideal_rem_time = calc_ideal_remaining_time(timestamp, filt_weight, target_empty_weight)
    # 7. 可视化（剔除异常值）
    # 定义异常值过滤函数
    def filter_outliers(data, threshold=3):
        """
        使用z-score方法剔除异常值
        threshold: z-score阈值，默认为3
        """
        if np.all(np.isnan(data)):
            return data
        
        # 创建掩码，初始所有点都是有效的
        mask = ~np.isnan(data)
        
        # 计算均值和标准差（忽略NaN值）
        mean = np.nanmean(data)
        std = np.nanstd(data)
        
        if std == 0:  # 防止除零错误
            return data
        
        # 计算z-scores
        z_scores = np.abs((data - mean) / std)
        
        # 标记异常值
        mask = mask & (z_scores < threshold)
        
        # 创建过滤后的数据副本
        filtered_data = data.copy()
        filtered_data[~mask] = np.nan
        
        return filtered_data, mask
    
     # 图1：剩余时间对比（剔除异常值）
    plt.figure(figsize=(12, 6))
    # 过滤异常值
    weight_rem_time_filtered, _ = filter_outliers(weight_rem_time)
    drip_rem_time_filtered, _ = filter_outliers(drip_rem_time)
    fused_rem_time_filtered, _ = filter_outliers(fused_rem_time)
    
    plt.plot(timestamp[plot_idx], weight_rem_time_filtered[plot_idx], '-', color='lightblue', markersize=4, label='Weight Sensor Remaining Time')
    plt.plot(timestamp[plot_idx], drip_rem_time_filtered[plot_idx], '-', color='lightgreen', markersize=4, label='Drip Sensor Remaining Time')
    plt.plot(timestamp[plot_idx], fused_rem_time_filtered[plot_idx], '-', color='red', label='Fused Remaining Time')
    plt.plot(timestamp[plot_idx], ideal_rem_time[plot_idx], '--', color='purple', label='Ideal Remaining Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Remaining Time (s)')
    plt.legend()
    plt.title('Comparison of Remaining Time Prediction Methods')
    plt.grid(True)
    
    # 设置横纵轴相同宽度
    ax = plt.gca()
    ax.set_aspect('equal')
    
    # 获取当前轴的范围
    x_min, x_max = ax.get_xlim()
    y_min, y_max = ax.get_ylim()
    
    # 取横纵轴范围的交集
    plot_min = max(x_min, y_min)
    plot_max = min(x_max, y_max)
    
    # 设置相同的轴范围
    ax.set_xlim(plot_min, plot_max)
    ax.set_ylim(plot_min, plot_max)
    
    plt.savefig('data/fig/1.剩余时间对比.png', dpi=300, bbox_inches='tight')
    plt.close()

    # 计算误差
    weight_error = weight_rem_time_filtered - ideal_rem_time
    drip_error = drip_rem_time_filtered - ideal_rem_time  
    fused_error = fused_rem_time_filtered - ideal_rem_time

    # 计算后50%数据的MAE
    data_len = len(weight_error)
    start_idx = int(data_len * 0.5)  # 从50%处开始计算
    
    weight_mae = np.nanmean(np.abs(weight_error[start_idx:]))
    drip_mae = np.nanmean(np.abs(drip_error[start_idx:]))
    fused_mae = np.nanmean(np.abs(fused_error[start_idx:]))

    # 图1.1：误差对比
    plt.figure(figsize=(12, 6))
    plt.plot(timestamp[plot_idx], weight_error[plot_idx], '-', color='lightblue', markersize=4, 
             label=f'Weight Sensor Error (Last 50% MAE: {weight_mae:.2f}s)')
    plt.plot(timestamp[plot_idx], drip_error[plot_idx], '-', color='lightgreen', markersize=4, 
             label=f'Drip Sensor Error (Last 50% MAE: {drip_mae:.2f}s)')
    plt.plot(timestamp[plot_idx], fused_error[plot_idx], '-', color='red', 
             label=f'Fused Error (Last 50% MAE: {fused_mae:.2f}s)')
    plt.axhline(y=0, color='black', linestyle='--', label='Zero Error')
    plt.axvline(x=timestamp[start_idx], color='gray', linestyle='--', label='50% Mark')
    plt.xlabel('Time (s)')
    plt.ylabel('Error (s)')
    plt.legend()
    plt.title('Error Analysis of Remaining Time Predictions (Last 50% MAE)')
    plt.grid(True)
    plt.savefig('data/fig/1.1.剩余时间误差.png', dpi=300, bbox_inches='tight')
    plt.close()
    
    # 图2：WPD对比（剔除异常值）
    plt.figure(figsize=(12, 6))
    # 过滤异常值
    wpd_cumulative_filtered, _ = filter_outliers(wpd_cumulative)
    
    plt.plot(timestamp[plot_idx], wpd_cumulative_filtered[plot_idx], '.', color='lightcoral', markersize=2, label='Raw WPD (Cumulative)')
    plt.plot(timestamp[plot_idx], wpd_kf[plot_idx], '-', color='red', label='Filtered WPD (Kalman)')
    plt.xlabel('Time (s)')
    plt.ylabel('Weight Per Drop (g/drop)')
    plt.legend()
    plt.ylim(0.03, 0.07)
    plt.title('Comparison of Raw and Filtered WPD')
    plt.grid(True)
    plt.savefig('data/fig/2.WPD对比.png', dpi=300, bbox_inches='tight')
    plt.close()
    
    # 图3：重量对比
    plt.figure(figsize=(12, 6))
    # 过滤异常值
    raw_weight_filtered, _ = filter_outliers(raw_weight)
    
    plt.plot(timestamp, raw_weight_filtered, '-', color='lightblue', markersize=4, label='Raw Weight')
    plt.plot(timestamp, filt_weight, '-', color='blue', label='Filtered Weight')
    plt.plot(timestamp, drip_est_weight, '-', color='green', label='Drip Estimated Weight')
    plt.plot(timestamp, fused_weight, '-', color='red', label='Fused Weight')
    plt.axhline(target_empty_weight, color='black', linestyle='--', label='Target Empty Weight')
    plt.xlabel('Time (s)')
    plt.ylabel('Weight (g)')
    # plt.xlim(200, 1500)
    # plt.ylim(480, 580)
    plt.legend()
    plt.title('Comparison of Weight Measurement Methods')
    plt.grid(True)
    plt.savefig('data/fig/3.重量对比.png', dpi=300, bbox_inches='tight')
    plt.close()
    
    # 图4：流速对比
    plt.figure(figsize=(12, 6))
    # 过滤异常值
    raw_flow_weight_gps_filtered, _ = filter_outliers(raw_flow_weight_gps)
    filt_weight_vel_filtered, _ = filter_outliers(-filt_weight_vel)
    raw_flow_drip_gps_filtered, _ = filter_outliers(raw_flow_drip_gps) 
    filt_drip_flow_gps_filtered, _ = filter_outliers(filt_drip_flow_gps)
    
    # 计算融合流速的范围
    fused_flow_valid = fused_flow[~np.isnan(fused_flow)]
    
    if len(fused_flow_valid) > 0:
        flow_min = np.min(fused_flow_valid)
        flow_max = np.max(fused_flow_valid)
        y_range = flow_max - flow_min
        
        # 扩大y轴范围10%
        y_min = max(0, flow_min - y_range * 0.1)  # 不小于0
        y_max = flow_max + y_range * 0.1
        
        plt.plot(timestamp[plot_idx], raw_flow_weight_gps_filtered[plot_idx], '.', color='lightblue', markersize=3, label='Raw Flow Rate (Weight)')
        plt.plot(timestamp[plot_idx], filt_weight_vel_filtered[plot_idx], '-', color='blue', label='Filtered Flow Rate (Weight)')
        plt.plot(timestamp[plot_idx], raw_flow_drip_gps_filtered[plot_idx], '.', color='lightgreen', markersize=3, label='Raw Flow Rate (Drip)')
        plt.plot(timestamp[plot_idx], filt_drip_flow_gps_filtered[plot_idx], '-', color='green', label='Filtered Flow Rate (Drip)')
        plt.plot(timestamp[plot_idx], fused_flow[plot_idx], '-', color='red', label='Fused Flow Rate')
        plt.xlabel('Time (s)')
        plt.ylabel('Flow Rate (g/s)')
        plt.ylim(y_min, y_max)
        plt.legend()
        plt.title('Comparison of Flow Rate Measurement Methods')
        plt.grid(True)
        plt.savefig('data/fig/4.流速对比.png', dpi=300, bbox_inches='tight')
        plt.close()
    else:
        print("警告: 所有流速数据均为NaN,无法绘图")
