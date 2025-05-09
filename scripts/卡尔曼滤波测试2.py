import numpy as np
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm

# 尝试设置中文字体，如果失败则忽略
try:
    plt.rcParams['font.sans-serif'] = ['SimHei'] # 指定默认字体
    plt.rcParams['axes.unicode_minus'] = False # 解决保存图像是负号'-'显示为方块的问题
except Exception as e:
    print(f"无法设置中文字体，将使用默认字体: {e}")

class WeightKalmanFilterPython:
    def __init__(self, process_noise=0.1, measurement_noise=1.0):
        self.Q = process_noise  # 过程噪声协方差
        self.R = measurement_noise  # 测量噪声协方差
        self.P = 1.0  # 估计误差协方差
        self.K = 0.0  # 卡尔曼增益
        self.x = 0.0  # 状态估计值（重量）
        self.v = 0.0  # 速度估计值（克/秒）

    def init(self, initial_weight):
        self.x = initial_weight
        self.v = 0.0
        self.P = 1.0

    def update(self, measurement, dt):
        # 预测步骤
        x_pred = self.x + self.v * dt
        P_pred = self.P + self.Q * dt # 在C++中是 P_pred = P + Q * dt

        # 更新步骤
        if (P_pred + self.R) == 0: # 避免除以零
            self.K = 0
        else:
            self.K = P_pred / (P_pred + self.R)
        
        self.x = x_pred + self.K * (measurement - x_pred)
        self.P = (1 - self.K) * P_pred

        # 更新速度估计 (与C++代码保持一致)
        if dt == 0: # 避免除以零
             # 如果dt为0，我们无法从当前步骤更新速度，可以保持v不变或设为0
             # 为了安全起见，这里保持v不变，或者根据具体情况处理
             pass # self.v = 0 # 或者保持 self.v 不变
        else:
            self.v = (measurement - self.x) / dt 
            # 注意C++中 self.x 是更新后的x, measurement是当前原始测量
            # self.v = (x_pred - self.x) / dt # 如果用预测前的x，可能更稳定一些，但与C++不一致
                                            # 或者 self.v_new = self.v + K_v * ( (measurement - x_pred)/dt - self.v) 如果v也是状态

        return self.x

def simulate_weight_sensor():
    # 模拟参数
    total_time = 600  # 总时间 (秒)，例如10分钟
    dt = 1.0  # 时间间隔 (秒)
    initial_weight = 500.0  # 初始重量 (g)
    final_weight = 450.0 # 输液结束后的重量 (g), 模拟输液50g
    
    num_points = int(total_time / dt)
    
    # 真实重量变化 (线性减少)
    true_weight = np.linspace(initial_weight, final_weight, num_points)
    
    # 传感器噪声
    measurement_noise_std = 0.5  # g
    measurements = true_weight + np.random.normal(0, measurement_noise_std, num_points)
    
    # 模拟病人移动引入的突发噪声
    spike_indices = [50, 150, 250, 350, 450, 550]
    spike_magnitudes = [5, -4, 6, -5, 4, -6] # g
    for i, index in enumerate(spike_indices):
        if index < num_points:
            measurements[index] += spike_magnitudes[i]
            
    # 初始化滤波器 (使用C++代码中的默认参数或根据需要调整)
    # processNoise = 0.1, measurementNoise = 1.0
    kf = WeightKalmanFilterPython(process_noise=0.001, measurement_noise=100.0) # 调大R来应对尖峰噪声
    kf.init(measurements[0]) # 用第一个测量值初始化
    
    filtered_weights = np.zeros(num_points)
    estimated_velocities = np.zeros(num_points)
    
    for i in range(num_points):
        filtered_weights[i] = kf.update(measurements[i], dt)
        estimated_velocities[i] = kf.v # 保存估计的速度

    # 绘图
    time_axis = np.arange(0, total_time, dt)
    
    plt.figure(figsize=(12, 8))
    
    plt.subplot(2, 1, 1)
    plt.plot(time_axis, measurements, 'g.', alpha=0.6, label='Measurement with Noise')
    plt.plot(time_axis, true_weight, 'k--', linewidth=2, label='real weight')
    plt.plot(time_axis, filtered_weights, 'r-', linewidth=1.5, label='Kalman Filter')
    plt.xlabel('Time (s)')
    plt.ylabel('Weight (g)')
    plt.title('Kalman Filter for Weight Sensor (Simulated C++ Implementation)')
    plt.legend()
    plt.grid(True)
    
    plt.subplot(2, 1, 2)
    plt.plot(time_axis, estimated_velocities, 'b-', label='Estimated Weight Change Velocity (g/s)')
    # 真实速度
    true_velocity = (final_weight - initial_weight) / total_time
    plt.axhline(true_velocity, color='k', linestyle='--', label=f'Real Average Velocity ({true_velocity:.2f} g/s)')
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity (g/s)')
    plt.title('Estimated Weight Change Velocity')
    plt.legend()
    plt.grid(True)
    
    plt.tight_layout()
    
    # 将图像保存到临时文件
    image_path = "/tmp/kalman_weight_filter_test.png"
    try:
        plt.savefig(image_path)
        print(f"PLOT_IMAGE:{image_path}") # 特殊标记，让工具显示图片
    except Exception as e:
        print(f"Error saving plot: {e}")
    plt.close() # 关闭图像，避免在某些环境下打开多个窗口

# 运行模拟
simulate_weight_sensor()
