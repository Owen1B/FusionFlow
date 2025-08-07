import numpy as np
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm

# 尝试设置中文字体，如果失败则忽略
try:
    plt.rcParams['font.sans-serif'] = ['SimHei'] # 指定默认字体
    plt.rcParams['axes.unicode_minus'] = False # 解决保存图像是负号'-'显示为方块的问题
except Exception as e:
    print(f"无法设置中文字体，将使用默认字体: {e}")

class WeightKalmanFilterPythonImproved:
    def __init__(self, sigma_a=0.05, measurement_noise_R=1.0):
        self.sigma_a = sigma_a  # Std dev of unmodeled acceleration (process noise strength)
        self.R_measurement_noise = measurement_noise_R # Measurement noise variance

        # State vector: x_state[0] = weight, x_state[1] = velocity
        self.x_state = np.array([0.0, 0.0])

        # Estimate error covariance matrix P (2x2)
        self.P_cov = np.array([[100.0, 0.0], [0.0, 10.0]]) # Initial high uncertainty
        
        # Measurement matrix H
        self.H_matrix = np.array([[1.0, 0.0]])

    def init(self, initial_weight, initial_velocity=0.0):
        self.x_state = np.array([initial_weight, initial_velocity])
        # Reset P_cov to initial uncertainty after a known start
        self.P_cov = np.array([[1.0, 0.0], [0.0, 1.0]])

    def update(self, measurement, dt):
        if dt <= 0:
            return self.x_state[0]

        # 1. Define State Transition Matrix (F)
        F_matrix = np.array([[1.0, dt],
                             [0.0, 1.0]])

        # 2. Define Process Noise Covariance Matrix (Q)
        dt2 = dt * dt
        dt3 = dt2 * dt
        dt4 = dt3 * dt
        sigma_a_sq = self.sigma_a * self.sigma_a

        Q_process_noise_cov = np.array([
            [(dt4 / 4.0) * sigma_a_sq, (dt3 / 2.0) * sigma_a_sq],
            [(dt3 / 2.0) * sigma_a_sq, dt2 * sigma_a_sq]
        ])

        # 3. Predict state: x_pred = F * x_state
        x_pred = F_matrix @ self.x_state

        # 4. Predict estimate covariance: P_pred = F * P_cov * F_transpose + Q
        P_pred = F_matrix @ self.P_cov @ F_matrix.T + Q_process_noise_cov

        # 5. Calculate Kalman Gain (K)
        # S_innovation_cov = H * P_pred * H_transpose + R
        S_innovation_cov = self.H_matrix @ P_pred @ self.H_matrix.T + self.R_measurement_noise
        if S_innovation_cov[0,0] == 0: S_innovation_cov[0,0] = 1e-9 # Avoid division by zero
        
        # K_gain = P_pred * H_transpose * S_inverse
        K_gain = P_pred @ self.H_matrix.T @ np.linalg.inv(S_innovation_cov)

        # 6. Update state estimate: x_state = x_pred + K_gain * (measurement - H * x_pred)
        innovation_y = measurement - self.H_matrix @ x_pred
        self.x_state = x_pred + K_gain @ innovation_y

        # 7. Update estimate covariance: P_cov = (I - K_gain * H) * P_pred
        I = np.eye(2)
        self.P_cov = (I - K_gain @ self.H_matrix) @ P_pred

        return self.x_state[0]

    def get_velocity(self):
        return self.x_state[1]


def simulate_weight_sensor_improved():
    # 模拟参数
    total_time = 600  # 总时间 (秒)，例如10分钟
    dt = 1.0  # 时间间隔 (秒)
    initial_weight = 500.0  # 初始重量 (g)
    final_weight = 450.0 # 输液结束后的重量 (g), 模拟输液50g
    
    num_points = int(total_time / dt)
    
    # 真实重量变化 (线性减少)
    true_weight = np.linspace(initial_weight, final_weight, num_points)
    true_velocity = (final_weight - initial_weight) / total_time # g/s
    
    # 传感器噪声
    measurement_noise_std = 0.5  # g (standard deviation)
    R_val = measurement_noise_std**2 # Variance
    measurements = true_weight + np.random.normal(0, measurement_noise_std, num_points)
    
    # 模拟病人移动引入的突发噪声
    spike_indices = [50, 150, 250, 350, 450, 550]
    spike_magnitudes = [5, -4, 6, -5, 4, -6] # g
    for i, index in enumerate(spike_indices):
        if index < num_points:
            measurements[index] += spike_magnitudes[i]
    
    # 初始化滤波器
    # sigma_a: Controls how much the filter trusts the constant velocity model.
    # Smaller sigma_a -> smoother velocity, slower reaction to actual velocity changes.
    # Larger sigma_a -> more responsive velocity, but can be noisier.
    # R_val: Measurement noise variance. Larger R -> more smoothing, less trust in measurements.
    sigma_a_param = 0.01 # Adjust this: e.g., 0.001 (very smooth) to 0.1 (responsive)
    R_param = 2.0 # Adjust this based on observed sensor noise and desired smoothing for spikes

    kf = WeightKalmanFilterPythonImproved(sigma_a=sigma_a_param, measurement_noise_R=R_param)
    kf.init(measurements[0], initial_velocity=0.0) # Initial velocity guess
    
    filtered_weights_improved = np.zeros(num_points)
    estimated_velocities_improved = np.zeros(num_points)
    
    for i in range(num_points):
        filtered_weights_improved[i] = kf.update(measurements[i], dt)
        estimated_velocities_improved[i] = kf.get_velocity()
    
    # 绘图
    time_axis = np.arange(0, total_time, dt)
    
    plt.figure(figsize=(14, 10))
    
    plt.subplot(2, 1, 1)
    plt.plot(time_axis, measurements, 'g.', alpha=0.5, label='带噪声的测量值')
    plt.plot(time_axis, true_weight, 'k--', linewidth=2, label='真实重量')
    plt.plot(time_axis, filtered_weights_improved, 'r-', linewidth=1.5, label='改进的卡尔曼滤波后重量')
    plt.xlabel('时间 (秒)')
    plt.ylabel('重量 (g)')
    plt.title(f'改进的重量卡尔曼滤波效果 (sigma_a={sigma_a_param}, R={R_param})')
    plt.legend()
    plt.grid(True)
    
    plt.subplot(2, 1, 2)
    plt.plot(time_axis, estimated_velocities_improved, 'b-', label='改进的卡尔曼滤波估计速度')
    plt.axhline(true_velocity, color='k', linestyle='--', label=f'真实平均速度 ({true_velocity:.2f} g/s)')
    plt.xlabel('时间 (秒)')
    plt.ylabel('速度 (g/s)')
    plt.title('估计的重量变化速度 (改进的滤波器)')
    plt.legend()
    plt.grid(True)
    
    plt.tight_layout()
    
    image_path = "/tmp/improved_kalman_weight_filter_test.png"
    try:
        plt.savefig(image_path)
        print(f"PLOT_IMAGE:{image_path}")
    except Exception as e:
        print(f"Error saving plot: {e}")
    plt.close()

# 运行模拟
simulate_weight_sensor_improved()