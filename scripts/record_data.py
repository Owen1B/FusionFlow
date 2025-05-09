import asyncio  
import websockets  
import pandas as pd  
import matplotlib.pyplot as plt  
from datetime import datetime  
import csv  
import os  
import time  
from matplotlib.ticker import MaxNLocator  
import matplotlib as mpl  

# 设置matplotlib支持中文显示  
plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'SimSun', 'Arial Unicode MS']  # 优先使用的中文字体  
plt.rcParams['axes.unicode_minus'] = False  # 解决负号显示问题  
plt.rcParams['font.family'] = 'sans-serif'  # 使用sans-serif字体族  

# 配置  
ESP32_IP = "192.168.0.102"  # ESP32的IP地址  
WEBSOCKET_PORT = 81  
SAVE_INTERVAL = 10  # 保存数据的间隔（秒）  
PLOT_INTERVAL = 10  # 更新图表的间隔（秒）  
DROPS_TIMEOUT = 180  # 滴数3分钟不变化就停止记录

# 创建结果文件夹
timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
result_dir = os.path.join("result", timestamp_str)
os.makedirs(result_dir, exist_ok=True)

# 设置输出文件路径
OUTPUT_FILE = os.path.join(result_dir, "infusion_data.csv")
PLOT_FILE = os.path.join(result_dir, "infusion_plots.png")

# 全局数据  
data = []  
columns = None  
new_data_buffer = []  # 用于缓存尚未保存的数据  
last_save_time = time.time()  # 上次保存数据的时间  
last_plot_time = time.time()  # 上次绘图的时间  
plotting_active = False  # 控制是否开始绘图，至少需要有2个数据点  
last_drops = 0  # 上次记录的滴数
last_drops_time = time.time()  # 上次滴数变化的时间

async def receive_data():  
    uri = f"ws://{ESP32_IP}:{WEBSOCKET_PORT}"  
    
    print(f"正在连接到 {uri}...")  
    
    try:  
        async with websockets.connect(uri) as websocket:  
            print("已连接!")  
            
            # 接收并处理传入的数据  
            while True:  
                message = await websocket.recv()  
                if not process_message(message):  # 如果返回False表示需要停止记录
                    print("滴数3分钟未变化,停止记录")
                    break
                
    except Exception as e:  
        print(f"连接错误: {e}")  
        # 断线重连  
        await asyncio.sleep(5)  
        print("尝试重新连接...")  
        asyncio.create_task(receive_data())  

def calculate_drop_rate(df, window_size=5):
    """计算滴速 - 使用精确的滴落时间戳"""
    # 初始化滴速列
    df['dropRate'] = 0.0
    
    # 处理每一行数据
    for i in range(len(df)):
        # 检查是否有滴落时间戳
        if 'drops:' in str(df.iloc[i]):
            # 提取时间戳部分
            timestamp_str = str(df.iloc[i]).split('drops:')[1]
            timestamps = [int(ts) for ts in timestamp_str.split(',')]
            
            if len(timestamps) >= 2:
                # 计算相邻滴落之间的时间间隔（毫秒）
                intervals = []
                for j in range(1, len(timestamps)):
                    interval = timestamps[j] - timestamps[j-1]
                    if interval > 0:  # 确保时间间隔为正
                        intervals.append(interval)
                
                if intervals:
                    # 计算平均间隔时间（毫秒）
                    avg_interval = sum(intervals) / len(intervals)
                    # 转换为滴速（滴/分钟）
                    df.loc[df.index[i], 'dropRate'] = 60000.0 / avg_interval
    
    # 使用移动平均平滑滴速
    if window_size > 1:
        df['dropRate'] = df['dropRate'].rolling(window=window_size, min_periods=1).mean()
    
    return df

def process_message(message):  
    global columns, data, new_data_buffer, plotting_active, last_drops, last_drops_time  
    
    message = message.strip()  
    
    if ',' in message:  
        if message.startswith("timestamp"):  
            # 这是标题行  
            columns = message.split(',')  
            print(f"接收到标题: {columns}")  
            
            # 如果文件不存在，创建并写入标题  
            file_exists = os.path.isfile(OUTPUT_FILE)  
            with open(OUTPUT_FILE, 'a', newline='') as f:  
                writer = csv.writer(f)  
                if not file_exists:  
                    writer.writerow(columns)  
            return True
        else:  
            # 这是数据行  
            values = message.split(',')  
            if len(values) >= len(columns):  
                row_data = {columns[i]: values[i] for i in range(len(columns))}  
                data.append(row_data)  
                new_data_buffer.append(values)  # 添加到缓冲区  
                
                # 确保有足够的数据开始绘图  
                if len(data) >= 2:  
                    plotting_active = True  
                
                # 转换为适当的类型  
                timestamp = int(row_data['timestamp'])  
                drops = int(row_data['drops'])  
                weight = float(row_data['weight'])  
                drop_rate = float(row_data['dropRate'])  # 新增滴速数据
                
                # 检查滴数是否变化
                if drops != last_drops:
                    last_drops = drops
                    last_drops_time = time.time()
                elif time.time() - last_drops_time > DROPS_TIMEOUT:
                    return False
                
                # 打印当前值  
                now = datetime.now().strftime("%H:%M:%S")  
                print(f"[{now}] 滴数: {drops}, 重量: {weight:.1f}g, 滴速: {drop_rate:.1f}滴/分")
            else:  
                print(f"警告: 数据格式不匹配 - {message}")  
    else:  
        # 这是一条普通消息  
        print(f"信息: {message}")  
    return True

async def save_data_periodically():  
    """定时保存数据到CSV文件"""  
    global new_data_buffer, last_save_time  
    
    while True:  
        await asyncio.sleep(1)  # 每秒检查一次  
        current_time = time.time()  
        
        # 如果距离上次保存已经过了指定间隔，并且有新数据  
        if (current_time - last_save_time >= SAVE_INTERVAL) and new_data_buffer:  
            # 保存数据  
            with open(OUTPUT_FILE, 'a', newline='') as f:  
                writer = csv.writer(f)  
                writer.writerows(new_data_buffer)  
            
            # 打印保存信息  
            save_time = datetime.now().strftime("%H:%M:%S")  
            print(f"[{save_time}] 已保存 {len(new_data_buffer)} 条数据记录到 {OUTPUT_FILE}")  
            
            # 清空缓冲区并更新保存时间  
            new_data_buffer = []  
            last_save_time = current_time  

async def update_plots_periodically():  
    """定时更新图表"""  
    global last_plot_time, plotting_active  
    
    while True:  
        await asyncio.sleep(1)  # 每秒检查一次  
        current_time = time.time()  
        
        # 如果距离上次更新图表已经过了指定间隔，并且有足够的数据可以绘图  
        if (current_time - last_plot_time >= PLOT_INTERVAL) and plotting_active:  
            # 更新图表  
            update_plots()  
            last_plot_time = current_time  

def check_font_availability():  
    """检查系统中可用的中文字体并打印信息"""  
    from matplotlib.font_manager import findfont, FontProperties  
    try:  
        font_path = findfont(FontProperties(family=['SimHei', 'Microsoft YaHei', 'SimSun', 'Arial Unicode MS']))  
        print(f"使用中文字体: {os.path.basename(font_path)}")  
        return True  
    except Exception as e:  
        print(f"警告: 找不到合适的中文字体: {e}")  
        print("将尝试使用系统默认字体...")  
        return False  

def update_plots():  
    """更新并保存图表 - 所有数据在同一图表上"""  
    if len(data) < 2:  
        return  
    
    try:  
        # 创建DataFrame  
        df = pd.DataFrame(data)  
        df['timestamp'] = pd.to_numeric(df['timestamp'])  
        df['drops'] = pd.to_numeric(df['drops'])  
        df['weight'] = pd.to_numeric(df['weight'])  
        df['dropRate'] = pd.to_numeric(df['dropRate'])  # 使用设备计算的滴速
        
        # 转换时间戳为相对时间(分钟)  
        start_time = df['timestamp'].iloc[0]  
        df['time_sec'] = (df['timestamp'] - start_time) / 1000.0  
        df['time_min'] = df['time_sec'] / 60.0  
        
        # 创建一个图表，但使用双Y轴  
        fig, ax1 = plt.subplots(figsize=(12, 8))  
        
        # 设置图表标题，包含当前时间  
        current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")  
        plt.title(f'输液监测数据 - 更新时间: {current_time}', fontsize=14)  
        
        # 左侧Y轴 - 重量  
        line1, = ax1.plot(df['time_min'], df['weight'], 'b-', linewidth=2, label='重量 (克)')  
        ax1.set_xlabel('时间 (分钟)')  
        ax1.set_ylabel('重量 (克)', color='blue')  
        ax1.tick_params(axis='y', labelcolor='blue')  
        ax1.grid(True, linestyle='--', alpha=0.7)  
        
        # 创建右侧Y轴  
        ax2 = ax1.twinx()  
        
        # 滴速 - 红色虚线
        line2, = ax2.plot(df['time_min'], df['dropRate'], 'r--', linewidth=2, label='滴速 (滴/分)')
        ax2.set_ylabel('滴速 (滴/分)', color='red')  
        ax2.tick_params(axis='y', labelcolor='red')  
        
        # 创建第二个右侧Y轴  
        ax3 = ax1.twinx()  
        # 将其移到右侧，稍微偏移一点  
        ax3.spines['right'].set_position(('outward', 60))  
        
        # 累计滴数 - 绿色点线
        line3, = ax3.plot(df['time_min'], df['drops'], 'g-.', linewidth=2, label='累计滴数')  
        ax3.set_ylabel('累计滴数', color='green')  
        ax3.tick_params(axis='y', labelcolor='green')  
        
        # 确保滴数y轴显示整数  
        ax3.yaxis.set_major_locator(MaxNLocator(integer=True))  
        
        # 添加图例  
        lines = [line1, line2, line3]  
        labels = [l.get_label() for l in lines]  
        fig.legend(lines, labels, loc='upper left', bbox_to_anchor=(0.1, 0.98),   
                   ncol=3, frameon=True, fontsize=10)  
        
        # 调整布局  
        plt.tight_layout()  
        
        # 保存图像到结果文件夹
        plt.savefig(PLOT_FILE, dpi=100)  
        plt.close()  
        
        print(f"[{current_time}] 已更新图表")  
        
    except Exception as e:  
        print(f"更新图表时出错: {e}")  

async def data_monitoring_mode():  
    """数据监控模式，同时运行数据接收和定时任务"""  
    # 创建定时保存数据和更新图表的任务  
    save_task = asyncio.create_task(save_data_periodically())  
    plot_task = asyncio.create_task(update_plots_periodically())  
    
    print(f"监测服务已启动:")  
    print(f"- 每 {SAVE_INTERVAL} 秒保存数据到 {OUTPUT_FILE}")  
    print(f"- 每 {PLOT_INTERVAL} 秒更新图表")  
    print(f"- 滴数 {DROPS_TIMEOUT} 秒不变化将停止记录")
    
    # 启动数据接收任务  
    await receive_data()  
    
    # 取消定时任务  
    save_task.cancel()  
    plot_task.cancel()  

async def main():  
    print("ESP32 智能输液监测系统 - Python客户端")  
    print("数据上传间隔: 10秒")
    print("图表更新间隔: 60秒")
    print("数据保存间隔: 300秒")
    print("滴数3分钟不变化将停止记录")
    
    # 检查中文字体可用性  
    check_font_availability()  
    
    # 直接启动数据监测
    await data_monitoring_mode()  

if __name__ == "__main__":  
    try:  
        asyncio.run(main())  
    except KeyboardInterrupt:  
        print("\n程序已手动停止")  
    finally:  
        # 确保退出前保存所有缓冲区数据  
        if new_data_buffer:  
            with open(OUTPUT_FILE, 'a', newline='') as f:  
                writer = csv.writer(f)  
                writer.writerows(new_data_buffer)  
            print(f"已保存 {len(new_data_buffer)} 条未写入的数据记录")  