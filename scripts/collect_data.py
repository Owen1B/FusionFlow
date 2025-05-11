import asyncio
import websockets
import pandas as pd
from datetime import datetime
import os

# --- 配置参数 ---
ESP32_IP_ADDRESS = "192.168.0.107"  # <--- 请将此替换为您的ESP32的IP地址!
WEBSOCKET_PORT = 81
TARGET_EMPTY_WEIGHT_G = 60.0  # 目标空袋重量，用于停止数据收集
SAVE_INTERVAL_POINTS = 60     # 每收集多少个数据点就保存一次文件
OUTPUT_DIR = "data/collected_infusion_data" # 数据保存的目录

# 更新列名以匹配ESP32发送的数据格式 (24列)
COLUMN_NAMES = [
    "timestamp_ms", "raw_w_g", "filt_w_g", "raw_flow_w_gps", "filt_flow_w_gps",
    "drops_p_cycle", "raw_dps", "filt_dps", "raw_flow_d_gps", "filt_flow_d_gps",
    "wpd_g_drip", "wpd_cal_active", "wpd_long_cal_active", "rem_weight_drip_g", 
    "fused_flow_gps", "fused_rem_weight_g", "rem_t_raw_w_s", "rem_t_filt_w_s", 
    "rem_t_raw_d_s", "rem_t_filt_d_s", "rem_t_fused_s", "drip_total_drops",
    "drip_initial_weight", "wpd_cumulative"
]

def ensure_output_dir_exists():
    if not os.path.exists(OUTPUT_DIR):
        os.makedirs(OUTPUT_DIR)
        print(f"已创建目录: {OUTPUT_DIR}")

def save_data_to_csv(data_list, filename):
    if not data_list:
        print(f"没有数据可保存到 {filename}")
        return
    df = pd.DataFrame(data_list)
    try:
        df.to_csv(filename, index=False)
        print(f"数据已保存到: {filename} (共 {len(data_list)} 个点)")
    except Exception as e:
        print(f"保存数据到 {filename} 时发生错误: {e}")

async def collect_infusion_data():
    uri = f"ws://{ESP32_IP_ADDRESS}:{WEBSOCKET_PORT}/"
    collected_data_points = []
    
    ensure_output_dir_exists()
    current_time_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_filename = os.path.join(OUTPUT_DIR, f"infusion_data_session_{current_time_str}.csv")
    
    print(f"准备通过WebSocket连接到ESP32: {uri}")
    print(f"收集的数据将保存到: {csv_filename}")
    print(f"每收集 {SAVE_INTERVAL_POINTS} 个数据点将进行一次保存。")

    points_since_last_save = 0

    try:
        async with websockets.connect(uri, ping_interval=None) as websocket:
            print("已成功连接到ESP32 WebSocket。")
            print(f"开始收集数据，直到'滤波后重量 (filt_w_g)' <= {TARGET_EMPTY_WEIGHT_G}g...")

            while True:
                message = await websocket.recv()

                if isinstance(message, str) and ',' in message:
                    try:
                        values = [float(v) for v in message.split(',')]
                        
                        if len(values) == len(COLUMN_NAMES):
                            data_point = dict(zip(COLUMN_NAMES, values))
                            collected_data_points.append(data_point)
                            points_since_last_save += 1
                            
                            current_filt_weight = data_point['filt_w_g']
                            print(f"数据点: 时间戳={data_point['timestamp_ms']:.0f}ms, 滤波重量={current_filt_weight:.2f}g, 点数={len(collected_data_points)}")

                            if points_since_last_save >= SAVE_INTERVAL_POINTS:
                                save_data_to_csv(collected_data_points, csv_filename)
                                points_since_last_save = 0

                            if current_filt_weight <= TARGET_EMPTY_WEIGHT_G:
                                print(f"\n目标重量 {TARGET_EMPTY_WEIGHT_G}g 已达到。停止数据收集。")
                                break
                        else:
                            if not message.lower().startswith("timestamp_ms"): # 避免过多打印表头警告
                                print(f"警告: 收到数据行，但列数不匹配 ({len(values)} 列，期望 {len(COLUMN_NAMES)} 列): {message[:100]}...")
                    
                    except ValueError:
                        if "<!DOCTYPE html>" in message.lower() or "<html" in message.lower():
                            pass # 忽略HTML页面内容
                        elif not message.lower().startswith("timestamp_ms"):
                             print(f"警告: 无法解析为数据行，已跳过: {message[:100]}...")
                        continue
                else:
                    # 忽略非CSV字符串或二进制数据
                    pass

    except websockets.exceptions.ConnectionClosedError:
        print("错误: 与ESP32的WebSocket连接被关闭。")
    except ConnectionRefusedError:
        print(f"错误: 无法连接到 {uri}。请检查ESP32是否已开机、网络连接是否正常、IP地址 ({ESP32_IP_ADDRESS}) 是否正确，以及WebSocket服务器是否正在运行。")
    except KeyboardInterrupt:
        print("\n用户手动中断了数据收集。")
    except Exception as e:
        print(f"在WebSocket通信过程中发生未知错误: {e}")
    finally:
        if collected_data_points:
            print("\n正在进行最终数据保存...")
            save_data_to_csv(collected_data_points, csv_filename)
        else:
            print("未收集到任何数据。")
        print("数据收集程序结束。")

if __name__ == "__main__":
    try:
        asyncio.run(collect_infusion_data())
    except KeyboardInterrupt: # 主asyncio循环也可能被中断
        print("\n程序被用户中断。") 