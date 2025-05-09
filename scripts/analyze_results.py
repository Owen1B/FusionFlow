import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime
from matplotlib.ticker import MaxNLocator
import matplotlib as mpl
from scipy import stats

# 设置matplotlib支持中文显示
plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'SimSun', 'Arial Unicode MS']
plt.rcParams['axes.unicode_minus'] = False
plt.rcParams['font.family'] = 'sans-serif'

# 修复中文字体问题
mpl.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'SimSun', 'Arial Unicode MS']
mpl.rcParams['axes.unicode_minus'] = False
mpl.rcParams['font.family'] = 'sans-serif'

def calculate_drop_weight(df):
    """计算每滴的重量"""
    # 计算重量变化
    df['weight_diff'] = -df['weight'].diff()  # 取负值,使重量变化为正
    df['drops_diff'] = df['drops'].diff()
    
    # 计算每分钟的重量变化和滴数变化
    # 使用滚动窗口计算1分钟内的变化
    window_size = 60  # 假设数据采集间隔为1秒，60个数据点代表1分钟
    
    # 确保数据足够长
    if len(df) > window_size:
        # 创建新的DataFrame来存储滚动窗口结果
        rolling_data = []
        
        for i in range(window_size, len(df)):
            window = df.iloc[i-window_size:i+1]
            weight_change = window['weight'].iloc[0] - window['weight'].iloc[-1]
            drops_change = window['drops'].iloc[-1] - window['drops'].iloc[0]
            
            if drops_change > 0:  # 避免除以零
                drop_weight = weight_change / drops_change
            else:
                drop_weight = np.nan
                
            rolling_data.append({
                'time_min': df['time_min'].iloc[i],
                'drop_weight': drop_weight
            })
        
        # 创建滚动窗口结果的DataFrame
        rolling_df = pd.DataFrame(rolling_data)
        
        # 去除异常值
        if not rolling_df.empty and not rolling_df['drop_weight'].isna().all():
            Q1 = rolling_df['drop_weight'].quantile(0.25)
            Q3 = rolling_df['drop_weight'].quantile(0.75)
            IQR = Q3 - Q1
            rolling_df['drop_weight'] = rolling_df['drop_weight'].clip(Q1 - 1.5 * IQR, Q3 + 1.5 * IQR)
        
        return df, rolling_df
    else:
        return df, pd.DataFrame(columns=['time_min', 'drop_weight'])

def process_folder(folder_path, all_results=None):
    """处理单个结果文件夹"""
    csv_file = os.path.join(folder_path, "infusion_data.csv")
    if not os.path.exists(csv_file):
        print(f"跳过文件夹 {folder_path}: 未找到CSV文件")
        return None
    
    print(f"处理文件夹: {folder_path}")
    
    # 读取CSV文件
    df = pd.read_csv(csv_file)
    
    # 转换时间戳为相对时间(分钟)
    start_time = df['timestamp'].iloc[0]
    df['time_sec'] = (df['timestamp'] - start_time) / 1000.0
    df['time_min'] = df['time_sec'] / 60.0
    
    # 计算每滴重量
    df, rolling_df = calculate_drop_weight(df)
    
    # 创建主图表
    fig, ax1 = plt.subplots(figsize=(12, 8))
    
    # 设置图表标题
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
    
    # 滴速
    line2, = ax2.plot(df['time_min'], df['dropRate'], 'r-', linewidth=2, label='滴速 (滴/分)')
    ax2.set_ylabel('滴速 (滴/分)', color='red')
    ax2.tick_params(axis='y', labelcolor='red')
    
    # 创建第二个右侧Y轴
    ax3 = ax1.twinx()
    ax3.spines['right'].set_position(('outward', 60))
    
    # 累计滴数
    line3, = ax3.plot(df['time_min'], df['drops'], 'g-.', linewidth=2, label='累计滴数')
    ax3.set_ylabel('累计滴数', color='green')
    ax3.tick_params(axis='y', labelcolor='green')
    ax3.yaxis.set_major_locator(MaxNLocator(integer=True))
    
    # 添加图例
    lines = [line1, line2, line3]
    labels = [l.get_label() for l in lines]
    fig.legend(lines, labels, loc='upper left', bbox_to_anchor=(0.1, 0.98),
               ncol=3, frameon=True, fontsize=10)
    
    # 调整布局
    plt.tight_layout()
    
    # 保存主图表
    plt.savefig(os.path.join(folder_path, "infusion_plots.png"), dpi=100)
    plt.close()
    
    # 创建滴数与重量的线性拟合图
    fig, ax = plt.subplots(figsize=(10, 6))
    
    # 绘制散点图
    ax.scatter(df['drops'], df['weight'], alpha=0.6, label='实际数据点')
    
    # 线性拟合
    mask = ~np.isnan(df['drops']) & ~np.isnan(df['weight'])
    fit_result = None
    if np.sum(mask) > 1:  # 确保有足够的数据点进行拟合
        slope, intercept, r_value, p_value, std_err = stats.linregress(df['drops'][mask], df['weight'][mask])
        
        # 绘制拟合线
        x_line = np.array([df['drops'].min(), df['drops'].max()])
        y_line = slope * x_line + intercept
        ax.plot(x_line, y_line, 'r-', linewidth=2, 
                label=f'线性拟合: y = {slope:.4f}x + {intercept:.2f}\nR² = {r_value**2:.4f}')
        
        # 计算平均每滴重量
        avg_drop_weight = -slope  # 取负值，因为重量随滴数增加而减少
        ax.text(0.05, 0.05, f'平均每滴重量: {avg_drop_weight:.4f}g', 
                transform=ax.transAxes, fontsize=12, bbox=dict(facecolor='white', alpha=0.8))
        
        # 保存拟合结果
        fit_result = {
            'folder': os.path.basename(folder_path),
            'slope': slope,
            'intercept': intercept,
            'r_squared': r_value**2,
            'avg_drop_weight': avg_drop_weight,
            'p_value': p_value,
            'std_err': std_err,
            'data_points': np.sum(mask),
            'min_drops': df['drops'].min(),
            'max_drops': df['drops'].max(),
            'min_weight': df['weight'].min(),
            'max_weight': df['weight'].max(),
            'mean_drop_rate': df['dropRate'].mean()
        }
        
        if all_results is not None:
            all_results.append(fit_result)
    
    ax.set_title('滴数与重量的线性关系', fontsize=14)
    ax.set_xlabel('累计滴数')
    ax.set_ylabel('重量 (克)')
    ax.grid(True, linestyle='--', alpha=0.7)
    ax.legend()
    
    # 使用ASCII字符替代特殊字符
    plt.tight_layout()
    plt.savefig(os.path.join(folder_path, "drops_weight_linear_fit.png"), dpi=100, 
                bbox_inches='tight')  # 添加bbox_inches参数避免裁剪
    plt.close()
    
    # 打印统计信息
    if not rolling_df.empty and not rolling_df['drop_weight'].isna().all():
        valid_drop_weight = rolling_df['drop_weight'].dropna()
        mean_drop_weight = valid_drop_weight.mean()
        
        print(f"文件夹 {folder_path} 的分析结果:")
        print(f"平均滴重: {mean_drop_weight:.3f}g")
        print(f"滴重标准差: {valid_drop_weight.std():.3f}g")
        print(f"滴重中位数: {valid_drop_weight.median():.3f}g")
        print(f"平均滴速: {df['dropRate'].mean():.1f}滴/分")
    else:
        print(f"文件夹 {folder_path} 没有足够的数据计算滴重")
    
    print("-" * 50)
    return fit_result

def create_summary_plots(all_results):
    """创建所有拟合结果的汇总图表和表格"""
    if not all_results:
        print("没有可用的拟合结果进行汇总")
        return
    
    # 创建结果目录
    summary_dir = os.path.join("result", "analysis")
    os.makedirs(summary_dir, exist_ok=True)
    
    # 创建DataFrame
    results_df = pd.DataFrame(all_results)
    
    # 保存为CSV
    results_df.to_csv(os.path.join(summary_dir, "all_fit_results.csv"), index=False)
    
    # 创建滴重分布图
    plt.figure(figsize=(12, 8))
    plt.bar(results_df['folder'], results_df['avg_drop_weight'], alpha=0.7)
    plt.axhline(y=results_df['avg_drop_weight'].mean(), color='r', linestyle='-', 
                label=f'平均值: {results_df["avg_drop_weight"].mean():.4f}g')
    plt.title('各实验的平均滴重比较', fontsize=14)
    plt.xlabel('实验ID')
    plt.ylabel('平均滴重 (g)')
    plt.xticks(rotation=45, ha='right')
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(summary_dir, "drop_weight_comparison.png"), dpi=100)
    plt.close()
    
    # 创建R²值分布图
    plt.figure(figsize=(12, 8))
    plt.bar(results_df['folder'], results_df['r_squared'], alpha=0.7)
    plt.axhline(y=results_df['r_squared'].mean(), color='r', linestyle='-', 
                label=f'平均值: {results_df["r_squared"].mean():.4f}')
    plt.title('各实验的R²值比较', fontsize=14)
    plt.xlabel('实验ID')
    plt.ylabel('R²值')
    plt.xticks(rotation=45, ha='right')
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(summary_dir, "r_squared_comparison.png"), dpi=100)
    plt.close()
    
    # 创建滴速与滴重的散点图
    plt.figure(figsize=(10, 8))
    plt.scatter(results_df['mean_drop_rate'], results_df['avg_drop_weight'], alpha=0.7)
    
    # 添加标签
    for i, txt in enumerate(results_df['folder']):
        plt.annotate(txt, (results_df['mean_drop_rate'].iloc[i], results_df['avg_drop_weight'].iloc[i]),
                    fontsize=8, alpha=0.8)
    
    plt.title('滴速与滴重的关系', fontsize=14)
    plt.xlabel('平均滴速 (滴/分)')
    plt.ylabel('平均滴重 (g)')
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.tight_layout()
    plt.savefig(os.path.join(summary_dir, "drop_rate_vs_weight.png"), dpi=100)
    plt.close()
    
    # 创建汇总表格图
    fig, ax = plt.subplots(figsize=(14, len(results_df) * 0.5 + 2))
    ax.axis('tight')
    ax.axis('off')
    
    # 准备表格数据
    table_data = results_df[['folder', 'avg_drop_weight', 'r_squared', 'mean_drop_rate', 'data_points']]
    table_data.columns = ['实验ID', '平均滴重(g)', 'R²值', '平均滴速(滴/分)', '数据点数']
    
    # 添加统计行
    stats_row = pd.DataFrame({
        '实验ID': ['平均值'],
        '平均滴重(g)': [results_df['avg_drop_weight'].mean()],
        'R²值': [results_df['r_squared'].mean()],
        '平均滴速(滴/分)': [results_df['mean_drop_rate'].mean()],
        '数据点数': [results_df['data_points'].mean()]
    })
    
    table_data = pd.concat([table_data, stats_row], ignore_index=True)
    
    # 创建表格
    table = ax.table(cellText=table_data.values, colLabels=table_data.columns, 
                    loc='center', cellLoc='center')
    
    # 设置表格样式
    table.auto_set_font_size(False)
    table.set_fontsize(10)
    table.scale(1, 1.5)
    
    # 高亮统计行
    for j in range(len(table_data.columns)):
        table[(len(results_df) + 1, j)].set_facecolor('#D7E4F5')
    
    plt.title('实验结果汇总表', fontsize=16, pad=20)
    plt.tight_layout()
    plt.savefig(os.path.join(summary_dir, "results_summary_table.png"), dpi=100, bbox_inches='tight')
    plt.close()
    
    print(f"汇总分析结果已保存到 {summary_dir} 目录")
    
    # 打印统计信息
    print("\n===== 所有实验的汇总统计 =====")
    print(f"实验总数: {len(results_df)}")
    print(f"平均滴重: {results_df['avg_drop_weight'].mean():.4f}g ± {results_df['avg_drop_weight'].std():.4f}g")
    print(f"平均R²值: {results_df['r_squared'].mean():.4f} ± {results_df['r_squared'].std():.4f}")
    print(f"平均滴速: {results_df['mean_drop_rate'].mean():.2f} ± {results_df['mean_drop_rate'].std():.2f}滴/分")
    print("=" * 35)

def main():
    result_dir = "result"
    all_results = []
    
    # 遍历所有结果文件夹
    for folder in os.listdir(result_dir):
        folder_path = os.path.join(result_dir, folder)
        if os.path.isdir(folder_path) and folder != "analysis":
            process_folder(folder_path, all_results)
    
    # 创建汇总分析
    create_summary_plots(all_results)

if __name__ == "__main__":
    main()