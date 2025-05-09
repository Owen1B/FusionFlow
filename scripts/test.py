import matplotlib
import matplotlib.font_manager as fm
import os
import shutil
import sys

# 获取字体文件路径 (假设已下载到 ~/Downloads)
font_path = os.path.expanduser("~/Downloads/SimHei.ttf")
if not os.path.exists(font_path):
    print(f"错误: 字体文件不存在于 {font_path}")
    print("请先下载 SimHei.ttf 字体文件到 ~/Downloads 目录")
    sys.exit(1)

# 1. 安装到用户字体目录
user_font_dir = os.path.expanduser("~/Library/Fonts")
if not os.path.exists(user_font_dir):
    os.makedirs(user_font_dir)
user_font_path = os.path.join(user_font_dir, "SimHei.ttf")
shutil.copy2(font_path, user_font_path)
print(f"字体已安装到用户字体目录: {user_font_path}")

# 2. 安装到 matplotlib 字体目录
mpl_font_dir = os.path.join(matplotlib.get_data_path(), "fonts/ttf")
mpl_font_path = os.path.join(mpl_font_dir, "SimHei.ttf")
try:
    shutil.copy2(font_path, mpl_font_path)
    print(f"字体已安装到 matplotlib 字体目录: {mpl_font_path}")
except PermissionError:
    print(f"无权限写入 {mpl_font_dir}，尝试使用 sudo 或管理员权限安装")

# 3. 清理 matplotlib 缓存 (通用方法)
cache_dir = matplotlib.get_cachedir()
print(f"Matplotlib 缓存目录: {cache_dir}")

try:
    # 尝试删除缓存文件
    for cache_file in os.listdir(cache_dir):
        cache_path = os.path.join(cache_dir, cache_file)
        if os.path.isfile(cache_path) and cache_file.endswith('.cache'):
            os.remove(cache_path)
            print(f"已删除缓存文件: {cache_file}")
    print("缓存清理完成")
except Exception as e:
    print(f"清理缓存时出错: {e}")

# 4. 重建字体管理器 (适用于各种版本)
print("正在重建字体缓存...")
try:
    # 尝试方法1: 使用较新版本的方法
    fm.fontManager.rebuild()
    print("使用 fontManager.rebuild() 重建成功")
except AttributeError:
    try:
        # 尝试方法2: 使用较旧版本的方法
        fm._rebuild()
        print("使用 _rebuild() 重建成功")
    except AttributeError:
        # 尝试方法3: 强制重新创建fontManager
        try:
            del fm.fontManager
            fm.fontManager = fm.FontManager()
            print("通过重新创建 fontManager 重建成功")
        except Exception as e:
            print(f"所有重建方法失败: {e}")
            print("请尝试重启Python会话来完成字体缓存重建")

# 5. 确认字体安装情况
print("\n检查字体安装状态...")
font_names = [f.name for f in fm.fontManager.ttflist]
if "SimHei" in font_names:
    simhei_font = fm.findfont("SimHei")
    print(f"成功: SimHei 字体已安装并注册在: {simhei_font}")
else:
    print("注意: SimHei 未在字体列表中找到")
    
    # 查找可用的中文字体
    chinese_fonts = []
    for f in fm.fontManager.ttflist:
        try:
            if any(char in f.name for char in ['宋', '黑', '楷', '圆', 'PingFang', 'Heiti']):
                chinese_fonts.append(f.name)
        except:
            pass
    
    if chinese_fonts:
        print("可用的中文字体有:")
        for font in sorted(set(chinese_fonts)):
            print(f" - {font}")
        
        # 推荐使用的Mac默认中文字体
        print("\n推荐使用以下Mac默认中文字体:")
        recommended = [f for f in chinese_fonts if any(name in f for name in ['PingFang', 'Heiti'])]
        for font in sorted(set(recommended)):
            print(f" - {font}")
    else:
        print("未找到任何中文字体")

