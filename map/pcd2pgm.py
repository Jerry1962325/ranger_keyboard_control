import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import sys
import os

def pcd2pgm(pcd_path, output_name="map", resolution=0.05, z_min=-0.5, z_max=1.0):
    print(f"正在读取点云: {pcd_path}")
    pcd = o3d.io.read_point_cloud(pcd_path)
    points = np.asarray(pcd.points)
    
    print(f"点云数量: {len(points)}")
    
    # 1. 高度过滤 (直通滤波)
    # 只保留地面以上一定高度，且低于天花板的点
    mask = (points[:, 2] > z_min) & (points[:, 2] < z_max)
    points = points[mask]
    print(f"过滤后点云数量: {len(points)}")

    if len(points) == 0:
        print("错误: 过滤后没有剩余点！请检查 z_min 和 z_max 参数。")
        return

    # 2. 计算地图边界
    x_min, y_min = np.min(points[:, :2], axis=0)
    x_max, y_max = np.max(points[:, :2], axis=0)
    
    width = int((x_max - x_min) / resolution) + 1
    height = int((y_max - y_min) / resolution) + 1
    
    print(f"地图尺寸: {width} x {height} 像素")
    
    # 3. 栅格化
    grid = np.zeros((height, width), dtype=np.uint8) # 0: 未知 (灰色)
    
    # 将坐标转换为像素索引
    idx_x = ((points[:, 0] - x_min) / resolution).astype(int)
    idx_y = ((points[:, 1] - y_min) / resolution).astype(int)
    
    # 简单的占据栅格逻辑：有点的地方就是障碍物 (黑色=0, 白色=255, 灰色=205)
    # 这里我们用 0 表示黑(障碍), 255 表示白(空闲)
    # 初始化全白 (假设全是空地)
    grid.fill(254) 
    
    # 标记障碍物 (黑色)
    # 注意图片坐标系：y轴通常是向下的，所以可能需要翻转，或者直接对应
    # ROS map_server 坐标系：原点在左下角
    grid[idx_y, idx_x] = 0 
    
    # 4. 保存 PNG 图片 (替代 PGM，兼容性更好)
    # 注意：numpy 数组的 [0,0] 是左上角，而 ROS 地图原点通常对应图片的左下角
    # 刚才计算 idx_y 时，y_min 对应 index 0。
    # 如果直接保存，y_min 会在图片顶部。
    # 所以我们需要上下翻转数组，让 y_min 跑到图片底部。
    grid = np.flipud(grid)
    
    # 使用 PIL 保存图片
    from PIL import Image
    image = Image.fromarray(grid)
    
    png_path = f"{output_name}.png"
    image.save(png_path)
    print(f"已保存图片: {png_path}")
    
    # 5. 保存 YAML 配置文件
    yaml_content = f"""image: {output_name}.png
resolution: {resolution}
origin: [{x_min}, {y_min}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
"""
    yaml_path = f"{output_name}.yaml"
    with open(yaml_path, 'w') as f:
        f.write(yaml_content)
    print(f"已保存配置: {yaml_path}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("用法: python3 pcd2pgm.py <pcd_file_path>")
    else:
        pcd_file = sys.argv[1]
        pcd2pgm(pcd_file)