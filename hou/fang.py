import os
import numpy as np
import rosbag
from scipy.optimize import minimize
import matplotlib.pyplot as plt


def calculate_topic_mean_coords(bag_path, topic_names):
    """
    读取单个ROS1 bag，计算每个Odometry话题所有位姿点的平均坐标
    :param bag_path: bag文件路径
    :param topic_names: 要处理的里程计话题列表
    :return: 字典 {话题名: 3D平均坐标[x,y,z]}
    """
    mean_coords = {topic: None for topic in topic_names}
    
    try:
        with rosbag.Bag(bag_path, 'r') as bag:
            for topic in topic_names:
                all_positions = []
                # 遍历该话题所有Odometry消息
                for _, msg, _ in bag.read_messages(topics=[topic]):
                    # 提取位姿的x,y,z坐标
                    x = msg.pose.pose.position.x
                    y = msg.pose.pose.position.y
                    z = msg.pose.pose.position.z
                    all_positions.append([x, y, z])
                
                if all_positions:
                    all_pos_np = np.array(all_positions, dtype=np.float64)
                    # 过滤nan和inf无效值
                    valid_mask = ~np.isnan(all_pos_np).any(axis=1) & ~np.isinf(all_pos_np).any(axis=1)
                    mean_coords[topic] = np.mean(all_pos_np[valid_mask], axis=0)
                else:
                    print(f"警告：话题 {topic} 在 {os.path.basename(bag_path)} 中无有效位姿")
                    
    except Exception as e:
        print(f"读取bag失败 {os.path.basename(bag_path)}: {str(e)}")
    return mean_coords


def fit_square(points_2d):
    """最小二乘法拟合正方形，输入4个二维点"""
    points = np.array(points_2d)
    
    def square_error(params):
        cx, cy, side, theta = params
        half = side / 2
        # 旋转矩阵
        rot_mat = np.array([
            [np.cos(theta), -np.sin(theta)],
            [np.sin(theta), np.cos(theta)]
        ])
        # 正方形标准顶点 + 旋转平移变换
        square_verts = np.array([[-half,-half], [half,-half], [half,half], [-half,half]])
        square_verts = square_verts @ rot_mat.T + np.array([cx, cy])
        
        # 计算输入点到最近正方形顶点的距离平方和（最小二乘目标）
        total_err = 0
        for p in points:
            total_err += np.min(np.sum((square_verts - p) ** 2, axis=1))
        return total_err
    
    # 初始值猜测 + 鲁棒优化求解
    init_center = np.mean(points, axis=0)
    init_side = np.max(np.max(points, axis=0) - np.min(points, axis=0))
    res = minimize(square_error, [init_center[0], init_center[1], init_side, 0.0], 
                   method='Nelder-Mead', tol=1e-10, options={'maxiter': 20000})
    return res.x[:2], res.x[2], res.x[3]


def generate_square_vertices(center, side, theta):
    """
    根据中心、边长、旋转角度生成正方形的四个顶点（闭合）
    :param center: 二维中心坐标 [cx, cy]
    :param side: 边长
    :param theta: 旋转角度（弧度）
    :return: (5,2)的数组，包含4个顶点+闭合点
    """
    cx, cy = center
    half = side / 2
    # 标准正方形顶点
    square_verts = np.array([[-half,-half], [half,-half], [half,half], [-half,half], [-half,-half]])
    # 旋转矩阵
    rot_mat = np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta), np.cos(theta)]
    ])
    # 旋转+平移
    square_verts = square_verts @ rot_mat.T + np.array([cx, cy])
    return square_verts


def main():
    # -------------------------- 配置（已帮你填好你的路径和话题） --------------------------
    bag_paths = [
        "/home/hou/RC2026/rosbag/bag10/9.bag",
        "/home/hou/RC2026/rosbag/bag10/10.bag",
        "/home/hou/RC2026/rosbag/bag10/11.bag",
        "/home/hou/RC2026/rosbag/bag10/12.bag"
    ]
    topic_names = [
        "/point_lio/odom"
    ]
    # 绘图颜色配置（可自定义）
    topic_colors = {
        "/point_lio/odom": "blue"
    }
    # -------------------------------------------------------------------------------------

    print("="*60)
    print("开始处理bag，计算每个话题所有位姿的平均坐标")
    print("="*60)

    # 1. 批量处理4个bag，收集每个话题的4个平均坐标
    topic_all_means = {topic: [] for topic in topic_names}
    for idx, bag_path in enumerate(bag_paths, 1):
        if not os.path.exists(bag_path):
            print(f"错误：第{idx}个bag不存在: {bag_path}")
            return
        print(f"\n处理第 {idx}/4 个bag: {os.path.basename(bag_path)}")
        mean_coords = calculate_topic_mean_coords(bag_path, topic_names)
        for topic in topic_names:
            if mean_coords[topic] is not None:
                topic_all_means[topic].append(mean_coords[topic])
                print(f"  ✅ 话题 {topic} 平均坐标: {mean_coords[topic].round(6)}")

    # 2. 每个话题单独拟合正方形，计算中心
    print("\n" + "="*60)
    print("正方形拟合结果")
    print("="*60)
    
    # 初始化绘图
    plt.figure(figsize=(10, 8))
    plt.rcParams['font.sans-serif'] = ['DejaVu Sans']  # 确保中文/特殊字符显示
    plt.grid(True, alpha=0.3)
    plt.xlabel('X Coordinate (m)', fontsize=12)
    plt.ylabel('Y Coordinate (m)', fontsize=12)
    plt.title('Square Fitting Results of XY Coordinates (Two Topics)', fontsize=14, fontweight='bold')

    for topic in topic_names:
        points_3d = np.array(topic_all_means[topic])
        if len(points_3d) != 4:
            print(f"\n❌ 话题 {topic} 有效点数不足4个，跳过拟合")
            continue
        
        # 提取XY坐标
        points_2d = points_3d[:, :2]
        
        print(f"\n📌 话题: {topic}")
        print(f"四个bag的平均位姿坐标(XY):\n{points_2d.round(6)}")
        
        # 拟合正方形
        center_2d, side, theta = fit_square(points_2d)
        print(f"✅ 拟合完成:")
        print(f"  正方形中心(XY): {center_2d.round(8)}")
        print(f"  正方形边长: {side:.8f} m")
        print(f"  旋转角度: {np.rad2deg(theta):.4f} °")
        
        # 生成正方形顶点
        square_verts = generate_square_vertices(center_2d, side, theta)
        
        # 绘制当前话题的原始点和拟合正方形
        color = topic_colors[topic]
        # 绘制原始4个点
        plt.scatter(points_2d[:, 0], points_2d[:, 1], color=color, s=80, 
                    label=f'{topic} (Original Points)', alpha=0.8, edgecolors='black')
        # 绘制拟合的正方形
        plt.plot(square_verts[:, 0], square_verts[:, 1], color=color, linewidth=2, 
                 label=f'{topic} (Fitted Square)', alpha=0.8)
        # 绘制正方形中心
        plt.scatter(center_2d[0], center_2d[1], color=color, s=150, 
                    marker='*', edgecolors='black', label=f'{topic} (Square Center)')

    # 调整图例（避免重复）
    handles, labels = plt.gca().get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    plt.legend(by_label.values(), by_label.keys(), fontsize=10, loc='best')
    
    # 保证坐标轴等比例（正方形不会变形）
    plt.axis('equal')
    
    # 保存图片（可选，注释掉则显示）
    # plt.savefig('/home/hou/RC2026/square_fitting_result.png', dpi=300, bbox_inches='tight')
    
    # 显示图片
    plt.show()


if __name__ == "__main__":
    main()