import time

import numpy as np
import open3d as o3d


# 点云边界提取
def edge_estimate(
    points,
    max_nn=30,
    normals_raduis=10,
    search_raduis=10,
    nb_points=30,
    angle=90,
    vis=False,
):
    """
    :param points: 传入的点云点坐标，np格式
    :param max_nn: 邻域内最大值
    :param normals_raduis:用于HybridSearch的邻域内搜索半径
    :param search_raduis:边界提取搜索半径
    :param nb_points:边界提取领域点
    :param angle:角度阈值
    :param vis:是否可视化
    :return:
    """

    pcd = o3d.t.geometry.PointCloud(points)
    pcd.estimate_normals(max_nn=max_nn, radius=normals_raduis)  # 计算点云法向量
    start_time = time.time()
    boundarys, mask = pcd.compute_boundary_points(
        radius=search_raduis, max_nn=nb_points, angle_threshold=angle
    )  # 边界提取的搜索半径、邻域最大点数和夹角阈值（角度制）
    end_time = time.time()
    elapsed_time = end_time - start_time
    print(f"The function took {elapsed_time} second to run")
    print(
        f"Detect {boundarys.point.positions.shape[0]} bnoundary points from {pcd.point.positions.shape[0]} points."
    )
    boundarys = boundarys.paint_uniform_color([1.0, 0.0, 0.0])
    boundarys_points = np.array(boundarys.to_legacy().points)

    if vis == True:
        pcd = pcd.paint_uniform_color([0.6, 0.6, 0.6])
        vis = o3d.visualization.Visualizer()
        vis.create_window(window_name="三维点云边界提取", width=1200, height=800)
        # 可视化参数设置
        opt = vis.get_render_option()
        opt.background_color = np.asarray([1, 1, 1])  # 设置背景色
        opt.point_size = 3  # 设置点的大小
        vis.add_geometry(boundarys.to_legacy())  # 加载边界点云到可视化窗口
        # vis.add_geometry(pcd.to_legacy())  # 加载原始点云到可视化窗口
        vis.run()  # 激活显示窗口，这个函数将阻塞当前线程，直到窗口关闭。
        vis.destroy_window()  # 销毁窗口，这个函数必须从主线程调用。

    return boundarys_points


pcd_path = r"D:/Data/GOM_Point/M1_gom.txt"
points = []
with open(pcd_path, "r") as f:
    for line in f:
        # 去除行尾的换行符，并按空格分割字符串
        data = line.strip().split()
        # 假设txt文件前三列是x, y, z坐标
        x, y, z = float(data[0]), float(data[1]), float(data[2])
        # 将坐标添加到列表中
        points.append([x, y, z])
# 将列表转换为NumPy数组
points_np = np.asarray(points, dtype=np.float32)
# start_time = time.time()
boundarys_points = edge_estimate(
    points_np,
    max_nn=30,
    normals_raduis=10,
    search_raduis=10,
    nb_points=20,
    angle=90,
    vis=False,
)

# save boundarys_points to txt file
with open("D:/Data/GOM_Point/M1_gom_boundary.txt", "w") as f:
    for point in boundarys_points:
        f.write(str(point[0]) + " " + str(point[1]) + " " + str(point[2]) + "\n")
