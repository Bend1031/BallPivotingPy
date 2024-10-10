import time
from math import ceil, log2, pow

import numpy as np
import open3d as o3d

from .Enums import VertexType

# from scipy.spatial import KDTree
from .KDtree import KDtree
from .Mesher import Mesher
from .Octree import Octree
from .Point import Point
from .utilities import *
from .Vertex import Vertex


class FileIO:

    @staticmethod
    def edge_estimate(
        points,
        points_normals,
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
        # pcd.estimate_normals(max_nn=max_nn, radius=normals_raduis)  # 计算点云法向量
        pcd.point.normals = o3d.core.Tensor(points_normals)
        start_time = time.time()
        boundarys, mask = pcd.compute_boundary_points(
            radius=search_raduis, max_nn=nb_points, angle_threshold=angle
        )  # 边界提取的搜索半径、邻域最大点数和夹角阈值（角度制）
        # 返回mask 为True的index

        # indices = np.where(mask)[0]
        indices = np.nonzero(mask)[0]

        end_time = time.time()
        elapsed_time = end_time - start_time
        print(f"The function took {elapsed_time} second to run")
        print(
            f"Detect {boundarys.point.positions.shape[0]} bnoundary points from {pcd.point.positions.shape[0]} points."
        )
        # boundarys = boundarys.paint_uniform_color([1.0, 0.0, 0.0])
        # boundarys_points = np.array(boundarys.to_legacy().points)

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

        return indices
        # return mask.to_legacy().points
        # return boundarys_points

    @staticmethod
    def read_and_sort_points(filename: str, octree: Octree, min_radius: float) -> bool:
        try:

            with open(filename, "r") as file:
                first_line = file.readline().strip()
                words = first_line.split()
                if len(words) != 6:
                    print(
                        "Each point must be given by 6 values (position + normal): x y z nx ny nz"
                    )
                    return False

                # 将第一行数据加入到input_vertices列表中
                x, y, z, nx, ny, nz = map(float, words)
                # vertex = Vertex(x, y, z, nx, ny, nz)
                input_vertices = [Vertex(x, y, z, nx, ny, nz)]
                xmin, ymin, zmin, xmax, ymax, zmax = (
                    float("inf"),
                    float("inf"),
                    float("inf"),
                    float("-inf"),
                    float("-inf"),
                    float("-inf"),
                )

                for line in file:
                    x, y, z, nx, ny, nz = map(float, line.split())
                    input_vertices.append(Vertex(x, y, z, nx, ny, nz))

                    xmin = min(xmin, x)
                    ymin = min(ymin, y)
                    zmin = min(zmin, z)
                    xmax = max(xmax, x)
                    ymax = max(ymax, y)
                    zmax = max(zmax, z)

                print(f"{len(input_vertices)} Points read")

                lx, ly, lz = xmax - xmin, ymax - ymin, zmax - zmin
                size = max(lx, ly, lz) * 1.1

                if min_radius > 0:
                    depth = int(ceil(log2(size / min_radius)))
                    adapted_size = pow(2, depth) * min_radius
                    margin = 0.5 * (adapted_size - size)
                    size = adapted_size
                    octree.setDepth(depth)
                else:
                    margin = 0.05 * size

                ox, oy, oz = xmin - margin, ymin - margin, zmin - margin
                octree.initialize(Point(ox, oy, oz), size)
                # 初始边界点设定
                # for id in indices:
                #     # tensor2numpy
                #     input_vertices[int(id.numpy())].setType(VertexType.BOUNDARY)
                octree.addPoints(input_vertices)

            return True

        except FileNotFoundError:
            print(f"File {filename} could not be opened")
            return False

    @staticmethod
    def save_points(filename: str, octree: Octree) -> bool:
        try:
            with open(filename, "w") as file:
                save_content(octree.get_root(), file)
            return True
        except FileNotFoundError:
            print(f"File {filename} could not be opened for writing")
            return False

    @staticmethod
    def save_mesh(filename: str, mesher: Mesher):
        try:
            with open(filename, "w") as out:
                # 写入PLY文件头
                out.write("ply\n")
                out.write("format ascii 1.0\n")
                out.write("comment Mesh generated by the Ball Pivoting Algorithm\n")
                out.write(f"element vertex {mesher.nVertices()}\n")
                out.write("property float x\n")
                out.write("property float y\n")
                out.write("property float z\n")
                out.write("property float nx\n")
                out.write("property float ny\n")
                out.write("property float nz\n")
                out.write(f"element face {mesher.nFacets()}\n")
                out.write("property list uchar int vertex_indices\n")
                out.write("end_header\n")

                # 写入顶点
                for vertex in mesher.vertices:
                    out.write(
                        f"{vertex.x} {vertex.y} {vertex.z} {vertex.nx} {vertex.ny} {vertex.nz}\n"
                    )
                for facet in mesher.facets:
                    v1, v2, v3 = facet.vertexes

                    mn = [
                        v1.nx + v2.nx + v3.nx,
                        v1.ny + v2.ny + v3.ny,
                        v1.nz + v2.nz + v3.nz,
                    ]
                    mn = normalize(mn)

                    # 计算边向量
                    u = (v2.x - v1.x, v2.y - v1.y, v2.z - v1.z)
                    v = (v3.x - v1.x, v3.y - v1.y, v3.z - v1.z)
                    n = cross_product(u, v)
                    n = normalize(n)

                    # # 检查法线方向并写入顶点索引
                    if n[0] * mn[0] + n[1] * mn[1] + n[2] * mn[2] > 0:
                        out.write(f"3 {v1.index} {v2.index} {v3.index}\n")
                    else:
                        out.write(f"3 {v1.index} {v3.index} {v2.index}\n")

            return True
        except FileNotFoundError:
            print(f"Error: file {filename} could not be opened")
            return False

    # @staticmethod
    # def save_content(node, file):
    #     # 如果节点深度不为0，则递归地保存其子节点的内容
    #     if node.get_depth() != 0:
    #         for i in range(8):
    #             child = node.get_child(i)
    #             if child is not None:
    #                 save_content(child, file)
    #     # 如果节点包含点，则保存这些点
    #     elif node.get_npts() != 0:
            for point in node.get_points():  # 假设get_points()返回一个包含所有点的列表
                # 假设每个点都有一个__str__方法，用于将其转换为字符串
                file.write(str(point) + "\n")
