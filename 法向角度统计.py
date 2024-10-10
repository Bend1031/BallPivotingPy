import numpy as np
import trimesh
from scipy.spatial import KDTree

mesh = trimesh.load(
    "D:/Data/GOM_Point/法线方向统计/M1_gom_norm_reverse_cxx_0.1_1.5.ply"
)
# mesh = trimesh.load("D:/Data/GOM_Point/法线方向统计/M1.ply")
vertices = mesh.vertices
faces = mesh.faces

pointsWithNormal = np.loadtxt("D:/Data/GOM_Point/M1_gom_norm_reverse.txt")
points = pointsWithNormal[:, :3]
normal = pointsWithNormal[:, 3:]
tree = KDTree(points)

# 初始化角度数组，注意维度和类型
angles = np.zeros((len(faces), 3), dtype=np.float64)

count_face = 0
with open("pyfile.txt", "w") as f:
    for i, (idx1, idx2, idx3) in enumerate(faces):
        # 提取三个顶点的法向量
        points1 = vertices[idx1]
        distance1, idx1 = tree.query(points1, k=1)
        if distance1 > 0.1:
            break
        normal1 = normal[idx1]
        # 顶点2
        points2 = vertices[idx2]
        distance2, idx2 = tree.query(points2, k=1)
        if distance2 > 0.1:
            break
        normal2 = normal[idx2]
        # 顶点3
        points3 = vertices[idx3]
        distance3, idx3 = tree.query(points3, k=1)
        if distance3 > 0.1:
            break
        normal3 = normal[idx3]

        # 归一化
        normal1 /= np.linalg.norm(normal1)
        normal2 /= np.linalg.norm(normal2)
        normal3 /= np.linalg.norm(normal3)

        # 计算三个顶点法向量之间的夹角
        angles[i, 0] = np.degrees(
            np.arccos(np.clip(np.dot(normal1, normal2), -1.0, 1.0))
        )
        angles[i, 1] = np.degrees(
            np.arccos(np.clip(np.dot(normal1, normal3), -1.0, 1.0))
        )
        angles[i, 2] = np.degrees(
            np.arccos(np.clip(np.dot(normal2, normal3), -1.0, 1.0))
        )

        if angles[i, 0] > 60 or angles[i, 1] > 60 or angles[i, 2] > 60:
            count_face += 1
            # 输出对应点坐标与法向
            # print(f"{vertices[idx1]}")
            # print(f"{pointsWithNormal[idx1]}")
            # print(f"{vertices[idx2]}")
            # print(f"{pointsWithNormal[idx2]}")
            # print(f"{vertices[idx3]}")
            # print(f"{pointsWithNormal[idx3]}")
            f.write(f"3 {idx1} {idx2} {idx3}\n")
print(count_face)
