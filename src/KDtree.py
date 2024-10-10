import numpy as np
from scipy.spatial import KDTree

from .Vertex import Vertex


class KDtree:
    def __init__(self, filepath):
        self.points = np.loadtxt(
            filepath, dtype=np.float32, delimiter=" ", usecols=(0, 1, 2)
        )
        self.points_norm = np.loadtxt(
            filepath, dtype=float, delimiter=" ", usecols=(0, 1, 2, 3, 4, 5)
        )
        print(f"{len(self.points_norm)} points loaded.")

        self.tree = KDTree(self.points)
        self.search_radius = None
        self.init_vertex()

    def init_vertex(self):
        self.vertices = []
        for point in self.points_norm:
            vertex = Vertex(
                float(point[0]),
                float(point[1]),
                float(point[2]),
                float(point[3]),
                float(point[4]),
                float(point[5]),
            )
            self.vertices.append(vertex)
        # for i in range(len(self.points)):
        #     self.vertices[i].setIndex(i)

    def setR(self, radius):
        self.search_radius = radius

    def getSortedNeighbors(self, query):
        """
        获取临近点

        Args:
            query (_type_): _description_

        Returns:
            _type_: _description_
        """
        query_point = np.array([query.x, query.y, query.z])

        # 搜索最近邻
        k = 30
        dist, indices = self.tree.query(query_point, k)
        neighbors = {float(d): self.vertices[idx] for d, idx in zip(dist, indices)}

        # 取中间80%
        renew_radius = sum(dist) / k
        # renew_radius = dist[5] * 3

        return neighbors, renew_radius

        # # 指定搜索半径（即最大距离）
        # radius = dist[1] * 2.5
        # # 如果搜索半径大于指定半径，则返回所有最近邻
        # if radius > self.search_radius:
        #     return neighbors
        # else:
        #     # 返回指定半径范围内的最近邻
        #     return {k: v for k, v in neighbors.items() if k <= radius}
