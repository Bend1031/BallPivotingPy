import math
from collections import OrderedDict, defaultdict, deque
from itertools import islice
from math import acos, pi, sqrt

from .Edge import Edge
from .Enums import EdgeType, VertexType
from .Facet import Facet
from .KDtree import KDtree
from .Octree import Octree
from .OctreeIterator import OctreeIterator
from .OctreeNode import OctreeNode
from .Point import Point
from .utilities import *
from .Vertex import Vertex


class Mesher:

    def __init__(self, octree: Octree, iterator: OctreeIterator, kdtree: KDtree):
        # 初始化成员变量
        self.octree = octree
        self.iterator = iterator
        self.ball_radius = self.iterator.getR()
        self.sq_ball_radius = self.ball_radius * self.ball_radius
        self.nfacets = 0
        self.nvertices = 0
        self.edge_front = deque()
        self.border_edges = deque()
        self.facets = []
        self.vertices = []
        self.kdtree = kdtree

    def nVertices(self):
        return self.nvertices

    def nFacets(self):
        return self.nfacets

    def nFrontEdges(self):
        return len(self.edge_front)

    def nBorderEdges(self):
        return len(self.border_edges)

    def reconstruct(self, radii):
        for radius in radii:
            self.changeRadius(radius)
            print(f"Ball radius: {self.ball_radius}")
            if len(self.edge_front) == 0:
                # isFindSeedTriangle = self.findSeedTriangle()
                isFindSeedTriangle = self.findSeedTriangleKDtree()
                if isFindSeedTriangle:
                    print("No seed triangle found, no triangulation done")
            else:
                self.expandTriangulation()

    def changeRadius(self, radius):
        self.setBallRadius(radius)

        # 创建一个新的deque来存储需要保留的边
        # new_border_edges = deque()

        # 遍历当前的border_edges
        while self.border_edges:
            e = self.border_edges.popleft()  # 或者使用pop()
            f = e.getFacet1()
            if f is not None:
                # center = Point()
                # # 此时为什么进行空球检测？
                # if self.emptyBallConfiguration(
                #     f.vertexes[0], f.vertexes[1], f.vertexes[2], center
                # ):
                #     e.setType(EdgeType.FRONT)
                #     self.edge_front.append(e)
                # else:
                #     # 如果不满足条件，将边重新添加到new_border_edges中
                #     new_border_edges.append(e)
                e.setType(EdgeType.FRONT)
                self.edge_front.append(e)

        # 将新的border_edges赋值给self.border_edges
        # self.border_edges = new_border_edges

    def setBallRadius(self, r):
        self.ball_radius = r
        self.sq_ball_radius = self.ball_radius**2

    def emptyBallConfiguration(self, v1: Vertex, v2: Vertex, v3: Vertex, center):
        """
        未知邻域点，进行搜索,多次球滚动

        Args:
            v1 (Vertex): _description_
            v2 (Vertex): _description_
            v3 (Vertex): _description_
            center (_type_): _description_

        Returns:
            _type_: _description_
        """
        # 计算球心坐标
        self.iterator.setR(self.ball_radius)
        if not self.computeBallCenter(v1, v2, v3, center=center):
            return False

        # facet_vertices = set()
        # facet_vertices.add(v1)
        # facet_vertices.add(v2)
        # facet_vertices.add(v3)

        return True

        # return self.iterator.containsOnly(center, facet_vertices)

    def fillHoles(self):
        temp_border_edges = list(self.border_edges)
        # cycles = self.find_cycles(temp_border_edges)
        # for cycle in cycles:
        #     if len(cycle) == 3:
        #         self.addFacet(Facet(cycle[0], cycle[1], cycle[2]))

        for e in temp_border_edges:
            # 在新建面的过程中，self.border_edges中边的类型会发生变化
            if e.getType() != EdgeType.BORDER:
                self.border_edges.remove(e)
                continue

            src = e.getSource()
            tgt = e.getTarget()
            v = src.findBorder(tgt)
            if v is None:
                continue
            if all(v_test.getType() == VertexType.BOUNDARY for v_test in [src, tgt, v]):
                continue
            self.addFacet(Facet(src, tgt, v))
            self.border_edges.remove(e)

    def find_cycles(self, edges):
        # 构建邻接表,无向图寻找成环的边
        graph = defaultdict(set)
        for edge in edges:
            source = edge.getSource()
            target = edge.getTarget()
            graph[source].add(target)
            graph[target].add(source)

        # DFS辅助函数
        def dfs(vertex, start, parent, path, cycles, visited, recursion_stack):
            visited.add(vertex)
            path.append(vertex)  # 追踪路径
            recursion_stack.add(vertex)

            for neighbour in graph[vertex]:
                if neighbour not in visited:
                    if dfs(
                        neighbour, start, vertex, path, cycles, visited, recursion_stack
                    ):
                        return True
                elif neighbour in recursion_stack and neighbour != parent:
                    # 使用字典记录位置减少查找开销（可选优化，此处未实现）
                    cycle = path[
                        path.index(neighbour) :
                    ]  # 从neighbour到当前vertex的路径
                    cycles.append(cycle)
                    # 避免重复搜索：标记路径上的所有节点为已访问
                    for node in cycle:
                        if node not in visited:
                            visited.add(node)
                    return True

            recursion_stack.remove(vertex)
            path.pop()  # 回溯
            return False

        # 初始化DFS所需变量
        visited = set()
        recursion_stack = set()
        cycles = []

        # 调用DFS搜索环
        for vertex in list(graph):
            if vertex not in visited:
                path = []  # 初始化路径为空
                dfs(vertex, vertex, None, path, cycles, visited, recursion_stack)

        # 返回找到的环
        return cycles

    def getFacets(self):
        return self.facets

    def getBorderEdges(self):
        return self.border_edges

    def getHoles():
        pass

    def findSeedTriangleKDtree(self):
        found = False
        for point in self.kdtree.vertices:
            # 孤立点才可以作为种子三角形
            if point.getType() == VertexType.ORPHAN:
                if self.trySeedKDtree(point):
                    found = True
                    self.expandTriangulation()
        return found

    def trySeedKDtree(self, v):
        neighbors, renew_radius = self.kdtree.getSortedNeighbors(v)
        self.setBallRadius(renew_radius)
        # 邻域点不足3个，无法形成三角形
        if len(neighbors) < 3:
            return False

        # 只需要遍历neighbors一次，用两个不同的变量表示两个不同的顶点
        for idx, vtest in enumerate(neighbors.values()):
            # 种子三角形第二个点也需是孤立点
            if vtest.getType() != VertexType.ORPHAN or vtest is v:
                continue
            values_iter = iter(neighbors.values())
            candidate = None
            for vtest2 in islice(values_iter, idx + 1, None):

                center = Point()
                if self.tryTriangleSeed(v, vtest, vtest2, neighbors, center):
                    candidate = vtest2
                    # 检查边类型，确保所有边都是活动边
                    break
            if candidate is not None:
                e1 = v.getLinkingEdge(candidate)
                e2 = vtest.getLinkingEdge(candidate)
                e3 = v.getLinkingEdge(vtest)

                # 检查e1、e2、e3这三个边（忽略空值）是否都是前向边（EdgeType.FRONT）。如果不是，就跳过当前循环的剩余部分。
                if not all(
                    edge.getType() == EdgeType.FRONT for edge in [e1, e2, e3] if edge
                ):
                    continue
                if all(
                    v.getType() == VertexType.BOUNDARY for v in [v, vtest, candidate]
                ):
                    continue
                # 三点成面，添加到面列表中，面中三点连成边
                self.addFacet(Facet(v, vtest, candidate, center))

                if self.nFacets() % 10000 == 0:
                    print(f"{self.nVertices()}vertices. Facets: {self.nFacets()}")
                    print(f"front: {len(self.edge_front)}")
                    print(f"border: {len(self.border_edges)}")

                e1 = v.getLinkingEdge(candidate)
                e2 = vtest.getLinkingEdge(candidate)
                e3 = v.getLinkingEdge(vtest)

                # 将活动边添加到edge_front中
                if e1 and e1.getType() == EdgeType.FRONT:
                    self.edge_front.appendleft(e1)
                if e2 and e2.getType() == EdgeType.FRONT:
                    self.edge_front.appendleft(e2)
                if e3 and e3.getType() == EdgeType.FRONT:
                    self.edge_front.appendleft(e3)

                if len(self.edge_front) > 0:
                    return True

        # 如果没有找到任何有效的种子三角形，则返回False
        return False

    def findSeedTriangle(self):
        found = False
        # 使用返回值来传递是否找到种子三角形的信息
        node = self.octree.getRoot()
        found = self._findSeedTriangle(node, found)
        return found

    def _findSeedTriangle(self, node: OctreeNode, found: bool):
        if node.getDepth() != 0:
            for i in range(8):
                if node.getChild(i) is not None:
                    self._findSeedTriangle(node.getChild(i), found)
        elif node.getNpts() != 0:
            for point in node.points:
                # 孤立点才可以作为种子三角形
                if point.getType() == VertexType.ORPHAN:
                    if self.trySeed(point):
                        found = True
                        self.expandTriangulation()

    def trySeed(self, v):
        # neighbors = OrderedDict()
        # # neighbors2 = OrderedDict()
        # self.iterator.setR(2.0 * self.ball_radius)
        # neighbors = self.iterator.getSortedNeighbors(v, neighbors)
        # self.iterator.setR(self.ball_radius)

        # self.kdtree.setR(2.0 * self.ball_radius)
        neighbors, renew_radius = self.kdtree.getSortedNeighbors(v)
        self.setBallRadius(renew_radius * 2)
        # 邻域点不足3个，无法形成三角形
        if len(neighbors) < 3:
            return False

        # 只需要遍历neighbors一次，用两个不同的变量表示两个不同的顶点
        for idx, vtest in enumerate(neighbors.values()):
            # 种子三角形第二个点也需是孤立点
            if vtest.getType() != VertexType.ORPHAN or vtest is v:
                continue
            values_iter = iter(neighbors.values())
            candidate = None
            for vtest2 in islice(values_iter, idx + 1, None):

                center = Point()
                if self.tryTriangleSeed(v, vtest, vtest2, neighbors, center):
                    candidate = vtest2
                    # 检查边类型，确保所有边都是活动边
                    break
            if candidate is not None:
                e1 = v.getLinkingEdge(candidate)
                e2 = vtest.getLinkingEdge(candidate)
                e3 = v.getLinkingEdge(vtest)

                # 检查e1、e2、e3这三个边（忽略空值）是否都是前向边（EdgeType.FRONT）。如果不是，就跳过当前循环的剩余部分。
                if not all(
                    edge.getType() == EdgeType.FRONT for edge in [e1, e2, e3] if edge
                ):
                    continue
                if all(
                    v.getType() == VertexType.BOUNDARY for v in [v, vtest, candidate]
                ):
                    continue
                # 三点成面，添加到面列表中，面中三点连成边
                self.addFacet(Facet(v, vtest, candidate, center))

                if self.nFacets() % 10000 == 0:
                    print(f"{self.nVertices()}vertices. Facets: {self.nFacets()}")
                    print(f"front: {len(self.edge_front)}")
                    print(f"border: {len(self.border_edges)}")

                e1 = v.getLinkingEdge(candidate)
                e2 = vtest.getLinkingEdge(candidate)
                e3 = v.getLinkingEdge(vtest)

                # 将活动边添加到edge_front中
                if e1 and e1.getType() == EdgeType.FRONT:
                    self.edge_front.appendleft(e1)
                if e2 and e2.getType() == EdgeType.FRONT:
                    self.edge_front.appendleft(e2)
                if e3 and e3.getType() == EdgeType.FRONT:
                    self.edge_front.appendleft(e3)

                if len(self.edge_front) > 0:
                    return True

        # 如果没有找到任何有效的种子三角形，则返回False
        return False

    def expandTriangulation(self):
        """沿着front边扩展三角剖分。"""

        while self.edge_front:
            edge = self.edge_front.popleft()  # 从队列中弹出一个元素

            # 不为活动边跳过
            if edge.getType() != EdgeType.FRONT:
                continue

            center = Point()  # 假设Point是一个类，这里创建一个实例
            candidate = self.findCandidateVertex(edge, center)

            if (
                candidate is None
                or candidate.getType() == VertexType.INNER
                or not candidate.isCompatibleWith(edge)
                # 所有点都为边界点
                or all(
                    v.getType() == VertexType.BOUNDARY
                    for v in [edge.getSource(), edge.getTarget(), candidate]
                )
            ):
                # 设为边界边
                edge.setType(EdgeType.BORDER)
                self.border_edges.append(edge)
                continue

            e1 = candidate.getLinkingEdge(edge.getSource())
            e2 = candidate.getLinkingEdge(edge.getTarget())

            if (e1 is not None and e1.getType() != EdgeType.FRONT) or (
                e2 is not None and e2.getType() != EdgeType.FRONT
            ):
                edge.setType(EdgeType.BORDER)
                self.border_edges.append(edge)
                continue

            self.addFacet(Facet(edge, candidate, center))

            e1 = candidate.getLinkingEdge(edge.getSource())
            e2 = candidate.getLinkingEdge(edge.getTarget())

            if e1.getType() == EdgeType.FRONT:
                self.edge_front.appendleft(e1)  # 如果e1是活动边，则将其添加到队列的开头
            if e2.getType() == EdgeType.FRONT:
                self.edge_front.appendleft(e2)  # 如果e2是活动边，则将其添加到队列的开头

            if self.nfacets % 10000 == 0:
                print(
                    f"{self.nvertices} vertices. {self.nfacets} facets. "
                    f"{len(self.edge_front)} front edges. "
                    f"{len(self.border_edges)} border edges."
                )

    def addFacet(self, face: Facet):
        self.addVertex(face.vertexes[0])
        self.addVertex(face.vertexes[1])
        self.addVertex(face.vertexes[2])

        self.facets.append(face)
        self.nfacets += 1

    def addVertex(self, v: Vertex):
        if v.index != -1:
            return

        v.setIndex(self.nvertices)
        self.vertices.append(v)
        self.nvertices += 1

    def tryTriangleSeed(
        self, v1: "Vertex", v2: "Vertex", v3: "Vertex", neighbors, center
    ):
        """
        判断三个顶点是否能构成一个有效的种子三角形。

        :param v1: 第一个顶点
        :param v2: 第二个顶点
        :param v3: 第三个顶点
        :param neighbors: 相邻顶点集合
        :param center: 球心
        :return: 如果能构成有效的种子三角形则返回True，否则返回False
        """
        # 检查v3顶点类型，需要为孤立点且和v1和v2兼容
        if (v3.getType() != VertexType.ORPHAN) or (not v3.isCompatibleWith(v1, v2)):
            return False

        # 应该都为none
        e1 = v1.getLinkingEdge(v3)
        e2 = v2.getLinkingEdge(v3)

        # 如果e1或e2有内部边，则返回False
        if (e1 is not None and e1.getType() == EdgeType.INNER) or (
            e2 is not None and e2.getType() == EdgeType.INNER
        ):
            return False

        # self.iterator.setR(self.ball_radius)
        # 如果计算球心位置
        if not self.computeBallCenter(v1, v2, v3, center):
            return False
        for v in neighbors.values():
            # 空球检测
            if v in (v1, v2, v3):
                continue
            # 假设dist2是一个全局函数，用于计算两点之间的距离的平方
            # 如果球心与v的距离小于球的半径，则返回False
            if dist2(center, v) < self.sq_ball_radius - 1e-16:
                return False

        return True

    def computeBallCenter(self, v1, v2, v3, center):
        if all(v.getType() == VertexType.BOUNDARY for v in [v1, v2, v3]):
            return False

        # 计算三角形的三边长

        c = dist2(v2, v1)
        b = dist2(v1, v3)
        a = dist2(v3, v2)

        alpha = a * (b + c - a)
        beta = b * (a + c - b)
        gamma = c * (a + b - c)
        temp = alpha + beta + gamma

        if temp < 1e-30:  # aligned case
            print("aligned case")
            return False

        alpha /= temp
        beta /= temp
        gamma /= temp

        x = alpha * v1.x + beta * v2.x + gamma * v3.x
        y = alpha * v1.y + beta * v2.y + gamma * v3.y
        z = alpha * v1.z + beta * v2.z + gamma * v3.z

        sq_circumradius = a * b * c
        a = math.sqrt(a)
        b = math.sqrt(b)
        c = math.sqrt(c)

        if self.is_narrow_triangle(a, b, c, angle_threshold_degrees=5):
            # print("该三角形为狭长三角形")
            return False

        sq_circumradius /= (a + b + c) * (b + c - a) * (c + a - b) * (a + b - c)
        # 计算假想中心到三角形的正交距离
        height = self.sq_ball_radius - sq_circumradius

        # 如果高度小于0，则无法形成包围球
        if height < 0.0:
            return False
        else:
            nx, ny, nz = self.computeNormal(v1, v2, v3)

            # 计算实际的球心（在假想中心上加上法线方向上的高度）
            height = math.sqrt(height)
            center.x = x + height * nx
            center.y = y + height * ny
            center.z = z + height * nz
            return True

    def is_narrow_triangle(self, a, b, c, angle_threshold_degrees=5):
        """
        判断三角形是否为狭长三角形，基于最小角度是否小于给定的角度阈值。

        :param a: 三角形边长a
        :param b: 三角形边长b
        :param c: 三角形边长c
        :param angle_threshold_degrees: 角度阈值，默认为5度
        :return: 如果三角形狭长（最小角度小于阈值），返回True；否则返回False
        """
        # 校验边长是否为正数及输入类型
        if not all(isinstance(side, (int, float)) and side > 0 for side in [a, b, c]):
            raise ValueError("边长必须为正数")

        # 计算角度
        def calculate_angle(side1, side2, side3):
            cos_value = (
                (side1**2 + side2**2 - side3**2) / (2 * side1 * side2)
                if 2 * side1 * side2 != 0
                else 0
            )
            return math.degrees(
                math.acos(max(-1, min(cos_value, 1)))
            )  # 简化max和min的使用

        angle_A = calculate_angle(b, c, a)
        angle_B = calculate_angle(a, c, b)
        angle_C = calculate_angle(a, b, c)

        # 找到最小角
        theta_min = min(angle_A, angle_B, angle_C)

        # 判断是否狭长
        return theta_min < angle_threshold_degrees

    def computeNormal(self, v1: Vertex, v2: Vertex, v3: Vertex):
        # 这里我们假设它们是作为返回值返回的
        u = (v2.x - v1.x, v2.y - v1.y, v2.z - v1.z)
        v = (v3.x - v1.x, v3.y - v1.y, v3.z - v1.z)
        n = cross_product(u, v)
        n = normalize(n)

        mn = [v1.nx + v2.nx + v3.nx, v1.ny + v2.ny + v3.ny, v1.nz + v2.nz + v3.nz]
        mn = normalize(mn)

        if (n[0] * mn[0] + n[1] * mn[1] + n[2] * mn[2]) < 0:
            return -n[0], -n[1], -n[2]
        return n

    def findCandidateVertex(self, edge: "Edge", candidateBallCenter: "Point"):
        src = edge.getSource()
        tgt = edge.getTarget()

        mp = midpoint(src, tgt)

        # d = self.ball_radius + sqrt(self.sq_ball_radius - dist2(mp, src))
        # neighbors = OrderedDict()
        # self.iterator.setR(d)
        # 以中点为球心计算临近点
        # neighbors = self.iterator.getNeighbors(mp, neighbors)
        # neighbors = self.iterator.getSortedNeighbors(mp, neighbors)
        # self.iterator.setR(self.ball_radius)

        neighbors, renew_radius = self.kdtree.getSortedNeighbors(mp)
        self.setBallRadius(renew_radius)
        # print(self.ball_radius)

        facet = edge.getFacet1()
        center = facet.getBallCenter()
        opp = edge.getOppositeVertex()

        vx, vy, vz = tgt.x - src.x, tgt.y - src.y, tgt.z - src.z
        vx, vy, vz = normalize([vx, vy, vz])

        ax, ay, az = center.x - mp.x, center.y - mp.y, center.z - mp.z
        ax, ay, az = normalize([ax, ay, az])

        candidate = None
        min_angle = 2.0 * pi

        # 如果候选点为已存在的三个点
        for v in neighbors.values():
            if v in (src, tgt, opp):
                continue
            if all(v_test.getType() == VertexType.BOUNDARY for v_test in [src, tgt, v]):
                continue

            new_center = Point()
            # 对于给定的 self.ball_radius，计算球是否存在
            if not self.computeBallCenter(src, tgt, v, new_center):
                continue

            bx, by, bz = new_center.x - mp.x, new_center.y - mp.y, new_center.z - mp.z
            bx, by, bz = normalize([bx, by, bz])

            cosinus = ax * bx + ay * by + az * bz
            cosinus = max(min(cosinus, 1.0), -1.0)
            # 弧度
            angle = acos(cosinus)

            cpx, cpy, cpz = cross_product(ax, ay, az, bx, by, bz)
            if cpx * vx + cpy * vy + cpz * vz < 0:
                angle = 2.0 * pi - angle

            if angle >= min_angle:
                continue

            if not self.checkEmptyBallConfiguration(src, tgt, v, neighbors, new_center):
                continue
            if all(v_test.getType() == VertexType.BOUNDARY for v_test in [src, tgt, v]):
                continue

            min_angle = angle
            candidate = v
            candidateBallCenter.x = new_center.x
            candidateBallCenter.y = new_center.y
            candidateBallCenter.z = new_center.z

        return candidate

    def checkEmptyBallConfiguration(self, v1, v2, v3, neighbors, center) -> bool:
        """
        已知邻域点，判断球是否存在

        Args:
            v1 (_type_): _description_
            v2 (_type_): _description_
            v3 (_type_): _description_
            neighbors (_type_): _description_
            center (_type_): _description_

        Returns:
            bool: _description_
        """
        for v in neighbors.values():
            # 排除自身
            if v in (v1, v2, v3):
                continue
            # 计算临近点与球心的距离
            if dist2(v, center) < self.sq_ball_radius - 1e-16:
                return False
        return True
