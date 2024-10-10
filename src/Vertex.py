from typing import Optional

from .Edge import Edge
from .Enums import EdgeType, VertexType
from .Point import Point
from .utilities import *


class Vertex(Point):
    """
    顶点类
    """

    def __init__(self, x=0.0, y=0.0, z=0.0, nx=0.0, ny=0.0, nz=0.0):
        # self.point = Point(x, y, z)
        super().__init__(x, y, z)

        self.nx = nx
        self.ny = ny
        self.nz = nz

        self.index = -1
        self.type = VertexType.ORPHAN

        self.adjacentEdges = set()
        self.adjacentFacets = set()


    def __hash__(self):
        return hash((self.x, self.y, self.z, self.nx, self.ny, self.nz))

    def addAdjacentEdge(self, edge):
        self.adjacentEdges.add(edge)

    def removeAdjacentEdge(self, edge):
        if edge in self.adjacentEdges:
            self.adjacentEdges.remove(edge)

    def addAdjacentFacet(self, facet):
        if facet not in self.adjacentFacets:
            self.adjacentFacets.add(facet)
            return True
        else:
            return False

    def removeAdjacentFacet(self, facet):
        if facet in self.adjacentFacets:
            self.adjacentFacets.remove(facet)

    def getIndex(self):
        return self.index

    def setIndex(self, index):
        self.index = index

    def getType(self):
        return self.type

    def setType(self, type):
        self.type = type

    def updateType(self):
        # 初始两种状态 VertexType.ORPHAN 、 VertexType.BOUNDARY不可互相转换
        # VertexType.ORPHAN->VertexType.FRONT->VertexType.INNER
        # VertexType.BOUNDARY->VertexType.INNER
        if len(self.adjacentEdges) == 0:
            if self.type == VertexType.BOUNDARY:
                return
            else:
                self.type = VertexType.ORPHAN
                return

        for edge in self.adjacentEdges:
            # 点存在与Front边或border边相连，可进行扩张
            if edge.getType() != EdgeType.INNER:
                if self.type == VertexType.BOUNDARY:
                    return
                self.type = VertexType.FRONT
                return
        # 如果所有边的类型都是inner，则设置类型为inner
        self.type = VertexType.INNER

    def getNx(self):
        return self.nx

    def getNy(self):
        return self.ny

    def getNz(self):
        return self.nz

    def getAdjacentEdges(self):
        return self.adjacentEdges

    def getAdjacentFacets(self):
        return self.adjacentFacets

    def getLinkingEdge(self, vertex: "Vertex") -> Optional["Edge"]:
        """
        获取两点之间连接的边

        Raises:
            ValueError: _description_

        Returns:
            _type_: _description_
        """
        common = get_common_element(self.adjacentEdges, vertex.adjacentEdges)
        # 空集合
        if not common:
            return None
        # 多于一个元素
        if len(common) > 1:
            raise ValueError("More than one common element found!")
        # 只有一个元素
        else:
            return common.pop()

    def isAdjacent(self, facet):
        return facet in self.adjacentFacets

    def findBorder(self, target_vertex: "Vertex") -> Optional["Vertex"]:
        """
        寻找可以与目标顶点形成边界环的顶点。

        Args:
            target_vertex (Vertex): 目标顶点，即边界的另一端点。

        Returns:
            Optional[Vertex]: 如果找到符合条件的顶点，则返回该顶点；否则返回None。
        """
        # 获取连接当前顶点与目标顶点的边界边
        boundary_edge = self.getLinkingEdge(target_vertex)
        if boundary_edge is None or boundary_edge.getType() != EdgeType.BORDER:
            return None

        # 获取边界边所属的facet，用于后续判断
        boundary_facet = boundary_edge.facet1

        # 遍历当前顶点的所有相邻边
        for adjacent_edge in self.adjacentEdges:
            # 只考虑边界类型的边
            if adjacent_edge.getType() != EdgeType.BORDER:
                continue

            # 获取相邻边的源顶点
            adjacent_vertex = adjacent_edge.getSource()
            if adjacent_vertex == self:
                # adjacent_vertex = adjacent_edge.getTarget()
                continue

            # 确保adjacent_vertex不在boundary_facet中
            if boundary_facet and boundary_facet.hasVertex(adjacent_vertex):
                continue

            # 检查adjacent_vertex是否与目标顶点有连接边，且该边也是边界类型
            linking_edge = adjacent_vertex.getLinkingEdge(target_vertex)
            if (
                linking_edge
                and linking_edge.getType() == EdgeType.BORDER
                and linking_edge.getSource() == target_vertex
            ):
                # 所有条件满足，返回adjacent_vertex
                return adjacent_vertex

        # 如果没有找到符合条件的顶点，返回None
        return None

    def isCompatibleWith(self, *args):
        """
        判断一个顶点是否和一条边或者两个顶点兼容(法线方向相同)。

        Args:
            *args: 可变参数列表，包含一条边或者两个顶点。
                如果是一条边，则参数为一个Edge类型的对象，表示源点和目标点组成的边。
                如果是两个顶点，则参数为两个Vertex类型的对象，分别表示源点和目标点。

        Returns:
            bool: 如果顶点与边或顶点兼容，则返回True；否则返回False。

        Raises:
            ValueError: 如果参数不符合要求，则会抛出ValueError异常。
        """
        # 提取源点和目标点
        if len(args) == 1 and isinstance(args[0], Edge):
            src, tgt = args[0].getSource(), args[0].getTarget()
        elif len(args) == 2 and all(isinstance(arg, Vertex) for arg in args):
            src, tgt = args
        else:
            raise ValueError("Invalid arguments. Expected an Edge or two Vertices.")

        # 计算叉积并归一化
        # 即为面法线方向
        v = cross_product(
            self.x - src.x,
            self.y - src.y,
            self.z - src.z,
            tgt.x - src.x,
            tgt.y - src.y,
            tgt.z - src.z,
        )
        nt = normalize(v)

        # 若面法线与候选点法线相反，调整面法线方向与点法线方向一致
        if dot(nt, [self.nx, self.ny, self.nz]) < 0:
            nt = [-x for x in nt]

        # 检查兼容性，是否面法线与三角形各个顶点法线方向相同
        return all(
            dot(nt, [vertex.nx, vertex.ny, vertex.nz]) >= 0 for vertex in [src, tgt]
        )
