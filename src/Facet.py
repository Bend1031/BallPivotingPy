from .Edge import Edge
from .Point import Point
from .Vertex import Vertex


class Facet:
    def __init__(self, *args):
        self.vertexes = [None] * 3
        self.ballCenter = None

        if (
            3 <= len(args) <= 4
            and all(isinstance(v, Vertex) for v in args[:3])
            and (len(args) != 4 or isinstance(args[3], Point))
        ):
            self._init_from_vertices(*args[:3])
            if len(args) == 4:
                self.ballCenter = args[3]
        # Handle initialization from edge and vertex with optional ballCenter
        elif (
            2 <= len(args) <= 3
            and isinstance(args[0], Edge)
            and isinstance(args[1], Vertex)
            and (len(args) != 3 or isinstance(args[2], Point))
        ):
            self._init_from_edge_vertex(args[0], args[1])
            if len(args) == 3:
                self.ballCenter = args[2]
        else:
            raise ValueError("Invalid arguments for Facet constructor")

    def _init_from_vertices(self, v1, v2, v3):
        """
        从三个顶点初始化三角形对象。

        Args:
            v1 (Vertex): 第一个顶点坐标。
            v2 (Vertex): 第二个顶点坐标。
            v3 (Vertex): 第三个顶点坐标。

        Returns:
            None

        """
        # 点添加到面
        self.vertexes = [v1, v2, v3]
        self._init_edges_and_facets([v1, v2, v3])

    def _init_from_edge_vertex(self, edge, vertex):
        """
        根据边和顶点初始化实例。

        Args:
            edge (Edge): 用于初始化实例的边对象。
            vertex (Vertex): 用于初始化实例的顶点对象，假设它既不是边的源顶点也不是目标顶点。

        Returns:
            None: 此函数没有返回值，而是直接修改实例的状态。

        """
        src, tgt = edge.getSource(), edge.getTarget()
        self.vertexes = [src, vertex, tgt]
        self._init_edges_for_edge_vertex_constructor(edge)

    def _init_edges_and_facets(self, vertices):
        for i in range(3):
            j = (i + 1) % 3
            e = vertices[i].getLinkingEdge(vertices[j])
            if e is None:
                # 点添加到边
                e = Edge(vertices[i], vertices[j])
            # 面添加到边
            e.addAdjacentFacet(self)
            # 面添加到点
            self.vertexes[i].addAdjacentFacet(self)
            self.vertexes[i].updateType()

    def _init_edges_for_edge_vertex_constructor(self, edge):
        edge.addAdjacentFacet(self)
        for i in range(2):
            j = (i + 1) % 3
            e = self.vertexes[i].getLinkingEdge(self.vertexes[j])
            if e is None:
                e = Edge(self.vertexes[i], self.vertexes[j])
            e.addAdjacentFacet(self)
        for vertex in self.vertexes:
            vertex.addAdjacentFacet(self)
            vertex.updateType()

    def setBallCenter(self, point):
        self.ballCenter = point

    def getBallCenter(self):
        return self.ballCenter

    def hasVertex(self, v):
        return v in self.vertexes

    def vertex(self, i):
        return self.vertexes[i % 3]

    def edge(self, i):
        return self.vertexes[(i + 1) % 3].getLinkingEdge(self.vertexes[(i + 2) % 3])
