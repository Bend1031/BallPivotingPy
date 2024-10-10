from .Point import Point


class OctreeNode:
    def __init__(self, origin: Point, size: float, depth: int):
        self.parent = None
        self.children = [None] * 8
        self.nchild = None
        # number of points included in the node or in the node's children
        self.npts: int = 0
        # 节点原点
        self.origin = origin
        self.size = size
        self.depth = depth
        self.xloc: int = 0
        self.yloc: int = 0
        self.zloc: int = 0
        self.points = []

    def setSize(self, size: float):
        self.size = size

    def getSize(self) -> float:
        return self.size

    def getNpts(self) -> int:
        return self.npts

    def setNchild(self, child_number: int):
        self.nchild = child_number

    def getNchild(self) -> int:
        return self.nchild

    def setOrigin(self, pt: Point):
        self.origin = pt

    def getOrigin(self) -> Point:
        return self.origin

    def setParent(self, node):
        self.parent = node

    def getParent(self):
        return self.parent

    def getChild(self, index: int):

        return self.children[index % 8]

    def setDepth(self, depth: int):
        self.depth = depth

    def getDepth(self) -> int:
        return self.depth

    def isInside(self, p: Point, d=0.0) -> bool:
        # 实现检查点是否在节点内的逻辑
        ox, oy, oz = self.origin.x(), self.origin.y(), self.origin.z()

        # 检查点是否在节点内的逻辑，拆分条件以提高可读性
        x_inside = (p.x() >= ox - d) and (p.x() < ox + self.size + d)
        y_inside = (p.y() >= oy - d) and (p.y() < oy + self.size + d)
        z_inside = (p.z() >= oz - d) and (p.z() < oz + self.size + d)

        return x_inside and y_inside and z_inside

    def getXloc(self) -> int:
        return self.xloc

    def setXloc(self, xloc: int):
        self.xloc = xloc

    def getYloc(self) -> int:
        return self.yloc

    def setYloc(self, yloc: int):
        self.yloc = yloc

    def getZloc(self) -> int:
        return self.zloc

    def setZloc(self, zloc: int):
        self.zloc = zloc

    def addPoint(self, point: Point):
        self.points.append(point)
        self.npts += 1

    def initializeChild(self, index: int, origin: Point):
        if not (0 <= index < len(self.children)):
            raise IndexError("Child index out of range")

        size = self.size / 2
        depth = self.depth - 1
        self.children[index] = OctreeNode(origin, size, depth)
        self.children[index].setParent(self)
        self.children[index].setNchild(index)

        return self.children[index]
