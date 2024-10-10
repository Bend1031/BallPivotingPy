from .OctreeNode import OctreeNode
from .Point import Point
from .utilities import *


class Octree:

    def __init__(
        self,
        origin=Point(),
        size=0,
        depth=0,
    ):
        self.depth = depth
        self.origin = origin
        self.size = size
        # self.binsize = 2**depth
        self.binsize = pow2(depth)  # 假设binsize是2的depth次方
        self.npoints = 0
        self.root = OctreeNode(Point(), 0, 0)
        self.root.setDepth(depth)
        # number of non-empty cells per level
        self.non_empty_cells = [0] * depth

    def addPoints(self, input_vertices):
        # 添加点到Octree
        for vertex in input_vertices:
            self.addPoint(vertex)
        return self.npoints

    def addPoint(self, pt):
        # 计算点的位置码
        codx = int((pt.x - self.origin.x) / self.size * self.binsize)
        cody = int((pt.y - self.origin.y) / self.size * self.binsize)
        codz = int((pt.z - self.origin.z) / self.size * self.binsize)

        node = self.getRoot()
        level = node.depth - 1

        # 遍历八叉树直到叶子节点
        while node.depth != 0:
            child_branch_bit = 1 << level
            x = (codx & child_branch_bit) >> level
            y = (cody & child_branch_bit) >> level
            z = (codz & child_branch_bit) >> level
            childIndex = (x << 2) + (y << 1) + z

            if node.getChild(childIndex) is None:
                childSize = node.getSize() / 2.0
                childDepth = node.getDepth() - 1
                origin = node.getOrigin()
                childOrigin = Point(
                    origin.x + x * childSize,
                    origin.y + y * childSize,
                    origin.z + z * childSize,
                )

                child = node.initializeChild(childIndex, childOrigin)

                child.setXloc(node.getXloc() + (x << childDepth))
                child.setYloc(node.getYloc() + (y << childDepth))
                child.setZloc(node.getZloc() + (z << childDepth))
                self.non_empty_cells[childDepth] += 1

            node = node.getChild(childIndex)
            level -= 1

        # 在叶子节点中添加点
        node.addPoint(pt)
        self.npoints += 1

    def printOctreeStat(self):
        size = self.size
        for i in range(self.depth - 1, -1, -1):
            print(
                f"level {i}: {size} ; mean number of points: "
                f"{self.npoints / self.non_empty_cells[i]:.2f}"
            )
            size /= 2

    def setDepth(self, depth: int):
        # 设置Octree的深度
        self.depth = depth
        self.binsize = pow2(depth)  # 假设binsize是2的depth次方
        self.non_empty_cells = [0] * depth

    def getDepth(self):
        # 获取Octree的深度
        return self.depth

    def initialize(self, origin, size: float):
        # 初始化Octree

        self.origin = origin
        self.size = size

        self.root = OctreeNode(self.origin, self.size, self.depth)
        self.root.setXloc(0)
        self.root.setYloc(0)
        self.root.setZloc(0)
        self.root.setParent(None)

    def getOrigin(self):
        # 获取Octree的origin
        return self.origin

    def getRoot(self):
        return self.root

    def getSize(self):
        return self.size

    def setSize(self, size):
        self.size = size

    def getBinSize(self):
        return self.binsize

    def getNodes(self, depth, starting_node, nodes):
        if starting_node.getDepth() == depth:
            nodes.append(starting_node)
        else:
            for i in range(8):
                child = starting_node.getChild(i)
                if child is not None:
                    self.getNodes(depth, child, nodes)

    def getNodesByChildCount(self, depth, starting_node, node_collection):
        # 首先收集特定深度的所有节点
        nodes_at_depth = []
        self.getNodes(depth, starting_node, nodes_at_depth)

        # 然后根据每个节点的子节点数量进行分类
        # 注意：这里我们假设子节点数量的范围是0到8（包括0和8，但通常八叉树不会有8个子节点）
        # 如果子节点数量可能超过8，则需要调整逻辑
        for node in nodes_at_depth:
            nchild = sum(1 for child in node.children if child is not None)
            if nchild < len(node_collection):
                node_collection[nchild].append(node)
            else:
                # 如果子节点数量超过预期，可以选择忽略或添加到最后一个列表中
                # 这里我们选择添加到最后一个列表中，但请注意这可能不是最佳做法
                node_collection[-1].append(node)

    def getNpoints(self):
        return self.npoints
