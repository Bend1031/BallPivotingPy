from collections import OrderedDict

from .Octree import Octree
from .OctreeNode import OctreeNode
from .Point import Point
from .utilities import *


class OctreeIterator:
    def __init__(self, octree: Octree):
        self.octree = octree
        self.activeDepth = self.octree.getDepth()
        self.radius = octree.getSize() / pow2(self.activeDepth)
        self.sqradius = self.radius * self.radius

    def getSortedNeighbors(self, query: "Point", neighbors: dict):
        query_node = self.locatePointNode(query)
        if query_node is not None:
            octree_origin = self.octree.getOrigin()
            node_origin = query_node.getOrigin()
            node_size = query_node.getSize()
            octree_size = self.octree.getSize()

        s = query_node.getDepth()
        xloc = []
        yloc = []
        zloc = []
        xloc.append(query_node.getXloc())
        yloc.append(query_node.getYloc())
        zloc.append(query_node.getZloc())

        if (query.x - self.radius < node_origin.x) and (
            query.x - self.radius > octree_origin.x
        ):
            xloc.append(self.getXLeftCode(query_node))
        if (query.x + self.radius > node_origin.x + node_size) and (
            query.x + self.radius < octree_origin.x + octree_size
        ):
            xloc.append(self.getXRightCode(query_node))

        if (query.y - self.radius < node_origin.y) and (
            query.y - self.radius > octree_origin.y
        ):
            yloc.append(self.getYLeftCode(query_node))
        if (query.y + self.radius > node_origin.y + node_size) and (
            query.y + self.radius < octree_origin.y + octree_size
        ):
            yloc.append(self.getYRightCode(query_node))

        if (query.z - self.radius < node_origin.z) and (
            query.z - self.radius > octree_origin.z
        ):
            zloc.append(self.getZLeftCode(query_node))
        if (query.z + self.radius > node_origin.z + node_size) and (
            query.z + self.radius < octree_origin.z + octree_size
        ):
            zloc.append(self.getZRightCode(query_node))

        for xi in xloc:
            for yi in yloc:
                for zi in zloc:
                    node = self.octree.getRoot()
                    node = self.traverseToLevel(node, xi, yi, zi, s)
                    if node is not None and node.getDepth() == s:
                        self.exploreSort(node, query, neighbors)

        # 对neigbors进行排序{key:value} key从小到大
        neighbors = OrderedDict(sorted(neighbors.items(), key=lambda t: t[0]))

        return neighbors

    def setR(self, radius):
        if radius < self.octree.getSize():
            self.radius = radius
            self.sqradius = self.radius * self.radius
            import math

            self.activeDepth = int(
                self.octree.getDepth()
                - math.floor(math.log2(self.octree.getSize() / (2.0 * self.radius)))
            )
            return True
        return False

    def getR(self):
        return self.radius

    def getSquareR(self):
        return self.sqradius

    def setDepth(self, depth):
        if depth >= 0 and self.octree.getDepth() >= depth:
            self.activeDepth = depth
            self.radius = self.octree.getSize() / (2.0**depth)
            self.sqradius = self.radius * self.radius
            return True
        return False

    def getDepth(self):
        return self.activeDepth

    def getNeighbors(self, query: "Point", neighbors: dict):
        query_node = self.locatePointNode(query)
        if query_node is not None:
            octree_origin = self.octree.getOrigin()
            node_origin = query_node.getOrigin()
            node_size = query_node.getSize()
            octree_size = self.octree.getSize()

        if query_node.getDepth() == self.activeDepth:

            xloc = []
            yloc = []
            zloc = []
            xloc.append(query_node.getXloc())
            yloc.append(query_node.getYloc())
            zloc.append(query_node.getZloc())

            if (query.x - self.radius < node_origin.x) and (
                query.x - self.radius > octree_origin.x
            ):
                xloc.append(self.getXLeftCode(query_node))
            if (query.x + self.radius > node_origin.x + node_size) and (
                query.x + self.radius < octree_origin.x + octree_size
            ):
                xloc.append(self.getXRightCode(query_node))

            if (query.y - self.radius < node_origin.y) and (
                query.y - self.radius > octree_origin.y
            ):
                yloc.append(self.getYLeftCode(query_node))
            if (query.y + self.radius > node_origin.y + node_size) and (
                query.y + self.radius < octree_origin.y + octree_size
            ):
                yloc.append(self.getYRightCode(query_node))

            if (query.z - self.radius < node_origin.z) and (
                query.z - self.radius > octree_origin.z
            ):
                zloc.append(self.getZLeftCode(query_node))
            if (query.z + self.radius > node_origin.z + node_size) and (
                query.z + self.radius < octree_origin.z + octree_size
            ):
                zloc.append(self.getZRightCode(query_node))

            for xi in xloc:
                for yi in yloc:
                    for zi in zloc:
                        node = self.octree.getRoot()
                        node = self.traverseToLevel(node, xi, yi, zi, self.activeDepth)
                        if node is not None and node.getDepth() == self.activeDepth:
                            self.exploreSort(node, query, neighbors)
        else:
            s = query_node.getDepth()
            xloc = []
            yloc = []
            zloc = []
            xloc.append(query_node.getXloc())
            yloc.append(query_node.getYloc())
            zloc.append(query_node.getZloc())

            if (query.x - self.radius < node_origin.x) and (
                query.x - self.radius > octree_origin.x
            ):
                xloc.append(self.getXLeftCode(query_node))
            if (query.x + self.radius > node_origin.x + node_size) and (
                query.x + self.radius < octree_origin.x + octree_size
            ):
                xloc.append(self.getXRightCode(query_node))

            if (query.y - self.radius < node_origin.y) and (
                query.y - self.radius > octree_origin.y
            ):
                yloc.append(self.getYLeftCode(query_node))
            if (query.y + self.radius > node_origin.y + node_size) and (
                query.y + self.radius < octree_origin.y + octree_size
            ):
                yloc.append(self.getYRightCode(query_node))

            if (query.z - self.radius < node_origin.z) and (
                query.z - self.radius > octree_origin.z
            ):
                zloc.append(self.getZLeftCode(query_node))
            if (query.z + self.radius > node_origin.z + node_size) and (
                query.z + self.radius < octree_origin.z + octree_size
            ):
                zloc.append(self.getZRightCode(query_node))

            for xi in xloc:
                for yi in yloc:
                    for zi in zloc:
                        node = self.octree.getRoot()
                        node = self.traverseToLevel(node, xi, yi, zi, s)
                        if node is not None and node.getDepth() == s:
                            self.exploreSort(node, query, neighbors)

        # 对neigbors进行排序{key:value} key从小到大
        # neighbors = OrderedDict(sorted(neighbors.items(), key=lambda t: t[0]))

        return neighbors

    def getXLeftCode(self, node):
        return node.getXloc() - 0x00000001

    def getXRightCode(self, node):
        return node.getXloc() + pow2(node.getDepth())

    def getYLeftCode(self, node):
        return node.getYloc() - 0x00000001

    def getYRightCode(self, node):
        return node.getYloc() + pow2(node.getDepth())

    def getZLeftCode(self, node):
        return node.getZloc() - 0x00000001

    def getZRightCode(self, node):
        return node.getZloc() + pow2(node.getDepth())

    def computeCode(self, point: "Point"):
        multiplier = 1.0 / self.octree.getSize() * self.octree.getBinSize()
        octree_origin = self.octree.getOrigin()
        codx = int((point.x - octree_origin.x) * multiplier)
        cody = int((point.y - octree_origin.y) * multiplier)
        codz = int((point.z - octree_origin.z) * multiplier)
        return codx, cody, codz

    def traverseToLevel(self, node_ref, x_loc_code, y_loc_code, z_loc_code, k):
        current_node = node_ref  # 获取当前节点的引用
        l = current_node.getDepth() - 1

        while current_node.getDepth() > k:
            child_branch_bit = 1 << l
            child_index = (
                (((x_loc_code & child_branch_bit) >> l) << 2)
                + (((y_loc_code & child_branch_bit) >> l) << 1)
                + ((z_loc_code & child_branch_bit) >> l)
            )
            # print(child_index)

            if current_node.getChild(child_index) is not None:
                current_node = current_node.getChild(child_index)
                l -= 1
            else:
                return current_node
        return current_node

    def locatePointNode(self, point: "Point"):
        codx, cody, codz = self.computeCode(point)
        node = self.octree.getRoot()
        node = self.traverseToLevel(node, codx, cody, codz, self.activeDepth)
        return node

    def explore(self, node, query_point, exceptions, check_dict):

        if not check_dict["check"]:
            return False  # 表示找到了符合条件的点或由于某种原因停止了探索

        # 如果节点不是叶节点（深度不为0），则递归探索子节点
        if node.getDepth() != 0:

            for i in range(8):  # 每个节点最多有8个子节点
                child_node = node.getChild(i)
                if child_node is not None:
                    self.explore(child_node, query_point, exceptions, check_dict)
                    if not check_dict[
                        "check"
                    ]:  # 如果在explore中修改了check_dict["check"]，则提前退出
                        break

        # 如果节点是叶节点且包含点，则检查这些点
        elif node.getNpts() != 0:
            for point in node.points:  # 假设get_points返回一个包含点的迭代器或列表
                sqdist = dist2(query_point, point)  # 计算平方距离
                if sqdist < self.sqradius and point not in exceptions:
                    # 如果找到了符合条件的点，则标记为不应继续探索并返回True
                    check_dict["check"] = False
                    return
                    # return True

        # return False  # 如果节点是叶节点且不包含点，则返回False

    def exploreSort(
        self, node: OctreeNode, query_point: Point, neighbors: dict
    ) -> None:
        if node.getDepth() != 0:
            for i in range(8):
                if node.getChild(i) is not None:
                    self.exploreSort(node.getChild(i), query_point, neighbors)
        elif node.getNpts() != 0:
            for point in node.points:
                # 距离的平方
                dist = dist2(query_point, point)  # 假设point有x, y, z属性
                if dist < self.sqradius:
                    neighbors[dist] = point

    def containsOnly(self, center, neighbors):
        query = center
        query_node = self.locatePointNode(query)
        if query_node is not None:
            octree_origin = self.octree.getOrigin()
            node_origin = query_node.getOrigin()
            node_size = query_node.getSize()
            octree_size = self.octree.getSize()

            s = query_node.getDepth()
            xloc = []
            yloc = []
            zloc = []
            xloc.append(query_node.getXloc())
            yloc.append(query_node.getYloc())
            zloc.append(query_node.getZloc())

            if (query.x - self.radius < node_origin.x) and (
                query.x - self.radius > octree_origin.x
            ):
                xloc.append(self.getXLeftCode(query_node))
            if (query.x + self.radius > node_origin.x + node_size) and (
                query.x + self.radius < octree_origin.x + octree_size
            ):
                xloc.append(self.getXRightCode(query_node))

            if (query.y - self.radius < node_origin.y) and (
                query.y - self.radius > octree_origin.y
            ):
                yloc.append(self.getYLeftCode(query_node))
            if (query.y + self.radius > node_origin.y + node_size) and (
                query.y + self.radius < octree_origin.y + octree_size
            ):
                yloc.append(self.getYRightCode(query_node))

            if (query.z - self.radius < node_origin.z) and (
                query.z - self.radius > octree_origin.z
            ):
                zloc.append(self.getZLeftCode(query_node))
            if (query.z + self.radius > node_origin.z + node_size) and (
                query.z + self.radius < octree_origin.z + octree_size
            ):
                zloc.append(self.getZRightCode(query_node))

            for xi in xloc:
                for yi in yloc:
                    for zi in zloc:
                        node = self.octree.getRoot()
                        node = self.traverseToLevel(node, xi, yi, zi, s)
                        check_dict = {"check": True}
                        if node is not None and node.getDepth() == s:
                            self.explore(node, query, neighbors, check_dict)
                        if check_dict["check"] == False:
                            return False
        return True
