import math

from .Point import Point


def pow2(x):
    """returns 2的x次方"""
    return 1 << x


def dist2(p1: Point, p2: Point):
    """
    计算两个点的欧几里得距离的平方。
    """
    return (p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2 + (p1.z - p2.z) ** 2


def get_common_element(set1: set, set2: set) -> set:
    """
    返回两个集合中的共同元素。
    如果没有共同元素，则抛出ValueError。
    """
    if not set1 or not set2:
        return None

    # 使用集合的交集操作来找到共同元素
    common_elements = set1.intersection(set2)
    return common_elements


def dot(v1, v2):
    """
    计算两个向量的点积。
    """
    return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2]


def cross_product(*args):
    """
    计算两个向量的叉积。
    """
    if len(args) == 2:
        x1, y1, z1 = args[0]
        x2, y2, z2 = args[1]
    elif len(args) == 6:
        x1, y1, z1 = args[0], args[1], args[2]
        x2, y2, z2 = args[3], args[4], args[5]

    out_x = y1 * z2 - z1 * y2
    out_y = z1 * x2 - x1 * z2
    out_z = x1 * y2 - y1 * x2
    return (out_x, out_y, out_z)


def normalize(v):
    """
    规范化向量v，并返回规范化的向量。
    """
    norm = math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)
    if norm == 0:
        return [0, 0, 0]
    return [v[0] / norm, v[1] / norm, v[2] / norm]


def midpoint(p1: Point, p2: Point):
    """
    计算两个点的中点。
    """
    return Point(0.5 * (p1.x + p2.x), 0.5 * (p1.y + p2.y), 0.5 * (p1.z + p2.z))
