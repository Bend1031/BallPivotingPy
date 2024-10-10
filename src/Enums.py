from enum import Enum


class VertexType(Enum):
    """
    点类型类：根据连接边状态定义

    Args:
        Enum (_type_): _description_
    """

    # 未与边相连的顶点
    ORPHAN = 0
    # 前沿点
    FRONT = 1
    # 与该点相连的边都是内部边的顶点
    INNER = 2
    # 边界点
    BOUNDARY = 3


class EdgeType(Enum):
    BORDER = 0
    FRONT = 1
    INNER = 2
