class Point:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        if not isinstance(x, (int, float)):
            raise ValueError("x must be a number")
        if not isinstance(y, (int, float)):
            raise ValueError("y must be a number")
        if not isinstance(z, (int, float)):
            raise ValueError("z must be a number")
        self.x = x
        self.y = y
        self.z = z
