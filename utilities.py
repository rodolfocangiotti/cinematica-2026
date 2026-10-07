from typing import Union


class Point2D(object):
    def __init__(
        self,
        x: Union[int, float],
        y: Union[int, float],
    ):
        self.x = x
        self.y = y


class Point3D(Point2D):
    def __init__(
        self,
        x: Union[int, float],
        y: Union[int, float],
        z: Union[int, float],
    ):
        super(__class__, self).__init__(x, y)
        self.z = z
