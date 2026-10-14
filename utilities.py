from typing import Union


class Point2D(object):
    def __init__(
        self,
        x: Union[int, float],
        y: Union[int, float],
    ):
        self._x = x
        self._y = y

    def __repr__(self) -> str:
        return f"<Point2D x={self._x:.3f}, y={self._y:.3f}>"

    @property
    def x(self) -> Union[int, float]:
        return self._x

    @property
    def y(self) -> Union[int, float]:
        return self._y


class Point3D(Point2D):
    def __init__(
        self,
        x: Union[int, float],
        y: Union[int, float],
        z: Union[int, float],
    ):
        super().__init__(x, y)
        self._z = z

    def __repr__(self) -> str:
        return f"<Point3D x={self._x:.3f}, y={self._y:.3f}, z={self._z:.3f}>"

    @property
    def z(self) -> Union[int, float]:
        return self._z
