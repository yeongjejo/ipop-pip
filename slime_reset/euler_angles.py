import math
from enum import Enum

from sensor.quaternion import Quaternion
from slime_reset.matrix3 import Matrix3

class EulerOrders(Enum):
    XYZ = 1
    YZX = 2
    ZXY = 3
    ZYX = 4
    YXZ = 5
    XZY = 6

class EulerAngles:
    def __init__(self, order, x, y, z):
        """
        :param order: Euler 회전 순서 (EulerOrders 열거형)
        :param x: X 축 회전 (라디안)
        :param y: Y 축 회전 (라디안)
        :param z: Z 축 회전 (라디안)
        """
        self.order = order
        self.x = x
        self.y = y
        self.z = z

    def get_order(self):
        return self.order

    def get_x(self):
        return self.x

    def get_y(self):
        return self.y

    def get_z(self):
        return self.z

    def to_quaternion(self):
        """
        이 EulerAngles와 동일한 회전을 나타내는 쿼터니언을 생성.
        :return: Quaternion 객체
        """
        cX = math.cos(self.x / 2)
        cY = math.cos(self.y / 2)
        cZ = math.cos(self.z / 2)
        sX = math.sin(self.x / 2)
        sY = math.sin(self.y / 2)
        sZ = math.sin(self.z / 2)

        if self.order == EulerOrders.XYZ:
            return Quaternion(
                cX * cY * cZ - sX * sY * sZ,
                cY * cZ * sX + cX * sY * sZ,
                cX * cZ * sY - cY * sX * sZ,
                cZ * sX * sY + cX * cY * sZ
            )
        elif self.order == EulerOrders.YZX:
            return Quaternion(
                cX * cY * cZ - sX * sY * sZ,
                cY * cZ * sX + cX * sY * sZ,
                cX * cZ * sY + cY * sX * sZ,
                cX * cY * sZ - cZ * sX * sY
            )
        elif self.order == EulerOrders.ZXY:
            return Quaternion(
                cX * cY * cZ - sX * sY * sZ,
                cY * cZ * sX - cX * sY * sZ,
                cX * cZ * sY + cY * sX * sZ,
                cZ * sX * sY + cX * cY * sZ
            )
        elif self.order == EulerOrders.ZYX:
            return Quaternion(
                cX * cY * cZ + sX * sY * sZ,
                cY * cZ * sX - cX * sY * sZ,
                cX * cZ * sY + cY * sX * sZ,
                cX * cY * sZ - cZ * sX * sY
            )
        elif self.order == EulerOrders.YXZ:
            return Quaternion(
                cX * cY * cZ + sX * sY * sZ,
                cY * cZ * sX + cX * sY * sZ,
                cX * cZ * sY - cY * sX * sZ,
                cX * cY * sZ - cZ * sX * sY
            )
        elif self.order == EulerOrders.XZY:
            return Quaternion(
                cX * cY * cZ + sX * sY * sZ,
                cY * cZ * sX - cX * sY * sZ,
                cX * cZ * sY - cY * sX * sZ,
                cZ * sX * sY + cX * cY * sZ
            )
        else:
            raise ValueError(f"Unknown Euler order: {self.order}")

    def to_matrix(self):
        """
        이 EulerAngles와 동일한 회전을 나타내는 행렬을 생성.
        :return: Matrix3 객체
        """
        cX = math.cos(self.x)
        cY = math.cos(self.y)
        cZ = math.cos(self.z)
        sX = math.sin(self.x)
        sY = math.sin(self.y)
        sZ = math.sin(self.z)

        if self.order == EulerOrders.XYZ:
            return Matrix3(
                cY * cZ, -cY * sZ, sY,
                cZ * sX * sY + cX * sZ, cX * cZ - sX * sY * sZ, -cY * sX,
                sX * sZ - cX * cZ * sY, cZ * sX + cX * sY * sZ, cX * cY
            )
        elif self.order == EulerOrders.YZX:
            return Matrix3(
                cY * cZ, sX * sY - cX * cY * sZ, cX * sY + cY * sX * sZ,
                sZ, cX * cZ, -cZ * sX,
                -cZ * sY, cY * sX + cX * sY * sZ, cX * cY - sX * sY * sZ
            )
        elif self.order == EulerOrders.ZXY:
            return Matrix3(
                cY * cZ - sX * sY * sZ, -cX * sZ, cZ * sY + cY * sX * sZ,
                cZ * sX * sY + cY * sZ, cX * cZ, sY * sZ - cY * cZ * sX,
                -cX * sY, sX, cX * cY
            )
        elif self.order == EulerOrders.ZYX:
            return Matrix3(
                cY * cZ, cZ * sX * sY - cX * sZ, cX * cZ * sY + sX * sZ,
                cY * sZ, cX * cZ + sX * sY * sZ, cX * sY * sZ - cZ * sX,
                -sY, cY * sX, cX * cY
            )
        elif self.order == EulerOrders.YXZ:
            return Matrix3(
                cY * cZ + sX * sY * sZ, cZ * sX * sY - cY * sZ, cX * sY,
                cX * sZ, cX * cZ, -sX,
                cY * sX * sZ - cZ * sY, cY * cZ * sX + sY * sZ, cX * cY
            )
        elif self.order == EulerOrders.XZY:
            return Matrix3(
                cY * cZ, -sZ, cZ * sY,
                sX * sY + cX * cY * sZ, cX * cZ, cX * sY * sZ - cY * sX,
                cY * sX * sZ - cX * sY, cZ * sX, cX * cY + sX * sY * sZ
            )
        else:
            raise ValueError(f"Unknown Euler order: {self.order}")

    def __str__(self):
        return f"EulerAngles(order={self.order}, x={self.x}, y={self.y}, z={self.z})"
