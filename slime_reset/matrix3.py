import numpy as np
import math

class Matrix3:
    NULL = np.zeros((3, 3), dtype=float)
    IDENTITY = np.eye(3, dtype=float)

    def __init__(self, xx, yx, zx, xy, yy, zy, xz, yz, zz):
        self.data = np.array([
            [xx, yx, zx],
            [xy, yy, zy],
            [xz, yz, zz]
        ])

    @staticmethod
    def from_vectors(x, y, z):
        """벡터로 행렬 초기화."""
        return Matrix3(
            x[0], y[0], z[0],
            x[1], y[1], z[1],
            x[2], y[2], z[2]
        )

    def get_column(self, index):
        return self.data[:, index]

    def get_row(self, index):
        return self.data[index, :]

    def unary_minus(self):
        return Matrix3(*(self.data * -1).flatten())

    def add(self, other):
        return Matrix3(*(self.data + other.data).flatten())

    def subtract(self, other):
        return Matrix3(*(self.data - other.data).flatten())

    def multiply_scalar(self, scalar):
        return Matrix3(*(self.data * scalar).flatten())

    def multiply_vector(self, vector):
        return np.dot(self.data, vector)

    def multiply_matrix(self, other):
        return Matrix3(*(np.dot(self.data, other.data).flatten()))

    def norm_sq(self):
        return np.sum(self.data ** 2)

    def norm(self):
        return math.sqrt(self.norm_sq())

    def det(self):
        return np.linalg.det(self.data)

    def trace(self):
        return np.trace(self.data)

    def transpose(self):
        return Matrix3(*(self.data.T).flatten())

    def inv(self):
        return Matrix3(*(np.linalg.inv(self.data).flatten()))

    def divide_scalar(self, scalar):
        return self.multiply_scalar(1.0 / scalar)

    def orthonormalize(self):
        """정규 직교화."""
        det = self.det()
        if det <= 0:
            raise ValueError("Determinant must be positive for orthonormalization")
        cur_matrix = self
        for _ in range(100):
            next_matrix = cur_matrix.add(cur_matrix.transpose().inv()).divide_scalar(2.0)
            if abs(next_matrix.det() - det) < 1e-6:
                return next_matrix
            cur_matrix = next_matrix
        return cur_matrix

    def to_quaternion(self):
        """정규 직교 행렬을 쿼터니언으로 변환."""
        trace = self.trace()
        if trace > 0:
            s = math.sqrt(trace + 1.0) * 2
            w = 0.25 * s
            x = (self.data[2, 1] - self.data[1, 2]) / s
            y = (self.data[0, 2] - self.data[2, 0]) / s
            z = (self.data[1, 0] - self.data[0, 1]) / s
        elif self.data[0, 0] > self.data[1, 1] and self.data[0, 0] > self.data[2, 2]:
            s = math.sqrt(1.0 + self.data[0, 0] - self.data[1, 1] - self.data[2, 2]) * 2
            w = (self.data[2, 1] - self.data[1, 2]) / s
            x = 0.25 * s
            y = (self.data[0, 1] + self.data[1, 0]) / s
            z = (self.data[0, 2] + self.data[2, 0]) / s
        elif self.data[1, 1] > self.data[2, 2]:
            s = math.sqrt(1.0 + self.data[1, 1] - self.data[0, 0] - self.data[2, 2]) * 2
            w = (self.data[0, 2] - self.data[2, 0]) / s
            x = (self.data[0, 1] + self.data[1, 0]) / s
            y = 0.25 * s
            z = (self.data[1, 2] + self.data[2, 1]) / s
        else:
            s = math.sqrt(1.0 + self.data[2, 2] - self.data[0, 0] - self.data[1, 1]) * 2
            w = (self.data[1, 0] - self.data[0, 1]) / s
            x = (self.data[0, 2] + self.data[2, 0]) / s
            y = (self.data[1, 2] + self.data[2, 1]) / s
            z = 0.25 * s
        return np.array([w, x, y, z])

    def to_euler_angles(self, order):
        """행렬을 오일러 각도로 변환."""
        if order == "XYZ":
            sy = math.sqrt(self.data[0, 0]**2 + self.data[1, 0]**2)
            singular = sy < 1e-6
            if not singular:
                x = math.atan2(self.data[2, 1], self.data[2, 2])
                y = math.atan2(-self.data[2, 0], sy)
                z = math.atan2(self.data[1, 0], self.data[0, 0])
            else:
                x = math.atan2(-self.data[1, 2], self.data[1, 1])
                y = math.atan2(-self.data[2, 0], sy)
                z = 0
        else:
            raise ValueError("Unsupported Euler order")
        return np.array([x, y, z])

    def __str__(self):
        return str(self.data)
