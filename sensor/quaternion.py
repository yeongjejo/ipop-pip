from tkinter import Place

import numpy as np

from sensor.sensor_axis import SensorAxis
import torch
import articulate as art
import math

from slime_reset.vector3 import Vector3

class Quaternion(SensorAxis):
    # NULL = np.array([0.0, 0.0, 0.0, 0.0])  # [w, x, y, z]
    # IDENTITY = np.array([1.0, 0.0, 0.0, 0.0])
    # I = np.array([0.0, 1.0, 0.0, 0.0])
    # J = np.array([0.0, 0.0, 1.0, 0.0])
    # K = np.array([0.0, 0.0, 0.0, 1.0])
    raw_q_w = 0
    raw_q_x = 0
    raw_q_y = 0
    raw_q_z = 0
    
    def __init__(self, w=0.0, x=0.0, y=0.0, z=0.0):
        super().__init__(x, y, z)
        self.__w = w

    @property
    def w(self):
        return self.__w

    @w.setter
    def w(self, value):
        self.__w = value


    def __mul__(self, other):
        if not isinstance(other, Quaternion):
            raise ValueError("곱할 객체는 Quaternion이어야 합니다.")

        # 쿼터니언 곱셈 수식 적용
        w1, x1, y1, z1 = self.w, self.x, self.y, self.z
        w2, x2, y2, z2 = other.w, other.x, other.y, other.z

        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
        z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2

        return Quaternion(w, x, y, z)

    def __repr__(self):
        return f"Quaternion(w={self.w}, x={self.x}, y={self.y}, z={self.z})"
    

    def quaternion_to_rotation_matrix(self):
        tensor_q = torch.tensor([self.__w, self.x, self.y, self.z])
        return  art.math.quaternion_to_rotation_matrix(tensor_q)
    
    
    def quaternion_inverse(self):
        """
        쿼터니언의 인버스를 계산하는 함수.
        
        파라미터:
        q : list or tuple
            4개의 요소를 가지는 쿼터니언 (w, x, y, z)
        
        반환값:
        q_inv : list
            입력 쿼터니언의 인버스
        """
        
        # 쿼터니언의 크기의 제곱을 계산
        norm_sq = self.__w**2 + self.x**2 + self.y**2 + self.z**2
        
        if norm_sq == 0:
            raise ValueError("Zero norm quaternion cannot have an inverse.")
        
        # 쿼터니언의 컨주게이트를 계산
        q_conjugate = [self.__w, -self.x, -self.y, -self.z]
        
        # 컨주게이트를 크기의 제곱으로 나눈다
        q_inv = [q_conjugate[0] / norm_sq, q_conjugate[1] / norm_sq, q_conjugate[2] / norm_sq, q_conjugate[3] / norm_sq]

        self.__w = q_inv[0]
        self.x = q_inv[1]
        self.y = q_inv[2]
        self.z = q_inv[3]
        
        
    def get_xyz(self):
        return [self.x, self.y, self.z]

    def get_re(self):
        return Quaternion(self.w, 0.0, 0.0, 0.0)

    def get_im(self):
        return Quaternion(0.0, self.x, self.y, self.z)

    def negate(self):
        return Quaternion(-self.w, -self.x, -self.y, -self.z)

    def add(self, other):
        return Quaternion(
            self.w + other.w,
            self.x + other.x,
            self.y + other.y,
            self.z + other.z
        )

    def subtract(self, other):
        return Quaternion(
            self.w - other.w,
            self.x - other.x,
            self.y - other.y,
            self.z - other.z
        )

    def dot(self, other):
        return self.w * other.w + self.x * other.x + self.y * other.y + self.z * other.z

    def len_sq(self):
        return self.w**2 + self.x**2 + self.y**2 + self.z**2

    def len(self):
        return math.sqrt(self.len_sq())

    def unit(self):
        length = self.len()
        return Quaternion.NULL if length == 0 else self.divide(length)

    def sandwich(self, vector):
        """Sandwich product."""
        return self.multiply(Quaternion(0.0, vector)).divide(self).to_vector3()

    def multiply_scalar(self, scalar):
        return Quaternion(
            self.w * scalar,
            self.x * scalar,
            self.y * scalar,
            self.z * scalar
        )

    def multiply(self, other):
        if isinstance(other, Quaternion):
            return Quaternion(
                self.w * other.w - self.x * other.x - self.y * other.y - self.z * other.z,
                self.x * other.w + self.w * other.x - self.z * other.y + self.y * other.z,
                self.y * other.w + self.z * other.x + self.w * other.y - self.x * other.z,
                self.z * other.w - self.y * other.x + self.x * other.y + self.w * other.z
            )
        elif isinstance(other, float):
            return self.multiply_scalar(other)

    def xyz(self):
        return [self.x, self.y, self.z]

    def project(self, vector):
        dot_product = np.dot(self.xyz(), vector.get_xyz())
        len_squared = vector.len_sq()
        projected_vector = [dot_product / len_squared * v for v in vector.get_xyz()]
        return Quaternion(self.w, *projected_vector)

    def from_rotation_vector(self, vector):
        v = vector.divide(2)
        return Quaternion(0.0, v.x, v.y, v.z).exp()

    def inv(self):
        length_squared = self.len_sq()
        return Quaternion(self.w / length_squared, -self.x / length_squared, -self.y / length_squared, -self.z / length_squared)

    def divide(self, scalar):
        if isinstance(scalar, Quaternion):
            return self.multiply(scalar.inv())
        return self.multiply_scalar(1.0 / scalar)

    def conjugate(self):
        return Quaternion(self.w, -self.x, -self.y, -self.z)

    def log(self):
        co = self.w
        si = self.len()
        if si == 0:
            return Quaternion(math.log(si), *[c / self.w for c in self.get_xyz()])
        ang = math.atan2(si, co)
        return Quaternion(math.log(self.len()), *(ang / si * c for c in self.get_xyz()))

    def exp(self):
        ang = self.len()
        length = math.exp(self.w)
        if ang == 0:
            return Quaternion(length, *[length * c for c in self.get_xyz()])
        co = math.cos(ang)
        si = math.sin(ang)
        return Quaternion(length * co, *(length * si / ang * c for c in self.get_xyz()))

    def pow(self, t):
        return self.log().multiply_scalar(t).exp()

    def twin_nearest(self, other):
        return self.negate() if self.dot(other) < 0 else self

    def interp_q(self, other, t):
        if t == 0:
            return self
        elif t == 1:
            return other
        elif t < 0.5:
            return other.divide(self).pow(t).multiply(self)
        else:
            return self.divide(other).pow(1 - t).multiply(other)

    def interp_r(self, other, t):
        return self.interp_q(other.twin_nearest(self), t)

    def angle_about_q(self, u):
        si = sum(ui * qi for ui, qi in zip(u, [self.x, self.y, self.z]))
        co = math.sqrt(sum(ui**2 for ui in u)) * self.w
        return math.atan2(si, co)

    def to_vector3(self):
        return [self.x, self.y, self.z]

    def to_matrix(self):
        d = self.len_sq()
        return [
            [
                (self.w**2 + self.x**2 - self.y**2 - self.z**2) / d,
                2 * (self.x * self.y - self.w * self.z) / d,
                2 * (self.w * self.y + self.x * self.z) / d
            ],
            [
                2 * (self.x * self.y + self.w * self.z) / d,
                (self.w**2 - self.x**2 + self.y**2 - self.z**2) / d,
                2 * (self.y * self.z - self.w * self.x) / d
            ],
            [
                2 * (self.x * self.z - self.w * self.y) / d,
                2 * (self.w * self.x + self.y * self.z) / d,
                (self.w**2 - self.x**2 - self.y**2 + self.z**2) / d
            ]
        ]

    def __str__(self):
        return f"Quaternion(w={self.w}, x={self.x}, y={self.y}, z={self.z})"

    