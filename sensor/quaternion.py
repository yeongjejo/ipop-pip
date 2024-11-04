from tkinter import Place

from sensor.sensor_axis import SensorAxis
import torch
import articulate as art
import math

class Quaternion(SensorAxis):
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
        
        
    def norm(self):
        if self.w == 0 or self.x == 0 or self.y == 0 or self.z == 0:
            return 0
        
        
        n_data = math.sqrt(self.w**2 + self.x**2 + self.y**2 + self.z**2)
        self.w /= n_data
        self.x /= n_data
        self.y /= n_data
        self.z /= n_data
