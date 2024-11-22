import math


class Vector3:
    NULL = None  # 나중에 초기화
    POS_X = None
    POS_Y = None
    POS_Z = None
    NEG_X = None
    NEG_Y = None
    NEG_Z = None

    def __init__(self, x, y, z):
        """
        3D 벡터 클래스 초기화.
        :param x: X 좌표
        :param y: Y 좌표
        :param z: Z 좌표
        """
        self.x = x
        self.y = y
        self.z = z

    def get_x(self):
        return self.x

    def get_y(self):
        return self.y

    def get_z(self):
        return self.z

    def negate(self):
        """벡터 부호 반전."""
        return Vector3(-self.x, -self.y, -self.z)

    def add(self, other):
        """벡터 덧셈."""
        return Vector3(self.x + other.x, self.y + other.y, self.z + other.z)

    def subtract(self, other):
        """벡터 뺄셈."""
        return Vector3(self.x - other.x, self.y - other.y, self.z - other.z)

    def dot(self, other):
        """벡터 내적."""
        return self.x * other.x + self.y * other.y + self.z * other.z

    def len_sq(self):
        """벡터 길이의 제곱."""
        return self.x**2 + self.y**2 + self.z**2

    def length_squared(self):
        """벡터 길이의 제곱 (별칭)."""
        return self.len_sq()

    def length(self):
        """벡터 길이."""
        return math.sqrt(self.len_sq())

    def cross(self, other):
        """벡터 외적."""
        return Vector3(
            self.y * other.z - self.z * other.y,
            self.z * other.x - self.x * other.z,
            self.x * other.y - self.y * other.x
        )

    def hadamard(self, other):
        """벡터의 각 요소를 곱한 벡터 반환."""
        return Vector3(self.x * other.x, self.y * other.y, self.z * other.z)

    def unit(self):
        """단위 벡터 반환."""
        length = self.length()
        return Vector3(0, 0, 0) if length == 0 else self.divide(length)

    def multiply(self, scalar):
        """스칼라 곱."""
        return Vector3(self.x * scalar, self.y * scalar, self.z * scalar)

    def divide(self, scalar):
        """스칼라 나눗셈."""
        return Vector3(self.x / scalar, self.y / scalar, self.z / scalar)

    def angle_to(self, other):
        """벡터 사이의 각도."""
        return math.atan2(self.cross(other).length(), self.dot(other))

    def __str__(self):
        """벡터 문자열 표현."""
        return f"Vector3(x={self.x}, y={self.y}, z={self.z})"

    @staticmethod
    def scale(scalar, vector):
        """스칼라 곱을 정적 메서드로 구현."""
        return vector.multiply(scalar)

    @staticmethod
    def divide_scalar(scalar, vector):
        """스칼라 나눗셈을 정적 메서드로 구현."""
        return vector.divide(scalar)


# 기본 정적 필드 초기화
Vector3.NULL = Vector3(0.0, 0.0, 0.0)
Vector3.POS_X = Vector3(1.0, 0.0, 0.0)
Vector3.POS_Y = Vector3(0.0, 1.0, 0.0)
Vector3.POS_Z = Vector3(0.0, 0.0, 1.0)
Vector3.NEG_X = Vector3(-1.0, 0.0, 0.0)
Vector3.NEG_Y = Vector3(0.0, -1.0, 0.0)
Vector3.NEG_Z = Vector3(0.0, 0.0, -1.0)


