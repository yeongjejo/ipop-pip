import math

from sensor.quaternion import Quaternion
from slime_reset.euler_angles import EulerAngles, EulerOrders
from slime_reset.vector3 import Vector3

class NewReset:
    def __init__(self):
        self.HalfHorizontal = EulerAngles(EulerOrders.YZX, 0.0, math.pi, 0.0).to_quaternion()
        self.gyroFix =  Quaternion(1, 0, 0, 0)
        self.attachmentFix =  Quaternion(1, 0, 0, 0)
        self.mountRotFix =  Quaternion(1, 0, 0, 0)
        self.yawFix =  Quaternion(1, 0, 0, 0)
        self.tposeDownFix =  Quaternion(1, 0, 0, 0)
        self.mountingOrientation = self.HalfHorizontal
        self.yawResetSmoothTimeRemain = 0.0

    def reset_full(self, raw_q):
        """완전 초기화 메서드."""
        reference_rotation =  Quaternion(1, 0, 0, 0)

        # 장착 방향에 따라 회전을 조정
        mounting_adjusted_rotation = raw_q.multiply(self.mountingOrientation)

        # 자이로스코프 보정
        self.gyroFix = self.fix_gyroscope(mounting_adjusted_rotation.multiply(self.tposeDownFix))

        # Attachment 보정
        self.attachmentFix = self.fix_attachment(mounting_adjusted_rotation)

        # T-pose (down)를 위한 180도 회전 추가
        if self.tposeDownFix !=  Quaternion(1, 0, 0, 0):
            self.attachmentFix = self.attachmentFix.multiply(self.HalfHorizontal)

        # Yaw 보정
        self.yawFix = self.fix_yaw(mounting_adjusted_rotation, reference_rotation)



    def adjust_to_reference(self, rotation):
        """레퍼런스 회전에 맞춰 회전 조정."""
        rotation = rotation.multiply(self.mountingOrientation)
        rotation = self.gyroFix.multiply(rotation)
        rotation = rotation.multiply(self.attachmentFix)
        rotation = self.mountRotFix.inv().multiply(rotation.multiply(self.mountRotFix))
        rotation = rotation.multiply(self.tposeDownFix)
        rotation = self.yawFix.multiply(rotation)
        return rotation

    def fix_gyroscope(self, sensor_rotation):
        """자이로스코프 보정."""
        return self.get_yaw_quaternion(sensor_rotation).inv()

    def get_yaw_quaternion(self, rotation):
        """Yaw 쿼터니언 추출."""
        yaw_angle = rotation.to_euler_angles(EulerOrders.YZX).get_y()
        return EulerAngles(EulerOrders.YZX, 0.0, yaw_angle, 0.0).to_quaternion()

    def fix_attachment(self, sensor_rotation):
        """Attachment 보정."""
        return self.gyroFix.multiply(sensor_rotation).inv()

    def fix_yaw(self, sensor_rotation, reference):
        """Yaw 보정."""
        rot = self.gyroFix.multiply(sensor_rotation)
        rot = rot.multiply(self.attachmentFix)
        rot = self.mountRotFix.inv().multiply(rot.multiply(self.mountRotFix))
        rot = self.get_yaw_quaternion(rot)
        return rot.inv().multiply(reference.project(Vector3.POS_Y).unit())
