from sensor.quaternion import Quaternion
from slime_reset import NanoTimer
from slime_reset.circular_array_list import CircularArrayList


class QuaternionMovingAverage:
    def __init__(self):
        self.filtered_quaternion =  Quaternion(1, 0, 0, 0)
        self.latest_quaternion =  Quaternion(1, 0, 0, 0)
        self.smoothing_quaternion = Quaternion(1, 0, 0, 0)
        self.predict_factor = 13
        self.rot_buffer = CircularArrayList(6)  # 버퍼 크기 6
        self.fps_timer = NanoTimer()

    def update(self, test=False):
        """필터링된 쿼터니언 업데이트."""
        if self.rot_buffer.size() > 0:
            quat_buf = self.latest_quaternion  # 가장 최근의 raw 쿼터니언 데이터

            # Applies the past rotations to the current rotation
            for q in self.rot_buffer.buf:
                if q is not None:
                    quat_buf = quat_buf.multiply(q)

            # Calculate how much to slerp
            amt = self.predict_factor * self.fps_timer.get_tpf()  # 13 * tpf

            # Slerps the target rotation to that predicted rotation by amt
            self.filtered_quaternion = self.filtered_quaternion.interp_r(quat_buf, amt)

            if test:
                print(f"Filtered Quaternion: {self.filtered_quaternion}")

    def add_quaternion(self, q, test=False):
        """새로운 쿼터니언을 추가."""
        if self.rot_buffer.size() == self.rot_buffer.capacity():
            self.rot_buffer.remove_last()

        # 이전 각도와의 차이 계산
        diff_quaternion = self.latest_quaternion.inv().multiply(q)
        self.rot_buffer.add(self.rot_buffer.size(), diff_quaternion)

        self.latest_quaternion = q

    def get_fps_timer(self):
        return self.fps_timer

    def get_filtered_quaternion(self):
        return self.filtered_quaternion

