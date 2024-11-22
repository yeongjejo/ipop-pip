import time

class NanoTimer:
    TIMER_RESOLUTION = 1_000_000_000  # 1초를 나노초로 표현
    INVERSE_TIMER_RESOLUTION = 1 / TIMER_RESOLUTION  # 초당 나노초 변환

    def __init__(self):
        self.start_time = time.time_ns()  # 타이머 시작 시간 (나노초)
        self.previous_time = self.start_time  # 이전 시간
        self.tpf = 0.0  # 프레임당 시간 (초)
        self.current_time = self.start_time  # 현재 시간

    def get_time_internal(self):
        """현재 시간을 초 단위로 반환합니다. 타이머는 0.0초에서 시작."""
        return time.time_ns() - self.start_time

    def get_time(self):
        """현재 시간을 반환합니다."""
        return self.current_time

    def get_tpf(self):
        """프레임당 시간을 반환합니다."""
        return self.tpf

    def update(self):
        """타이머를 업데이트합니다."""
        self.current_time = self.get_time_internal()  # 현재 시간 갱신
        self.tpf = (self.current_time - self.previous_time) * self.INVERSE_TIMER_RESOLUTION  # 프레임당 시간 계산
        