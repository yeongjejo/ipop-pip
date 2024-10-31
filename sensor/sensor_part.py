from enum import Enum

class SensorPart(Enum):
    WAIST = 48
    BACK = 49
    HEAD = 50
    LEFT_UPPER_ARM = 51
    LEFT_LOWER_ARM = 52
    LEFT_HAND = 53
    # LEFT_FINGER = "LEFT_FINGER"
    LEFT_SHOULDER = 54
    RIGHT_UPPER_ARM = 55
    RIGHT_LOWER_ARM = 56
    RIGHT_HAND = 57
    # RIGHT_FINGER = "RIGHT_FINGER"
    RIGHT_SHOULDER = 58
    LEFT_UPPER_LEG = 59
    LEFT_LOWER_LEG = 60
    LEFT_FOOT = 61
    # LEFT_FOOT_PAD = "LEFT_FOOT_PAD"
    RIGHT_UPPER_LEG = 62
    RIGHT_LOWER_LEG = 63
    RIGHT_FOOT = 64
    # RIGHT_FOOT_PAD = "RIGHT_FOOT_PAD"

def get_sensor_part(value):
    try:
        return SensorPart(value)
    except ValueError:
        return None  # 잘못된 값일 경우 None 반환