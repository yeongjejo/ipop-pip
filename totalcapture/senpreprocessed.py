import csv
import numpy as np
from typing import List, Dict
import threading

import socket, struct, time
import json

from data_manager import DataManager
from sensor.quaternion import Quaternion
from sensor.acc import Acc
from sensor.gyro import Gyro
from sensor.mag import Mag
from sensor.sensor_part import SensorPart


def quat_mul(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ], dtype=float)

def quat_inverse(q):
    w, x, y, z = q
    norm2 = np.dot(q,q)
    return np.array([w, -x, -y, -z], dtype=float) / norm2


def parse_quat_accel_csv(file_path: str):
    """
    output_quat_accel.csv 형식 파싱
    반환: frames 리스트
      frames[i][joint] = {
          'quat': np.array([x,y,z,w]),
          'accel': np.array([ax,ay,az])
      }
    """
    frames: List[Dict[str, Dict[str, np.ndarray]]] = []

    with open(file_path, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        # DictReader는 각 셀을 "헤더: 문자열"로 줌
        for row in reader:
            frame: Dict[str, Dict[str, np.ndarray]] = {}
            for col in row:
                if row[col].strip() == "":
                    continue
                if "_quat" in col:
                    joint = col.replace("_quat", "")
                    quat_vals = [float(x) for x in row[col].split()]
                    if len(quat_vals) == 4:
                        frame.setdefault(joint, {})["quat"] = np.array(quat_vals, dtype=float)
                elif "_accel" in col:
                    joint = col.replace("_accel", "")
                    accel_vals = [float(x) for x in row[col].split()]
                    if len(accel_vals) == 3:
                        frame.setdefault(joint, {})["accel"] = np.array(accel_vals, dtype=float)
            frames.append(frame)

    return frames


class TotalcaptureData(threading.Thread):
    testX = 0
    testY = 0
    testZ = 0

    def __init__(self):
        super().__init__()
        self._running = True

    def stop(self):
        self._running = False  # 스레드 종료 플래그 설정
        self.join()  # 스레드가 종료될 때까지 대기

    def run(self):
        self._running = True

        file_path = "./totalcapture/output_quat_accel.csv"  # 저장된 csv 경로
        frames = parse_quat_accel_csv(file_path)

        bone_seq = ["Hips", "Spine3", "Head", "LeftArm", "LeftForeArm", "LeftHand", "RightArm", "RightForeArm", "RightHand", "LeftUpLeg", "LeftLeg", "LeftFoot", "RightUpLeg", "RightLeg", "RightFoot"]
        sensor_part = [SensorPart.WAIST, SensorPart.BACK, SensorPart.HEAD, SensorPart.LEFT_UPPER_ARM, SensorPart.LEFT_LOWER_ARM, SensorPart.LEFT_HAND, SensorPart.RIGHT_UPPER_ARM, SensorPart.RIGHT_LOWER_ARM, SensorPart.RIGHT_HAND, SensorPart.LEFT_UPPER_LEG, SensorPart.LEFT_LOWER_LEG, SensorPart.LEFT_FOOT, SensorPart.RIGHT_UPPER_LEG, SensorPart.RIGHT_LOWER_LEG, SensorPart.RIGHT_FOOT]

        gyro = Gyro(0.0, 0.0, 0.0)
        mag = Mag(0.0, 0.0, 0.0)
        while self._running:
            for idx, f in enumerate(frames):
                time.sleep(0.016)
                send_data = []
                # print(f["RightHand"]["quat"][0] == "NaN")
                for i, bone in enumerate(bone_seq):
                    quaternion = f[bone]["quat"].tolist()
                    quaternion = Quaternion(quaternion[3], quaternion[0], quaternion[1], quaternion[2])
                    acc = f[bone]["accel"].tolist()
                    acc = Acc(acc[0], acc[1], acc[2])
                    if bone in ["LeftHand", "RightHand"]:
                        quaternion = Quaternion(1.0, 0.0, 0.0, 0.0)
                        acc = Acc(0.0, 0.0, 0.0)

                    # 센서 정보 저장
                    DataManager().sensor_data = [sensor_part[i], [gyro, acc, mag, quaternion]]

                DataManager().set_pickle_data()