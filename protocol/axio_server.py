import socket
import json
import threading

from data_manager import DataManager
from sensor.acc import Acc
from sensor.gyro import Gyro
from sensor.mag import Mag
from sensor.quaternion import Quaternion
from sensor.sensor_part import SensorPart


class AxioServer(threading.Thread):
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
        # 수신할 IP와 포트 설정
        UDP_IP = "127.0.0.1"
        UDP_PORT = 12345

        # 소켓 생성
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(('', UDP_PORT))

        # print(f"Listening on {UDP_IP}:{UDP_PORT}...")

        while True:
            data, addr = sock.recvfrom(9999999)  # 버퍼 크기 설정
            # print("데이터수신")
            part_name_num = {
                'Waist' : 48,
                'Back' : 49,
                'LeftUpperArm': 51,
                'LeftForeArm' : 52,
                'RightUpperArm' : 55,
                'RightForeArm' : 56,
                'LeftUpperLeg' : 59,
                'LeftLeg' : 60,
                'RightUpperLeg' : 62,
                'RightLeg' : 63,
            }
            try:
                # 바이트 데이터를 문자열로 디코딩 후 JSON 파싱
                json_data = json.loads(data.decode('utf-8'))
                # 전체 데이터 반복
                tpose_check = False
                zero_check = False
                for item in json_data:
                    name = item.get('name', 'Unknown')
                    part_num = 0
                    if name in part_name_num.keys():
                        part_num = part_name_num[name]
                    else:
                        continue
                    acc = item.get('acc', [0, 0, 0])
                    rotation = item.get('rotation', [0, 0, 0, 0])
                    tpose_check = item.get('time')
                    # print(tpose_check)

                    sensor_part = SensorPart(part_num)
                    q = Quaternion(rotation[0], rotation[1], rotation[2], rotation[3])
                    a = Acc(acc[0], acc[1], acc[2])
                    m = Mag(0.0, 0.0, 0.0)
                    g = Gyro(0.0, 0.0, 0.0)
                    # print(name, q)

                    if SensorPart.WAIST == sensor_part and rotation[0] == 1.0:
                        zero_check = True
                        break
                    # print(name, q)
                    DataManager().sensor_data = [sensor_part, [g, a, m, q]]
                # print('-'*30)

                if zero_check:
                    continue
                # print(11111)
                DataManager().setIpopIMUData()
                DataManager().axioStart = True
                DataManager().tpose_check = tpose_check

            except json.JSONDecodeError as e:
                print("JSON decode error:", e)
                print("Raw data:", data)


