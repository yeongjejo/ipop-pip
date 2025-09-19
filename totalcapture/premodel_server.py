import socket
import json
import threading


from data_manager import DataManager
from sensor.acc import Acc
from sensor.gyro import Gyro
from sensor.mag import Mag
from sensor.quaternion import Quaternion
from sensor.sensor_part import SensorPart


class PreModelServer(threading.Thread):
    def __init__(self):
        super().__init__()
        self._running = True

    def stop(self):
        self._running = False  # 스레드 종료 플래그 설정
        self.join()  # 스레드가 종료될 때까지 대기

    def run(self):
        # 수신할 IP와 포트 설정
        UDP_PORT = 5001

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
                # for i in range(1000):
                #     continue
                json_data = json.loads(data.decode('utf-8'))
                DataManager().premodel_output_q= []
                DataManager().premodel_output_vel= []
                for item in json_data:
                    DataManager().premodel_output_q.append(item['quaternion'])
                    DataManager().premodel_output_vel.append(item['velocity'])
                    DataManager().premodel_cref = item['cref']
                #     print(item['velocity'])
                # print(DataManager().premodel_output_vel)
                # print('----')

                DataManager().udp_switch = True


            except json.JSONDecodeError as e:
                print("JSON decode error:", e)
                print("Raw data:", data)

