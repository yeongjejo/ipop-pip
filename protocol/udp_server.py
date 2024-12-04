import threading
import socket
import struct

from data_manager import DataManager
from sensor.acc import Acc
from sensor.gyro import Gyro
from sensor.mag import Mag
from sensor.quaternion import Quaternion
from sensor.sensor_part import SensorPart
from station_info import StationInfo

import numpy as np


import torch
import math





class UDPServer(threading.Thread):

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
        

        # port = 56775
        port = 56476
        # port = 55000
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(('', port))
        station_info = StationInfo()
        
        while self._running:
            # buffer = bytearray(907)  # 수신할 데이터 사이즈 설정
            buffer = bytearray(949)  # 수신할 데이터 사이즈 설정
            
            # print("000000000000000000")
            # 데이터 수신
            sock.recv_into(buffer)
            
            # print("데이터 수신 확인")
            
            # 데이터 확인
            receive_station_byte_data = buffer
            
            # print(receive_station_byte_data)
            # continue
            
            # 헤더 데이터 저장
            station_info.header = [receive_station_byte_data[0], receive_station_byte_data[1]]

            # 버전 저장
            station_info.version = receive_station_byte_data[2] & 0xFF

            # 센서 데이터 저장
            # iMotion_Parsing_(PIP packet)241016_파이썬_파싱 문서 참고
            start_byte_num = 3
            
            # 0이면 t-pose 진행중
            # 1이면 동작 실행
            # print("티포즈 데이터 확인 : " + str(receive_station_byte_data[904] & 0xFF))
            # if receive_station_byte_data[904] & 0xFF == 0:
            #     DataManager().t_pose_set_end = False
            #     print("Runing T-Pose...")
            #     continue
            # elif DataManager().t_pose_set_end == False or DataManager().t_pose_set_end == None:
            #     print("End T-Pose!!!")
            #     DataManager().t_pose_set_end = True
            


            while True:
                # 모든센서 저장이 끝나면 종료
                # if start_byte_num > 852:
                if start_byte_num > 926:
                    break

                sensor_byte_data = receive_station_byte_data[start_byte_num:start_byte_num+53] # 추출할 센서 데이터
                start_byte_num += 53 # 다음 탐색할 센서의 바이트 시작 번호
                
                # 센서 번호 저장
                try:
                    sensor_part = SensorPart(sensor_byte_data[0] & 0xFF)
                except ValueError:
                    continue
                
                # 자이로 x, y, z 계산
                gyroX = self.cul_byte_data(sensor_byte_data[1:5])
                gyroY = self.cul_byte_data(sensor_byte_data[5:9])
                gyroZ = self.cul_byte_data(sensor_byte_data[9:13])
                gyro = Gyro(gyroX, gyroY, gyroZ)


                # 가속도 x, y, z 계산
                accX = self.cul_byte_data(sensor_byte_data[13:17])
                accY = self.cul_byte_data(sensor_byte_data[17:21])
                accZ = self.cul_byte_data(sensor_byte_data[21:25])
                raw_acc = Acc(accX, accY, accZ)
                acc = Acc(accX, accY, accZ)
                # acc = Acc(accY, accZ, accX)
                # acc.norm()
            
    
                # 자기계 x, y, z 계산
                magX = self.cul_byte_data(sensor_byte_data[25:29])
                magY = self.cul_byte_data(sensor_byte_data[29:33])
                magZ = self.cul_byte_data(sensor_byte_data[33:37])
                mag = Mag(magX, magY, magZ)


                # 쿼터니언 w, x, y, z 계산
                qW = self.cul_byte_data(sensor_byte_data[37:41])
                qX = self.cul_byte_data(sensor_byte_data[41:45])
                qY = self.cul_byte_data(sensor_byte_data[45:49])
                qZ = self.cul_byte_data(sensor_byte_data[49:53])
                quaternion = Quaternion(qW, qX, qY,  qZ)
                
                
                
                
                
                if sensor_part == SensorPart.LEFT_HAND:
                    # if sensor_part not in DataManager().hand_inv:
                    #     DataManager().hand_inv[sensor_part] = quaternion
                    
                    # quaternion = quaternion *  DataManager().hand_inv[sensor_part]
                    # print(DataManager().hand_inv[sensor_part])
                    # print(quaternion)
                    # print('-'*50)
                    DataManager().test_hand_q[0] = quaternion.w
                    DataManager().test_hand_q[1] = quaternion.x
                    DataManager().test_hand_q[2] = quaternion.y
                    DataManager().test_hand_q[3] = quaternion.z
                    # continue
                elif sensor_part == SensorPart.RIGHT_HAND:
                    # if sensor_part not in DataManager().hand_inv:
                    #     DataManager().hand_inv[sensor_part] = quaternion
                    
                    # quaternion = quaternion *  DataManager().hand_inv[sensor_part]
                    DataManager().test_hand_q[4] = quaternion.w
                    DataManager().test_hand_q[5] = quaternion.x
                    DataManager().test_hand_q[6] = quaternion.y
                    DataManager().test_hand_q[7] = quaternion.z
                    # continue
                
                
                # qAccX = (-1.0) * 2.0 * (quaternion.x * quaternion.z - quaternion.w * quaternion.y)
                # qAccY = (-1.0) * 2.0 * (quaternion.y * quaternion.z + quaternion.w * quaternion.x)
                # qAccZ = 1.0 - 2.0 * (quaternion.w * quaternion.w + quaternion.z * quaternion.z)
                
            
                # if sensor_part == SensorPart.LEFT_LOWER_ARM:
                #     print(accX, accY, accZ)
                #     print(qAccX, qAccY, qAccZ)
                #     print('-'*50)
                
    
            
                
                part_sequence = [SensorPart.LEFT_LOWER_ARM, SensorPart.RIGHT_LOWER_ARM, SensorPart.LEFT_LOWER_LEG, SensorPart.RIGHT_LOWER_LEG, SensorPart.HEAD, SensorPart.WAIST]
       
            

                
                # 센서 정보 저장
                DataManager().sensor_data = [sensor_part, [gyro, acc, mag, quaternion]]
            
            l_finger_a = self.cul_byte_finger_data(receive_station_byte_data[905:907])
            l_finger_b = self.cul_byte_finger_data(receive_station_byte_data[909:911])
            l_finger_c = self.cul_byte_finger_data(receive_station_byte_data[913:915])
            l_finger_d = self.cul_byte_finger_data(receive_station_byte_data[917:919])
            l_finger_e = self.cul_byte_finger_data(receive_station_byte_data[921:923])
            
            
            r_finger_a = self.cul_byte_finger_data(receive_station_byte_data[926:928])
            r_finger_b = self.cul_byte_finger_data(receive_station_byte_data[930:932])
            r_finger_c = self.cul_byte_finger_data(receive_station_byte_data[934:936])
            r_finger_d = self.cul_byte_finger_data(receive_station_byte_data[938:940])
            r_finger_e = self.cul_byte_finger_data(receive_station_byte_data[942:944])
            
            if l_finger_e != 0:
                DataManager().test_finger[0] = l_finger_e
                DataManager().test_finger[1] = l_finger_d
                DataManager().test_finger[2] = l_finger_c
                DataManager().test_finger[3] = l_finger_b
                DataManager().test_finger[4] = l_finger_a
                
                DataManager().test_finger[5] = r_finger_e
                DataManager().test_finger[6] = r_finger_d
                DataManager().test_finger[7] = r_finger_c
                DataManager().test_finger[8] = r_finger_b
                DataManager().test_finger[9] = r_finger_a
            
            
            # print("손가락 확인", l_finger_a, l_finger_b, l_finger_c, l_finger_d, l_finger_e)
            # print("손가락 확인", l_finger_a)
            # print("-----------------------------")
            # if DataManager().t_pose_set_end:
            # print(DataManager().sensor_data)
            DataManager().set_pickle_data()
            
        sock.close()

    def cul_byte_data(self, sensor_data):
        int_bits = (sensor_data[3] & 0xFF) << 24 | \
                    (sensor_data[2] & 0xFF) << 16 | \
                    (sensor_data[1] & 0xFF) << 8  | \
                    (sensor_data[0] & 0xFF)

        float_value = struct.unpack('<f', struct.pack('<I', int_bits))[0]
        return float_value

    def cul_byte_finger_data(self, sensor_data):
        int_bits = (sensor_data[0]) << 8 | (sensor_data[1])
        # int_bits = (sensor_data[1] & 0xFF) << 8  | \
        #             (sensor_data[0] & 0xFF)

        float_value = np.float16(int_bits)
        return int_bits

    
    def rotation_matrix_to_euler_angle(self, r: torch.Tensor, seq='XYZ'):
        """
        Turn rotation matrices into euler angles. (torch, batch)

        :param r: Rotation matrix tensor that can reshape to [batch_size, 3, 3].
        :param seq: 3 characters belonging to the set {'X', 'Y', 'Z'} for intrinsic
                    rotations, or {'x', 'y', 'z'} for extrinsic rotations (radians).
                    See scipy for details.
        :return: Euler angle tensor of shape [batch_size, 3].
        """
        from scipy.spatial.transform import Rotation
        rot = Rotation.from_matrix(r.clone().detach().cpu().view(-1, 3, 3).numpy())
        ret = torch.from_numpy(rot.as_euler(seq)).float().to(r.device)

        # print(ret)
        return torch.round(ret * 10000) / 10000
    
    
    def get_coordinate_dict(self, w, x, y, z, acc_x, acc_y, acc_z):
        acc_init = 1.2
        coordinate_dict = {
            # 모두 양수일 경우
            0: [Quaternion(w, x, y, z), Acc(acc_x, acc_y, acc_z)],
            1: [Quaternion(w, x, z, y), Acc(acc_x, acc_z, acc_y)],
            2: [Quaternion(w, y, x, z), Acc(acc_y, acc_x, acc_z)],
            3: [Quaternion(w, y, z, x), Acc(acc_y, acc_z, acc_x)],
            4: [Quaternion(w, z, x, y), Acc(acc_z, acc_x, acc_y)],
            5: [Quaternion(w, z, y, x), Acc(acc_z, acc_y, acc_x)],

            # x에 -1를 곱할 경우
            6: [Quaternion(w, -x, y, z), Acc(-acc_x, acc_y, acc_z)],
            7: [Quaternion(w, -x, z, y), Acc(-acc_x, acc_z, acc_y)],
            8: [Quaternion(w, y, -x, z), Acc(acc_y, -acc_x, acc_z)],
            9: [Quaternion(w, y, z, -x), Acc(acc_y, acc_z, -acc_x)],
            10: [Quaternion(w, z, -x, y), Acc(acc_z, -acc_x, acc_y)],
            11: [Quaternion(w, z, y, -x), Acc(acc_z, acc_y, -acc_x)],

            # y에 -1를 곱할 경우
            12: [Quaternion(w, x, -y, z), Acc(acc_x, -acc_y, acc_z)],
            13: [Quaternion(w, x, z, -y), Acc(acc_x, acc_z, -acc_y)],
            14: [Quaternion(w, -y, x, z), Acc(-acc_y, acc_x, acc_z)],
            15: [Quaternion(w, -y, z, x), Acc(-acc_y, acc_z, acc_x)],
            16: [Quaternion(w, z, x, -y), Acc(acc_z, acc_x, -acc_y)],
            17: [Quaternion(w, z, -y, x), Acc(acc_z, -acc_y, acc_x)],

            # z에 -1를 곱할 경우
            18: [Quaternion(w, x, y, -z), Acc(acc_x, acc_y, -acc_z)],
            19: [Quaternion(w, x, -z, y), Acc(acc_x, -acc_z, acc_y)],
            20: [Quaternion(w, y, x, -z), Acc(acc_y, acc_x, -acc_z)],
            21: [Quaternion(w, y, -z, x), Acc(acc_y, -acc_z, acc_x)],
            22: [Quaternion(w, -z, x, y), Acc(-acc_z, acc_x, acc_y)],
            23: [Quaternion(w, -z, y, x), Acc(-acc_z, acc_y, acc_x)],

            # x, y에 -1를 곱할 경우
            24: [Quaternion(w, -x, -y, z), Acc(-acc_x, -acc_y, acc_z)],
            25: [Quaternion(w, -x, z, -y), Acc(-acc_x, acc_z, -acc_y)],
            26: [Quaternion(w, -y, -x, z), Acc(-acc_y, -acc_x, acc_z)],
            27: [Quaternion(w, -y, z, -x), Acc(-acc_y, acc_z, -acc_x)],
            28: [Quaternion(w, z, -x, -y), Acc(acc_z, -acc_x, -acc_y)],
            29: [Quaternion(w, z, -y, -x), Acc(acc_z, -acc_y, -acc_x)],

            # x, z에 -1를 곱할 경우
            30: [Quaternion(w, -x, y, -z), Acc(-acc_x, acc_y, -acc_z)],
            31: [Quaternion(w, -x, -z, y), Acc(-acc_x, -acc_z, acc_y)],
            32: [Quaternion(w, y, -x, -z), Acc(acc_y, -acc_x, -acc_z)],
            33: [Quaternion(w, y, -z, -x), Acc(acc_y, -acc_z, -acc_x)],
            34: [Quaternion(w, -z, -x, y), Acc(-acc_z, -acc_x, acc_y)],
            35: [Quaternion(w, -z, y, -x), Acc(-acc_z, acc_y, -acc_x)],

            # y, z에 -1를 곱할 경우
            36: [Quaternion(w, x, -y, -z), Acc(acc_x, -acc_y, -acc_z)],
            37: [Quaternion(w, x, -z, -y), Acc(acc_x, -acc_z, -acc_y)],
            38: [Quaternion(w, -y, x, -z), Acc(-acc_y, acc_x, -acc_z)],
            39: [Quaternion(w, -y, -z, x), Acc(-acc_y, -acc_z, acc_x)],
            40: [Quaternion(w, -z, x, -y), Acc(-acc_z, acc_x, -acc_y)],
            41: [Quaternion(w, -z, -y, x), Acc(-acc_z, -acc_y, acc_x)],

            # 전부 -1를 곱할 경우
            42: [Quaternion(w, -x, -y, -z), Acc(-acc_x, -acc_y, -acc_z)],
            43: [Quaternion(w, -x, -z, -y), Acc(-acc_x, -acc_z, -acc_y)],
            44: [Quaternion(w, -y, -x, -z), Acc(-acc_y, -acc_x, -acc_z)],
            45: [Quaternion(w, -y, -z, -x), Acc(-acc_y, -acc_z, -acc_x)],
            46: [Quaternion(w, -z, -x, -y), Acc(-acc_z, -acc_x, -acc_y)],
            47: [Quaternion(w, -z, -y, -x), Acc(-acc_z, -acc_y, -acc_x)],
        
        }
        return coordinate_dict
    
    
    