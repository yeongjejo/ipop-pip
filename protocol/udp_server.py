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
        port = 55001
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(('', port))
        station_info = StationInfo()
        
        while self._running:
            buffer = bytearray(907)  # 수신할 데이터 사이즈 설정
            
            # print("000000000000000000")
            # 데이터 수신
            sock.recv_into(buffer)
            
            # print("1111111111")
            
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
                if start_byte_num > 852:
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
                # quaternion.norm()
                
                # print(f"part = {sensor_part} \t\t w: {quaternion.w}  x: {quaternion.x} y: {quaternion.y} z: {quaternion.z}  // 가속도 x: {accX}  y: {accY}  z: {accZ}")
                
                
                # if qW == 1.0 or qW == 0 or qX == 0 or qZ == 0 or accX == 0  or accY == 0  or accZ == 0:
                #     continue
                
                
                # accX = accX - qAccX 
                # accY = accY - qAccY
                # accZ = accZ * -1 - qAccZ
                
                # print(accX)
                
                
                # accX *= 9.81
                # accY *= 9.81
                # accZ *= 9.81
                
                
                # accX *= 2.5
                # accY *= 2.5
                # accZ *= 2.5
                
                
                
                # temp = quaternion.z
                # quaternion.z = quaternion.y
                # quaternion.y = temp
                # quaternion.x *= -1.0
                
                # acc.x = acc.x * -1 - qAccX
                # acc.y = acc.y *-1 - qAccY
                # acc.z = acc.z *-1 - qAccZ
                
                
                # print(acc.z)
                
                
                # acc.x *= 9.8
                # acc.y *= 9.8
                # acc.z *= 9.8
                
                
                
                # temp = quaternion.z
                # quaternion.z = quaternion.y
                # quaternion.y = temp
                # quaternion.x *= -1.0
                
                # quaternion.norm()
  
  
                qAccX = (-1.0) * 2.0 * (quaternion.x * quaternion.z - quaternion.w * quaternion.y)
                qAccY = (-1.0) * 2.0 * (quaternion.y * quaternion.z + quaternion.w * quaternion.x)
                qAccZ = 1.0 - 2.0 * (quaternion.w * quaternion.w + quaternion.z * quaternion.z)
                
                qAcc = Acc(qAccX, qAccY, qAccZ)
                # qAcc.norm()
                
                acc = Acc(0.0, 0.0, 0.0)
                acc.x = -raw_acc.x - qAcc.x
                acc.y = -raw_acc.y - qAcc.y
                acc.z = -raw_acc.z - qAcc.z                
                # acc.norm()
                
                acc.x *= 9.81
                acc.y *= 9.81
                acc.z *= 9.81


                # acc.norm()

                # # # # 센서마다 축 보정
                # if sensor_part == SensorPart.WAIST:
                    
                    # print(-raw_acc.z)
                    # print(qAcc.z)
                    # print(-raw_acc.z - qAcc.z)
                    # print(f"중력 가속도 제거 x = {acc.x} y = {acc.y} z = {acc.z}")
                    # print(f"쿼터니언 가속도 x = {qAcc.x} y = {qAcc.y} z = {qAcc.z}")
                    # print(f"222센서 가속도 x = {acc.x:.2f},\t 쿼터니언 가속도 x = {qAcc.x:.2f},\t 센서 가속도 y = {acc.y :.2f},\t 쿼터니언 가속도 y = {qAcc.y:.2f},\t 센서 가속도 z = {acc.z:.2f},\t 쿼터니언 가속도 z = {qAcc.z:.2f}")
                #     # print(f"센서 가속도 x = {accX:.5f},\t 쿼터니언 가속도 x = {qAccX:.2f},\t 센서 가속도 y = {accY:.2f},\t 쿼터니언 가속도 y = {qAccY:.2f},\t 센서 가속도 z = {accZ:.2f},\t 쿼터니언 가속도 z = {qAccZ:.2f}")
                # #     tx = accZ * -1  - qAccX
                # #     ty = accX - qAccY
                # # #     tz = accZ  * -1  - qAccZ
                #     # print(f"허리 센서 가속도 x = {accX:.2f},\t 쿼터니언 가속도 x = {qAccX:.2f},\t 센서 가속도 y = {accY:.2f},\t 쿼터니언 가속도 y = {qAccY:.2f},\t 센서 가속도 z = {accZ:.2f},\t 쿼터니언 가속도 z = {qAccZ:.2f}")
                    
                #     # print(f"허리 w: {quaternion.w}  x: {quaternion.x} y: {quaternion.y} z: {quaternion.z}  // 가속도 x: {accX}  y: {accY}  z: {accZ}")
                    
                # #     quaternion, acc = self.get_coordinate_dict(qW, qX, qY, qZ, accX, accY, accZ)[15] # 수정 해야 할수도
                    
                #     # print(f"w: {quaternion.w}  x: {quaternion.x} y: {quaternion.y} z: {quaternion.z}  // 가속도 x: {accX}  y: {accY}  z: {accZ}")
                # elif sensor_part == SensorPart.HEAD:
                #     print(f"333센서 가속도 x = {acc.x:.2f},\t 쿼터니언 가속도 x = {qAcc.x:.2f},\t 센서 가속도 y = {acc.y :.2f},\t 쿼터니언 가속도 y = {qAcc.y:.2f},\t 센서 가속도 z = {acc.z:.2f},\t 쿼터니언 가속도 z = {qAcc.z:.2f}")
                #     # print(f"머리 w: {quaternion.w}  x: {quaternion.x} y: {quaternion.y} z: {quaternion.z}  // 가속도 x: {accX}  y: {accY}  z: {accZ}")
                # #     quaternion, acc = self.get_coordinate_dict(qW, qX, qY, qZ, accX, accY, accZ)[15]
                    
                    
                # elif sensor_part == SensorPart.RIGHT_LOWER_ARM:
                # #     tempx = quaternion.x
                # #     tempy = quaternion.y
                # #     tempz = quaternion.z
                #     # quaternion.x *=  -1
                #     # quaternion.y *= -1
                #     # quaternion.z *= -1
                    
                    
                # #     quaternion.norm()
                    
                #     # acc.x *= -1
                #     # acc.y *= -1
                #     # acc.z *= -1
                    
                #     # qAccX = (-1.0) * 2.0 * (quaternion.x * quaternion.z - quaternion.w * quaternion.y)
                #     # qAccY = (-1.0) * 2.0 * (quaternion.y * quaternion.z + quaternion.w * quaternion.x)
                #     # qAccZ = 1.0 - 2.0 * (quaternion.w * quaternion.w + quaternion.z * quaternion.z)
                    
                # #     # # print(f"오른 팔 w: {quaternion.w}  x: {quaternion.x} y: {quaternion.y} z: {quaternion.z}  // 가속도 x: {accX}  y: {accY}  z: {accZ}")
                #     print(f"11센서 가속도 x = {acc.x:.2f},\t 쿼터니언 가속도 x = {qAcc.x:.2f},\t 센서 가속도 y = {acc.y:.2f},\t 쿼터니언 가속도 y = {qAcc.y:.2f},\t 센서 가속도 z = {acc.z:.2f},\t 쿼터니언 가속도 z = {qAcc.z:.2f}")
                #     # print(f"오른팔 w: {quaternion.w}  x: {quaternion.x} y: {quaternion.y} z: {quaternion.z}  // 가속도 x: {accX}  y: {accY}  z: {accZ}")
                #     # quaternion, acc = self.get_coordinate_dict(qW, qX, qY, qZ, accX, accY, accZ)[31]
                    
                # elif sensor_part == SensorPart.LEFT_LOWER_ARM:
                #     # temp = quaternion.z
                #     # quaternion.z = quaternion.y
                #     # quaternion.y = temp
                #     # quaternion.x *= -1.0
                #     # quaternion.norm()
                    

                    
                #     # acc.x = acc.x - qAccX
                #     # acc.y = acc.y *-1 - qAccZ
                #     # acc.z = acc.z *-1 - qAccY
                    
                # #     quaternion.norm()
                    
                #     print(f"44센서 가속도 x = {acc.x:.2f},\t 쿼터니언 가속도 x = {qAcc.x:.2f},\t 센서 가속도 y = {acc.y:.2f},\t 쿼터니언 가속도 y = {qAcc.y:.2f},\t 센서 가속도 z = {acc.z:.2f},\t 쿼터니언 가속도 z = {qAcc.z:.2f}")
                    # print(f"왼팔 w: {quaternion.w}  x: {quaternion.x} y: {quaternion.y} z: {quaternion.z}  // 가속도 x: {accX}  y: {accY}  z: {accZ}")
                #     quaternion, acc = self.get_coordinate_dict(qW, qX, qY, qZ, accX, accY, accZ)[37]
                #     # 25, 31, 37
                    
                # elif sensor_part == SensorPart.LEFT_LOWER_LEG:
                    
                # #     accX *= 1.25
                # #     accY *= 1.25
                #     print(f"55센서 가속도 x = {accX:.2f},\t 쿼터니언 가속도 x = {qAcc.x:.2f},\t 센서 가속도 y = {accY:.2f},\t 쿼터니언 가속도 y = {qAcc.y:.2f},\t 센서 가속도 z = {acc.z:.2f},\t 쿼터니언 가속도 z = {qAcc.z:.2f}")
                #     accZ *= 1.25
                    # print(f"왼다리 w: {quaternion.w}  x: {quaternion.x} y: {quaternion.y} z: {quaternion.z}  // 가속도 x: {accX}  y: {accY}  z: {accZ}")
                #     # quaternion, acc = self.get_coordinate_dict(qW, qX, qY, qZ, accX, accY, accZ)[25]
                #     quaternion, acc = self.get_coordinate_dict(qW, qX, qY, qZ, accX, accY, accZ)[15]
                #     # 25, 31, 37
                # elif sensor_part == SensorPart.RIGHT_LOWER_LEG:
                # #     accX *= 1.25
                # #     accY *= 1.25
                # #     accZ *= 1.25
                #     print(f"66센서 가속도 x = {acc.x:.2f},\t 쿼터니언 가속도 x = {qAcc.x:.2f},\t 센서 가속도 y = {acc.y:.2f},\t 쿼터니언 가속도 y = {qAcc.y:.2f},\t 센서 가속도 z = {acc.z:.2f},\t 쿼터니언 가속도 z = {qAcc.z:.2f}")
                    # print(f"센서 가속도 x = {accZ}, 쿼터니언 가속도 x = {qAccZ}")
                    # print(f"오른 다리 w: {quaternion.w}  x: {quaternion.x} y: {quaternion.y} z: {quaternion.z}  // 가속도 x: {accX}  y: {accY}  z: {accZ}")
                #     # quaternion, acc = self.get_coordinate_dict(qW, qX, qY, qZ, accX, accY, accZ)[1]
                #     quaternion, acc = self.get_coordinate_dict(qW, qX, qY, qZ, accX, accY, accZ)[15]
                    
                # quaternion.norm()
                # acc.norm()
                
                
                
                
                                
                # 쿼터니언 inv 보정
                # if DataManager().get_first_sensor_data_inv(sensor_part) == 0:
                #     if quaternion.w != 0:
                #         quaternion.quaternion_inverse()
                #         quaternion.norm()
                #         DataManager().first_sensor_data_inv = [sensor_part, quaternion]
                # else:
                #     quaternion = DataManager().get_first_sensor_data_inv(sensor_part) * quaternion 
                #     quaternion.norm()
                
                
                

                
                
                # 쿼터니언 축을 맞춘다
                # 축을 맞춘 쿼터니언 각으로 쿼터니언 가속도를 구한다
                # 쿼터니언 가속도 - 센서 가속도 = 0 이 되도록 센서 가속도에 -1 곱할지 여부를 결정한다 (가속도 축맞춤 작업)
                
                # 쿼터니언 가속도
                # qAccX = (-1.0)*2.0*(quaternion.x*quaternion.z-quaternion.w*quaternion.y)
                # qAccY = (-1.0)*2.0*(quaternion.y*quaternion.z+quaternion.w*quaternion.x)
                # qAccZ = 1.0-2.0*(quaternion.w*quaternion.w+quaternion.z*quaternion.z)
                
                # accQ = Acc(qAccX, qAccY, qAccZ)
                # accQ.norm()
                
                # if sensor_part == SensorPart.WAIST:
                #     acc.x = accZ - accQ.x
                #     acc.y = accY * -1 - accQ.y
                #     acc.z = accX * -1 - accQ.z
                
                #     # print(f"보정X : {acc.x :.2f},\t 보정Y : {acc.y:.2f},\t 보정Z : {acc.z:.2f},")
                #     # print(f"RawX : {accX:.2f},\t RawY : {accY:.2f},\t RawZ : {accZ:.2f},\t 보정X : {accQ.x:.2f},\t 보정Y : {accQ.y:.2f},\t 보정Z : {accQ.Z:.2f},")
                    
                # #     # acc.x = accZ
                # #     # acc.y = accY * -1
                # #     # acc.z = accX * -1
                    
                # #     # print(f"센서 가속도 x = {accX * -1}, 쿼터니언 가속도 x = {qAccZ}, 결과 {accX * -1 - qAccZ}")
                    # print(f"센서 가속도 x = {accZ}, 쿼터니언 가속도 x = {accQ.z}")
                # #     # print(f"센서 가속도 x ={accY}, 쿼터니언
                # #     # 가속도 x = {qAccZ}")
                # elif sensor_part == SensorPart.HEAD:
                #     acc.x = accZ * -1 - qAccX
                #     acc.y = accY - qAccY
                #     acc.z = accX * -1 - qAccZ
                    
                # #     # acc.x = accZ * -1
                # #     # acc.y = accY
                # #     # acc.z = accX * -1
                    
                # #     # print(f"센서 가속도 x = {accX * -1}, 쿼터니언 가속도 x = {qAccZ}, 결과 {accZ * -1 - qAccZ}")
                # elif sensor_part == SensorPart.RIGHT_LOWER_ARM:
                #     acc.x = accY - qAccX
                #     acc.y = accX * -1 - qAccY
                #     acc.z = accZ * -1 - qAccZ
                    
                # #     # acc.x = accY
                # #     # acc.y = accX * -1
                # #     # acc.z = accZ * -1
                    
                # #     # print(f"센서 가속도 x = {accZ * -1}, 쿼터니언 가속도 x = {qAccZ}, 결과 {accZ * -1 - qAccZ}")
                # elif sensor_part == SensorPart.LEFT_LOWER_ARM:
                #     acc.x = accY * -1 - qAccX
                #     acc.y = accX - qAccY
                #     acc.z = accZ * -1 - qAccZ
                    
                # #     # acc.x = accY * -1
                # #     # acc.y = accX
                # #     # acc.z = accZ * -1
                    
                
                # #     # print(f"센서 가속도 x = {accZ * -1}, 쿼터니언 가속도 x = {qAccZ}, 결과 {accZ * -1 - qAccZ}")
                    
                    
                # elif sensor_part == SensorPart.LEFT_LOWER_LEG:
                #     acc.x = accY * -1 - qAccX
                #     acc.y = accZ * -1 - qAccY
                #     acc.z = accX * -1 - qAccZ
                    
                # #     # acc.x = accY * -1
                # #     # acc.y = accZ * -1
                # #     # acc.z = accX * -1
                    
                # #     # print(f"센서 가속도 x ={accY}, 쿼터니언 가속도 x = {qAccX}, 결과 {accY * -1 - qAccX}")
                # #     # print(f"센서 가속도 x ={accZ}, 쿼터니언 가속도 x = {qAccY}, 결과 {accZ * -1 - qAccY}")
                # #     # print(f"센서 가속도 x ={accX}, 쿼터니언 가속도 x = {qAccZ}, 결과 {accX * -1 - qAccZ}")
                    
                # elif sensor_part == SensorPart.RIGHT_LOWER_LEG:
                #     # acc.x = accY - qAccX
                #     # acc.y = accZ - qAccY
                #     # acc.z = accX * -1 - qAccZ
                    
                #     #테스트용 지워도 무방
                #     acc.x = accY - qAccZ
                #     acc.y = accZ - qAccX
                #     acc.z = accX * -1 - qAccY
                #     # print(f"센서 가속도 y ={accY}, 쿼터니언 가속도 x = {qAccY}, 결과 {accY * -1 - qAccX}")
                #     # print(f"센서 가속도 z ={accZ}, 쿼터니언 가속도 y = {qAccZ}, 결과 {accZ * -1 - qAccY}")
                #     # print(f"센서 가속도 x ={accX}, 쿼터니언 가속도 z = {qAccX}, 결과 {accX * -1 - qAccY}")
                    
                #     print(f"보정X : {acc.x :.2f},\t 보정Y : {acc.y:.2f},\t 보정Z : {acc.z:.2f},")
                    # print(f"보정X : {acc.x :.2f},\t 보정Y : {acc.y:.2f},\t 보정Z : {acc.z:.2f},")
                    
                # 가속도 inv 처리
                # if DataManager().sensor_test_acc_data[sensor_part][0] == 999999999 and DataManager().sensor_test_acc_data[sensor_part][1] == 999999999 and DataManager().sensor_test_acc_data[sensor_part][2] == 999999999:
                #     DataManager().sensor_test_acc_data[sensor_part][0] = acc.x
                #     DataManager().sensor_test_acc_data[sensor_part][1] = acc.y
                #     DataManager().sensor_test_acc_data[sensor_part][2] = acc.z
                    
                # acc.x -= DataManager().sensor_test_acc_data[sensor_part][0]
                # acc.y -= DataManager().sensor_test_acc_data[sensor_part][1]
                # acc.z -= DataManager().sensor_test_acc_data[sensor_part][2]
                
                
                # acc.x *= 9.8*2
                # acc.y *= 9.8*2
                # acc.z *= 9.8*2
                
                # acc.set_abs()
                    # acc.x = accY
                    # acc.y = accZ
                    # acc.z = accX * -1
                    
                    # print(f"센서 가속도 x ={accZ}, 쿼터니언 가속도 x = {qAccY}, 결과 {accX * -1 - qAccZ}")
                
                # accX *= -1
                # accY *= -1
                # accZ *= -1
                    
                # acc.set_abs()
                    
                # if sensor_part == SensorPart.HEAD:
                #     print(f"w: {quaternion.w}  x: {quaternion.x} y: {quaternion.y} z: {quaternion.z}", end='\r', flush=True)

                # if (acc.x == 0.0 or acc.x == 1.0 or acc.y == 0.0 or acc.y == 1.0 or acc.z == 0.0 or acc.z == 1.0):
                #     continue

                
                # 센서 정보 저장
                DataManager().sensor_data = [sensor_part, [gyro, acc, mag, quaternion]]
                
            
            # print("-----------------------------")
            # if DataManager().t_pose_set_end:
            DataManager().set_pickle_data()
            
        sock.close()

    def cul_byte_data(self, sensor_data):
        int_bits = (sensor_data[3] & 0xFF) << 24 | \
                    (sensor_data[2] & 0xFF) << 16 | \
                    (sensor_data[1] & 0xFF) << 8  | \
                    (sensor_data[0] & 0xFF)

        float_value = struct.unpack('<f', struct.pack('<I', int_bits))[0]
        return float_value

    # def cul_axis_data:

    
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
    