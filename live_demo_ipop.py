import torch
import struct
from pygame.time import Clock
from log_test import rotation_matrix_to_quaternion
from net import PIP
import articulate as art
import os
from config import *
#import keyboard
from data_manager import DataManager
from noitom_log import get_noitom_log_test_data
from protocol.udp_server import UDPServer
from protocol.udp_station_broadcast_receiver import UDPStationBroadcastReceiver
import time
import socket
import numpy as np
import matplotlib.pyplot as plt
import math

from protocol.xsesn_udp_server import XsensUDPServer

from sensor.quaternion import Quaternion
from sensor.sensor_part import SensorPart
from xsens_log import get_xsens_log_test_data, test_num

class IMUSet:
    g = 9.8
    test_i = 0

    def __init__(self, test):

        self.n_imus = 0
        self.test = test


    def get_noitom(self):
        try:
            data, addr = self.cs.recvfrom(172)
            format = '>f'

            parsed_data = [struct.unpack(format, data[i:i + 4])[0] for i in range(2, len(data) - 2, 4)]

            np_data = np.array(parsed_data)

            q, a = [], []
            for i in range(0, 42, 7):
                q.append([np_data[i], np_data[i + 1], np_data[i + 2], np_data[i + 3]])
                a.append([np_data[i + 4], np_data[i + 5], np_data[i + 6]])
                #quat = torch.tensor([np_data[i], np_data[i + 1], np_data[i + 2], np_data[i + 3]])
                #acc = torch.tensor([np_data[i + 4], np_data[i + 5], np_data[i + 6]])
                #q.append(quat)
                #a.append(acc)

            R = art.math.quaternion_to_rotation_matrix(torch.tensor(q))  # rotation is not changed
            a = -torch.tensor(a) / 1000 * self.g  # acceleration is reversed
            a = R.bmm(a.unsqueeze(-1)).squeeze(-1) + torch.tensor([0., 0., self.g])  # calculate global free acceleration
            return R, a

        except socket.timeout:
            print('[warning] no imu data received for 3 seconds')

    def get_ipop(self):
        if self.test:
            test_a_list, test_q_list, _ = DataManager().process_dipimu()
            
            # test_a_list, test_q_list, _ = get_xsens_log_test_data()

            q = test_q_list[self.test_i]
            a = test_a_list[self.test_i]
            
            
            self.test_i += 0
            
            # print("1"*90)
            # a = -torch.tensor(a) / 1000 * 9.8                        # acceleration is reversed
            # a = q.bmm(a.unsqueeze(-1)).squeeze(-1) + torch.tensor([0., 0., 9.8])   # calculate global free acceleration

            
            return q, a

        q = DataManager().test_q
        r = DataManager().test_r
        a = DataManager().test_acc
        # a = -torch.tensor(a) / 1000 * 9.8  # acceleration is reversed
        # a = r.bmm(a.unsqueeze(-1)).squeeze(-1) + torch.tensor([0., 0., 9.8])  # calculate global free acceleration

        # print(q)
        
        
	
        return r, a



    def clear(self):
        pass
def tpose_calibration_ipop_2023():
    #c = input('Used cached RMI? [y]/n    (If you choose no, put imu 1 straight (x = Forward, y = Left, z = Up).')
    #if c == 'n' or c == 'N':
    #    imu_set.clear()
    #    RSI = art.math.quaternion_to_rotation_matrix(torch.tensor(imu_set.get_ipop()[0][0])).view(3, 3).t()
    #    RMI = torch.tensor([[0, 1, 0], [0, 0, 1], [1, 0, 0]], dtype=torch.double).mm(RSI)
    #    torch.save(RMI, os.path.join(paths.temp_dir, 'RMI.pt'))
    #else:
    #    RMI = torch.load(os.path.join(paths.temp_dir, 'RMI.pt'))

    imu_set.clear()
    RSI = imu_set.get_ipop()[0][5].view(3, 3).t()
    # print(imu_set.get_ipop())
    RMI = torch.tensor([[0, 1, 0], [0, 0, 1], [1, 0, 0.]]).mm(RSI)
    # print(imu_set.get_ipop())
    # print('-----------------------')
    torch.save(RMI, os.path.join(paths.temp_dir, 'RMI.pt'))

    #input('Stand straight in T-pose and press enter. The calibration will begin in 3 seconds')
    time.sleep(3)
    imu_set.clear()
    RIS = imu_set.get_ipop()[0]
    RSB = RMI.matmul(RIS).transpose(1, 2).matmul(torch.eye(3))
    return RMI, RSB


def tpose_calibration_ipop_2024(test, imu_set):
    RSI = imu_set.get_ipop()[0][5].view(3, 3).t()

    # print(f'RSI.shape: {RSI.shape}\nRSI:\n{RSI}')

    if test:
        # RMI = torch.eye(3)
        RMI = torch.tensor([[0, 1, 0], [0, 0, 1], [1, 0, 0.]]).mm(RSI)

    else:
        # RMI = torch.eye(3)
        RMI = torch.tensor([[1, 0, 0], [0, 0, 1], [0, 1, 0.]]).mm(RSI)
        
        
    RIS = imu_set.get_ipop()[0]
    
    RSB = RMI.matmul(RIS).transpose(1, 2).matmul(torch.eye(3))  # [6, 3, 3]

    test = torch.eye(3).mm(RSB[0])
    # print(np.linalg.det(test))


    return RMI, RSB


  
  
def test_mode():
    g_x = []
    g_y1 = []
    g_y2 = []
    g_y3 = []
    g_y4 = []
    
    # 실시간 센서 코드!!!!!!!!!!!!

    i_test = 0
    
    imu_set = IMUSet(True)
    net = PIP()
    # print("121212121")
    RMI, RSB = tpose_calibration_ipop_2024(True, imu_set)
    #RMI, RSB = tpose_calibration_noitom()
    imu_set.clear()
    clock = Clock()
    start_time = 10000
    # 테스트 코드!!!!!!!!!!!!
    # test_a_list, test_q_list, pose_gt = get_xsens_log_test_data()
    # test_a_list, test_q_list, pose_gt = get_noitom_log_test_data()
    test_a_list, test_q_list, pose_gt = DataManager().process_dipimu()
    
    # print(test_a_list[309])
    
    # print(test_q_list[309])
    
    # for i in range(len(test_q_list[:600])):
    for i in range(len(test_q_list[:])):
        # clock.tick(800)
        # time.sleep(0.05)
    # for i in range(20):
        # clock.tick(1000)
        # time.sleep(0.01)
        q = test_q_list[i]
        a = test_a_list[i]
        
        # print(q)

        # p = pose_gt[i]
        
        # print(a)

        g_x.append(i)
        # g_y.append(np.linalg.norm(np.array(a[1])))
        g_y1.append(a[5][0])
        g_y2.append(a[5][1])
        g_y3.append(a[5][2])
        g_y4.append(np.linalg.norm(np.array(a[1])))
        
        # if i==309:
        #     raw_qs = rotation_matrix_to_quaternion(q)
        #     part_str = ["왼팔", "오른팔", "왼다리", "오른다리", "머리", "허리"]
        #     index = 0
        #     for raw_q in raw_qs:
        #         qAccX = (-1.0) * 2.0 * (raw_q[1] * raw_q[3] - raw_q[0] * raw_q[2])
        #         qAccY = (-1.0) * 2.0 * (raw_q[2] * raw_q[3] + raw_q[0]* raw_q[1])
        #         qAccZ = 1.0 - 2.0 * (raw_q[0] * raw_q[0] + raw_q[3] * raw_q[3])
        #         print(f"RMB 계산전 {part_str[index]} 쿼터니언 가속도 x:{qAccX} y:{qAccY} z:{qAccZ}")
        #         index+=1
            
        #     print()
            # print(art.math.rotation_matrix_to_euler_angle(q[0]) * 180 / math.pi)
            # print(RSB)
            # print(RMI)
            
        RMB = RMI.matmul(q).matmul(RSB)
        # if i==309:
        #     raw_qs = rotation_matrix_to_quaternion(RMB)
        #     part_str = ["왼팔", "오른팔", "왼다리", "오른다리", "머리", "허리"]
        #     index = 0
        #     for raw_q in raw_qs:
        #         qAccX = (-1.0) * 2.0 * (raw_q[1] * raw_q[3] - raw_q[0] * raw_q[2])
        #         qAccY = (-1.0) * 2.0 * (raw_q[2] * raw_q[3] + raw_q[0]* raw_q[1])
        #         qAccZ = 1.0 - 2.0 * (raw_q[0] * raw_q[0] + raw_q[3] * raw_q[3])
        #         print(f"RMB 계산후 {part_str[index]} 쿼터니언 가속도 x:{qAccX} y:{qAccY} z:{qAccZ}")
                
        #         index+=1
        #     print("+"*30)
        a = torch.tensor(a)
        
        # a = -torch.tensor(a) / 1000 * 9.8                         # acceleration is reversed
        # a = q.bmm(a.unsqueeze(-1)).squeeze(-1) + torch.tensor([0., 0., 9.8])
        
        
        # a[0][2] *= -1
        # a[1][2] *= -1
        # a[2][2] *= -1
        # a[3][2] *= -1
        # a[4][2] *= -1
        # a[5][2] *= -1
        aM = a.mm(RMI.t())

        #RIS, aI = imu_set.get_noitom()
        #RMB = RMI.matmul(RIS).matmul(RSB)
        #aM = aI.mm(RMI.t())
        
        start_time = time.perf_counter()        
        pose, tran, cj, grf = net.forward_frame(aM.view(1, 6, 3).float(), RMB.view(1, 6, 3, 3).float(), return_grf=True)
        #pose, tran = net.forward()
        end_time = time.perf_counter()
        execution_time = (end_time-start_time) * 1000
        pose = art.math.rotation_matrix_to_axis_angle(pose).view(-1, 72)
        tran = tran.view(-1, 3)


        # tran = torch.zeros(1,3)

        # grf *= 1000.0
            
        # send motion to Unity
        s = ','.join(['%g' % v for v in pose.view(-1)]) + '#' + \
            ','.join(['%g' % v for v in tran.view(-1)]) + '#' + \
            ','.join(['%d' % v for v in cj]) + '#' + \
            (','.join(['%g' % v for v in grf.view(-1)]) if grf is not None else '') + '$'
        # print(s)
        print("-----------------------------")
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        server_address = ('192.168.201.100', 5005)
        sock.sendto(s.encode('utf-8'), server_address)

        # 2차원 그래프 그리기
    #plt.plot(g_x, g_y, marker='o', linestyle='-', color='b', label="Data 1")
    
    # plt.figure()
    # plt.subplot(4, 1, 1)
    # plt.plot(g_x, g_y1, marker='o', linestyle='-', label='X', color='r' )
    # plt.xlabel("time-step")
    # plt.ylabel("X")
    # plt.legend()
    # plt.subplot(4, 1, 2)
    # plt.plot(g_x, g_y2, marker='o', linestyle='-', label='Y', color='g')  
    # plt.xlabel("time-step")
    # plt.ylabel("Y")
    # plt.legend()
    # plt.subplot(4, 1, 3)
    # plt.plot(g_x, g_y3, marker='o', linestyle='-', label='Z', color='b')
    # # 그래프에 제목, 축 이름 설정
    # plt.xlabel("time-step")
    # plt.ylabel("Z")
    # # 범례 추가
    # plt.legend()
    # plt.subplot(4, 1, 4)
    # plt.plot(g_x, g_y4, marker='o', linestyle='-', label='Z', color='b')
    # # 그래프에 제목, 축 이름 설정
    # plt.xlabel("time-step")
    # plt.ylabel("S")
    # # 범례 추가
    # plt.legend()

    # # 그래프 보여주기
    # plt.show()



if __name__ == '__main__':
    UDPStationBroadcastReceiver().start()
    time.sleep(1)
    UDPServer().start()
    # XsensUDPServer().start()
    # time.sleep(99999)


    test = False
    
    if test:
        test_mode()
    
        # time.sleep(1000)
    
    g_x = []
    g_y1 = []
    g_y2 = []
    g_y3 = []
    g_y4 = []
    
    # 실시간 센서 코드!!!!!!!!!!!!

    i_test = 0
    
    imu_set = IMUSet(test)
    net = PIP()
    clock = Clock()
    RMI, RSB = [0, 0]
    start_time = 10000
    part_sequence = [SensorPart.LEFT_LOWER_ARM, SensorPart.RIGHT_LOWER_ARM, SensorPart.LEFT_LOWER_LEG, SensorPart.RIGHT_LOWER_LEG, SensorPart.HEAD, SensorPart.WAIST]
    re_tpose = True
    
    while not test:
        
        # if DataManager().t_pose_set_end == None or not DataManager().t_pose_set_end:
        #     # print("121212121")
        #     re_tpose = True
        #     continue
        
        if re_tpose:
            time.sleep(2)
                        
            imu_set = IMUSet(test)
            net = PIP()
            RMI, RSB = tpose_calibration_ipop_2024(test, imu_set)
            imu_set.clear()
            
            re_tpose = False
        

        clock.tick(59)
        q, a = imu_set.get_ipop()
        RMB = RMI.matmul(q).matmul(RSB)
        a = torch.tensor(a)
        aM = a

        # print(np.linalg.det(RMB[5].mm(torch.eye(3))))
        
        new_rmn = []        
        # RMB = rotation_matrix_to_quaternion(RMB)
        
        for r in RMB:
        #     r = Quaternion(r[0], -r[1], -r[2], -r[3])
        #     r = r.quaternion_to_rotation_matrix()
        # #     r = rotation_matrix_to_quaternion(r)
            # print(r)
        
        #     r = torch.squeeze(r)
        #     r = torch.eye(3).mm(r)
            
            # x축 반전
            # testa = torch.tensor([[-1., 0., 0.],
            #                     [0., 1., 0.],
            #                     [0., 0., 1.]])
            # r = testa.mm(r)
            # r = torch.eye(3).mm(r)
            
        
            # testa = torch.tensor([[1., 0., 0.],
            #                     [0., 1., 0.],
            #                     [0., 0., -1.]])
            # r = testa.mm(r)
            # r = torch.eye(3).mm(r)
            
            # # y-1
            # testa = torch.tensor([[1., 0., 0.],
            #                     [0., -1., 0.],
            #                     [0., 0., 1.]])
            # r = testa.mm(r)
            # r = torch.eye(3).mm(r)
            
            # testa = torch.tensor([[1., 0., 0.],
            #                 [0., 1., 0.],
            #                 [0., 0., -1.]])
            # r = testa.mm(r)
            # r = torch.eye(3).mm(r)
            
            
            # # # z-1
            # testa = torch.tensor([[1., 0., 0.],
            #                     [0., 1., 0.],
            #                     [0., 0., -1.]])
            # r = testa.mm(r)
            # r = torch.eye(3).mm(r)
            
            # testa = torch.tensor([[1., 0., 0.],
            #                 [0., -1., 0.],
            #                 [0., 0., 1.]])
            # r = testa.mm(r)
            # r = torch.eye(3).mm(r)
            
            new_rmn.append(r)
            
            print(np.linalg.det(r))
        #         #    print(r)
        
        # new_rmn = torch.tensor(new_rmn)
        new_rmn = torch.squeeze(torch.stack(new_rmn))
        # print(RMB)
        # print('-------------')
                
                
        
        #RIS, aI = imu_set.get_noitom()
        #RMB = RMI.matmul(RIS).matmul(RSB)
        #aM = aI.mm(RMI.t())
        # start_time = time.perf_counter()        
        pose, tran, cj, grf = net.forward_frame(aM.view(1, 6, 3).float(), new_rmn.view(1, 6, 3, 3).float(), return_grf=True)
        # end_time = time.perf_counter()
        # execution_time = (end_time-start_time) * 1000
        # print(execution_time)
        pose = art.math.rotation_matrix_to_axis_angle(pose).view(-1, 72)
        tran = tran.view(-1, 3)
        
        
        
        # tran = torch.zeros(1,3)
        
        
        # tran = torch.zeros(1,3) # 테스트용 (추후 삭제)   
                
        # send motion to Unity
        s = ','.join(['%g' % v for v in pose.view(-1)]) + '#' + \
            ','.join(['%g' % v for v in tran.view(-1)]) + '#' + \
            ','.join(['%d' % v for v in cj]) + '#' + \
            (','.join(['%g' % v for v in grf.view(-1)]) if grf is not None else '') + '$'
        # print(s)
        #print("-----------------------------")
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        server_address = ('192.168.201.100', 5005)
        sock.sendto(s.encode('utf-8'), server_address)
        
        
        # print(333333333333333333)
        
        #try:
        #    conn.send(s.encode('utf8'))

        #except:
        #    break

        #data['aM'].append(aM)
        #data['RMB'].append(RMB)

        #if is_executable and keyboard.is_pressed('q'):
        #    break

        #print('\rfps: ', clock.get_fps(), end='')

    # if is_executable:
    #     os.system('taskkill /F /IM "%s"' % os.path.basename(paths.unity_file))

    # # data['aM'] = torch.stack(data['aM'])
    # # data['RMB'] = torch.stack(data['RMB'])
    # #torch.save(data, os.path.join(paths.live_record_dir, 'xsens' + datetime.datetime.now().strftime('%Y%m%d%H%M%S') + '.pt'))
    # print('\rFinish.')
    
    
    
    

      