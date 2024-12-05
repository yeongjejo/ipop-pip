import torch
import struct
from pygame.time import Clock
from log_test import rotation_matrix_to_quaternion
from net import PIP
import articulate as art
import os
from config import *
from data_manager import DataManager
from protocol.udp_server import UDPServer
from protocol.udp_station_broadcast_receiver import UDPStationBroadcastReceiver
import time
import socket
import numpy as np
from sensor.sensor_part import SensorPart

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

        q = DataManager().test_q
        r = DataManager().test_r
        a = DataManager().test_acc
        axis = DataManager().axis
        hand = DataManager().test_hand
        
        a = -torch.tensor(a) * 9.8                        # acceleration is reversed
        a = r.bmm(a.unsqueeze(-1)).squeeze(-1) + torch.tensor([0., 0., 9.8])  
        # a = -torch.tensor(a) / 1000 * 9.8  # acceleration is reversed
        # a = r.bmm(a.unsqueeze(-1)).squeeze(-1) + torch.tensor([0., 0., 9.8])  # calculate global free acceleration

        # print(q)
        
        # print(hand[2:])
        
	
        return r, a, hand, axis



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
        RMI = torch.eye(3)
        # RMI = torch.tensor([[0, 1, 0], [0, 0, 1], [1, 0, 0.]]).mm(RSI)

    else:
        # RMI = torch.eye(3)
        RMI = torch.tensor([[1, 0, 0], [0, 1, 0], [0, 0, 1.]]).mm(RSI)
        

        # RMI = torch.tensor([[0, 1, 0], [0, 0, 1], [1, 0, 0.]]).mm(RSI)
        
        # RMI = torch.tensor([[0, 0, 1], [1, 0, 0], [0, 1, 0.]]).mm(RSI)
        
        
        # RMI = torch.tensor([[1, 0, 0], [0, 0, 1], [0, -1, 0.]]).mm(RSI)
        
        
    RIS, _, handRIS, axisRIS = imu_set.get_ipop()
    
    RSB = RMI.matmul(RIS).transpose(1, 2).matmul(torch.eye(3))  # [6, 3, 3]
    
    
    RSB_hand = RMI.matmul(handRIS).transpose(1, 2).matmul(torch.eye(3))  # [6, 3, 3]
    

    # test = torch.eye(3).mm(RSB[0])
    # print(np.linalg.det(test))


    return RMI, RSB, RSB_hand


  
  

if __name__ == '__main__':
    UDPStationBroadcastReceiver().start()
    time.sleep(1)
    UDPServer().start()
    # XsensUDPServer().start()
    # time.sleep(99999)

        # time.sleep(1000)
    
    g_x = []
    g_y1 = []
    g_y2 = []
    g_y3 = []
    g_y4 = []
    
    # 실시간 센서 코드!!!!!!!!!!!!
    test = False
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
            RMI, RSB, RSB_hand = tpose_calibration_ipop_2024(test, imu_set)
            imu_set.clear()
            
            re_tpose = False
        

        clock.tick(59)
        q, a, hand_q = imu_set.get_ipop()
        RMB = RMI.matmul(q).matmul(RSB)
        RMB_hand = RMI.matmul(hand_q).matmul(RSB_hand)
        
        # print(rotation_matrix_to_quaternion(RMB_hand))
        # print(RMB_hand.shape)
        
        
        test_hand_q = [0, 0, 0, 0, 0, 0, 0, 0]
        hand_r = rotation_matrix_to_quaternion(RMB_hand)
        test_hand_q[0] = float(hand_r[6][0])
        test_hand_q[1] = float(hand_r[6][1])
        test_hand_q[2] = float(hand_r[6][2])
        test_hand_q[3] = float(hand_r[6][3])
        
        test_hand_q[4] = float(hand_r[7][0])
        test_hand_q[5] = float(hand_r[7][1])
        test_hand_q[6] = float(hand_r[7][2])
        test_hand_q[7] = float(hand_r[7][3])
        
        
        # print('test_hand_q', test_hand_q)
        # a = torch.tensor(a)
        aM = a.mm(RMI.t())

        # print(np.linalg.det(RMB[5].mm(torch.eye(3))))

        #RIS, aI = imu_set.get_noitom()
        #RMB = RMI.matmul(RIS).matmul(RSB)
        #aM = aI.mm(RMI.t())
        # start_time = time.perf_counter()        
        # print(test_hand_q)
        pose, tran, cj, grf = net.forward_frame(aM.view(1, 6, 3).float(), RMB.view(1, 6, 3, 3).float(), return_grf=True)
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
            ','.join(['%g' % v for v in test_hand_q]) + '#' + \
            ','.join(['%g' % v for v in DataManager().test_finger]) + '#' + \
            (','.join(['%g' % v for v in grf.view(-1)]) if grf is not None else '') + '$'
        print(','.join(['%g' % v for v in test_hand_q]) + '#')

        #print("-----------------------------")
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # server_address = ('192.168.201.100', 5005)
        server_address = ('192.168.201.201', 50001)
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
    
    
    
    

      