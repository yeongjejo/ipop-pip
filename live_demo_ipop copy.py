import torch
import struct
from pygame.time import Clock
from net import PIP
import articulate as art
import os
from config import *
#import keyboard
from data_manager import DataManager
from protocol.udp_server import UDPServer
from protocol.udp_station_broadcast_receiver import UDPStationBroadcastReceiver
import time
import socket
import numpy as np
import matplotlib.pyplot as plt

class IMUSet:
    g = 9.8
    test_i = -1

    def __init__(self, test):

        self.n_imus = 6
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
            # self.test_i += 300
            self.test_i = 0

            q = test_q_list[self.test_i]
            a = test_a_list[self.test_i]
            # a = -torch.tensor(a) / 1000 * 9.8                        # acceleration is reversed
            # a = q.bmm(a.unsqueeze(-1)).squeeze(-1) + torch.tensor([0., 0., 9.8])   # calculate global free acceleration

            
            return q, a

        q = DataManager().test_q
        r = DataManager().test_r
        a = DataManager().test_acc
        # a = -torch.tensor(a) / 1000 * 9.8  # acceleration is reversed
        # a = r.bmm(a.unsqueeze(-1)).squeeze(-1) + torch.tensor([0., 0., 9.8])  # calculate global free acceleration

        
        
        
	
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
    RSI = imu_set.get_ipop()[0][0].view(3, 3).t()
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
    RSI = imu_set.get_ipop()[0][0].view(3, 3).t()

    # print(f'RSI.shape: {RSI.shape}\nRSI:\n{RSI}')

    if test:
        RMI = torch.eye(3)
        # RSI = imu_set.get_ipop()[0][0].view(3, 3).t()
        # print(imu_set.get_ipop())
        # RMI = torch.tensor([[0, 1, 0], [0, 0, 1], [1, 0, 0.]]).mm(RSI)
    else:
        RMI = torch.tensor([[0, -1, 0], [0, 0, -1], [0, 0, -1.]]).mm(RSI)#torch.tensor([[0, 1, 0], [0, 0, 1], [1, 0, 0.]])#torch.tensor([[0, 1, 0], [0, 0, 1], [1, 0, 0.]]).mm(RSI) #torch.eye(3)
        # RMI = torch.eye(3)

    # print(f'RMI.shape: {RMI.shape}\nRMI:\n{RMI}')
    time.sleep(3)
    # print('-----------------------')

    RIS = imu_set.get_ipop()[0]
    
    # print(RIS)
    RSB = RMI.matmul(RIS).transpose(1, 2).matmul(torch.eye(3))  # [6, 3, 3

    # print(f'RSB.shape: {RSB.shape}\nRSB:\n{RSB}')


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
    test_a_list, test_q_list, pose_gt = DataManager().process_dipimu()
    
    print(test_a_list[309])
    
    print(test_q_list[309])
    
    # for i in range(len(test_q_list[:600])):
    for i in range(len(test_q_list[:])):
    # for i in range(20):
        # clock.tick(1000)
        # time.sleep(0.01)
        q = test_q_list[i]
        a = test_a_list[i]

        p = pose_gt[0]
        
        # print(a)

        g_x.append(i)
        # g_y.append(np.linalg.norm(np.array(a[1])))
        g_y1.append(a[5][0])
        g_y2.append(a[5][1])
        g_y3.append(a[5][2])
        g_y4.append(np.linalg.norm(np.array(a[1])))
        

        RMB = RMI.matmul(q).matmul(RSB)
        a = torch.tensor(a)
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
        s = ','.join(['%g' % v for v in p.view(-1)]) + '#' + \
            ','.join(['%g' % v for v in tran.view(-1)]) + '#' + \
            ','.join(['%d' % v for v in cj]) + '#' + \
            (','.join(['%g' % v for v in grf.view(-1)]) if grf is not None else '') + '$'
        # print(s)
        # print("-----------------------------")
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        server_address = ('192.168.201.198', 5005)
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
    # UDPStationBroadcastReceiver().start()
    # time.sleep(3)
    UDPServer().start()


    os.makedirs(paths.temp_dir, exist_ok=True)
    os.makedirs(paths.live_record_dir, exist_ok=True)

    is_executable = False
    #server_for_unity = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    #server_for_unity.bind(('', 8888))
    #server_for_unity.listen(1)
    #print('Server start. Waiting for unity3d to connect.')
    ##if paths.unity_file != '' and os.path.exists(paths.unity_file):
    #    win32api.ShellExecute(0, 'open', os.path.abspath(paths.unity_file), '', '', 1)
    #    is_executable = True
    #conn, addr = server_for_unity.accept()


    test = True
    
    if test:
        test_mode()
    
        # time.sleep(1000)
    

    # print(DataManager().process_dipimu()[0])

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
    
    re_tpose = True
    # print("00000000000000000000000")
    while not test:
        
        # print(1111111111)
        if DataManager().t_pose_set_end == None or not DataManager().t_pose_set_end:
            # print("121212121")
            re_tpose = True
            continue
        
        # # print("121212121")
        if re_tpose:
            time.sleep(2)
                        
            imu_set = IMUSet(test)
            net = PIP()
            # print("121212121")
            RMI, RSB = tpose_calibration_ipop_2024(test, imu_set)
            #RMI, RSB = tpose_calibration_noitom()
            imu_set.clear()
            
        #     # data = {'RMI': RMI, 'RSB': RSB, 'aM': [], 'RMB': []}
            re_tpose = False
        
        # clock.tick(70)
        # print("111111111111111")
        # if i_test == 9999999999999999999999999999999999999999999999999999:

        #     plt.figure()
        #     plt.subplot(4, 1, 1)
        #     plt.plot(g_x, g_y1, marker='o', linestyle='-', label='X', color='r' )
        #     plt.xlabel("time-step")
        #     plt.ylabel("X")
        #     plt.legend()
        #     plt.subplot(4, 1, 2)
        #     plt.plot(g_x, g_y2, marker='o', linestyle='-', label='Y', color='g')  
        #     plt.xlabel("time-step")
        #     plt.ylabel("Y")
        #     plt.legend()
        #     plt.subplot(4, 1, 3)
        #     plt.plot(g_x, g_y3, marker='o', linestyle='-', label='Z', color='b')   
        #     plt.xlabel("time-step")
        #     plt.ylabel("Z")
            
        #     plt.subplot(4, 1, 4)
        #     plt.plot(g_x, g_y4, marker='o', linestyle='-', label='Z', color='b')   
        #     plt.xlabel("time-step")
        #     plt.ylabel("S")
        #     plt.legend()

        #         # 그래프에 제목, 축 이름 설정
        #     # plt.title("2D Graph of Two Arrays")
        #     # plt.xlabel("X-axis")
        #     # plt.ylabel("Y-axis")

        #     # 범례 추가
        #     plt.legend()

        #     # 그래프 보여주기
        #     plt.show()
        #     # break;

        # # clock.tick(60)
        
    # RMI, RSB = tpose_calibration_ipop()
    
        # 싱크조절
        # now_time = time.perf_counter()
        # if (now_time-start_time) * 1000 < 10:
        #     continue
        
        # print((now_time-start_time) * 1000)
        
        # start_time = now_time
        
        try:
            q, a = imu_set.get_ipop()
            RMB = RMI.matmul(q).matmul(RSB)
            a = torch.tensor(a)
        except:
            print(RMI)
            print(RSB)
            print(q)
            print(a)
        # print(22222222222222222)
        
        # a = -torch.tensor(a) * 9.8                         # acceleration is reversed
        # a = -torch.tensor(a) * 9.8                         # acceleration is reversed
        # a = q.bmm(a.unsqueeze(-1)).squeeze(-1) + torch.tensor([0., 0., 9.8])   # calculate global free acceleration
        aM = a.mm(RMI.t())

        
        
        # #가속도 그래프 확인용 (추후 삭제)
        g_x.append(i_test)
        g_y1.append(a[5][0])
        g_y2.append(a[5][1])
        g_y3.append(a[5][2])
        g_y4.append(np.linalg.norm(np.array(a[5])))
        i_test += 1

        
        #RIS, aI = imu_set.get_noitom()
        #RMB = RMI.matmul(RIS).matmul(RSB)
        #aM = aI.mm(RMI.t())
        # start_time = time.perf_counter()        
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
            (','.join(['%g' % v for v in grf.view(-1)]) if grf is not None else '') + '$'
        # print(s)
        #print("-----------------------------")
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        server_address = ('192.168.201.198', 5005)
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
    
    
    
    
  