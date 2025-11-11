import torch
import struct
from pygame.time import Clock
from torch.nn.parallel.comm import broadcast

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
import multiprocessing


class IMUSet:
    g = 9.8
    test_i = 0

    def __init__(self):
        self.n_imus = 0


    def get_ipop(self):

        r = DataManager().test_r
        a = DataManager().test_acc
        hand = DataManager().test_hand
        
        a = -torch.tensor(a) * 9.8                        # acceleration is reversed
        a = r.bmm(a.unsqueeze(-1)).squeeze(-1) + torch.tensor([0., 0., 9.8])

        return r, a, hand

    def clear(self):
        pass


def tpose_calibration_ipop_2024(imu_set):
    RSI = imu_set.get_ipop()[0][5].view(3, 3).t()

    RMI = torch.tensor([[1, 0, 0], [0, 1, 0], [0, 0, 1.]]).mm(RSI)
    RIS, _, handRIS = imu_set.get_ipop()


    RSB = RMI.matmul(RIS).transpose(1, 2).matmul(torch.eye(3))  # [6, 3, 3]
    RSB_hand = RMI.matmul(handRIS).transpose(1, 2).matmul(torch.eye(3))  # [6, 3, 3]

    return RMI, RSB, RSB_hand


  
  

def run_pip(udp_server_port, unity_ip, unity_port):

    # UDPStationBroadcastReceiver(broadcast_port).start()
    # print(1)
    # time.sleep(1)
    UDPServer(udp_server_port).start()

    # 실시간 센서 코드!!!!!!!!!!!!
    imu_set = IMUSet()
    net = PIP()
    clock = Clock()
    RMI, RSB = [0, 0]
    re_tpose = True
    
    while True:
        
        # if DataManager().t_pose_set_end == None or not DataManager().t_pose_set_end:
        #     re_tpose = True
        #     continue
        
        if re_tpose:
            time.sleep(2)
                        
            imu_set = IMUSet()
            net = PIP()
            RMI, RSB, RSB_hand = tpose_calibration_ipop_2024(imu_set)
            imu_set.clear()
            
            re_tpose = False
        

        clock.tick(59)
        q, a, hand_q = imu_set.get_ipop()
        RMB = RMI.matmul(q).matmul(RSB)
        RMB_hand = RMI.matmul(hand_q).matmul(RSB_hand)


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
        

        aM = a.mm(RMI.t())

        pose, tran, cj, grf = net.forward_frame(aM.view(1, 6, 3).float(), RMB.view(1, 6, 3, 3).float(), return_grf=True)

        pose = art.math.rotation_matrix_to_axis_angle(pose).view(-1, 72)
        tran = tran.view(-1, 3)

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
        server_address = (unity_ip, unity_port)
        sock.sendto(s.encode('utf-8'), server_address)



def test_udp_client(ip, port6, port7):
    try:
        # UDP 소켓 생성
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # 송신할 데이터 설정
        send_data = bytearray(10)
        send_data[0] = 0xFA
        send_data[1] = 0xEA
        send_data[2] = (192) & 0xFF  # IP 주소
        send_data[3] = (168) & 0xFF
        send_data[4] = (201) & 0xFF
        send_data[5] = (104) & 0xFF
        send_data[6] = port6  # 포트 번호
        send_data[7] = port7
        send_data[8] = 0xFB
        send_data[9] = 0xFF

        # IP 주소와 포트 설정
        server_address = (ip, 65000)
        # 데이터 전송
        sock.sendto(send_data, server_address)

    except Exception as e:
        print(f"Error2: {e}")


if __name__ == '__main__':

    processes = []
    broadcast_port =  [56439, 56853, 56439, 56439]
    broadcast_ip = ['192.168.201.152', '192.168.201.151']

    for i in range(2):  # CPU 코어 수에 맞게 조정 가능
        udp_server_port = 17171 + i # 17171, 17172, 17173, 17174
        # udp_server_port = 56439 + i # 17171, 17172, 17173, 17174


        # 브로드 캐스트 테스트용 (추후 삭제)
        port6 = (broadcast_port[i] >> 8) & 0xFF
        port7 = broadcast_port[i] & 0xFF
        test_udp_client(broadcast_ip[i], port6, port7)


        unity_ip = '192.168.201.155'
        unity_port = 8888 + i # 8888, 8889, 8890, 8891
        process = multiprocessing.Process(target=run_pip, args=(broadcast_port[i], unity_ip, unity_port))
        processes.append(process)
        process.start()

    # 모든 프로세스가 종료될 때까지 대기
    for process in processes:
        process.join()

    print('종료!!!!!!!!!!!!')

