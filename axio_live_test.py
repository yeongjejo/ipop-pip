import numpy as np
import torch
from pygame.time import Clock
from net import PIP
import articulate as art
from data_manager import DataManager
import time
import socket
import json

from protocol.axio_server import AxioServer
from totalcapture.premodel_server import PreModelServer
from totalcapture.senpreprocessed import TotalcaptureIMUData, TotalcaptureViconData


class IMUSet:
    g = 9.8
    test_i = 0

    def __init__(self):
        self.n_imus = 0


    def get_ipop(self):

        r = DataManager().ipop_imu_r
        ten_r = DataManager().premodel_imu_r
        ten_a = DataManager().premodel_imu_acc
        a = DataManager().ipop_imu_acc

        ten_a = torch.tensor(ten_a)
        a = torch.tensor(a)


        return r, ten_a, ten_r, a




def tpose_calibration_ipop_2024(imu_set):
    RSI = imu_set.get_ipop()[0][5].view(3, 3).t()

    RMI = torch.tensor([[1, 0, 0], [0, 1, 0], [0, 0, 1.]]).mm(RSI)
    RMI2 = torch.tensor([[1, 0, 0], [0, 1, 0], [0, 0, 1.]]).mm(RSI)


    # RMI2 = torch.tensor([[0, -1, 0], [-1, 0, 0], [0, 0, 1.]]).mm(RSI)

    # RMI = torch.tensor([[0, 1, 0], [0, 0, 1], [1, 0, 0.]]).mm(RSI)
    # RMI = torch.tensor([[-1, 0, 0], [0, 1, 0], [0, 0, -1.]]).mm(RSI)
    # RMI2 = torch.tensor([[1, 0, 0], [0, -1, 0], [0, 0, 1.]]).mm(RSI)
    RIS, _, RIS2, _ = imu_set.get_ipop()


    RSB = RMI.matmul(RIS).transpose(1, 2).matmul(torch.eye(3))  # [6, 3, 3]
    RSB2 = RMI.matmul(RIS2).transpose(1, 2).matmul(torch.eye(3))  # [6, 3, 3]
    # RSB_hand = RMI.matmul(handRIS).transpose(1, 2).matmul(torch.eye(3))  # [6, 3, 3]

    return RMI, RSB, RMI2, RSB2



if __name__ == '__main__':
    print('실행 시작')
    AxioServer().start()

    print('333')

    clock = Clock()

    print('333')
    while True:
        if DataManager().axioStart is not True:
            print('0000')
            continue

        print('11111')

        imu_set = IMUSet()
        net = PIP()
        RMI, RSB, RMI2, RSB2 = tpose_calibration_ipop_2024(imu_set)
        i = 0


        while True:
            # print(1)
            clock.tick(60)

            pre_q, pre_a, pre_q2, pre_a2 = imu_set.get_ipop()

            pre_RMB = RMI.matmul(pre_q).matmul(RSB)
            pre_RMB2 = RMI.matmul(pre_q2).matmul(RSB2)
            a = pre_a
            a2 = pre_a2


            aM = a.mm(RMI2.t())
            aM2 = a2.mm(RMI2.t())


            pip_pose, tran, cj, grf = net.forward_frame(pre_RMB2, aM, pre_RMB.view(1, 6, 3, 3), aM2.view(1, 6, 3))

            contact_check = 0

            pose = torch.zeros(24, 3)
            joint_seq = [16, 18, 17, 19, 1, 4, 2, 5, 0, 9]
            axis_rot = art.math.rotation_matrix_to_axis_angle(pre_RMB2)
            for j, j_angle in enumerate(axis_rot):
                pose[joint_seq[j]] = j_angle


            tran = tran.view(-1, 3).view(-1).tolist()
            # pip_pose = art.math.rotation_matrix_to_axis_angle(pip_pose).view(-1, 72)
            # tran = tran.view(-1, 3)
            # s = ','.join(['%g' % v for v in pip_pose.view(-1)]) + '#' + \
            #     ','.join(['%g' % v for v in tran.view(-1)]) + '#' + \
            #     ','.join(['%d' % v for v in cj]) + '#' + \
            #     (','.join(['%g' % v for v in grf.view(-1)]) if grf is not None else '') + '$'
            #
            # # print("-----------------------------")
            # sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            # # server_address = ('192.168.201.100', 5005)
            # server_address = ('192.168.201.199', 8888)
            # sock.sendto(s.encode('utf-8'), server_address)

            pip_pose = art.math.rotation_matrix_to_axis_angle(pip_pose)

            q = art.math.axis_angle_to_quaternion(pose)
            # print(q)
            bone_seq = [0, 3, 6, 9, 12, 15, 13, 16, 18, 20, 14, 17, 19, 21, 1, 4, 7, 2, 5, 8]

            send_data = []
            send_data2 = []
            tesq = [0, 0]
            for index in bone_seq:
                p = [0.0, 0.0, 0.0]
                p2 = [0.0, 0.0, 0.0]
                if index == 0:
                    p = [0.0, 0.0, 0.0]


                rotation = q[index].tolist()
                frame_bone_data = {
                    "time": contact_check,
                    "name": "test",
                    "position": tran,
                    "rotation": rotation,
                    # "rotation": [bone[1].w, -bone[1].x, -bone[1].z, bone[1].y],
                    "acc": [0.0, 0.0, 0.0],
                    "lp": [0.0, 0.0, 0.0],
                    "rp": [0.0, 0.0, 0.0],
                }

                # frame_bone_data2 = {
                #     "time": contact_check,
                #     "name": "test",
                #     "position": joint[index].tolist(),
                #     "rotation": [1.0, 0.0, 0.0, 0.0],
                #     # "rotation": [bone[1].w, -bone[1].x, -bone[1].z, bone[1].y],
                #     "acc": [0.0, 0.0, 0.0],
                #     "lp": [0.0, 0.0, 0.0],
                #     "rp": [0.0, 0.0, 0.0],
                # }

                send_data.append(frame_bone_data)
                # send_data2.append(frame_bone_data2)

            # print(send_data)\\
            data = json.dumps(send_data).encode("utf-8")
            data2 = json.dumps(send_data2).encode("utf-8")
            # # data = json.dumps(DataManager().totalcapture_gt[i]).encode("utf-8")
            # # print(data)
            #
            TARGET_IP = DataManager().set_ip
            # TARGET_PORT = 5005
            # sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            # sock.sendto(data, (TARGET_IP, TARGET_PORT))
            # #

            # TARGET_PORT = 5006
            # sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            # sock.sendto(data2, (TARGET_IP, TARGET_PORT))

            TARGET_PORT = 5007

            # print(data)
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.sendto(data, (TARGET_IP, TARGET_PORT))
            print('전송완')


            i += 1