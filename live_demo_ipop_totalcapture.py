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

from velocitymodel.utils import TotalCaptureDataset
from velocitymodel.module.module import *


class IMUSet:
    g = 9.8
    test_i = 0

    def __init__(self):
        self.n_imus = 0


    def get_ipop(self):

        r = DataManager().ipop_imu_r
        premodel_r = DataManager().premodel_imu_r
        a = DataManager().ipop_imu_acc

        a = torch.tensor(a)
        #
        # if self.n_imus == 0:
        #     print("0번 : ", torch.tensor(a))


        #
        # a = -torch.tensor(a) * 9.8 / 10.0                       # acceleration is reversed
        # # a = r.bmm(a.unsqueeze(-1)).squeeze(-1) + torch.tensor([0., 0., 9.8])
        # a = r.bmm(a.unsqueeze(-1)).squeeze(-1)
        # if self.n_imus == 0:
        #     print("1번 : ", a)
        # a[0] += torch.tensor([0., 0., 9.8])
        # a[1] += torch.tensor([0., 0., 9.8])
        # a[2] += torch.tensor([9.8, 0., 0.])
        # a[3] += torch.tensor([9.8, 0., 0.])
        # a[4] += torch.tensor([0., -9.8, 0.])
        # a[5] += torch.tensor([9.8, 0., 0.])
        # if self.n_imus == 0:
        #     print("2번 : ",a)
        # self.n_imus += 1
        # print(a[5])

        return r, a, premodel_r




def tpose_calibration_ipop_2024(imu_set):
    RSI = imu_set.get_ipop()[0][5].view(3, 3).t()

    RMI = torch.tensor([[1, 0, 0], [0, 1, 0], [0, 0, 1.]]).mm(RSI)
    RMI2 = torch.tensor([[1, 0, 0], [0, 1, 0], [0, 0, 1.]]).mm(RSI)


    # RMI2 = torch.tensor([[0, -1, 0], [-1, 0, 0], [0, 0, 1.]]).mm(RSI)

    # RMI = torch.tensor([[0, 1, 0], [0, 0, 1], [1, 0, 0.]]).mm(RSI)
    # RMI = torch.tensor([[-1, 0, 0], [0, 1, 0], [0, 0, -1.]]).mm(RSI)
    # RMI2 = torch.tensor([[1, 0, 0], [0, -1, 0], [0, 0, 1.]]).mm(RSI)
    RIS, _, RIS2 = imu_set.get_ipop()


    RSB = RMI.matmul(RIS).transpose(1, 2).matmul(torch.eye(3))  # [6, 3, 3]
    RSB2 = RMI.matmul(RIS2).transpose(1, 2).matmul(torch.eye(3))  # [6, 3, 3]
    # RSB_hand = RMI.matmul(handRIS).transpose(1, 2).matmul(torch.eye(3))  # [6, 3, 3]

    return RMI, RSB, RMI2, RSB2





if __name__ == '__main__':
    PreModelServer().start()
    AxioServer().start()

    while True:
        if DataManager().axioStart is not True:
            continue

        imu_set = IMUSet()
        net = PIP()
        net2 = PIP()
        RMI, RSB, RMI2, RSB2 = tpose_calibration_ipop_2024(imu_set)
        i = 0

        imu_switch = True
        pre_q = None
        pre_a = None
        pre_RMB = None
        pre_RMB2 = None
        reset = 'start'


        # while i < len(DataManager.totalcapture_imu_acc):
        while True:
            if DataManager().tpose_check:
                time.sleep(2)
                DataManager().axioStart = False
                imu_switch = True
                DataManager().udp_switch = False
                DataManager().udp_sending = False
                DataManager().pre_position = [0.0, 0.0, 0.0]
                print('reset')
                break

            q = pre_q
            a = pre_a
            RMB = pre_RMB
            RMB2 = pre_RMB2
            if not DataManager().udp_sending:
                if imu_switch:
                    imu_switch = False
                    pre_q, pre_a, pre_q2 = imu_set.get_ipop()
                    pre_RMB = RMI.matmul(pre_q).matmul(RSB)
                    pre_RMB2 = RMI.matmul(pre_q2).matmul(RSB2)
                    q = pre_q
                    a = pre_a
                    RMB = pre_RMB
                    RMB2 = pre_RMB2


                # 프리 모델이 적용할 데이터 전송
                send_data = []
                for rot in art.math.axis_angle_to_quaternion(art.math.rotation_matrix_to_axis_angle(RMB2)):
                    rot = rot.tolist()
                    # print(DataManager().premodel_root_p[i])
                    # print(DataManager().pre_position)
                    # print('-'*30)
                    frame_bone_data = {
                        "time": "5",
                        "name": reset,
                        # "position": [-x_p / 0.0254 / 3.0, y_p / 0.0254 / 3.0, -z_p / 0.0254 / 3.0],
                        # "position": DataManager().premodel_root_p[i],
                        "position": DataManager().pre_position,
                        "rotation": [rot[0], rot[1], rot[2], rot[3]],
                        "acc": [0.0, 0.0, 0.0]
                    }
                    send_data.append(frame_bone_data)

                data = json.dumps(send_data).encode("utf-8")

                TARGET_IP = "192.168.201.199"
                TARGET_PORT = 5005
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sock.sendto(data, (TARGET_IP, TARGET_PORT))
                DataManager().udp_sending = True

            elif DataManager().udp_switch:
                reset = 'end'
                aM = a.mm(RMI2.t())

                pose, tran, cj, grf, contact_check = net.forward_frame(a.view(1, 6, 3).float(), q.view(1, 6, 3, 3).float(), return_grf=True, check_rbdl=False)

                pose = art.math.rotation_matrix_to_axis_angle(pose).view(-1, 72)


                q = art.math.axis_angle_to_quaternion(pose)
                bone_seq = [0, 3, 6, 9, 12, 15, 13, 16, 18, 20, 14, 17, 19, 21, 1, 4, 7, 2, 5, 8]

                send_data = []
                send_data2 = []
                for index in bone_seq:
                    p = [0.0, 0.0, 0.0]
                    p2 = [0.0, 0.0, 0.0]
                    if index == 0:
                        p = tran.view(-1, 3).tolist()[0]

                    rotation = q[index].tolist()
                    frame_bone_data = {
                        "time": contact_check,
                        "name": "test",
                        "position": p,
                        "rotation": rotation,
                        # "rotation": [bone[1].w, -bone[1].x, -bone[1].z, bone[1].y],
                        "acc": [0.0, 0.0, 0.0]
                    }
                    send_data.append(frame_bone_data)


                data = json.dumps(send_data).encode("utf-8")
                TARGET_IP = "192.168.201.199"
                TARGET_PORT = 5007
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sock.sendto(data, (TARGET_IP, TARGET_PORT))


                DataManager().udp_switch = False
                DataManager().udp_sending = False

                imu_switch = True
                i += 1
