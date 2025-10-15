import torch
from pygame.time import Clock
from ipop_net import PIP
import articulate as art
from data_manager import DataManager
import time
import socket
import json

from totalcapture.premodel_server import PreModelServer
from totalcapture.senpreprocessed import TotalcaptureIMUData, TotalcaptureViconData

from velocitymodel.utils import TotalCaptureDataset
from velocitymodel.module.module import *


class IMUSet:
    g = 9.8
    test_i = 0

    def __init__(self):
        self.n_imus = 0


    def get_ipop(self, i):

        r = DataManager().totalcapture_imu_r[i]
        premodel_r = DataManager().premodel_imu_r[i]
        a = DataManager().totalcapture_imu_acc[i]

        a = -torch.tensor(a) * 9.8 / 10.0                       # acceleration is reversed
        a = r.bmm(a.unsqueeze(-1)).squeeze(-1) + torch.tensor([0., 0., 9.8])

        return r, a, premodel_r




def tpose_calibration_ipop_2024(imu_set):
    RSI = imu_set.get_ipop(0)[0][5].view(3, 3).t()

    # RMI = torch.tensor([[0, 1, 0], [0, 0, 1], [1, 0, 0.]]).mm(RSI)
    RMI = torch.tensor([[-1, 0, 0], [0, 1, 0], [0, 0, -1.]]).mm(RSI)
    RMI2 = torch.tensor([[1, 0, 0], [0, -1, 0], [0, 0, 1.]]).mm(RSI)
    RIS, _, RIS2 = imu_set.get_ipop(0)


    RSB = RMI.matmul(RIS).transpose(1, 2).matmul(torch.eye(3))  # [6, 3, 3]
    RSB2 = RMI.matmul(RIS2).transpose(1, 2).matmul(torch.eye(3))  # [6, 3, 3]
    # RSB_hand = RMI.matmul(handRIS).transpose(1, 2).matmul(torch.eye(3))  # [6, 3, 3]

    return RMI, RSB, RMI2, RSB2





if __name__ == '__main__':
    PreModelServer().start()
    print('데이터셋 준비중..)')
    TotalcaptureIMUData().setTotalcaptureIMUData()
    print('데이터셋 준비중..(1/2)')
    TotalcaptureViconData().setTotalcaptureViconData()
    print('데이터셋 준비중..(2/2)')
    print('데이터셋 완료!!!')



    while True:
        imu_set = IMUSet()
        net = PIP()
        net2 = PIP()
        RMI, RSB, RMI2, RSB2 = tpose_calibration_ipop_2024(imu_set)
        i = 0
        pre_speed = [0.0, 0.0, 0.0]

        while i < len(DataManager.totalcapture_imu_acc):
            time.sleep(DataManager().time_setting)
        # while i < 476:
            if i == 0:
                pre_speed = [0.0, 0.0, 0.0]
                # pre_position = [0.0, 11.0, 0.0]

            q, a, q2 = imu_set.get_ipop(i)
            RMB = RMI.matmul(q).matmul(RSB)
            RMB2 = RMI.matmul(q2).matmul(RSB2)

            if not DataManager().udp_switch and not DataManager().udp_sending:

                # 프리 모델이 적용할 데이터 전송
                send_data = []
                for test in art.math.axis_angle_to_quaternion(art.math.rotation_matrix_to_axis_angle(RMB2)):
                    test = test.tolist()
                    frame_bone_data = {
                        "time": "5",
                        "name": "test",
                        "position": [0.0, 0.0, 0.0],
                        "rotation": [test[0], test[1], test[2], test[3]],
                        "acc": [0.0, 0.0, 0.0],
                        "lp": [],
                        "rp": [],
                    }
                    send_data.append(frame_bone_data)

                data = json.dumps(send_data).encode("utf-8")


                TARGET_IP =  DataManager().set_ip
                TARGET_PORT = 5005
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sock.sendto(data, (TARGET_IP, TARGET_PORT))
                DataManager().udp_sending = True

            elif DataManager().udp_switch:
                # print(2)
                # print(a[5], i)
                # print('+++')
                aM = a.mm(RMI2.t())
                #
                # x, _, _ = velocity_input_list[i]
                # x = x.unsqueeze(0)
                #
                #
                # x = x.to("cpu")
                # with torch.no_grad():
                position  = DataManager().totalcapture_gt[i][0]['position']
                pose, tran, l_foot_p, r_foot_p, contact_check = net.forward_frame(aM.view(1, 6, 3).float(), RMB.view(1, 6, 3, 3).float(), position=position, return_grf=True, check_rbdl=True)
                # pose2, tran2, cj2, grf2 = net2.forward_frame(aM.view(1, 6, 3).float(), RMB.view(1, 6, 3, 3).float(), return_grf=True, check_rbdl=True)



                # pose2 = art.math.rotation_matrix_to_axis_angle(pose2).view(-1, 72)
                pose = art.math.rotation_matrix_to_axis_angle(pose).view(-1, 72)


                q = art.math.axis_angle_to_quaternion(pose)
                # q2 = art.math.axis_angle_to_quaternion(pose2)
                bone_seq = [0, 3, 6, 9, 12, 15, 13, 16, 18, 20, 14, 17, 19, 21, 1, 4, 7, 2, 5, 8]
                contact_check = 0
                contact_left = 0
                contact_right = 0

                th = 0.4
                if DataManager().totalcapture_gt[i][-4]['position'][1] > th:
                    #print('GT LFOOT: 0.0')
                    pass
                elif DataManager().totalcapture_gt[i][-4]['position'][1] < 0.0:
                    #print('GT LFOOT: 1.0')
                    contact_check = 1
                    contact_left = 1
                else:
                    #print('GT LFOOT: ', DataManager().totalcapture_gt[i][-4]['position'][1])
                    contact_check = 1
                    contact_left = 1

                if DataManager().totalcapture_gt[i][-1]['position'][1] > th:
                    #print('GT RFOOT: 0.0')
                    pass
                elif DataManager().totalcapture_gt[i][-1]['position'][1] < 0.0:
                    #print('GT RFOOT: 1.0')
                    contact_right = 1
                    if contact_check == 0:
                        contact_check = 2
                    elif contact_check == 1:
                        contact_check = 3
                else:
                    #print('GT RFOOT: ', DataManager().totalcapture_gt[i][-1]['position'][1])
                    contact_right = 1
                    if contact_check == 0:
                        contact_check = 2
                    elif contact_check == 1:
                        contact_check = 3

                #print(contact_left, contact_right,sep=',')
                left_x_pos = DataManager().totalcapture_gt[i][-4]['position'][0]/20-0.55026
                left_y_pos = DataManager().totalcapture_gt[i][-4]['position'][1]/20-0.55026
                left_z_pos = DataManager().totalcapture_gt[i][-4]['position'][2]/20-0.55026

                right_x_pos = DataManager().totalcapture_gt[i][-1]['position'][0] / 20 - 0.55026
                right_y_pos = DataManager().totalcapture_gt[i][-1]['position'][1] / 20 - 0.55026
                right_z_pos = DataManager().totalcapture_gt[i][-1]['position'][2] / 20 - 0.55026
                #print(left_x_pos, left_y_pos, left_z_pos, sep=',')
                #print(right_x_pos, right_y_pos, right_z_pos, sep=',')

                #print(left_y_pos,right_y_pos, sep=',')
                #print('-' * 30)

                send_data = []
                send_data2 = []
                for index in bone_seq:
                    p = [0.0, 0.0, 0.0]
                    p2 = [0.0, 0.0, 0.0]
                    if index == 0:
                        p = tran.view(-1, 3).tolist()[0]
                        # p2 = tran2.view(-1, 3).tolist()[0]
                #
                    rotation = q[index].tolist()
                    frame_bone_data = {
                        "time": contact_check,
                        "name": "test",
                        "position": p,
                        "rotation": rotation,
                        "acc": [0.0, 0.0, 0.0],
                        "lp": l_foot_p,
                        "rp": r_foot_p,
                    }
                    send_data.append(frame_bone_data)

                    # frame_bone_data2 = {
                    #     "time": "1",
                    #     "name": "test",
                    #     "position": p2,
                    #     "rotation": q2[index].tolist(),
                    #     # "rotation": [bone[1].w, -bone[1].x, -bone[1].z, bone[1].y],
                    #     "acc": [0.0, 0.0, 0.0]
                    # }
                    # send_data2.append(frame_bone_data2)

                # print(send_data)\\
                data = json.dumps(send_data).encode("utf-8")
                # data2 = json.dumps(send_data2).encode("utf-8")
                # # data = json.dumps(DataManager().totalcapture_gt[i]).encode("utf-8")
                # # print(data)
                #
                TARGET_IP =  DataManager().set_ip
                # TARGET_PORT = 5005
                # sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                # sock.sendto(data, (TARGET_IP, TARGET_PORT))
                # #
                TARGET_PORT = 5007
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sock.sendto(data, (TARGET_IP, TARGET_PORT))

                # TARGET_PORT = 5008
                # sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                # sock.sendto(data2, (TARGET_IP, TARGET_PORT))

                # # #
                TARGET_PORT = 5006
                data = json.dumps(DataManager().totalcapture_gt[i]).encode("utf-8")
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sock.sendto(data, (TARGET_IP, TARGET_PORT))

                DataManager().udp_switch = False
                DataManager().udp_sending = False
                i += 1
