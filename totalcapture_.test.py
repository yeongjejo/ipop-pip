import torch
from pygame.time import Clock
from net import PIP
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
    TotalcaptureIMUData().setTotalcaptureIMUData()
    print(222)
    TotalcaptureViconData().setTotalcaptureViconData()
    print(1111)

    clock = Clock()
    #
    # # 속도 모델 가져오기
    model = VelocityTFWModule.load_from_checkpoint("./velocitymodel/checkpoints/250912-TC-Transformer.ckpt")
    model.eval()
    data_path = "./velocitymodel/preprocessed_data"
    velocity_input_list = TotalCaptureDataset(data_path=data_path, mode='valid', name="version_tc.pickle")

    # 속도 모델 가져오기
    # model = VelocityTFWModule.load_from_checkpoint("velocitymodel/checkpoints/250913-TC_GRF-Transformer-v4.ckpt")
    # model.eval()
    # data_path = "velocitymodel/preprocessed_data"
    # velocity_input_list = TotalCaptureDataset(data_path=data_path, mode='valid', name="version_tc_s5_acting3.pickle")

    x, y = velocity_input_list[0]


    while True:
        imu_set = IMUSet()
        net = PIP()
        net2 = PIP()
        RMI, RSB, RMI2, RSB2 = tpose_calibration_ipop_2024(imu_set)
        i = 0
        x_p = 0.0
        y_p = 11.0
        z_p = 0.0
        pre_speed = [0.0, 0.0, 0.0]

        while i < len(DataManager.totalcapture_imu_acc):
        # while i < 500:
            if i == 0:
                x_p = 0.0
                y_p = 11.0
                z_p = 0.0
                pre_speed = [0.0, 0.0, 0.0]
            q, a, q2 = imu_set.get_ipop(i)
            RMB = RMI.matmul(q).matmul(RSB)
            RMB2 = RMI.matmul(q2).matmul(RSB2)

            if not DataManager().udp_switch and not DataManager().udp_sending:
                # clock.tick(60)
                # print(art.math.axis_angle_to_quaternion(art.math.rotation_matrix_to_axis_angle(RMB2)))

                # 프리 모델이 적용할 데이터 전송
                send_data = []

                x, y = velocity_input_list[i]
                x = x.unsqueeze(0)

                x = x.to("cpu")
                with torch.no_grad():
                    outputs = model(x).squeeze(0)
                    outputs = velocity_input_list.denormalize_tensor(outputs)
                    model_v = outputs.cpu().tolist()

                pre_speed[0] = model_v[0]
                pre_speed[1] = model_v[1]
                pre_speed[2] = model_v[2]

                x_p = x_p + model_v[0] * 1.6e-2
                y_p = y_p + model_v[1] * 1.6e-2
                z_p = z_p + model_v[2] * 1.6e-2


                for test in art.math.axis_angle_to_quaternion(art.math.rotation_matrix_to_axis_angle(RMB2)):
                    test = test.tolist()
                    # print(DataManager().premodel_root_p[i])
                    # print(DataManager().pre_position)
                    # print('-'*30)
                    frame_bone_data = {
                        "time": "5",
                        "name": "test",
                        # "position": [-x_p / 0.0254 / 3.0, y_p / 0.0254 / 3.0, -z_p / 0.0254 / 3.0],
                        # "position": DataManager().premodel_root_p[i],
                        "position": DataManager().pre_position,
                        "rotation": [test[0], test[1], test[2], test[3]],
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
                aM = a.mm(RMI2.t())

                x, y = velocity_input_list[i]
                x = x.unsqueeze(0)


                x = x.to("cpu")
                with torch.no_grad():
                    outputs = model(x).squeeze(0)
                    model_grf = outputs.cpu().tolist()

                pose, tran, cj, grf, contact_check = net.forward_frame(aM.view(1, 6, 3).float(), RMB.view(1, 6, 3, 3).float(),  return_grf=True, check_rbdl=False)
                # pose2, tran2, cj2, grf2 = net2.forward_frame(aM.view(1, 6, 3).float(), RMB.view(1, 6, 3, 3).float(), return_grf=True, check_rbdl=True)



                # pose2 = art.math.rotation_matrix_to_axis_angle(pose2).view(-1, 72)
                pose = art.math.rotation_matrix_to_axis_angle(pose).view(-1, 72)


                q = art.math.axis_angle_to_quaternion(pose)
                # q2 = art.math.axis_angle_to_quaternion(pose2)
                bone_seq = [0, 3, 6, 9, 12, 15, 13, 16, 18, 20, 14, 17, 19, 21, 1, 4, 7, 2, 5, 8]

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
                        # "rotation": [bone[1].w, -bone[1].x, -bone[1].z, bone[1].y],
                        "acc": [0.0, 0.0, 0.0]
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
                TARGET_IP = "192.168.201.199"
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


            # 유니티 전송용
            # tran = tran.view(-1, 3)
            #
            # # send motion to Unity
            # s = ','.join(['%g' % v for v in pose.view(-1)]) + '#' + \
            #     ','.join(['%g' % v for v in tran.view(-1)]) + '#' + \
            #     ','.join(['%d' % v for v in cj]) + '#' + \
            #     (','.join(['%g' % v for v in grf.view(-1)]) if grf is not None else '') + '$'
            # # print(','.join(['%g' % v for v in test_hand_q]) + '#')
            #
            # sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            # server_address = ('192.168.201.199', 8888)
            # # server_address = ('127.0.0.1', 8888)
            # sock.sendto(s.encode('utf-8'), server_address)
            #
