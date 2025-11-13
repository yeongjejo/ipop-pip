import numpy as np
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





if __name__ == '__main__':
    # PreModelServer().start()
    print('데이터셋 준비중..)')


    clock = Clock()

    while True:
        net = PIP()
        i = 0

        rot_ckpt = torch.load("v_vrot.pt", map_location="cpu")
        acc_ckpt = torch.load("v_vacc.pt", map_location="cpu")
        # joint_ckpt = torch.load("test6.pt", map_location="cpu")
        # joint_ckpt = torch.load("v_new_pose.pt", map_location="cpu")
        joint_ckpt = torch.load("v_joint_gt.pt", map_location="cpu")
        contactv_ckpt = torch.load("v_contact.pt", map_location="cpu")


        ten_joint_pose = torch.load("data/dataset_work/v_new_pose.pt", map_location="cpu")
        ten_joint_rot = torch.load("data/dataset_work/v_vrot.pt", map_location="cpu")
        ten_joint_acc = torch.load("data/dataset_work/v_vacc.pt", map_location="cpu")



        scenario = 52
        # 첫 번째 몇 개 요소만 보기
        pre_l_p = np.zeros(3)
        pre_r_p = np.zeros(3)

        for rot, acc, joint, con_list, ten_pose, ten_rot, ten_acc in zip(rot_ckpt[scenario], acc_ckpt[scenario], joint_ckpt[scenario], contactv_ckpt[scenario], ten_joint_pose[scenario], ten_joint_rot[scenario], ten_joint_acc[scenario]):
            clock.tick(60)

            contact_check2 = 0
            # print(joint.shape)
            # print(acc.shape)
            # print(rot.shape)
            #
            # print('rot', rot.shape)
            # print('acc', acc.shape)
            # print('-'*30)

            # q, a, q2 = imu_set.get_ipop(i)
            # RMB = RMI.matmul(q).matmul(RSB)
            # RMB2 = RMI.matmul(q2).matmul(RSB2)
            #
            # aM = a.mm(RMI2.t())

            # _, tran, cj, grf, contact_check = net.forward_frame(acc.view(1, 6, 3).float(), rot.view(1, 6, 3, 3).float(), joint.view(1, 24, 3), ten_pose, ten_rot, ten_acc,  return_grf=True, check_rbdl=False)
            _, tran, cj, grf = net.forward_frame(acc.view(1, 6, 3).float(), rot.view(1, 6, 3, 3).float(), joint.view(1, 24, 3), ten_pose, ten_rot, ten_acc,  return_grf=True, check_rbdl=False)

            contact_check = 0
            # torch.Size([24, 3])
            # print('shpae', pose.shape)


            pose = torch.zeros(24, 3)
            joint_seq = [16, 18, 17, 19, 1, 4, 2, 5, 0, 9]
            axis_rot = art.math.rotation_matrix_to_axis_angle(ten_rot)
            for j, j_angle in enumerate(axis_rot):
                pose[joint_seq[j]] = j_angle

            # pose = pose.view(72)


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
                    # p = [0.0, 0.0, 0.0]
                    p = tran.view(-1, 3).tolist()[0]
                    # p2 = tran2.view(-1, 3).tolist()[0]

                # print("확인용", joint[index].tolist())

                if index == 0:
                    if con_list[0] == 1.0:
                        contact_check2 = 1
                        tesq[0] = 1
                    if con_list[1] == 1.0:
                        tesq[1] = 1
                        if contact_check2 == 1:
                            contact_check2 = 3
                        else:
                            contact_check2 = 2

                    # print(tesq)
                    # print('-'*30)

                    #
                    #
                    # if pre_l_p.sum() == 0.0:
                    #     contact_check2 = 3
                    # else:
                    #     # print(joint[11].clone().numpy())
                    #     distances = np.linalg.norm(joint[10].clone().numpy() - pre_l_p)
                    #     if distances < 0.008:
                    #         contact_check2 = 1
                    #         print(1)
                    #
                    # if pre_r_p.sum() == 0.0:
                    #     contact_check2 = 3
                    # else:
                    #     # print(joint[11].clone().numpy())
                    #     distances = np.linalg.norm(joint[11].clone().numpy() - pre_r_p)
                    #     if distances < 0.008:
                    #         if contact_check2 == 1:
                    #             contact_check2 = 3
                    #         else:
                    #             contact_check2 = 2
                    #
                    #     # print(f"{distances:.6f}")
                    # pre_l_p = joint[10].clone().numpy()
                    # pre_r_p = joint[11].clone().numpy()

                rotation = q[index].tolist()
                frame_bone_data = {
                    "time": contact_check,
                    "name": "test",
                    "position": p,
                    "rotation": rotation,
                    # "rotation": [bone[1].w, -bone[1].x, -bone[1].z, bone[1].y],
                    "acc": [0.0, 0.0, 0.0],
                    "lp": [0.0, 0.0, 0.0],
                    "rp": [0.0, 0.0, 0.0],
                }

                frame_bone_data2 = {
                    "time": contact_check,
                    "name": "test",
                    "position": joint[index].tolist(),
                    "rotation": [1.0, 0.0, 0.0, 0.0],
                    # "rotation": [bone[1].w, -bone[1].x, -bone[1].z, bone[1].y],
                    "acc": [0.0, 0.0, 0.0],
                    "lp": [0.0, 0.0, 0.0],
                    "rp": [0.0, 0.0, 0.0],
                }

                send_data.append(frame_bone_data)
                send_data2.append(frame_bone_data2)

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

            TARGET_PORT = 5006
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.sendto(data2, (TARGET_IP, TARGET_PORT))

            TARGET_PORT = 5007
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.sendto(data, (TARGET_IP, TARGET_PORT))


            i += 1
        break