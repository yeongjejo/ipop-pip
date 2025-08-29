import torch
from pygame.time import Clock
from net import PIP
import articulate as art
from data_manager import DataManager
import time
import socket
import json

from totalcapture.senpreprocessed import TotalcaptureIMUData


class IMUSet:
    g = 9.8
    test_i = 0

    def __init__(self):
        self.n_imus = 0


    def get_ipop(self, i):

        r = DataManager().totalcapture_imu_r[i]
        a = DataManager().totalcapture_imu_acc[i]

        
        a = -torch.tensor(a) * 9.8 / 10.0                       # acceleration is reversed
        a = r.bmm(a.unsqueeze(-1)).squeeze(-1) + torch.tensor([0., 0., 9.8])

        return r, a




def tpose_calibration_ipop_2024(imu_set):
    RSI = imu_set.get_ipop(0)[0][5].view(3, 3).t()

    # RMI = torch.tensor([[0, 1, 0], [0, 0, 1], [1, 0, 0.]]).mm(RSI)
    RMI = torch.tensor([[-1, 0, 0], [0, 1, 0], [0, 0, -1.]]).mm(RSI)
    RMI2 = torch.tensor([[1, 0, 0], [0, -1, 0], [0, 0, 1.]]).mm(RSI)
    RIS, _ = imu_set.get_ipop(0)


    RSB = RMI.matmul(RIS).transpose(1, 2).matmul(torch.eye(3))  # [6, 3, 3]
    # RSB_hand = RMI.matmul(handRIS).transpose(1, 2).matmul(torch.eye(3))  # [6, 3, 3]

    return RMI, RSB, RMI2


  
  

if __name__ == '__main__':
    TotalcaptureIMUData().setTotalcaptureIMUData()
    test = True


    clock = Clock()
    
    while True:
        imu_set = IMUSet()
        net = PIP()
        RMI, RSB, RMI2 = tpose_calibration_ipop_2024(imu_set)

        for i in range(len(DataManager.totalcapture_imu_acc)):
            clock.tick(60)
            q, a = imu_set.get_ipop(i)
            RMB = RMI.matmul(q).matmul(RSB)


            aM = a.mm(RMI2.t())
            pose, tran, cj, grf = net.forward_frame(aM.view(1, 6, 3).float(), RMB.view(1, 6, 3, 3).float(), return_grf=True)


            pose = art.math.rotation_matrix_to_axis_angle(pose).view(-1, 72)


            q = art.math.axis_angle_to_quaternion(pose)
            bone_seq = [0, 9, 15, 16, 18, 20, 17, 19, 21, 1, 4, 7, 2, 5, 8]

            send_data = []
            for i in bone_seq:
                p = [0.0, 0.0, 0.0]
                if i == 0:
                    p = tran.view(-1, 3).tolist()[0]
                frame_bone_data = {
                    "time": "1",
                    "name": "test",
                    "position": p,
                    "rotation": q[i].tolist(),
                    # "rotation": [bone[1].w, -bone[1].x, -bone[1].z, bone[1].y],
                    "acc": [0.0, 0.0, 0.0]
                }
                send_data.append(frame_bone_data)
            # print(send_data)
            data = json.dumps(send_data).encode("utf-8")
            TARGET_IP = "192.168.201.199"
            TARGET_PORT = 5005

            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.sendto(data, (TARGET_IP, TARGET_PORT))


            tran = tran.view(-1, 3)

            # send motion to Unity
            s = ','.join(['%g' % v for v in pose.view(-1)]) + '#' + \
                ','.join(['%g' % v for v in tran.view(-1)]) + '#' + \
                ','.join(['%d' % v for v in cj]) + '#' + \
                (','.join(['%g' % v for v in grf.view(-1)]) if grf is not None else '') + '$'
            # print(','.join(['%g' % v for v in test_hand_q]) + '#')

            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            server_address = ('192.168.201.199', 8888)
            # server_address = ('127.0.0.1', 8888)
            sock.sendto(s.encode('utf-8'), server_address)

