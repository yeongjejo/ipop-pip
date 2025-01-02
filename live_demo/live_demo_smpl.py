import torch
from pygame.time import Clock
from log_test import rotation_matrix_to_quaternion
from net.net import PIP
import articulate as art
from data_manager import DataManager
from net.smpl_net import SMPL_PIP
from protocol.udp_server import UDPServer
from protocol.udp_station_broadcast_receiver import UDPStationBroadcastReceiver
import time
import socket
from sensor.sensor_part import SensorPart


class IMUSet:
    g = 9.8
    test_i = 0

    def __init__(self, test):
        self.n_imus = 0  # 확인
        self.test = test

    def get_ipop(self):
        r = DataManager().test_r
        a = DataManager().test_acc
        smpl = DataManager().test_SMPL
        hand = DataManager().test_hand
        # print(a)

        a = -torch.tensor(a) * 9.8  # acceleration is reversed
        a = r.bmm(a.unsqueeze(-1)).squeeze(-1) + torch.tensor([0., 0., 9.8])

        return r, a, hand, smpl


def tpose_calibration_ipop_2024(test, imu_set):
    RSI = imu_set.get_ipop()[0][5].view(3, 3).t()

    RMI = torch.tensor([[1, 0, 0], [0, 1, 0], [0, 0, 1.]]).mm(RSI)

    RIS, _, handRIS, smpl = imu_set.get_ipop()

    RSB = RMI.matmul(RIS).transpose(1, 2).matmul(torch.eye(3))  # [6, 3, 3]
    RSM = RMI.matmul(smpl).transpose(1, 2).matmul(torch.eye(3))

    RSB_hand = RMI.matmul(handRIS).transpose(1, 2).matmul(torch.eye(3))  # [6, 3, 3]

    return RMI, RSB, RSB_hand, RSM


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
    net = SMPL_PIP()
    clock = Clock()
    RMI, RSB, RSM = [0, 0, 0]
    start_time = 10000
    part_sequence = [SensorPart.LEFT_LOWER_ARM, SensorPart.RIGHT_LOWER_ARM, SensorPart.LEFT_LOWER_LEG,
                     SensorPart.RIGHT_LOWER_LEG, SensorPart.HEAD, SensorPart.WAIST]
    re_tpose = True

    while not test:

        # if DataManager().t_pose_set_end == None or not DataManager().t_pose_set_end:
        #     # print("121212121")
        #     re_tpose = True
        #     continue

        if re_tpose:
            time.sleep(2)

            imu_set = IMUSet(test)
            net = SMPL_PIP()
            RMI, RSB, RSB_hand, RSM = tpose_calibration_ipop_2024(test, imu_set)

            re_tpose = False

        clock.tick(59)
        q, a, hand_q, smpl = imu_set.get_ipop()
        RMB = RMI.matmul(q).matmul(RSB)
        RMB_hand = RMI.matmul(hand_q).matmul(RSB_hand)

        smpl = RMI.matmul(smpl).matmul(RSM)
        smpl = RMB[5].t().matmul(smpl)

        # 0  (왼 허벅지)
        # 3  (오 허벅지)
        # 6  (1 허리)
        # 9  (왼 무릎)
        # 12 (오른 무릎)
        # 15 (2 허리)
        # 18 (왼쪽 발목)
        # 21 (오른쪽 발목)
        # 24 (3 허리)
        # 27 (왼 발까락)
        # 30 (오른 발까락)
        # 33 (1 목[머리])
        # 36 (왼 어깨) ----------------------------
        # 39 (오른 어깨) -------------------------
        # 42 (2 목[머리])
        # 45 (왼 어깨) -----------------------
        # 48 (오른 어깨) ----------------------
        # 51 (왼 팔꿈치)
        # 54 (오른 팔꿈치)
        # 57 (왼 손[손목])
        # 60 (오른 손[손목])\
        # print(1)
        smpl_axis = art.math.rotation_matrix_to_axis_angle(smpl)
        # print(smpl_axis.shape)
        axis_part = [51, 54, 9, 12, 42, 15, 57, 60, 18, 21, 0, 3, 45, 48]
        axis = tensor = torch.zeros(1, 69)
        for index, axis_num in enumerate(axis_part):
            axis[0][axis_num] = smpl_axis[index][0]
            axis[0][axis_num + 1] = smpl_axis[index][1]
            axis[0][axis_num + 2] = smpl_axis[index][2]

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

        if torch.all(axis == 0):
            continue

        start_time = time.time()
        pose, tran, cj, grf = net.forward_frame(aM.view(1, 6, 3).float(), RMB.view(1, 6, 3, 3).float(), axis,
                                                return_grf=True)
        elapsed_time = time.time() - start_time
        print(f"Function executed in: {elapsed_time:.4f} seconds")

        pose = art.math.rotation_matrix_to_axis_angle(pose).view(-1, 72)
        tran = tran.view(-1, 3)

        # send motion to Unity
        # s = ','.join(['%g' % v for v in pose.view(-1)]) + '#' + \
        #     ','.join(['%g' % v for v in tran.view(-1)]) + '#' + \
        #     ','.join(['%d' % v for v in cj]) + '#' + \
        #     ','.join(['%g' % v for v in test_hand_q]) + '#' + \
        #     ','.join(['%g' % v for v in DataManager().test_finger]) + '#' + \
        #     (','.join(['%g' % v for v in grf.view(-1)]) if grf is not None else '') + '$'

        s = ','.join(['%g' % v for v in pose.view(-1)]) + '#' + \
            ','.join(['%g' % v for v in tran.view(-1)]) + '#' + \
            ','.join(['%d' % v for v in cj]) + '#' + \
            (','.join(['%g' % v for v in grf.view(-1)]) if grf is not None else '') + '$'
        # print(','.join(['%g' % v for v in test_hand_q]) + '#')

        # print("-----------------------------")
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # server_address = ('192.168.201.100', 5005)
        server_address = ('192.168.0.91', 5005)
        sock.sendto(s.encode('utf-8'), server_address)




