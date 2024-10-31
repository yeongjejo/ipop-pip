import numpy as np
import time
import socket
import torch
import struct
from pygame.time import Clock
from net import PIP
import articulate as art
import os
from config import *
from pynput import keyboard

class IMUSet:
    g = 9.8

    def __init__(self):

        self.n_imus = 6
        self.cs = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.cs.bind(('', 8777))
        self.cs.settimeout(3)

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

            R = art.math.quaternion_to_rotation_matrix(torch.tensor(q))  # rotation is not changed
            a = -torch.tensor(a) / 1000 * self.g  # acceleration is reversed
            a = R.bmm(a.unsqueeze(-1)).squeeze(-1) + torch.tensor([0., 0., self.g])  # calculate global free acceleration
            return R, a

        except socket.timeout:
            print('[warning] no imu data received for 3 seconds')

    def get_ipop(self):
        try:
            data, addr = self.cs.recvfrom(172)
            format = '>f'
            parsed_data = [struct.unpack(format, data[i:i + 4])[0] for i in range(2, len(data) - 2, 4)]
            np_data = np.array(parsed_data)

            q, a = [], []
            for i in range(0, 42, 7):
                q.append([np.float32(np_data[i]), np.float32(np_data[i + 1]), np.float32(np_data[i + 2]), np.float32(np_data[i + 3])])
                a.append([np.float32(np_data[i + 4]), np.float32(np_data[i + 5]), np.float32(np_data[i + 6])])

            return q, a

        except socket.timeout:
            print('[warning] no imu data received for 3 seconds')

    def clear(self):
        pass

def tpose_calibration_ipop():
    imu_set.clear()
    RSI = art.math.quaternion_to_rotation_matrix(torch.tensor(imu_set.get_ipop()[0][0])).view(3, 3).t()
    RMI = torch.tensor([[0, 1, 0], [0, 0, 1], [1, 0, 0]], dtype=torch.float).mm(RSI)
    #torch.save(RMI, os.path.join(paths.temp_dir, 'RMI.pt'))

    imu_set.clear()
    RIS = art.math.quaternion_to_rotation_matrix(torch.tensor(imu_set.get_ipop()[0]))
    RSB = RMI.matmul(RIS).transpose(1, 2).matmul(torch.eye(3))
    return RMI, RSB

def tpose_calibration_noitom():

    c = input('Used cached RMI? [y]/n    (If you choose no, put imu 1 straight (x = Forward, y = Left, z = Up).')
    if c == 'n' or c == 'N':
        imu_set.clear()
        RSI = imu_set.get_noitom()[0][0].view(3, 3).t()
        RMI = torch.tensor([[1, 0, 0], [0, 0, 1], [0, -1, 0.]]).mm(RSI)
        torch.save(RMI, os.path.join(paths.temp_dir, 'RMI.pt'))
    else:
        print("여기!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        RMI = torch.load(os.path.join(paths.temp_dir, 'RMI.pt'))

    input('Stand straight in T-pose and press enter. The calibration will begin in 3 seconds')
    time.sleep(3)
    imu_set.clear()
    RIS = imu_set.get_noitom()[0]
    RSB = RMI.matmul(RIS).transpose(1, 2).matmul(torch.eye(3))
    return RMI, RSB

if __name__ == '__main__':
    os.makedirs(paths.temp_dir, exist_ok=True)
    os.makedirs(paths.live_record_dir, exist_ok=True)

    is_executable = False
    server_for_unity = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_for_unity.bind(('', 8888))
    server_for_unity.listen(1)
    print('Server start. Waiting for unity3d to connect.')

    if paths.unity_file != '' and os.path.exists(paths.unity_file):
        os.system(f"xdg-open {os.path.abspath(paths.unity_file)}")
        is_executable = True

    conn, addr = server_for_unity.accept()

    time.sleep(2)
    imu_set = IMUSet()
    RMI, RSB = tpose_calibration_ipop()
    net = PIP()
    clock = Clock()
    imu_set.clear()
    data = {'RMI': RMI, 'RSB': RSB, 'aM': [], 'RMB': []}

    def on_press(key):
        global is_executable
        if key == keyboard.Key.esc:  # Change this to your desired quit key
            is_executable = True

    listener = keyboard.Listener(on_press=on_press)
    listener.start()

    while True:
        clock.tick(60)

        q, a = imu_set.get_ipop()
        RMB = RMI.matmul(art.math.quaternion_to_rotation_matrix(torch.tensor(q))).matmul(RSB)
        a = torch.tensor(a)
        aM = a.mm(RMI.t())

        pose, tran, cj, grf = net.forward_frame(aM.view(1, 6, 3).float(), RMB.view(1, 6, 3, 3).float(), return_grf=True)
        pose = art.math.rotation_matrix_to_axis_angle(pose).view(-1, 72)
        tran = tran.view(-1, 3)

        # send motion to Unity
        s = ','.join(['%g' % v for v in pose.view(-1)]) + '#' + \
            ','.join(['%g' % v for v in tran.view(-1)]) + '#' + \
            ','.join(['%d' % v for v in cj]) + '#' + \
            (','.join(['%g' % v for v in grf.view(-1)]) if grf is not None else '') + '$'

        try:
            conn.send(s.encode('utf8'))
        except:
            break

        data['aM'].append(aM)
        data['RMB'].append(RMB)

        if is_executable:
            break

        print('\rfps: ', clock.get_fps(), end='')

    data['aM'] = torch.stack(data['aM'])
    data['RMB'] = torch.stack(data['RMB'])
    print('\rFinish.')

