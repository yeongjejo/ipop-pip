r"""
    Preprocess DIP-IMU and TotalCapture test dataset.
    Synthesize AMASS dataset.

    Please refer to the `paths` in `config.py` and set the path of each dataset correctly.
"""


import csv
import articulate as art
import torch
import os
import pickle
from config import paths, amass_data
import numpy as np
import glob

from log_test import rotation_matrix_to_quaternion
from sensor.sensor_part import SensorPart

def process_dipimu():
    imu_mask = [7, 8, 11, 12, 0, 2]
    test_split = ['s_07']
    accs, oris, poses, trans = [], [], [], []

    for subject_name in test_split:
        for motion_name in os.listdir(os.path.join(paths.raw_dipimu_dir, subject_name)):
            path = os.path.join(paths.raw_dipimu_dir, subject_name, motion_name)
            data = pickle.load(open(path, 'rb'), encoding='latin1')
            acc = torch.from_numpy(data['imu_acc'][:, imu_mask]).float()
            ori = torch.from_numpy(data['imu_ori'][:, imu_mask]).float()
            pose = torch.from_numpy(data['gt']).float() 

            # fill nan with nearest neighbors
            for _ in range(4):
                acc[1:].masked_scatter_(torch.isnan(acc[1:]), acc[:-1][torch.isnan(acc[1:])])
                ori[1:].masked_scatter_(torch.isnan(ori[1:]), ori[:-1][torch.isnan(ori[1:])])
                acc[:-1].masked_scatter_(torch.isnan(acc[:-1]), acc[1:][torch.isnan(acc[:-1])])
                ori[:-1].masked_scatter_(torch.isnan(ori[:-1]), ori[1:][torch.isnan(ori[:-1])])
                
            acc, ori, pose = acc[6:-6], ori[6:-6], pose[6:-6]


            # print(acc[0][0])
            # print(ori[0][0])
            # print(pose)
            # print(motion_name)

            if torch.isnan(acc).sum() == 0 and torch.isnan(ori).sum() == 0 and torch.isnan(pose).sum() == 0:
                if motion_name == '04.pkl':
                        # 998
                        # print(len(ori), len(ori[0]), len(ori[0][0]), len(ori[0][0][0]))
                        # num = int((len(ori)/2))
                        # t = ori[num:]
                        # print(acc.tolist())
                        # print(ori[0])
                        
                        tq = [" qW, ", " qX, ", " qY, ", " qZ, "]
                        tacc = [" accX, ", " accY, ", " accZ, "]
                        part_sequence = [SensorPart.LEFT_LOWER_ARM, SensorPart.RIGHT_LOWER_ARM, SensorPart.LEFT_LOWER_LEG, SensorPart.RIGHT_LOWER_LEG, SensorPart.HEAD, SensorPart.WAIST]
        
                        title_str = ""
                        for data in part_sequence:
                            for q in tq:
                                title_str += str(data) + str(q)
                        
                        for data in part_sequence:
                            for acct in tacc:
                                title_str += str(data) + str(acct)
                        
                        title_str += "\n"
                        # print(title_str)
                        for item1, item2 in zip(ori, acc):
                            q = rotation_matrix_to_quaternion(item1).tolist()
                            acc = item2.tolist()
                            
                            q_str = str(q).replace('[', '').replace(']', '')
                            acc_str = str(acc).replace('[', '').replace(']', '')
                            
                            # print('----')
                            title_str += (q_str + ", " + acc_str + ", " )
                            title_str += "\n"
                            # print(q, acc)
                            
                        # 문자열을 리스트로 변환 (쉼표로 구분된 데이터)
                        lines = title_str.split('\n')

                        # CSV 파일로 저장
                        with open('output.csv', mode='w', newline='', encoding='utf-8') as file:
                            writer = csv.writer(file)
    
                            # 각 줄을 쉼표로 나누어서 CSV에 저장
                            for line in lines:
                                data = line.split(',')  # 쉼표로 구분된 데이터 리스트로 변환
                                writer.writerow(data)  # 리스트를 한 행으로 저장
                        
                        return acc.tolist(), ori, pose
                
                
                accs.append(acc.clone())
                oris.append(ori.clone())
                poses.append(pose.clone())
                trans.append(torch.zeros(pose.shape[0], 3))  # dip-imu does not contain translations
            else:
                print('DIP-IMU: %s/%s has too much nan! Discard!' % (subject_name, motion_name))

    os.makedirs(paths.dipimu_dir, exist_ok=True)
    torch.save({'acc': accs, 'ori': oris, 'pose': poses, 'tran': trans}, os.path.join(paths.dipimu_dir, 'test.pt'))
    print('Preprocessed DIP-IMU dataset is saved at', paths.dipimu_dir)


def process_totalcapture():
    inches_to_meters = 0.0254
    file_name = 'gt_skel_gbl_pos.txt'

    accs, oris, poses, trans = [], [], [], []
    for file in sorted(os.listdir(paths.raw_totalcapture_dip_dir)):
        data = pickle.load(open(os.path.join(paths.raw_totalcapture_dip_dir, file), 'rb'), encoding='latin1')
        ori = torch.from_numpy(data['ori']).float()[:, torch.tensor([2, 3, 0, 1, 4, 5])]
        acc = torch.from_numpy(data['acc']).float()[:, torch.tensor([2, 3, 0, 1, 4, 5])]
        pose = torch.from_numpy(data['gt']).float().view(-1, 24, 3)

        # acc/ori and gt pose do not match in the dataset
        if acc.shape[0] < pose.shape[0]:
            pose = pose[:acc.shape[0]]
        elif acc.shape[0] > pose.shape[0]:
            acc = acc[:pose.shape[0]]
            ori = ori[:pose.shape[0]]

        assert acc.shape[0] == ori.shape[0] and ori.shape[0] == pose.shape[0]
        accs.append(acc)    # N, 6, 3
        oris.append(ori)    # N, 6, 3, 3
        poses.append(pose)  # N, 24, 3

    for subject_name in ['S1', 'S2', 'S3', 'S4', 'S5']:
        for motion_name in sorted(os.listdir(os.path.join(paths.raw_totalcapture_official_dir, subject_name))):
            if subject_name == 'S5' and motion_name == 'acting3':
                continue   # no SMPL poses
            f = open(os.path.join(paths.raw_totalcapture_official_dir, subject_name, motion_name, file_name))
            line = f.readline().split('\t')
            index = torch.tensor([line.index(_) for _ in ['LeftFoot', 'RightFoot', 'Spine']])
            pos = []
            while line:
                line = f.readline()
                pos.append(torch.tensor([[float(_) for _ in p.split(' ')] for p in line.split('\t')[:-1]]))
            pos = torch.stack(pos[:-1])[:, index] * inches_to_meters
            pos[:, :, 0].neg_()
            pos[:, :, 2].neg_()
            trans.append(pos[:, 2] - pos[:1, 2])   # N, 3

    # match trans with poses
    for i in range(len(accs)):
        if accs[i].shape[0] < trans[i].shape[0]:
            trans[i] = trans[i][:accs[i].shape[0]]
        assert trans[i].shape[0] == accs[i].shape[0]

    # remove acceleration bias
    for iacc, pose, tran in zip(accs, poses, trans):
        pose = art.math.axis_angle_to_rotation_matrix(pose).view(-1, 24, 3, 3)
        _, _, vert = body_model.forward_kinematics(pose, tran=tran, calc_mesh=True)
        vacc = _syn_acc(vert[:, vi_mask])
        for imu_id in range(6):
            for i in range(3):
                d = -iacc[:, imu_id, i].mean() + vacc[:, imu_id, i].mean()
                iacc[:, imu_id, i] += d

    os.makedirs(paths.totalcapture_dir, exist_ok=True)
    torch.save({'acc': accs, 'ori': oris, 'pose': poses, 'tran': trans},
               os.path.join(paths.totalcapture_dir, 'test.pt'))
    print('Preprocessed TotalCapture dataset is saved at', paths.totalcapture_dir)


if __name__ == '__main__':
    # process_amass()
    process_dipimu()
    # process_totalcapture()
