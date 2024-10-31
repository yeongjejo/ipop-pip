import articulate as art
from net import PIP
import torch
import os
import csv


def show_3d(predict_values, ground_values):
    import numpy as np
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D

    print(predict_values.shape)
    print(ground_values.shape)
    print(predict_values)
    pose_values_1 = predict_values.reshape(24,3)
    pose_values_2 = ground_values.reshape(24,3)

    # 2개의 피규어 생성
    fig = plt.figure(figsize=(12, 6))

    # 첫 번째 포즈 시각화
    ax1 = fig.add_subplot(121, projection='3d')
    ax1.scatter(pose_values_1[:, 0], pose_values_1[:, 1], pose_values_1[:, 2], c='r', marker='o')
    ax1.set_title('Predict Pose')
    ax1.set_xlabel('X Label')
    ax1.set_ylabel('Y Label')
    ax1.set_zlabel('Z Label')

    # 두 번째 포즈 시각화
    ax2 = fig.add_subplot(122, projection='3d')
    ax2.scatter(pose_values_2[:, 0], pose_values_2[:, 1], pose_values_2[:, 2], c='b', marker='o')
    ax2.set_title('Ground Truth')
    ax2.set_xlabel('X Label')
    ax2.set_ylabel('Y Label')
    ax2.set_zlabel('Z Label')

    plt.show()

def show_2d(predict_values, ground_values):
    import numpy as np
    import matplotlib.pyplot as plt
    
    pose_values_1 = predict_values.reshape(24,3)
    pose_values_2 = ground_values.reshape(24,3)


    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))

    # 첫 번째 포즈 시각화 (x, y)
    ax1.scatter(pose_values_1[:, 1], pose_values_1[:, 2], c='r', marker='o')
    ax1.set_title('Predict (X, Y)')
    ax1.set_xlabel('X Label')
    ax1.set_ylabel('Y Label')
    ax1.grid(True)

    # 두 번째 포즈 시각화 (x, y) 0/1 1/2 , 1/0 0/2
    ax2.scatter(pose_values_2[:, 0], pose_values_2[:, 2], c='b', marker='o')
    ax2.set_title('Ground (X, Y)')
    ax2.set_xlabel('X Label')
    ax2.set_ylabel('Y Label')
    ax2.grid(True)
    
    plt.show()


data_dir = './data/dataset_work/IPOP'
data_dir_1 = './data/dataset_work/DIP_IMU'
net = PIP()

_, _, poses_1, _ = torch.load(os.path.join(data_dir_1, 'test.pt')).values()
accs, rots, poses, _ = torch.load(os.path.join(data_dir, 'test.pt')).values()

all_pose = []
all_tran = []
for i in range(300, 2000):
    pose, tran = net.forward_frame(accs[0][i].view(1, 6, 3), rots[0][i].view(1, 6, 3, 3),return_grf=False)
    pose = art.math.rotation_matrix_to_axis_angle(pose).view(-1, 72).tolist()
    tran = tran.view(-1, 3).tolist()
    # 첫 번째 포즈 시각화
    all_pose.append(pose)
    all_tran.append(tran)
    
with open('pose.csv', mode='w', newline='', encoding='utf-8') as file:
    writer = csv.writer(file)
    # 리스트의 각 행을 파일에 작성
    writer.writerows(all_pose)
    
with open('tran.csv', mode='w', newline='', encoding='utf-8') as file:
    writer = csv.writer(file)
    # 리스트의 각 행을 파일에 작성
    writer.writerows(all_tran)
    #show_3d(pose, poses_1[0][i].view(-1,72))# poses[0][0].view(-1,72))
    
