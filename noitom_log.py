import csv

import torch

from sensor.acc import Acc
from sensor.quaternion import Quaternion


def get_noitom_log_test_data():
    
    # 필요한 부위 및 데이터 정리
    part_data = ['Hips', 'RightLeg', 'LeftLeg', 'Head', 'RightForeArm', 'LeftForeArm']
    list_data = ['-Sensor-Quat-x', '-Sensor-Quat-y', '-Sensor-Quat-z', '-Sensor-Quat-w','-Sensor-Acce-x', '-Sensor-Acce-y', '-Sensor-Acce-z', '-Bone-Quat-x', '-Bone-Quat-y', '-Bone-Quat-z', '-Bone-Quat-w']
    
    part_list = []
    for part in part_data:
        for list in list_data:
            part_list.append(part + list)
    
    # print(part_list)
    
    # key   : 추출해야 부위(part_list)
    # value : 2차원 리스트
    #         0 : 추출 해야 되는 인덱스
    #         1 : 저장할[저장된] 리스트
    # ex    :  data_hash = {'Hips-Sensor-Quat-x' : [1, [0.111, 0.222, 0.333 ...]], ...}
    data_hash = {} 
    

    # CSV 파일 경로
    file_path = '/home/user/Desktop/log/noitom/take001_chr01.csv'
    
    data_num = 0
    
    reader = None
    # CSV 파일 열기
    with open(file_path, mode='r', newline='', encoding='utf-8') as file:
        reader = csv.reader(file)
        # 파일의 각 행을 읽어 출력
        for i, csv_data in enumerate(reader):
            if i == 0:
                data_num = csv_data[0]
            elif i == 1:
                for j, part in enumerate(csv_data):
                    if part in part_list:
                        # data_hash[j] = []
                        data_hash[part] = [j , []]
                
                continue
            
            # for key in data_hash.keys():
            #     data_hash[key].append(csv_data[key])
                
                
            for key, value in data_hash.items():
                data_hash[key][1].append(csv_data[data_hash[key][0]])
                
                
    # 센서 부위에 대한 순서 정의
    part_sequence = [
        'LEFT_LOWER_ARM', 'RIGHT_LOWER_ARM', 'LEFT_LOWER_LEG', 'RIGHT_LOWER_LEG', 'HEAD', 'WAIST'
    ]
    
    # for key, value in data_hash.items():
    #     # print(len(value[1]))
    #     print(key)


    LEFT_LOWER_ARM_q = cul_noitom_qua(data_hash['LeftForeArm-Sensor-Quat-w'][1], data_hash['LeftForeArm-Sensor-Quat-x'][1], data_hash['LeftForeArm-Sensor-Quat-y'][1], data_hash['LeftForeArm-Sensor-Quat-z'][1])
    RIGHT_LOWER_ARM_q = cul_noitom_qua(data_hash['RightForeArm-Sensor-Quat-w'][1], data_hash['RightForeArm-Sensor-Quat-x'][1], data_hash['RightForeArm-Sensor-Quat-y'][1], data_hash['RightForeArm-Sensor-Quat-z'][1])
    LEFT_LOWER_LEG_q = cul_noitom_qua(data_hash['LeftLeg-Sensor-Quat-w'][1], data_hash['LeftLeg-Sensor-Quat-x'][1], data_hash['LeftLeg-Sensor-Quat-y'][1], data_hash['LeftLeg-Sensor-Quat-z'][1])
    RIGHT_LOWER_LEG_q = cul_noitom_qua(data_hash['RightLeg-Sensor-Quat-w'][1], data_hash['RightLeg-Sensor-Quat-x'][1], data_hash['RightLeg-Sensor-Quat-y'][1], data_hash['RightLeg-Sensor-Quat-z'][1])
    HEAD_q = cul_noitom_qua(data_hash['Head-Sensor-Quat-w'][1], data_hash['Head-Sensor-Quat-x'][1], data_hash['Head-Sensor-Quat-y'][1], data_hash['Head-Sensor-Quat-z'][1])
    WAIST_q = cul_noitom_qua(data_hash['Hips-Sensor-Quat-w'][1], data_hash['Hips-Sensor-Quat-x'][1], data_hash['Hips-Sensor-Quat-y'][1], data_hash['Hips-Sensor-Quat-z'][1])
 
    LEFT_LOWER_ARM_acc = cul_noitom_acc(data_hash['LeftForeArm-Sensor-Acce-x'][1], data_hash['LeftForeArm-Sensor-Acce-y'][1], data_hash['LeftForeArm-Sensor-Acce-z'][1])
    RIGHT_LOWER_ARM_acc = cul_noitom_acc(data_hash['RightForeArm-Sensor-Acce-x'][1], data_hash['RightForeArm-Sensor-Acce-y'][1], data_hash['RightForeArm-Sensor-Acce-z'][1])
    LEFT_LOWER_LEG_acc = cul_noitom_acc(data_hash['LeftLeg-Sensor-Acce-x'][1], data_hash['LeftLeg-Sensor-Acce-y'][1], data_hash['LeftLeg-Sensor-Acce-z'][1])
    RIGHT_LOWER_LEG_acc = cul_noitom_acc(data_hash['RightLeg-Sensor-Acce-x'][1], data_hash['RightLeg-Sensor-Acce-y'][1], data_hash['RightLeg-Sensor-Acce-z'][1])
    HEAD_acc = cul_noitom_acc(data_hash['Head-Sensor-Acce-x'][1], data_hash['Head-Sensor-Acce-y'][1], data_hash['Head-Sensor-Acce-z'][1])
    WAIST_acc = cul_noitom_acc(data_hash['Hips-Sensor-Acce-x'][1], data_hash['Hips-Sensor-Acce-y'][1], data_hash['Hips-Sensor-Acce-z'][1])
    
    
    # for l_arm_q, r_arm_q, l_leg_q, r_leg_q, head_q, waist_q, l_arm_acc, r_arm_acc, l_leg_acc, r_lef_acc, head_acc, waist_acc in zip(LEFT_LOWER_ARM_q, RIGHT_LOWER_ARM_q, LEFT_LOWER_LEG_q, )
    
    r = []
    acc = []
    for i in range(int(data_num)):
        frame_acc = []
        frame_r = []
        
        frame_acc.append([LEFT_LOWER_ARM_acc[i].x, LEFT_LOWER_ARM_acc[i].y, LEFT_LOWER_ARM_acc[i].z])
        frame_acc.append([RIGHT_LOWER_ARM_acc[i].x, RIGHT_LOWER_ARM_acc[i].y, RIGHT_LOWER_ARM_acc[i].z])
        frame_acc.append([LEFT_LOWER_LEG_acc[i].x, LEFT_LOWER_LEG_acc[i].y, LEFT_LOWER_LEG_acc[i].z])
        frame_acc.append([RIGHT_LOWER_LEG_acc[i].x, RIGHT_LOWER_LEG_acc[i].y, RIGHT_LOWER_LEG_acc[i].z])
        frame_acc.append([HEAD_acc[i].x, HEAD_acc[i].y, HEAD_acc[i].z])
        frame_acc.append([WAIST_acc[i].x, WAIST_acc[i].y, WAIST_acc[i].z])
        
        
        frame_r.append(LEFT_LOWER_ARM_q[i].quaternion_to_rotation_matrix())
        frame_r.append(RIGHT_LOWER_ARM_q[i].quaternion_to_rotation_matrix())
        frame_r.append(LEFT_LOWER_LEG_q[i].quaternion_to_rotation_matrix())
        frame_r.append(RIGHT_LOWER_LEG_q[i].quaternion_to_rotation_matrix())
        frame_r.append(HEAD_q[i].quaternion_to_rotation_matrix())
        frame_r.append(WAIST_q[i].quaternion_to_rotation_matrix())
        
        
        frame_r = torch.squeeze(torch.stack(frame_r))
        r.append(frame_r)
        acc.append(frame_acc)
        
    print(acc)

        
    
    
    
    
def cul_noitom_qua(part_w, part_x, part_y, part_z):
    list = []
    
    for w, x, y, z in zip(part_w, part_x, part_y, part_z):
        list.append(Quaternion(float(w), float(x), float(y), float(z)))
        
    return list

def cul_noitom_acc(part_x, part_y, part_z):
    list = []
    
    for x, y, z in zip(part_x, part_y, part_z):
        list.append(Acc(float(x), float(y), float(z)))
        
    return list
            
            
    
    
    # 필요한 부위 및 데이터 정리
    # part_data = ['Hips', 'RightLeg', 'LeftLeg', 'Head', 'RightForeArm', 'LeftForeArm']
    # list_data = ['-Sensor-Quat-x', '-Sensor-Quat-y', '-Sensor-Quat-z', '-Sensor-Quat-w','-Sensor-Acce-x', '-Sensor-Acce-y', '-Sensor-Acce-z', '-Bone-Quat-x', '-Bone-Quat-y', '-Bone-Quat-z', '-Bone-Quat-w']
    
    # part_list = []
    # for part in part_data:
    #     for list in list_data:
    #         part_list.append(part + list)
    
    # print(part_list)
    

    
    
    # for i, v in enumerate(a):
    #     print(str(i) + " : " + str(v))

get_noitom_log_test_data()


