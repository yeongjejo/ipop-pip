
import pandas as pd
import os
import glob
import torch
from sensor.acc import Acc
from sensor.quaternion import Quaternion



def test_num(num):
    data = {
        0: [[1, 0, 0], [0, 1, 0], [0, 0, 1.]],
        1: [[1, 0, 0], [0, 0, 1], [0, 1, 0.]],
        
        
        2: [[0, 1, 0], [1, 0, 0], [0, 0, 1.]],
        3: [[0, 1, 0], [0, 0, 1], [1, 0, 0.]],
        
        4: [[0, 0, 1], [0, 1, 0], [1, 0, 0.]],
        5: [[0, 0, 1], [1, 0, 0], [0, 1, 0.]],
    }
    
    return data[num]


# 센서 부위를 기준으로 파일을 정렬하는 함수
def get_part_from_filename(file_path):
    # 파일 경로에서 센서 이름을 추출 (예: LEFT_LOWER_ARM, RIGHT_LOWER_ARM 등)
    filename = file_path.split('/')[-1]
    
    # print(filename)
    # 파일 이름에서 '_LEGS' 또는 '_ARM'을 기준으로 센서 부위를 추출
    if 'LEFT' in filename:
        part = filename.split('_')[0] + "_" + filename.split('_')[1]  + "_" + filename.split('_')[2]# 예: LEFT_LOWER_ARM
    elif 'RIGHT' in filename:
        part = filename.split('_')[0] + "_" + filename.split('_')[1]  + "_" + filename.split('_')[2]# 예: RIGHT_LOWER_ARM
    else:
        part = filename.split('_')[0]  # 예: WAIST
    # print(part)
    return part
    # return 1


def get_xsens_log_test_data():
    # CSV 파일들이 있는 폴더 경로
    folder_path = '/home/user/Desktop/log/ts'

    # 폴더 내의 모든 CSV 파일들의 경로를 출력
    csv_files = glob.glob(os.path.join(folder_path, '**', '*.csv'), recursive=True)

    file_paths = []

    # CSV 파일 경로 출력
    for path in csv_files:
        
        file_paths.append(path)
        

    # print(file_paths)
    # 센서 부위에 대한 순서 정의
    part_sequence = [
        'LEFT_LOWER_ARM', 'RIGHT_LOWER_ARM', 'LEFT_LOWER_LEG', 'RIGHT_LOWER_LEG', 'HEAD', 'WAIST'
    ]

    file_paths = sorted(file_paths, key=lambda x: part_sequence.index(get_part_from_filename(x)))
    # print(sorted_files)




    # 각 파일을 DataFrame으로 읽기
    dfs = [pd.read_csv(file_path, usecols=lambda column: 'Unnamed' not in column) for file_path in file_paths]



    r = []
    acc = []
    i = 0
    # 각 파일에서 동일한 인덱스의 행을 동시에 출력
    for rows in zip(*[df.iterrows() for df in dfs]):
        if i < 10:
            i += 1
            continue
        
        i += 1
        
        frame_acc = []
        frame_r = []
        for j, (index, row) in enumerate(rows):
            tq = Quaternion(float(index[5].strip()), float(index[6].strip()), float(index[7].strip()), float(index[8].strip()))
            ta = Acc(float(index[2].strip()), float(index[3].strip()), float(index[4].strip()))
            
            
            qAccX = (-1.0) * 2.0 * (tq.x * tq.z - tq.w * tq.y)
            qAccY = (-1.0) * 2.0 * (tq.y * tq.z + tq.w * tq.x)
            qAccZ = 1.0 - 2.0 * (tq.w * tq.w + tq.z * tq.z)
            
            re_acc_x = -ta.x - qAccX
            re_acc_y = -ta.y - qAccY
            re_acc_z = -ta.z - qAccZ
            
            
            
            # if j==1 and i <= 20:
            #     print(qAccZ, ta.z)
                # print(re_acc_x, re_acc_y, re_acc_z)
            
            frame_acc.append([re_acc_x, re_acc_y, re_acc_z])
            # print("-"*50)
            frame_r.append(Quaternion(float(index[5].strip()), float(index[6].strip()), float(index[7].strip()), float(index[8].strip())).quaternion_to_rotation_matrix())
            # print(f"File {j+1} - Row {index}: {row.tolist()}")
            
        frame_r = torch.squeeze(torch.stack(frame_r))
        r.append(frame_r)
        acc.append(frame_acc)
        
        
        # frame_r = torch.tensor(frame_r)
        # print(frame_r)
        # print("=" * 50)  # 구분선
        
    return acc, r, ""
        
    
