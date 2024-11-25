import pickle
import time

import torch

from sensor.sensor_part import SensorPart
import numpy as np

import os
import pickle
from config import paths, amass_data


class DataManager():
    _instance = None  # 싱글톤 용도
    __check = True
    __sensor_data = {part: [] for part in SensorPart}
    
    
    
    sensor_test_acc_data = {part: [999999999, 999999999, 999999999] for part in SensorPart}
    
    
    __first_sensor_data_inv = {part: 0 for part in SensorPart} # inv용 센서 첫 데이터 저장

    __acc_pickle_data = []
    __ori_pickle_data = []
    __q_pickle_data = []
    test_acc = []
    test_q = []
    test_r = []
    
    check = True
    
    t_pose_set_end = None

    # 싱글톤 설정
    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            cls._instance = super(DataManager, cls).__new__(cls)
        return cls._instance
    
    @property
    def sensor_data(self):
        return self.__sensor_data
    
    @property
    def first_sensor_data_inv(self):
        return self.__first_sensor_data_inv
    
    def get_first_sensor_data_inv(self, key):
        return self.__first_sensor_data_inv[key]

    @sensor_data.setter
    def sensor_data(self, data):
        self.__sensor_data[data[0]] = data[1]
        # self.__sensor_data[data[0]].append(data[1])

    @first_sensor_data_inv.setter
    def first_sensor_data_inv(self, data):
        self.__first_sensor_data_inv[data[0]] = data[1]

    def set_sensor_value(self, key, value):
        self.__sensor_data[key] = value

    # 7, 8, 11, 12, 0, 2
    def set_pickle_data(self):
        if self.check:
        
            part_sequence = [SensorPart.LEFT_LOWER_ARM, SensorPart.RIGHT_LOWER_ARM, SensorPart.LEFT_LOWER_LEG, SensorPart.RIGHT_LOWER_LEG, SensorPart.HEAD, SensorPart.WAIST]
            frame_acc_sensor_data = []
            frame_ori_sensor_data = []
            frame_q_sensor_data = []
            for part in part_sequence:
                try:
                    frame_acc_sensor_data.append([self.__sensor_data[part][1].x, self.__sensor_data[part][1].y, self.__sensor_data[part][1].z])

                    frame_ori_sensor_data.append(self.__sensor_data[part][3].quaternion_to_rotation_matrix())
                    frame_q_sensor_data.append([self.__sensor_data[part][3].w, self.__sensor_data[part][3].x, self.__sensor_data[part][3].y, self.__sensor_data[part][3].z])
                except:
                    print(part)
                    print(self.__sensor_data[part][1])
                    print(self.__sensor_data[part][3])
                    return
                    
            self.__acc_pickle_data.append(frame_acc_sensor_data)
            self.__ori_pickle_data.append(frame_ori_sensor_data)
            self.__q_pickle_data.append(frame_q_sensor_data)
            self.test_acc = frame_acc_sensor_data
            self.test_q = frame_q_sensor_data
            self.test_r = torch.squeeze(torch.stack(frame_ori_sensor_data))
        # print(self.test_r)
    

    def save_pickle_file(self):
        pickle_dic = {}

        pickle_dic['imu_acc'] = np.array(self.__acc_pickle_data)
        pickle_dic['imu_ori'] = np.array(self.__ori_pickle_data)
        pickle_dic['gt'] = np.zeros(72)

        # pickle 파일로 저장
        with open('data.pkl', 'wb') as f:
            pickle.dump(pickle_dic, f)
         


    def clear(self):
        self.__sensor_data = {part: [] for part in SensorPart}
        self.__acc_pickle_data = []
        self.__ori_pickle_data = []
        self.__q_pickle_data = []
        
        
    def get_log_data(self):
        self.check = False
        time.sleep(2)
        returnData1 = self.__q_pickle_data
        returnData2 = self.__acc_pickle_data
        self.check = True
        return returnData1, returnData2
        


 
    def process_dipimu(self):
        imu_mask = [7, 8, 11, 12, 0, 2] # 사용할 센서
        test_split = ['s_07']
        accs, oris, poses, trans = [], [], [], []

        for subject_name in test_split:
            for motion_name in os.listdir(os.path.join(paths.raw_dipimu_dir, subject_name)):
                path = os.path.join(paths.raw_dipimu_dir, subject_name, motion_name)
                data = pickle.load(open(path, 'rb'), encoding='latin1')
                acc = torch.from_numpy(data['imu_acc'][:, imu_mask]).float()
                ori = torch.from_numpy(data['imu_ori'][:, imu_mask]).float() # 이건 아마 회전 행렬
                pose = torch.from_numpy(data['gt']).float()



                # fill nan with nearest neighbors
                for _ in range(4):
                    acc[1:].masked_scatter_(torch.isnan(acc[1:]), acc[:-1][torch.isnan(acc[1:])])
                    ori[1:].masked_scatter_(torch.isnan(ori[1:]), ori[:-1][torch.isnan(ori[1:])])
                    acc[:-1].masked_scatter_(torch.isnan(acc[:-1]), acc[1:][torch.isnan(acc[:-1])])
                    ori[:-1].masked_scatter_(torch.isnan(ori[:-1]), ori[1:][torch.isnan(ori[:-1])])

                acc, ori, pose = acc[6:-6], ori[6:-6], pose[6:-6]
                if torch.isnan(acc).sum() == 0 and torch.isnan(ori).sum() == 0 and torch.isnan(pose).sum() == 0:

                    if motion_name == '04.pkl':
                        # 998
                        # print(len(ori), len(ori[0]), len(ori[0][0]), len(ori[0][0][0]))
                        # num = int((len(ori)/2))
                        # t = ori[num:]
                        return acc.tolist(), ori, pose
                    

                    accs.append(acc.clone())
                    # print(acc.clone())
                    oris.append(ori.clone())
                    poses.append(pose.clone())
                    # print(len(pose))
                    # print('----')
                    trans.append(torch.zeros(pose.shape[0], 3))  # dip-imu does not contain translations
                else:
                    print('DIP-IMU: %s/%s has too much nan! Discard!' % (subject_name, motion_name))

        os.makedirs(paths.dipimu_dir, exist_ok=True)
        torch.save({'acc': accs, 'ori': oris, 'pose': poses, 'tran': trans}, os.path.join(paths.dipimu_dir, 'test.pt'))
        print('Preprocessed DIP-IMU dataset is saved at', paths.dipimu_dir)