import pickle
import time

import torch

from sensor.sensor_part import SensorPart
import numpy as np

import os
import pickle
from config import paths, amass_data


class DataManager():
    
    totalcapture_data_list = ['acting3', 'freestyle1', 'freestyle3', 'rom3', 'walking2']
    selected_totalcapture_data = 0 # 0~4 까지 원하는 케이스 선택

    # set_ip = "192.168.201.199"
    # set_ip = "192.168.0.15"
    # set_ip = "127.0.0.1"
    set_ip = "192.168.215.117"
    
    time_setting = 0.005 # 플레이 시간
    
    _instance = None  # 싱글톤 용도
    __check = True
    __sensor_data = {part: [] for part in SensorPart}

    totalcapture_imu_acc = []
    totalcapture_imu_r = []

    ipop_imu_acc = []
    ipop_imu_r = []
    ipop_imu_raw_r = []

    premodel_imu_r = []
    premodel_imu_acc = []
    premodel_root_p = []

    premodel_output_vel = []
    premodel_output_q = []
    premodel_cref = []

    totalcapture_vicon_ori = []
    totalcapture_vicon_pose = []
    totalcapture_vicon_local_ori = []

    totalcapture_gt= []

    pre_position = [0.0, 0.0, 0.0]

    axioStart = False
    tpose_check = False

    udp_switch = False
    udp_sending = False

    # 싱글톤 설정
    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            cls._instance = super(DataManager, cls).__new__(cls)
        return cls._instance

    @property
    def sensor_data(self):
        return self.__sensor_data


    @sensor_data.setter
    def sensor_data(self, data):
        self.__sensor_data[data[0]] = data[1]
        # self.__sensor_data[data[0]].append(data[1])

    def set_sensor_value(self, key, value):
        self.__sensor_data[key] = value



    def setTotalcaptureIMUData(self):
        part_sequence = [SensorPart.LEFT_LOWER_ARM, SensorPart.RIGHT_LOWER_ARM, SensorPart.LEFT_LOWER_LEG,
                         SensorPart.RIGHT_LOWER_LEG, SensorPart.HEAD, SensorPart.WAIST]

        premodel_part_sequence = [SensorPart.LEFT_LOWER_ARM, SensorPart.RIGHT_LOWER_ARM, SensorPart.LEFT_LOWER_LEG,
                         SensorPart.RIGHT_LOWER_LEG, SensorPart.HEAD, SensorPart.WAIST, SensorPart.LEFT_UPPER_ARM, SensorPart.RIGHT_UPPER_ARM,
                        SensorPart.LEFT_UPPER_LEG, SensorPart.RIGHT_UPPER_LEG, SensorPart.BACK]

        frame_acc_sensor_data = []
        frame_ori_sensor_data = []
        frame_premodel_sensor_data = []
        for part in premodel_part_sequence:
            try:
                frame_premodel_sensor_data.append(self.__sensor_data[part][3].quaternion_to_rotation_matrix())
                if part in part_sequence:
                    frame_acc_sensor_data.append([self.__sensor_data[part][1].x, self.__sensor_data[part][1].y, self.__sensor_data[part][1].z])
                    frame_ori_sensor_data.append(self.__sensor_data[part][3].quaternion_to_rotation_matrix())
            except:
                print(part)
                print(self.__sensor_data[part][1])
                print(self.__sensor_data[part][3])
                return


        self.totalcapture_imu_acc.append(frame_acc_sensor_data)
        self.totalcapture_imu_r.append(torch.squeeze(torch.stack(frame_ori_sensor_data)))
        self.premodel_imu_r.append(torch.squeeze(torch.stack(frame_premodel_sensor_data)))

    def setIpopIMUData(self):
        part_sequence = [SensorPart.LEFT_LOWER_ARM, SensorPart.RIGHT_LOWER_ARM, SensorPart.LEFT_LOWER_LEG,
                         SensorPart.RIGHT_LOWER_LEG, SensorPart.BACK, SensorPart.WAIST]

        # premodel_part_sequence = [SensorPart.LEFT_LOWER_ARM, SensorPart.RIGHT_LOWER_ARM, SensorPart.LEFT_LOWER_LEG,
        #                  SensorPart.RIGHT_LOWER_LEG, SensorPart.BACK, SensorPart.WAIST, SensorPart.LEFT_UPPER_ARM, SensorPart.RIGHT_UPPER_ARM,
        #                 SensorPart.LEFT_UPPER_LEG, SensorPart.RIGHT_UPPER_LEG]

        premodel_part_sequence = [SensorPart.LEFT_UPPER_ARM,SensorPart.LEFT_LOWER_ARM, SensorPart.RIGHT_UPPER_ARM, SensorPart.RIGHT_LOWER_ARM,
                                  SensorPart.LEFT_UPPER_LEG, SensorPart.LEFT_LOWER_LEG, SensorPart.RIGHT_UPPER_LEG, SensorPart.RIGHT_LOWER_LEG,
                                SensorPart.WAIST, SensorPart.BACK,]

        frame_acc_sensor_data = [[],[],[],[],[],[]]
        frame_ori_sensor_data = [[],[],[],[],[],[]]
        frame_raw_ori_sensor_data = [[],[],[],[],[],[]]
        frame_premodel_sensor_data = []
        frame_premodel_sensor_acc = []
        list_s = [0,1,2,3,5,4]
        cnt = 0
        for part in premodel_part_sequence:
            try:
                frame_premodel_sensor_data.append(self.__sensor_data[part][3].quaternion_to_rotation_matrix())
                frame_premodel_sensor_acc.append([self.__sensor_data[part][1].x, self.__sensor_data[part][1].y, self.__sensor_data[part][1].z])
                if part in part_sequence:
                    frame_acc_sensor_data[list_s[cnt]] = [self.__sensor_data[part][1].x, self.__sensor_data[part][1].y, self.__sensor_data[part][1].z]
                    frame_ori_sensor_data[list_s[cnt]] = self.__sensor_data[part][3].quaternion_to_rotation_matrix()
                    frame_raw_ori_sensor_data[list_s[cnt]] = self.__sensor_data[part][2].quaternion_to_rotation_matrix()
                    cnt += 1
                    # frame_acc_sensor_data.append([self.__sensor_data[part][1].x, self.__sensor_data[part][1].y, self.__sensor_data[part][1].z])
                    # frame_ori_sensor_data.append(self.__sensor_data[part][3].quaternion_to_rotation_matrix())
            except:
                print(part)
                print(self.__sensor_data[part][1])
                print(self.__sensor_data[part][3])
                return


        # empty_list =  [frame_acc_sensor_data[4][0], frame_acc_sensor_data[4][1], frame_acc_sensor_data[4][2]]
        # frame_acc_sensor_data[4] = [frame_acc_sensor_data[5][0], frame_acc_sensor_data[5][1], frame_acc_sensor_data[5][2]]
        # frame_acc_sensor_data[5] = empty_list
        #
        # empty_list =  [frame_ori_sensor_data[4][0], frame_ori_sensor_data[4][1], frame_ori_sensor_data[4][2]]
        # frame_ori_sensor_data[4] = [frame_ori_sensor_data[5][0], frame_ori_sensor_data[5][1], frame_ori_sensor_data[5][2]]
        # frame_ori_sensor_data[5] = empty_list
        self.ipop_imu_acc = frame_acc_sensor_data
        self.ipop_imu_r = torch.squeeze(torch.stack(frame_ori_sensor_data))
        self.ipop_imu_raw_r = torch.squeeze(torch.stack(frame_raw_ori_sensor_data))
        self.premodel_imu_r = torch.squeeze(torch.stack(frame_premodel_sensor_data))
        self.premodel_imu_acc = frame_premodel_sensor_acc