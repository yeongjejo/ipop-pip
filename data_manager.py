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

    totalcapture_imu_acc = []
    totalcapture_imu_r = []

    premodel_imu_r = []
    premodel_root_p = []

    premodel_output_vel = []
    premodel_output_q = []

    totalcapture_vicon_ori = []
    totalcapture_vicon_pose = []
    totalcapture_vicon_local_ori = []

    totalcapture_gt= []

    pre_position = [0.0, 0.0, 0.0]

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