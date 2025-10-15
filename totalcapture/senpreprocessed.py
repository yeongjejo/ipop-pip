import csv
import numpy as np
from typing import List, Dict

import pandas as pd
from data_manager import DataManager
from sensor.quaternion import Quaternion
from sensor.acc import Acc
from sensor.gyro import Gyro
from sensor.mag import Mag
from sensor.sensor_part import SensorPart


class TotalcaptureIMUData():
    def __init__(self):
        pass

    def parse_quat_accel_csv(self, file_path: str):
        """
        output_quat_accel.csv 형식 파싱
        반환: frames 리스트
          frames[i][joint] = {
              'quat': np.array([x,y,z,w]),
              'accel': np.array([ax,ay,az])
          }
        """
        frames: List[Dict[str, Dict[str, np.ndarray]]] = []

        with open(file_path, "r", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            # DictReader는 각 셀을 "헤더: 문자열"로 줌
            for row in reader:
                frame: Dict[str, Dict[str, np.ndarray]] = {}
                for col in row:
                    if row[col].strip() == "":
                        continue
                    if "_quat" in col:
                        joint = col.replace("_quat", "")
                        quat_vals = [float(x) for x in row[col].split()]
                        if len(quat_vals) == 4:
                            frame.setdefault(joint, {})["quat"] = np.array(quat_vals, dtype=float)
                    elif "_accel" in col:
                        joint = col.replace("_accel", "")
                        accel_vals = [float(x) for x in row[col].split()]
                        if len(accel_vals) == 3:
                            frame.setdefault(joint, {})["accel"] = np.array(accel_vals, dtype=float)
                frames.append(frame)

        return frames


    def setTotalcaptureIMUData(self):
        path = 'totalcapture/data/' + DataManager().totalcapture_data_list[DataManager().selected_totalcapture_data] + '/'
        file_path = path+"output_quat_accel.csv"  # 저장된 csv 경로
        frames = self.parse_quat_accel_csv(file_path)

        bone_seq = ["Hips", "Spine3", "Head", "LeftArm", "LeftForeArm", "LeftHand", "RightArm", "RightForeArm", "RightHand", "LeftUpLeg", "LeftLeg", "LeftFoot", "RightUpLeg", "RightLeg", "RightFoot"]
        sensor_part = [SensorPart.WAIST, SensorPart.BACK, SensorPart.HEAD, SensorPart.LEFT_UPPER_ARM, SensorPart.LEFT_LOWER_ARM, SensorPart.LEFT_HAND, SensorPart.RIGHT_UPPER_ARM, SensorPart.RIGHT_LOWER_ARM, SensorPart.RIGHT_HAND, SensorPart.LEFT_UPPER_LEG, SensorPart.LEFT_LOWER_LEG, SensorPart.LEFT_FOOT, SensorPart.RIGHT_UPPER_LEG, SensorPart.RIGHT_LOWER_LEG, SensorPart.RIGHT_FOOT]

        gyro = Gyro(0.0, 0.0, 0.0)
        mag = Mag(0.0, 0.0, 0.0)
        for idx, f in enumerate(frames):
            for i, bone in enumerate(bone_seq):
                quaternion = f[bone]["quat"].tolist()
                quaternion = Quaternion(quaternion[3], quaternion[0], quaternion[1], quaternion[2])
                acc = f[bone]["accel"].tolist()
                acc = Acc(acc[0], acc[1], acc[2])
                if bone in ["LeftHand", "RightHand"]:
                    quaternion = Quaternion(1.0, 0.0, 0.0, 0.0)
                    acc = Acc(0.0, 0.0, 0.0)

                # 센서 정보 저장
                DataManager().sensor_data = [sensor_part[i], [gyro, acc, mag, quaternion]]

            DataManager().setTotalcaptureIMUData()



class TotalcaptureViconData():
    def __init__(self):
        pass

    def quat_mul(self, q1, q2):
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        return np.array([
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
        ], dtype=float)

    def quat_inverse(self, q):
        w, x, y, z = q
        norm2 = np.dot(q, q)
        return np.array([w, -x, -y, -z], dtype=float) / norm2

    def load_joint_data(self, file_path, data_form):
        with open(file_path, 'r', encoding='utf-8') as f:
            lines = f.readlines()

        # 첫 줄: 관절 이름
        headers = lines[0].strip().split('\t')

        # 나머지 줄: 각 줄은 값들이 \t 구분되어 있음
        data = []
        for line in lines[1:]:
            values = line.strip().split('\t')
            # 각 관절마다 4개의 값 (x,y,z,w)
            frame = []
            for v in values:
                nums = v.split()
                frame.append([float(n) for n in nums])
            data.append(frame)

        # DataFrame으로 변환
        # 멀티컬럼 구조: (Joint, Component)
        # cols = pd.MultiIndex.from_product([headers, ['x', 'y', 'z', 'w']], names=['Joint', 'Component'])
        cols = pd.MultiIndex.from_product([headers, data_form], names=['Joint', 'Component'])
        df = pd.DataFrame([sum(frame, []) for frame in data], columns=cols)

        return df


    def setTotalcaptureViconData(self):
        path = 'totalcapture/data/' + DataManager().totalcapture_data_list[DataManager().selected_totalcapture_data] + '/'
        # 사용 예시
        pose = self.load_joint_data(path + "gt_skel_gbl_pos.txt", ['x', 'y', 'z'])
        ori = self.load_joint_data(path + "gt_skel_gbl_ori.txt", ['x', 'y', 'z', 'w'])

        first_q = []
        # print(pose.shape)
        for (idx, pose_row), (_, ori_row) in zip(pose.iterrows(), ori.iterrows()):
            # print(idx)
            axio_bone_seq = [
                [pose_row.Hips, ori_row.Hips],
                [pose_row.Spine1, ori_row.Spine1],
                [pose_row.Spine2, ori_row.Spine2],
                [pose_row.Spine3, ori_row.Spine3],
                [pose_row.Neck, ori_row.Neck],
                [pose_row.Head, ori_row.Head],
                [pose_row.LeftShoulder, ori_row.LeftShoulder],
                [pose_row.LeftArm, ori_row.LeftArm],
                [pose_row.LeftForeArm, ori_row.LeftForeArm],
                [pose_row.LeftHand, ori_row.LeftHand],
                [pose_row.RightShoulder, ori_row.RightShoulder],
                [pose_row.RightArm, ori_row.RightArm],
                [pose_row.RightForeArm, ori_row.RightForeArm],
                [pose_row.RightHand, ori_row.RightHand],
                [pose_row.LeftUpLeg, ori_row.LeftUpLeg],
                [pose_row.LeftLeg, ori_row.LeftLeg],
                [pose_row.LeftFoot, ori_row.LeftFoot],
                [pose_row.RightUpLeg, ori_row.RightUpLeg],
                [pose_row.RightLeg, ori_row.RightLeg],
                [pose_row.RightFoot, ori_row.RightFoot]
            ]

            smpl_bone_seq = [
                [pose_row.Hips, ori_row.Hips],
                [pose_row.LeftUpLeg, ori_row.LeftUpLeg],
                [pose_row.RightUpLeg, ori_row.RightUpLeg],
                [pose_row.Spine1, ori_row.Spine1],
                [pose_row.LeftLeg, ori_row.LeftLeg],
                [pose_row.RightLeg, ori_row.RightLeg],
                [pose_row.Spine2, ori_row.Spine2],
                [pose_row.LeftFoot, ori_row.LeftFoot],
                [pose_row.RightFoot, ori_row.RightFoot],
                [pose_row.Spine3, ori_row.Spine3],
                #10 왼발
                #11 오른발
                [pose_row.Neck, ori_row.Neck],
                [pose_row.LeftShoulder, ori_row.LeftShoulder],
                [pose_row.RightShoulder, ori_row.RightShoulder],
                [pose_row.Head, ori_row.Head],
                [pose_row.LeftArm, ori_row.LeftArm],
                [pose_row.RightArm, ori_row.RightArm],
                [pose_row.LeftForeArm, ori_row.LeftForeArm],
                [pose_row.RightForeArm, ori_row.RightForeArm],
                #20 왼손
                #21 오른손
                #22 왼손가락
                #23 오른손가락
            ]

            p_c_bone = {
                0 : [1, 2, 3],
                1 : [4],
                2 : [5],
                3 : [6],
                4 : [7],
                5 : [8],
                6 : [9],
                9 : [10, 11, 12],
                10 : [13],
                11 : [14],
                12 : [15],
                14 : [16],
                15 : [17]

            }

            vicon_pose = []
            vicon_ori = []
            for i, bone in enumerate(smpl_bone_seq):
                q = np.array([bone[1].w, -bone[1].x, bone[1].y, -bone[1].z])
                if idx == 0:
                    first_q.append(self.quat_inverse(q))

                vicon_pose.append([-bone[0].x, bone[0].y, -bone[0].z])
                vicon_ori.append(self.quat_mul(q, first_q[i]).tolist())

            DataManager().totalcapture_vicon_pose.append(vicon_pose)
            DataManager().totalcapture_vicon_ori.append(vicon_ori)

            local_ori = [vicon_ori[0]]
            for p, c_list in p_c_bone.items():
                p_q = np.array([vicon_ori[p][0], vicon_ori[p][1], vicon_ori[p][2], vicon_ori[p][3]])
                p_inv = self.quat_inverse(p_q)
                for c in c_list:
                    c_q = np.array([vicon_ori[c][0], vicon_ori[c][1], vicon_ori[c][2], vicon_ori[c][3]])
                    local_ori.append(self.quat_mul(p_inv, c_q).tolist())

            DataManager().totalcapture_vicon_local_ori.append(local_ori)


            sned_data = []
            for i, bone in enumerate(axio_bone_seq):
                frame_pose = []
                q = np.array([bone[1].w, -bone[1].x, bone[1].y, -bone[1].z])
                if idx == 0:
                    first_q.append(self.quat_inverse(q))
                    if i == 0:
                        f_root_postion = [-bone[0].x / 3.0, bone[0].y / 3.0, -bone[0].z / 3.0]

                frame_pose = [(-bone[0].x / 3.0) - f_root_postion[0], (bone[0].y / 3.0) - f_root_postion[1] + 11.0,
                              (-bone[0].z / 3.0) - f_root_postion[2]]
                if i == 0:
                    DataManager().premodel_root_p.append(frame_pose)

                frame_bone_data = {
                    "time": "5",
                    "name": "test",
                    # "position": [0.0, 0.0, 0.0],
                    "position": frame_pose,
                    "rotation": self.quat_mul(q, first_q[i]).tolist(),
                    # "rotation": [bone[1].w, -bone[1].x, -bone[1].z, bone[1].y],
                    "acc": [0.0, 0.0, 0.0],
                    "lp": [],
                    "rp": [],
                }
                sned_data.append(frame_bone_data)

            DataManager().totalcapture_gt.append(sned_data)

