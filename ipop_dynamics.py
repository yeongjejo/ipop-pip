import torch
import numpy as np
import pybullet as p
from winxpgui import FlashWindowEx

import articulate as art
from articulate.utils.bullet import *
from articulate.utils.rbdl import *
from data_manager import DataManager
from utils import *
from qpsolvers import solve_qp
from config import paths
import pandas as pd


class PhysicsOptimizer:
    test_contact_joints = ['LHIP', 'RHIP', 'SPINE1', 'LKNEE', 'RKNEE', 'SPINE2',
                           'SPINE3', 'LSHOULDER', 'RSHOULDER', 'HEAD',
                           'LELBOW', 'RELBOW', 'LHAND', 'RHAND', 'LFOOT', 'RFOOT'
                           ]  # 'LANKLE', 'RANKLE', 'NECK', 'LWRIST', 'RWRIST', 'LCLAVICLE', 'RCLAVICLE'

    all_joint = ['ROOT', 'LHIP', 'RHIP', 'SPINE1', 'LKNEE', 'RKNEE', 'SPINE2', 'LANKLE', 'RANKLE',
                     'SPINE3', 'LFOOT', 'RFOOT', 'NECK', 'LCLAVICLE', 'RCLAVICLE', 'HEAD', 'LSHOULDER',
                     'RSHOULDER', 'LELBOW', 'RELBOW', 'LWRIST', 'RWRIST', 'LHAND', 'RHAND']

    def __init__(self, debug=True):
        self.root_zero_model = RBDLModel(paths.physics_model_file, update_kinematics_by_hand=True) # 허리 고정 모델
        self.visual_model = RBDLModel(paths.physics_model_file, update_kinematics_by_hand=True) # 시각화 모델
        self.params = read_debug_param_values_from_json(paths.physics_parameter_file)


        # states
        self.last_x = []
        self.q = None
        self.qdot = np.zeros(self.root_zero_model.qdot_size)
        self.reset_states()

        self.stance = []
        self.stance_cnt_list = [0, 0]
        self.pre_root_position = np.array([0.0, 0.0, 0.0])
        self.l_stance_cnt = 0
        self.r_stance_cnt = 0

        # 옵저버 알고리즘 용도
        self.pre_stance = [] # 이전에 접촉 관절 저장
        self.old_stance_position = None

        #const
        self.v_lo = 0.4
        self.v_hi = 0.5
        # self.v_lo = 10.0
        # self.v_hi = 18.0
        # self.v_lo = 0.5
        # self.v_hi = 0.8

        self.root_position = 0.0
        self.vfoot_th_low = 0.20
        self.vfoot_th_high = 0.40
        self.vy_up_th = 0.65
        self.vy_ay_up_th = 6.0
        self.dz_th = -0.7
        self.hys_time = 0.8
        self.hys_short_time = 0.48

        self.init = 0


        self.vy_prev = 0

        self.state_l = 'STANCE'
        self.state_r = 'STANCE'

        #앵커 위치
        self.anchor_L = np.array([0.0, 0.0, 0.0])
        self.anchor_R = np.array([0.0, 0.0, 0.0])


        self.timer_L = 0
        self.timer_R = 0

        self.hys_short_l = 0
        self.hys_short_r = 0

        self.g_eff = 7.0
        self.b_y = 1.0
        self.lam = 0.2


    def reset_states(self):
        self.last_x = []
        self.q = None
        self.qdot = np.zeros(self.root_zero_model.qdot_size)

    def get_joint_position(self, q, joint_name):
        joint_id = vars(Body)[joint_name]
        pos = self.visual_model.calc_body_position(q, joint_id)

        return pos

    # 지면 접촉 관절(양발) 판별
    def foot_stance_check(self, q_ref, qdot, joint_name, stance_cnt_index):
        v = self.root_zero_model.calc_point_velocity(q_ref, qdot, vars(Body)[joint_name])
        # print(stance_cnt_index, "v : ", v)
        norm_v = np.linalg.norm(v)
        # print(stance_cnt_index, " : ", norm_v)

        # 테스트
        # joint_id = vars(Body)[joint_name]
        # pos = self.root_zero_model.calc_body_position(q_ref, joint_id)
        # norm_v = pos[1] * 12.0 + 11

        # print(stance_cnt_index, " : ", norm_v)
        if joint_name in self.stance: # 기존에 접촉중일 경우
            if norm_v > self.v_hi: # 접촉 해제 조건
                self.stance_cnt_list[stance_cnt_index] += 1
            else:
                self.stance_cnt_list[stance_cnt_index] = 0
            if joint_name == 'RFOOT':
                self.r_stance_cnt = self.stance_cnt_list[stance_cnt_index]
            elif joint_name == 'LFOOT':
                self.l_stance_cnt = self.stance_cnt_list[stance_cnt_index]
            # if self.stance_cnt_list[stance_cnt_index] >= 3: # 접촉 해제
            #     # print(self.stance)
            #     # print(joint_name)
            #     # print(stance_cnt_index, "번 접촉 해제~~~~~~~~~~~~~")
            #     self.stance_cnt_list[stance_cnt_index] = 0
            #     self.stance.remove(joint_name)

        else:  # 기존에 접촉중이 아닐경우
            if norm_v < self.v_lo: # 접촉 진입 조건
                self.stance_cnt_list[stance_cnt_index] += 1
            else:
                self.stance_cnt_list[stance_cnt_index] = 0

            if self.stance_cnt_list[stance_cnt_index] >= 1: # 접촉 판단
                # print(self.stance)
                # print(joint_name)
                # print(stance_cnt_index, "번 접촉 판단!!!!!!!!!!!!!!!!!")
                self.stance_cnt_list[stance_cnt_index] = 0
                self.stance.append(joint_name)

    def omega_body_from_R(self, cur_root):
        r_delta = self.pre_root_position.T @ cur_root
        if r_delta == 0.0:
            return np.zeros(3)

        # np.trace(r_delta)
        trace = np.clip((r_delta - 1.0) * 0.5, -1.0, 1.0)
        theta = np.arccos(trace)
        if theta < 1e-12:
            return np.zeros(3)

        skew = (r_delta - r_delta.T) * 0.5
        axis = np.array([skew[2, 1], skew[0, 2], skew[1, 0]]) / np.sin(theta)

        return axis * (theta / self.params['delta_t'])


    def optimize_frame(self, pose, jvel, contact, acc, check_rbdl, return_grf=False):
        zero_q = smpl_to_rbdl(pose, torch.zeros(3))[0] # 허리 위치 0으로 고정된 q

        q = self.q # 허리 위치가 변하는 q
        qdot = self.qdot

        # 첫데이터 넘김
        if q is None:
            self.q = zero_q
            return pose, torch.zeros(3), [], [], 3

        root_x = DataManager().totalcapture_gt[0][0]['position'][0]

        self.root_zero_model.update_kinematics(zero_q, qdot, np.zeros(self.root_zero_model.qdot_size))
        self.visual_model.update_kinematics(q, qdot, np.zeros(self.visual_model.qdot_size))


        # self.root_zero_model.update_kinematics(zero_q, np.zeros(self.root_zero_model.qdot_size), np.zeros(self.root_zero_model.qdot_size))
        # self.visual_model.update_kinematics(q, np.zeros(self.root_zero_model.qdot_size), np.zeros(self.visual_model.qdot_size))

        # for join in ['ROOT', 'LHIP', 'RHIP', 'SPINE1', 'LKNEE', 'RKNEE', 'SPINE2', 'LANKLE', 'RANKLE',
        #              'SPINE3', 'LFOOT', 'RFOOT', 'NECK', 'LCLAVICLE', 'RCLAVICLE', 'HEAD', 'LSHOULDER',
        #              'RSHOULDER', 'LELBOW', 'RELBOW', 'LWRIST', 'RWRIST', 'LHAND', 'RHAND']:
        #     joint_id = vars(Body)[join]
        #     pos = self.root_zero_model.calc_body_position(q, joint_id)
        #     vel = self.root_zero_model.calc_point_velocity(zero_q, np.zeros(self.root_zero_model.qdot_size), joint_id)
        #     # print(joint_id, " : ", pos[0] * 12.0, ", ", pos[1] * 12.0 + 11.0, ", ", pos[2] * 12.0)
        #     print(joint_id, " : ", vel[0], ", ", vel[1], ", ", vel[2])
        # print('-'*30)

        # 지면 접촉 판별
        self.foot_stance_check(zero_q, qdot, 'LFOOT', 0) # 왼발 지면 접촉 판별
        self.foot_stance_check(zero_q, qdot, 'RFOOT', 1)  # 오른발 지면 접촉 판별

        # 루트(허리) 이동
        contact_check = 3
        root_position = self.get_joint_position(q, 'ROOT')
        if len(self.stance) == 1: # 싱글 스텐스 (옵저버 알고리즘)
            joint_name = self.stance[0]
            if joint_name == 'LFOOT':
                contact_check = 1
            else:
                contact_check = 2
            if len(self.pre_stance) == 0 or joint_name not in self.pre_stance:
                self.pre_stance = []
                self.pre_stance.append(joint_name) # 지면 접촉 관절 저장
                self.old_stance_position = self.get_joint_position(q, joint_name)

            now_stance_position = self.get_joint_position(q, joint_name)
            root_position = np.add(self.old_stance_position, np.subtract(root_position, now_stance_position))



        elif len(self.stance) == 2:  # 더블 스텐스 조건
            contact_check = 3
            name = 'Double'

            #접촉 시간이 더 긴걸 우선으로
            r_cnt = self.r_stance_cnt
            l_cnt = self.l_stance_cnt
            if l_cnt >= r_cnt :
                double_stance_position = self.get_joint_position(q, self.stance[1])
                name = 'Double2'
            else:
                double_stance_position = self.get_joint_position(q, self.stance[0])
                name = 'Double1'


            double_stance_position = np.divide(np.add(self.get_joint_position(q, self.stance[0]), self.get_joint_position(q, self.stance[1])), np.array([2]))
            # 아래 if else 문은 양다리중 낮은 높이에 있는 걸 루트포지션으로 주석하면 양다리 평균을 루트 포지션으로
            if (self.get_joint_position(q, self.stance[0])[1] <=   self.get_joint_position(q, self.stance[1])[1]):
                double_stance_position = self.get_joint_position(q, self.stance[0])
                name = 'Double1'
            else:
                double_stance_position = self.get_joint_position(q, self.stance[1])
                name = 'Double2'
            if len(self.pre_stance) == 0 or name not in self.pre_stance:
                self.pre_stance = []
                self.pre_stance.append(name) # 지면 접촉 관절 저장
                self.old_stance_position = double_stance_position



            root_position = np.add(self.old_stance_position, np.subtract(root_position, double_stance_position))
        else: #점프조건
            contact_check = 0
            self.pre_stance = []
            # # # 발 속도 기준으로 허리포지션 이동
            # l_foot_v = self.root_zero_model.calc_point_velocity(zero_q, qdot, vars(Body)['LFOOT'])
            # r_foot_v = self.root_zero_model.calc_point_velocity(zero_q, qdot, vars(Body)['RFOOT'])
            # foot_v_avg = np.add(np.divide(np.add(l_foot_v, r_foot_v), np.array([2])), np.array([0.0, -9.81, 0.0]))
            # v_root = qdot[3:6] + foot_v_avg
            # r_root = v_root * self.params['delta_t']
            # #
            # print(r_root)
            # # print( np.add(self.pre_root_position, foot_v_avg))
            # # print('-'*30)
            # root_position = np.add(self.pre_root_position, r_root)
            #
            #
            # if root_position[1]< 0.0:
            #     root_position[1] = 0.0
            #     print(11111111)
            #
            # # '내일 와서 할거 / vs코드 에러 나는거 확인 / 검은색 모델 삭제 (상대쿼터니언 직접 구현)'


        # root_qdot = self.omega_body_from_R(root_position)
        self.pre_root_position = root_position

        l_foot_p = self.root_zero_model.calc_body_position(q, vars(Body)['LFOOT'])
        r_foot_p = self.root_zero_model.calc_body_position(q, vars(Body)['RFOOT'])
        root_position = torch.tensor(root_position)

        print(root_position)
        q = smpl_to_rbdl(pose, root_position)[0]
        qdot = (q - self.q) / self.params['delta_t']

        self.q = q
        self.qdot = qdot
        # print(qdot)

        pose_opt, tran_opt = rbdl_to_smpl(q)

        pose_opt = torch.from_numpy(pose_opt).float()[0]
        tran_opt = torch.from_numpy(tran_opt).float()[0]


        DataManager().pre_position = [tran_opt.tolist()[0], tran_opt.tolist()[1], tran_opt.tolist()[2]]

        #
        # self.visual_model.update_kinematics(q, qdot, np.zeros(self.visual_model.qdot_size))
        
        #  나중에는 pose_opt, tran_opt만 있으면됨 (l_foot_p, r_foot_p, contact_check)는 테스트용
        return pose_opt, tran_opt, l_foot_p.tolist(), r_foot_p.tolist(), contact_check

    def optimize_frame_1(self, pose, jvel, contact, acc, check_rbdl, position, return_grf=False):
        contact_check = 0

        position = torch.tensor(position)
        position[0] = position[0] / 20 - 0.55026
        position[1] = position[1] / 20 - 0.55026
        position[2] = position[2] / 20 - 0.55026

        dt = self.params['delta_t']
        floor_height = -0.87

        # 첫데이터 넘김
        if self.q is None:
            #print(dt)
            #zero_q = smpl_to_rbdl(pose, torch.zeros(3))[0]  # 허리 위치 0으로 고정된 q
            zero_q = smpl_to_rbdl(pose, torch.zeros(3))[0]
            self.visual_model.update_kinematics(zero_q, np.zeros(self.root_zero_model.qdot_size), np.zeros(self.root_zero_model.qdot_size))
            l_foot_p = self.visual_model.calc_body_position(zero_q, vars(Body)['LFOOT'])
            r_foot_p = self.visual_model.calc_body_position(zero_q, vars(Body)['RFOOT'])

            diff_l_abs = abs(l_foot_p[1] - floor_height)
            diff_r_abs = abs(r_foot_p[1] - floor_height)

            floor_abs = 0
            if diff_l_abs >= diff_r_abs :
                if l_foot_p[1] <= floor_height :
                    floor_abs = diff_l_abs
                else :
                    floor_abs = -1.0 * diff_l_abs
            else :
                if r_foot_p[1] <= floor_height :
                    floor_abs = diff_r_abs
                else :
                    floor_abs = -1.0 * diff_r_abs

            #position[1] = position[1] + floor_abs
            zero_q[1] = zero_q[1] + floor_abs

            zero_q = smpl_to_rbdl(pose, zero_q[0:3])[0]

            self.visual_model.update_kinematics(zero_q, np.zeros(self.root_zero_model.qdot_size), np.zeros(self.root_zero_model.qdot_size))

            l_foot_p = self.visual_model.calc_body_position(zero_q, vars(Body)['LFOOT'])
            r_foot_p = self.visual_model.calc_body_position(zero_q, vars(Body)['RFOOT'])

            self.anchor_L = l_foot_p.copy()
            self.anchor_R = r_foot_p.copy()

            pose_opt, tran_opt = rbdl_to_smpl(zero_q)

            pose_opt = torch.from_numpy(pose_opt).float()[0]
            tran_opt = torch.from_numpy(tran_opt).float()[0]

            self.q = zero_q
            return pose_opt, tran_opt, [], [], contact_check
            #return pose, torch.zeros(3), [], [], 3

        #10 frame 넘김
        if self.init < 10:
            zero_q = smpl_to_rbdl(pose, self.q[0:3])[0]
            self.visual_model.update_kinematics(zero_q, np.zeros(self.root_zero_model.qdot_size),
                                                np.zeros(self.root_zero_model.qdot_size))
            l_foot_p = self.visual_model.calc_body_position(zero_q, vars(Body)['LFOOT'])
            r_foot_p = self.visual_model.calc_body_position(zero_q, vars(Body)['RFOOT'])

            self.anchor_L = l_foot_p.copy()
            self.anchor_R = r_foot_p.copy()
            pose_opt, tran_opt = rbdl_to_smpl(zero_q)
            pose_opt = torch.from_numpy(pose_opt).float()[0]
            tran_opt = torch.from_numpy(tran_opt).float()[0]
            self.init +=1
            print(self.qdot[0:3])
            return pose_opt, tran_opt, [], [], contact_check

        #1. q, qdot 입력
        pre_q = self.q
        pre_qdot = self.qdot
        self.q = smpl_to_rbdl(pose, pre_q[0:3])[0]
        self.qdot = (self.q - pre_q) / dt
        self.qdot[0:3] = pre_qdot[0:3]

        #2. root_pred 계산 - 중력적용 해야함
        root_now = pre_q[0:3]
        root_pred = root_now + pre_qdot[0:3] * dt

        #root_pred가 날라가는것 방지
        # dx = root_pred - root_now
        # n = np.linalg.norm(dx)
        # if n > 0.10:  #임계값 조절해야함
        #     root_pred = root_now + dx * (0.10/n)

        self.q[0:3] = root_pred
        self.qdot[0:3] = (self.q[0:3] - pre_q[0:3]) / dt

        #3. 양발 stance 결정하기
        self.visual_model.update_kinematics(self.q, self.qdot,
                                                np.zeros(self.root_zero_model.qdot_size))
        l_foot_p = self.visual_model.calc_body_position(self.q, vars(Body)['LFOOT'])
        r_foot_p = self.visual_model.calc_body_position(self.q, vars(Body)['RFOOT'])

        joint_id = vars(Body)['LFOOT']
        vl_foot = self.visual_model.calc_point_velocity(self.q, self.qdot, joint_id)
        joint_id = vars(Body)['RFOOT']
        vr_foot = self.visual_model.calc_point_velocity(self.q, self.qdot, joint_id)

        stance_cand_l = False
        stance_cand_r = False
        swing_cand_l = False
        swing_cand_r = False

        #허리속도 ,  허리속도 y축, 허리가속도 y축
        v_root = self.qdot[0:3]
        vy = float(v_root[1])
        ay = (vy - pre_qdot[1])/dt

        if vy <= 0.0 :
            if l_foot_p[1] < self.dz_th :
                stance_cand_l = True
            if r_foot_p[1] < self.dz_th :
                stance_cand_r = True

        if vy > self.vy_up_th and ay > self.vy_ay_up_th :   #and or 선택
            swing_cand_r = True
            swing_cand_l = True

        #발끝속도
        sL = np.linalg.norm(vl_foot)
        sR = np.linalg.norm(vr_foot)
        if sL > self.vfoot_th_high :
                swing_cand_l = True
        elif sL < self.vfoot_th_low and l_foot_p[1] < self.dz_th :
            stance_cand_l = True

        if sR > self.vfoot_th_high :
            swing_cand_r = True
        elif sR < self.vfoot_th_low and r_foot_p[1] < self.dz_th :
            stance_cand_r = True

        #stance, swing 결정
        prev_state_l = self.state_l
        prev_state_r = self.state_r
        cand_state_L = None
        cand_state_R = None

        if swing_cand_l and not stance_cand_l :
            cand_state_L = "SWING"
        elif stance_cand_l and not swing_cand_l :
            cand_state_L = "STANCE"
        else :
            cand_state_L = prev_state_l

        if swing_cand_r and not stance_cand_r :
            cand_state_R = "SWING"
        elif stance_cand_r and not swing_cand_r :
            cand_state_R = "STANCE"
        else :
            cand_state_R = prev_state_r

        #시간 유지 3frame, state 최종 결정
        timer_l = self.hys_short_l
        timer_r = self.hys_short_r

        if cand_state_L != prev_state_l :
            timer_l += dt
            if timer_l >= self.hys_short_time :
                self.state_l = cand_state_L
                timer_l = 0
        else :
            timer_l = 0.0

        if cand_state_R != prev_state_r :
            timer_r += dt
            if timer_r >= self.hys_short_time :
                self.state_r = cand_state_R
                timer_r = 0
        else :
            timer_r = 0.0

        #유지 시간
        self.hys_short_l = timer_l
        self.hys_short_r = timer_r

        print(self.state_l,self.state_r,sep=',')

        #발 고정 위치 결정 (anchor)
        if self.state_l == 'STANCE':
            if self.anchor_L is None:
                self.anchor_L = l_foot_p.copy()
            else :
                pass
        else :
            self.anchor_L = None

        if self.state_r == 'STANCE':
            if self.anchor_R is None:
                self.anchor_R = r_foot_p.copy()
            else :
                pass
        else :
            self.anchor_R = None

        #root 위치 계산
        stance_feet = []
        J_list,err_list = [],[]
        #STANCE 발에 대해 constraint 추가
        if self.anchor_L is not None :
            joint_id = vars(Body)["LFOOT"]
            J = self.visual_model.calc_point_Jacobian(self.q, joint_id, np.zeros(3))
            J_list.append(J[:,0:3])
            err_list.append(self.anchor_L - l_foot_p)
            stance_feet.append("L")

        if self.anchor_R is not None :
            joint_id = vars(Body)["RFOOT"]
            J = self.visual_model.calc_point_Jacobian(self.q, joint_id, np.zeros(3))
            J_list.append(J[:,0:3])
            err_list.append(self.anchor_R - r_foot_p)
            stance_feet.append("R")

        #STANCE 발이 없으면 자유 낙하 (swing) 수정해야함
        if len(J_list) == 0:
            root_refined = self.q[0:3]
            v_root_refined = self.qdot[0:3]
            a_y = -7.0
            # v_root_refined = v_root + np.array([0.0, a_y * dt, 0.0])
            # root_refined = root_pred + v_root_refined * dt
        #발끝 포인트 기준 허리 위치 계산
        else :
            Jc = np.vstack(J_list)
            err = np.concatenate(err_list)

            W = np.diag([0.5, 1.5, 0.5] * len(J_list))
            Jw = W @ Jc
            rw = W @ err

            AtA = Jw.T @ Jw + 1e-4 * np.eye(3)
            Atb = Jw.T @ rw
            d_root = np.linalg.solve(AtA, Atb)

            d_norm = np.linalg.norm(d_root)
            if d_norm > 0.01 :
                d_root *= 0.01/d_norm

            #root얻데이트
            root_refined = pre_q[0:3] + d_root
            v_root_refined = (root_refined - pre_q[0:3]) / dt

        self.q[0:3] = root_refined
        self.qdot = (self.q - pre_q)/dt

        self.visual_model.update_kinematics(self.q, self.qdot,
                                                np.zeros(self.root_zero_model.qdot_size))



        pose_opt, tran_opt = rbdl_to_smpl(self.q)

        pose_opt = torch.from_numpy(pose_opt).float()[0]
        tran_opt = torch.from_numpy(tran_opt).float()[0]

        DataManager().pre_position = [tran_opt.tolist()[0], tran_opt.tolist()[1], tran_opt.tolist()[2]]

        #
        # self.visual_model.update_kinematics(q, qdot, np.zeros(self.visual_model.qdot_size))

        #  나중에는 pose_opt, tran_opt만 있으면됨 (l_foot_p, r_foot_p, contact_check)는 테스트용
        return pose_opt, tran_opt, l_foot_p.tolist(), r_foot_p.tolist(), contact_check