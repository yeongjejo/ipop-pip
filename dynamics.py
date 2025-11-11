import torch
import numpy as np
import pybullet as p

import articulate as art
from articulate.utils.bullet import *
from articulate.utils.rbdl import *
from data_manager import DataManager
from utils import *
from qpsolvers import solve_qp
from config import paths
import pandas as pd

import math


class PhysicsOptimizer:
    test_contact_joints = ['LHIP', 'RHIP', 'SPINE1', 'LKNEE', 'RKNEE', 'SPINE2',
                           'SPINE3', 'LSHOULDER', 'RSHOULDER', 'HEAD',
                           'LELBOW', 'RELBOW', 'LHAND', 'RHAND', 'LFOOT', 'RFOOT'
                           ]  # 'LANKLE', 'RANKLE', 'NECK', 'LWRIST', 'RWRIST', 'LCLAVICLE', 'RCLAVICLE'

    def __init__(self, debug=True):
        mu = 0.6
        supp_poly_size = 0.2
        self.tetee = []
        self.debug = debug
        self.model = RBDLModel(paths.physics_model_file, update_kinematics_by_hand=True)
        self.zero_model = RBDLModel(paths.physics_model_file, update_kinematics_by_hand=True)
        self.params = read_debug_param_values_from_json(paths.physics_parameter_file)
        self.friction_constraint_matrix = np.array([[np.sqrt(2), -mu, 0],
                                                    [-np.sqrt(2), -mu, 0],
                                                    [0, -mu, np.sqrt(2)],
                                                    [0, -mu, -np.sqrt(2)]])
        self.support_polygon = np.array([[-supp_poly_size / 2,  0,  -supp_poly_size / 2],
                                         [ supp_poly_size / 2,  0,  -supp_poly_size / 2],
                                         [-supp_poly_size / 2,  0,   supp_poly_size / 2],
                                         [ supp_poly_size / 2,  0,   supp_poly_size / 2]])

        if debug:
            p.connect(p.GUI)
            p.configureDebugVisualizer(flag=p.COV_ENABLE_Y_AXIS_UP, enable=1)
            self.id_robot = p.loadURDF(paths.physics_model_file, [0, 0, 0], useFixedBase=False, flags=p.URDF_MERGE_FIXED_LINKS)
            change_color(self.id_robot, [198 / 255, 238 / 255, 0, 1.0])
            p.loadURDF(paths.plane_file, [0, -0.881, 0.0], [-0.7071068, 0, 0, 0.7071068])
            load_debug_params_into_bullet_from_json(paths.physics_parameter_file)

        # states
        self.last_x = []
        self.q = None
        self.qdot = np.zeros(self.model.qdot_size)
        self.reset_states()

        # ▶ CSV 파일 초기화 (헤더 작성)
        self.csv_path = 'realtime_vectors.csv'
        self.columns = ['X', 'Y', 'Z']
        self.df = pd.DataFrame(columns=self.columns)
        self.df.to_csv(self.csv_path, index=False)

        self.test1 = [0.0, 0.0, 0.0]

        self.p_frame_list = [0.0, 0.0, 0.0]

    # ▶ 실시간 데이터 저장 함수
    def save_vector_to_csv(self, vector, file_path):
        df = pd.DataFrame([vector], columns=['X', 'Y', 'Z'])
        df.to_csv(file_path, mode='a', index=False, header=False)


    def reset_states(self):
        self.last_x = []
        self.q = None
        self.qdot = np.zeros(self.model.qdot_size)

    def map_value_to_0_1(self, value):
        """
        Maps a value in the range [-1.0, -0.8] to the range [1, 0].
        - A value near -1.0 will return a result close to 1.
        - A value near -0.8 will return a result close to 0.
        - Values outside the range are clamped to 1 or 0.
        """
        # Clamp out-of-range values
        if value <= -0.87:
            return 0.99
        elif value >= -0.8:
            return 0.01

        # Linear interpolation: result = (value - min) / (max - min)
        # Inverse direction because -1.0 maps to 1 and -0.8 maps to 0
        return max((-0.8 - value) / (-0.8 + 0.87), 0.01)  # or (value + 1.0) / 0.2

    def rotation_matrix_to_euler(self, R):
        """
        회전행렬 R(3x3) → 오일러각 (roll, pitch, yaw)
        ZYX 순서 (yaw → pitch → roll) 기준
        """
        assert R.shape == (3, 3)

        # pitch 계산 (asin 때문에 범위 체크 필요)
        pitch = -np.arcsin(R[2, 0])

        if abs(R[2, 0]) < 0.999999:  # 특이점 회피
            roll = np.arctan2(R[2, 1], R[2, 2])
            yaw = np.arctan2(R[1, 0], R[0, 0])
        else:
            # Gimball lock
            roll = np.arctan2(-R[1, 2], R[1, 1])
            yaw = 0.0

        return roll, pitch, yaw


    def optimize_frame(self, pose, jvel, contact, acc, check_rbdl, return_grf=False):
        # print(pose)
        q_ref = smpl_to_rbdl(pose, torch.zeros(3))[0]
        # print(q_ref)
        # print('+'*30)

        v_ref = jvel.numpy()
        c_ref = contact.sigmoid().numpy()

        a_ref = acc.numpy()
        q = self.q
        qdot = self.qdot



        if q is None:
            self.q = q_ref
            if return_grf:
                return pose, torch.zeros(3), [], None, 3
            else:
                return pose, torch.zeros(3)

            

        # determine the contact joints and points

        # print("Q : ", len(self.q))
        # formatted = [f"{x:.3f}" for x in self.q]
        # print("Q : ", formatted )
        self.model.update_kinematics(q, qdot, np.zeros(self.model.qdot_size))
        self.zero_model.update_kinematics(q_ref, qdot, np.zeros(self.model.qdot_size))

        # for join in ['ROOT', 'LFOOT', 'RFOOT']:
        #     joint_id = vars(Body)[join]
        #     pos = self.zero_model.calc_body_position(q, joint_id)
        #     print(joint_id, " : ", pos[0] * 12.0, ", ", pos[1] * 12.0 + 11.0, ", ", pos[2] * 12.0)
        # print('-'*30)

        Js = [np.empty((0, self.model.qdot_size))]
        collision_points, collision_joints = [], []
        for joint_name in self.test_contact_joints:
            joint_id = vars(Body)[joint_name]
            pos = self.model.calc_body_position(q, joint_id)
            # if joint_id == Body.LFOOT and c_ref[0] > 0.5 and pos[1] <= self.params['floor_y'] + 0.03 or \
            #    joint_id == Body.RFOOT and c_ref[1] > 0.5 and pos[1] <= self.params['floor_y'] + 0.03 or \
            if joint_id == Body.LFOOT and c_ref[0] > 0.5 and pos[1] <= self.params['floor_y'] + 0.03 or \
               joint_id == Body.RFOOT and c_ref[1] > 0.5 and pos[1] <= self.params['floor_y'] + 0.03 or \
               pos[1] <= self.params['floor_y']:
                collision_joints.append(joint_name)
                for ps in self.support_polygon + pos:
                    collision_points.append(ps)
                    pb = self.model.calc_base_to_body_coordinates(q, joint_id, ps)
                    Js.append(self.model.calc_point_Jacobian(q, joint_id, pb))
        Js = np.vstack(Js)
        nc = len(collision_points)

        # minimize   ||A1 * qddot - b1||^2     for A1, b1 in zip(As1, bs1)
        #            + ||A2 * lambda - b2||^2  for A2, b2 in zip(As2, bs2)
        #            + ||A3 * tau - b3||^2     for A3, b3 in zip(As3, bs3)
        # s.t.       G1 * qddot <= h1          for G1, h1 in zip(Gs1, hs1)
        #            G2 * lambda <= h2         for G2, h2 in zip(Gs2, hs2)
        #            G3 * tau <= h3            for G3, h3 in zip(Gs3, hs3)
        #            A_ * x = b_
        As1, bs1, As2, bs2, As3, bs3 = [np.zeros((0, self.model.qdot_size))], [np.empty(0)], [np.empty((0, nc * 3))], \
                                       [np.empty(0)], [np.zeros((0, self.model.qdot_size))], [np.empty(0)]
        Gs1, hs1, Gs2, hs2, Gs3, hs3 = [np.zeros((0, self.model.qdot_size))], [np.empty(0)], [np.empty((0, nc * 3))], \
                                       [np.empty(0)], [np.zeros((0, self.model.qdot_size))], [np.empty(0)]
        A_, b_ = None, None

        # joint angle PD controller
        if True:
            A = np.hstack((np.zeros((self.model.qdot_size - 3, 3)), np.eye((self.model.qdot_size - 3))))
            b = self.params['kp_angular'] * art.math.angle_difference(q_ref[3:], q[3:]) - self.params['kd_angular'] * qdot[3:]
            As1.append(A)  # 72 * 75
            bs1.append(b)  # 72


        # joint position PD controller (using joint velocity to determine target joint position)
        if True:
            for joint_name, v in zip(['ROOT', 'LHIP', 'RHIP', 'SPINE1', 'LKNEE', 'RKNEE', 'SPINE2', 'LANKLE', 'RANKLE',
                                      'SPINE3', 'LFOOT', 'RFOOT', 'NECK', 'LCLAVICLE', 'RCLAVICLE', 'HEAD', 'LSHOULDER',
                                      'RSHOULDER', 'LELBOW', 'RELBOW', 'LWRIST', 'RWRIST'], v_ref[:22]):
                joint_id = vars(Body)[joint_name]
                if joint_id == Body.LFOOT or joint_id == Body.RFOOT: continue
                cur_vel = self.model.calc_point_velocity(q, qdot, joint_id)
                a_des = self.params['kp_linear'] * v * self.params['delta_t'] - self.params['kd_linear'] * cur_vel
                A = self.model.calc_point_Jacobian(q, joint_id)
                b = -self.model.calc_point_acceleration(q, qdot, np.zeros(75), joint_id) + a_des
                As1.append(A * self.params['coeff_jvel'])
                bs1.append(b * self.params['coeff_jvel'])

        # Signorini’s conditions of lambda
        if True:
            if nc != 0:
                A = [np.eye(3) * max(cp[1] - self.params['floor_y'], 0.005) for cp in collision_points]
                A = art.math.block_diagonal_matrix_np(A)
                As2.append(A * self.params['coeff_lambda'])
                bs2.append(np.zeros(nc * 3))

        # tau size
        if True:
            As3.append(art.math.block_diagonal_matrix_np([
                np.eye(6) * self.params['coeff_virtual'],
                np.eye(self.model.qdot_size - 6) * self.params['coeff_tau']
            ]))
            bs3.append(np.zeros(self.model.qdot_size))

        # contacting body joint velocity
        if True:
            for joint_name in self.test_contact_joints[:-2]:
                joint_id = vars(Body)[joint_name]
                pos = self.model.calc_body_position(q, joint_id)
                if pos[1] <= self.params['floor_y']:
                    J = self.model.calc_point_Jacobian(q, joint_id)
                    v = self.model.calc_point_velocity(q, qdot, joint_id)
                    Gs1.append(-self.params['delta_t'] * J)
                    hs1.append(v - [-1e-1, 0, -1e-1])
                    Gs1.append(self.params['delta_t'] * J)
                    hs1.append(-v + [1e-1, 1e2, 1e-1])

        # contacting foot velocity
        contact_check = 0
        if True:
            for joint_name, stable in zip(['LFOOT', 'RFOOT'], c_ref):
            # for joint_name, stable in zip(['LANKLE', 'RANKLE'], c_ref):
                joint_id = vars(Body)[joint_name]
                pos = self.model.calc_body_position(q, joint_id)
                J = self.model.calc_point_Jacobian(q, joint_id)
                # print(J.shape)
                v = self.model.calc_point_velocity(q, qdot, joint_id)
                # print(v)

                if check_rbdl:
                    # 1) 발끝 속도
                    v_th = 0.5
                    v = self.zero_model.calc_point_velocity(q_ref, qdot, joint_id)
                    p_v = np.exp(-(np.linalg.norm(v)**2) / (2*v_th**2))

                    # 2) 발 위치
                    h_th = 0.3
                    #py = self.zero_model.calc_body_position(q_ref, joint_id).tolist()[1]
                    py = self.model.calc_body_position(q, joint_id).tolist()[1]
                    #if py < 0.0:
                    #   py = 0.0
                    p_h = np.exp(-(py**2) / (2*h_th**2))

                    # 3) 각도
                    # roll, pitch, yaw = self.rotation_matrix_to_euler(self.zero_model.calc_body_orientation(q_ref, joint_id)) #롤 피치 요 순서대로 리턴되는지 확인해야됨!(피치 롤이 바뀐거같아)
                    roll, pitch, yaw = art.math.rotation_matrix_to_euler_angle_np(self.zero_model.calc_body_orientation(q_ref, joint_id)).tolist()[0]
                    # print('roll', roll)
                    # theta = np.sqrt(roll**2**2)
                    theta = np.sqrt(pitch**2 + yaw**2)
                    # print(theta)
                    theta_th = 0.17
                    p_f = np.exp(-(theta**2) / (2*theta_th**2))

                    # 4) 연속 시간 충족
                    #p_frame = p_v * p_h * p_f
                    p_frame = p_v * p_h
                    self.p_frame_list.append(p_frame)
                    self.p_frame_list.pop(0)
                    p_t = sum(self.p_frame_list) / 2.0
                    #p_t = sum(self.p_frame_list) / 3.0

                    # 5) 발끝 접선 속도
                    sigma_tan = 0.08
                    vL = self.zero_model.calc_point_velocity(q_ref, qdot, joint_id)
                    n = vL / np.linalg.norm(vL)
                    v_tan = vL - n * np.dot(v, n)
                    p_sl = np.exp(-(np.linalg.norm(v_tan)**2) / (2*sigma_tan**2))

                    # 6) 자코비안
                    J_check = self.zero_model.calc_point_Jacobian(q_ref, joint_id, self.zero_model.calc_body_position(q_ref, joint_id))
                    j_root = J_check[:, :6]
                    j_q = J_check[:, 6:]

                    b_check = -j_q @ qdot[6:]

                    v_root, residuals, rank, s = np.linalg.lstsq(j_root, b_check, rcond=None)
                    p_res = None
                    if residuals.size > 0:
                        p_res = np.sqrt(residuals[0])
                    else:
                        r = j_root @ v_root - b_check
                        p_res = np.linalg.norm(r)

                    # 최종 결합
                    w_v = 1.0
                    w_h = 1.0
                    w_theta = 1.0
                    w_t = 1.0
                    w_res = 1.0
                    w_sl = 1.0
                    # 1차
                    #new_stable = (w_v*p_v + w_h*p_h + w_theta*p_f + w_t*p_t) / (w_v + w_h + w_theta + w_t)
                    new_stable = (w_v * p_v + w_h * p_h + w_t * p_t) / (w_v + w_h + w_t)
                    # # 2차

                    # print('p_v : ', p_v)
                    # print('p_h : ', p_h)
                    # print('p_f : ', p_f)
                    # print('p_t : ', p_t)
                    # print('p_res : ', p_res)
                    # print('p_sl : ', p_sl)
                    # new_stable = (w_v*p_v + w_h*p_h + w_theta*p_f + w_t*p_t + w_res*p_res + w_sl*p_sl) / (w_v + w_h + w_theta + w_t + w_res + w_sl)

                    # 지면 접촉 확률 확인용 록,
                    #print(joint_name + "ipop 계산 : ", new_stable)
                    if joint_name == 'LFOOT':
                        #print(p_h, stable, sep=',')
                        pass
                    if joint_name == 'RFOOT':
                        #print(p_h, stable, sep=',')
                        pass
                        #print(joint_name + "PIP 계산 : ", stable)
                    #print(joint_name + "Velocity 계산 : ", p_v)
                    #print(joint_name + "PIP 계산 : ", stable)
    
                    # 0.85 이상이면 땅에 고정 이하이면 값이 작을수록 많이 이동 할수 있음
                    th = -np.log(min(new_stable, 0.84999) / 0.85)
                    th_y = (self.params['floor_y'] - pos[1]) / self.params['delta_t']
                    Gs1.append(-self.params['delta_t'] * J)
                    hs1.append(v - [-th, th_y, -th])
                    Gs1.append(self.params['delta_t'] * J)
                    hs1.append(-v + [th, max(th, th_y) + 1e-6, th])

                    # IPOP 지면 접촉 확률 알고리즘
                    if joint_name == 'LFOOT' and new_stable > 0.45:
                        contact_check = 1
                    elif joint_name == 'RFOOT' and contact_check == 0 and new_stable > 0.45:
                        contact_check = 2
                    elif joint_name == 'RFOOT' and contact_check == 1 and new_stable > 0.45:
                        contact_check = 3



                else:
                    contact_th = 0.75
                    # contact_th = 0.9
                    if joint_name == 'LFOOT' and stable > contact_th:
                        contact_check = 1
                    elif joint_name == 'RFOOT' and contact_check == 0 and stable > contact_th:
                        contact_check = 2
                    elif joint_name == 'RFOOT' and contact_check == 1 and stable > contact_th:
                        contact_check = 3

                    # th = -np.log(min(stable, 0.84999) / 0.85)
                    th = -np.log(min(stable, 0.84999) / 0.85)
                    th_y = (self.params['floor_y'] - pos[1]) / self.params['delta_t']
                    Gs1.append(-self.params['delta_t'] * J)
                    hs1.append(v - [-th, th_y, -th])
                    Gs1.append(self.params['delta_t'] * J)
                    hs1.append(-v + [th, max(th, th_y) + 1e-6, th])


        # GRF friction cone constraint
        if True:
            if nc > 0:
                Gs2.append(art.math.block_diagonal_matrix_np([self.friction_constraint_matrix] * nc))
                hs2.append(np.zeros(nc * 4))

        # equation of motion (equality constraint)
        if True:
            M = self.model.calc_M(q)
            h = self.model.calc_h(q, qdot)
            A_ = np.hstack((-M, Js.T, np.eye(self.model.qdot_size)))
            b_ = h


        As1, bs1, As2, bs2, As3, bs3 = np.vstack(As1), np.concatenate(bs1), np.vstack(As2), np.concatenate(bs2), np.vstack(As3), np.concatenate(bs3)
        Gs1, hs1, Gs2, hs2, Gs3, hs3 = np.vstack(Gs1), np.concatenate(hs1), np.vstack(Gs2), np.concatenate(hs2), np.vstack(Gs3), np.concatenate(hs3)
        G_ = art.math.block_diagonal_matrix_np([Gs1, Gs2, Gs3])
        h_ = np.concatenate((hs1, hs2, hs3))
        P_ = art.math.block_diagonal_matrix_np([np.dot(As1.T, As1), np.dot(As2.T, As2), np.dot(As3.T, As3)])
        q_ = np.concatenate((-np.dot(As1.T, bs1), -np.dot(As2.T, bs2), -np.dot(As3.T, bs3)))



        # fast solvers are less accurate/robust, and may fail
        init = self.last_x if len(self.last_x) == len(q_) else None
        x = solve_qp(P_, q_, G_, h_, A_, b_, solver='quadprog', initvals=init)

        if x is None or np.linalg.norm(x) > 10000:
            x = solve_qp(P_, q_, G_, h_, A_, b_, solver='cvxopt', initvals=init)

        qddot = x[:self.model.qdot_size]
        GRF = x[self.model.qdot_size:-self.model.qdot_size]
        tau = x[-self.model.qdot_size:]

        qdot = qdot + qddot * self.params['delta_t']
        q = q + qdot * self.params['delta_t']



        self.q = q
        self.qdot = qdot
        self.last_x = x

        if self.debug:
            set_pose(self.id_robot, q)
            self.params = read_debug_param_values_from_bullet()

        pose_opt, tran_opt = rbdl_to_smpl(q)

        pose_opt = torch.from_numpy(pose_opt).float()[0]
        tran_opt = torch.from_numpy(tran_opt).float()[0]

        self.test1 += v_ref[0] * self.params['delta_t']


        if not return_grf:
            return pose_opt, tran_opt
        else:
            cj = [vars(art.SMPLJoint)[_].value for _ in collision_joints]
            grf = torch.from_numpy(GRF).float().view(-1, 4, 3).sum(dim=1) if len(cj) > 0 else None

            DataManager().pre_position = [tran_opt.tolist()[0], tran_opt.tolist()[1], tran_opt.tolist()[2]]

            return pose_opt, tran_opt, cj, grf, contact_check
        return pose_opt, tran_opt
