import smplx
from torch.nn.utils.rnn import *
import articulate as art
from articulate.utils.torch import *
from config import *
from data_manager import DataManager
from utils import *
from dynamics import PhysicsOptimizer
from torch.nn.functional import relu
import torch


class PIP(torch.nn.Module):
    name = 'PIP'
    n_hidden = 256

    def __init__(self):
        super(PIP, self).__init__()
        self.rnn1 = RNNWithInit(input_size=72,
                                output_size=joint_set.n_leaf * 3,
                                hidden_size=self.n_hidden,
                                num_rnn_layer=2,
                                dropout=0.4)
        self.rnn2 = RNN(input_size=72 + joint_set.n_leaf * 3,
                        output_size=joint_set.n_full * 3,
                        hidden_size=self.n_hidden,
                        num_rnn_layer=2,
                        dropout=0.4)
        self.rnn3 = RNN(input_size=72 + joint_set.n_full * 3,
                        output_size=joint_set.n_reduced * 6,
                        hidden_size=self.n_hidden,
                        num_rnn_layer=2,
                        dropout=0.4)
        self.rnn4 = RNNWithInit(input_size=72 + joint_set.n_full * 3,
                                output_size=24 * 3,
                                hidden_size=self.n_hidden,
                                num_rnn_layer=2,
                                dropout=0.4)
        self.rnn5 = RNN(input_size=72 + joint_set.n_full * 3,
                        output_size=2,
                        hidden_size=64,
                        num_rnn_layer=2,
                        dropout=0.4)

        body_model = art.ParametricModel(paths.smpl_file)
        self.inverse_kinematics_R = body_model.inverse_kinematics_R
        self.forward_kinematics = body_model.forward_kinematics
        self.dynamics_optimizer = PhysicsOptimizer(debug=False)
        self.rnn_states = [None for _ in range(5)]

        self.load_state_dict(torch.load(paths.weights_file))
        self.eval()
        self.index = 0
        self.max = 0.0
        self.min = 0.0

        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        model_folder = r'C:\Users\ipop1\OneDrive\바탕 화면\smplx'
        use_face_contour = False
        num_betas = 10
        num_expression_coeffs = 10
        ext = 'pkl'
        # print('glb_axis', glb_axis)
        # print('+'*50)
        # smplx.create(model_folder)
        self.smpl_model = smplx.create(model_folder, model_type='smpl',
                                       gender='male', use_face_contour=use_face_contour,
                                       num_betas=num_betas,
                                       num_expression_coeffs=num_expression_coeffs,
                                       ext=ext,
                                       )

    def _reduced_glb_6d_to_full_local_mat(self, root_rotation, glb_reduced_pose):
        glb_reduced_pose = art.math.r6d_to_rotation_matrix(glb_reduced_pose).view(-1, joint_set.n_reduced, 3, 3)
        global_full_pose = torch.eye(3, device=glb_reduced_pose.device).repeat(glb_reduced_pose.shape[0], 24, 1, 1)
        global_full_pose[:, joint_set.reduced] = glb_reduced_pose
        pose = self.inverse_kinematics_R(global_full_pose).view(-1, 24, 3, 3)
        pose[:, joint_set.ignored] = torch.eye(3, device=pose.device)
        pose[:, 0] = root_rotation.view(-1, 3, 3)
        return pose

    def forward(self, x):
        r"""
        Forward.

        :param x: A list in length [batch_size] which contains 3-tuple
                  (tensor [num_frames, 72], tensor [15], tensor [72]).
        """
        x, lj_init, jvel_init = list(zip(*x))
        leaf_joint = self.rnn1(list(zip(x, lj_init)))
        full_joint = self.rnn2([torch.cat(_, dim=-1) for _ in zip(leaf_joint, x)])
        global_6d_pose = self.rnn3([torch.cat(_, dim=-1) for _ in zip(full_joint, x)])
        joint_velocity = self.rnn4(list(zip([torch.cat(_, dim=-1) for _ in zip(full_joint, x)], jvel_init)))
        contact = self.rnn5([torch.cat(_, dim=-1) for _ in zip(full_joint, x)])
        return leaf_joint, full_joint, global_6d_pose, joint_velocity, contact

    @torch.no_grad()
    def predict(self, glb_acc, glb_rot, init_pose):
        r"""
        Predict the results for evaluation.

        :param glb_acc: A tensor that can reshape to [num_frames, 6, 3].
        :param glb_rot: A tensor that can reshape to [num_frames, 6, 3, 3].
        :param init_pose: A tensor that can reshape to [1, 24, 3, 3].
        :return: Pose tensor in shape [num_frames, 24, 3, 3] and
                 translation tensor in shape [num_frames, 3].
        """
        self.dynamics_optimizer.reset_states()
        init_pose = init_pose.view(1, 24, 3, 3)
        init_pose[0, 0] = torch.eye(3)
        lj_init = self.forward_kinematics(init_pose)[1][0, joint_set.leaf].view(-1)
        jvel_init = torch.zeros(24 * 3)
        x = (normalize_and_concat(glb_acc, glb_rot), lj_init, jvel_init)
        leaf_joint, full_joint, global_6d_pose, joint_velocity, contact = [_[0] for _ in self.forward([x])]
        pose = self._reduced_glb_6d_to_full_local_mat(glb_rot.view(-1, 6, 3, 3)[:, -1], global_6d_pose)
        joint_velocity = joint_velocity.view(-1, 24, 3).bmm(glb_rot[:, -1].transpose(1, 2)) * vel_scale
        pose_opt, tran_opt = [], []
        for p, v, c, a in zip(pose, joint_velocity, contact, glb_acc):
            p, t = self.dynamics_optimizer.optimize_frame(p, v, c, a)
            pose_opt.append(p)
            tran_opt.append(t)
        pose_opt, tran_opt = torch.stack(pose_opt), torch.stack(tran_opt)
        return pose_opt, tran_opt

    @torch.no_grad()
    def forward_frame(self, glb_acc, glb_rot, check_rbdl, return_grf=False):
        r"""
        Forward. Currently only support 1 subject.

        :param glb_acc: A tensor in [num_subjects, 6, 3].
        :param glb_rot: A tensor in [num_subjects, 6, 3, 3].
        :param return_grf: Whether to return ground reaction force.
        :return: If return_grf is False, return (pose, translation).
                 If return_grf is True, return (pose, translation, collision_joints, contact_forces).
        """
        imu = normalize_and_concat(glb_acc, glb_rot)

        x, self.rnn_states[0] = self.rnn1.rnn(relu(self.rnn1.linear1(imu), inplace=True).unsqueeze(0), self.rnn_states[0])
        x = self.rnn1.linear2(x[0])
        x = torch.cat([x, imu], dim=1)

        x, self.rnn_states[1] = self.rnn2.rnn(relu(self.rnn2.linear1(x), inplace=True).unsqueeze(0), self.rnn_states[1])
        x = self.rnn2.linear2(x[0])

        #
        # betas = torch.randn([1, self.smpl_model.num_betas], dtype=torch.float32)
        # expression = torch.randn([1, self.smpl_model.num_expression_coeffs], dtype=torch.float32)
        #
        # smpl_body_axis, test_pose = self.smpl_model(betas=betas, expression=expression, body_pose=glb_axis, return_verts=True)
        #
        # test_pose = test_pose.flatten()
        # test_pose = test_pose.unsqueeze(0)
        # test_pose = test_pose[0][:-3]
        # test_pose = test_pose.reshape(1, -1)
        #
        # # # 1차 수정
        # test_pose = test_pose[:, 3:]
        # # last_value = x[0, -3].view(1, 1)
        # last_value = torch.tensor([[0.0]])
        # test_pose = torch.cat((test_pose, last_value), dim=1)
        # # last_value = x[0, -2].view(1, 1)
        # test_pose = torch.cat((test_pose, last_value), dim=1)
        # # last_value = x[0, -1].view(1, 1)
        # test_pose = torch.cat((test_pose, last_value), dim=1)

        x = torch.cat([x, imu], dim=1)
        # x = torch.cat([test_pose, imu], dim=1)

        x1, self.rnn_states[2] = self.rnn3.rnn(relu(self.rnn3.linear1(x), inplace=True).unsqueeze(0),
                                               self.rnn_states[2])
        global_6d_pose = self.rnn3.linear2(x1[0])


        x1, self.rnn_states[3] = self.rnn4.rnn(relu(self.rnn4.linear1(x), inplace=True).unsqueeze(0),
                                               self.rnn_states[3])
        joint_velocity = self.rnn4.linear2(x1[0])

        x1, self.rnn_states[4] = self.rnn5.rnn(relu(self.rnn5.linear1(x), inplace=True).unsqueeze(0),
                                               self.rnn_states[4])

        # print(x1.shape)
        contact = self.rnn5.linear2(x1[0])

        pose = self._reduced_glb_6d_to_full_local_mat(glb_rot[:, -1].cpu(), global_6d_pose.cpu())
        joint_velocity = (joint_velocity.view(-1, 24, 3).bmm(glb_rot[:, -1].transpose(1, 2)) * vel_scale).cpu()

        #

        # if not check_rbdl:
        pose[0] = art.math.quaternion_to_rotation_matrix(torch.tensor(DataManager().premodel_output_q)* 1.0)
        joint_velocity[0] = torch.tensor(DataManager().premodel_output_vel)


        # TODO: multiple people
        return self.dynamics_optimizer.optimize_frame(pose[0], joint_velocity[0], contact[0].cpu(), glb_acc.cpu(), check_rbdl,
                                                      return_grf=return_grf)
