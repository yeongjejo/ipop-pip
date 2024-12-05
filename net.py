import smplx
from torch.nn.utils.rnn import *
import articulate as art
from articulate.utils.torch import *
from config import *
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
    def forward_frame(self, glb_acc, glb_rot, glb_axis, test_print, return_grf=False):
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

        model_folder = r'C:\Users\USER\Desktop\smplx'
        model_type = 'smpl'
        gender = 'male'
        use_face_contour = False
        num_betas = 10
        num_expression_coeffs = 10
        ext = 'pkl'
        # smplx.create(model_folder)
        model = smplx.create(model_folder, model_type='smpl',
                             gender='male', use_face_contour=use_face_contour,
                             num_betas=num_betas,
                             num_expression_coeffs=num_expression_coeffs,
                             ext=ext,
                             body_pose=glb_axis)

        # print(model)
        betas = torch.randn([1, model.num_betas], dtype=torch.float32)
        expression = torch.randn(
            [1, model.num_expression_coeffs], dtype=torch.float32)
        _, test_pose = model(betas=betas, expression=expression,
                             return_verts=True)

        test_pose = test_pose.flatten()
        test_pose = test_pose.unsqueeze(0)
        test_pose = test_pose[0][:-3]
        test_pose = test_pose.reshape(1, -1)

        if test_print:
            print('x', x)
            print('수정전 test_pose', test_pose)

        # 1차 수정
        test_pose = test_pose[:, 3:]
        last_value = x[0, -3].view(1, 1)
        test_pose = torch.cat((test_pose, last_value), dim=1)
        last_value = x[0, -2].view(1, 1)
        test_pose = torch.cat((test_pose, last_value), dim=1)
        last_value = x[0, -1].view(1, 1)
        test_pose = torch.cat((test_pose, last_value), dim=1)

        if test_print:
            print('수정후 test_pose', test_pose)

        # 2차 수정
        x[0, 0:3] = test_pose[0, 0:3]
        x[0, 3:6] = test_pose[0, 3:6]

        # 어깨
        x[0, 36:39] = test_pose[0, 45:48]
        x[0, 39:42] = test_pose[0, 48:51]
        x[0, 45:48] = test_pose[0, 36:39]
        x[0, 48:51] = test_pose[0, 39:42]

        # 발
        # x[0, 9:12] = test_pose[0, 9:12]
        # x[0, 12:15] = test_pose[0, 12:15]
        # x[0, 18:21] = test_pose[0, 18:21]
        # x[0, 21:24] = test_pose[0, 21:24]



        x = torch.cat([x, imu], dim=1)

        x1, self.rnn_states[2] = self.rnn3.rnn(relu(self.rnn3.linear1(x), inplace=True).unsqueeze(0), self.rnn_states[2])
        global_6d_pose = self.rnn3.linear2(x1[0])

        x1, self.rnn_states[3] = self.rnn4.rnn(relu(self.rnn4.linear1(x), inplace=True).unsqueeze(0), self.rnn_states[3])
        joint_velocity = self.rnn4.linear2(x1[0])

        x1, self.rnn_states[4] = self.rnn5.rnn(relu(self.rnn5.linear1(x), inplace=True).unsqueeze(0), self.rnn_states[4])
        contact = self.rnn5.linear2(x1[0])

        pose = self._reduced_glb_6d_to_full_local_mat(glb_rot[:, -1].cpu(), global_6d_pose.cpu())
        joint_velocity = (joint_velocity.view(-1, 24, 3).bmm(glb_rot[:, -1].transpose(1, 2)) * vel_scale).cpu()
       

        # TODO: multiple people
        return self.dynamics_optimizer.optimize_frame(pose[0], joint_velocity[0], contact[0].cpu(), glb_acc.cpu(), return_grf=return_grf)
