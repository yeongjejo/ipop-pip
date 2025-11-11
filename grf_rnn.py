import smplx
from torch.nn.utils.rnn import *
import articulate as art
from articulate.utils.torch import *
from config import *
from data_manager import DataManager
from utils import *
from dynamics import PhysicsOptimizer
# from ipop_dynamics import PhysicsOptimizer
from torch.nn.functional import relu
import torch

from utils import ten_normalize_and_concat


class GRFRNN(torch.nn.Module):
    name = 'GRFRNN'
    n_hidden = 256

    def __init__(self):
        super(GRFRNN, self).__init__()

        self.rnn5 = RNN(input_size=120 + joint_set.n_full * 3,
                        output_size=2,
                        hidden_size=64,
                        num_rnn_layer=2,
                        dropout=0.4)

        body_model = art.ParametricModel(paths.smpl_file)
        self.inverse_kinematics_R = body_model.inverse_kinematics_R
        self.forward_kinematics = body_model.forward_kinematics
        self.dynamics_optimizer = PhysicsOptimizer(debug=False)
        self.rnn_states = [None for _ in range(5)]

        self.rnn5.load_w()

        # self.load_state_dict(torch.load(paths.weights_file))
        self.eval()
        self.rnn5.eval()
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
        import torch

        dev = next(self.parameters()).device  # 모델이 올라간 디바이스 (cuda:0 또는 cpu)

        # 1) 입력 텐서를 모델 디바이스/float32로 통일
        glb_acc = glb_acc.to(device=dev, dtype=torch.float32, non_blocking=True)
        glb_rot = glb_rot.to(device=dev, dtype=torch.float32, non_blocking=True)
        init_pose = init_pose.to(device=dev, dtype=torch.float32, non_blocking=True)

        # 2) init_pose를 기존 코드와 동일하게 펼친 뒤 디바이스 유지
        init_pose = init_pose.view(-1)[3:].unsqueeze(0)  # shape: [1, 24*9 - 3] (코드 의도 유지)

        # 3) normalize_and_concat 결과도 같은 디바이스로
        x_seq = ten_normalize_and_concat(glb_acc, glb_rot).to(dev)  # shape: [T, 72] 등

        # 4) 배치 한 개 형태로 forward에 전달
        #    forward가 (x, lj_init, jvel_init)을 기대한다면, 아래처럼 0 초기화 추가:
        #    jvel_init = torch.zeros(24 * 3, device=dev)
        #    batch = [(x_seq, init_pose, jvel_init)]
        #    그렇지 않고 (x, init_pose)만 받는 구현이면 아래 그대로 사용
        batch = [(x_seq, init_pose)]

        contact = [c[0] for c in self.forward(batch)]
        return contact

    @torch.no_grad()
    def forward(self, x):
        r"""
        Forward.

        :param x: A list in length [batch_size] which contains 3-tuple
                  (tensor [num_frames, 72], tensor [15], tensor [72]).
        """

        x, full_joint = list(zip(*x))
        # leaf_joint = self.rnn1(list(zip(x, lj_init)))
        # full_joint = self.rnn2([torch.cat(_, dim=-1) for _ in zip(leaf_joint, x)])
        # global_6d_pose = self.rnn3([torch.cat(_, dim=-1) for _ in zip(full_joint, x)])
        # joint_velocity = self.rnn4(list(zip([torch.cat(_, dim=-1) for _ in zip(full_joint, x)], jvel_init)))
        # print(x[0].shape)
        # print(full_joint[0].shape)
        # for abb_ in zip(full_joint, x):
        #     print(zip(full_joint, x))
        contact = self.rnn5([torch.cat(_, dim=-1) for _ in zip(full_joint, x)])
        # print(torch.sigmoid(contact[0]))
        # return torch.sigmoid(contact[0])

        # print(contact.requires_grad)
        return contact
        # return leaf_joint, full_joint, global_6d_pose, joint_velocity, contact


        # #
        # pose = art.math.quaternion_to_rotation_matrix(torch.tensor(DataManager().premodel_output_q)* 1.0)
        # joint_velocity = torch.tensor(DataManager().premodel_output_vel)
        #
        # # TODO: multiple people
        # return self.dynamics_optimizer.optimize_frame(pose, joint_velocity, contact[0].cpu(), glb_acc.cpu(), check_rbdl,
        #                                               return_grf=return_grf)
