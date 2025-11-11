
import articulate as art
from articulate.utils.torch import *
from config import *
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
        # self.dynamics_optimizer = PhysicsOptimizer(debug=False)
        self.rnn_states = [None for _ in range(5)]

        self.rnn5.load_w()

        # self.load_state_dict(torch.load(paths.weights_file))
        self.eval()
        self.rnn5.eval()

    def _reduced_glb_6d_to_full_local_mat(self, root_rotation, glb_reduced_pose):
        glb_reduced_pose = art.math.r6d_to_rotation_matrix(glb_reduced_pose).view(-1, joint_set.n_reduced, 3, 3)
        global_full_pose = torch.eye(3, device=glb_reduced_pose.device).repeat(glb_reduced_pose.shape[0], 24, 1, 1)
        global_full_pose[:, joint_set.reduced] = glb_reduced_pose
        pose = self.inverse_kinematics_R(global_full_pose).view(-1, 24, 3, 3)
        pose[:, joint_set.ignored] = torch.eye(3, device=pose.device)
        pose[:, 0] = root_rotation.view(-1, 3, 3)
        return pose


    @torch.no_grad()
    def predict(self, glb_acc, glb_rot, init_pose):
        import torch

        dev = next(self.parameters()).device  # 모델이 올라간 디바이스 (cuda:0 또는 cpu)

        glb_acc = glb_acc.to(device=dev, dtype=torch.float32, non_blocking=True)
        glb_rot = glb_rot.to(device=dev, dtype=torch.float32, non_blocking=True)
        init_pose = init_pose.to(device=dev, dtype=torch.float32, non_blocking=True)

        init_pose = init_pose.view(-1)[3:].unsqueeze(0)

        x_seq = ten_normalize_and_concat(glb_acc, glb_rot).to(dev)


        batch = [(x_seq, init_pose)]

        print(batch.shape)


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
