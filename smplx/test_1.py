from smplx import SMPL
import torch

# SMPL 모델 로드
model = SMPL(model_path='basicmodel_m_lbs_10_207_0_v1.0.0.pkl')

# 임의의 포즈와 형상 벡터
body_pose = torch.randn(1, 72)  # SMPL 포즈 벡터
betas = torch.randn(1, 10) # SMPL 베타 벡터 (형상 파라미터)

# 3D vertices와 joints 생성
root = pose [:, -3:]
output = model.forward(betas=betas, global_orient=pose[:, :3])
vertices = output.vertices
joints = output.joints
body_pose = output.body_pose
print(vertices)
print(joints)
print(body_pose)
print(joints.shape)
print(vertices.shape)
print(body_pose.shape)
