from tkinter import Place

from sensor.sensor_axis import SensorAxis
import torch
import articulate as art
import math

from sensor.quaternion import Quaternion
import torch


def cul_q_acc(quaternion):
    qAccX = (-1.0) * 2.0 * (quaternion.x * quaternion.z - quaternion.w * quaternion.y)
    qAccY = (-1.0) * 2.0 * (quaternion.y * quaternion.z + quaternion.w * quaternion.x)
    qAccZ = 1.0 - 2.0 * (quaternion.w * quaternion.w + quaternion.z * quaternion.z)
    return qAccX, qAccY, qAccZ

def rotation_matrix_to_quaternion(r: torch.Tensor):
    r"""
    Turn rotation matrices into quaternions wxyz. (torch, batch)

    :param r: Rotation matrix tensor of shape [batch_size, 3, 3].
    :return: Quaternion tensor of shape [batch_size, 4].
    """
    batch_size = r.size(0)

    # 추출하는 요소
    trace = r[:, 0, 0] + r[:, 1, 1] + r[:, 2, 2]
    cond1 = trace > 0
    cond2 = (r[:, 0, 0] >= r[:, 1, 1]) & (r[:, 0, 0] >= r[:, 2, 2])
    cond3 = (r[:, 1, 1] > r[:, 2, 2])

    q = torch.zeros((batch_size, 4), dtype=r.dtype, device=r.device)

    # Trace가 양수일 때
    s1 = torch.sqrt(trace + 1.0) * 2
    q[cond1, 0] = 0.25 * s1[cond1]
    q[cond1, 1] = (r[cond1, 2, 1] - r[cond1, 1, 2]) / s1[cond1]
    q[cond1, 2] = (r[cond1, 0, 2] - r[cond1, 2, 0]) / s1[cond1]
    q[cond1, 3] = (r[cond1, 1, 0] - r[cond1, 0, 1]) / s1[cond1]

    # r[0, 0]이 가장 클 때
    s2 = torch.sqrt(1.0 + r[:, 0, 0] - r[:, 1, 1] - r[:, 2, 2]) * 2
    q[~cond1 & cond2, 0] = (r[~cond1 & cond2, 2, 1] - r[~cond1 & cond2, 1, 2]) / s2[~cond1 & cond2]
    q[~cond1 & cond2, 1] = 0.25 * s2[~cond1 & cond2]
    q[~cond1 & cond2, 2] = (r[~cond1 & cond2, 0, 1] + r[~cond1 & cond2, 1, 0]) / s2[~cond1 & cond2]
    q[~cond1 & cond2, 3] = (r[~cond1 & cond2, 0, 2] + r[~cond1 & cond2, 2, 0]) / s2[~cond1 & cond2]

    # r[1, 1]이 가장 클 때
    s3 = torch.sqrt(1.0 + r[:, 1, 1] - r[:, 0, 0] - r[:, 2, 2]) * 2
    q[~cond1 & ~cond2 & cond3, 0] = (r[~cond1 & ~cond2 & cond3, 0, 2] - r[~cond1 & ~cond2 & cond3, 2, 0]) / s3[~cond1 & ~cond2 & cond3]
    q[~cond1 & ~cond2 & cond3, 1] = (r[~cond1 & ~cond2 & cond3, 0, 1] + r[~cond1 & ~cond2 & cond3, 1, 0]) / s3[~cond1 & ~cond2 & cond3]
    q[~cond1 & ~cond2 & cond3, 2] = 0.25 * s3[~cond1 & ~cond2 & cond3]
    q[~cond1 & ~cond2 & cond3, 3] = (r[~cond1 & ~cond2 & cond3, 1, 2] + r[~cond1 & ~cond2 & cond3, 2, 1]) / s3[~cond1 & ~cond2 & cond3]

    # r[2, 2]가 가장 클 때
    s4 = torch.sqrt(1.0 + r[:, 2, 2] - r[:, 0, 0] - r[:, 1, 1]) * 2
    q[~cond1 & ~cond2 & ~cond3, 0] = (r[~cond1 & ~cond2 & ~cond3, 1, 0] - r[~cond1 & ~cond2 & ~cond3, 0, 1]) / s4[~cond1 & ~cond2 & ~cond3]
    q[~cond1 & ~cond2 & ~cond3, 1] = (r[~cond1 & ~cond2 & ~cond3, 0, 2] + r[~cond1 & ~cond2 & ~cond3, 2, 0]) / s4[~cond1 & ~cond2 & ~cond3]
    q[~cond1 & ~cond2 & ~cond3, 2] = (r[~cond1 & ~cond2 & ~cond3, 1, 2] + r[~cond1 & ~cond2 & ~cond3, 2, 1]) / s4[~cond1 & ~cond2 & ~cond3]
    q[~cond1 & ~cond2 & ~cond3, 3] = 0.25 * s4[~cond1 & ~cond2 & ~cond3]

    return q


# 차렷 자세
# [SensorPart.LEFT_LOWER_ARM, SensorPart.RIGHT_LOWER_ARM, SensorPart.LEFT_LOWER_LEG, SensorPart.RIGHT_LOWER_LEG, SensorPart.HEAD, SensorPart.WAIST]
# acc = [[0.16374559700489044, -0.1120217964053154, 0.0035693019162863493], [0.05629219114780426, -0.11285070329904556, 0.049275148659944534], [0.018072880804538727, -0.05502019077539444, -0.0034163789823651314], [0.040248360484838486, -0.05427727848291397, -0.011102619580924511], [0.012752249836921692, -0.04400873929262161, 0.06726399809122086], [0.015420890413224697, -0.049260251224040985, 0.017035380005836487]]
# q = [[[-5.1023e-02,  9.9043e-01, -1.2825e-01],
#          [-9.7970e-01, -2.4710e-02,  1.9894e-01],
#          [ 1.9387e-01,  1.3579e-01,  9.7158e-01]],  # 왼팔

#         [[-1.6119e-02, -9.6344e-01,  2.6742e-01],
#          [ 9.7746e-01,  4.1130e-02,  2.0709e-01],
#          [-2.1052e-01,  2.6473e-01,  9.4106e-01]],  # 오른팔

#         [[ 9.9686e-01,  6.3931e-02,  4.6792e-02],
#          [-6.3602e-02,  9.9794e-01, -8.4955e-03],
#          [-4.7239e-02,  5.4928e-03,  9.9887e-01]], # 왼다리

#         [[ 9.9840e-01, -4.6207e-02,  3.2595e-02],
#          [ 4.6152e-02,  9.9893e-01,  2.4210e-03],
#          [-3.2672e-02, -9.1281e-04,  9.9947e-01]], # 오른 다리

#         [[ 9.9967e-01, -2.0414e-02, -1.5664e-02],
#          [ 2.0064e-02,  9.9955e-01, -2.2193e-02],
#          [ 1.6110e-02,  2.1871e-02,  9.9963e-01]], # 머리

#         [[ 9.9948e-01,  1.0812e-02, -3.0464e-02],
#          [-1.0627e-02,  9.9992e-01,  6.2269e-03],
#          [ 3.0529e-02, -5.8999e-03,  9.9952e-01]]]  #허리


# acc = [[-22.209009170532227, -1.9666359424591064, 0.8644970059394836], [17.121570587158203, -1.7815639972686768, -6.758988857269287], [-0.016582349315285683, -9.526618003845215, -1.982782006263733], [-1.5869280099868774, -9.053971290588379, 0.5326644778251648], [-0.7516897916793823, -11.274689674377441, -2.199686050415039], [0.6616982221603394, -9.660760879516602, -1.0022640228271484]]
# q = [[[ 0.8645,  0.3465, -0.3641],
#          [ 0.0016,  0.7225,  0.6914],
#          [ 0.5027, -0.5983,  0.6240]],

#         [[ 0.5160, -0.5608,  0.6476],
#          [ 0.0525,  0.7752,  0.6295],
#          [-0.8550, -0.2908,  0.4294]],

#         [[ 0.9954,  0.0207,  0.0939],
#          [-0.0050,  0.9863, -0.1649],
#          [-0.0960,  0.1637,  0.9818]],

#         [[ 0.9819,  0.1451, -0.1221],
#          [-0.1553,  0.9848, -0.0783],
#          [ 0.1088,  0.0958,  0.9894]],

#         [[ 0.9981, -0.0515,  0.0344],
#          [ 0.0528,  0.9978, -0.0398],
#          [-0.0322,  0.0415,  0.9986]],

#         [[ 0.9969,  0.0194,  0.0769],
#          [-0.0164,  0.9991, -0.0402],
#          [-0.0776,  0.0388,  0.9962]]]


# q = torch.tensor(q)

# print(acc)
# quaternions = rotation_matrix_to_quaternion(q)
# print(quaternions)

# part = ["왼팔", "오른팔", "왼다리", "오른다리", "머리", "허리"]

# for i, v in enumerate(part):
#     print(v + " 중력 가속도 = " + str(cul_q_acc(Quaternion(quaternions[i][0], quaternions[i][1], quaternions[i][2],  quaternions[i][3]))))

