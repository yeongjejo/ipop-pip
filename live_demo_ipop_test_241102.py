import torch
from data_manager import DataManager
from protocol.udp_server import UDPServer
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
import numpy as np

def y_only_euler_to_quaternion(input_quaternion):
    """
    입력 쿼터니언에서 Y축 회전 각도를 추출하여, Y축 각도만 반영된 쿼터니언을 YZX 순서로 생성합니다.
    
    :param input_quaternion: (w, x, y, z) 형태의 입력 쿼터니언
    :return: Y축 각도만 반영된 (w, x, y, z) 형태의 쿼터니언
    """
    # 입력 쿼터니언을 (w, x, y, z)에서 (x, y, z, w) 순서로 변환
    input_quat_xyzw = [input_quaternion[1], input_quaternion[2], input_quaternion[3], input_quaternion[0]]
    #print(f"quat: {input_quaternion}")
    # 입력 쿼터니언을 오일러 각도로 변환 (YZX 순서)
    rotation = R.from_quat(input_quat_xyzw)
    euler_angles = rotation.as_euler('yzx', degrees=False)
    #print(f"euler: {euler_angles}")
    # Y축 회전 각도만 유지하고 z와 x를 0으로 설정
    y_angle = euler_angles[0]
    new_euler_angles = [y_angle, 0, 0]  # y, z, x 순서
    
    # 새로운 오일러 각도로부터 쿼터니언 생성
    new_rotation = R.from_euler('yzx', new_euler_angles)
    new_quaternion = new_rotation.as_quat()  # (x, y, z, w) 형태
    
    # (x, y, z, w) 순서를 (w, x, y, z)로 재정렬
    quaternion_wxyz = [new_quaternion[3], new_quaternion[0], new_quaternion[1], new_quaternion[2]]
    
    return quaternion_wxyz

def x_only_euler_to_quaternion(input_quaternion):
    """
    입력 쿼터니언에서 Y축 회전 각도를 추출하여, Y축 각도만 반영된 쿼터니언을 YZX 순서로 생성합니다.
    
    :param input_quaternion: (w, x, y, z) 형태의 입력 쿼터니언
    :return: Y축 각도만 반영된 (w, x, y, z) 형태의 쿼터니언
    """
    # 입력 쿼터니언을 (w, x, y, z)에서 (x, y, z, w) 순서로 변환
    input_quat_xyzw = [input_quaternion[1], input_quaternion[2], input_quaternion[3], input_quaternion[0]]
    
    # 입력 쿼터니언을 오일러 각도로 변환 (YZX 순서)
    rotation = R.from_quat(input_quat_xyzw)
    euler_angles = rotation.as_euler('yzx', degrees=False)
    
    # X축 회전 각도만 유지하고 z와 x를 0으로 설정
    x_angle = euler_angles[2]
    new_euler_angles = [0, 0, x_angle]  # y, z, x 순서
    
    # 새로운 오일러 각도로부터 쿼터니언 생성
    new_rotation = R.from_euler('yzx', new_euler_angles)
    new_quaternion = new_rotation.as_quat()  # (x, y, z, w) 형태
    
    # (x, y, z, w) 순서를 (w, x, y, z)로 재정렬
    quaternion_wxyz = [new_quaternion[3], new_quaternion[0], new_quaternion[1], new_quaternion[2]]
    
    return quaternion_wxyz

def get_quaternion_xyzw(quaternion):
    quaternion_xyzw = [quaternion[1], quaternion[2], quaternion[3], quaternion[0]]
    
    return quaternion_xyzw

def quaternion_inverse(quaternion):
    """
    쿼터니언의 인버스를 계산합니다.
    :param quaternion: (w, x, y, z) 형태의 쿼터니언
    :return: (w, -x, -y, -z) 형태의 인버스 쿼터니언
    """
    w, x, y, z = quaternion
    norm = w**2 + x**2 + y**2 + z**2
    if norm == 0:
        raise ValueError("0 쿼터니언의 인버스는 존재하지 않습니다.")
    
    # 인버스 계산 (단위 쿼터니언인 경우 부호만 반대로)
    inv_quaternion = [w / norm, -x / norm, -y / norm, -z / norm]
    
    return inv_quaternion

def quaternion_multiply(q1, q2):
    """
    두 개의 쿼터니언을 곱합니다.
    :param q1: 첫 번째 쿼터니언 (w1, x1, y1, z1)
    :param q2: 두 번째 쿼터니언 (w2, x2, y2, z2)
    :return: 곱셈 결과 쿼터니언 (w, x, y, z)
    """
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    
    return [w, x, y, z]



class IMUSet:
    g = 9.8
    test_i = -1

    def __init__(self, test):
        self.n_imus = 6
        self.test = test

    def get_ipop(self):
        q = DataManager().test_q
        r = DataManager().test_r
        a = DataManager().test_acc
        return q, a

if __name__ == '__main__':
    UDPServer().start()
    test = False
    imu_set = IMUSet(test)

    plt.ion()
    fig = plt.figure(figsize=(10, 6))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_title("Rotating Box and Quaternion Axes")

    # Box dimensions centered at origin
    width, depth, height = 3.0, 0.5, 1.0  # Width, depth, and height
    x = [-width / 2, width / 2, width / 2, -width / 2, -width / 2, width / 2, width / 2, -width / 2]
    y = [-depth / 2, -depth / 2, depth / 2, depth / 2, -depth / 2, -depth / 2, depth / 2, depth / 2]
    z = [-height / 2, -height / 2, -height / 2, -height / 2, height / 2, height / 2, height / 2, height / 2]


    # Define vertices of the box
    vertices = np.array([x, y, z]).T
    count = 0
    mountingAdjustedRotation = None
    
    gyrofix = [1.0, 0.0, 0.0, 0.0]
    attachmentFix = [1.0, 0.0, 0.0, 0.0]
    
    while not test:
        try:
            # Get real-time quaternion data
            joint_index = 4
            raw_data_quaternion, a = imu_set.get_ipop()
            raw_data_quaternion = torch.tensor(raw_data_quaternion)
            raw_data_quaternion = raw_data_quaternion[joint_index]
            raw_data_quaternion = [raw_data_quaternion[0], -raw_data_quaternion[3], raw_data_quaternion[1], raw_data_quaternion[2]]
            
            if count == 50:
            #     mountingAdjustedRotation = raw_data_quaternion
            
                gyrofix_y = y_only_euler_to_quaternion(raw_data_quaternion)
                #print(gyrofix_y)
                gyrofix = quaternion_inverse(gyrofix_y)
                
                # gyrofix_x = x_only_euler_to_quaternion(mountingAdjustedRotation)
                # gyrofix_x = quaternion_inverse(gyrofix_x)
                # gyrofix_z = y_only_euler_to_quaternion(mountingAdjustedRotation)
                # gyrofix_z = quaternion_inverse(gyrofix_z)
                
                
                # gyrofix = quaternion_multiply(raw_data_quaternion, gyrofix_y)
                # gyrofix = quaternion_multiply(gyrofix, gyrofix_x)
                #quaternion = quaternion_multiply(raw_data_quaternion, gyrofix)
                attachmentFix = quaternion_multiply(gyrofix, raw_data_quaternion)
                attachmentFix = quaternion_inverse(attachmentFix)
                
                print("리셋!!!!!!!!!!!")
                
            
            count += 1

            # Clear previous plot
            ax.cla()
            ax.set_title("Rotating Box and Quaternion Axes")
            ax.set_xlim([-3, 3])
            ax.set_ylim([-3, 3])
            ax.set_zlim([-3, 3])
            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Z')

            # Convert quaternion to rotation matrix using scipy # 1, 2, 3  1, 3, 2  2, 1, 3,  2, 3, 1  3, 1, 2  3, 2, 1
            
            #IMU SENSOR
            #quaternion = [q[joint_index][1].item(), q[joint_index][3].item(), q[joint_index][2].item(), q[joint_index][0].item()]  # [x, y, z, w]
            
            #processed
          
            # gyrofix = y_only_euler_to_quaternion(30*0.0175)
            #print(f'mount: {mountingAdjustedRotation}')

            
            # attachmentFix = quaternion_multiply(raw_data_quaternion, attachmentFix)
            
            raw_data_quaternion = quaternion_multiply(gyrofix, raw_data_quaternion)
            raw_data_quaternion = quaternion_multiply(raw_data_quaternion, attachmentFix)

            # quaternion_test = quaternion_inverse((0.707, 0.707, 0.0, 0.0))
            # gyrofix_1 = y_only_euler_to_quaternion()
            # gyrofix_1 = quaternion_inverse(gyrofix_1)
            
            # gyrofix_test = quaternion_multiply(gyrofix, quaternion_test)
            # 90degree (0.707, 0.0, 0.0, 0.707) w, x, y, z
            # yawfix = quaternion_multiply(gyrofix, mountingAdjustedRotation)
            # yawfix = quaternion_multiply(yawfix, attachmentFix)
            # yawfix = quaternion_multiply(quaternion_inverse([1,0,0,0]), quaternion_multiply())
            
            
            #quaternion = raw_data_quaternion
            # raw_data_quaternion = gyrofixe
            
            # visualization
            raw_data_quaternion = [raw_data_quaternion[1], raw_data_quaternion[2], raw_data_quaternion[3], raw_data_quaternion[0]]
            r = R.from_quat(raw_data_quaternion)
            rotation_matrix = r.as_matrix()

            # Rotate vertices
            rotated_vertices = vertices @ rotation_matrix.T

            # Plot the rotated box
            ax.quiver(0, 0, 0, 2, 0, 0, color='r', arrow_length_ratio=0.05, label='X-axis')  # X-axis in red
            ax.quiver(0, 0, 0, 0, 2, 0, color='g', arrow_length_ratio=0.05, label='Y-axis')  # Y-axis in green
            ax.quiver(0, 0, 0, 0, 0, 2, color='b', arrow_length_ratio=0.05, label='Z-axis')  # Z-axis in blue

            # Apply rotation to local axes and plot as arrows
            origin = np.array([0, 0, 0])
            local_axes = np.identity(3)  # Local X, Y, Z axes before rotation
            rotated_axes = local_axes @ rotation_matrix.T

            ax.quiver(*origin, *rotated_axes[0] * 2, color='r', linestyle=':', arrow_length_ratio=0.05, label='Rotated X-axis')
            ax.quiver(*origin, *rotated_axes[1] * 2, color='g', linestyle=':', arrow_length_ratio=0.05, label='Rotated Y-axis')
            ax.quiver(*origin, *rotated_axes[2] * 2, color='b', linestyle=':', arrow_length_ratio=0.05, label='Rotated Z-axis')

            ax.legend()

            plt.draw()
            plt.pause(0.1)

        except Exception as e:
            print("Error:", e)

    plt.ioff()  # Disable interactive mode
    plt.show()
