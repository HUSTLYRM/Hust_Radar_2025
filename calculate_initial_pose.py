import numpy as np


def rotation_matrix(pitch, yaw):
    """
    计算绕 Y 轴和 Z 轴旋转的合成旋转矩阵
    pitch: 绕 Y 轴的旋转角度 (弧度)
    yaw: 绕 Z 轴的旋转角度 (弧度)
    """
    # 绕 Y 轴旋转的矩阵
    R_y = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])

    # 绕 Z 轴旋转的矩阵
    R_z = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])

    # 合成旋转矩阵
    R_total = np.dot(R_z, R_y)

    return R_total
import math
T = [-1.12, -9.335, 3.66]
yaw = math.atan((-7.85708 + 9.335) / (14.11634 + 1.12))
print(yaw)
d = math.sqrt((-7.85708 + 9.335)**2 + (14.11634 + 1.12)**2)
print(d)
pitch = math.atan((4.1 - 0.81888) / d)
print(pitch)
R = rotation_matrix(pitch,yaw)
print(R)
transformation_matrix = np.array([[R[0][0], R[0][1], R[0][2], T[0]],
                                   [R[1][0], R[1][1], R[1][2], T[1]],
                                   [R[2][0], R[2][1], R[2][2], T[2]],
                                   [0, 0, 0, 1]])
print(transformation_matrix)
# [[ 0.9732233  -0.09654654  0.20860292 -1.12      ]
#  [ 0.09440234  0.99532847  0.02023442 -9.335     ]
#  [-0.20958199  0.          0.97779108  3.66      ]
#  [ 0.          0.          0.          1.        ]]