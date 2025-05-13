import numpy as np

def is_point_nearby_numpy(target, points, radius=5.0):
    target = np.array(target)
    points = np.array(points)
    distances = np.linalg.norm(points - target, axis=1)
    return np.any(distances <= radius)

# 示例
target = [10, 10]
points = [[12, 13], [4, 5], [16, 18]]
print(is_point_nearby_numpy(target, points))  # True or False
