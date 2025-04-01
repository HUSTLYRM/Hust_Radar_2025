import time
# 将数据变成torch_tensor，移到GPU上运算
import torch
import numpy as np


class FastSearch:
    def __init__(self, device="cuda", Radius_upperbound=3):
        self.dynamic_pcd = None  # 动态障碍物点云，是寻找的目标
        self.device = device
        self.src_pcd = None
        self.r = Radius_upperbound  # 设定最大半径上限

    def select_points(self, pc, pcd):
        # pc: [x,y,z,t]
        # pcd:[x,y,z,t]
        x, y, z, t = pc
        threshold = 2
        # 选出时间戳在10s以内的点
        dt = 5
        pcd = pcd[pcd[:,3]> t - dt]
        pcd = pcd[pcd[:,3]< t + dt]
        # 选出pcd中横坐标小于x-threshold的点
        pcd = pcd[pcd[:, 0] > x - threshold]
        # 选出pcd中横坐标大于x+threshold的点
        pcd = pcd[pcd[:, 0] < x + threshold]
        # 选出pcd中纵坐标小于y-threshold的点
        pcd = pcd[pcd[:, 1] < y + threshold]
        pcd = pcd[pcd[:, 1] > y - threshold]
        # 选出pcd中纵坐标大于y+threshold的点
        return pcd

    # 计算所有点到目标点的欧氏距离（只计算xy维度，避免使用torch.norm）
    def compute_distances(self, point_cloud_tensor, point_0_tensor):
        """
        计算点云中所有点到点云0的欧氏距离，只计算xy维度
        :param point_cloud_tensor: 点云数据，PyTorch Tensor，形状为 (N, 3)
        :param point_0_tensor: 目标点，PyTorch Tensor，形状为 (3,)
        :return: 每个点到目标点的欧氏距离（xy维度），PyTorch Tensor，形状为 (N,)
        """
        # 只选择 x 和 y 维度
        point_cloud_xy = point_cloud_tensor[:, :2]
        point_0_xy = point_0_tensor[:2]

        # 计算 xy 维度的欧氏距离的平方，避免计算开方
        return torch.sum((point_cloud_xy - point_0_xy) ** 2, dim=1)

    # 查找距离目标点最近的点，并判断最小距离是否超过设定的阈值
    def find_nearest_point(self, point_cloud, point_0):
        """
        找到点云中距离点云0最近的点（只考虑xy维度）
        :param point_cloud: 点云数据，NumPy 数组，形状为 (N, 3)
        :param point_0: 查询点，NumPy 数组，形状为 (3,)
        :return: 最近点，NumPy 数组，形状为 (3,)
        :return: 最近点的距离，float
        :return: 是否超过阈值，bool
        """
        # 转换为 PyTorch 张量，并移动到指定设备
        point_cloud_tensor = torch.tensor(point_cloud, dtype=torch.float32).to(self.device)
        point_0_tensor = torch.tensor(point_0, dtype=torch.float32).to(self.device)

        # 计算每个点到点云0的距离（只考虑xy维度）
        distances_squared = self.compute_distances(point_cloud_tensor, point_0_tensor)

        # 找到最小距离的索引（最小平方距离）
        min_distance_idx = torch.argmin(distances_squared)

        # 获取距离最近的点
        nearest_point = point_cloud_tensor[min_distance_idx]

        # 获取最小距离的平方根（如果需要最短距离）
        min_distance = torch.sqrt(distances_squared[min_distance_idx])

        # 判断最小距离是否超过阈值
        distance_exceeds_threshold = min_distance.item() > self.r

        # 转换为 NumPy 数组并返回
        nearest_point_xyz = nearest_point.cpu().numpy()  # 移动回 CPU 并转为 NumPy 数组
        return nearest_point_xyz, min_distance.item(), distance_exceeds_threshold

    # 示例使用


# if __name__ == "__main__":
#     # 假设 point_cloud 和 point_0 是你已经有的 NumPy 数组
#     for i in range(50):
#         point_cloud = np.random.rand(150, 3)  # 示例点云数据
#         point_0 = np.array([1.0, 2.0, 3.0])  # 示例查询点
#         f = FastSearch(device="cuda", Radius_upperbound=5)
#         t1 = time.time()
#         # 查找距离最近的点
#         point_cloud = f.select_points(point_0, point_cloud)
#         nearest_point, min_distance, is_avail = f.find_nearest_point(point_cloud, point_0)
#         t2 = time.time()
#         print("用时：", (t2 - t1) * 1000, "ms")
#         print(f"最接近的点是: {nearest_point}, 距离是: {min_distance}")
