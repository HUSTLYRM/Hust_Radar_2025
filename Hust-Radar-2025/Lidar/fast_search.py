import numpy as np
import torch
import time
# 将数据变成torch_tensor，移到GPU上运算
class FastSearch:
    def __init__(self,device = "cuda",Radius_upperbound = 5):
        self.dynamic_pcd = None # 动态障碍物点云，是寻找的目标
        self.device = device
        self.src_pcd = None
        self.r = Radius_upperbound

     # 计算所有点到目标点的欧氏距离
    def compute_distances(self,point_cloud_tensor, point_0_tensor):
        """
        计算点云中所有点到点云0的距离
        :param point_cloud_tensor: 点云数据，PyTorch Tensor，形状为 (N, 3)
        :param point_0_tensor: 目标点，PyTorch Tensor，形状为 (3,)
        :return: 每个点到目标点的欧氏距离，PyTorch Tensor，形状为 (N,)
        """

        return torch.norm(point_cloud_tensor - point_0_tensor, dim=1)

    # 查找距离目标点最近的点
    def find_nearest_point(self,point_cloud, point_0):
        """
        找到点云中距离点云0最近的点
        :param point_cloud: 点云数据，NumPy 数组，形状为 (N, 3)
        :param point_0: 查询点，NumPy 数组，形状为 (3,)
        :return: 最近点，NumPy 数组，形状为 (3,)
        :return: 最近点的距离，float
        """

        # 转换为 PyTorch 张量
        point_cloud_tensor = torch.tensor(point_cloud, dtype=torch.float32)
        point_0_tensor = torch.tensor(point_0, dtype=torch.float32)

        # 计算每个点到点云0的距离
        distances = self.compute_distances(point_cloud_tensor, point_0_tensor)

        # 找到最小距离的索引
        min_distance_idx = torch.argmin(distances)

        # 获取距离最近的点
        nearest_point = point_cloud_tensor[min_distance_idx]

        # 获取最小距离
        min_distance = distances[min_distance_idx]

        # 返回最近点的 (x, y, z) 和最小距离
        nearest_point_xyz = np.array(nearest_point)  # 转换为 (x, y, z) 元组
        return nearest_point_xyz, min_distance.item()

    # 示例使用
if __name__ == "__main__":
    # 假设 point_cloud 和 point_0 是你已经有的 NumPy 数组
    point_cloud = np.random.rand(15000, 3)  # 示例点云数据
    point_0 = np.array([1.0, 2.0, 3.0])  # 示例查询点
    f = FastSearch(device = "cuda",Radius_upperbound = 5)
    t1 = time.time()
    # 查找距离最近的点
    nearest_point, min_distance = f.find_nearest_point(point_cloud, point_0)
    t2 = time.time()
    print("用时：",(t2-t1)*1000,"ms")
    print(f"最接近的点是: {nearest_point}, 距离是: {min_distance}")
