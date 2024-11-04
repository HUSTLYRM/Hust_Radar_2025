import open3d as o3d
import numpy as np
class Clear():
    def __init__(self,pcd , trans_matrix, tgt_pcd_path = '/home/nvidia/RadarWorkspace/code/Radar/pcd_data/tgt.pcd'):
        self.src_pcd = pcd
        self.trans_matrix = trans_matrix
        self.tgt_pcd = o3d.io.read_point_cloud(tgt_pcd_path)
        self.pcd = None
        self.output_pcd = None
        self.threshold = 0.01

    def trans_pcd(self):
        self.pcd = self.src_pcd.transform(self.trans_matrix)

    def compute_difference(self):
        # KDTree 搜索
        kdtree = o3d.geometry.KDTreeFlann(self.pcd)

        # 标记重合点
        LA01 = np.zeros(len(self.pcd.points), dtype=int)  # Acloud 中的重合标记
        LB01 = np.zeros(len(self.tgt_pcd.points), dtype=int)  # Bcloud 中的重合标记

        for i in range(len(self.tgt_pcd.points)):
            [k, idx, _] = kdtree.search_knn_vector_3d(self.tgt_pcd.points[i], 1)
            if k > 0 and np.linalg.norm(self.tgt_pcd.points[i] - self.pcd.points[idx[0]]) <= self.threshold:
                LB01[i] = 1
                LA01[idx[0]] = 1

        # 提取不同点
        LA = [i for i in range(len(self.pcd.points)) if LA01[i] == 0]
        # LB = [i for i in range(len(self.tgt_pcd.points)) if LB01[i] == 0]

        src_recloud = self.pcd.select_by_index(LA)  # A 中剩余的点
        # tgt_cloud = self.tgt_pcd.select_by_index(LB)  # B 中剩余的点

        return src_recloud

    def visualize_point_clouds(self,Acloud, Bcloud, Brecloud):
        vis = o3d.visualization.Visualizer()
        vis.create_window()

        vis.add_geometry(Acloud)
        vis.add_geometry(Bcloud)
        vis.add_geometry(Brecloud)

        vis.run()
        vis.destroy_window()
