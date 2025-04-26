import yaml
import numpy as np
import cv2


class Vision_Locator:
    def __init__(self, intrinsic_matrix, dist_coeffs, world_rvec, world_tvec, extrinsic_matrix,img = None):
        """
        初始化 Vision_Locator 类
        :param intrinsic_matrix: 相机的内参矩阵 (4x4)
        :param extrinsic_matrix: 相机的外参矩阵 (4x4)
        """
        self.armor_height = 0.15
        self.h_list = [0.0, 0.2, 0.4, 0.6, 0.8]
        self.perspective_matrix = {}


        self.K = intrinsic_matrix
        self.dist_coeffs = dist_coeffs

        self.world_rvec = world_rvec
        self.world_tvec = world_tvec

        self.compute_persepective_matrix()

        self.minimap = cv2.imread("/home/nvidia/RadarWorkspace/code/Hust-Radar-2024/Lidar/RM2024.png")

        self.points_map = {}  # 用于存放区域的信息
        self.points_map["Middle_Line"] = Parser_Points("Middle_Line", intrinsic_matrix, dist_coeffs, world_rvec,
                                                       world_tvec, extrinsic_matrix,img)
        self.points_map["Left_Road"] = Parser_Points("Left_Road", intrinsic_matrix, dist_coeffs, world_rvec, world_tvec,
                                                     extrinsic_matrix,img)
        self.points_map["Right_Road"] = Parser_Points("Right_Road", intrinsic_matrix, dist_coeffs, world_rvec,
                                                      world_tvec, extrinsic_matrix,img)
        self.points_map["Self_Ring_High"] = Parser_Points("Self_Ring_High", intrinsic_matrix, dist_coeffs, world_rvec,
                                                          world_tvec, extrinsic_matrix,img)
        self.points_map["Enemy_Ring_High"] = Parser_Points("Enemy_Ring_High", intrinsic_matrix, dist_coeffs, world_rvec,
                                                           world_tvec, extrinsic_matrix,img)
        self.points_map["Enemy_Left_High"] = Parser_Points("Enemy_Left_High", intrinsic_matrix, dist_coeffs, world_rvec,
                                                           world_tvec, extrinsic_matrix,img)
        self.points_map["Enemy_Right_High"] = Parser_Points("Enemy_Right_High", intrinsic_matrix, dist_coeffs,
                                                            world_rvec, world_tvec, extrinsic_matrix,img)
        self.points_map["Enemy_Buff"] = Parser_Points("Enemy_Buff", intrinsic_matrix, dist_coeffs, world_rvec,
                                                      world_tvec, extrinsic_matrix,img)

        self.points_map["Middle_Line"].heights = 0.0
        self.points_map["Left_Road"].heights = 0.2
        self.points_map["Right_Road"].heights = 0.2
        self.points_map["Self_Ring_High"].heights = 0.6
        self.points_map["Enemy_Ring_High"].heights = 0.6
        self.points_map["Enemy_Left_High"].heights = 0.4
        self.points_map["Enemy_Right_High"].heights = 0.4
        self.points_map["Enemy_Buff"].heights = 0.8



        # 计算外参矩阵 [R | t]
        self.extrinsic_matrix = extrinsic_matrix

    def compute_persepective_matrix(self):
        for h in self.h_list:
            # 定义世界坐标系中的四个点
            world_points = [
                [12, -6, self.armor_height + h],  # 点1
                [16, -6, self.armor_height + h],  # 点2
                [16, -8, self.armor_height + h],  # 点3
                [12, -8, self.armor_height + h],  # 点4
            ]
            world_points = np.array(world_points, dtype=np.float32)
            image_points, _ = cv2.projectPoints(world_points, self.world_rvec, self.world_tvec, self.K,
                                                self.dist_coeffs)
            # print("image_points", image_points)
            # 将 2D 世界坐标转换为图像坐标（仅考虑平面上的 2D 坐标）
            world_points2D = np.array([
                [12, -6],  # 点1
                [16, -6],  # 点2
                [16, -8],  # 点3
                [12, -8],  # 点4
            ], dtype=np.float32)

            # 获取从图像坐标到世界坐标系的透视变换矩阵
            Perspective_matrix = cv2.getPerspectiveTransform(image_points.reshape(-1, 2), world_points2D)
            self.perspective_matrix[h] = Perspective_matrix


    def get_height(self, input_point):
        # 获取点的高度
        for points in self.points_map.values():
            height = points.return_height(input_point)
            if height > 0:
                return height
        return 0

    def get_2d(self, input_point, height):
        """
        将图像坐标映射到世界坐标系的 2D 坐标。

        :param input_point: 输入的图像坐标（2D），例如 [x, y]
        :param height: 物体的高度（用于调整 3D 点的高度）
        :return: 转换后的 2D 世界坐标
        """
        # 定义世界坐标系中的四个点


        # 获取从图像坐标到世界坐标系的透视变换矩阵
        if self.perspective_matrix[height] is None:
            world_points = [
                [12, -6, self.armor_height + height],  # 点1
                [16, -6, self.armor_height + height],  # 点2
                [16, -8, self.armor_height + height],  # 点3
                [12, -8, self.armor_height + height],  # 点4
            ]

            # 将世界坐标投影到图像坐标
            world_points = np.array(world_points, dtype=np.float32)
            image_points, _ = cv2.projectPoints(world_points, self.world_rvec, self.world_tvec, self.K,
                                                self.dist_coeffs)
            # print("image_points", image_points)
            # 将 2D 世界坐标转换为图像坐标（仅考虑平面上的 2D 坐标）
            world_points2D = np.array([
                [12, -6],  # 点1
                [16, -6],  # 点2
                [16, -8],  # 点3
                [12, -8],  # 点4
            ], dtype=np.float32)
            Perspective_matrix = cv2.getPerspectiveTransform(image_points.reshape(-1, 2), world_points2D)
        else:
            Perspective_matrix = self.perspective_matrix[height]

        # 应用透视变换，将输入的图像坐标投影到世界坐标
        src_point_mat = np.array([input_point], dtype=np.float32)
        src_point_mat = src_point_mat.reshape(1, 1, 2)

        # 进行透视变换
        dst_point_mat = cv2.perspectiveTransform(src_point_mat, Perspective_matrix)
        results = tuple(dst_point_mat[0][0])


        # 返回变换后的 2D 世界坐标
        return results

    def parser(self, xy):
        # TODO
        temp_height = self.get_height(xy)
        if temp_height > 0.79:
            return [19.322, -1.915]
        return self.get_2d(xy, temp_height)

    def post_process(self, xy, color):
        if color == 'Blue':
            # 转换坐标
            x, y, z, _ = xy
            [x, y, z] = [28 - x, 15 - y, z]
        else:
            [x, y, z] = xy
        return [x, y, z]

    def visualize(self, input_points):
        Map_clone = self.minimap.copy()
        for center_point, id in input_points:
            center_point = (center_point[0], center_point[1])  # 调整坐标
            if id > 99:
                # 在地图上绘制蓝色物体的位置
                cv2.circle(Map_clone, (int((Map_clone.shape[1] * center_point[0]) / 28),
                                       int(Map_clone.shape[0] * (15 - center_point[1]) / 15)),
                           20, (200, 0, 0), -1)  # 绘制圆
                cv2.putText(Map_clone, str(id),
                            (int((Map_clone.shape[1] * center_point[0]) / 28 - 10),
                             int(Map_clone.shape[0] * (15 - center_point[1]) / 15 + 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)  # 标注编号
            else:
                cv2.circle(Map_clone, (int((Map_clone.shape[1] * center_point[0]) / 28),
                                       int(Map_clone.shape[0] * (15 - center_point[1]) / 15)),
                           20, (0, 0, 200), -1)  # 绘制圆
                cv2.putText(Map_clone, str(id),
                            (int((Map_clone.shape[1] * center_point[0]) / 28 - 10),
                             int(Map_clone.shape[0] * (15 - center_point[1]) / 15 + 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)  # 标注编号

        try:
            map_show = cv2.resize(Map_clone, [1000, 450])
            cv2.imshow("location", map_show)
            cv2.waitKey(1)
            return map_show
        except:
            print("error")


class Parser_Points():
    def __init__(self, name, intrinsic_matrix, dist_coeffs, world_rvec, world_tvec, extrinsic_matrix,img = None):
        self.points_path = '/home/nvidia/RadarWorkspace/code/Hust-Radar-2024/Lidar/points.yaml'  # TODO
        self.name = name
        self.extrinsic_matrix = extrinsic_matrix
        self.K = intrinsic_matrix
        self.dist_coeffs = dist_coeffs
        self.debug_img = img
        self.world_rvec = world_rvec
        self.world_tvec = world_tvec
        # 读取3D点
        self.points_3d = self.read_points(name)
        # 从世界坐标系到相机坐标系
        self.points_2d = self.world_to_camera()
        self.heights = 0.0


    def read_points(self, points_name):
        # 使用 yaml 读取点数据
        with open(self.points_path, 'r') as file:
            points_data = yaml.safe_load(file)

        # 确保节点存在
        if points_name not in points_data:
            raise Exception(f"{points_name} 节点为空或不存在")

        points = []
        for point in points_data[points_name]:
            x = point['x']
            y = point['y']
            z = point['z']
            points.append((x, y, z))

        return points

    def float2int(self, float_points):
        # 将浮点型的2D点转化为整型
        return [[int(p[0]), int(p[1])] for p in float_points]

    def world_to_camera(self):
        # 3D世界坐标系点到2D相机坐标系的投影
        temp_2d, _ = cv2.projectPoints(
            np.array(self.points_3d, dtype=np.float32),
            self.world_rvec,
            self.world_tvec,
            self.K,
            self.dist_coeffs
        )
        # 转换为整型

        results = self.float2int(temp_2d.reshape(-1, 2))
        # print("region _results", results)
        self.region_vis(results)
        return results

    def region_vis(self,region_results):
        img = self.debug_img.copy()
        polygon = np.array(region_results, np.int32).reshape((-1, 1, 2))
        cv2.polylines(img, [polygon], True, (0, 255, 0), 2)
        img = cv2.resize(img, (1000, 640))
        cv2.imshow(self.name, img)
        file_name ="/home/nvidia/RadarWorkspace/code/Hust_Radar_2025-main/debug_img/"+self.name+ "_24" + ".png"
        cv2.imwrite(file_name, img)
        cv2.waitKey(10000)


    def return_height(self, input_point):
        # 判断点是否在多边形内
        inside = False
        # 使用 cv2.pointPolygonTest 判断点是否在多边形内
        # input_point 是 cv2.Point2f 类型的点
        if cv2.pointPolygonTest(np.array(self.points_2d, dtype=np.int32),
                                (int(input_point[0][0][0]), int(input_point[0][0][1])),
                                False) > 0:
            # 点在多边形内部，返回高度值
            return self.heights
        else:
            # 点不在多边形内部，返回 0
            return 0
