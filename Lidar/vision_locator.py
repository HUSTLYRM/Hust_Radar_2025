import yaml
import numpy as np
import cv2


class Vision_Locator:
    def __init__(self, intrinsic_matrix, dist_coeffs, world_rvec, world_tvec, extrinsic_matrix,img = None):
        """
        初始�? Vision_Locator �?
        :param intrinsic_matrix: 相机的内参矩�? (4x4)
        :param extrinsic_matrix: 相机的外参矩�? (4x4)
        """
        self.armor_height = 0.15
        self.K = intrinsic_matrix
        self.dist_coeffs = dist_coeffs

        self.world_rvec = world_rvec
        self.world_tvec = world_tvec

        self.minimap = cv2.imread('/home/nvidia/RadarWorkspace/code/Hust_Radar_2025-main/Lidar/RMUC25_map.jpg')

        self.points_map = {}  # 用于存放区域的信�?
        self.points_map["Center_high"] = Parser_Points("Center_high", intrinsic_matrix, dist_coeffs, world_rvec,
                                                       world_tvec, extrinsic_matrix,img)

        self.points_map["Enemy_Hero_High"] = Parser_Points("Enemy_Hero_High", intrinsic_matrix, dist_coeffs, world_rvec,
                                                           world_tvec,
                                                           extrinsic_matrix,img)
        self.points_map["Self_Hero_High"] = Parser_Points("Self_Hero_High", intrinsic_matrix, dist_coeffs, world_rvec,
                                                          world_tvec, extrinsic_matrix,img)

        self.points_map["Enemy_Left_High"] = Parser_Points("Enemy_Left_High", intrinsic_matrix, dist_coeffs, world_rvec,
                                                           world_tvec, extrinsic_matrix,img)
        self.points_map["Self_Left_High"] = Parser_Points("Self_Left_High", intrinsic_matrix, dist_coeffs, world_rvec,
                                                          world_tvec, extrinsic_matrix,img)

        self.points_map["Enemy_Slope"] = Parser_Points("Enemy_Slope", intrinsic_matrix, dist_coeffs, world_rvec,
                                                       world_tvec, extrinsic_matrix,img)
        self.points_map["Self_Slope"] = Parser_Points("Self_Slope", intrinsic_matrix,
                                                      dist_coeffs, world_rvec, world_tvec,
                                                      extrinsic_matrix,img)

        self.points_map["Enemy_Left_High_Slope"] = Parser_Points("Enemy_Left_High_Slope", intrinsic_matrix,
                                                                 dist_coeffs, world_rvec, world_tvec,
                                                                 extrinsic_matrix,img)

        self.points_map["Enemy_Right_High"] = Parser_Points("Enemy_Right_High", intrinsic_matrix, dist_coeffs,
                                                            world_rvec,
                                                            world_tvec, extrinsic_matrix,img)
        self.points_map["Self_Right_High"] = Parser_Points("Self_Right_High", intrinsic_matrix, dist_coeffs, world_rvec,
                                                           world_tvec, extrinsic_matrix,img)

        self.points_map["Self_Fortress"] = Parser_Points("Self_Fortress", intrinsic_matrix, dist_coeffs,
                                                         world_rvec, world_tvec, extrinsic_matrix,img)
        self.points_map["Enemy_Fortress"] = Parser_Points("Enemy_Fortress", intrinsic_matrix, dist_coeffs, world_rvec,
                                                          world_tvec, extrinsic_matrix,img)

        self.points_map["Self_Narrow_Path"] = Parser_Points("Self_Narrow_Path", intrinsic_matrix, dist_coeffs,
                                                            world_rvec,
                                                            world_tvec, extrinsic_matrix,img)

        self.points_map["Center_high"].heights = 0.3

        self.points_map["Enemy_Slope"].heights = 0.325
        self.points_map["Self_Slope"].heights = 0.325

        self.points_map["Enemy_Left_High_Slope"].heights = 0.08

        self.points_map["Enemy_Fortress"].heights = 0.151
        self.points_map["Self_Fortress"].heights = 0.151

        self.points_map["Enemy_Hero_High"].heights = 0.6
        self.points_map["Self_Hero_High"].heights = 0.6
        #  注意对应关系
        self.points_map["Enemy_Right_High"].heights = 0.2
        self.points_map["Self_Right_High"].heights = 0.2

        self.points_map["Enemy_Left_High"].heights = 0.2
        self.points_map["Self_Left_High"].heights = 0.2

        self.points_map["Self_Narrow_Path"].heights = 0.0

        # 计算并存储透视变换矩阵
        self.h_list = [0.0, 0.08, 0.151, 0.2, 0.325, 0.3, 0.6]
        self.Perspective_matrix = self._calculate_perspective_matrix()
        # print("self_matrix", self.Perspective_matrix)

        # 计算外参矩阵 [R | t]
        self.extrinsic_matrix = extrinsic_matrix

    def _calculate_perspective_matrix(self):
        # 定义世界坐标系中的四个点
        ans = {}
        for h in self.h_list:
            world_points = [
                [12, -6, self.armor_height + h],  # 点1
                [16, -6, self.armor_height + h],  # 点2
                [16, -8, self.armor_height + h],  # 点3
                [12, -8, self.armor_height + h],  # 点4
            ]

            # 将世界坐标投影到图像坐标
            world_points = np.array(world_points, dtype=np.float32)
            image_points, _ = cv2.projectPoints(world_points, self.world_rvec, self.world_tvec, self.K,
                                                self.dist_coeffs)

            # 2D 世界坐标转换为图像坐标（仅考虑平面上的 2D 坐标）
            world_points2D = np.array([
                [12, -6],  # 点1
                [16, -6],  # 点2
                [16, -8],  # 点3
                [12, -8],  # 点4
            ], dtype=np.float32)

            # 获取从图像坐标到世界坐标系的透视变换矩阵
            ans[h] = cv2.getPerspectiveTransform(image_points.reshape(-1, 2), world_points2D)

        return ans

    def get_height(self, input_point):
        # 获取点的高度
        for points in self.points_map.values():
            height = points.return_height(input_point)
            if height > 0:
                return height
        return 0

    def get_2d(self, input_point, height):
        """
        将图像坐标映射到世界坐标系的 2D 坐标�?

        :param input_point: 输入的图像坐标（2D），例如 [x, y]
        :param height: 物体的高度（用于调整 3D 点的高度�?
        :return: 转换后的 2D 世界坐标
        """
        if self.h_list[height] is None:
            # 世界坐标系中的四个点
            world_points = [
                [12, -6, self.armor_height + height],  # �?1
                [16, -6, self.armor_height + height],  # �?2
                [16, -8, self.armor_height + height],  # �?3
                [12, -8, self.armor_height + height],  # �?4
            ]

            # 将世界坐标投影到图像坐标
            world_points = np.array(world_points, dtype=np.float32)
            image_points, _ = cv2.projectPoints(world_points, self.world_rvec, self.world_tvec, self.K,
                                                self.dist_coeffs)
            # print("image_points", image_points)
            # �? 2D 世界坐标转换为图像坐标（仅考虑平面上的 2D 坐标�?
            world_points2D = np.array([
                [12, -6],  # �?1
                [16, -6],  # �?2
                [16, -8],  # �?3
                [12, -8],  # �?4
            ], dtype=np.float32)

            Perspective_matrix = cv2.getPerspectiveTransform(image_points.reshape(-1, 2), world_points2D)

            # 应用透视变换，将输入的图像坐标投影到世界坐标
            src_point_mat = np.array([input_point], dtype=np.float32)
            src_point_mat = src_point_mat.reshape(1, 1, 2)

        else:
            Perspective_matrix = self.Perspective_matrix[height]
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
            x, y, z = xy
            [x, y, z] = [28 - x, 15 - y, z]
        else:
            [x, y, z] = xy
        return [x, y, z]

    def visualize(self, input_points):
        Map_clone = self.minimap.copy()
        for center_point, id in input_points:
            center_point = (center_point[0], center_point[1])  # 调整坐标
            if id > 99:
                # 在地图上绘制蓝色物体的位�?
                cv2.circle(Map_clone, (int((Map_clone.shape[1] * center_point[0]) / 28),
                                       int(Map_clone.shape[0] * (15 - center_point[1]) / 15)),
                           20, (200, 0, 0), -1)  # 绘制�?
                cv2.putText(Map_clone, str(id),
                            (int((Map_clone.shape[1] * center_point[0]) / 28 - 10),
                             int(Map_clone.shape[0] * (15 - center_point[1]) / 15 + 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)  # 标注编号
            else:
                cv2.circle(Map_clone, (int((Map_clone.shape[1] * center_point[0]) / 28),
                                       int(Map_clone.shape[0] * (15 - center_point[1]) / 15)),
                           20, (0, 0, 200), -1)  # 绘制�?
                cv2.putText(Map_clone, str(id),
                            (int((Map_clone.shape[1] * center_point[0]) / 28 - 10),
                             int(Map_clone.shape[0] * (15 - center_point[1]) / 15 + 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)  # 标注编号

        try:
            map_show = cv2.resize(Map_clone, [1000, 640])
            cv2.imshow("location", map_show)
            cv2.waitKey(1)
        except:
            print("error")


class Parser_Points():
    def __init__(self, name, intrinsic_matrix, dist_coeffs, world_rvec, world_tvec, extrinsic_matrix,img = None):
        self.name = name
        self.debug_img = img
        self.points_path = '/home/nvidia/RadarWorkspace/code/Hust_Radar_2025-main/Lidar/rm25_points.yaml'  # TODO
        self.extrinsic_matrix = extrinsic_matrix
        self.K = intrinsic_matrix
        self.dist_coeffs = dist_coeffs
        self.world_rvec = world_rvec
        self.world_tvec = world_tvec
        # 读取3D�?
        self.points_3d = self.read_points(name)
        # 从世界坐标系到相机坐标系
        self.points_2d = self.world_to_camera()
        self.heights = 0.0

    def read_points(self, points_name):
        # 使用 yaml 读取点数�?
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
            # print("points",points)

        return points

    def float2int(self, float_points):
        # 将浮点型�?2D点转化为整型
        return [[int(p[0]), int(p[1])] for p in float_points]

    def world_to_camera(self):
        # 3D世界坐标系点�?2D相机坐标系的投影
        points = np.array(self.points_3d, dtype=np.float64).reshape(-1, 1, 3)
        # print("points", points)
        temp_2d, _ = cv2.projectPoints(
            points,
            self.world_rvec,
            self.world_tvec,
            self.K,
            self.dist_coeffs
        )
        # 转换为整�?

        results = self.float2int(temp_2d.reshape(-1, 2))
        # print("region _results", results)
        # self.region_vis(results)
        return results

    def _ensure_results_in_image(self, results):
        """
        确保结果中的点坐标在图像范围内（width: 1280, height: 640）。
        如果坐标小于0，则设置为0；如果坐标超出图像范围，则设置为边界值。
        """
        width = 1280
        height = 640
        for i in range(len(results)):
            if results[i][0] < 0:
                results[i][0] = 0
            if results[i][0] > width:
                results[i][0] = width
            if results[i][1] < 0:
                results[i][1] = 0
            if results[i][1] > height:
                results[i][1] = height
        return results

    def region_vis(self, region_results):
        '''

        Args:
            region_results:

        Returns:
            visualize results

        '''
        # 将区域画在图片上
        img = self.debug_img.copy()
        polygon = np.array(region_results, dtype=np.int32).reshape(-1, 1, 2)
        # 画点
        cv2.polylines(img, [polygon], True, (0, 0, 255), 2)
        # 调整大小
        img = cv2.resize(img, [1000, 640])
        cv2.imshow(self.name, img)
        # 保存图片 文件名是name
        file_name ="/home/nvidia/RadarWorkspace/code/Hust_Radar_2025-main/debug_img/"+self.name+ "_25" + ".png"
        cv2.imwrite(file_name, img)
        cv2.waitKey(1000)

    def return_height(self, input_point):
        # 判断点是否在多边形内
        inside = False
        # 使用 cv2.pointPolygonTest 判断点是否在多边形内
        # input_point �? cv2.Point2f 类型的点
        if cv2.pointPolygonTest(np.array(self.points_2d, dtype=np.int32),
                                (int(input_point[0][0][0]), int(input_point[0][0][1])),
                                False) > 0:
            # 点在多边形内部，返回高度�?
            return self.heights
        else:
            # 点不在多边形内部，返�? 0
            return 0
