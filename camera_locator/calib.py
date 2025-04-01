import sys
sys.path.append("..")
from detect import Capture
from ruamel.yaml import YAML
import threading
import time
import cv2
import numpy as np
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QPixmap, QImage, QTextCursor
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QPushButton, QVBoxLayout, QHBoxLayout, QTextEdit, QGridLayout

cam_name = "new_cam"
color = [(255, 255, 255), (0, 255, 0), (0, 0, 255)]
binocular_camera_cfg_path = "/home/nvidia/RadarWorkspace/code/Radar_Develop/configs/bin_cam_config.yaml"
capture = Capture(binocular_camera_cfg_path,cam_name)

class MyUI(QWidget):
    def __init__(self,img):
        """
        Args:
            state: R/B
            cam_name:
            capturing: 是否进行采集
        """
        super(MyUI,self).__init__()
        binocular_camera_cfg_path = "/home/nvidia/RadarWorkspace/code/Radar_Develop/configs/bin_cam_config.yaml"
        main_cfg_path = "/home/nvidia/RadarWorkspace/code/Radar_Develop/configs/main_config.yaml"
        main_cfg = YAML().load(open(main_cfg_path, encoding='Utf-8', mode='r'))
        self.capture = Capture(binocular_camera_cfg_path,cam_name)
        self.initUI()
        self.state = main_cfg['global']['my_color'] # TODO 记得修改
        self.camera_image = img

    def initUI(self):
        # 左上角部分
        self.left_top_label = QLabel(self)
        self.left_top_label.setFixedSize(1350, 1000)
        self.left_top_label.setStyleSheet("border: 2px solid black;")
        self.left_top_label.mousePressEvent = self.left_top_clicked
        self.image_points = [[(0, 0), (0, 0), (0, 0), (0, 0)], [(0, 0), (0, 0), (0, 0), (0, 0)],
                             [(0, 0), (0, 0), (0, 0), (0, 0)], [(0, 0), (0, 0), (0, 0), (0, 0)]]
        self.map_points = [[(0, 0), (0, 0), (0, 0), (0, 0)], [(0, 0), (0, 0), (0, 0), (0, 0)],
                           [(0, 0), (0, 0), (0, 0), (0, 0)], [(0, 0), (0, 0), (0, 0), (0, 0)]]
        self.image_count = 0
        self.map_count = 0
        # 右上角部分
        self.right_top_label = QLabel(self)
        self.right_top_label.setFixedSize(550, 900)
        self.right_top_label.setStyleSheet("border: 2px solid black;")
        self.right_top_label.mousePressEvent = self.right_top_clicked

        # 左下角部分
        self.left_bottom_text = QTextEdit(self)
        self.left_bottom_text.setFixedSize(300, 60)

        # 右下角部分
        self.button1 = QPushButton('开始标定', self)
        self.button1.setFixedSize(100, 30)
        self.button1.clicked.connect(self.button1_clicked)

        self.button2 = QPushButton('切换高度', self)
        self.button2.setFixedSize(100, 30)
        self.button2.clicked.connect(self.button2_clicked)

        self.button3 = QPushButton('加载坐标', self)
        self.button3.setFixedSize(100, 30)
        self.button3.clicked.connect(self.button3_clicked)

        self.button4 = QPushButton('保存计算', self)
        self.button4.setFixedSize(100, 30)
        self.button4.clicked.connect(self.button4_clicked)
        self.height = 0
        self.T = []
        if self.state == 'Red':
            self.save_path = 'arrays_test_red.npy'
            right_image_path = "map_red.jpg"  # 替换为右边图片的路径
        else:
            self.save_path = 'arrays_test_blue.npy'
            right_image_path = "map_blue.jpg"  # 替换为右边图片的路径

        # _,left_image = self.camera_capture.read()
        left_image = self.camera_image
        right_image = cv2.imread(right_image_path)

        # 记录缩放比例
        self.left_scale_x = left_image.shape[1] / 1350.0
        self.left_scale_y = left_image.shape[0] / 1000.0

        self.right_scale_x = right_image.shape[1] / 550.0
        self.right_scale_y = right_image.shape[0] / 900.0
        left_image = cv2.cvtColor(left_image, cv2.COLOR_BGR2RGB)
        self.left_image = cv2.resize(left_image, (1350, 1000))
        right_image = cv2.cvtColor(right_image, cv2.COLOR_BGR2RGB)
        self.right_image = cv2.resize(right_image, (550, 900))
        # 缩放图像
        self.update_images()

        self.camera_timer = QTimer(self)
        self.camera_timer.timeout.connect(self.update_camera)
        self.camera_timer.start(50)  # 50毫秒更新一次相机
        # 设置按钮样式
        self.set_button_style(self.button1)
        self.set_button_style(self.button2)
        self.set_button_style(self.button3)
        self.set_button_style(self.button4)

        grid_layout = QGridLayout()
        grid_layout.addWidget(self.button1, 0, 0)
        grid_layout.addWidget(self.button2, 0, 1)
        grid_layout.addWidget(self.button3, 1, 0)
        grid_layout.addWidget(self.button4, 1, 1)

        buttons_and_text_widget = QWidget()

        hbox_buttons_and_text = QHBoxLayout(buttons_and_text_widget)
        hbox_buttons_and_text.addLayout(grid_layout)
        hbox_buttons_and_text.addWidget(self.left_bottom_text)

        vbox_left = QVBoxLayout()
        vbox_left.addWidget(self.left_top_label)

        vbox_right = QVBoxLayout()
        vbox_right.addWidget(self.right_top_label)
        vbox_right.addWidget(buttons_and_text_widget)

        hbox = QHBoxLayout()
        hbox.addLayout(vbox_left)
        hbox.addLayout(vbox_right)

        self.setLayout(hbox)
        self.setGeometry(0, 0, 1900, 1000)
        self.setWindowTitle('Calibration UI')
        self.setWindowFlags(Qt.FramelessWindowHint)
        self.show()

    def keyPressEvent(self, event):
        # 按下键盘事件
        if event.key() == Qt.Key_Escape:
            self.close()

    def update_images(self):

        left_pixmap = self.convert_cvimage_to_pixmap(self.left_image)
        self.left_top_label.setPixmap(left_pixmap)


        right_pixmap = self.convert_cvimage_to_pixmap(self.right_image)
        self.right_top_label.setPixmap(right_pixmap)

    def update_camera(self):
        if self.capturing:
            img0 = self.camera_image
            left_image = cv2.cvtColor(img0, cv2.COLOR_BGR2RGB)
            self.left_image = cv2.resize(left_image, (1350, 1000))
            self.update_images()

    def left_top_clicked(self, event):
        # 图像点击事件
        if not self.capturing:
            x = int(event.pos().x() * self.left_scale_x)
            y = int(event.pos().y() * self.left_scale_y)

            self.image_points[self.height][self.image_count % 4] = (x, y)

            cv2.circle(self.left_image, (int(x / self.left_scale_x), int(y / self.left_scale_y)), 4, color[self.height],
                       -1)
            cv2.putText(self.left_image, str(self.image_count % 4),
                        (int(x / self.left_scale_x), int(y / self.left_scale_y)), cv2.FONT_HERSHEY_SIMPLEX, 1,
                        color[self.height], 3)
            self.image_count += 1
            self.update_images()
            self.append_text(f'图像真实点击坐标：({x}, {y})')

    def right_top_clicked(self, event):
        # 地图点击事件
        if not self.capturing:
            x = int(event.pos().x() * self.right_scale_x)
            y = int(event.pos().y() * self.right_scale_y)
            self.map_points[self.height][self.map_count % 4] = (x, y)

            cv2.circle(self.right_image, (int(x / self.right_scale_x), int(y / self.right_scale_y)), 4,
                       color[self.height],
                       -1)
            cv2.putText(self.right_image, str(self.map_count % 4),
                        (int(x / self.right_scale_x), int(y / self.right_scale_y)), cv2.FONT_HERSHEY_SIMPLEX, 1,
                        color[self.height], 2)
            self.map_count += 1
            self.update_images()
            self.append_text(f'地图真实点击坐标：({x}, {y})')

    def button1_clicked(self):
        # 按钮1点击事件
        self.append_text('开始标定')
        self.capturing = False

        print('开始标定')

    def button2_clicked(self):
        # 按钮2点击事件
        self.append_text('切换高度')
        self.image_count = 0
        self.map_count = 0
        self.height = (self.height + 1) % 3
        print('切换高度')

    def button3_clicked(self):
        # 按钮3点击事件
        self.append_text('加载坐标')  # 该功能还未制作

        print('加载坐标')

    def button4_clicked(self):
        # 按钮4点击事件
        print(self.image_points)
        print(self.map_points)
        for i in range(0, 3):
            image_point = np.array(self.image_points[i], dtype=np.float32)
            map_point = np.array(self.map_points[i], dtype=np.float32)
            self.T.append(cv2.getPerspectiveTransform(image_point, map_point))

        np.save(self.save_path, self.T)

        self.append_text('保存计算')
        print('保存计算', self.save_path)
        time.sleep(1)
        sys.exit()

    def convert_cvimage_to_pixmap(self, cvimage):
        height, width, channel = cvimage.shape
        bytes_per_line = 3 * width
        qimage = QImage(cvimage.data, width, height, bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qimage)
        return pixmap

    def set_button_style(self, button):
        button.setStyleSheet("QPushButton { font-size: 18px; }")

    def append_text(self, text):
        # 在文本组件中追加文本
        current_text = self.left_bottom_text.toPlainText()
        self.left_bottom_text.setPlainText(current_text + '\n' + text)
        # 自动向下滚动文本组件
        cursor = self.left_bottom_text.textCursor()
        cursor.movePosition(QTextCursor.End)
        self.left_bottom_text.setTextCursor(cursor)


def main(camera_mode="test"):
 # 'test':测试模式,'camera':海康相机,'video':USB相机（videocapture）
    camera_image = None
    if camera_mode == 'test':
        camera_image = cv2.imread('test_image.jpg')
    elif camera_mode == 'camera':
        # 海康相机图像获取线程
        camera_image = Capture.get_frame()
    elif camera_mode == 'video':
        # USB相机图像获取线程
        pass

    while camera_image is None:
        print("等待图像。。。")
        time.sleep(0.5)
    app = QApplication(sys.argv)
    MyUI(camera_image)
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()

# 老代码
# import math
# import cv2
# import numpy as np
#
# from global_variables import *
#
#
# def get_wld_coord(cam_pos, cam_coord):
#     wld_coord = np.matmul(cam_pos.pose_R, cam_coord)  # np.transpose(tag.pose_R)
#     wld_coord += cam_pos.pose_t
#     return wld_coord
#
#
# def euler2rotation(roll, yaw, pitch):
#     R_z = np.array([[math.cos(roll), -math.sin(roll), 0.],
#                     [math.sin(roll), math.cos(roll), 0.],
#                     [0., 0., 1.]])
#     R_x = np.array([[1., 0., 0.],
#                     [0., math.cos(pitch), -math.sin(pitch)],
#                     [0., math.sin(pitch), math.cos(pitch)]])
#     R_y = np.array([[math.cos(yaw), 0., math.sin(yaw)],
#                     [0., 1., 0.],
#                     [-math.sin(yaw), 0., math.cos(yaw)]])
#     # rotation = np.around(np.matmul(np.matmul(R_z, R_x), R_y), decimals=2)
#     rotation = np.matmul(np.matmul(R_z, R_x), R_y)
#     return rotation
#
#
# def xyz2translation(x, y, z):
#     return np.array([[x],
#                      [y],
#                      [z]])
#
#
# class CameraPoseSolver:
#     def __init__(self, color, left_cam_cfg):
#         self.color = color
#         self.main_cam_info = left_cam_cfg
#         # main pos: left camera in bincam w.r.t bottom left of the field (origin of the map)
#         self.main_pose_R = None
#         self.main_pose_t = None
#         # extrinsic: between the cameras
#         self.R_longfocal2main = None
#         self.R_right2main = None
#         self.R_realsense2main = None
#         # [w, h]: the outer bound
#         self.landmark_size = MySize(660, 495)
#         # temp
#         self.main2field_R = None
#         self.main2field_t = None
#         # anchor or constant
#         self.flag = 0  # constant by default
#         # constants
#         self.cam_pos = None
#         self.sign = None
#         self.cos_arc_pitch = None
#         self.sin_arc_pitch = None
#         self.cos_arc_yaw = None
#         self.sin_arc_yaw = None
#
#     def init_by_anchor(self, anchor):
#         self.flag = 1
#         true_points = None
#         if self.color == RED:
#             # TO-DO: 确定左上角z坐标 (615 为图纸值) -> 615 mm to ground
#             top_left = [8805, 15000 - 5730, 615]
#             top_right = [top_left[0], top_left[1] - self.landmark_size.w, top_left[2]]
#             bottom_right = [top_left[0], top_left[1] - self.landmark_size.w, top_left[2] - self.landmark_size.h]
#             bottom_left = [top_left[0], top_left[1], top_left[2] - self.landmark_size.h]
#
#             true_points = np.array([top_left, top_right, bottom_right, bottom_left], dtype=np.float32)
#         elif self.color == BLUE:
#             top_left = [19195, 15000 - 9270, 615]
#             top_right = [top_left[0], top_left[1] + self.landmark_size.w, top_left[2]]
#             bottom_right = [top_left[0], top_left[1] + self.landmark_size.w, top_left[2] - self.landmark_size.h]
#             bottom_left = [top_left[0], top_left[1], top_left[2] - self.landmark_size.h]
#
#             true_points = np.array([top_left, top_right, bottom_right, bottom_left], dtype=np.float32)
#         else:
#             print('Bad color set')
#
#         print('p3d_np')
#         print(true_points)
#         print('p2d')
#         print(anchor.vertexes)
#         pixel_points = np.array(anchor.vertexes, dtype=np.float32)
#         print('p2d_np')
#         print(pixel_points)
#
#         _, rotation_vector, self.main2field_t = cv2.solvePnP(true_points,
#                                                              pixel_points,
#                                                              np.array(self.main_cam_info['intrinsic']),
#                                                              np.array(self.main_cam_info['distortion'])
#                                                              # or empty np.array(1, 4) and remap before passing
#                                                              )
#
#         self.main_pose_R = cv2.Rodrigues(rotation_vector)[0]
#         self.main2field_R = np.transpose(self.main_pose_R)
#         pass
#
#     def init_by_constant(self, constant_dict):
#         self.flag = 0
#
#         cam_bias = [225., 395., 1399.5]
#         if self.color == BLUE:  # self = BLUE
#             radar_base = [28988.19, 6017.49, 2500]  # blue base
#             self.cam_pos = [radar_base[0] + cam_bias[0], radar_base[1] - cam_bias[1], radar_base[2] + cam_bias[2]]
#             self.sign = -1
#         elif self.color == RED:  # self = RED
#             radar_base = [-987.55, 9018.02, 2500]  # red base
#             self.cam_pos = [radar_base[0] - cam_bias[0], radar_base[1] + cam_bias[1], radar_base[2] + cam_bias[2]]
#             self.sign = 1
#
#         arc_pitch = 2. * np.pi / 180
#         arc_yaw = 11. * np.pi / 180
#
#         self.sin_arc_pitch = np.sin(arc_pitch)
#         self.cos_arc_pitch = np.cos(arc_pitch)
#         self.sin_arc_yaw = np.sin(arc_yaw)
#         self.cos_arc_yaw = np.cos(arc_yaw)
#         pass
#
#     def load_intrinsics(self):
#         pass
#
#     def get_field_coord(self, cam_coord):
#         field_coord = None
#         if self.flag == 0:
#             x = cam_coord[2][0] * self.cos_arc_pitch
#             y = cam_coord[0][0]
#             x_ = x * self.cos_arc_yaw - y * self.sin_arc_yaw
#             y_ = x * self.sin_arc_yaw + y * self.cos_arc_yaw
#             field_coord = [[self.sign * x_ + self.cam_pos[0]],
#                             [-1 * self.sign * y_ + self.cam_pos[1]],
#                             [0.]]
#             pass
#         elif self.flag == 1:
#             print('main2field_R')
#             print(self.main2field_R)
#             print('main2field_t')
#             print(self.main2field_t)
#             print('cam_coord')
#             print(cam_coord)
#
#             field_coord = cam_coord - self.main2field_t
#             field_coord = np.matmul(self.main2field_R, field_coord)
#
#         return field_coord
