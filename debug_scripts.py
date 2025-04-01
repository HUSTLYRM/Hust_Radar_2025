import cv2
import random
import time
from Lidar.Converter import *

if '__main__' == __name__:
    converter = Converter("Blue","C:\Files\E\Radar\Hust-Radar-2024-main\configs\converter_config.yaml")
    img = cv2.imread("C:\Files\E\Radar\debug_img.png")
    # print(time.time())
    converter.camera_to_field_init(img=img)
    # 随机生成一系列点：要求在图像的一半以下

