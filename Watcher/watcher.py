import os
import struct
import sys
import time

import numpy as np
import pygame
from threading import Thread
from threading import Lock
import my_serial as messager
from raw_read import reader


class Draw:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.ttf_abs = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'resource', 'IPix.ttf')

    def drawText(self, window, text, posx, posy, textHeight=15, fontColor=(0, 0, 0), backgroudColor=(255, 255, 255)):
        fontObj = pygame.font.Font(self.ttf_abs, textHeight)  # 通过字体文件获得字体对象
        textSurfaceObj = fontObj.render(text, True, fontColor, backgroudColor)  # 配置要显示的文字
        textRectObj = textSurfaceObj.get_rect()  # 获得要显示的对象的rect
        textRectObj.center = (posx, posy)  # 设置显示对象的坐标
        window.blit(textSurfaceObj, textRectObj)  # 绘制字

    def draw_axis(self, window):
        color = 0, 0, 0
        width = 2
        # 画坐标系
        pygame.draw.line(window, color, (0, 0), (self.width, 0), width)
        pygame.draw.line(window, color, (0, 0), (0, self.height), width)
        pygame.draw.line(window, color, (self.width, 0), (self.width - 10, 10), width)
        pygame.draw.line(window, color, (0, self.height), (10, self.height - 10), width)
        x_list = []
        y_list = []
        self.drawText(window, str(0), 10, 10, textHeight=12)
        self.drawText(window, str(self.width), self.width - 10, 14, textHeight=12)
        self.drawText(window, str(self.height), 15, self.height - 10, textHeight=12)
        n1 = 15  # 轴分为n段
        n2 = 6
        for i in range(1, n1):
            x_list.append(int(self.width / n1 * i))
        for i in range(1, n2):
            y_list.append(int(self.height / n2 * i))
        for x_item in x_list:
            pygame.draw.line(window, color, (x_item, 0), (x_item, 10), width)
            self.drawText(window, str(x_item), x_item, 12, textHeight=15)
        for y_item in y_list:
            pygame.draw.line(window, color, (0, y_item), (10, y_item), width)
            self.drawText(window, str(y_item), 16, y_item, textHeight=14)


class Car(pygame.sprite.Sprite):
    def __init__(self, car_color, color, num, width=30, height=30):
        pygame.sprite.Sprite.__init__(self)
        self.image = pygame.Surface((width, height))
        self.image.fill(color)
        self.font = pygame.font.SysFont("Arial", 18, True)
        self.textSurf = self.font.render(str(num), True, [255, 255, 255])
        w = self.textSurf.get_width()
        h = self.textSurf.get_height()
        self.image.blit(self.textSurf, [width/2 - w/2, height/2 - h/2])
        self.rect = self.image.get_rect()
        self.rect.center = (-15, -15)

        self.life = 4

        self.num = num
        self.next_x = 0
        self.next_y = 0

        self.lock = Lock()

    def update(self):
        self.lock.acquire()
        self.rect.center = (self.next_x, self.next_y)
        self.life -= 1
        if self.life == 0:
            self.rect.center = (0, 0)
        self.lock.release()

    def move_to(self, x, y):
        self.lock.acquire()
        self.next_x = x
        self.next_y = y
        self.life = 4
        self.lock.release()


class Scene:
    def __init__(self, width, height):
        pygame.init()
        pygame.display.set_caption('visualize')
        self.width = width
        self.height = height
        self.window = pygame.display.set_mode([self.width, self.height])
        self.bg = pygame.image.load("./bg.png")

        self.bg = pygame.transform.smoothscale(self.bg, self.window.get_size())

        self.draw = Draw(width, height)
        self.cam = (0, 0)
        self.red_cars = pygame.sprite.Group()
        self.blue_cars = pygame.sprite.Group()
        for i in range(5):
            self.red_cars.add(Car(0, [255, 0, 0], i+1))
            self.blue_cars.add(Car(1, [0, 0, 255], i+1))

        self.portx = 'COM8'
        self.ser = messager.serial_init(self.portx)
        self.reader = Thread(target=SerialReceive, kwargs={}, name='Serial_Read_Thread')
        self.exit_signal = False

    def run(self):
        self.reader.start()
        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.exit_signal = True
                    break
            # self.window.fill([255, 255, 255])
            self.window.blit(self.bg, (0, 0))
            self.draw.draw_axis(self.window)
            pygame.draw.rect(self.window, [255, 0, 0], [self.cam[0], self.cam[1] - 8, 8, 16], 3)
            self.draw.drawText(self.window, 'camera', self.cam[0] + 38, self.cam[1] + 20)
            self.red_cars.update()
            self.blue_cars.update()
            self.red_cars.draw(self.window)
            self.blue_cars.draw(self.window)
            pygame.display.update()


def find_0xa5(_ser):
    global scene
    last_bytes = 0
    while not scene.exit_signal:
        current_bytes = _ser.in_waiting
        if last_bytes - current_bytes < -30:
            length = 30
            _info = _ser.read(length)

            for _index, number in enumerate(_info):
                if number == 165:
                    if _index + 4 < length:
                        print('ok', _index, length)
                        _data_length = _info[_index + 1] + _info[_index + 2] * 256
                        seq = _info[_index + 3]
                        CRC8 = _info[_index + 4]

                        _header = messager.struct.pack('B', 165) + \
                                  messager.struct.pack('H', _data_length) + \
                                  messager.struct.pack('B', seq)

                        crc8 = messager.get_crc8_check_byte(_header)
                        if CRC8 == crc8:
                            print('success!')
                            return _index, _info[_index:]
            last_bytes = current_bytes


def recv(serial, len):
    while True:
        data = serial.read(len)
        if data == '':

            continue
        else:
            break
    # time.sleep(0.01)
    return data


def SerialReceive():
    global scene
    blood_temp = [0 for _ in range(16)]
    blood_max = [0 for _ in range(16)]

    current_time_stamp = 420

    visible_list_total = np.zeros((5, 7), dtype=np.int8)

    index, info = find_0xa5(scene.ser)
    data_length = info[1] + info[2] * 256

    # 保证 read 同步至一个帧尾
    while not scene.exit_signal:
        if data_length + 9 - len(info) > 0:  # 未读完当前段
            scene.ser.read(data_length + 9 - len(info))
            break
        else:  # 当前段全部位于info中
            info = info[data_length + 9:]
            if len(info) >= 3:
                data_length = info[1] + info[2] * 256
            else:
                scene.ser.read()

    while not scene.exit_signal:
        header = recv(scene.ser, 5)

        if len(header) == 5:
            data_length = header[0] + header[1] * 256
            frame_length = 2 + data_length + 2
            frame_body = scene.ser.read(frame_length)
            if frame_length > 4:
                cmd_id = frame_body[0] + frame_body[1] * 256
                # 0503
                if cmd_id == 773:
                    num_in_bytes = frame_body[2:4]
                    x_in_bytes = frame_body[4:8]
                    y_in_bytes = frame_body[8:12]
                    num = struct.unpack('h', num_in_bytes)
                    x = struct.unpack('f', x_in_bytes)
                    y = struct.unpack('f', y_in_bytes)
                    # print(int(num[0] - 101))

                    x = int(x[0] * 1412 / 28000 * 1000)
                    y = 757 - int(y[0] * 757 / 15000 * 1000)
                    print(str(num[0])+' x: '+str(x)+' y: '+str(y))
                    if 1 <= num[0] <= 5:
                        scene.red_cars.sprites()[int(num[0] - 1)].move_to(x, y)
                    elif 101 <= num[0] <= 105:
                        scene.blue_cars.sprites()[int(num[0]-101)].move_to(x, y)
                # 0201/0202/0203
        else:
            # 保证 read 同步至一个帧尾
            while not scene.exit_signal:
                if data_length + 9 - len(info) > 0:  # 未读完当前段
                    scene.ser.read(data_length + 9 - len(info))
                    break
                else:  # 当前段全部位于info中
                    info = info[data_length + 9:]
                    if len(info) >= 3:
                        data_length = info[1] + info[2] * 256
                    else:
                        scene.ser.read()

    print('Serial Receive Thread Exit!')


scene = Scene(1412, 757)  # Scene(1200, 600)
if __name__ == '__main__':
    scene.run()



