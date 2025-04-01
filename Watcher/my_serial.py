# -*- encoding:utf-8 -*-
import serial
import serial.tools.list_ports
# import libscrc
import struct
import time
import random

CRC8_TABLE = [
    0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
    0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
    0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
    0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
    0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
    0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
    0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
    0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
    0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
    0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
    0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
    0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
    0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
    0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
    0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
    0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
]

CRC16_TABLE = [
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
    0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
    0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
    0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
    0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
    0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
    0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
    0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
    0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
    0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
    0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
    0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
    0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
    0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
    0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
    0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
    0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
    0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
    0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
    0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
    0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
    0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
    0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
    0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
    0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
    0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
    0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
    0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
    0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
    0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
    0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78,
]


# 1个byte为两位十六进制数
# frame_header:SOF + data_length + seq + CRC8
# SOF:0xA5，1byte
# data_length:15:0x0F
# seq:包序号，递增，1byte(0-255)
# CRC8，1byte

# cmd_id
# 0301:绘图
# 0302:自定义
# 0303:小地图

# CRC16  (get frame_tail)
def get_crc16_check_byte(data):
    crc = 0xffff
    for byte in data:
        crc = ((crc >> 8) ^ CRC16_TABLE[(crc ^ byte & 0xff) & 0xff])
    return crc


def get_crc8_check_byte(data):
    crc = 0xff
    for byte in data:
        crc_index = crc ^ byte
        crc = CRC8_TABLE[crc_index]
    return crc


# 构建帧头
SOF = b'\xA5'
seq = 0         # 目前均为单包数据，且无重发机制?
alarm_layer = 5
car2client = {1: b'\x01\x01',
              2: b'\x02\x01',
              3: b'\x03\x01',
              4: b'\x04\x01',
              5: b'\x05\x01',
              101: b'\x65\x01',
              102: b'\x66\x01',
              103: b'\x67\x01',
              104: b'\x68\x01',
              105: b'\x69\x01'
              }


# 串口初始化
# bytesize=8
# stopbits=1
# 通信方式是串口，配置为波特率 115200，8 位数据位，1 位停止位，无硬件流控，无校验位。
# 默认8位数据，1位停止位，无硬件流控，无校验位。故不设定
# 超时设置：timex，None：永远等待操作，0为立即返回请求结果，其他值为等待超时时间(单位为秒）
# portx：端口
def serial_init(port):
    # portx='/dev/ttyUSB0'
    port_list = list(serial.tools.list_ports.comports())

    if len(port_list) == 0:
        print('无可用串口')
    else:
        for i in range(0, len(port_list)):
            print(port_list[i])

    bps = 115200
    timex = 0.5
    ser = serial.Serial(port, bps, timeout=timex)

    return ser


def get_frame_header(data_length=14):
    # frame header
    # +--------+--------------+--------+--------+
    # | SOF    | data_length  | seq    | CRC8   |
    # +--------+--------------+--------+--------+
    # | 1-byte | 2-byte       | 1-byte | 1-byte |
    # +--------+--------------+--------+--------+
    #
    # SOF: start of frame, a fixed byte at the beginnig of frame header
    #      the value is 0xA5 in v1.4 protocol
    #      单字节，接收的数据应为 A5
    #
    # data_length: 不包含 cmd_id 和 frame_tail
    #              (construct of a frame:
    #                   [ frame_head  | cmd_id  | data    | frame_tail ]
    #                     5-byte        2-byte    n-byte    2-byte
    #              )
    #              双字节，以data_length=14(10)为例，接收时表现为 0E 00，低字节在前
    #
    # seq: packet sequence number
    #      not used now?
    #
    # struct.py
    # https://docs.python.org/3.8/library/struct.html#struct-format-strings
    # format: struct member type -> size
    # 'H': unsigned short -> 2 bytes
    # 'b': short -> 2 bytes
    # 'B': unsigned char -> 1 byte
    # 'f': float -> 4 bytes
    # 'fff' or '3f' means continuous 3 float values
    # 'I': unsigned int -> 4 bytes

    global SOF, seq
    _frame_header = SOF + \
                    struct.pack('H', data_length) + \
                    struct.pack('B', seq)
    frame_header = _frame_header + struct.pack('B', get_crc8_check_byte(_frame_header))
    return frame_header


def send_enemy_location(ser, carID, x, y):
    tx_buff = get_frame_header(10)
    tx_buff += b'\x05\x03' + \
               struct.pack('h', carID) + struct.pack('ff', x, y)
    # @2022 有帖子提到, 当前的串口通信保留了当时的角度信息, 故需预留一个4bytes的0
    # + struct.pack('fff', X, Y, 0.0)

    CRC16 = get_crc16_check_byte(tx_buff)
    frame_tail = bytes([CRC16 & 0x00ff, (CRC16 & 0xff00) >> 8])
    tx_buff += frame_tail
    ser.write(tx_buff)

    return tx_buff


def send_alarm(ser, mode, srcID, graph_num, msg=''):
    rcvID = car2client[srcID]
    # graph_ctrl
    if mode == 'delete':
        # 删除模式中, 仅需匹配:data 段 cmd_id、operate(3)、layer、graph_name
        data = b'\x01\x01'
        # sender ID
        data += struct.pack('H', srcID)
        # rcv ID
        data += rcvID

        # graph
        operate = 3
        layer = alarm_layer << 6
        opt_1 = layer + operate
        opt_2 = 0
        radius = 0
        length = 6 + 15
        # graph_name
        graph = struct.pack('BBB', graph_num, 0, 0)
        graph += struct.pack('I', opt_1)
        graph += struct.pack('I', opt_2)
        graph += struct.pack('I', radius)

        data += graph
    elif mode == 'circle':
        data = b'\x01\x01'
        # sender ID
        data += struct.pack('H', srcID)
        # rcv ID
        data += rcvID

        # graph
        operate = 1
        graph_class = 2 << 3
        layer = alarm_layer << 6
        color = 4 << 10
        angle = 0 << 14
        opt_1 = angle + color + layer + graph_class + operate

        # 图形为圆时, width为圆环的厚度, 不能超过半径
        width = 15
        x = 192 << 10
        y = 800 << 21
        opt_2 = y + x + width
        radius = 60
        length = 6 + 15

        # graph_name
        graph = struct.pack('BBB', graph_num, 0, 0)
        graph += struct.pack('I', opt_1)
        graph += struct.pack('I', opt_2)
        graph += struct.pack('I', radius)

        data += graph
    elif mode == 'char':
        '''warn'''
        data = b'\x10\x01'
        # sender ID
        data += struct.pack('H', srcID)
        # rcv ID
        data += rcvID

        # graph
        operate = 1
        graph_class = 7 << 3
        layer = alarm_layer << 6
        color = 0 << 10
        size = 20 << 14
        char_len = 5 << 23
        opt_1 = char_len + size + color + layer + graph_class + operate

        width = 4
        x = 153 << 10
        y = 812 << 21
        opt_2 = y + x + width
        radius = 0
        length = 6 + 15 + 30
        string = 'warning!'

        if len(string) < 30:
            string = string + ''.join(['\0' for i in range(30 - len(string))])

        char = bytes()
        for c in string:
            char = char + struct.pack('B', ord(c))

        # graph_name
        graph = struct.pack('BBB', graph_num, 0, 0)
        graph += struct.pack('I', opt_1)
        graph += struct.pack('I', opt_2)
        graph += struct.pack('I', radius)
        graph += char
        data += graph

    tx_buff = get_frame_header(data_length=length) + \
              b'\x01\x03' + \
              data
    # frame_tail
    CRC16 = get_crc16_check_byte(tx_buff)
    frame_tail = bytes([CRC16 & 0x00ff, (CRC16 & 0xff00) >> 8])
    tx_buff += frame_tail

    ser.write(tx_buff)
    return tx_buff


def send_random(ser, label, seed, b):
    tx_buff = get_frame_header()
    tx_buff += b'\x05\x03' + \
               struct.pack('h', label) + struct.pack('ff', seed, b)

    CRC16 = get_crc16_check_byte(tx_buff)
    frame_tail = bytes([CRC16 & 0x00ff, (CRC16 & 0xff00) >> 8])
    tx_buff += frame_tail
    ser.write(tx_buff)

    return tx_buff


def send_guard_position(ser, radar_id, guard_id, x, y, ):
    tx_buff = get_frame_header(data_length=6+8) + \
              b'\x01\x03' + \
              b'\x00\x02' + \
              struct.pack('H', radar_id) + \
              struct.pack('H', guard_id) + \
              struct.pack('f', x) + \
              struct.pack('f', y)

    # frame_tail
    CRC16 = get_crc16_check_byte(tx_buff)
    frame_tail = bytes([CRC16 & 0x00ff, (CRC16 & 0xff00) >> 8])
    tx_buff += frame_tail

    ser.write(tx_buff)
    return tx_buff


portx = 'COM8'
# can use visual serial port to test

if __name__ == '__main__':

    serial = serial_init(portx)

    target_robot_ID_list = [
        # 1,  # 红英雄
        # 2,  # 红工程
        # 3,  # 红步兵3
        # 4,  # 红步兵4
        # 5,  # 红步兵5

        101,  # 蓝英雄
        # 102,# 蓝工程
        # 103,# 蓝步兵3
        # 104,# 蓝步兵4
        # 105 # 蓝步兵5
    ]

    target_position_x_max = 15
    target_position_y_max = 15
    count = 0

    while (1):
        start = time.time()
        target_robot_ID = random.choice(target_robot_ID_list)
        target_position_x = 27.2  # np.random.rand() * 20
        target_position_y = 14.1  # np.random.rand() * 10
        print(count)
        time.sleep(0.4)
        # tx_buff = SendMapData(ser,target_robot_ID,target_position_x+0.5,target_position_y)
        time.sleep(0.1)
        # tx_buff = SendMapData(ser,target_robot_ID+1,target_position_x+0.1,target_position_y)
        # time.sleep(0.1)
        # tx_buff = SendMapData(ser,target_robot_ID+2,target_position_x+0.2,target_position_y)
        # time.sleep(0.1)
        tx_buff = send_enemy_location(serial, target_robot_ID + 3, target_position_x + 0.1, target_position_y)
        time.sleep(0.1)
        tx_buff = send_enemy_location(serial, target_robot_ID + 4, target_position_x, target_position_y)
        time.sleep(0.1)
        for i in tx_buff:
            print(hex(i), i, end='\n')
        # for i in tx_buff:
        #     print(i,end=' ')
        # print("\ntarget_robot_ID:{}\ntarget_position_X:{}\ntarget_position_Y:{}".format(target_robot_ID,target_position_x,target_position_y))

        # ser.write(tx_buff)
        time.sleep(0.1)

        end = time.time()
        sendtime = end - start
        print('Send time:', sendtime)

        robot = 2

        if (count // 5) % 2:
            Mode = random.choice(['char', 'circle'])
            tx_buff = send_alarm(serial, 'char', robot, 255)
            time.sleep(0.01)
            tx_buff = send_alarm(serial, 'circle', robot, 254)
        else:
            Mode = 'delete'

            tx_buff = send_alarm(serial, Mode, robot, 255)
            time.sleep(0.01)
            tx_buff = send_alarm(serial, Mode, robot, 254)

        tx_buff = send_enemy_location(serial, target_robot_ID, target_position_x, target_position_y)
        for i in tx_buff:
            print(hex(i), i, end='\n')

        count += 1
        if count > 200:
            break
        # time.sleep(0.2)
    # print("串口详情参数：", ser)

    # #十六进制的发送
    # result=ser.write(chr(0x06).encode("utf-8"))#写数据
    # print("写总字节数:",result)

    # #十六进制的读取
    # print(ser.read().hex())#读一个字节

    # print("---------------")

    serial.close()  # 关闭串口
