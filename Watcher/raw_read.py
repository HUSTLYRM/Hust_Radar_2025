import time

import my_serial as messager


def recv(serial, len):
    while True:
        data = serial.read(len)
        if data == '':
            time.sleep(0.01)
            continue
        else:
            break
    return data


def reader(ser):
    header = messager.get_frame_header()
    msg_id = recv(ser, 5)

