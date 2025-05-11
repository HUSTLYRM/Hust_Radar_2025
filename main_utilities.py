import cv2
import numpy as np


# ... existing code ...

def visualize(input_points):
    # Read the map image
    Map_clone = cv2.imread('/home/nvidia/RadarWorkspace/code/Hust-Radar-2024/camera_locator/map.jpg')

    for center_point in input_points:
        center_point = (center_point[0], center_point[1])  # Convert to tuple for coordinates

        # Draw a red circle on the map at the given coordinates
        cv2.circle(Map_clone,
                   (int((Map_clone.shape[1] * center_point[0]) / 28),
                    int(Map_clone.shape[0] * (15 - center_point[1]) / 15)),
                   20, (200, 0, 0), -1)  # Draw filled circle

        # Write the ID near the point
        cv2.putText(Map_clone, str(id),
                    (int((Map_clone.shape[1] * center_point[0]) / 28 - 10),
                     int(Map_clone.shape[0] * (15 - center_point[1]) / 15 + 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)  # Write ID text

    try:
        # Resize the map and display it
        map_show = cv2.resize(Map_clone, [1000, 640])
        cv2.imshow("location", map_show)
        cv2.waitKey(1)
    except:
        print("Error")


def get_new_box(xyxy,xywh):
    '''

    Args:
        box: XYXY

    Returns:NEW BOX [x1,y1,x2,y2]

    '''
    x1,y1,x2,y2 = xyxy
    x ,y ,w1,h1 = xywh
    # 找到bbox中心最下方的点
    new_x = x+w1/2
    new_y = y2
    # 如果（x，y）位于图像下半部分
    if y > 3036 / 2:
        # 计算新的x1和y1
        w = x2-x1
        h = y2-y1
        new_x1 = x1+w/2
        new_y1 = y1+h/2+h/3
        new_w = w
        new_h = h

    else:
        w = x2-x1
        h = y2-y1
        new_x1 = x1+w/2
        new_y1 = y1+h/2+h/7
        new_w = w
        new_h = h
    return [new_x,new_y,new_x1,new_y1]
