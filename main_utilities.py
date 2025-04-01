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


def get_new_box(xywh_box):
    div_times = 4
    new_w = xywh_box[2] / div_times
    new_h = xywh_box[3] / div_times
    # Generate new bounding box
    new_xywh_box = [xywh_box[0], xywh_box[1] + new_h, new_w, new_h]
    return new_xywh_box
