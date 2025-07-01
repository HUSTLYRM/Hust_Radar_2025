
import numpy as np
from ruamel.yaml import YAML
import math
import cv2
from collections import deque
def covert_cord(x,y):
    new_x = 28.0 * ( x / 1280)
    new_y = 15 - 15.0 * ( y / 679)
    return new_x,new_y

destination_dict = {1 : [18.0, 4.85] , 3 : [[18.75 , 11.1]] , 4 : [23.14 , 3.08] , 5 : [10.0 , 10.15] , 7 : [9.25 , 3.9] , 8 : [4.86 , 11.92]}
engine_dict = {1 : [2.34 , 0.88] , 2 : [25.6 , 14.1]}


def calculate_distances(x, y, corners):
    """计算点(x, y)到所有角点的距离"""
    distances = []
    for corner in corners:
        corner_x, corner_y = corner
        # 计算欧氏距离
        distance = math.sqrt((x - corner_x) ** 2 + (y - corner_y) ** 2)
        distances.append((corner, distance))
    return distances


def find_nearest_farthest(x, y, corners):
    """找出最近和最远的角点"""
    if not corners:
        return None, None

    distances = calculate_distances(x, y, corners)

    # 按距离排序
    sorted_distances = sorted(distances, key=lambda item: item[1])

    nearest = sorted_distances[0][0]  # 最近的角点
    farthest = sorted_distances[-1][0]  # 最远的角点

    return nearest, farthest

class Area:
    # 区域数据结构：存储area角点，颜色，序号，标志位信息，根据标志位信息推出目标点
    def __init__(self,
                 id,
                 vertices,
                 symbol
                 ):
        self.id = id
        self.color = "Blue " if id <= 4 else "Red"
        self.vertices = vertices
        self.symbol = symbol
        self.destination = self.process_symbol()


    def process_symbol(self):
        if self.symbol == 'hero shooting':
            return destination_dict
        if self.symbol == 'engine exchange':
            return engine_dict

    def get_destination(self , id):
        # print("id",id)
        if self.symbol == 'hero shooting':
            if not self.destination.get(id):
                raise ValueError("当symbol为'hero'时，id必须是1到7之间的数")
            return
        elif self.symbol == 'engine exchange':
            if not self.destination.get(id):
                raise ValueError("当symbol为'engine'时，id必须是2到2之间的数")
            return
        print(self.destination)
        return self.destination[id]


class Predictor:
    def __init__(self,
                 my_color : str,
                 type : str):
        self.map_cfg_path = './area_data.yaml'
        self.map_cfg = YAML().load(open(self.map_cfg_path, encoding='Utf-8', mode='r'))
        self.my_color = my_color
        self.type = type
        self.xy = deque(maxlen = 5) # 记录五个敌方坐标

        self.area_list = {}
        self.init_area_list()

    def init_area_list(self):
        for area in self.map_cfg["nodes"]:
            vertices = np.array(area['corners'])
            new_vertices = []
            for vertice in vertices:
                new_vertices.append(covert_cord(vertice[0],vertice[1]))
            self.area_list[area['id']] = Area(area['id'], np.array(new_vertices,dtype=np.float32), area['symbol'])


    def update_cord(self , xy):
        for area in self.area_list.values():
            if area.color == self.my_color:
                continue
            vertices = area.vertices
            vertices_reshaped = vertices.reshape(-1, 1, 2)
            if cv2.pointPolygonTest(vertices_reshaped, (xy[0], xy[1]), False) >= 0:
                self.xy.append([xy, area.id])

                break

    def get_result(self):
        if self.type == 'hero':
            return self.get_hero_result()
        elif self.type == 'engine':
            return self.get_engine_result()
        else:
            raise ValueError("type must be 'hero' or 'engine'")

    def get_hero_result(self):
        if not self.xy:  # 更简洁的空列表检查
            print("Warning: self.xy is empty!")
            return None  # 显式返回None

        xy, area_id = self.xy[-1]
        try:
            tmp = self.area_list.get(area_id)
            if area_id == 2 or area_id == 6:
                return [0.0,0.0]
            destination = tmp.destination.get(area_id)
            return destination
        except KeyError:
            print(f"Error: Area ID {area_id} not found in area_list!")
            return None
        except Exception as e:
            print(f"Unexpected error in get_destination: {e}")
            return None

    def get_engine_result(self):
        if not self.xy:  # 更简洁的空列表检查
            print("Warning: self.xy is empty!")
            return None  # 显式返回None

        xy, area_id = self.xy[-1]
        try:
            if area_id == 2 :
                tmp = self.area_list.get(area_id)
                destination = tmp.destination.get(2)
                return destination
            elif area_id == 6:
                tmp = self.area_list.get(area_id)
                destination = tmp.destination.get(1)
                return destination
            else:
                return [0.0,0.0]
        except KeyError:
            print(f"Error: Area ID {area_id} not found in area_list!")
            return None
        except Exception as e:
            print(f"Unexpected error in get_destination: {e}")
            return None


if __name__ == '__main__':
    predictor = Predictor('Blue' , 'hero')
    for i in range(10):
        x , y = [20.79 , 9.91]
        predictor.update_cord((x,y))
        result = predictor.get_result()
        if result is not None:
            print("result" , predictor.get_result())






















