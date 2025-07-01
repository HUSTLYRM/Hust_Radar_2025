import numpy as np
from ruamel.yaml import YAML
import cv2
from collections import deque


destination_dict = {"start blue": [25.6 , 14.1], "start red": [2.34 , 0.88],"silver left blue": [18.8 , 8.70] , "silver right blue": [18.8 , 6.27] , "blue gold":[14.67 , 6.94] , "red gold":[13.40 , 8.15] , "silver left red": [8.95 , 6.33] , "silver right red": [8.95 , 8.72]}

id_to_str = {1 : "start blue" , 6 : "start red" ,  3 : "silver left blue" , 8 : "silver right red" , 4 : "silver right blue" , 9 : "silver right red" , 5 : "blue gold" , 10 : "red gold"}
# str_to_id = {"start blue" : 1 , "start red" : 6 , "end blue" : 2 , "end red" : 7 , "silver left blue" : 3 , "silver right blue" : 8 , "blue gold" : 4 , "red gold" : 9 , "silver left red" : 5 , "silver right red" : 10}

class Area:
    # 区域数据结构：存储area角点，颜色，序号，标志位信息，根据标志位信息推出目标点
    def __init__(self,
                 id,
                 vertices,
                 symbol
                 ):
        self.id = id
        self.color = "Blue " if id <= 5 else "Red"
        self.vertices = vertices
        self.symbol = symbol
        self.destination = self.process_symbol()


    def process_symbol(self):
        return id_to_str

    def get_destination(self , id):
        # print("id",id)
        # print("self.destination",self.destination)
        if id not in self.destination.keys():
            return None


        return destination_dict[self.destination[id]]


class Predictor:
    def __init__(self,
                 my_color : str,
                 type : str):
        self.map_cfg_path = './engine_area_data.yaml'
        self.map_cfg = YAML().load(open(self.map_cfg_path, encoding='Utf-8', mode='r'))
        self.my_color = my_color
        self.type = type
        self.xy = deque(maxlen = 5) # 记录五个敌方坐标
        self.area_list = {}
        self.init_area_list()

    def init_area_list(self):
        for area in self.map_cfg["nodes"]:
            vertices = np.array(area['corners'])
            print(vertices)
            id = int(area['id'])
            self.area_list[id] = Area(id, np.array(vertices,dtype=np.float32), area['description'])


    def update_cord(self , xy):
        for area in self.area_list.values():
            if area.color == self.my_color:
                continue
            vertices = area.vertices
            vertices_reshaped = vertices.reshape(-1, 1, 2)
            if cv2.pointPolygonTest(vertices_reshaped, xy, False) >= 0:
                self.xy.append([xy, area.id])
                break

    def get_result(self):
        if self.type == 'engine':
            return self.get_engine_result()
        else:
            raise ValueError("type must be 'hero' or 'engine'")

    def get_engine_result(self):
        if not self.xy:  # 更简洁的空列表检查
            print("Warning: self.xy is empty!")
            return None  # 显式返回None

        xy, area_id = self.xy[-1]
        try:
            tmp = self.area_list.get(area_id)
            if area_id == 2 or area_id == 7:
                return [0.0,0.0]
            destination = tmp.get_destination(area_id)
            return destination
        except KeyError:
            print(f"Error: Area ID {area_id} not found in area_list!")
            return None
        except Exception as e:
            print(f"Unexpected error in get_destination: {e}")
            return None




if __name__ == '__main__':
    predictor = Predictor('Blue' , 'engine')
    test_xy = [
        [16.144420131291028, 9.151515151515152],
        [23.86433260393873,10.303030303030303],
        [19.024070021881837,10.606060606060606],
        [16.11378555798687,12.606060606060606]
    ]
    for xy in test_xy:
        x , y = xy
        predictor.update_cord((x,y))
        result = predictor.get_result()
        if result is not None:
            print("result" , predictor.get_result())

