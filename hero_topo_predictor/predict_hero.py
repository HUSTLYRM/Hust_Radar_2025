import sys
import numpy as np
from ruamel.yaml import YAML
import cv2
from collections import deque
def covert_cord(x,y):
    new_x = 28.0 * ( x / 1280)
    new_y = 15.0 * ( y / 679)
    return new_x,new_y


class Area:
    def __init__(self,
                 id,
                 vertices,
                 ):
        self.id = id
        self.color = "Blue " if id <= 4 else "Red"
        self.vertices = vertices
        self.symbol = id % 2
        self.destination =  self.init_destination()

    def init_destination(self):
        if self.symbol == 1 and self.id == 1:
            return [18.0, 4.85]
        elif self.symbol == 1 and self.id == 3:
            return [18.75 , 11.1]
        elif self.symbol == 1 and self.id == 5:
            return [10.0 , 10.15]
        elif self.symbol == 0 and self.id == 7:
            return [9.25 , 3.9]
        else:
            return [0.0 , 0.0]



class Hero_Predict:
    def __init__(self,
                 my_color):
        self.map_cfg_path = './area_data.yaml'
        self.map_cfg = YAML().load(open(self.map_cfg_path, encoding='Utf-8', mode='r'))
        self.my_color = my_color
        self.hero_xy = deque(maxlen = 5) # 记录五个敌方英雄坐标
        self.hero_area = deque(maxlen = 5) # 记录敌方英雄经过区域
        self.area_list = {}
        self.init_area_list()

    def init_area_list(self):
        for area in self.map_cfg["nodes"]:
            vertices = np.array(area['corners'])
            new_vertices = []
            for vertice in vertices:
                new_vertices.append(covert_cord(vertice[0],vertice[1]))
            area = Area(area['id'],new_vertices)
            self.area_list[area.id] = area

    def update_hero(self,hero_xy):
        if hero_xy == []:
            return
        x, y = hero_xy
        self.hero_xy.append([x,y])
        for area in self.area_list.values():
            if cv2.pointPolygonTest(np.array(area.vertices, dtype=np.float32), (x, y), False) >= 0:
                self.hero_area.append(area.id)
                return self.area_list[area.id].destination

















