import math

class Hero_Assit():
    def __init__(self,xyz):
        if len(xyz) < 3:
            raise ValueError("xyz列表必须包含至少三个元素")
        self.xyz = xyz
        print(xyz)
        self.initial_velocity = 16
        self.g = 9.81
        self.k1 = 0.41 * 1.169 * (2 * math.pi * 0.02125 * 0.02125) // 2 // 0.041
        self.d = None
        self.base_xyz = [0,0,0]# 待修改
        self.z = xyz[2]
        self.theta = {"yaw":0.0,"pitch":0.0}
    '''    def set_Hero_xyz(self,car_id = 1):
        result = self.car_list.get_car_id(car_id)
        xyz = result.get_field_xyz
        return xyz'''
    def get_d(self):
        self.d =  math.sqrt((self.xyz[0] - self.base_xyz[0]) ** 2 + (self.xyz[1] - self.base_xyz[1]) ** 2 )
        return self.d
    def get_z(self,z):
        self.z = abs(z-self.base_xyz[2])

    def get_theta(self):
        self.theta["yaw"] = math.atan2(self.xyz[1] - self.base_xyz[1], self.xyz[0] - self.base_xyz[0])
        self.theta["pitch"] = math.atan2(self.xyz[2] - self.base_xyz[2], math.sqrt((self.xyz[0] - self.base_xyz[0]) ** 2 + (self.xyz[1] - self.base_xyz[1]) ** 2))
        return self.theta


    def calculate_flight_time(self):
        """
        计算飞行时间 T
        """
        theta_rad = self.theta["pitch"]
        T = (math.exp(self.k1 * self.d / (self.initial_velocity * math.cos(theta_rad))) - 1) / (self.k1 * self.initial_velocity * math.cos(theta_rad))
        return T

    def calculate_vertical_displacement(self, T):
        """
        计算垂直方向位移差 δz
        """
        theta_rad = self.theta["pitch"]
        delta_z = self.z - (self.initial_velocity * math.sin(theta_rad) * T) / math.cos(theta_rad) + (0.5 * self.g * T ** 2) / (
                    math.cos(theta_rad) ** 2)
        return delta_z

    def calculate_derivative_theta(self):
        """
        计算 θ 的导数（用于牛顿法迭代）
        """
        theta_rad = self.theta["pitch"]
        f_prime = -self.d * math.sin(theta_rad) * math.exp(self.k1 * self.d / (self.initial_velocity * math.cos(theta_rad))) / (
                    self.initial_velocity * math.cos(theta_rad) ** 2)
        return f_prime

    def newton_method_for_theta(self, tolerance=1e-5, max_iterations=100):
        """
        使用牛顿法迭代求解发射角度 θ
        """
        theta = self.theta["pitch"]
        for _ in range(max_iterations):
            T = self.calculate_flight_time()
            delta_z = self.calculate_vertical_displacement(T)

            # 检查收敛条件
            if abs(delta_z) < tolerance:
                break

            # 计算导数并更新 θ
            f_prime = self.calculate_derivative_theta()
            theta = theta - delta_z / f_prime

        self.theta["pitch"] = theta  # 更新类属性
        return theta


'''hero = Hero_Assit([2,18,9])
print(hero.get_theta())
'''
'''# 参数设置
target_x, target_y, target_z = 10, 5, 2  # 目标位置 (x, y, z)
turret_x, turret_y, turret_z = 0, 0, 1  # 炮台位置 (x, y, z)
initial_velocity = 20  # 初速度（m/s）
g = 9.81  # 重力加速度（m/s²）
k1 = 0.05  # 空气阻力系数

# 计算水平距离和垂直距离
horizontal_distance = math.sqrt((target_x - turret_x) ** 2 + (target_y - turret_y) ** 2)
vertical_distance = target_z - turret_z

# 使用牛顿法求解发射角度
optimal_theta = newton_method_for_theta(vertical_distance, horizontal_distance, initial_velocity, g, k1)

# 计算偏航角 yaw
yaw = math.degrees(math.atan2(target_y - turret_y, target_x - turret_x))

print(f"Yaw: {yaw:.2f} degrees")
print(f"Pitch (Optimal Angle): {optimal_theta:.2f} degrees")'''
