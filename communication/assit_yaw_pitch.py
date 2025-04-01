import math
from random import Random

import numpy as np
import scipy.integrate as integrate
import scipy.optimize as opt


class BallisticTrajectory:
    def __init__(self, hero_xyz, v0, wind_speed=0):
        """
        初始化目标坐标（base）和发射坐标（hero），包括z轴的高度，发射速度v0为常量
        """
        self.base_x = 25.591
        self.base_y = 7.5
        self.base_z = 1.0855
        self.hero_x = hero_xyz[0]
        self.hero_y = hero_xyz[1]
        self.hero_z = hero_xyz[2]
        self.v0 = v0  # 固定发射速度
        self.wind_speed = wind_speed  # 微风影响的速度，暂时没有具体实现

        # 物理常数
        self.g = 9.8  # 重力加速度 (m/s^2)
        self.rho = 1.225  # 空气密度 (kg/m³)
        self.Cd = 0.47  # 空气阻力系数 (假设高尔夫球)
        self.A = 0.003  # 高尔夫球的横截面积 (m²) 约为0.03m直径的球
        self.m = 0.045  # 高尔夫球的质量 (kg) 约为45g

    def equations(self, t, state):
        """
        方程组，描述物体的三维运动，包括x, y, z坐标的变化
        """
        x, y, z, vx, vy, vz = state  # 当前状态：位置x, y, z，速度vx, vy, vz
        v = math.sqrt(vx ** 2 + vy ** 2 + vz ** 2)  # 总速度

        # 计算加速度（空气阻力的影响）
        ax = - (1 / 2) * self.Cd * self.rho * self.A * v * vx / self.m  # 水平方向的加速度
        ay = - (1 / 2) * self.Cd * self.rho * self.A * v * vy / self.m  # 水平方向的加速度
        az = -self.g - (1 / 2) * self.Cd * self.rho * self.A * v * vz / self.m  # 垂直方向的加速度

        return [vx, vy, vz, ax, ay, az]

    def calculate_trajectory(self, theta, phi):
        """
        计算子弹的轨迹，包括水平面（xy）和高度（z），其中发射速度v0为常量
        """
        # 初始条件：位置和速度
        theta_rad = math.radians(theta)  # 发射角度
        phi_rad = math.radians(phi)  # 水平角度

        vx0 = self.v0 * math.cos(theta_rad) * math.cos(phi_rad)  # 初始速度的水平方向分量
        vy0 = self.v0 * math.cos(theta_rad) * math.sin(phi_rad)  # 初始速度的水平方向分量
        vz0 = self.v0 * math.sin(theta_rad)  # 初始速度的垂直方向分量

        # 初始状态 [x, y, z, vx, vy, vz]
        initial_state = [self.hero_x, self.hero_y, self.hero_z, vx0, vy0, vz0]

        # 时间范围：从0到某个足够大的时间
        t_span = (0, 10)  # 10秒内解算
        t_eval = np.linspace(0, 10, 500)  # 计算的时间点

        # 使用数值方法解方程
        solution = integrate.solve_ivp(self.equations, t_span, initial_state, t_eval=t_eval)

        # 获取计算的轨迹
        x_vals = solution.y[0]
        y_vals = solution.y[1]
        z_vals = solution.y[2]

        # 找到与目标x坐标最接近的点
        target_idx = np.argmin(np.abs(x_vals - self.base_x))
        y_at_base = y_vals[target_idx]
        z_at_base = z_vals[target_idx]

        return y_at_base, z_at_base, x_vals, y_vals, z_vals, solution.t

    def objective(self, params):
        """
        优化目标：使得子弹的y位置与base_y相等，z位置与base_z相等
        """
        theta, phi = params
        y_at_base, z_at_base, _, _, _, _ = self.calculate_trajectory(theta, phi)
        return (y_at_base - self.base_y) ** 2 + (z_at_base - self.base_z) ** 2  # 目标是最小化与目标坐标的差距

    def find_optimal_parameters(self, initial_guess=None):
        """
        查找最佳发射角度和水平角度（发射速度v0为常量）
        """
        if initial_guess is None:
            initial_guess = [40, 0]
        result = opt.minimize(self.objective, np.array(initial_guess), bounds=[(20, 70), (0, 360)])

        # 返回最优的发射角度和水平角度
        theta_optimal, phi_optimal = result.x
        return theta_optimal, phi_optimal


# 使用示例
if __name__ == "__main__":
    # 设置目标坐标和发射点坐标，包括z轴
    base_x, base_y, base_z = 20, 0, 0.15  # 目标在20米远，y=0，z=5米
    hero_xyz = [5, 6, 0,15]  # 发射点在原点，高度1米
    random_int = Random(10)
    # 发射速度
    v0 = 16 + 0.1 * random_int.randint(a=-1,b=1)  # 固定发射速度 30 m/s

    # 创建 BallisticTrajectory 实例
    trajectory_solver = BallisticTrajectory(hero_xyz, v0)

    # 查找最佳的发射角度和水平角度
    theta_optimal, phi_optimal = trajectory_solver.find_optimal_parameters()

    # 输出结果
    print(f"Optimal launch angle (theta): {theta_optimal:.2f} degrees")
    print(f"Optimal horizontal angle (phi): {phi_optimal:.2f} degrees")
