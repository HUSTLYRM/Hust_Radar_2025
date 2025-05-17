import numpy as np
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise


class CarKalmanPredictor:
    """
    一个基于卡尔曼滤波器的小车位置预测器。
    它假设一个近似恒定速度(Constant Velocity, CV)模型，
    并通过测量(x, y)坐标来更新状态，并预测未来的(x, y)坐标。
    """

    def __init__(self, initial_x: float, initial_y: float,
                 dt: float = 1.0,
                 process_noise_std: float = 0.5,
                 measurement_noise_std: float = 2.0):
        """
        初始化预测器。

        参数:
        - initial_x (float): 小车初始的x坐标。
        - initial_y (float): 小车初始的y坐标。
        - dt (float): 时间步长 (例如，两次测量之间的时间间隔，单位：秒)。
        - process_noise_std (float): 过程噪声的标准差。
          这反映了模型(恒定速度)与真实运动之间的不确定性（例如，加速度）。
          值越大，滤波器对模型依赖越小，对测量依赖越大。
        - measurement_noise_std (float): 测量噪声的标准差。
          这反映了(x, y)坐标测量本身的不确定性。
          值越大，滤波器对测量依赖越小，对模型依赖越大。
        """
        self.dt = dt

        # 状态向量 [x, y, vx, vy]' (位置x, 位置y, 速度x, 速度y)
        # dim_x: 状态向量的维度
        # dim_z: 测量向量的维度 (我们只测量x, y)
        self.kf = KalmanFilter(dim_x=4, dim_z=2)

        # 1. 初始状态 (x)
        # 假设初始速度为0，滤波器会逐渐学习速度
        self.kf.x = np.array([[initial_x], [initial_y], [0.], [0.]])

        # 2. 状态转移矩阵 (F)
        # x_k = F * x_{k-1}
        # x = x_prev + vx * dt
        # y = y_prev + vy * dt
        # vx = vx_prev
        # vy = vy_prev
        self.kf.F = np.array([[1, 0, dt, 0],
                              [0, 1, 0, dt],
                              [0, 0, 1, 0],
                              [0, 0, 0, 1]])

        # 3. 测量函数/矩阵 (H)
        # z = H * x
        # 我们只测量位置x和y
        self.kf.H = np.array([[1, 0, 0, 0],
                              [0, 1, 0, 0]])

        # 4. 初始状态协方差矩阵 (P)
        # 表示初始状态估计的不确定性。
        # 对角线元素较大表示初始估计不确定性高。
        # 速度的初始不确定性可以设得比位置高，因为我们假设初始速度为0。
        self.kf.P = np.diag([measurement_noise_std ** 2, measurement_noise_std ** 2, 100., 100.])

        # 5. 测量噪声协方差矩阵 (R)
        # 表示测量值的不确定性。
        self.kf.R = np.diag([measurement_noise_std ** 2, measurement_noise_std ** 2])

        # 6. 过程噪声协方差矩阵 (Q)
        # 表示模型本身的不确定性（例如，小车不是严格的恒定速度运动）。
        # Q_discrete_white_noise 假设噪声是离散的白噪声，作用于最高阶导数（这里是加速度）。
        # dim=2 表示对x和y方向独立建模，block_size=2 表示 (位置, 速度) 块。
        # var 是过程噪声的方差。
        # self.kf.Q = Q_discrete_white_noise(dim=2, dt=dt, var=process_noise_std**2, block_size=2, order_by_dim=False)
        # 上述 Q_discrete_white_noise(..., order_by_dim=False) 生成的Q矩阵结构是 [x, vx, y, vy]
        # 为了匹配我们的状态向量 [x, y, vx, vy]，我们需要调整或手动构建。
        # 手动构建更标准形式的Q (针对加速度噪声)：
        # G 是噪声输入矩阵，假设噪声是加速度 ax, ay
        # x_new = x + v*dt + 0.5*ax*dt^2
        # y_new = y + v*dt + 0.5*ay*dt^2
        # vx_new = vx + ax*dt
        # vy_new = vy + ay*dt
        # G = [[0.5*dt^2, 0       ],
        #      [0,        0.5*dt^2],
        #      [dt,       0       ],
        #      [0,        dt      ]]
        # Q_kf = G * Q_noise_input * G.T
        # Q_noise_input = diag([sigma_ax^2, sigma_ay^2])
        # sigma_ax^2 = sigma_ay^2 = process_noise_std**2

        # 一个常用的Q矩阵形式（基于离散白噪声加速度模型）
        sigma_sq = process_noise_std ** 2
        self.kf.Q = np.array([
            [0.25 * dt ** 4, 0, 0.5 * dt ** 3, 0],
            [0, 0.25 * dt ** 4, 0, 0.5 * dt ** 3],
            [0.5 * dt ** 3, 0, dt ** 2, 0],
            [0, 0.5 * dt ** 3, 0, dt ** 2]
        ]) * sigma_sq

        # 如果上面的Q太复杂或效果不佳，可以从一个简单的对角阵开始调优：
        # q_pos = (0.5 * dt**2 * process_noise_std)**2 # 如果process_noise_std是加速度的标准差
        # q_vel = (dt * process_noise_std)**2
        # self.kf.Q = np.diag([q_pos, q_pos, q_vel, q_vel])
        # 或者更简单地，如果process_noise_std是一个调整因子:
        # self.kf.Q = np.eye(4) * (process_noise_std**2)

    def update(self, measurement_x: float, measurement_y: float):
        """
        使用新的测量值更新卡尔曼滤波器的状态。

        参数:
        - measurement_x (float): 当前测量的x坐标。
        - measurement_y (float): 当前测量的y坐标。
        """
        z = np.array([[measurement_x], [measurement_y]])
        self.kf.update(z)

    def predict(self) -> tuple[float, float]:
        """
        预测小车在下一个时间步的位置 (x, y)。
        这个方法会推进卡尔曼滤波器的内部状态到下一个时间点。

        返回:
        - tuple[float, float]: 预测的 (x, y) 坐标。
        """
        self.kf.predict()
        predicted_x = self.kf.x[0, 0]
        predicted_y = self.kf.x[1, 0]
        return predicted_x, predicted_y

    def get_current_estimate(self) -> tuple[float, float]:
        """
        获取当前滤波后的状态估计 (x, y, vx, vy)。
        这通常在 `update` 步骤之后调用，以获取对当前时刻状态的最佳估计。

        返回:
        - tuple[float, float]: 估计的 (x, y, vx, vy)。
        """
        x_est = self.kf.x[0, 0]
        y_est = self.kf.x[1, 0]
        vx_est = self.kf.x[2, 0]
        vy_est = self.kf.x[3, 0]
        return x_est, y_est


# --- 示例使用 ---
if __name__ == "__main__":
    import matplotlib.pyplot as plt

    # 模拟参数
    total_time_steps = 100
    dt_sim = 0.5  # 时间步长
    measurement_noise_sim = 1.5  # 模拟测量噪声的标准差
    process_noise_sim_accel = 0.3  # 模拟小车实际运动中的加速度标准差

    # 1. 生成小车的真实轨迹 (例如：一个近似直线运动，一个带转弯的运动)
    true_positions_car1 = []
    current_pos_car1 = np.array([0., 0.])
    current_vel_car1 = np.array([2., 1.])  # 初始速度 (m/s)

    true_positions_car2 = []
    current_pos_car2 = np.array([50., 0.])
    current_vel_car2 = np.array([-1., 1.5])  # 初始速度 (m/s)

    for t_step in range(total_time_steps):
        true_positions_car1.append(current_pos_car1.copy())
        true_positions_car2.append(current_pos_car2.copy())

        # 模拟小车1的运动 (近似直线)
        accel_car1 = np.random.randn(2) * process_noise_sim_accel  # 随机加速度
        current_pos_car1 += current_vel_car1 * dt_sim + 0.5 * accel_car1 * dt_sim ** 2
        current_vel_car1 += accel_car1 * dt_sim

        # 模拟小车2的运动 (带一点转弯趋势)
        if t_step > 30 and t_step < 70:  # 模拟转弯
            accel_car2 = np.array([0.1, -0.2]) + np.random.randn(2) * process_noise_sim_accel
        else:
            accel_car2 = np.random.randn(2) * process_noise_sim_accel
        current_pos_car2 += current_vel_car2 * dt_sim + 0.5 * accel_car2 * dt_sim ** 2
        current_vel_car2 += accel_car2 * dt_sim

    true_positions_car1 = np.array(true_positions_car1)
    true_positions_car2 = np.array(true_positions_car2)

    # 2. 生成带噪声的测量值
    measurements_car1 = true_positions_car1 + np.random.randn(total_time_steps, 2) * measurement_noise_sim
    measurements_car2 = true_positions_car2 + np.random.randn(total_time_steps, 2) * measurement_noise_sim

    # 3. 初始化预测器
    # 注意：这里的 process_noise_std 和 measurement_noise_std 是给滤波器的参数，
    # 它们不一定等于模拟中使用的 *_sim 值，而是需要调整以获得最佳滤波效果。
    # 通常，measurement_noise_std 可以根据传感器规格设置。
    # process_noise_std 是一个调整参数，反映了你对恒速模型的信任程度。
    car1_predictor = CarKalmanPredictor(
        initial_x=measurements_car1[0, 0],
        initial_y=measurements_car1[0, 1],
        dt=dt_sim,
        process_noise_std=0.8,  # 调优参数
        measurement_noise_std=measurement_noise_sim  # 假设我们知道测量噪声水平
    )
    car2_predictor = CarKalmanPredictor(
        initial_x=measurements_car2[0, 0],
        initial_y=measurements_car2[0, 1],
        dt=dt_sim,
        process_noise_std=1.0,  # 可以为不同车辆设置不同参数
        measurement_noise_std=measurement_noise_sim
    )

    # 4. 运行预测与更新循环
    predictions_car1 = []
    filtered_states_car1 = []
    predictions_car2 = []
    filtered_states_car2 = []

    # 对于第一个时间点，我们只有初始估计，没有"预测"自上一个状态
    # 所以通常先进行一次预测（基于初始状态），然后进行第一次更新
    pred_x1, pred_y1 = car1_predictor.predict()  # 预测 t=1 的状态 (基于 t=0 的初始x)
    predictions_car1.append((pred_x1, pred_y1))
    est_x1, est_y1 = car1_predictor.get_current_estimate()  # t=0 的初始状态 (经内部predict后可能略变)
    filtered_states_car1.append((est_x1, est_y1))

    pred_x2, pred_y2 = car2_predictor.predict()
    predictions_car2.append((pred_x2, pred_y2))
    est_x2, est_y2 = car2_predictor.get_current_estimate()
    filtered_states_car2.append((est_x2, est_y2))

    for i in range(total_time_steps):
        # 小车1
        # a. 使用当前测量值更新滤波器状态
        # 注意：对于第一个测量值 (i=0)，它已用于初始化，
        # 严格的KF循环是 predict -> update。
        # 如果在循环开始前已经调用了predict()，那么第一次迭代可以直接update。
        if i > 0:  # 第一个测量值已用于初始化或第一次预测/更新循环
            car1_predictor.update(measurements_car1[i, 0], measurements_car1[i, 1])

        # b. 获取当前滤波后的状态 (可选，用于绘图比较)
        current_est_x1, current_est_y1, _, _ = car1_predictor.get_current_estimate()
        if i == 0 and len(filtered_states_car1) > 0:  # 替换掉初始的predict后的状态
            filtered_states_car1[-1] = (current_est_x1, current_est_y1)
        else:
            filtered_states_car1.append((current_est_x1, current_est_y1))

        # c. 预测下一个时间步的位置
        # 只有在不是最后一步时才预测下一个，因为没有下一个测量了
        if i < total_time_steps - 1:
            next_pred_x1, next_pred_y1 = car1_predictor.predict()
            predictions_car1.append((next_pred_x1, next_pred_y1))

        # 小车2
        if i > 0:
            car2_predictor.update(measurements_car2[i, 0], measurements_car2[i, 1])

        current_est_x2, current_est_y2 = car2_predictor.get_current_estimate()
        if i == 0 and len(filtered_states_car2) > 0:
            filtered_states_car2[-1] = (current_est_x2, current_est_y2)
        else:
            filtered_states_car2.append((current_est_x2, current_est_y2))

        if i < total_time_steps - 1:
            next_pred_x2, next_pred_y2 = car2_predictor.predict()
            predictions_car2.append((next_pred_x2, next_pred_y2))

    # 确保长度匹配，predictions是针对下一个时间步的，所以会少一个
    # filtered_states 是针对当前时间步的
    predictions_car1_np = np.array(predictions_car1)
    filtered_states_car1_np = np.array(filtered_states_car1)
    predictions_car2_np = np.array(predictions_car2)
    filtered_states_car2_np = np.array(filtered_states_car2)

    # 5. 绘图比较
    plt.figure(figsize=(14, 10))

    # Car 1 plot
    plt.subplot(2, 1, 1)
    plt.plot(true_positions_car1[:, 0], true_positions_car1[:, 1], 'g-', label='Car 1 True Path', linewidth=2)
    plt.scatter(measurements_car1[:, 0], measurements_car1[:, 1], color='gray', s=20, label='Car 1 Measurements')
    plt.plot(filtered_states_car1_np[:, 0], filtered_states_car1_np[:, 1], 'b--',
             label='Car 1 Filtered Path (Estimate)', linewidth=1.5)
    # 预测是从上一步状态对当前步的预测，或者从当前步对下一步的预测。
    # predictions_car1_np 是对 t=1...N 的预测。filtered_states_car1_np 是对 t=0...N-1 的估计。
    # 绘制预测时，需要注意其对应的时间点。
    # 这里的 predictions_car1_np[i] 是在处理完 measurements_car1[i] 之后，对 i+1 时刻的预测。
    # 所以，它应该与 true_positions_car1[i+1] 对齐。
    if len(predictions_car1_np) > 0:
        plt.plot(predictions_car1_np[:-1, 0], predictions_car1_np[:-1, 1], 'r:',
                 label='Car 1 Predicted Path (One-step ahead)', linewidth=1.5)
        # Plot last prediction point separately
        plt.scatter(predictions_car1_np[-1, 0], predictions_car1_np[-1, 1], color='red', marker='x', s=50,
                    label='Car 1 Final Prediction')

    plt.title('Car 1 Tracking and Prediction')
    plt.xlabel('X position')
    plt.ylabel('Y position')
    plt.legend()
    plt.axis('equal')
    plt.grid(True)

    # Car 2 plot
    plt.subplot(2, 1, 2)
    plt.plot(true_positions_car2[:, 0], true_positions_car2[:, 1], 'g-', label='Car 2 True Path', linewidth=2)
    plt.scatter(measurements_car2[:, 0], measurements_car2[:, 1], color='gray', s=20, label='Car 2 Measurements')
    plt.plot(filtered_states_car2_np[:, 0], filtered_states_car2_np[:, 1], 'b--',
             label='Car 2 Filtered Path (Estimate)', linewidth=1.5)
    if len(predictions_car2_np) > 0:
        plt.plot(predictions_car2_np[:-1, 0], predictions_car2_np[:-1, 1], 'r:',
                 label='Car 2 Predicted Path (One-step ahead)', linewidth=1.5)
        plt.scatter(predictions_car2_np[-1, 0], predictions_car2_np[-1, 1], color='red', marker='x', s=50,
                    label='Car 2 Final Prediction')

    plt.title('Car 2 Tracking and Prediction')
    plt.xlabel('X position')
    plt.ylabel('Y position')
    plt.legend()
    plt.axis('equal')
    plt.grid(True)

    plt.tight_layout()
    plt.show()