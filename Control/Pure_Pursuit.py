"""
该代码实现了一个简单的纯追踪路径跟踪算法，并结合 PID 控制器来调整车辆速度。
通过使用 Reeds-Shepp 算法生成路径，并在模拟过程中实时更新和绘制车辆位置，
展示了移动机器人沿预定路径运动的过程。
"""

import os
import sys
import math
import numpy as np
import matplotlib.pyplot as plt
sys.path.append(f"D:\Leo的大三\智能机器人\大作业\myjob3\Search_based_Planning")
sys.path.append(f"D:\Leo的大三\智能机器人\大作业\myjob3\Control")
sys.path.append(f"D:\Leo的大三\智能机器人\大作业\myjob3\CurvesGenerator")

import Control.draw_control as draw_control
import CurvesGenerator.reeds_shepp as rs


"""
包含各种配置参数，例如 PID 控制的比例增益、系统配置（前视距离、时间步长、停止距离等）以及车辆配置（车身尺寸、轮胎尺寸、最大转向角等）。

PID 控制器配置: 用于控制车辆的速度和加速度。
Kp：比例增益，用于 PID 控制器中的比例项，控制加速度的调整幅度。

系统配置: 用于控制车辆在路径上的行为，包括前视距离、时间步长、停止距离等。
Ld：前视距离，决定了车辆在追踪路径时要看的距离。通常，前视距离会根据车辆速度进行动态调整。
kf：前视增益系数，用于动态调整前视距离。
dt：时间步长，表示模拟或控制的时间间隔。
dist_stop：停止距离，当车辆距离目标点小于此距离时，认为车辆到达目标点并停止。
dc：车辆中心偏移距离，表示车辆中心相对于路径点的偏移量。

车辆配置: 描述车辆的物理属性和限制条件，如尺寸、轮胎规格、最大转向角和加速度等。
RF：车辆前部到后轴的距离。
RB：车辆后部到后轴的距离。
W：车辆宽度。
WD：车辆左右轮之间的距离。
WB：车辆轴距，即前后轴之间的距离。
TR：轮胎半径。
TW：轮胎宽度。
MAX_STEER：最大转向角，表示车辆转向角的限制范围。
MAX_ACCELERATION：最大加速度，表示车辆加速或减速的限制范围。
"""
class C:
    # PID config
    Kp = 0.15  # proportional gain

    # system config
    Ld = 0.25  # look ahead distance
    kf = 0.05  # look forward gain
    dt = 0.05  # T step
    dist_stop = 0.2  # stop distance
    dc = 0.0

    # vehicle config
    RF = 1  # [m] distance from rear to vehicle front end of vehicle
    RB = 0.2  # [m] distance from rear to vehicle back end of vehicle
    W = 0.8  # [m] width of vehicle
    WD = 0.7 * W  # [m] distance between left-right wheels
    WB = 0.8  # [m] Wheel base
    TR = 0.1  # [m] Tyre radius
    TW = 0.12  # [m] Tyre width
    # MAX_STEER = 0.30
    MAX_STEER = 1
    MAX_ACCELERATION = 0.5


"""
Node 类：表示车辆的状态（位置、方向、速度和运动方向）。
update 方法：根据加速度 a 和转向角 delta 更新车辆状态。
limit_input 方法：限制转向角在合理范围内。
"""
class Node:
    def __init__(self, x, y, yaw, v, direct):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.direct = direct

    def update(self, a, delta, direct):
        # delta = self.limit_input(delta)
        self.x += self.v * math.cos(self.yaw) * C.dt
        self.y += self.v * math.sin(self.yaw) * C.dt
        self.yaw += self.v / C.WB * math.tan(delta) * C.dt
        self.direct = direct
        self.v += self.direct * a * C.dt

    @staticmethod
    def limit_input(delta):
        if delta > 1.2 * C.MAX_STEER:
            return 1.2 * C.MAX_STEER

        if delta < -1.2 * C.MAX_STEER:
            return -1.2 * C.MAX_STEER

        return delta

"""
Nodes 类：用于存储多个节点（车辆状态）信息，便于记录和绘图。
"""
class Nodes:
    def __init__(self):
        self.x = []
        self.y = []
        self.yaw = []
        self.v = []
        self.t = []
        self.direct = []

    def add(self, t, node):
        self.x.append(node.x)
        self.y.append(node.y)
        self.yaw.append(node.yaw)
        self.v.append(node.v)
        self.t.append(t)
        self.direct.append(node.direct)


"""
PATH 类：表示参考路径，包含路径坐标和一些方法。
target_index 方法：计算目标点索引，目标点距离为 Lf。
calc_nearest_ind 方法：计算离当前节点最近的路径点索引。
calc_distance 方法：计算当前节点和路径点之间的距离。
"""
class PATH:
    def __init__(self, cx, cy):
        self.cx = cx
        self.cy = cy
        self.ind_end = len(self.cx) - 1
        self.index_old = None

    def target_index(self, node):
        """
        search index of target point in the reference path.
        the distance between target point and current position is ld
        :param node: current information
        :return: index of target point
        """

        if self.index_old is None:
            self.calc_nearest_ind(node)

        Lf = C.kf * node.v + C.Ld

        for ind in range(self.index_old, self.ind_end + 1):
            if self.calc_distance(node, ind) > Lf:
                self.index_old = ind
                return ind, Lf

        self.index_old = self.ind_end

        return self.ind_end, Lf

    def calc_nearest_ind(self, node):
        """
        calc index of the nearest point to current position
        :param node: current information
        :return: index of nearest point
        """

        dx = [node.x - x for x in self.cx]
        dy = [node.y - y for y in self.cy]
        ind = np.argmin(np.hypot(dx, dy))
        self.index_old = ind

    def calc_distance(self, node, ind):
        return math.hypot(node.x - self.cx[ind], node.y - self.cy[ind])

"""
纯跟踪控制器用于横向控制，即控制车辆的方向和转向角度.
计算目标点和追踪距离 Lf，根据当前节点和目标点计算转向角 delta。
目标点 (tx, ty) 和 当前车辆位置 (node.x, node.y) 之间的角度 (alpha) 用于计算转向角 (delta)。
target_index 方法用于找到下一个目标点，从而确保车辆始终朝着目标路径前进。
"""
def pure_pursuit(node, ref_path, index_old):
    """
    pure pursuit controller
    :param node: current information
    :param ref_path: reference path: x, y, yaw, curvature
    :param index_old: target index of last time
    :return: optimal steering angle
    """
    """
    node：当前车辆的状态信息，包括位置 (x, y)、方向 (yaw)、速度 (v) 和运动方向 (direct)。
    ref_path：参考路径对象，包含路径点的坐标 (cx, cy) 以及相关的方法。
    index_old：上一次计算的目标点索引，用于避免重复计算。
    """
    """
    通过 target_index 方法获取目标点的索引 ind 和前瞻距离 Lf。前瞻距离是一个动态值，取决于车辆的当前速度和配置参数。
    """
    ind, Lf = ref_path.target_index(node)  # target point and pursuit distance
    ind = max(ind, index_old)
    """
    根据目标点索引获取目标点的 x 坐标和 y 坐标。
    """
    tx = ref_path.cx[ind]
    ty = ref_path.cy[ind]
    """
    alpha：目标点相对于车辆的角度。通过计算目标点和车辆当前点的角度差，并减去车辆当前的航向角得到。
    delta：需要的转向角。通过前瞻距离和车辆到目标点的角度来计算。
    """
    alpha = math.atan2(ty - node.y, tx - node.x) - node.yaw
    delta = math.atan2(2.0 * C.WB * math.sin(alpha), Lf)

    return delta, ind

"""
PID 控制器用于纵向控制，即控制车辆的速度和加速度。
根据目标速度 target_v 和当前速度 v 计算加速度 a。如果离终点较近，根据当前速度调整加速度，以便逐渐减速。
当车辆接近目标位置时（距离小于 5 米），根据当前速度进一步调整加速度以确保平稳减速和停车。
"""
def pid_control(target_v, v, dist, direct):
    """
    PID controller and design speed profile.
    :param target_v: target speed (forward and backward are different)
    :param v: current speed
    :param dist: distance from current position to end position
    :param direct: current direction
    :return: desired acceleration
    """
    """
    这行代码是 PID 控制器的核心部分，实际上这里实现的是比例控制（P 控制）。计算公式为：
        a = 0.3 * (target_v - direct * v)
    其中，K_p 是比例增益系数，这里为 0.3。
    target_v - direct * v 是速度误差，即目标速度与当前速度之间的差异。
    通过比例控制器，将速度误差转换为加速度（或减速度），从而控制车辆速度。
    """
    a = 0.3 * (target_v - direct * v)

    """
    当车辆距离目标位置小于 10 米时，进入减速过程，以确保车辆平稳停车：
    如果当前速度 v 大于 3 m/s，将加速度设为 -2.5 m/s²，使车辆快速减速。
    如果当前速度 v 小于 -2 m/s（倒退行驶），将加速度设为 -1.0 m/s²，使车辆快速减速。
    """
    if dist < 5:
        if v > 3.0:
            a = -2.5
        elif v < -2.0:
            a = -1.0

    return a



"""
根据输入的目标位置和方向，使用 Reeds-Shepp 算法生成路径段。
将路径段连接起来，生成整体路径的坐标列表 x_all 和 y_all。

输入：s，一个包含目标位置和方向的列表（(x, y, yaw)）。

输出：路径的坐标列表 path_x 和 path_y，以及其他相关信息（如 yaw, direct）。

generate_path 函数使用 Reeds-Shepp 算法生成路径段。

Reeds-Shepp 算法用于生成可以在前进和倒退之间自由切换的最优路径，适合于有方向性约束的车辆，如车不能在原地转向。

将路径段连接起来，生成整体路径的坐标列表 x_all 和 y_all。

理想路径：由 generate_path 函数生成，使用 Reeds-Shepp 算法根据输入的目标位置和方向生成路径段，然后连接成完整路径。
generate_path 函数生成了所有路径段的坐标 path_x, path_y 和 x_all, y_all，作为参考路径。
"""
def generate_path(s):
    """
    divide paths into some sections, in each section, the direction is the same.
    :param s: target position and yaw
    :return: sections
    """

    """
    max_c: 最大曲率，由最大转向角和轴距计算得出。在路径生成中的作用是决定车辆能够达到的最大转向能力。具体来说，最大曲率与车辆的最大转向角和轴距有关，是在路径规划中用来约束路径曲率的关键参数。
    path_x, path_y, yaw, direct: 用于存储各段路径的坐标、航向角和方向。
    x_rec, y_rec, yaw_rec, direct_rec: 用于暂存当前段路径的坐标、航向角和方向。
    direct_flag: 当前段的运动方向标志，初始为1.0（表示前进）。
    """
    max_c = math.tan(C.MAX_STEER) / C.WB  # max curvature
    path_x, path_y, yaw, direct = [], [], [], []
    x_rec, y_rec, yaw_rec, direct_rec = [], [], [], []
    direct_flag = 1.0

    """
    遍历输入的目标位置列表 s，依次计算相邻点之间的最优路径。
    将每段路径的 x 坐标、y 坐标、航向角和方向存储在相应的列表中。
    如果路径方向发生变化（前进变后退或反之），则将当前段路径存储到 path_x 等列表中，并重置 x_rec 等临时存储列表。
    """
    for i in range(len(s) - 1):
        """
        将当前点和下一个点的 x 坐标、y 坐标和航向角（转换为弧度）提取出来。
        调用 rs.calc_optimal_path 计算最优路径，这里 max_c 用于限制曲率。
        """
        s_x, s_y, s_yaw = s[i][0], s[i][1], np.deg2rad(s[i][2])
        g_x, g_y, g_yaw = s[i + 1][0], s[i + 1][1], np.deg2rad(s[i + 1][2])
        path_i = rs.calc_optimal_path(s_x, s_y, s_yaw,
                                      g_x, g_y, g_yaw, max_c)

        """
        存储路径点
        对于计算得到的路径，提取其 x 坐标、y 坐标、航向角和方向，并进行存储：
        
        将路径点的 x 坐标、y 坐标、航向角和方向分别存储到 ix, iy, iyaw 和 idirect。
        遍历路径点，如果路径点的方向与当前方向标志 direct_flag 相同，直接存储到 x_rec, y_rec, yaw_rec 和 direct_rec。
        如果路径点的方向与当前方向标志不同，并且当前段路径不为空且方向一致，将当前段路径存储到 path_x, path_y, yaw 和 direct，然后重置临时存储列表。
        """
        ix = path_i.x
        iy = path_i.y
        iyaw = path_i.yaw
        idirect = path_i.directions

        for j in range(len(ix)):
            if idirect[j] == direct_flag:
                x_rec.append(ix[j])
                y_rec.append(iy[j])
                yaw_rec.append(iyaw[j])
                direct_rec.append(idirect[j])
            else:
                if len(x_rec) == 0 or direct_rec[0] != direct_flag:
                    direct_flag = idirect[j]
                    continue

                path_x.append(x_rec)
                path_y.append(y_rec)
                yaw.append(yaw_rec)
                direct.append(direct_rec)
                x_rec, y_rec, yaw_rec, direct_rec = \
                    [x_rec[-1]], [y_rec[-1]], [yaw_rec[-1]], [-direct_rec[-1]]

    """
    遍历完所有目标位置后，将最后一段路径存储到 path_x, path_y, yaw 和 direct
    """
    path_x.append(x_rec)
    path_y.append(y_rec)
    yaw.append(yaw_rec)
    direct.append(direct_rec)

    x_all, y_all = [], []

    for ix, iy in zip(path_x, path_y):
        x_all += ix
        y_all += iy

    """
    x 和 y：多个段的路径坐标，每个段的 x 坐标和 y 坐标。具体来说，x 和 y 是包含多个路径段的列表。
    yaw：每个段的方向（航向角）列表，对应于路径点的方向。
    direct：每个段的运动方向列表（前进或后退），表示路径段的行驶方向。
    path_x 和 path_y：完整路径的 x 坐标和 y 坐标，是连接所有路径段后得到的整体路径。
    """
    return path_x, path_y, yaw, direct, x_all, y_all

"""
初始化车辆状态和路径信息。
通过循环模拟车辆运动，更新车辆状态。
使用 PID 控制器和纯追踪控制器分别计算加速度和转向角。
实时绘制车辆位置、参考路径和实际路径。
"""
def main():
    # generate path: [x, y, yaw]
    states = [(0, 0, 0), (20, 15, 0), (35, 20, 90), (40, 0, 180),
              (20, 0, 120), (5, -10, 180), (15, 5, 30)]

    # states = [(-3, 3, 120), (10, -7, 30), (10, 13, 30), (20, 5, -25),
    #           (35, 10, 180), (30, -10, 160), (5, -12, 90)]

    """
    x 和 y：多个段的路径坐标，每个段的 x 坐标和 y 坐标。具体来说，x 和 y 是包含多个路径段的列表。
    yaw：每个段的方向（航向角）列表，对应于路径点的方向。
    direct：每个段的运动方向列表（前进或后退），表示路径段的行驶方向。
    path_x 和 path_y：完整路径的 x 坐标和 y 坐标，是连接所有路径段后得到的整体路径。
    """
    x, y, yaw, direct, path_x, path_y = generate_path(states)

    print(x)
    print(len(x))
    print(y)
    print(len(y))
    print(yaw)
    print(len(yaw))
    print(direct)
    print(len(direct))
    print(path_x)
    print(len(path_x))
    print(path_y)
    print(len(path_x))



    # simulation
    """
    maxTime：仿真时间的最大值。
    yaw_old：上一个时间步的航向角。
    x0, y0, yaw0, direct0：初始化车辆的起始位置、方向和行驶方向。
    x_rec, y_rec：记录车辆在仿真过程中的轨迹点。
    """
    maxTime = 100.0
    yaw_old = 0.0
    x0, y0, yaw0, direct0 = x[0][0], y[0][0], yaw[0][0], direct[0][0]
    x_rec, y_rec = [], []
    """
    遍历每一段路径，初始化仿真时间t。
    创建一个Node对象，表示车辆的状态（位置、方向、速度、行驶方向）。
    创建一个Nodes对象，用于存储仿真过程中车辆的状态。
    将当前车辆状态添加到nodes中。
    创建一个PATH对象，表示当前路径段的参考轨迹。
    使用target_index方法找到车辆在参考轨迹中的目标点索引。
    """
    for cx, cy, cyaw, cdirect in zip(x, y, yaw, direct):
        t = 0.0
        # 初始化车辆的初始位置、方向、速度和运动方向。
        node = Node(x=x0, y=y0, yaw=yaw0, v=0.0, direct=direct0)
        nodes = Nodes()
        nodes.add(t, node)
        ref_trajectory = PATH(cx, cy)
        target_ind, _ = ref_trajectory.target_index(node)

        """
        判断当前路径段的行驶方向（前进或后退），设置相应的目标速度、前瞻距离、停止距离和速度变化。
        """
        while t <= maxTime:
            if cdirect[0] > 0:
                target_speed = 30.0 / 3.6
                C.Ld = 4.0
                C.dist_stop = 1.5
                C.dc = -1.1
            else:
                target_speed = 20.0 / 3.6
                C.Ld = 2.5
                C.dist_stop = 0.2
                C.dc = 0.2
            """
            计算车辆位置与当前路径段终点的距离，如果距离小于停止距离，结束当前路径段的仿真。
            """
            xt = node.x + C.dc * math.cos(node.yaw)
            yt = node.y + C.dc * math.sin(node.yaw)
            dist = math.hypot(xt - cx[-1], yt - cy[-1])

            if dist < C.dist_stop:
                break

            """
            使用PID控制算法计算车辆加速度。
            使用纯追踪算法计算转向角和目标点索引。
            更新仿真时间t。
            更新车辆状态并存储在nodes中。
            记录车辆当前位置。
            """
            acceleration = pid_control(target_speed, node.v, dist, cdirect[0])
            delta, target_ind = pure_pursuit(node, ref_trajectory, target_ind)

            t += C.dt

            node.update(acceleration, delta, cdirect[0])
            nodes.add(t, node)
            x_rec.append(node.x)
            y_rec.append(node.y)
            """
            计算转向角，并更新上一个时间步的航向角。
            更新车辆的起始位置、方向和行驶方向，为下一个路径段做准备。
            """
            dy = (node.yaw - yaw_old) / (node.v * C.dt)
            steer = rs.pi_2_pi(-math.atan(C.WB * dy))

            yaw_old = node.yaw
            x0 = nodes.x[-1]
            y0 = nodes.y[-1]
            yaw0 = nodes.yaw[-1]
            direct0 = nodes.direct[-1]
            """
            清除当前绘图。
            绘制车辆当前位置、参考路径、车辆轨迹和目标点。
            使用draw_car函数绘制车辆。
            设置绘图标题和退出事件。
            暂停以更新绘图。
            """
            # animation
            plt.cla()
            plt.plot(node.x, node.y, marker='.', color='k')
            plt.plot(path_x, path_y, color='gray', linewidth=2)
            plt.plot(x_rec, y_rec, color='darkviolet', linewidth=2)
            plt.plot(cx[target_ind], cy[target_ind], ".r")
            draw_control.draw_car(node.x, node.y, yaw_old, steer, C)

            # for m in range(len(states)):
            #     draw_control.Arrow(states[m][0], states[m][1], np.deg2rad(states[m][2]), 2, 'blue')

            plt.axis("equal")
            plt.title("PurePursuit: v=" + str(node.v * 3.6)[:4] + "km/h")
            plt.gcf().canvas.mpl_connect('key_release_event',
                                         lambda event:
                                         [exit(0) if event.key == 'escape' else None])
            plt.pause(0.001)

    plt.show()


if __name__ == '__main__':
    main()
