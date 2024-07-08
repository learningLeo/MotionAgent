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
C 类定义了各种配置参数，包括 PID 控制器参数、系统参数和车辆参数。

PID控制器参数（Kp）
Kp：这是用于纵向控制（PID控制器）中的比例增益系数。 纵向控制器的一部分，主要用于速度控制。

系统配置参数
K_theta：方向误差校正增益，控制车辆方向误差（横向控制）。
K_e：横向误差校正增益，控制车辆横向误差（横向控制）。
dt：时间步长，控制器每次更新的时间间隔。
dist_stop：停止距离，当车辆到达目标点的距离小于此值时，认为车辆到达目标，停止运动。

车辆配置参数
RF：车辆后轴中心到车头的距离。
RB：车辆后轴中心到车尾的距离。
W：车辆宽度。
WD：车辆左右车轮之间的距离，定义为宽度的0.7倍。
WB：轴距，即车辆前轴和后轴之间的距离。
TR：轮胎半径。
TW：轮胎宽度。
MAX_STEER：最大转向角度（单位未定义，通常为弧度）。
"""
class C:
    # PID config
    Kp = 0.15

    # System config
    K_theta = 1.0
    K_e = 0.5
    dt = 0.05
    dist_stop = 0.2

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

"""
Node 类表示车辆的状态，包括位置、方向、速度和行驶方向。
update 方法用于根据加速度 a 和转向角 delta 更新车辆状态。
"""
class Node:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, direct=1.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.direct = direct

    def update(self, a, delta, direct):
        self.x += self.v * math.cos(self.yaw) * C.dt
        self.y += self.v * math.sin(self.yaw) * C.dt
        self.yaw += self.v / C.WB * math.tan(delta) * C.dt
        self.direct = direct
        self.v += self.direct * a * C.dt

"""
PATH 类表示参考路径。
calc_theta_e_and_er 方法计算车辆当前的横向误差 er 和方向误差 theta_e。
nearest_index 方法找到离当前车辆位置最近的路径点的索引。
"""
class PATH:
    def __init__(self, cx, cy, cyaw, ccurv):
        self.cx = cx
        self.cy = cy
        self.cyaw = cyaw
        self.ccurv = ccurv
        self.len = len(self.cx)
        self.s0 = 1

    def calc_theta_e_and_er(self, node):
        """
        calc theta_e and er.
        theta_e = theta_car - theta_path
        er = lateral distance in frenet frame

        :param node: current information of vehicle
        :return: theta_e and er
        """

        ind = self.nearest_index(node)

        k = self.ccurv[ind]
        yaw = self.cyaw[ind]

        rear_axle_vec_rot_90 = np.array([[math.cos(node.yaw + math.pi / 2.0)],
                                         [math.sin(node.yaw + math.pi / 2.0)]])

        vec_target_2_rear = np.array([[node.x - self.cx[ind]],
                                      [node.y - self.cy[ind]]])

        er = np.dot(vec_target_2_rear.T, rear_axle_vec_rot_90)
        theta_e = pi_2_pi(node.yaw - self.cyaw[ind])

        return theta_e, er, k, yaw, ind

    def nearest_index(self, node):
        """
        find the index of the nearest point to current position.
        :param node: current information
        :return: nearest index
        """

        dx = [node.x - x for x in self.cx]
        dy = [node.y - y for y in self.cy]
        dist = np.hypot(dx, dy)
        self.s0 += np.argmin(dist[self.s0:self.len])

        return self.s0


"""
rear_wheel_feedback_control 方法实现了后轮反馈控制器，通过计算方向误差和横向误差来决定最佳转向角。
基于车辆的后轮反馈控制（Rear-Wheel Feedback Control）。
这是一个基于反馈的横向控制方法，它通过计算车辆相对于参考路径的误差来调整转向角，从而使车辆沿着参考路径行驶。
具体来说，这个方法结合了几何关系和车辆动力学模型来计算控制输入。
"""
def rear_wheel_feedback_control(node, ref_path):
    """
    rear wheel feedback controller
    :param node: current information
    :param ref_path: reference path: x, y, yaw, curvature
    :return: optimal steering angle
    """
    """
    计算误差
    方向误差 theta_e：车辆当前方向与路径方向之间的角度误差。
    横向误差 er：车辆后轴中心点到路径的横向距离。
    这些误差通过 ref_path.calc_theta_e_and_er(node) 函数计算得到。
    """
    theta_e, er, k, yaw, ind = ref_path.calc_theta_e_and_er(node)
    vr = node.v
    """
    控制公式分为三部分：
    1. 曲率补偿项: vr * k * math.cos(theta_e) / (1.0 - k * er) 
        vr：车辆速度。
        k：路径的曲率。
        这部分补偿路径曲率引起的误差。
    2. 方向误差校正项：- C.K_theta * abs(vr) * theta_e
        C.K_theta：方向误差校正增益。
        这部分根据方向误差调整角速度。
    3. 横向误差校正项: - C.K_e * vr * math.sin(theta_e) * er / theta_e
        C.K_e：横向误差校正增益。
        这部分根据横向误差调整角速度
    """
    omega = vr * k * math.cos(theta_e) / (1.0 - k * er) - \
            C.K_theta * abs(vr) * theta_e - C.K_e * vr * math.sin(theta_e) * er / theta_e
    """
    最终的角速度 omega 由上述三部分计算得到，然后通过以下公式计算转向角 delta：
    C.WB：车辆轴距。
    """
    delta = math.atan2(C.WB * omega, vr)

    return delta, ind


"""
pi_2_pi 方法将角度归一化到 [-pi, pi] 范围。
pid_control 方法实现了 PID 控制器，用于速度控制。
"""
def pi_2_pi(angle):
    if angle > math.pi:
        return angle - 2.0 * math.pi
    if angle < -math.pi:
        return angle + 2.0 * math.pi

    return angle


def pid_control(target_v, v, dist, direct):
    """
    using LQR as lateral controller, PID as longitudinal controller (speed control)
    :param target_v: target speed
    :param v: current speed
    :param dist: distance to end point
    :param direct: current direction of vehicle, 1.0: forward, -1.0: backward
    :return: acceleration
    """

    a = 0.3 * (target_v - direct * v)

    if dist < 5:
        if v > 2:
            a = -3.0
        elif v < -2:
            a = -1.0

    return a

"""
generate_path 方法使用 Reeds-Shepp 路径生成器设计路径，并分段保存不同方向的路径。
"""
def generate_path(s):
    """
    design path using reeds-shepp path generator.
    divide paths into sections, in each section the direction is the same.
    :param s: objective positions and directions.
    :return: paths
    """

    max_c = math.tan(C.MAX_STEER) / C.WB
    path_x, path_y, yaw, direct, rc = [], [], [], [], []
    x_rec, y_rec, yaw_rec, direct_rec, rc_rec = [], [], [], [], []
    direct_flag = 1.0

    for i in range(len(s) - 1):
        s_x, s_y, s_yaw = s[i][0], s[i][1], np.deg2rad(s[i][2])
        g_x, g_y, g_yaw = s[i + 1][0], s[i + 1][1], np.deg2rad(s[i + 1][2])

        path_i = rs.calc_optimal_path(s_x, s_y, s_yaw,
                                      g_x, g_y, g_yaw, max_c)

        irc, rds = rs.calc_curvature(path_i.x, path_i.y, path_i.yaw, path_i.directions)

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
                rc_rec.append(irc[j])
            else:
                if len(x_rec) == 0 or direct_rec[0] != direct_flag:
                    direct_flag = idirect[j]
                    continue

                path_x.append(x_rec)
                path_y.append(y_rec)
                yaw.append(yaw_rec)
                direct.append(direct_rec)
                rc.append(rc_rec)
                x_rec, y_rec, yaw_rec, direct_rec, rc_rec = \
                    [x_rec[-1]], [y_rec[-1]], [yaw_rec[-1]], [-direct_rec[-1]], [rc_rec[-1]]

    path_x.append(x_rec)
    path_y.append(y_rec)
    yaw.append(yaw_rec)
    direct.append(direct_rec)
    rc.append(rc_rec)

    x_all, y_all = [], []
    for ix, iy in zip(path_x, path_y):
        x_all += ix
        y_all += iy

    return path_x, path_y, yaw, direct, rc, x_all, y_all


def main():
    # generate path
    states = [(0, 0, 0), (20, 15, 0), (35, 20, 90), (40, 0, 180),
              (20, 0, 120), (5, -10, 180), (15, 5, 30)]
    #
    # states = [(-3, 3, 120), (10, -7, 30), (10, 13, 30), (20, 5, -25),
    #           (35, 10, 180), (30, -10, 160), (5, -12, 90)]

    x_ref, y_ref, yaw_ref, direct, curv, x_all, y_all = generate_path(states)

    maxTime = 100.0
    yaw_old = 0.0
    x0, y0, yaw0, direct0 = \
        x_ref[0][0], y_ref[0][0], yaw_ref[0][0], direct[0][0]

    x_rec, y_rec, yaw_rec, direct_rec = [], [], [], []

    for cx, cy, cyaw, cdirect, ccurv in zip(x_ref, y_ref, yaw_ref, direct, curv):
        t = 0.0
        node = Node(x=x0, y=y0, yaw=yaw0, v=0.0, direct=cdirect[0])
        ref_trajectory = PATH(cx, cy, cyaw, ccurv)

        while t < maxTime:
            if cdirect[0] > 0:
                speed_ref = 30.0 / 3.6
                C.Ld = 3.5
            else:
                speed_ref = 15.0 / 3.6
                C.Ld = 2.5

            delta, ind = rear_wheel_feedback_control(node, ref_trajectory)

            dist = math.hypot(node.x - cx[-1], node.y - cy[-1])

            acceleration = pid_control(speed_ref, node.v, dist, node.direct)
            node.update(acceleration, delta, node.direct)
            t += C.dt

            if dist <= C.dist_stop:
                break

            x_rec.append(node.x)
            y_rec.append(node.y)
            yaw_rec.append(node.yaw)
            direct_rec.append(node.direct)

            dy = (node.yaw - yaw_old) / (node.v * C.dt)
            steer = rs.pi_2_pi(-math.atan(C.WB * dy))

            yaw_old = node.yaw
            x0 = x_rec[-1]
            y0 = y_rec[-1]
            yaw0 = yaw_rec[-1]

            plt.cla()
            plt.plot(x_all, y_all, color='gray', linewidth=2.0)
            plt.plot(x_rec, y_rec, linewidth=2.0, color='darkviolet')
            plt.plot(cx[ind], cy[ind], '.r')
            draw_control.draw_car(node.x, node.y, node.yaw, steer, C)
            plt.axis("equal")
            plt.title("RearWheelFeedback: v=" + str(node.v * 3.6)[:4] + "km/h")
            plt.gcf().canvas.mpl_connect('key_release_event',
                                         lambda event:
                                         [exit(0) if event.key == 'escape' else None])
            plt.pause(0.001)

    plt.show()


if __name__ == '__main__':
    main()
