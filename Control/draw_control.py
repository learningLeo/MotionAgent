import math
import numpy as np
import matplotlib.pyplot as plt


"""
x, y：箭头起点的位置。
theta：箭头方向的角度。
L：箭头的长度。
c：箭头的颜色。
Arrow 类绘制了一条从 (x, y) 到 (x_end, y_end) 的直线，以及两个箭头头部，从而形成一个箭头形状。
"""
class Arrow:
    def __init__(self, x, y, theta, L, c):
        angle = np.deg2rad(30)
        d = 0.4 * L
        w = 2

        x_start = x
        y_start = y
        x_end = x + L * np.cos(theta)
        y_end = y + L * np.sin(theta)

        theta_hat_L = theta + math.pi - angle
        theta_hat_R = theta + math.pi + angle

        x_hat_start = x_end
        x_hat_end_L = x_hat_start + d * np.cos(theta_hat_L)
        x_hat_end_R = x_hat_start + d * np.cos(theta_hat_R)

        y_hat_start = y_end
        y_hat_end_L = y_hat_start + d * np.sin(theta_hat_L)
        y_hat_end_R = y_hat_start + d * np.sin(theta_hat_R)

        plt.plot([x_start, x_end], [y_start, y_end], color=c, linewidth=w)
        plt.plot([x_hat_start, x_hat_end_L],
                 [y_hat_start, y_hat_end_L], color=c, linewidth=w)
        plt.plot([x_hat_start, x_hat_end_R],
                 [y_hat_start, y_hat_end_R], color=c, linewidth=w)

"""
draw_car 函数用于绘制车辆及其四个轮子，并用箭头指示车辆的方向。
"""
def draw_car(x, y, yaw, steer, C, color='black'):
    """
    car 数组定义了车辆的边界。
    wheel 数组定义了一个轮子的边界。
    """
    car = np.array([[-C.RB, -C.RB, C.RF, C.RF, -C.RB],
                    [C.W / 2, -C.W / 2, -C.W / 2, C.W / 2, C.W / 2]])

    wheel = np.array([[-C.TR, -C.TR, C.TR, C.TR, -C.TR],
                      [C.TW / 4, -C.TW / 4, -C.TW / 4, C.TW / 4, C.TW / 4]])
    """ 复制 wheel 数组以创建四个轮子。 """
    rlWheel = wheel.copy()
    rrWheel = wheel.copy()
    frWheel = wheel.copy()
    flWheel = wheel.copy()
    """
    Rot1 是根据车辆方向 yaw 计算的旋转矩阵。
    Rot2 是根据转向角 steer 计算的旋转矩阵。
    """
    Rot1 = np.array([[math.cos(yaw), -math.sin(yaw)],
                     [math.sin(yaw), math.cos(yaw)]])

    Rot2 = np.array([[math.cos(steer), math.sin(steer)],
                     [-math.sin(steer), math.cos(steer)]])
    """
    旋转前轮
    前轮（frWheel 和 flWheel）根据转向角进行旋转。
    将前轮平移到正确的位置。
    后轮（rrWheel 和 rlWheel）直接平移到正确的位置。
    所有轮子和车身根据车辆的方向 yaw 进行旋转。
    """
    frWheel = np.dot(Rot2, frWheel)
    flWheel = np.dot(Rot2, flWheel)

    frWheel += np.array([[C.WB], [-C.WD / 2]])
    flWheel += np.array([[C.WB], [C.WD / 2]])
    rrWheel[1, :] -= C.WD / 2
    rlWheel[1, :] += C.WD / 2

    frWheel = np.dot(Rot1, frWheel)
    flWheel = np.dot(Rot1, flWheel)

    rrWheel = np.dot(Rot1, rrWheel)
    rlWheel = np.dot(Rot1, rlWheel)
    car = np.dot(Rot1, car)
    """
    平移车轮
    将所有轮子和平移到车辆的当前位置 (x, y)。
    """
    frWheel += np.array([[x], [y]])
    flWheel += np.array([[x], [y]])
    rrWheel += np.array([[x], [y]])
    rlWheel += np.array([[x], [y]])
    car += np.array([[x], [y]])
    """
    使用 plt.plot 绘制车辆轮廓和轮子。
    使用 Arrow 类绘制方向箭头。
    """
    plt.plot(car[0, :], car[1, :], color)
    plt.plot(frWheel[0, :], frWheel[1, :], color)
    plt.plot(rrWheel[0, :], rrWheel[1, :], color)
    plt.plot(flWheel[0, :], flWheel[1, :], color)
    plt.plot(rlWheel[0, :], rlWheel[1, :], color)
    Arrow(x, y, yaw, C.WB * 0.6, color)
