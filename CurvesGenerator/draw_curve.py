"""
这段代码用于绘制一个汽车的简化模型，并在汽车的前端绘制一个箭头指示其方向。代码主要由两个类组成：Arrow和Car，以及一个主程序块。
"""


import matplotlib.pyplot as plt
import numpy as np
PI = np.pi


"""
Arrow 类用于绘制一个箭头，表示汽车的方向。
__init__ 方法接受箭头的起始位置 (x, y)，方向 theta，长度 L，以及颜色 c。
箭头的主体由起点 (x_start, y_start) 到终点 (x_end, y_end) 的直线段组成。
箭头的两侧翼分别由终点向两侧延伸形成，用两个角度 theta_hat_L 和 theta_hat_R 计算出两侧翼的端点。
"""
class Arrow:
    def __init__(self, x, y, theta, L, c):
        angle = np.deg2rad(30)
        d = 0.5 * L
        w = 2

        x_start = x
        y_start = y
        x_end = x + L * np.cos(theta)
        y_end = y + L * np.sin(theta)

        theta_hat_L = theta + PI - angle
        theta_hat_R = theta + PI + angle

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
Car 类用于绘制一个简化的汽车模型。
__init__ 方法接受汽车的起始位置 (x, y)，方向 yaw，宽度 w，以及长度 L。
计算汽车的底部中心点 (xB, yB)，再通过底部中心点计算出底部左右顶点 (x_BL, y_BL) 和 (x_BR, y_BR)。
根据左右底部顶点，计算汽车的前部顶点 (x_FL, y_FL) 和 (x_FR, y_FR)。
使用 plt.plot 方法绘制汽车的矩形轮廓。
最后，调用 Arrow 类在汽车前部绘制一个箭头表示汽车的行驶方向。
"""
class Car:
    def __init__(self, x, y, yaw, w, L):
        theta_B = PI + yaw

        xB = x + L / 4 * np.cos(theta_B)
        yB = y + L / 4 * np.sin(theta_B)

        theta_BL = theta_B + PI / 2
        theta_BR = theta_B - PI / 2

        x_BL = xB + w / 2 * np.cos(theta_BL)        # Bottom-Left vertex
        y_BL = yB + w / 2 * np.sin(theta_BL)
        x_BR = xB + w / 2 * np.cos(theta_BR)        # Bottom-Right vertex
        y_BR = yB + w / 2 * np.sin(theta_BR)

        x_FL = x_BL + L * np.cos(yaw)               # Front-Left vertex
        y_FL = y_BL + L * np.sin(yaw)
        x_FR = x_BR + L * np.cos(yaw)               # Front-Right vertex
        y_FR = y_BR + L * np.sin(yaw)

        plt.plot([x_BL, x_BR, x_FR, x_FL, x_BL],
                 [y_BL, y_BR, y_FR, y_FL, y_BL],
                 linewidth=1, color='black')

        Arrow(x, y, yaw, L / 2, 'black')
        # plt.axis("equal")
        # plt.show()


if __name__ == '__main__':
    # Arrow(-1, 2, 60)
    Car(0, 0, 1, 2, 60)
