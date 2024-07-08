import sys
import math
import matplotlib.pyplot as plt

sys.path.append(f"D:\Leo的大三\智能机器人\大作业\myjob3\Search_based_Planning")
sys.path.append(f"D:\Leo的大三\智能机器人\大作业\myjob3\Control")
sys.path.append(f"D:\Leo的大三\智能机器人\大作业\myjob3\CurvesGenerator")

from Search_based_Planning import plotting
from Search_based_Planning import env as envpy
from Search_based_Planning import Astar as Astarjob
import Control.Rear_Wheel_Feedback as RWjob
import Control.draw_control as draw_control
import CurvesGenerator.reeds_shepp as rs


# 计算航向角
def calculate_angles(path):
    if len(path) == 1:
        return [0]
    path1 = path[0:len(path) - 1]
    path2 = path[1:]

    # print("path1")
    # print(path1)
    # print("path2")
    # print(path2)
    angles_list = []
    for i in range(0, len(path) - 1):
        x1, y1 = path1[i]
        x2, y2 = path2[i]
        dx = x2 - x1
        dy = y2 - y1

        # 使用atan2计算角度
        angle = math.atan2(dy, dx)
        # 将角度转换为度数
        angle_degrees = math.degrees(angle)

        angles_list.append(angle_degrees)

    angles_list.append(0)
    return angles_list


# Rear_Wheel 和 PID 共同作用仿真
def draw_Rear_Wheel_simulation(C, x_ref, y_ref, yaw_ref, direct, curv, x_all, y_all):
    env = envpy.Env1()
    obs = env.obs_map()

    obs_x = [ob[0] for ob in obs]
    obs_y = [ob[1] for ob in obs]

    maxTime = 100.0
    yaw_old = 0.0
    x0, y0, yaw0, direct0 = \
        x_ref[0][0], y_ref[0][0], yaw_ref[0][0], direct[0][0]

    x_rec, y_rec, yaw_rec, direct_rec = [], [], [], []

    for cx, cy, cyaw, cdirect, ccurv in zip(x_ref, y_ref, yaw_ref, direct, curv):
        t = 0.0
        node = RWjob.Node(x=x0, y=y0, yaw=yaw0, v=0.0, direct=cdirect[0])
        ref_trajectory = RWjob.PATH(cx, cy, cyaw, ccurv)

        while t < maxTime:
            if cdirect[0] > 0:
                speed_ref = 30.0 / 3.6
                C.Ld = 3.5
            else:
                speed_ref = 15.0 / 3.6
                C.Ld = 2.5

            delta, ind = RWjob.rear_wheel_feedback_control(node, ref_trajectory)

            dist = math.hypot(node.x - cx[-1], node.y - cy[-1])

            acceleration = RWjob.pid_control(speed_ref, node.v, dist, node.direct)
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
            plt.plot(obs_x, obs_y, "sk")
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


"""
main 函数初始化起点和终点，创建AStar实例并进行搜索，最后使用plotting模块绘制路径和动画。
初始化车辆状态和路径信息。
通过循环模拟车辆运动，更新车辆状态。
使用 PID 控制器和纯追踪控制器分别计算加速度和转向角。
实时绘制车辆位置、参考路径和实际路径。
"""


def main():
# **********************************************************************************************************************
# ---------------------------------------------  A*算法规划生成路径  ------------------------------------------------------
# **********************************************************************************************************************
    # 定义初始点和终点
    s_start = (5, 5)
    s_goal = (45, 25)
    # 规划路径
    astar = Astarjob.AStar(s_start, s_goal, "euclidean")
    plotAstar = plotting.Plotting(s_start, s_goal)
    path, visited = astar.searching()
    # 绘制路径和访问点
    plotAstar.animation(path, visited, "A*")  # animation
# **********************************************************************************************************************
# --------------------------------------------  处理生成途径点和航向角  ----------------------------------------------------
# **********************************************************************************************************************
    # 反转路径
    reversed_path = list(reversed(path))
    # 生成航向脚
    angles_list = calculate_angles(reversed_path)
    # 合并途径点和航向角
    un_states = [(reversed_path[i][0], reversed_path[i][1], angles_list[i]) for i in range(len(reversed_path))]
    # 缩小途径点转向角相同的点不必变换
    states = [un_states[0]]
    for i in range(1, len(un_states)):
        last_pos = len(states) - 1
        last_angle = states[last_pos][2]
        if un_states[i][2] != last_angle:
            states.append(un_states[i])
# **********************************************************************************************************************
# --------------------------------------------  生成reeds_shepp曲线  -----------------------------------------------------
# **********************************************************************************************************************
    x_ref, y_ref, yaw_ref, direct, curv, x_all, y_all = RWjob.generate_path(states)
# **********************************************************************************************************************
# ------------------------------------------ 使用Rear Wheel横向控制和PID纵向控制 -----------------------------------------
# **********************************************************************************************************************
    C = RWjob.C
    draw_Rear_Wheel_simulation(C, x_ref, y_ref, yaw_ref, direct, curv, x_all, y_all)


if __name__ == '__main__':
    main()
