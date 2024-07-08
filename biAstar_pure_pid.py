import sys
import math
import matplotlib.pyplot as plt

sys.path.append(f"D:\Leo的大三\智能机器人\大作业\myjob3\Search_based_Planning")
sys.path.append(f"D:\Leo的大三\智能机器人\大作业\myjob3\Control")
sys.path.append(f"D:\Leo的大三\智能机器人\大作业\myjob3\CurvesGenerator")

from Search_based_Planning import plotting
from Search_based_Planning import env as envpy
from Search_based_Planning import Bidirectional_a_star as BiAstarjob
import Control.Pure_Pursuit as PPjob
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


# pure_pursuit 和 PID 共同作用仿真
def draw_pure_PID_simulation(C, x, y, yaw, direct, path_x, path_y):
    # simulation
    """
    maxTime：仿真时间的最大值。
    yaw_old：上一个时间步的航向角。
    x0, y0, yaw0, direct0：初始化车辆的起始位置、方向和行驶方向。
    x_rec, y_rec：记录车辆在仿真过程中的轨迹点。
    """
    env = envpy.Env1()
    obs = env.obs_map()

    obs_x = [ob[0] for ob in obs]
    obs_y = [ob[1] for ob in obs]

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
        node = PPjob.Node(x=x0, y=y0, yaw=yaw0, v=0.0, direct=direct0)
        nodes = PPjob.Nodes()
        nodes.add(t, node)
        ref_trajectory = PPjob.PATH(cx, cy)
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
            acceleration = PPjob.pid_control(target_speed, node.v, dist, cdirect[0])
            delta, target_ind = PPjob.pure_pursuit(node, ref_trajectory, target_ind)

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
            plt.plot(obs_x, obs_y, "sk")
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


"""
main 函数初始化起点和终点，创建AStar实例并进行搜索，最后使用plotting模块绘制路径和动画。
初始化车辆状态和路径信息。
通过循环模拟车辆运动，更新车辆状态。
使用 PID 控制器和纯追踪控制器分别计算加速度和转向角。
实时绘制车辆位置、参考路径和实际路径。
"""


def main():
# **********************************************************************************************************************
# ---------------------------------------------  bi-A*算法规划生成路径  ------------------------------------------------------
# **********************************************************************************************************************
    # 定义初始点和终点
    s_start = (5, 5)
    s_goal = (45, 25)
    # 规划路径
    bastar = BiAstarjob.BidirectionalAStar(s_start, s_goal, "euclidean")
    plot = plotting.Plotting(s_start, s_goal)
    path, visited_fore, visited_back = bastar.searching()
    # 绘制路径和访问点
    plot.animation_bi_astar(path, visited_fore, visited_back, "Bidirectional-A*")  # animation
# **********************************************************************************************************************
# --------------------------------------------  处理生成途径点和航向角  ----------------------------------------------------
# **********************************************************************************************************************
    # 反转路径
    # reversed_path = list(reversed(path))
    reversed_path = path
    # 生成航向角度
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
    """
    x 和 y：多个段的路径坐标，每个段的 x 坐标和 y 坐标。具体来说，x 和 y 是包含多个路径段的列表。
    yaw：每个段的方向（航向角）列表，对应于路径点的方向。
    direct：每个段的运动方向列表（前进或后退），表示路径段的行驶方向。
    path_x 和 path_y：完整路径的 x 坐标和 y 坐标，是连接所有路径段后得到的整体路径。
    """
    x, y, yaw, direct, path_x, path_y = PPjob.generate_path(states)
# **********************************************************************************************************************
# ------------------------------------------ 使用pure_pursuit横向控制和PID纵向控制 -----------------------------------------
# **********************************************************************************************************************
    C = PPjob.C
    draw_pure_PID_simulation(C, x, y, yaw, direct, path_x, path_y)


if __name__ == '__main__':
    main()
