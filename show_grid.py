import sys
import math
import matplotlib.pyplot as plt

sys.path.append(f"D:\Leo的大三\智能机器人\大作业\myjob3\Search_based_Planning")
sys.path.append(f"D:\Leo的大三\智能机器人\大作业\myjob3\Control")
sys.path.append(f"D:\Leo的大三\智能机器人\大作业\myjob3\CurvesGenerator")

from Search_based_Planning import env as envpy

def main():
    env = envpy.Env1()
    obs = env.obs_map()

    obs_x = [ob[0] for ob in obs]
    obs_y = [ob[1] for ob in obs]

    # 定义初始点和终点
    s_start = (5, 5)
    s_goal = (45, 25)
    plt.plot(s_start[0], s_start[1], "or")
    plt.plot(s_goal[0], s_goal[1], "ob")
    plt.plot(obs_x, obs_y, "sk")
    plt.show()
main()