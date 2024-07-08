# MotionAgent
智能机器人大作业

## 参考代码
- [zhm-real/MotionPlanning](https://github.com/zhm-real/MotionPlanning)
- [zhm-real/PathPlanning](https://github.com/zhm-real/PathPlanning)

## 运行环境
Python 3.6 or above
- [SciPy](https://scipy.org/)
- [cvxpy](https://github.com/cvxgrp/cvxpy)


## 代码解释
1. **Control文件夹**
   - **Pure_Pursuit.py**: 纯追踪横向控制转向角和角速度，PID纵向控制速度
   - **Rear_Wheel_Feedback.py**: 后轮反馈横向控制转向角和角速度，PID纵向控制速度
   - **draw_control.py**: 用于实现在轨迹跟踪过程中绘制机器人的函数
2. **CurvesGenerator文件夹**
   - **reeds_shepp.py**： 实现了Reeds-Shepp路径的计算和生成
   -  **draw_curve.py**： 用于可视化reeds_shepp.py
3. **Search_based_Planning文件夹**
   - **Env.py**： 定义设计场景问题，包括障碍物
   - **plotting.py**： 用于绘制地图、规划路径、规划访问过的点
   - **Dijkstra.py**： Dijkstra算法实现
   - **Astar.py**: A*算法实现
   - **Bidirectional_a_star.py**： 双向A*算法实现
5. **Dijkstra_pure_pid.py**： Dijkstra规划算法 + Reeds-Shepp路径 + Pure_Pursuit横向控制 + PID纵向控制
6. **Dijkstra_rear_wheel_pid.py**： Dijkstra规划算法 + Reeds-Shepp路径 + Rear_Wheel_Feedback横向控制 + PID纵向控制
7. **Astar_pure_pid.py**： Astar规划算法 + Reeds-Shepp路径 + Pure_Pursuit横向控制 + PID纵向控制
8. **Astar_rear_wheel_pid.py**： Astar规划算法 + Reeds-Shepp路径 + Rear_Wheel_Feedback横向控制 + PID纵向控制
9. **biAstar_pure_pid.py**： biAstar规划算法 + Reeds-Shepp路径 + Pure_Pursuit横向控制 + PID纵向控制
10. **biAstar_rear_wheel_pid.py**: biAstar规划算法 + Reeds-Shepp路径 + Rear_Wheel_Feedback横向控制 + PID纵向控制

