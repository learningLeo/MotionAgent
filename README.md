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
