"""
Env 类定义了一个二维环境，包括其尺寸、可移动方向和障碍物位置。
__init__ 方法初始化了环境的尺寸、运动方向和障碍物位置。
update_obs 方法允许更新障碍物位置。
obs_map 方法定义了环境中所有的障碍物位置，包括边界障碍物和内部障碍物。
这个类可以用于模拟机器人在二维平面内的运动和路径规划。
"""

class Env1:
    def __init__(self):
        """
        self.x_range 和 self.y_range 定义了环境的尺寸。
        self.motions 定义了八个方向的移动，分别表示上下左右及对角线方向。
        self.obs 通过 self.obs_map() 方法初始化障碍物的位置。
        """
        self.x_range = 51  # size of background
        self.y_range = 31
        self.motions = [(-1, 0), (-1, 1), (0, 1), (1, 1),
                        (1, 0), (1, -1), (0, -1), (-1, -1)]
        self.obs = self.obs_map()

    def update_obs(self, obs):
        """
        update_obs 方法用于更新障碍物的位置。
        参数 obs 是一个包含障碍物位置的集合。
        """
        self.obs = obs

    def obs_map(self):
        """
        Initialize obstacles' positions
        :return: map of obstacles
        obs_map 方法用于初始化障碍物的位置，并返回一个包含障碍物位置的集合。
        该方法首先定义了障碍物集合 obs。
        然后通过四个循环将环境的边界定义为障碍物：
        第一、二个循环将环境的上边和下边设置为障碍物。
        第三、四个循环将环境的左边和右边设置为障碍物。
        最后，通过一些循环定义环境内部的障碍物：
        """
        x = self.x_range
        y = self.y_range
        obs = set()

        for i in range(x):
            obs.add((i, 0))
        for i in range(x):
            obs.add((i, y - 1))

        for i in range(y):
            obs.add((0, i))
        for i in range(y):
            obs.add((x - 1, i))

        for i in range(17, 21):
            obs.add((i, 15))
        for i in range(15):
            obs.add((20, i))

        for i in range(20, 30):
            obs.add((30, i))
        for i in range(16):
            obs.add((40, i))

        for i in range(15,25):
            for j in  range(20,25):
                obs.add((i,j))

        for i in range(10,20):
            obs.add((i,5))

        return obs


