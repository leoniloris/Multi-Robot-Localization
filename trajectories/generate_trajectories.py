from path_planner import a_star
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
import re
import os

text = open(f'{os.environ["HOME"]}/catkin_ws/src/multi_robot_localization/src/occupancy_grid.cpp', mode='r').read()
occupancy_grid_file = re.findall(r'/occupancy_grid/(.*)"', text)[0]
occupancy_grid = np.loadtxt(f'{os.environ["HOME"]}/catkin_ws/src/multi_robot_localization/occupancy_grid/{occupancy_grid_file}', delimiter=",")

r1_path = a_star(occupancy_grid, (10, 30), (130, 90)) +
          a_star(occupancy_grid, (130, 90) , (100, 110)) +
          a_star(occupancy_grid, (100, 110), (120, 20)) +
          a_star(occupancy_grid, (120, 20) , (40, 100)) +
          a_star(occupancy_grid, (40, 100) ,(60, 10)) +
          a_star(occupancy_grid, (60, 10)  ,(120, 60))


r2_path = a_star(occupancy_grid, (130, 90), (10, 100)) +
          a_star(occupancy_grid, (10, 100), (20, 30)) +
          a_star(occupancy_grid, (20, 30), (50, 10)) +
          a_star(occupancy_grid, (50, 10), (110, 20)) +
          a_star(occupancy_grid, (110, 20), (100, 110)) +
          a_star(occupancy_grid, (100, 110), (20, 60))
