#!ipython -i

import os
import sys
sys.path.append(f'{os.environ["HOME"]}/catkin_ws/src/multi_robot_localization/scripts')
import path_planner
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
import re

text = open(f'{os.environ["HOME"]}/catkin_ws/src/multi_robot_localization/src/occupancy_grid.cpp', mode='r').read()
occupancy_grid_file = re.findall(r'/occupancy_grid/(.*)"', text)[0]
occupancy_grid = np.loadtxt(f'{os.environ["HOME"]}/catkin_ws/src/multi_robot_localization/occupancy_grid/{occupancy_grid_file}', delimiter=",")

r1_path = path_planner.a_star(occupancy_grid, (10, 30), (130, 90)) +\
          path_planner.a_star(occupancy_grid, (130, 90) , (100, 110)) +\
          path_planner.a_star(occupancy_grid, (100, 110), (120, 20)) +\
          path_planner.a_star(occupancy_grid, (120, 20) , (40, 100)) +\
          path_planner.a_star(occupancy_grid, (40, 100) ,(60, 10)) +\
          path_planner.a_star(occupancy_grid, (60, 10)  ,(120, 60))


r2_path = path_planner.a_star(occupancy_grid, (130, 90), (10, 100)) +\
          path_planner.a_star(occupancy_grid, (10, 100), (20, 30)) +\
          path_planner.a_star(occupancy_grid, (20, 30), (50, 10)) +\
          path_planner.a_star(occupancy_grid, (50, 10), (110, 20)) +\
          path_planner.a_star(occupancy_grid, (110, 20), (100, 110)) +\
          path_planner.a_star(occupancy_grid, (100, 110), (20, 60))

# !echo "{r1_path}" > '{os.environ["HOME"]}/catkin_ws/src/multi_robot_localization/trajectories/a_star_1.txt'
# !echo "{r2_path}" > '{os.environ["HOME"]}/catkin_ws/src/multi_robot_localization/trajectories/a_star_2.txt'