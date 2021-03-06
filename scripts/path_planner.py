from dataclasses import dataclass, field
from typing import Tuple

import matplotlib.pyplot as plt
from numpy.core.numeric import roll
import seaborn as sns
import numpy as np
import os

ADJACENT_CELL_DISPLACEMENTS = [
    (-1, -1, 1), (0, -1, 1), (1, -1, 1),
    (-1,  0, 1),             (1,  0, 1),
    (-1,  1, 1), (0,  1, 1), (1,  1, 1),
    #          (0, -1, 1),
    # (-1,  0, 1),         (1,  0, 1),
    #          (0,  1, 1),

]
# ADJACENT_CELL_DISPLACEMENTS = [
#              (0, -1),
#     (-1,  0),         (1,  0),
#              (0,  1),
# ]



@dataclass
class Cell:
    parent: 'typing.Any'
    position: Tuple[int, int]

    distance_from_begining: int = 0
    cost: int = 0

    def __eq__(self, other):
        return self.position == other.position

    def __contains__(self, other):
        return self.position == other.position

    def hit_the_wall(self, occupancy_grid):
        return occupancy_grid[self.position[0], self.position[1]] != 0

    def is_outside_grid(self, shape):
        return self.position[0] > (shape[0] - 1) or self.position[0] < 0 or\
            self.position[1] > (shape[1] - 1) or self.position[1] < 0

    def displace(self, displacement, heuristic):
        self.position = (self.position[0] + displacement[0],
                         self.position[1] + displacement[1])
        self.distance_from_begining += displacement[2]
        self.cost = self.distance_from_begining + heuristic[self.position[0], self.position[1]]


def trace_back_path_with_smallest_cost(parent_cell, occupancy_grid):
    def smooth_path(path):
        x,y = np.asarray(list(zip(*path))[0]), np.asarray(list(zip(*path))[1])
        window_size = 20
        x_filtered = []
        y_filtered = []
        for idx in range(-window_size//2, len(x)):
            w_begin = np.clip(idx-(window_size//2-1), 0, len(x), dtype=np.int)
            w_end = np.clip(idx+(window_size//2), w_begin+1, len(x), dtype=np.int)
            w_x = x[w_begin:w_end]
            w_y = y[w_begin:w_end]
            std_x = w_x.std()
            std_y = w_y.std()
            x_filter_order = np.clip(int(window_size/(std_x+0.001)), 0, window_size-3)
            y_filter_order = np.clip(int(window_size/(std_y+0.001)), 0, window_size-3)
            subwindow_x_begin = window_size//2 - (x_filter_order//2) - 1
            subwindow_x_end = int(np.clip(window_size//2 + (x_filter_order//2) + 1, subwindow_x_begin+1, 10e10))
            subwindow_y_begin = window_size//2 - (y_filter_order//2) - 1
            subwindow_y_end = int(np.clip(window_size//2 + (y_filter_order//2) + 1, subwindow_y_begin+1, 10e10))

            to_filter_x = w_x[subwindow_x_begin:subwindow_x_end]
            to_filter_y = w_y[subwindow_y_begin:subwindow_y_end]
            if len(to_filter_x) != 0 and len(to_filter_y) != 0:
                x_filtered.append(to_filter_x.mean())
                y_filtered.append(to_filter_y.mean())
        return list(zip(x_filtered,y_filtered))


    path = []
    current = parent_cell
    while current is not None:
        path.append(current.position)
        current = current.parent
    path = path[::-1]
    return smooth_path(path)


def get_adjacent_cells(parent_cell, occupancy_grid, heuristic):
    adjacent_cells = []
    for cell_displacement in ADJACENT_CELL_DISPLACEMENTS:
        adjacent_cell = Cell(parent_cell, parent_cell.position)
        adjacent_cell.displace(cell_displacement, heuristic)

        if adjacent_cell.is_outside_grid(occupancy_grid.shape):
            continue

        if adjacent_cell.hit_the_wall(occupancy_grid):
            continue

        adjacent_cells.append(adjacent_cell)
    return adjacent_cells


def create_heuristic_matrix(end_point: Tuple[int, int], occupancy_grid):
    h = np.zeros(occupancy_grid.shape, dtype=int)
    for (row, col) in np.asarray(np.meshgrid(range(occupancy_grid.shape[0]), range(occupancy_grid.shape[1]))).T.reshape(-1, 2):
        h[row, col] = np.sqrt((row-end_point[0])**2 + (col-end_point[1])**2)
    return h


def increase_wall_sizes(occupancy_grid, x_wall_increase=4, y_wall_increase=5):
    from scipy.ndimage import convolve
    cramming_kernel = np.ones((x_wall_increase*2, y_wall_increase*2))
    return np.clip(convolve(occupancy_grid, cramming_kernel), a_min=0, a_max=1)


def find_free_start_end(start, end, occupancy_grid):
    radius_search = 30
    start_set = False
    end_set = False
    for r in range(radius_search):
        for displacement in [[1, 0], [0, 1], [-1, 0], [0, -1]]:
            start_to_test = (np.clip(start[0] + r*displacement[0], 0, occupancy_grid.shape[0]), np.clip(start[1] + r*displacement[1], 0, occupancy_grid.shape[1]))
            end_to_test = (np.clip(end[0] + r*displacement[0], 0, occupancy_grid.shape[0]), np.clip(end[1] + r*displacement[1], 0, occupancy_grid.shape[1]))
            if occupancy_grid[start_to_test[0], start_to_test[1]] == 0 and not start_set:
                start = start_to_test
                start_set = True
            if occupancy_grid[end_to_test[0], end_to_test[1]] == 0 and not end_set:
                end = end_to_test
                end_set = True

            if start_set and end_set:
                return start, end

def a_star(occupancy_grid, start: Tuple[int, int], end: Tuple[int, int], x_wall_increase=4, y_wall_increase=6):
    occupancy_grid = increase_wall_sizes(occupancy_grid, x_wall_increase, y_wall_increase)
    heuristic = create_heuristic_matrix(end, occupancy_grid)
    start, end = find_free_start_end(start, end, occupancy_grid)
    start_cell = Cell(None, start)
    end_cell = Cell(None, end)

    cells_yet_to_visit = [start_cell]
    visited_cells = []

    # TO DEBUG
    global saving_stuff
    saving_stuff = []
    while len(cells_yet_to_visit) > 0:
        cell_with_smallest_cost = min(cells_yet_to_visit, key=lambda c: c.cost)
        cells_yet_to_visit.remove(cell_with_smallest_cost)
        visited_cells.append(cell_with_smallest_cost)

        has_found_end_cell = cell_with_smallest_cost == end_cell
        if has_found_end_cell:
            path = trace_back_path_with_smallest_cost(cell_with_smallest_cost, occupancy_grid)
            plot_stuff(saving_stuff, path, occupancy_grid)
            return path

        for adjacent_cell in get_adjacent_cells(cell_with_smallest_cost, occupancy_grid, heuristic):
            if adjacent_cell in visited_cells:
                continue

            for visited_cell in visited_cells:
                if adjacent_cell == visited_cell:
                    break
            else:
                for cell_to_visit in cells_yet_to_visit:
                    if adjacent_cell == cell_to_visit and adjacent_cell.distance_from_begining >= cell_to_visit.distance_from_begining:
                        break
                else:
                    cells_yet_to_visit.append(adjacent_cell)
                    # TO DEBUG
                    saving_stuff.append(adjacent_cell.position)


def plot_stuff(saving_stuff, path, occupancy_grid):
    fig, ax = plt.subplots()
    x, y = list(zip(*saving_stuff))
    path_x, path_y = list(zip(*path))
    sns.heatmap(occupancy_grid, ax=ax)
    ax.scatter(y, x, alpha=0.3)
    ax.scatter(path_y, path_x, alpha=1)
    plt.pause(0.1)


def test():
    import re
    text = open(f'{os.environ["HOME"]}/catkin_ws/src/multi_robot_localization/src/occupancy_grid.cpp', mode='r').read()
    occupancy_grid_file = re.findall(r'/occupancy_grid/(.*)"', text)[0]
    occupancy_grid = np.loadtxt(f'{os.environ["HOME"]}/catkin_ws/src/multi_robot_localization/occupancy_grid/{occupancy_grid_file}', delimiter=",")
    try:
        import time
        a = time.time()
        # path = a_star(occupancy_grid, (130, 90), (10, 20))
        # path = a_star(occupancy_grid, (130, 90), (10, 100))
        {1: [[121.09612381376786, 80.54053159092268], [127.39905068583731, 10.05674727907513]], 2: [[127.39905068583731, 10.05674727907513], [121.09612381376786, 80.54053159092268]]}
        robot_begin_end = [[13.688589115913405, 105.02804535566482], [12.84592691880681, 109.36434319973365]]
        path = a_star(occupancy_grid, tuple(np.int_(robot_begin_end[0])), tuple(np.int_(robot_begin_end[1]))   )

        print((time.time()-a))
    except KeyboardInterrupt as e:
        print(e)

    # # TO DEBUG
    # plot_stuff(saving_stuff, path, occupancy_grid)

if __name__ == '__main__':
    test()
