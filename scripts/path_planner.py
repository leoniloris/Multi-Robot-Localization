from dataclasses import dataclass, field
from typing import Tuple

import matplotlib.pyplot as plt
from numpy.core.numeric import roll
import seaborn as sns
import numpy as np
import os

ADJACENT_CELL_DISPLACEMENTS = [
    (-1, -1), (0, -1), (1, -1),
    (-1,  0),          (1,  0),
    (-1,  1), (0,  1), (1,  1),
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
        return occupancy_grid[int(self.position[0]), int(self.position[1])] != 0

    def is_outside_grid(self, shape):
        return self.position[0] > (shape[0] - 1) or self.position[0] < 0 or\
            self.position[1] > (shape[1] - 1) or self.position[1] < 0

    def displace(self, displacement, heuristic):
        self.position = (self.position[0] + displacement[0],
                         self.position[1] + displacement[1])
        self.distance_from_begining += 1#displacement[2]
        self.cost = self.distance_from_begining + heuristic[int(np.floor(self.position[0])), int(np.floor(self.position[1]))]


def trace_back_path_with_smallest_cost(parent_cell):
    def rolling_window(data, window_size):
        shape = data.shape[:-1] + (data.shape[-1]-window_size//2, window_size)
        strides = data.strides + (data.strides[-1],)
        return np.lib.stride_tricks.as_strided(data, shape=shape, strides=strides)

    def smooth_path(path):
        x,y = np.asarray(list(zip(*path))[0]), np.asarray(list(zip(*path))[1])
        window_size = 20
        x_filtered = []
        for w in rolling_window(x, window_size):
            filter_order = np.clip(int(window_size/(w.std()+0.001)), 0, window_size-3)
            begin = window_size//2  -   (filter_order//2) - 1
            end = window_size//2  +   (filter_order//2) + 1
            x_filtered.append(w[begin:end].mean())

        y_filtered = []
        for w in rolling_window(y, window_size):
            filter_order = np.clip(int(window_size/(w.std()+0.001)), 0, window_size-3)
            begin = window_size//2  -   (filter_order//2) - 1
            end = window_size//2  +   (filter_order//2) + 1
            y_filtered.append(w[begin:end].mean())
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


def a_star(occupancy_grid, start: Tuple[int, int], end: Tuple[int, int]):
    heuristic = create_heuristic_matrix(end, occupancy_grid)
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
            return trace_back_path_with_smallest_cost(cell_with_smallest_cost)

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


def test():
    import re
    text = open(f'{os.environ["HOME"]}/catkin_ws/src/multi_robot_localization/src/occupancy_grid.cpp', mode='r').read()
    occupancy_grid_file = re.findall(r'/occupancy_grid/(.*)"', text)[0]
    occupancy_grid = np.loadtxt(f'{os.environ["HOME"]}/catkin_ws/src/multi_robot_localization/occupancy_grid/{occupancy_grid_file}', delimiter=",")

    try:
        import time
        a = time.time()
        path = a_star(occupancy_grid, (130, 90), (10, 20))
        # path = a_star(occupancy_grid, (130, 90), (10, 100))
        print((time.time()-a))
    except KeyboardInterrupt as e:
        print(e)

    # TO DEBUG
    x, y = list(zip(*saving_stuff))
    path_x, path_y = list(zip(*path))
    sns.heatmap(occupancy_grid)
    plt.scatter(y, x, alpha=0.3)
    plt.scatter(path_y, path_x, alpha=1)
    plt.show()

if __name__ == '__main__':
    test()
