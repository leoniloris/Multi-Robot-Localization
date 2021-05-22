from dataclasses import dataclass, field
from typing import Tuple
import numpy as np


# ADJACENT_CELL_DISPLACEMENTS = [
#     (-1, -1), (0, -1), (1, -1),
#     (-1,  0),          (1,  0),
#     (-1,  1), (0,  1), (1,  1),
# ]
ADJACENT_CELL_DISPLACEMENTS = [
             (0, -1),
    (-1,  0),         (1,  0),
             (0,  1),
]


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
        self.distance_from_begining += 1
        self.cost = self.distance_from_begining + \
            heuristic[self.position[0], self.position[1]]


def trace_back_path_with_smallest_cost(parent_cell):
    path = []
    current = parent_cell
    while current is not None:
        path.append(current.position)
        current = current.parent
    return path[::-1]


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


def create_heuristic_matrix(end_point: Tuple[int, int], shape: Tuple[int, int]):
    h = np.zeros(shape, dtype=int)
    for (row, col) in np.asarray(np.meshgrid(range(shape[0]), range(shape[1]))).T.reshape(-1, 2):
        h[row, col] = np.sqrt((row-end_point[0])**2 + (col-end_point[1])**2)
    return h


def a_star(occupancy_grid, start: Tuple[int, int], end: Tuple[int, int]):
    heuristic = create_heuristic_matrix(end, occupancy_grid.shape)
    start_cell = Cell(None, start)
    end_cell = Cell(None, end)

    cells_yet_to_visit = [start_cell]
    visited_cells = []

    ## TO DEBUG
    # global saving_stuff
    # saving_stuff = []
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
                    ## TO DEBUG
                    # saving_stuff.append(adjacent_cell.position)


if __name__ == '__main__':
    import os
    from scipy.ndimage import convolve
    occupancy_grid = np.loadtxt(os.environ["HOME"] +
                                "/catkin_ws/src/multi_robot_localization/occupancy_grid/rooms_small.csv", delimiter=",")
    cramming_kernel = np.ones((5, 5))
    occupancy_grid = np.clip(convolve(occupancy_grid, cramming_kernel), a_min=0, a_max=1)

    try:
        import time
        a = time.time()
        a_star(occupancy_grid, (130, 90), (10, 100))
        print((time.time()-a))
    except KeyboardInterrupt as e:
        print(e)

    ## TO DEBUG
    # x, y = list(zip(*saving_stuff))
    # import seaborn as sns
    # import matplotlib.pyplot as plt
    # sns.heatmap(occupancy_grid)
    # plt.scatter(y, x, alpha=0.3)
    # plt.show()
