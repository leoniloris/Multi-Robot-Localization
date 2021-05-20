from dataclasses import dataclass, field
from typing import Tuple
import numpy as np


ADJACENT_CELL_DISPLACEMENTS = [
    (-1, -1), (0, -1), (1, -1),
    (-1,  0),          (1,  0),
    (-1,  1), (0,  1), (1,  1),
]


@dataclass
class Cell:
    parent: 'typing.Any'
    position: Tuple[int, int]

    distance_from_begining: int = 0
    cost: int = 0

    def __eq__(self, other):
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
        h[row, col] = (row-end_point[0])**2 + (col-end_point[1])**2
    return h


def a_star(occupancy_grid, start: Tuple[int, int], end: Tuple[int, int]):
    heuristic = create_heuristic_matrix(end, occupancy_grid.shape)
    start_cell = Cell(None, start)
    end_cell = Cell(None, end)

    cells_yet_to_visit = [start_cell]
    visited_cells = []

    while len(cells_yet_to_visit) > 0:
        cell_with_smallest_cost = min(cells_yet_to_visit, key=lambda c: c.cost)
        cells_yet_to_visit.remove(cell_with_smallest_cost)
        visited_cells.append(cell_with_smallest_cost)

        has_found_end_cell = cell_with_smallest_cost == end_cell
        if has_found_end_cell:
            return trace_back_path_with_smallest_cost(cell_with_smallest_cost)

        for adjacent_cell in get_adjacent_cells(cell_with_smallest_cost, occupancy_grid, heuristic):
            adjacent_cell.distance_from_begining = cell_with_smallest_cost.distance_from_begining + 1
            adjacent_cell.cost = adjacent_cell.distance_from_begining + \
                heuristic[adjacent_cell.position[0], adjacent_cell.position[1]]
            print(adjacent_cell.distance_from_begining, adjacent_cell.cost)

            for visited_cell in visited_cells:
                if adjacent_cell == visited_cell:
                    continue

            for open_cell in cells_yet_to_visit:
                if adjacent_cell == open_cell and adjacent_cell.distance_from_begining > open_cell.distance_from_begining:
                    continue

            cells_yet_to_visit.append(adjacent_cell)


def test():
    occupancy_grid = np.asarray([[0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
                                 [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
                                 [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
                                 [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
                                 [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
                                 [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                 [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
                                 [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
                                 [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
                                 [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])

    start = (0, 0)
    end = (7, 6)

    path = a_star(occupancy_grid, start, end)
    assert [(0, 0), (1, 1), (2, 2), (3, 3), (4, 3),
            (5, 4), (6, 5), (7, 6)] == path
    print(path)


if __name__ == '__main__':
    test()
