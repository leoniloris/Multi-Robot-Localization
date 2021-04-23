#include "occupancy_grid.h"

#include <stdint.h>

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "ros/ros.h"

using namespace std;

// center: row 709 column 1361

// y (columns) scale: 7 [meters] per 273 [cells|columns]
#define COLUMN_CELLS_PER_METER 273.0 / 7.0

// x (rows) scale: 3.5 [meters] per 198 [cells|rows]
#define ROW_CELLS_PER_METER 198.0 / 3.5

OccupancyGrid::OccupancyGrid(const std::string& path) {
    string line;
    ifstream f(path.c_str());
    if (!f.is_open()) {
        ROS_ERROR_STREAM("error while opening occupancy grid");
        exit(1);
    }

    while (getline(f, line)) {
        string val;
        vector<uint8_t> row;
        stringstream s(line);

        while (getline(s, val, ','))
            row.push_back(stoi(val));
        grid.push_back(row);
    }
    f.close();

    n_rows = grid.size();
    n_columns = grid[0].size();
    ROS_INFO_STREAM("occupancy grid loaded: " << n_rows << " rows by " << n_columns << " columns (cells).");
}

bool OccupancyGrid::is_path_free(double x1_meters, double y1_meters, double x2_meters, double y2_meters) {
    const uint16_t x1_cells = x1_meters * ROW_CELLS_PER_METER;
    const uint16_t y1_cells = y1_meters * COLUMN_CELLS_PER_METER;
    const uint16_t x2_cells = x2_meters * ROW_CELLS_PER_METER;
    const uint16_t y2_cells = y2_meters * COLUMN_CELLS_PER_METER;

    const uint16_t n_cells_to_check = max(abs(x2_cells - x1_cells), abs(y2_cells - y1_cells));

    uint16_t cell_to_check_x;
    uint16_t cell_to_check_y;

    for (uint16_t cell_idx_in_path = 1; cell_idx_in_path <= n_cells_to_check; cell_idx_in_path++) {
        const double path_proportion_to_finish = (double)cell_idx_in_path / (double)n_cells_to_check;

        cell_to_check_x = x1_cells + (uint16_t)((x2_cells - x1_cells) * path_proportion_to_finish);
        cell_to_check_y = y1_cells + (uint16_t)((y2_cells - y1_cells) * path_proportion_to_finish);

        const bool is_cell_out_of_map = (cell_to_check_y >= n_columns) || (cell_to_check_x >= n_rows);
        if (is_cell_out_of_map) {
            return false;
        }

        const bool is_cell_occupied = grid[cell_to_check_y][cell_to_check_x] == 1;
        if (is_cell_occupied) {
            return false;
        }
    }
    return true;
}