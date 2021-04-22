#include "occupancy_grid.h"

#include <stdint.h>

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "ros/ros.h"

using namespace std;

OccupancyGrid::OccupancyGrid(const std::string& path) {
    string line;
    ifstream f(path.c_str());
    if (!f.is_open()) {
        ROS_ERROR_STREAM("error while opening occupancy array");
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

    n_columns = grid[0].size();
    n_rows = grid.size();
    ROS_INFO_STREAM("occupancy grid loaded: " << n_rows << " by " << n_columns << "pixels.");
}

bool OccupancyGrid::is_path_free(double x1, double y1, double x2, double y2) {
    uint16_t path_length = (uint16_t)max(abs(x2 - x1), abs(y2 - y1));

    uint16_t cell_to_check_x;
    uint16_t cell_to_check_y;

    for (uint16_t path_idx = 1; path_idx <= path_length; path_idx++) {
        const double path_location = (double)path_idx / (double)path_length;
        cell_to_check_x = (uint16_t) (x1 + (x2-x1) * path_location);
        cell_to_check_y = (uint16_t) (y1 + (y2-y1) * path_location);

        if (cell_to_check_y >= n_columns || cell_to_check_x >= n_rows) {
            return false;
        }

        const bool is_cell_occupied = grid[cell_to_check_y][cell_to_check_x] == 1;
        if (is_cell_occupied) {
            return false;
        }
    }
    return true;
}