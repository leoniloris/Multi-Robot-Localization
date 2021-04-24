#include "occupancy_grid.h"

#include <geometry_msgs/Pose2D.h>
#include <stdint.h>

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "math_utilities.h"
#include "ros/ros.h"

using namespace std;

// center: row 709 column 1361

geometry_msgs::Pose2D meters_to_cells(geometry_msgs::Pose2D pose_meters) {
    geometry_msgs::Pose2D pose_cells;
    pose_cells.x = pose.x * ROW_CELLS_PER_METER;
    pose_cells.y = pose.y * COLUMN_CELLS_PER_METER;
    return pose_cells;
}

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

bool OccupancyGrid::is_path_free(double x_begin, double y_begin, double x_end, double y_end) {
    bool has_reached_end_of_path = false;
    free_path_length(x_begin, y_begin, x_end, y_end, &has_reached_end_of_path);
    return has_reached_end_of_path;
}

double OccupancyGrid::free_path_length(double x_begin, double y_begin, double x_end, double y_end, bool* has_reached_end_of_path) {
    const uint16_t n_cells_to_check = max(abs(x_end - x_begin), abs(y_end - y_begin));
    static uint16_t x_to_check;
    static uint16_t y_to_check;
    *has_reached_end_of_path = true;

    uint16_t cell_idx_in_path = 1;
    for (cell_idx_in_path; cell_idx_in_path <= n_cells_to_check; cell_idx_in_path++) {
        const double path_proportion_to_finish = (double)cell_idx_in_path / (double)n_cells_to_check;

        x_to_check = x_begin + (uint16_t)((x_end - x_begin) * path_proportion_to_finish);
        y_to_check = y_begin + (uint16_t)((y_end - y_begin) * path_proportion_to_finish);

        const bool is_cell_out_of_map = (y_to_check >= n_columns) || (x_to_check >= n_rows);
        if (is_cell_out_of_map) {
            *has_reached_end_of_path = false;
            break;
        }

        const bool is_cell_occupied = grid[x_to_check][y_to_check] == 1;
        if (is_cell_occupied) {
            *has_reached_end_of_path = false;
            break;
        }
    }
    return L2_DISTANCE((x_to_check - (double)x_begin), (y_to_check - (double)y_begin));
}

double OccupancyGrid::distance_until_obstacle(double x_begin, double y_begin, double angle) {
    const uint16_t x_max = x_cells + cos(angle) * n_rows;
    const uint16_t y_max = y_cells + sin(angle) * n_columns;
    return free_path_length(x_begin, y_begin, x_end, y_end, /*dummy argument*/ (bool[]){true});
}
