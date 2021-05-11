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

geometry_msgs::Pose2D meters_to_cells(geometry_msgs::Pose2D pose_meters) {
    geometry_msgs::Pose2D pose_cells;
    pose_cells.x = pose_meters.x * CELLS_PER_METER;
    pose_cells.y = pose_meters.y * CELLS_PER_METER;
    pose_cells.theta = pose_meters.theta;
    return pose_cells;
}

double meters_to_cells(double distance) {
    return distance * CELLS_PER_METER;
}

OccupancyGrid::OccupancyGrid() {
    const string home_folder = string(getenv("HOME"));
    const string path = home_folder + string("/catkin_ws/src/multi_robot_localization/occupancy_grid/rooms_small.csv");
    // const string path = home_folder + string("/catkin_ws/src/multi_robot_localization/occupancy_grid/cross_small.csv");


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
    bool will_reach_end_of_path = false;
    free_path_length(x_begin, y_begin, x_end, y_end, &will_reach_end_of_path);
    return will_reach_end_of_path;
}

double OccupancyGrid::free_path_length(double x_begin, double y_begin, double x_end, double y_end, bool* has_reached_end_of_path) {
    const uint16_t n_cells_to_check = (uint16_t)ceil(L2_DISTANCE(x_end - x_begin, y_end - y_begin));
    static uint16_t x_to_check;
    static uint16_t y_to_check;
    *has_reached_end_of_path = true;

    for (uint16_t cell_idx_in_path = 0; cell_idx_in_path <= n_cells_to_check; cell_idx_in_path++) {
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
    const double max_size = (double)max(n_rows, n_columns);

    const double x_max = x_begin + cos(angle) * max_size;
    const double y_max = y_begin + sin(angle) * max_size;
    static bool _dummy_has_reached_end_of_path = true;
    // printf("from (%f,%f) to (%f,%f),%f\n", x_begin, y_begin, x_max, y_max, fmod(angle * 180 / PI, 360));
    return free_path_length(x_begin, y_begin, x_max, y_max, &_dummy_has_reached_end_of_path);
}
