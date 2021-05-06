#pragma once

#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <stdint.h>

#include <string>
#include <vector>

#define CELLS_PER_METER (100.0 / 10.0)
#define X_CENTER (35)
#define Y_CENTER (70)

geometry_msgs::Pose2D meters_to_cells(geometry_msgs::Pose2D pose);
double meters_to_cells(double distance);

class OccupancyGrid {
    std::vector<std::vector<uint8_t>> grid;
    uint16_t n_columns;
    uint16_t n_rows;

   public:
    OccupancyGrid();
    ~OccupancyGrid(){};
    bool is_path_free(double x_begin, double y_begin, double x_end, double y_end);
    double free_path_length(double x_begin, double y_begin, double x_end, double y_end, bool* has_reached_end_of_path);
    uint16_t width_cells() { return n_columns; };
    uint16_t height_cells() { return n_rows; };
    double distance_until_obstacle(double x_begin, double y_begin, double angle);
};
