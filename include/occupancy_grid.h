#pragma once

#include <geometry_msgs/Pose2D.h>
#include <stdint.h>

#include <string>
#include <vector>

// y (columns) scale: 23 [meters] per 895 [cells|columns]
#define COLUMN_CELLS_PER_METER (895.0 / 23.0)

// x (rows) scale: 13 [meters] per 733 [cells|rows]
#define ROW_CELLS_PER_METER (733.0 / 13.0)

geometry_msgs::Pose2D meters_to_cells(geometry_msgs::Pose2D pose);

class OccupancyGrid {
    std::vector<std::vector<uint8_t>> grid;
    uint16_t n_columns;
    uint16_t n_rows;

   public:
    OccupancyGrid(const std::string& path);
    ~OccupancyGrid(){};
    bool is_path_free(double x_begin, double y_begin, double x_end, double y_end);
    double free_path_length(double x_begin, double y_begin, double x_end, double y_end, bool* has_reached_end_of_path);
    uint16_t width_cells() { return n_columns; };
    uint16_t height_cells() { return n_rows; };
    double distance_until_obstacle(double x_begin, double y_begin, double angle);
};
