#pragma once

#include <stdint.h>

#include <string>
#include <vector>

// y (columns) scale: 23 [meters] per 895 [cells|columns]
#define COLUMN_CELLS_PER_METER (895.0 / 23.0)

// x (rows) scale: 13 [meters] per 733 [cells|rows]
#define ROW_CELLS_PER_METER (733.0 / 13.0)

class OccupancyGrid {
    std::vector<std::vector<uint8_t>> grid;
    uint16_t n_columns;
    uint16_t n_rows;

   public:
    OccupancyGrid(const std::string& path);
    ~OccupancyGrid(){};
    bool is_path_free(double x1_meters, double y1_meters, double x2_meters, double y2_meters);
    double width_meters() { return (double)n_columns / (COLUMN_CELLS_PER_METER); };
    double height_meters() { return (double)n_rows / (ROW_CELLS_PER_METER); };
};
