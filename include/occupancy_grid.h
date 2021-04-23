#pragma once

#include <stdint.h>

#include <string>
#include <vector>

// y (columns) scale: 7 [meters] per 273 [cells|columns]
#define COLUMN_CELLS_PER_METER (273.0 / 7.0)

// x (rows) scale: 3.5 [meters] per 198 [cells|rows]
#define ROW_CELLS_PER_METER (198.0 / 3.5)

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
