#pragma once

#include <stdint.h>

#include <string>
#include <vector>

class OccupancyGrid {
    std::vector<std::vector<uint8_t>> grid;

   public:
    OccupancyGrid(const std::string& path);
    ~OccupancyGrid(){};
    bool is_path_free(double x1_meters, double y1_meters, double x2_meters, double y2_meters);

    uint16_t n_columns;
    uint16_t n_rows;
};
