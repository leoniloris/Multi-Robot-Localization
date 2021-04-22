#pragma once

#include <vector>
#include <stdint.h>
#include <string>

class OccupancyGrid {
    std::vector<std::vector<uint8_t>> grid;
    uint16_t n_columns;
    uint16_t n_rows;

   public:
    OccupancyGrid(const std::string& path);
    ~OccupancyGrid(){};
    bool is_path_free(double x1, double y1, double x2, double y2);

};
