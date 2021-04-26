#pragma once

#include <cmath>

#include "particle_filter.h"

#define L2_DISTANCE(a, b) (sqrt(pow((a), 2) + pow((b), 2)))
#define INNER_PRODUCT(dx1, dy1, dx2, dy2) (dx1 * dy1 + dx2 * dy2)
#define GAUSSIAN_LIKELIHOOD(mean, std, x) exp(-(pow((mean) - (x), 2)) / pow((std), 2) / 2.0)
