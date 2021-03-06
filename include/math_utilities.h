#pragma once

#include <cmath>

#include "particle_filter.h"

#define EPS 0.00000001

#define L2_DISTANCE(a, b) (sqrt(pow((a), 2) + pow((b), 2)))
#define INNER_PRODUCT(dx1, dy1, dx2, dy2) (dx1 * dy1 + dx2 * dy2)
#define GAUSSIAN_LIKELIHOOD(mean, std, x) exp(-(pow((mean) - (x), 2)) / pow((std), 2) / 2.0)
#define PERIODIC_GAUSSIAN_LIKELIHOOD(mean, std, x, period) \
    (GAUSSIAN_LIKELIHOOD((mean-(period)), std, x) +\
     GAUSSIAN_LIKELIHOOD(mean, std, x) +\
     GAUSSIAN_LIKELIHOOD((mean+(period)), std, x))
