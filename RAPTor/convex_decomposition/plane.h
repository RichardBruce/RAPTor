#pragma once

/* Standard headers */
#
/* Common headers */
#include "point_t.h"

/* Convex decomposition headers */


namespace raptor_convex_decomposition
{
enum class axis_t { x_axis = 0, y_axis = 1, z_axis = 2 };


struct plane
{
    /* Default Ctor */
    plane() = default;

    /* Ctor */
    plane(const point_t &n, const float p, const axis_t major_axis) : n(n), p(p), major_axis(major_axis) {  }

    float distance(const point_t &pt) const
    {
        return dot_product(pt, n) - p;
    }

    point_t n;
    float   p;
    axis_t  major_axis;
};
}; /* namespace raptor_convex_decomposition */
