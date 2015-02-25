#include "checker_board_mapper.h"


namespace raptor_raytracer
{
/***********************************************************
    Overloaded virtual texture mapping function. Takes the 
    destination of the ray (query point) and returns the colour
    at the location and an alpha value
************************************************************/
float checker_board_mapper::sample_texture(ext_colour_t *const c, const point_t &dst, const point_t &n, const point_t &vt) const
{
    const point_t grid_pos((dst - cnt) / grid_size);
    if ((static_cast<int>(grid_pos.x) ^ static_cast<int>(grid_pos.z) ^ static_cast<int>(grid_pos.z)) & 0x1)
    {
        (*c) = rgb;
    }

    return this->op;
}

float checker_board_mapper::sample_texture_monochrome(point_t *const p, const point_t &dst, const point_t &n, const point_t &vt, const int x_off, const int y_off) const
{
    // const point_t grid_pos((dst - cnt) / grid_size);
    // if ((static_cast<int>(grid_pos.x) ^ static_cast<int>(grid_pos.z) ^ static_cast<int>(grid_pos.z)) & 0x1)
    // {
    //     (*p) = point_t(, , rgb.r);
    // }

    return this->op;
}
}; /* namespace raptor_raytracer */
