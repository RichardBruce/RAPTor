#include "checker_board_mapper.h"


namespace raptor_raytracer
{
/***********************************************************
    Overloaded virtual texture mapping function. Takes the 
    destination of the ray (query point) and returns the colour
    at the location and an alpha value
************************************************************/
float checker_board_mapper::run_procedure(ext_colour_t *const c, const point_t &dst, const point_t &dir, const point_t &n) const
{
    const point_t grid_pos((dst - _cnt) / _grid_size);
    if ((static_cast<int>(grid_pos.x) ^ static_cast<int>(grid_pos.z) ^ static_cast<int>(grid_pos.z)) & 0x1)
    {
        (*c) = _rgb;
    }

    return _op;
}
}; /* namespace raptor_raytracer */
