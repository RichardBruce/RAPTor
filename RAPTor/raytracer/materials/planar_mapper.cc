#include "planar_mapper.h"

#include "ext_colour_t.h"


namespace raptor_raytracer
{
void planar_mapper::texture_coordinates(float *const u_co, float *const v_co, const point_t<> &dst, const point_t<> &n) const
{
    /* Find the u,v co-ordinates of the hit */
    const point_t<> dist((dst - _c) + (((_s * _u) + (_s * _v)) * 0.5f));
    
    /* Scale */
    *u_co = (dot_product(dist, _u)) * (static_cast<float>(_w) / std::fabs(dot_product(_s, _u)));
    *v_co = (dot_product(dist, _v)) * (static_cast<float>(_h) / std::fabs(dot_product(_s, _v)));
}
}; /* namespace raptor_raytracer */
