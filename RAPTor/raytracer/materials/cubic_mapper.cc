#include "cubic_mapper.h"

#include "ext_colour_t.h"


namespace raptor_raytracer
{
void cubic_mapper::texture_coordinates(float *const u_co, float *const v_co, const point_t &dst, const point_t &n) const
{
    /* Find the u,v co-ordinates of the hit */
    const point_t dist((dst - _c) + (((_s * _u) + (_s * _v)) * 0.5f));

    /* Work out manhattan distances */
    const float dist_u = dot_product(dist, _u) + dot_product(dist, _n);
    const float dist_v = dot_product(dist, _v);
    
    /* Scale */
    *u_co = dist_u * (static_cast<float>(_w) / fabs(dot_product(_s, _u)));
    *v_co = dist_v * (static_cast<float>(_h) / fabs(dot_product(_s, _v)));
}
}; /* namespace raptor_raytracer */
