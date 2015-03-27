/* Standard headers */

/* Boost headers */

/* Common headers */
#include "ext_colour_t.h"

/* Raytracer headers */
#include "mapper_falloff.h"


namespace raptor_raytracer
{
void mapper_falloff::falloff(ext_colour_t *const c, const point_t &dst) const
{
    switch (_type)
    {
        case falloff_type_t::cubic :
            {
                const point_t dist(dst - _cnt);
                const point_t fall(dist * _grad);
                const float all_fall = (std::fabs(fall.x) + std::fabs(fall.y) + std::fabs(fall.z)) * 255.0f;
                (*c).r = std::max(0.0f, (*c).r - all_fall);
                (*c).g = std::max(0.0f, (*c).g - all_fall);
                (*c).b = std::max(0.0f, (*c).b - all_fall);
            }
            break;
        case falloff_type_t::spherical :
            {
                const float dist = magnitude(dst - _cnt);
                const point_t fall(dist * _grad);
                const float all_fall = std::max(0.0f, 1.0f - fall.x) * std::max(0.0f, 1.0f - fall.y) * std::max(0.0f, 1.0f - fall.z);
                (*c) *= all_fall;
            }
            break;
        case falloff_type_t::linear_x :
            {
                const float dist = dst.x - _cnt.x;
                const float fall = dist * _grad.x;
                (*c) *= std::max(0.0f, 1.0f - fall);
            }
            break;
        case falloff_type_t::linear_y :
            {
                const float dist = dst.y - _cnt.y;
                const float fall = dist * _grad.y;
                (*c) *= std::max(0.0f, 1.0f - fall);
            }
            break;
        case falloff_type_t::linear_z :
            {
                const float dist = dst.z - _cnt.z;
                const float fall = dist * _grad.z;
                (*c) *= std::max(0.0f, 1.0f - fall);
            }
            break;
        case falloff_type_t::none :
            break;
    }
}
}; /* namespace raptor_raytracer */
