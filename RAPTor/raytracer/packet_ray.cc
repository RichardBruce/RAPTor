#include "packet_ray.h"

namespace raptor_raytracer
{
#ifdef SIMD_PACKET_TRACING

const packet_ray_to_co_ordinate packet_ray_to_co_ordinate_lut;
const packet_ray_to_pixel       packet_ray_to_pixel_lut;

#endif /* #ifdef SIMD_PACKET_TRACING */
}; /* namespace raptor_raytracer */
