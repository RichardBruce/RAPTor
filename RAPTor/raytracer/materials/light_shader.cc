
#include "light_shader.h"


namespace raptor_raytracer
{
void light_shader::shade(const ray_trace_engine &r, ray &i, const line &n, const hit_t h, ext_colour_t *const c) const
{
    *c = this->rgb;
}
}; /* namespace raptor_raytracer */
