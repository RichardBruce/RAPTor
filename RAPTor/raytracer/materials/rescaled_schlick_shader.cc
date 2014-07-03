
#include "rescaled_schlick_shader.h"


namespace raptor_raytracer
{
void rescaled_schlick_shader::shade(const ray &i, const list<ray> &l, const line &n, ext_colour_t *const c, ray *const rl, ray *const rr) const
{
    /* For each light */
    for(list<ray>::const_iterator iter=l.begin(); iter!=l.end(); ++iter)
    {
        c->r += 0.0;
        c->g += 0.0;
        c->b += 0.0;
    }
}
}; /* namespace raptor_raytracer */
