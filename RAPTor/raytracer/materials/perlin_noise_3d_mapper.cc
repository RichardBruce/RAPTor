#include "perlin_noise_3d_mapper.h"


namespace raptor_raytracer
{
/***********************************************************
  Overloaded virtual texture mapping function. Takes the 
  destination of the ray (query point) and returns the colour
  at the location and an alpha value
************************************************************/
float perlin_noise_3d_mapper::sample_texture(ext_colour_t *const c, const point_t &dst, const point_t &n, const point_t &vt) const
{
    const float x_co    = dst.x;
    const float y_co    = dst.y;
    const float z_co    = dst.z;
    float total         = 0.0f;
    float frequency     = 1.0f / this->z;
    float amplitude     = 1.0f;

    /* Process each octave */
    for (int i = 0; i < this->o; ++i)
    {
        total += this->perlin.interpolated_noise(x_co * frequency, y_co * frequency, z_co * frequency) * amplitude;
        frequency *= 2.0f;
        amplitude *= this->p;
    }

    /*  Move total into the range [0-2] and scale the colour */
    total++;
    (*c) = ext_colour_t((this->rgb.r * total), (this->rgb.g * total), (this->rgb.b * total));

    /* Return if the texture map generated a bright enough colour */
    if (total > 1.0f)
    {
        return 0.0f;
    }
    else
    {
        return this->op;
    }
}
}; /* namespace raptor_raytracer */
