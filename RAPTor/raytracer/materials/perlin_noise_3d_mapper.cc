#include "perlin_noise_3d_mapper.h"


/***********************************************************
  Overloaded virtual texture mapping function. Takes the 
  destination of the ray (query point) and returns the colour
  at the location and an alpha value
************************************************************/
fp_t perlin_noise_3d_mapper::texture_map(const point_t &dst, const point_t &dir, ext_colour_t *const c, const point_t &vt) const
{
    fp_t x_co       = dst.x;
    fp_t y_co       = dst.y;
    fp_t z_co       = dst.z;
    fp_t total      = 0.0;
    fp_t frequency  = 1.0 / this->z;
    fp_t amplitude  = 1.0;

    /* Process each octave */
    for (int i = 0; i < this->o; ++i)
    {
        total += this->perlin.interpolated_noise(x_co * frequency, y_co * frequency, z_co * frequency) * amplitude;
        frequency *= (fp_t)2.0;
        amplitude *= this->p;
    }

    /*  Move total into the range [0-2] and scale the colour */
    total++;
    (*c) = ext_colour_t((this->rgb.r * total), (this->rgb.g * total), (this->rgb.b * total));
    
    /* Return if the texture map generated a bright enough colour */
    if (total > (fp_t)1.0)
    {
        return (fp_t)0.0;
    }
    else
    {
        return (fp_t)this->op;
    }
}
