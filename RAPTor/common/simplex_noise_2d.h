#ifndef __SIMPLEX_NOISE_2D_H__
#define __SIMPLEX_NOISE_2D_H__

/* Common headers */
#include "common.h"

/* Class for generating perlin noise */
class simplex_noise_2d
{
    public :
        /* Allow default CTOR Copy CTOR, assignment and DTOR */

        /* Method to generate noise */
        fp_t interpolated_noise(const fp_t x, const fp_t y) const;
};

#endif /* #ifndef __SIMPLEX_NOISE_2D_H__ */
