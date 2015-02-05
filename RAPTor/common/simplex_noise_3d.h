#ifndef __SIMPLEX_NOISE_3D_H__
#define __SIMPLEX_NOISE_3D_H__

/* Common headers */
#include "common.h"

/* Class for generating perlin noise */
class simplex_noise_3d
{
    public :
        /* Allow default CTOR Copy CTOR, assignment and DTOR */

        /* Method to generate noise */
        float interpolated_noise(const float x, const float y, const float z) const;
};

#endif /* #ifndef __SIMPLEX_NOISE_3D_H__ */
