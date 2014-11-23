#ifndef __PERLIN_NOISE_3D_H__
#define __PERLIN_NOISE_3D_H__

/* Common headers */
#include "common.h"

/* Class for generating perlin noise */
class perlin_noise_3d
{
    public :
        /* CTOR */
        perlin_noise_3d(const int s) : _s(s) {  };

        /* Allow default Copy CTOR, assignment and DTOR */

        /* Method to generate noise */
        float interpolated_noise(const float x, const float y, const float z) const;

        /* Getters */
        int seed() const { return _s; }

    private :
        float noise(const int x, const int y, const int z)              const;
        float smooth_noise(const int x, const int y, const int z)       const;
        float interpolate(const float a, const float b, const float x)  const;

        const int _s;   /* Random seed  */
};

#endif /* #ifndef __PERLIN_NOISE_3D_H__ */
