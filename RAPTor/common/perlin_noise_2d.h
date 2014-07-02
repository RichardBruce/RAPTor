#ifndef __PERLIN_NOISE_2D_H__
#define __PERLIN_NOISE_2D_H__

/* Common headers */
#include "common.h"

/* Class for generating perlin noise */
class perlin_noise_2d
{
    public :
        /* CTOR */
        perlin_noise_2d(const int s) : _s(s) { };

        /* Allow default Copy CTOR, assignment and DTOR */

        /* Method to generate noise */
        fp_t interpolated_noise(const fp_t x, const fp_t y) const;

        /* Getters */
        int seed() const { return _s; }

    private :
        fp_t noise(const int x, const int y)                        const;
        fp_t smooth_noise(const int x, const int y)                 const;
        fp_t interpolate(const fp_t a, const fp_t b, const fp_t x)  const;

        const int _s;   /* Random seed  */
};

#endif /* #ifndef __PERLIN_NOISE_2D_H__ */
