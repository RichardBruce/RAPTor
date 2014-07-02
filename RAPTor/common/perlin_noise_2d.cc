#include "perlin_noise_2d.h"

fp_t perlin_noise_2d::noise(const int x, const int y) const
{
    int n = x + y * 57 + this->_s;
    n = (n << 13) ^ n;
    return (fp_t) (1.0 - ((n * (n * n * 15731 + 789221) + 1376312589) & 0x7fffffff) / 1073741824.0);
    //return value is always in range [-1.0, 1.0]
}


fp_t perlin_noise_2d::smooth_noise(const int x, const int y) const
{
    const fp_t corners = (noise(x-1, y-1) + noise(x+1, y-1) + noise(x-1, y+1) + noise(x+1, y+1)) / 16.0;
    const fp_t sides   = (noise(x-1, y)   + noise(x+1, y)   + noise(x,   y-1) + noise(x, y+1))   / 8.0;
    const fp_t center  =  noise(x, y ) / 4.0;
    return corners + sides + center;
}


fp_t perlin_noise_2d::interpolate(const fp_t a, const fp_t b, const fp_t x) const
{
    const fp_t ft = x * PI;
    const fp_t f = (1.0 - cos(ft)) * 0.5;

    return  a * (1.0 - f) + b * f;
}


fp_t perlin_noise_2d::interpolated_noise(const fp_t x, const fp_t y) const
{
    const int wx  = (int)((float)((int)x) + 0.5);
    const fp_t px = x - wx;

    const int wy  = (int)((float)((int)y) + 0.5);
    const fp_t py = y - wy;

    const fp_t v1 = smooth_noise(wx,     wy);
    const fp_t v2 = smooth_noise(wx + 1, wy);
    const fp_t v3 = smooth_noise(wx,     wy + 1);
    const fp_t v4 = smooth_noise(wx + 1, wy + 1);

    const fp_t i1 = interpolate(v1, v2, px);
    const fp_t i2 = interpolate(v3, v4, px);

    return interpolate(i1, i2, py);
}
