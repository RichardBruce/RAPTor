#include "perlin_noise_2d.h"

float perlin_noise_2d::noise(const int x, const int y) const
{
    int n = x + y * 57 + this->_s;
    n = (n << 13) ^ n;
    return (1.0f - ((n * (n * n * 15731 + 789221) + 1376312589) & 0x7fffffff) / 1073741824.0f);
    //return value is always in range [-1.0, 1.0]
}


float perlin_noise_2d::smooth_noise(const int x, const int y) const
{
    const float corners = (noise(x-1, y-1) + noise(x+1, y-1) + noise(x-1, y+1) + noise(x+1, y+1)) / 16.0f;
    const float sides   = (noise(x-1, y)   + noise(x+1, y)   + noise(x,   y-1) + noise(x, y+1))   / 8.0f;
    const float center  =  noise(x, y ) / 4.0f;
    return corners + sides + center;
}


float perlin_noise_2d::interpolate(const float a, const float b, const float x) const
{
    const float ft = x * PI;
    const float f = (1.0f - cos(ft)) * 0.5f;

    return  a * (1.0f - f) + b * f;
}


float perlin_noise_2d::interpolated_noise(const float x, const float y) const
{
    const int wx    = (int)(static_cast<float>((int)x) + 0.5f);
    const float px  = x - wx;

    const int wy    = (int)(static_cast<float>((int)y) + 0.5f);
    const float py  = y - wy;

    const float v1 = smooth_noise(wx,     wy);
    const float v2 = smooth_noise(wx + 1, wy);
    const float v3 = smooth_noise(wx,     wy + 1);
    const float v4 = smooth_noise(wx + 1, wy + 1);

    const float i1 = interpolate(v1, v2, px);
    const float i2 = interpolate(v3, v4, px);

    return interpolate(i1, i2, py);
}
