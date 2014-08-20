#include "perlin_noise_2d_mapper.h"


namespace raptor_raytracer
{
void perlin_noise_2d_mapper::shade(const ray_trace_engine &r, ray &i, const line &n, const hit_t h, ext_colour_t *const c) const
{
    fp_t x = i.get_x1();
    fp_t y = i.get_y1();
    fp_t total = 0.0;
    fp_t frequency = 1.0;
    fp_t amplitude = 1.0;

    for (int i = 0; i < this->o; ++i)
    {
        total = total + this->perlin.interpolated_noise(x * frequency / this->z, y * frequency / this->z) * amplitude;
        frequency *= 2.0;
        amplitude *= this->p;
    }

    (*c) = ext_colour_t((this->rgb.r * (total + 1.0) * 127.5), (this->rgb.g * (total + 1.0) * 127.5), (this->rgb.b * (total + 1.0) * 127.5));
}


//void perlin_noise_2d_mapper::Generate()
//{    
//    for (unsigned int y = 0; y < this->h; ++y)
//    {
//        for (unsigned int x = 0; x < this->w; ++x)
//        {
//            fp_t total = 0;
//            fp_t frequency = 1;
//            fp_t amplitude = 1;
//
//            for (int i = 0; i < this->o; ++i)
//            {
//                total = total + InterpolatedNoise(x * frequency / this->z, y * frequency / this->z) * amplitude;
//                frequency *= 2;
//                amplitude *= this->p;
//            }
//
//            this->map_data[y * this->w + x] = (255 << 24) | ((unsigned char) (this->rgb.r * (total + 1) * 127.5) << 16) | ((unsigned char) (this->rgb.g * (total + 1) * 127.5) << 8) | 
//                (unsigned char) (this->rgb.b * (total + 1) * 127.5);
//        }
//    }
//}
//
//void perlin_noise_2d_mapper::GenerateNormalized()
//{    
//    fp_t min = 0;
//    fp_t max = 0;
//    fp_t maxColorMultiplier;
//    fp_t * pDataFloat = new fp_t[this->w * this->h];
//
//    //Generate raw fp_t data
//    for (unsigned int y = 0; y < this->h; ++y)
//    {
//        for (unsigned int x = 0; x < this->w; ++x)
//        {
//            fp_t total = 0;
//            fp_t frequency = 1;
//            fp_t amplitude = 1;
//
//            for (int i = 0; i < this->o; ++i)
//            {
//                total = total + InterpolatedNoise(x * frequency / this->z, y * frequency / this->z) * amplitude;
//                frequency *= 2;
//                amplitude *= this->p;
//            }
//
//            pDataFloat[y * this->w + x] = total;
//
//            min = total < min ? total : min;
//            max = total > max ? total : max;
//        }
//    }
//
//    //Normalize color multipliers
//    maxColorMultiplier = this->rgb.r > this->rgb.g ? this->rgb.r : this->rgb.g;
//    maxColorMultiplier = this->rgb.b > maxColorMultiplier ? this->rgb.b : maxColorMultiplier;
//    this->rgb.r /= maxColorMultiplier;
//    this->rgb.g /= maxColorMultiplier;
//    this->rgb.b /= maxColorMultiplier;
//
//    //Normalize raw floats, factor in color multipliers, and convert to bitmap color format
//    for (unsigned int i = 0; i < this->w * this->h; ++i)
//    {
//            this->map_data[i] = (255 << 24) | ((unsigned char) (this->rgb.r * ((pDataFloat[i] - min) / (max - min)) * 255) << 16) | 
//                ((unsigned char) (this->rgb.g * ((pDataFloat[i] - min) / (max - min)) * 255) << 8) | (unsigned char) (this->rgb.b * ((pDataFloat[i] - min) / (max - min)) * 255);
//    }
//
//    delete [] pDataFloat;
//}
}; /* namespace raptor_raytracer */
