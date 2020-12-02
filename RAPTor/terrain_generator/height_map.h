#pragma once

/* Common headers */
#include "common.h"
#include "point_t.h"

namespace raptor_terrain
{
template<class NG>
class height_map
{
    public :
        height_map(const NG &noise, const float a, const float z, const float p, const int o, const int x, const int y)
        : _noise(noise), _a(a), _z(z), _p(p), _o(o), _x(x), _y(y) {  };

        point_t<>* generate(const float xs, const float ys) const
        {
            point_t<> *verts = new point_t<> [this->_x * this->_y];
            for (int i = 0; i < this->_x; ++i)
            {
                /* For each cell */
                const float x = static_cast<float>(i) * this->_z;
                for (int j = 0; j < this->_y; ++j)
                {
                    const float y = static_cast<float>(j) * this->_z;

                    /* Generate height of different scales */
                    float frequency  = 1.0f;
                    float amplitude  = this->_a;
                    float total      = 0.0f;
                    for (int k = 0; k < this->_o; ++k)
                    {
                        total += (this->_noise.interpolated_noise(x * frequency, y * frequency) * amplitude);
                        frequency *= 2.0f;
                        amplitude *= this->_p;
                    }

                    verts[(i * this->_y) + j] = point_t<>(i * xs, total, j * ys);
                }
            }

            return verts;
        }

    private :
        const NG &  _noise;     /* Noise generator      */
        const float _a;         /* Amplitude            */
        const float _z;         /* Zoom                 */
        const float _p;         /* Persistence          */
        const int   _o;         /* Octaves              */
        const int   _x;         /* Number of x cells    */
        const int   _y;         /* Number of y cells    */
};
}; /* namespace raptor_terrain */
