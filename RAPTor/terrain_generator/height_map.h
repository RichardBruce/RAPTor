#ifndef __HEIGHT_MAP_H__
#define __HEIGHT_MAP_H__

/* Common headers */
#include "common.h"
#include "point_t.h"

namespace raptor_terrain
{
template<class NG>
class height_map
{
    public :
        height_map(const NG &noise, const fp_t a, const fp_t z, const fp_t p, const int o, const int x, const int y)
        : _noise(noise), _a(a), _z(z), _p(p), _o(o), _x(x), _y(y) {  };

        point_t* generate(const fp_t xs, const fp_t ys) const
        {
            point_t *verts = new point_t [this->_x * this->_y];
            for (int i = 0; i < this->_x; ++i)
            {
                /* For each cell */
                const fp_t x = static_cast<fp_t>(i) * this->_z;
                for (int j = 0; j < this->_y; ++j)
                {
                    const fp_t y = static_cast<fp_t>(j) * this->_z;

                    /* Generate height of different scales */
                    fp_t frequency  = 1.0;
                    fp_t amplitude  = this->_a;
                    fp_t total      = 0.0;
                    for (int k = 0; k < this->_o; ++k)
                    {
                        total += (this->_noise.interpolated_noise(x * frequency, y * frequency) * amplitude);
                        frequency *= 2.0;
                        amplitude *= this->_p;
                    }

                    verts[(i * this->_y) + j] = point_t(i * xs, total, j * ys);
                }
            }

            return verts;
        }

    private :
        const NG &  _noise;     /* Noise generator      */
        const fp_t  _a;         /* Amplitude            */
        const fp_t  _z;         /* Zoom                 */
        const fp_t  _p;         /* Persistence          */
        const int   _o;         /* Octaves              */
        const int   _x;         /* Number of x cells    */
        const int   _y;         /* Number of y cells    */
};
}; /* namespace raptor_terrain */

#endif /* #ifndef __HEIGHT_MAP_H__ */
