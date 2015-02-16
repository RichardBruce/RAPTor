#ifndef __SECONDARY_RAY_DATA_H__
#define __SECONDARY_RAY_DATA_H__

/* Standard headers */

/* Boost headers */

/* Common headers */

/* Ray tracer headers */
#include "ext_colour_t.h"
#include "ray.h"


namespace raptor_raytracer
{
class secondary_ray_data
{
    public :
        secondary_ray_data() :
            _nr(0.0f) {  }

        /* Access functions */
        /* Getters */
        ray *           rays()              { return &_rays[0];     }
        ext_colour_t *  colours()           { return &_colours[0];  }
        float           number()    const   { return _nr;           }
        float           value()     const   { return _value;        }

        ext_colour_t average_colour() const
        {
            ext_colour_t avg;
            for (int i = 0; i < static_cast<int>(_nr); ++i)
            {
                avg += _colours[i];
            }

            return avg / _nr;
        }

        /* Setters */
        secondary_ray_data& colours(ext_colour_t *const colours)
        {
            _colours = colours;
            return *this;
        }

        secondary_ray_data& number(const float nr)
        {
            _nr = nr;
            return *this;
        }

        secondary_ray_data& value(const float value)
        {
            _value = value;
            return *this;
        }

    private :
        ray             _rays[MAX_SECONDARY_RAYS];
        ext_colour_t  * _colours;
        float           _nr;
        float           _value;
};
}; /* namespace raptor_raytracer */

#endif /* #ifndef __SECONDARY_RAY_DATA_H__ */
