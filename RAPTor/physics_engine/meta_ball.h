#pragma once

/* Standard headers */

/* Boost headers */

/* Common headers */
#include "point_t.h"



namespace raptor_physics
{
class meta_ball
{
    public :
        meta_ball(const point_t<> &center, const float strength, const bool negate) : 
            _center(center), _strength(strength), _negate(negate)
        {  }

        float value(const point_t<> &pos) const
        {
            const point_t<> dist(pos - _center);
            const float r_sq = dot_product(dist, dist) / (_strength * _strength);
            if (r_sq > 1.0f)
            {
                return 0.0f;
            }

            const float r = std::sqrt(r_sq);
            const float value =  1.0f - (r * r * r * (r * (r * 6.0f - 15.0f) + 10.0f));
            return _negate ? -value : value;
        }

        point_t<> normal(const point_t<> &pos) const
        {
            return normalise(pos - _center);
        }

        const point_t<>&  center() const { return _center;    }
        float           radius() const { return _strength;  }


    private :
        const point_t<> _center;
        const float     _strength;
        const bool      _negate;
};
}; /* namespace raptor_physics */
