#ifndef __COLLISION_INFO_H__
#define __COLLISION_INFO_H__

/* Standard headers */
#include <limits>

/* Common headers */
#include "point_t.h"

/* Physics headers */
#include "simplex.h"
#include "physics_object.h"


namespace raptor_physics
{
/* Class to hold information about a collision */
class collision_info
{
    public :
        /* CTOR */
        collision_info(simplex *s, const simplex &other_s, const float t, const collision_t type)
            : _s(s), 
              _other_s(&other_s),
              _t(t), 
              _last_t(t),
              _type(type),
              _repeat(0) {  };

        /* Update to the latest collision test */
        collision_info& update(simplex *s, const simplex &other_s, const float t, const collision_t type) 
        {
            _s.reset(s);
            _other_s = &other_s;

            if (fabs(t - _last_t) > raptor_physics::EPSILON)
            {
                _repeat = 0;
            }

            _t = t;
            _last_t = t;
            _type = type;

            return *this;
        }

        /* Update after a successful retest */
        collision_info& successful_retest_update(simplex *s, const simplex &other_s) 
        {
            _s.reset(s);
            _other_s = &other_s;
            _type = to_certain(_type);
            return *this;
        }

        /* Set the collision time to have never happened */
        collision_info& void_collision()
        {
            _t = std::numeric_limits<float>::max();
            _type = NO_COLLISION;
            return *this;
        }

        point_t get_normal_of_collision() const
        {
            return is_uncertain(_type) ? point_t(0.0f, 0.0f, 0.0f) : _s->normal_of_impact(*_other_s);
        }

        point_t get_point_of_collision() const
        {
            return is_uncertain(_type) ? point_t(0.0f, 0.0f, 0.0f) : _s->center_of_impact(*_other_s, get_normal_of_collision());
        }

        bool switch_to_sliding()
        {
            if (++_repeat > 1)
            {
                _type = SLIDING_COLLISION;
                return true;
            }
            else
            {
                return false;
            }
        }


        /* Access functions */
        const std::unique_ptr<simplex>& get_simplex() const { return _s;    }
        float                           get_time()    const { return _t;    }
        collision_t                     get_type()    const { return _type; }
        
    private :
        std::unique_ptr<simplex>    _s;         /* The final simplex for this side of collision detection               */
        const simplex *             _other_s;   /* The final simplex for the other side of collision detection          */
        float                       _t;         /* The time of the collision                                            */
        float                       _last_t;    /* The last time seen, not cleared by voiding                           */
        collision_t                 _type;      /* The type of collision                                                */
        int                         _repeat;    /* The number of times this pair has collided without time advancing    */
};
}; /* namespace raptor_physics */

#endif /* #define __COLLISION_INFO_H__ */
