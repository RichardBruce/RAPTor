#ifndef __TRACKING_INFO_H__
#define __TRACKING_INFO_H__

/* Standard headers */
#include <map>

/* Boost headers */
#include "boost/noncopyable.hpp"

/* Physics headers */
#include "collision_info.h"

namespace raptor_physics
{
/* Forward declaratons */
class physics_object;

/* Class to track collisions with an object */
template<class PO>
class tracking_info : private boost::noncopyable
{
    public :
        /* Convinience typedef */
        typedef typename std::map<PO*, collision_info*>::const_iterator collision_info_const_iter;

        /* CTOR */
        tracking_info(PO *vg, simplex *s, const simplex &other_s, const float t, const collision_t type) 
            : _collisions(new std::map<PO*, collision_info*>({ {vg, new collision_info(s, other_s, t, type)} })),
              _collide(vg),
              _time(t),
              _type(type) {  };

        /* DTOR */
        ~tracking_info()
        {
            for (auto& p : (*_collisions))
            {
                delete p.second;
            }
            delete _collisions;
        }

        /* Update the collision info with another object */
        tracking_info& update(PO *vg, simplex *s, const simplex &other_s, const float t, const collision_t type)
        {
            /* Insert or update the collision cache */
            auto collision = _collisions->find(vg);
            if (collision != _collisions->end())
            {
                collision->second->update(s, other_s, t, type);
            }
            else
            {
                _collisions->insert({ vg, new collision_info(s, other_s, t, type) });
            }

            /* Track the first impact */
            if ((_collide == nullptr) || (t < _time))
            {
                _collide = vg;
                _time = t;
                _type = type;
            }
            else if (_collide == vg)
            {
                find_first_collision();
            }

            return *this;
        }

        /* Update the collision info after a successful retest */
        tracking_info& successful_retest_update(PO *vg, simplex *s, const simplex &other_s)
        {
            auto collision = _collisions->find(vg);

            /* If this fails it is not a retest */
            assert (collision != _collisions->end());

            collision->second->successful_retest_update(s, other_s);
            _type = to_certain(_type);

            /* No collision times changed so no need to update the first impact */

            return *this;
        }

        /* Void any collision with vg */
        tracking_info& void_collision(PO *const vg)
        {
            /* Reset the collision. */
            auto col = _collisions->find(vg);
            if (col != _collisions->end())
            {
                col->second->void_collision();
            }

            /* Find a new first collision */
            if (_collide == vg)
            {
                find_first_collision();
            }

            return *this;
        }

        /* Access functions */
        collision_t get_first_collision_type()  const { return _type;       }
        float       get_first_collision_time()  const { return _time;       }
        PO*         get_first_collision()       const { return _collide;    }

        /* Get the collision information with vg */
        collision_info* operator[](PO *const key)
        {
            auto coll_iter = _collisions->find(key);
            if (coll_iter == _collisions->end())
            {
                return nullptr;
            }
            else
            {
                return coll_iter->second;
            }
        }

        collision_info_const_iter begin()   const { return _collisions->cbegin();   }
        collision_info_const_iter end()     const { return _collisions->cend();     }

    private :
        const tracking_info& find_first_collision()
        {
            _collide = nullptr;
            _time = std::numeric_limits<float>::max();
            _type = collision_t::NO_COLLISION;
            for (auto& c : (*_collisions))
            {
                float obj_time = c.second->get_time();
                if (obj_time < _time)
                {
                    _collide = c.first;
                    _time = obj_time;
                    _type = c.second->get_type();
                }
            }

            return *this;
        }

        std::map<PO*, collision_info*> *    _collisions;    /* A cache of all collision with this object    */
        PO                             *    _collide;       /* The first thing to be hit                    */
        float                               _time;          /* The time of the first collision              */
        collision_t                         _type;          /* The type of collision                        */
};
}; /* namespace raptor_physics */

#endif /* #define __TRACKING_INFO_H__ */
