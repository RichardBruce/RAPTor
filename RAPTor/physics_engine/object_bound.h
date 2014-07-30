#ifndef __OBJECT_BOUND_H__
#define __OBJECT_BOUND_H__

/* Boost headers */
#include "boost/noncopyable.hpp"

/* Common headers */
#include "common.h"


namespace raptor_physics
{
/* Forward declarations */
class physics_object;

/* Enumerate axes */
enum axis_t { X_AXIS = 0, Y_AXIS = 1, Z_AXIS = 2 };

/* Class to represent a bound of an object in an axis */
class object_bound : private boost::noncopyable
{
    public :
        object_bound(const physics_object *const object, const fp_t bound, const bool min)
        :  _object(object), _bound(bound), _index(-1), _min(min) { }

        /* Allow default DTOR */

        /* Access functions */
        object_bound& bound(const fp_t bound)
        {
            _bound = bound;
            return *this;
        }

        object_bound& index(const int index)
        {
            _index = index;
            return *this;
        }

        const physics_object *  object()    const { return _object; }
        fp_t                    bound()     const { return _bound;  }
        int                     index()     const { return _index;  }
        bool                    min()       const { return _min;    }

        /* Comparison functions */
        bool operator<(const object_bound &o) const { return _bound < o._bound; }

        bool compare_and_swap(object_bound *const o)
        {
            if (_bound < o->_bound)
            {
                std::swap(_index, o->_index);
                return true;
            }

            return false;
        }

    private :
        const physics_object *const _object;
        fp_t                        _bound;
        int                         _index;
        const bool                  _min;
};
}; /* namespace raptor_physics */

#endif /* #ifndef __OBJECT_BOUND_H__ */
