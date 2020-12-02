#pragma once

/* Standard headers */

/* Boost headers */
#include "boost/serialization/access.hpp"
#include "boost/shared_array.hpp"

/* Common headers */
#include "common.h"
#include "point_t.h"

namespace raptor_raytracer
{
/* Forward delcarations */
class ext_colour_t;

enum class falloff_type_t : char { cubic = 0, spherical = 1, linear_x = 2, linear_y = 3, linear_z = 4, none = 5 };

/* Pure virtual class for material data and shading */
class mapper_falloff
{
    public :
        mapper_falloff(const point_t<> &cnt, const point_t<> &grad, const falloff_type_t type) : 
        _cnt(cnt), _grad(grad), _type(type) {  };

        void falloff(ext_colour_t *const c, const point_t<> &dst) const;

    private :
        friend class boost::serialization::access;
        
        const point_t<>         _cnt;
        const point_t<>         _grad;
        const falloff_type_t    _type;
};
}; /* namespace raptor_raytracer */

namespace boost { 
namespace serialization {
template<class Archive>
inline void save_construct_data(Archive & ar, const raptor_raytracer::mapper_falloff *t, const unsigned int file_version)
{
    ar << t->_cnt;
    ar << t->_grad;
    ar << t->_type;
}

template<class Archive>
inline void load_construct_data(Archive & ar, raptor_raytracer::mapper_falloff *t, const unsigned int file_version)
{
    /* Retreive the fields */
    point_t<>                           cnt;
    point_t<>                           grad;
    raptor_raytracer::falloff_type_t    type;
    ar >> cnt;
    ar >> grad;
    ar >> type;
    
    /* Use plaement new to create the class */
    ::new(t)raptor_raytracer::mapper_falloff(cnt, grad, type);
}
}; /* namespace serialization */
}; /* namespace boost */
