#ifndef __GRADIENT_MAPPER_H__
#define __GRADIENT_MAPPER_H__

/* Standard headers */

/* Boost headers */
#include "boost/serialization/access.hpp"
#include "boost/shared_array.hpp"

/* Common headers */
#include "common.h"
#include "logging.h"
#include "point_t.h"
#include "ext_colour_t.h"

/* Ray tracer headers */
#include "texture_mapper.h"


namespace raptor_raytracer
{
enum class grad_interpolation_t : char { line = 0, spline = 1, step = 2 };
enum class grad_of_t : char { previous_layer = 0, bump = 1, slope = 2, incidence_angle = 3, light_incidence = 4, distance_to_camera = 5, distance_to_object = 6, x_distance_to_object = 7, y_distance_to_object = 8, z_distance_to_object = 9, weight_map = 10 };

/* Forward declarations */
class ray;

/* Class to represent textures that graients */
class gradient_mapper : public texture_mapper
{
    public :
        gradient_mapper(const std::vector<float> &keys, const std::vector<std::pair<ext_colour_t, float>> &value, const std::vector<grad_interpolation_t> &grads, const grad_of_t grad_of) : 
            texture_mapper(), _keys(keys), _value(value), _grads(grads), _grad_of(grad_of) { }

        virtual ~gradient_mapper() { };

        float texture_map(const ray &r, ext_colour_t *const c, const point_t &n, const point_t &vt) const override;
        float texture_map_monochrome(const ray &r, point_t *const p, const point_t &n, const point_t &vt, const int x_off = 0, const int y_off = 0) const override;

        /* Texture mapping function. Takes the destination and direction 
           of the incident ray and returns either a fp_t (alpha, kd, ks, t, r....), a colour (rgb) or both */
        float sample_texture(ext_colour_t *const c, const point_t &dst, const point_t &n, const point_t &vt) const
        {
            assert(false);
            return 0.0f;
        }

        float sample_texture_monochrome(point_t *const c, const point_t &dst, const point_t &n, const point_t &vt, const int x_off, const int y_off) const
        {
            assert(false);
            return 0.0f;
        }

    private :
        friend class boost::serialization::access;
        
        /* Serialisation left blank and left to save_construct_data and load_construct_data */
        template<class Archive>
        void serialize(Archive &ar, const unsigned int version) { }

        const std::vector<float>                            _keys;
        const std::vector<std::pair<ext_colour_t, float>>   _value;
        const std::vector<grad_interpolation_t>             _grads;
        const grad_of_t                                     _grad_of;
};
}; /* namespace raptor_raytracer */

namespace boost { 
namespace serialization {
template<class Archive>
inline void save_construct_data(Archive & ar, const raptor_raytracer::gradient_mapper *t, const unsigned int file_version)
{
    // ar << t->falloff();
    ar << t->_keys;
    ar << t->_value;
    ar << t->_grads;
    ar << t->_grad_of;
}

template<class Archive>
inline void load_construct_data(Archive & ar, raptor_raytracer::gradient_mapper *t, const unsigned int file_version)
{
    /* Retreive the fields */
    using raptor_raytracer::ext_colour_t;
    using raptor_raytracer::grad_interpolation_t;
    using raptor_raytracer::grad_of_t;
    std::vector<float>                          keys;
    std::vector<std::pair<ext_colour_t, float>> value;
    std::vector<grad_interpolation_t>           grads;
    grad_of_t                                  grad_of;
    // ar >> falloff;
    ar >> keys;
    ar >> value;
    ar >> grads;
    ar >> grad_of;
    
    /* Use plaement new to create the class */
    ::new(t)raptor_raytracer::gradient_mapper(keys, value, grads, grad_of);
    // t->falloff(falloff);
}
}; /* namespace serialization */
}; /* namespace boost */

#endif /* #ifndef __GRADIENT_MAPPER_H__ */
