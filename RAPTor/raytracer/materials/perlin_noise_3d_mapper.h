#pragma once

/* Boost headers */
#include "boost/serialization/access.hpp"

/* Common headers */
#include "common.h"
#include "ext_colour_t.h"
#include "point_t.h"
#include "texture_mapper.h"
#include "perlin_noise_3d.h"


namespace raptor_raytracer
{
/* Pure virtual class for material data and shading */
class perlin_noise_3d_mapper : public procedural_texture_mapper
{
    public :
        perlin_noise_3d_mapper(const ext_colour_t &rgb, const float p, const float z, const int o, const int s, const float op = 1.0f, const float u_ps = 0.1f, const float v_ps = 0.1f) : 
            procedural_texture_mapper(u_ps, v_ps), _perlin(s), _rgb(rgb * 0.5f), _p(p), _z(z), _op(op), _o(o) {  };

        virtual ~perlin_noise_3d_mapper() { };

    protected :
        float run_procedure(ext_colour_t *const c, const point_t<> &dst, const point_t<> &dir, const point_t<> &n) const override;
        
    private :
        friend class boost::serialization::access;
        
        /* Serialisation left blank and left to save_construct_data and load_construct_data */
        template<class Archive>
        void serialize(Archive &ar, const unsigned int version) { }

        const perlin_noise_3d   _perlin;    /* Perlin noise generator           */
        const ext_colour_t      _rgb;       /* The colour of the texture        */
        const float             _p;         /* The persistence of the octaves   */
        const float             _z;         /* Zoom                             */
        const float             _op;        /* Opaqueness of the texture        */
        const int               _o;         /* Number of octaves to include     */
};
}; /* namespace raptor_raytracer */

namespace boost { 
namespace serialization {
template<class Archive>
inline void save_construct_data(Archive & ar, const raptor_raytracer::perlin_noise_3d_mapper *t, const unsigned int file_version)
{
    ar << t->falloff();
    ar << t->_rgb;
    ar << t->_p;
    ar << t->_z;
    ar << t->_op;
    ar << t->_o;
    ar << t->_perlin.seed();
}


template<class Archive>
inline void load_construct_data(Archive & ar, raptor_raytracer::perlin_noise_3d_mapper *t, const unsigned int file_version)
{
    /* Retreive the fields */
    raptor_raytracer::mapper_falloff *falloff;
    raptor_raytracer::ext_colour_t rgb;
    float p, z, op;
    int o, s;
    ar >> falloff;
    ar >> rgb;
    ar >> p;
    ar >> z;
    ar >> op;
    ar >> o;
    ar >> s;
    
    /* Use plaement new to create the class */
    ::new(t)raptor_raytracer::perlin_noise_3d_mapper(rgb, p, z, o, s, op);
    t->falloff(falloff);
}
}; /* namespace serialization */
}; /* namespace boost */
