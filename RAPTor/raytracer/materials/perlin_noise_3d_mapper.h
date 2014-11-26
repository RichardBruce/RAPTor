#ifndef __PERLIN_NOISE_3D_MAPPER_H__
#define __PERLIN_NOISE_3D_MAPPER_H__

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
class perlin_noise_3d_mapper : public texture_mapper
{
    public :
        perlin_noise_3d_mapper(const ext_colour_t &rgb, const float p, const float z, const int o, const int s, const float op = 1.0f) : 
            texture_mapper(), perlin(s), rgb(rgb * 0.5f), p(p), z(z), op(op), o(o) {  };

        virtual ~perlin_noise_3d_mapper() { };

        /* Texture mapping function. Takes the destination and direction 
           of the incident ray and returns either a float (alpha, kd, ks, t, r....), a colour (rgb) or both */
        float texture_map(ext_colour_t *const c, const point_t &dst, const point_t &n, const point_t &vt) const;

    private :
        friend class boost::serialization::access;
        
        /* Serialisation left blank and left to save_construct_data and load_construct_data */
        template<class Archive>
        void serialize(Archive &ar, const unsigned int version) { }

        const perlin_noise_3d   perlin; /* Perlin noise generator           */
        const ext_colour_t      rgb;    /* The colour of the texture        */
        const float             p;      /* The persistence of the octaves   */
        const float             z;      /* Zoom                             */
        const float             op;     /* Opaqueness of the texture        */
        const int               o;      /* Number of octaves to include     */
};
}; /* namespace raptor_raytracer */

namespace boost { 
namespace serialization {
template<class Archive>
inline void save_construct_data(Archive & ar, const raptor_raytracer::perlin_noise_3d_mapper *t, const unsigned int file_version)
{
    ar << t->rgb;
    ar << t->p;
    ar << t->z;
    ar << t->op;
    ar << t->o;
    ar << t->perlin.seed();
}


template<class Archive>
inline void load_construct_data(Archive & ar, raptor_raytracer::perlin_noise_3d_mapper *t, const unsigned int file_version)
{
    /* Retreive the fields */
    raptor_raytracer::ext_colour_t rgb;
    float p, z, op;
    int o, s;
    ar >> rgb;
    ar >> p;
    ar >> z;
    ar >> op;
    ar >> o;
    ar >> s;
    
    /* Use plaement new to create the class */
    ::new(t)raptor_raytracer::perlin_noise_3d_mapper(rgb, p, z, o, s, op);
}
}; /* namespace serialization */
}; /* namespace boost */

#endif /* #ifndef __PERLIN_NOISE_3D_MAPPER_H__ */
