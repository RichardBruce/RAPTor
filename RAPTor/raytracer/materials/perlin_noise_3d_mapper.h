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
        perlin_noise_3d_mapper(const ext_colour_t &rgb, fp_t p, fp_t z, int o, int s, fp_t op = 1.0) : 
            texture_mapper(), perlin(s), rgb(rgb), p(p), z(z), op(op), o(o)
            {
	            /* Normalise the colour */
	            this->rgb.r /= 2.0;
	            this->rgb.g /= 2.0;
	            this->rgb.b /= 2.0;
            };
        virtual ~perlin_noise_3d_mapper() { };

        /* Texture mapping function. Takes the destination and direction 
           of the incident ray and returns either a fp_t (alpha, kd, ks, t, r....), a colour (rgb) or both */
        fp_t texture_map(const point_t &dst, const point_t &dir, ext_colour_t *const c, const point_t &vt) const;

    private :
        friend class boost::serialization::access;
        
        /* Serialisation left blank and left to save_construct_data and load_construct_data */
        template<class Archive>
        void serialize(Archive &ar, const unsigned int version) { }

        const perlin_noise_3d & perlin; /* Perlin noise generator           */
        ext_colour_t            rgb;    /* The colour of the texture        */
        const fp_t              p;      /* The persistence of the octaves   */
        const fp_t              z;      /* Zoom                             */
        const fp_t              op;     /* Opaqueness of the texture        */
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
    fp_t p, z, op;
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
