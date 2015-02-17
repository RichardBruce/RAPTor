#ifndef __CYLINDRICAL_MAPPER_H__
#define __CYLINDRICAL_MAPPER_H__

/* Boost headers */
#include "boost/serialization/access.hpp"

/* Common headers */
#include "common.h"
#include "logging.h"
#include "point_t.h"

/* Ray tracer headers */
#include "texture_mapper.h"


namespace raptor_raytracer
{
/* Pure virtual class for material data and shading */
class cylindrical_mapper : public texture_mapper
{
    public :
        cylindrical_mapper(const char *const filename, const point_t &c, const point_t &n, const point_t &s, const float r) 
            : texture_mapper(), c(c), u(point_t(n.y, n.z, n.x)), v(n), s(s), r(r)
            {
                METHOD_LOG;
                
                /* Decompress the jpeg */
                this->cpp = read_jpeg(&this->img, filename, &this->h, &this->w);   
                assert((this->cpp == 1) || (this->cpp == 3));

                /* Find the V vector */
                BOOST_LOG_TRIVIAL(trace) << "Loaded cyclindric texture map with parameters: ";
                BOOST_LOG_TRIVIAL(trace) << "U vec : " << this->u;
                BOOST_LOG_TRIVIAL(trace) << "V vec : " << this->v;
                BOOST_LOG_TRIVIAL(trace) << "Size  : " << this->s;
                BOOST_LOG_TRIVIAL(trace) << "Height: " << this->h;
                BOOST_LOG_TRIVIAL(trace) << "Width : " << this->w;
            };

        cylindrical_mapper(const point_t &c, const point_t &u, const point_t &v, const point_t &s,
            const float r, float *const img, const unsigned h, const unsigned w, const unsigned cpp)
            : texture_mapper(), c(c), u(u), v(v), s(s), r(r), img(img), h(h), w(w), cpp(cpp) { }

        virtual ~cylindrical_mapper() 
        { 
            delete [] img;
        };

        /* Texture mapping function. Takes the destination and direction 
           of the incident ray and returns either a float (alpha, kd, ks, t, r....), a colour (rgb) or both */
        float texture_map(ext_colour_t *const c, const point_t &dst, const point_t &n, const point_t &vt) const;

    private :
        friend class boost::serialization::access;
        
        /* Serialisation left blank and left to save_construct_data and load_construct_data */
        template<class Archive>
        void serialize(Archive &ar, const unsigned int version) { }

        const point_t       c;      /* Center of the texture                */
        const point_t       u;      /* U vector in the plane of the texture */
        const point_t       v;      /* V vector in the plane of the texture */
        const point_t       s;      /* Size of the texture                  */
        const float         r;      /* Radius of the cylinder               */
        float              *img;    /* Image data                           */
        unsigned int        h;      /* Image height                         */
        unsigned int        w;      /* Image width                          */
        unsigned int        cpp;    /* Componants per pixel                 */
};
}; /* namespace raptor_raytracer */

namespace boost { 
namespace serialization {
template<class Archive>
inline void save_construct_data(Archive & ar, const raptor_raytracer::cylindrical_mapper *t, const unsigned int file_version)
{
    ar << t->c;
    ar << t->u;
    ar << t->v;
    ar << t->s;
    ar << t->r;
    ar << t->img;
    ar << t->h;
    ar << t->w;
    ar << t->cpp;
}

template<class Archive>
inline void load_construct_data(Archive & ar, raptor_raytracer::cylindrical_mapper *t, const unsigned int file_version)
{
    /* Retreive the fields */
    point_t c, u, v, s;
    float r;
    float *img;
    unsigned int h, w, cpp;
    ar >> c;
    ar >> u;
    ar >> v;
    ar >> s;
    ar >> r;
    ar >> img;
    ar >> h;
    ar >> w;
    ar >> cpp;
    
    /* Use plaement new to create the class */
    ::new(t)raptor_raytracer::cylindrical_mapper(c, u, v, s, r, img, h, w, cpp);
}
}; /* namespace serialization */
}; /* namespace boost */

#endif /* #ifndef __CYLINDRICAL_MAPPER_H__ */
