#ifndef __CYLINDRICAL_MAPPER_H__
#define __CYLINDRICAL_MAPPER_H__

/* Standard headers */

/* Boost headers */
#include "boost/serialization/access.hpp"
#include "boost/shared_array.hpp"

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
        cylindrical_mapper(const boost::shared_array<float> &img, const point_t &c, const point_t &n, const point_t &s, const float r, 
            const unsigned int h, const unsigned int w, const unsigned int cpp, const int cycles = 1) : 
            texture_mapper(), c(c), u(point_t(n.y, n.z, n.x)), v(n), s(s), r(r), img(img), h(h), w(w), cpp(cpp), cycles(cycles)
            {
                METHOD_LOG;
                
                /* Checks */
                assert((this->cpp == 1) || (this->cpp == 3));

                /* Find the V vector */
                BOOST_LOG_TRIVIAL(trace) << "Loaded cyclindric texture map with parameters: ";
                BOOST_LOG_TRIVIAL(trace) << "U vec : " << this->u;
                BOOST_LOG_TRIVIAL(trace) << "V vec : " << this->v;
                BOOST_LOG_TRIVIAL(trace) << "Size  : " << this->s;
                BOOST_LOG_TRIVIAL(trace) << "Height: " << this->h;
                BOOST_LOG_TRIVIAL(trace) << "Width : " << this->w;
            };

        cylindrical_mapper(const point_t &c, const point_t &u, const point_t &v, const point_t &s, const float r, 
            const boost::shared_array<float> &img, const unsigned int h, const unsigned int w, const unsigned int cpp, const int cycles) :
            texture_mapper(), c(c), u(u), v(v), s(s), r(r), img(img), h(h), w(w), cpp(cpp), cycles(cycles) { }

        virtual ~cylindrical_mapper() {  };

        /* Texture mapping function. Takes the destination and direction 
           of the incident ray and returns either a float (alpha, kd, ks, t, r....), a colour (rgb) or both */
        float sample_texture(ext_colour_t *const c, const point_t &dst, const point_t &n, const point_t &vt) const;
        float sample_texture_monochrome(point_t *const c, const point_t &dst, const point_t &n, const point_t &vt, const int x_off, const int y_off) const;

    private :
        friend class boost::serialization::access;
        
        /* Serialisation left blank and left to save_construct_data and load_construct_data */
        template<class Archive>
        void serialize(Archive &ar, const unsigned int version) { }

        const point_t               c;      /* Center of the texture                            */
        const point_t               u;      /* U vector in the plane of the texture             */
        const point_t               v;      /* V vector in the plane of the texture             */
        const point_t               s;      /* Size of the texture                              */
        const float                 r;      /* Radius of the cylinder                           */
        boost::shared_array<float>  img;    /* Image data                                       */
        const unsigned int          h;      /* Image height                                     */
        const unsigned int          w;      /* Image width                                      */
        const unsigned int          cpp;    /* Componants per pixel                             */
        const int                   cycles; /* Cycles of the texture per cycle of the cylinder  */
};
}; /* namespace raptor_raytracer */

namespace boost { 
namespace serialization {
template<class Archive>
inline void save_construct_data(Archive & ar, const raptor_raytracer::cylindrical_mapper *t, const unsigned int file_version)
{
    ar << t->falloff();
    ar << t->c;
    ar << t->u;
    ar << t->v;
    ar << t->s;
    ar << t->r;
    ar << t->img;
    ar << t->h;
    ar << t->w;
    ar << t->cpp;
    ar << t->cycles;
}

template<class Archive>
inline void load_construct_data(Archive & ar, raptor_raytracer::cylindrical_mapper *t, const unsigned int file_version)
{
    /* Retreive the fields */
    raptor_raytracer::mapper_falloff *falloff;
    point_t c, u, v, s;
    float r;
    float *img;
    unsigned int h, w, cpp;
    int cycles;
    ar >> falloff;
    ar >> c;
    ar >> u;
    ar >> v;
    ar >> s;
    ar >> r;
    ar >> img;
    ar >> h;
    ar >> w;
    ar >> cpp;
    ar >> cycles;
    
    /* Use plaement new to create the class */
    ::new(t)raptor_raytracer::cylindrical_mapper(c, u, v, s, r, boost::shared_array<float>(img), h, w, cpp, cycles);
    t->falloff(falloff);
}
}; /* namespace serialization */
}; /* namespace boost */

#endif /* #ifndef __CYLINDRICAL_MAPPER_H__ */
