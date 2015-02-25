#ifndef __CUBIC_MAPPER_H__
#define __CUBIC_MAPPER_H__

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
/* Forward delcarations */
class ext_colour_t;

/* Pure virtual class for material data and shading */
class cubic_mapper : public texture_mapper
{
    public :
        cubic_mapper(const boost::shared_array<float> &im, const point_t &c, const point_t &n, const point_t &s, const texture_wrapping_mode_t uw, 
            const texture_wrapping_mode_t vw, const unsigned int cpp, const unsigned int w, const unsigned int h, const int u_off = 0, 
            const int v_off = 0, const int u_max = -1, const int v_max = -1) : 
            texture_mapper(), c(c), n(n), u(point_t(n.y, n.z, -n.x)), s(s), uw(uw), vw(vw), img(im), h(h), w(w), cpp(cpp), u_off(u_off), v_off(v_off)
            {
                METHOD_LOG;
                
                /* Checks */
                assert((this->cpp == 1) || (this->cpp == 3));
                this->u_max = (u_max < 0) ? w : u_max;
                this->v_max = (v_max < 0) ? h : v_max;

                /* Find the V vector */
                if (n.x)
                {
                    this->u = point_t( 0.0f, 0.0f, -1.0f);
                }
                else if (n.y)
                {
                    this->u = point_t(-1.0f, 0.0f,  0.0f);
                }
                else
                {
                    this->u = point_t( 1.0f, 0.0f,  0.0f);
                }
                
                cross_product(n, this->u, &this->v);

                BOOST_LOG_TRIVIAL(trace) << "Loaded cubic texture map with parameters: ";
                BOOST_LOG_TRIVIAL(trace) << "U vec : " << this->u;
                BOOST_LOG_TRIVIAL(trace) << "V vec : " << this->v;
                BOOST_LOG_TRIVIAL(trace) << "Size  : " << this->s;
                BOOST_LOG_TRIVIAL(trace) << "Height: " << this->h;
                BOOST_LOG_TRIVIAL(trace) << "Width : " << this->w;
            }

        cubic_mapper(const boost::shared_array<float> &im, const point_t &u, const point_t &v, const point_t &c, const point_t &n, const point_t &s, 
            const unsigned cpp, const unsigned w, const unsigned h, const texture_wrapping_mode_t uw, const texture_wrapping_mode_t vw,
            const int u_off = 0, const int v_off = 0, const int u_max = -1, const int v_max = -1)
            : texture_mapper(), c(c), n(n), u(u), s((u * s.x) + (v * s.y) + (n * s.z)), uw(uw), vw(vw), v(v), img(im), h(h), w(w), cpp(cpp),
              u_off(u_off), v_off(v_off), u_max(u_max < 0 ? w : u_max), v_max(v_max < 0 ? h : v_max)
            { 
                assert((this->cpp == 1) || (this->cpp == 3));
            }

        virtual ~cubic_mapper() { };

        /* Texture mapping function. Takes the destination and direction 
           of the incident ray and returns either a fp_t (alpha, kd, ks, t, r....), a colour (rgb) or both */
        float sample_texture(ext_colour_t *const c, const point_t &dst, const point_t &n, const point_t &vt) const;
        float sample_texture_monochrome(point_t *const c, const point_t &dst, const point_t &n, const point_t &vt, const int x_off, const int y_off) const;

    private :
        friend class boost::serialization::access;
        
        /* Serialisation left blank and left to save_construct_data and load_construct_data */
        template<class Archive>
        void serialize(Archive &ar, const unsigned int version) { }

        const point_t                   c;      /* Center of the texture                */
        const point_t                   n;      /* Normal of the texture                */
        point_t                         u;      /* U vector in the plane of the texture */
        const point_t                   s;      /* Size of the texture                  */
        const texture_wrapping_mode_t   uw;     /* U wrapping mode                      */
        const texture_wrapping_mode_t   vw;     /* V wrapping mode                      */
        point_t                         v;      /* V vector in the plane of the texture */
        boost::shared_array<float>      img;    /* Image data                           */
        unsigned int                    h;      /* Image height                         */
        unsigned int                    w;      /* Image width                          */
        unsigned int                    cpp;    /* Componants per pixel                 */
        const int                       u_off;  /* U offset to be added to every pixel  */
        const int                       v_off;  /* V offset to be added to every pixel  */
        unsigned int                    u_max;  /* Max u value for early wrapping       */
        unsigned int                    v_max;  /* Max v value for early wrapping       */
};
}; /* namespace raptor_raytracer */

namespace boost { 
namespace serialization {
template<class Archive>
inline void save_construct_data(Archive & ar, const raptor_raytracer::cubic_mapper *t, const unsigned int file_version)
{
    ar << t->falloff();
    ar << t->c;
    ar << t->n;
    ar << t->u;
    ar << t->s;
    ar << t->uw;
    ar << t->vw;
    ar << t->v;
    ar << t->img;
    ar << t->h;
    ar << t->w;
    ar << t->cpp;
    ar << t->u_off;
    ar << t->v_off;
    ar << t->u_max;
    ar << t->v_max;
}

template<class Archive>
inline void load_construct_data(Archive & ar, raptor_raytracer::cubic_mapper *t, const unsigned int file_version)
{
    /* Retreive the fields */
    raptor_raytracer::mapper_falloff *falloff;
    point_t c, n, u, s, v;
    raptor_raytracer::texture_wrapping_mode_t uw, vw;
    float *img;
    unsigned int h, w, cpp, u_max, v_max;
    int u_off, v_off;
    ar >> falloff;
    ar >> c;
    ar >> n;
    ar >> u;
    ar >> s;
    ar >> uw;
    ar >> vw;
    ar >> v;
    ar >> img;
    ar >> h;
    ar >> w;
    ar >> cpp;
    ar >> u_off;
    ar >> v_off;
    ar >> u_max;
    ar >> v_max;
    
    /* Use plaement new to create the class */
    ::new(t)raptor_raytracer::cubic_mapper(boost::shared_array<float>(img), u, v, c, n, s, cpp, w, h, uw, vw, u_off, v_off, u_max, v_max);
    t->falloff(falloff);
}
}; /* namespace serialization */
}; /* namespace boost */

#endif /* #ifndef __CUBIC_MAPPER_H__ */
