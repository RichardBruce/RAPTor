#ifndef __PLANAR_MAPPER_H__
#define __PLANAR_MAPPER_H__

/* Boost headers */
#include "boost/serialization/access.hpp"

/* Common headers */
#include "common.h"
#include "point_t.h"
#include "texture_mapper.h"

namespace raptor_raytracer
{
/* Forward delcarations */
class ext_colour_t;

/* Pure virtual class for material data and shading */
class planar_mapper : public texture_mapper
{
    public :
        planar_mapper(const fp_t *const im, const point_t &c, const point_t &n, const point_t &s, 
                      const unsigned cpp, const unsigned w, const unsigned h, const texture_wrapping_mode_t uw, const texture_wrapping_mode_t vw,
                      const int u_off = 0, const int v_off = 0, const int u_max = -1, const int v_max = -1) : 
            texture_mapper(), c(c), n(n), u(point_t(n.y, n.z, -n.x)), s(s), uw(uw), vw(vw), img(im), h(h), w(w), cpp(cpp),
            u_off(u_off), v_off(v_off)
            {
                assert((this->cpp == 1) || (this->cpp == 3));
                
                if (u_max < 0)
                {
                    this->u_max = w;
                }
                else
                {
                    this->u_max = u_max;
                }

                if (v_max < 0)
                {
                    this->v_max = h;
                }
                else
                {
                    this->v_max = v_max;
                }

                /* Find the V vector */
                if (n.x)
                {
                    this->u = point_t(0.0,0.0,-1.0);
                }
                else if (n.y)
                {
                    this->u = point_t(-1.0,0.0,0.0);
                }
                else
                {
                    this->u = point_t(1.0,0.0,0.0);
                }
                
                cross_product(this->u, n, &this->v);
                this->u = -this->u;
//                cout << "u vec: " << this->u.x << ", " << this->u.y << ", " << this->u.z << endl;
//                cout << "v vec: " << this->v.x << ", " << this->v.y << ", " << this->v.z << endl;
            };

        planar_mapper(const fp_t *const im, const point_t &u, const point_t &v, const point_t &c, const point_t &n, const point_t &s, 
            const unsigned cpp, const unsigned w, const unsigned h, const texture_wrapping_mode_t uw, const texture_wrapping_mode_t vw,
            const int u_off = 0, const int v_off = 0, const int u_max = -1, const int v_max = -1)
            : texture_mapper(), c(c), n(n), u(u), s((u * s.x) + (v * s.y) + (n * s.z)), uw(uw), vw(vw), v(v), img(im), h(h), w(w), cpp(cpp),
              u_off(u_off), v_off(v_off)
            { 
                assert((this->cpp == 1) || (this->cpp == 3));

                if (u_max < 0)
                {
                    this->u_max = w;
                }
                else
                {
                    this->u_max = u_max;
                }

                if (v_max < 0)
                {
                    this->v_max = h;
                }
                else
                {
                    this->v_max = v_max;
                }
            };

        virtual ~planar_mapper() { };

        /* Texture mapping function. Takes the destination and direction 
           of the incident ray and returns either a fp_t (alpha, kd, ks, t, r....), a colour (rgb) or both */
        fp_t texture_map(const point_t &dst, const point_t &dir, ext_colour_t *const c, const point_t &vt) const;

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
        const fp_t               *const img;    /* Image data                           */
        unsigned                        h;      /* Image height                         */
        unsigned                        w;      /* Image width                          */
        unsigned                        cpp;    /* Componants per pixel                 */
        const int                       u_off;  /* U offset to be added to every pixel  */
        const int                       v_off;  /* V offset to be added to every pixel  */
        unsigned                        u_max;  /* Max u value for early wrapping       */
        unsigned                        v_max;  /* Max v value for early wrapping       */
};
}; /* namespace raptor_raytracer */

namespace boost { 
namespace serialization {
template<class Archive>
inline void save_construct_data(Archive & ar, const raptor_raytracer::planar_mapper *t, const unsigned int file_version)
{
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
inline void load_construct_data(Archive & ar, raptor_raytracer::planar_mapper *t, const unsigned int file_version)
{
    /* Retreive the fields */
    point_t c, n, u, s, v;
    raptor_raytracer::texture_wrapping_mode_t uw, vw;
    fp_t *img;
    unsigned h, w, cpp, u_max, v_max;
    int u_off, v_off;
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
    ::new(t)raptor_raytracer::planar_mapper(img, u, v, c, n, s, cpp, w, h, uw, vw, u_off, v_off, u_max, v_max);
}
}; /* namespace serialization */
}; /* namespace boost */

#endif /* #ifndef __PLANAR_MAPPER_H__ */
