#pragma once

/* Standard headers */

/* Boost headers */
#include "boost/serialization/access.hpp"
#include "boost/shared_array.hpp"

/* Common headers */
#include "common.h"
#include "logging.h"
#include "point_t.h"
#include "texture_mapper.h"


namespace raptor_raytracer
{
/* Forward delcarations */
class ext_colour_t;

/* Pure virtual class for material data and shading */
class planar_mapper : public image_texture_mapper
{
    public :
        planar_mapper(const boost::shared_array<float> &img, const point_t<> &c, const point_t<> &n, const point_t<> &s, 
              const unsigned cpp, const unsigned w, const unsigned h, const texture_wrapping_mode_t uw, const texture_wrapping_mode_t vw,
              const int u_off = 0, const int v_off = 0, const int u_max = -1, const int v_max = -1) : 
                image_texture_mapper(img, c, s, point_t<>(n.y + n.z, 0.0f, n.x), point_t<>(0.0f, n.x + n.z, n.y), 1.0f / static_cast<float>(w), 1.0f / static_cast<float>(h), w, h, cpp, uw, vw, u_off, v_off, u_max, v_max), _n(n)
            {
                // BOOST_LOG_TRIVIAL(trace) << "n vec: " << _n;
                // BOOST_LOG_TRIVIAL(trace) << "u vec: " << _u;
                // BOOST_LOG_TRIVIAL(trace) << "v vec: " << _v;
            };

        planar_mapper(const boost::shared_array<float> &img, const point_t<> &u, const point_t<> &v, const point_t<> &c, const point_t<> &n, const point_t<> &s, 
            const unsigned cpp, const unsigned w, const unsigned h, const texture_wrapping_mode_t uw, const texture_wrapping_mode_t vw,
            const int u_off = 0, const int v_off = 0, const int u_max = -1, const int v_max = -1) :
                image_texture_mapper(img, c, point_t<>((u * s.x) + (v * s.y) + (n * s.z)), u, v, 1.0f / static_cast<float>(w), 1.0f / static_cast<float>(h), w, h, cpp, uw, vw, u_off, v_off, u_max, v_max), _n(n)
            {  };

        virtual ~planar_mapper() { };
    
    protected :
        void texture_coordinates(float *const u_co, float *const v_co, const point_t<> &dst, const point_t<> &n) const override;

    private :
        friend class boost::serialization::access;
        
        /* Serialisation left blank and left to save_construct_data and load_construct_data */
        template<class Archive>
        void serialize(Archive &ar, const unsigned int version) { }

        const point_t<>   _n; /* Normal of the texture    */
};
}; /* namespace raptor_raytracer */

namespace boost { 
namespace serialization {
template<class Archive>
inline void save_construct_data(Archive & ar, const raptor_raytracer::planar_mapper *t, const unsigned int file_version)
{
    ar << t->falloff();
    ar << t->_c;
    ar << t->_n;
    ar << t->_u;
    ar << t->_s;
    ar << t->_uw;
    ar << t->_vw;
    ar << t->_v;
    ar << t->_img;
    ar << t->_h;
    ar << t->_w;
    ar << t->_cpp;
    ar << t->_u_off;
    ar << t->_v_off;
    ar << t->_u_max;
    ar << t->_v_max;
}

template<class Archive>
inline void load_construct_data(Archive & ar, raptor_raytracer::planar_mapper *t, const unsigned int file_version)
{
    /* Retreive the fields */
    raptor_raytracer::mapper_falloff *falloff;
    point_t<> c, n, u, s, v;
    raptor_raytracer::texture_wrapping_mode_t uw, vw;
    float *img;
    unsigned h, w, cpp, u_max, v_max;
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
    ::new(t)raptor_raytracer::planar_mapper(boost::shared_array<float>(img), u, v, c, n, s, cpp, w, h, uw, vw, u_off, v_off, u_max, v_max);
    t->falloff(falloff);
}
}; /* namespace serialization */
}; /* namespace boost */
