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
class cylindrical_mapper : public image_texture_mapper
{
    public :
        cylindrical_mapper(const boost::shared_array<float> &img, const point_t &c, const point_t &n, const point_t &s, const float r, 
            const unsigned int h, const unsigned int w, const unsigned int cpp, const int cycles = 1) : 
            image_texture_mapper(img, c, s, point_t(n.y + n.z, 0.0f, n.x), n, 1.0f / static_cast<float>(w), 1.0f / static_cast<float>(h), w, h, cpp, texture_wrapping_mode_t::tile, texture_wrapping_mode_t::blank), _r(r), _cycles(cycles)
            {
                /* Find the V vector */
                BOOST_LOG_TRIVIAL(trace) << "Loaded cyclindric texture map with parameters: ";
                BOOST_LOG_TRIVIAL(trace) << "U vec : " << _u;
                BOOST_LOG_TRIVIAL(trace) << "V vec : " << _v;
                BOOST_LOG_TRIVIAL(trace) << "Size  : " << _s;
                BOOST_LOG_TRIVIAL(trace) << "Height: " << _h;
                BOOST_LOG_TRIVIAL(trace) << "Width : " << _w;
            };

        cylindrical_mapper(const point_t &c, const point_t &u, const point_t &v, const point_t &s, const float r, 
            const boost::shared_array<float> &img, const unsigned int h, const unsigned int w, const unsigned int cpp, const int cycles) :
            image_texture_mapper(img, c, s, u, v, 1.0f / static_cast<float>(w), 1.0f / static_cast<float>(h), w, h, cpp, texture_wrapping_mode_t::tile, texture_wrapping_mode_t::blank), _r(r), _cycles(cycles) { }

        virtual ~cylindrical_mapper() {  };

    protected :
        void texture_coordinates(float *const u_co, float *const v_co, const point_t &dst, const point_t &n) const override;

    private :
        friend class boost::serialization::access;
        
        /* Serialisation left blank and left to save_construct_data and load_construct_data */
        template<class Archive>
        void serialize(Archive &ar, const unsigned int version) { }

        const float _r;         /* Radius of the cylinder                           */
        const int   _cycles;    /* Cycles of the texture per cycle of the cylinder  */
};
}; /* namespace raptor_raytracer */

namespace boost { 
namespace serialization {
template<class Archive>
inline void save_construct_data(Archive & ar, const raptor_raytracer::cylindrical_mapper *t, const unsigned int file_version)
{
    ar << t->falloff();
    ar << t->_c;
    ar << t->_u;
    ar << t->_v;
    ar << t->_s;
    ar << t->_r;
    ar << t->_img;
    ar << t->_h;
    ar << t->_w;
    ar << t->_cpp;
    ar << t->_cycles;
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
