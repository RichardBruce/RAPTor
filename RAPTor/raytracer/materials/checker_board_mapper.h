#ifndef __CHECKER_BOARD_MAPPER_H__
#define __CHECKER_BOARD_MAPPER_H__

/* Boost headers */
#include "boost/serialization/access.hpp"

/* Common headers */
#include "common.h"
#include "ext_colour_t.h"
#include "point_t.h"
#include "texture_mapper.h"


namespace raptor_raytracer
{
/* Pure virtual class for material data and shading */
class checker_board_mapper : public texture_mapper
{
    public :
        checker_board_mapper(const ext_colour_t &rgb, const point_t &cnt, const point_t &grid_size, const float op = 1.0f) : 
            texture_mapper(), rgb(rgb), cnt(cnt), grid_size(grid_size), op(op) {  };

        virtual ~checker_board_mapper() { };

        /* Texture mapping function. Takes the destination and direction 
           of the incident ray and returns either a float (alpha, kd, ks, t, r....), a colour (rgb) or both */
        float sample_texture(ext_colour_t *const c, const point_t &dst, const point_t &n, const point_t &vt) const;
        float sample_texture_monochrome(point_t *const c, const point_t &dst, const point_t &n, const point_t &vt, const int x_off, const int y_off) const;

    private :
        friend class boost::serialization::access;
        
        /* Serialisation left blank and left to save_construct_data and load_construct_data */
        template<class Archive>
        void serialize(Archive &ar, const unsigned int version) { }

        const ext_colour_t  rgb;        /* The colour of the texture    */
        const point_t       cnt;        /* Center                       */
        const point_t       grid_size;  /* The size of the squares      */
        const float         op;         /* Opaqueness of the texture    */
};
}; /* namespace raptor_raytracer */

namespace boost { 
namespace serialization {
template<class Archive>
inline void save_construct_data(Archive & ar, const raptor_raytracer::checker_board_mapper *t, const unsigned int file_version)
{
    ar << t->falloff();
    ar << t->rgb;
    ar << t->cnt;
    ar << t->grid_size;
    ar << t->op;
}


template<class Archive>
inline void load_construct_data(Archive & ar, raptor_raytracer::checker_board_mapper *t, const unsigned int file_version)
{
    /* Retreive the fields */
    raptor_raytracer::mapper_falloff *falloff;
    raptor_raytracer::ext_colour_t rgb;
    point_t grid_size;
    point_t cnt;
    float op;
    ar >> falloff;
    ar >> rgb;
    ar >> cnt;
    ar >> grid_size;
    ar >> op;
    
    /* Use plaement new to create the class */
    ::new(t)raptor_raytracer::checker_board_mapper(rgb, cnt, grid_size, op);
    t->falloff(falloff);
}
}; /* namespace serialization */
}; /* namespace boost */

#endif /* #ifndef __CHECKER_BOARD_MAPPER_H__ */
