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
class checker_board_mapper : public procedural_texture_mapper
{
    public :
        checker_board_mapper(const ext_colour_t &rgb, const point_t &cnt, const point_t &grid_size, const float op = 1.0f, const float u_ps = 0.1f, const float v_ps = 0.1f) : 
            procedural_texture_mapper(u_ps, v_ps), _rgb(rgb), _cnt(cnt), _grid_size(grid_size), _op(op) {  };

        virtual ~checker_board_mapper() { };

    protected :
        float run_procedure(ext_colour_t *const c, const point_t &dst, const point_t &dir, const point_t &n) const override;

    private :
        friend class boost::serialization::access;
        
        /* Serialisation left blank and left to save_construct_data and load_construct_data */
        template<class Archive>
        void serialize(Archive &ar, const unsigned int version) { }

        const ext_colour_t  _rgb;       /* The colour of the texture    */
        const point_t       _cnt;       /* Center                       */
        const point_t       _grid_size; /* The size of the squares      */
        const float         _op;        /* Opaqueness of the texture    */
};
}; /* namespace raptor_raytracer */

namespace boost { 
namespace serialization {
template<class Archive>
inline void save_construct_data(Archive & ar, const raptor_raytracer::checker_board_mapper *t, const unsigned int file_version)
{
    ar << t->falloff();
    ar << t->_rgb;
    ar << t->_cnt;
    ar << t->_grid_size;
    ar << t->_op;
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
