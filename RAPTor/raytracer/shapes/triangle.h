#ifndef __TRIANGLE_H__
#define __TRIANGLE_H__

#include "shape.h"
#include "line.h"
#include "ray.h"
#include "kdt_node.h"


namespace raptor_raytracer
{
class triangle : public shape
{
    public :
        triangle(material *const m, const point_t &a, const point_t &b, const point_t &c, bool l=false);
        virtual  ~triangle(){};
        
        /* Shape virtual functions */
        virtual fp_t    is_intersecting(const ray *const r, hit_t *const h) const;
        virtual line    normal_at_point(ray       *const r, const hit_t h) const;
        virtual point_t get_random_point(const int i) const;

    private : 
        int      k;
        fp_t     n_u, n_v, b_n_u, b_n_v, c_n_u, c_n_v, n_dot;
        point_t  a;
        vector_t n;
};
}; /* namespace raptor_raytracer */

#endif /* #ifndef __TRIANGLE_H__ */
