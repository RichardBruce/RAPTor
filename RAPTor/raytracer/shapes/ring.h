#ifndef __RING_H__
#define __RING_H__

#include "shape.h"
#include "line.h"
#include "ray.h"
#include "kdt_node.h"


class ring : public shape
{
    public :
        ring(material *const m, const point_t &c, const vector_t &n, const fp_t r, const fp_t i=0.0, const bool l=false);
        virtual  ~ring(){};
        
        /* Access functions */
        void set_radius(const fp_t r)   { this->r_sq = (r*r);       }
        
        fp_t get_radius() const           { return sqrt(this->r_sq);  }
        
        /* Shape virtual functions */
        virtual fp_t    is_intersecting(const ray *const r, hit_t *const h) const;
        virtual line    normal_at_point(ray       *const r, const hit_t h) const;
        virtual point_t get_random_point(const int i) const;

    private : 
        fp_t     r_sq, i_sq;
        vector_t n;
};

#endif /* #ifndef __RING_H__ */
