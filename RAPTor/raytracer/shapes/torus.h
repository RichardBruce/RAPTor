#ifndef __TORUS_H__
#define __TORUS_H__

#include "shape.h"
#include "line.h"
#include "ray.h"
#include "kdt_node.h"


class torus : public shape
{
    public :
        torus(material *const m, point_t c, fp_t r=1.0, fp_t a=1.0, 
              vector_t n=vector_t(0.0,0.0,0.0), fp_t theta=0.0, bool l=false);
        virtual  ~torus(){};
        
        /* Access functions */
        void set_radius(fp_t r) { this->r = r;      }
        
        fp_t get_radius() const   { return this->sr;  }
        
        /* Shape virtual functions */
        virtual fp_t  is_intersecting(const ray *const r, hit_t *const h) const;
        virtual line    normal_at_point(ray     *const r, const hit_t h) const;
        virtual point_t get_random_point(const int i) const;

    private : 
        int  sturmian_roots(const ray *const r, fp_t bottom_bracket, fp_t top_bracket) const;
        fp_t secant_roots(const ray *const r, fp_t bottom_bracket, fp_t top_bracket) const;
        
        vector_t rot_axis;
        point_t  rot_point;
        fp_t     r, a, sr, theta;
        bool     rotate;
};

#endif /* #ifndef __TORUS_H__ */
