#ifndef __CYLINDER_H__
#define __CYLINDER_H__

#include "shape.h"
#include "line.h"
#include "ray.h"
#include "kdt_node.h"


namespace raptor_raytracer
{
class cylinder : public shape
{
    public :
        cylinder(material *const m, const point_t &c, const fp_t h, const fp_t r, const fp_t i=0.0, 
                 const vector_t &n=vector_t(0.0,0.0,0.0), const fp_t theta=0.0, bool const l=false);

        cylinder(material *const m, const point_t &c, const point_t &b, const fp_t r, const fp_t i=0.0, const bool l=false);

        virtual  ~cylinder(){};
        
        /* Access functions */
        void set_radius(fp_t r) { this->r = r;      }
        void set_height(fp_t h) { this->h = h;      }
        
        fp_t get_radius() const { return this->r;   }
        fp_t get_height() const { return this->h;   }
        
        /* Shape virtual functions */
        virtual fp_t    is_intersecting(const ray *const r, hit_t *const h) const;
        virtual line    normal_at_point(ray       *const r, const hit_t h) const;
        virtual point_t get_random_point(const int i) const;

    private : 
        vector_t rot_axis;
        fp_t     r, h, i_sq, theta;
        bool     rotate;
};
}; /* namespace raptor_raytracer */

#endif /* #ifndef __CYLINDER_H__ */
