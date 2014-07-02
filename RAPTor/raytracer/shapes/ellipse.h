#ifndef __ELLIPSE_H__
#define __ELLIPSE_H__

#include "shape.h"
#include "line.h"
#include "ray.h"
#include "kdt_node.h"


class ellipse : public shape
{
    public :
        ellipse(material *const m, point_t c, fp_t h, fp_t r, fp_t a=1.0, fp_t i=0.0, 
                vector_t n=vector_t(0.0,0.0,0.0), fp_t theta=0.0, bool l=false);
        virtual  ~ellipse(){};
        
        /* Access functions */
        void   set_radius(fp_t r) { this->r = r; }
        void   set_height(fp_t h) { this->h = h; }
        
        fp_t get_radius() const   { return this->r; }
        fp_t get_height() const   { return this->h; }
        
        /* Shape virtual functions */
        virtual fp_t    is_intersecting(const ray *const r, hit_t *const h) const;
        virtual line    normal_at_point(ray       *const r, const hit_t h) const;
        virtual point_t get_random_point(const int i) const;

    private : 
        vector_t rot_axis;
        point_t  rot_point;
        fp_t     r, h, a, c, i_sq, a_sq, theta;
        bool     rotate;
};

#endif /* #ifndef __ELLIPSE_H__ */
