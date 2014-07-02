#ifndef __SPHERE_H__
#define __SPHERE_H__

#include "shape.h"
#include "line.h"
#include "ray.h"
#include "kdt_node.h"


class sphere : public shape
{
    public :
        sphere(material *const m, point_t c, fp_t r=1.0, bool l=false) : shape(m,c,l), r(r) 
        {
            this->set_bounding_box(point_t((c.x + this->r), (c.y + this->r), (c.z + this->r)), point_t((c.x - this->r), (c.y - this->r), (c.z - this->r)));
        };
        virtual  ~sphere(){};
        
        /* Access functions */
        void set_radius(fp_t r) { this->r = r;      }
        
        fp_t get_radius() const { return this->r;   }
        
        /* Shape virtual functions */
        virtual fp_t    is_intersecting(const ray *const r, hit_t *const h) const;
        virtual line    normal_at_point(ray       *const r, const hit_t h) const;
        virtual point_t get_random_point(const int i) const;

    private : 
        fp_t r;
};

#endif /* #ifndef __SPHERE_H__ */
