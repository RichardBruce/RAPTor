#ifndef __AAB_H__
#define __AAB_H__

#include "shape.h"
#include "line.h"
#include "ray.h"
#include "kdt_node.h"


namespace raptor_raytracer
{
class aab : public shape
{
    public :
        aab(material *const m, point_t p, fp_t x_width=0.0, fp_t y_width=0.0, fp_t z_width=0.0, bool l=false) : 
            shape(m,p,l) 
            {
                this->set_bounding_box(point_t((p.x + x_width), (p.y + y_width), (p.z + z_width)), p);
            };
        aab(material *const m, point_t top, point_t bot, bool l=false) : 
            shape(m,bot,l) 
            { 
                this->set_bounding_box(top, bot);
            };
        virtual ~aab(){};
        
        /* Access functions */
        void set_x_width(const fp_t x) { this->set_bounding_box(point_t((this->get_x0() + x), this->get_y0(), this->get_z0()), this->get_centre()); }
        void set_y_width(const fp_t y) { this->set_bounding_box(point_t(this->get_x0(), (this->get_y0() + y), this->get_z0()), this->get_centre()); }
        void set_z_width(const fp_t z) { this->set_bounding_box(point_t(this->get_x0(), this->get_y0(), (this->get_z0() + z)), this->get_centre()); }
        fp_t get_x_width()             { return (this->highest_x() - this->get_x0());  }
        fp_t get_y_width()             { return (this->highest_y() - this->get_y0());  }
        fp_t get_z_width()             { return (this->highest_z() - this->get_z0());  }
        
        /* Shape virtual functions */
        virtual fp_t    is_intersecting(const ray *const r, hit_t *const h) const;
        virtual line    normal_at_point(ray       *const r, const hit_t h) const;
        virtual point_t get_random_point(const int i) const;

    private : 
};
}; /* namespace raptor_raytracer */

#endif /* #ifndef __AAB_H__ */
