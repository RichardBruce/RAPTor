#ifndef __PLANE_H__
#define __PLANE_H__

#include "shape.h"
#include "line.h"
#include "ray.h"
#include "kdt_node.h"


class plane : public shape
{
    public :
        plane(material *const m, point_t p, fp_t x_norm=0.0, fp_t y_norm=0.0, fp_t z_norm=0.0, bool l=false) : 
            shape(m,p,l), x_n(x_norm), y_n(y_norm), z_n(z_norm) 
            {
                fp_t hi_x, lo_x, hi_y, lo_y, hi_z, lo_z;
                if(this->x_n != 1.0)
                {
                    lo_x = -DOUBLE_MAX;
                    hi_x =  DOUBLE_MAX;
                }
                else
                {
                    lo_x = this->get_x0();
                    hi_x = this->get_x0();
                }

                if(this->y_n != 1.0)
                {
                    lo_y = -DOUBLE_MAX;
                    hi_y =  DOUBLE_MAX;
                }
                else
                {
                    lo_y = this->get_x0();
                    hi_y = this->get_x0();
                }

                if(this->z_n != 1.0)
                {
                    lo_z = -DOUBLE_MAX;
                    hi_z =  DOUBLE_MAX;
                }
                else
                {
                    lo_z = this->get_x0();
                    hi_z = this->get_x0();
                }
                this->set_bounding_box(point_t(hi_x, hi_y, hi_z), point_t(lo_x, lo_y, lo_z));
            }
        virtual ~plane(){};
        
        /* Access functions */
        void set_normal(fp_t x_norm, fp_t y_norm, fp_t z_norm) 
        { 
            this->x_n = x_norm; 
            this->y_n = y_norm; 
            this->z_n = z_norm; 
        }
        
        void    set_x_norm(const fp_t x)  { this->x_n = x;    }
        void    set_y_norm(const fp_t y)  { this->y_n = y;    }
        void    set_z_norm(const fp_t z)  { this->z_n = z;    }
                
        fp_t  get_x_norm()                { return this->x_n; }
        fp_t  get_y_norm()                { return this->y_n; }
        fp_t  get_z_norm()                { return this->z_n; }
        
        /* Shape virtual functions */
        virtual fp_t  is_intersecting(const ray *const r, hit_t *const h) const;
        virtual line    normal_at_point(ray     *const r, const hit_t h) const;
        /* A plane has zero thickness so cannot refract light */
        virtual point_t get_random_point(const int i)      const { return point_t(0.0,0.0,0.0); }

    private : 
        fp_t x_n, y_n, z_n;
};

#endif /* #ifndef __PLANE_H__ */
