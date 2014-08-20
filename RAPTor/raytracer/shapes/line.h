#ifndef __LINE_H__
#define __LINE_H__

#include "common.h"
#include "point_t.h"


namespace raptor_raytracer
{
class line
{
    public :
        /* Constructor for a line with no end i.e a normal */
        line(point_t o, fp_t x, fp_t y, fp_t z) : ogn(o), dir(x, y, z) { };
        line(point_t o, point_t d) : ogn(o), dir(d) { };
        ~line(){ };
        
       /* Access functions */
        void    set_x_grad(fp_t x)  { this->dir.x = x;          }
        void    set_y_grad(fp_t y)  { this->dir.y = y;          }
        void    set_z_grad(fp_t z)  { this->dir.z = z;          }
        void    set_ogn(point_t o)  { this->ogn   = o;          }
        void    set_dir(point_t d)  { this->dir   = d;          }
        
        fp_t    get_x_grad() const  { return this->dir.x;       }
        fp_t    get_y_grad() const  { return this->dir.y;       }
        fp_t    get_z_grad() const  { return this->dir.z;       }
        point_t get_ogn()    const  { return this->ogn;         }
        point_t get_dir()    const  { return this->dir;         }
        
        /* Methods for manipulating a line */
        void    opposite_dir()      { this->dir = -this->dir;   }
        void    rotate(const vector_t r, const point_t c, fp_t theta);
        
    private :
        point_t ogn, dir;
};
}; /* namespace raptor_raytracer */

#endif /* #ifndef __LINE_H__ */
