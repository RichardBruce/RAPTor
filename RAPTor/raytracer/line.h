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
        line(const point_t &o, const float x, const float y, const float z) : ogn(o), dir(x, y, z) { };
        line(const point_t &o, const point_t &d) : ogn(o), dir(d) { };
        line(){ };
        ~line(){ };
        
       /* Access functions */
        void    set_x_grad(const float x)   { this->dir.x = x;          }
        void    set_y_grad(const float y)   { this->dir.y = y;          }
        void    set_z_grad(const float z)   { this->dir.z = z;          }
        void    set_ogn(const point_t &o)   { this->ogn   = o;          }
        void    set_dir(const point_t &d)   { this->dir   = d;          }
        
        float   get_x_grad() const  { return this->dir.x;       }
        float   get_y_grad() const  { return this->dir.y;       }
        float   get_z_grad() const  { return this->dir.z;       }
        point_t get_ogn()    const  { return this->ogn;         }
        point_t get_dir()    const  { return this->dir;         }
        
        /* Methods for manipulating a line */
        void    opposite_dir()      { this->dir = -this->dir;   }
        void    rotate(const vector_t &r, const point_t &c, float theta);
        
    private :
        point_t ogn;
        point_t dir;
};
}; /* namespace raptor_raytracer */

#endif /* #ifndef __LINE_H__ */
