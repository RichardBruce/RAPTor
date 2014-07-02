#ifndef __RAY_H__
#define __RAY_H__

#include "line.h"

/* Forward declarations */
class triangle;

class ray
{
    public :
        /* Constructor if we havent intersected anything yet */
        ray(point_t o, fp_t x=0.0, fp_t y=0.0, fp_t z=0.0, fp_t m=1.0, int c=1) : 
            ogn(o), dst(0.0,0.0,0.0), dir(point_t(x, y, z)), length(0.0), magn(m), componant(c) { };
        
        /* Constructor if we have intersected something, ie/ a shadow ray */
        ray(point_t o, point_t d, fp_t m=1.0, int c=1) : 
            ogn(o), dst(d), magn(m), componant(c)
        {
            this->dir    = this->dst - this->ogn;
            this->length = magnitude(this->dir);
            this->dir   /= this->length;
        }
        
        ray(const point_t &o, const point_t &dst, const point_t &dir, const fp_t l, const fp_t m=1.0, int c=1) :
            ogn(o), dst(dst), dir(dir), length(l), magn(m), componant(c) { };

        /* Default constructor for arrays */
        ray() : ogn(0.0,0.0,0.0), dst(0.0,0.0,0.0), length(0.0), magn(0.0), componant(0)  { };

        ~ray(){ };
        
        /* Access functions */
        void set_up(point_t o, fp_t x=0.0, fp_t y=0.0, fp_t z=0.0, fp_t m=1.0, int c=1)
        {
            this->ogn       = o;
            this->dir       = point_t(x, y, z);
            this->magn      = m;
            this->componant = c;
        }
        
        void set_up(point_t o, point_t d, fp_t m=1.0, int c=1)
        {
            this->dst       = d;
            this->ogn       = o;
            this->magn      = m;
            this->componant = c;

            this->dir    = this->dst - this->ogn;
            this->length = magnitude(this->dir);
            this->dir   /= this->length;
        }
        
        void set_up(const point_t &o, const point_t &dst, const point_t &dir, const fp_t l, const fp_t m=1.0, int c=1)
        {
            this->ogn       = o;
            this->dst       = dst;
            this->dir       = dir;
            this->length    = l;
            this->magn      = m;
            this->componant = c;
        }

        fp_t    get_x0()            const   { return this->ogn.x;       }
        fp_t    get_y0()            const   { return this->ogn.y;       }
        fp_t    get_z0()            const   { return this->ogn.z;       }
        fp_t    get_x1()            const   { return this->dst.x;       }
        fp_t    get_y1()            const   { return this->dst.y;       }
        fp_t    get_z1()            const   { return this->dst.z;       }
        fp_t    get_x_grad()        const   { return this->dir.x;       }
        fp_t    get_y_grad()        const   { return this->dir.y;       }
        fp_t    get_z_grad()        const   { return this->dir.z;       }
        fp_t    get_length()        const   { return this->length;      }
        point_t get_ogn()           const   { return this->ogn;         }
        point_t get_dst()           const   { return this->dst;         }
        point_t get_dir()           const   { return this->dir;         }

        void set_x0(const fp_t x)           { this->ogn.x = x;          }
        void set_y0(const fp_t y)           { this->ogn.y = y;          }
        void set_z0(const fp_t z)           { this->ogn.z = z;          }
        void set_x1(const fp_t x)           { this->dst.x = x;          }
        void set_y1(const fp_t y)           { this->dst.y = y;          }
        void set_z1(const fp_t z)           { this->dst.z = z;          }
        void set_x_grad(const fp_t x)       { this->dir.x = x;          }
        void set_y_grad(const fp_t y)       { this->dir.y = y;          }
        void set_z_grad(const fp_t z)       { this->dir.z = z;          }
        void set_ogn(const point_t &o)      { this->ogn = o;            }
        void set_dst(const point_t &d)      { this->dst = d;            }
        void set_dir(const point_t &d)      { this->dir = d;            }

        void set_magnitude(const fp_t m)    { this->magn = m;           }

        fp_t get_magnitude()        const   { return this->magn;        }
        int  get_componant()        const   { return this->componant;   }

        /* Allow the length of the ray to be changed for improved numerical stability */
        void change_length(fp_t d)
        {
            this->dst       += this->dir * d;
            this->length    += d;
        }
        
        /* Get an offset start point for secondary rays */
        point_t offset_start_point(const line &n, const int d) const
        {
            assert((d == -1) || (d == 1));
        
            assert(sizeof(unsigned) == sizeof(fp_t));
            static const fp_t offset[2] = { FP_DELTA, -FP_DELTA };
    
            point_t start;
            
            /* X offset */
            unsigned x = bit_cast<fp_t, unsigned>(this->dst.x);
            if ((x & 0x7fffffff) < bit_cast<fp_t, unsigned>(FP_DELTA_SMALL))
            {
                start.x += FP_DELTA_SMALL * n.get_x_grad();
            }
            else
            {
                x += int(offset[(x >> 31) & 0x1] * n.get_x_grad() * d);
                start.x = bit_cast<unsigned, fp_t>(x);
            }
    
            /* Y offset */
            unsigned y = bit_cast<fp_t, unsigned>(this->dst.y);
            if ((y & 0x7fffffff) < bit_cast<fp_t, unsigned>(FP_DELTA_SMALL))
            {
                start.y += FP_DELTA_SMALL * n.get_y_grad();
            }
            else
            {
                y += int(offset[(y >> 31) & 0x1] * n.get_y_grad() * d);
                start.y = bit_cast<unsigned, fp_t>(y);
            }

            /* Z offset */
            unsigned z = bit_cast<fp_t, unsigned>(this->dst.z);
            if ((z & 0x7fffffff) < bit_cast<fp_t, unsigned>(FP_DELTA_SMALL))
            {
                start.z += FP_DELTA_SMALL * n.get_z_grad();
            }
            else
            {
                z += int(offset[(z >> 31) & 0x1] * n.get_z_grad() * d);
                start.z = bit_cast<unsigned, fp_t>(z);
            }
            
            return start;
        }

        /* Ray Functions */
        point_t calculate_destination(const fp_t d);
        ray     rotate(const vector_t &r, const point_t &c, const fp_t theta) const;
        point_t rotate_dst(const vector_t &r, const point_t &c, const fp_t theta) const;
        fp_t    find_rays(ray rays[], const light &l, const line &n, const hit_t h) const;
        fp_t    get_magnitude_beer(const fp_t a);
        fp_t    reflect(ray rays[], const line &n, const fp_t r, const fp_t dr=0.0) const;
        fp_t    refract(ray rays[], const line &n, const fp_t t, fp_t ri, const hit_t h, const fp_t dr=0.0) const;
        
    private : 
        point_t ogn, dst, dir;
        fp_t    length, magn;
        int     componant;
};

#endif /* #ifndef __RAY_H__ */
