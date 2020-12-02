#pragma once

#include "line.h"


namespace raptor_raytracer
{
/* Forward declarations */
class triangle;

class ray
{
    public :
        /* Constructor if we havent intersected anything yet */
        ray(const point_t<> &o, const float x = 0.0f, const float y = 0.0f, const float z = 0.0f, const float m = 1.0f, const int c = 1) : 
            ogn(o), dst(0.0f, 0.0f, 0.0f), dir(point_t<>(x, y, z)), length(0.0f), magn(m), componant(c) { };
        
        /* Constructor if we have intersected something, ie/ a shadow ray */
        ray(const point_t<> &o, const point_t<> &d, const float m = 1.0f, const int c = 1) : 
            ogn(o), dst(d), magn(m), componant(c)
        {
            this->dir    = this->dst - this->ogn;
            this->length = magnitude(this->dir);
            this->dir   /= this->length;
        }
        
        ray(const point_t<> &o, const point_t<> &dst, const point_t<> &dir, const float l, const float m = 1.0f, int c = 1) :
            ogn(o), dst(dst), dir(dir), length(l), magn(m), componant(c) { };

        /* Default constructor for arrays */
        ray() : ogn(0.0f, 0.0f, 0.0f), dst(0.0f, 0.0f, 0.0f), length(0.0f), magn(0.0f), componant(0)  { };

        ~ray(){ };
        
        /* Access functions */
        void set_up(const point_t<> &o, const float x = 0.0f, const float y = 0.0f, const float z = 0.0f, const float m = 1.0f, const int c = 1)
        {
            this->ogn       = o;
            this->dir       = point_t<>(x, y, z);
            this->magn      = m;
            this->componant = c;
        }
        
        void set_up(const point_t<> &o, const point_t<> &d, const float m = 1.0f, const int c = 1)
        {
            this->dst       = d;
            this->ogn       = o;
            this->magn      = m;
            this->componant = c;

            this->dir    = this->dst - this->ogn;
            this->length = magnitude(this->dir);
            this->dir   /= this->length;
        }
        
        void set_up(const point_t<> &o, const point_t<> &dst, const point_t<> &dir, const float l, const float m = 1.0f, int c = 1)
        {
            this->ogn       = o;
            this->dst       = dst;
            this->dir       = dir;
            this->length    = l;
            this->magn      = m;
            this->componant = c;
        }

        float   get_x0()            const   { return this->ogn.x;       }
        float   get_y0()            const   { return this->ogn.y;       }
        float   get_z0()            const   { return this->ogn.z;       }
        float   get_x1()            const   { return this->dst.x;       }
        float   get_y1()            const   { return this->dst.y;       }
        float   get_z1()            const   { return this->dst.z;       }
        float   get_x_grad()        const   { return this->dir.x;       }
        float   get_y_grad()        const   { return this->dir.y;       }
        float   get_z_grad()        const   { return this->dir.z;       }
        float   get_length()        const   { return this->length;      }
        point_t<> get_ogn()           const   { return this->ogn;         }
        point_t<> get_dst()           const   { return this->dst;         }
        point_t<> get_dir()           const   { return this->dir;         }

        void set_x0(const float x)      { this->ogn.x = x;  }
        void set_y0(const float y)      { this->ogn.y = y;  }
        void set_z0(const float z)      { this->ogn.z = z;  }
        void set_x1(const float x)      { this->dst.x = x;  }
        void set_y1(const float y)      { this->dst.y = y;  }
        void set_z1(const float z)      { this->dst.z = z;  }
        void set_x_grad(const float x)  { this->dir.x = x;  }
        void set_y_grad(const float y)  { this->dir.y = y;  }
        void set_z_grad(const float z)  { this->dir.z = z;  }
        void set_ogn(const point_t<> &o)  { this->ogn = o;    }
        void set_dst(const point_t<> &d)  { this->dst = d;    }
        void set_dir(const point_t<> &d)  { this->dir = d;    }

        void set_magnitude(const float m)    { this->magn = m;           }

        float   get_magnitude()        const   { return this->magn;        }
        int     get_componant()        const   { return this->componant;   }

        /* Allow the length of the ray to be changed for improved numerical stability */
        void change_length(const float d)
        {
            this->dst       += this->dir * d;
            this->length    += d;
        }

        void set_geometry_normal(const point_t<> &n)
        {
            this->geo_norm = n;
        }
        
        /* Get an offset start point for secondary rays */
        point_t<> offset_start_point(const int d) const
        {
            assert((d == -1) || (d == 1));
        
            assert(sizeof(unsigned) == sizeof(float));
            static const float offset[2] = { FP_DELTA, -FP_DELTA };
    
            point_t<> start;
            
            /* X offset */
            unsigned x = bit_cast<float, unsigned>(this->dst.x);
            if ((x & 0x7fffffff) < bit_cast<float, unsigned>(FP_DELTA_SMALL))
            {
                start.x += FP_DELTA_SMALL * geo_norm.x;
            }
            else
            {
                x += int(offset[(x >> 31) & 0x1] * geo_norm.x * d);
                start.x = bit_cast<unsigned, float>(x);
            }
    
            /* Y offset */
            unsigned y = bit_cast<float, unsigned>(this->dst.y);
            if ((y & 0x7fffffff) < bit_cast<float, unsigned>(FP_DELTA_SMALL))
            {
                start.y += FP_DELTA_SMALL * geo_norm.y;
            }
            else
            {
                y += int(offset[(y >> 31) & 0x1] * geo_norm.y * d);
                start.y = bit_cast<unsigned, float>(y);
            }

            /* Z offset */
            unsigned z = bit_cast<float, unsigned>(this->dst.z);
            if ((z & 0x7fffffff) < bit_cast<float, unsigned>(FP_DELTA_SMALL))
            {
                start.z += FP_DELTA_SMALL * geo_norm.z;
            }
            else
            {
                z += int(offset[(z >> 31) & 0x1] * geo_norm.z * d);
                start.z = bit_cast<unsigned, float>(z);
            }
            
            return start;
        }

        /* Ray Functions */
        point_t<> calculate_destination(const float d);
        ray     rotate(const vector_t &r, const point_t<> &c, const float theta) const;
        float   find_rays(ray rays[], const light &l, const hit_t h) const;
        float   get_magnitude_beer(const float a);
        float   reflect(ray rays[], const point_t<> &n, const float r, const float dr = 0.0f) const;
        float   refract(ray rays[], const point_t<> &n, const float t, float ri, const hit_t h, const float dr = 0.0f) const;
        
    private : 
        point_t<> ogn;
        point_t<> dst;
        point_t<> dir;
        point_t<> geo_norm;
        float   length, magn;
        int     componant;
};
}; /* namespace raptor_raytracer */
