#ifndef __SHAPE_H__
#define __SHAPE_H__

#include "common.h"
#include "line.h"
#include "material.h"

/* Forward declaration */
class ray;


/* Abstract base class all shapes must implement */
class shape
{
    public :
        shape(material *const m, point_t c, bool l=false) : m(m), centre(c), light(l) { };
        virtual ~shape() {  };
        
        /* Read of light intensity is only valid if this is a light */
        const ext_colour_t& get_light_intensity()   const       { assert(this->light); return this->m->get_light_intensity();  }
        
        /* Location functions for creating shadow rays */
        const point_t& get_centre()	            const           { return this->centre;                          }
        fp_t           get_x0()    	            const           { return this->centre.x;                        }
        fp_t           get_y0()    	            const           { return this->centre.y;                        }
        fp_t           get_z0()      	        const           { return this->centre.z;                        }

        /* Find out if this is a light/transparent object for shadow rays to ignore */
        bool get_light()                        const           { return this->light;                           }
        bool is_transparent()                   const           { return this->m->is_transparent();             }

        /* Bounding box access functions for spatial subdivision */
        fp_t lowest_x()                         const           { return this->b.x;                             }
        fp_t lowest_y()                         const           { return this->b.y;                             }
        fp_t lowest_z()                         const           { return this->b.z;                             }
        fp_t highest_x()                        const           { return this->t.x;                             }
        fp_t highest_y()                        const           { return this->t.y;                             }
        fp_t highest_z()                        const           { return this->t.z;                             }
        bool is_intersecting_x(const fp_t s)    const           { return ((this->t.x > s) & (this->b.x < s));   }
        bool is_intersecting_y(const fp_t s)    const           { return ((this->t.y > s) & (this->b.y < s));   }
        bool is_intersecting_z(const fp_t s)    const           { return ((this->t.z > s) & (this->b.z < s));   }
        
        /* Virtual members to be implemented by the derived class */
        /* is_intersecting should return the distance along the line
           l that the shape and the line intersect. If the shape and 
           the line do not intersect 'DOUBLE_MAX' should be returned */
        virtual fp_t is_intersecting(const ray *const r, hit_t *const h) const = 0;
        
        /* normal_at_point should return a line normal to the shape at
           the point p */
        virtual line    normal_at_point(ray *const r, const hit_t h) const = 0;
        virtual point_t get_random_point(const int i)      const = 0;

        /* Call the materials shader with the shapes normal */
        void shade(const ray_trace_engine &r, ray &i, const hit_t h, ext_colour_t *const c) const
        {
            this->m->shade(r, i, this->normal_at_point(&i, h), h, c);
        }
    
    protected :
        /* Access functions for dervied classes only */
        void set_centre(const point_t &p)                           { this->centre = p;                             }
        void set_x0(const fp_t x)                                   { this->centre.x = x;                           }
        void set_y0(const fp_t y)                                   { this->centre.y = y;                           }
        void set_z0(const fp_t z)                                   { this->centre.z = z;                           }
        void set_light(const bool l)                                { this->light = l;                              }

        /* Shape bounding box access functions */
        void set_bounding_box(const point_t &t, const point_t &b)   { this->t = t; this->b = b;                     }
        void set_top(const point_t &t)                              { this->t = t;                                  }
        void set_bot(const point_t &b)                              { this->b = b;                                  }

    private :
        material        *const m;
        point_t 		centre, t, b;
        bool    		light;
};


/* Useful sort criteria for shapes */
class sort_shape_by_lowest_x
{
    public :
        bool operator() (const shape *const a, const shape *const b) { return (a->lowest_x() < b->lowest_x()); }
};


class sort_shape_by_lowest_y
{
    public :
        bool operator() (const shape *const a, const shape *const b) { return (a->lowest_y() < b->lowest_y()); }
};


class sort_shape_by_lowest_z
{
    public :
        bool operator() (const shape *const a, const shape *const b) { return (a->lowest_z() < b->lowest_z()); }
};


class sort_shape_by_highest_x
{
     public :
        bool operator() (const shape *const a, const shape *const b) { return (a->highest_x() < b->highest_x()); }
};


class sort_shape_by_highest_y
{
     public :
        bool operator() (const shape *const a, const shape *const b) { return (a->highest_y() < b->highest_y()); }
};


class sort_shape_by_highest_z
{
    public :
        bool operator() (const shape *const a, const shape *const b) { return (a->highest_z() < b->highest_z()); }
};


#endif /* #ifndef __SHAPE_H__ */
