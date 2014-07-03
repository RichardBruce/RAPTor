#ifndef __LIGHT_H__
#define __LIGHT_H__

#include "triangle.h"


namespace raptor_raytracer
{
/* Enumerate the major axis and direction of the lights direction */
enum light_direction_t { x_pos = 0, y_pos = 1, z_pos = 2, x_neg = 3, y_neg = 4, z_neg = 5 };

/* Class to represent a light */
class light
{
    public :
        /* CTOR for spherical light */
        light(const ext_colour_t &rgb, const point_t &c, const fp_t d, const fp_t r)                  
            : t(NULL), rgb(rgb/(fp_t)255.0), c(c), n(2.0), r(r), d(d), s_a(0.0), s_b(0.0), n_dir(x_pos) { };

        /* CTOR for triangle defined light */
        light(const ext_colour_t &rgb, const point_t &c, const fp_t d, const vector<triangle *> *const t)
            : t(t), rgb(rgb/(fp_t)255.0), c(c), n(2.0), r(0.0), d(d), s_a(0.0), s_b(0.0), n_dir(x_pos) { };

        /* CTOR for spot light */
        light(const ext_colour_t &rgb, const point_t &c, const point_t &n, const fp_t d, const fp_t s_a, const fp_t s_b, const fp_t r)
            : t(NULL), rgb(rgb/(fp_t)255.0), c(c), n(n), r(r), d(d), s_a(s_a), s_b(s_b), n_dir(x_pos) { };

        /* CTOR for triangle defined spot light */
        light(const ext_colour_t &rgb, const point_t &c, const point_t &n, const fp_t d, const fp_t s_a, const fp_t s_b, const vector<triangle *> *const t)
            : t(t), rgb(rgb/(fp_t)255.0), c(c), n(n), r(0.0), d(d), s_a(s_a), s_b(s_b), n_dir(x_pos) { };

        /* CTOR for directional light */
        light(const ext_colour_t &rgb, const point_t &n, const fp_t d)
            : t(NULL), rgb(rgb/(fp_t)255.0), c(), n(n), r(0.0), d(d), s_a(0.0), s_b(0.0), n_dir(find_major_direction()) {  };

        /* Copy CTOR */
        light(const light &l)
            : t(l.t), rgb(l.rgb), c(l.c), n(l.n), r(l.r), d(l.d), s_a(l.s_a), s_b(l.s_b), n_dir(l.n_dir) { };
        
        const point_t      & get_centre()                       const   { return this->c;               }
        
        /* Use inverse distance square law for light intensity */
        /* Distance is scaled by this->d */
        const ext_colour_t   get_light_intensity(const point_t &dir, const fp_t c)  const   
        {
            const fp_t dist     = c * this->d;
            const fp_t d_scale  = ((dist * dist) + (fp_t)1.0);
            if (this->s_b != (fp_t)0.0)
            {
                const fp_t a = acos(dot_product(dir, this->n));
                const fp_t p = (a - this->s_a) / (this->s_b - this->s_a);
                const fp_t f = max((fp_t)0.0, min((fp_t)1.0, ((fp_t)1.0 - p)));
                return (this->rgb * (f / d_scale));
            }
            else
            {
                return this->rgb / d_scale;
            }
        }
        
        /* Soft shadow destination picking */
        int find_rays(ray *const r, const point_t &d, const int n) const
        {
            /* Directional light */
            if ((this->n.x != (fp_t)2.0) && (this->s_b == (fp_t)0.0))
            {
                /* The light should be parallel light and from infinetly far away */
                /* Place the light outside the scene bounding box and in the direction of this->n */
                fp_t dist;
                switch (this->n_dir)
                {
                    case x_pos :
                        dist = (triangle::get_scene_upper_bounds().x - d.x) / this->n.x;
                        break;

                    case y_pos :
                        dist = (triangle::get_scene_upper_bounds().y - d.y) / this->n.y;
                        break;

                    case z_pos :
                        dist = (triangle::get_scene_upper_bounds().z - d.z) / this->n.z;
                        break;

                    case x_neg :
                        dist = (triangle::get_scene_lower_bounds().x - d.x) / this->n.x;
                        break;

                    case y_neg :
                        dist = (triangle::get_scene_lower_bounds().y - d.y) / this->n.y;
                        break;

                    case z_neg :
                        dist = (triangle::get_scene_lower_bounds().z - d.z) / this->n.z;
                        break;
                    
                    default :
                        assert(false);
                }
                
                /* Create a new ray */
//                cout << d.x << ", " << d.y << ", " << d.z << ", ";
                point_t c(d + (this->n * dist));
//                cout << c.x << ", " << c.y << ", " << c.z << endl;
                r[0].set_up(d, c);
                
                /* only 1 ray is needed because they will all follow the same path */
                return 1;
            }
            /* Avoid work for n == 1 */
            else if (n == 1)
            {
                r[0].set_up(d, this->c);
                return 1;
            }
            /* Generate spherical light */
            else if (this->t == NULL)
            {
                /* Pick scales in each co-ordinate */
                const fp_t a_scale  = 1.0/(fp_t)((n >> 4) + ((n & 0xe) != 0) + 1);
                const fp_t b_scale  = 1.0/(fp_t)((n >> 4) + ((n & 0xc) != 0) + 1);
                const fp_t r_scale  = 1.0/(fp_t)((n >> 4) + ((n & 0x8) != 0) + 1);

                /* Create n rays, each with a random offset in a fixed volume */
                for (int i = 0; i < n; i++)
                {
                    /* Pick random offsets */
                    const int eigths    = (i >> 2) & ~0x1;
                    const fp_t a_off    = gen_random_mersenne_twister() * (a_scale * (fp_t)(eigths + (i & 0x1))) * ((fp_t)2.0 * PI);
                    const fp_t b_off    = gen_random_mersenne_twister() * (b_scale * (fp_t)(eigths + (i & 0x2))) * ((fp_t)2.0 * PI);
                    const fp_t r_off    = gen_random_mersenne_twister() * (r_scale * (fp_t)(eigths + (i & 0x4))) * this->r;

                    /* Convert to cartesian co-ordinates */
                    const fp_t sin_a    = cos_lut.get_sin(a_off);
                    const fp_t cos_a    = cos_lut.get_cos(a_off);
                    const fp_t sin_b    = cos_lut.get_sin(b_off);
                    const fp_t cos_b    = cos_lut.get_cos(b_off);
                    point_t offset((r_off * cos_a * sin_b), (r_off * sin_a * sin_b), (r_off * cos_b));

                    /* Create a new ray */
                    r[i].set_up(d, (this->c + offset));
                }
            }
            /* Use the triangles to generate an area light */
            else
            {
                /* Point to take every triangle */
                const int ppt       = n / this->t->size();
                
                /* How often to take an extra point */
                const fp_t stride   = (fp_t)this->t->size() / (fp_t)max((int)(n - (this->t->size() * ppt)), 1);

                /* Create n rays */
                fp_t extra_point    = (fp_t)0.0;
                int  ray_nr         = 0;
                for(int i = 0; i < (int)this->t->size(); i ++)
                {
                    /* Create ppt + 1 new ray to the light */
                    if (i == (int)extra_point)
                    {
                        (*this->t)[i]->find_rays(&r[ray_nr], d, ppt + 1);
                        ray_nr      += (ppt + 1);
                        extra_point += stride;
                    }
                    /* Create ppt new rays to the light */
                    else
                    {
                        (*this->t)[i]->find_rays(&r[ray_nr], d, ppt);
                        ray_nr      += ppt;
                    }
                }
                
                /* Check enough rays were created */
//                if(ray_nr != n)
//                {
//                    cout << "ray_nr: " << ray_nr << ", n: " << n << ", stride: " << stride << ", ppt: " << ppt << ", tri: " << this->t->size() << endl;
//                    assert(false);
//                }
                assert(ray_nr == n);
            }

            return n;
        }
    
    private :
        light& operator=(const light &l) { return *this; }

        light_direction_t find_major_direction()
        {
            /* Pick the major direction and magnitude of the light */
            if (fabs(n.x) > fabs(n.y))
            {
                if (fabs(n.x) > fabs(n.z))
                {
                    if (n.x < (fp_t)0.0)
                    {
                        return x_neg;
                    }
                    else
                    {
                        return x_pos;
                    }
                }
                else
                {
                    if (n.z < (fp_t)0.0)
                    {
                        return z_neg;
                    }
                    else
                    {
                        return z_pos;
                    }
                }
            }
            else
            {
                if (fabs(n.y) > fabs(n.z))
                {
                    if (n.y < (fp_t)0.0)
                    {
                        return y_neg;
                    }
                    else
                    {
                        return y_pos;
                    }
                }
                else
                {
                    if (n.z < (fp_t)0.0)
                    {
                        return z_neg;
                    }
                    else
                    {
                        return z_pos;
                    }
                }
            }
        }

        const vector<triangle *> *const t;      /* Triangles forming a face                 */
        const ext_colour_t              rgb;    /* The colour of the light                  */
        const point_t                   c;      /* Centre of the light                      */
        const point_t                   n;      /* Direction of the light                   */
        const fp_t                      r;      /* Radius of the light                      */
        const fp_t                      d;      /* Drop off with distance                   */
        const fp_t                      s_a;    /* Angle where spotlight starts to fade     */
        const fp_t                      s_b;    /* Angle where spotlight finishes fading    */
        const light_direction_t         n_dir;  /* Major axis and direction of the light    */
};
}; /* namespace raptor_raytracer */

#endif /* #ifndef __LIGHT_H__ */
