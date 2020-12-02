#pragma once

#include "triangle.h"
#include "primitive_store.h"


namespace raptor_raytracer
{
/* Enumerate the major axis and direction of the lights direction */
enum class light_direction_t : char { x_pos = 0, y_pos = 1, z_pos = 2, x_neg = 3, y_neg = 4, z_neg = 5 };

/* Class to represent a light */
class light
{
    public :
        /* CTOR for spherical light */
        light(const ext_colour_t &rgb, const point_t<> &c, const float d, const float r)                  
            : e(nullptr), t(nullptr), rgb(rgb / 255.0f), c(c), n(2.0f), r(r), d(d), s_a(0.0f), s_b(0.0f), n_dir(light_direction_t::x_pos) { };

        /* CTOR for triangle defined light */
        light(const primitive_store *const e, const ext_colour_t &rgb, const point_t<> &c, const float d, const std::vector<int> *const t)
            : e(e), t(t), rgb(rgb / 255.0f), c(c), n(2.0f), r(0.0f), d(d), s_a(0.0f), s_b(0.0f), n_dir(light_direction_t::x_pos) { };

        /* CTOR for spot light */
        light(const ext_colour_t &rgb, const point_t<> &c, const point_t<> &n, const float d, const float s_a, const float s_b, const float r)
            : e(nullptr), t(nullptr), rgb(rgb / 255.0f), c(c), n(n), r(r), d(d), s_a(s_a), s_b(s_b), n_dir(light_direction_t::x_pos) { };

        /* CTOR for triangle defined spot light */
        light(const primitive_store *const e, const ext_colour_t &rgb, const point_t<> &c, const point_t<> &n, const float d, const float s_a, const float s_b, const std::vector<int> *const t)
            : e(e), t(t), rgb(rgb / 255.0f), c(c), n(n), r(0.0f), d(d), s_a(s_a), s_b(s_b), n_dir(light_direction_t::x_pos) { };

        /* CTOR for directional light */
        light(const ext_colour_t &rgb, const point_t<> &n, const float d)
            : e(nullptr), t(nullptr), rgb(rgb / 255.0f), c(), n(n), r(0.0f), d(d), s_a(0.0f), s_b(0.0f), n_dir(find_major_direction()) {  };

        /* Copy CTOR */
        light(const light &l)
            : e(l.e), t(l.t), rgb(l.rgb), c(l.c), n(l.n), r(l.r), d(l.d), s_a(l.s_a), s_b(l.s_b), n_dir(l.n_dir) { };
        
        const point_t<>      & get_centre()                       const   { return this->c;               }
        
        /* Use inverse distance square law for light intensity */
        /* Distance is scaled by this->d */
        const ext_colour_t   get_light_intensity(const point_t<> &dir, const float c)  const   
        {
            const float dist    = c * this->d;
            const float d_scale = ((dist * dist) + 1.0f);
            if (this->s_b != 0.0f)
            {
                const float a = acos(dot_product(dir, this->n));
                const float p = (a - this->s_a) / (this->s_b - this->s_a);
                const float f = std::max(0.0f, std::min(1.0f, (1.0f - p)));
                return (this->rgb * (f / d_scale));
            }
            else
            {
                return this->rgb / d_scale;
            }
        }
        
        /* Soft shadow destination picking */
        int find_rays(ray *const r, const point_t<> &d, const int n) const
        {
            /* Directional light */
            if ((this->n.x != 2.0f) && (this->s_b == 0.0f))
            {
                /* The light should be parallel light and from infinetly far away */
                /* Place the light outside the scene bounding box and in the direction of this->n */
                float dist;
                switch (this->n_dir)
                {
                    case light_direction_t::x_pos :
                    case light_direction_t::y_pos :
                    case light_direction_t::z_pos :
                        dist = 1.0e15f;
                        break;

                    case light_direction_t::x_neg :
                    case light_direction_t::y_neg :
                    case light_direction_t::z_neg :
                        dist = -1.0e15f;
                        break;
                    
                    default :
                        assert(false);
                }
                
                /* Create a new ray */
                const point_t<> c(d + (this->n * dist));
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
            else if (this->t == nullptr)
            {
                /* Pick scales in each co-ordinate */
                const float a_scale = 1.0f / static_cast<float>((n >> 4) + ((n & 0xe) != 0) + 1);
                const float b_scale = 1.0f / static_cast<float>((n >> 4) + ((n & 0xc) != 0) + 1);
                const float r_scale = 1.0f / static_cast<float>((n >> 4) + ((n & 0x8) != 0) + 1);

                /* Create n rays, each with a random offset in a fixed volume */
                for (int i = 0; i < n; i++)
                {
                    /* Pick random offsets */
                    const int eigths    = (i >> 2) & ~0x1;
                    const float a_off   = gen_random_mersenne_twister() * (a_scale * static_cast<float>(eigths + (i & 0x1))) * (2.0f * PI);
                    const float b_off   = gen_random_mersenne_twister() * (b_scale * static_cast<float>(eigths + (i & 0x2))) * (2.0f * PI);
                    const float r_off   = gen_random_mersenne_twister() * (r_scale * static_cast<float>(eigths + (i & 0x4))) * this->r;

                    /* Convert to cartesian co-ordinates */
                    const float sin_a   = cos_lut.get_sin(a_off);
                    const float cos_a   = cos_lut.get_cos(a_off);
                    const float sin_b   = cos_lut.get_sin(b_off);
                    const float cos_b   = cos_lut.get_cos(b_off);
                    point_t<> offset((r_off * cos_a * sin_b), (r_off * sin_a * sin_b), (r_off * cos_b));

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
                const float stride  = static_cast<float>(this->t->size()) / static_cast<float>(std::max((int)(n - (this->t->size() * ppt)), 1));

                /* Create n rays */
                float extra_point   = 0.0f;
                int  ray_nr         = 0;
                for (int i = 0; i < static_cast<int>(this->t->size()); ++i)
                {
                    /* Create ppt + 1 new ray to the light */
                    if (i == (int)extra_point)
                    {
                        e->primitive((*this->t)[i])->find_rays(&r[ray_nr], d, ppt + 1);
                        ray_nr      += (ppt + 1);
                        extra_point += stride;
                    }
                    /* Create ppt new rays to the light */
                    else
                    {
                        e->primitive((*this->t)[i])->find_rays(&r[ray_nr], d, ppt);
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
                    if (n.x < 0.0f)
                    {
                        return light_direction_t::x_neg;
                    }
                    else
                    {
                        return light_direction_t::x_pos;
                    }
                }
                else
                {
                    if (n.z < 0.0f)
                    {
                        return light_direction_t::z_neg;
                    }
                    else
                    {
                        return light_direction_t::z_pos;
                    }
                }
            }
            else
            {
                if (fabs(n.y) > fabs(n.z))
                {
                    if (n.y < 0.0f)
                    {
                        return light_direction_t::y_neg;
                    }
                    else
                    {
                        return light_direction_t::y_pos;
                    }
                }
                else
                {
                    if (n.z < 0.0f)
                    {
                        return light_direction_t::z_neg;
                    }
                    else
                    {
                        return light_direction_t::z_pos;
                    }
                }
            }
        }

        const primitive_store *const    e;      /* All the scene primitives                 */
        const std::vector<int> *const   t;      /* Triangles forming a face                 */
        const ext_colour_t              rgb;    /* The colour of the light                  */
        const point_t<>                   c;      /* Centre of the light                      */
        const point_t<>                   n;      /* Direction of the light                   */
        const float                     r;      /* Radius of the light                      */
        const float                     d;      /* Drop off with distance                   */
        const float                     s_a;    /* Angle where spotlight starts to fade     */
        const float                     s_b;    /* Angle where spotlight finishes fading    */
        const light_direction_t         n_dir;  /* Major axis and direction of the light    */
};
}; /* namespace raptor_raytracer */
