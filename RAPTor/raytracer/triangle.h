#ifndef __TRIANGLE_H__
#define __TRIANGLE_H__

/* Boost headers */
#include "boost/noncopyable.hpp"

/* Ray tracer headers */
#include "common.h"
#include "line.h"
#include "ray.h"
#ifdef SIMD_PACKET_TRACING
#include "packet_ray.h"
#include "frustrum.h"
#endif
#include "material.h"

namespace raptor_raytracer
{
class triangle : private boost::noncopyable
{
    public :
        triangle(material *const m, const point_t &a, const point_t &b, const point_t &c, bool l = false, const point_t *v_n = nullptr, const point_t *v_t = nullptr);
        ~triangle()
        {
            /* REVISIT -- Consider using indices for these */
            if (this->vn != nullptr)
            {
                delete [] this->vn;
            }

            if (this->vt != nullptr)
            {
                delete [] this->vt;
            }
        };
        
        /* Static scene bounding box access */
        static void reset_scene_bounding_box()                  
        {
            triangle::scene_top = point_t(-MAX_DIST, -MAX_DIST, -MAX_DIST);
            triangle::scene_bot = point_t( MAX_DIST,  MAX_DIST,  MAX_DIST);
        }
        
        static point_t & get_scene_upper_bounds()               { return triangle::scene_top;                   }
        static point_t & get_scene_lower_bounds()               { return triangle::scene_bot;                   }
        
        /* Find out if this is a light/transparent object for shadow rays to ignore */
        bool get_light()                            const       { return this->light;                           }
        bool is_transparent()                       const       { return this->m->is_transparent();             }

        /* Shadow ray targetting */
        const point_t& get_centre()	                const       { return this->vertex_c;                        }
        
        /* Frustrum separation algorithm */
        point_t get_vertex_a()                      const       { return this->vertex_a;                        }
        point_t get_vertex_b()                      const       { return this->vertex_b;                        }
        point_t get_vertex_c()                      const       { return this->vertex_c;                        }

        /* KD-tree node classification */
        float get_x0()                               const       { return this->vertex_c.x;                      }
        float get_y0()                               const       { return this->vertex_c.y;                      }
        float get_z0()                               const       { return this->vertex_c.z;                      }
        
        /* Bounding box access functions for spatial subdivision */
        float    lowest_x()                          const       { return this->b.x;                             }
        float    lowest_y()                          const       { return this->b.y;                             }
        float    lowest_z()                          const       { return this->b.z;                             }
        point_t lowest_point()                      const       { return this->b;                               }
        float    highest_x()                         const       { return this->t.x;                             }
        float    highest_y()                         const       { return this->t.y;                             }
        float    highest_z()                         const       { return this->t.z;                             }
        point_t highest_point()                     const       { return this->t;                               }
        bool    is_intersecting_x(const float s)     const       { return ((this->t.x > s) && (this->b.x < s));  }
        bool    is_intersecting_y(const float s)     const       { return ((this->t.y > s) && (this->b.y < s));  }
        bool    is_intersecting_z(const float s)     const       { return ((this->t.z > s) && (this->b.z < s));  }
        
        /* Ray tracing functions */
        inline void         is_intersecting(const ray *const r, hit_description *const h) const;
#ifdef SIMD_PACKET_TRACING
        inline void         is_intersecting(const packet_ray *const r, packet_hit_description *const h, const triangle **const i_o, const unsigned int size) const;
        inline void         is_intersecting(const frustrum &f, const packet_ray *const r, packet_hit_description *const h, const triangle **const i_o, const unsigned *c, const unsigned size) const;
#endif
        inline line         normal_at_point(ray       *const r, hit_description *const h) const;
        inline void         find_rays(ray *const r, const point_t &d, const int n) const;
        inline point_t      get_random_point(const int i) const;
        
        /* Generate secondary rays for packet tracing */
        inline line generate_rays(const ray_trace_engine &r, ray &i, hit_description *const h, ray *const rl, ray *const rf, float *const n_rl, float *const n_rf) const
        {
            line norm = this->normal_at_point(&i, h);
            this->m->generate_rays(r, i, norm, h->h, rl, rf, n_rl, n_rf);
            return norm;
        }
        
        /* Call the materials shader with the triangles normal */
        inline void shade(const ray_trace_engine &r, ray &i, const line &n, const hit_description &h, ext_colour_t *const c) const
        {
            /* Interpolate the texture co-ordinate if possible */
            point_t text(MAX_DIST);
            if (this->vt != nullptr)
            {
                text = (h.u * this->vt[2]) + (h.v * this->vt[1]) + ((1.0f - (h.u + h.v)) * this->vt[0]);
            }

            /* Call the material shader */
            this->m->shade(r, i, n, h.h, c, text);
        }
        
        inline void combind_secondary_rays(const ray_trace_engine &r, ext_colour_t &c, const ray *const rl, const ray *const rf, const ext_colour_t *const c_rl, const ext_colour_t *const c_rf, const float *const n_rl, const float *const n_rf) const
        {
            this->m->combind_secondary_rays(r, c, rl, rf, c_rl, c_rf, n_rl, n_rf);
        }

    
    /* Physics functions */
    inline float is_intersecting(const triangle *const t, const point_t &tp, const point_t &p) const;

    private : 
        /* Primvate functions */
        void calculate_pre_computes();
        
        material        *const m;       /* Pointer to the triangles shader                      */
        static point_t  scene_top;      /* Scene bounding box upper vertex                      */
        static point_t  scene_bot;      /* Scene bounding box lower vertex                      */
        point_t         *vn;            /* Pointer to vertex normals, if available else nullptr    */
        point_t         *vt;            /* Pointer to vertex textures, if available else nullptr   */
        point_t         a;              /* Projected vertex a                                   */
        point_t         vertex_a;       /* Vertex a of the triangle                             */
        point_t         vertex_b;       /* Vertex b of the triangle                             */
        point_t         vertex_c;       /* Vertex c of the triangle                             */
        point_t         n;              /* Normal of the triangle                               */
        point_t         t;              /* Triangle bounding box upper vertex                   */
        point_t         b;              /* Triangle bounding box lower vertex                   */
        float           n_u;
        float           n_v;
        float           b_n_u;
        float           b_n_v;
        float           c_n_u;
        float           c_n_v;
        int             k;
        bool            light;
};



/***********************************************************
 Pre compute calculation function.
 
 The major axis and normal of the triangle are pre-calculated.
 The scaling factors for conversion to barycentric 
 co-ordinates and into the plane of the triangle are also 
 pre calculated.
************************************************************/
inline void triangle::calculate_pre_computes()
{
    /* Vertex checks */
    assert(this->vertex_a != this->vertex_b);
    assert(this->vertex_a != this->vertex_c);
    assert(this->vertex_b != this->vertex_c);

    /* Pick the bounds of the triangle */
    float hi_x, lo_x, hi_y, lo_y, hi_z, lo_z;
    hi_x = std::max(this->vertex_a.x, std::max(this->vertex_b.x, this->vertex_c.x));
    lo_x = std::min(this->vertex_a.x, std::min(this->vertex_b.x, this->vertex_c.x));
    hi_y = std::max(this->vertex_a.y, std::max(this->vertex_b.y, this->vertex_c.y));
    lo_y = std::min(this->vertex_a.y, std::min(this->vertex_b.y, this->vertex_c.y));
    hi_z = std::max(this->vertex_a.z, std::max(this->vertex_b.z, this->vertex_c.z));
    lo_z = std::min(this->vertex_a.z, std::min(this->vertex_b.z, this->vertex_c.z));
    
    /* If the triangle falls into a plane add some width to it */
    if (fabs(hi_x - lo_x) < EPSILON)
    {
        lo_x -= (EPSILON * 0.5f);
        hi_x += (EPSILON * 0.5f);
    }
    if (fabs(hi_y - lo_y) < EPSILON)
    {
        lo_y -= (EPSILON * 0.5f);
        hi_y += (EPSILON * 0.5f);
    }
    if (fabs(hi_z - lo_z) < EPSILON)
    {
        lo_z -= (EPSILON * 0.5f);
        hi_z += (EPSILON * 0.5f);
    }
    this->t = point_t(hi_x, hi_y, hi_z);
    this->b = point_t(lo_x, lo_y, lo_z);
    
    /* Update scene bounding box */
    triangle::scene_top.x = std::max(triangle::scene_top.x, hi_x);
    triangle::scene_top.y = std::max(triangle::scene_top.y, hi_y);
    triangle::scene_top.z = std::max(triangle::scene_top.z, hi_z);
    triangle::scene_bot.x = std::min(triangle::scene_bot.x, lo_x);
    triangle::scene_bot.y = std::min(triangle::scene_bot.y, lo_y);
    triangle::scene_bot.z = std::min(triangle::scene_bot.z, lo_z);

    /* Calculate the normal */
    const point_t dir_b(this->vertex_b - this->vertex_a);
    const point_t dir_c(this->vertex_c - this->vertex_a);
    cross_product(dir_b, dir_c, &this->n);
    assert(this->n != 0.0f);
    
    /* Pick the major axis */
    float k_value, u_value, v_value, b_u_value, b_v_value, c_u_value, c_v_value;
    if (fabs(this->n.x) > fabs(this->n.y))
    {
        if (fabs(this->n.x) > fabs(this->n.z))
        {
            this->k   = 0;
            k_value   = this->n.x;
            u_value   = this->n.y;
            v_value   = this->n.z;
            b_u_value = dir_b.y;
            b_v_value = dir_b.z;
            c_u_value = dir_c.y;
            c_v_value = dir_c.z;
        }
        else
        {
            this->k   = 2;
            this->a.x = this->vertex_a.z;
            this->a.y = this->vertex_a.x;
            this->a.z = this->vertex_a.y;
            k_value   = this->n.z;
            u_value   = this->n.x;
            v_value   = this->n.y;
            b_u_value = dir_b.x;
            b_v_value = dir_b.y;
            c_u_value = dir_c.x;
            c_v_value = dir_c.y;
        }
    }
    else
    {
        if (fabs(this->n.y) > fabs(this->n.z))
        {
            this->k   = 1;
            this->a.x = this->vertex_a.y;
            this->a.y = this->vertex_a.z;
            this->a.z = this->vertex_a.x;
            k_value   = this->n.y;
            u_value   = this->n.z;
            v_value   = this->n.x;
            b_u_value = dir_b.z;
            b_v_value = dir_b.x;
            c_u_value = dir_c.z;
            c_v_value = dir_c.x;
        }
        else
        {
            this->k   = 2;
            this->a.x = this->vertex_a.z;
            this->a.y = this->vertex_a.x;
            this->a.z = this->vertex_a.y;
            k_value   = this->n.z;
            u_value   = this->n.x;
            v_value   = this->n.y;
            b_u_value = dir_b.x;
            b_v_value = dir_b.y;
            c_u_value = dir_c.x;
            c_v_value = dir_c.y;
        }
    }
    
    /* Precompute for intersection */
    float k_normal = 1.0/k_value;
    this->n_u = u_value * k_normal;
    this->n_v = v_value * k_normal;
    this->a.x = dot_product(this->n, this->vertex_a) * k_normal;
    
    float i_normal = 1.0/((b_u_value * c_v_value) - (b_v_value * c_u_value));
    this->b_n_u =  b_u_value * i_normal;
    this->b_n_v = -b_v_value * i_normal;
    this->c_n_u =  c_v_value * i_normal;
    this->c_n_v = -c_u_value * i_normal;
    
    /* Normalise the normal */
    normalise(&this->n);
    return;
}


/***********************************************************
 Constructor for the triangle.
 
 The constructor takes 3 points defining the vetices of the 
 triangle.
 
 The major axis and normal of the triangle are pre-calculated.
 The scaling factors for conversion to barycentric 
 co-ordinates and into the plane of the triangle are also 
 pre calculated. Pre calculation is done by the private 
 member this->calculate_pre_computes()
************************************************************/
inline triangle::triangle(material *const m, const point_t &a, const point_t &b, const point_t &c, bool l, const point_t *v_n, const point_t *v_t) : 
            m(m), a(a), vertex_a(a), vertex_b(b), vertex_c(c), n_u(0.0), n_v(0.0), b_n_u(0.0), b_n_v(0.0), c_n_u(0.0), c_n_v(0.0), k(0), light(l)
{ 
    /* Store vertex normals */
    if (v_n != nullptr)
    {
        this->vn = new point_t [3];
        this->vn[0] = v_n[0];
        this->vn[1] = v_n[1];
        this->vn[2] = v_n[2];
    }
    else
    {
        this->vn = nullptr;
    }
    
    /* Store texture vertex */
    if (v_t != nullptr)
    {
        this->vt = new point_t [3];
        this->vt[0] = v_t[0];
        this->vt[1] = v_t[1];
        this->vt[2] = v_t[2];
    }
    else
    {
        this->vt = nullptr;
    }
    
    this->calculate_pre_computes();
}


/***********************************************************
 is_intersecting returns the distance along the ray r that 
 the triangle and the ray intersect. If the triangle and the 
 ray do not intersect 'DOUBLE_MAX' is be returned.
 
 h should indicate how the ray is interacting with the primitive.
 
 The ray is intersected with an infinite plane embedding the 
 triangle. The point of intersection is check to be within 
 the triangle or not using barycentric co-ordinates.
************************************************************/
inline void triangle::is_intersecting(const ray *const r, hit_description *const h) const 
{
    /* Pick componants based on the major axis of the triangle */
    float r_k, r_u, r_v, dir_k, dir_u, dir_v;
    switch (this->k)
    {
        case  0 : r_k   = r->get_x0();
                  r_u   = r->get_y0();
                  r_v   = r->get_z0();
                  dir_k = r->get_x_grad();
                  dir_u = r->get_y_grad();
                  dir_v = r->get_z_grad();
                  break;
        case  1 : r_k   = r->get_y0();
                  r_u   = r->get_z0();
                  r_v   = r->get_x0();
                  dir_k = r->get_y_grad();
                  dir_u = r->get_z_grad();
                  dir_v = r->get_x_grad();
                  break;
        case  2 : r_k   = r->get_z0();
                  r_u   = r->get_x0();
                  r_v   = r->get_y0();
                  dir_k = r->get_z_grad();
                  dir_u = r->get_x_grad();
                  dir_v = r->get_y_grad();
                  break;
        default : assert(false);
                  break;
    }
    
    /* Calculate distance to intersection with the plane of the triangle */
    float ray_dot_normal = dir_k + (this->n_u * dir_u) + (this->n_v * dir_v);
    if (ray_dot_normal == 0.0f)
    {
        h->d = MAX_DIST;
        return;
    }

    /* NOTE -- this->a.x stores (n dot a) * (1.0 / k_value) */
    float dist = (this->a.x - r_k - (this->n_u * r_u) - (this->n_v * r_v)) / ray_dot_normal;
    if ((dist < EPSILON) || (dist > h->d))
    {
        h->d = MAX_DIST;
        return;
    }

    /* Check intersection is within the bounds of the triangle */
    /* NOTE -- this->a has already been translated into the major axis */
    float h_u  = (r_u - this->a.y) + (dist * dir_u);
    float h_v  = (r_v - this->a.z) + (dist * dir_v);
    float beta = (h_v * this->b_n_u) + (h_u  * this->b_n_v);
    if (beta < 0.0f)
    {
        h->d = MAX_DIST;
        return;
    }
    
    float gamma = (h_u * this->c_n_u) + (h_v * this->c_n_v);
    if ((gamma < 0.0f) || ((beta + gamma) > 1.0f))
    {
        h->d = MAX_DIST;
        return;
    }
    
    /* Record the hit and return */
    h->d = dist;
    h->u = beta;
    h->v = gamma;
    return;
}


#ifdef SIMD_PACKET_TRACING
/***********************************************************
 
************************************************************/
inline void triangle::is_intersecting(const packet_ray *const r, packet_hit_description *const h, const triangle **const i_o, const unsigned int size) const 
{
    for (unsigned int i = 0; i < size; i++)
    {
        /* Pick componants based on the major axis of the triangle */
        const vfp_t r_k     = r[i].get_ogn(          this->k     );
        const vfp_t r_u     = r[i].get_ogn(mod_3_lut[this->k + 1]);
        const vfp_t r_v     = r[i].get_ogn(mod_3_lut[this->k + 2]);
        const vfp_t dir_k   = r[i].get_dir(          this->k     );
        const vfp_t dir_u   = r[i].get_dir(mod_3_lut[this->k + 1]);
        const vfp_t dir_v   = r[i].get_dir(mod_3_lut[this->k + 2]);
                         
        /* Calculate distance to intersection with the plane of the triangle */
        /* NOTE -- this->a.x stores (n dot a) * (1.0 / k_value) */
        const vfp_t rdn (dir_k + (vfp_t(this->n_u) * dir_u) + (vfp_t(this->n_v) * dir_v));
        const vfp_t dist((vfp_t(this->a.x) - r_k - (vfp_t(this->n_u) * r_u) - (vfp_t(this->n_v) * r_v)) * inverse(rdn));
    
        vfp_t mask(dist > vfp_zero);
        mask    &= (h[i].d > dist);
        
        /* Check intersection is within the bounds of the triangle */
        /* NOTE -- this->a has already been translated into the major axis */
        const vfp_t h_u  ((r_u - vfp_t(  this->a.y)) + (dist * dir_u));
        const vfp_t h_v  ((r_v - vfp_t(  this->a.z)) + (dist * dir_v));
        const vfp_t beta ((h_v * vfp_t(this->b_n_u)) + (h_u  * vfp_t(this->b_n_v)));
        const vfp_t gamma((h_u * vfp_t(this->c_n_u)) + (h_v  * vfp_t(this->c_n_v)));
    
        mask    &= (beta  >= vfp_zero);
        mask    &= (gamma >= vfp_zero);
        mask    &= ((beta + gamma) <= vfp_one);

        int int_mask = move_mask(mask);
        if (int_mask > 0)
        {
            /* Record the hit and return */
            h[i].d  = mov_p(mask, dist,  h[i].d);
            h[i].u  = mov_p(mask, beta,  h[i].u);
            h[i].v  = mov_p(mask, gamma, h[i].v);
            
            if ((int_mask & 0x1) > 0)
            {
                i_o[(i << LOG2_SIMD_WIDTH)    ] = this;
            }

            if ((int_mask & 0x2) > 0)
            {
                i_o[(i << LOG2_SIMD_WIDTH) + 1] = this;
            }

            if ((int_mask & 0x4) > 0)
            {
                i_o[(i << LOG2_SIMD_WIDTH) + 2] = this;
            }

            if ((int_mask & 0x8) > 0)
            {
                i_o[(i << LOG2_SIMD_WIDTH) + 3] = this;
            }
        }
    }
    
    return;
}


/***********************************************************
 
************************************************************/
inline void triangle::is_intersecting(const frustrum &f, const packet_ray *const r, packet_hit_description *const h, const triangle **const i_o, const unsigned *c, const unsigned size) const 
{
    /* Edge culling */
    /* Frustrum dot triangle */
    const vfp_t full_fdn(((f.get_dir(0) * vfp_t(n.x)) + (f.get_dir(1) * vfp_t(n.y)) + (f.get_dir(2) * vfp_t(n.z))));
    const int f_dir = move_mask(full_fdn);

    /* Can only use if coherency check relative to the triangle passes */
    if ((f_dir == 0x0) || (f_dir == ((1 << SIMD_WIDTH) - 1)))
    {
        /* Pick componants based on the major axis of the triangle */
        const vfp_t r_k     = f.get_ogn(          this->k     );
        const vfp_t r_u     = f.get_ogn(mod_3_lut[this->k + 1]);
        const vfp_t r_v     = f.get_ogn(mod_3_lut[this->k + 2]);
        const vfp_t dir_k   = f.get_dir(          this->k     );
        const vfp_t dir_u   = f.get_dir(mod_3_lut[this->k + 1]);
        const vfp_t dir_v   = f.get_dir(mod_3_lut[this->k + 2]);
                         
        /* Calculate distance to intersection with the plane of the triangle */
        /* NOTE -- this->a.x stores (n dot a) * (1.0 / k_value) */
        const vfp_t rdn (dir_k + (vfp_t(this->n_u) * dir_u) + (vfp_t(this->n_v) * dir_v));
        const vfp_t dist((vfp_t(this->a.x) - r_k - (vfp_t(this->n_u) * r_u) - (vfp_t(this->n_v) * r_v)) * inverse(rdn));

/* Near and far plane culling */
//        if ((move_mask(dist > f.get_tfar() ) == ((1 << SIMD_WIDTH) - 1)) || 
//            (move_mask(dist < f.get_torg0()) == ((1 << SIMD_WIDTH) - 1)))
//        {
//            return;
//        }
/* End near and far plane culling */

        const vfp_t h_u  ((r_u - vfp_t(  this->a.y)) + (dist * dir_u));
        const vfp_t h_v  ((r_v - vfp_t(  this->a.z)) + (dist * dir_v));
        const vfp_t beta ((h_v * vfp_t(this->b_n_u)) + (h_u  * vfp_t(this->b_n_v)));
        const vfp_t gamma((h_u * vfp_t(this->c_n_u)) + (h_v  * vfp_t(this->c_n_v)));
        vfp_t valid = (beta >= vfp_zero);
        const vfp_t valid0 = (gamma          >= vfp_zero);
        const vfp_t valid1 = ((beta + gamma) <= vfp_one);

        if ((move_mask(valid) == 0x0) || (move_mask(valid0) == 0x0) || (move_mask(valid1) == 0x0))
        {
            return;
        }

#ifdef APERTURE_CULLING
        valid  &= (valid0 & valid1);
        // const int m0  = move_mask(valid); 

        /* check_aperture = 0 -- ignore (hit is guaranteed), 1 -- check */
        /* u.v checks maybe dropped if all the rays hit this triangle */
        /* the frustrum tfar may be updated if all the rays hit this triangle */
        // int check_aperture = (m0 != 0xf); 

/* Near and far plane culling */
//        if (check_aperture == 0)
//        {
//            if (f.neg_dir())
//            {
//                if (move_mask(dist < f.get_torg1()) == 0xf)
//                {
//                    f.set_tfar(dist);
//                }
//
//            }
//            else
//            {
//                if (move_mask(dist > f.get_torg1()) == 0xf) 
//                {
//                    f.set_tfar(dist);
//                }
//            }
//        }
/* End near and far plane culling */
#endif
    }
/* End edge culling */ 
    
    /* Perform full intersection test */
    for (unsigned i = 0; i < size; i++) 
    {
        const unsigned int pkt_addr = c[i];

        /* Pick componants based on the major axis of the triangle */
        const vfp_t r_k     = r[pkt_addr].get_ogn(          this->k     );
        const vfp_t r_u     = r[pkt_addr].get_ogn(mod_3_lut[this->k + 1]);
        const vfp_t r_v     = r[pkt_addr].get_ogn(mod_3_lut[this->k + 2]);
        const vfp_t dir_k   = r[pkt_addr].get_dir(          this->k     );
        const vfp_t dir_u   = r[pkt_addr].get_dir(mod_3_lut[this->k + 1]);
        const vfp_t dir_v   = r[pkt_addr].get_dir(mod_3_lut[this->k + 2]);
                         
        /* Calculate distance to intersection with the plane of the triangle */
        /* NOTE -- this->a.x stores (n dot a) * (1.0 / k_value) */
        const vfp_t rdn (dir_k + (vfp_t(this->n_u) * dir_u) + (vfp_t(this->n_v) * dir_v));
        const vfp_t dist((vfp_t(this->a.x) - r_k - (vfp_t(this->n_u) * r_u) - (vfp_t(this->n_v) * r_v)) * inverse(rdn));
    
        vfp_t mask(dist > vfp_zero);
        mask    &= (h[pkt_addr].d > dist);
        
        /* Check intersection is within the bounds of the triangle */
        /* NOTE -- this->a has already been translated into the major axis */
        const vfp_t h_u  ((r_u - vfp_t(  this->a.y)) + (dist * dir_u));
        const vfp_t h_v  ((r_v - vfp_t(  this->a.z)) + (dist * dir_v));
        const vfp_t beta ((h_v * vfp_t(this->b_n_u)) + (h_u  * vfp_t(this->b_n_v)));
        const vfp_t gamma((h_u * vfp_t(this->c_n_u)) + (h_v  * vfp_t(this->c_n_v)));
    
        /* These checks may be skipped if check_aperture == 0 */
        mask    &= (beta  >= vfp_zero);
        mask    &= (gamma >= vfp_zero);
        mask    &= ((beta + gamma) <= vfp_one);

        int int_mask = move_mask(mask);
        if (int_mask > 0)
        {
            /* Record the hit and return */
            h[pkt_addr].d  = mov_p(mask, dist,  h[pkt_addr].d);
            h[pkt_addr].u  = mov_p(mask, beta,  h[pkt_addr].u);
            h[pkt_addr].v  = mov_p(mask, gamma, h[pkt_addr].v);
            
            if ((int_mask & 0x1) > 0)
            {
                i_o[(pkt_addr << LOG2_SIMD_WIDTH)    ] = this;
            }

            if ((int_mask & 0x2) > 0)
            {
                i_o[(pkt_addr << LOG2_SIMD_WIDTH) + 1] = this;
            }

            if ((int_mask & 0x4) > 0)
            {
                i_o[(pkt_addr << LOG2_SIMD_WIDTH) + 2] = this;
            }

            if ((int_mask & 0x8) > 0)
            {
                i_o[(pkt_addr << LOG2_SIMD_WIDTH) + 3] = this;
            }
        }
    }

    return;
}
#endif /* #ifdef SIMD_PACKET_TRACING */


/***********************************************************
 normal_at_point returns a line normal to the triangle at the 
 point p. h is used to determine from which side the triangle 
 is hit.
 
 Return the precalculated normal or the interpolated normal
 if vertex normals are used. Also set weather the ray is
 entering of leaving the triangle.
************************************************************/
inline line triangle::normal_at_point(ray *const r, hit_description *const h) const
{
    const float denom  = dot_product(this->n, r->get_dir());
#ifdef SINGLE_PRECISION
    /* Re-calculate the intesection of the ray with the plane of the triangle */
    /* This gives a more accurate hit point to adjust the ray to */
    const float num    = dot_product(this->n, (this->vertex_c - r->get_dst()));
    r->change_length(num/denom);
#endif /* #ifdef SINGLE_PRECISION */

    /* Interpolate the vertex normals */
    point_t normal;
    if (this->vn != nullptr)
    {
        normal = (h->u * this->vn[2]) + (h->v * this->vn[1]) + ((1.0f - (h->u + h->v)) * this->vn[0]);
    }
    else
    {
        normal = this->n;
    }

    /* Construct the normal line */
    line l(r->get_dst(), normal);
    
    /* If the line is leaving the volume enclosed by 
       the triangle use the opposite normal */
    if (denom > 0.0)
    {
        l.opposite_dir();
        h->h = hit_t::in_out;
    }
    else
    {
        h->h = hit_t::out_in;
    }
    
    return l;
}


/***********************************************************
  get_random point returns a periodically random, evenly 
  distribute point on the triangle.

  This is incorrect and for a cube.
************************************************************/
inline point_t triangle::get_random_point(const int i) const
{
    float x_h8 = (this->highest_x() - this->lowest_x())/8.0;
    float y_h8 = (this->highest_y() - this->lowest_y())/8.0;
    float z_h8 = (this->highest_z() - this->lowest_z())/8.0;
    
    float x = this->vertex_c.x + (static_cast<float> (i     & 0x3) * x_h8) + (gen_random_mersenne_twister() * x_h8);
    float y = this->vertex_c.z + (static_cast<float>((i>>2) & 0x3) * y_h8) + (gen_random_mersenne_twister() * y_h8);
    float z = this->vertex_c.y + (static_cast<float>((i>>2) & 0x3) * z_h8) + (gen_random_mersenne_twister() * z_h8);
    
    return point_t(x,y,z);
}


/***********************************************************
  .
************************************************************/
inline void triangle::find_rays(ray *const r, const point_t &d, const int n) const
{
    /* Scales */
    const float beta_scale    = 1.0f / static_cast<float>((n >> 4) + ((n & 0xe) != 0) + 1);
    const float gamma_scale   = 1.0f / static_cast<float>((n >> 4) + ((n & 0xc) != 0) + 1);

    /* Create n rays, each with a random offset in a fixed volume */
    for (int i = 0; i < n; i++)
    {
        /* Generate a legal barycenrtic co-ordinate */
        const int eigths    = (i >> 2) & ~0x1;
        const float beta     = gen_random_mersenne_twister() * (beta_scale  * static_cast<float>(eigths + (i & 0x1)));
        const float gamma    = gen_random_mersenne_twister() * (gamma_scale * static_cast<float>(eigths + (i & 0x2))) * (1.0f - beta);
        const float alpha    =1.0f - (beta + gamma);

        /* Convert to cartesian co-ordinates */
        point_t p = (this->vertex_a * alpha) + (this->vertex_b * beta) + (this->vertex_c * gamma);
        
        /* Create a new ray */
        r[i].set_up(d, p);
    }
    
    return;
}


inline void intersect(const point_t &v, const point_t &d, float *const interval)
{
    (interval[0]) = v.x + (v.y - v.x) * d.x / (d.x - d.y);
    (interval[1]) = v.x + (v.z - v.x) * d.x / (d.x - d.z);
    
    return;
}


inline void compute_intervals(const point_t &v, const point_t &d, const float d0d1, const float d0d2, float *const interval)
{
    if (d0d1 > 0.0f)                      
    {                                  
        /* here we know that d0d2<=0.0 */
        /* that is d0, d1 are on the same side, d2 on the other or on the plane */
        intersect(point_t(v.z, v.x, v.y), point_t(d.z, d.x, d.y), interval);       
    }                                                  
    else if (d0d2 > 0.0f)                                 
    {                                                  
        /* here we know that d0d1<=0.0 */                
        intersect(point_t(v.y, v.x, v.z), point_t(d.y, d.x, d.z), interval);       
    }                                                  
    else if (((d.y * d.z) > 0.0f) || (d.x != 0.0f))
    {                                                  
        /* here we know that d0d1<=0.0 or that d0!=0.0 */
        intersect(v, d, interval);       
    }                                                  
    else if (d.y != 0.0f)                                  
    {                                                  
        intersect(point_t(v.y, v.x, v.z), point_t(d.y, d.x, d.z), interval);       
    }                                                  
    else if (d.z != 0.0f)                                  
    {                                                  
        intersect(point_t(v.z, v.x, v.y), point_t(d.z, d.x, d.y), interval);       
    }                                                  
    else                                               
    {
        /* triangles are coplanar */
        std::cout << "co planar" << std::endl;
        return;// coplanar_tri_tri(n1,v0,v1,v2,u0,u1,u2);
    }
}


/***********************************************************
  .
************************************************************/
inline float triangle::is_intersecting(const triangle *const t, const point_t &tp, const point_t &p) const
{
    /* Test the vertices of t are on the same side of this */
    const float this_d   = dot_product(-this->n, this->vertex_a + p);
    const float t_da     = dot_product(t->vertex_a + tp, this->n) + this_d;
    const float t_db     = dot_product(t->vertex_b + tp, this->n) + this_d;
    const float t_dc     = dot_product(t->vertex_c + tp, this->n) + this_d;
    if (((t_da < 0.0) && (t_db < 0.0) && (t_dc < 0.0)) ||
        ((t_da > 0.0) && (t_db > 0.0) && (t_dc > 0.0)))
    {
        /* No collision possible */
        return 0.0;
    }

    /* Test the vertices of this are on the same side of t */
    const float t_d      = dot_product(-t->n, t->vertex_a + tp);
    const float this_da  = dot_product(this->vertex_a + p, t->n) + t_d;
    const float this_db  = dot_product(this->vertex_b + p, t->n) + t_d;
    const float this_dc  = dot_product(this->vertex_c + p, t->n) + t_d;
    if (((this_da < 0.0) && (this_db < 0.0) && (this_dc < 0.0)) ||
        ((this_da > 0.0) && (this_db > 0.0) && (this_dc > 0.0)))
    {
        /* No collision possible */
        return 0.0;
    }
    
    /* Compute the line of intersection */
    point_t l;
    cross_product(this->n, t->n, &l);
    
    /* compute and index to the largest component of l */
    float max_l  = fabs(l.x);
    float b      = fabs(l.y);
    float c      = fabs(l.z);
    int  index  = 0;
    if(b > max_l) 
    {
        index   = 1;
    }

    if(c > max_l)
    {
        index   = 2;
    }

    /* this is the simplified projection onto l */
    const point_t this_p(this->vertex_a[index] +  p[index], 
                         this->vertex_b[index] +  p[index],
                         this->vertex_c[index] +  p[index]);
    const point_t    t_p(   t->vertex_a[index] + tp[index],    
                            t->vertex_b[index] + tp[index],    
                            t->vertex_c[index] + tp[index]);

    /* compute interval for triangle 1 */
    float this_inter[2] = { 0.0f, 0.0f };
    compute_intervals(this_p, point_t(this_da, this_db, this_dc), (this_da * this_db), (this_da * this_dc), this_inter);

    /* compute interval for triangle 2 */
    float t_inter[2] = { 0.0f, 0.0f };
    compute_intervals(t_p, point_t(t_da, t_db, t_dc), (t_da * this_db), (t_da * this_dc), t_inter);

    const float this_lower   = std::min(this_inter[0], this_inter[1]);
    const float this_upper   = std::max(this_inter[0], this_inter[1]);
    const float t_lower      = std::min(t_inter[0],    t_inter[1]);
    const float t_upper      = std::max(t_inter[0],    t_inter[1]);

    if((this_upper < t_lower) || (t_upper < this_lower))
    {
        return 0.0f;
    }

    /* Collision */
    return std::min((this_upper - t_lower), (t_upper - this_lower));
}


/* Useful sort criteria for triangles */
class sort_triangle_by_lowest_x
{
    public :
        bool operator() (const triangle *const a, const triangle *const b) { return (a->lowest_x() < b->lowest_x()); }
};


class sort_triangle_by_lowest_y
{
    public :
        bool operator() (const triangle *const a, const triangle *const b) { return (a->lowest_y() < b->lowest_y()); }
};


class sort_triangle_by_lowest_z
{
    public :
        bool operator() (const triangle *const a, const triangle *const b) { return (a->lowest_z() < b->lowest_z()); }
};


class sort_triangle_by_highest_x
{
     public :
        bool operator() (const triangle *const a, const triangle *const b) { return (a->highest_x() < b->highest_x()); }
};


class sort_triangle_by_highest_y
{
     public :
        bool operator() (const triangle *const a, const triangle *const b) { return (a->highest_y() < b->highest_y()); }
};


class sort_triangle_by_highest_z
{
    public :
        bool operator() (const triangle *const a, const triangle *const b) { return (a->highest_z() < b->highest_z()); }
};
}; /* namespace raptor_raytracer */

#endif /* #ifndef __TRIANGLE_H__ */
