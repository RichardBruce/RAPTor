#ifndef __TRIANGLE_H__
#define __TRIANGLE_H__

/* Boost headers */
#include "boost/noncopyable.hpp"

/* Ray tracer headers */
#include "common.h"
#include "line.h"
#include "ray.h"
#include "secondary_ray_data.h"
#ifdef SIMD_PACKET_TRACING
#include "packet_ray.h"
#include "frustrum.h"
#endif
#include "material.h"


namespace raptor_raytracer
{
class triangle : boost::noncopyable
{
    public :
        triangle(material *const m, const point_t &a, const point_t &b, const point_t &c, bool l = false, const point_t *v_n = nullptr, const point_t *v_t = nullptr);
        ~triangle()
        {
            /* REVISIT -- Consider using indices for these */
            if (this->vnt != nullptr)
            {
                delete [] this->vnt;
            }
        };

        triangle(triangle &&r) :
            m(r.m), vnt(r.vnt), vertex_a(r.vertex_a), vertex_b(r.vertex_b), vertex_c(r.vertex_c), n(r.n)
        {
            r.vnt = nullptr;
        }
        
        /* Find out if this is a light/transparent object for shadow rays to ignore */
        bool get_light()            const { return this->m & 0x1;                           }
        bool is_transparent()       const { return get_material()->is_transparent();        }

        /* Shadow ray targetting */
        const point_t& get_centre() const { return this->vertex_c;                          }
        
        /* Frustrum separation algorithm */
        point_t get_vertex_a()      const { return this->vertex_a;                          }
        point_t get_vertex_b()      const { return this->vertex_b;                          }
        point_t get_vertex_c()      const { return this->vertex_c;                          }

        /* KD-tree node classification */
        float get_x0()              const { return this->vertex_c.x;                        }
        float get_y0()              const { return this->vertex_c.y;                        }
        float get_z0()              const { return this->vertex_c.z;                        }

        point_t low_bound()         const { return min(vertex_a, min(vertex_b, vertex_c));  }
        point_t high_bound()        const { return max(vertex_a, max(vertex_b, vertex_c));  }
        
        /* Ray tracing functions */
        inline void is_intersecting(const ray *const r, hit_description *const h) const;
#ifdef SIMD_PACKET_TRACING
        inline void is_intersecting(const packet_ray *const r, packet_hit_description *const h, const triangle **const i_o, const unsigned int size) const;
        inline void is_intersecting(const frustrum &f, const packet_ray *const r, packet_hit_description *const h, const triangle **const i_o, const unsigned *c, const unsigned size) const;
#endif
        inline point_t normal_at_point(ray *const r, hit_description *const h) const;
        inline void find_rays(ray *const r, const point_t &d, const int n) const;
        
        /* Generate secondary rays for packet tracing */
        inline point_t generate_rays(const ray_trace_engine &r, ray &i, point_t *const text, hit_description *const h, secondary_ray_data *const rl, secondary_ray_data *const rf) const
        {
            /* Calculate the normal */
            point_t norm(normal_at_point(&i, h));

            /* Interpolate the texture co-ordinate if possible */
            point_t vt(MAX_DIST);
            if ((this->vnt != nullptr) && (this->vnt[3].x != MAX_DIST))
            {
                vt = (h->u * this->vnt[5]) + (h->v * this->vnt[4]) + ((1.0f - (h->u + h->v)) * this->vnt[3]);
            }

            get_material()->generate_rays(r, i, &norm, vt, h->h, rl, rf);

            (*text) = vt;
            return norm;
        }
        
        /* Call the materials shader with the triangles normal */
        inline void shade(const ray_trace_engine &r, ray &i, const point_t &n, const point_t &vt, const hit_description &h, ext_colour_t *const c) const
        {
            /* Call the material shader */
            get_material()->shade(r, i, n, h.h, c, vt);
        }
        
        inline void combind_secondary_rays(const ray_trace_engine &r, ext_colour_t *const c, const secondary_ray_data &rl, const secondary_ray_data &rf) const
        {
            get_material()->combind_secondary_rays(r, c, rl, rf);
        }

    private : 
        material * get_material() const
        {
            return reinterpret_cast<material *>(this->m & ~0x1);
        }

        static_assert(sizeof(std::int64_t) == sizeof(material *), "Error: Material pointers dont fit in std::int64_t");
        std::int64_t    m;          /* Pointer to the triangles shader          */
        point_t       * vnt;        /* Pointer to vertex normals and textures   */
        point_t         vertex_a;   /* Vertex a of the triangle                 */
        point_t         vertex_b;   /* Vertex b of the triangle                 */
        point_t         vertex_c;   /* Vertex c of the triangle                 */
        point_t         n;          /* Normal of the triangle                   */
};


/***********************************************************
 Constructor for the triangle.
 
 The constructor takes 3 points defining the vetices of the 
 triangle.
 
 The normal and the bounding box of the triangle are 
 precomputed.
************************************************************/
inline triangle::triangle(material *const m, const point_t &a, const point_t &b, const point_t &c, bool l, const point_t *v_n, const point_t *v_t) : 
    m(reinterpret_cast<std::int64_t>(m)), vertex_a(a), vertex_b(b), vertex_c(c)
{ 
    /* Vertex checks */
    assert(this->vertex_a != this->vertex_b);
    assert(this->vertex_a != this->vertex_c);
    assert(this->vertex_b != this->vertex_c);

    if (l)
    {
        this->m |= 0x1;
    }

    if ((v_n != nullptr) || (v_t != nullptr))
    {
        this->vnt = new point_t [6];
    }
    else
    {
        this->vnt = nullptr;
    }

    /* Store vertex normals */
    if (v_n != nullptr)
    {
        this->vnt[0] = v_n[0];
        this->vnt[1] = v_n[1];
        this->vnt[2] = v_n[2];
    }
    else if (v_t != nullptr)
    {
        this->vnt[0].x = MAX_DIST;
    }

    /* Store texture vertex */
    if (v_t != nullptr)
    {
        this->vnt[3] = v_t[0];
        this->vnt[4] = v_t[1];
        this->vnt[5] = v_t[2];
    }
    else if (v_n != nullptr)
    {
        this->vnt[3].x = MAX_DIST;
    }

    /* Calculate the normal */
    const point_t dir_b(this->vertex_b - this->vertex_a);
    const point_t dir_c(this->vertex_c - this->vertex_a);
    cross_product(dir_b, dir_c, &this->n);
    assert(this->n != 0.0);
    normalise(&this->n);
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
    /* Find vectors for two edges sharing V1 */
    const point_t e1(vertex_c - vertex_a);
    const point_t e2(vertex_b - vertex_a);
    
    /* Begin calculating determinant - also used to calculate u parameter */
    const point_t P(cross_product(r->get_dir(), e2));

    /* if determinant is near zero, ray lies in plane of triangle */
    const float det = dot_product(e1, P);
    if (det > -0.000001f && det < 0.000001f)
    {
        h->d = MAX_DIST;
        return;
    }
    const float inv_det = 1.0f / det;

    /* calculate distance from V1 to ray origin */
    const point_t T(r->get_ogn() - vertex_a);

    /* Calculate u parameter and test bound */
    const float u = dot_product(T, P) * inv_det;
    if (u < 0.0f)
    {
        h->d = MAX_DIST;
        return;
    }

    /* Calculate V parameter and test bound */
    const point_t Q(cross_product(T, e1));
    const float v = dot_product(r->get_dir(), Q) * inv_det;
    if ((v < 0.0f) || ((u + v) > 1.0f))
    {
        h->d = MAX_DIST;
        return;
    }

    const float t = dot_product(e2, Q) * inv_det;
    if(t > EPSILON)
    {
        h->d = t;
        h->u = u;
        h->v = v;
    }
}


#ifdef SIMD_PACKET_TRACING
/***********************************************************
 
************************************************************/
inline void triangle::is_intersecting(const packet_ray *const r, packet_hit_description *const h, const triangle **const i_o, const unsigned int size) const 
{
    /* Find vectors for two edges sharing V1 */
    const vfp_t e1_x(vertex_c.x - vertex_a.x);
    const vfp_t e1_y(vertex_c.y - vertex_a.y);
    const vfp_t e1_z(vertex_c.z - vertex_a.z);
    const vfp_t e2_x(vertex_b.x - vertex_a.x);
    const vfp_t e2_y(vertex_b.y - vertex_a.y);
    const vfp_t e2_z(vertex_b.z - vertex_a.z);

    for (unsigned int i = 0; i < size; ++i)
    {
        /* Pick componants based on the major axis of the triangle */
        const vfp_t r_x(r[i].get_ogn(0));
        const vfp_t r_y(r[i].get_ogn(1));
        const vfp_t r_z(r[i].get_ogn(2));
        const vfp_t dir_x(r[i].get_dir(0));
        const vfp_t dir_y(r[i].get_dir(1));
        const vfp_t dir_z(r[i].get_dir(2));
    
        /* Begin calculating determinant - also used to calculate u parameter */
        const vfp_t p_x((dir_y * e2_z) - (e2_y * dir_z));
        const vfp_t p_y((dir_z * e2_x) - (e2_z * dir_x));
        const vfp_t p_z((dir_x * e2_y) - (e2_x * dir_y));

        /* if determinant is near zero, ray lies in plane of triangle */
        const vfp_t det((e1_x * p_x) + (e1_y * p_y) + (e1_z * p_z));
        const vfp_t inv_det = inverse(det);//1.0f / det;

        /* calculate distance from V1 to ray origin */
        const vfp_t t_x(r_x - vertex_a.x);
        const vfp_t t_y(r_y - vertex_a.y);
        const vfp_t t_z(r_z - vertex_a.z);

        /* Calculate u parameter and test bound */
        const vfp_t u(((t_x * p_x) + (t_y * p_y) + (t_z * p_z)) * inv_det);

        /* Calculate V parameter and test bound */
        const vfp_t q_x((t_y * e1_z) - (e1_y * t_z));
        const vfp_t q_y((t_z * e1_x) - (e1_z * t_x));
        const vfp_t q_z((t_x * e1_y) - (e1_x * t_y));
        const vfp_t v(((dir_x * q_x) + (dir_y * q_y) + (dir_z * q_z)) * inv_det);

        const vfp_t dist(((e2_x * q_x) + (e2_y * q_y) + (e2_z * q_z)) * inv_det);
        
        vfp_t mask(dist > vfp_zero);
        mask    &= (h[i].d > dist);
        mask    &= (u >= vfp_zero);
        mask    &= (v >= vfp_zero);
        mask    &= ((u + v) <= vfp_one);

        int int_mask = move_mask(mask);
        if (int_mask > 0)
        {
            /* Record the hit and return */
            h[i].d  = mov_p(mask, dist,  h[i].d);
            h[i].u  = mov_p(mask, u,  h[i].u);
            h[i].v  = mov_p(mask, v, h[i].v);
            
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
//     const vfp_t full_fdn(((f.get_dir(0) * vfp_t(n.x)) + (f.get_dir(1) * vfp_t(n.y)) + (f.get_dir(2) * vfp_t(n.z))));
//     const int f_dir = move_mask(full_fdn);

//     /* Can only use if coherency check relative to the triangle passes */
//     if ((f_dir == 0x0) || (f_dir == ((1 << SIMD_WIDTH) - 1)))
//     {
//         /* Pick componants based on the major axis of the triangle */
//         const vfp_t r_k     = f.get_ogn(          this->k     );
//         const vfp_t r_u     = f.get_ogn(mod_3_lut[this->k + 1]);
//         const vfp_t r_v     = f.get_ogn(mod_3_lut[this->k + 2]);
//         const vfp_t dir_k   = f.get_dir(          this->k     );
//         const vfp_t dir_u   = f.get_dir(mod_3_lut[this->k + 1]);
//         const vfp_t dir_v   = f.get_dir(mod_3_lut[this->k + 2]);
                         
//         /* Calculate distance to intersection with the plane of the triangle */
//         /* NOTE -- this->a.x stores (n dot a) * (1.0 / k_value) */
//         const vfp_t rdn (dir_k + (vfp_t(this->n_u) * dir_u) + (vfp_t(this->n_v) * dir_v));
//         const vfp_t dist((vfp_t(this->a.x) - r_k - (vfp_t(this->n_u) * r_u) - (vfp_t(this->n_v) * r_v)) * inverse(rdn));

// /* Near and far plane culling */
// //        if ((move_mask(dist > f.get_tfar() ) == ((1 << SIMD_WIDTH) - 1)) || 
// //            (move_mask(dist < f.get_torg0()) == ((1 << SIMD_WIDTH) - 1)))
// //        {
// //            return;
// //        }
// /* End near and far plane culling */

//         const vfp_t h_u  ((r_u - vfp_t(  this->a.y)) + (dist * dir_u));
//         const vfp_t h_v  ((r_v - vfp_t(  this->a.z)) + (dist * dir_v));
//         const vfp_t beta ((h_v * vfp_t(this->b_n_u)) + (h_u  * vfp_t(this->b_n_v)));
//         const vfp_t gamma((h_u * vfp_t(this->c_n_u)) + (h_v  * vfp_t(this->c_n_v)));
//         vfp_t valid = (beta >= vfp_zero);
//         const vfp_t valid0 = (gamma          >= vfp_zero);
//         const vfp_t valid1 = ((beta + gamma) <= vfp_one);

//         if ((move_mask(valid) == 0x0) || (move_mask(valid0) == 0x0) || (move_mask(valid1) == 0x0))
//         {
//             return;
//         }

// #ifdef APERTURE_CULLING
//         valid  &= (valid0 & valid1);
//         // const int m0  = move_mask(valid); 

//         /* check_aperture = 0 -- ignore (hit is guaranteed), 1 -- check */
//         /* u.v checks maybe dropped if all the rays hit this triangle */
//         /* the frustrum tfar may be updated if all the rays hit this triangle */
//         // int check_aperture = (m0 != 0xf); 

// /* Near and far plane culling */
// //        if (check_aperture == 0)
// //        {
// //            if (f.neg_dir())
// //            {
// //                if (move_mask(dist < f.get_torg1()) == 0xf)
// //                {
// //                    f.set_tfar(dist);
// //                }
// //
// //            }
// //            else
// //            {
// //                if (move_mask(dist > f.get_torg1()) == 0xf) 
// //                {
// //                    f.set_tfar(dist);
// //                }
// //            }
// //        }
// /* End near and far plane culling */
// #endif
//     }
// /* End edge culling */ 

    /* Find vectors for two edges sharing V1 */
    const vfp_t e1_x(vertex_c.x - vertex_a.x);
    const vfp_t e1_y(vertex_c.y - vertex_a.y);
    const vfp_t e1_z(vertex_c.z - vertex_a.z);
    const vfp_t e2_x(vertex_b.x - vertex_a.x);
    const vfp_t e2_y(vertex_b.y - vertex_a.y);
    const vfp_t e2_z(vertex_b.z - vertex_a.z);

    /* Perform full intersection test */
    for (unsigned i = 0; i < size; i++) 
    {
        const unsigned int pkt_addr = c[i];

        /* Pick componants based on the major axis of the triangle */
        const vfp_t r_x(r[pkt_addr].get_ogn(0));
        const vfp_t r_y(r[pkt_addr].get_ogn(1));
        const vfp_t r_z(r[pkt_addr].get_ogn(2));
        const vfp_t dir_x(r[pkt_addr].get_dir(0));
        const vfp_t dir_y(r[pkt_addr].get_dir(1));
        const vfp_t dir_z(r[pkt_addr].get_dir(2));
    
        /* Begin calculating determinant - also used to calculate u parameter */
        const vfp_t p_x((dir_y * e2_z) - (e2_y * dir_z));
        const vfp_t p_y((dir_z * e2_x) - (e2_z * dir_x));
        const vfp_t p_z((dir_x * e2_y) - (e2_x * dir_y));

        /* if determinant is near zero, ray lies in plane of triangle */
        const vfp_t det((e1_x * p_x) + (e1_y * p_y) + (e1_z * p_z));
        const vfp_t inv_det = inverse(det);

        /* calculate distance from V1 to ray origin */
        const vfp_t t_x(r_x - vertex_a.x);
        const vfp_t t_y(r_y - vertex_a.y);
        const vfp_t t_z(r_z - vertex_a.z);

        /* Calculate u parameter and test bound */
        const vfp_t u(((t_x * p_x) + (t_y * p_y) + (t_z * p_z)) * inv_det);

        /* Calculate V parameter and test bound */
        const vfp_t q_x((t_y * e1_z) - (e1_y * t_z));
        const vfp_t q_y((t_z * e1_x) - (e1_z * t_x));
        const vfp_t q_z((t_x * e1_y) - (e1_x * t_y));
        const vfp_t v(((dir_x * q_x) + (dir_y * q_y) + (dir_z * q_z)) * inv_det);

        const vfp_t dist(((e2_x * q_x) + (e2_y * q_y) + (e2_z * q_z)) * inv_det);
        
        vfp_t mask(dist > vfp_zero);
        mask    &= (h[pkt_addr].d > dist);
        mask    &= (u >= vfp_zero);
        mask    &= (v >= vfp_zero);
        mask    &= ((u + v) <= vfp_one);

        int int_mask = move_mask(mask);
        if (int_mask > 0)
        {
            /* Record the hit and return */
            h[pkt_addr].d  = mov_p(mask, dist,  h[pkt_addr].d);
            h[pkt_addr].u  = mov_p(mask, u, h[pkt_addr].u);
            h[pkt_addr].v  = mov_p(mask, v, h[pkt_addr].v);
            
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
inline point_t triangle::normal_at_point(ray *const r, hit_description *const h) const
{
    const float denom  = dot_product(this->n, r->get_dir());
    /* Re-calculate the intesection of the ray with the plane of the triangle */
    /* This gives a more accurate hit point to adjust the ray to */
    const float num    = dot_product(this->n, (this->vertex_c - r->get_dst()));
    r->change_length(num / denom);

    /* Interpolate the vertex normals */
    point_t shader_norm;
    if ((this->vnt != nullptr) && (this->vnt[0].x != MAX_DIST))
    {
        shader_norm = (h->u * this->vnt[2]) + (h->v * this->vnt[1]) + ((1.0f - (h->u + h->v)) * this->vnt[0]);
    }
    else
    {
        shader_norm = this->n;
    }

    /* If the line is leaving the volume enclosed by 
       the triangle use the opposite normal */
    if (denom > 0.0f)
    {
        shader_norm = -shader_norm;
        r->set_geometry_normal(-this->n);
        h->h = hit_t::in_out;
    }
    else
    {
        r->set_geometry_normal(this->n);
        h->h = hit_t::out_in;
    }
    
    return shader_norm;
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
        const float beta    = gen_random_mersenne_twister() * (beta_scale  * static_cast<float>(eigths + (i & 0x1)));
        const float gamma   = gen_random_mersenne_twister() * (gamma_scale * static_cast<float>(eigths + (i & 0x2))) * (1.0f - beta);
        const float alpha   = 1.0f - (beta + gamma);

        /* Convert to cartesian co-ordinates */
        point_t p = (this->vertex_a * alpha) + (this->vertex_b * beta) + (this->vertex_c * gamma);
        
        /* Create a new ray */
        r[i].set_up(d, p);
    }
    
    return;
}
}; /* namespace raptor_raytracer */

#endif /* #ifndef __TRIANGLE_H__ */
