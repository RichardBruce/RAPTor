#pragma once

/* Standard headers */

/* Boost headers */
#include "boost/noncopyable.hpp"

/* Common headers */
#include "common.h"

/* Ray tracer headers */
#include "raytracer.h"
#include "triangle.h"
#include "ray.h"
#include "packet_ray.h"
#include "frustrum.h"


namespace raptor_raytracer
{
/* This is most of a bih node, but it doesnt know where is children are or which access it is split in */
class bih_node
{
    public :
        bih_node() = default;
        /* Allow default CTOR, DTOR, copy CTOR and assignment operator (for using in vector) */
        
        /* Tree construction */
        void create_leaf_node(const int b, const int e)
        {
            this->b = b;
            this->e = e;
        }

        void create_generic_node(const float l, const float r)
        {
            this->l = l;
            this->r = r;
        }
        
        /* Tree traversal */        
        float get_left_split() const
        {
            return this->l;
        }

        float get_right_split() const
        {
            return this->r;
        }

        /* Leaf node tests */
        bool is_empty() const
        {
            return (this->e < this->b);
        }

        int size() const
        {
            return (this->e - this->b) + 1;
        }

        int test_leaf_node_nearest(const primitive_store &e, const ray *const r, hit_description *const h) const
        {
            int end = this->e;
            int intersecting_object = -1;
            for (int i = this->b; i <= end; ++i)
            {
                hit_description hit_type(h->d);
                e.indirect_primitive(i)->is_intersecting(r, &hit_type);
                if (hit_type.d < h->d)
                {
                    *h = hit_type;
                    intersecting_object = e.indirection(i);
                }
            }
    
            return intersecting_object;
        }

        bool test_leaf_node_nearer(const primitive_store &e, const ray *const r, const float max) const
        {
            int end = this->e;
            for (int i = this->b; i <= end; ++i)
            {
                const auto *tri = e.indirect_primitive(i);
                if (tri->get_light() || tri->is_transparent())
                {
                    continue;
                }
        
                hit_description hit_type(max);
                tri->is_intersecting(r, &hit_type);
                if (hit_type.d < max)
                {
                    return true;
                }
            }

            return false;
        }

#ifdef SIMD_PACKET_TRACING
        void test_leaf_node_nearest(const primitive_store &e, const packet_ray *const r, vint_t *const i_o, packet_hit_description *const h, const unsigned int size) const
        {
            int end = this->e;
            for (int i = this->b; i <= end; ++i)
            {
                e.indirect_primitive(i)->is_intersecting(r, h, i_o, size, e.indirection(i));
            }
            
            return;
        }

        void test_leaf_node_nearest(const primitive_store &e, frustrum &f, const packet_ray *const r, vint_t *const i_o, packet_hit_description *const h, const unsigned *c, const unsigned int size) const
        {
            int end = this->e;
            for (int i = this->b; i <= end; ++i)
            {
                const auto *tri = e.indirect_primitive(i);
                const bool outside = f.cull(tri->get_vertex_a(), tri->get_vertex_b(), tri->get_vertex_c());
                if (!outside)
                {
                    tri->is_intersecting(f, r, h, i_o, c, size, e.indirection(i));
                }
            }

            return;
        }

        void test_leaf_node_nearer(const primitive_store &e, const packet_ray *const r, vfp_t *const c, const vfp_t &t, packet_hit_description *const h) const
        {
            vint_t i_o;
            int end = this->e;
            for (int i = this->b; i <= end; ++i)
            {
                const auto *tri = e.indirect_primitive(i);
                if (tri->get_light() || tri->is_transparent())
                {
                    continue;
                }
        
                tri->is_intersecting(r, h, &i_o, 1, e.indirection(i));
                (*c) |= (h->d < t);
                if (move_mask(*c) == ((1 << SIMD_WIDTH) - 1))
                {
                    return;
                }
            }

            return;
        }
        
        void test_leaf_node_nearer(const primitive_store &e, frustrum &f, const packet_ray *const r, vfp_t *const closer, const vfp_t *const t, packet_hit_description *const h, const unsigned *c, const unsigned int size) const
        {
            
            int end = this->e;
            vint_t i_o[MAXIMUM_PACKET_SIZE];
            for (int i = this->b; i <= end; ++i)
            {
                const auto *tri = e.indirect_primitive(i);
                if (tri->get_light() || tri->is_transparent())
                {
                    continue;
                }

                const bool outside = f.cull(tri->get_vertex_a(), tri->get_vertex_b(), tri->get_vertex_c());
                if (!outside)
                {
                    tri->is_intersecting(f, r, h, i_o, c, size, e.indirection(i));
                    vfp_t done = vfp_true;
                    for (unsigned int j = 0; j < size; ++j)
                    {
                        closer[c[j]] |= (h[c[j]].d < t[c[j]]);
                        done &= closer[c[j]];
                    }
                    
                    if (move_mask(done) == ((1 << SIMD_WIDTH) - 1))
                    {
                        return;
                    }
                }
            }

            return;
        }
#endif /* #ifdef SIMD_PACKET_TRACING */
        
    private : 
        /* Unions of mutually exclusive data */
        union
        {
            int     b;  /* Index of primitives                  */
            float   l;  /* Position of the left split plane     */
        };
        
        union
        {
            int     e;  /* Index of last primitive              */
            float   r;  /* Position of the right split plane    */
        };
};
}; /* namespace raptor_raytracer */
