#ifndef __BVH_NODE_H__
#define __BVH_NODE_H__

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
class bvh_node
{
    public :
        bvh_node() { };
        /* Allow (almost) default CTOR, DTOR, copy CTOR and assignment operator (for using in vector) */
        
        /* Tree construction */
        void create_leaf_node(const int b, const int e)
        {
            _b = b;
            _e = e;
        }

        void create_generic_node(const point_t &low, const point_t &high)
        {
            _low    = low;
            _high   = high;
        }
        
        /* Set statics */
        static void set_primitives(std::vector<triangle *> *const p) { bvh_node::o = p; }

        /* Tree traversal */        
        const point_t & low_point() const
        {
            return _low;
        }

        const point_t & high_point() const
        {
            return _high;
        }

        /* Leaf node tests */
        bool is_empty() const
        {
            return (_e < _b);
        }

        int size() const
        {
            return (_e - _b) + 1;
        }

        triangle * test_leaf_node_nearest(const ray *const r, hit_description *const h) const
        {
            triangle *intersecting_object = nullptr;
    
            int end = _e;
            for (int i = _b; i <= end; ++i)
            {
                hit_description hit_type(h->d);
                (*bvh_node::o)[i]->is_intersecting(r, &hit_type);
                if (hit_type.d < h->d)
                {
                    *h = hit_type;
                    intersecting_object = (*bvh_node::o)[i];
                }
            }
    
            return intersecting_object;
        }

        bool test_leaf_node_nearer(const ray *const r, const float max) const
        {
            int end = _e;
            for (int i = _b; i <= end; ++i)
            {
                if ((*bvh_node::o)[i]->get_light() || (*bvh_node::o)[i]->is_transparent())
                {
                    continue;
                }
        
                hit_description hit_type(max);
                (*bvh_node::o)[i]->is_intersecting(r, &hit_type);
                if (hit_type.d < max)
                {
                    return true;
                }
            }

            return false;
        }

#ifdef SIMD_PACKET_TRACING
        void test_leaf_node_nearest(const packet_ray *const r, const triangle **const i_o, packet_hit_description *const h, const unsigned int size) const
        {
            int end = _e;
            for (int i = _b; i <= end; ++i)
            {
                (*bvh_node::o)[i]->is_intersecting(r, h, i_o, size);
            }
            
            return;
        }

        void test_leaf_node_nearest(frustrum &f, const packet_ray *const r, const triangle **const i_o, packet_hit_description *const h, const unsigned *c, const unsigned int size) const
        {
            int end = _e;
            for (int i = _b; i <= end; ++i)
            {
                const bool outside = f.cull((*bvh_node::o)[i]->get_vertex_a(), (*bvh_node::o)[i]->get_vertex_b(), (*bvh_node::o)[i]->get_vertex_c());
                if (!outside)
                {
                    (*bvh_node::o)[i]->is_intersecting(f, r, h, i_o, c, size);
                }
            }

            return;
        }

        void test_leaf_node_nearer(const packet_ray *const r, vfp_t *const c, const vfp_t &t, packet_hit_description *const h) const
        {
            const triangle * i_o[SIMD_WIDTH];

            int end = _e;
            for (int i = _b; i <= end; ++i)
            {
                if ((*bvh_node::o)[i]->get_light() || (*bvh_node::o)[i]->is_transparent())
                {
                    continue;
                }
        
                (*bvh_node::o)[i]->is_intersecting(r, h, &i_o[0], 1);
                (*c) |= (h->d < t);
                if (move_mask(*c) == ((1 << SIMD_WIDTH) - 1))
                {
                    return;
                }
            }

            return;
        }
        
        void test_leaf_node_nearer(frustrum &f, const packet_ray *const r, vfp_t *const closer, const vfp_t *const t, packet_hit_description *const h, const unsigned *c, const unsigned int size) const
        {
            const triangle * i_o[MAXIMUM_PACKET_SIZE << LOG2_SIMD_WIDTH];
            
            int end = _e;
            for (int i = _b; i <= end; ++i)
            {
                if ((*bvh_node::o)[i]->get_light() || (*bvh_node::o)[i]->is_transparent())
                {
                    continue;
                }

                const bool outside = f.cull((*bvh_node::o)[i]->get_vertex_a(), (*bvh_node::o)[i]->get_vertex_b(), (*bvh_node::o)[i]->get_vertex_c());
                if (!outside)
                {
                    (*bvh_node::o)[i]->is_intersecting(f, r, h, i_o, c, size);
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
            int     _b;     /* Index of primitives                  */
            point_t _low;   /* Position of the bottom left corner   */
        };
        
        union
        {
            int     _e;     /* Index of last primitive              */
            point_t _high;  /* Position of the top right corner     */
        };

        static std::vector<triangle *> *    o;  /* Vector of primitives */
};
}; /* namespace raptor_raytracer */

#endif /* #ifndef __BVH_NODE_H__ */
