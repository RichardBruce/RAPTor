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
        void create_leaf_node(const point_t &high, const point_t &low, const int b, const int e)
        {
            _low    = low;
            _high   = high;
            _b      = b;
            _e      = e;
            _leaf   = true;
        }

        void create_generic_node(const std::vector<bvh_node> &nodes, const int left, const int right)
        {
            assert(left != right);
            _low    = min(nodes[left]._low, nodes[right]._low);
            _high   = max(nodes[left]._high, nodes[right]._high);
            _left   = left;
            _right  = right;
            _leaf   = false;
        }

        float combined_surface_area(const bvh_node &r) const
        {
            /* Size of combined node */
            const point_t bl(min(_low, r._low));
            const point_t tr(max(_high, r._high));

            /* Edges of new node */
            const point_t dist(tr - bl);

            return (dist.x * dist.y) + (dist.x * dist.z) + (dist.y * dist.z);
        }
        
        /* Set statics */
        static void set_primitives(std::vector<triangle *> *const p) { bvh_node::o = p; }

        /* Tree traversal */        
        float intersection_distance(const ray &r, const point_t &i_rd) const
        {
            /* Calculate distances to high and low axis aligned planes */
            point_t low_range(_low - r.get_ogn());
            point_t high_range(_high - r.get_ogn());
            point_t low_dist(low_range * i_rd);
            point_t high_dist(high_range * i_rd);

            /* Make sure high is the highest */
            if (low_dist.x > high_dist.x)
            {
                std::swap(low_dist.x, high_dist.x);
            }

            if (low_dist.y > high_dist.y)
            {
                std::swap(low_dist.y, high_dist.y);
            }

            if (low_dist.z > high_dist.z)
            {
                std::swap(low_dist.z, high_dist.z);
            }
            // BOOST_LOG_TRIVIAL(trace) << "Low: " << low_dist << ", high: " << high_dist;

            /* Return miss if exit before enter else entry distance */
            const float exit_t  = std::min(std::min(high_dist.x, high_dist.y), high_dist.z);
            const float enter_t = std::max(std::max(low_dist.x, low_dist.y), low_dist.z);
            // BOOST_LOG_TRIVIAL(trace) << "Enter: " << enter_t << ", exit: " << exit_t;
            if (std::max(0.0f, enter_t) > exit_t)
            {
                return MAX_DIST;
            }
            else
            {
                return enter_t;
            }
        }

#ifdef SIMD_PACKET_TRACING
        vfp_t intersection_distance(const packet_ray &r, const vfp_t *const i_rd) const
        {
            /* Calculate distances to high and low axis aligned planes */
            const vfp_t rang_x0(vfp_t(_low.x)  - r.get_ogn(0));
            const vfp_t rang_x1(vfp_t(_high.x) - r.get_ogn(0));
            const vfp_t dist_x0(rang_x0 * i_rd[0]);
            const vfp_t dist_x1(rang_x1 * i_rd[0]);

            const vfp_t low_x(min(dist_x0, dist_x1));
            const vfp_t high_x(max(dist_x0, dist_x1));

            const vfp_t rang_y0(vfp_t(_low.y)  - r.get_ogn(1));
            const vfp_t rang_y1(vfp_t(_high.y) - r.get_ogn(1));
            const vfp_t dist_y0(rang_y0 * i_rd[1]);
            const vfp_t dist_y1(rang_y1 * i_rd[1]);

            const vfp_t low_y(min(dist_y0, dist_y1));
            const vfp_t high_y(max(dist_y0, dist_y1));

            const vfp_t rang_z0(vfp_t(_low.z)  - r.get_ogn(2));
            const vfp_t rang_z1(vfp_t(_high.z) - r.get_ogn(2));
            const vfp_t dist_z0(rang_z0 * i_rd[2]);
            const vfp_t dist_z1(rang_z1 * i_rd[2]);

            const vfp_t low_z(min(dist_z0, dist_z1));
            const vfp_t high_z(max(dist_z0, dist_z1));

            /* Return miss if exit before enter else entry distance */
            const vfp_t enter_t(max(max(low_x, low_y), max(low_z, vfp_zero)));
            const vfp_t exit_t(min(min(high_x, high_y), high_z));
            return mov_p((enter_t > exit_t), vfp_t(MAX_DIST), enter_t);
        }
#endif /* #ifdef SIMD_PACKET_TRACING */

        const point_t& high_point() const { return _high;   }
        const point_t& low_point()  const { return _low;    }
        int left_index()            const { return _left;   }
        int right_index()           const { return _right;  }
        bool is_leaf()              const { return _leaf;   }

        /* Leaf node tests */
        bool is_empty() const
        {
            return (_e == _b);
        }

        int size() const
        {
            return _e - _b;
        }

        triangle * test_leaf_node_nearest(const ray *const r, hit_description *const h) const
        {
            triangle *intersecting_object = nullptr;
    
            const int end = _e;
            for (int i = _b; i < end; ++i)
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
            const int end = _e;
            for (int i = _b; i < end; ++i)
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
            const int end = _e;
            for (int i = _b; i < end; ++i)
            {
                (*bvh_node::o)[i]->is_intersecting(r, h, i_o, size);
            }

            // for (int i = 0; i < size; ++i)
            // {
            //     for (int j = 0; j < SIMD_WIDTH; ++j)
            //     {
            //         ray             ray_0 = r[i].extract(j);
            //         hit_description hit_0 = h[i][j];
            //         triangle *intersecting_object = test_leaf_node_nearest(&ray_0, &hit_0);
            //         if (hit_0.d < h[i].d[j])
            //         {
            //             i_o[(i * SIMD_WIDTH) + j] = intersecting_object;
            //             h[i].d[j] = hit_0.d;
            //             h[i].u[j] = hit_0.u;
            //             h[i].v[j] = hit_0.v;
            //         }
            //     }
            // }
            
            return;
        }

        void test_leaf_node_nearest(frustrum &f, const packet_ray *const r, const triangle **const i_o, packet_hit_description *const h, const unsigned *c, const unsigned int size) const
        {
            const int end = _e;
            for (int i = _b; i < end; ++i)
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

            const int end = _e;
            for (int i = _b; i < end; ++i)
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
            
            const int end = _e;
            for (int i = _b; i < end; ++i)
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
            int     _left;  /* Left child node                      */
            int     _b;     /* Index of primitives                  */
        };
        
        union
        {
            int     _right; /* Right child node                     */
            int     _e;     /* Index of last primitive              */
        };

        point_t _low;       /* Position of the bottom left corner   */
        point_t _high;      /* Position of the top right corner     */
        bool    _leaf;      /* Weather this is a leaf node          */

        static std::vector<triangle *> *    o;  /* Vector of primitives */
};
}; /* namespace raptor_raytracer */

#endif /* #ifndef __BVH_NODE_H__ */
