#ifndef __BIH_NODE_H__
#define __BIH_NODE_H__

#include "common.h"
#include "raytracer.h"
#include "triangle.h"
#include "ray.h"
#include "packet_ray.h"
#include "frustrum.h"


namespace raptor_raytracer
{
class bih_node
{
    public :
        /* Constructor */
        bih_node() : c(0) { };
        
        /* Tree construction */
        void create_leaf_node(const int b, const int e)
        {
            assert(static_cast<axis_t>(this->c & 0x3) == axis_t::not_set);
            this->b = b;
            this->e = e;
        }

        void create_generic_node(const unsigned c, const float l, const float r, const axis_t a)
        {
            assert((static_cast<int>(a) & 0xfffffffc) == 0);
            assert((c & 0xc0000000) == 0);
            this->l = l;
            this->r = r;
            this->c = c | (static_cast<int>(a) << 30);
        }
        
        /* Set statics */
        static void set_primitives(std::vector<triangle *> *const p)    { bih_node::o   = p;            }
        static const std::vector<bih_node> * resize_node_array(const int size)
        {
            bih_node::bih.resize(size);
            return &bih_node::bih;
        }

        /* Get elements of statics */
        static bih_node & get_node_array(const unsigned int i)
        {
            if (i >= bih_node::bih.size())
            {
                bih_node::bih.resize(i + 1);
            }

            return bih_node::bih[i];
        }


        /* Tree traversal */        
        const axis_t get_split_axis() const
        {
            return static_cast<axis_t>((this->c >> 30) & 0x3);
        }

        const float get_left_split() const
        {
            return this->l;
        }

        const float get_right_split() const
        {
            return this->r;
        }

        const bih_node * get_left_child() const
        {
            return &bih_node::bih[(this->c & 0x3fffffff)];
        }

        const bih_node * get_right_child() const
        {
            return &bih_node::bih[(this->c & 0x3fffffff) + 1];
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

        triangle * test_leaf_node_nearest(const ray *const r, hit_description *const h) const
        {
            triangle *intersecting_object = NULL;
    
            int end = this->e;
            for (int i = this->b; i <= end; i++)
            {
#ifdef SPATIAL_SUBDIVISION_STATISTICS
                ++nit;
#endif
                hit_description hit_type(h->d);
                (*bih_node::o)[i]->is_intersecting(r, &hit_type);
                if (hit_type.d < h->d)
                {
                    *h = hit_type;
                    intersecting_object = (*bih_node::o)[i];
                }
            }

#ifdef SPATIAL_SUBDIVISION_STATISTICS
            if (h->d < MAX_DIST)
            {
                ++ritm;
            }
#endif
    
            return intersecting_object;
        }

        bool test_leaf_node_nearer(const ray *const r, const float max) const
        {
            int end = this->e;
            for (int i = this->b; i <= end; i++)
            {
#ifdef SPATIAL_SUBDIVISION_STATISTICS
                ++nit;
#endif
                if ((*bih_node::o)[i]->get_light() || (*bih_node::o)[i]->is_transparent())
                {
                    continue;
                }
        
                hit_description hit_type(max);
                (*bih_node::o)[i]->is_intersecting(r, &hit_type);
                if (hit_type.d < max)
                {
#ifdef SPATIAL_SUBDIVISION_STATISTICS
                    ++ritm;
#endif
                    return true;
                }
            }
            return false;
        }

#ifdef SIMD_PACKET_TRACING
        void test_leaf_node_nearest(const packet_ray *const r, const triangle **const i_o, packet_hit_description *const h, const unsigned int size) const
        {
            int end = this->e;
            for (int i = this->b; i <= end; i++)
            {
                (*bih_node::o)[i]->is_intersecting(r, h, i_o, size);
            }
            
            return;
        }

        void test_leaf_node_nearest(frustrum &f, const packet_ray *const r, const triangle **const i_o, packet_hit_description *const h, const unsigned *c, const unsigned int size) const
        {
            int end = this->e;
            for (int i = this->b; i <= end; i++)
            {
                bool outside = f.cull((*bih_node::o)[i]->get_vertex_a(), (*bih_node::o)[i]->get_vertex_b(), (*bih_node::o)[i]->get_vertex_c());
                if (!outside)
                {
                    (*bih_node::o)[i]->is_intersecting(f, r, h, i_o, c, size);
                }
            }

            return;
        }

        void test_leaf_node_nearer(const packet_ray *const r, vfp_t *const c, const vfp_t &t, packet_hit_description *const h) const
        {
            const triangle * i_o[SIMD_WIDTH];

            int end = this->e;
            for (int i = this->b; i <= end; i++)
            {
                if ((*bih_node::o)[i]->get_light() || (*bih_node::o)[i]->is_transparent())
                {
                    continue;
                }
        
                (*bih_node::o)[i]->is_intersecting(r, h, &i_o[0], 1);
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
            
            int end = this->e;
            for (int i = this->b; i <= end; i++)
            {
                if ((*bih_node::o)[i]->get_light() || (*bih_node::o)[i]->is_transparent())
                {
                    continue;
                }

                bool outside = f.cull((*bih_node::o)[i]->get_vertex_a(), (*bih_node::o)[i]->get_vertex_b(), (*bih_node::o)[i]->get_vertex_c());
                if (!outside)
                {
                    (*bih_node::o)[i]->is_intersecting(f, r, h, i_o, c, size);
                    vfp_t done = vfp_true;
                    for (unsigned int j = 0; j < size; j++)
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

        unsigned                            c;      /* Index to the left child              */
        static std::vector<triangle *> *    o;      /* Vector of primitives                 */
        static std::vector<bih_node>        bih;    /* BIH nodes                            */
};
}; /* namespace raptor_raytracer */

#endif /* #ifndef __BIH_NODE_H__ */
