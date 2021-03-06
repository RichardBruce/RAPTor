#pragma once

#include "common.h"
#include "triangle.h"
#include "ray.h"
#include "raytracer.h"


namespace raptor_raytracer
{
class kdt_node
{
    public :
        kdt_node(float s = 0.0f, axis_t n = axis_t::not_set) : c(nullptr), split_pos(s), normal(n) { };
        ~kdt_node() 
        {
            /* Clean up dynamic memory usage */
            if (this->normal == axis_t::not_set)
            {
                delete this->p;
            }
        }

        kdt_node(kdt_node &&k)
        : split_pos(k.split_pos), normal(k.normal)
        {
            if (this->normal == axis_t::not_set)
            {
                p = k.p;
                k.p = nullptr;
                k.normal = axis_t::x_axis;
            }
        }
        
        /* Access functions */
        std::vector<int>&   get_primitives()              { return *this->p;            }
        kdt_node *          get_left()              const { return &this->c[0];         }
        kdt_node *          get_right()             const { return &this->c[1];         }
        float               get_split_position()    const { return this->split_pos;     }
        axis_t              get_normal()            const { return this->normal;        }
        int                 get_size()              const { return this->p->size();     }
        bool                is_empty()              const { return this->p->empty();    }
        
        void            set_primitives(std::vector<int> *p)  { this->p = p;             }
        kdt_node &      split_node(kdt_node *const k, const float s, const axis_t n) 
        {
            this->normal    = n;
            this->split_pos = s;
            this->c         = k;
            return *this;
        } 
        
        /* test_leaf_node_nearest tests this object if is a leaf node and returns a pointer 
           to the nearest object found. The distance to this object is returned in m. If no 
           object is found nullptr is returned */
        int test_leaf_node_nearest(const primitive_store &e, const ray *const r, hit_description *const h, const float min) const
        {
            int intersecting_object = -1;
            for (int i : (*this->p))
            {
                hit_description hit_type;
                e.primitive(i)->is_intersecting(r, &hit_type);
                if ((hit_type.d < ((h->d) + (1.0f * EPSILON))) && (hit_type.d > (min - (1.0f * EPSILON))))
                {
                    *h = hit_type;
                    intersecting_object = i;
                }
            }
            
            return intersecting_object;
        }
        
        /* test_leaf_node_nearer tests this object if it is a leaf node to find an object
           that intersects with the ray r and is closer than max. If a closer intersecting 
           obeject is found true is return otherwise false is returned */
        bool test_leaf_node_nearer(const primitive_store &e, const ray *const r, const float max) const
        {
            for (int i : (*this->p))
            {
                const auto *const tri = e.primitive(i);
                if (tri->get_light() || tri->is_transparent())
                {
                    continue;
                }
                
                hit_description hit_type;
                tri->is_intersecting(r, &hit_type);
                if ((hit_type.d < max) && (hit_type.d > (1.0f * EPSILON)))
                {
                    return true;
                }
            }
            
            return false;
        }

#ifdef SIMD_PACKET_TRACING
        void test_leaf_node_nearest(const primitive_store &e, const packet_ray *const r, vint_t *const i_o, packet_hit_description *const h) const
        {
            for (int i : (*this->p))
            {
                e.primitive(i)->is_intersecting(r, h, i_o, 1, i);
            }
            
            return;
        }

        void test_leaf_node_nearest(const primitive_store &e, frustrum &f, const packet_ray *const r, vint_t *const i_o, packet_hit_description *const h, const unsigned *c, const unsigned int size) const
        {
            for (int i : (*this->p))
            {
                const auto *const tri = e.primitive(i);
                bool outside = f.cull(tri->get_vertex_a(), tri->get_vertex_b(), tri->get_vertex_c());
                if (!outside)
                {
                    tri->is_intersecting(f, r, h, i_o, c, size, i);
                }
            }

            return;
        }

        void test_leaf_node_nearer(const primitive_store &e, const packet_ray *const r, vfp_t *const c, const vfp_t &t, packet_hit_description *const h) const
        {
            vint_t i_o;
            for (int i : (*this->p))
            {
                const auto *const tri = e.primitive(i);
                if (tri->get_light() || tri->is_transparent())
                {
                    continue;
                }
        
                tri->is_intersecting(r, h, &i_o, 1, i);
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
            vint_t i_o[MAXIMUM_PACKET_SIZE];
            for (int i : (*this->p))
            {
                const auto *const tri = e.primitive(i);
                if (tri->get_light() || tri->is_transparent())
                {
                    continue;
                }

                bool outside = f.cull(tri->get_vertex_a(), tri->get_vertex_b(), tri->get_vertex_c());
                if (!outside)
                {
                    tri->is_intersecting(f, r, h, i_o, c, size, i);
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
        /* Prevent copying of this class */
        kdt_node(const kdt_node &);
        kdt_node& operator=(const kdt_node &);

        /* The node may point to a child node or primitives */
        union 
        {
            kdt_node            *c;
            std::vector<int>    *p;
        };

        float   split_pos;
        axis_t  normal;
};
}; /* namespace raptor_raytracer */
