#ifndef __KDT_NODE_H__
#define __KDT_NODE_H__

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
        primitive_list& get_primitives()                    { return *this->p;          }
        kdt_node *      get_left()                const     { return &this->c[0];       }
        kdt_node *      get_right()               const     { return &this->c[1];       }
        float           get_split_position()      const     { return this->split_pos;   }
        axis_t          get_normal()              const     { return this->normal;      }
        int             get_size()                const     { return this->p->size();   }
        bool            is_empty()                const     { return this->p->empty();  }
        
        void            set_primitives(primitive_list *p)   { this->p = p;              }
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
        triangle * test_leaf_node_nearest(const ray *const r, hit_description *const h, const float min) const;
        
        /* test_leaf_node_nearer tests this object if it is a leaf node to find an object
           that intersects with the ray r and is closer than max. If a closer intersecting 
           obeject is found true is return otherwise false is returned */
        bool    test_leaf_node_nearer(const ray *const r, const float max) const;

#ifdef SIMD_PACKET_TRACING
        void test_leaf_node_nearest(const packet_ray *const r, const triangle **const i_o, packet_hit_description *const h) const
        {
            for (primitive_list::const_iterator i=this->p->begin(); i!=this->p->end(); ++i)
            {
                (*i)->is_intersecting(r, h, i_o, 1);
            }
            
            return;
        }

        void test_leaf_node_nearest(frustrum &f, const packet_ray *const r, const triangle **const i_o, packet_hit_description *const h, const unsigned *c, const unsigned int size) const
        {
            for (primitive_list::const_iterator i=this->p->begin(); i!=this->p->end(); ++i)
            {
                bool outside = f.cull((*i)->get_vertex_a(), (*i)->get_vertex_b(), (*i)->get_vertex_c());
                if (!outside)
                {
                    (*i)->is_intersecting(f, r, h, i_o, c, size);
                }
            }

            return;
        }

        void test_leaf_node_nearer(const packet_ray *const r, vfp_t *const c, const vfp_t &t, packet_hit_description *const h) const
        {
            const triangle * i_o[SIMD_WIDTH];

            for (primitive_list::const_iterator i=this->p->begin(); i!=this->p->end(); ++i)
            {
                if ((*i)->get_light() || (*i)->is_transparent())
                {
                    continue;
                }
        
                (*i)->is_intersecting(r, h, &i_o[0], 1);
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
            
            for (primitive_list::const_iterator i=this->p->begin(); i!=this->p->end(); ++i)
            {
                if ((*i)->get_light() || (*i)->is_transparent())
                {
                    continue;
                }

                bool outside = f.cull((*i)->get_vertex_a(), (*i)->get_vertex_b(), (*i)->get_vertex_c());
                if (!outside)
                {
                    (*i)->is_intersecting(f, r, h, i_o, c, size);
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
            kdt_node        *c;
            primitive_list  *p;
        };

        float   split_pos;
        axis_t  normal;
};
}; /* namespace raptor_raytracer */

#endif /* #ifndef __KDT_NODE_H__ */
