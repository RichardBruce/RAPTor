#ifndef __BIH_H__
#define __BIH_H__

/* Standard headers */

/* Boost headers */

/* Common headers */
#include "point_t.h"
#include "simd.h"

/* Ray tracer headers */
#include "common.h"
#include "ssd.h"
#include "bih_node.h"
#include "bih_builder.h"


namespace raptor_raytracer
{
class bih : public ssd
{
    public :
        /* CTOR, build the tree */
        /* Maximum theoretical size is everything.size() * 6, but this is very unlikely */
        bih(const primitive_list &everything)
        : bih_base(everything.size() * 3)
        {            
            /* Grab an array to hold the BIH */
            const vector<triangle *> *bih_prim = build_bih(&everything, &bih_base);
        }

        /* Traversal functions */
#ifdef SIMD_PACKET_TRACING
        /* SIMD BIH traversal */

        void        frustrum_find_nearest_object(const packet_ray *const r, const triangle **const i_o, packet_hit_description *const h, int size) const override;
        void        frustrum_found_nearer_object(const packet_ray *const r, const vfp_t *t, vfp_t *closer, unsigned int size) const override;

        void        find_nearest_object(const packet_ray *const r, const triangle **const i_o, packet_hit_description *const h) const override;
        vfp_t       found_nearer_object(const packet_ray *const r, const vfp_t &t) const override;

#endif /* #ifdef SIMD_PACKET_TRACING */

        /* bih traversal */
        triangle*   find_nearest_object(const ray *const r, hit_description *const h) const override;
        bool        found_nearer_object(const ray *const r, const float t) const override;

    private :
        /* Stack element for tracing through the bih */
        struct bih_stack_element
        {
#ifdef SIMD_PACKET_TRACING
            vfp_t               vt_max;
            vfp_t               vt_min;
#endif
            point_t             u;
            point_t             l;
            const bih_node     *n;
            float               t_max;
            float               t_min;
        };

#ifdef SIMD_PACKET_TRACING
        inline int  find_leaf_node(const frustrum &r, bih_stack_element *const entry_point, bih_stack_element **const out, unsigned int size) const;
        inline bool find_leaf_node(const packet_ray &r, bih_stack_element *const entry_point, bih_stack_element **const out, const vfp_t *const i_rd) const;

        void        find_nearest_object(const packet_ray *const r, const triangle **const i_o, packet_hit_description *const h,
                                        bih_stack_element entry_point, bih_stack_element *exit_point) const;
        vfp_t       found_nearer_object(const packet_ray *const r, const vfp_t &t, bih_stack_element entry_point, bih_stack_element *exit_point) const;

#endif /* #ifdef SIMD_PACKET_TRACING */

        inline bool find_leaf_node(const ray &r, bih_stack_element *const entry_point, bih_stack_element **const out, const point_t &i_rd) const;

        /* The stack is mutable because it will never be known to a user of this class */
        mutable bih_stack_element   bih_stack[MAX_BIH_STACK_HEIGHT];
        std::vector<bih_node>       bih_base;
};
}; /* namespace raptor_raytracer */

#endif /* #ifndef __BIH_H__ */
