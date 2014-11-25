#ifndef __KD_TREE_H__
#define __KD_TREE_H__

/* Standard headers */

/* Boost headers */

/* Common headers */
#include "point_t.h"
#include "simd.h"

/* Ray tracer headers */
#include "common.h"


namespace raptor_physics
{
class kd_tree
{
    public :
        static void build(const primitive_list &everything)
        {
            kdt_node kdt_base;

            /* Build a KD-tree to speed up ray tracing */    
            /* The base of the tree will hold everything for now, but adding it will 
              just be a waste of time for now */
            build_kd_tree(&everything, &kdt_base, axis_t::x_axis);
        }

#ifdef SIMD_PACKET_TRACING
        /* SIMD KD-tree traversal */
        inline void find_kdt_leaf_node(const packet_ray *const r, kdt_stack_element **const out, kdt_stack_element *const entry_point, const vfp_t *const i_rd, const int *const near_offset) const;
        inline bool find_kdt_leaf_node(const frustrum &r, kdt_stack_element *const entry_point, kdt_stack_element **const out, const int *const near_offset, unsigned size) const;

        void        kdt_find_nearest_object(const packet_ray *const r, const triangle **const i_o, packet_hit_description *const h) const;
        vfp_t       kdt_found_nearer_object(const packet_ray *const r, const vfp_t &t) const;

        void        kdt_frustrum_found_nearer_object(const packet_ray *const r, const vfp_t *t, vfp_t *closer, unsigned size) const;
        void        kdt_frustrum_find_nearest_object(const packet_ray *const r, const triangle **const i_o, packet_hit_description *const h, int size) const;

        void        kdt_find_nearest_object(const packet_ray *const r, const triangle **const i_o, packet_hit_description *const h,
                                                kdt_stack_element entry_point, kdt_stack_element *exit_point) const;
        vfp_t       kdt_found_nearer_object(const packet_ray *const r, const vfp_t &t, kdt_stack_element entry_point, kdt_stack_element *exit_point) const;

#endif /* #ifdef SIMD_PACKET_TRACING */
        /* kdt traversal */
        inline void find_kdt_leaf_node(const ray *const r, const kdt_node **const n, kdt_stack_element **const out, const kdt_stack_element *const entry_point) const;
        triangle*   kdt_find_nearest_object(const ray *const r, hit_description *const h) const;
        bool        kdt_found_nearer_object(const ray *const r, const fp_t t) const;

    private :
        /* Stack element for tracing through the kd tree */
        struct kdt_stack_element
        {
#ifdef SIMD_PACKET_TRACING
            vfp_t               vt_max;
            vfp_t               vt_min;
            vfp_t               m;
            point_t             u;
            point_t             l;
            float               t_max;
            float               t_min;
#endif
            const kdt_node     *n;
            kdt_stack_element  *s;
            point_t             p;
            float               d;
        };
 
        /* The stack is mutable because it will never be known to a user of this class */
        mutable kdt_stack_element   kdt_stack[MAX_KDT_STACK_HEIGHT];
};
}; /* namespace */

#endif /* #ifndef __KD_TREE_H__ */
