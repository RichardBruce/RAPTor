#pragma once

/* Standard headers */
#include <vector>

/* Boost headers */

/* Common headers */
#include "point_t.h"
#include "simd.h"

/* Ray tracer headers */
#include "common.h"
#include "ssd.h"
#include "bvh_node.h"
#include "bvh_node.h"
#include "bvh_builder.h"


namespace raptor_raytracer
{
class bvh : public ssd
{
    public :
        /* CTOR */
        // cppcheck-suppress uninitMemberVar
        bvh(primitive_store &everything) :
        _prims(everything), _builder(), _bvh_base(new std::vector<bvh_node>()), _root_node(0)
        {
            /* Build the heirarchy */
            _root_node = _builder.build(&everything, _bvh_base.get());
        }

        /* Copy CTOR */
        bvh(const bvh &b) : _prims(b._prims), _bvh_base(b._bvh_base), _root_node(0) {  }

        /* Assignment prohibited by base class */
        /* Allow default DTOR */

        /* Traversal functions */
#ifdef SIMD_PACKET_TRACING
        /* SIMD BIH traversal */
        void        frustrum_find_nearest_object(const packet_ray *const r, const triangle **const i_o, packet_hit_description *const h, int size) const override;
        void        frustrum_found_nearer_object(const packet_ray *const r, const vfp_t *t, vfp_t *closer, const unsigned int size) const override;

        void        find_nearest_object(const packet_ray *const r, const triangle **const i_o, packet_hit_description *const h) const override;
        vfp_t       found_nearer_object(const packet_ray *const r, const vfp_t &t) const override;
#endif /* #ifdef SIMD_PACKET_TRACING */

        /* bvh traversal */
        triangle*   find_nearest_object(const ray *const r, hit_description *const h) const override;
        bool        found_nearer_object(const ray *const r, const float t) const override;

    private :
        /* Stack element for tracing through the bvh */
        struct bvh_stack_element
        {
#ifdef SIMD_PACKET_TRACING
            bvh_stack_element() :
                vt_min_ptr(&vt_min[0]), t_max(0.0f), t_min(0.0f), idx(0)
            {  }

            vfp_t   vt_min[MAXIMUM_PACKET_SIZE];
            vfp_t * vt_min_ptr;
#endif
            float   t_max;
            float   t_min;
            int     idx;
        };

#ifdef SIMD_PACKET_TRACING
        inline int  find_leaf_node(const frustrum &f, const packet_ray *const r, bvh_stack_element *const entry_point, bvh_stack_element **const out, const packet_hit_description *const h, const int size) const;
        inline bool find_leaf_node(const packet_ray &r, bvh_stack_element *const entry_point, bvh_stack_element **const out, const vfp_t *const i_rd, const vfp_t &t_max) const;

        void        find_nearest_object(const packet_ray *const r, const triangle **const i_o, packet_hit_description *const h,
                                        bvh_stack_element entry_point, bvh_stack_element *exit_point) const;
        vfp_t       found_nearer_object(const packet_ray *const r, const vfp_t &t, bvh_stack_element entry_point, bvh_stack_element *exit_point) const;
#endif /* #ifdef SIMD_PACKET_TRACING */

        inline bool find_leaf_node(const ray &r, bvh_stack_element *const entry_point, bvh_stack_element **const out, const point_t &i_rd, const float t_max) const;

        /* The stack is mutable because it will never be known to a user of this class */
        const primitive_store &                 _prims;
        mutable bvh_stack_element               _bvh_stack[MAX_BVH_STACK_HEIGHT];
        bvh_builder                             _builder;
        std::shared_ptr<std::vector<bvh_node>>  _bvh_base;
        mutable bvh_stack_element               _entry_point;
        int                                     _root_node;
};
}; /* namespace raptor_raytracer */
