#ifndef __BVH_BUILDER_H__
#define __BVH_BUILDER_H__

/* Standard headers */
#include <atomic>
#include <vector>

/* Boost headers */

/* Common headers */

/* Ray tracer headers */
#include "bvh_node.h"
#include "triangle.h"


namespace raptor_raytracer
{
class bvh_builder
{
    public :
        /* alpha = 0.4, delta = 20 should give a high quality build */
        /* alpha = 0.3, delta = 4 should give a faster, low quality, build */
        bvh_builder(const float alpha = 0.4f, const float delta = 20.0f, const float max_leaf_sah_factor = 0.00000000009f, const int max_down_phase_depth = 35) :
        _primitives(nullptr), _nodes(nullptr), _alpha(alpha), _epsilon(1e-8f), _delta(delta), _max_leaf_sah_factor(max_leaf_sah_factor), _max_leaf_sah(0.0f), _max_down_phase_depth(max_down_phase_depth), _depth(0), _next_node(0) {  }

        int build(primitive_list *const primitives, std::vector<bvh_node> *const nodes);
    
    private :
        int reduction_function(const int n) const;
        float cost_function(const bvh_node &l, const bvh_node &r) const;
        float cost_function(const triangle *const l, const point_t &low, const point_t &high, const float nodes) const;
        axis_t divide_spatial_bounds(point_t *const bm, point_t *const tm, const point_t &bl, const point_t &tr, const axis_t axis) const;

        int build_leaf_node(int *const cost_b, int *const cost_e, const int b, const int e, const int node_size);
        int build_layer(int *const cost_b, int *const cost_e, const point_t &bl, const point_t &tr, const int b, const int e, axis_t axis = axis_t::z_axis);
        int build_layer_primitive(int *const cost_b, int *const cost_e, const point_t &bl, const point_t &tr, const int b, const int e, axis_t axis);
        void combine_nodes(int *const cost_b, int *const cost_e, const int layer_size, const int cost_start_idx, const int cost_begin, const int cost_m, int cost_end);

        // void dump_cost(const int cost_start_idx, const int base_idx) const;

        primitive_list *        _primitives;
        std::unique_ptr<int []> _code_buffer;
        std::vector<float>      _cost_matrix;
        std::vector<int>        _cost_addrs;
        std::vector<bvh_node> * _nodes;
        const float             _alpha;
        const float             _epsilon;
        const float             _delta;
        const float             _max_leaf_sah_factor;
        float                   _max_leaf_sah;
        const int               _max_down_phase_depth;
        int                     _depth;
        std::atomic<int>        _next_node;
};
}; /* namespace raptor_raytracer */

#endif /* __BVH_BUILDER_H__ */
