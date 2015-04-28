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
        bvh_builder(const float alpha = 0.4f, const float delta = 20.0f, const int max_node_size = MAX_BVH_NODE_SIZE, const int max_down_phase_depth = 30) :
        _primitives(nullptr), _nodes(nullptr), _alpha(alpha), _epsilon(1e-8f), _delta(delta), _max_node_size(max_node_size), _max_down_phase_depth(max_down_phase_depth), _depth(0), _rows(0), _next_node(0) {  }

        int build(primitive_list *const primitives, std::vector<bvh_node> *const nodes);
    
    private :
        int reduction_function(const int n) const;
        int start_index(const int col) const;
        float cost_function(const bvh_node &l, const bvh_node &r) const;

        void build_leaf_node(int *const cost_b, int *const cost_e, const int b, const int e, const int node_size);
        void build_layer(int *const cost_b, int *const cost_e, const int b, const int e);
        void build_layer_primitive(int *const cost_b, int *const cost_e, const int b, const int e, axis_t axis = axis_t::x_axis);
        void combine_nodes(int *const cost_b, int *const cost_e, const int layer_size, const int cost_begin, const int cost_m, int cost_end, const int cost_start_idx);

        // void dump_cost() const;

        primitive_list *                _primitives;
        primitive_list                  _prim_buffer;
        std::unique_ptr<int []>         _morton_codes;
        std::unique_ptr<int []>         _code_buffer;
        std::vector<float>              _cost_matrix;
        std::vector<int>                _cost_addrs;
        std::vector<bvh_node> *         _nodes;
        const float                     _alpha;
        const float                     _epsilon;
        const float                     _delta;
        const int                       _max_node_size;
        const int                       _max_down_phase_depth;
        int                             _depth;
        int                             _rows;
        std::atomic<int>                _next_node;
};
}; /* namespace raptor_raytracer */

#endif /* __BVH_BUILDER_H__ */