#ifndef __BIH_BUILDER_H__
#define __BIH_BUILDER_H__

/* Standard headers */
#include <atomic>
#include <vector>

/* Ray tracer headers */
#include "bih_block.h"
#include "triangle.h"


namespace raptor_raytracer
{
class bih_builder
{
    public :
        bih_builder(const int max_node_size = MAX_BIH_NODE_SIZE) :
        _primitives(nullptr), _blocks(nullptr), _min_node_volume(calculate_min_node_volume()), _max_node_size(max_node_size), _depth(0), _next_block(1) {  }

        void build(primitive_list *const primitives, std::vector<bih_block> *const blocks);
    
    private :
        void divide_bih_block(point_t bl, point_t tr, const point_t &node_bl, const point_t &node_tr, const int block_idx, const int b, const int e);
        void divide_bih_node(block_splitting_data *const split_data, const int in_idx, const int out_idx, const int block_idx, const int node_idx);

        float calculate_min_node_volume() const
        {
            const point_t min_node((triangle::get_scene_upper_bounds() - triangle::get_scene_lower_bounds()) * 0.01f);
            return min_node.x * min_node.y * min_node.z;
        }

        struct block_splitting_data
        {
            point_t bl[8];
            point_t tr[8];
            point_t node_bl[8];
            point_t node_tr[8];
            int     end[8];
            int     begin[8];
        };

        primitive_list *            _primitives;
        std::vector<bih_block> *    _blocks;
        const float                 _min_node_volume;
        const int                   _max_node_size;
        int                         _depth;
        std::atomic<int>            _next_block;
};
}; /* namespace raptor_raytracer */

#endif /* __BIH_BUILDER_H__ */
