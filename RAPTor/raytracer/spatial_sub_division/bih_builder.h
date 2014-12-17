#ifndef __BIH_BUILDER_H__
#define __BIH_BUILDER_H__

/* Standard headers */
#include <atomic>
#include <vector>

/* Ray tracer headers */
#include "bih_block.h"


namespace raptor_raytracer
{
class bih_builder
{
    public :
        bih_builder(const int max_node_size = MAX_BIH_NODE_SIZE) :
        _primitives(nullptr), _blocks(nullptr), _max_node_size(max_node_size), _depth(0), _next_block(1) {  }

        void build(primitive_list *const primitives, std::vector<bih_block> *const blocks);
    
    private :
        void divide_bih_node(point_t bl, point_t tr, const int block_idx, const int node_idx, const int b, const int e);

        primitive_list *            _primitives;
        std::vector<bih_block> *    _blocks;
        const int                   _max_node_size;
        int                         _depth;
        std::atomic<int>            _next_block;
};
}; /* namespace raptor_raytracer */

#endif /* __BIH_BUILDER_H__ */
