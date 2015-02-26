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
        bvh_builder(const int max_node_size = MAX_BVH_NODE_SIZE) :
        _primitives(nullptr), _nodes(nullptr), _max_node_size(max_node_size), _next_node(1) {  }

        void build(primitive_list *const primitives, std::vector<bvh_node> *const nodes);
    
    private :

        primitive_list *                _primitives;
        std::unique_ptr<triangle * []>  _prim_buffer;
        std::unique_ptr<int []>         _morton_codes;
        std::unique_ptr<int []>         _code_buffer;
        std::vector<bvh_node> *         _nodes;
        const int                       _max_node_size;
        std::atomic<int>                _next_node;
};
}; /* namespace raptor_raytracer */

#endif /* __BVH_BUILDER_H__ */
