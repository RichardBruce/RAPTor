#include "kdt_builder.h"

#ifdef THREADED_RAY_TRACE
#include "task.h"
#include "task_scheduler_init.h"
#endif /* #ifdef THREADED_RAY_TRACE */


namespace raptor_raytracer
{
void kdt_builder::build(const primitive_list *const objects, std::vector<kdt_node> *const nodes, axis_t normal)
{
    /* Create a voxel to hold everything */
    const int nr_primitives = objects->size();

    /* Cache primitive bounds */
    std::vector<voxel_aab_data> ping(nr_primitives << 1);
    std::vector<voxel_aab_data> pong(nr_primitives << 1);
    for (int i = 0; i < nr_primitives; ++i)
    {
        ping[i].prim = (*objects)[i];
        ping[i].low  = (*objects)[i]->lowest_point();
        ping[i].high = (*objects)[i]->highest_point();
    }
    voxel base(&ping, &pong, 0, 0, nr_primitives, triangle::get_scene_upper_bounds(), triangle::get_scene_lower_bounds(), normal);

    /* Create an array to hold all nodes */
    _nodes = nodes;
    _nodes->resize(std::max(1, nr_primitives << 1));

#ifdef THREADED_RAY_TRACE
    /* Allocate and spawn the root node */
    kd_tree_build_task& root = *new(tbb::task::allocate_root()) kd_tree_build_task(&base, _nodes);
    tbb::task::spawn_root_and_wait(root);
#else
    /* Build the tree, top down */
    int child_idx = 1;
    divide_kdt_node(&base, &child_idx, 0);
    assert(_depth == 0);
#endif /* #ifdef THREADED_RAY_TRACE */

    return;
}

void kdt_builder::divide_kdt_node(voxel *const base, int *const child_idx, const int node_idx)
{
    /* Check the tree is not too deep */
    ++_depth;
    
    /* Divide the node */
    voxel right_divide(base->divide(&(*_nodes)[node_idx], &(*_nodes)[*child_idx], _depth));
    
    /* Check if a leaf node was create */
    if ((*_nodes)[node_idx].get_normal() != axis_t::not_set)
    {
        /* If no leaf node was created recurse */
        const int left_idx = *child_idx;
        const int right_idx = *child_idx + 1;
        (*child_idx) += 2;

        divide_kdt_node(base, child_idx, left_idx);
        divide_kdt_node(&right_divide, child_idx, right_idx);
    }
    
    /* Decrease the depth */
    --_depth;
    return;
}
}; /* namespace raptor_raytracer */
