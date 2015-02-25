#include "kd_tree_builder.h"

#ifdef THREADED_RAY_TRACE
#include "task.h"
#include "task_scheduler_init.h"
#endif /* #ifdef THREADED_RAY_TRACE */


namespace raptor_raytracer
{
/* Tree depth */
static unsigned depth = 0;


/**********************************************************
 
**********************************************************/
void divide_kdt_node(voxel *const base, kdt_node *const kdt_subdiv)
{
    /* Check the tree is not too deep */
    ++depth;
    assert(depth < MAX_KDT_STACK_HEIGHT);
    
    /* Divide the node */
    voxel right_divide = base->divide(kdt_subdiv, depth);
    
    /* Check if a leaf node was create */
    if (kdt_subdiv->get_normal() != axis_t::not_set)
    {
        /* If no leaf node was created recurse */
        divide_kdt_node(base,          kdt_subdiv->get_left());
        divide_kdt_node(&right_divide, kdt_subdiv->get_right());
    }
    
    /* Decrease the depth */
    --depth;
    return;
}


/**********************************************************
 
**********************************************************/
void build_kd_tree(const primitive_list *const objects, kdt_node *const kdt_subdiv, axis_t normal)
{
    /* Create a voxel to hold everything */
    primitive_list *object_copy = new primitive_list;
    *object_copy = *objects;
    voxel base(object_copy, triangle::get_scene_upper_bounds(), triangle::get_scene_lower_bounds(), normal);

#ifdef THREADED_RAY_TRACE
    /* Allocate and spawn the root node */
    kd_tree_build_task& root = *new(tbb::task::allocate_root()) kd_tree_build_task(&base, kdt_subdiv);
    tbb::task::spawn_root_and_wait(root);
#else
    /* Build the tree, top down */
    divide_kdt_node(&base, kdt_subdiv);
    assert(depth == 0);
#endif /* #ifdef THREADED_RAY_TRACE */

    return;
}
}; /* namespace raptor_raytracer */
