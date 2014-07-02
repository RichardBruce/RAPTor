#include "kd_tree_builder.h"

#ifdef THREADED_RAY_TRACE
#include "task.h"
#include "task_scheduler_init.h"

using namespace tbb;
#endif /* #ifdef THREADED_RAY_TRACE */


/* Tree depth */
static unsigned depth = 0;


/**********************************************************
 
**********************************************************/
void divide_kdt_node(voxel *const base, kdt_node *const kdt_subdiv)
{
    /* Check the tree is not too deep */
    depth++;
    assert(depth < MAX_KDT_STACK_HEIGHT);
    
    /* Divide the node */
    voxel right_divide = base->divide(kdt_subdiv);
    
    /* Check if a leaf node was create */
    if (kdt_subdiv->get_normal() != not_set)
    {
#ifdef SPATIAL_SUBDIVISION_STATISTICS
        ng++;
#endif
        /* If no leaf node was created recurse */
        divide_kdt_node(base,          kdt_subdiv->get_left());
        divide_kdt_node(&right_divide, kdt_subdiv->get_right());
    }
#ifdef SPATIAL_SUBDIVISION_STATISTICS
    else
    {
        /* Collect stats on the leaf node */
        ne++;
        max_depth    = max(max_depth, depth);
        nee         += kdt_subdiv->is_empty();
        ner          = max(ner, (unsigned)kdt_subdiv->get_size());       
        ave_ob      += kdt_subdiv->get_size();
    }
#endif
    
    /* Decrease the depth */
    depth--;
    return;
}


/**********************************************************
 
**********************************************************/
void build_kd_tree(const primitive_list *const objects, kdt_node *const kdt_subdiv, axis normal)
{
    /* Create a voxel to hold everything */
    primitive_list *object_copy = new primitive_list;
    *object_copy = *objects;
    voxel base(object_copy, triangle::get_scene_upper_bounds(), triangle::get_scene_lower_bounds(), normal);

#ifdef THREADED_RAY_TRACE
    /* Allocate and spawn the root node */
    kd_tree_build_task& root = *new(task::allocate_root()) kd_tree_build_task(&base, kdt_subdiv);
    task::spawn_root_and_wait(root);
#else
    /* Build the tree, top down */
    divide_kdt_node(&base, kdt_subdiv);
    assert(depth == 0);
#endif /* #ifdef THREADED_RAY_TRACE */

    return;
}
