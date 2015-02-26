#ifndef __KD_TREE_BUILDER__
#define __KD_TREE_BUILDER__

#include "ray.h"
#include "voxel.h"
#include "kdt_node.h"

class kdt_node;

namespace raptor_raytracer
{
#ifdef THREADED_RAY_TRACE
/**********************************************************
 
**********************************************************/
class kd_tree_build_task : public task
{
    public  :
        kd_tree_build_task(voxel *const b, kdt_node *const k, unsigned d=0) : b(b), k(k), d(d) 
        {
            /* Check the tree is not too deep */
            assert(this->d < MAX_KDT_STACK_HEIGHT);
        }

        task *execute()
        {
            voxel right_divide = this->b->divide(this->k);
    
            /* Check if a leaf node was create */
            if (this->k->get_normal() != not_set)
            {
                /* If no leaf node was created recurse */
                /* Allocate 2 new children */
                kd_tree_build_task& left  = *new(allocate_child()) kd_tree_build_task(this->b,       this->k->get_left(),  this->d + 1);
                kd_tree_build_task& right = *new(allocate_child()) kd_tree_build_task(&right_divide, this->k->get_right(), this->d + 1);
                
                /* Set the ref count to two children + the parent to wait for the children */
                set_ref_count(3);
                
                /* Spawn and wait */
                spawn(left);
                spawn_and_wait_for_all(right);
            }

            return nullptr;
        }
        
    private :
        voxel    *const b;
        kdt_node *const k;
        unsigned        d;
};
#endif /* #ifdef THREADED_RAY_TRACE */

class kdt_builder
{
    public :
        kdt_builder() :
            _depth(0), _primitives(nullptr), _nodes(nullptr) {  }

        /* Function to build a kd tree containg the object given in objects and with the first split
           in the axis_t given by normal */
        void build(const primitive_list *const objects, std::vector<kdt_node> *const nodes, axis_t normal);

    private :
        void divide_kdt_node(voxel *const base, int *const child_idx, const int node_idx);
  
        primitive_list *        _primitives;
        std::vector<kdt_node> * _nodes;
        int                     _depth;
};
}; /* namespace raptor_raytracer */

#endif /* #ifndef __KD_TREE_BUILDER__ */
