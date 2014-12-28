#ifndef __BIH_BLOCK_H__
#define __BIH_BLOCK_H__

/* Standard headers */

/* Boost headers */

/* Common headers */

/* Ray tracer headers */
#include "bih_node.h"


namespace raptor_raytracer
{
/* Helper functions to convert an index to node or block index and back again */
inline int node_index(const int idx) { return idx & 0x7; }
inline int block_index(const int idx) { return idx >> 3; }
inline int bih_index(const int block_idx, const int node_idx) { return (block_idx << 3) | node_idx; }

/* Class to hold a block of 7 bih nodes on a cache line */
class bih_block
{
    public :
        /* Tree construction */
        void create_leaf_node(const int b, const int e, const int idx)
        {
            assert((idx < 7) || !"Error: Index of node is too large");
            static_assert((static_cast<int>(axis_t::not_set) & 0xfffffffc) == 0x0, "Expected axis_t::not_set to have a value of 0");

            /* Check the parent node is not a leaf */
            assert((idx != 1) || (static_cast<axis_t>(_axes        & 0x3) != axis_t::not_set) || !"Error: Parent of index 1 is a leaf");
            assert((idx != 2) || (static_cast<axis_t>(_axes        & 0x3) != axis_t::not_set) || !"Error: Parent of index 2 is a leaf");
            assert((idx != 3) || (static_cast<axis_t>((_axes >> 2) & 0x3) != axis_t::not_set) || !"Error: Parent of index 3 is a leaf");
            assert((idx != 4) || (static_cast<axis_t>((_axes >> 2) & 0x3) != axis_t::not_set) || !"Error: Parent of index 4 is a leaf");
            assert((idx != 5) || (static_cast<axis_t>((_axes >> 4) & 0x3) != axis_t::not_set) || !"Error: Parent of index 5 is a leaf");
            assert((idx != 6) || (static_cast<axis_t>((_axes >> 4) & 0x3) != axis_t::not_set) || !"Error: Parent of index 6 is a leaf");

            _axes &= ~((0x3 << (idx << 1)) | (1 << (idx + 16)));
            _nodes[idx].create_leaf_node(b, e);
        }

        void create_generic_node(const float l, const float r, const int idx, const axis_t a)
        {
            assert((idx < 7) || !"Error: Index of node is too large");
            assert((static_cast<int>(a) & 0xfffffffc) == 0);
            _axes |= (static_cast<int>(a) << (idx << 1)) | (0x1 << (idx + 16));

            _nodes[idx].create_generic_node(l, r);
        }

        void set_child_block(const unsigned int child)
        {
            assert((child & 0x7) == 0x0);
            _child = child;
        }

        int get_child_block() const
        {
            return _child;
        }
        
        /* Get size of next block layer */
        int child_blocks_required()
        {
            return __builtin_popcount(_axes >> 19) << 1;
        }

        /* Tree traversal */        
        axis_t get_split_axis(const int idx) const
        {
            assert((idx < 7) || !"Error: Index of node is too large");
            return static_cast<axis_t>((_axes >> (idx << 1)) & 0x3);
        }

        int get_left_child(const int block_idx, const int node_idx) const
        {
            assert((node_idx < 7) || !"Error: Index of node is too large");

            /* Child is part of another block */
            const int node_idx_m3 = node_idx - 3;
            if (node_idx_m3 >= 0)
            {
                /* Child indices should be handed out in multiples of 8 block of 7 nodes*/
                const int pre_children = __builtin_popcount((_axes >> 19) & ((1 << node_idx_m3) - 1));
                return _child + (pre_children << 4);
            }
            /* The child is in one of the upper levels */
            else
            {
                return (block_idx << 3) | ((node_idx << 1) | 0x1);
            }
        }

        int get_right_child(const int block_idx, const int node_idx) const
        {
            assert((node_idx < 7) || !"Error: Index of node is too large");

            /* Child is part of another block */
            const int node_idx_m3 = node_idx - 3;
            if (node_idx_m3 >= 0)
            {
                /* Child indices should be handed out in multiples of 8 block of 7 nodes*/
                const int pre_children = __builtin_popcount((_axes >> 19) & ((1 << node_idx_m3) - 1));
                return _child + ((pre_children << 4) | 0x8);
            }
            /* The child is in one of the upper levels */
            else
            {
                return (block_idx << 3) | ((node_idx + 1) << 1);
            }
        }

        const bih_node * get_node(const int idx) const
        {
            assert((idx < 7) || !"Error: Index of node is too large");
            return &_nodes[idx];
        }

    private :
        bih_node                        _nodes[7];  /* 3 levels of bih nodes                */
        unsigned int                    _child;     /* Index to the left child              */
        unsigned int                    _axes;      /* Axes of all nodes packed together    */
} __attribute__ ((aligned(64)));
}; /* namespace raptor_raytracer */

#endif /* #ifndef __BIH_BLOCK_H__ */
