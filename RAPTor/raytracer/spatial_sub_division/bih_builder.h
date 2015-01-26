#ifndef __BIH_BUILDER_H__
#define __BIH_BUILDER_H__

/* Standard headers */
#include <atomic>
#include <vector>

/* Boost headers */

/* Ray tracer headers */
#include "bih_block.h"
#include "triangle.h"


namespace raptor_raytracer
{
class bih_builder
{
    public :
        bih_builder(const int max_node_size = MAX_BIH_NODE_SIZE) :
        _primitives(nullptr), _blocks(nullptr), _max_node_size(max_node_size), _depth(0), _next_block(1) {  }

        void build(primitive_list *const primitives, std::vector<bih_block> *const blocks);
    
    private :
        struct block_splitting_data
        {
            const point_t *         hist_bl[8];
            const point_t *         hist_tr[8];
            point_t                 bl[8];
            point_t                 tr[8];
            point_t                 node_bl[8];
            point_t                 node_tr[8];
            const unsigned int *    bins[8];
            int                     level[8];
            int                     end[8];
            int                     begin[8];
        };

        /* Radix sort */
        // cppcheck-suppress unusedPrivateFunction
        void bucket_build();
        void bucket_build_mid(point_t *const bl, point_t *const tr, unsigned int *const hist, const int b, const int e);
        void bucket_build_low(point_t *const bl, point_t *const tr, unsigned int *const hist, const int b, const int e);

        point_t convert_to_primitve_builder(point_t *const bl, triangle **const active_prims, const int b, const int e);
        point_t convert_to_primitve_builder(point_t *const bl, triangle **const active_prims, const int b, const int e, const int begin_mc, const int end_mc, const int level);
        void level_switch(block_splitting_data *const split_data, const int block_idx, const int node_idx, const int data_idx);

        void divide_bih_block(const point_t *const bl, const point_t *const tr, const unsigned int *const bins, const point_t &node_bl, const point_t &node_tr, const int block_idx, const int b, const int e, const int level = 2);
        void divide_bih_block(block_splitting_data *const split_data, const int block_idx, const int node_idx);
        bool divide_bih_node_binned(block_splitting_data *const split_data, const int in_idx, const int out_idx, const int block_idx, const int node_idx);

        void divide_bih_block(point_t bl, point_t tr, const point_t &node_bl, const point_t &node_tr, const int block_idx, const int b, const int e, const int node_idx = 0);
        void divide_bih_node(block_splitting_data *const split_data, const int in_idx, const int out_idx, const int block_idx, const int node_idx);

        primitive_list *                _primitives;
        std::unique_ptr<triangle * []>  _prim_buffer;
        std::unique_ptr<int []>         _morton_codes;
        std::unique_ptr<int []>         _code_buffer;
        std::vector<bih_block> *        _blocks;
        point_t                         _widths;
        const int                       _max_node_size;
        int                             _depth;
        std::atomic<int>                _next_block;
};
}; /* namespace raptor_raytracer */

#endif /* __BIH_BUILDER_H__ */
