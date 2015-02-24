/* Standard headers */
#include <iomanip>

/* Boost headers */

/* Common headers */
#include "logging.h"

/* Ray tracer headers */
#include "bih_builder.h"
#include <chrono>


namespace raptor_raytracer
{
const int histogram_size = 1024;

#define NODE_BUILD_STEP(in_idx, out_idx, node_idx) \
    if (split_data->level[(in_idx)] == -1) \
    { \
        divide_bih_node(split_data, (in_idx), (out_idx), block_idx, (node_idx)); \
    } \
    else if (divide_bih_node_binned(split_data, (in_idx), (out_idx), block_idx, (node_idx))) \
    { \
        level_switch(split_data, block_idx, (node_idx), (in_idx)); \
        return; \
    }

/* Build a Bounding Interval Heirarchy for _primitives into blocks */
void bih_builder::build(primitive_list *const primitives, std::vector<bih_block> *const blocks)
{
    // BOOST_LOG_TRIVIAL(trace) << "BIH construction has begun";

    /* Maximum theoretical size is everything.size() * 6 node, but this is very unlikely */
    _primitives = primitives;
    _blocks = blocks;
    _blocks->resize(std::max(1, static_cast<int>(_primitives->size() * 0.2f)));

    /* Check if we have anything to do */
    if (_primitives->size() <= static_cast<unsigned int>(_max_node_size))
    {
        /* Create one leaf node that hold the whole scene */
        (*_blocks)[0].create_leaf_node(0, _primitives->size() - 1, 0);
    }
    else
    {
        _next_block = 1;
        _bounds.reset(new bih_voxel_data [_primitives->size()]);

        /* For small data sets run the primitive builder */
        /* This needs tuning to each machine. On intel i7 laptop this should be ~750,000, but on amd llano the binned algorithm shouldnt be used */
        if (_primitives->size() < 28000000)
        {
            /* Cache primitive min and max */
            for (unsigned int i = 0; i < _primitives->size(); ++i)
            {
                _bounds[i].low  = (*_primitives)[i]->lowest_point();
                _bounds[i].high = (*_primitives)[i]->highest_point();
            }
    
            divide_bih_block(triangle::get_scene_lower_bounds(), triangle::get_scene_upper_bounds(), triangle::get_scene_lower_bounds(), triangle::get_scene_upper_bounds(), 0, 0, _primitives->size() - 1);
        }
        /* For large data sets run a bucket based builder */
        else
        {
            bucket_build();
        }
    }
    // BOOST_LOG_TRIVIAL(trace) << "BIH construction used: " << _next_block << " blocks";
}

void bih_builder::bucket_build()
{
    /* Calculate Morton codes and build MSB histogram */
    const point_t scene_width(triangle::get_scene_upper_bounds() - triangle::get_scene_lower_bounds());
    _width = std::max(std::max(scene_width.x, scene_width.y), scene_width.z) * (1.00001f / 1024.0f);
    _width_epsilon = _width * 1.09f;
    _width_inv = 1.0f / _width;

    unsigned int hist[histogram_size + 1];
    memset(hist, 0, histogram_size * sizeof(float));
    _code_buffer.reset(new int [_primitives->size()]);
    _prim_buffer.reset(new triangle *[_primitives->size()]);
    _morton_codes.reset(new int [_primitives->size()]);

    const vfp_t x_mul(_width_inv);
    const vfp_t y_mul(_width_inv);
    const vfp_t z_mul(_width_inv);
    const vfp_t scene_lo_x(triangle::get_scene_lower_bounds().x);
    const vfp_t scene_lo_y(triangle::get_scene_lower_bounds().y);
    const vfp_t scene_lo_z(triangle::get_scene_lower_bounds().z);
    point_t bl[histogram_size];
    point_t tr[histogram_size];
    std::fill_n(bl, histogram_size, point_t( MAX_DIST,  MAX_DIST,  MAX_DIST));
    std::fill_n(tr, histogram_size, point_t(-MAX_DIST, -MAX_DIST, -MAX_DIST));
    for (int i = 0; i <= (static_cast<int>(_primitives->size()) - SIMD_WIDTH); i += SIMD_WIDTH)
    {
        /* Calculate morton code*/
        const vfp_t lo_x((*_primitives)[i]->lowest_x(), (*_primitives)[i + 1]->lowest_x(), (*_primitives)[i + 2]->lowest_x(), (*_primitives)[i + 3]->lowest_x());
        const vfp_t lo_y((*_primitives)[i]->lowest_y(), (*_primitives)[i + 1]->lowest_y(), (*_primitives)[i + 2]->lowest_y(), (*_primitives)[i + 3]->lowest_y());
        const vfp_t lo_z((*_primitives)[i]->lowest_z(), (*_primitives)[i + 1]->lowest_z(), (*_primitives)[i + 2]->lowest_z(), (*_primitives)[i + 3]->lowest_z());

        const vfp_t hi_x((*_primitives)[i]->highest_x(), (*_primitives)[i + 1]->highest_x(), (*_primitives)[i + 2]->highest_x(), (*_primitives)[i + 3]->highest_x());
        const vfp_t hi_y((*_primitives)[i]->highest_y(), (*_primitives)[i + 1]->highest_y(), (*_primitives)[i + 2]->highest_y(), (*_primitives)[i + 3]->highest_y());
        const vfp_t hi_z((*_primitives)[i]->highest_z(), (*_primitives)[i + 1]->highest_z(), (*_primitives)[i + 2]->highest_z(), (*_primitives)[i + 3]->highest_z());

        const vfp_t x(((hi_x + lo_x) * 0.5f) - scene_lo_x);
        const vfp_t y(((hi_y + lo_y) * 0.5f) - scene_lo_y);
        const vfp_t z(((hi_z + lo_z) * 0.5f) - scene_lo_z);
        vint_t mc(morton_code(x, y, z, x_mul, y_mul, z_mul));
        mc.store(&_morton_codes[i]);

        /* Build histogram of morton codes */
        const unsigned int pos_0 = _morton_codes[i    ] >> 20;
        const unsigned int pos_1 = _morton_codes[i + 1] >> 20;
        const unsigned int pos_2 = _morton_codes[i + 2] >> 20;
        const unsigned int pos_3 = _morton_codes[i + 3] >> 20;
        ++hist[pos_0];
        ++hist[pos_1];
        ++hist[pos_2];
        ++hist[pos_3];

        /* Move and track primitive bounds */
        bl[pos_0] = min(bl[pos_0], (*_primitives)[i    ]->lowest_point());
        tr[pos_0] = max(tr[pos_0], (*_primitives)[i    ]->highest_point());
        bl[pos_1] = min(bl[pos_1], (*_primitives)[i + 1]->lowest_point());
        tr[pos_1] = max(tr[pos_1], (*_primitives)[i + 1]->highest_point());
        bl[pos_2] = min(bl[pos_2], (*_primitives)[i + 2]->lowest_point());
        tr[pos_2] = max(tr[pos_2], (*_primitives)[i + 2]->highest_point());
        bl[pos_3] = min(bl[pos_3], (*_primitives)[i + 3]->lowest_point());
        tr[pos_3] = max(tr[pos_3], (*_primitives)[i + 3]->highest_point());
    }

    for (int i = (static_cast<int>(_primitives->size()) & ~(SIMD_WIDTH - 1)); i < static_cast<int>(_primitives->size()); ++i)
    {
        /* Calculate morton code*/
        const point_t t((((*_primitives)[i]->highest_point() + (*_primitives)[i]->lowest_point()) * 0.5f) - triangle::get_scene_lower_bounds());
        _morton_codes[i] = morton_code(t.x, t.y, t.z, _width_inv, _width_inv, _width_inv);

        /* Build histogram of morton codes */
        const unsigned int pos = _morton_codes[i] >> 20;
        ++hist[pos];
        bl[pos] = min(bl[pos], (*_primitives)[i]->lowest_point());
        tr[pos] = max(tr[pos], (*_primitives)[i]->highest_point());
    }

    /* Sum the histogram */
    unsigned int sum = 0;
    for (int i = 0; i < histogram_size + 1; ++i)
    {
        const unsigned int tmp = hist[i] + sum;
        hist[i] = sum - 1;
        sum = tmp;
    }

    /* Move based on histogram */
    for (int i = 0; i < static_cast<int>(_primitives->size()); ++i)
    {
        /* Calculate destination and move morton code */
        const unsigned int data = _morton_codes[i];
        const unsigned int pos = data >> 20;
        _code_buffer[++hist[pos]] = data;

        /* Move primitive */
        _prim_buffer[hist[pos]] = (*_primitives)[i];
    }

    /* Un-increment the histogram */
    for (int i = histogram_size; i > 0; --i)
    {
        hist[i] = hist[i - 1] + 1;
    }
    hist[0] = 0;

    /* Kick of the block builder, breadth first within blocks, depth first outside blocks */
    divide_bih_block(bl, tr, hist, triangle::get_scene_lower_bounds(), triangle::get_scene_lower_bounds() + (_width * 1024.0f), triangle::get_scene_lower_bounds(), triangle::get_scene_upper_bounds(), 0, 0, histogram_size);
}

void bih_builder::bucket_build_mid(point_t *const bl, point_t *const tr, unsigned int *const hist, const int b, const int e)
{
    /* Build histogram */
    const int mc_check = _code_buffer[b] >> 23;
    for (int i = b; i < e; ++i)
    {
        ++hist[(_code_buffer[i] >> 10) & 0x3ff];
        assert((_code_buffer[i] >> 23) == mc_check);
    }
    
    /* Sum the histogram */
    unsigned int sum = b;
    for (int i = 0; i < histogram_size + 1; ++i)
    {
        const unsigned int tmp = hist[i] + sum;
        hist[i] = sum - 1;
        sum = tmp;
    }

    /* Move based on histogram */
    for (int i = b; i < e; ++i)
    {
        /* Calculate destination and move morton code */
        const unsigned int data = _code_buffer[i];
        const unsigned int pos = (data >> 10) & 0x3ff;
        _morton_codes[++hist[pos]] = data;

        /* Move primitive */
        (*_primitives)[hist[pos]] = _prim_buffer[i];

        /* Move and track primitive bounds */
        bl[pos] = min(bl[pos], _prim_buffer[i]->lowest_point());
        tr[pos] = max(tr[pos], _prim_buffer[i]->highest_point());
    }

    /* Un-increment the histogram */
    for (int i = histogram_size; i > 0; --i)
    {
        hist[i] = hist[i - 1] + 1;
    }
    hist[0] = b;
}

// __attribute__((optimize("unroll-loops")))
void bih_builder::bucket_build_low(point_t *const bl, point_t *const tr, unsigned int *const hist, const int b, const int e)
{
    /* Build histogram */
    const int mc_check = _morton_codes[b] >> 13;
    for (int i = b; i < e; ++i)
    {
        ++hist[_morton_codes[i] & 0x3ff];
        assert((_morton_codes[i] >> 13) == mc_check);
    }
    
    /* Sum the histogram */
    unsigned int sum = b;
    for (int i = 0; i < histogram_size + 1; ++i)
    {
        const unsigned int tmp = hist[i] + sum;
        hist[i] = sum - 1;
        sum = tmp;
    }

    /* Move based on histogram, no need to update the morton codes as they arent need again */
    for (int i = b; i < e; ++i)
    {
        /* Calculate destination and move morton code */
        const unsigned int data = _morton_codes[i];
        const unsigned int pos = data & 0x3ff;
        _code_buffer[++hist[pos]] = data;

        /* Move primitive */
        _prim_buffer[hist[pos]] = (*_primitives)[i];

        /* Move and track primitive bounds */
        bl[pos] = min(bl[pos], (*_primitives)[i]->lowest_point());
        tr[pos] = max(tr[pos], (*_primitives)[i]->highest_point());
    }

    /* Un-increment the histogram */
    for (int i = histogram_size; i > 0; --i)
    {
        hist[i] = hist[i - 1] + 1;
    }
    hist[0] = b;
}

void bih_builder::convert_to_primitve_builder(const int b, const int e)
{
    /* Get primitives in the right list for the non-binned node builder */
    for (int i = b; i < e; ++i)
    {
        (*_primitives)[i] = _prim_buffer[i];
        _bounds[i].low  = _prim_buffer[i]->lowest_point();
        _bounds[i].high = _prim_buffer[i]->highest_point();
    }
}

void bih_builder::convert_to_primitve_builder(triangle **const active_prims, const int b, const int e)
{
    /* Get primitives in the right list for the non-binned node builder */
    if (active_prims != _primitives->data())
    {
        for (int i = b; i < e; ++i)
        {
            (*_primitives)[i] = _prim_buffer[i];
        }
    }

    /* Cache bounds */
    for (int i = b; i < e; ++i)
    {
        _bounds[i].low  = (*_primitives)[i]->lowest_point();
        _bounds[i].high = (*_primitives)[i]->highest_point();
    }
}

void bih_builder::level_switch(block_splitting_data *const split_data, const int block_idx, const int node_idx, const int data_idx)
{
    const int e                     = split_data->end[data_idx];
    const int b                     = split_data->begin[data_idx];
    const int level                 = split_data->level[data_idx];
    const unsigned int *const bins  = split_data->bins[data_idx];

    /* If not many primitives left call the primitive builder */
    // if ((level != 0) && ((bins[e] - bins[b]) < 500))
    // {
    //     const int prim_begin = bins[b];
    //     const int prim_end = bins[e];
    //     triangle **active_prims = (level == 0x1) ? _primitives->data() : _prim_buffer.get();

    //     convert_to_primitve_builder(active_prims, prim_begin, prim_end);
    //     split_data->end[data_idx]   = prim_end - 1;
    //     split_data->begin[data_idx] = prim_begin;
    //     split_data->level[data_idx] = -1;

    //     divide_bih_block(split_data, block_idx, node_idx);
    //     return;
    // }
    
    switch (level)
    {
        /* Level 0 complete, call standard divider */
        case 0 :
            /* Set up state and call the partial block builder */
            convert_to_primitve_builder(bins[b], bins[e]);
            split_data->end[data_idx]   = bins[e] - 1;
            split_data->begin[data_idx] = bins[b];
            split_data->level[data_idx] = -1;
            divide_bih_block(split_data, block_idx, node_idx);
            break;
        /* Level 1 complete, call low bits radix sort*/
        case 1 :
        {
            unsigned int hist[histogram_size + 1];
            memset(hist, 0, histogram_size * sizeof(float));
            
            point_t bl[histogram_size];
            point_t tr[histogram_size];
            std::fill_n(bl, histogram_size, point_t( MAX_DIST,  MAX_DIST,  MAX_DIST));
            std::fill_n(tr, histogram_size, point_t(-MAX_DIST, -MAX_DIST, -MAX_DIST));

            bucket_build_low(bl, tr, hist, bins[b], bins[e]);

            /* Kick of the block builder, breadth first within blocks, depth first outside blocks */
            split_data->hist_bl[data_idx]   = bl;
            split_data->hist_tr[data_idx]   = tr;
            split_data->begin[data_idx]     = 0;
            split_data->end[data_idx]       = histogram_size;
            split_data->level[data_idx]     = 0;
            split_data->bins[data_idx]      = hist;
            divide_bih_block(split_data, block_idx, node_idx);
            break;
        }
        /* Level 2 complete, call mid bits radix sort */
        case 2 :
        {
            unsigned int hist[histogram_size + 1];
            memset(hist, 0, histogram_size * sizeof(float));
            
            point_t bl[histogram_size];
            point_t tr[histogram_size];
            std::fill_n(bl, histogram_size, point_t( MAX_DIST,  MAX_DIST,  MAX_DIST));
            std::fill_n(tr, histogram_size, point_t(-MAX_DIST, -MAX_DIST, -MAX_DIST));

            bucket_build_mid(bl, tr, hist, bins[b], bins[e]);

            /* Kick of the block builder, breadth first within blocks, depth first outside blocks */
            split_data->hist_bl[data_idx]   = bl;
            split_data->hist_tr[data_idx]   = tr;
            split_data->begin[data_idx]     = 0;
            split_data->end[data_idx]       = histogram_size;
            split_data->level[data_idx]     = 1;
            split_data->bins[data_idx]      = hist;
            divide_bih_block(split_data, block_idx, node_idx);
            break;
        }
        default :
            assert(false);
    }
}

/* Divide within a block, breadth first, with outside the block depth first */
void bih_builder::divide_bih_block(const point_t *const hist_bl, const point_t *const hist_tr, const unsigned int *const bins, const point_t &bl, const point_t &tr, const point_t &node_bl, const point_t &node_tr, const int block_idx, const int b, const int e, const int level, const int depth)
{
    block_splitting_data split_data;
    split_data.hist_bl[0]   = hist_bl;
    split_data.hist_tr[0]   = hist_tr;
    split_data.bl[0]        = bl;
    split_data.tr[0]        = tr;
    split_data.end[0]       = e;
    split_data.begin[0]     = b;
    split_data.depth[0]     = depth;
    split_data.bins[0]      = bins;
    split_data.level[0]     = level;
    split_data.node_tr[0]   = node_tr;
    split_data.node_bl[0]   = node_bl;
    
    /* Split block root */
    if (divide_bih_node_binned(&split_data, 0, 2, block_idx, 0))
    {
        level_switch(&split_data, block_idx, 0, 0);
        return;
    }

    if ((*_blocks)[block_idx].get_split_axis(0) == axis_t::not_set)
    {
        return;
    }

    /* Split level 1 */
    if (divide_bih_node_binned(&split_data, 2, 4, block_idx, 1))
    {
        level_switch(&split_data, block_idx, 1, 2);
        return;
    }
    
    if (divide_bih_node_binned(&split_data, 3, 6, block_idx, 2))
    {
        level_switch(&split_data, block_idx, 2, 3);
        return;
    }

    /* Split level 2 */
    if ((*_blocks)[block_idx].get_split_axis(1) != axis_t::not_set)
    {
        if (divide_bih_node_binned(&split_data, 4, 0, block_idx, 3))
        {
            level_switch(&split_data, block_idx, 3, 4);
            return;
        }

        if (divide_bih_node_binned(&split_data, 5, 2, block_idx, 4))
        {
            level_switch(&split_data, block_idx, 4, 5);
            return;
        }
    }

    if ((*_blocks)[block_idx].get_split_axis(2) != axis_t::not_set)
    {
        if (divide_bih_node_binned(&split_data, 6, 4, block_idx, 5))
        {
            level_switch(&split_data, block_idx, 5, 6);
            return;
        }

        if (divide_bih_node_binned(&split_data, 7, 6, block_idx, 6))
        {
            level_switch(&split_data, block_idx, 6, 7);
            return;
        }
    }

    /* Get extra blocks if needed */
    int child_idx = 0;
    const int blocks_required = (*_blocks)[block_idx].child_blocks_required();
    if (blocks_required > 0)
    {
        child_idx = _next_block.fetch_add(blocks_required, std::memory_order_relaxed);
        const int new_size = child_idx + blocks_required;
        if (new_size > static_cast<int>(_blocks->size()))
        {
            _blocks->resize(new_size);
        }
        
        (*_blocks)[block_idx].set_child_block(child_idx << 3);
    }

    /* Recurse if required */
    for (int i = 0; i < 4; ++i)
    {
        if ((*_blocks)[block_idx].get_split_axis(i + 3) != axis_t::not_set)
        {
            const int left_idx = i << 1;
            const int right_idx = left_idx + 1;
            
            /* Recurse left */
            divide_bih_block(split_data.hist_bl[left_idx], split_data.hist_tr[left_idx], split_data.bins[left_idx], split_data.bl[left_idx], split_data.tr[left_idx], split_data.node_bl[left_idx], split_data.node_tr[left_idx], child_idx, split_data.begin[left_idx], split_data.end[left_idx], split_data.level[left_idx], split_data.depth[left_idx]);
            ++child_idx;
            
            /* Recurse right */
            divide_bih_block(split_data.hist_bl[right_idx], split_data.hist_tr[right_idx], split_data.bins[right_idx], split_data.bl[right_idx], split_data.tr[right_idx], split_data.node_bl[right_idx], split_data.node_tr[right_idx], child_idx, split_data.begin[right_idx], split_data.end[right_idx], split_data.level[right_idx], split_data.depth[right_idx]);
            ++child_idx;
        }
    }

    return;
}

void bih_builder::divide_bih_block(block_splitting_data *const split_data, const int block_idx, const int node_idx)
{
    /* Split block root */
    if (node_idx <= 0)
    {
        NODE_BUILD_STEP(0, 2, 0);
        if ((*_blocks)[block_idx].get_split_axis(0) == axis_t::not_set)
        {
            return;
        }

    }

    /* Split level 1 */
    if (node_idx <= 1)
    {
        NODE_BUILD_STEP(2, 4, 1);
    }
    
    if (node_idx <= 2)
    {
        NODE_BUILD_STEP(3, 6, 2);
    }

    /* Split level 2 */
    if ((*_blocks)[block_idx].get_split_axis(1) != axis_t::not_set)
    {
        if (node_idx <= 3)
        {
            NODE_BUILD_STEP(4, 0, 3);
        }

        if (node_idx <= 4)
        {
            NODE_BUILD_STEP(5, 2, 4);
        }
    }

    if ((*_blocks)[block_idx].get_split_axis(2) != axis_t::not_set)
    {
        if (node_idx <= 5)
        {
            NODE_BUILD_STEP(6, 4, 5);
        }

        if (node_idx <= 6)
        {
            NODE_BUILD_STEP(7, 6, 6);
        }
    }

    /* Get extra blocks if needed */
    int child_idx = 0;
    const int blocks_required = (*_blocks)[block_idx].child_blocks_required();
    if (blocks_required > 0)
    {
        child_idx = _next_block.fetch_add(blocks_required, std::memory_order_relaxed);
        const int new_size = child_idx + blocks_required;
        if (new_size > static_cast<int>(_blocks->size()))
        {
            _blocks->resize(new_size);
        }
        
        (*_blocks)[block_idx].set_child_block(child_idx << 3);
    }

    /* Recurse if required */
    for (int i = 0; i < 4; ++i)
    {
        if ((*_blocks)[block_idx].get_split_axis(i + 3) != axis_t::not_set)
        {
            const int left_idx = i << 1;
            const int right_idx = left_idx + 1;
            if (split_data->level[left_idx] == -1)
            {
                /* Recurse left */
                divide_bih_block(split_data->bl[left_idx], split_data->tr[left_idx], split_data->node_bl[left_idx], split_data->node_tr[left_idx], child_idx, split_data->begin[left_idx], split_data->end[left_idx], split_data->depth[left_idx]);
                ++child_idx;
                
                /* Recurse right */
                divide_bih_block(split_data->bl[right_idx], split_data->tr[right_idx], split_data->node_bl[right_idx], split_data->node_tr[right_idx], child_idx, split_data->begin[right_idx], split_data->end[right_idx], split_data->depth[right_idx]);
                ++child_idx;
            }
            else
            {
                /* Recurse left */
                divide_bih_block(split_data->hist_bl[left_idx], split_data->hist_tr[left_idx], split_data->bins[left_idx], split_data->bl[left_idx], split_data->tr[left_idx], split_data->node_bl[left_idx], split_data->node_tr[left_idx], child_idx, split_data->begin[left_idx], split_data->end[left_idx], split_data->level[left_idx], split_data->depth[left_idx]);
                ++child_idx;
                
                /* Recurse right */
                divide_bih_block(split_data->hist_bl[right_idx], split_data->hist_tr[right_idx], split_data->bins[right_idx], split_data->bl[right_idx], split_data->tr[right_idx], split_data->node_bl[right_idx], split_data->node_tr[right_idx], child_idx, split_data->begin[right_idx], split_data->end[right_idx], split_data->level[right_idx], split_data->depth[right_idx]);
                ++child_idx;
            }
        }
    }

    return;
}

bool bih_builder::divide_bih_node_binned(block_splitting_data *const split_data, const int in_idx, const int out_idx, const int block_idx, const int node_idx)
{
    int e = split_data->end[in_idx];
    int b = split_data->begin[in_idx];
    point_t tr(split_data->tr[in_idx]);
    point_t bl(split_data->bl[in_idx]);
    const int level = split_data->level[in_idx];
    const int depth = split_data->depth[in_idx] + 1;
    const point_t &node_tr = split_data->node_tr[in_idx];
    const point_t &node_bl = split_data->node_bl[in_idx];
    const point_t *const hist_bl = split_data->hist_bl[in_idx];
    const point_t *const hist_tr = split_data->hist_tr[in_idx];
    const unsigned int *const bins = split_data->bins[in_idx];

    /* Checks */
    assert(depth <= MAX_BIH_STACK_HEIGHT);
    assert(static_cast<unsigned int>(e) <= histogram_size);

    /* Create leaf */
    if ((static_cast<int>(bins[e] - bins[b]) <= _max_node_size) || (depth == MAX_BIH_STACK_HEIGHT))
    {
        /* Possibly the primitives need moving back to the right list */
        if (!(level & 0x1))
        {
            for (unsigned int i = bins[b]; i < bins[e]; ++i)
            {
                (*_primitives)[i] = _prim_buffer[i];
            }
        }

        (*_blocks)[block_idx].create_leaf_node(bins[b], bins[e] - 1, node_idx);
        return false;
    }

    /* Pick longest side to divide in with a preference to z for floating point ties */
    point_t tm;
    point_t bm(bl);
    float split_pnt;
    float dx = tr.x - bl.x;
    float dy = tr.y - bl.y;
    float dz = tr.z - bl.z;
    axis_t split_axis = axis_t::not_set;
    while (true)
    {
        if (dx > (dy + (_width * 0.09f)))
        {
            if (dx > (dz + (_width * 0.09f)))
            {
                split_pnt  = bl.x + (dx * 0.5f);
                if ((split_pnt < node_bl.x) && (dx > _width_epsilon))
                {
                    bl.x = split_pnt;
                    dx *= 0.5f;
                }
                else if ((split_pnt > node_tr.x) && (dx > _width_epsilon))
                {
                    tr.x = split_pnt;
                    dx *= 0.5f;
                }
                else
                {
                    split_axis = axis_t::x_axis;
                    bm         = point_t(split_pnt, bl.y, bl.z);
                    tm         = point_t(split_pnt, tr.y, tr.z);
                    break;
                }
            }
            else
            {
                split_pnt  = bl.z + (dz * 0.5f);
                if (split_pnt < node_bl.z)
                {
                    bl.z = split_pnt;
                    dz *= 0.5f;
                }
                else if (split_pnt > node_tr.z)
                {
                    tr.z = split_pnt;
                    dz *= 0.5f;
                }
                else
                {
                    split_axis = axis_t::z_axis;
                    bm         = point_t(bl.x, bl.y, split_pnt);
                    tm         = point_t(tr.x, tr.y, split_pnt);
                    break;
                }
            }
        }
        else
        {
            if (dy > (dz + (_width * 0.09f)))
            {
                split_pnt  = bl.y + (dy * 0.5f);
                if (split_pnt < node_bl.y)
                {
                    bl.y = split_pnt;
                    dy *= 0.5f;
                }
                else if (split_pnt > node_tr.y)
                {
                    tr.y = split_pnt;
                    dy *= 0.5f;
                }
                else
                {
                    split_axis = axis_t::y_axis;
                    bm         = point_t(bl.x, split_pnt, bl.z);
                    tm         = point_t(tr.x, split_pnt, tr.z);
                    break;
                }
            }
            else
            {
                split_pnt  = bl.z + (dz * 0.5f);
                if (split_pnt < node_bl.z)
                {
                    bl.z = split_pnt;
                    dz *= 0.5f;
                }
                else if (split_pnt > node_tr.z)
                {
                    tr.z = split_pnt;
                    dz *= 0.5f;
                }
                else
                {
                    split_axis = axis_t::z_axis;
                    bm         = point_t(bl.x, bl.y, split_pnt);
                    tm         = point_t(tr.x, tr.y, split_pnt);
                    break;
                }
            }
        }
    }

    /* Calculate which bins are are looking at */
    const point_t code_pnt(bm + (_width * 0.09f) - triangle::get_scene_lower_bounds());
    const int split_mc = morton_code(code_pnt.x, code_pnt.y, code_pnt.z, _width_inv, _width_inv, _width_inv);
    const int split_idx = (split_mc >> (level * 10)) & 0x3ff;

    /* Check we have enough bins to work on */
    if ((split_mc & ((0x1 << (level * 10)) - 1)) || (dx < _width_epsilon))
    {
        split_data->tr[in_idx]      = tr;
        split_data->bl[in_idx]      = bl;
        split_data->begin[in_idx]   = split_idx;
        split_data->end[in_idx]     = split_idx + 1;
        assert((static_cast<int>(bins[split_idx + 1] - bins[split_idx]) > _max_node_size) || !"Error: Switching level lost primitives");
        return true;
    }

    /* Partition primitives */
    point_t node_tm(node_tr);
    point_t node_bm(node_bl);
    float max_left  = -MAX_DIST;
    float min_right =  MAX_DIST;
    switch (split_axis)
    {
        case axis_t::x_axis :
        {
            /* Track node bounds */
            int i = b;
            for (; i < split_idx; ++i)
            {
                max_left = std::max(max_left, hist_tr[i].x);
            }

            for (; i < e; ++i)
            {
                min_right = std::min(min_right, hist_bl[i].x);
            }

            assert(((node_bl.x <= split_pnt) && (node_bl.x >= bl.x     )) || ((max_left  <= split_pnt) && (max_left  >= bl.x     )) || ((bl.x      <= max_left ) && (bl.x      >= node_bl.x)) || ((split_pnt <= max_left ) && (split_pnt >= node_bl.x)) || (max_left  == -MAX_DIST) || !"Error: x bounds no longer overlap");
            assert(((min_right <= tr.x     ) && (min_right >= split_pnt)) || ((node_tr.x <= tr.x     ) && (node_tr.x >= split_pnt)) || ((split_pnt <= node_tr.x) && (split_pnt >= min_right)) || ((tr.x      <= node_tr.x) && (tr.x      >= min_right)) || (min_right ==  MAX_DIST) || !"Error: x bounds no longer overlap");

            /* Check this node is small than its parents */
            if ((min_right == MAX_DIST) && (max_left == node_tm.x))
            {
                split_data->bl[in_idx] = bl;
                split_data->tr[in_idx] = tm;
                split_data->begin[in_idx] = b;
                split_data->end[in_idx] = split_idx;
                split_data->depth[in_idx] = depth;
                return divide_bih_node_binned(split_data, in_idx, out_idx, block_idx, node_idx);
            }

            if ((max_left == -MAX_DIST) && (min_right == node_bm.x))
            {
                split_data->bl[in_idx] = bm;
                split_data->tr[in_idx] = tr;
                split_data->begin[in_idx] = split_idx;
                split_data->end[in_idx] = e;
                split_data->depth[in_idx] = depth;
                return divide_bih_node_binned(split_data, in_idx, out_idx, block_idx, node_idx);
            }

            node_tm.x = max_left;
            node_bm.x = min_right;
            break;
        }
        case axis_t::y_axis :
        {
            /* Track node bounds */
            int i = b;
            for (; i < split_idx; ++i)
            {
                max_left = std::max(max_left, hist_tr[i].y);
            }

            for (; i < e; ++i)
            {
                min_right = std::min(min_right, hist_bl[i].y);
            }

            assert(((node_bl.y <= split_pnt) && (node_bl.y >= bl.y     )) || ((max_left  <= split_pnt) && (max_left  >= bl.y     )) || ((bl.y      <= max_left ) && (bl.y      >= node_bl.y)) || ((split_pnt <= max_left ) && (split_pnt >= node_bl.y)) || (max_left  == -MAX_DIST) || !"Error: y bounds no longer overlap");
            assert(((min_right <= tr.y     ) && (min_right >= split_pnt)) || ((node_tr.y <= tr.y     ) && (node_tr.y >= split_pnt)) || ((split_pnt <= node_tr.y) && (split_pnt >= min_right)) || ((tr.y      <= node_tr.y) && (tr.y      >= min_right)) || (min_right ==  MAX_DIST) || !"Error: y bounds no longer overlap");

            /* Check this node is small than its parents */
            if ((min_right == MAX_DIST) && (max_left == node_tm.y))
            {

                split_data->bl[in_idx] = bl;
                split_data->tr[in_idx] = tm;
                split_data->begin[in_idx] = b;
                split_data->end[in_idx] = split_idx;
                split_data->depth[in_idx] = depth;
                return divide_bih_node_binned(split_data, in_idx, out_idx, block_idx, node_idx);
            }

            if ((max_left == -MAX_DIST) && (min_right == node_bm.y))
            {
                split_data->bl[in_idx] = bm;
                split_data->tr[in_idx] = tr;
                split_data->begin[in_idx] = split_idx;
                split_data->end[in_idx] = e;
                split_data->depth[in_idx] = depth;
                return divide_bih_node_binned(split_data, in_idx, out_idx, block_idx, node_idx);
            }

            node_tm.y = max_left;
            node_bm.y = min_right;
            break;
        }
        case axis_t::z_axis :
        {
            /* Track node bounds */
            int i = b;
            for (; i < split_idx; ++i)
            {
                max_left = std::max(max_left, hist_tr[i].z);
            }

            for (; i < e; ++i)
            {
                min_right = std::min(min_right, hist_bl[i].z);
            }

            assert(((node_bl.z <= split_pnt) && (node_bl.z >= bl.z     )) || ((max_left  <= split_pnt) && (max_left  >= bl.z     )) || ((bl.z      <= max_left ) && (bl.z      >= node_bl.z)) || ((split_pnt <= max_left ) && (split_pnt >= node_bl.z)) || (max_left  == -MAX_DIST) || !"Error: z bounds no longer overlap");
            assert(((min_right <= tr.z     ) && (min_right >= split_pnt)) || ((node_tr.z <= tr.z     ) && (node_tr.z >= split_pnt)) || ((split_pnt <= node_tr.z) && (split_pnt >= min_right)) || ((tr.z      <= node_tr.z) && (tr.z      >= min_right)) || (min_right ==  MAX_DIST) || !"Error: z bounds no longer overlap");

            /* Check this node is small than its parents */
            if ((min_right == MAX_DIST) && (max_left == node_tm.z))
            {
                split_data->bl[in_idx] = bl;
                split_data->tr[in_idx] = tm;
                split_data->begin[in_idx] = b;
                split_data->end[in_idx] = split_idx;
                split_data->depth[in_idx] = depth;
                return divide_bih_node_binned(split_data, in_idx, out_idx, block_idx, node_idx);
            }

            if ((max_left == -MAX_DIST) && (min_right == node_bm.z))
            {
                split_data->bl[in_idx] = bm;
                split_data->tr[in_idx] = tr;
                split_data->begin[in_idx] = split_idx;
                split_data->end[in_idx] = e;
                split_data->depth[in_idx] = depth;
                return divide_bih_node_binned(split_data, in_idx, out_idx, block_idx, node_idx);
            }

            node_tm.z = max_left;
            node_bm.z = min_right;
            break;
        }
        default :
            assert(false);
    }

    /* Construct the BIH nodes */
    (*_blocks)[block_idx].create_generic_node(max_left, min_right, node_idx, split_axis);

    /* Left node recursion data */
    split_data->hist_bl[out_idx]    = hist_bl;
    split_data->hist_tr[out_idx]    = hist_tr;
    split_data->tr[out_idx]         = tm;
    split_data->bl[out_idx]         = bl;
    split_data->begin[out_idx]      = b;
    split_data->end[out_idx]        = split_idx;
    split_data->depth[out_idx]      = depth;
    split_data->bins[out_idx]       = bins;
    split_data->level[out_idx]      = level;
    split_data->node_bl[out_idx]    = node_bl;
    split_data->node_tr[out_idx]    = node_tm;

    /* Right node recursion data */
    const int out_idx_p1 = out_idx + 1;
    split_data->hist_bl[out_idx_p1] = hist_bl;
    split_data->hist_tr[out_idx_p1] = hist_tr;
    split_data->tr[out_idx_p1]      = tr;
    split_data->bl[out_idx_p1]      = bm;
    split_data->begin[out_idx_p1]   = split_idx;
    split_data->end[out_idx_p1]     = e;
    split_data->depth[out_idx_p1]   = depth;
    split_data->bins[out_idx_p1]    = bins;
    split_data->level[out_idx_p1]   = level;
    split_data->node_bl[out_idx_p1] = node_bm;
    split_data->node_tr[out_idx_p1] = node_tr;
    return false;
}

/* Divide within a block, breadth first, with outside the block depth first */
void bih_builder::divide_bih_block(point_t bl, point_t tr, const point_t &node_bl, const point_t &node_tr, const int block_idx, const int b, const int e, const int depth, const int node_idx)
{
    block_splitting_data split_data;
    split_data.end[0]       = e;
    split_data.begin[0]     = b;
    split_data.depth[0]     = depth;
    split_data.tr[0]        = tr;
    split_data.bl[0]        = bl;
    split_data.node_tr[0]   = node_tr;
    split_data.node_bl[0]   = node_bl;
    
    /* Split block root */
    divide_bih_node(&split_data, 0, 2, block_idx, 0);
    if ((*_blocks)[block_idx].get_split_axis(0) == axis_t::not_set)
    {
        return;
    }

    /* Split level 1 */
    divide_bih_node(&split_data, 2, 4, block_idx, 1);
    divide_bih_node(&split_data, 3, 6, block_idx, 2);

    /* Split level 2 */
    if ((*_blocks)[block_idx].get_split_axis(1) != axis_t::not_set)
    {
        divide_bih_node(&split_data, 4, 0, block_idx, 3);
        divide_bih_node(&split_data, 5, 2, block_idx, 4);
    }

    if ((*_blocks)[block_idx].get_split_axis(2) != axis_t::not_set)
    {
        divide_bih_node(&split_data, 6, 4, block_idx, 5);
        divide_bih_node(&split_data, 7, 6, block_idx, 6);
    }

    /* Get extra blocks if needed */
    int child_idx = 0;
    const int blocks_required = (*_blocks)[block_idx].child_blocks_required();
    if (blocks_required > 0)
    {
        child_idx = _next_block.fetch_add(blocks_required, std::memory_order_relaxed);
        const int new_size = child_idx + blocks_required;
        if (new_size > static_cast<int>(_blocks->size()))
        {
            _blocks->resize(new_size);
        }
        
        (*_blocks)[block_idx].set_child_block(child_idx << 3);
    }

    /* Recurse if required */
    for (int i = 0; i < 4; ++i)
    {
        if ((*_blocks)[block_idx].get_split_axis(i + 3) != axis_t::not_set)
        {
            /* Recurse left */
            const int left_idx = i << 1;
            divide_bih_block(split_data.bl[left_idx], split_data.tr[left_idx], split_data.node_bl[left_idx], split_data.node_tr[left_idx], child_idx, split_data.begin[left_idx], split_data.end[left_idx], split_data.depth[left_idx]);
            ++child_idx;
            
            /* Recurse right */
            const int right_idx = left_idx + 1;
            divide_bih_block(split_data.bl[right_idx], split_data.tr[right_idx], split_data.node_bl[right_idx], split_data.node_tr[right_idx], child_idx, split_data.begin[right_idx], split_data.end[right_idx], split_data.depth[right_idx]);
            ++child_idx;
        }
    }

    return;
}

/* Find a split position and divide the bih node in 2 */
void bih_builder::divide_bih_node(block_splitting_data *const split_data, const int in_idx, const int out_idx, const int block_idx, const int node_idx)
{
    const int e = split_data->end[in_idx];
    const int b = split_data->begin[in_idx];
    const int depth = split_data->depth[in_idx] + 1;
    const point_t &tr = split_data->tr[in_idx];
    const point_t &bl = split_data->bl[in_idx];
    const point_t &node_tr = split_data->node_tr[in_idx];
    const point_t &node_bl = split_data->node_bl[in_idx];

    /* Checks */
    assert(depth <= MAX_BIH_STACK_HEIGHT);
    assert(static_cast<unsigned int>(e) < _primitives->size());
    assert((static_cast<unsigned int>(b) < _primitives->size()) || (b > e));

    /* Create leaf */
    if (((e - b) <= _max_node_size) || (depth == MAX_BIH_STACK_HEIGHT))
    {
        (*_blocks)[block_idx].create_leaf_node(b, e, node_idx);
        return;
    }

    /* Pick longest side to divide in */
    point_t bm;
    point_t tm;
    float split_pnt;
    axis_t split_axis   = axis_t::not_set;
    const float dx      = tr.x - bl.x;
    const float dy      = tr.y - bl.y;
    const float dz      = tr.z - bl.z;
    if (dx > dy)
    {
        if (dx > dz)
        {
            split_axis = axis_t::x_axis;
            split_pnt  = bl.x + (dx * 0.5f);
            bm         = point_t(split_pnt, bl.y, bl.z);
            tm         = point_t(split_pnt, tr.y, tr.z);
        }
        else
        {
            split_axis = axis_t::z_axis;
            split_pnt  = bl.z + (dz * 0.5f);
            bm         = point_t(bl.x, bl.y, split_pnt);
            tm         = point_t(tr.x, tr.y, split_pnt);
        }
    }
    else
    {
        if (dy > dz)
        {
            split_axis = axis_t::y_axis;
            split_pnt  = bl.y + (dy * 0.5f);
            bm         = point_t(bl.x, split_pnt, bl.z);
            tm         = point_t(tr.x, split_pnt, tr.z);
        }
        else
        {
            split_axis = axis_t::z_axis;
            split_pnt  = bl.z + (dz * 0.5f);
            bm         = point_t(bl.x, bl.y, split_pnt);
            tm         = point_t(tr.x, tr.y, split_pnt);
        }
    }

    /* Partition primitives */
    int left_size;
    int right_size;
    point_t node_tm(node_tr);
    point_t node_bm(node_bl);
    float max_left  = -MAX_DIST;
    float min_right =  MAX_DIST;
    int top         = e;
    int bottom      = b;
    switch (split_axis)
    {
        case axis_t::x_axis :
        {
            /* Sort primitives about split and track bounds */
            const float dbl_split = 2.0f * split_pnt;
            while (bottom < top)
            {
                while (dbl_split < (_bounds[top].high.x + _bounds[top].low.x) && bottom < top)
                {
                    min_right = std::min(min_right, _bounds[top--].low.x);
                }
                
                while (dbl_split >= (_bounds[bottom].high.x + _bounds[bottom].low.x) && bottom < top)
                {
                    max_left = std::max(max_left, _bounds[bottom++].high.x);
                }

                if (bottom < top)
                {
                    std::swap((*_primitives)[bottom], (*_primitives)[top]);
                    std::swap(_bounds[bottom], _bounds[top]);
                    
                    max_left = std::max(max_left, _bounds[bottom++].high.x);
                    min_right = std::min(min_right, _bounds[top--].low.x);
                }
            }

            /* Parition the last primitive */
            if (bottom == top)
            {
                if (dbl_split >= (_bounds[bottom].high.x + _bounds[bottom].low.x))
                {
                    max_left = std::max(max_left, _bounds[bottom++].high.x);
                }
                else
                {
                    min_right = std::min(min_right, _bounds[top].low.x);
                }
            }

            assert(((node_bl.x <= split_pnt) && (node_bl.x >= bl.x     )) || ((max_left  <= split_pnt) && (max_left  >= bl.x     )) || ((bl.x      <= max_left ) && (bl.x      >= node_bl.x)) || ((split_pnt <= max_left ) && (split_pnt >= node_bl.x)) || (max_left  == -MAX_DIST) || !"Error: x bounds no longer overlap");
            assert(((min_right <= tr.x     ) && (min_right >= split_pnt)) || ((node_tr.x <= tr.x     ) && (node_tr.x >= split_pnt)) || ((split_pnt <= node_tr.x) && (split_pnt >= min_right)) || ((tr.x      <= node_tr.x) && (tr.x      >= min_right)) || (min_right ==  MAX_DIST) || !"Error: x bounds no longer overlap");

            /* Adjust grid cell bounds */
            float pos = (tm.x - bl.x) * 0.5f;
            while (((bl.x + pos) > max_left) && (max_left > -MAX_DIST))
            {
                tm.x = bl.x + pos;
                pos *= 0.5f;
            }

            pos = (tr.x - bm.x) * 0.5f;
            while (((tr.x - pos) < min_right) && (min_right < MAX_DIST))
            {
                bm.x = tr.x - pos;
                pos *= 0.5f;
            }

            /* Check this node is small than its parents */
            right_size = e - bottom; /* 1 less than the actual node size */
            if ((right_size == -1) && (max_left == node_tm.x))
            {
                split_data->tr[in_idx] = tm;
                split_data->depth[in_idx] = depth;
                divide_bih_node(split_data, in_idx, out_idx, block_idx, node_idx);
                return;
            }

            left_size = bottom - b;
            if ((left_size == 0) && (min_right == node_bm.x))
            {
                split_data->bl[in_idx] = bm;
                split_data->depth[in_idx] = depth;
                divide_bih_node(split_data, in_idx, out_idx, block_idx, node_idx);
                return;
            }

            node_tm.x = max_left;
            node_bm.x = min_right;
            break;
        }
        case axis_t::y_axis :
        {
            /* Sort primitives about split and track bounds */
            const float dbl_split = 2.0f * split_pnt;
            while (bottom < top)
            {
                while (dbl_split < (_bounds[top].high.y + _bounds[top].low.y) && bottom < top)
                {
                    min_right = std::min(min_right, _bounds[top--].low.y);
                }
                
                while (dbl_split >= (_bounds[bottom].high.y + _bounds[bottom].low.y) && bottom < top)
                {
                    max_left = std::max(max_left, _bounds[bottom++].high.y);
                }

                if (bottom < top)
                {
                    std::swap((*_primitives)[bottom], (*_primitives)[top]);
                    std::swap(_bounds[bottom], _bounds[top]);
                    
                    max_left = std::max(max_left, _bounds[bottom++].high.y);
                    min_right = std::min(min_right, _bounds[top--].low.y);
                }
            }

            /* Parition the last primitive */
            if (bottom == top)
            {
                if (dbl_split >= (_bounds[bottom].high.y + _bounds[bottom].low.y))
                {
                    max_left = std::max(max_left, _bounds[bottom++].high.y);
                }
                else
                {
                    min_right = std::min(min_right, _bounds[top].low.y);
                }
            }

            assert(((node_bl.y <= split_pnt) && (node_bl.y >= bl.y     )) || ((max_left  <= split_pnt) && (max_left  >= bl.y     )) || ((bl.y      <= max_left ) && (bl.y      >= node_bl.y)) || ((split_pnt <= max_left ) && (split_pnt >= node_bl.y)) || (max_left  == -MAX_DIST) || !"Error: y bounds no longer overlap");
            assert(((min_right <= tr.y     ) && (min_right >= split_pnt)) || ((node_tr.y <= tr.y     ) && (node_tr.y >= split_pnt)) || ((split_pnt <= node_tr.y) && (split_pnt >= min_right)) || ((tr.y      <= node_tr.y) && (tr.y      >= min_right)) || (min_right ==  MAX_DIST) || !"Error: y bounds no longer overlap");

            /* Adjust grid cell bounds */
            float pos = (tm.y - bl.y) * 0.5f;
            while (((bl.y + pos) > max_left) && (max_left > -MAX_DIST))
            {
                tm.y = bl.y + pos;
                pos *= 0.5f;
            }

            pos = (tr.y - bm.y) * 0.5f;
            while (((tr.y - pos) < min_right) && (min_right < MAX_DIST))
            {
                bm.y = tr.y - pos;
                pos *= 0.5f;
            }

            /* Check this node is small than its parents */
            right_size = e - bottom; /* 1 less than the actual node size */
            if ((right_size == -1) && (max_left == node_tm.y))
            {
                split_data->tr[in_idx] = tm;
                split_data->depth[in_idx] = depth;
                divide_bih_node(split_data, in_idx, out_idx, block_idx, node_idx);
                return;
            }

            left_size = bottom - b;
            if ((left_size == 0) && (min_right == node_bm.y))
            {
                split_data->bl[in_idx] = bm;
                split_data->depth[in_idx] = depth;
                divide_bih_node(split_data, in_idx, out_idx, block_idx, node_idx);
                return;
            }

            node_tm.y = max_left;
            node_bm.y = min_right;
            break;
        }
        case axis_t::z_axis :
        {
            /* Sort primitives about split and track bounds */
            const float dbl_split = 2.0f * split_pnt;
            while (bottom < top)
            {
                while (dbl_split < (_bounds[top].high.z + _bounds[top].low.z) && bottom < top)
                {
                    min_right = std::min(min_right, _bounds[top--].low.z);
                }
                
                while (dbl_split >= (_bounds[bottom].high.z + _bounds[bottom].low.z) && bottom < top)
                {
                    max_left = std::max(max_left, _bounds[bottom++].high.z);
                }

                if (bottom < top)
                {
                    std::swap((*_primitives)[bottom], (*_primitives)[top]);
                    std::swap(_bounds[bottom], _bounds[top]);
                    
                    max_left = std::max(max_left, _bounds[bottom++].high.z);
                    min_right = std::min(min_right, _bounds[top--].low.z);
                }
            }

            /* Parition the last primitive */
            if (bottom == top)
            {
                if (dbl_split >= (_bounds[bottom].high.z + _bounds[bottom].low.z))
                {
                    max_left = std::max(max_left, _bounds[bottom++].high.z);
                }
                else
                {
                    min_right = std::min(min_right, _bounds[top].low.z);
                }
            }

            assert(((node_bl.z <= split_pnt) && (node_bl.z >= bl.z     )) || ((max_left  <= split_pnt) && (max_left  >= bl.z     )) || ((bl.z      <= max_left ) && (bl.z      >= node_bl.z)) || ((split_pnt <= max_left ) && (split_pnt >= node_bl.z)) || (max_left  == -MAX_DIST) || !"Error: z bounds no longer overlap");
            assert(((min_right <= tr.z     ) && (min_right >= split_pnt)) || ((node_tr.z <= tr.z     ) && (node_tr.z >= split_pnt)) || ((split_pnt <= node_tr.z) && (split_pnt >= min_right)) || ((tr.z      <= node_tr.z) && (tr.z      >= min_right)) || (min_right ==  MAX_DIST) || !"Error: z bounds no longer overlap");

            /* Adjust grid cell bounds */
            float pos = (tm.z - bl.z) * 0.5f;
            while (((bl.z + pos) > max_left) && (max_left > -MAX_DIST))
            {
                tm.z = bl.z + pos;
                pos *= 0.5f;
            }

            pos = (tr.z - bm.z) * 0.5f;
            while (((tr.z - pos) < min_right) && (min_right < MAX_DIST))
            {
                bm.z = tr.z - pos;
                pos *= 0.5f;
            }

            /* Check this node is small than its parents */
            right_size = e - bottom; /* 1 less than the actual node size */
            if ((right_size == -1) && (max_left == node_tm.z))
            {
                split_data->tr[in_idx] = tm;
                split_data->depth[in_idx] = depth;
                divide_bih_node(split_data, in_idx, out_idx, block_idx, node_idx);
                return;
            }

            left_size = bottom - b;
            if ((left_size == 0) && (min_right == node_bm.z))
            {
                split_data->bl[in_idx] = bm;
                split_data->depth[in_idx] = depth;
                divide_bih_node(split_data, in_idx, out_idx, block_idx, node_idx);
                return;
            }

            node_tm.z = max_left;
            node_bm.z = min_right;
            break;
        }
        default :
            assert(false);
    }

    /* Construct the BIH nodes */
    (*_blocks)[block_idx].create_generic_node(max_left, min_right, node_idx, split_axis);

    /* Left node recursion data */
    split_data->level[out_idx]      = -1;
    split_data->begin[out_idx]      = b;
    split_data->end[out_idx]        = std::max(0, bottom - 1);
    split_data->depth[out_idx]      = depth;
    split_data->tr[out_idx]         = tm;
    split_data->bl[out_idx]         = bl;
    split_data->node_tr[out_idx]    = node_tm;
    split_data->node_bl[out_idx]    = node_bl;

    /* Right node recursion data */
    const int out_idx_p1 = out_idx + 1;
    split_data->level[out_idx_p1]   = -1;
    split_data->begin[out_idx_p1]   = bottom;
    split_data->end[out_idx_p1]     = e;
    split_data->depth[out_idx_p1]   = depth;
    split_data->tr[out_idx_p1]      = tr;
    split_data->bl[out_idx_p1]      = bm;
    split_data->node_tr[out_idx_p1] = node_tr;
    split_data->node_bl[out_idx_p1] = node_bm;
}
}; /* namespace raptor_raytracer */
