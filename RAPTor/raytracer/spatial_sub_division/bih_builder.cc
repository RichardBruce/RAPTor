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
    // // BOOST_LOG_TRIVIAL(trace) << "BIH construction has begun";
    // BOOST_LOG_TRIVIAL(trace) << "Scene bounds: " << triangle::get_scene_lower_bounds() << " - " << triangle::get_scene_upper_bounds();

    /* Maximum theoretical size is everything.size() * 6 node, but this is very unlikely */
    _primitives = primitives;
    _blocks = blocks;
    _blocks->resize(std::max(1, static_cast<int>(_primitives->size() * 0.4f)));

    /* Check if we have anything to do */
    if (_primitives->size() <= static_cast<unsigned int>(_max_node_size))
    {
        /* Create one leaf node that hold the whole scene */
        (*_blocks)[0].create_leaf_node(0, _primitives->size() - 1, 0);
    }
    else
    {
        /* Divide the nodes */
        _depth = 0;
        _next_block = 1;
        // divide_bih_block(triangle::get_scene_lower_bounds(), triangle::get_scene_upper_bounds(), triangle::get_scene_lower_bounds(), triangle::get_scene_upper_bounds(), 0, 0, _primitives->size() - 1);
        bucket_build();
        assert(_depth == 0);
    }
    // BOOST_LOG_TRIVIAL(trace) << "BIH construction used: " << _next_block << " blocks";
}

void bih_builder::bucket_build()
{
    /* Calculate Morton codes and build MSB histogram */
    const point_t scene_width(triangle::get_scene_upper_bounds() - triangle::get_scene_lower_bounds());
    const float widest = std::max(std::max(scene_width.x, scene_width.y), scene_width.z) / 1024.0f;
    _widths = point_t(widest, widest, widest);    /* TODO - Possibly divide down by average primitive width to reduce the number of occupied bins */

    unsigned int hist[histogram_size + 1];
    memset(hist, 0, histogram_size * sizeof(float));
    _code_buffer.resize(_primitives->size());
    _prim_buffer.resize(_primitives->size());
    _morton_codes.resize(_primitives->size());

    const point_t widths_inv(1.0f / _widths.x, 1.0f / _widths.y, 1.0f / _widths.z);
    const vfp_t x_mul(widths_inv.x);
    const vfp_t y_mul(widths_inv.y);
    const vfp_t z_mul(widths_inv.z);
    const vfp_t scene_lo_x(triangle::get_scene_lower_bounds().x);
    const vfp_t scene_lo_y(triangle::get_scene_lower_bounds().y);
    const vfp_t scene_lo_z(triangle::get_scene_lower_bounds().z);
    for (int i = 0; i <= (static_cast<int>(_primitives->size()) - SIMD_WIDTH); i += SIMD_WIDTH)
    {
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

        ++hist[_morton_codes[i    ] >> 20];
        ++hist[_morton_codes[i + 1] >> 20];
        ++hist[_morton_codes[i + 2] >> 20];
        ++hist[_morton_codes[i + 3] >> 20];
    }

    for (int i = (static_cast<int>(_primitives->size()) & ~(SIMD_WIDTH - 1)); i < static_cast<int>(_primitives->size()); ++i)
    {
        const point_t t((((*_primitives)[i]->highest_point() + (*_primitives)[i]->lowest_point()) * 0.5f) - triangle::get_scene_lower_bounds());
        _morton_codes[i] = morton_code(t.x, t.y, t.z, widths_inv.x, widths_inv.y, widths_inv.z);
        ++hist[_morton_codes[i] >> 20];
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
    point_t bl[histogram_size];
    point_t tr[histogram_size];
    std::fill_n(bl, histogram_size, point_t( MAX_DIST,  MAX_DIST,  MAX_DIST));
    std::fill_n(tr, histogram_size, point_t(-MAX_DIST, -MAX_DIST, -MAX_DIST));
    for (int i = 0; i < static_cast<int>(_primitives->size()); ++i)
    {
        const unsigned int data = _morton_codes[i];
        const unsigned int pos = data >> 20;
        _code_buffer[++hist[pos]] = data;

        auto prim = (*_primitives)[i];
        _prim_buffer[hist[pos]] = prim;
        bl[pos] = min(bl[pos], prim->lowest_point());
        tr[pos] = max(tr[pos], prim->highest_point());
    }

    /* Un-increment the histogram */
    for (int i = histogram_size; i > 0; --i)
    {
        hist[i] = hist[i - 1] + 1;
    }
    hist[0] = 0;

    /* Kick of the block builder, breadth first within blocks, depth first outside blocks */
    divide_bih_block(bl, tr, hist, triangle::get_scene_lower_bounds(), triangle::get_scene_upper_bounds(), 0, 0, histogram_size);
}

void bih_builder::bucket_build_mid(point_t *const bl, point_t *const tr, unsigned int *const hist, const int b, const int e)
{
    // BOOST_LOG_TRIVIAL(trace) << "Radix sort based on mid 11 bits";

    /* Build histogram */
    const int mc_check = _code_buffer[b] >> 22;
    // BOOST_LOG_TRIVIAL(trace) << "Mid sorting primitives: " << b << " - " << e;
    for (int i = b; i < e; ++i)
    {
        // BOOST_LOG_TRIVIAL(trace) << "Morton code: " << _code_buffer[i];
        ++hist[(_code_buffer[i] >> 10) & 0x3ff];
        assert((_code_buffer[i] >> 22) == mc_check);
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
        const unsigned int data = _code_buffer[i];
        const unsigned int pos = (data >> 10) & 0x3ff;
        _morton_codes[++hist[pos]] = data;

        auto prim = _prim_buffer[i];
        (*_primitives)[hist[pos]] = prim;
        bl[pos] = min(bl[pos], prim->lowest_point());
        tr[pos] = max(tr[pos], prim->highest_point());
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
    // BOOST_LOG_TRIVIAL(trace) << "Radix sort based on low 11 bits";
    
    /* Build histogram */
    const int mc_check = _morton_codes[b] >> 11;
    for (int i = b; i < e; ++i)
    {
        ++hist[_morton_codes[i] & 0x3ff];
        assert((_morton_codes[i] >> 11) == mc_check);
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
        const unsigned int data = _morton_codes[i];
        const unsigned int pos = data & 0x3ff;
        _code_buffer[++hist[pos]] = data;

        auto prim = (*_primitives)[i];
        _prim_buffer[hist[pos]] = prim;
        bl[pos] = min(bl[pos], prim->lowest_point());
        tr[pos] = max(tr[pos], prim->highest_point());
    }

    // /* Remember un-incremented bins */
    // unsigned int hist_bounds[histogram_size + 1];
    // memcpy(hist_bounds, hist, (histogram_size + 1) * sizeof(unsigned int));

    // /* For all bins */
    // for (unsigned int i = 0; i < histogram_size; ++i)
    // {
    //     /* Until we have update to top of this bin */
    //     unsigned int j = hist[i] + 1;
    //     unsigned int move_to = j;
    //     auto prim = (*_primitives)[j];
    //     while (j != (hist_bounds[i + 1] + 1))
    //     {
    //         const unsigned int data = _morton_codes[move_to];
    //         const unsigned int pos = data & 0x3ff;
    //         _code_buffer[++hist[pos]] = data;

    //         /* Get primitive bounds */
    //         bl[pos] = min(bl[pos], prim->lowest_point());
    //         tr[pos] = max(tr[pos], prim->highest_point());

    //         /* Primitive belongs in the empty, move to the next slot */
    //         if (pos == i)
    //         {
    //             (*_primitives)[j++] = prim;
    //             prim = (*_primitives)[j];
    //             move_to = j;
    //         }
    //         /* Swap primitive to its new position */
    //         else
    //         {
    //             move_to = hist[pos];
    //             std::swap((*_primitives)[move_to], prim);
    //         }
    //     }
    // }

    /* Un-increment the histogram */
    for (int i = histogram_size; i > 0; --i)
    {
        hist[i] = hist[i - 1] + 1;
    }
    hist[0] = b;
}

int morton_decode(int morton)
{
    morton &= 0x09249249;
    morton |= (morton >> 2);

    morton &= 0x030c30c3;
    morton |= (morton >> 4);

    morton &= 0x0300f00f;
    morton |= (morton >> 8);

    morton &= 0x030000ff;
    morton |= (morton >> 16);

    morton &= 0x000003ff;
    return morton;
}

point_t bih_builder::convert_to_primitve_builder(point_t *const bl, const primitive_list *const active_prims, const int b, const int e)
{
    /* Calculate grid cell size */
    const int mc = _code_buffer[b];
    const point_t mul(morton_decode(mc), morton_decode(mc >> 1), morton_decode(mc >> 2));
    const point_t grid_bound((mul * _widths) + triangle::get_scene_lower_bounds());

    /* Get primitives in the right list for the non-binned node builder */
    for (int i = b; i < e; ++i)
    {
        (*_primitives)[i] = _prim_buffer[i];
    }

    /* Set up state and call the partial block builder */
    *bl = grid_bound;
    return grid_bound + _widths;
}

point_t bih_builder::convert_to_primitve_builder(point_t *const bl, const primitive_list *const active_prims, const int b, const int e, const int begin_mc, const int end_mc, const int level)
{
    const int mc = (active_prims != _primitives) ? _code_buffer[b] : _morton_codes[b];
    const int mc_upper_bits = mc & ~((0x400 << (level * 10)) - 1);
    
    /* Calculate grid cell size, this is a little conservative, bit represents the gird so far */
    const int lower_mc = mc_upper_bits | (begin_mc << (level * 10));
    const int x_mc = morton_decode(lower_mc);
    const int y_mc = morton_decode(lower_mc >> 1);
    const int z_mc = morton_decode(lower_mc >> 2);
    const point_t lower_mul(x_mc, y_mc, z_mc);
    const point_t lower_grid((lower_mul * _widths) + triangle::get_scene_lower_bounds());
    // BOOST_LOG_TRIVIAL(trace) << "Cells morton code: " << std::hex << mc << std::dec;

    /* Get primitives in the right list for the non-binned node builder */
    if (active_prims != _primitives)
    {
        for (int i = b; i < e; ++i)
        {
            (*_primitives)[i] = _prim_buffer[i];
        }
    }

    /* Work out how far to the top of the cell */
    point_t grid_width(_widths);
    if (end_mc > 0)
    {
        const int upper_mc = mc_upper_bits | ((end_mc << (level * 10)) - 1);
        // BOOST_LOG_TRIVIAL(trace) << "End bin: " << std::hex << end_mc << std::dec << ", level: " << level;
        // BOOST_LOG_TRIVIAL(trace) << "End morton code: " << std::hex << upper_mc << std::dec << ", widths: " << _widths;

        const int x_end_mc = morton_decode(upper_mc);
        const int y_end_mc = morton_decode(upper_mc >> 1);
        const int z_end_mc = morton_decode(upper_mc >> 2);
        const point_t end_mul(x_end_mc - x_mc, y_end_mc - y_mc, z_end_mc - z_mc);
        grid_width += end_mul * _widths;
    }

    /* Set up state and call the partial block builder */
    // BOOST_LOG_TRIVIAL(trace) << "Grid points: " << lower_grid << " - " << (lower_grid + grid_width);
    *bl = lower_grid;
    return lower_grid + grid_width;
}

void bih_builder::level_switch(block_splitting_data *const split_data, const int block_idx, const int node_idx, const int data_idx)
{
    const int e                     = split_data->end[data_idx];
    const int b                     = split_data->begin[data_idx];
    const int level                 = split_data->level[data_idx];
    const unsigned int *const bins  = split_data->bins[data_idx];
    
   // BOOST_LOG_TRIVIAL(trace) << "Level: " << level << " complete moving to next level with bins: " << b << " - " << e << " and primitives: " << bins[b] << " - " << bins[e];
    switch (level)
    {
        /* Level 0 complete, call standard divider */
        case 0 :
            /* Set up state and call the partial block builder */
            split_data->tr[data_idx]    = convert_to_primitve_builder(&split_data->bl[data_idx], _primitives, bins[b], bins[e]);
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
void bih_builder::divide_bih_block(const point_t *const bl, const point_t *const tr, const unsigned int *const bins, const point_t &node_bl, const point_t &node_tr, const int block_idx, const int b, const int e, const int level)
{
    // BOOST_LOG_TRIVIAL(trace) << "Building block: " << block_idx << " for bins: " << b << " - " << e << " at level: " << level;

    block_splitting_data split_data;
    split_data.hist_bl[0]       = bl;
    split_data.hist_tr[0]       = tr;
    split_data.end[0]           = e;
    split_data.begin[0]         = b;
    split_data.bins[0]          = bins;
    split_data.level[0]         = level;
    split_data.node_tr[0]       = node_tr;
    split_data.node_bl[0]       = node_bl;
    
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
    ++_depth;
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
    ++_depth;
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
        // // BOOST_LOG_TRIVIAL(trace) << "Set child block: " << (child_idx << 3);
    }

    /* Recurse if required */
    ++_depth;
    for (int i = 0; i < 4; ++i)
    {
        if ((*_blocks)[block_idx].get_split_axis(i + 3) != axis_t::not_set)
        {
            const int left_idx = i << 1;
            const int right_idx = left_idx + 1;
            
            // const unsigned int *const left_bins = split_data.bins[left_idx];
            // const unsigned int *const right_bins = split_data.bins[right_idx];
            // if ((left_bins[split_data.end[left_idx]] - left_bins[split_data.begin[left_idx]]) < 1000)
            // {
            //     /* Recurse left */
            //     // BOOST_LOG_TRIVIAL(trace) << "Recursing left to primitive builder";
            //     const int left_begin = left_bins[split_data.begin[left_idx]];
            //     const int left_end = left_bins[split_data.end[left_idx]];
            
            //     point_t left_bl;
            //     const int left_level = split_data.level[left_idx];
            //     const primitive_list *const left_active_prims = (left_level == 0x1) ? _primitives : &_prim_buffer;
            //     const point_t left_tr(convert_to_primitve_builder(&left_bl, left_active_prims, left_begin, left_end, split_data.begin[left_idx], split_data.end[left_idx], left_level));

            //     divide_bih_block(left_bl, left_tr, split_data.node_bl[left_idx], split_data.node_tr[left_idx], child_idx, left_begin, left_end - 1);
            // }
            // else
            // {
                /* Recurse left */
                divide_bih_block(split_data.hist_bl[left_idx], split_data.hist_tr[left_idx], split_data.bins[left_idx], split_data.node_bl[left_idx], split_data.node_tr[left_idx], child_idx, split_data.begin[left_idx], split_data.end[left_idx], split_data.level[left_idx]);
            // }
            ++child_idx;
            
            // if ((left_bins[split_data.end[right_idx]] - left_bins[split_data.begin[right_idx]]) < 1000)
            // {
            //     /* Recurse right */
            //     // BOOST_LOG_TRIVIAL(trace) << "Recursing right to primitive builder";
            //     const int right_begin = right_bins[split_data.begin[right_idx]];
            //     const int right_end = right_bins[split_data.end[right_idx]];
            
            //     point_t right_bl;
            //     const int right_level = split_data.level[right_idx];
            //     const primitive_list *const right_active_prims = (right_level == 0x1) ? _primitives : &_prim_buffer;
            //     const point_t right_tr(convert_to_primitve_builder(&right_bl, right_active_prims, right_begin, right_end, split_data.begin[right_idx], split_data.end[right_idx], right_level));

            //     divide_bih_block(right_bl, right_tr, split_data.node_bl[right_idx], split_data.node_tr[right_idx], child_idx, right_begin, right_end - 1);
            // }
            // else
            // {
                /* Recurse right */
                divide_bih_block(split_data.hist_bl[right_idx], split_data.hist_tr[right_idx], split_data.bins[right_idx], split_data.node_bl[right_idx], split_data.node_tr[right_idx], child_idx, split_data.begin[right_idx], split_data.end[right_idx], split_data.level[right_idx]);
            // }
            ++child_idx;
        }
    }

    // BOOST_LOG_TRIVIAL(trace) << "Block builder complete";
    _depth -= 3;
    return;
}

void bih_builder::divide_bih_block(block_splitting_data *const split_data, const int block_idx, const int node_idx)
{
    // BOOST_LOG_TRIVIAL(trace) << "Partial building block: " << block_idx << ", " << node_idx;

    /* Split block root */
    if (node_idx <= 0)
    {
        NODE_BUILD_STEP(0, 2, 0);
        if ((*_blocks)[block_idx].get_split_axis(0) == axis_t::not_set)
        {
            return;
        }

        ++_depth;
    }

    /* Split level 1 */
    if (node_idx <= 1)
    {
        NODE_BUILD_STEP(2, 4, 1);
    }
    
    if (node_idx <= 2)
    {
        NODE_BUILD_STEP(3, 6, 2);
        ++_depth;
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
        // // BOOST_LOG_TRIVIAL(trace) << "Set child block: " << (child_idx << 3);
    }

    /* Recurse if required */
    ++_depth;
    for (int i = 0; i < 4; ++i)
    {
        if ((*_blocks)[block_idx].get_split_axis(i + 3) != axis_t::not_set)
        {
            const int left_idx = i << 1;
            const int right_idx = left_idx + 1;
            if (split_data->level[left_idx] == -1)
            {
                /* Recurse left */
                divide_bih_block(split_data->bl[left_idx], split_data->tr[left_idx], split_data->node_bl[left_idx], split_data->node_tr[left_idx], child_idx, split_data->begin[left_idx], split_data->end[left_idx]);
                ++child_idx;
                
                /* Recurse right */
                divide_bih_block(split_data->bl[right_idx], split_data->tr[right_idx], split_data->node_bl[right_idx], split_data->node_tr[right_idx], child_idx, split_data->begin[right_idx], split_data->end[right_idx]);
                ++child_idx;
            }
            else
            {
                /* Recurse left */
                divide_bih_block(split_data->hist_bl[left_idx], split_data->hist_tr[left_idx], split_data->bins[left_idx], split_data->node_bl[left_idx], split_data->node_tr[left_idx], child_idx, split_data->begin[left_idx], split_data->end[left_idx], split_data->level[left_idx]);
                ++child_idx;
                
                /* Recurse right */
                divide_bih_block(split_data->hist_bl[right_idx], split_data->hist_tr[right_idx], split_data->bins[right_idx], split_data->node_bl[right_idx], split_data->node_tr[right_idx], child_idx, split_data->begin[right_idx], split_data->end[right_idx], split_data->level[right_idx]);
                ++child_idx;
            }
        }
    }

    // BOOST_LOG_TRIVIAL(trace) << "Block builder complete";
    _depth -= 3;
    return;
}

bool bih_builder::divide_bih_node_binned(block_splitting_data *const split_data, const int in_idx, const int out_idx, const int block_idx, const int node_idx)
{
    int e = split_data->end[in_idx];
    int b = split_data->begin[in_idx];
    const int level = split_data->level[in_idx];
    const point_t &node_tr = split_data->node_tr[in_idx];
    const point_t &node_bl = split_data->node_bl[in_idx];
    const point_t *const bl = split_data->hist_bl[in_idx];
    const point_t *const tr = split_data->hist_tr[in_idx];
    const unsigned int *const bins = split_data->bins[in_idx];
    // BOOST_LOG_TRIVIAL(trace) << "Building node: " << node_idx << " for bins: " << std::hex << b << " - " << e << std::dec;

    /* Checks */
    assert(_depth < MAX_BIH_STACK_HEIGHT);
    assert(static_cast<unsigned int>(e) <= histogram_size);
    
    /* Check if we are down to 1 bin */
    if (b == e)
    {
        split_data->begin[out_idx]  = b;
        split_data->end[out_idx]    = e;
        // BOOST_LOG_TRIVIAL(trace) << "Run out of bins";
        return true;
    }
    
    /* Create leaf */
    if ((static_cast<int>(bins[e] - bins[b]) <= _max_node_size) || ((_depth + 1) == MAX_BIH_STACK_HEIGHT))
    {
        /* Possibly the primitives need moving back to the right list */
        if (!(level & 0x1))
        {
            for (unsigned int i = bins[b]; i < bins[e]; ++i)
            {
                (*_primitives)[i] = _prim_buffer[i];
            }
        }

        // BOOST_LOG_TRIVIAL(trace) << "Creating leaf at index: " << bih_index(block_idx, node_idx) << " with indices: " << bins[b] << " - " << bins[e];
        (*_blocks)[block_idx].create_leaf_node(bins[b], bins[e] - 1, node_idx);
        return false;
    }

    /* Moving towards the left bin look to split the node in 2 */
    int bin_range = (e - b) >> 1;
    int next_bin_level = bin_range >> 3;
    int split_idx = b | bin_range;

    assert((b & std::max(0, bin_range - 1)) == 0);
    assert(__builtin_popcount(bin_range) <= 1);

    // BOOST_LOG_TRIVIAL(trace) << "Try to divide bin at: " << std::hex << (b | bin_range) << " checking for primitives between: " << (b | next_bin_level) << " - " << (b | bin_range | next_bin_level) << std::dec;
    while ((bins[b | next_bin_level] == bins[b | bin_range | next_bin_level]) && (bin_range > 0))
    {
        if (bins[b] == bins[split_idx])
        {
            b += bin_range;
        }
        else if (bins[split_idx] == bins[e])
        {
            e -= bin_range;
        }
        else
        {
            break;
        }

        bin_range >>= 1;
        next_bin_level >>= 1;
        split_idx = b | bin_range;
        // BOOST_LOG_TRIVIAL(trace) << "No primitives, trying again at: " << std::hex << (b | bin_range) << " checking for primitives between: " << (b | next_bin_level) << " - " << (b | bin_range | next_bin_level) << std::dec;
    }

    if (bin_range == 0)
    {
        split_data->begin[out_idx]  = b;
        split_data->end[out_idx]    = b + 1;
        // BOOST_LOG_TRIVIAL(trace) << "Run out of bins";
        return true;
    }
    // BOOST_LOG_TRIVIAL(trace) << "Success at: " << std::hex << bin_range << " with bins: " << std::dec << bins[b | (bin_range >> 3)] << " - " << bins[b | bin_range | (bin_range >> 3)];

    /* Calculate split axis */
    const axis_t split_axis = static_cast<axis_t>(((31 - __builtin_clz(bin_range) + level) % 3) + 1);

    /* Iterate over primitive to left and right to find the node bounds */
    point_t node_tm(node_tr);
    point_t node_bm(node_bl);
    fp_t max_left   = -MAX_DIST;
    fp_t min_right  =  MAX_DIST;
    // BOOST_LOG_TRIVIAL(trace) << "Split is in: " << static_cast<int>(split_axis) << " at bin: " << std::hex << split_idx << std::dec << " primitive: " << bins[b] << " - " << bins[b + bin_range] << " - " << bins[b + (bin_range << 1)];
    switch (split_axis)
    {
        case axis_t::x_axis :
        {
            int i = b;
            for (; i < split_idx; ++i)
            {
                max_left = std::max(max_left, tr[i].x);
            }

            for (; i < e; ++i)
            {
                min_right = std::min(min_right, bl[i].x);
            }

            node_tm.x = max_left;
            node_bm.x = min_right;
            break;
        }
        case axis_t::y_axis :
        {
            int i = b;
            for (; i < split_idx; ++i)
            {
                max_left = std::max(max_left, tr[i].y);
            }

            for (; i < e; ++i)
            {
                min_right = std::min(min_right, bl[i].y);
            }

            node_tm.y = max_left;
            node_bm.y = min_right;
            break;
        }
        case axis_t::z_axis :
        {
            int i = b;
            for (; i < split_idx; ++i)
            {
                max_left = std::max(max_left, tr[i].z);
            }

            for (; i < e; ++i)
            {
                min_right = std::min(min_right, bl[i].z);
            }

            node_tm.z = max_left;
            node_bm.z = min_right;
            break;
        }
        default :
            assert(false);
    }

    /* Construct the BIH nodes */
    // BOOST_LOG_TRIVIAL(trace) << "Creating internal node at index: " << bih_index(block_idx, node_idx) << " with splits at: " << max_left << ", " << min_right << " in: " << static_cast<int>(split_axis);
    // BOOST_LOG_TRIVIAL(trace) << "Left node bounds: " << node_bl << " - " << node_tm;
    // BOOST_LOG_TRIVIAL(trace) << "Right node bounds: " << node_bm << " - " << node_tr;
    (*_blocks)[block_idx].create_generic_node(max_left, min_right, node_idx, split_axis);

    /* Left node recursion data */
    split_data->hist_bl[out_idx]    = bl;
    split_data->hist_tr[out_idx]    = tr;
    split_data->begin[out_idx]      = b;
    split_data->end[out_idx]        = split_idx;
    split_data->bins[out_idx]       = bins;
    split_data->level[out_idx]      = level;
    split_data->node_bl[out_idx]    = node_bl;
    split_data->node_tr[out_idx]    = node_tm;

    /* Right node recursion data */
    const int out_idx_p1 = out_idx + 1;
    split_data->hist_bl[out_idx_p1] = bl;
    split_data->hist_tr[out_idx_p1] = tr;
    split_data->begin[out_idx_p1]   = split_idx;
    split_data->end[out_idx_p1]     = split_idx + bin_range;
    split_data->bins[out_idx_p1]    = bins;
    split_data->level[out_idx_p1]   = level;
    split_data->node_bl[out_idx_p1] = node_bm;
    split_data->node_tr[out_idx_p1] = node_tr;
    return false;
}

/* Divide within a block, breadth first, with outside the block depth first */
void bih_builder::divide_bih_block(point_t bl, point_t tr, const point_t &node_bl, const point_t &node_tr, const int block_idx, const int b, const int e, const int node_idx)
{
    // BOOST_LOG_TRIVIAL(trace) << "Primitive block builder for block: " << block_idx << " and primitives: " << b << " - " << e;
    // BOOST_LOG_TRIVIAL(trace) << "Node size: " << node_bl << " - " << node_tr;
    // BOOST_LOG_TRIVIAL(trace) << "Grid size: " << bl << " - " << tr;
    block_splitting_data split_data;
    split_data.end[0]       = e;
    split_data.begin[0]     = b;
    split_data.tr[0]        = tr;
    split_data.bl[0]        = bl;
    split_data.node_tr[0]   = node_tr;
    split_data.node_bl[0]   = node_bl;
    
    /* Split block root */
    divide_bih_node(&split_data, 0, 2, block_idx, 0);
    if ((*_blocks)[block_idx].get_split_axis(0) == axis_t::not_set)
    {
        // BOOST_LOG_TRIVIAL(trace) << "Primitive block builder complete at root";
        return;
    }

    /* Split level 1 */
    ++_depth;
    divide_bih_node(&split_data, 2, 4, block_idx, 1);
    divide_bih_node(&split_data, 3, 6, block_idx, 2);

    /* Split level 2 */
    ++_depth;
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
        // // BOOST_LOG_TRIVIAL(trace) << "Set child block: " << (child_idx << 3);
    }

    /* Recurse if required */
    ++_depth;
    for (int i = 0; i < 4; ++i)
    {
        if ((*_blocks)[block_idx].get_split_axis(i + 3) != axis_t::not_set)
        {
            /* Recurse left */
            const int left_idx = i << 1;
            divide_bih_block(split_data.bl[left_idx], split_data.tr[left_idx], split_data.node_bl[left_idx], split_data.node_tr[left_idx], child_idx, split_data.begin[left_idx], split_data.end[left_idx]);
            ++child_idx;
            
            /* Recurse right */
            const int right_idx = left_idx + 1;
            divide_bih_block(split_data.bl[right_idx], split_data.tr[right_idx], split_data.node_bl[right_idx], split_data.node_tr[right_idx], child_idx, split_data.begin[right_idx], split_data.end[right_idx]);
            ++child_idx;
        }
    }

    // BOOST_LOG_TRIVIAL(trace) << "Primitive block builder complete";
    _depth -= 3;
    return;
}

/* Find a split position and divide the bih node in 2 */
void bih_builder::divide_bih_node(block_splitting_data *const split_data, const int in_idx, const int out_idx, const int block_idx, const int node_idx)
{
    const int e = split_data->end[in_idx];
    const int b = split_data->begin[in_idx];
    const point_t &tr = split_data->tr[in_idx];
    const point_t &bl = split_data->bl[in_idx];
    const point_t &node_tr = split_data->node_tr[in_idx];
    const point_t &node_bl = split_data->node_bl[in_idx];

    /* Checks */
    assert(_depth < MAX_BIH_STACK_HEIGHT);
    assert(static_cast<unsigned int>(e) < _primitives->size());
    assert((static_cast<unsigned int>(b) < _primitives->size()) || (b > e));

    // BOOST_LOG_TRIVIAL(trace) << "Building node: " << node_idx << " for primitives: " << b << " - " << e;
    // BOOST_LOG_TRIVIAL(trace) << "Grid bounding box " << bl << " - " << tr;
    // BOOST_LOG_TRIVIAL(trace) << "Node bounding box " << node_bl << " - " << node_tr;

    /* Create leaf */
    if (((e - b) <= _max_node_size) || ((_depth + 1) == MAX_BIH_STACK_HEIGHT))
    {
        // // BOOST_LOG_TRIVIAL(trace) << "Creating leaf at index: " << bih_index(block_idx, node_idx) << " with indices: " << b << " - " << e;
        (*_blocks)[block_idx].create_leaf_node(b, e, node_idx);
        
        // for (unsigned int i = b; i <= e; ++i)
        // {
        //   // BOOST_LOG_TRIVIAL(trace) << "creating leaf for: " << i;
        // }
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
    // BOOST_LOG_TRIVIAL(trace) << "Attempting to split at: " << split_pnt;

    /* Partition _primitives */
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
            while (bottom < top)
            {
                while (split_pnt < (((*_primitives)[top]->highest_x() + (*_primitives)[top]->lowest_x()) * 0.5f) && top > bottom)
                {
                    min_right = std::min(min_right, (*_primitives)[top--]->lowest_x());
                }
                std::swap((*_primitives)[bottom], (*_primitives)[top]);
                
                while (split_pnt >= (((*_primitives)[bottom]->highest_x() + (*_primitives)[bottom]->lowest_x()) * 0.5f) && bottom < top)
                {
                    max_left = std::max(max_left, (*_primitives)[bottom++]->highest_x());
                }
                std::swap((*_primitives)[bottom], (*_primitives)[top]);
            }             

            /* Parition the last primitive */
            if (split_pnt >= (((*_primitives)[bottom]->highest_x() + (*_primitives)[bottom]->lowest_x()) * 0.5f))
            {
                max_left = std::max(max_left, (*_primitives)[bottom++]->highest_x());
            }
            else
            {
                min_right = std::min(min_right, (*_primitives)[top]->lowest_x());
            }

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

            right_size = e - bottom; /* 1 less than the actual node size */
            if ((right_size == -1) && (max_left == node_tm.x))
            {
                // // BOOST_LOG_TRIVIAL(trace) << "Blank node recursing left: " << max_left << ", " << node_tm.x;
                const point_t width(tm - bl);
                if ((width.x * width.y * width.z) < _min_node_volume)
                {
                    (*_blocks)[block_idx].create_leaf_node(b, std::max(0, bottom - 1), node_idx);
                }
                else
                {
                    split_data->tr[in_idx] = tm;
                    divide_bih_node(split_data, in_idx, out_idx, block_idx, node_idx);
                }
                return;
            }

            left_size = bottom - b;
            if ((left_size == 0) && (min_right == node_bm.x))
            {
                // // BOOST_LOG_TRIVIAL(trace) << "Blank node recursing right: " << min_right << ", " << node_bm.x;
                const point_t width(tr - bm);
                if ((width.x * width.y * width.z) < _min_node_volume)
                {
                    (*_blocks)[block_idx].create_leaf_node(bottom, e, node_idx);
                }
                else
                {
                    split_data->bl[in_idx] = bm;
                    divide_bih_node(split_data, in_idx, out_idx, block_idx, node_idx);
                }
                return;
            }

            node_tm.x = max_left;
            node_bm.x = min_right;

            break;
        }
        case axis_t::y_axis :
        {
            while (bottom < top)
            {
                while (split_pnt < (((*_primitives)[top]->highest_y() + (*_primitives)[top]->lowest_y()) * 0.5f) && top > bottom)
                {
                    min_right = std::min(min_right, (*_primitives)[top--]->lowest_y());
                }
                std::swap((*_primitives)[bottom], (*_primitives)[top]);
         
                while (split_pnt >= (((*_primitives)[bottom]->highest_y() + (*_primitives)[bottom]->lowest_y()) * 0.5f) && bottom < top)
                {
                    max_left = std::max(max_left, (*_primitives)[bottom++]->highest_y());
                }
                std::swap((*_primitives)[bottom], (*_primitives)[top]);
            }

            /* Parition the last primitive */
            if (split_pnt >= (((*_primitives)[bottom]->highest_y() + (*_primitives)[bottom]->lowest_y()) * 0.5f))
            {
                max_left = std::max(max_left, (*_primitives)[bottom++]->highest_y());
            }
            else
            {
                min_right = std::min(min_right, (*_primitives)[top]->lowest_y());
            }

            float pos = (tm.y - bl.y) * 0.5f;
            // BOOST_LOG_TRIVIAL(trace) << "max_left: " << max_left << ", pos: " << pos << ", bl.y: " << bl.y << ", tm.y: " << tm.y;
            while (((bl.y + pos) > max_left) && (max_left > -MAX_DIST))
            {
                tm.y = bl.y + pos;
                pos *= 0.5f;
            }

            pos = (tr.y - bm.y) * 0.5f;
            // BOOST_LOG_TRIVIAL(trace) << "min_right: " << min_right << ", pos: " << pos << ", tr.y: " << tr.y << ", bm.y: " << bm.y;
            while (((tr.y - pos) < min_right) && (min_right < MAX_DIST))
            {
                bm.y = tr.y - pos;
                pos *= 0.5f;
            }

            right_size = e - bottom; /* 1 less than the actual node size */
            if ((right_size == -1) && (max_left == node_tm.y))
            {
                // // BOOST_LOG_TRIVIAL(trace) << "Blank node recursing left: " << max_left << ", " << node_tm.y;
                const point_t width(tm - bl);
                if ((width.x * width.y * width.z) < _min_node_volume)
                {
                    (*_blocks)[block_idx].create_leaf_node(b, std::max(0, bottom - 1), node_idx);
                }
                else
                {
                    split_data->tr[in_idx] = tm;
                    divide_bih_node(split_data, in_idx, out_idx, block_idx, node_idx);
                }
                return;
            }

            left_size = bottom - b;
            if ((left_size == 0) && (min_right == node_bm.y))
            {
                // // BOOST_LOG_TRIVIAL(trace) << "Blank node recursing right: " << min_right << ", " << node_bm.y;
                const point_t width(tr - bm);
                if ((width.x * width.y * width.z) < _min_node_volume)
                {
                    (*_blocks)[block_idx].create_leaf_node(bottom, e, node_idx);
                }
                else
                {
                    split_data->bl[in_idx] = bm;
                    divide_bih_node(split_data, in_idx, out_idx, block_idx, node_idx);
                }
                return;
            }

            node_tm.y = max_left;
            node_bm.y = min_right;
            break;
        }
        case axis_t::z_axis :
        {
            while (bottom < top)
            {
                while (split_pnt < (((*_primitives)[top]->highest_z() + (*_primitives)[top]->lowest_z()) * 0.5f) && top > bottom)
                {
                    min_right = std::min(min_right, (*_primitives)[top--]->lowest_z());
                }
                std::swap((*_primitives)[bottom], (*_primitives)[top]);
         
                while (split_pnt >= (((*_primitives)[bottom]->highest_z() + (*_primitives)[bottom]->lowest_z()) * 0.5f) && bottom < top)
                {
                    max_left = std::max(max_left, (*_primitives)[bottom++]->highest_z());
                }
                std::swap((*_primitives)[bottom], (*_primitives)[top]);
            }

            /* Parition the last primitive */
            if (split_pnt >= (((*_primitives)[bottom]->highest_z() + (*_primitives)[bottom]->lowest_z()) * 0.5f))
            {
                max_left = std::max(max_left, (*_primitives)[bottom++]->highest_z());
            }
            else
            {
                min_right = std::min(min_right, (*_primitives)[top]->lowest_z());
            }

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

            right_size = e - bottom; /* 1 less than the actual node size */
            if ((right_size == -1) && (max_left == node_tm.z))
            {
                // // BOOST_LOG_TRIVIAL(trace) << "Blank node recursing left: " << max_left << ", " << node_tm.z;
                const point_t width(tm - bl);
                if ((width.x * width.y * width.z) < _min_node_volume)
                {
                    (*_blocks)[block_idx].create_leaf_node(b, std::max(0, bottom - 1), node_idx);
                }
                else
                {
                    split_data->tr[in_idx] = tm;
                    divide_bih_node(split_data, in_idx, out_idx, block_idx, node_idx);
                }
                return;
            }

            left_size = bottom - b;
            if ((left_size == 0) && (min_right == node_bm.z))
            {
                // // BOOST_LOG_TRIVIAL(trace) << "Blank node recursing right: " << min_right << ", " << node_bm.z;
                const point_t width(tr - bm);
                if ((width.x * width.y * width.z) < _min_node_volume)
                {
                    (*_blocks)[block_idx].create_leaf_node(bottom, e, node_idx);
                }
                else
                {
                    split_data->bl[in_idx] = bm;
                    divide_bih_node(split_data, in_idx, out_idx, block_idx, node_idx);
                }
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
    // BOOST_LOG_TRIVIAL(trace) << "Creating internal node at index: " << bih_index(block_idx, node_idx) << " with splits at: " << max_left << ", " << min_right << " in: " << static_cast<int>(split_axis);
    (*_blocks)[block_idx].create_generic_node(max_left, min_right, node_idx, split_axis);

    /* Left node recursion data */
    split_data->level[out_idx]      = -1;
    split_data->begin[out_idx]      = b;
    split_data->end[out_idx]        = std::max(0, bottom - 1);
    split_data->tr[out_idx]         = tm;
    split_data->bl[out_idx]         = bl;
    split_data->node_tr[out_idx]    = node_tm;
    split_data->node_bl[out_idx]    = node_bl;

    /* Right node recursion data */
    const int out_idx_p1 = out_idx + 1;
    split_data->level[out_idx_p1]   = -1;
    split_data->begin[out_idx_p1]   = bottom;
    split_data->end[out_idx_p1]     = e;
    split_data->tr[out_idx_p1]      = tr;
    split_data->bl[out_idx_p1]      = bm;
    split_data->node_tr[out_idx_p1] = node_tr;
    split_data->node_bl[out_idx_p1] = node_bm;
}
}; /* namespace raptor_raytracer */
