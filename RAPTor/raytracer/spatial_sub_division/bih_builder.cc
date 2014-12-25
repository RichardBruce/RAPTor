/* Standard headers */

/* Boost headers */

/* Common headers */
#include "logging.h"

/* Ray tracer headers */
#include "bih_builder.h"


namespace raptor_raytracer
{
/* Build a Bounding Interval Heirarchy for primitives into blocks */
void bih_builder::build(primitive_list *const primitives, std::vector<bih_block> *const blocks)
{
    // BOOST_LOG_TRIVIAL(trace) << "BIH construction has begun";

    /* Maximum theoretical size is everything.size() * 6 node, but this is very unlikely */
    _primitives = primitives;
    _blocks = blocks;
    _blocks->resize(std::max(1ul, _primitives->size() >> 1));

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
        divide_bih_node(triangle::get_scene_lower_bounds(), triangle::get_scene_upper_bounds(), triangle::get_scene_lower_bounds(), triangle::get_scene_upper_bounds(), 0, 0, 0, _primitives->size() - 1);
        assert(_depth == 0);
    }
    BOOST_LOG_TRIVIAL(trace) << "BIH construction used: " << _next_block << " blocks";
}

// void node_size(const primitive_list &primitives, point_t *const bl, point_t *const tr, const int b, const int e)
// {
//     *bl = primitives[b]->lowest_point();
//     *tr = primitives[b]->highest_point();
//     for (int i = b + 1; i < e; ++i)
//     {
//         *bl = min(*bl, primitives[i]->lowest_point());
//         *tr = max(*tr, primitives[i]->highest_point());
//     }
// }

// void bucket_build(primitive_list *const primitives, std::vector<bih_block> *const blocks)
// {
//     /* Calculate the average primitive sizes */
//     float x_width = 0.0f;
//     float y_width = 0.0f;
//     float z_width = 0.0f;
//     for (for auto t : *primitives)
//     {
//         x_width += t->highest_x() - t->lowest_x();
//         y_width += t->highest_y() - t->lowest_y();
//         z_width += t->highest_z() - t->lowest_z();
//     }

//     const float nr_primitives_inv = 1.0f / primitives->size();
//     x_width *= nr_primitives_inv;
//     y_width *= nr_primitives_inv;
//     z_width *= nr_primitives_inv;

//     /* Calculate Morton codes */
//     const point_t scene_width(triangle::get_scene_upper_bounds() - triangle::get_scene_lower_bounds());
//     const vfp_t x_div(scene_width.x / x_width);
//     const vfp_t y_div(scene_width.y / y_width);
//     const vfp_t z_div(scene_width.z / z_width);
//     std::vector<int> morton_codes(primitives->size());
//     for (int i = 0; i < (primitives->size() - SIMD_WIDTH); ++i)
//     {
//         const point_t t0((*primitives)[i]->get_vertex_a());
//         const point_t t1((*primitives)[i + 1]->get_vertex_a());
//         const point_t t2((*primitives)[i + 2]->get_vertex_a());
//         const point_t t3((*primitives)[i + 3]->get_vertex_a());
//         vint_t mc(morton_code(vfp_t(t0.x, t1.x, t2.x, t3.x), vfp_t(t0.y, t1.y, t2.y, t3.y), vfp_t(t0.z, t1.z, t2.z, t3.z), x_div, y_div, z_div));
//         mc.save(&morton_codes[i]);
//     }

//     /* Begin MSB radix sort */
//     std::vector<int> sort_buffer(primitives->size());

//     /* Build histogram */
//     const int histogram_size = 2049;
//     unsigned int hist[histogram_size];
//     memset(hist, 0, histogram_size * sizeof(float));
//     for (int i = 0; i < primitives->size(); ++i)
//     {
//         ++hist[morton_codes[i] >> 22];
//     }
    
//     /* Sum the histogram */
//     unsigned int sum = 0;
//     for (int i = 0; i < histogram_size; ++i)
//     {
//         const unsigned int tmp = hist[i] + sum;
//         hist[i] = sum - 1;
//         sum = tmp;
//     }

//     /* Move based on histogram */
//     for (int i = 0; i < primitives->size(); ++i)
//     {
//         const unsigned int data = array[i];
//         const unsigned int pos = data >> 22;
//         sort_buffer[++hist[pos]] = data;
//     }

//     /* For bins containing a lot of data radix sort again, for bins not much data bin normal sub division procedure */
//     for (int i = 1; i < histogram_size - 1; ++i)
//     {
//         const int bucket_size = hist[i] - hist[i - 1];
//         if (bucket_size > 512)
//         {
//             bucket_build_mid();
//         }
//         else if (bucket_size > 0)
//         {
//             point_t node_bl;
//             point_t node_tr;
//             node_size(const primitive_list &primitives, &bl, &tr, hist[i - 1], hist[i]);
//             divide_bih_node(point_t bl, point_t tr, node_bl, node_tr, const int block_idx, const int node_idx, hist[i - 1], hist[i]);
//         }
//     }

//     /* Breadth first, bottom up, partitioning of the data */
//     for (int stride = 2; stride <= histogram_size; stride <<= 1)
//     {
//         for (int i = (stride >> 1); i < histogram_size; i += stride)
//         {
//             /* If there are primitives in either partition then we should split here */
//             if (((hist[i] - hist[i - stride]) > 0) || ((hist[i + stride] - hist[i]) > 0))
//             {
//     // (*_blocks)[block_idx].create_generic_node(max_left, min_right, node_idx, split_axis);
//             }
//         }
//     }
// }

// void bucket_build_mid(std::vector<int> *const morton_codes, std::vector<int> *const sort_buffer, const int b, const int e)
// {
//     /* Build histogram */
//     const int histogram_size = 2049;
//     unsigned int hist[histogram_size];
//     memset(hist, 0, histogram_size * sizeof(float));
//     for (int i = b; i < e; ++i)
//     {
//         ++hist[(morton_codes[i] >> 11) & 0x7ff];
//     }
    
//     /* Sum the histogram */
//     unsigned int sum = 0;
//     for (int i = 0; i < histogram_size; ++i)
//     {
//         const unsigned int tmp = hist[i] + sum;
//         hist[i] = sum - 1;
//         sum = tmp;
//     }

//     /* Move based on histogram */
//     for (int i = b; i < e; ++i)
//     {
//         const unsigned int data = morton_codes[i];
//         const unsigned int pos = (data >> 11) & 0x7ff;
//         sort_buffer[b + ++hist[pos]] = data;
//     }

//     /* Breadth first partitioning of the data */
//     for (int stride = histogram_size; stride > 1; stride >>= 1)
//     {
//         for (int i = (stride >> 1); i < histogram_size; i += stride)
//         {
//             /* If there are primitives in either partition then we should split here */
//             if (((hist[i] - hist[i - stride]) > 0) || ((hist[i + stride] - hist[i]) > 0))
//             {
//     // (*_blocks)[block_idx].create_generic_node(max_left, min_right, node_idx, split_axis);
//             }
//         }
//     }

//     /* For bins containing a lot of data radix sort again, for bins not much data bin normal sub division procedure */
//     for (int i = 1; i < histogram_size - 1; ++i)
//     {
//         const int bucket_size = hist[i] - hist[i - 1];
//         if (bucket_size > 512)
//         {
//             bucket_build_low();
//         }
//         else if (bucket_size > 0)
//         {
//             divide_bih_node(point_t bl, point_t tr, const point_t &node_bl, const point_t &node_tr, const int block_idx, const int node_idx, hist[i - 1], hist[i]);
//         }
//     }
// }

// void bucket_build_low(std::vector<int> *const morton_codes, std::vector<int> *const sort_buffer, const int b, const int e)
// {
//     /* Build histogram */
//     const int histogram_size = 2049;
//     unsigned int hist[histogram_size];
//     memset(hist, 0, histogram_size * sizeof(float));
//     for (int i = b; i < e; ++i)
//     {
//         ++hist[morton_codes[i] & 0x7ff];
//     }
    
//     /* Sum the histogram */
//     unsigned int sum = 0;
//     for (int i = 0; i < histogram_size; ++i)
//     {
//         const unsigned int tmp = hist[i] + sum;
//         hist[i] = sum - 1;
//         sum = tmp;
//     }

//     /* Move based on histogram */
//     for (int i = b; i < e; ++i)
//     {
//         const unsigned int data = morton_codes[i];
//         const unsigned int pos = data & 0x7ff;
//         sort_buffer[b + ++hist[pos]] = data;
//     }

//     /* Breadth first partitioning of the data */
//     for (int stride = histogram_size; stride > 1; stride >>= 1)
//     {
//         for (int i = (stride >> 1); i < histogram_size; i += stride)
//         {
//             /* If there are primitives in either partition then we should split here */
//             if (((hist[i] - hist[i - stride]) > 0) || ((hist[i + stride] - hist[i]) > 0))
//             {
//     // (*_blocks)[block_idx].create_generic_node(max_left, min_right, node_idx, split_axis);
//             }
//         }
//     }

//     /* For bins containing a data begin normal sub division procedure */
//     for (int i = 1; i < histogram_size - 1; ++i)
//     {
//         const int bucket_size = hist[i] - hist[i - 1];
//         if (bucket_size > 0)
//         {
//             divide_bih_node(point_t bl, point_t tr, const point_t &node_bl, const point_t &node_tr, const int block_idx, const int node_idx, hist[i - 1], hist[i]);
//         }
//     }
// }

/* Find a split position and divide the bih node in 2 */
void bih_builder::divide_bih_node(point_t bl, point_t tr, const point_t &node_bl, const point_t &node_tr, const int block_idx, const int node_idx, const int b, const int e)
{
    // BOOST_LOG_TRIVIAL(trace) << "Grid bounding box " << bl << " - " << tr;
    // BOOST_LOG_TRIVIAL(trace) << "Node bounding box " << node_bl << " - " << node_tr;

    /* _depth check */
    ++_depth;
    assert(_depth < MAX_BIH_STACK_HEIGHT);
    assert(static_cast<unsigned int>(e) < _primitives->size());
    assert((static_cast<unsigned int>(b) < _primitives->size()) || (b > e));
    
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
                // BOOST_LOG_TRIVIAL(trace) << "Blank node recursing left: " << max_left << ", " << node_tm.x;
                --_depth;
                const point_t width(tm - bl);
                if ((width.x * width.y * width.z) < _min_node_volume)
                {
                    (*_blocks)[block_idx].create_leaf_node(b, std::max(0, bottom - 1), node_idx);
                }
                else
                {
                    divide_bih_node(bl, tm, node_bl, node_tr, block_idx, node_idx, b, e);
                }
                return;
            }

            left_size = bottom - b;
            if ((left_size == 0) && (min_right == node_bm.x))
            {
                // BOOST_LOG_TRIVIAL(trace) << "Blank node recursing right: " << min_right << ", " << node_bm.x;
                --_depth;
                const point_t width(tr - bm);
                if ((width.x * width.y * width.z) < _min_node_volume)
                {
                    (*_blocks)[block_idx].create_leaf_node(bottom, e, node_idx);
                }
                else
                {
                    divide_bih_node(bm, tr, node_bl, node_tr, block_idx, node_idx, b, e);
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

            right_size = e - bottom; /* 1 less than the actual node size */
            if ((right_size == -1) && (max_left == node_tm.y))
            {
                // BOOST_LOG_TRIVIAL(trace) << "Blank node recursing left: " << max_left << ", " << node_tm.y;
                --_depth;
                const point_t width(tm - bl);
                if ((width.x * width.y * width.z) < _min_node_volume)
                {
                    (*_blocks)[block_idx].create_leaf_node(b, std::max(0, bottom - 1), node_idx);
                }
                else
                {
                    divide_bih_node(bl, tm, node_bl, node_tr, block_idx, node_idx, b, e);
                }
                return;
            }

            left_size = bottom - b;
            if ((left_size == 0) && (min_right == node_bm.y))
            {
                // BOOST_LOG_TRIVIAL(trace) << "Blank node recursing right: " << min_right << ", " << node_bm.y;
                --_depth;
                const point_t width(tr - bm);
                if ((width.x * width.y * width.z) < _min_node_volume)
                {
                    (*_blocks)[block_idx].create_leaf_node(bottom, e, node_idx);
                }
                else
                {
                    divide_bih_node(bm, tr, node_bl, node_tr, block_idx, node_idx, b, e);
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
                // BOOST_LOG_TRIVIAL(trace) << "Max left: " << tm.z << ", " << bl.z << ", " << pos << ", " << max_left;
                tm.z = bl.z + pos;
                pos *= 0.5f;
            }

            pos = (tr.z - bm.z) * 0.5f;
            while (((tr.z - pos) < min_right) && (min_right < MAX_DIST))
            {
                // BOOST_LOG_TRIVIAL(trace) << "Min right: " << bm.z << ", " << tr.z << ", " << pos << ", " << min_right;
                bm.z = tr.z - pos;
                pos *= 0.5f;
            }

            right_size = e - bottom; /* 1 less than the actual node size */
            if ((right_size == -1) && (max_left == node_tm.z))
            {
                // BOOST_LOG_TRIVIAL(trace) << "Blank node recursing left: " << max_left << ", " << node_tm.z;
                --_depth;
                const point_t width(tm - bl);
                if ((width.x * width.y * width.z) < _min_node_volume)
                {
                    (*_blocks)[block_idx].create_leaf_node(b, std::max(0, bottom - 1), node_idx);
                }
                else
                {
                    divide_bih_node(bl, tm, node_bl, node_tr, block_idx, node_idx, b, e);
                }
                return;
            }

            left_size = bottom - b;
            if ((left_size == 0) && (min_right == node_bm.z))
            {
                // BOOST_LOG_TRIVIAL(trace) << "Blank node recursing right: " << min_right << ", " << node_bm.z;
                --_depth;
                const point_t width(tr - bm);
                if ((width.x * width.y * width.z) < _min_node_volume)
                {
                    (*_blocks)[block_idx].create_leaf_node(bottom, e, node_idx);
                }
                else
                {
                    divide_bih_node(bm, tr, node_bl, node_tr, block_idx, node_idx, b, e);
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

    /* Get extra blocks if needed */
    if ((*_blocks)[block_idx].requires_block_children(node_idx))
    {
        const int blocks_required = bih_block::child_blocks_required();
        const int child = _next_block.fetch_add(blocks_required);

        const int new_size = child + blocks_required;
        if (new_size > static_cast<int>(_blocks->size()))
        {
            _blocks->resize(new_size);
        }
        
        (*_blocks)[block_idx].set_child_block(child << 3);
        // BOOST_LOG_TRIVIAL(trace) << "Set child block: " << (child << 3);
    }

    /* Construct the BIH nodes */
    // BOOST_LOG_TRIVIAL(trace) << "Creating internal node at index: " << bih_index(block_idx, node_idx) << " with splits at: " << max_left << ", " << min_right << " in: " << static_cast<int>(split_axis);
    (*_blocks)[block_idx].create_generic_node(max_left, min_right, node_idx, split_axis);

    /* Recurse left */
    const int left_idx = (*_blocks)[block_idx].get_left_child(block_idx, node_idx);
    const int left_block = block_index(left_idx);
    const int left_node = node_index(left_idx);
    if ((left_size <= _max_node_size) || ((_depth + 1) == MAX_BIH_STACK_HEIGHT))
    {
        // BOOST_LOG_TRIVIAL(trace) << "Creating leaf for left child at index: " << bih_index(left_block, left_node) << " with indices: " << b << " - " << (bottom - 1);
        (*_blocks)[left_block].create_leaf_node(b, std::max(0, bottom - 1), left_node);
    }
    else
    {
        // BOOST_LOG_TRIVIAL(trace) << "Recursing for left child with indices: " << b << " - " << (bottom - 1);
        divide_bih_node(bl, tm, node_bl, node_tm, left_block, left_node, b, bottom - 1);
    }

    /* Recurse right */
    const int right_idx = (*_blocks)[block_idx].get_right_child(block_idx, node_idx);
    const int right_block = block_index(right_idx);
    const int right_node = node_index(right_idx);
    if ((right_size < _max_node_size) || ((_depth + 1) == MAX_BIH_STACK_HEIGHT))
    {
        // BOOST_LOG_TRIVIAL(trace) << "Creating leaf for right child at index: " << bih_index(right_block, right_node) << "  with indices: " << bottom << " - " << e;
        (*_blocks)[right_block].create_leaf_node(bottom, e, right_node);
    }
    else
    {
        // BOOST_LOG_TRIVIAL(trace) << "Recursing for right child with indices: " << bottom << " - " << e;
        divide_bih_node(bm, tr, node_bm, node_tr, right_block, right_node, bottom, e);
    }
    
    --_depth;
    return;
}
}; /* namespace raptor_raytracer */
