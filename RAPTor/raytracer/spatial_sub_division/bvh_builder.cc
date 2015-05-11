/* Standard headers */

/* Boost headers */

/* Common headers */
#include "logging.h"

/* Ray tracer headers */
#include "bvh_builder.h"


namespace raptor_raytracer
{
const int histogram_size = 1024;


/* Build a Bounding Volumne Heirarchy for _primitives into nodes */
int bvh_builder::build(primitive_list *const primitives, std::vector<bvh_node> *const nodes)
{
    BOOST_LOG_TRIVIAL(trace) << "BVH construction has begun";

    /* Maximum theoretical size is everything.size() * 6 node, but this is very unlikely */
    _primitives = primitives;
    _nodes = nodes;
    _nodes->resize(std::max(1, static_cast<int>(_primitives->size()) << 1));
    _code_buffer.reset(new int [_primitives->size()]);
    _prim_buffer.resize(_primitives->size());
    _morton_codes.reset(new int [_primitives->size()]);

    /* Calculate Morton codes and build histograms */
    const point_t scene_width(triangle::get_scene_upper_bounds() - triangle::get_scene_lower_bounds());
    const point_t width(scene_width * (1.00001f / 1024.0f));
    const point_t width_inv(1.0f / width);
    _max_leaf_sah = ((scene_width.x * scene_width.y) + (scene_width.x *  scene_width.z) + (scene_width.y * scene_width.z)) * _max_leaf_sah_factor;

    unsigned int hist0[histogram_size * 3];
    unsigned int *hist1 = hist0 + histogram_size;
    unsigned int *hist2 = hist1 + histogram_size;
    memset(hist0, 0, histogram_size * 3 * sizeof(float));

    const vfp_t x_mul(width_inv.x);
    const vfp_t y_mul(width_inv.y);
    const vfp_t z_mul(width_inv.z);
    const vfp_t scene_lo_x(triangle::get_scene_lower_bounds().x);
    const vfp_t scene_lo_y(triangle::get_scene_lower_bounds().y);
    const vfp_t scene_lo_z(triangle::get_scene_lower_bounds().z);
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
        ++hist0[ _morton_codes[i    ]        & 0x3ff];
        ++hist1[(_morton_codes[i    ] >> 10) & 0x3ff];
        ++hist2[ _morton_codes[i    ] >> 20         ];

        ++hist0[ _morton_codes[i + 1]        & 0x3ff];
        ++hist1[(_morton_codes[i + 1] >> 10) & 0x3ff];
        ++hist2[ _morton_codes[i + 1] >> 20         ];

        ++hist0[ _morton_codes[i + 2]        & 0x3ff];
        ++hist1[(_morton_codes[i + 2] >> 10) & 0x3ff];
        ++hist2[ _morton_codes[i + 2] >> 20         ];

        ++hist0[ _morton_codes[i + 3]        & 0x3ff];
        ++hist1[(_morton_codes[i + 3] >> 10) & 0x3ff];
        ++hist2[ _morton_codes[i + 3] >> 20         ];
    }

    for (int i = (static_cast<int>(_primitives->size()) & ~(SIMD_WIDTH - 1)); i < static_cast<int>(_primitives->size()); ++i)
    {
        /* Calculate morton code*/
        const point_t t((((*_primitives)[i]->highest_point() + (*_primitives)[i]->lowest_point()) * 0.5f) - triangle::get_scene_lower_bounds());
        _morton_codes[i] = morton_code(t.x, t.y, t.z, width_inv.x, width_inv.y, width_inv.z);

        /* Build histogram of morton codes */
        ++hist0[ _morton_codes[i]        & 0x3ff];
        ++hist1[(_morton_codes[i] >> 10) & 0x3ff];
        ++hist2[ _morton_codes[i] >> 20         ];
    }

    /* Sum the histogram */
    /* TODO - Perhaps change this to LSB radix sort */
    /* Then I will have bin count from the last sweep, but I probably need more primitive sweeps for LSB versus MSB radix sort */
    unsigned int sum0 = 0;
    unsigned int sum1 = 0;
    unsigned int sum2 = 0;
    for (int i = 0; i < histogram_size; ++i)
    {
        const unsigned int tmp0 = hist0[i] + sum0;
        hist0[i] = sum0 - 1;
        sum0 = tmp0;

        const unsigned int tmp1 = hist1[i] + sum1;
        hist1[i] = sum1 - 1;
        sum1 = tmp1;

        const unsigned int tmp2 = hist2[i] + sum2;
        hist2[i] = sum2 - 1;
        sum2 = tmp2;
    }

    /* Move based on histogram 0 */
    for (int i = 0; i < static_cast<int>(_primitives->size()); ++i)
    {
        /* Calculate destination and move morton code */
        const unsigned int data = _morton_codes[i];
        const unsigned int pos = data & 0x3ff;
        _code_buffer[++hist0[pos]]  = data;
        _prim_buffer[hist0[pos]]    = (*_primitives)[i];
    }

    /* Move based on histogram 1 */
    for (int i = 0; i < static_cast<int>(_primitives->size()); ++i)
    {
        /* Calculate destination and move morton code */
        const unsigned int data = _code_buffer[i];
        const unsigned int pos = (data >> 10) & 0x3ff;
        _morton_codes[++hist1[pos]] = data;
        (*_primitives)[hist1[pos]]  = _prim_buffer[i];
    }

    /* Move based on histogram 2 */
    for (int i = 0; i < static_cast<int>(_primitives->size()); ++i)
    {
        /* Calculate destination and move morton code */
        const unsigned int data = _morton_codes[i];
        const unsigned int pos = data >> 20;
        _code_buffer[++hist2[pos]]  = data;
        _prim_buffer[hist2[pos]]    = (*_primitives)[i];
    }
    _prim_buffer.swap(*_primitives);

    /* Allocate the cost matrix */
    // _rows = std::ceil(0.5f * (1.0f + std::pow(n, _epsilon * -0.5f) * std::sqrt(std::pow(n, _epsilon) + 16.0f * std::pow(_delta, 0.5f + _epsilon) * std::pow(n, (1.0f + _epsilon) * 0.5f))));
    _rows = 4 * static_cast<int>(_delta * 0.5f * std::pow(_primitives->size() / 2.0f / (_delta * 0.5f), 0.5f - _alpha / 2.0f) + 1e-5f);
    _cost_matrix.resize(((_rows * _rows) - _rows) >> 1, 0);
    _cost_addrs.reserve(_rows);

    /* Begin recursion */
    int cost_b = 0x0;
    int cost_e = 0x40000000;
    build_layer(&cost_b, &cost_e, triangle::get_scene_lower_bounds(), triangle::get_scene_upper_bounds(), 0, _primitives->size());

    /* Combine last layer */
    combine_nodes(&cost_b, &cost_e, 1, cost_b, cost_e, cost_e, 0);
    assert(cost_e == 1);
    assert(_depth == 0);

    return _next_node - 1;
}

/* Calculate the number of bvh nodes that should leave this layer */
int bvh_builder::reduction_function(const int n) const
{
    return std::max(1.0f, (std::pow(_delta, 0.5f + _epsilon) * std::pow(static_cast<float>(n), _alpha) * 0.5f));
    // return static_cast<int>(0.5f * _delta * std::pow(static_cast<float>(n)/(0.5f * _delta)/2, 0.5f - _alpha) - 1e-8) + 1;
}
// int(minSize*pow(float(len)/minSize/2, 0.5-alpha)-epsi)+1;
// 4*int(minSize*pow(model->numTri/2.0/minSize, 0.5-alpha/2)+1e-5)

/* Find the surface area from combining these two nodes */
float bvh_builder::cost_function(const bvh_node &l, const bvh_node &r) const
{
    return l.combined_surface_area(r);
}

float bvh_builder::cost_function(const triangle *const l, const point_t &low, const point_t &high, const float nodes) const
{
    /* Size of combined node */
    const point_t bl(min(low, l->lowest_point()));
    const point_t tr(max(high, l->highest_point()));

    /* Edges of new node */
    const point_t dist(tr - bl);

    return ((dist.x * dist.y) + (dist.x * dist.z) + (dist.y * dist.z)) * nodes;
}

int bvh_builder::start_index(const int col) const
{
    /* Work out the address to the nearest even row */
    const int even_col = col & ~0x1;

    /* Width of the top row */
    const int top_width = _rows - even_col;

    /* Address of even row */
    int addr = (_rows + top_width - 1) * (even_col >> 1);

    /* Add in for an odd col */
    if (col & 0x1)
    {
        addr += _rows - col;
    }

    return addr;
}

axis_t bvh_builder::divide_spatial_bounds(point_t *const bm, point_t *const tm, const point_t &bl, const point_t &tr, const axis_t axis) const
{
    switch (axis)
    {
        case axis_t::x_axis : 
        {
            const float split = (tr.x + bl.x) * 0.5f;
            bm->x = split;
            tm->x = split;
            return axis_t::z_axis;
        }

        case axis_t::y_axis : 
        {
            const float split = (tr.y + bl.y) * 0.5f;
            bm->y = split;
            tm->y = split;
            return axis_t::x_axis;
        }

        case axis_t::z_axis : 
        {
            const float split = (tr.z + bl.z) * 0.5f;
            bm->z = split;
            tm->z = split;
            return axis_t::y_axis;
        }

        default :
            assert(false);
            return axis_t::not_set;
    }
}

void bvh_builder::build_leaf_node(int *const cost_b, int *const cost_e, const int b, const int e, const int node_size)
{
    /* Check we have enough space */
    const int cost_begin = _cost_addrs.size();
    if (static_cast<int>(_cost_addrs.size() + node_size) > _rows)
    {
        const int start_idx = start_index(_cost_addrs.size());
        const int prev_rows = _rows;

        _rows = _cost_addrs.size() + node_size;
        _cost_matrix.resize(((_rows * _rows) - _rows) >> 1, 0);

        /* Move data */
        int from_idx = start_idx - ((prev_rows - cost_begin) << 1) - 1;    /* Where I should write new results and back two rows */
        int out_idx = from_idx + ((_rows - prev_rows) * (cost_begin - 2));
        for (int i = cost_begin - 1; i > 0; --i)
        {
            for (int j = cost_begin - i; j > 0; --j)
            {
                assert(out_idx  >= 0);
                assert(from_idx >= 0);
                _cost_matrix[out_idx--] = _cost_matrix[from_idx--];
            }

            out_idx  -= (_rows - cost_begin);
            from_idx -= (prev_rows - cost_begin);
        }
        assert(out_idx  < 0);
        assert(from_idx < 0);
    }

    /* Add new nodes */
    const int start_idx = start_index(_cost_addrs.size());
    *cost_b = _cost_addrs.size();

    for (int i = b; i < e; )
    {
        int node_end = i + 1;
        int layer_top = e;
        point_t low((*_primitives)[i]->lowest_point());
        point_t high((*_primitives)[i]->highest_point());
        while (node_end < layer_top)
        {
            if (cost_function((*_primitives)[node_end], low, high, node_end - i + 1) > _max_leaf_sah)
            {
                std::swap((*_primitives)[node_end], (*_primitives)[--layer_top]);
            }
            else
            {
                low     = min(low, (*_primitives)[node_end]->lowest_point());
                high    = max(high, (*_primitives)[node_end]->highest_point());
                ++node_end;
            }
        }
        /* Reserve space in cost address */
        const int node_idx = _next_node++;
        _cost_addrs.push_back(node_idx);

        /* Build node */
        (*_nodes)[node_idx].create_leaf_node(high, low, i, node_end);
        i = node_end;
    }

    *cost_e = _cost_addrs.size();
    const int cost_end = *cost_e;

    /* Initial population of the cost matrix */
    int row_idx = start_idx;
    for (int i = cost_begin; i < cost_end; ++i)
    {
        for (int j = i + 1; j < cost_end; ++j)
        {
            const float cost = cost_function((*_nodes)[_cost_addrs[i]], (*_nodes)[_cost_addrs[j]]);
            _cost_matrix[row_idx++] = cost;
        }

        /* Leave some blank space and move to the next row*/
        row_idx += (_rows - cost_end);
        assert(row_idx >= 0);
    }

    combine_nodes(cost_b, cost_e, reduction_function(std::max(cost_end - cost_begin, static_cast<int>(_delta))), cost_begin, cost_end, cost_end, start_idx);
}

void bvh_builder::build_layer(int *const cost_b, int *const cost_e, const point_t &bl, const point_t &tr, const int b, const int e, axis_t axis)
{
    assert(_depth < _max_down_phase_depth);
    ++_depth;

    /* Check if we want a leaf node */
    const int node_size = e - b;
    if ((node_size < _delta) || (_depth == _max_down_phase_depth))
    {
        build_leaf_node(cost_b, cost_e, b, e, node_size);
        --_depth;
        return;
    }
    /* No more morton codes, begin primitive partitioning */
    else if (*cost_b == (*cost_e - 1))
    {
        build_layer_primitive(cost_b, cost_e, bl, tr, b, e, axis);
        --_depth;
        return;
    }

    /* Divide the morton codes in 2 again */
    const int morton_middle = *cost_b + ((*cost_e - *cost_b) >> 1);
    const int middle_idx = std::distance(&_code_buffer[0], std::lower_bound(&_code_buffer[b], &_code_buffer[e], morton_middle));
    assert((middle_idx == e) || (_code_buffer[middle_idx]     >= morton_middle));
    assert((middle_idx == 0) || (_code_buffer[middle_idx - 1] <  morton_middle));

    /* Shrink node bounds */
    point_t bm(bl);
    point_t tm(tr);
    axis = divide_spatial_bounds(&bm, &tm, bl, tr, axis);

    /* If some primitives are in the left node then recurse left */
    int cost_begin = *cost_b;
    int cost_m = morton_middle;
    if (b != middle_idx)
    {
        build_layer(&cost_begin, &cost_m, bl, tm, b, middle_idx, axis);
    }

    /* If some primitives are in the right node then recurse right */
    int cost_end = *cost_e;
    int cost_mr = morton_middle;
    if (middle_idx != e)
    {
        build_layer(&cost_mr, &cost_end, bm, tr, middle_idx, e, axis);
    }
    else
    {
        cost_end = cost_m;
    }

    /* We didnt recurse left, use the right recursion to partition the left bounds */
    if (b == middle_idx)
    {
        cost_m = cost_mr;
        cost_begin = cost_mr;
    }


    const int start_idx = start_index(cost_begin);
    combine_nodes(cost_b, cost_e, reduction_function(node_size), cost_begin, cost_m, cost_end, start_idx);
    --_depth;
}

void bvh_builder::build_layer_primitive(int *const cost_b, int *const cost_e, const point_t &bl, const point_t &tr, const int b, const int e, axis_t axis)
{
    assert(_depth < _max_down_phase_depth);
    ++_depth;

    /* Check if we want a leaf node */
    const int node_size = e - b;
    if ((node_size < _delta) || (_depth == _max_down_phase_depth))
    {
        build_leaf_node(cost_b, cost_e, b, e, node_size);
        --_depth;
        return;
    }

    /* Sort primitives about split and track bounds */
    int middle_idx;
    int top     = e - 1;
    int bottom  = b;
    switch (axis)
    {
        case axis_t::x_axis : 
        {
            const float dbl_split = (tr.x + bl.x);
            while (bottom < top)
            {
                /* Skip primitives in the correct place*/
                while (dbl_split < ((*_primitives)[top]->highest_point().x + (*_primitives)[top]->lowest_point().x) && bottom < top)
                {
                    --top;
                }

                while (dbl_split >= ((*_primitives)[bottom]->highest_point().x + (*_primitives)[bottom]->lowest_point().x) && bottom < top)
                {
                    ++bottom;
                }

                /* Swap incorrect primitives */
                if (bottom < top)
                {
                    std::swap((*_primitives)[bottom], (*_primitives)[top]);
                    --top;
                    ++bottom;
                }
            }

            middle_idx = bottom;
            if ((top == bottom) && (dbl_split >= ((*_primitives)[top]->highest_point().x + (*_primitives)[top]->lowest_point().x)))
            {
                ++middle_idx;
            }
            break;
        }

        case axis_t::y_axis : 
        {
            const float dbl_split = (tr.y + bl.y);
            while (bottom < top)
            {
                /* Skip primitives in the correct place*/
                while (dbl_split < ((*_primitives)[top]->highest_point().y + (*_primitives)[top]->lowest_point().y) && bottom < top)
                {
                    --top;
                }

                while (dbl_split >= ((*_primitives)[bottom]->highest_point().y + (*_primitives)[bottom]->lowest_point().y) && bottom < top)
                {
                    ++bottom;
                }

                /* Swap incorrect primitives */
                if (bottom < top)
                {
                    std::swap((*_primitives)[bottom], (*_primitives)[top]);
                    --top;
                    ++bottom;
                }
            }

            middle_idx = bottom;
            if ((top == bottom) && (dbl_split >= ((*_primitives)[top]->highest_point().y + (*_primitives)[top]->lowest_point().y)))
            {
                ++middle_idx;
            }
            break;
        }

        case axis_t::z_axis : 
        {
            const float dbl_split = (tr.z + bl.z);
            while (bottom < top)
            {
                /* Skip primitives in the correct place*/
                while (dbl_split < ((*_primitives)[top]->highest_point().z + (*_primitives)[top]->lowest_point().z) && bottom < top)
                {
                    --top;
                }

                while (dbl_split >= ((*_primitives)[bottom]->highest_point().z + (*_primitives)[bottom]->lowest_point().z) && bottom < top)
                {
                    ++bottom;
                }

                /* Swap incorrect primitives */
                if (bottom < top)
                {
                    std::swap((*_primitives)[bottom], (*_primitives)[top]);
                    --top;
                    ++bottom;
                }
            }

            middle_idx = bottom;
            if ((top == bottom) && (dbl_split >= ((*_primitives)[top]->highest_point().z + (*_primitives)[top]->lowest_point().z)))
            {
                ++middle_idx;
            }
            break;
        }

        default :
            assert(false);
    }

    // const int middle_idx = (b + e) >> 1;

    /* Shrink node bounds */
    point_t bm(bl);
    point_t tm(tr);
    axis = divide_spatial_bounds(&bm, &tm, bl, tr, axis);

    /* If some primitives are in the left node then recurse left */
    int cost_begin = *cost_b;
    int cost_m = middle_idx;
    if (b != middle_idx)
    {
        build_layer_primitive(&cost_begin, &cost_m, bl, tm, b, middle_idx, axis);
    }

    /* If some primitives are in the right node then recurse right */
    int cost_end = *cost_e;
    int cost_mr = middle_idx;
    if (middle_idx != e)
    {
        build_layer_primitive(&cost_mr, &cost_end, bm, tr, middle_idx, e, axis);
    }
    else
    {
        cost_end = cost_m;
    }

    /* We didnt recurse left, use the right recursion to partition the left bounds */
    if (b == middle_idx)
    {
        cost_m = cost_mr;
        cost_begin = cost_mr;
    }

    const int start_idx = start_index(cost_begin);
    combine_nodes(cost_b, cost_e, reduction_function(node_size), cost_begin, cost_m, cost_end, start_idx);
    --_depth;
}

void bvh_builder::combine_nodes(int *const cost_b, int *const cost_e, const int layer_size, const int cost_begin, const int cost_m, int cost_end, const int cost_start_idx)
{
    /* Below us is build combine the current internal nodes */
    /* Build the new piece of the cost matrix */
    // BOOST_LOG_TRIVIAL(trace) << "Combinig: " << cost_begin << " via: " << cost_m << " to: " << cost_end;
    // dump_cost();
    int row_idx = cost_start_idx + cost_m - cost_begin;
    for (int i = cost_begin; i < cost_m; ++i)
    {
        for (int j = cost_m; j < cost_end; ++j)
        {
            const float cost = cost_function((*_nodes)[_cost_addrs[i]], (*_nodes)[_cost_addrs[j]]);
            // BOOST_LOG_TRIVIAL(trace) << "Populating: " << _cost_addrs[i] << ", " << _cost_addrs[j] << ", at: " << (row_idx + j - cost_m - 1) << ", cost: " << cost;
            _cost_matrix[row_idx + j - cost_m - 1] = cost;
        }

        /* Leave some blank space and move to the next row*/
        row_idx += (_rows - i - 2);
        assert(row_idx >= 0);
    }
    // dump_cost();

    /* Until we have sufficiently few remaining node */
    while ((cost_end - cost_begin) > layer_size)
    {
        /* Search for lowest cost */
        int lowest_i;
        int lowest_j;
        int row_idx = cost_start_idx;
        float lowest_cost = std::numeric_limits<float>::max();
        for (int i = cost_begin; i < cost_end - 1; ++i)
        {
            for (int j = i + 1; j < cost_end; ++j)
            {
                const float cost = _cost_matrix[row_idx++];
                if (cost < lowest_cost)
                {
                    lowest_i    = i;
                    lowest_j    = j;
                    lowest_cost = cost;
                }
            }

            /* Leave some blank space and move to the next row*/
            row_idx += (_rows - cost_end);
            assert(row_idx >= 0);
        }

        /* Make the lowest cost row and column into a new node */
        const int node_idx = _next_node++;
        (*_nodes)[node_idx].create_generic_node(*_nodes, _cost_addrs[lowest_i], _cost_addrs[lowest_j]);
        // assert(lowest_cost == cost_function((*_nodes)[_cost_addrs[lowest_i]], (*_nodes)[_cost_addrs[lowest_j]]));
        _cost_addrs[lowest_i] = node_idx;
        _cost_addrs[lowest_j] = _cost_addrs.back();
        _cost_addrs.pop_back();
        --cost_end;

        /* Calculate costs versus the new node */
        row_idx = cost_start_idx;
        for (int i = cost_begin; i < lowest_i; ++i)
        {
            const float cost = cost_function((*_nodes)[node_idx], (*_nodes)[_cost_addrs[i]]);
            _cost_matrix[row_idx + lowest_i - i - 1] = cost;
            row_idx += (_rows - i  - 1);
            assert(row_idx >= 0);
        }

        for (int i = lowest_i + 1; i < cost_end; ++i)
        {
            const float cost = cost_function((*_nodes)[node_idx], (*_nodes)[_cost_addrs[i]]);
            _cost_matrix[row_idx + i - lowest_i - 1] = cost;
        }

        /* Move the top column in to replace its space */
        row_idx = cost_start_idx;
        int row_size = cost_end - cost_begin;
        for (int i = cost_begin; i < lowest_j; ++i)
        {
            if (i != lowest_i) /* Note - j is always higher than i */
            {
                _cost_matrix[row_idx + lowest_j - i - 1] = _cost_matrix[row_idx + row_size - 1];
            }
            row_idx += (_rows - i - 1);
            --row_size;
            assert(row_idx >= 0);
        }

        int col_idx = row_idx;
        row_idx += ((_rows - lowest_j) - 2);
        --row_size;
        for (int i = lowest_j + 1; i < cost_end; ++i)
        {
            _cost_matrix[col_idx++] = _cost_matrix[row_idx + row_size];
            row_idx += (_rows - i - 1);
            --row_size;
        }
        // dump_cost();
    }

    /* Pass up the costs */
    *cost_e = cost_end;
    *cost_b = cost_begin;
}

// void bvh_builder::dump_cost() const
// {
//     for (int i = 0; i < std::min(_rows, static_cast<int>(_cost_addrs.size())); ++i)
//     {
//     }

//     int mat_idx = 0;
//     for (int i = 0; i < std::min(_rows, static_cast<int>(_cost_addrs.size())); ++i)
//     {
//         for (int j = 0; j <= i; ++j)
//         {
//         }

//         for (int j = i + 1; j < std::min(_rows, static_cast<int>(_cost_addrs.size())); ++j)
//         {
//         }
//         mat_idx += (_rows - std::min(_rows, static_cast<int>(_cost_addrs.size())));
//     }
// }
}; /* namespace raptor_raytracer */
