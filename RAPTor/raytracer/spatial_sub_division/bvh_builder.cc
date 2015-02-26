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
void bvh_builder::build(primitive_list *const primitives, std::vector<bvh_node> *const nodes)
{
    // BOOST_LOG_TRIVIAL(trace) << "BVH construction has begun";

    /* Maximum theoretical size is everything.size() * 6 node, but this is very unlikely */
    _primitives = primitives;
    _nodes = nodes;
    _nodes->resize(std::max(1, static_cast<int>(_primitives->size() * 0.2f)));
    _code_buffer.reset(new int [_primitives->size()]);
    _prim_buffer.reset(new triangle *[_primitives->size()]);
    _morton_codes.reset(new int [_primitives->size()]);

    /* Calculate Morton codes and build histograms */
    const point_t scene_width(triangle::get_scene_upper_bounds() - triangle::get_scene_lower_bounds());
    const float width_inv = 1.0f / std::max(std::max(scene_width.x, scene_width.y), scene_width.z) * (1.00001f / 1024.0f);

    unsigned int hist0[histogram_size * 3];
    unsigned int *hist1 = hist0 + histogram_size;
    unsigned int *hist2 = hist1 + histogram_size;
    memset(hist0, 0, histogram_size * 3 * sizeof(float));

    const vfp_t x_mul(width_inv);
    const vfp_t y_mul(width_inv);
    const vfp_t z_mul(width_inv);
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
        _morton_codes[i] = morton_code(t.x, t.y, t.z, width_inv, width_inv, width_inv);

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
        const unsigned int pos = data >> 20;
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
        const unsigned int pos = data & 0x3ff;
        _code_buffer[++hist2[pos]]  = data;
        _prim_buffer[hist2[pos]]    = (*_primitives)[i];
    }

    /* Sweep the morton codes to find the primitives used for the first level of clustering */
    // int i = 0;
    // while (i < static_cast<int>(_primitives->size()))
    // {
    //     /* Build a cluster of morton codes until we have a leaf nodes worth of primitives */
    //     int prim_cnt = 1;
    //     const int cluster_start = i;
    //     while (prim_cnt < _max_node_size)
    //     {
    //         /* Count the primitives with the same morton code */
    //         const int run_start = i;
    //         while (_code_buffer[run_start] == _code_buffer[++i])
    //         {
    //             ++prim_cnt;
    //         }
    //     }
    
    //     /* Maintain a power of 2 long cluster to keep things square */
    //     // i = next_power_of_2(i - i);

    //     /* Build nodes for all primitives between cluster start and i */
    // }
}
}; /* namespace raptor_raytracer */
