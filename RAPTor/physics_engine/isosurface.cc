/* Standard headers */

/* Boost headers */

/* Common headers */

#include "physics_common.h"
#include "isosurface.h"


namespace raptor_physics
{
void isosurface::to_triangles(raptor_raytracer::primitive_store *p)
{
    if (_meta_balls.empty())
    {
        return;
    }
    
    /* Find are covered by meta balls */
    float min_radius = _meta_balls[0].radius();
    point_t lower_bound(_meta_balls[0].center() - _meta_balls[0].radius());
    point_t upper_bound(_meta_balls[0].center() + _meta_balls[0].radius());
    for (std::size_t i = 1; i < _meta_balls.size(); ++i)
    {
        min_radius = std::min(min_radius, _meta_balls[i].radius());
        lower_bound = min(lower_bound, (_meta_balls[i].center() - _meta_balls[i].radius()));
        upper_bound = max(upper_bound, (_meta_balls[i].center() + _meta_balls[i].radius()));
    }
    std::cout << "Lower grid bounds: " << lower_bound << ", upper grid bounds: " << upper_bound << std::endl;

    /* The grid needs to be atleast min_radius or we would miss entire balls */
    const float grid_cell_size = (min_radius * 0.0625);
    const point_t grid_size(upper_bound - lower_bound);
    const int x_size = grid_size.x / grid_cell_size;
    const int y_size = grid_size.y / grid_cell_size;
    const int z_size = grid_size.z / grid_cell_size;
    const int x_row = y_size * z_size;
    const int y_row = z_size;
    std::cout << "grid cells: " << x_size << ", " << y_size << ", " << z_size << std::endl;
    
    std::vector<int> cell_usage((x_size * x_row) + 1, 0);    
    for (int i = 0; i < static_cast<int>(_meta_balls.size()); ++i)
    {
        const auto &m = _meta_balls[i];
        const int width = m.radius() / (grid_cell_size * 0.5f);
        const point_ti<int> lower_cell((m.center() - m.radius() - lower_bound) / grid_cell_size);
        for (int x = lower_cell.x; x < (lower_cell.x + width); ++x)
        {
            const int x_offset = x * x_row;
            for (int y = lower_cell.y; y < (lower_cell.y + width); ++y)
            {
                const int y_offset = x_offset + (y * z_size);
                for (int z = lower_cell.z; z < (lower_cell.z + width); ++z)
                {
                    ++cell_usage[y_offset + z + 1];
                }
            }
        }
    }

    unsigned int sum = 0;
    for (int i = 1; i < static_cast<int>(cell_usage.size()); ++i)
    {
        const unsigned int tmp = cell_usage[i] + sum;
        cell_usage[i] = sum;
        sum = tmp;
    }

    _grid.resize(sum);
    for (int i = 0; i < static_cast<int>(_meta_balls.size()); ++i)
    {
        const auto &m = _meta_balls[i];
        const int width = m.radius() / (grid_cell_size * 0.5f);
        const point_ti<int> lower_cell((m.center() - m.radius() - lower_bound) / grid_cell_size);
        for (int x = lower_cell.x; x < (lower_cell.x + width); ++x)
        {
            const int x_offset = x * x_row;
            for (int y = lower_cell.y; y < (lower_cell.y + width); ++y)
            {
                const int y_offset = x_offset + (y * z_size);
                for (int z = lower_cell.z; z < (lower_cell.z + width); ++z)
                {
                    _grid[cell_usage[y_offset + z + 1]++] = i;
                }
            }
        }
    }

    /* Run marching cubes to make triangles */
    const int grid_value_mask = next_power_of_two(((y_size + 1) * (z_size + 1)) + z_size + 2) - 1;
    _grid_values.reset(new float[grid_value_mask + 1]);
    std::memset(&_grid_values[0], 0, (grid_value_mask + 1) * sizeof(float));

    point_t v_n[3];
    point_t normals[12];
    point_t intersects[12];
    const point_t vertex_offsets[8] = 
    {
        point_t(grid_cell_size, grid_cell_size, grid_cell_size),
        point_t(grid_cell_size, grid_cell_size,           0.0f),
        point_t(grid_cell_size,           0.0f,           0.0f),
        point_t(grid_cell_size,           0.0f, grid_cell_size),
        point_t(          0.0f, grid_cell_size, grid_cell_size),
        point_t(          0.0f, grid_cell_size,           0.0f),
        point_t(          0.0f,           0.0f,           0.0f),
        point_t(          0.0f,           0.0f, grid_cell_size),
    };

    const int x_cell = (y_size + 1) * (z_size + 1);
    const int y_cell = z_size + 1;
    // std::cout << "x cell: " << x_cell << ", y cell: " << y_cell << std::endl;

    int grid_value_offset = 0;
    point_t pos(lower_bound.x + grid_cell_size, lower_bound.y, lower_bound.z);
    for (int x = 0; x < x_size; ++x)
    {
        for (int z = 0; z < (z_size + 1); ++z)
        {
            _grid_values[grid_value_offset++] = 0;
            grid_value_offset &= grid_value_mask;
        }

        pos.y = lower_bound.y + grid_cell_size;
        const int x_offset = x * x_row;
        for (int y = 0; y < y_size; ++y)
        {
            _grid_values[grid_value_offset++] = 0;
            grid_value_offset &= grid_value_mask;

            pos.z = lower_bound.z + grid_cell_size;
            const int y_offset = x_offset + (y * y_row);
            for (int z = 0; z < z_size; ++z)
            {
                /* Calculate newest grid value */
                const int grid_offset = y_offset + z;
                grid_value_offset = calculate_grid_value(&_grid[cell_usage[grid_offset]], &_grid[cell_usage[grid_offset + 1]], pos, grid_value_offset, grid_value_mask);
                // std::cout << "Value at: " << x << ", " << y << ", " << z << " (" << pos << ") is: " << _grid_values[(grid_value_offset - 1) & grid_value_mask] << ", address: " << std::dec << ((grid_value_offset - 1) & grid_value_mask) << std::endl;

                /* Find the vertices inside the isosurface */
                int in_verts = 0;
                const float grid_value_0 = _grid_values[(grid_value_offset - x_cell - y_cell - 2) & grid_value_mask];
                const float grid_value_1 = _grid_values[(grid_value_offset - x_cell - y_cell - 1) & grid_value_mask];
                const float grid_value_2 = _grid_values[(grid_value_offset - x_cell          - 1) & grid_value_mask];
                const float grid_value_3 = _grid_values[(grid_value_offset - x_cell          - 2) & grid_value_mask];
                const float grid_value_4 = _grid_values[(grid_value_offset         - y_cell  - 2) & grid_value_mask];
                const float grid_value_5 = _grid_values[(grid_value_offset         - y_cell  - 1) & grid_value_mask];
                const float grid_value_6 = _grid_values[(grid_value_offset                   - 1) & grid_value_mask];
                const float grid_value_7 = _grid_values[(grid_value_offset                   - 2) & grid_value_mask];
                in_verts |= (grid_value_0 > _threshold);
                in_verts |= (grid_value_1 > _threshold) << 1;
                in_verts |= (grid_value_2 > _threshold) << 2;
                in_verts |= (grid_value_3 > _threshold) << 3;
                in_verts |= (grid_value_4 > _threshold) << 4;
                in_verts |= (grid_value_5 > _threshold) << 5;
                in_verts |= (grid_value_6 > _threshold) << 6;
                in_verts |= (grid_value_7 > _threshold) << 7;

                /* All vertices are inside or outside so nothing to do*/
                const int edge_value = edge_table[in_verts];
                if (edge_value == 0x000)
                {
                    pos.z += grid_cell_size;
                    continue;
                }

                /* Use the edge table to find edges that flip side and calculate the intersect */
                // std::cout << "Using grid values: " <<
                //     grid_value_0 << ", from: " << std::dec << ((grid_value_offset - x_cell - y_cell - 2) & grid_value_mask) << std::endl <<
                //     grid_value_1 << ", from: " << std::dec << ((grid_value_offset - x_cell - y_cell - 1) & grid_value_mask) << std::endl <<
                //     grid_value_2 << ", from: " << std::dec << ((grid_value_offset - x_cell          - 1) & grid_value_mask) << std::endl <<
                //     grid_value_3 << ", from: " << std::dec << ((grid_value_offset - x_cell          - 2) & grid_value_mask) << std::endl <<
                //     grid_value_4 << ", from: " << std::dec << ((grid_value_offset         - y_cell  - 2) & grid_value_mask) << std::endl <<
                //     grid_value_5 << ", from: " << std::dec << ((grid_value_offset         - y_cell  - 1) & grid_value_mask) << std::endl <<
                //     grid_value_6 << ", from: " << std::dec << ((grid_value_offset                   - 1) & grid_value_mask) << std::endl <<
                //     grid_value_7 << ", from: " << std::dec << ((grid_value_offset                   - 2) & grid_value_mask) << std::endl;
                // std::cout << "Grid " << x << ", " << y << ", " << z << ", gives vertices: " << std::hex << in_verts << ", egdes: " << std::hex << edge_value << std::endl;
                if (edge_value & 0x001)
                {
                    intersects[0] = bisect(&_grid[cell_usage[grid_offset]], &_grid[cell_usage[grid_offset + 1]], pos - vertex_offsets[0], pos - vertex_offsets[1], grid_value_0, grid_value_1);
                    assert(min(intersects[0], lower_bound) == lower_bound);
                    assert(max(intersects[0], upper_bound) == upper_bound);

                    normals[0] = normal(&_grid[cell_usage[grid_offset]], &_grid[cell_usage[grid_offset + 1]], intersects[0]);
                    // std::cout << "Case 0 got intersect: " << intersects[0] << ", from position: " << (pos) << " and " << (pos - vertex_offsets[1]) << ", normal: " << normals[0] << std::endl;
                }

                if (edge_value & 0x002)
                {
                    intersects[1] = bisect(&_grid[cell_usage[grid_offset]], &_grid[cell_usage[grid_offset + 1]], pos - vertex_offsets[1], pos - vertex_offsets[2], grid_value_1, grid_value_2);
                    assert(min(intersects[1], lower_bound) == lower_bound);
                    assert(max(intersects[1], upper_bound) == upper_bound);

                    normals[1] = normal(&_grid[cell_usage[grid_offset]], &_grid[cell_usage[grid_offset + 1]], intersects[1]);
                    // std::cout << "Case 1 got intersect: " << intersects[1] << ", from position: " << (pos - vertex_offsets[1]) << " and " << (pos - vertex_offsets[2]) << ", normal: " << normals[1] << std::endl;
                }

                if (edge_value & 0x004)
                {
                    intersects[2] = bisect(&_grid[cell_usage[grid_offset]], &_grid[cell_usage[grid_offset + 1]], pos - vertex_offsets[2], pos - vertex_offsets[3], grid_value_2, grid_value_3);
                    assert(min(intersects[2], lower_bound) == lower_bound);
                    assert(max(intersects[2], upper_bound) == upper_bound);

                    normals[2] = normal(&_grid[cell_usage[grid_offset]], &_grid[cell_usage[grid_offset + 1]], intersects[2]);
                    // std::cout << "Case 2 got intersect: " << intersects[2] << ", from position: " << (pos - vertex_offsets[2]) << " and " << (pos - vertex_offsets[3]) << ", normal: " << normals[2] << std::endl;
                }

                if (edge_value & 0x008)
                {
                    intersects[3] = bisect(&_grid[cell_usage[grid_offset]], &_grid[cell_usage[grid_offset + 1]], pos - vertex_offsets[3], pos - vertex_offsets[0], grid_value_3, grid_value_0);
                    assert(min(intersects[3], lower_bound) == lower_bound);
                    assert(max(intersects[3], upper_bound) == upper_bound);

                    normals[3] = normal(&_grid[cell_usage[grid_offset]], &_grid[cell_usage[grid_offset + 1]], intersects[3]);
                    // std::cout << "Case 3 got intersect: " << intersects[3] << ", from position: " << (pos - vertex_offsets[3]) << " and " << (pos) << ", normal: " << normals[3] << std::endl;
                }

                if (edge_value & 0x010)
                {
                    intersects[4] = bisect(&_grid[cell_usage[grid_offset]], &_grid[cell_usage[grid_offset + 1]], pos - vertex_offsets[4], pos - vertex_offsets[5], grid_value_4, grid_value_5);
                    assert(min(intersects[4], lower_bound) == lower_bound);
                    assert(max(intersects[4], upper_bound) == upper_bound);

                    normals[4] = normal(&_grid[cell_usage[grid_offset]], &_grid[cell_usage[grid_offset + 1]], intersects[4]);
                    // std::cout << "Case 4 got intersect: " << intersects[4] << ", from position: " << (pos - vertex_offsets[4]) << " and " << (pos - vertex_offsets[5]) << ", normal: " << normals[4] << std::endl;
                }

                if (edge_value & 0x020)
                {
                    intersects[5] = bisect(&_grid[cell_usage[grid_offset]], &_grid[cell_usage[grid_offset + 1]], pos - vertex_offsets[5], pos - vertex_offsets[6], grid_value_5, grid_value_6);
                    assert(min(intersects[5], lower_bound) == lower_bound);
                    assert(max(intersects[5], upper_bound) == upper_bound);

                    normals[5] = normal(&_grid[cell_usage[grid_offset]], &_grid[cell_usage[grid_offset + 1]], intersects[5]);
                    // std::cout << "Case 5 got intersect: " << intersects[5] << ", from position: " << (pos - vertex_offsets[5]) << " and " << (pos - vertex_offsets[6]) << ", normal: " << normals[5] << std::endl;
                }

                if (edge_value & 0x040)
                {
                    intersects[6] = bisect(&_grid[cell_usage[grid_offset]], &_grid[cell_usage[grid_offset + 1]], pos - vertex_offsets[6], pos - vertex_offsets[7], grid_value_6, grid_value_7);
                    assert(min(intersects[6], lower_bound) == lower_bound);
                    assert(max(intersects[6], upper_bound) == upper_bound);

                    normals[6] = normal(&_grid[cell_usage[grid_offset]], &_grid[cell_usage[grid_offset + 1]], intersects[6]);
                    // std::cout << "Case 6 got intersect: " << intersects[6] << ", from position: " << (pos - vertex_offsets[6]) << " and " << (pos - vertex_offsets[7]) << ", normal: " << normals[6] << std::endl;
                }

                if (edge_value & 0x080)
                {
                    intersects[7] = bisect(&_grid[cell_usage[grid_offset]], &_grid[cell_usage[grid_offset + 1]], pos - vertex_offsets[7], pos - vertex_offsets[4], grid_value_7, grid_value_4);
                    assert(min(intersects[7], lower_bound) == lower_bound);
                    assert(max(intersects[7], upper_bound) == upper_bound);

                    normals[7] = normal(&_grid[cell_usage[grid_offset]], &_grid[cell_usage[grid_offset + 1]], intersects[7]);
                    // std::cout << "Case 7 got intersect: " << intersects[7] << ", from position: " << (pos - vertex_offsets[7]) << " and " << (pos - vertex_offsets[4]) << ", normal: " << normals[7] << std::endl;
                }

                if (edge_value & 0x100)
                {
                    intersects[8] = bisect(&_grid[cell_usage[grid_offset]], &_grid[cell_usage[grid_offset + 1]], pos - vertex_offsets[0], pos - vertex_offsets[4], grid_value_0, grid_value_4);
                    assert(min(intersects[8], lower_bound) == lower_bound);
                    assert(max(intersects[8], upper_bound) == upper_bound);

                    normals[8] = normal(&_grid[cell_usage[grid_offset]], &_grid[cell_usage[grid_offset + 1]], intersects[8]);
                    // std::cout << "Case 8 got intersect: " << intersects[8] << ", from position: " << (pos) << " and " << (pos - vertex_offsets[4]) << ", normal: " << normals[8] << std::endl;
                }

                if (edge_value & 0x200)
                {
                    intersects[9] = bisect(&_grid[cell_usage[grid_offset]], &_grid[cell_usage[grid_offset + 1]], pos - vertex_offsets[1], pos - vertex_offsets[5], grid_value_1, grid_value_5);
                    assert(min(intersects[9], lower_bound) == lower_bound);
                    assert(max(intersects[9], upper_bound) == upper_bound);

                    normals[9] = normal(&_grid[cell_usage[grid_offset]], &_grid[cell_usage[grid_offset + 1]], intersects[9]);
                    // std::cout << "Case 9 got intersect: " << intersects[9] << ", from position: " << (pos - vertex_offsets[1]) << " and " << (pos - vertex_offsets[5]) << ", normal: " << normals[9] << std::endl;
                }

                if (edge_value & 0x400)
                {
                    intersects[10] = bisect(&_grid[cell_usage[grid_offset]], &_grid[cell_usage[grid_offset + 1]], pos - vertex_offsets[2], pos - vertex_offsets[6], grid_value_2, grid_value_6);
                    assert(min(intersects[10], lower_bound) == lower_bound);
                    assert(max(intersects[10], upper_bound) == upper_bound);

                    normals[10] = normal(&_grid[cell_usage[grid_offset]], &_grid[cell_usage[grid_offset + 1]], intersects[10]);
                    // std::cout << "Case 10 got intersect: " << intersects[10] << ", from position: " << (pos - vertex_offsets[2]) << " and " << (pos - vertex_offsets[6]) << ", normal: " << normals[10] << std::endl;
                }

                if (edge_value & 0x800)
                {
                    intersects[11] = bisect(&_grid[cell_usage[grid_offset]], &_grid[cell_usage[grid_offset + 1]], pos - vertex_offsets[3], pos - vertex_offsets[7], grid_value_3, grid_value_7);
                    assert(min(intersects[11], lower_bound) == lower_bound);
                    assert(max(intersects[11], upper_bound) == upper_bound);

                    normals[11] = normal(&_grid[cell_usage[grid_offset]], &_grid[cell_usage[grid_offset + 1]], intersects[11]);
                    // std::cout << "Case 11 got intersect: " << intersects[11] << ", from position: " << (pos - vertex_offsets[3]) << " and " << (pos - vertex_offsets[7]) << ", normal: " << normals[11] << std::endl;
                }

                
                /* Use the triangle table to build triangles based on the inside and outside edges */
                const int *const tri_verts = triangle_table[in_verts];
                // std::cout << "Grid " << x << ", " << y << ", " << z << ", building triangle:" << std::endl;
                for (int i = 0; tri_verts[i] != -1; i += 3)
                {
                    // std::cout << "a: " << intersects[tri_verts[i]] << ", b: " << intersects[tri_verts[i + 1]] << ", c: " << intersects[tri_verts[i + 2]] << std::endl;
                    v_n[0] = normals[tri_verts[i]];
                    v_n[1] = normals[tri_verts[i + 1]];
                    v_n[2] = normals[tri_verts[i + 2]];
                    p->emplace_back(_mat, intersects[tri_verts[i]], intersects[tri_verts[i + 1]], intersects[tri_verts[i + 2]], false, v_n);
                }

                // std::cout << std::endl;
                pos.z += grid_cell_size;
            }
            pos.y += grid_cell_size;
        }
        pos.x += grid_cell_size;
    }
}

int isosurface::calculate_grid_value(const int *grid_begin, const int *const grid_end, const point_t &pos, const int grid_value_offset, const int grid_value_mask)
{
    /* Calculate newest grid value */
    _grid_values[grid_value_offset] = 0.0f;
    for (const int *i = grid_begin; i < grid_end; ++i)
    {
        const auto &m = _meta_balls[*i];
        _grid_values[grid_value_offset] += m.value(pos);
    }

    return (grid_value_offset + 1) & grid_value_mask;
}

point_t isosurface::bisect(const int *grid_begin, const int *const grid_end, const point_t &pa, const point_t &pb, const float va, const float vb) const
{
    float lv;
    float uv;
    point_t lp;
    point_t up;
    if (va < vb)
    {
        lp = pa;
        up = pb;
        lv = va - _threshold;
        uv = vb - _threshold;
    }
    else
    {
        lp = pb;
        up = pa;
        lv = vb - _threshold;
        uv = va - _threshold;
    }

    float value;
    int iter = 0;
    point_t guess;
    point_t dist;
    do
    {
        // std::cout << "Upper point: " << up << ", upper value: " << uv << ", lower point: " << lp << ", lower value: " << lv << std::endl;
        guess = lp + ((up - lp) * (-lv / (uv - lv)));
        
        value = 0.0f;
        for (const int *i = grid_begin; i < grid_end; ++i)
        {
            const auto &m = _meta_balls[*i];
            value += m.value(guess);
        }
        value -= _threshold;
        // std::cout << "Guess: " << guess << ", got value: " << value << std::endl;

        if (value > 0.0f)
        {
            up = guess;
            uv = value;
        }
        else
        {
            lp = guess;
            lv = value;
        }

        dist = up - lp;
    } while ((std::fabs(value) > 0.001f) && (dot_product(dist, dist) > 0.001f) && (++iter < 5));

    return guess;
}

point_t isosurface::normal(const int *const grid_begin, const int *const grid_end, const point_t &pos) const
{
    point_t normal;
    for (const int *i = grid_begin; i < grid_end; ++i)
    {
        const auto &m = _meta_balls[*i];
        normal += m.normal(pos) * m.value(pos);
    }

    // std::cout << "raw normal: " << normal << std::endl;
    return normalise(normal / static_cast<float>(grid_end - grid_begin));
}
}; /* namespace raptor_physics */
