/* Standard headers */
#include <cassert>
#include <queue>

/* Convex decomposition headers */
#include "dac_convex_hull.h"
#include "volume.h"


namespace raptor_convex_decomposition
{
#define FINDMINMAX(x0, x1, x2, min, max)    \
          min = max = x0;                   \
          if (x1 < min) min = x1;           \
          if (x1 > max) max = x1;           \
          if (x2 < min) min = x2;           \
          if (x2 > max) max = x2;

#define AXISTEST_X01(a, b, fa, fb)                                      \
    p0 = (a * v0.y) - (b * v0.z);                                       \
    p2 = (a * v2.y) - (b * v2.z);                                       \
    if (p0 < p2) { min = p0; max = p2; } else { min = p2; max = p0; }   \
    rad = (fa * boxhalfsize.y) + (fb * boxhalfsize.z);                  \
    if ((min > rad) || (max < -rad)) return false;

#define AXISTEST_X2(a, b, fa, fb)                                       \
    p0 = (a * v0.y) - (b * v0.z);                                       \
    p1 = (a * v1.y) - (b * v1.z);                                       \
    if (p0 < p1) { min = p0; max = p1; } else { min = p1; max = p0;}    \
    rad = (fa * boxhalfsize.y) + (fb * boxhalfsize.z);                  \
    if ((min > rad) || (max < -rad)) return false;

#define AXISTEST_Y02(a, b, fa, fb)                                      \
    p0 = (-a * v0.x) + (b * v0.z);                                      \
    p2 = (-a * v2.x) + (b * v2.z);                                      \
    if (p0 < p2) { min = p0; max = p2; } else { min = p2; max = p0; }   \
    rad = (fa * boxhalfsize.x) + (fb * boxhalfsize.z);                  \
    if ((min > rad) || (max < -rad)) return false;

#define AXISTEST_Y1(a, b, fa, fb)                                       \
    p0 = (-a * v0.x) + (b * v0.z);                                      \
    p1 = (-a * v1.x) + (b * v1.z);                                      \
    if (p0 < p1) { min = p0; max = p1; } else { min = p1; max = p0; }   \
    rad = (fa * boxhalfsize.x) + (fb * boxhalfsize.z);                  \
    if ((min > rad) || (max < -rad)) return false;

#define AXISTEST_Z12(a, b, fa, fb)                                      \
    p1 = (a * v1.x) - (b * v1.y);                                       \
    p2 = (a * v2.x) - (b * v2.y);                                       \
    if (p2 < p1) { min = p2; max = p1; } else { min = p1; max = p2; }   \
    rad = (fa * boxhalfsize.x) + (fb * boxhalfsize.y);                  \
    if ((min > rad) || (max < -rad)) return false;

#define AXISTEST_Z0(a, b, fa, fb)                                       \
    p0 = (a * v0.x) - (b * v0.y);                                       \
    p1 = (a * v1.x) - (b * v1.y);                                       \
    if (p0 < p1) { min = p0; max = p1; } else { min = p1; max = p0; }   \
    rad = (fa * boxhalfsize.x) + (fb * boxhalfsize.y);                  \
    if ((min > rad) || (max < -rad)) return false;

bool plane_box_overlap(const point_t &normal, const point_t &vert, const point_t &maxbox)
{
    point_t vmin;
    point_t vmax;
    if (normal.x > 0.0f)
    {
        vmin.x = -maxbox.x - vert.x;
        vmax.x =  maxbox.x - vert.x;
    }
    else
    {
        vmin.x =  maxbox.x - vert.x;
        vmax.x = -maxbox.x - vert.x;
    }

    if (normal.y > 0.0f)
    {
        vmin.y = -maxbox.y - vert.y;
        vmax.y =  maxbox.y - vert.y;
    }
    else
    {
        vmin.y =  maxbox.y - vert.y;
        vmax.y = -maxbox.y - vert.y;
    }

    if (normal.z > 0.0f)
    {
        vmin.z = -maxbox.z - vert.z;
        vmax.z =  maxbox.z - vert.z;
    }
    else
    {
        vmin.z =  maxbox.z - vert.z;
        vmax.z = -maxbox.z - vert.z;
    }

    if (dot_product(normal, vmin) > 0.0f) 
    {
        return false;
    }

    if (dot_product(normal, vmax) >= 0.0f)
    {
        return true;
    }

    return false;
}

bool triangle_box_overlap(const point_t &boxcenter, const point_t &boxhalfsize, const point_t &triver0, const point_t &triver1, const point_t &triver2)
{
    const point_t v0(triver0 - boxcenter);
    const point_t v1(triver1 - boxcenter);
    const point_t v2(triver2 - boxcenter);

    /* Compute triangle edges */
    const point_t e0(v1 - v0);
    const point_t e1(v2 - v1);
    const point_t e2(v0 - v2);

    float min,max,p0,p1,p2,rad;
    float fex = std::fabs(e0.x);
    float fey = std::fabs(e0.y);
    float fez = std::fabs(e0.z);

    AXISTEST_X01(e0.z, e0.y, fez, fey);
    AXISTEST_Y02(e0.z, e0.x, fez, fex);
    AXISTEST_Z12(e0.y, e0.x, fey, fex);

    fex = std::fabs(e1.x);
    fey = std::fabs(e1.y);
    fez = std::fabs(e1.z);

    AXISTEST_X01(e1.z, e1.y, fez, fey);
    AXISTEST_Y02(e1.z, e1.x, fez, fex);
    AXISTEST_Z0(e1.y, e1.x, fey, fex);

    fex = std::fabs(e2.x);
    fey = std::fabs(e2.y);
    fez = std::fabs(e2.z);

    AXISTEST_X2(e2.z, e2.y, fez, fey);
    AXISTEST_Y1(e2.z, e2.x, fez, fex);
    AXISTEST_Z12(e2.y, e2.x, fey, fex);

    /* test in x-direction */
    FINDMINMAX(v0.x, v1.x, v2.x, min, max);
    if ((min > boxhalfsize.x) || (max < -boxhalfsize.x))
    {
        return false;
    }

    /* test in y-direction */
    FINDMINMAX(v0.y ,v1.y ,v2.y ,min, max);
    if ((min > boxhalfsize.y) || (max < -boxhalfsize.y))
    {
        return false;
    }

    /* test in z-direction */
    FINDMINMAX(v0.z ,v1.z ,v2.z ,min, max);
    if ((min > boxhalfsize.z) || (max < -boxhalfsize.z))
    {
        return false;
    }

    const point_t normal(cross_product(e0, e1));
    if (!plane_box_overlap(normal, v0, boxhalfsize))
    {
        return false;
    }

    return true;   /* box and triangle overlaps */
}

void volume::fill_outside_surface(const int i0, const int j0, const int k0, const int i1, const int j1, const int k1)
{
    const int neighbours[6][3] =   {{  1,  0,  0 },
                                    {  0,  1,  0 },
                                    {  0,  0,  1 },
                                    { -1,  0,  0 },
                                    {  0, -1,  0 },
                                    {  0,  0, -1 }};
    std::queue<point_ti<>> fifo; 
    for (int i = i0; i < i1; ++i)
    {
        for (int j = j0; j < j1; ++j)
        {
            for (int k = k0; k < k1; ++k)
            {
                if (get_voxel(i, j, k) == voxel_value_t::primitive_undefined)
                {
                    fifo.emplace(i, j, k);
                    get_voxel(i, j, k) = voxel_value_t::primitive_outside_surface;
                    ++_prim_outside_surface;
                    while (!fifo.empty())
                    {
                        const point_ti<> &current = fifo.front();
                        for (int h = 0; h < 6; ++h)
                        {
                            const int a = current.x + neighbours[h][0];
                            const int b = current.y + neighbours[h][1];
                            const int c = current.z + neighbours[h][2];
                            if ((a < 0) || (a >= _dim[0]) || (b < 0) || (b >= _dim[1]) || (c < 0) || (c >= _dim[2]))
                            {
                                continue;
                            }

                            voxel_value_t &v = get_voxel(a, b, c);
                            if (v == voxel_value_t::primitive_undefined)
                            {
                                v = voxel_value_t::primitive_outside_surface;
                                ++_prim_outside_surface;
                                fifo.push(point_ti<>(a, b, c));
                            }
                        }
                        fifo.pop();
                    }
                }
            }
        }
    }
}

void volume::fill_inside_surface()
{
    for (int i = 0; i < _dim[0]; ++i)
    {
        for (int j = 0; j < _dim[1]; ++j)
        {
            for (int k = 0; k < _dim[2]; ++k)
            {
                voxel_value_t &v = get_voxel(i, j, k);
                if (v == voxel_value_t::primitive_undefined)
                {
                    v = voxel_value_t::primitive_inside_surface;
                    ++_prim_inside_surface;
                }
            }
        }
    }
}

convex_mesh* volume::convert_to_convex_mesh(const voxel_value_t value) const
{
    auto *const mesh = new convex_mesh;
    for (int i = 0; i < _dim[0]; ++i)
    {
        for (int j = 0; j < _dim[1]; ++j)
        {
            for (int k = 0; k < _dim[2]; ++k)
            {
                const voxel_value_t &v = get_voxel(i, j, k);
                if (v == value)
                {
                    const point_t p0(point_t((i - 0.5f), (j - 0.5f), (k - 0.5f)) * _scale);
                    const point_t p1(point_t((i + 0.5f), (j - 0.5f), (k - 0.5f)) * _scale);
                    const point_t p2(point_t((i + 0.5f), (j + 0.5f), (k - 0.5f)) * _scale);
                    const point_t p3(point_t((i - 0.5f), (j + 0.5f), (k - 0.5f)) * _scale);
                    const point_t p4(point_t((i - 0.5f), (j - 0.5f), (k + 0.5f)) * _scale);
                    const point_t p5(point_t((i + 0.5f), (j - 0.5f), (k + 0.5f)) * _scale);
                    const point_t p6(point_t((i + 0.5f), (j + 0.5f), (k + 0.5f)) * _scale);
                    const point_t p7(point_t((i - 0.5f), (j + 0.5f), (k + 0.5f)) * _scale);
                    const int s = static_cast<int>(mesh->number_of_points());
                    mesh->add_point(p0 + _min_bb);
                    mesh->add_point(p1 + _min_bb);
                    mesh->add_point(p2 + _min_bb);
                    mesh->add_point(p3 + _min_bb);
                    mesh->add_point(p4 + _min_bb);
                    mesh->add_point(p5 + _min_bb);
                    mesh->add_point(p6 + _min_bb);
                    mesh->add_point(p7 + _min_bb);
                    mesh->add_triangle(s + 0, s + 2, s + 1);
                    mesh->add_triangle(s + 0, s + 3, s + 2);
                    mesh->add_triangle(s + 4, s + 5, s + 6);
                    mesh->add_triangle(s + 4, s + 6, s + 7);
                    mesh->add_triangle(s + 7, s + 6, s + 2);
                    mesh->add_triangle(s + 7, s + 2, s + 3);
                    mesh->add_triangle(s + 4, s + 1, s + 5);
                    mesh->add_triangle(s + 4, s + 0, s + 1);
                    mesh->add_triangle(s + 6, s + 5, s + 1);
                    mesh->add_triangle(s + 6, s + 1, s + 2);
                    mesh->add_triangle(s + 7, s + 0, s + 4);
                    mesh->add_triangle(s + 7, s + 3, s + 0);
                }
            }
        }
    }

    return mesh;
}

voxel_set* volume::convert_to_voxel_set() const
{
    auto *const vset = new voxel_set;
    vset->set_scale(_scale);
    vset->set_min_bb(_min_bb);
    vset->reserve(_prim_inside_surface + _prim_on_surface);
    for (int i = 0; i < _dim[0]; ++i)
    {
        for (int j = 0; j < _dim[1]; ++j)
        {
            for (int k = 0; k < _dim[2]; ++k)
            {
                const voxel_value_t &value = get_voxel(i, j, k);
                if ((value == voxel_value_t::primitive_inside_surface) || (value == voxel_value_t::primitive_on_surface))
                {
                    vset->add(voxel(point_ti<>(i, j, k), value));
                }
            }
        }
    }

    return vset;
}

tetrahedron_set* volume::convert_to_tetrahedron_set() const
{
    auto *const tset = new tetrahedron_set;
    tset->set_scale(_scale);
    tset->reserve(5 * (_prim_inside_surface + _prim_on_surface));
    for (int i = 0; i < _dim[0]; ++i)
    {
        for (int j = 0; j < _dim[1]; ++j)
        {
            for (int k = 0; k < _dim[2]; ++k)
            {
                const voxel_value_t &value = get_voxel(i, j, k);
                if ((value == voxel_value_t::primitive_inside_surface) || (value == voxel_value_t::primitive_on_surface))
                {
                    const point_t p1((point_t((i - 0.5f), (j - 0.5f), (k - 0.5f)) * _scale) + _min_bb);
                    const point_t p2((point_t((i + 0.5f), (j - 0.5f), (k - 0.5f)) * _scale) + _min_bb);
                    const point_t p3((point_t((i + 0.5f), (j + 0.5f), (k - 0.5f)) * _scale) + _min_bb);
                    const point_t p4((point_t((i - 0.5f), (j + 0.5f), (k - 0.5f)) * _scale) + _min_bb);
                    const point_t p5((point_t((i - 0.5f), (j - 0.5f), (k + 0.5f)) * _scale) + _min_bb);
                    const point_t p6((point_t((i + 0.5f), (j - 0.5f), (k + 0.5f)) * _scale) + _min_bb);
                    const point_t p7((point_t((i + 0.5f), (j + 0.5f), (k + 0.5f)) * _scale) + _min_bb);
                    const point_t p8((point_t((i - 0.5f), (j + 0.5f), (k + 0.5f)) * _scale) + _min_bb);
                    tset->add(tetrahedron(p2, p4, p7, p5, value));
                    tset->add(tetrahedron(p6, p2, p7, p5, value));
                    tset->add(tetrahedron(p3, p4, p7, p2, value));
                    tset->add(tetrahedron(p1, p4, p2, p5, value));
                    tset->add(tetrahedron(p8, p5, p7, p4, value));
                }
            }
        }
    }

    return tset;
}

void volume::compute_principal_axes(float (&rot)[3][3]) const
{
    /* Find center */
    int cnt = 0;
    point_t barycenter(0.0f, 0.0f, 0.0f);
    for (int i = 0; i < _dim[0]; ++i)
    {
        for (int j = 0; j < _dim[1]; ++j)
        {
            for (int k = 0; k < _dim[2]; ++k)
            {
                const voxel_value_t &value = get_voxel(i, j, k);
                if (value == voxel_value_t::primitive_inside_surface || value == voxel_value_t::primitive_on_surface)
                {
                    barycenter.x += i;
                    barycenter.y += j;
                    barycenter.z += k;
                    ++cnt;
                }
            }
        }
    }
    barycenter /= static_cast<float>(cnt);

    /* Run pca */
    float cov[3][3] = { { 0.0f, 0.0f, 0.0f },
                        { 0.0f, 0.0f, 0.0f },
                        { 0.0f, 0.0f, 0.0f } };
    for (int i = 0; i < _dim[0]; ++i)
    {
        for (int j = 0; j < _dim[1]; ++j)
        {
            for (int k = 0; k < _dim[2]; ++k)
            {
                const voxel_value_t &value = get_voxel(i, j, k);
                if (value == voxel_value_t::primitive_inside_surface || value == voxel_value_t::primitive_on_surface)
                {
                    const float x = i - barycenter.x;
                    const float y = j - barycenter.y;
                    const float z = k - barycenter.z;
                    cov[0][0] += x * x;
                    cov[1][1] += y * y;
                    cov[2][2] += z * z;
                    cov[0][1] += x * y;
                    cov[0][2] += x * z;
                    cov[1][2] += y * z;
                }
            }
        }
    }
    cov[1][0] = cov[0][1];
    cov[2][0] = cov[0][2];
    cov[2][1] = cov[1][2];
    float d[3][3];
    diagonalise(cov, rot, d);
}
} /* namespace raptor_convex_decomposition */
