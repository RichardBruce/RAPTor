/* Standard headers */
#include <cassert>
#include <queue>

/* Convex decomposition headers */
#include "dac_convex_hull.h"
#include "volume.h"


namespace raptor_convex_decomposition
{
const float epsilon = 0.00001f;

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
        vmin.x =  maxbox.z - vert.z;
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

void diagonalise(const float (&a)[3][3], float (&q)[3][3], float (&d)[3][3])
{
    // a must be a symmetric matrix.
    // returns q and d such that 
    // Diagonal matrix d = qt * a * q;  and  a = q*d*qt
    const int maxsteps = 24;
    float o[3], m[3];
    float qu [4] = { 0.0f, 0.0f, 0.0f, 1.0f };
    float jr[4];
    float aq[3][3];
    for (int i = 0; i < maxsteps; ++i)
    {
        // quat to matrix
        const float sqx = qu[0] * qu[0];
        const float sqy = qu[1] * qu[1];
        const float sqz = qu[2] * qu[2];
        const float sqw = qu[3] * qu[3];
        q[0][0]  = ( sqx - sqy - sqz + sqw);
        q[1][1]  = (-sqx + sqy - sqz + sqw);
        q[2][2]  = (-sqx - sqy + sqz + sqw);
        float tmp1     = qu[0] * qu[1];
        float tmp2     = qu[2] * qu[3];
        q[1][0]  = 2.0f * (tmp1 + tmp2);
        q[0][1]  = 2.0f * (tmp1 - tmp2);
        tmp1     = qu[0] * qu[2];
        tmp2     = qu[1] * qu[3];
        q[2][0]  = 2.0f * (tmp1 - tmp2);
        q[0][2]  = 2.0f * (tmp1 + tmp2);
        tmp1     = qu[1] * qu[2];
        tmp2     = qu[0] * qu[3];
        q[2][1]  = 2.0f * (tmp1 + tmp2);
        q[1][2]  = 2.0f * (tmp1 - tmp2);

        // aq = a * q
        aq[0][0] = (q[0][0] * a[0][0]) + (q[1][0] * a[0][1]) + (q[2][0] * a[0][2]);
        aq[0][1] = (q[0][1] * a[0][0]) + (q[1][1] * a[0][1]) + (q[2][1] * a[0][2]);
        aq[0][2] = (q[0][2] * a[0][0]) + (q[1][2] * a[0][1]) + (q[2][2] * a[0][2]);
        aq[1][0] = (q[0][0] * a[0][1]) + (q[1][0] * a[1][1]) + (q[2][0] * a[1][2]);
        aq[1][1] = (q[0][1] * a[0][1]) + (q[1][1] * a[1][1]) + (q[2][1] * a[1][2]);
        aq[1][2] = (q[0][2] * a[0][1]) + (q[1][2] * a[1][1]) + (q[2][2] * a[1][2]);
        aq[2][0] = (q[0][0] * a[0][2]) + (q[1][0] * a[1][2]) + (q[2][0] * a[2][2]);
        aq[2][1] = (q[0][1] * a[0][2]) + (q[1][1] * a[1][2]) + (q[2][1] * a[2][2]);
        aq[2][2] = (q[0][2] * a[0][2]) + (q[1][2] * a[1][2]) + (q[2][2] * a[2][2]);
        // d = Qt * aq
        d[0][0] = (aq[0][0] * q[0][0]) + (aq[1][0] * q[1][0]) + (aq[2][0] * q[2][0]); 
        d[0][1] = (aq[0][0] * q[0][1]) + (aq[1][0] * q[1][1]) + (aq[2][0] * q[2][1]); 
        d[0][2] = (aq[0][0] * q[0][2]) + (aq[1][0] * q[1][2]) + (aq[2][0] * q[2][2]); 
        d[1][0] = (aq[0][1] * q[0][0]) + (aq[1][1] * q[1][0]) + (aq[2][1] * q[2][0]); 
        d[1][1] = (aq[0][1] * q[0][1]) + (aq[1][1] * q[1][1]) + (aq[2][1] * q[2][1]); 
        d[1][2] = (aq[0][1] * q[0][2]) + (aq[1][1] * q[1][2]) + (aq[2][1] * q[2][2]); 
        d[2][0] = (aq[0][2] * q[0][0]) + (aq[1][2] * q[1][0]) + (aq[2][2] * q[2][0]); 
        d[2][1] = (aq[0][2] * q[0][1]) + (aq[1][2] * q[1][1]) + (aq[2][2] * q[2][1]); 
        d[2][2] = (aq[0][2] * q[0][2]) + (aq[1][2] * q[1][2]) + (aq[2][2] * q[2][2]);
        o[0]    = d[1][2];
        o[1]    = d[0][2];
        o[2]    = d[0][1];
        m[0]    = std::fabs(o[0]);
        m[1]    = std::fabs(o[1]);
        m[2]    = std::fabs(o[2]);

        const int k0    = (m[0] > m[1] && m[0] > m[2]) ? 0 : (m[1] > m[2]) ? 1 : 2;                 // index of largest element of offdiag
        const int k1    = (k0 + 1) % 3;
        const int k2    = (k0 + 2) % 3;
        if (o[k0] == 0.0f)
        {
            break;                                                                                  // diagonal already
        }

        float thet      = (d[k2][k2] - d[k1][k1]) / (2.0f * o[k0]);
        const float sgn = (thet > 0.0f) ? 1.0f : -1.0f;
        thet           *= sgn;                                                                              // make it positive
        const float t   = sgn / (thet + ((thet < 1.E6f) ? std::sqrt(thet * thet + 1.0f) : thet)) ;  // sign(t)/(|t|+sqrt(t^2+1))
        const float c   = 1.0f / std::sqrt(t * t + 1.0f);                                           //  c= 1/(t^2+1) , t=s/c 
        if (c == 1.0f)
        {
            break;                                                                                  // no room for improvement - reached machine precision.
        }
        jr[0 ]  = jr[1] = jr[2] = jr[3] = 0.0f;
        jr[k0]  = sgn * std::sqrt((1.0f - c) / 2.0f);                                               // using 1/2 angle identity sin(a/2) = sqrt((1-cos(a))/2)  
        jr[k0] *= -1.0f;                                                                            // since our quat-to-matrix convention was for v*m instead of m*v
        jr[3 ]  = std::sqrt(1.0f - jr[k0] * jr[k0]);
        if(jr[3] == 1.0f)
        {
            break;                                                                                  // reached limits of floating point precision
        }
        qu[0]    = ((qu[3] * jr[0]) + (qu[0] * jr[3]) + (qu[1] * jr[2]) - (qu[2] * jr[1]));
        qu[1]    = ((qu[3] * jr[1]) - (qu[0] * jr[2]) + (qu[1] * jr[3]) + (qu[2] * jr[0]));
        qu[2]    = ((qu[3] * jr[2]) + (qu[0] * jr[1]) - (qu[1] * jr[0]) + (qu[2] * jr[3]));
        qu[3]    = ((qu[3] * jr[3]) - (qu[0] * jr[0]) - (qu[1] * jr[1]) - (qu[2] * jr[2]));
        const float mq  = std::sqrt((qu[0] * qu[0]) + (qu[1] * qu[1]) + (qu[2] * qu[2]) + (qu[3] * qu[3]));
        qu[0]   /= mq;
        qu[1]   /= mq;
        qu[2]   /= mq;
        qu[3]   /= mq;
    }
}

float primitive_set::compute_preferred_cutting_direction(point_t *const dir)
{
    const float ex = eigen_value(axis_t::x_axis);
    const float ey = eigen_value(axis_t::y_axis);
    const float ez = eigen_value(axis_t::z_axis);
    const float vx = (ey - ez) * (ey - ez);
    const float vy = (ex - ez) * (ex - ez);
    const float vz = (ex - ey) * (ex - ey);
    if ((vx < vy) && (vx < vz))
    {
        const float e = (ey * ey) + (ez * ez);
        (*dir) = point_t(1.0f, 0.0f, 0.0f);
        return (e == 0.0f) ? 0.0f : (1.0f - vx / e);
    }
    else if ((vy < vx) && (vy < vz))
    {
        const float e = (ex * ex) + (ez * ez);
        (*dir) = point_t(0.0f, 1.0f, 0.0f);
        return (e == 0.0f) ? 0.0f : (1.0f - vy / e);
    }
    else
    {
        const float e = (ex * ex) + (ey * ey);
        (*dir) = point_t(0.0f, 0.0f, 1.0f);
        return (e == 0.0f) ? 0.0f : (1.0f - vz / e);
    }
}

void voxel_set::compute_bounding_box()
{
    if (_voxels.empty())
    {
        return;
    }

    _min_bb_voxels = _voxels[0].coord;
    _max_bb_voxels = _voxels[0].coord;
    point_t bary(_voxels[0].coord);
    for (int i = 1; i < static_cast<int>(_voxels.size()); ++i)
    {
        bary += static_cast<point_t>(_voxels[i].coord);
        _min_bb_voxels = min(_min_bb_voxels, _voxels[i].coord);
        _max_bb_voxels = max(_max_bb_voxels, _voxels[i].coord);
    }
    
    bary /= static_cast<float>(_voxels.size());
    _barycenter = static_cast<point_ti>(bary + 0.5f);
}

void voxel_set::compute_convex_hull(convex_mesh *const mesh, const int sampling, const int cluster_size) const
{
    if (_voxels.empty())
    {
        return;
    }

    int p = 0;
    int s = 0;
    std::vector<point_t> cpoints;
    std::vector<point_t> points(std::ceil(cluster_size / 8.0f) * 8);
    while (p < static_cast<int>(_voxels.size()))
    {
        int qu = 0;
        while ((qu < static_cast<int>(points.size())) && (p < static_cast<int>(_voxels.size())))
        {
            if (_voxels[p].loc == voxel_value_t::primitive_on_surface)
            {
                ++s;
                if (s == sampling)
                {
                    s = 0;
                    get_points(_voxels[p], &points[qu]);
                    qu += 8;
                }
            }
            ++p;
        }
        
        points.resize(qu);
        const dac_convex_hull ch(points);
        cpoints.insert(cpoints.end(), ch.vertices().begin(), ch.vertices().end());
    }

    /* Conpute hull */
    const dac_convex_hull ch(cpoints);

    /* Add points to mesh */
    for (const auto &v : ch.vertices())
    {
        mesh->add_point(v);
    }

    /* Add triangulated faces to mesh */
    for (const auto &face : ch.faces())
    {
        const int a = face[0];
        for (int i = 2; i < static_cast<int>(face.size()); ++i)
        {
            mesh->add_triangle(a, face[i - 1], face[i]);
        }
    }
}

bool voxel_set::add(const voxel &v)
{
    if (v.loc == voxel_value_t::primitive_on_surface)
    {
        ++_prim_on_surface;
    }
    else if (v.loc == voxel_value_t::primitive_inside_surface)
    {
        ++_prim_inside_surface;
    }

    _voxels.push_back(v);
    return true;
}

void voxel_set::get_points(const voxel &v, point_t *const pts) const 
{
    const int i = v.coord.x;
    const int j = v.coord.y;
    const int k = v.coord.z;
    pts[0] = (point_t((i - 0.5f), (j - 0.5f), (k - 0.5f)) * _scale) + _min_bb;
    pts[1] = (point_t((i + 0.5f), (j - 0.5f), (k - 0.5f)) * _scale) + _min_bb;
    pts[2] = (point_t((i + 0.5f), (j + 0.5f), (k - 0.5f)) * _scale) + _min_bb;
    pts[3] = (point_t((i - 0.5f), (j + 0.5f), (k - 0.5f)) * _scale) + _min_bb;
    pts[4] = (point_t((i - 0.5f), (j - 0.5f), (k + 0.5f)) * _scale) + _min_bb;
    pts[5] = (point_t((i + 0.5f), (j - 0.5f), (k + 0.5f)) * _scale) + _min_bb;
    pts[6] = (point_t((i + 0.5f), (j + 0.5f), (k + 0.5f)) * _scale) + _min_bb;
    pts[7] = (point_t((i - 0.5f), (j + 0.5f), (k + 0.5f)) * _scale) + _min_bb;
}

void voxel_set::intersect(const plane &p, std::vector<point_t> *const pos_pts, std::vector<point_t> *const neg_pts, const int sampling) const
{
    if (_voxels.empty())
    {
        return;
    }

    int sp = 0;
    int sn = 0;
    point_t pts[8];
    for (const auto &v : _voxels)
    {
        const point_t pt(get_point(v));
        const float d = p.distance(pt);
        if (d >= 0.0f)
        {
            if (d <= _scale)
            {
                get_points(v, pts);
                pos_pts->insert(pos_pts->end(), &pts[0], &pts[8]);
            }
            else
            {
                if (++sp == sampling)
                {
                    get_points(v, pts);
                    pos_pts->insert(pos_pts->end(), &pts[0], &pts[8]);
                    sp = 0;
                }
            }
        }
        else
        {
            if (-d <= _scale)
            {
                get_points(v, pts);
                neg_pts->insert(neg_pts->end(), &pts[0], &pts[8]);
            }
            else
            {
                if (++sn == sampling)
                {
                    get_points(v, pts);
                    neg_pts->insert(neg_pts->end(), &pts[0], &pts[8]);
                    sn = 0;
                }
            }
        }
    }
}

voxel_set* voxel_set::select_on_surface() const
{
    if (_voxels.empty())
    {
        return nullptr;
    }

    auto *const surf = new voxel_set;
    surf->_min_bb               = _min_bb;
    surf->_scale                = _scale;
    surf->_unit_volume          = _unit_volume;
    surf->_prim_on_surface      = 0;
    surf->_prim_inside_surface  = 0;
    for (const auto &v : _voxels)
    {
        if (v.loc == voxel_value_t::primitive_on_surface)
        {
            surf->_voxels.push_back(v);
            ++surf->_prim_on_surface;
        }
    }

    return surf;
}

void voxel_set::compute_cut_volumes(const plane &p, float *const pos_volume, float *const neg_volume) const
{
    if (_voxels.empty())
    {
        (*neg_volume) = 0.0f;
        (*pos_volume) = 0.0f;
        return;
    }

    /* Classify voxels */
    int pos_voxels = 0;
    for (const auto &v : _voxels)
    {
        const float d = p.distance(get_point(v));
        pos_voxels += (d >= 0.0f);
    }

    /* Multiply by unit size */
    const int neg_voxels = _voxels.size() - pos_voxels;
    (*pos_volume) = _unit_volume * pos_voxels;
    (*neg_volume) = _unit_volume * neg_voxels;
}

void voxel_set::cut(const plane &p, primitive_set **const pos_part, primitive_set **const neg_part) const
{
    if (_voxels.empty())
    {
        return;
    }

    /* Check for if we need to new */
    if ((*pos_part) == nullptr)
    {
        *pos_part = new voxel_set;
    }

    if ((*neg_part) == nullptr)
    {
        *neg_part = new voxel_set;
    }

    voxel_set *const pos_cut = static_cast<voxel_set *>(*pos_part);
    voxel_set *const neg_cut = static_cast<voxel_set *>(*neg_part);
    pos_cut->_voxels.clear();
    neg_cut->_voxels.clear();
    pos_cut->_voxels.reserve(_voxels.size());
    neg_cut->_voxels.reserve(_voxels.size());
    neg_cut->_min_bb                = _min_bb;
    pos_cut->_min_bb                = _min_bb;
    neg_cut->_scale                 = _scale;
    pos_cut->_scale                 = _scale;
    neg_cut->_unit_volume           = _unit_volume;
    pos_cut->_unit_volume           = _unit_volume;
    neg_cut->_prim_on_surface       = 0;
    pos_cut->_prim_on_surface       = 0;
    neg_cut->_prim_inside_surface   = 0;
    pos_cut->_prim_inside_surface   = 0;

    for (auto v : _voxels)
    {
        const point_t pt(get_point(v));
        const float d = p.distance(pt);
        if (d >= 0.0f)
        {
            if ((v.loc == voxel_value_t::primitive_on_surface) || (d <= _scale))
            {
                v.loc = voxel_value_t::primitive_on_surface;
                pos_cut->_voxels.push_back(v);
                ++pos_cut->_prim_on_surface;
            }
            else
            {
                pos_cut->_voxels.push_back(v);
                ++pos_cut->_prim_inside_surface;
            }
        }
        else
        {
            if ((v.loc == voxel_value_t::primitive_on_surface) || (-d <= _scale))
            {
                v.loc = voxel_value_t::primitive_on_surface;
                neg_cut->_voxels.push_back(v);
                ++neg_cut->_prim_on_surface;
            }
            else
            {
                neg_cut->_voxels.push_back(v);
                ++neg_cut->_prim_inside_surface;
            }
        }
    }
}

void voxel_set::convert(convex_mesh *const mesh, const voxel_value_t value) const
{
    if (_voxels.empty())
    {
        return;
    }

    point_t pts[8];
    for (const auto &v : _voxels)
    {
        if (v.loc == value)
        {
            /* add points */
            get_points(v, pts);
            const int s = static_cast<int>(mesh->number_of_points());
            for (int k = 0; k < 8; ++k)
            {
                mesh->add_point(pts[k]);
            }

            /* add triangles */
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

void voxel_set::compute_axes_aligned_clipping_planes(std::vector<plane> *const planes, const int downsampling) const
{
    const point_ti &min_pt = get_min_bb_voxels();
    const point_ti &max_pt = get_max_bb_voxels();
    for (int i = min_pt.x; i <= max_pt.x; i += downsampling)
    {
        const point_t pt(get_point(point_t(i + 0.5f, 0.0f, 0.0f)));
        planes->emplace_back(point_t(1.0f, 0.0f, 0.0f), pt.x, axis_t::x_axis);
    }

    for (int i = min_pt.y; i <= max_pt.y; i += downsampling)
    {
        const point_t pt(get_point(point_t(0.0f, i + 0.5f, 0.0f)));
        planes->emplace_back(point_t(0.0f, 1.0f, 0.0f), pt.y, axis_t::y_axis);
    }

    for (int i = min_pt.z; i <= max_pt.z; i += downsampling)
    {
        const point_t pt(get_point(point_t(0.0f, 0.0f, i + 0.5f)));
        planes->emplace_back(point_t(0.0f, 0.0f, 1.0f), pt.z, axis_t::z_axis);
    }
}

void voxel_set::refine_axes_aligned_clipping_planes(std::vector<plane> *const planes, const plane &best, const int downsampling, const int index) const
{
    const point_ti &min_pt = get_min_bb_voxels();
    const point_ti &max_pt = get_max_bb_voxels();
    if (best.major_axis == axis_t::x_axis)
    {
        for (int i = std::max(min_pt.x, index - downsampling); i <= std::min(max_pt.x, index + downsampling); ++i)
        {
            const point_t pt(get_point(point_t(i + 0.5f, 0.0f, 0.0f)));
            planes->emplace_back(plane(point_t(1.0f, 0.0f, 0.0f), pt.x, axis_t::x_axis));
        }
    }
    else if (best.major_axis == axis_t::y_axis)
    {
        for (int i = std::max(min_pt.y, index - downsampling); i <= std::min(max_pt.y, index + downsampling); ++i)
        {
            const point_t pt(get_point(point_t(0.0f, i + 0.5f, 0.0f)));
            planes->emplace_back(plane(point_t(0.0f, 1.0f, 0.0f), pt.y, axis_t::y_axis));
        }
    }
    else
    {
        for (int i = std::max(min_pt.z, index - downsampling); i <= std::min(max_pt.z, index + downsampling); ++i)
        {
            const point_t pt(get_point(point_t(0.0f, 0.0f, i + 0.5f)));
            planes->emplace_back(plane(point_t(0.0f, 0.0f, 1.0f), pt.z, axis_t::z_axis));
        }
    }
}

void voxel_set::compute_principal_axes()
{
    if (_voxels.empty())
    {
        return;
    }

    float cov[3][3] = { { 0.0f, 0.0f, 0.0f },
                        { 0.0f, 0.0f, 0.0f },
                        { 0.0f, 0.0f, 0.0f } };
    for (const auto &v : _voxels)
    {
        const point_t diff(v.coord - _barycenter);
        cov[0][0] += diff.x * diff.x;
        cov[1][1] += diff.y * diff.y;
        cov[2][2] += diff.z * diff.z;
        cov[0][1] += diff.x * diff.y;
        cov[0][2] += diff.x * diff.z;
        cov[1][2] += diff.y * diff.z;
    }
    cov[0][0] /= _voxels.size();
    cov[1][1] /= _voxels.size();
    cov[2][2] /= _voxels.size();
    cov[0][1] /= _voxels.size();
    cov[0][2] /= _voxels.size();
    cov[1][2] /= _voxels.size();
    cov[1][0]  = cov[0][1];
    cov[2][0]  = cov[0][2];
    cov[2][1]  = cov[1][2];
    diagonalise(cov, _q, _d);
}

void tetrahedron_set::compute_bounding_box()
{
    if (_tetrahedra.empty())
    {
        return;
    }

    _min_bb     = _tetrahedra[0].pts[0];
    _max_bb     = _tetrahedra[0].pts[0];
    _barycenter = point_t(0.0f, 0.0f, 0.0f);
    for (const auto &t : _tetrahedra)
    {
        for (int i = 0; i < 4; ++i)
        {
            _min_bb = min(_min_bb, t.pts[i]);
            _max_bb = max(_max_bb, t.pts[i]);
            _barycenter += t.pts[i];
        }
    }
    _barycenter /= static_cast<float>(4 * _tetrahedra.size());
}

void tetrahedron_set::compute_convex_hull(convex_mesh *const mesh, const int sampling, const int cluster_size) const
{
    if (_tetrahedra.empty())
    {
        return;
    }

    int p = 0;
    int s = 0;
    std::vector<point_t> cpoints;
    std::vector<point_t> points(std::ceil(cluster_size / 5.0f) * 5);
    while (p < static_cast<int>(_tetrahedra.size()))
    {
        int qu = 0;
        while ((qu < static_cast<int>(points.size())) && (p < static_cast<int>(_tetrahedra.size())))
        {
            if (_tetrahedra[p].loc == voxel_value_t::primitive_on_surface)
            {
                ++s;
                if (s == sampling)
                {
                    s = 0;
                    points[qu++] = _tetrahedra[p].pts[0];
                    points[qu++] = _tetrahedra[p].pts[1];
                    points[qu++] = _tetrahedra[p].pts[2];
                    points[qu++] = _tetrahedra[p].pts[3];
                    assert((_tetrahedra[p].pts[0] + epsilon) >= _min_bb);
                    assert((_tetrahedra[p].pts[1] + epsilon) >= _min_bb);
                    assert((_tetrahedra[p].pts[2] + epsilon) >= _min_bb);
                    assert((_tetrahedra[p].pts[3] + epsilon) >= _min_bb);
                }
            }
            ++p;
        }
        
        points.resize(qu);
        const dac_convex_hull ch(points);
        cpoints.insert(cpoints.end(), ch.vertices().begin(), ch.vertices().end());
    }

    /* Conpute hull */
    const dac_convex_hull ch(cpoints);

    /* Add points to mesh */
    for (const auto &v : ch.vertices())
    {
        mesh->add_point(v);
    }

    /* Add triangulated faces to mesh */
    for (const auto &face : ch.faces())
    {
        const int a = face[0];
        for (int i = 2; i < static_cast<int>(face.size()); ++i)
        {
            mesh->add_triangle(a, face[i - 1], face[i]);
        }
    }
}

bool tetrahedron_set::add(tetrahedron t)
{
    const float v = trapezium_volume(t.pts[0], t.pts[1], t.pts[2], t.pts[3]);
    if (std::fabs(v) < epsilon)
    {
        return false;
    }
    else if (v < 0.0f)
    {
        point_t tmp = t.pts[0];
        t.pts[0]   = t.pts[1];
        t.pts[1]   = tmp;
    }

    if (t.loc == voxel_value_t::primitive_on_surface)
    {
        ++_prim_on_surface;
    }
    else if (t.loc == voxel_value_t::primitive_inside_surface)
    {
        ++_prim_inside_surface;
    }

    _tetrahedra.push_back(t);
    return true;
}

void tetrahedron_set::add_clipped_tetrahedra(const point_t (&pts) [10], const int size)
{
    assert((size == 4) || (size == 6) || !"Error: Wrong number of points to create a clipped tetrahedron");

    if (size == 4)
    {
        tetrahedron t(pts[0], pts[1], pts[2], pts[3], voxel_value_t::primitive_on_surface);
        add(t);
    }
    else if (size == 6)
    {
        const int tet_faces[4][3] = { { 0, 1, 2 }, { 2, 1, 3 }, { 3, 1, 0 }, { 3, 0, 2 } };        
        const int tet[15][4] = { { 2, 3, 4, 5 }, { 1, 3, 4, 5 }, { 1, 2, 4, 5 }, { 1, 2, 3, 5 }, { 1, 2, 3, 4 },
                                 { 0, 3, 4, 5 }, { 0, 2, 4, 5 }, { 0, 2, 3, 5 }, { 0, 2, 3, 4 }, { 0, 1, 4, 5 },
                                 { 0, 1, 3, 5 }, { 0, 1, 3, 4 }, { 0, 1, 2, 5 }, { 0, 1, 2, 4 }, { 0, 1, 2, 3 } };
        const int rem[15][2]  = { { 0, 1 }, { 0, 2 }, { 0, 3 }, { 0, 4 }, { 0, 5 }, 
                                  { 1, 2 }, { 1, 3 }, { 1, 4 }, { 1, 5 }, { 2, 3 }, 
                                  { 2, 4 }, { 2, 5 }, { 3, 4 }, { 3, 5 }, { 4, 5 } };
        float max_volume    = 0.0f;
        int  h0             = -1;
        tetrahedron tetrahedron0;
        tetrahedron0.loc = voxel_value_t::primitive_on_surface;
        for (int h = 0; h < 15; ++h)
        {
            const float v = trapezium_volume(pts[tet[h][0]], pts[tet[h][1]], pts[tet[h][2]], pts[tet[h][3]]);
            if (v > max_volume)
            {
                h0 = h;
                tetrahedron0.pts[0] = pts[tet[h][0]];
                tetrahedron0.pts[1] = pts[tet[h][1]];
                tetrahedron0.pts[2] = pts[tet[h][2]];
                tetrahedron0.pts[3] = pts[tet[h][3]];
                max_volume          = v;
            }
            else if (-v > max_volume)
            {
                h0                  = h;
                tetrahedron0.pts[0] = pts[tet[h][1]];
                tetrahedron0.pts[1] = pts[tet[h][0]];
                tetrahedron0.pts[2] = pts[tet[h][2]];
                tetrahedron0.pts[3] = pts[tet[h][3]];
                max_volume          = -v;
            }
        }

        if (h0 == -1)
        {
            return;
        }

        if (!add(tetrahedron0))
        {
            return;
        }

        int a0  = rem[h0][0];
        int a1  = rem[h0][1];
        int h1  = -1;
        max_volume  = 0.0f;
        tetrahedron tetrahedron1;
        tetrahedron1.loc = voxel_value_t::primitive_on_surface;
        for (int h = 0; h < 4; ++h)
        {
            const float v = trapezium_volume(pts[a0], tetrahedron0.pts[tet_faces[h][0]], tetrahedron0.pts[tet_faces[h][1]], tetrahedron0.pts[tet_faces[h][2]]);
            if (v > max_volume)
            {
                h1                  = h;
                tetrahedron1.pts[0] = pts[a0];
                tetrahedron1.pts[1] = tetrahedron0.pts[tet_faces[h][0]];
                tetrahedron1.pts[2] = tetrahedron0.pts[tet_faces[h][1]];
                tetrahedron1.pts[3] = tetrahedron0.pts[tet_faces[h][2]];
                max_volume          = v;
            }
        }

        if (h1 != -1)
        {
            add(tetrahedron1);
        }

        max_volume  = 0.0f;
        int h2      = -1;
        tetrahedron tetrahedron2;
        tetrahedron2.loc = voxel_value_t::primitive_on_surface;
        for (int h = 0; h < 4; ++h)
        {
            if (h == h1)
            {
                continue;
            }

            const float v = trapezium_volume(pts[a0], tetrahedron0.pts[tet_faces[h][0]], tetrahedron0.pts[tet_faces[h][1]], tetrahedron0.pts[tet_faces[h][2]]);
            if (v > max_volume)
            {
                h2                  = h;
                tetrahedron2.pts[0] = pts[a1];
                tetrahedron2.pts[1] = tetrahedron0.pts[tet_faces[h][0]];
                tetrahedron2.pts[2] = tetrahedron0.pts[tet_faces[h][1]];
                tetrahedron2.pts[3] = tetrahedron0.pts[tet_faces[h][2]];
                max_volume          = v;
            }
        }

        if (h1 != -1)
        {
            for (int h = 0; h < 4; ++h)
            {
                if (h == 1)
                {
                    continue;
                }

                const float v = trapezium_volume(pts[a1], tetrahedron1.pts[tet_faces[h][0]], tetrahedron1.pts[tet_faces[h][1]], tetrahedron1.pts[tet_faces[h][2]]);
                if (v > max_volume)
                {
                    h2                  = h;
                    tetrahedron2.pts[0] = pts[a1];
                    tetrahedron2.pts[1] = tetrahedron1.pts[tet_faces[h][0]];
                    tetrahedron2.pts[2] = tetrahedron1.pts[tet_faces[h][1]];
                    tetrahedron2.pts[3] = tetrahedron1.pts[tet_faces[h][2]];
                    max_volume          = v;
                }
            }
        }

        if (h2 != -1)
        {
            add(tetrahedron2);
        }
    }
}

void tetrahedron_set::intersect(const plane &p, std::vector<point_t> *const pos_pts, std::vector<point_t> *const neg_pts, const int sampling) const
{
    if (_tetrahedra.empty())
    {
        return;
    }

    int sp = 0;
    int sn = 0;
    for (const auto &t : _tetrahedra)
    {
        int npos = 0;
        int nneg = 0;
        float cen_dist = 0.0f;
        for (int i = 0; i < 4; ++i)
        {
            const float dist = p.distance(t.pts[i]);
            cen_dist += dist;
            if (dist > 0.0f)
            {
                ++npos;
            }
            else
            {
                ++nneg;
            }
        }

        if (cen_dist >= 0.0f)
        {
            if (nneg == 0)
            {
                pos_pts->insert(pos_pts->end(), &t.pts[0], &t.pts[4]);
            }
            else
            {
                if (++sp == sampling)
                {
                    pos_pts->insert(pos_pts->end(), &t.pts[0], &t.pts[4]);
                    sp = 0;
                }
            }
        }
        else
        {
            if (npos == 0)
            {
                neg_pts->insert(neg_pts->end(), &t.pts[0], &t.pts[4]);
            }
            else
            {
                if (++sn == sampling)
                {
                    neg_pts->insert(neg_pts->end(), &t.pts[0], &t.pts[4]);
                    sn = 0;
                }
            }
        }
    }
}

void tetrahedron_set::compute_cut_volumes(const plane &p, float *const pos_volume, float *const neg_volume) const
{
    if (_tetrahedra.empty())
    {
        (*neg_volume) = 0.0f;
        (*pos_volume) = 0.0f;
        return;
    }

    /* Classify tetrahedron and sum volume */
    float pos_vol = 0.0f;
    float neg_vol = 0.0f;
    for (const auto &t : _tetrahedra)
    {
        float cen_dist = 0.0f;
        for (int i = 0; i < 4; ++i)
        {
            cen_dist += p.distance(t.pts[i]);
        }

        const float vol = std::fabs(trapezium_volume(t.pts[0], t.pts[1], t.pts[2], t.pts[3]));
        if (cen_dist >= 0.0f)
        {
            pos_vol += vol;
        }
        else
        {
            neg_vol += vol;
        }
    }

    (*pos_volume) = pos_vol * (1.0f / 6.0f);
    (*neg_volume) = neg_vol * (1.0f / 6.0f);
}

tetrahedron_set* tetrahedron_set::select_on_surface() const
{
    if (_tetrahedra.empty())
    {
        return nullptr;
    }

    auto *const surf = new tetrahedron_set;
    surf->_barycenter            = _barycenter;
    surf->_min_bb                = _min_bb;
    surf->_max_bb                = _max_bb;
    surf->_scale                 = _scale;
    surf->_prim_on_surface       = 0;
    surf->_prim_inside_surface   = 0;
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            surf->_q[i][j] = _q[i][j];
            surf->_d[i][j] = _d[i][j];
        }
    }

    for (const auto &t : _tetrahedra)
    {
        if (t.loc == voxel_value_t::primitive_on_surface)
        {
            surf->_tetrahedra.push_back(t);
            ++surf->_prim_on_surface;
        }
    }

    return surf;
}

void tetrahedron_set::cut(const plane &p, primitive_set **const pos_part, primitive_set **const neg_part) const
{
    if (_tetrahedra.empty())
    {
        return;
    }

    /* Check for if we need to new */
    if ((*pos_part) == nullptr)
    {
        *pos_part = new tetrahedron_set;
    }

    if ((*neg_part) == nullptr)
    {
        *neg_part = new tetrahedron_set;
    }

    tetrahedron_set * const pos_cut = static_cast<tetrahedron_set *>(*pos_part);
    tetrahedron_set * const neg_cut = static_cast<tetrahedron_set *>(*neg_part);
    pos_cut->_tetrahedra.clear();
    neg_cut->_tetrahedra.clear();
    pos_cut->_tetrahedra.reserve(_tetrahedra.size());
    neg_cut->_tetrahedra.reserve(_tetrahedra.size());
    neg_cut->_min_bb                = _min_bb;
    pos_cut->_min_bb                = _min_bb;
    neg_cut->_max_bb                = _max_bb;
    pos_cut->_max_bb                = _max_bb;
    neg_cut->_barycenter            = _barycenter;
    pos_cut->_barycenter            = _barycenter;
    neg_cut->_scale                 = _scale;
    pos_cut->_scale                 = _scale;
    neg_cut->_prim_on_surface       = 0;
    pos_cut->_prim_on_surface       = 0;
    neg_cut->_prim_inside_surface   = 0;
    pos_cut->_prim_inside_surface   = 0;
    memcpy(neg_cut->_q, _q, 9 * sizeof(float));
    memcpy(neg_cut->_d, _d, 9 * sizeof(float));
    memcpy(pos_cut->_q, _q, 9 * sizeof(float));
    memcpy(pos_cut->_d, _d, 9 * sizeof(float));

    int sign[4];
    point_t pos_pts[10];
    point_t neg_pts[10];
    const int edges [6][2]= {{0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 3}, {2, 3}};
    for (const auto &t : _tetrahedra)
    {
        /* Classify points of the tetrahedron as above or below the split */
        int npos = 0;
        int nneg = 0;
        for (int i = 0; i < 4; ++i)
        {
            const float dist = p.distance(t.pts[i]);
            if (dist > 0.0f)
            {
                sign[i]         = 1;
                pos_pts[npos]   = t.pts[i];
                ++npos;
            }
            else
            {
                sign[i]         = -1;
                neg_pts[nneg]   = t.pts[i];
                ++nneg;
            }
        }

        /* Totally inside positive cut */
        if (npos == 4)
        {
            pos_cut->add(t);
        }
        /* Totally inside negative sut */
        else if (nneg == 4)
        {
            neg_cut->add(t);
        }
        /* Split */
        else
        {
            for (int j = 0; j < 6; ++j)
            {
                if ((sign[edges[j][0]] * sign[edges[j][1]]) == -1)
                {
                    const point_t p0(t.pts[edges[j][0]]);
                    const point_t p1(t.pts[edges[j][1]]);
                    const float delta = dot_product((p1 - p0), p.n);
                    const float alpha = p.distance(p1) / delta;
                    assert(alpha >= 0.0f && alpha <= 1.0f);

                    const point_t s((alpha * p0) + ((1.0f - alpha) * p1));
                    assert((s.x + epsilon) >= _min_bb.x);
                    assert((s.y + epsilon) >= _min_bb.y);
                    assert((s.z + epsilon) >= _min_bb.z);

                    pos_pts[npos++] = s;
                    neg_pts[nneg++] = s;
                }
            }

            neg_cut->add_clipped_tetrahedra(neg_pts, nneg);
            pos_cut->add_clipped_tetrahedra(pos_pts, npos);
        }
    }
}

void tetrahedron_set::convert(convex_mesh *const mesh, const voxel_value_t value) const
{
    if (_tetrahedra.empty())
    {
        return;
    }

    for (const auto &t : _tetrahedra)
    {
        if (t.loc == value)
        {
            const int s = mesh->number_of_points();
            mesh->add_point(t.pts[0]);
            mesh->add_point(t.pts[1]);
            mesh->add_point(t.pts[2]);
            mesh->add_point(t.pts[3]);
            mesh->add_triangle(s + 0, s + 1, s + 2);
            mesh->add_triangle(s + 2, s + 1, s + 3);
            mesh->add_triangle(s + 3, s + 1, s + 0);
            mesh->add_triangle(s + 3, s + 0, s + 2);
        }
    }
}

void tetrahedron_set::compute_axes_aligned_clipping_planes(std::vector<plane> *const planes, const int downsampling) const
{
    const point_t   &min_pt = get_min_bb();
    const point_t   &max_pt = get_max_bb();
    const float     scale   = get_scale();
    for (int i = 0; i <= static_cast<int>((max_pt.x - min_pt.x) / scale + 0.5f); i += downsampling)
    {
        const float x = min_pt.x + (scale * i);
        planes->emplace_back(plane(point_t(1.0f, 0.0f, 0.0f), x, axis_t::x_axis));
    }

    for (int j = 0; j <= static_cast<int>((max_pt.y - min_pt.y) / scale + 0.5f); j += downsampling)
    {
        const float y = min_pt.y + (scale * j);
        planes->emplace_back(plane(point_t(0.0f, 1.0f, 0.0f), y, axis_t::y_axis));
    }

    for (int k = 0; k <= static_cast<int>((max_pt.z - min_pt.z) / scale + 0.5f); k += downsampling)
    {
        const float z = min_pt.z + (scale * k);
        planes->emplace_back(plane(point_t(0.0f, 0.0f, 1.0f), z, axis_t::z_axis));
    }
}

void tetrahedron_set::refine_axes_aligned_clipping_planes(std::vector<plane> *const planes, const plane &best, const int downsampling, const int index) const
{
    const point_t   &min_pt = get_min_bb();
    const point_t   &max_pt = get_max_bb();
    const float     scale   = get_scale();
    if (best.major_axis == axis_t::x_axis)
    {
        const int i0 = std::max(0, index - downsampling);
        const int i1 = static_cast<int>(std::min((max_pt.x - min_pt.x) / scale + 0.5f, static_cast<float>(index + downsampling)));
        for (int i = i0; i <= i1; ++i)
        {
            const float x = min_pt.x + (scale * i);
            planes->emplace_back(plane(point_t(1.0f, 0.0f, 0.0f), x, axis_t::x_axis));
        }
    }
    else if (best.major_axis == axis_t::y_axis)
    {
        const int j0 = std::max(0, index - downsampling);
        const int j1 = static_cast<int>(std::min((max_pt.y - min_pt.y) / scale + 0.5f, static_cast<float>(index + downsampling)));
        for (int j = j0; j <= j1; ++j)
        {
            const float y = min_pt.y + (scale * j);
            planes->emplace_back(plane(point_t(0.0f, 1.0f, 0.0f), y, axis_t::y_axis));
        }
    }
    else
    {
        const int k0 = std::max(0, index - downsampling);
        const int k1 = static_cast<int>(std::min((max_pt.z - min_pt.z) / scale + 0.5f, static_cast<float>(index + downsampling)));
        for (int k = k0; k <= k1; ++k)
        {
            const float z = min_pt.z + (scale * k);
            planes->emplace_back(plane(point_t(0.0f, 0.0f, 1.0f), z, axis_t::z_axis));
        }
    }
}

float tetrahedron_set::compute_volume() const
{
    if (_tetrahedra.empty())
    {
        return 0.0f;
    }

    float volume = 0.0f;
    for (const auto &t : _tetrahedra)
    {
        volume += std::fabs(trapezium_volume(t.pts[0], t.pts[1], t.pts[2], t.pts[3]));
    }

    return volume * (1.0f / 6.0f);
}

float tetrahedron_set::max_volume_error() const
{
    if (_tetrahedra.empty())
    {
        return 0.0f;
    }

    float volume = 0.0f;
    for (const auto &t : _tetrahedra)
    {
        if (t.loc == voxel_value_t::primitive_on_surface)
        {
            volume += std::fabs(trapezium_volume(t.pts[0], t.pts[1], t.pts[2], t.pts[3]));
        }
    }

    return volume * (1.0f / 6.0f);
}

void tetrahedron_set::compute_principal_axes()
{
    if (_tetrahedra.empty())
    {
        return;
    }

    float cov[3][3] = { { 0.0f, 0.0f, 0.0f },
                        { 0.0f, 0.0f, 0.0f },
                        { 0.0f, 0.0f, 0.0f } };
    for (const auto &t : _tetrahedra)
    {
        for (int i = 0; i < 4; ++i)
        {
            const point_t diff(t.pts[i] - _barycenter);
            cov[0][0] += diff.x * diff.x;
            cov[1][1] += diff.y * diff.y;
            cov[2][2] += diff.z * diff.z;
            cov[0][1] += diff.x * diff.y;
            cov[0][2] += diff.x * diff.z;
            cov[1][2] += diff.y * diff.z;
        }
    }
    const float n = _tetrahedra.size() * 4.0f;
    cov[0][0] /= n;
    cov[1][1] /= n;
    cov[2][2] /= n;
    cov[0][1] /= n;
    cov[0][2] /= n;
    cov[1][2] /= n;
    cov[1][0]  = cov[0][1];
    cov[2][0]  = cov[0][2];
    cov[2][1]  = cov[1][2];
    diagonalise(cov, _q, _d);
}

void tetrahedron_set::align_to_principal_axes()
{
    if (_tetrahedra.empty())
    {
        return;
    }

    /* Rotate all points */
    for (auto &t : _tetrahedra)
    {
        for (int i = 0; i < 4; ++i)
        {
            const point_t diff(t.pts[i] - _barycenter);
            t.pts[i].x = (_q[0][0] * diff.x) + (_q[1][0] * diff.y) + (_q[2][0] * diff.z) + _barycenter.x;
            t.pts[i].y = (_q[0][1] * diff.x) + (_q[1][1] * diff.y) + (_q[2][1] * diff.z) + _barycenter.y;
            t.pts[i].z = (_q[0][2] * diff.x) + (_q[1][2] * diff.y) + (_q[2][2] * diff.z) + _barycenter.z;
        }
    }

    compute_bounding_box();
}

void tetrahedron_set::revert_align_to_principal_axes()
{
    if (_tetrahedra.empty())
    {
        return;
    }

    /* Inverse rotate all points */
    for (auto &t : _tetrahedra)
    {
        for (int i = 0; i < 4; ++i)
        {
            const point_t diff(t.pts[i] - _barycenter);
            t.pts[i].x = (_q[0][0] * diff.x) + (_q[0][1] * diff.y) + (_q[0][2] * diff.z) + _barycenter.x;
            t.pts[i].y = (_q[1][0] * diff.x) + (_q[1][1] * diff.y) + (_q[1][2] * diff.z) + _barycenter.y;
            t.pts[i].z = (_q[2][0] * diff.x) + (_q[2][1] * diff.y) + (_q[2][2] * diff.z) + _barycenter.z;
        }
    }

    compute_bounding_box();
}

void volume::fill_outside_surface(const int i0, const int j0, const int k0, const int i1, const int j1, const int k1)
{
    const int neighbours[6][3] =   {{  1,  0,  0 },
                                    {  0,  1,  0 },
                                    {  0,  0,  1 },
                                    { -1,  0,  0 },
                                    {  0, -1,  0 },
                                    {  0,  0, -1 }};
    std::queue<point_ti> fifo; 
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
                        const point_ti &current = fifo.front();
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
                                fifo.push(point_ti(a, b, c));
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
                    vset->add(voxel(point_ti(i, j, k), value));
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
