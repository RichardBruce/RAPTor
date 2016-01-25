#include "voxel_set.h"

namespace raptor_convex_decomposition
{
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
    _barycenter = static_cast<point_ti<>>(bary + 0.5f);
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
    const point_ti<> &min_pt = get_min_bb_voxels();
    const point_ti<> &max_pt = get_max_bb_voxels();
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
    const point_ti<> &min_pt = get_min_bb_voxels();
    const point_ti<> &max_pt = get_max_bb_voxels();
    if (best.major_axis == axis_t::x_axis)
    {
        for (int i = std::max(min_pt.x, min_pt.x + index - downsampling); i <= std::min(max_pt.x, min_pt.x + index + downsampling); ++i)
        {
            const point_t pt(get_point(point_t(i + 0.5f, 0.0f, 0.0f)));
            planes->emplace_back(plane(point_t(1.0f, 0.0f, 0.0f), pt.x, axis_t::x_axis));
        }
    }
    else if (best.major_axis == axis_t::y_axis)
    {
        for (int i = std::max(min_pt.y, min_pt.y + index - downsampling); i <= std::min(max_pt.y, min_pt.y + index + downsampling); ++i)
        {
            const point_t pt(get_point(point_t(0.0f, i + 0.5f, 0.0f)));
            planes->emplace_back(plane(point_t(0.0f, 1.0f, 0.0f), pt.y, axis_t::y_axis));
        }
    }
    else
    {
        for (int i = std::max(min_pt.z, min_pt.z + index - downsampling); i <= std::min(max_pt.z, min_pt.z + index + downsampling); ++i)
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
}; /* namespace raptor_convex_decomposition */