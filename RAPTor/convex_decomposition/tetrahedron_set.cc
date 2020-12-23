#include "tetrahedron_set.h"

namespace raptor_convex_decomposition
{
const float epsilon = 0.00001f;

void tetrahedron_set::compute_bounding_box()
{
    if (_tetrahedra.empty())
    {
        return;
    }

    _min_bb     = _tetrahedra[0].pts[0];
    _max_bb     = _tetrahedra[0].pts[0];
    _barycenter = point_t<>(0.0f, 0.0f, 0.0f);
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
    std::vector<point_t<>> cpoints;
    std::vector<point_t<>> points(std::ceil(cluster_size / 5.0f) * 5);
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
    const float v = tetrahedron_volume(t.pts[0], t.pts[1], t.pts[2], t.pts[3]);
    if (v == 0.0f)
    {
        return false;
    }
    else if (v < 0.0f)
    {
        std::swap(t.pts[0], t.pts[1]);
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

void tetrahedron_set::add_clipped_tetrahedra(const point_t<> (&pts) [10], const int size)
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
            const float v = tetrahedron_volume(pts[tet[h][0]], pts[tet[h][1]], pts[tet[h][2]], pts[tet[h][3]]);
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
            const float v = tetrahedron_volume(pts[a0], tetrahedron0.pts[tet_faces[h][0]], tetrahedron0.pts[tet_faces[h][1]], tetrahedron0.pts[tet_faces[h][2]]);
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

            const float v = tetrahedron_volume(pts[a0], tetrahedron0.pts[tet_faces[h][0]], tetrahedron0.pts[tet_faces[h][1]], tetrahedron0.pts[tet_faces[h][2]]);
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

                const float v = tetrahedron_volume(pts[a1], tetrahedron1.pts[tet_faces[h][0]], tetrahedron1.pts[tet_faces[h][1]], tetrahedron1.pts[tet_faces[h][2]]);
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

void tetrahedron_set::intersect(const plane &p, std::vector<point_t<>> *const pos_pts, std::vector<point_t<>> *const neg_pts, const int sampling) const
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

        const float vol = std::fabs(tetrahedron_volume(t.pts[0], t.pts[1], t.pts[2], t.pts[3]));
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
    point_t<> pos_pts[10];
    point_t<> neg_pts[10];
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
                    const point_t<> p0(t.pts[edges[j][0]]);
                    const point_t<> p1(t.pts[edges[j][1]]);
                    const float delta = dot_product((p1 - p0), p.n);
                    const float alpha = p.distance(p1) / delta;
                    assert(alpha >= 0.0f && alpha <= 1.0f);

                    const point_t<> s((alpha * p0) + ((1.0f - alpha) * p1));
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
    const point_t<>   &min_pt = get_min_bb();
    const point_t<>   &max_pt = get_max_bb();
    const float     scale   = get_scale();
    for (int i = 0; i <= static_cast<int>((max_pt.x - min_pt.x) / scale + 0.5f); i += downsampling)
    {
        const float x = min_pt.x + (scale * i);
        planes->emplace_back(plane(point_t<>(1.0f, 0.0f, 0.0f), x, axis_t::x_axis));
    }

    for (int j = 0; j <= static_cast<int>((max_pt.y - min_pt.y) / scale + 0.5f); j += downsampling)
    {
        const float y = min_pt.y + (scale * j);
        planes->emplace_back(plane(point_t<>(0.0f, 1.0f, 0.0f), y, axis_t::y_axis));
    }

    for (int k = 0; k <= static_cast<int>((max_pt.z - min_pt.z) / scale + 0.5f); k += downsampling)
    {
        const float z = min_pt.z + (scale * k);
        planes->emplace_back(plane(point_t<>(0.0f, 0.0f, 1.0f), z, axis_t::z_axis));
    }
}

void tetrahedron_set::refine_axes_aligned_clipping_planes(std::vector<plane> *const planes, const plane &best, const int downsampling, const int index) const
{
    const point_t<>   &min_pt = get_min_bb();
    const point_t<>   &max_pt = get_max_bb();
    const float     scale   = get_scale();
    if (best.major_axis == axis_t::x_axis)
    {
        const int i0 = std::max(0, index - downsampling);
        const int i1 = static_cast<int>(std::min((max_pt.x - min_pt.x) / scale + 0.5f, static_cast<float>(index + downsampling)));
        for (int i = i0; i <= i1; ++i)
        {
            const float x = min_pt.x + (scale * i);
            planes->emplace_back(plane(point_t<>(1.0f, 0.0f, 0.0f), x, axis_t::x_axis));
        }
    }
    else if (best.major_axis == axis_t::y_axis)
    {
        const int j0 = std::max(0, index - downsampling);
        const int j1 = static_cast<int>(std::min((max_pt.y - min_pt.y) / scale + 0.5f, static_cast<float>(index + downsampling)));
        for (int j = j0; j <= j1; ++j)
        {
            const float y = min_pt.y + (scale * j);
            planes->emplace_back(plane(point_t<>(0.0f, 1.0f, 0.0f), y, axis_t::y_axis));
        }
    }
    else
    {
        const int k0 = std::max(0, index - downsampling);
        const int k1 = static_cast<int>(std::min((max_pt.z - min_pt.z) / scale + 0.5f, static_cast<float>(index + downsampling)));
        for (int k = k0; k <= k1; ++k)
        {
            const float z = min_pt.z + (scale * k);
            planes->emplace_back(plane(point_t<>(0.0f, 0.0f, 1.0f), z, axis_t::z_axis));
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
        volume += std::fabs(tetrahedron_volume(t.pts[0], t.pts[1], t.pts[2], t.pts[3]));
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
            volume += std::fabs(tetrahedron_volume(t.pts[0], t.pts[1], t.pts[2], t.pts[3]));
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
            const point_t<> diff(t.pts[i] - _barycenter);
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
            const point_t<> diff(t.pts[i] - _barycenter);
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
            const point_t<> diff(t.pts[i] - _barycenter);
            t.pts[i].x = (_q[0][0] * diff.x) + (_q[0][1] * diff.y) + (_q[0][2] * diff.z) + _barycenter.x;
            t.pts[i].y = (_q[1][0] * diff.x) + (_q[1][1] * diff.y) + (_q[1][2] * diff.z) + _barycenter.y;
            t.pts[i].z = (_q[2][0] * diff.x) + (_q[2][1] * diff.y) + (_q[2][2] * diff.z) + _barycenter.z;
        }
    }

    compute_bounding_box();
}
}; /* namespace raptor_convex_decomposition */
