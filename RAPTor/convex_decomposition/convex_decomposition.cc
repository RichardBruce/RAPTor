/* Standard headers */
#include <fstream>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <limits>

/* Convex decomposition headers */
#include "convex_decomposition.h"
#include "dac_convex_hull.h"
#include "incremental_convex_hull.h"
#include "simd.h"


namespace raptor_convex_decomposition
{
inline float compute_concavity(const float volume, const float ch_volume, const float volume_0)
{
    return std::fabs(ch_volume - volume) / volume_0;
}

int convex_decomposition::compute_best_clipping_plane(const std::unique_ptr<primitive_set> &pset, const float volume, const std::vector<plane> &planes, const point_t<> &cut_dir, const float alpha, const float beta, const int dn_samp, plane *const best_plane)
{
    std::unique_ptr<convex_mesh             []> chs(new convex_mesh[2]);
    std::unique_ptr<std::vector<point_t<>>  []> pts(new std::vector<point_t<>>[2]);
    std::unique_ptr<primitive_set> on_surf(pset->select_on_surface());

    std::unique_ptr<primitive_set *[]> psets(nullptr);
    if (!_params.approx_hulls)
    {
        psets.reset(new primitive_set *[2]);
        psets[0] = nullptr;
        psets[1] = nullptr;
    }

    int index       = 0;
    int best_i      = -1;
    int best_index  = 0;
    float min_cost  = std::numeric_limits<float>::max();
    for (int i = 0; i < static_cast<int>(planes.size()); ++i)
    {
        /* Compute axis index */
        const plane &p = planes[i];
        if ((i > 0) && (p.major_axis != planes[i - 1].major_axis))
        {
            index = 0;
        }

        /* Compute convex-hulls */
        convex_mesh &left_hull = chs[0];
        convex_mesh &right_hull = chs[1];
        right_hull.clear();
        left_hull.clear();
        if (_params.approx_hulls)
        {
            std::vector<point_t<>> &left_pts  = pts[0];
            std::vector<point_t<>> &right_pts = pts[1];
            right_pts.clear();
            left_pts.clear();
            on_surf->intersect(p, &right_pts, &left_pts, dn_samp * 32);
            pset->convex_hull().cut(&right_pts, &left_pts, p.n, p.p);
            right_hull.compute_convex_hull(right_pts);
            left_hull.compute_convex_hull(left_pts);
        }
        else
        {
            on_surf->cut(p, &psets[0], &psets[1]);
            psets[0]->compute_convex_hull(&right_hull, dn_samp);
            psets[1]->compute_convex_hull(&left_hull, dn_samp);
        }

        /* Compute clipped volumes */
        float left_vol  = 0.0f;
        float right_vol = 0.0f;
        pset->compute_cut_volumes(p, &right_vol, &left_vol);
        const float left_conc   = compute_concavity(left_vol, left_hull.volume(), _volume_hull);
        const float right_conc  = compute_concavity(right_vol, right_hull.volume(), _volume_hull);
        const float concavity   = (left_conc + right_conc);

        /* Compute cost */
        const float balance     = alpha * std::fabs(left_vol - right_vol) / _volume_hull;
        const float symmetry    = beta * dot_product(cut_dir, p.n);
        const float cost        = concavity + balance + symmetry;
        if ((cost < min_cost) || ((cost == min_cost) && (i < best_i)))
        {
            (*best_plane)   = p;
            best_index      = index;
            min_cost        = cost;
            best_i          = i;
        }

        ++index;
    }

    if (psets != nullptr)
    {
        for (int i = 0; i < 2; ++i)
        {
            delete psets[i];
        }
    }

    return best_index;
}

void convex_decomposition::compute_acd()
{
    std::vector<std::unique_ptr<primitive_set>> parts;
    std::vector<std::unique_ptr<primitive_set>> input_parts;
    std::vector<std::unique_ptr<primitive_set>> tmp;
    input_parts.push_back(std::move(_pset));
    std::vector<plane> planes;
    int depth = 0;
    bool first_iter = true;
    while ((depth++ < _params.depth) && !input_parts.empty())
    {
        for (auto &pset : input_parts)
        {
            const float volume = pset->compute_volume();
            pset->compute_bounding_box();
            pset->compute_principal_axes();
            if (_params.pca)
            {
                pset->align_to_principal_axes();
            }

            pset->compute_convex_hull(&pset->convex_hull());
            const float ch_volume = std::fabs(pset->convex_hull().volume());
            if (first_iter)
            {
                first_iter = false;
                _volume_hull = ch_volume;
            }

            const float concavity = compute_concavity(volume, ch_volume, _volume_hull);
            const float error     = 1.01f * pset->max_volume_error() / _volume_hull;
            if ((concavity > _params.concavity) && (concavity > error)) 
            {
                planes.clear();
                point_t<> cut_dir;
                const float w = pset->compute_preferred_cutting_direction(&cut_dir);
                pset->compute_axes_aligned_clipping_planes(&planes, _params.plane_down_sampling);

                plane best_plane;
                const int best_index = compute_best_clipping_plane(pset, volume, planes, cut_dir, concavity * _params.alpha, w * concavity * _params.beta, _params.hull_down_sampling, &best_plane);
                if ((_params.plane_down_sampling > 1) || (_params.hull_down_sampling > 1))
                {
                    planes.clear();
                    pset->refine_axes_aligned_clipping_planes(&planes, best_plane, _params.plane_down_sampling, best_index * _params.plane_down_sampling);
                    compute_best_clipping_plane(pset, volume, planes, cut_dir, concavity * _params.alpha, w * concavity * _params.beta, 1, &best_plane);
                }

                primitive_set *left     = nullptr;
                primitive_set *right    = nullptr;
                pset->cut(best_plane, &right, &left);
                tmp.emplace_back(left);
                tmp.emplace_back(right);
                if (_params.pca)
                {
                    right->revert_align_to_principal_axes();
                    left->revert_align_to_principal_axes();
                }
            }
            else
            {
                if (_params.pca)
                {
                    pset->revert_align_to_principal_axes();
                }
                parts.emplace_back(std::move(pset));
            }
        }

        input_parts.swap(tmp);
        tmp.clear();
    }
    
    /* Compute convex hull and revert rotation */
    _convex_hulls.clear();
    parts.insert(parts.end(), std::make_move_iterator(input_parts.begin()), std::make_move_iterator(input_parts.end()));
    for (int i = 0; i < static_cast<int>(parts.size()); ++i)
    {
        _convex_hulls.emplace_back(new convex_mesh);
        parts[i]->compute_convex_hull(_convex_hulls[i].get());
        for (auto &pt : _convex_hulls[i]->points())
        {
            pt[0] = (_rot[0][0] * pt.x) + (_rot[0][1] * pt.y) + (_rot[0][2] * pt.z) + _barycenter[0];
            pt[1] = (_rot[1][0] * pt.x) + (_rot[1][1] * pt.y) + (_rot[1][2] * pt.z) + _barycenter[1];
            pt[2] = (_rot[2][0] * pt.x) + (_rot[2][1] * pt.y) + (_rot[2][2] * pt.z) + _barycenter[2];
        }
    }
    parts.clear();
}

void compute_convex_hull(const std::unique_ptr<convex_mesh> &ch1, const std::unique_ptr<convex_mesh> &ch2, std::vector<point_t<>> *const pts, convex_mesh *const combine_hull)
{
    pts->clear();
    pts->insert(pts->end(), ch1->points().begin(), ch1->points().end());
    pts->insert(pts->end(), ch2->points().begin(), ch2->points().end());

    const dac_convex_hull ch(*pts);

    /* Add points to mesh */
    combine_hull->clear();
    for (const auto &v : ch.vertices())
    {
        combine_hull->add_point(v);
    }

    /* Add triangulated faces to mesh */
    for (const auto &face : ch.faces())
    {
        const int a = face[0];
        for (int i = 2; i < static_cast<int>(face.size()); ++i)
        {
            combine_hull->add_triangle(a, face[i - 1], face[i]);
        }
    }
}

void convex_decomposition::merge_convex_hulls()
{
    if (_convex_hulls.size() < 2)
    {
        return;
    }

    /* Populate the cost matrix */
    int idx = 0;
    convex_mesh combine_hull;
    std::vector<point_t<>> pts;
    std::vector<float> cost_matrix(((_convex_hulls.size() * _convex_hulls.size()) - _convex_hulls.size()) >> 1);
    for (int p1 = 1; p1 < static_cast<int>(_convex_hulls.size()); ++p1)
    {
        const float volume1 = _convex_hulls[p1]->volume();
        for (int p2 = 0; p2 < p1; ++p2)
        {
            compute_convex_hull(_convex_hulls[p1], _convex_hulls[p2], &pts, &combine_hull);
            cost_matrix[idx++] = compute_concavity(volume1 + _convex_hulls[p2]->volume(), combine_hull.volume(), _volume_hull);
        }
    }

    /* Until we cant merge below the maximum cost */
    int cost_end = _convex_hulls.size();
    const float threshold = _params.gamma;
    while (true)
    {
        /* Search for lowest cost */
        float lowest_cost   = std::numeric_limits<float>::max();
        const long addr     = min_element(cost_matrix.data(), &lowest_cost, 0, cost_matrix.size());
        const int addr_i    = (static_cast<int>(std::sqrt(1 + (8 * addr))) - 1) >> 1;
        const int lowest_i  = addr_i + 1;
        const int lowest_j  = addr - ((addr_i * (addr_i + 1)) >> 1);
        assert(lowest_i >= 0);
        assert(lowest_j >= 0);
        assert(lowest_i < cost_end);
        assert(lowest_j < cost_end);

        /* Check if we should merge these hulls */
        if (lowest_cost >= threshold)
        {
            break;
        }

        /* Make the lowest cost row and column into a new hull */
        convex_mesh *const cch = new convex_mesh;
        compute_convex_hull(_convex_hulls[lowest_i], _convex_hulls[lowest_j], &pts, cch);
        _convex_hulls[lowest_i].swap(_convex_hulls.back());
        _convex_hulls[lowest_j].reset(cch);
        _convex_hulls.pop_back();
        cost_end = cost_end - 1;

        /* Calculate costs versus the new hull */
        int row_idx = ((lowest_j - 1) * lowest_j) >> 1;
        const float volume1 = _convex_hulls[lowest_j]->volume();
        for (int i = 0; i < lowest_j; ++i)
        {
            compute_convex_hull(_convex_hulls[lowest_j], _convex_hulls[i], &pts, &combine_hull);
            cost_matrix[row_idx++] = compute_concavity(volume1 + _convex_hulls[i]->volume(), combine_hull.volume(), _volume_hull);
        }

        row_idx += lowest_j;
        for (int i = lowest_j + 1; i < cost_end; ++i)
        {
            compute_convex_hull(_convex_hulls[lowest_j], _convex_hulls[i], &pts, &combine_hull);
            cost_matrix[row_idx] = compute_concavity(volume1 + _convex_hulls[i]->volume(), combine_hull.volume(), _volume_hull);
            row_idx += i;
            assert(row_idx >= 0);
        }

        /* Move the top column in to replace its space */
        const int erase_idx = ((cost_end - 1) * cost_end) >> 1;
        if (lowest_i < cost_end)
        {
            row_idx = (addr_i * lowest_i) >> 1;
            int top_row = erase_idx;
            for (int i = 0; i < lowest_i; ++i)
            {
                if (i != lowest_j)
                {
                    cost_matrix[row_idx] = cost_matrix[top_row];
                }
                ++row_idx;
                ++top_row;
            }

            ++top_row;
            row_idx += lowest_i;
            for (int i = lowest_i + 1; i < (cost_end + 1); ++i)
            {
                cost_matrix[row_idx] = cost_matrix[top_row++];
                row_idx += i;
                assert(row_idx >= 0);
            }
        }
        cost_matrix.resize(erase_idx);
    }
}

void convex_decomposition::simplify_convex_hulls()
{
    if (_params.max_vertices_per_hull < 4)
    {
        return;
    }

    const float min_volume = _volume_hull * _params.min_volume_per_hull;
    for (int i = 0; i < static_cast<int>(_convex_hulls.size()); ++i)
    {
        incremental_convex_hull ic_hull(_convex_hulls[i]->points());
        ic_hull.add_points(_convex_hulls[i]->points());
        ic_hull.process(_params.max_vertices_per_hull, min_volume);

        _convex_hulls[i]->clear();
        tm_mesh &mesh = ic_hull.mesh();
        mesh.points_and_triangles(&_convex_hulls[i]->points(), &_convex_hulls[i]->triangles());

        ic_hull.clear();
    }
}
}
