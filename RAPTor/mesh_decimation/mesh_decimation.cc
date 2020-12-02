/* Standard headers */

/* Boost headers */

/* Common headers */
#include "sort.h"

/* Mesh decimation headers */
#include "mesh_decimation.h"


namespace raptor_mesh_decimation
{
bool mesh_decimation::can_update(std::vector<bool> &removed, const vertex *v, const point_t<double> &p, const vertex *to)
{
    for (int i = 0; i < v->links(); ++i)
    {
        /* Find face */
        const auto &l = _links[v->first_link() + i];
        face &f = _faces[l.face()];
        if (f.removed())
        {
            continue;
        }

        /* Check if an edge has collapsed to a point */
        const vertex* v0 = f.vertices((l.vertex() + 1) % 3);
        const vertex* v1 = f.vertices((l.vertex() + 2) % 3);
        if ((v0 == to) || (v1 == to))
        {
            removed[i] = true;
            continue;
        }

        /* Check for face flipped */
        removed[i] = false;
        const point_t<double> d0(v0->position() - p);
        const point_t<double> d1(v1->position() - p);
        const point_t<double> n(normalise(cross_product(d0, d1)));
        if (dot_product(n, f.normal()) <= 0.1)
        {
            return false;
        }
    }

    return true;
}

void mesh_decimation::update_mesh(std::vector<bool> &removed, int &cleaned, const vertex *v, vertex *to)
{
    point_t<double> p;
    for (int i = 0; i < v->links(); ++i)
    {
        /* Find face */
        const link &l = _links[v->first_link() + i];
        face &f = _faces[l.face()];
        if (f.removed())
        {
            continue;
        }

        /* Remove collapsed faces */
        if (removed[i])
        {
            f.remove();
            ++cleaned;
            continue;
        }

        /* Update vertex and cost */
        f.vertices(l.vertex(), to);
        f.face_error(_options.max_cumulative_cost);

        /* Push link for new vertex */
        _links.push_back(l);
    }
}

void mesh_decimation::build_links()
{
    /* Clean link location */
    for (auto &v : _vertices)
    {
        v.reset_link();
    }

    /* Count faces that each vertex is in */
    for (const auto &f : _faces)
    {
        for (int i = 0; i < 3; ++i)
        {
            f.vertices(i)->add_links();
        }
    }

    /* Prefix sum into each vertex start location */
    int first_link = 0;
    for (auto &v : _vertices)
    {
        v.first_link(first_link);
        first_link += v.links();
        v.links(0);
    }

    /* Update links */
    _links.resize(_faces.size() * 3);
    for (size_t i = 0; i < _faces.size(); ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            vertex *v = _faces[i].vertices(j);
            _links[v->first_link() + v->links()] = link(i, j);
            v->add_links();
        }
    }
}

void mesh_decimation::decimate()
{
    /* Initialise face costs */
    for (auto &f : _faces)
    {
        f.face_error(_options.max_cumulative_cost);
    }

    int cleaned = 0;
    bool progress = true;
    std::vector<bool> removed0;
    std::vector<bool> removed1;
    std::vector<double> sortarry(_faces.size());    
    std::vector<double> percentiles(_faces.size());    
    while (progress)
    {
        /* Build percentiles */
        int live_faces = 0;
        for (size_t i = 0; i < _faces.size(); ++i)
        {
            if (!_faces[i].removed() && (_faces[i].cost() < _options.max_merge_cost))
            {
                percentiles[live_faces++] = _faces[i].cost();
            }
        }
        percentiles.resize(live_faces);
        radix_sort(percentiles.data(), sortarry.data(), live_faces);

        /* Decimate based on the percentile cost curve */
        progress = false;
        const double items_per_perc = live_faces / 100.0;
        double max_perc = std::min(live_faces - 1.0, std::max(_options.percentile_pass_max * items_per_perc, 1.0));
        for (double percetile_idx = std::max(items_per_perc, 1.0); percetile_idx <= max_perc; percetile_idx += std::max(_options.percentile_pass_stride * items_per_perc, 1.0))
        {
            const double threshold = percentiles[std::floor(percetile_idx)];
            for (auto &f : _faces)
            {
                if (f.removed() || (f.cost() > threshold))
                {
                    continue;
                }

                /* Check if we can upate */
                point_t<double> p;
                vertex *v0 = f.vertices(f.index());
                vertex *v1 = f.vertices((f.index() + 1) % 3);
                v0->vertex_error(v1, _options.max_cumulative_cost, p);
                removed0.resize(v0->links());
                removed1.resize(v1->links());
                if (!can_update(removed0, v0, p, v1) || !can_update(removed1, v1, p, v0))
                {
                    max_perc = std::min(static_cast<double>(live_faces - 1), max_perc + 1.0);
                    continue;
                }

                /* Update vertex and its fan */
                v0->update(v1, p, f.cost());

                const int first_link = _links.size();
                update_mesh(removed0, cleaned, v0, v0);
                update_mesh(removed1, cleaned, v1, v0);
                const int links = _links.size() - first_link;
                if (links <= v0->links())
                {
                    if (links)
                    {
                        memcpy(&_links[v0->first_link()], &_links[first_link], links * sizeof(link));
                        _links.resize(first_link);
                    }
                }
                else
                {
                    v0->first_link(first_link);
                }
                v0->links(links);

                /* Done */
                if (++_iteration >= _options.max_iterations)
                {
                    return;
                }

                progress = true;
            }

            /* Check if its face cleaning time */
            if (cleaned > (_options.face_clean_perc * _faces.size()))
            {
                cleaned = 0;
                _faces.erase(std::remove_if(_faces.begin(), _faces.end(), [](const face &f) { return f.removed(); }), _faces.end());
                build_links();
            }
        }
    }
}

mesh_decimation::mesh_decimation(const std::vector<point_t<>> &points, const std::vector<point_ti<>> &triangles, const mesh_decimation_options &options) : _options(options), _cost(0.0), _iteration(0)
{
    /* Find the model size */
    point_t<> max_bound(points[0]);
    point_t<> min_bound(points[0]);
    for (const auto &p : points)
    {
        max_bound = max(max_bound, p);
        min_bound = min(min_bound, p);
    }
    const point_t<> offset((max_bound + min_bound) * 0.5f);
    const point_t<> scale((max_bound - min_bound) * 0.5f);
    const double max_scale_inv = 1.0 / std::max(scale.x, std::max(scale.y, scale.z));
    const point_t<> scale_inv(max_scale_inv, max_scale_inv, max_scale_inv);

    /* Build rescaled vertices and faces */
    _vertices.reserve(points.size());
    for (const auto &p : points)
    {
        const point_t<> local((p - offset) * scale_inv);
        _vertices.emplace_back(point_t<double>(local));
    }

    _faces.reserve(triangles.size());
    for (const auto t : triangles)
    {
        _faces.emplace_back(&_vertices[t.x], &_vertices[t.y], &_vertices[t.z]);
    }

    /* Build link from vertex to face */
    build_links();

    /* Find boundary vertices */
    std::vector<std::pair<vertex*, int>> vertex_id;
    for (auto &v : _vertices)
    {
        vertex_id.clear();
        for (int i = v.first_link(); i < (v.first_link() + v.links()); ++i)
        {
            const face &f = _faces[_links[i].face()];
            for (int j = 0; j < 3; ++j)
            {
                const auto vit = std::find_if(vertex_id.begin(), vertex_id.end(), [id = f.vertices(j)](const auto &p) { return p.first == id; });
                if (vit == vertex_id.end())
                {
                    vertex_id.emplace_back(f.vertices(j), 0);
                }
                else
                {
                    ++(vit->second);
                }
            }
        }

        for (const auto &p : vertex_id)
        {
            if (p.second == 0)
            {
                p.first->boundary(true);
            }
        }
    }

    /* Decimate */
    _options.absolute_costs(_vertices.size());
    decimate();

    /* Compact output */
    for (auto &v : _vertices)
    {
        v.links(0);
    }

    /* Find live face and mark their vertices */
    int face_idx = 0;
    for (const auto &f : _faces)
    {
        if (!f.removed())
        {
            _faces[face_idx++] = f;
            for (int i = 0; i < 3; ++i)
            {
                f.vertices(i)->links(1);
            }
        }
    }
    _faces.resize(face_idx);

    /* Move vertices and remember where to */
    /* This destorys the vertex decimation information for faster moving, buts its ok, we're done if we sum the cost as we go */
    int vertex_idx = 0;
    for (auto &v : _vertices)
    {
        if (v.links() > 0)
        {
            _cost += v.cost();
            v.first_link(vertex_idx);
            _vertices[vertex_idx++].position(v.position());
        }
    }

    /* Update face indexes. Done decimating so dont rebuild links */
    for (auto &f : _faces)
    {
        for (int i = 0; i < 3; ++i)
        {
            f.vertices(i, &_vertices[f.vertices(i)->first_link()]);
        }
    }
    _vertices.resize(vertex_idx);
}

void mesh_decimation::compact_output(std::vector<point_t<>> &vertices, std::vector<point_ti<>> &triangles)
{
    vertices.reserve(_vertices.size());
    for (const auto &v : _vertices)
    {
        vertices.emplace_back(v.position().x, v.position().y, v.position().z);
    }
    
    triangles.reserve(_faces.size());
    for (const auto &f : _faces)
    {
        triangles.emplace_back(std::distance(&_vertices[0], f.vertices(0)), std::distance(&_vertices[0], f.vertices(1)), std::distance(&_vertices[0], f.vertices(2)));
    }
}
} /* namespace raptor_mesh_decimation */
