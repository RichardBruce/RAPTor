/* Standard headers */

/* Boost headers */

/* Common headers */

/* Mesh decimation headers */
#include "mesh_decimation.h"


namespace raptor_mesh_decimation
{
std::priority_queue<mesh_decimation::queue_entry> mesh_decimation::decimation_costs()
{
    /* Depth first search to build vertex merge costs */
    std::priority_queue<mesh_decimation::queue_entry> costs;
    for (auto &e : _edges)
    {
        if (!e.removed() && !e.complex())
        {
            e.recalculate(false);
            costs.emplace(&e, e.merge_cost());
        }
    }

    return costs;
}

void mesh_decimation::next_cost(std::priority_queue<mesh_decimation::queue_entry> *const costs)
{
    costs->pop();
    if (costs->empty())
    {
        (*costs) = decimation_costs();
    }
}

bool mesh_decimation::can_merge_fan(const edge *const start, const edge *const end, vertex *const around, const point_t &vm)
{
    const edge *step = start;
    do
    {
        /* Check for triangles that would flip */
        const face *r = step->right_face(around);
        if (r->flipped(around, vm))
        {
            return true;
        }

        step = r->next_edge(step);
    } while (step != end);

    return false;
}

std::pair<face*, edge*> mesh_decimation::remove_edge(edge *merge, vertex *v0, vertex *v1)
{
    /* Get triangles next to merge */
    std::cout << "Removing duplicate edge " << std::hex << merge << std::dec << std::endl;
    face *left = merge->left_face(v0);
    edge *left_next = left->next_edge(merge);
    edge *left_previous = left->previous_edge(merge);
    face *left_merge_face = left_next->right_face(v1);
    assert(!left_next->complex() && !left_previous->complex());

    face *right = merge->right_face(v0);
    edge *right_next = right->next_edge(merge);
    edge *right_previous = right->previous_edge(merge);
    face *right_merge_face = right_next->right_face(v0);
    assert(!right_next->complex() && !right_previous->complex());

    assert(left_next != merge);
    assert(left_previous != merge);
    assert(left_previous != left_next);
    assert(left_next->left_face(v1) == left);
    assert(left_previous->right_face(v0) == left);
    
    assert(right_next != merge);
    assert(right_previous != merge);
    assert(right_previous != right_next);
    assert(right_next->left_face(v0) == right);
    assert(right_previous->right_face(v1) == right);

    assert(left_merge_face != left);
    assert(left_merge_face != right);
    assert(right_merge_face != left);
    assert(right_merge_face != right);

    /* Drop triangles by merging its remaining edges*/
    left_previous->update_right_face(left_merge_face, v0);
    left_next->removed(true);

    right_previous->update_right_face(right_merge_face, v1);
    right_next->removed(true);

    /* Update edges of surviving faces */
    left_merge_face->update_edge(left_next, left_previous);
    right_merge_face->update_edge(right_next, right_previous);

    /* Merge the edge vertices */
    merge->removed(true);
    left->removed(true);
    right->removed(true);
    return { left_merge_face, left_previous };
}

void mesh_decimation::decimate(edge *root)
{
    /* Get initial costs */
    auto costs(decimation_costs());

    /* While costs below limit merge edges */
    std::cout << "Decimating from " << costs.top().cost << " to " << _options.max_cost << std::endl;
    float recalc_cost = std::numeric_limits<float>::max();
    while ((_iteration < _options.max_iterations) && !costs.empty())
    {
        /* Check if the edge is still valid */
        edge *merge = costs.top().e;
        if (merge->removed())
        {
            std::cout << "Edge already removed" << std::endl;
            next_cost(&costs);
            continue;
        }

        const float cost = costs.top().cost;
        if (merge->recalculate())
        {
            std::cout << "Edge requires recalculating " << cost << std::endl;
            next_cost(&costs);
            recalc_cost = std::min(recalc_cost, cost);
            continue;
        }

        /* Check if we have a chance to reduce cost by recalculating */
        std::cout << "Attempting at cost " << cost << std::endl;
        if (cost > std::max(1.0f, (recalc_cost * 1.1f)))
        {
            std::cout << "Cost recalculation at iteration " << _iteration << " at cost " << cost << " versus recalc cost " << recalc_cost << std::endl;
            recalc_cost = std::numeric_limits<float>::max();
            costs = decimation_costs();
            continue;
        }

        /* Check if the cost is too high */
        if (cost > _options.max_cost)
        {
            // std::cout << "Max cost hit, stopping" << std::endl;
            break;
        }

        /* Check for complex vertices */
        auto *v0 = merge->vertex0();
        auto *v1 = merge->vertex1();
        if (v0->complex() || v1->complex())
        {
            next_cost(&costs);
            continue;
        }

        std::cout << "Merging " << std::hex << merge << std::dec << " at cost " << cost << " iteration " << _iteration << std::hex << " dropping faces " << merge->edges_left_face() << ", " << merge->edges_right_face() << std::dec << std::endl;
        std::cout << "End vertices " << v0->position() << " and " << v1->position() << std::endl;

        /* Get triangles next to merge */
        face *left = merge->edges_left_face();
        std::cout << "Left vertices " << left->vertice(0)->position() << " and " << left->vertice(1)->position() << " and " << left->vertice(2)->position() << std::endl;
        edge *left_next = left->next_edge(merge);
        edge *left_previous = left->previous_edge(merge);
        if (left_next->complex() || left_previous->complex())
        {
            next_cost(&costs);
            continue;
        }
        face *left_merge_face = left_previous->left_face(v0);
        std::cout << std::hex << "Left next " << left_next << " faces " << left_next->edges_left_face() << ", " << left_next->edges_right_face() << " updated with left prev " << left_previous << " face " << left_merge_face << ", " << left_previous->right_face(v0) << std::dec << std::endl;

        face *right = merge->edges_right_face();
        std::cout << "Right vertices " << right->vertice(0)->position() << " and " << right->vertice(1)->position() << " and " << right->vertice(2)->position() << std::endl;
        edge *right_next = right->next_edge(merge);
        edge *right_previous = right->previous_edge(merge);
        if (right_next->complex() || right_previous->complex())
        {
            next_cost(&costs);
            continue;
        }
        face *right_merge_face = right_previous->left_face(v1);
        std::cout << std::hex << "Right next " << right_next << " faces " << right_next->edges_left_face() << ", " << right_next->edges_right_face() << " updated with right prev " << right_previous << " face " << right_merge_face << ", " << right_previous->right_face(v1) << std::dec << std::endl;

        /* Handle cases that have reduce to back to back triangles */
        if ((left_merge_face == right) || (right_merge_face == left))
        {
            merge->removed(true);
            left->removed(true);
            right->removed(true);
            if (left_merge_face == right)
            {
                std::cout << "Wrapping left merge face to right" << std::endl;
                left_previous->removed(true);
            }

            if (right_merge_face == left)
            {
                std::cout << "Wrapping right merge face to left" << std::endl;
                right_previous->removed(true);
            }

            next_cost(&costs);
            ++_iteration;
            std::cout << std::endl;
            continue;
        }

        assert(left_next != merge);
        assert(left_previous != merge);
        assert(left_previous != left_next);
        assert(left_next->left_face(v1) == left);
        assert(left_previous->right_face(v0) == left);
        
        assert(right_next != merge);
        assert(right_previous != merge);
        assert(right_previous != right_next);
        assert(right_next->left_face(v0) == right);
        assert(right_previous->right_face(v1) == right);

        assert(left_merge_face != left);
        assert(left_merge_face != right);
        assert(right_merge_face != left);
        assert(right_merge_face != right);

        /* Check if any faces would flip */
        const point_t &vm = merge->merge_point();
        if (can_merge_fan(left_next, right_previous, v1, vm) || can_merge_fan(right_next, left_previous, v0, vm))
        {
            std::cout << "Triangle flipped" << std::endl;
            costs.pop();
            continue;
        }

        /* Flag merged edges for recalculate */
        edge *step = right_next;
        do
        {
            face *r = step->right_face(v0);
            step->recalculate(true);
            step = r->next_edge(step);
        } while (step != left_previous);
        std::cout << "v0 invalidated" << std::endl;

        /* Update vertices of surviving edges and faces */
        std::cout << "Remove at cost " << cost << std::endl;
        step = left_next;
        do
        {
            /* Check for triangle collapse to line */
            face *r = step->right_face(v1);
            edge *duplicate = r->update_vertex(v1, v0);
            if (duplicate != nullptr)
            {
                assert(duplicate != right_previous);

                auto [ f, e ] = remove_edge(duplicate, v0, v1);
                f->update_vertex(v1, v0);
                assert(!left_next->removed());
                assert(!right_next->removed());

                step->update_vertex(v1, v0);
                step->recalculate(true);
                if ((e == right_previous) || right_previous->removed())
                {
                    std::cout << "Next edge is done" << std::endl;
                    break;
                }
                e->recalculate(true);

                step = f->next_edge(e);
            }
            else
            {
                step->update_vertex(v1, v0);
                step->recalculate(true);
                step = r->next_edge(step);
            }
        } while (step != right_previous);
        std::cout << "v1 updated" << std::endl;

        /* Drop triangles by merging its remaining edges*/
        left_next->update_left_face(left_merge_face, v0);
        left_previous->removed(true);

        right_next->update_left_face(right_merge_face, v0);
        right_previous->removed(true);

        /* Update edges of surviving faces */
        left_merge_face->update_edge(left_next, v0);
        right_merge_face->update_edge(right_next, v0);

        /* Merge the edge vertices */
        v0->collapse(v1, vm, cost);
        merge->removed(true);
        left->removed(true);
        right->removed(true);

        costs.pop();
        ++_iteration;
        std::cout << std::endl;
    }
    // std::cout << "Done, iteration " << _iteration << " costs " << costs.size() << std::endl;
}

void mesh_decimation::compact_output(std::vector<point_t> &vertices, std::vector<point_ti<>> &tris)
{
    /* Grab plenty of space */
    std::map<const vertex *, int> written;
    vertices.reserve(_vertices.size());
    tris.reserve(_faces.size());

    /* Find a valid face and write it out */
    for (const auto &f : _faces)
    {
        if (f.removed())
        {
            continue;
        }

        point_ti<> tri;
        for (int i = 0; i < 3; ++i)
        {
            tri[i] = vertices.size();
            if (const auto &wit = written.emplace(f.vertice(i), tri[i]); wit.second)
            {
                const vertex *v = f.vertice(i);
                assert(!v->removed());
                vertices.emplace_back(v->position());
            }
            else
            {
                tri[i] = wit.first->second;
            }
        }

        tris.emplace_back(tri);
    }
}

mesh_decimation::mesh_decimation(const std::vector<point_t> &points, const std::vector<point_ti<>> &triangles, const mesh_decimation_options &options) : _options(options), _iteration(0)
{
    /* Find the model size */
    point_t max_bound(points[0]);
    point_t min_bound(points[0]);
    for (const auto &p : points)
    {
        max_bound = max(max_bound, p);
        min_bound = min(min_bound, p);
    }
    const point_t offset((max_bound + min_bound) * 0.5f);
    const point_t scale((max_bound - min_bound) * 0.5f);
    const float max_scale_inv = 1.0f / std::max(scale.x, std::max(scale.y, scale.z));
    const point_t scale_inv(max_scale_inv, max_scale_inv, max_scale_inv);

    /* Build vertices, triangles and edges */
    point_t com;
    _vertices.reserve(points.size());
    for (const auto &p : points)
    {
        const point_t local((p - offset) * scale_inv);
        com += local;
        _vertices.emplace_back(local);
    }
    com /= static_cast<float>(_vertices.size());

    _faces.reserve(triangles.size());
    _edges.reserve(triangles.size() * 3);
    for (const auto &t : triangles)
    {
        _faces.emplace_back(&_vertices[t.x], &_vertices[t.y], &_vertices[t.z]);
        add_edge(&_vertices[t.x], &_vertices[t.y], &_faces.back());
        add_edge(&_vertices[t.y], &_vertices[t.z], &_faces.back());
        add_edge(&_vertices[t.z], &_vertices[t.x], &_faces.back());
        _vertices[t.x].add_face(_faces.back());
        _vertices[t.y].add_face(_faces.back());
        _vertices[t.z].add_face(_faces.back());
    }

    /* Fix up boundary edges */
    for (const auto &ue : _unique_edges)
    {
        _edges.push_back(ue);
        
        /* Add last face to edge */
        auto &e = _edges.back();
        e.complex(true);
        e.vertex0()->complex(true);
        e.vertex1()->complex(true);
        std::cout << "Boundary edge " << e.vertex0()->position() << " and " << e.vertex1()->position() << std::endl;

        /* Add edge to faces */
        if (face *f = e.edges_left_face(); f != nullptr)
        {
            f->add_edge(&e);
        }

        if (face *f = e.edges_right_face(); f != nullptr)
        {
            f->add_edge(&e);
        }
    }

    /* Check edges */
    for (const auto &e : _edges)
    {
        assert(e.check());
    }

    /* Check faces */
    for (const auto &f : _faces)
    {
        assert(f.check());
    }

    const float com_cost = std::accumulate(_vertices.begin(), _vertices.end(), 0.0f, [com](const float s, const vertex &v) { return s + v.merge_error(com); });
    _options.absolute_costs(com_cost, _vertices.size());
    // std::cout << "com cost " << com_cost << " absolute cost limit " << _options.max_cost << " absolute iteration limit " << _options.max_iterations << std::endl;
    decimate(&_edges[0]);
}
} /* namespace raptor_mesh_decimation */
