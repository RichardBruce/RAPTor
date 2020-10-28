/* Standard headers */
#include <map>
#include <set>

/* Boost headers */

/* Common headers */
#include "point_t.h"

/* Mesh decimation headers */
#include "mesh_decimation.h"


namespace raptor_mesh_decimation
{
void decimation_costs(std::map<float, edge*> &costs, std::set<edge *> &processed, edge *e)
{
    costs.emplace(e->merge_cost(), e);

    edge *l0 = e->edges_left_face()->next_edge(e);
    if (processed.emplace(l0).second)
    {
        decimation_costs(costs, processed, l0);
    }

    edge *l1 = e->edges_left_face()->previous_edge(e);
    if (processed.emplace(l1).second)
    {
        decimation_costs(costs, processed, l1);
    }

    edge *r0 = e->edges_right_face()->next_edge(e);
    if (processed.emplace(r0).second)
    {
        decimation_costs(costs, processed, r0);
    }

    edge *r1 = e->edges_right_face()->previous_edge(e);
    if (processed.emplace(r1).second)
    {
        decimation_costs(costs, processed, r1);
    }
}

std::map<float, edge*> decimation_costs(edge *root)
{
    /* Depth first search to build vertex merge costs */
    std::set<edge *> processed;
    std::map<float, edge *> costs;
    decimation_costs(costs, processed, root);

    return costs;
}

void decimate(edge *root, const float max_cost)
{
    /* Get initial costs */
    auto costs(decimation_costs(root));

    /* While costs below limit merge edges */
    for (const float cost = costs.begin()->first; cost < max_cost;)
    {
        edge *merge = costs.begin()->second;

        /* Drop left triangle by merging its remaining edges*/
        face *left = merge->edges_left_face();
        edge *left_next = left->next_edge(merge);
        edge *left_previous = left->previous_edge(merge);
        left_next->left_face(left_previous->left_face(merge->vertex0()), merge->vertex1());

        /* Drop right triangle by merging its remaining edges*/
        face *right = merge->edges_right_face();
        edge *right_next = right->next_edge(merge);
        edge *right_previous = right->previous_edge(merge);
        right_next->left_face(right_previous->left_face(merge->vertex1()), merge->vertex0());

        /* Update remaining edges end point */
        const point_t &merge_point = merge->merge_point();
        right_next->vertex0()->collapse(left_next->vertex0(), merge_point, cost);
        left_next->vertex0(right_next->vertex0());

        costs.erase(costs.begin());
    }
}

mesh_decimation::mesh_decimation(const std::vector<point_t> &points, const std::vector<point_ti<>> &triangles, const mesh_decimation_options &options) : _options(options)
{
    /* Build vertices, triangles and edges */
    _vertices.reserve(points.size());
    for (const auto &p : points)
    {
        _vertices.emplace_back(p);
    }

    _faces.reserve(triangles.size());
    for (const auto &t : triangles)
    {
        _faces.emplace_back(&_vertices[t.x], &_vertices[t.y], &_vertices[t.z]);
        add_edge(&_vertices[t.x], &_vertices[t.y], &_faces.back());
        add_edge(&_vertices[t.y], &_vertices[t.z], &_faces.back());
        add_edge(&_vertices[t.x], &_vertices[t.z], &_faces.back());
    }

    decimate(&_edges[0], 0.0);
}
} /* namespace raptor_mesh_decimation */
