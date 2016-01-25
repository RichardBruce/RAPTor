#pragma once

/* Standard headers */
#include <limits>
#include <vector>

/* Boost headers */
#include "boost/noncopyable.hpp"

/* Common headers */
#include "point_t.h"

/* Convex decomposition headers */
#include "tm_mesh.h"


namespace raptor_convex_decomposition
{
enum class convex_hull_error_t : char { ok = 0, not_enough_points = 1, inconsistent = 2 };

class incremental_convex_hull : private boost::noncopyable
{
    public :
        /* Ctor */
        incremental_convex_hull() : _last_processed(_mesh.get_vertices().end()), _flat(false) {  }

        /* Access functions */
        tm_mesh&    mesh()                        { return _mesh;                       }
        int         number_of_vertices()    const { return _mesh.number_of_vertices();  }
        int         number_of_triangles()   const { return _mesh.number_of_triangles(); }
        bool        flat()                  const { return _flat;                       }

        void add_points(const std::vector<point_t> &points)
        {
            for (int i = 0; i < static_cast<int>(points.size()); ++i)
            {
                _mesh.add_vertex(points[i], i);
            }
        }

        void add_point(const point_t &pt, const int id)
        {
            _mesh.add_vertex(pt, id);
        }

        convex_hull_error_t process()
        {
            /* Check we have enough vertices to make a hull */
            if (_mesh.number_of_vertices() < 3)
            {
                return convex_hull_error_t::not_enough_points;
            }

            /* First time processing */
            auto &vertices = _mesh.get_vertices();
            if (_last_processed == vertices.end())
            {
                _last_processed = vertices.begin();
            }

            /* Only enough vertices for a triangle */
            if (_mesh.number_of_vertices() == 3)
            {
                return triangle_hull();
            }

            /* Start again if currently flat */
            restart_if_flat();

            /* Create the first triangle */
            if (_mesh.number_of_triangles() == 0)
            {
                if (!double_sided_triangle())
                {
                    return convex_hull_error_t::inconsistent;
                }
            }

            /* Find the first unprocessed verterx */
            /* Process all remaining vertices */
            ++_last_processed;
            for (; _last_processed != vertices.end(); )
            {
                if (process_point(&_last_processed))
                {
                    ++_last_processed;
                    _mesh.clean_up(&_delete_triangles, &_update_edges, &_delete_edges, _last_processed);
                    assert(_mesh.check_consistency() || !"Error: Inconsistent mesh");
                }
            }

            /* Extra processing for flat hulls */
            clean_up_flat_mesh();

            --_last_processed;
            return convex_hull_error_t::ok;
        }

        convex_hull_error_t process(const int max_points, const float min_volume)
        {
            /* Check we have enough vertices to make a hull */
            if ((max_points < 3) || (_mesh.number_of_vertices() < 3))
            {
                return convex_hull_error_t::not_enough_points;
            }

            /* First time processing */
            auto &vertices = _mesh.get_vertices();
            if (_last_processed == vertices.end())
            {
                _last_processed = vertices.begin();
            }

            /* Only enough vertices for a triangle */
            if (_mesh.number_of_vertices() == 3)
            {
                return triangle_hull();
            }

            /* Start again if currently flat */
            restart_if_flat();
            
            /* Create the first triangle */
            int points_added = 0;  
            if (_mesh.number_of_triangles() == 0)
            {
                if (!double_sided_triangle())
                {
                    return convex_hull_error_t::inconsistent;
                }
                else
                {
                    points_added += 3;
                }
            }

            /* Process remaining vertices */
            ++_last_processed;
            while ((_last_processed != vertices.end()) && (points_added < max_points))
            {
                if (!find_max_volume_point(&_last_processed, (points_added > 3) ? min_volume : 0.0f))
                {
                    break;
                }

                if (process_point(&_last_processed))
                {
                    ++points_added;
                    points_added -= _mesh.clean_up(&_delete_triangles, &_update_edges, &_delete_edges, _last_processed);
                    if (!_mesh.check_consistency())
                    {
                        // std::cout.precision(17);
                        // std::cout << "max_points: " << max_points << ", min_volume: " << min_volume << std::endl;
                        // for (const auto &v : _mesh.get_vertices())
                        // {
                        //     std::cout << std::fixed << v.position() << std::endl;
                        // }
                    }
                    assert(_mesh.check_consistency() || !"Error: Inconsistent mesh");
                    ++_last_processed;
                }
            }

            /* Delete remaining points */
            while (_last_processed != vertices.end())
            {
                _last_processed = vertices.erase(_last_processed);
            }

            /* Extra processing for flat hulls */
            clean_up_flat_mesh();

            --_last_processed;
            return convex_hull_error_t::ok;
        }

        bool is_inside(const point_t &pt, const float eps)
        {
            if (_flat)
            {
                for (const auto &t : _mesh.get_triangles())
                {
                    const point_t ver0(t.vertex(0)->position());
                    const point_t ver1(t.vertex(1)->position());
                    const point_t ver2(t.vertex(2)->position());
                    const point_t a(ver1 - ver0);
                    const point_t b(ver2 - ver0);
                    const point_t c(pt - ver0);

                    const float a_dot_a = dot_product(a, a);
                    const float a_dot_b = dot_product(a, b);
                    const float a_dot_c = dot_product(a, c);
                    const float b_dot_b = dot_product(b, b);
                    const float b_dot_c = dot_product(b, c);
                    const float denom = 1.0f / ((a_dot_a * b_dot_b) - (a_dot_b * a_dot_b));
                    const float u = ((b_dot_b * a_dot_c) - (a_dot_b * b_dot_c)) * denom;
                    const float v = ((a_dot_a * b_dot_c) - (a_dot_b * a_dot_c)) * denom;
                    
                    if ((u >= 0.0f) && (u <= 1.0f) && (v >= 0.0f) && ((u + v) <= 1.0f))
                    {
                        return true;
                    }
                }

                return false;
            }
            else
            {
                for (const auto &t : _mesh.get_triangles())
                {
                    const float vol = t.volume_with_point(pt);
                    if (vol < eps)
                    {
                        return false;
                    }
                }

                return true;
            }
        }

        void clear()
        {    
            _flat = false;
            _mesh.clear();
            _delete_edges.clear();
            _update_edges.clear();
            _delete_triangles.clear();
            _last_processed = _mesh.get_vertices().end();
        }

    private :
        void restart_if_flat()
        {
            if (_flat)
            {
                _mesh.get_edges().clear();
                _mesh.get_triangles().clear();
                _last_processed = _mesh.get_vertices().begin();
                _flat = false;
            }
        }

        convex_hull_error_t triangle_hull()
        {
            _flat = true;
            tmm_vertex_iter v0 = _last_processed++;
            tmm_vertex_iter v1 = _last_processed++;
            tmm_vertex_iter v2 = _last_processed;

            /* Calculate plane normal */
            const point_t p0(v0->position());
            const point_t p1(v1->position());
            const point_t p2(v2->position());
            _normal = normalise(cross_product((p1 - p0), (p2 - p0)));

            _mesh.add_triangle(v0, v1, v2);
            _mesh.add_triangle(v2, v1, v0);
            return convex_hull_error_t::ok;
        }

        void clean_up_flat_mesh()
        {
            /* Not flat, nothing to do */
            if (!_flat)
            {
                return;
            }

            auto &triangles = _mesh.get_triangles();
            std::vector<tmm_triangle_iter> to_duplicate;
            for (auto t = triangles.begin(); t != triangles.end(); ++t)
            {
                /* Triangle has dummy vertices then mark for delete */
                if ((t->vertex(0)->name() == dummy_index) || (t->vertex(1)->name() == dummy_index) || (t->vertex(2)->name() == dummy_index))
                {
                    _delete_triangles.push_back(t);
                    for (int k = 0; k < 3; ++k)
                    {
                        for (int h = 0; h < 2; ++h)
                        {
                            if (t->edge(k)->triangle(h) == t)
                            {
                                t->edge(k)->triangle(triangles.end(), h);
                                break;
                            }
                        }
                    }
                }
                /* Otherwise mark for duplication */
                else
                {
                    to_duplicate.push_back(t);
                }
            }

            /* Find triangle less edges and mark for delete */
            for (auto e = _mesh.get_edges().begin(); e != _mesh.get_edges().end(); ++e)
            {
                if ((e->triangle(0) == triangles.end()) && (e->triangle(1) == triangles.end())) 
                {
                    _delete_edges.push_back(e);
                }
            }

            /* Remove dummy vertices and mark remaining as unprocessed */
            auto &vertices = _mesh.get_vertices();
            for (auto v = vertices.begin(); v != vertices.end(); )
            {
                if (v->name() == dummy_index)
                {
                    v = vertices.erase(v);
                }
                else
                {
                    ++v;
                }
            }

            /* Clean up and add duplicates */
            _mesh.clean_edges(&_update_edges, &_delete_edges);
            _mesh.clean_triangles(&_delete_triangles);
            for (const auto &t : to_duplicate)
            {
                _mesh.add_triangle(t->vertex(1), t->vertex(0), t->vertex(2));
            }
        }

        bool find_max_volume_point(tmm_vertex_iter *const v, const float min_volume)
        {
            auto &vertices = _mesh.get_vertices();
            
            float volume                = 0.0f;
            float max_volume            = min_volume;
            tmm_vertex_iter v_iter      = *v;
            tmm_vertex_iter vmax_iter   = vertices.end();
            while (v_iter != vertices.end())
            {
                if (compute_point_volume(&v_iter, &volume, false))
                {
                    if (volume > max_volume)
                    {
                        max_volume = volume;
                        vmax_iter = v_iter;
                    }
                    ++v_iter;
                }
            }

            /* Nothing adds enough volume */
            if (vmax_iter == vertices.end())
            {
                return false;
            }

            /* Move max volume element to the front */
            if (vmax_iter != (*v))
            {
                vertices.splice(*v, vertices, vmax_iter);
                --(*v);
            }

            return true;
        }

        bool compute_point_volume(tmm_vertex_iter *const v, float *const total_volume, bool mark_visible_faces)
        {
            /* Compute volume for all triangles */
            float tv        = 0.0f;
            bool visible    = false;
            const point_t pos((*v)->position());
            for (auto t = _mesh.get_triangles().begin(); t != _mesh.get_triangles().end(); ++t)
            {
                const float vol = t->volume_with_point(pos);
                // std::cout << "vol: " << vol << std::endl;
                if (vol < 0.0f)
                {
                    tv -= vol;
                    if (mark_visible_faces)
                    {
                        t->visible(true);
                        _delete_triangles.push_back(t);
                    }
                    visible = true;
                }
            }
            (*total_volume) = tv;
            assert((static_cast<int>(_delete_triangles.size()) != _mesh.number_of_triangles()) || !"Error: All mesh triangles are visible");

            /* If no faces visible from p then p is inside the hull */
            if (!visible && mark_visible_faces)
            {
                (*v) = _mesh.get_vertices().erase(*v);
                _delete_triangles.clear();
                return false;
            }
            return true;
        }

        bool process_point(tmm_vertex_iter *const v)
        {
            /* Check point visibility and mark visible faces */
            float total_volume;
            if (!compute_point_volume(v, &total_volume, true))
            {
                return false;
            }

            /* Classify edges base on the visibility of their triangles */
            _delete_edges.clear();
            _update_edges.clear();
            for (auto e = _mesh.get_edges().begin(); e != _mesh.get_edges().end(); ++e) 
            {
                int visible = 0;
                for (int k = 0; k < 2; ++k)
                {
                    if (e->triangle(k)->visible())
                    {
                        ++visible;
                    }
                }

                /* Edge is no invisible, mark for delete */
                if (visible == 2)
                {
                    // std::cout << "deleting edge: " << e->vertex(0)->id() << " to: " <<  e->vertex(1)->id() << std::endl;
                    _delete_edges.push_back(e);
                }
                /* Edge is on the visible-invisible boudary, add a new face here */
                else if (visible == 1)
                {
                    // std::cout << "updating edge: " << e->vertex(0)->id() << " to: " <<  e->vertex(1)->id() << std::endl;
                    e->new_face(_mesh.make_cone_face(e, *v));
                    _update_edges.push_back(e);
                }
            }

            // std::cout << std::endl;
            return true;
        }

        bool double_sided_triangle()
        {
            /* Find three non colinear points */
            _flat = false;
            auto &vertices = _mesh.get_vertices();
            tmm_vertex_iter v0 = _last_processed++;
            tmm_vertex_iter v1 = _last_processed++;
            tmm_vertex_iter v2 = _last_processed;
            while (co_linear(v0->position(), v1->position(), v2->position()))
            {
                ++v0;
                ++v1;
                ++v2;
                if (v2 == vertices.end())
                {
                    return false;
                }
            }

            _last_processed = v2;

            /* Create triangles */
            const tmm_triangle_iter f0 = _mesh.make_face(v0, v1, v2, _mesh.get_triangles().end());
            _mesh.make_face(v2, v1, v0, f0);

            /* Find a fourth non-coplanar point to form tetrahedron */
            tmm_vertex_iter v3 = v2;
            ++v3;

            /* Move vertices to the start */
            if (v0 != vertices.begin())
            {
                vertices.splice(vertices.begin(), vertices, v0, v3);
            }

            float vol = 0.0f;
            while ((std::fabs(vol) < epsilon) && (v3 != vertices.end()))
            {
                vol = tetrahedron_volume(v0->position(), v1->position(), v2->position(), v3->position());
                ++v3;
            }

            /* Mesh is flat, fake it as slightly less flat */
            if (std::fabs(vol) < epsilon)
            {
                _flat = true;

                /* Compute the barycenter */
                point_t bary(0.0f, 0.0f, 0.0f);
                for (const auto &v : vertices)
                {
                    bary += v.position();
                }
                bary /= static_cast<float>(_mesh.number_of_vertices());

                /* Compute the normal to the plane */
                const point_t p0(v0->position());
                const point_t p1(v1->position());
                const point_t p2(v2->position());
                _normal = normalise(cross_product((p1 - p0), (p2 - p0)));

                /* Add dummy vertex placed at (bary + normal) */
                add_point(bary + _normal, dummy_index); 
                vertices.splice(++v2, vertices, --vertices.end());
                return true;
            }

            /* Move the point creating a volume to be the next one processed*/
            --v3;
            ++v2;
            if (v2 != v3)
            {
                vertices.splice(v2, vertices, v3);
            }

            return true;
        }

        tm_mesh                         _mesh;
        std::vector<tmm_edge_iter>      _delete_edges;
        std::vector<tmm_edge_iter>      _update_edges;
        std::vector<tmm_triangle_iter>  _delete_triangles; 
        tmm_vertex_iter                 _last_processed;
        point_t                         _normal;
        bool                            _flat;

        static constexpr float epsilon      = 1.0e-8f;
        static constexpr int   dummy_index  = std::numeric_limits<int>::max();
};
} /* namespace raptor_convex_decomposition */
