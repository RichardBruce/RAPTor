#pragma once

/* Standard headers */
#include <cassert>
#include <cstdint>
#include <limits>
#include <vector>

/* Boost headers */
#include "boost/noncopyable.hpp"
#include "boost/pool/object_pool.hpp"

/* Common headers */
#include "point_t.h"


namespace raptor_convex_decomposition
{
class dac_convex_hull : private boost::noncopyable
{
    public :
        /* Ctor */
        dac_convex_hull(const std::vector<point_t> &coords) : _edge_pool(6 * coords.size())
        {
            if (coords.empty())
            {
                return;
            }

            /* Compute the convex hull */
            compute(coords);

            std::vector<vertex *> old_vertices;
            old_vertices.push_back(_vertex_list);
            _vertex_list->copy = 0;
            int copied = 0;
            while (copied < static_cast<int>(old_vertices.size()))
            {
                vertex* v = old_vertices[copied];
                _vertices.push_back(get_coordinates(v));
                if (v->edges != nullptr)
                {
                    edge* e = v->edges;
                    do
                    {
                        if (e->copy < 0)
                        {
                            e->copy = 1;
                            if (e->target->copy < 0)
                            {
                                e->target->copy = old_vertices.size();
                                old_vertices.push_back(e->target);
                            }
                        }
                        e = e->next;
                    } while (e != v->edges);
                }
                ++copied;
            }

            for (int i = 0; i < static_cast<int>(copied); ++i)
            {
                vertex* v = old_vertices[i];
                if (v->edges != nullptr)
                {
                    edge* e = v->edges;
                    do
                    {
                        if (e->copy >= 0)
                        {
                            _faces.push_back(std::vector<int>());
                            edge* f = e;
                            do
                            {
                                f->copy = -1;
                                _faces.back().push_back(f->target->copy);
                                f = f->reverse->prev;
                            } while (f != e);
                        }
                        e = e->next;
                    } while (e != v->edges);
                }
            }

            return;
        }

        /* Calculate hull volume */
        float volume() const
        {
            /* For each face */
            float vol = 0.0f;
            for (const auto &f : _faces)
            {
                /* Break face into triangles */
                const point_t a(_vertices[f[0]]);
                for (int i = 1; i < static_cast<int>(f.size() - 1); ++i)
                {
                    /* Find volume as sum of simplices */
                    vol += tetrahedron_volume(a, _vertices[f[i]], _vertices[f[i + 1]], _center);
                }
            }

            return vol * (1.0f / 6.0f);
        }

        /* Access functions */
        const std::vector<point_t>&             vertices()  const { return _vertices;   }
        const std::vector<std::vector<int>>&    faces()     const { return _faces;      }

    private :
        class Rational64
        {
            private :
                uint64_t m_numerator;
                uint64_t m_denominator;
                int sign;
                
            public :
                Rational64(int64_t numerator, int64_t denominator)
                {
                    if (numerator > 0)
                    {
                        sign = 1;
                        m_numerator = (uint64_t) numerator;
                    }
                    else if (numerator < 0)
                    {
                        sign = -1;
                        m_numerator = (uint64_t) -numerator;
                    }
                    else
                    {
                        sign = 0;
                        m_numerator = 0;
                    }
                    if (denominator > 0)
                    {
                        m_denominator = (uint64_t) denominator;
                    }
                    else if (denominator < 0)
                    {
                        sign = -sign;
                        m_denominator = (uint64_t) -denominator;
                    }
                    else
                    {
                        m_denominator = 0;
                    }
                }
                
                bool isNegativeInfinity() const
                {
                    return (sign < 0) && (m_denominator == 0);
                }
                
                bool isNaN() const
                {
                    return (sign == 0) && (m_denominator == 0);
                }
                
                int compare(const Rational64& b) const;
        };

        class edge;

        class vertex
        {
            public :
                vertex* next;
                vertex* prev;
                edge* edges;
                point_ti<std::int64_t> point;
                int copy;
                
                vertex(): next(nullptr), prev(nullptr), edges(nullptr), copy(-1)
                {
                }

                point_ti<std::int64_t> operator-(const vertex& b) const
                {
                    return point - b.point;
                }

                float xvalue() const
                {
                    return static_cast<float>(point.x);
                }

                float yvalue() const
                {
                    return static_cast<float>(point.y);
                }

                float zvalue() const
                {
                    return static_cast<float>(point.z);
                }
        };

        class edge
        {
            public :
                edge* next;
                edge* prev;
                edge* reverse;
                vertex* target;
                int copy;

                ~edge()
                {
                    next = nullptr;
                    prev = nullptr;
                    reverse = nullptr;
                    target = nullptr;
                }

                void link(edge* n)
                {
                    assert(reverse->target == n->reverse->target);
                    next = n;
                    n->prev = this;
                }
        };
    
        class intermediate_hull
        {
            public :
                vertex* minXy;
                vertex* maxXy;
                vertex* minYx;
                vertex* maxYx;
                
                intermediate_hull() : minXy(nullptr), maxXy(nullptr), minYx(nullptr), maxYx(nullptr) {  }
        };

        void remove_edge_pair(edge* e)
        {
            edge* n = e->next;
            edge* r = e->reverse;
            assert(e->target && r->target);

            if (n != e)
            {
                n->prev = e->prev;
                e->prev->next = n;
                r->target->edges = n;
            }
            else
            {
                r->target->edges = nullptr;
            }
            
            n = r->next;            
            if (n != r)
            {
                n->prev = r->prev;
                r->prev->next = n;
                e->target->edges = n;
            }
            else
            {
                e->target->edges = nullptr;
            }

            _edge_pool.free(e);
            _edge_pool.free(r);
        }
    
        enum class orientation_t : char { none, clockwise, counter_clockwise };
        
        void compute_internal(intermediate_hull *const result, const int start, const int end);
        bool merge_projection(intermediate_hull *const h0, intermediate_hull *const h1, vertex*& c0, vertex*& c1);
        void merge(intermediate_hull *const h0, intermediate_hull *const h1);
        void compute(const std::vector<point_t> &coords);
        point_t get_coordinates(const vertex* v);

        static orientation_t get_orientation(const edge &prev, const edge *const next, const point_ti<std::int64_t> &s, const point_ti<std::int64_t> &t);
        edge* find_max_angle(Rational64 *const min_cot, const vertex &start, const point_ti<std::int64_t> &s, const point_ti<std::int64_t> &rxs, const point_ti<std::int64_t> &sxrxs, const bool ccw);
        void find_edge_for_coplanar_faces(const vertex &c0, const vertex &c1, edge*& e0, edge*& e1, const vertex *const stop0, const vertex *const stop1);

        edge* new_edge_pair(vertex* from, vertex* to);

        std::vector<point_t>            _vertices;      /* Vertices of the output hull                                                                                                      */
        std::vector<std::vector<int>>   _faces;         /* Faces of the convex hull. Each entry is an index into the "edges" array pointing to an edge of the face. Faces are planar n-gons */
        vertex*                         _vertex_list;
        point_t                         _scaling;
        point_t                         _center;
        boost::object_pool<edge>        _edge_pool;
        std::vector<vertex>             _original_vertices;
        int                             _merge_stamp;
        int                             _min_axis;
        int                             _med_axis;
        int                             _max_axis;
};
} /* namespace raptor_convex_decomposition */
