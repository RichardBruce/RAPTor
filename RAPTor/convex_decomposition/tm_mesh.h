#pragma once

/* Standard headers */
#include <list>

/* Boost headers */
#include "boost/noncopyable.hpp"
#include "boost/pool/pool_alloc.hpp"

/* Common headers */
#include "point_t.h"


namespace raptor_convex_decomposition
{
class tmm_vertex;
class tmm_edge;
class tmm_triangle;
class tm_mesh;

template<class T>
using allocator = boost::fast_pool_allocator<T, boost::default_user_allocator_new_delete, boost::details::pool::null_mutex, 1024, 0>;

using tmm_vertex_iter   = std::list<tmm_vertex, allocator<tmm_vertex>>::iterator;
using tmm_edge_iter     = std::list<tmm_edge, allocator<tmm_edge>>::iterator;
using tmm_triangle_iter = std::list<tmm_triangle, allocator<tmm_triangle>>::iterator;

class tmm_vertex
{
    public :
        tmm_vertex(const tmm_edge_iter &duplicate, const point_ti<std::int64_t> &pos, const int id) :
            _duplicate(duplicate),
            _pos(pos),
            _name(id),
            _id(id),
            _on_hull(false)
        {  }

        /* Getters */
        tmm_edge_iter                   duplicate() const { return _duplicate;  }
        const point_ti<std::int64_t>&   position()  const { return _pos;        }
        int                             name()      const { return _name;       }
        int                             id()        const { return _id;         }
        bool                            on_hull()   const { return _on_hull;    }

        /* Setters */
        tmm_vertex& duplicate(const tmm_edge_iter &dup)
        {
            _duplicate = dup;
            return *this;
        }

        tmm_vertex& id(const int i)
        {
            _id = i;
            return *this;
        }

        tmm_vertex& on_hull(const bool oh)
        {
            _on_hull = oh;
            return *this;
        }

    private :
        tmm_edge_iter           _duplicate;        // pointer to incident cone edge (or NULL)
        point_ti<std::int64_t>  _pos;
        int                     _name;
        int                     _id;
        bool                    _on_hull;
};

class tmm_edge
{        
    public :
        tmm_edge(const tmm_triangle_iter &t0, const tmm_triangle_iter &t1, const tmm_triangle_iter &nf, const tmm_vertex_iter &v0, const tmm_vertex_iter &v1) :
            _triangles({ t0, t1 }),
            _vertices({ v0, v1 }),
            _new_face(nf)
        {  }

        /* Getters */
        const tmm_triangle_iter triangle(const int i)   const { return _triangles[i];   }
        const tmm_vertex_iter   vertex(const int i)     const { return _vertices[i];    }
        const tmm_triangle_iter new_face()              const { return _new_face;       }

        /* Setters */
        tmm_edge& triangle(const tmm_triangle_iter &t, const int i)
        {
            _triangles[i] = t;
            return *this;
        }

        tmm_edge& vertex(const tmm_vertex_iter &v, const int i)
        {
            _vertices[i] = v;
            return *this;
        }

        tmm_edge& new_face(const tmm_triangle_iter &nf)
        {
            _new_face = nf;
            return *this;
        }

    private :
        tmm_triangle_iter   _triangles[2];
        tmm_vertex_iter     _vertices[2];
        tmm_triangle_iter   _new_face;
};

class tmm_triangle
{
    public :
        tmm_triangle(const tmm_edge_iter &e0, const tmm_edge_iter &e1, const tmm_edge_iter &e2, const tmm_vertex_iter &v0, const tmm_vertex_iter &v1, const tmm_vertex_iter &v2) :
            _edges({ e0, e1, e2 }),
            _vertices({ v0, v1, v2 }),
            _visible(false)
        {  }

        /* Find the volume of 3 vertices with a point */
        std::int64_t volume_with_point(const point_ti<std::int64_t> &pt) const
        {
            return tetrahedron_volume(_vertices[0]->position(), _vertices[1]->position(), _vertices[2]->position(), pt);
        }

        /* Access functions */
        /* Getters */
        const tmm_edge_iter     edge(const int i)   const { return _edges[i];       }
        const tmm_vertex_iter   vertex(const int i) const { return _vertices[i];    }
        bool                    visible()           const { return _visible;        }
        
        /* Setters */
        tmm_triangle& edge(const tmm_edge_iter &e, const int i)
        {
            _edges[i] = e;
            return *this;
        }

        tmm_triangle& vertex(const tmm_vertex_iter &v, const int i)
        {
            _vertices[i] = v;
            return *this;
        }

        tmm_triangle& visible(const bool v)
        {
            _visible = v;
            return *this;
        }

    private :
        tmm_edge_iter   _edges[3];
        tmm_vertex_iter _vertices[3];
        bool            _visible;
};

class tm_mesh : private boost::noncopyable
{
    public :
        tm_mesh(const point_t &offset, const point_t &scale_inv) : _offset(offset), _scale_inv(scale_inv) {  }

        int                             number_of_vertices()    const { return _vertices.size();    }
        int                             number_of_edges()       const { return _edges.size();       }
        int                             number_of_triangles()   const { return _triangles.size();   }
        const std::list<tmm_vertex, allocator<tmm_vertex>> &        get_vertices()          const { return _vertices;           }
        const std::list<tmm_edge, allocator<tmm_edge>> &            get_edges()             const { return _edges;              }
        const std::list<tmm_triangle, allocator<tmm_triangle>> &    get_triangles()         const { return _triangles;          }

        std::list<tmm_vertex, allocator<tmm_vertex>> &              get_vertices()  { return _vertices;     }
        std::list<tmm_edge, allocator<tmm_edge>> &                  get_edges()     { return _edges;        }
        std::list<tmm_triangle, allocator<tmm_triangle>> &          get_triangles() { return _triangles;    }

        tmm_vertex_iter add_vertex(const point_t &pt, const int id)
        {
            _vertices.push_back(tmm_vertex(_edges.end(), point_ti<std::int64_t>((pt - _offset) * _scale_inv), id));
            return --_vertices.end();
        }

        tmm_vertex_iter add_vertex(const point_ti<std::int64_t> &pt, const int id)
        {
            _vertices.push_back(tmm_vertex(_edges.end(), pt, id));
            return --_vertices.end();
        }

        tmm_edge_iter add_edge(const tmm_vertex_iter &v0, const tmm_vertex_iter &v1)
        {
            _edges.push_back(tmm_edge(_triangles.end(), _triangles.end(), _triangles.end(), v0, v1));
            return --_edges.end();
        }

        tmm_triangle_iter add_triangle(const tmm_vertex_iter &v0, const tmm_vertex_iter &v1, const tmm_vertex_iter &v2)
        {
            _triangles.push_back(tmm_triangle(_edges.end(), _edges.end(), _edges.end(), v0, v1, v2));
            return --_triangles.end();
        }

        tmm_triangle_iter add_triangle(const tmm_edge_iter &e0, const tmm_edge_iter &e1, const tmm_edge_iter &e2, const tmm_vertex_iter &v0, const tmm_vertex_iter &v1, const tmm_vertex_iter &v2)
        {
            _triangles.push_back(tmm_triangle(e0, e1, e2, v0, v1, v2));
            return --_triangles.end();
        }

        void clear()
        {
            _vertices.clear();
            _edges.clear();
            _triangles.clear();
        }

        void points_and_triangles(std::vector<point_t> *const points, std::vector<point_ti<>> *const triangles)
        {
            /* Reserve space */
            points->reserve(_vertices.size());
            triangles->reserve(_triangles.size());

            /* Push and re-index vertices */
            int idx = 0;
            for (auto &v : _vertices)
            {
                points->push_back((point_t(v.position()) / _scale_inv) + _offset);
                v.id(idx++);
            }

            /* Push triangles */
            for (const auto &t : _triangles)
            {
                triangles->emplace_back(t.vertex(0)->id(), t.vertex(1)->id(), t.vertex(2)->id());
            }
        }

        bool check_consistency()
        {
            for (const auto &e : _edges)
            {
                for (int f = 0; f < 2; ++f)
                {
                    if (e.triangle(f) == _triangles.end())
                    {
                        return false;
                    }
                }
            }

            for (auto t = _triangles.begin(); t != _triangles.end(); ++t)
            {
                for (int e = 0; e < 3; ++e)
                {
                    int found = 0;
                    for (int k = 0; k < 2; ++k)
                    {
                        if (t->edge(e)->triangle(k) == t)
                        {
                            ++found;
                        }
                    }

                    if (found != 1)
                    {
                        return false;
                    }
                }
            }

            return true;
        }

        int clean_up(std::vector<tmm_triangle_iter> *const delete_triangles, std::vector<tmm_edge_iter> *const update_edges, std::vector<tmm_edge_iter> *const delete_edges, const tmm_vertex_iter &v_end)
        {
            clean_edges(update_edges, delete_edges);
            clean_triangles(delete_triangles);
            return clean_vertices(v_end);
        }

        void clean_edges(std::vector<tmm_edge_iter> *const update_edges, std::vector<tmm_edge_iter> *const delete_edges)
        {
            /* Update edge triangles to the new triangles */
            for (auto &e : (*update_edges))
            {
                if (e->new_face() != _triangles.end())
                {
                    if (e->triangle(0)->visible())
                    {
                        e->triangle(e->new_face(), 0);
                    }
                    else
                    {
                        e->triangle(e->new_face(), 1);
                    }
                    e->new_face(_triangles.end());
                }           
            }

            /* Deleted edges marked for deletion */
            for (const auto &e : (*delete_edges))
            {
                _edges.erase(e);
            }

            delete_edges->clear();
            update_edges->clear();
        }

        void clean_triangles(std::vector<tmm_triangle_iter> *const delete_triangles)
        {
            /* Deleted triangles marked for deletion */
            for (const auto &t : (*delete_triangles))
            {
                _triangles.erase(t);
            }

            delete_triangles->clear();
        }

        int clean_vertices(const tmm_vertex_iter &end)
        {
            /* Find vertices still attached to an edge */
            for (auto &e : _edges)
            {
                e.vertex(0)->on_hull(true);
                e.vertex(1)->on_hull(true);
            }

            /* Delete all the vertices that have been processed but are not on the hull */
            int deleted = 0;
            auto v = _vertices.begin();
            do 
            {
                if (!v->on_hull())
                {
                    v = _vertices.erase(v);
                    ++deleted;
                }
                else
                {
                    v->duplicate(_edges.end());
                    v->on_hull(false);
                    ++v;
                }
            } while ((v != end) && (v != _vertices.end()));

            return deleted;
        }

        /* Create a new triangle */
        tmm_triangle_iter make_face(const tmm_vertex_iter &v0, const tmm_vertex_iter &v1, const tmm_vertex_iter &v2, const tmm_triangle_iter &fold)
        {
            /* Create new edges if they dont exist */
            int tri_idx = 0;
            tmm_edge_iter e0;
            tmm_edge_iter e1;
            tmm_edge_iter e2;
            if (fold == _triangles.end())
            {
                e0 = add_edge(v0, v1);
                e1 = add_edge(v1, v2);
                e2 = add_edge(v2, v0);            
            }
            /* Back facing, reuse existing edges, in reverse order */
            else
            {
                e0 = fold->edge(2);
                e1 = fold->edge(1);
                e2 = fold->edge(0);
                e0->vertex(v0, 0);
                e0->vertex(v1, 1);
                e1->vertex(v1, 0);
                e1->vertex(v2, 1);
                e2->vertex(v2, 0);
                e2->vertex(v0, 1);
                tri_idx = 1;
            }

            /* Build triangle */
            const tmm_triangle_iter t = add_triangle(e0, e1, e2, v0, v1, v2);    

            /* Link edges to the face */
            e0->triangle(t, tri_idx);
            e1->triangle(t, tri_idx);
            e2->triangle(t, tri_idx);
            return t;
        }

        tmm_triangle_iter make_cone_face(const tmm_edge_iter &e, const tmm_vertex_iter &p)
        {
            /* Create two new edges if they don't already exist */
            tmm_edge_iter new_edges[2];
            for (int i = 0; i < 2; ++i)
            {
                new_edges[i] = e->vertex(i)->duplicate();
                if (new_edges[i] == _edges.end())
                {
                    new_edges[i] = add_edge(e->vertex(i), p);
                    e->vertex(i)->duplicate(new_edges[i]);
                }
            }

            /* Make the new triangle */
            const tmm_triangle_iter new_tri = make_ccw(e, new_edges[0], new_edges[1], p);

            /* Attach new edges to the triangle */
            for (int i = 0; i < 2; ++i)
            {
                for (int j = 0; j < 2; ++j)
                {
                    if (new_edges[i]->triangle(j) == _triangles.end())
                    {
                        new_edges[i]->triangle(new_tri, j);
                        break;
                    }
                }
            }

            return new_tri;
        }

    private :
        /* Make sure a triangle is counter clock wise */
        tmm_triangle_iter make_ccw(const tmm_edge_iter &e, const tmm_edge_iter &e0, const tmm_edge_iter &e1, const tmm_vertex_iter &v)
        {
            /* The visible triangle adjacent to e */
            tmm_triangle_iter fv; 
            if (e->triangle(0)->visible())
            {
                fv = e->triangle(0);
            }
            else
            {
                fv = e->triangle(1);
            }
            assert(fv != _triangles.end());

            /* Make sure the edges vertices have the same orientation on th enew triangle */
            int v_idx;
            const tmm_vertex_iter v0 = e->vertex(0);
            const tmm_vertex_iter v1 = e->vertex(1);
            for (v_idx = 0; fv->vertex(v_idx) != v0; ++v_idx);
            
            if (fv->vertex((v_idx + 1) % 3) != e->vertex(1))
            {
                return add_triangle(e, e0, e1, v1, v0, v);
            }
            else
            {
                return add_triangle(e0, e, e1, v0, v1, v);
            }
        }

        std::list<tmm_vertex, allocator<tmm_vertex>>        _vertices;
        std::list<tmm_edge, allocator<tmm_edge>>            _edges;
        std::list<tmm_triangle, allocator<tmm_triangle>>    _triangles;
        const point_t                                       _offset;
        const point_t                                       _scale_inv;
};
}; /* namespace raptor_convex_decomposition */
