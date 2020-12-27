#pragma once

/* Standard Headers */
#include <cmath>
#include <iostream>
#include <numeric>
#include <vector>

/* Boost headers */
#include "boost/optional.hpp"

/* Common headers */
#include "point2d.h"


class dac_convex_hull_3d;
bool save_vrml(const dac_convex_hull_3d &hull, const std::string &file = "test.wrl");

class projected_vertex
{
    public:
        projected_vertex(const point2d<long> &p, const size_t i) : _p(p), _i(i) { }

        bool operator<(const projected_vertex &rhs) const
        {
            if (_p.x != rhs._p.x)
            {
                return _p.x < rhs._p.x;
            }

            return _p.y < rhs._p.y;
        }

        bool operator==(const projected_vertex &rhs) const
        {
            return _p == rhs._p;
        }

        const point2d<long> &position() const { return _p; }
        size_t index() const { return _i; }

    private:
        point2d<long>   _p; /* The 2d project vertex                */
        size_t          _i; /* The index of the original vertex     */
};

class projected_hull
{
    public:
        projected_hull(std::shared_ptr<std::vector<projected_vertex>> &scratch, projected_vertex *const begin, projected_vertex *const end) : 
            _scratch(scratch), _begin(begin), _end(end) {  }

        size_t size() const { return _end - _begin; }

        projected_vertex &at(const size_t i) { return *(_begin + i); }
        projected_vertex &operator[](const size_t i) { return at(i); }

        const projected_vertex &at(const size_t i) const { return *(_begin + i); }
        const projected_vertex &operator[](const size_t i) const { return at(i); }

        projected_hull& merge(const projected_hull &rhs)
        {
            std::cout << "Merging 2d hull " << std::endl;
            for (size_t i = 0; i < size(); ++i)
            {
                std::cout << at(i).position() << std::endl;
            }

            std::cout << "Versus " << std::endl;
            for (size_t i = 0; i < rhs.size(); ++i)
            {
                std::cout << rhs.at(i).position() << std::endl;
            }

            /* Find the closest points */
            const size_t a_idx = std::distance(_begin, std::max_element(_begin, _end, [](const auto &l, const auto &r){ return (l.position().x < r.position().x) || ((l.position().x == r.position().x) && (l.position().y < r.position().y));}));
            const size_t b_idx = std::distance(rhs._begin, std::min_element(rhs._begin, rhs._end, [](const auto &l, const auto &r){ return l.position().x < r.position().x;}));
            std::cout << "Starting from left " << at(a_idx).position() << ", right " << rhs.at(b_idx).position() << std::endl;

            /* Move line up to find top tangent */
            auto [a_top, b_top] = find_tangent_top(rhs, a_idx, b_idx);
            std::cout << "Got top tangent left " << at(a_top).position() << ", right " << rhs.at(b_top).position() << std::endl;

            /* Move line down to find bottom tangent */
            auto [a_bot, b_bot] = find_tangent_bottom(rhs, a_idx, b_idx);
            std::cout << "Got bottom tangent left " << at(a_bot).position() << ", right " << rhs.at(b_bot).position() << std::endl;

            /* Merge vertices */
            /* Copy what I can into space left in a */
            a_top = increment(a_top);
            bool wrote_last_b = false;
            const size_t b_end = rhs.increment(b_bot);
            if (a_top != a_bot)
            {
                do
                {
                    at(a_top) = rhs.at(b_top);
                    std::cout << "Copied " << at(a_top).position() << std::endl;
                    wrote_last_b = (b_top == b_bot);
                    a_top = increment(a_top);
                    b_top = rhs.increment(b_top);
                } while ((a_top != a_bot) && (b_top != b_end));
            }

            /* Need more space */
            if ((a_top == a_bot) && !wrote_last_b)
            {
                const size_t scratch_size = std::distance(_begin + a_top, _end);
                std::cout << "Need more space, scratch size " << scratch_size << std::endl;
                std::move(_begin + a_top, _end, _scratch->begin());  /* Make space, it all needs shuffling up anyway */
                if (b_top <= b_bot)
                {
                    std::cout << "No wrap around required, moving " << (b_bot - b_top) << std::endl;
                    std::move(rhs._begin + b_top, rhs._begin + b_bot + 1, _begin + a_top);                  /* Move down new data   */
                    for (auto *i = (rhs._begin + b_top); i < (rhs._begin + b_bot); ++i)
                    {
                        std::cout << "Moving down " << i->position() << std::endl;
                    }

                    projected_vertex *move_end = _begin + a_top + (b_bot - b_top) + 1;
                    std::move(_scratch->begin(), _scratch->begin() + scratch_size, move_end);           /* Return old data      */
                    _end = move_end + scratch_size;
                }
                /* Wrap around new two inserts */
                else if (b_top > b_bot)
                {
                    /* Oh dear, need to move out the lower data as well */
                    std::cout << "Wrap around required" << std::endl;
                    std::move(rhs._begin, rhs._begin + b_end, _scratch->begin() + scratch_size);

                    std::move(rhs._begin + b_top, rhs._end, _begin + a_top);                            /* Move down new data   */
                    std::move(_scratch->begin() + scratch_size, _scratch->begin() + scratch_size + b_end, _begin + a_top + rhs.size() - b_top);     /* Move down new data   */

                    projected_vertex *move_end = _begin + a_top + rhs.size() - b_top + b_end;
                    std::move(_scratch->begin(), _scratch->begin() + scratch_size, move_end);           /* Return old data      */
                    _end = move_end + scratch_size;
                }
            }
            /* Too much space */
            else if (b_top == b_end)
            {
                if (a_top < a_bot)
                {
                    std::move(_begin + a_bot, _end, _begin + a_top);
                    _end = _begin + a_top + std::distance(_begin + a_bot, _end);
                }
                /* Wrap around cant just update the ends */
                else if (a_top > a_bot)
                {
                    _end = _begin + a_top;
                    _begin = _begin + a_bot;
                }
            }

            std::cout << "Merged " << std::endl;
            for (size_t i = 0; i < size(); ++i)
            {
                std::cout << at(i).position() << std::endl;
            }
            std::cout << std::endl;

            return *this;
        }

        std::pair<size_t, size_t> find_tangent(const projected_hull &rhs, const long dir) const
        {
            std::cout << "Finding tangent " << std::endl;
            for (size_t i = 0; i < size(); ++i)
            {
                std::cout << at(i).position() << std::endl;
            }

            std::cout << "Versus " << std::endl;
            for (size_t i = 0; i < rhs.size(); ++i)
            {
                std::cout << rhs.at(i).position() << std::endl;
            }

            const size_t a_idx = std::distance(_begin, std::max_element(_begin, _end, [](const auto &l, const auto &r){ return (l.position().x < r.position().x) || ((l.position().x == r.position().x) && (l.position().y < r.position().y));}));
            const size_t b_idx = std::distance(rhs._begin, std::min_element(rhs._begin, rhs._end, [](const auto &l, const auto &r){ return l.position().x < r.position().x;}));
            return find_tangent(rhs, a_idx, b_idx, dir);
        }

        std::pair<size_t, size_t> find_tangent(const projected_hull &rhs, size_t a_idx, size_t b_idx, const long dir) const
        {
            if (dir > 0)
            {
                const auto [a_tang, b_tang] = find_tangent_top(rhs, a_idx, b_idx);
                return std::pair(at(a_tang).index(), rhs.at(b_tang).index());
            }
            else
            {
                const auto [a_tang, b_tang] = find_tangent_bottom(rhs, a_idx, b_idx);
                return std::pair(at(a_tang).index(), rhs.at(b_tang).index());
            }
        }

    private:
        std::pair<size_t, size_t> find_tangent_top(const projected_hull &rhs, size_t a_idx, size_t b_idx) const
        {
            /* Begin moving a and b index towards the tangent one at a time */
            bool advanced = true;
            while (advanced)
            {
                advanced = false;
                const size_t next_a = decrement(a_idx);
                const long wa = winding(at(a_idx).position(), rhs.at(b_idx).position(), at(next_a).position());
                if ((wa > 0) || ((wa == 0) && (magnitude_sq(rhs.at(b_idx).position() - at(next_a).position()) > magnitude_sq(rhs.at(b_idx).position() - at(a_idx).position()))))
                {
                    advanced = true;
                    a_idx = next_a;
                }

                const size_t next_b = rhs.increment(b_idx);
                const long wb = winding(at(a_idx).position(), rhs.at(b_idx).position(), rhs.at(next_b).position());
                if ((wb > 0) || ((wb == 0) && (magnitude_sq(at(a_idx).position() - rhs.at(next_b).position()) > magnitude_sq(at(a_idx).position() - rhs.at(b_idx).position()))))
                {
                    advanced = true;
                    b_idx = next_b;
                }
            }

            return std::pair(a_idx, b_idx);
        }

        std::pair<size_t, size_t> find_tangent_bottom(const projected_hull &rhs, size_t a_idx, size_t b_idx) const
        {
            /* Begin moving a and b index towards the tangent one at a time */
            bool advanced = true;
            while (advanced)
            {
                advanced = false;
                const size_t next_a = increment(a_idx);
                const long wa = -winding(at(a_idx).position(), rhs.at(b_idx).position(), at(next_a).position());
                if ((wa > 0) || ((wa == 0) && (magnitude_sq(rhs.at(b_idx).position() - at(next_a).position()) > magnitude_sq(rhs.at(b_idx).position() - at(a_idx).position()))))
                {
                    advanced = true;
                    a_idx = next_a;
                }

                const size_t next_b = rhs.decrement(b_idx);
                const long wb = -winding(at(a_idx).position(), rhs.at(b_idx).position(), rhs.at(next_b).position());
                if ((wb > 0) || ((wb == 0) && (magnitude_sq(at(a_idx).position() - rhs.at(next_b).position()) > magnitude_sq(at(a_idx).position() - rhs.at(b_idx).position()))))
                {
                    advanced = true;
                    b_idx = next_b;
                }
            }

            return std::pair(a_idx, b_idx);
        }

        long increment(long idx) const
        {
            ++idx;
            if (idx == static_cast<long>(size()))
            {
                return 0;
            }
            
            return idx;
        }

        long decrement(long idx) const
        {
            --idx;
            if (idx < 0)
            {
                return size() - 1;
            }
            
            return idx;
        }

        std::shared_ptr<std::vector<projected_vertex>> _scratch;
        projected_vertex *                             _begin;
        projected_vertex *                             _end;
};

class face;
class vertex;

class edge
{
    public:
        edge(vertex *v0, vertex *v1) : _v0(std::min(v0, v1)), _v1(std::max(v0, v1)), _done(false) { }

        bool done() const { return _done; }
        void done(const bool d) { _done = d; }

        vertex * start() { return _v0; }
        void start(vertex *v) { _v0 = v; }

        vertex *end() { return _v1; }
        void end(vertex *v) { _v1 = v; }

        bool contains(const vertex *const v)
        {
            return (v == _v0) || (v == _v1);
        }

        vertex* next_vertex(const vertex *const v)
        {
            if (v == _v0)
            {
                return _v1;
            }

            assert(v == _v1);
            return _v0;
        }

        void update(vertex *from, vertex *to)
        {
            if (from == _v0)
            {
                _v0 = to;
                return;
            }

            assert(from == _v1);
            _v1 = to;
        }

        std::shared_ptr<face> left_face(const vertex *const v)
        {
            if (v == _v0)
            {
                return _f0;
            }

            assert(v == _v1);
            return _f1;
        }

        void left_face(const vertex *const v, const std::shared_ptr<face> &f)
        {
            if (v == _v0)
            {
                _f0 = f;
                return;
            }

            assert(v == _v1);
            _f1 = f;
        }

        std::shared_ptr<face> right_face(const vertex *const v)
        {
            if (v == _v0)
            {
                return _f1;
            }

            assert(v == _v1);
            return _f0;
        }

        void right_face(const vertex *const v, const std::shared_ptr<face> &f)
        {
            if (v == _v0)
            {
                _f1 = f;
                return;
            }

            assert(v == _v1);
            _f0 = f;
        }

        void update(const std::shared_ptr<face> &from, const std::shared_ptr<face> &to)
        {
            if (from == _f0)
            {
                _f0 = to;
                return;
            }

            if (from == _f1)
            {
                _f1 = to;
            }
        }

        bool check()
        {
            return (_v0 != nullptr) && (_v1 != nullptr) && (_f0 != nullptr) && (_f1 != nullptr);
        }

    private:
        vertex *                _v0;
        vertex *                _v1;
        std::shared_ptr<face>   _f0;
        std::shared_ptr<face>   _f1;
        bool                    _done;  /* A flag for graph search */
};

class vertex
{
    public:
        vertex() = default;
        vertex(const point_ti<long> &p) : _p(p), _done(false), _removed(false) { }

        point_ti<long> position() const { return _p; }
        void position(const point_ti<long> &p) { _p = p; }

        bool done() const { return _done; }
        void done(const bool d) { _done = d; }

        bool removed() const { return _removed; }
        void remove() { _removed = true; }

        int edges() const { return _edges.size(); }
        std::shared_ptr<edge> edge_at(const int i) const { return _edges[i]; }
        void edge_at(const std::shared_ptr<edge> &e, const int i)
        {
            _edges[i] = e;
        }

        void add_edge(const std::shared_ptr<edge> &e)
        {
            _edges.emplace_back(e);
        }

        void erase_edge(const int i)
        {
            _edges.erase(_edges.begin() + i);
        }

        void erase_edge(const std::shared_ptr<edge> &e)
        {
            if (const auto &fit = std::find(_edges.begin(), _edges.end(), e); fit != _edges.end())
            {
                _edges.erase(fit);
            }
        }

        void clear_edges()
        {
            _edges.clear();
        }

    private:
        point_ti<long>                      _p;
        std::vector<std::shared_ptr<edge>>  _edges;
        bool                                _done;
        bool                                _removed;
};

class face
{
    public:
        face(vertex *v0, vertex *v1, vertex *v2) : _vertices{v0, v1, v2} { }

        vertex * vertice(const int i) { return _vertices[i]; }
        void vertice(vertex *v, const int i) { _vertices[i] = v; }

        std::shared_ptr<edge> edge_at(const int i) const { return _edges[i]; }
        void edge_at(const std::shared_ptr<edge> &e, const int i) { _edges[i] = e; }

        std::shared_ptr<edge> next_live_edge(const std::shared_ptr<edge> &e)
        {
            std::shared_ptr<edge> next;
            if (e == _edges[0])
            {
                next = _edges[1]->done() ? _edges[2] : _edges[1];
            }
            else if (e == _edges[1])
            {
                next = _edges[2]->done() ? _edges[0] : _edges[2];
            }
            else
            {
                assert(e == _edges[2]);
                next = _edges[0]->done() ? _edges[1] : _edges[0];
            }

            if (next->done())
            {
                return nullptr;
            }
            else
            {
                return next;
            }
        }

        point_ti<long> normal() const
        {
            const point_ti<long> a(_vertices[0]->position());
            const point_ti<long> e0(_vertices[1]->position() - a);
            const point_ti<long> e1(_vertices[2]->position() - a);
            return cross_product(e1, e0);
        }

        bool check() const
        {
            if ((_edges[0]->left_face(_vertices[0]).get() != this) || (_edges[1]->left_face(_vertices[1]).get() != this) || (_edges[2]->left_face(_vertices[2]).get() != this))
            {
                return false;
            }

            if (!_edges[0]->contains(_vertices[0]) || !_edges[1]->contains(_vertices[1]) || !_edges[2]->contains(_vertices[2]))
            {
                return false;
            }

            return true;
        }

        bool contains(const vertex *const v) const
        {
            return (_vertices[0] == v) || (_vertices[1] == v) || (_vertices[2] == v);
        }

    private:
        vertex *                _vertices[3];
        std::shared_ptr<edge>   _edges[3];
};

class dac_convex_hull_3d
{
    public:
        dac_convex_hull_3d(const projected_hull &proj_hull, std::vector<vertex> &vertices, const int begin, const int end) :
            _proj_hull(proj_hull),
            _vertices(vertices),
            _begin(begin),
            _end(end)
        { }

        std::vector<vertex> all_vertices() const
        {
            return _vertices;
        }

        std::vector<vertex> hull_vertices() const
        {
            std::vector<vertex> ret;
            for (int i = _begin; i < _end; ++i)
            {
                if (const auto &v = _vertices[i]; !v.removed())
                {
                    ret.emplace_back(v);
                }
            }

            return ret;
        }

        std::vector<std::tuple<int, int, int>> hull_face_indices() const
        {
            /* Find remaining vertices */
            std::vector<const vertex *> verts;
            for (int i = _begin; i < _end; ++i)
            {
                if (const auto &v = _vertices[i]; !v.removed())
                {
                    verts.emplace_back(&v);
                }
            }

            /* Find their faces */
            std::set<std::shared_ptr<face>> faces;
            for (const auto &v : verts)
            {
                for (int i = 0; i < v->edges(); ++i)
                {
                    faces.emplace(v->edge_at(i)->left_face(v));
                    faces.emplace(v->edge_at(i)->right_face(v));
                }
            }

            /* Build indices */
            std::vector<std::tuple<int, int, int>> ret;
            ret.reserve(faces.size());
            for (const auto &f : faces)
            {
                if (f == nullptr)
                {
                    continue;
                }

                ret.emplace_back(std::distance(&_vertices[0], f->vertice(0)), std::distance(&_vertices[0], f->vertice(1)), std::distance(&_vertices[0], f->vertice(2)));
            }

            return ret;
        }

        bool is_convex() const
        {
            /* Find remaining vertices */
            std::cout << "Is convex" << std::endl;
            std::vector<const vertex *> verts;
            for (int i = _begin; i < _end; ++i)
            {
                if (const auto &v = _vertices[i]; !v.removed())
                {
                    verts.emplace_back(&v);
                }
            }

            /* Find their faces */
            std::set<std::shared_ptr<face>> faces;
            for (const auto &v : verts)
            {
                for (int i = 0; i < v->edges(); ++i)
                {
                    faces.emplace(v->edge_at(i)->left_face(v));
                    faces.emplace(v->edge_at(i)->right_face(v));
                }
            }

            /*Check all faces versus all vertices for exhaustive test */
            for (const auto &f : faces)
            {
                if (f == nullptr)
                {
                    continue;
                }

                const point_ti<long> normal(f->normal());
                const point_ti<long> a(f->vertice(0)->position());
                std::cout << "Face normal " << normal << ", from edges " << (f->vertice(1)->position() - a) << " - " << (f->vertice(2)->position() - a) << std::endl;
                for (const auto *v : verts)
                {                    
                    const long dot = dot_product(normal, v->position() - a);
                    if (dot > 0)
                    {
                        std::cout << "Face " << a << " - " << f->vertice(1)->position() << " - " << f->vertice(2)->position() << " failed versus " << v->position() << std::endl << std::endl;
                        return false;
                    }
                }
            }

            std::cout << std::endl;
            return true;
        }

        dac_convex_hull_3d& merge(const dac_convex_hull_3d &rhs)
        {
            /* Save input hulls */
            std::cout << "Merge " << _merge_num << std::endl;
            save_vrml(*this, "left_" + std::to_string(_merge_num) + ".wrl");
            save_vrml(rhs, "right_" + std::to_string(_merge_num) + ".wrl");

            /* Get top tangent from 2d hull */
            const auto [a_top_idx, b_top_idx] = _proj_hull.find_tangent(rhs._proj_hull, 1);
            auto *l = &_vertices[a_top_idx];
            auto *r = &_vertices[b_top_idx];
            std::cout << "Found top vertices " << a_top_idx << " (" << l->position() << "), " << b_top_idx << " (" << r->position() << ")" << std::endl;

            /* Wrap */
            std::vector<std::shared_ptr<edge>> wrapping_edges(wrap(&l, &r));

            /* Remove hidden faces */
            remove_hidden_features(wrapping_edges, l, r);

            /* Add new edges */
            add_new_faces(wrapping_edges, l, r);

            /* Merge 2d hull */
            _proj_hull.merge(rhs._proj_hull);   /* TODO: reuse previously found tangent */
            _end = std::max(_end, rhs._end);
            _begin = std::min(_begin, rhs._begin);

            save_vrml(*this, "out_" + std::to_string(_merge_num) + ".wrl");
            ++_merge_num;
            return *this;
        }

        std::vector<std::shared_ptr<edge>> wrap(vertex **l, vertex **r)
        {
            int iter = 0;
            vertex *l_iter = *l;
            vertex *r_iter = *r;
            point_ti<long> normal_left;
            point_ti<long> normal_right;
            bool done_left = ((*l)->edges() == 0);
            bool done_right = ((*r)->edges() == 0);
            std::vector<std::shared_ptr<edge>> ret;
            std::vector<std::pair<vertex *, vertex *>> visited{{*l, *r}};
            do
            {
                std::shared_ptr<edge> next_left;
                std::shared_ptr<edge> next_right;
                boost::optional<long> area_left;
                boost::optional<long> area_right;
                if (!done_left)
                {
                    std::cout << "Searching for edge on left, done " << done_left << ", " << done_right << std::endl;
                    next_left = find_wrapping_edge(area_left, *l, *r, l_iter, r_iter, normal_left, 1, done_right);
                    if ((next_left == nullptr) || area_left)
                    {
                        std::cout << "Searching for edge on right, done " << done_left << ", " << done_right << std::endl;
                        next_right = find_wrapping_edge(area_right, *r, *l, r_iter, l_iter, normal_right, -1, done_left);
                    }
                }
                else
                {
                    std::cout << "Searching for edge on right, done " << done_left << ", " << done_right << std::endl;
                    next_right = find_wrapping_edge(area_right, *r, *l, r_iter, l_iter, normal_right, -1, done_left);
                    if ((next_right == nullptr) || area_right)
                    {
                        std::cout << "Searching for edge on left, done " << done_left << ", " << done_right << std::endl;
                        next_left = find_wrapping_edge(area_left, *l, *r, l_iter, r_iter, normal_left, 1, done_right);
                    }
                }
                
                std::shared_ptr<edge> nxt;
                assert((next_left != nullptr) || (next_right != nullptr) || !"No wrapping vertex found");
                if ((next_right != nullptr) && (!area_right || (area_right > area_left)))
                {
                    nxt = next_right;
                    normal_left = normal_right;
                    r_iter = nxt->next_vertex(r_iter);
                    std::cout << "Wrapped on right to " << r_iter->position() << ", updating normal " << normal_right << std::endl;
                    done_right = (r_iter == (*r));
                }
                else
                {
                    nxt = next_left;
                    normal_right = normal_left;
                    l_iter = nxt->next_vertex(l_iter);
                    std::cout << "Wrapped on left to " << l_iter->position() << ", updating normal " << normal_left << std::endl;
                    done_left = (l_iter == (*l));
                }

                ret.push_back(nxt);
                assert(++iter < 100);
                visited.emplace_back(l_iter, r_iter);
            } while (std::count_if(visited.begin(), visited.end(), [&l_iter, &r_iter](const auto &p) { return (p.first == l_iter) && (p.second == r_iter); }) < 2);

            std::cout << "Loop created at " << std::distance(visited.begin(), std::find(visited.begin(), visited.end(), std::make_pair(l_iter, r_iter))) << std::endl;
            ret.erase(ret.begin(), ret.begin() + std::distance(visited.begin(), std::find(visited.begin(), visited.end(), std::make_pair(l_iter, r_iter))));
            *l = l_iter;
            *r = r_iter;

            std::cout << std::endl;
            return ret;
        }

        void remove_hidden_features(const std::vector<std::shared_ptr<edge>> &edges, vertex *l, vertex *r)
        {
            /* Nothing wrapped, nothing to remove */
            if (edges.empty())
            {
                return;
            }

            /* Flag the boundary as done */
            std::cout << "Removing hidden features" << std::endl;
            for (auto &e : edges)
            {
                e->done(true);
                e->end()->done(true);
                e->start()->done(true);
            }

            /* Seach for a start vertex in the hidden regions */
            bool l_wrapped = false;
            bool r_wrapped = false;
            std::shared_ptr<edge> l_start;
            std::shared_ptr<edge> r_start;
            for (const auto &e : edges)
            {
                if (!l_wrapped && e->contains(l))
                {
                    l_wrapped = true;
                    if (e->left_face(l) != nullptr)
                    {
                        std::cout << std::distance(&_vertices[0], l) << " Left face " << e->left_face(l)->vertice(0)->position() << " to " << e->left_face(l)->vertice(1)->position() << " to " << e->left_face(l)->vertice(2)->position() << std::endl;
                        l_start = e->left_face(l)->next_live_edge(e);
                        (l_start != nullptr) && std::cout << "Got left interior edge " << l_start->start()->position() << " to " << l_start->end()->position() << std::endl;
                    }
                }
                else if (!r_wrapped && e->contains(r))
                {
                    r_wrapped = true;
                    if (e->right_face(r) != nullptr)
                    {
                        r_start = e->right_face(r)->next_live_edge(e);
                        (r_start != nullptr) && std::cout << "Got right interior edge " << r_start->start()->position() << " to " << r_start->end()->position() << std::endl;
                    }
                }
            }

            /* If we didnt wrap on one hull the only the start vertex can live */
            if (!l_wrapped)
            {
                if (l->edges() > 0)
                {
                    std::cout << "No wrapping on left selecting arbitary vertex for removal" << std::endl;
                    l_start = l->edge_at(0);
                    l->clear_edges();
                    l->done(true);
                }
            }

            if (!r_wrapped)
            {
                if (r->edges() > 0)
                {
                    std::cout << "No wrapping on right selecting arbitary vertex for removal" << std::endl;
                    r_start = r->edge_at(0);
                    r->clear_edges();
                    r->done(true);
                }
            }

            /* Clean region with a start point */
            if (l_start != nullptr)
            {
                std::cout << "Removing hidden features on left" << std::endl;
                remove_hidden_features(l_start);
            }

            if (r_start != nullptr)
            {
                std::cout << "Removing hidden features on right" << std::endl;
                remove_hidden_features(r_start);                
            }

            /* Remove boundary flag */
            l->done(false);
            r->done(false);
            for (auto &e : edges)
            {
                e->done(false);
                e->end()->done(false);
                e->start()->done(false);
            }
            std::cout << std::endl;
        }

        void add_new_faces(const std::vector<std::shared_ptr<edge>> &edges, vertex *l, vertex *r)
        {
            /* No edges wrapped means point hull that we join with an edges */
            std::cout << "Adding new faces" << std::endl;
            if (edges.empty())
            {
                assert(l->edges() == 0);
                assert(r->edges() == 0);
                auto e = std::make_shared<edge>(l, r);
                l->add_edge(e);
                r->add_edge(e);
                std::cout << "    Nothing to do" << std::endl;
                return;
            }

            std::shared_ptr<face> new_face;
            auto first_edge = std::make_shared<edge>(l, r);
            auto last_edge = first_edge;
            l->add_edge(first_edge);
            r->add_edge(first_edge);
            for (int i = 0; i < static_cast<int>(edges.size()); ++i)
            {
                /* Advance */
                auto &e = edges[i];
                if (e->contains(l))
                {
                    std::cout << "    Edge add advanced on left " << std::distance(&_vertices[0], l) << " to " << std::distance(&_vertices[0], e->next_vertex(l)) << " with right " << std::distance(&_vertices[0], r) << std::endl;
                    new_face = std::make_shared<face>(r, l, e->next_vertex(l));
                    e->left_face(l, new_face);
                    last_edge->right_face(l, new_face);
                    l = e->next_vertex(l);
                }
                else
                {
                    assert(e->contains(r) || !"Edge not advancing on either polygon");
                    std::cout << "    Edge add advanced on right " << std::distance(&_vertices[0], r) << " to " << std::distance(&_vertices[0], e->next_vertex(r)) << " with left " << std::distance(&_vertices[0], l) << std::endl;
                    new_face = std::make_shared<face>(r, l, e->next_vertex(r));
                    e->right_face(r, new_face);
                    last_edge->right_face(l, new_face);
                    r = e->next_vertex(r);
                }

                if (i < static_cast<int>(edges.size() - 1))
                {
                    auto nxt_edge = std::make_shared<edge>(l, r);
                    l->add_edge(nxt_edge);
                    r->add_edge(nxt_edge);
                    if (e->contains(l))
                    {
                        new_face->edge_at(last_edge, 0);
                        new_face->edge_at(e, 1);
                        new_face->edge_at(nxt_edge, 2);
                    }
                    else
                    {
                        new_face->edge_at(last_edge, 0);
                        new_face->edge_at(nxt_edge, 1);
                        new_face->edge_at(e, 2);
                    }

                    last_edge = nxt_edge;
                    last_edge->left_face(l, new_face);
                }
                /* All the way back to the start and the first edge needs it leading face */
                else
                {
                    if (e->contains(l))
                    {
                        new_face->edge_at(last_edge, 0);
                        new_face->edge_at(e, 1);
                        new_face->edge_at(first_edge, 2);
                    }
                    else
                    {
                        new_face->edge_at(last_edge, 0);
                        new_face->edge_at(first_edge, 1);
                        new_face->edge_at(e, 2);
                    }

                    first_edge->left_face(l, new_face);
                }
                
                assert(new_face->check() || !"Poorly constructed face edges");
            }

            std::cout << std::endl;
        }

    private:
        void remove_vertex(const std::shared_ptr<edge> &e, vertex *const v)
        {
            /* If the vertex is a wrapping vertex it need to live, but not connect to hidden feature */
            if (v->done())
            {
                std::cout << "        Erase from " << v->position() << std::endl;
                v->erase_edge(e);
            }
            /* Remove interior vertices we didnt get all ready */
            else if (!v->removed())
            {
                std::cout << "        Removing hidden features from " << v->position() << std::endl;
                v->remove();
                v->clear_edges();
            }
        }
        void remove_hidden_features(const std::shared_ptr<edge> &e)
        {
            /* Kill vertices or at least unlink from removed edges */
            std::cout << "    Removing with edge " << e->start()->position() << " to " << e->end()->position() << std::endl;
            remove_vertex(e, e->end());
            remove_vertex(e, e->start());

            /* Search for live edges through connected faces */
            e->done(true);
            if (const auto &l = e->left_face(e->start()); l != nullptr)
            {
                for (int i = 0; i < 3; ++i)
                {
                    if (const auto &n = l->edge_at(i); !n->done())
                    {
                        remove_hidden_features(n);
                    }
                    else
                    {
                        n->update(l, nullptr);
                    }
                }
            }

            if (const auto &r = e->right_face(e->start()); r != nullptr)
            {
                for (int i = 0; i < 3; ++i)
                {
                    if (const auto &n = r->edge_at(i); !n->done())
                    {
                        remove_hidden_features(n);
                    }
                    else
                    {
                        n->update(r, nullptr);
                    }
                }
            }

            /* Delete faces */
            e->left_face(e->start(), nullptr);
            e->right_face(e->start(), nullptr);
        }

        std::shared_ptr<edge> find_wrapping_edge(boost::optional<long> &area, vertex *const l_top, vertex *const r_top, vertex *const l, vertex *const r, point_ti<long> &normal, const long sign, const bool that_done) const
        {
            /* Work trough all a and b point to find the gift wrapping point */
            int guess_idx = 0;
            long max_area = -1; /* Unfortunately I cant rule out 0 area triangles */
            int max_area_idx = 0;
            bool failed = true;
            const point_ti<long> &lp = l->position();
            const point_ti<long> &rp = r->position();
            while (failed && (guess_idx < l->edges()))
            {
                /* Check the guess against the current vertices neighbours */
                failed = false;
                long area = std::numeric_limits<long>::max();
                const auto &e = l->edge_at(guess_idx);
                const auto *const guess = e->next_vertex(l);
                const point_ti<long> &gp = guess->position();
                std::cout << "    Guessing vertex " << std::distance(&_vertices[0], e->next_vertex(l)) << " at " << gp << std::endl;
                for (int i = 0; i < l->edges(); ++i)
                {
                    if (i == guess_idx)
                    {
                        continue;
                    }
                    
                    const auto &te = l->edge_at(i);
                    const auto *const test = te->next_vertex(l);
                    const point_ti<long> &tp = test->position();
                    std::cout << "    Testing this vertex " << std::distance(&_vertices[0], l->edge_at(i)->next_vertex(l)) << " at " << tp << std::endl;
                    if (!can_wrap(area, e, te, lp, rp, gp, tp, normal, sign, test == l_top, guess == l_top))
                    {
                        failed = true;
                        ++guess_idx;
                        break;
                    }
                }

                /* Check the guess against the current vertices neighbours in the other hull */
                if (!failed)
                {
                    for (int i = 0; i < r->edges(); ++i)
                    {
                        const auto &te = r->edge_at(i);
                        const auto *const test = te->next_vertex(r);
                        const point_ti<long> &tp = test->position();
                        std::cout << "    Testing that vertex " << std::distance(&_vertices[0], r->edge_at(i)->next_vertex(r)) << " at " << tp << std::endl;
                        if (!can_wrap(area, e, te, lp, rp, gp, tp, normal, sign, test == r_top, guess == l_top))
                        {
                            failed = true;
                            ++guess_idx;
                            break;
                        }
                    }
                }

                if (!failed && (area < std::numeric_limits<long>::max()))
                {
                    if (area > max_area)
                    {
                        max_area = area;
                        max_area_idx = guess_idx;
                    }

                    ++guess_idx;
                    failed = true;
                }
            }

            if (failed && (max_area > -1))
            {
                failed = false;
                area = max_area;
                guess_idx = max_area_idx;
            }

            if (!failed)
            {
                normal = cross_product(lp - rp, lp - l->edge_at(guess_idx)->next_vertex(l)->position()) * sign;
                return l->edge_at(guess_idx);
            }

            return nullptr;
        }

        bool can_wrap(long &area, const std::shared_ptr<edge> &ge, const std::shared_ptr<edge> &te, const point_ti<long> &l, const point_ti<long> &r, const point_ti<long> &guess, const point_ti<long> &test, const point_ti<long> &normal, const long sign, const bool fail_area_check, const bool succeed_area_check) const
        {
            /* Try to exclude by point outside face */
            const long volume = tetrahedron_volume(l, r, guess, test) * sign;
            if (volume < 0)
            {
                std::cout << "        Negative volume " << volume << ", cant wrap" << std::endl;
                return false;
            }
            /* Tie break for flat meshes */
            else if (volume == 0)
            {
                /* Reject edge if its faces face the opposite way to the new face */
                std::cout << "        Zero volume, extra tests left " << l << ", right " << r << ", guess " << guess << ", test " << test << std::endl;

                const point_ti<long> guess_normal(cross_product(l - r, l - guess) * sign);
                std::cout << "        Normal " << normal << ", Guess normal " << guess_normal << std::endl;
                if (const auto &[lf, rf] = std::tuple{ ge->left_face(ge->start()), ge->right_face(ge->start()) }; (lf != nullptr) && (rf != nullptr))
                {
                    const point_ti<long> lf_normal(lf->normal());
                    const point_ti<long> rf_normal(rf->normal());

                    std::cout << "        Guess left face normal " << lf_normal << std::endl;
                    std::cout << "        Guess right face normal "<< rf_normal << std::endl;
                    if ((dot_product(guess_normal, lf_normal) < 0) && (dot_product(guess_normal, rf_normal) < 0))
                    {
                        std::cout << "        Zero volume, edge used in wrong direction, cant wrap" << std::endl;
                        return false;
                    }
                }

                const point_ti<long> test_normal(cross_product(l - r, l - test) * sign);
                if (const auto &[lf, rf] = std::tuple{ te->left_face(te->start()), te->right_face(te->start()) }; (lf != nullptr) && (rf != nullptr))
                {
                    const point_ti<long> lf_normal(lf->normal());
                    const point_ti<long> rf_normal(rf->normal());

                    std::cout << "        Test left face normal " << lf_normal << std::endl;
                    std::cout << "        Test right face normal "<< rf_normal << std::endl;
                    if ((dot_product(test_normal, lf_normal) < 0) && (dot_product(test_normal, rf_normal) < 0))
                    {
                        std::cout << "        Zero volume, test edge using in wrong direction, can wrap" << std::endl;
                        return true;
                    }
                }

                /* Reject if normal is flipped, but we can find a test vertex that doesnt flip the normal */
                const bool test_flip = dot_product(test_normal, normal) <= 0;
                const bool guess_flip = dot_product(guess_normal, normal) <= 0;
                std::cout << "        Normal " << normal << ", Test normal " << test_normal << std::endl;
                if (guess_flip)
                {
                    if (!test_flip)
                    {
                        std::cout << "        Zero volume, normal flipped when test normal not, cant wrap" << std::endl;
                        return false;
                    }
                }

                /* Reject if we can find a bigger area triangle that has the same normal */
                const long test_area = triangle_area22(l, r, test);
                const long guess_area = triangle_area22(l, r, guess);
                std::cout << "        Zero volume, guess area " << guess_area << ", updating area " << area << ", test area " << test_area << std::endl;
                // if (!no_area_check && (guess_flip == test_flip))
                // {
                //     const long test_area = triangle_area22(l, r, test);
                //     const long guess_area = triangle_area22(l, r, guess);
                //     std::cout << "        Test area " << test_area << ", guess area " << guess_area << std::endl;
                //     if (test_area > guess_area)
                //     {
                //         std::cout << "        Zero volume, larger test area, cant wrap" << std::endl;
                //         return false;
                //     }
                // }

                if (fail_area_check && !succeed_area_check  && (guess_flip == test_flip))
                {
                    std::cout << "        Zero volume, force fail area test, cant wrap" << std::endl;
                    return false;
                }

                area = std::min(area, guess_area);
            }

            std::cout << "        Positive volume " << volume << ", can wrap" << std::endl;
            return true;
        }

        projected_hull          _proj_hull;
        std::vector<vertex> &   _vertices;
        int                     _begin;
        int                     _end;
        static int              _merge_num;
};


bool save_vrml(const dac_convex_hull_3d &hull, const std::string &file)
{
    std::ofstream fout(file);
    if (!fout.is_open())
    {
        std::cout << "Couldnt open file" << std::endl;
        return false;
    }

    // const auto hull_vertices(hull.hull_vertices());
    const auto hull_vertices(hull.all_vertices());
    if (hull_vertices.empty())
    {
        std::cout << "No vertices" << std::endl;
        return false;
    }

    const auto hull_face_indices(hull.hull_face_indices());
    if (hull_face_indices.empty())
    {
        std::cout << "No faces" << std::endl;
        return false;
    }

    fout.setf(std::ios::fixed, std::ios::floatfield);
    fout.setf(std::ios::showpoint);
    fout.precision(6);
    fout << "#VRML V2.0 utf8" << std::endl;
    fout << "" << std::endl;
    fout << "# Vertices: " << hull_vertices.size() << std::endl;
    fout << "# Triangles: " << hull_face_indices.size() << std::endl;
    fout << "" << std::endl;
    fout << "Group {" << std::endl;
    fout << "    children [" << std::endl;
    fout << "        Shape {" << std::endl;
    fout << "            appearance Appearance {" << std::endl;
    fout << "                material Material {" << std::endl;
    fout << "                    diffuseColor 0.5  0.5  0.5" << std::endl;
    fout << "                    ambientIntensity 0.3" << std::endl;
    fout << "                    specularColor 0 0 0" << std::endl;
    fout << "                    emissiveColor 0 0 0" << std::endl;
    fout << "                    shininess 0" << std::endl;
    fout << "                    transparency 0" << std::endl;
    fout << "                }" << std::endl;
    fout << "            }" << std::endl;
    fout << "            geometry IndexedFaceSet {" << std::endl;
    fout << "                ccw TRUE" << std::endl;
    fout << "                solid TRUE" << std::endl;
    fout << "                convex TRUE" << std::endl;
    fout << "                coord DEF co Coordinate {" << std::endl;
    fout << "                    point [" << std::endl;
    for (const auto &v : hull_vertices)
    {
        fout << "                        " << v.position() << "," << std::endl;
    }
    fout << "                    ]" << std::endl;
    fout << "                }" << std::endl;

    fout << "                coordIndex [ " << std::endl;
    for (const auto &[v0, v1, v2] : hull_face_indices)
    {
        fout << "                        " << v0 << ", "<< v1 << ", "<< v2 << ", -1," << std::endl;
    }
    fout << "                ]" << std::endl;
    fout << "            }" << std::endl;
    fout << "        }" << std::endl;
    fout << "    ]" << std::endl;
    fout << "}" << std::endl;

    return true;
}

void project_points(std::vector<projected_vertex> &proj_vertices, std::vector<vertex> &vertices)
{
    /* Sort vertices by x then y then z */
    std::sort(vertices.begin(), vertices.end(), [](const auto &lhs, const auto &rhs)
    {
        const auto &lp = lhs.position();
        const auto &rp = rhs.position();
        if (lp.x != rp.x)
        {
            return lp.x < rp.x;
        }

        if (lp.y != rp.y)
        {
            return lp.y < rp.y;
        }

        return lp.z < rp.z;
    });

    /* Remove all but the most extreme z for each x, y plane. This also removes any co-incident points */
    int wr_idx = 0;
    vertex *lowest = &vertices[0];
    vertex *highest = &vertices[0];
    for (int i = 1; i < static_cast<int>(vertices.size()); ++i)
    {
        const auto &lp = lowest->position();
        const auto &p = vertices[i].position();
        std::cout << "Sorted to " << p << std::endl;
        if ((p.x == lp.x) && (p.y == lp.y))
        {
            if (p.z != lp.z)
            {
                highest = &vertices[i];
            }
        }
        else
        {
            vertices[wr_idx++] = *lowest;
            if (highest != lowest)
            {
                vertices[wr_idx++] = *highest;
            }

            lowest = &vertices[i];
            highest = &vertices[i];
        }
    }

    vertices[wr_idx++] = *lowest;
    if (highest != lowest)
    {
        vertices[wr_idx++] = *highest;
    }

    vertices.erase(vertices.begin() + wr_idx, vertices.end());

    /* Create projected vertices in x, y */
    int idx = 0;
    proj_vertices.reserve(vertices.size());
    std::transform(vertices.begin(), vertices.end(), std::back_inserter(proj_vertices), [&idx](const auto &v)
    {
        std::cout << "Projecting from " << v.position() << ", to " << v.position().x << ", " << v.position().y << std::endl;
        return projected_vertex(point2d<long>(v.position().x, v.position().y), idx++);
    });
}

projected_hull build(std::shared_ptr<std::vector<projected_vertex>> &scratch, std::vector<projected_vertex> &proj_vertices, const int begin, const int end)
{
    /* Small enough make a hull to start merging */
    const int size = end - begin;
    if (size < 4)
    {
        /* Check winding for 2d hull triangles */
        if (size == 3)
        {
            /* Flip incorrect winding */
            const long w = winding(proj_vertices[begin].position(), proj_vertices[begin + 1].position(), proj_vertices[begin + 2].position());
            if (w > 0)
            {
                std::swap(proj_vertices[begin], proj_vertices[begin + 1]);
            }
        }

        return projected_hull(scratch, &proj_vertices[begin], &proj_vertices[end]);
    }

    /* Recurse */
    const int mid = begin + (size >> 1);
    auto left(build(scratch, proj_vertices, begin, mid));
    const auto &right = build(scratch, proj_vertices, mid, end);

    /* Return merged */
    return left.merge(right);
}

void create_face(vertex *v0, vertex *v1, vertex *v2, const std::shared_ptr<edge> &e0, const std::shared_ptr<edge> &e1, const std::shared_ptr<edge> &e2)
{
    auto f = std::make_shared<face>(v0, v1, v2);

    /* Add edges to face */
    f->edge_at(e0, 0);
    f->edge_at(e1, 1);
    f->edge_at(e2, 2);

    /* Add face to edges */
    e0->left_face(v0, f);
    e1->left_face(v1, f);
    e2->left_face(v2, f);

    /* Check face */
    assert(f->check());
}

std::shared_ptr<edge> create_edge(vertex *v0, vertex *v1)
{
    /* Check vertices*/
    assert(v0 != nullptr);
    assert(v1 != nullptr);
    assert(v0 != v1);

    const auto &e = std::make_shared<edge>(v0, v1);
    v0->add_edge(e);
    v1->add_edge(e);
    return e;
}

void create_triangles(vertex *const v0, vertex *const v1, vertex *const v2)
{
    /* Create edges */
    const auto &e0 = create_edge(v0, v1);
    const auto &e1 = create_edge(v1, v2);
    const auto &e2 = create_edge(v2, v0);

    /* Create left face */
    create_face(v0, v1, v2, e0, e1, e2);

    /* Create right face */
    create_face(v0, v2, v1, e2, e1, e0);
}

dac_convex_hull_3d build(std::shared_ptr<std::vector<projected_vertex>> &scratch, std::vector<vertex> &vertices, std::vector<projected_vertex> &proj_vertices, const int begin, const int end)
{
    /* Small enough make a hull to start merging */
    const int size = end - begin;
    if (size < 4)
    {
        if (size > 2)
        {
            /* Create triangle */
            create_triangles(&vertices[begin], &vertices[begin + 1], &vertices[begin + 2]);
        }
        else if (size > 1)
        {
            /* Create edge */
            create_edge(&vertices[begin], &vertices[begin + 1]);
        }

        /* Check winding for 2d hull triangles */
        int proj_end = end;
        int proj_begin = begin;
        if (size == 3)
        {
            /* Flip incorrect winding */
            const long w = winding(proj_vertices[proj_begin].position(), proj_vertices[proj_begin + 1].position(), proj_vertices[proj_begin + 2].position());
            if (w > 0)
            {
                std::swap(proj_vertices[proj_begin], proj_vertices[proj_begin + 1]);
            }
            /* Crush co-linear points */
            else if (w == 0)
            {
                proj_vertices[proj_begin + 1] = proj_vertices[proj_begin + 2];
                --proj_end;
            }
        }
        /* Crush co-incident points */
        else if (size == 2)
        {
            if (proj_vertices[proj_begin] == proj_vertices[proj_begin + 1])
            {
                // proj_vertices[proj_begin] = proj_vertices[proj_begin + 1];
                --proj_end;
            }
        }

        return dac_convex_hull_3d(projected_hull(scratch, &proj_vertices[proj_begin], &proj_vertices[proj_end]), vertices, begin, end);
    }

    /* This is a busy x co-ordinate, lets 2d hull it a bit */
    if (vertices[begin].position().x == vertices[end - 1].position().x)
    {
        /* Reproject to y-z */
        int idx = begin;
        std::cout << std::endl << "Building reprojected 2d hull" << std::endl;
        std::transform(&vertices[begin], &vertices[end], &proj_vertices[begin], [&idx](const auto &v)
        {
            std::cout << "Re-projecting from " << v.position() << ", to " << v.position().y << ", " << v.position().z << std::endl;
            return projected_vertex(point2d<long>(v.position().y, v.position().z), idx++);
        });

        /* Build 2d hull in y-z */
        auto yz_hull = build(scratch, proj_vertices, begin, end);

        /* Convert 2d to 3d hull */
        /* Find surviving vertices, track the most extreme y vertices at the same time */
        int max_y_idx = 0;
        int min_y_idx = 0;
        std::vector<vertex> tmp(yz_hull.size());
        for (int i = 0; i < static_cast<int>(yz_hull.size()); ++i)
        {
            tmp[i] = vertices[yz_hull[i].index()];
            if (tmp[i].position().y > tmp[max_y_idx].position().y)
            {
                max_y_idx = i;
            }

            if (tmp[i].position().y < tmp[min_y_idx].position().y)
            {
                min_y_idx = i;
            }
        }
        std::move(&tmp[0], &tmp[yz_hull.size()], &vertices[begin]);
        for (int i = (begin + yz_hull.size()); i < end; ++i)
        {
            vertices[i].remove();
        }

        /* Reproject the extreme y vertices */
        proj_vertices[begin] = projected_vertex(point2d<long>(tmp[max_y_idx].position().x, tmp[max_y_idx].position().y), begin + max_y_idx);
        proj_vertices[begin + 1] = projected_vertex(point2d<long>(tmp[min_y_idx].position().x, tmp[min_y_idx].position().y), begin + min_y_idx);

        /* Build edges and faces */
        std::shared_ptr<edge> back_last;
        std::shared_ptr<edge> front_last;
        vertex *const v0 = &vertices[begin];
        /* First triangle with 2 outside edges */
        if (yz_hull.size() > 3)
        {
            /* Create edges */
            vertex *v1 = &vertices[begin + 1];
            vertex *v2 = &vertices[begin + 2];
            std::cout << "Building first face " << v0->position() << " to " << v1->position() << " to " << v2->position() << std::endl;
            const auto &e0 = create_edge(v0, v1);
            const auto &e1 = create_edge(v1, v2);
            front_last = create_edge(v2, v0);
            back_last = create_edge(v2, v0);

            /* Create front and back faces */
            create_face(v0, v1, v2, e0, e1, front_last);
            create_face(v0, v2, v1, back_last, e1, e0);
        }
        else if (yz_hull.size() == 3)
        {
            create_triangles(v0, &vertices[begin + 1], &vertices[begin + 2]);
        }
        else if (yz_hull.size() == 2)
        {
            create_edge(v0, &vertices[begin + 1]);
        }

        /* Middle triangles with 1 outside edge */
        for (int i = begin + 3; i < (begin + static_cast<int>(yz_hull.size()) - 1); ++i)
        {
            /* Create edges */
            vertex *const v_last = &vertices[i - 1];
            vertex *const v_next = &vertices[i];
            std::cout << "Building middle face " << v0->position() << " to " << v_last->position() << " to " << v_next->position() << std::endl;

            const auto &e0 = create_edge(v_last, v_next);
            const auto &e1 = create_edge(v_next, v0);
            const auto &e2 = create_edge(v_next, v0);

            /* Create front and back faces */
            create_face(v0, v_last, v_next, front_last, e0, e1);
            create_face(v0, v_next, v_last, e2, e0, back_last);

            front_last = e1;
            back_last = e2;
        }

        /* Last triangle with 1 outside edges */
        if (yz_hull.size() > 3)
        {
            /* Create edges */
            vertex *const vm2 = &vertices[begin + (yz_hull.size() - 2)];
            vertex *const vm1 = &vertices[begin + (yz_hull.size() - 1)];
            std::cout << "Building last face from " << v0->position() << " to " << vm2->position() << " to " << vm1->position() << std::endl;

            const auto &e0 = create_edge(vm2, vm1);
            const auto &e1 = create_edge(vm1, v0);

            /* Create front and back faces */
            create_face(v0, vm2, vm1, front_last, e0, e1);
            create_face(v0, vm1, vm2, e1, e0, back_last);
        }

        return dac_convex_hull_3d(projected_hull(scratch, &proj_vertices[begin], &proj_vertices[begin + 1]), vertices, begin, begin + yz_hull.size());
    }

    /* Recurse */
    int mid = begin + (size >> 1);
    while (vertices[mid].position().x == vertices[mid - 1].position().x)
    {
        if (vertices[mid].position().x == vertices[end - 1].position().x)
        {
            --mid;
        }
        else
        {
            ++mid;
        }
    }

    auto left(build(scratch, vertices, proj_vertices, begin, mid));
    const auto &right = build(scratch, vertices, proj_vertices, mid, end);

    /* Return merged */
    return left.merge(right);
}

dac_convex_hull_3d build(std::vector<projected_vertex> &proj_vertices, std::vector<vertex> &vertices)
{
    /* Clean up the points a bit */
    project_points(proj_vertices, vertices);

    /* Recurse */
    auto scratch(std::make_shared<std::vector<projected_vertex>>(vertices.size(), projected_vertex(point2d<long>(0, 0), 0)));
    return build(scratch, vertices, proj_vertices, 0, proj_vertices.size());
}