#pragma once


/* Standard headers */
#include <map>
#include <numeric>
#include <queue>
#include <set>
#include <vector>

/* Boost headers */

/* Common headers */
#include "point_t.h"

/* Mesh decimation headers */
#include "mesh_decimation_options.h"


namespace raptor_mesh_decimation
{
template<class T>
class symetric_matrix4
{
    public:
        explicit symetric_matrix4(const T data)
        {
            std::fill(&_data[0], &_data[10], data);
        }

        symetric_matrix4(std::initializer_list<T> data)
        {
            std::copy(data.begin(), data.end(), &_data[0]);
        }

        symetric_matrix4(const T a, const T b, const T c, const T d)
        {
            _data[0] = a * a;
            _data[1] = a * b;
            _data[2] = a * c;
            _data[3] = a * d;

            _data[4] = b * b;
            _data[5] = b * c;
            _data[6] = b * d;

            _data[7] = c * c;
            _data[8] = c * d;

            _data[9] = d * d;
        }

        symetric_matrix4& operator+=(const symetric_matrix4 &rhs)
        {
            for (int i = 0; i < 10; ++i)
            {
                _data[i] += rhs._data[i];
            }

            return *this;
        }

        symetric_matrix4& operator-=(const symetric_matrix4 &rhs)
        {
            for (int i = 0; i < 10; ++i)
            {
                _data[i] -= rhs._data[i];
            }

            return *this;
        }

        symetric_matrix4 operator+(const symetric_matrix4 &rhs) const
        {
            return 
            {
                _data[0] + rhs._data[0],
                _data[1] + rhs._data[1],
                _data[2] + rhs._data[2],
                _data[3] + rhs._data[3],
                _data[4] + rhs._data[4],
                _data[5] + rhs._data[5],
                _data[6] + rhs._data[6],
                _data[7] + rhs._data[7],
                _data[8] + rhs._data[8],
                _data[9] + rhs._data[9]
            };
        }

        symetric_matrix4 operator-(const symetric_matrix4 &rhs) const
        {
            return 
            {
                _data[0] - rhs._data[0],
                _data[1] - rhs._data[1],
                _data[2] - rhs._data[2],
                _data[3] - rhs._data[3],
                _data[4] - rhs._data[4],
                _data[5] - rhs._data[5],
                _data[6] - rhs._data[6],
                _data[7] - rhs._data[7],
                _data[8] - rhs._data[8],
                _data[9] - rhs._data[9]
            };
        }

        T determinant(const int i11, const int i12, const int i13, const int i21, const int i22, const int i23, const int i31, const int i32, const int i33) const
        {
            return  (_data[i11] * ((_data[i22] * _data[i33]) - (_data[i23] * _data[i32]))) +
                    (_data[i12] * ((_data[i23] * _data[i31]) - (_data[i21] * _data[i33]))) +
                    (_data[i13] * ((_data[i21] * _data[i32]) - (_data[i22] * _data[i31])));
        }

        T quadric(const point_t &p) const
        {
            return  (_data[0] * p.x * p.x) + (2.0f * _data[1] * p.x * p.y) + (2.0f * _data[2] * p.x * p.z) + (2.0f * _data[3] * p.x) + 
                    (_data[4] * p.y * p.y) + (2.0f * _data[5] * p.y * p.z) + (2.0f * _data[6] * p.y) + 
                    (_data[7] * p.z * p.z) + (2.0f * _data[8] * p.z) + 
                    _data[9];
        }

        void dump() const
        {
            std::cout << _data[0] << ", " << _data[1] << ", " << _data[2] << ", " << _data[3] << std::endl;
            std::cout << _data[1] << ", " << _data[4] << ", " << _data[5] << ", " << _data[6] << std::endl;
            std::cout << _data[2] << ", " << _data[5] << ", " << _data[7] << ", " << _data[8] << std::endl;
            std::cout << _data[3] << ", " << _data[6] << ", " << _data[8] << ", " << _data[9] << std::endl;
        }

    private:
        T _data[10];
};

class edge;
class face;
class vertex
{
    public:
        explicit vertex(const point_t &p) : _q(0.0f), _p(p), _cost(0.0f), _complex(false), _removed(false) { }

        bool complex() const { return _complex; }
        void complex(const bool c) { _complex = c; }

        bool removed() const { return _removed; }
        void removed(const bool r) { _removed = r; }

        const point_t& position() const { return _p; }
        void position(const point_t &p) { _p = p; }

        void add_face(const face &f);

        void collapse(vertex *const v1, const point_t &v_new, const float cost)
        {
            _cost += cost;
            _q += v1->_q;
            _p = v_new;
        }

        float merge_error(const vertex &v1, point_t &v_new) const
        {
            symetric_matrix4<float> q(_q + v1._q);
            // std::cout << "Input points " << _p << " and " << v1._p << std::endl;
            // std::cout << "Input matrix " << std::endl;
            // q.dump();
            const float det = q.determinant(0, 1, 2, 1, 4, 5, 2, 5, 7);

            /* Cant invert cost, this could be a planar vertex, removing it would be free */
            // std::cout << "Det " << det << std::endl;
            if (std::fabs(det) < 0.001f)
            {
                // std::cout << "Det too small" << std::endl;
                v_new = v1.position();
            }
            else
            {
                const float det_inv = 1.0f / det;
                const float det_x = q.determinant(1, 2, 3, 4, 5, 6, 5, 7, 8);
                const float det_y = q.determinant(0, 2, 3, 1, 5, 6, 2, 7, 8);
                const float det_z = q.determinant(0, 1, 3, 1, 4, 6, 2, 5, 8);
                v_new = max(min(_p, v1._p), min(max(_p, v1._p), point_t(-det_x, det_y, -det_z) * det_inv));
            }
            // std::cout << "v new " << v_new << ", cost " << q.quadric(v_new) << std::endl;

            // assert(q.quadric(v_new) > -1.0f);
            return q.quadric(v_new);
        }

        float merge_error(const point_t &p) const
        {
            return _q.quadric(p);
        }

        float cost() const { return _cost; }

    private:
        symetric_matrix4<float> _q;
        point_t                 _p;
        float                   _cost;
        bool                    _complex;
        bool                    _removed;
};

class edge
{
    public:
        edge(face *const f, vertex *const v0, vertex *const v1) : _removed(false), _recalc(true), _complex(false)
        {
            if (v0 < v1)
            {
                _faces[0] = f;
                _faces[1] = nullptr;
                _vertices[0] = v0;
                _vertices[1] = v1;
            }
            else
            {
                _faces[0] = nullptr;
                _faces[1] = f;
                _vertices[0] = v1;
                _vertices[1] = v0;
            }
        }

        bool operator<(const edge &e) const
        {
            if (_vertices[0] == e._vertices[0])
            {
                return _vertices[1] < e._vertices[1];
            }

            return (_vertices[0] < e._vertices[0]);
        }

        bool removed() const { return _removed; }
        void removed(const bool r) { _removed = r; }

        bool recalculate() const { return _recalc; }
        void recalculate(const bool r) { _recalc = r; }

        bool complex() const { return _complex; }
        void complex(const bool c) const { _complex = c; }

        vertex *vertex0() const { return _vertices[0]; }
        vertex *vertex1() const { return _vertices[1]; }
        void vertex0(vertex *v) { _vertices[0] = v; }
        void vertex1(vertex *v) { _vertices[1] = v; }

        vertex* other_vertex(const vertex *const from)
        {
            if (_vertices[0] == from)
            {
                return _vertices[1];
            }

            assert(_vertices[1] == from);
            return _vertices[0];
        }

        point_t &merge_point() { return _merge; }

        face *left_face(const vertex *const from)  const { return (_vertices[0] == from) ? _faces[0] : _faces[1]; }
        face *right_face(const vertex *const from) const { return (_vertices[0] == from) ? _faces[1] : _faces[0]; }

        void edges_left_face(face *f)  { _faces[0] = f;                    }
        void edges_right_face(face *f) { _faces[1] = f;                    }
        face *edges_left_face()  const { return left_face(_vertices[0]);   }
        face *edges_right_face() const { return right_face(_vertices[0]);  }

        void update_left_face(face *f, const vertex *const from)
        {
            if (_vertices[0] == from)
            {
                _faces[0] = f;
            }
            else
            {
                assert(_vertices[1] == from);
                _faces[1] = f;
            }
        }

        void update_right_face(face *f, const vertex *const from)
        {
            if (_vertices[0] == from)
            {
                _faces[1] = f;
            }
            else
            {
                assert(_vertices[1] == from);
                _faces[0] = f;
            }
        }

        float merge_cost()
        {
            // std::cout << std::hex << "Merge cost for " << this << " with vertices " << _vertices[0] << ", " << _vertices[1] << std::dec << std::endl;
            return _vertices[0]->merge_error(*_vertices[1], _merge);
        }

        void update_vertex(vertex *const v0, vertex *const v1)
        {
            if (_vertices[0] == v0)
            {
                _vertices[0] = v1;
            }
            else
            {
                assert(_vertices[1] == v0);
                _vertices[1] = v1;
            }

            assert(_vertices[0] != _vertices[1]);
        }

        bool check() const
        {
            return (_complex || ((_faces[0] != nullptr) && (_faces[1] != nullptr))) && (_vertices[0] != nullptr) && (_vertices[1] != nullptr);
        }

    private:
        face           *_faces[2];
        vertex         *_vertices[2];
        point_t         _merge;
        bool            _removed;
        bool            _recalc;
        mutable bool    _complex;
};

class face
{
    public:
        face(vertex *const v0, vertex *const v1, vertex *const v2) :
            _vertices{v0, v1, v2}, _edges{nullptr}, _removed(false) { }

        bool removed() const { return _removed; }
        void removed(const bool r) { _removed = r; }

        point_t normal() const
        {
            const point_t e0(_vertices[1]->position() - _vertices[0]->position());
            const point_t e1(_vertices[2]->position() - _vertices[0]->position());
            return cross_product(e0, e1);
        }

        bool flipped(vertex *const v, const point_t &p) const
        {
            const point_t n0(normal());
            const point_t t(v->position());
            v->position(p);

            const point_t n1(normal());
            v->position(t);
            
            // std::cout << "Normals " << n0 << " and " << n1 << std::endl;
            return dot_product(n0, n1) < 0.0f;
        }

        void add_edge(edge *const e)
        {
            if (_vertices[0] == e->vertex0())
            {
                if (_vertices[1] == e->vertex1())
                {
                    assert(_edges[0] == nullptr);
                    _edges[0] = e;
                }
                else
                {
                    assert(_vertices[2] == e->vertex1());
                    assert(_edges[2] == nullptr);
                    _edges[2] = e;
                }
            }
            else if (_vertices[1] == e->vertex0())
            {
                if (_vertices[2] == e->vertex1())
                {
                    assert(_edges[1] == nullptr);
                    _edges[1] = e;
                }
                else
                {
                    assert(_vertices[0] == e->vertex1());
                    assert(_edges[0] == nullptr);
                    _edges[0] = e;
                }
            }
            else
            {
                assert(_vertices[2] == e->vertex0());
                if (_vertices[0] == e->vertex1())
                {
                    assert(_edges[2] == nullptr);
                    _edges[2] = e;
                }
                else
                {
                    assert(_vertices[1] == e->vertex1());
                    assert(_edges[1] == nullptr);
                    _edges[1] = e;
                }
            }

            return;
        }

        edge *next_edge(const edge *e)
        {
            assert(_edges[0] != _edges[1]);
            assert(_edges[0] != _edges[2]);
            assert(_edges[1] != _edges[2]);

            if (_edges[0] == e)
            {
                return _edges[1];
            }

            if (_edges[1] == e)
            {
                return _edges[2];
            }

            assert(_edges[2] == e);
            return _edges[0];
        }

        const edge *next_edge(const edge *e) const
        {
            assert(_edges[0] != _edges[1]);
            assert(_edges[0] != _edges[2]);
            assert(_edges[1] != _edges[2]);

            if (_edges[0] == e)
            {
                return _edges[1];
            }

            if (_edges[1] == e)
            {
                return _edges[2];
            }

            assert(_edges[2] == e);
            return _edges[0];
        }

        edge *previous_edge(const edge *e)
        {
            assert(_edges[0] != _edges[1]);
            assert(_edges[0] != _edges[2]);
            assert(_edges[1] != _edges[2]);

            if (_edges[1] == e)
            {
                return _edges[0];
            }

            if (_edges[2] == e)
            {
                return _edges[1];
            }

            assert(_edges[0] == e);
            return _edges[2];   
        }

        void update_edge(edge *const e, vertex *const v)
        {
            if (_vertices[0] == v)
            {
                _edges[0] = e;
            }
            else if (_vertices[1] == v)
            {
                _edges[1] = e;
            }
            else
            {
                assert(_vertices[2] == v);
                _edges[2] = e;
            }

            assert(_edges[0] != _edges[1]);
            assert(_edges[0] != _edges[2]);
            assert(_edges[1] != _edges[2]);
        }

        void update_edge(edge *const from, edge *const to)
        {
            if (_edges[0] == from)
            {
                _edges[0] = to;
            }
            else if (_edges[1] == from)
            {
                _edges[1] = to;
            }
            else
            {
                assert(_edges[2] == from);
                _edges[2] = to;
            }

            assert(_edges[0] != _edges[1]);
            assert(_edges[0] != _edges[2]);
            assert(_edges[1] != _edges[2]);
        }

        const vertex *opposite(const edge *const e) const
        {
            if (_edges[0] == e)
            {
                assert((e->vertex0() == _vertices[0]) || (e->vertex0() == _vertices[1]));
                assert((e->vertex1() == _vertices[0]) || (e->vertex1() == _vertices[1]));
                return _vertices[2];
            }

            if (_edges[1] == e)
            {
                assert((e->vertex0() == _vertices[1]) || (e->vertex0() == _vertices[2]));
                assert((e->vertex1() == _vertices[1]) || (e->vertex1() == _vertices[2]));
                return _vertices[0];
            }

            assert((e->vertex0() == _vertices[0]) || (e->vertex0() == _vertices[2]));
            assert((e->vertex1() == _vertices[0]) || (e->vertex1() == _vertices[2]));
            assert(_edges[2] == e);
            return _vertices[1];
        }

        edge* update_vertex(vertex *const v0, vertex *const v1)
        {
            assert(_vertices[0] != _vertices[1]);
            assert(_vertices[0] != _vertices[2]);
            assert(_vertices[1] != _vertices[2]);

            if (_vertices[0] == v0)
            {
                if (_vertices[1] == v1)
                {
                    return _edges[0];
                }

                if (_vertices[2] == v1)
                {
                    return _edges[2];
                }

                _vertices[0] = v1;
            }
            else if (_vertices[1] == v0)
            {
                if (_vertices[0] == v1)
                {
                    return _edges[0];
                }

                if (_vertices[2] == v1)
                {
                    return _edges[1];
                }

                _vertices[1] = v1;
            }
            else
            {
                assert(_vertices[2] == v0);
                if (_vertices[0] == v1)
                {
                    return _edges[2];
                }

                if (_vertices[1] == v1)
                {
                    return _edges[1];
                }

                _vertices[2] = v1;
            }

            assert(_vertices[0] != _vertices[1]);
            assert(_vertices[0] != _vertices[2]);
            assert(_vertices[1] != _vertices[2]);
            return nullptr;
        }

        bool check() const
        {
            return (_vertices[0] != nullptr) && (_vertices[1] != nullptr) && (_vertices[2] != nullptr) && (_edges[0] != nullptr) && (_edges[1] != nullptr) && (_edges[2] != nullptr);
        }

        const vertex *vertice(const int i) const { return _vertices[i]; }
        edge *edges(const int i) { return _edges[i]; }

    private:
        vertex  *_vertices[3]; /* Vertices in anti clockwise order */
        edge    *_edges[3];
        bool    _removed;
};

inline void vertex::add_face(const face &f)
{
    const point_t &n = normalise(f.normal());
    _q += symetric_matrix4(n.x, n.y, n.z, -dot_product(n, _p));
}

class mesh_decimation
{
    public:
        mesh_decimation(const std::vector<point_t> &points, const std::vector<point_ti<>> &triangles, const mesh_decimation_options &options);

        std::vector<point_ti<>> triangles() const
        {
            std::vector<point_ti<>> ret;
            for (const auto &f : _faces)
            {
                if (!f.removed())
                {
                    assert(!f.vertice(0)->removed());
                    assert(!f.vertice(1)->removed());
                    assert(!f.vertice(2)->removed());
                    ret.emplace_back(std::distance(&_vertices[0], f.vertice(0)), std::distance(&_vertices[0], f.vertice(1)), std::distance(&_vertices[0], f.vertice(2)));
                }
            }

            return ret;
        }

        std::vector<point_t> vertices() const
        {
            /* Must output all vertices because the faces arent re-pointed */
            std::vector<point_t> ret;
            for (const auto &v : _vertices)
            {
                ret.emplace_back(v.position());
            }

            return ret;
        }

        void compact_output(std::vector<point_t> &vertices, std::vector<point_ti<>> &tris);

        float cost() const
        {
            return std::accumulate(_vertices.begin(), _vertices.end(), 0.0f, [](const float s, const vertex &v) { return s + v.cost(); });
        }

        int iterations() const
        {
            return _iteration;
        }

    private:
        struct queue_entry
        {
            queue_entry(edge *const e, const float cost) : e(e), cost(cost) { }

            bool operator<(const queue_entry &q) const { return cost > q.cost; }

            edge *e;
            float cost;
        };

        void decimate(edge *root);
        std::pair<face*, edge*> remove_edge(edge *merge, vertex *v0, vertex *v1);
        std::priority_queue<queue_entry> decimation_costs();
        void next_cost(std::priority_queue<mesh_decimation::queue_entry> *const costs);
        bool can_merge_fan(const edge *const start, const edge *const end, vertex *const around, const point_t &vm);

        void add_edge(vertex *const v0, vertex *const v1, face *const f)
        {
            if (const auto &fit = _unique_edges.emplace(f, v0, v1); !fit.second)
            {
                _edges.push_back(*fit.first);
                // End vertices -9.28726, 134.434, -17.6481 and -8.72672, 133.726, -18.313
                /* Add last face to edge */
                auto &e = _edges.back();
                if (e.left_face(v0) != nullptr)
                {
                    e.complex(true);
                    e.vertex0()->complex(true);
                    e.vertex1()->complex(true);
                    e.update_right_face(f, v0);
                    // std::cout << "Double edge " << fit.first->vertex0()->position() << " and " << fit.first->vertex1()->position() << std::endl;
                }
                else
                {
                    e.update_left_face(f, v0);
                    // std::cout << "Finishing edge " << fit.first->vertex0()->position() << " and " << fit.first->vertex1()->position() << std::endl;
                }

                /* Add edge to faces */
                e.edges_left_face()->add_edge(&e);
                e.edges_right_face()->add_edge(&e);

                _unique_edges.erase(fit.first);
            }
            else
            {
                if (const auto &cit = _created_edges.emplace(f, v0, v1); !cit.second)
                {
                    // std::cout << "Duplicate edge " << fit.first->vertex0()->position() << " and " << fit.first->vertex1()->position() << std::endl;
                    cit.first->complex(true);
                    fit.first->complex(true);
                    fit.first->vertex0()->complex(true);
                    fit.first->vertex1()->complex(true);
                }
                else
                {
                    // std::cout << "Created edge " << fit.first->vertex0()->position() << " and " << fit.first->vertex1()->position() << std::endl;
                }
            }
        }

        mesh_decimation_options _options;
        std::vector<vertex>     _vertices;
        std::vector<face>       _faces;
        std::vector<edge>       _edges;
        std::set<edge>          _unique_edges;
        std::set<edge>          _created_edges;
        int                     _iteration;
};
} /* namespace raptor_mesh_decimation */
