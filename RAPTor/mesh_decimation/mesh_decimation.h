#pragma once


/* Standard headers */
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

        T quadric(const point_t &p)
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
        explicit vertex(const point_t &p) : _q(0.0f), _p(p), _cost(0.0f) { }

        const point_t& position() const { return _p; }

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
            std::cout << "merged cost " << std::endl;
            q.dump();
            const float det = q.determinant(0, 1, 2, 1, 4, 5, 2, 5, 7);
            std::cout << "determinant " << det << std::endl;

            /* Cant invert cost, this could be a planar vertex, removing it would be free */
            if (det == 0.0)
            {
                std::cout << "cannot invert" << std::endl;
                v_new = v1.position();
            }
            else
            {
                const float det_inv = 1.0f / det;
                v_new.x =  det_inv * q.determinant(1, 2, 3, 4, 5, 6, 5, 7, 8);
                v_new.y = -det_inv * q.determinant(0, 2, 3, 1, 5, 6, 2, 7, 8);
                v_new.z =  det_inv * q.determinant(0, 1, 3, 1, 4, 6, 2, 5, 8);
            }

            return q.quadric(v_new);
        }

    private:
        symetric_matrix4<float> _q;
        point_t                 _p;
        float                   _cost;
};

class edge
{
    public:
        edge(face *const f, vertex *const v0, vertex *const v1)
        {
            if (v0 < v1)
            {
                _faces[0] = f;
                _vertices[0] = v0;
                _vertices[1] = v1;
            }
            else
            {
                _faces[1] = f;
                _vertices[0] = v1;
                _vertices[1] = v0;
            }
        }

        bool operator<(const edge &e) const
        {
            if (_vertices[0] < e._vertices[0])
            {
                return true;
            }

            return _vertices[1] < e._vertices[1];
        }

        vertex* vertex0() { return _vertices[0]; }
        vertex* vertex1() { return _vertices[1]; }
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

        face *left_face(const vertex *const from) { return (_vertices[0] == from) ? _faces[0] : _faces[1]; }
        face *right_face(const vertex *const from) { return (_vertices[0] == from) ? _faces[1] : _faces[0]; }
        face *edges_left_face() { return left_face(_vertices[0]); }
        face *edges_right_face() { return right_face(_vertices[0]); }

        void left_face(face *f, const vertex *const from)
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

        void right_face(face *f, const vertex *const from)
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
            return _vertices[0]->merge_error(*_vertices[1], _merge);
        }

    private:
        face   *_faces[2];
        vertex *_vertices[2];
        point_t _merge;
};

class face
{
    public:
        face(vertex *const v0, vertex *const v1, vertex *const v2) :
            _v0(v0), _v1(v1), _v2(v2) { }

        point_t normal() const
        {
            const point_t e0(_v1->position() - _v0->position());
            const point_t e1(_v2->position() - _v0->position());
            return cross_product(e0, e1);
        }

        void add_edge(edge *const e)
        {
            if (_v0 == e->vertex0())
            {
                _edges[0] = e;
            }
            else if (_v1 == e->vertex0())
            {
                _edges[1] = e;
            }
            else
            {
                assert(_v2 == e->vertex0());
                _edges[2] = e;
            }

            return;
        }

        edge *next_edge(const edge *e)
        {
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
            if (_edges[1] == e)
            {
                return _edges[0];
            }

            if (_edges[2] == e)
            {
                return _edges[1];
            }

            assert(_edges[0] == e);
            return _edges[0];   
        }

    private:
        vertex  *_v0; /* Vertices in anti clockwise order */
        vertex  *_v1;
        vertex  *_v2;
        edge    *_edges[3];
};

inline void vertex::add_face(const face &f)
{
    const point_t &n = normalise(f.normal());
    _q += symetric_matrix4(n.x, n.y, n.z, dot_product(n, _p));
}

class mesh_decimation
{
    public:
        mesh_decimation(const std::vector<point_t> &points, const std::vector<point_ti<>> &triangles, const mesh_decimation_options &options);

    private:
        void add_edge(vertex *const v0, vertex *const v1, face *const f)
        {
            if (const auto &fit = _unique_edges.emplace(f, v0, v1); !fit.second)
            {
                _edges.push_back(*fit.first);

                /* Add last face to edge */
                auto &e = _edges.back();
                e.left_face(f, v0);

                /* Add edge to faces */
                e.edges_left_face()->add_edge(&e);
                e.edges_right_face()->add_edge(&e);

                _unique_edges.erase(fit.first);
            }
        }

        mesh_decimation_options _options;
        std::vector<vertex>     _vertices;
        std::vector<face>       _faces;
        std::vector<edge>       _edges;
        std::set<edge>          _unique_edges;
};
} /* namespace raptor_mesh_decimation */
