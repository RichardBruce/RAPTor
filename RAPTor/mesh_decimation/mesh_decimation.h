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
            return (_data[i11] * ((_data[i22] * _data[i33]) - (_data[i23] * _data[i32]))) + 
                   (_data[i12] * ((_data[i23] * _data[i31]) - (_data[i21] * _data[i33]))) + 
                   (_data[i13] * ((_data[i21] * _data[i32]) - (_data[i22] * _data[i31])));
        }

        T quadric(const point_t<double> &p) const
        {
            return  (2.0 * ((_data[1] * p.x * p.y) + (_data[2] * p.x * p.z) + (_data[3] * p.x) + 
                            (_data[5] * p.y * p.z) + (_data[6] * p.y) + 
                            (_data[8] * p.z))) + 
                    (_data[0] * p.x * p.x) + (_data[4] * p.y * p.y) + (_data[7] * p.z * p.z) + _data[9];
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

class vertex
{
    public:
        vertex() : _q(0.0) { }
        vertex(const point_t<double> &p) : _q(0.0), _p(p), _cost(0.0), _boundary(false) { }

        double cost() const { return _cost; }

        point_t<double> position() const { return _p; }
        void position(const point_t<double> &p) { _p = p; }

        int links() const { return _links; }
        void links(const int t) { _links = t; }
        void add_links() { ++_links; }

        int first_link() const { return _first_link; }
        void first_link(const int t) { _first_link = t; }
        void reset_link()
        {
            _first_link = 0;
            _links = 0;
        }

        void boundary(const bool boundary) { _boundary = boundary; }

        void add_face(const symetric_matrix4<double> &q)
        {
            _q += q;
        }

        void update(const vertex *v, const point_t<double> &p, const double cost)
        {
            _p = p;
            _q += v->_q;
            _cost += (v->_cost + cost);
        }

        point_t<double> vertex_error(const vertex *v) const
        {
            /* Attempt to optimise */
            symetric_matrix4<double> q(_q + v->_q);
            if (!(_boundary & v->_boundary))
            {
                const double det = q.determinant(0, 1, 2, 1, 4, 5, 2, 5, 7);
                if (det != 0.0)
                {
                    return (1.0 / det) * point_t<double>(-q.determinant(1, 2, 3, 4, 5, 6, 5, 7, 8), q.determinant(0, 2, 3, 1, 5, 6, 2, 7, 8), -q.determinant(0, 1, 3, 1, 4, 6, 2, 5, 8));
                }
            }

            /* Fall back */
            point_t<double> v_new(_p);
            double cost = q.quadric(_p);
            if (const double c = q.quadric(v->_p); c < cost)
            {
                cost = c;
                v_new = v->_p;
            }

            const point_t<double> p_mid((_p + v->_p) * 0.5);
            if (const double c = q.quadric(p_mid); c < cost)
            {
                cost = c;
                v_new = p_mid;
            }

            return v_new;
        }

        double vertex_error(const vertex *v, const double max_cumulative_cost, point_t<double> &v_new) const
        {
            /* Dont merged edges into the body of meshes */
            if (_boundary ^ v->_boundary)
            {
                return std::numeric_limits<double>::infinity();
            }

            /* Attempt to optimise */
            symetric_matrix4<double> q(_q + v->_q);
            if (!(_boundary & v->_boundary))
            {
                const double det = q.determinant(0, 1, 2, 1, 4, 5, 2, 5, 7);
                if (det != 0.0)
                {
                    v_new = (1.0 / det) * point_t<double>(-q.determinant(1, 2, 3, 4, 5, 6, 5, 7, 8), q.determinant(0, 2, 3, 1, 5, 6, 2, 7, 8), -q.determinant(0, 1, 3, 1, 4, 6, 2, 5, 8));
                    
                    const double cost = q.quadric(v_new);
                    if ((_cost + v->_cost + cost) > max_cumulative_cost)
                    {
                        return std::numeric_limits<double>::infinity();
                    }

                    return cost;
                }
            }

            /* Fall back */
            v_new = _p;
            double cost = q.quadric(_p);
            if (const double c = q.quadric(v->_p); c < cost)
            {
                cost = c;
                v_new = v->_p;
            }

            const point_t<double> p_mid((_p + v->_p) * 0.5);
            if (const double c = q.quadric(p_mid); c < cost)
            {
                cost = c;
                v_new = p_mid;
            }

            /* Ignore this edge if the cumulative cost would be too high */
            if ((_cost + v->_cost + cost) > max_cumulative_cost)
            {
                cost = std::numeric_limits<double>::infinity();
            }

            return cost;
        }

    private:
        symetric_matrix4<double>    _q;             /* Quadric cost matrix */
        point_t<double>             _p;             /* Position */
        double                      _cost;          /* Accumulated cost */
        int                         _first_link;    /* Index of first link */
        int                         _links;         /* Number of faces this vertex links to */
        bool                        _boundary;      /* If this is a vertex on the boundary */

};

class alignas(64) face
{
    public:
        face() = default;
        face(vertex *v0, vertex *v1, vertex *v2) : _v{v0, v1, v2}, _removed(false)
        {
            const point_t<double> &p0 = _v[0]->position();
            _n = normalise(cross_product(_v[1]->position() - p0, _v[2]->position() - p0));
            _v[0]->add_face(symetric_matrix4<double>(_n.x, _n.y, _n.z, dot_product(-_n, p0)));
            _v[1]->add_face(symetric_matrix4<double>(_n.x, _n.y, _n.z, dot_product(-_n, p0)));
            _v[2]->add_face(symetric_matrix4<double>(_n.x, _n.y, _n.z, dot_product(-_n, p0)));
        }

        /* Calculate the cost of merging each edge and take the smallest */
        double face_error(const double max_cumulative_cost)
        {
            _min_idx = 0;
            point_t<double> p;
            _min_cost = _v[0]->vertex_error(_v[1], max_cumulative_cost, p);
            if (const double c = _v[1]->vertex_error(_v[2], max_cumulative_cost, p); c < _min_cost)
            {
                _min_cost = c;
                _min_idx = 1;
            }

            if (const double c = _v[2]->vertex_error(_v[0], max_cumulative_cost, p); c < _min_cost)
            {
                _min_cost = c;
                _min_idx = 2;
            }

            return _min_cost;
        }

        point_t<double> normal() const { return _n; }

        vertex* vertices(const int idx) const { return _v[idx]; }
        void vertices(const int idx, vertex *v) { _v[idx] = v; }

        void remove() { _removed = true; }
        bool removed() const { return _removed; }

        double cost() const { return _min_cost; }
        int index() const { return _min_idx; }

    private:
        vertex *        _v[3];      /* Index of this faces vertices */
        point_t<double> _n;         /* The normal of this face */
        double          _min_cost;  /* The lowest edge merge cost */
        int             _min_idx;   /* The index of the lowest edge */
        bool            _removed;   /* Could save this byte, but still 1 per cache line */
};

class link
{
    public:
        link() = default;
        link(const int face_idx, const int vertex_idx) : _face_idx(face_idx), _vertex_idx(vertex_idx) { }

        int face()      const { return _face_idx;   }
        int vertex()    const { return _vertex_idx; }

    private:
        int _face_idx;      /* A face that this vertex belongs to */
        int _vertex_idx;    /* The index of this vertex within the above face */
};

class mesh_decimation
{
    public:
        mesh_decimation(const std::vector<point_t<>> &points, const std::vector<point_ti<>> &triangles, const mesh_decimation_options &options);

        void compact_output(std::vector<point_t<>> &vertices, std::vector<point_ti<>> &triangles);

        int iterations() const { return _iteration; }
        double cost() const { return _cost; }

    private:
        bool can_update(std::vector<bool> &removed, const vertex *v, const point_t<double> &p, const vertex *to);
        void update_mesh(std::vector<bool> &removed, int &cleaned, const vertex *v, vertex *to);
        void build_links();
        void decimate();

        mesh_decimation_options _options;
        std::vector<face>       _faces;
        std::vector<vertex>     _vertices;
        std::vector<link>       _links;
        double                  _cost;
        int                     _iteration;
};
} /* namespace raptor_mesh_decimation */
