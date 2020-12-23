#pragma once

/* Standard headers */
#include <vector>

/* Common headers */
#include "point_t.h"

/* Convex decomposition headers */
#include "dac_convex_hull.h"


namespace raptor_convex_decomposition
{
class convex_mesh
{
    public :
        /* Default CTOR */
        convex_mesh() = default;

        /* CTOR */
        convex_mesh(const std::vector<point_t<>> &points, const std::vector<point_ti<>> &triangles) : _points(points), _triangles(triangles) {  }

        /* Access functions */
        int number_of_points()      const { return _points.size();      }
        int number_of_triangles()   const { return _triangles.size();   }
        const point_t<> &                 get_point(const int i)  const { return _points[i];  }
              point_t<> &                 get_point(const int i)        { return _points[i];  }
        const std::vector<point_t<>> &    points()                const { return _points;     }
              std::vector<point_t<>> &    points()                      { return _points;     }
        const std::vector<point_ti<>> & triangles()             const { return _triangles;  }
              std::vector<point_ti<>> & triangles()                   { return _triangles;  }

        /* Clear the mesh */
        convex_mesh& clear()
        {
            _points.clear();
            _triangles.clear();
            return *this;
        }

        /* Add a point */
        convex_mesh& add_point(const point_t<> &p)
        {
            _points.push_back(p);
            return *this;
        }

        /* Add a new triangle */
        convex_mesh& add_triangle(const int a, const int b, const int c)
        {
            _triangles.push_back(point_ti<>(a, b, c));
            return *this;
        }

        float volume() const
        {
            if (_points.empty() || _triangles.empty())
            {
                return 0.0f;
            }

            /* Find center */
            point_t<> com(0.0f, 0.0f, 0.0f);
            for (int i = 0; i < static_cast<int>(_points.size()); ++i)
            {
                com += _points[i];
            }
            com /= static_cast<float>(_points.size());

            /* Find volume as sum of simplices */
            float vol = 0.0f;
            for (const auto &t : _triangles)
            {
                vol += tetrahedron_volume(_points[t.x], _points[t.y], _points[t.z], com);
            }

            return vol * (1.0f / 6.0f);
        }

        void compute_convex_hull(const std::vector<point_t<>> &pts)
        {
            /* Compute convex hull */
            const dac_convex_hull ch(pts);

            /* Add points to mesh */
            _points.clear();
            for (const auto &v : ch.vertices())
            {
                add_point(v);
            }

            /* Add triangulated faces to mesh */
            _triangles.clear();
            for (const auto &face : ch.faces())
            {
                const int a = face[0];
                for (int i = 2; i < static_cast<int>(face.size()); ++i)
                {
                    add_triangle(a, face[i - 1], face[i]);
                }
            }
        }

        void cut(std::vector<point_t<>> *const pos_cut, std::vector<point_t<>> *const neg_cut, const point_t<> &n, const float p) const
        {
            if (_points.size() == 0)
            {
                return;
            }       

            /* Partition points above and below */
            for (int i = 0; i < static_cast<int>(_points.size()); ++i)
            {
                const point_t<> &pt = _points[i];
                const float d = dot_product(n, pt) - p;
                if (d > 0.0f)
                {
                    pos_cut->push_back(pt);
                }
                else if (d < 0.0f)
                {
                    neg_cut->push_back(pt);
                }
                else
                {
                    pos_cut->push_back(pt);
                    neg_cut->push_back(pt);
                }
            }
        }

        bool is_inside(const point_t<> &pt) const
        {
            if (_triangles.empty())
            {
                return false;
            }

            /* Check for positive volume when added to all triangles */
            for (const auto &t : _triangles)
            {
                const float volume = tetrahedron_volume(_points[t.x], _points[t.y], _points[t.z], pt);
                if (volume < 0.0f)
                {
                    return false;
                }
            }

            return true;
        }

        float diagonal() const
        {
            if (_points.size() == 0)
            {
                return 0.0f;
            }

            /* Find bounding box */
            point_t<> min_box(_points[0]);
            point_t<> max_box(_points[0]);
            for (int i = 1; i < static_cast<int>(_points.size()); ++i)
            {
                min_box = min(min_box, _points[i]);
                max_box = max(max_box, _points[i]);
            }

            /* Take diagonal length */
            return magnitude(max_box - min_box);
        }

    private :
        std::vector<point_t<>>    _points;
        std::vector<point_ti<>> _triangles;
};
} /* namespace raptor_convex_decomposition */