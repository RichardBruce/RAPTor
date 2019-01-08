#pragma once

/* Shared headers */
#include <vector>

/* Boost headers */
#include "boost/noncopyable.hpp"

/* Common headers */
#include "common.h"
#include "point_t.h"
#include "physics_common.h"
#include "quaternion_t.h"


namespace raptor_physics
{
inline bool same_side(const point_t &p, const point_t &a, const point_t &ab, const point_t &fn)
{
    /* Normal of the triangle formed by the tested point */
    point_t cp;
    cross_product((p - a), ab, &cp);
    
    /* Check the normal is in the same direction as the face normal */
    if (dot_product(cp, fn) > 0.0f)
    {
        return true;
    }
    else
    {
        return false;
    }
}

class world_polygon
{
    public :
        /* CTOR */
        world_polygon(const std::vector<point_t> verts) : _verts(verts) {  };

        /* Copy CTOR */
        world_polygon(const world_polygon &p) : _verts(p._verts) {  };


        /* Find all points below a plane with point p and normal n */
        void points_above_plane(const point_t &p, const point_t &n)
        {
            int to_idx = 0;
            for (const auto &vert : _verts)
            {
                const point_t diff(vert - p);
                const float dist = dot_product(n, diff) + dot_product(fabs(diff), point_t(raptor_physics::EPSILON, raptor_physics::EPSILON, raptor_physics::EPSILON));
                if (dist >= 0.0f)
                {
                    _verts[to_idx++] = vert;
                }
            }

            _verts.resize(to_idx);
        }

        /* Search vertices for the most extreme along n */
        point_t extreme_vertices(point_t *const neg, const point_t &n) const
        {
            float most_pos = -std::numeric_limits<float>::max();
            float most_neg =  std::numeric_limits<float>::max();
            const point_t *pos_vert = nullptr;
            const point_t *neg_vert = nullptr;

            /* Search for most positive and negative */
            /* TODO - This could be a little more clever for convex polygons where we could use slope following to eliminate some tests */
            for (const auto &vert : _verts)
            {
                const float dist = dot_product(n, vert);
                if (dist < most_neg)
                {
                    most_neg = dist;
                    neg_vert = &vert;
                }
                else if (dist > most_pos)
                {
                    most_pos = dist;
                    pos_vert = &vert;
                }
            }

            (*neg) = (*neg_vert);
            return *pos_vert;
        }

        /* Find the geometric center of the polygon */
        point_t center() const
        {
            point_t c(_verts[0]);
            for (int i = 1; i < static_cast<int>(_verts.size()); ++i)
            {
                c += _verts[i];
            }

            return c / static_cast<float>(_verts.size());
        }

        /* Find the union of two polygons */
        void intersection(const world_polygon &p, const point_t &n, const point_t &dir);

        /* Access functions */
        int             number_of_vertices()    const { return _verts.size();   }
        const point_t&  vertex(const int i)     const { return _verts[i];       }
        
    private :
        float intersection(const point_t &a, const point_t &ma, const point_t &b, const point_t &mb) const;

        /* Check if a point is "inside" an edge */
        float is_inside_edge(const point_t &c, const point_t &t, const point_t &n)
        {
            const point_t to_clip(c - t);
            return dot_product(n, to_clip);
        }

        /* Check if 2 points are the same */
        bool is_coincident(const point_t &a, const point_t &b)
        {
            const point_t edge(a - b);
            return dot_product(edge, edge) < raptor_physics::EPSILON;
        }

        std::vector<point_t>    _verts;
};

class polygon : private boost::noncopyable
{
    public :
        /* CTOR */
        polygon(const std::vector<point_t> *const pts, const std::vector<int> &verts) : _pts(pts), _verts(verts) {  };

        polygon(const polygon &rhs) : _pts(rhs._pts), _verts(rhs._verts) {  };

        world_polygon to_world_polygon(const quaternion_t &o, const point_t &t) const
        {
            std::vector<point_t> global_pts;
            for (const int &vert : _verts)
            {
                const point_t global_pt(o.rotate((*_pts)[vert]) + t);
                global_pts.push_back(global_pt);
            }

            return world_polygon(global_pts);
        }

        bool contains_vertices(const int *const begin, const int *const end) const
        {
            return std::all_of(begin, end, [this](const int i)
            {
                return std::find(_verts.begin(), _verts.end(), i) != _verts.end();
            });
        }

        point_t normal() const
        {
            assert(_verts.size() > 2);

            /* Length of 2 arbitary sides */
            const point_t ab((*_pts)[_verts[1]] - (*_pts)[_verts[0]]);
            const point_t bc((*_pts)[_verts[2]] - (*_pts)[_verts[1]]);

            /* Normal */
            return normalise(cross_product(ab, bc));
        }

        void to_triangles(std::vector<int> *const tris) const;

        /* Access functions */
        int             number_of_vertices()    const { return _verts.size();       }
        const point_t&  vertex(const int i)     const { return (*_pts)[_verts[i]];  }

    private :
        point_t centre() const
        {
            /* Add all the points together */
            point_t total;
            for (const int v : _verts)
            {
                total += (*_pts)[v];
            }

            /* Average the points and return */
            return total / static_cast<float>(_verts.size());
        }

        int find_furthest(const point_t &c) const
        {
            int max_point   = 0;
            float max_dist  = 0.0f;

            /* Compare the distance of the points from the centre point */
            for (int i = 0; i < static_cast<int>(_verts.size()); ++i)
            {
                const int v         = _verts[i];
                const float dist    = magnitude(c - (*_pts)[v]);
                if (dist > max_dist)
                {
                    max_dist  = dist;
                    max_point = i;
                }
            }
            
            /* Return the most distant point */
            return max_point;
        }

        bool is_straight_line(const point_t &a, point_t b, point_t c) const
        {
            b -= a;
            c -= a;
            point_t n;
            cross_product(b, c, &n);
            return (n == 0.0f);
        }

        bool is_in_triangle(const std::unique_ptr<char []> &invalid, const point_t &a, const point_t &b, const point_t &c, const point_t &fn) const
        {
            /* Pre-calculate some useful vectors */
            const point_t ab(a - b);
            const point_t ca(c - a);
            const point_t bc(b - c);
            
            /* Check each point if it is inside the triangle */
            for (int i = 0; i < static_cast<int>(_verts.size()); ++i)
            {
                /* No need to test for invalid points */
                if (invalid[i])
                {
                    continue;
                }

                /* Check the point is on the same side of AB as C, BC as A and CA as B */
                const point_t pt((*_pts)[_verts[i]]);
                if (same_side(pt, a, ab, fn) && same_side(pt, b, bc, fn) && same_side(pt, c, ca, fn))
                {
                    return true;
                }
            }

            return false;
        }

        bool is_intersecting(const point_t &a, const point_t &ma, const point_t &b, const point_t &mb) const
        {
            const point_t r(a - b);
            const float ma_dot_ma = dot_product(ma, ma);
            const float ma_dot_mb = dot_product(ma, mb);
            const float mb_dot_mb = dot_product(mb, mb);

            /* Check for parallel lines */
            const float d = (ma_dot_ma * mb_dot_mb) - (ma_dot_mb * ma_dot_mb);
            if (fabs(d) < raptor_physics::EPSILON)
            {
                return false;
            }

            /* Check in line segment */
            const float ma_dot_r = dot_product(ma, r);
            const float mb_dot_r = dot_product(mb, r);
            const float s = ((ma_dot_mb * mb_dot_r) - (mb_dot_mb * ma_dot_r)) / d;
            if ((s < raptor_physics::EPSILON) || (s > (1.0f - raptor_physics::EPSILON)))
            {
                return false;
            }

            const float t = ((ma_dot_ma * mb_dot_r) - (ma_dot_mb * ma_dot_r)) / d;
            if ((t < raptor_physics::EPSILON) || (t > (1.0f - raptor_physics::EPSILON)))
            {
                return false;
            }

            /* Check distance */
            const point_t dir((a + (s * ma)) - (b + (t * mb)));
            return (dot_product(dir, dir) < raptor_physics::EPSILON);
        }

        bool intersects_polygon(const std::unique_ptr<char []> &invalid, const point_t &a, const point_t &b, const point_t &c) const
        {
            /* Pre-calculate some useful vectors */
            const point_t ab(a - b);
            const point_t ca(c - a);
            const point_t bc(b - c);

            /* Check each edge if it intersects the triangle */
            for (int i = 0; i < static_cast<int>(_verts.size()) - 1; ++i)
            {
                /* No need to test for invalid edges */
                if (invalid[i])
                {
                    continue;
                }

                /* Find next valid point */
                int i_p1 = i;
                do
                {
                    ++i_p1;
                    if (i_p1 == static_cast<int>(_verts.size()))
                    {
                        i_p1 = 0;
                    }
                } while (invalid[i_p1] == true);

                const point_t pt((*_pts)[_verts[i]]);
                const point_t m_pt((*_pts)[_verts[i_p1]] - pt);
                if (is_intersecting(b, ab, pt, m_pt) || is_intersecting(a, ca, pt, m_pt) || is_intersecting(c, bc, pt, m_pt))
                {
                    return true;
                }
            }

            return false;
        }

        const std::vector<point_t> *const   _pts;   /* Points that may define the vertices of the polygon   */
        const std::vector<int>              _verts; /* Anti-clockwise ordered list of vertices              */
};
}; /* namespace raptor_physics */
