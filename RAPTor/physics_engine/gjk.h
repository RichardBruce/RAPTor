#pragma once

/* Boost headers */
#include "boost/noncopyable.hpp"

/* Common header */
#include "logging.h"
#include "common.h"
#include "quaternion_t.h"
#include "simplex.h"

/* Forward declarations */
class gjk_test_access;

namespace raptor_physics
{
/* Forward declarations */
class vertex_group;

class gjk : private boost::noncopyable
{
    public :
        /* Initial CTOR. */
        gjk(const vertex_group &vg_a, const vertex_group &vg_b, simplex *const sim_a, simplex *const sim_b)
            : _a(vg_a),
              _b(vg_b),
              _a_simplex(sim_a),   /* simplex of vertex 0 */
              _b_simplex(sim_b)    /* simplex of vertex 0 */
        {
            // METHOD_LOG;
        };

        /* Allow default DTOR */

        /* Finds the shortest distance between two convex hulls. */
        /* Returns true if the hulls penetrate. */
        /* dist is set to the distance between the two hulls or zero if they penetrate. */
        /* rel_disp is the relative displacement of the two hulls. */
        bool find_minimum_distance(point_t *const dist, const point_t &fixed_rel_disp, const point_t &float_rel_disp, const quaternion_t &oa, const quaternion_t &ob);
    
    private :
        /* Friend class for testing */
        friend gjk_test_access;
        
        /* Find the closest vertex, edge or face to the origin. */
        /* Returns the size and vertices in the closest feature and the direction to the origin. */
        int find_closest_feature_to_origin(const point_t *const c_space, int *const verts, point_t *const dir) const;
        
        /* Tests if an edge is the closest feature to the origin. */
        /* Returns true if this is the closest edge fature. */
        /* Sets the vertices of the edge and the direction to the origin. */
        bool test_edge(const point_t &norm0, const point_t &norm1, const float dot0, const float dot1,
            const point_t& diff, const point_t &c_space, const int i, const int j,
            int *const verts, point_t *const dir) const
        {
            if ((dot0 >= 0.0f) && (dot1 >= 0.0f))
            {
                /* It is important norm isnt normalised and the equality be in the conditions */
                /* This covers the case that a face has degraded to a line */
                const point_t cross0(cross_product(diff, norm0));
                const point_t cross1(cross_product(norm1, diff));
                if ((dot_product(c_space, cross0) >= 0.0f) && (dot_product(c_space, cross1) >= 0.0f))
                {
                    /* Return this edge as the closest */
                    verts[0] = i;
                    verts[1] = j;
                    const float magn_diff = dot_product(diff, diff);
                    if (magn_diff > raptor_physics::EPSILON)
                    {
                        const float dot_prod = dot_product(c_space, diff);
                        (*dir) = c_space - ((diff * dot_prod) / magn_diff);
                    }
                    else
                    {
                        (*dir) = c_space;
                    }
                    return true;
                }
            }
            
            return false;
        }

        /* Check if a line from the origin moving in the triangles normal will intersect the triangle */
        bool in_triangle(const point_t &n, const point_t &d0, const point_t &d1, const point_t &c, point_t *d, int *verts, const int v0, const int v1, const int v2) const;
        
        const vertex_group &    _a;         /* The vertices of object a */
        const vertex_group &    _b;         /* The vertcies of object b */
        simplex         *const  _a_simplex; /* The simplex in object a  */
        simplex         *const  _b_simplex; /* The simplex in object b  */
};
}; /* namespace raptor_physics */
