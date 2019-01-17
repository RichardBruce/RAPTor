#pragma once

/* Standard headers */

/* Boost headers */
#include "boost/noncopyable.hpp"

/* Common headers */
#include "point_t.h"
#include "quaternion_t.h"

/* Physics headers */
#include "object_bound.h"
#include "force.h"
#include "integrators.h"


namespace raptor_physics
{
/* Forward declarations */
class simplex;
class physics_engine;

/* Class to wrap a vertex group with the physics engine specific functionality */
class particle : private boost::noncopyable
{
    public :
        particle(const point_t &com, const float radius, const unsigned int t = 0) : _com(com), _radius(radius), _type(t) {  }

        bool has_collided(particle *const pa, simplex **const manifold_a, simplex **const manifold_b, point_t *const d, const float t0, const float t1)
        {
            return has_collided(pa, manifold_a, manifold_b, d, _o, po->_o, t0, t1);
        }

        bool has_collided(particle *const pa, simplex **const manifold_a, simplex **const manifold_b, point_t *const d, const quaternion_t &oa, const quaternion_t &ob, const float t0, const float t1)
        {
            // METHOD_LOG;

            /* Check for self intersection and return an empty simplex */
            // *manifold_a = new simplex(*this, true);
            // *manifold_b = new simplex(*pa, false);
            if (po == this)
            {
                return false;
            }
            
            /* Calculate the relative displacement */
            const point_t rel_disp(_com - pa->_com);
            const point_t rel_disp0((_lv * (t0 - _cur_t)) - (pa->_lv * (t0 - pa->_cur_t)));
            const point_t rel_disp1((_lv * (t1 - _cur_t)) - (pa->_lv * (t1 - pa->_cur_t)));
            const point_t rel_disp_diff(rel_disp1 - rel_disp0);
            const bool sig_disp = dot_product(rel_disp_diff, rel_disp_diff) > raptor_physics::EPSILON;

            /* Test for collision between a point and a line if the particles are moving relative to eachother */
            const float radii = _radius + pa->_radius;
            if (sig_disp)
            {
                /* Are we going in the right direction, if not then test with the initial point */
                const point_t norm_disp(normalise(rel_disp_diff));
                const float norm_dist = dot_product(norm_disp, rel_disp);
                if (norm_dist < 0.0f)
                {
                    const float dist = magnitude(rel_disp);

                    (*d) = rel_disp - ((rel_disp / dist) * radii);
                    return dist <= radii;
                }

                /* Is the closest point within the radius */
                const point_t closest(norm_disp * norm_dist);
                const float dist = magnitude(closest);

                (*d) = closest - ((closest / dist) * radii);
                return dist <= radii;
            }
            /* Test for collision between spheres if the particles arent moving relative to eachother */
            else
            {
                const float dist = magnitude(rel_disp);

                (*d) = rel_disp - ((rel_disp / dist) * radii);
                return dist <= radii;
            }
        }

        collision_t exactly_resolve_collisions(particle *const pa, simplex **const manifold_a, simplex **const manifold_b, float *const t)
        {
            const float min_t = std::max(_cur_t, pa->_cur_t);
            const float max_t = *t;

        }

    private :
        std::vector<force *>   *        _forces;
        aggregate_force                 _agg_force;
        object_bound *                  _upper_bound[3];    /* The upper bound of the object in this time step              */
        object_bound *                  _lower_bound[3];    /* The lower bound of the object in this time step              */
        point_t                         _com;               /* Center of mass                                               */
        point_t                         _v;                 /* Velocity                                                     */
        point_t                         _lv;                /* Descretised velocity for this frame                          */
        float                           _cur_t;             /* The committed time                                           */
        float                           _t_step;            /* The time we are simulating to                                */
        const float                     _radius;            /* Radius of the particle                                       */
        const unsigned int              _type;              /* The type of the material to associate it with a collider     */
};
}; /* namespace raptor_physics */
