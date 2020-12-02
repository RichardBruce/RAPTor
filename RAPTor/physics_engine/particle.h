#pragma once

/* Standard headers */

/* Boost headers */
#include "boost/noncopyable.hpp"

/* Common headers */
#include "point_t.h"
#include "quaternion_t.h"

/* Physics headers */
#include "collision.h"
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
        particle(const point_t<> &com, const float radius, const unsigned int t) : 
            _forces(new std::vector<force *>()),
            _agg_force(*_forces),
            _com(com),
            _cur_t(0.0f),
            _radius(radius),
            _type(t)
        {  }
        
        particle& set_velocity(const point_t<> &v) 
        {
            _v  = v;
            _lv = v;
            return *this;
        }

        collision_t resolve_collisions(particle *const pa, simplex **const manifold_a, simplex **const manifold_b, float *const t, const bool sliding)
        {
            // METHOD_LOG;

            /* Check for self intersection and return an empty simplex */
            // *manifold_a = new simplex(*this, true);
            // *manifold_b = new simplex(*pa, false);
            if (pa == this)
            {
                return collision_t::NO_COLLISION;
            }

            const float t1 = *t;
            const float t0 = std::max(_cur_t, pa->_cur_t);
            
            /* Calculate the relative displacement */
            const point_t<> rel_disp(_com - pa->_com);
            const point_t<> rel_disp0((pa->_lv * (t0 - pa->_cur_t)) - (_lv * (t0 - _cur_t)));
            const point_t<> rel_disp1((pa->_lv * (t1 - pa->_cur_t)) - (_lv * (t1 - _cur_t)));
            const point_t<> rel_disp_diff(rel_disp1 - rel_disp0);
            const bool sig_disp = dot_product(rel_disp_diff, rel_disp_diff) > raptor_physics::EPSILON;

            /* Test for collision between a point and a line if the particles are moving relative to eachother */
            const float radii = _radius + pa->_radius;
            if (sig_disp)
            {
                /* Check for starting inside */
                const point_t<> r_o(rel_disp + rel_disp0);
                const float r_sq = radii * radii;
                const float h_sq = dot_product(r_o, r_o);
                if (h_sq < r_sq)
                {
                    BOOST_LOG_TRIVIAL(trace) << "Started in contact, immediate collision";
                    *t = t0;
                    return collision_t::COLLISION;
                }

                /* Check for moving away */
                const float r_l = sqrt(dot_product(rel_disp_diff, rel_disp_diff));
                const point_t<> r_d(rel_disp_diff / r_l);
                BOOST_LOG_TRIVIAL(trace) << "Origin: " << r_o << ", dir: " << r_d;
                const float a = dot_product(r_o, r_d);
                if (a < 0.0f)
                {
                    BOOST_LOG_TRIVIAL(trace) << "Moving away, no collision";
                    return collision_t::NO_COLLISION;
                }

                /* Check for never getting close */
                const float o_sq = h_sq - (a * a);
                if (o_sq > r_sq)
                {
                    BOOST_LOG_TRIVIAL(trace) << "Closest point " << o_sq << " outside of sphere " << r_sq << ", no collision";
                    return collision_t::NO_COLLISION;
                }

                /* Always take the first collision and find the time */
                const float d = a - sqrt(r_sq - o_sq);
                BOOST_LOG_TRIVIAL(trace) << "Distance to closest point " << a << " with adjustment " << o_sq << " and radius sq " << r_sq;
                if (d > r_l)
                {
                    return collision_t::NO_COLLISION;
                }

                *t = t0 + (d / magnitude(_lv - pa->_lv));
                return collision_t::COLLISION;
            }
            /* Test for collision between spheres if the particles arent moving relative to eachother */
            else
            {
                *t = t1;
                BOOST_LOG_TRIVIAL(trace) << "No movement, checking for instant collision";
                return (dot_product(rel_disp, rel_disp) <= (radii * radii)) ? collision_t::COLLISION : collision_t::NO_COLLISION;
            }

            return collision_t::NO_COLLISION;
        }

    private :
        std::vector<force *>   *        _forces;
        aggregate_force                 _agg_force;
        object_bound *                  _upper_bound[3];    /* The upper bound of the object in this time step              */
        object_bound *                  _lower_bound[3];    /* The lower bound of the object in this time step              */
        point_t<>                       _com;               /* Center of mass                                               */
        point_t<>                       _v;                 /* Velocity                                                     */
        point_t<>                       _lv;                /* Descretised velocity for this frame                          */
        float                           _cur_t;             /* The committed time                                           */
        float                           _t_step;            /* The time we are simulating to                                */
        const float                     _radius;            /* Radius of the particle                                       */
        const unsigned int              _type;              /* The type of the material to associate it with a collider     */
};
}; /* namespace raptor_physics */
