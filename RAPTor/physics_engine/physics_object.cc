
/* Standard headers */
#include <algorithm>
#include <limits>
#include <memory>

/* Physics headers */
#include "physics_common.h"
#include "physics_object.h"
#include "integrators.h"
#include "gjk.h"

/* Shared headers */
#include "point_t.h"


namespace raptor_physics
{
const int collision_resolution_max = 50;

bool physics_object::has_collided(physics_object *const po, simplex **const manifold_a, 
    simplex **const manifold_b, point_t *const d, const fp_t t0, const fp_t t1)
{
    METHOD_LOG;

    /* Check for self intersection and return an empty simplex */
    *manifold_a = new simplex(*this);
    *manifold_b = new simplex(*po);
    if (po == this)
    {
        BOOST_LOG_TRIVIAL(trace) << "Ignoring self collision";
        return false;
    }
    
    /* Calculate the relative displacement */
    rk4_integrator integ;
    const point_t rel_disp(_i->center_of_mass() - po->_i->center_of_mass());
    const point_t rel_disp0(integ.project_translation(_agg_force, *_i, _v, t0 - _cur_t) - integ.project_translation(po->_agg_force, *po->_i, po->_v, t0 - po->_cur_t));
    const point_t rel_disp1(integ.project_translation(_agg_force, *_i, _v, t1 - _cur_t) - integ.project_translation(po->_agg_force, *po->_i, po->_v, t1 - po->_cur_t));
    const point_t rel_disp_diff(rel_disp1 - rel_disp0);
    const bool sig_disp = dot_product(rel_disp_diff, rel_disp_diff) > raptor_physics::EPSILON;
    BOOST_LOG_TRIVIAL(trace) << "Relative displacement: " << rel_disp0 << " -> " << rel_disp1;
    
    /* Test for collision */
    gjk tester(*_vg, *po->_vg, *manifold_a, *manifold_b);
    const bool collided = tester.find_minimum_distance(d, (sig_disp ? rel_disp0 : rel_disp1) + rel_disp, (sig_disp ? rel_disp_diff : point_t(0.0, 0.0, 0.0)), -_o, -po->_o);
    return collided;
}


/* TODO -- There has to be a faster way to handle objects in resting contact */
/* TODO -- Re-write using POSSIBLY_ to avoid some full collision detection */
collision_t physics_object::exactly_resolve_collisions(physics_object *const po, simplex **const manifold_a, simplex **const manifold_b, fp_t *const t)
{
    METHOD_LOG;

    /* Check for self intersection and return an empty simplex */
    if (po == this)
    {
        *manifold_a = new simplex(*this);
        *manifold_b = new simplex(*po);
        return NO_COLLISION;
    }

    const fp_t min_t = std::max(_cur_t, po->_cur_t);
    const fp_t max_t = *t;
    BOOST_LOG_TRIVIAL(trace) << "Solving within time range: " << min_t << " to " << max_t;

    /* Test at start position */
    point_t dir1;
    simplex *sim_a;
    simplex *sim_b;
    has_collided(po, &sim_a, &sim_b, &dir1, min_t, min_t);
    const fp_t d_t0 = magnitude(dir1);
    assert((d_t0 > raptor_physics::EPSILON) || !"Error: Object must start separated");

    /* Test during time period, note d_t1 <= dt_0 */
    point_t dir;
    has_collided(po, manifold_a, manifold_b, &dir, min_t, max_t);
    fp_t d_t1 = magnitude(dir);

    /* The objects never get within weld ditance, all is well */
    if (d_t1 > raptor_physics::WELD_DISTANCE)
    {
        delete sim_a;
        delete sim_b;
        BOOST_LOG_TRIVIAL(trace) << "Object separate, no collision";
        return NO_COLLISION;
    }
    /* The objects never get any closer, they are sliding or separating */
    else if ((d_t0 - d_t1) < raptor_physics::EPSILON)
    {
        delete sim_a;
        delete sim_b;
        delete *manifold_a;
        delete *manifold_b;
        has_collided(po, manifold_a, manifold_b, &dir, max_t, max_t);
        d_t1 = magnitude(dir);

        /* Look for objects moving slightly apart */
        /* It would be really bad if the contact force resolution slightly over did it here, but we have to be strict or there will be wobble */
        if ((d_t0 - d_t1) < 0.0)
        {
            BOOST_LOG_TRIVIAL(trace) << "Object separating, no collision";
            return NO_COLLISION;
        }
        /* The objects are already sliding */
        else
        {
            BOOST_LOG_TRIVIAL(trace) << "Sliding collision";
            return SLIDING_COLLISION;
        }
    }
    /* If distance at t0 is within the weld distance (and dt1 is closer) it is about to slide */
    else if (d_t0 < raptor_physics::WELD_DISTANCE)
    {
        const point_t noc(sim_a->normal_of_impact(*sim_b));
        if (dot_product(noc, get_velocity() - po->get_velocity()) > 0.0)
        {
            delete sim_a;
            delete sim_b;

            /* Multiple collision may happen in the time step, but the times and distances are so small we just want to find one */
            fp_t t_lo   = min_t;
            fp_t t_mid  = (min_t + max_t) * 0.5;
            fp_t t_hi   = max_t;
            int res_col_cnt = 0;
            do
            {
                delete *manifold_a;
                delete *manifold_b;

                BOOST_LOG_TRIVIAL(trace) << "Objects " << d_t0 << " (" << t_lo << ") apart and moving to " << t_hi << "(" << d_t1 << ")";
                has_collided(po, manifold_a, manifold_b, &dir, t_mid, t_mid);
                const fp_t d_mid = magnitude(dir);

                if ((d_mid > d_t0) && (dot_product(dir1, dir) > 0.0))
                {
                    t_lo = t_mid;
                }
                else
                {
                    t_hi = t_mid;
                    d_t1 = d_mid;
                }
                t_mid = (t_lo + t_hi) * 0.5;
                assert ((++res_col_cnt < collision_resolution_max) || !"Error: Stuck in the sliding collision resolution loop");
            } while (fabs(d_t0 - d_t1) > raptor_physics::EPSILON);

            *t = t_mid;
            BOOST_LOG_TRIVIAL(trace) << "Sliding collision about to happen at: " << t_mid;
            return SLIDING_COLLISION;
        }
        else
        {
            delete *manifold_a;
            delete *manifold_b;
            *manifold_a = sim_a;
            *manifold_b = sim_b;
            *t = min_t;
            return COLLISION;
        }
    }
    delete sim_a;
    delete sim_b;
    
    /* There was a collision bring the objects in contact without penetrating each other */
    BOOST_LOG_TRIVIAL(trace) << "Objects started apart at " << min_t << " (" << d_t0 << ") and collided by " << max_t << "(" << d_t1 << ")";
    int res_col_cnt = 0;
    std::pair<fp_t, fp_t> p_t0(min_t, d_t0);
    std::pair<fp_t, fp_t> p_t1(max_t, d_t1);
    fp_t adjusted_t = (p_t0.first + p_t1.first) * 0.5;
    while (true)
    {
        /* Clean the last (no longer needed) manifolds */
        delete *manifold_a;
        delete *manifold_b;
    
        /* Test for collisions */
        const bool collided = has_collided(po, manifold_a, manifold_b, &dir, min_t, adjusted_t);
        d_t1 = magnitude(dir);

        if ((d_t1 > (0.25 * raptor_physics::WELD_DISTANCE)) && (d_t1 < raptor_physics::WELD_DISTANCE) && ((d_t0 - d_t1) > raptor_physics::EPSILON))
        {
            break;
        }

        /* If the objects penetrate then rollback time */
        if (collided)
        {
            p_t1.first = adjusted_t;
            p_t1.second = d_t1;

            adjusted_t = (p_t0.first + p_t1.first) * 0.5;
        }
        /* Else use the "velocity" to estimate a contact time */
        else
        {
            /* Update p0 or p1 to keep the root bracketed */
            if (d_t1 > (0.25 * raptor_physics::WELD_DISTANCE))
            {
                p_t0.first = adjusted_t;
                p_t0.second = d_t1;
            }
            else
            {
                p_t1.first = adjusted_t;
                p_t1.second = d_t1;
            }

            const fp_t disp = p_t0.second - p_t1.second;
            const fp_t move = d_t1 - (0.5 * raptor_physics::WELD_DISTANCE);
            const fp_t last_adj = ((p_t1.first - p_t0.first) / disp) * move;
            adjusted_t = std::min(adjusted_t + last_adj, max_t);
        }
        assert(adjusted_t > (min_t + raptor_physics::EPSILON));
        assert((++res_col_cnt < collision_resolution_max) || !"Error: Stuck in the collision resolution loop");
    }
    
    *t = adjusted_t;
    return COLLISION;
}


collision_t physics_object::conservatively_resolve_collisions(physics_object *const po, simplex **const manifold_a, simplex **const manifold_b, fp_t *const t)
{
    METHOD_LOG;

    /* Check for self intersection and return an empty simplex */
    if (po == this)
    {
        *manifold_a = new simplex(*this);
        *manifold_b = new simplex(*po);
        return NO_COLLISION;
    }

    const fp_t min_t = std::max(_cur_t, po->_cur_t);
    const fp_t max_t = *t;
    BOOST_LOG_TRIVIAL(trace) << "Solving between " << min_t << " and " << max_t;
//    BOOST_LOG_TRIVIAL(trace) << "w: " << _w.x << " " << _w.y << " " << _w.z;

    /* Check location at time 0, this test is exact because no movement is involved */
    point_t dir;
    has_collided(po, manifold_a, manifold_b, &dir, min_t, min_t);

    /* Retreive the distance between the objects and the closest points */
    const fp_t d_t0 = magnitude(dir);
    BOOST_LOG_TRIVIAL(trace) << "Initial separation: " << d_t0;
    assert((d_t0 > raptor_physics::EPSILON) || !"Error: Object must start separated");
    
    point_t p_a((*manifold_a)->get_vertex(0));
    point_t p_b((*manifold_b)->get_vertex(0));
    vertex_to_global(&p_a);
    po->vertex_to_global(&p_b);
//    BOOST_LOG_TRIVIAL(trace) << "closest point on a: " << p_a.x << " " << p_a.y << " " << p_a.z;
//    BOOST_LOG_TRIVIAL(trace) << "closest point on b: " << p_b.x << " " << p_b.y << " " << p_b.z;

    /* Check the worst case distance at max_t */
    const point_t noc(dir / d_t0);
    const fp_t max_d = project_maximum_movement_onto((*po), p_a, p_b, noc, max_t);
    fp_t d_t1 = d_t0 - max_d;

    /* Objects start in contact */
    if (d_t0 < raptor_physics::WELD_DISTANCE)
    {
        if (d_t1 < raptor_physics::WELD_DISTANCE)
        {
            return close_contact_collision_detection(po, manifold_a, manifold_b, t, noc);
        }
        /* Penetrates so impact at min_t */
        // if (d_t1 < (0.25 * raptor_physics::WELD_DISTANCE))
        // {
        //     (*t) = min_t;
        //     BOOST_LOG_TRIVIAL(trace) << "resting collision: " << d_t1;
        //     return RESTING_COLLISION;
        // }
        /* Sliding so impact at max_t */
        // else if (d_t1 < raptor_physics::WELD_DISTANCE)
        // {
        //     (*t) = max_t;
        //     BOOST_LOG_TRIVIAL(trace) << "possible sliding: " << d_t1;
        //     return POSSIBLE_SLIDING_COLLISION;
        // }
        /* Splitting */
    }

    /* Never any contact or splitting */
    if (d_t1 > raptor_physics::WELD_DISTANCE)
    {
        BOOST_LOG_TRIVIAL(trace) << "no collision " << d_t1;
        return NO_COLLISION;
    }

    /* Iterate until no contact possible */
    int res_col_cnt = 0;
    fp_t adjusted_t = max_t;
    std::pair<fp_t, fp_t> p_t1;
    std::pair<fp_t, fp_t> p_t0(min_t, d_t0);
    BOOST_LOG_TRIVIAL(trace) << "Objects started apart at " << min_t << " (" << d_t0 << ") and collided by " << max_t << "(" << d_t1 << ")";
    do
    {
        assert ((++res_col_cnt < collision_resolution_max) || !"Error: Stuck in the collision resolution loop");

        /* Track the interval containing the collision */
        if (d_t1 < (0.25 * raptor_physics::WELD_DISTANCE))
        {
            p_t1.first = adjusted_t;
            p_t1.second = d_t1;
        }
        else
        {
            p_t0.first = adjusted_t;
            p_t0.second = d_t1;
        }

        /* Refine guess */
        const fp_t vel = (p_t0.second - p_t1.second) / (p_t1.first - p_t0.first);
        const fp_t adj = d_t1 - (0.5 * raptor_physics::WELD_DISTANCE);
        adjusted_t += adj / vel;

        /* Get worst case movement at adjusted t */
        const fp_t max_d = project_maximum_movement_onto((*po), p_a, p_b, noc, adjusted_t);

        /* Update worst case distance at adjusted t */
        d_t1 = d_t0 - max_d;
        BOOST_LOG_TRIVIAL(trace) << "Objects " << p_t0.second << " (" << p_t0.first << ") apart and moving to " << p_t1.second << "(" << p_t1.first << ")";
    } while ((d_t1 < (0.25 * raptor_physics::WELD_DISTANCE)) || (d_t1 > raptor_physics::WELD_DISTANCE));

    (*t) = adjusted_t;
    BOOST_LOG_TRIVIAL(trace) << "possible collision at: " << adjusted_t << " dist: " << d_t1;
    return POSSIBLE_COLLISION;
}


/* This function is to be called by a conservative collision detection scheme when two object are very close */
/* Conservative schemes do badly in this situation because the two objects will conservatively collide instantly */
collision_t physics_object::close_contact_collision_detection(physics_object *const po, simplex **const manifold_a, simplex **const manifold_b, fp_t *const t, const point_t &noc) const
{
    METHOD_LOG;
    BOOST_LOG_TRIVIAL(trace) << "Determining exact collision time up to: " << *t;

    /* Check for objects moving together, if so they collide instantly */
    // const fp_t va_dot_vb = dot_product(_v, po->_v);
    // if (va_dot_vb < 0.0)
    // {
    //     BOOST_LOG_TRIVIAL(trace) << "Objects translating towards each other, instant impact";
    //     *t = 0.0;
    //     return RESTING_COLLISION;
    // }
        
    const point_t n((*manifold_a)->normal_of_impact(**manifold_b));
    // const point_t poc((*manifold_a)->center_of_impact(**manifold_b, n));
    // if (va_dot_vb == 0.0)
    // {
    //     //if (dot_product(_v, n) < 0.0)
    //     if (dot_product(get_velocity(poc), n) < 0.0)
    //     {
    //         BOOST_LOG_TRIVIAL(trace) << "Object a translating towards b, instant impact";
    //         *t = 0.0;
    //         return RESTING_COLLISION;
    //     }

    //     //if (dot_product(po->_v, n) > 0.0)
    //     if (dot_product(po->get_velocity(poc), n) > 0.0)
    //     {
    //         BOOST_LOG_TRIVIAL(trace) << "Objects b translating towards a, instant impact";
    //         *t = 0.0;
    //         return RESTING_COLLISION;
    //     }
    // }

    fp_t full_t = *t;
    rk4_integrator integ;
    quaternion_t full_rot(normalise(integ.project_rotation(_agg_force, *_i, _o, _w, full_t - _cur_t)));
    while (fabs(full_rot.w) < 0.99)
    {
        full_t = (_cur_t + full_t) * 0.5;
        full_rot = integ.project_rotation(_agg_force, *_i, _o, _w, full_t);
        BOOST_LOG_TRIVIAL(trace) << "Only testing up to time: " << full_t << " because of fast rotation";
        assert((full_t > _cur_t) || !"Error: Unable to make progress because of extreme rotation");
    }

    /* Initial relative state, less the safety margin */
    const point_t x0(_i->center_of_mass() - po->_i->center_of_mass() - (n * (0.25 * raptor_physics::WELD_DISTANCE)));

    const fp_t r0 = 1.0;
    const point_t q0(0.0, 0.0, 0.0);

    /* Final relative state */
    const point_t x1(x0 + integ.project_translation(_agg_force, *_i, _v, full_t - _cur_t) - integ.project_translation(po->_agg_force, *po->_i, po->_v, full_t - po->_cur_t));

    const quaternion_t rot(integ.project_rotation(_agg_force, *_i, _o, _w, full_t - _cur_t));
    const point_t q1(rot.x, rot.y, rot.z);
    const fp_t r1 = rot.w;

    /* Compute exact time of intersection */
    const fp_t frac_t = _vg->find_intersection_time(*(po->_vg), n, x0, x1, q0, q1, r0, r1, full_t);
    
    BOOST_LOG_TRIVIAL(trace) << "Exact collision time at: " << (full_t * frac_t);
    if (frac_t <= 1.0) 
    {
        BOOST_LOG_TRIVIAL(trace) << "Resting collision";
        (*t) = 0.0;
        return SLIDING_COLLISION;
    }
    else if (full_t < *t)
    {
        *t = full_t;
        BOOST_LOG_TRIVIAL(trace) << "No collision in time limit, requesting retest at limit";
        return POSSIBLE_COLLISION;
    }
    else
    {
        BOOST_LOG_TRIVIAL(trace) << "No collision";
        return NO_COLLISION;
    }
}


physics_object& physics_object::commit_movement(const fp_t t)
{
    /* Update time */
    if (t < _cur_t)
    {
        return *this;
    }
    const fp_t dt = t - _cur_t;
    _cur_t = t;

    /* Rotate */
    point_t vel;
    rk4_integrator integ;
    _o += integ.project_rotation(_agg_force, *_i, &vel, _o, _w, dt);
    _w = vel;
    normalise(&_o);

    /* Translate */
    _i->move_center_of_mass(integ.project_translation(_agg_force, *_i, &vel, _v, dt));
    _v = vel;

    /* Derement forces */
    for (auto& f : (*_forces))
    {
        if (f->commit(dt))
        {
            delete f;
            f = nullptr;
        }
    }
    
    /* Remove spent forces */
    _forces->erase(std::remove(_forces->begin(), _forces->end(), nullptr), _forces->end());
    
    return *this;
}


fp_t physics_object::project_maximum_rotation_onto(const point_t &n, const point_t &po, const fp_t t) const
{
    rk4_integrator integ;
    const point_t apo_ang_vel(integ.project_translation(_agg_force, *_i, _v, t - _cur_t));

    /* Find the vertex moving the most along n */
    fp_t max_rot;
    const point_t w_cross_n(cross_product(apo_ang_vel, n));
//    BOOST_LOG_TRIVIAL(trace) << "n: " << n.x << " " << n.y << " " << n.z;
//    BOOST_LOG_TRIVIAL(trace) << "w cross n: " << w_cross_n.x << " " << w_cross_n.y << " " << w_cross_n.z;
    //const int rot_pot = raptor_physics::find_support_vertex(*_verts, w_cross_n, _i->center_of_mass(), po, n, &max_rot);
    _vg->find_support_vertex(apo_ang_vel, _i->center_of_mass(), po, n, &max_rot);
//    BOOST_LOG_TRIVIAL(trace) << "Most rotating point: " << _verts->get_row(rot_pot).x << " "  << _verts->get_row(rot_pot).y << " " << _verts->get_row(rot_pot).z;

    return max_rot;
}


fp_t physics_object::project_maximum_movement_onto(const physics_object &p, const point_t &p_a, const point_t &p_b, const point_t &n, const fp_t t) const
{
    /* Translation */
    rk4_integrator integ;
    const point_t tra_rel(integ.project_translation(p._agg_force, *p._i, p._v, t - p._cur_t) - integ.project_translation(_agg_force, *_i, _v, t - _cur_t));
    const fp_t tra = dot_product(tra_rel, n);

    /* Rotation */
    const fp_t rot = p.project_maximum_rotation_onto(-n, p_b, t) + project_maximum_rotation_onto(n, p_a, t);
    BOOST_LOG_TRIVIAL(trace) << "tra: " << tra << " rot: " << rot;

    /* Total */
    return tra + rot;
}
}; /* namespace raptor_physics */
