
/* Standard headers */
#include <algorithm>
#include <limits>
#include <memory>

/* Physics headers */
#include "physics_common.h"
#include "physics_object.h"
#include "gjk.h"

/* Shared headers */
#include "point_t.h"


namespace raptor_physics
{
const int collision_resolution_max = 50;

bool physics_object::has_collided(physics_object *const po, simplex **const manifold_a, simplex **const manifold_b, point_t<> *const d, const float t0, const float t1)
{
    return has_collided(po, manifold_a, manifold_b, d, _o, po->_o, t0, t1);
}

bool physics_object::has_collided(physics_object *const po, simplex **const manifold_a, simplex **const manifold_b, point_t<> *const d, const quaternion_t &oa, const quaternion_t &ob, const float t0, const float t1)
{
    // METHOD_LOG;

    /* Check for self intersection and return an empty simplex */
    *manifold_a = new simplex(*this, true);
    *manifold_b = new simplex(*po, false);
    if (po == this)
    {
        // BOOST_LOG_TRIVIAL(trace) << "Ignoring self collision";
        return false;
    }
    
    /* Calculate the relative displacement */
    const point_t<> rel_disp(_i->center_of_mass() - po->_i->center_of_mass());
    const point_t<> rel_disp0((_lv * (t0 - _cur_t)) - (po->_lv * (t0 - po->_cur_t)));
    const point_t<> rel_disp1((_lv * (t1 - _cur_t)) - (po->_lv * (t1 - po->_cur_t)));
    const point_t<> rel_disp_diff(rel_disp1 - rel_disp0);
    const bool sig_disp = dot_product(rel_disp_diff, rel_disp_diff) > raptor_physics::EPSILON;
    // BOOST_LOG_TRIVIAL(trace) << "Relative displacement: " << rel_disp0 << " -> " << rel_disp1;
    // BOOST_LOG_TRIVIAL(trace) << "Transformation a: " << (_lv * (t1 - _cur_t)) << ", orientation a: " << oa;
    // BOOST_LOG_TRIVIAL(trace) << "Transformation b: " << (po->_lv * (t1 - po->_cur_t)) << ", orientation b: " << ob;
    
    // BOOST_LOG_TRIVIAL(trace) << "Point 7 under transform: " << _vg->get_vertex(-oa, _i->center_of_mass() + (_lv * (t1 - _cur_t)), 7);

    /* Test for collision */
    gjk tester(*_vg, *po->_vg, *manifold_a, *manifold_b);
    const bool collided = tester.find_minimum_distance(d, (sig_disp ? rel_disp0 : rel_disp1) + rel_disp, (sig_disp ? rel_disp_diff : point_t<>(0.0, 0.0, 0.0)), oa, ob);
    return collided;
}

/* TODO -- There has to be a faster way to handle objects in resting contact */
/* TODO -- Re-write using POSSIBLY_ to avoid some full collision detection */
collision_t physics_object::exactly_resolve_collisions(physics_object *const po, simplex **const manifold_a, simplex **const manifold_b, float *const t)
{
    // METHOD_LOG;

    /* Check for self intersection and return an empty simplex */
    if (po == this)
    {
        *manifold_a = new simplex(*this, true);
        *manifold_b = new simplex(*po, false);
        return collision_t::NO_COLLISION;
    }

    const float min_t = std::max(_cur_t, po->_cur_t);
    const float max_t = *t;
    // BOOST_LOG_TRIVIAL(trace) << "Solving within time range: " << min_t << " to " << max_t;

    /* Test at start position */
    point_t<> dir1;
    simplex *sim_a;
    simplex *sim_b;
    has_collided(po, &sim_a, &sim_b, &dir1, min_t, min_t);
    const float d_t0 = magnitude(dir1);
    assert((d_t0 > raptor_physics::EPSILON) || !"Error: Object must start separated");

    /* Test during time period, note d_t1 <= dt_0 */
    point_t<> dir;
    has_collided(po, manifold_a, manifold_b, &dir, min_t, max_t);
    float d_t1 = magnitude(dir);

    /* The objects never get within weld ditance, all is well */
    if (d_t1 > raptor_physics::WELD_DISTANCE)
    {
        delete sim_a;
        delete sim_b;
        // BOOST_LOG_TRIVIAL(trace) << "Object separate, no collision";
        return collision_t::NO_COLLISION;
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
        if (d_t1 > (d_t0 + raptor_physics::EPSILON))
        {
            // BOOST_LOG_TRIVIAL(trace) << "Object separating, no collision";
            return collision_t::NO_COLLISION;
        }
        /* The objects are already sliding */
        else
        {
            // BOOST_LOG_TRIVIAL(trace) << "Sliding collision";
            return collision_t::SLIDING_COLLISION;
        }
    }
    /* If distance at t0 is within the weld distance (and dt1 is closer) it is about to slide */
    else if (d_t0 < raptor_physics::WELD_DISTANCE)
    {
        const point_t<> noc(sim_a->normal_of_impact(*sim_b));
        if (dot_product(noc, get_velocity() - po->get_velocity()) > 0.0)
        {
            delete sim_a;
            delete sim_b;

            /* Multiple collision may happen in the time step, but the times and distances are so small we just want to find one */
            float t_lo  = min_t;
            float t_mid = (min_t + max_t) * 0.5;
            float t_hi  = max_t;
            int res_col_cnt = 0;
            do
            {
                delete *manifold_a;
                delete *manifold_b;

                // BOOST_LOG_TRIVIAL(trace) << "Objects " << d_t0 << " (" << t_lo << ") apart and moving to " << t_hi << "(" << d_t1 << ")";
                has_collided(po, manifold_a, manifold_b, &dir, t_mid, t_mid);
                const float d_mid = magnitude(dir);

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
            // BOOST_LOG_TRIVIAL(trace) << "Sliding collision about to happen at: " << t_mid;
            return collision_t::SLIDING_COLLISION;
        }
        else
        {
            delete *manifold_a;
            delete *manifold_b;
            *manifold_a = sim_a;
            *manifold_b = sim_b;
            *t = min_t;
            return collision_t::COLLISION;
        }
    }
    delete sim_a;
    delete sim_b;
    
    /* There was a collision bring the objects in contact without penetrating each other */
    // BOOST_LOG_TRIVIAL(trace) << "Objects started apart at " << min_t << " (" << d_t0 << ") and collided by " << max_t << "(" << d_t1 << ")";
    int res_col_cnt = 0;
    std::pair<float, float> p_t0(min_t, d_t0);
    std::pair<float, float> p_t1(max_t, d_t1);
    float adjusted_t = (p_t0.first + p_t1.first) * 0.5;
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

            const float disp = p_t0.second - p_t1.second;
            const float move = d_t1 - (0.5 * raptor_physics::WELD_DISTANCE);
            const float last_adj = ((p_t1.first - p_t0.first) / disp) * move;
            adjusted_t = std::min(adjusted_t + last_adj, max_t);
        }
        assert(adjusted_t > (min_t + raptor_physics::EPSILON));
        assert((++res_col_cnt < collision_resolution_max) || !"Error: Stuck in the collision resolution loop");
    }
    
    *t = adjusted_t;
    return collision_t::COLLISION;
}


collision_t physics_object::conservatively_resolve_collisions(physics_object *const po, simplex **const manifold_a, simplex **const manifold_b, float *const t, const bool sliding)
{
    // METHOD_LOG;

    /* Check for self intersection and return an empty simplex */
    if (po == this)
    {
        *manifold_a = new simplex(*this, true);
        *manifold_b = new simplex(*po, false);
        return collision_t::NO_COLLISION;
    }

    const float min_t = std::max(_cur_t, po->_cur_t);
    const float max_t = *t;
    // BOOST_LOG_TRIVIAL(trace) << "Solving between " << min_t << " and " << max_t;
   // BOOST_LOG_TRIVIAL(trace) << "w: " << _lw;

    /* Check location at time 1, this test is exact because no movement is involved */
    /* This test may miss collisions between time 0 and 1 */
    point_t<> dir;
    // const quaternion_t r2(normalise(_o + (quaternion_t(0.0f, _lw.x, _lw.y, _lw.z) * _o) * (0.5f * (max_t - _cur_t))));
    // const quaternion_t r3(normalise(po->_o + (quaternion_t(0.0f, po->_lw.x, po->_lw.y, po->_lw.z) * po->_o) * (0.5f * (max_t - po->_cur_t))));
    // has_collided(po, manifold_a, manifold_b, &dir, r2, r3, max_t, max_t);
    // float d_t1 = magnitude(dir);

    /* Check location at time 0, this test is exact because no movement is involved */
    const quaternion_t r0(normalise(_o + (quaternion_t(0.0f, _lw.x, _lw.y, _lw.z) * _o) * (0.5f * (min_t - _cur_t))));
    const quaternion_t r1(normalise(po->_o + (quaternion_t(0.0f, po->_lw.x, po->_lw.y, po->_lw.z) * po->_o) * (0.5f * (min_t - po->_cur_t))));
    has_collided(po, manifold_a, manifold_b, &dir, r0, r1, min_t, min_t);
    const float d_t0 = magnitude(dir);
    assert((d_t0 > ((0.25f * raptor_physics::WELD_DISTANCE) - raptor_physics::EPSILON)) || !"Error: Object must start separated");

    /* Objects start in contact */
    const point_t<> noc(dir / d_t0);
    if (d_t0 < raptor_physics::WELD_DISTANCE)
    {
        // if (d_t1 < (0.25f * raptor_physics::WELD_DISTANCE))
        // {
            return close_contact_collision_detection(po, manifold_a, manifold_b, t, noc, min_t, d_t0, sliding);
        // }
        // else
        // {
            // BOOST_LOG_TRIVIAL(trace) << "Sliding collision";
            // return collision_t::SLIDING_COLLISION;
        // }
    }

    /* Check the worst case distance at max_t */
    point_t<> p_a((*manifold_a)->get_vertex(0));
    point_t<> p_b((*manifold_b)->get_vertex(0));
    vertex_to_global(&p_a);
    po->vertex_to_global(&p_b);
    const float max_d = project_maximum_movement_onto((*po), p_a, p_b, noc, max_t);
    float d_t1 = d_t0 - max_d;

    /* The objects never get within weld ditance, all is well */
    if (d_t1 > raptor_physics::WELD_DISTANCE)
    {
        BOOST_LOG_TRIVIAL(trace) << "Object separate, no collision d t0: " << d_t0 << ", d t1: " << d_t1;
        return collision_t::NO_COLLISION;
    }

    /* Iterate until no contact possible */
    int res_col_cnt = 0;
    float adjusted_t = max_t;
    std::pair<float, float> p_t1;
    std::pair<float, float> p_t0(min_t, d_t0);
    BOOST_LOG_TRIVIAL(trace) << "Objects started apart at " << min_t << " (" << d_t0 << ") and collided by " << max_t << "(" << d_t1 << ")";
    while ((d_t1 < (0.25f * raptor_physics::WELD_DISTANCE)) || (d_t1 > raptor_physics::WELD_DISTANCE))
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
        const float vel = (p_t0.second - p_t1.second) / (p_t1.first - p_t0.first);
        const float adj = d_t1 - (0.5 * raptor_physics::WELD_DISTANCE);
        adjusted_t += adj / vel;
        // BOOST_LOG_TRIVIAL(trace) << "New guess: " << adjusted_t << ", min: " << min_t;

        /* Get worst case movement at adjusted t */
        const float max_d = project_maximum_movement_onto((*po), p_a, p_b, noc, adjusted_t);

        /* Update worst case distance at adjusted t */
        d_t1 = d_t0 - max_d;
        // BOOST_LOG_TRIVIAL(trace) << "Objects " << p_t0.second << " (" << p_t0.first << ") apart and moving to " << p_t1.second << "(" << p_t1.first << ")";
    }

    (*t) = adjusted_t;
    // BOOST_LOG_TRIVIAL(trace) << "possible collision at: " << adjusted_t << " dist: " << d_t1;
    return collision_t::POSSIBLE_COLLISION;
}


/* This function is to be called by a conservative collision detection scheme when two object are too close */
/* Conservative schemes do badly in this situation because the two objects will conservatively collide instantly */
collision_t physics_object::close_contact_collision_detection(physics_object *const po, simplex **const manifold_a, simplex **const manifold_b, float *const toc, const point_t<> &noc, float min_t, const float d_t0, const bool sliding)
{
    // METHOD_LOG;
    BOOST_LOG_TRIVIAL(trace) << "Determining exact collision time up to: " << *toc;
    
    /* Pick an object to act as the plane */
    simplex *pm = *manifold_a;
    simplex *om = *manifold_b;
    const physics_object *pl = this;
    const physics_object *ol = po;
    // BOOST_LOG_TRIVIAL(trace) << "Simplex sizes: " << (*manifold_a)->unique_size() << " and " << (*manifold_b)->unique_size();
    if ((*manifold_b)->unique_size() >= (*manifold_a)->unique_size())
    {
        std::swap(pl, ol);
        std::swap(pm, om);
        // BOOST_LOG_TRIVIAL(trace) << "Using other object for plane";
    }

    /* Get the definition of the plane*/
    point_t<> plane_n0(noc);
    if (pl == this)
    {
        plane_n0 = -plane_n0;
    }
    const quaternion_t plo_inv(-pl->_o);
    const point_t<> plane_n(plo_inv.rotate(plane_n0));

    const point_t<> plane_w(pm->get_vertex(0));
    const point_t<> plane_w0(pl->_o.rotate(plane_w) + pl->_i->center_of_mass());
    // BOOST_LOG_TRIVIAL(trace) << "Plane normal: " << plane_n << ", plane point: " << plane_w;
    // BOOST_LOG_TRIVIAL(trace) << "Points in initial config";
    // for (int i = 0; i < ol->_vg->get_number_of_vertices(); ++i)
    // {
        // const point_t<> p(ol->_vg->get_vertex(ol->_o, ol->_i->center_of_mass(), i));
        // BOOST_LOG_TRIVIAL(trace) << "Point: " << i << " at: " << p;
    // }

    /* Check for min_t collision */
    point_t<> ol_t;
    quaternion_t ol_r;
    ol->configuration_at_time(&ol_r, &ol_t, min_t);

    point_t<> pl_t;
    quaternion_t pl_r;
    pl->configuration_at_time(&pl_r, &pl_t, min_t);


    if (!sliding)
    {
        /* Get velocity of ol */
        const point_t<> ol_poc(pm->center_of_impact(*om, pm->normal_of_impact(*om)) - ol->_i->center_of_mass());
        const point_t<> ol_speed(ol->_lv + cross_product(ol->_lw, ol_poc));

        /* Get velocity of pl */
        const point_t<> poc(ol_r.rotate(ol_poc) + ol_t);
        const point_t<> pl_poc(pl_r.rotate(poc - pl_t));
        const point_t<> pl_speed(pl->_lv + cross_product(pl->_lw, pl_poc));

        /* Get the closing velocity */
        const point_t<> speed_noc(pl_r.rotate(plane_n));
        BOOST_LOG_TRIVIAL(trace) << "Closing velocity: " << (pl_speed - ol_speed) << ", ol v: " << ol->_lv << ", ol w: " << cross_product(ol->_lw, ol_poc);    
        if (dot_product(speed_noc, pl_speed - ol_speed) > 0.0f)
        {
            (*toc) = min_t;
            BOOST_LOG_TRIVIAL(trace) << "Object colliding at: " << (*toc) << ", distance: " << d_t0;
            return collision_t::COLLISION;
        }
    }

    /* Find target configuration */
    ol->configuration_at_time(&ol_r, &ol_t, (*toc));
    pl->configuration_at_time(&pl_r, &pl_t, (*toc));
        
    /* Find the deepest point below the plane at t */
    float max_pen;
    int max_idx = find_deepest_intersection(*ol->_vg, pl_r, ol_r, pl_t, ol_t, plane_n, plane_w, &max_pen);

    /* Distance is increasing, no collision */
    if (max_pen > (d_t0 + raptor_physics::EPSILON))
    {
        BOOST_LOG_TRIVIAL(trace) << "Object separate, no collision";
        return collision_t::NO_COLLISION;
    }
    /* The objects never get too close, they are sliding */
    else if (max_pen > (0.25f * raptor_physics::WELD_DISTANCE))
    {
        BOOST_LOG_TRIVIAL(trace) << "Sliding collision";
        return collision_t::SLIDING_COLLISION;
    }

    /* Objecs collision, find out when */
    BOOST_LOG_TRIVIAL(trace) << "Objects started: " << d_t0 << " apart and moving to: " << max_pen;
    int res_bil_col = 0;
    do
    {
        assert((++res_bil_col < 10) || !"Error: Stuck in the bilateral collision resolution loop");

        /* Find a time that the deepest point came in contact */
        const point_t<> p0(ol->_vg->get_vertex(ol->_o, ol->_i->center_of_mass(), max_idx));
        const float dist0 = dot_product(p0 - plane_w0, plane_n0);

        int res_max_pen = 0;
        float new_pen = max_pen;
        float adjusted_t = (*toc);
        std::pair<float, float> p_t1;
        std::pair<float, float> p_t0(min_t, dist0);
        while ((new_pen < (0.25f * raptor_physics::WELD_DISTANCE)) || (new_pen > raptor_physics::WELD_DISTANCE))
        {
            /* Track the interval containing the collision */
            assert ((++res_max_pen < 25) || !"Error: Stuck in the max penetration resolution loop");
            if (new_pen < (0.25f * raptor_physics::WELD_DISTANCE))
            {
                p_t1.first = adjusted_t;
                p_t1.second = new_pen;
            }
            else
            {
                p_t0.first = adjusted_t;
                p_t0.second = new_pen;
            }

            /* Refine guess using the secant rule */
            const float vel = (p_t1.second - p_t0.second) / (p_t1.first - p_t0.first);
            const float adj = (0.5f * raptor_physics::WELD_DISTANCE) - p_t0.second;
            adjusted_t = p_t0.first + (adj / vel);
            // BOOST_LOG_TRIVIAL(trace) << "In time: " << p_t0.first << " -> " << p_t1.first << ", moving: " << p_t0.second << " -> " << p_t1.second << ", vel = " << vel;
            // BOOST_LOG_TRIVIAL(trace) << "Would like to move distance: " << adj << " = " << (0.5f * raptor_physics::WELD_DISTANCE)  << " - " << p_t0.second;
            // BOOST_LOG_TRIVIAL(trace) << "Set new guess: " << adjusted_t << ", min time: " << min_t;

            /* These are not the roots you are looking for, try bisection */
            if ((adjusted_t < (min_t + raptor_physics::EPSILON)) || (adjusted_t > ((*toc) + raptor_physics::EPSILON)))
            {
                adjusted_t = p_t0.first + ((p_t1.first - p_t0.first) * 0.5f);
                // BOOST_LOG_TRIVIAL(trace) << "Switched to bisection, new guess: " << adjusted_t << ", min time: " << min_t;
            }

            /* Update target configuration */
            ol->configuration_at_time(&ol_r, &ol_t, adjusted_t);
            pl->configuration_at_time(&pl_r, &pl_t, adjusted_t);

            /* Update distance */
            const point_t<> plane_ng(pl_r.rotate(plane_n));
            const point_t<> plane_wg(pl_r.rotate(plane_w) + pl_t);
            const point_t<> p(ol->_vg->get_vertex(ol_r, ol_t, max_idx));
            new_pen = dot_product(p - plane_wg, plane_ng);
            BOOST_LOG_TRIVIAL(trace) << "Time guess: " << adjusted_t << " new distance: " << new_pen;

        }
        // BOOST_LOG_TRIVIAL(trace) << "Point moved out of surface, moving to next point";
        
        /* Find the deepest point below the plane at t */
        max_idx = find_deepest_intersection(*ol->_vg, pl_r, ol_r, pl_t, ol_t, plane_n, plane_w, &max_pen);

        /* Time of collision was ok */
        (*toc) = adjusted_t;
        if ((max_pen > (0.25f * raptor_physics::WELD_DISTANCE)) && (max_pen < raptor_physics::WELD_DISTANCE))
        {
            // BOOST_LOG_TRIVIAL(trace) << "Object possibly colliding at: " << (*toc);
            break;
        }
    } while (max_pen < raptor_physics::WELD_DISTANCE);

    return collision_t::POSSIBLE_COLLISION;
}

void physics_object::configuration_at_time(quaternion_t *const o, point_t<> *const tr, const float t) const
{
    const float dt = t - _cur_t;
    *tr = _i->center_of_mass() + (_lv * dt);
    *o = normalise(_o + (quaternion_t(0.0f, _lw.x, _lw.y, _lw.z) * _o) * (dt * 0.5f));
    // BOOST_LOG_TRIVIAL(trace) << "Target translation: " << (*tr) << " and orientation: " << (*o);
}


int physics_object::find_deepest_intersection(const vertex_group &vg, const quaternion_t &pl_r, const quaternion_t &ol_r, const point_t<> &pl_t, const point_t<> &ol_t, const point_t<> &plane_n, const point_t<> &plane_w, float *const d) const
{
    int max_idx = -1;
    float max_pen = raptor_physics::WELD_DISTANCE;
    const point_t<> plane_ng(pl_r.rotate(plane_n));
    const point_t<> plane_wg(pl_r.rotate(plane_w) + pl_t);
    // BOOST_LOG_TRIVIAL(trace) << "Plane point: " << plane_wg << ", and normal: " << plane_ng;
    for (int i = 0; i < vg.get_number_of_vertices(); ++i)
    {
        const point_t<> p(vg.get_vertex(ol_r, ol_t, i));
        const float dist = dot_product(p - plane_wg, plane_ng);
        // BOOST_LOG_TRIVIAL(trace) << "Point: " << vg.get_vertex(i) << " moving to: " << p << ", dist: " << dist;
        if (dist < max_pen)
        {
            max_idx = i;
            max_pen = dist;
        }
    }
    // BOOST_LOG_TRIVIAL(trace) << "Deepest point: " << max_idx << ", sank to: " << max_pen;

    (*d) = max_pen;
    return max_idx;
}

float physics_object::project_maximum_rotation_onto(const point_t<> &n, const point_t<> &po, const float t) const
{
    const float dt = t - _cur_t;

    /* Find the vertex moving the most along n */
    float max_rot;
    const point_t<> apo_ang_vel(_lw * dt);
    const point_t<> w_cross_n(cross_product(apo_ang_vel, n));
   // BOOST_LOG_TRIVIAL(trace) << "n: " << n;
   // BOOST_LOG_TRIVIAL(trace) << "w0: " << _lw << ", w1: " << vel << ", apo_ang_vel: " << apo_ang_vel;
   // BOOST_LOG_TRIVIAL(trace) << "w cross n: " << w_cross_n;
    //const int rot_pot = raptor_physics::find_support_vertex(*_verts, w_cross_n, _i->center_of_mass(), po, n, &max_rot);
    _vg->find_support_vertex(apo_ang_vel, _i->center_of_mass(), po, n, &max_rot);

    return max_rot;
}


float physics_object::project_maximum_movement_onto(const physics_object &p, const point_t<> &p_a, const point_t<> &p_b, const point_t<> &n, const float t) const
{
    /* Translation */
    const point_t<> tra_rel((p._lv * (t - p._cur_t)) - (_lv * (t - _cur_t)));
    const float tra = dot_product(tra_rel, n);

    /* Rotation */
    const float rot = p.project_maximum_rotation_onto(-n, p_b, t) + project_maximum_rotation_onto(n, p_a, t);
    // BOOST_LOG_TRIVIAL(trace) << "tra: " << tra << " rot: " << rot;

    /* Total */
    return tra + rot;
}

physics_object& physics_object::commit_movement(const float t)
{
    // METHOD_LOG;

    /* Update time */
    if (t < _cur_t)
    {
        return *this;
    }
    const float dt = t - _cur_t;
    _cur_t = t;

    /* Rotate */
    point_t<> vel;
    rk4_integrator integ;

    integ.project_rotation(_agg_force, *_i, &vel, _o, _w, dt);
    _w = vel;
    _o += (quaternion_t(0.0f, _lw.x, _lw.y, _lw.z) * _o) * (dt * 0.5f);
    normalise(&_o);

    /* Translate */
    integ.project_translation(_agg_force, *_i, &vel, _v, dt);
    _v = vel;
    _i->move_center_of_mass(_lv * dt);

    /* Update bounds for the remainder of the time step */
    update_bounds();

    /* Update the descrtised motion for this frame */
    const float lv_sq = dot_product(_lv, _lv);
    if (fabs(lv_sq) > raptor_physics::EPSILON)
    {
        _lv = (_lv / std::sqrt(lv_sq)) * magnitude(_v);
    }

    const float lw_sq = dot_product(_lw, _lw);
    if (fabs(lw_sq) > raptor_physics::EPSILON)
    {
        _lw = (_lw / std::sqrt(lw_sq)) * magnitude(_w);
    }

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
}; /* namespace raptor_physics */
