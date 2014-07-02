/* Standard headers */

/* Common headers */
#include "logging.h"

/* Physics headers */
#include "physics_engine.h"
#include "simplex.h"
#include "contact_graph.h"


namespace raptor_physics
{
physics_engine& physics_engine::advance_time(const fp_t t)
{
    METHOD_LOG;

    /* Time only goes one way */
    assert(t >= 0.0);
    
    /* Simulate multiple smaller time steps */
    fp_t t_now = 0.0;
    int frm_seg_cnt = 0;
    while (t_now < t)
    {
        /* Calculate the time step that means the max displacement cant be exceeded */
        fp_t max_v = 0.0;
        for (auto &p : (*_moving_objects))
        {
            max_v = std::max(max_v, p.second->get_speed());
        }    
        fp_t t_step = std::min(_max_d / max_v, t - t_now);
        BOOST_LOG_TRIVIAL(info) << "New frame segment, step size: " << t_step;
    

        /* Calculate the movement of all objects, but dont commit to it */
        for (auto &p : (*_moving_objects))
        {
            p.second->begin_time_step();
            BOOST_LOG_TRIVIAL(trace) << p.first << " started frame segment at: " << p.second->get_center_of_mass() << " moving: " << p.second->get_velocity() << " force: " << p.second->get_force();
        }

        for (auto &c : (*_collision_cache))
        {
            const int from_idx = std::find_if(_objects->begin(), _objects->end(), [=](const std::pair<int, physics_object*> &p)
                {
                    return p.second == c.first;
                })->first;

            for (auto &t : (*c.second))
            {
                const int to_idx = std::find_if(_objects->begin(), _objects->end(), [=](const std::pair<int, physics_object*> &p)
                    {
                        return p.second == t.first;
                    })->first;
                BOOST_LOG_TRIVIAL(trace) << "Normal between " << from_idx << " and " << to_idx << ": " << t.second->get_normal_of_collision();
            }
        }

        /* Retest sliding collisions */
        for (auto& p0 : (*_collision_cache))
        {
            for (auto& p1 : (*p0.second))
            {
                if (p1.second->get_type() == POSSIBLE_SLIDING_COLLISION)
                {
                    if (retest_and_cache(p0.first, p1.first, t_step))
                    {
                        BOOST_LOG_TRIVIAL(info) << "Possible sliding collision retested ok";
                    }
                }
            }
        }

        /* Foreach object in contact resolve forces with its contacts */
        BOOST_LOG_TRIVIAL(info) << "Beginning contact resolution";
        contact_graph<physics_object> cg;
        for (auto &p : (*_collision_cache))
        {
            /* Find all objects left in contact */
            // contact_graph_node *root = new contact_graph_node(_objects, *_collision_cache, p.first);
            // if (root->size() > 0)
            // {
            //     /* Iterate until the system settles */
            //     point_t inf_norm;
            //     do
            //     {
            //         inf_norm = root->resolve_forces_iteration(_objects, t_step);
            //     } while (magnitude(inf_norm) > raptor_physics::CONTACT_RESOLUTION_RESIDUAL);

            //     /* Clean up the contact graph */
            //     root->void_collisions(*this);
            // }

            // delete root;
            cg.rebuild(*_collision_cache, p.first);
            if (cg.number_of_edges() > 0)
            {
                cg.resolve_forces(t_step);
                cg.void_collisions(_collision_cache);
            }
        }

        BOOST_LOG_TRIVIAL(trace) << "After contact resolution:";
        for (auto &p : (*_objects))
        {
            BOOST_LOG_TRIVIAL(trace) << p.first << " started frame segment at: " << p.second->get_center_of_mass() << " moving: " << p.second->get_velocity() << " force: " << p.second->get_force();
        }

        /* Full collision detection */
        BOOST_LOG_TRIVIAL(info) << "Beginning full collision detection";
        for (auto p0 = _moving_objects->begin(); p0 != _moving_objects->end(); ++p0)
        {
            collision_detect_versus(p0, t_step, false);
        }


        /* Refine the collision in time order */
        int col_ref_cnt = 0;
        BOOST_LOG_TRIVIAL(info) << "Beginning collision refinement";
        while (true) 
        {
            /* If no collisions now, then find the earliest collision and process it */
            auto fc = std::find_if(_collision_cache->begin(), _collision_cache->end(), is_collision(NO_COLLISION, true));
            if (fc == _collision_cache->end())
            {
                /* No more collisions to process */
                BOOST_LOG_TRIVIAL(info) << "No collisions left to process";
                break;
            }

            for (auto p = _collision_cache->begin(); p != _collision_cache->end(); ++p)
            {
                if (((*p).second->get_first_collision_type() != NO_COLLISION) &&
                    ((*p).second->get_first_collision_time() < fc->second->get_first_collision_time()))
                {
                    fc = p;
                }
            }

            /* Check for iteration complete */
            const fp_t c_time = fc->second->get_first_collision_time();
            BOOST_LOG_TRIVIAL(info) << "Iteration time moved to: " << c_time;
            assert((c_time <= t_step) || !"Error: Stepped out the end of the times slot");
            if (fc->second->get_first_collision_type() == SLIDING_COLLISION)
            {
                BOOST_LOG_TRIVIAL(info) << "Next collision is sliding, moving to next frame for contact resolution";
                t_step = c_time;
                break;
            }

            /* Track the objects for repeat collisions */
            physics_object *vg_a = fc->first;
            physics_object *vg_b = fc->second->get_first_collision();
            collision_info *const col_a = (*(*_collision_cache)[vg_a])[vg_b];
            collision_info *const col_b = (*(*_collision_cache)[vg_b])[vg_a];
            if (col_a->switch_to_sliding() | col_b->switch_to_sliding()) /* Definately want bitwise or so both legs get run */
            {
                BOOST_LOG_TRIVIAL(trace) << "Moving to sliding, moving to next frame for contact resolution";
                t_step = c_time;
                break;
            }
            
            /* Move the objects */
            vg_a->commit_movement(c_time);
            vg_b->commit_movement(c_time);

            /* Check the collision is good */
            collision_t c_type = fc->second->get_first_collision_type();
            if (is_uncertain(c_type))
            {
                if (!retest_and_cache(vg_a, vg_b, t_step))
                {
                    /* Not actually colliding, try again later */
                    BOOST_LOG_TRIVIAL(info) << "Uncertain collision was conservative";
                    assert((++col_ref_cnt < 50) || !"Error: Stuck in the collision refinement loop.");
                    continue;
                }
                else
                {
                    BOOST_LOG_TRIVIAL(info) << "Uncertain collision retested ok";
                }
            }
            c_type = to_certain(c_type);

            /* Process the collision. */
            const point_t noc = col_a->get_normal_of_collision();
            const point_t poc = col_a->get_point_of_collision();
            pair_collider(vg_a->get_physical_type(), vg_b->get_physical_type())->collide(vg_a, vg_b, poc, noc, c_type);

            /* Void collisions */
            void_all_collisions_with(vg_a);
            void_all_collisions_with(vg_b);

            /* Collide these objects with all others */
            int vg_a_idx = (*std::find_if(_objects->begin(), _objects->end(), [&] (const std::pair<int, physics_object*>& p) { return p.second == vg_a; })).first;
            int vg_b_idx = (*std::find_if(_objects->begin(), _objects->end(), [&] (const std::pair<int, physics_object*>& p) { return p.second == vg_b; })).first;
            for (auto& p : (*_objects))
            {
                /* No need to re-collide a and b */
                if ((p.second == vg_a) || (p.second == vg_b))
                {
                    continue;
                }

                /* Re-collide */
                BOOST_LOG_TRIVIAL(trace) << "Refinement testing: " << vg_a_idx << " (vg_a) versus " << p.first;
                collide_and_cache(vg_a, p.second, t_step);
                BOOST_LOG_TRIVIAL(trace) << "Refinement testing: " << vg_b_idx << " (vg_b) versus " << p.first;
                collide_and_cache(vg_b, p.second, t_step);
            }

            /* Re-collide a and b, if they stick we need to use the contact resolution stage */
            BOOST_LOG_TRIVIAL(trace) << "Refinement testing: " << vg_a_idx << " (vg_a) versus " << vg_b_idx << " (vg_b)";
            collide_and_cache(vg_a, vg_b, t_step);
            assert((++col_ref_cnt < 50) || !"Error: Stuck in the collision refinement loop");
        }

        /* Commit all remaining movement */
        for (auto& p : (*_moving_objects))
        {
            p.second->commit_movement(t_step);
        }

        /* Update time */
        t_now += t_step;
        assert((frm_seg_cnt < 50) || !"Error: Stuck in the frame segment loop");
    }
    
    return *this;
}


bool physics_engine::collide_and_cache(physics_object *const vg_a, physics_object *const vg_b, const fp_t t)
{
    simplex *simplex_a;
    simplex *simplex_b;
    fp_t toc = t;
    collision_t collision_type = vg_a->resolve_collisions(vg_b, &simplex_a, &simplex_b, &toc);
    if (collision_type != NO_COLLISION)
    {
        /* Update all info */
        BOOST_LOG_TRIVIAL(trace) << "Collided at: " << toc;
        update_tracking_info(vg_a, vg_b, simplex_a, *simplex_b, toc, collision_type);
        update_tracking_info(vg_b, vg_a, simplex_b, *simplex_a, toc, collision_type);
    }
    else
    {
        delete simplex_a;
        delete simplex_b;
    }

    return false;
}


bool physics_engine::retest_and_cache(physics_object *const vg_a, physics_object *const vg_b, const fp_t t)
{
    simplex *simplex_a;
    simplex *simplex_b;
    fp_t toc = t;
    collision_t collision_type = vg_a->resolve_collisions(vg_b, &simplex_a, &simplex_b, &toc);
    if (collision_type != NO_COLLISION)
    {
        /* Retest as resting collsion means retest passed. Just update the simplices */
        if (collision_type != POSSIBLE_COLLISION)
        {
            update_tracking_info(vg_a, vg_b, simplex_a, *simplex_b);
            update_tracking_info(vg_b, vg_a, simplex_b, *simplex_a);

            return true;
        }
        /* Possible collision means not quite there yet */
        else
        {
            update_tracking_info(vg_a, vg_b, simplex_a, *simplex_b, toc, collision_type);
            update_tracking_info(vg_b, vg_a, simplex_b, *simplex_a, toc, collision_type);

            return false;
        }

    }

    /* Possible that wasnt anything */
    (*_collision_cache)[vg_a]->void_collision(vg_b);
    (*_collision_cache)[vg_b]->void_collision(vg_a);

    return false;
}


physics_engine& physics_engine::collision_detect_versus(const const_obj_iter &v, const fp_t t, const bool all)
{
    for (auto p = all ? _objects->begin() : v; p != _objects->end(); ++p)
    {
        collide_and_cache((*v).second, (*p).second, t);
    }

    return *this;
}
}; /* namespace raptor_physics */
