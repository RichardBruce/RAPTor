/* Standard headers */
#include <limits>

/* Physics headers */
#include "contact_graph_node.h"
#include "physics_common.h"
#include "physics_object.h"
#include "physics_engine.h"


namespace raptor_physics
{
point_t contact_graph_node::resolve_forces_iteration(std::unordered_map<int, physics_object*> *o, const float t_step)
{
    std::pair<point_t, point_t> perp_f;
    std::pair<point_t, point_t> proj_f;
    std::pair<point_t, point_t> perp_v;
    std::pair<point_t, point_t> proj_v;

    /* One iteration of resolution */
    point_t node_f(_r->get_force());
    point_t node_v(_r->get_velocity());
    point_t inf_norm(0.0, 0.0, 0.0);
    for (auto &c : _c)
    {
//        int vg_a_idx = (*std::find_if(o->begin(), o->end(), [&] (const std::pair<int, physics_object*>& p) { return p.second == _r; })).first;
//        int vg_b_idx = (*std::find_if(o->begin(), o->end(), [&] (const std::pair<int, physics_object*>& p) { return p.second == c->_r; })).first;
//        cout << "Resolving forces between: " << vg_a_idx << " and " << vg_b_idx << endl;

        /* Get the components */
        const point_t noc((*_t)[c->_r]->get_normal_of_collision());
        componentise(&perp_f, &proj_f, node_f, c->_r->get_force(), noc);
        componentise(&perp_v, &proj_v, node_v, c->_r->get_velocity(),noc);

        /* Resolve forces and velocities */
        const point_t perp_chg(calculate_perpendicular_force(&perp_f, proj_f, perp_v,
            noc, _r->get_mass(), c->_r->get_mass(), 0.1, 0.1, t_step));

        const point_t proj_chg(calculate_projected_forces(&proj_f, noc, _r->get_mass(), c->_r->get_mass()));

        const point_t vel_chg(average_volocities(&proj_v, _r->get_mass(), c->_r->get_mass()));

        /* Rebuild the forces and velocities */
        node_f = perp_f.first + proj_f.first;
        c->_r->set_force(floor_to_zero(perp_f.second + proj_f.second, raptor_physics::TOO_SMALL_F));
        
        node_v = perp_v.first + proj_v.first;
        c->_r->set_velocity(floor_to_zero(perp_v.second + proj_v.second, raptor_physics::TOO_SMALL_V));

        /* Update the infinity norm */
        inf_norm = max(inf_norm, proj_chg);
        inf_norm = max(inf_norm, perp_chg);
        inf_norm = max(inf_norm, vel_chg);
    }

    /* Rebuild the root node */
    _r->set_force(floor_to_zero(node_f, raptor_physics::TOO_SMALL_F));
    _r->set_velocity(floor_to_zero(node_v, raptor_physics::TOO_SMALL_V));

    /* Recurse, dont process the last node, its where we came from */
    for (auto &c : _c)
    {
        inf_norm = max(inf_norm, c->resolve_forces_iteration(o, t_step));
    }

    /* Return the infinity normal */
    return inf_norm;
}


const contact_graph_node& contact_graph_node::void_collisions(physics_engine &p) const
{
    p.void_all_collisions_with(_r);
    for (auto &c : _c)
    {
        c->void_collisions(p);
    }

    return *this;
}


const contact_graph_node& contact_graph_node::componentise(std::pair<point_t, point_t> *const perp, std::pair<point_t, point_t> *const proj, const point_t &a, const point_t &b, const point_t &noc) const
{
    using raptor_physics::project_vector;
    
    /* Project the forces */
    const point_t for_a_proj(project_vector(a, -noc));
    const point_t for_a_perp(a - for_a_proj);

    const point_t for_b_proj(project_vector(b,  noc));
    const point_t for_b_perp(b - for_b_proj);

    /* Append to the results */
    proj->first     = for_a_proj;
    perp->first     = for_a_perp;
    proj->second    = for_b_proj;
    perp->second    = for_b_perp;

    return *this;
}


point_t contact_graph_node::calculate_projected_forces(std::pair<point_t, point_t> *const proj_f, const point_t &noc, const float m_a, const float m_b) const
{
    /* Save the original state */
    const point_t first(proj_f->first);
    const point_t second(proj_f->second);

    /* Process the forces acting on the objects */
    const float total_m = m_a + m_b;
    const float a_dot_noc = dot_product(proj_f->first, noc);

    /* Forces push in the same direction */
//    cout << "Input projected forces: " << endl;
//    cout << "First: " << proj_f->first.x << " " << proj_f->first.y << " " << proj_f->first.z << endl;
//    cout << "Secon: " << proj_f->second.x << " " << proj_f->second.y << " " << proj_f->second.z << endl;
    if (dot_product(proj_f->first, proj_f->second) > 0.0)
    {
        const point_t acc_a_proj(proj_f->first / m_a);
        const point_t acc_b_proj(proj_f->second / m_b);

        const float acc_a_magn = magnitude(acc_a_proj);
        const float acc_b_magn = magnitude(acc_b_proj);

        /* Trailer (a) accelerating fastest */
        if ((acc_a_magn > acc_b_magn) && (a_dot_noc < 0.0))
        {
//            cout << "trailing a fastest" << endl;
            if (total_m == numeric_limits<float>::infinity())
            {
                proj_f->first   = 0.0;
                proj_f->second  = 0.0;
            }
            else
            {
                const point_t net_a_proj(acc_a_proj - acc_b_proj);
                const point_t net_f_proj(net_a_proj * m_a);
                proj_f->first   = (proj_f->first - net_f_proj) + (net_f_proj * (m_a / total_m));
                proj_f->second  = proj_f->second + (net_f_proj * (m_b / total_m));
            }
        }
        /* Trailer (b) accelerating fastest */
        else if ((acc_b_magn > acc_a_magn) && (a_dot_noc > 0.0))
        {
//            cout << "trailing b fastest" << endl;
            if (total_m == numeric_limits<float>::infinity())
            {
                proj_f->first   = 0.0;
                proj_f->second  = 0.0;
            }
            else
            {
                const point_t net_a_proj(acc_b_proj - acc_a_proj);
                const point_t net_f_proj(net_a_proj * m_b);
                proj_f->first   = proj_f->first + (net_f_proj * (m_a / total_m));
                proj_f->second  = (proj_f->second - net_f_proj) + (net_f_proj * (m_b / total_m));
            }
        }
        /* Leader accelerating fastest, no forces to update */
    }
    /* Forces push in opposite directions */
    else
    {
        /* Pushing against each other */
        const float b_dot_noc = dot_product(proj_f->second, noc);
        if ((a_dot_noc < 0.0) || (b_dot_noc > 0.0))
        {
//            cout << "pushing together" << endl;
            if (total_m == numeric_limits<float>::infinity())
            {
                proj_f->first   = 0.0;
                proj_f->second  = 0.0;
            }
            else
            {
                const point_t net_f_proj(proj_f->first + proj_f->second);
                proj_f->first   = (net_f_proj * (m_a / total_m));
                proj_f->second  = (net_f_proj * (m_b / total_m));
            }
        }
        /* Pushing apart */
    }

   cout << "Resultant projected forces: " << endl;
   cout << "First: " << proj_f->first.x << " " << proj_f->first.y << " " << proj_f->first.z << endl;
   cout << "Secon: " << proj_f->second.x << " " << proj_f->second.y << " " << proj_f->second.z << endl;

    /* Return the maximum percent change */
    const point_t first_chg((first - proj_f->first) / first);
    const point_t second_chg((second - proj_f->second) / second);
    return max(fabs(first_chg), fabs(second_chg));

}


point_t contact_graph_node::calculate_perpendicular_force(std::pair<point_t, point_t> *const perp_f, const std::pair<point_t, point_t> &proj_f, const std::pair<point_t, point_t> &perp_v, const point_t &noc, const float m_a, const float m_b, const float mus, const float muk, const float t) const
{
    /* Save the original state */
    const point_t first(perp_f->first);
    const point_t second(perp_f->second);

    /* Process the forces acting on the objects */
    const float a_dot_noc = dot_product(proj_f.first, noc);
    const point_t net_v_perp(perp_v.first - perp_v.second);

    /* Forces push in the same direction */
//    cout << "Input perphendicular forces: " << endl;
//    cout << "net v: " << net_v_perp.x << " " << net_v_perp.y << " " << net_v_perp.z << endl;
//    cout << "First: " << perp_f->first.x << " " << perp_f->first.y << " " << perp_f->first.z << endl;
//    cout << "Secon: " << perp_f->second.x << " " << perp_f->second.y << " " << perp_f->second.z << endl;
    if (dot_product(proj_f.first, proj_f.second) > 0.0)
    {
        const point_t acc_a_proj(proj_f.first / m_a);
        const point_t acc_b_proj(proj_f.second / m_a);

        const float acc_a_magn = magnitude(acc_a_proj);
        const float acc_b_magn = magnitude(acc_b_proj);

        /* Trailer (a) accelerating fastest */
        if ((acc_a_magn > acc_b_magn) && (a_dot_noc < 0.0))
        {
            const point_t net_f_proj(proj_f.first - proj_f.second);
            apply_friction(perp_f, net_f_proj, net_v_perp, mus, muk, m_a, m_b, t);
        }
        /* Trailer (b) accelerating fastest */
        else if ((acc_b_magn > acc_a_magn) && (a_dot_noc > 0.0))
        {
            const point_t net_f_proj(proj_f.second - proj_f.first);
            apply_friction(perp_f, net_f_proj, net_v_perp, mus, muk, m_a, m_b, t);
        }
        /* Leader accelerating fastest, no forces to update */
    }
    /* Forces push in opposite directions */
    else
    {
        /* Pushing against each other */
        const float b_dot_noc = dot_product(proj_f.second, noc);
        if ((a_dot_noc < 0.0) || (b_dot_noc > 0.0))
        {
            const point_t net_f_proj(proj_f.first - proj_f.second);
            apply_friction(perp_f, net_f_proj, net_v_perp, mus, muk, m_a, m_b, t);
        }
        /* Pushing apart */
    }

//    cout << "Resultant perphendicular forces: " << endl;
//    cout << "First: " << perp_f->first.x << " " << perp_f->first.y << " " << perp_f->first.z << endl;
//    cout << "Secon: " << perp_f->second.x << " " << perp_f->second.y << " " << perp_f->second.z << endl;

    /* Return the maximum percent change */
    const point_t first_chg((first - perp_f->first) / first);
    const point_t second_chg((second - perp_f->second) / second);
    return max(fabs(first_chg), fabs(second_chg));
}


const contact_graph_node& contact_graph_node::apply_friction(std::pair<point_t, point_t> *const perp_f, const point_t &net_f_proj, const point_t &net_v_perp, const float mus, const float muk, const float m_a, const float m_b, const float t) const
{
    const float magn_f_proj = magnitude(net_f_proj);

    /* Kinetic friction */
    if (magnitude(net_v_perp) > raptor_physics::EPSILON)
    {
        const float fric_k = magn_f_proj * muk;
//        cout << "Friction k: " << fric_k << endl;
        const point_t norm_v_perp = normalise(net_v_perp);

        const point_t max_f_fric(norm_v_perp * fric_k);
        const point_t chg_a_vel((perp_f->first - max_f_fric) * (t / m_a));
        const point_t chg_b_vel((perp_f->second + max_f_fric) * (t / m_b));
        const point_t chg_v_perp(chg_a_vel - chg_b_vel);

        /* Bring the objects to a relative halt */
        if (dot_product(net_v_perp, chg_v_perp) < 0.0)
        {
//            cout << "full friction would change direction" << endl;
            /* Apply symetric force */
            if ((m_a != numeric_limits<float>::infinity()) && (m_b != numeric_limits<float>::infinity()))
            {
                const point_t chg_a_perp((net_v_perp * 0.5) / t);
                perp_f->first   -= chg_a_perp * m_a;
                perp_f->second  += chg_a_perp * m_b;
            }
            /* Apply one sided force */
            else if (m_a != numeric_limits<float>::infinity())
            {
                const point_t chg_a_perp(net_v_perp / t);
//                cout << "accelerating first at: " << chg_a_perp.x << " " << chg_a_perp.y << " " << chg_a_perp.z << endl;
                perp_f->first = chg_a_perp * -m_a;
            }
            else if (m_b != numeric_limits<float>::infinity())
            {
                const point_t chg_a_perp(net_v_perp / t);
//                cout << "accelerating second at: " << chg_a_perp.x << " " << chg_a_perp.y << " " << chg_a_perp.z << endl;
//                cout << "mass: " << m_b << endl;
                perp_f->second = chg_a_perp * m_b;
            }
        }
        /* Slow the motion by the force of friction */
        else
        {
            if (m_a != numeric_limits<float>::infinity())
            {
                perp_f->first   -= norm_v_perp * fric_k;
            }

            if (m_b != numeric_limits<float>::infinity())
            {
                perp_f->second  += norm_v_perp * fric_k;
            }
        }
    }
    /* Static friction */
    else
    {
        const float fric_s = magn_f_proj * mus;
//        cout << "Friction s: " << fric_s << endl;
        const float mag_a_perp = magnitude(perp_f->first);
        const float mag_b_perp = magnitude(perp_f->second);

        /* Reduce the size of the forces by the smallest of friction or the force itself */
        if (mag_a_perp > 0.0)
        {
            perp_f->first   -= (perp_f->first  / mag_a_perp) * std::min(fric_s, mag_a_perp);
        }

        if (mag_b_perp)
        {
            perp_f->second  -= (perp_f->second / mag_b_perp) * std::min(fric_s, mag_b_perp);
        }
    }

    return *this;
}


point_t contact_graph_node::average_volocities(std::pair<point_t, point_t> *const v, const float m_a, const float m_b) const
{
    /* Save the original state */
    const point_t first(v->first);
    const point_t second(v->second);

    /* Average */
    if ((m_a != numeric_limits<float>::infinity()) && (m_b != numeric_limits<float>::infinity()))
    {
        v->first    = (v->first + v->second) * 0.5;
        v->second   = v->first;
    }
    /* Infinite mass objects cant move */
    else
    {
        v->first    = 0.0;
        v->second   = 0.0;
    }

   cout << "Resultant projected velocities: " << endl;
   cout << "First: " << v->first.x << " " << v->first.y << " " << v->first.z << endl;
   cout << "Secon: " << v->second.x << " " << v->second.y << " " << v->second.z << endl;

    /* Return the maximum percent change */
    const point_t first_chg((first - v->first) / first);
    const point_t second_chg((second - v->second) / second);
    return max(fabs(first_chg), fabs(second_chg));
}
}; /* namespace raptor_physics */
