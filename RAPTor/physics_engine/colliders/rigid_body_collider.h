#pragma once

#include "collider.h"

/* Common headers */
#include "physics_common.h"
#include "logging.h"

/* Physics headers */
#include "inertia_tensor.h"


namespace raptor_physics
{
template<class Object>
void log_collision_state(Object *const po_a, Object *const po_b, const std::string &phase, const point_t<> &poc)
{
    BOOST_LOG_TRIVIAL(trace) << phase << " v a: " << po_a->get_velocity();
    BOOST_LOG_TRIVIAL(trace) << phase << " v b: " << po_b->get_velocity();
    BOOST_LOG_TRIVIAL(trace) << phase << " w a: " << po_a->get_angular_velocity();
    BOOST_LOG_TRIVIAL(trace) << phase << " w b: " << po_b->get_angular_velocity();
    BOOST_LOG_TRIVIAL(trace) << phase << " t a: " << po_a->get_velocity(poc);
    BOOST_LOG_TRIVIAL(trace) << phase << " t b: " << po_b->get_velocity(poc);
}

/* Function to process an instantaneous, frictionless collision */
/* The function is templated for testing */
template<class Object>
void instantaneous_frictionless_collide(Object *const po_a, Object *const po_b, const point_t<> &poc, const point_t<> &noc, const float cor)
{
    METHOD_LOG;

    /* Log state */
    BOOST_LOG_TRIVIAL(trace) << "Input v a: " << po_a->get_velocity();
    BOOST_LOG_TRIVIAL(trace) << "Input v b: " << po_b->get_velocity();
    BOOST_LOG_TRIVIAL(trace) << "Input w a: " << po_a->get_angular_velocity();
    BOOST_LOG_TRIVIAL(trace) << "Input w b: " << po_b->get_angular_velocity();
    BOOST_LOG_TRIVIAL(trace) << "Input t a: " << po_a->get_velocity(poc);
    BOOST_LOG_TRIVIAL(trace) << "Input t b: " << po_b->get_velocity(poc);

    BOOST_LOG_TRIVIAL(trace) << "Point of collision: " << poc;
    BOOST_LOG_TRIVIAL(trace) << "Normal of collision: " << noc;
    BOOST_LOG_TRIVIAL(trace) << "Center of mass a: " << po_a->get_center_of_mass();
    BOOST_LOG_TRIVIAL(trace) << "Center of mass b: " << po_b->get_center_of_mass();

    BOOST_LOG_TRIVIAL(trace) << "Coefficient of restitution: " << cor;
    BOOST_LOG_TRIVIAL(trace) << "Closing speed: " << dot_product(po_a->get_velocity(poc) - po_b->get_velocity(poc), noc);

    /* Projected velocity */
    const point_t<> t_a_abso(po_a->get_velocity(poc));
    const point_t<> t_b_abso(po_b->get_velocity(poc));
    const point_t<> t_a_proj(project_vector(t_a_abso, -noc));
    const point_t<> t_b_proj(project_vector(t_b_abso,  noc));

    /* The closing speed */
    const point_t<> closing_v(t_b_proj - t_a_proj);
    BOOST_LOG_TRIVIAL(trace) << "Closing velocity: " << closing_v;

    /* Inertia matrices */
    const inertia_tensor_view inertia_a(po_a->get_orientated_tensor());
    const inertia_tensor_view inertia_b(po_b->get_orientated_tensor());

    /* Rotational component of impulse */
    const point_t<> ra(poc - po_a->get_center_of_mass());
    const point_t<> rb(poc - po_b->get_center_of_mass());
    const point_t<> ra_cross_n(cross_product(ra, noc));
    const point_t<> rb_cross_n(cross_product(rb, noc));

    const point_t<> a_cross_r(cross_product(ra_cross_n / inertia_a, ra));
    const point_t<> b_cross_r(cross_product(rb_cross_n / inertia_b, rb));
    const float rot_comp = dot_product((a_cross_r + b_cross_r), noc);

    /* Linear component of impulse */
    const float total_m_inv = (1.0f / po_a->get_mass()) + (1.0f / po_b->get_mass());
    const float tra_comp = dot_product(noc, noc * total_m_inv);

    /* Impulse */
    const float denom = rot_comp + tra_comp;
    const float numer = dot_product(closing_v * (1.0f + cor), noc);
    const float impul = numer / denom;
    BOOST_LOG_TRIVIAL(trace) << "Impulse: " << impul;

    /* Apply the impulse */
    const point_t<> a_mult_i(ra_cross_n / inertia_a);
    const point_t<> b_mult_i(rb_cross_n / inertia_b);
    po_a->apply_impulse(noc, a_mult_i,  impul);
    po_b->apply_impulse(noc, b_mult_i, -impul);

    /* Log state */
    BOOST_LOG_TRIVIAL(trace) << "Exit v a: " << po_a->get_velocity();
    BOOST_LOG_TRIVIAL(trace) << "Exit v b: " << po_b->get_velocity();
    BOOST_LOG_TRIVIAL(trace) << "Exit w a: " << po_a->get_angular_velocity();
    BOOST_LOG_TRIVIAL(trace) << "Exit w b: " << po_b->get_angular_velocity();
    BOOST_LOG_TRIVIAL(trace) << "Separation speed: " << dot_product(po_a->get_velocity(poc) - po_b->get_velocity(poc), noc);
}

/* Function to process an instantaneous collision */
/* The function is templated for testing */
/* 
    Basing these calculations on the following formula
    Velocity of a point vt = v + cross(w, rp)
    Velocity after an impule = v + (i * n / m) + (mu * i * t / m) + cross(w + (cross(rp, i * n) / it) + (cross(rp, mu * i * t) / it), rp) 
    Relative velocity after an impule =  va - vb + (i * n / ma) + (mu * i * t / ma) - (i * n / mb) - (mu * i * t / mb) + 
                                         cross(wa + (cross(rpa, i * n) / ita), rpa) + (cross(rpa, mu * i * t) / ita), rpa) - cross(wb + (cross(rpb, i * n) / itb) + (cross(rpb, mu * i * t) / itb), rpb) 
    Newtons law of restitution = (va2 - vb2).n = e(va1 - vb1).n 
    Impulse with friction 
    i = (e - 1)(vta - vtb).n / (((1 / ma) - (1 / mb)) + cross(cross(rpa, n) / ita), rpa), rpa).n + cross(cross(rpa, mu * t) / ita), rpa).n - icross(cross(rpb, n) / itb), rpb).n - cross(rpb, mu * t) / itb), rpb).n)
    Working out when the impulses bring the tangent velocity to 0
    i = (vtb - vta).t / ((mu / ma) - (mu / mb) + (cross((cross(rpa, n) / ita) + (cross(rpa, mu * t) / ita), rpa) - cross((cross(rpb, n) / itb) + (cross(rpb, mu * t) / itb), rpb)).t)
*/
template<class Object>
void instantaneous_collide(Object *const po_a, Object *const po_b, const point_t<> &poc, const point_t<> &noc, const float cor, const float mu)
{
    METHOD_LOG;

    /* Log state */
    log_collision_state(po_a, po_b, "Input", poc);

    BOOST_LOG_TRIVIAL(trace) << "Point of collision: " << poc;
    BOOST_LOG_TRIVIAL(trace) << "Normal of collision: " << noc;
    BOOST_LOG_TRIVIAL(trace) << "Center of mass a: " << po_a->get_center_of_mass();
    BOOST_LOG_TRIVIAL(trace) << "Center of mass b: " << po_b->get_center_of_mass();

    BOOST_LOG_TRIVIAL(trace) << "Coefficient of restitution: " << cor;
    BOOST_LOG_TRIVIAL(trace) << "Coefficient of friction: " << mu;

    /* Projected velocity */
    const point_t<> t_a_abso(po_a->get_velocity(poc));
    const point_t<> t_b_abso(po_b->get_velocity(poc));
    const point_t<> t_a_proj(project_vector(t_a_abso, -noc));
    const point_t<> t_b_proj(project_vector(t_b_abso,  noc));
    BOOST_LOG_TRIVIAL(trace) << "projected a: " << t_a_proj << ", projected b: " << t_b_proj;
    const point_t<> t_a_tanj(t_a_abso - t_a_proj);
    const point_t<> t_b_tanj(t_b_abso - t_b_proj);

    /* The closing speeds */
    const point_t<> proj_v_close(t_b_proj - t_a_proj);
    const point_t<> tanj_v_close(t_b_tanj - t_a_tanj);
    
    /* The tanjent vector */
    point_t<> t_noc(tanj_v_close);
    const float tanj_v_magn = magnitude(tanj_v_close);
    if (tanj_v_magn > raptor_physics::EPSILON)
    {
        t_noc /= tanj_v_magn;
    }
    BOOST_LOG_TRIVIAL(trace) << "Closing velocity projected: " << proj_v_close;
    BOOST_LOG_TRIVIAL(trace) << "Closing speed: " << dot_product(po_a->get_velocity(poc) - po_b->get_velocity(poc), noc);
    BOOST_LOG_TRIVIAL(trace) << "Tanjent velocity: " << tanj_v_close << ", tanjent: " << t_noc;

    /* Oriented inertia tensors */
    const inertia_tensor_view inertia_a(po_a->get_orientated_tensor());
    const inertia_tensor_view inertia_b(po_b->get_orientated_tensor());

    /* Calculate the maximum impulse with friction */
    /* Linear contribution */
    const float mu_ma = mu / po_a->get_mass();
    const float mu_mb = mu / po_b->get_mass();

    /* Angular contribution */
    const point_t<> ra(poc - po_a->get_center_of_mass());
    const point_t<> rb(poc - po_b->get_center_of_mass());
    const point_t<> ra_cross_n(cross_product(ra, noc));
    const point_t<> rb_cross_n(cross_product(rb, noc));
    const point_t<> ra_cross_t(cross_product(ra, t_noc));
    const point_t<> rb_cross_t(cross_product(rb, t_noc));
    const point_t<> a_mult_n(ra_cross_n / inertia_a);
    const point_t<> b_mult_n(rb_cross_n / inertia_b);
    const point_t<> a_mult_t(ra_cross_t / inertia_a);
    const point_t<> b_mult_t(rb_cross_t / inertia_b);
    const point_t<> a_mult_t_mu(a_mult_t * mu);
    const point_t<> b_mult_t_mu(b_mult_t * mu);

    /* Max impulse with friction */
    BOOST_LOG_TRIVIAL(trace) << "Expected linear change: " << (mu_ma + mu_mb);
    BOOST_LOG_TRIVIAL(trace) << "Expected rotational change: " << dot_product(cross_product(a_mult_n + a_mult_t_mu, ra) + cross_product(b_mult_n + b_mult_t_mu, rb), t_noc);
    BOOST_LOG_TRIVIAL(trace) << "Expected rotational change normal: " << dot_product(cross_product(a_mult_n, ra) + cross_product(b_mult_n, rb), t_noc);
    BOOST_LOG_TRIVIAL(trace) << "Expected rotational change tangent: " << dot_product(cross_product(a_mult_t_mu, ra) + cross_product(b_mult_t_mu, rb), t_noc);
    const point_t<> angular_change(cross_product(a_mult_n + a_mult_t_mu, ra) + cross_product(b_mult_n + b_mult_t_mu, rb));
    const float max_fric_i = tanj_v_magn / (mu_ma + mu_mb + dot_product(angular_change, t_noc));
    BOOST_LOG_TRIVIAL(trace) << "Maximum with friction impulse: " << max_fric_i;
    assert((dot_product(cross_product(a_mult_t_mu, ra) + cross_product(b_mult_t_mu, rb), t_noc) >= 0.0f) || !"Error: Rotation caused by tangent impulse became negative");

    /* Calculate the impule as given by the law of restitution */
    /* Linear contribution */
    const float ma_inv = 1.0f / po_a->get_mass();
    const float mb_inv = 1.0f / po_b->get_mass();
    const float n_lin = ma_inv + mb_inv;

    /* Angular contribution, see above */

    /* Max reactive impulse */
    BOOST_LOG_TRIVIAL(trace) << "Expected linear change: " << (ma_inv + mb_inv);
    BOOST_LOG_TRIVIAL(trace) << "Expected rotational change: " << dot_product(cross_product(a_mult_n + a_mult_t_mu, ra) + cross_product(b_mult_n + b_mult_t_mu, rb), noc);
    BOOST_LOG_TRIVIAL(trace) << "Expected rotational change normal: " << dot_product(cross_product(a_mult_n, ra) + cross_product(b_mult_n, rb), noc);
    BOOST_LOG_TRIVIAL(trace) << "Expected rotational change tangent: " << dot_product(cross_product(a_mult_t_mu, ra) + cross_product(b_mult_t_mu, rb), noc);
    const float reac_num = dot_product(proj_v_close * (1.0f + cor), noc);
    const float reac_den = n_lin + dot_product(angular_change, noc);
    const float max_reac_i = reac_num / reac_den;
    BOOST_LOG_TRIVIAL(trace) << "Maximum reactive impulse: " << max_reac_i << ", numerator: " << reac_num << ", denominator: " << reac_den;

    /* Apply friction impulses. A negative impulse bringing sliding to a halt indicates the normal impulse accelerating sliding more than the friction impulse can slow it */
    const float min_i   = (max_fric_i < 0.0f) ? max_reac_i : std::min(max_reac_i, max_fric_i);
    const float fric_i  = mu * min_i;
    BOOST_LOG_TRIVIAL(trace) << "Reactive impulse with friction: " << min_i << ", Friction impulse: " << fric_i;
    assert(fric_i > -raptor_physics::EPSILON);

    /* Apply the friction impulse */
    po_a->apply_impulse(t_noc, a_mult_t,  fric_i, false);
    po_b->apply_impulse(t_noc, b_mult_t, -fric_i, false);
    log_collision_state(po_a, po_b, "Post friction", poc);

    const bool apply_full_reac = (min_i == max_reac_i);
    po_a->apply_impulse(noc, a_mult_n,  min_i, false);
    po_b->apply_impulse(noc, b_mult_n, -min_i, false);
    log_collision_state(po_a, po_b, "Post reaction with friction", poc);

    /* Assert the objects didnt change direction or speed up */
    assert((dot_product(po_b->get_velocity(poc) - po_a->get_velocity(poc), t_noc) > -raptor_physics::EPSILON) || !"Error: point changed direction in friction");
    assert((dot_product(po_b->get_velocity(poc) - po_a->get_velocity(poc), t_noc) <= tanj_v_magn) || (max_fric_i < 0.0f) || !"Error: point gained velocity in friction");

    /* If we couldnt apply the full reactive impulse then calculate the remaining impulse with no friction */
    if (!apply_full_reac)
    {
        /* Apply the reactive impulse */
        const float reac_i  = (reac_num - (reac_den * min_i)) / (n_lin + dot_product(cross_product(a_mult_n, ra) + cross_product(b_mult_n, rb), noc));
        BOOST_LOG_TRIVIAL(trace) << "Reactive impulse: " << reac_i;
        assert(reac_i > -raptor_physics::EPSILON);

        po_a->apply_impulse(noc, a_mult_n,  reac_i, false);
        po_b->apply_impulse(noc, b_mult_n, -reac_i, false);
        log_collision_state(po_a, po_b, "Post reaction", poc);
    }

    /* Assert the objects are separating, but not too quickly */
    assert((std::fabs(dot_product(po_a->get_velocity(poc) - po_b->get_velocity(poc), noc)) <= std::fabs(dot_product(t_a_abso - t_b_abso, noc))) || !"Error: point gained velocity in collision");
    assert((dot_product(po_a->get_velocity(poc) - po_b->get_velocity(poc), noc) * dot_product(t_a_abso - t_b_abso, noc) <= raptor_physics::EPSILON) || !"Error: point didnt change direction in collision");
    BOOST_LOG_TRIVIAL(trace) << "Separation speed: " << dot_product(po_a->get_velocity(poc) - po_b->get_velocity(poc), noc);
    BOOST_LOG_TRIVIAL(trace) << "Tangent speed: " << dot_product(po_a->get_velocity(poc) - po_b->get_velocity(poc), t_noc);
    BOOST_LOG_TRIVIAL(trace) << "a_mult_n: " << a_mult_n;
    BOOST_LOG_TRIVIAL(trace) << "b_mult_n: " << b_mult_n;
    BOOST_LOG_TRIVIAL(trace) << "a_mult_t: " << a_mult_t;
    BOOST_LOG_TRIVIAL(trace) << "b_mult_t: " << b_mult_t;

    /* Update the objects bounds */
    po_a->update_bounds();
    po_b->update_bounds();
    log_collision_state(po_a, po_b, "Exit", poc);
}

/* Class to process rigid body collisions */
class rigid_body_collider : public collider
{
    public :
        rigid_body_collider(const float cor, const float mus, const float mud) : collider(cor, mus, mud) {  };
        
        virtual const rigid_body_collider& collide(physics_object *const po_a, physics_object *const po_b, const point_t<> &poc, const point_t<> &noc, const collision_t type) const
        {
            METHOD_LOG;

            /* Only process these types of collision */
            assert(type == collision_t::COLLISION);

            /* Actual collision processing */
            instantaneous_collide(po_a, po_b, poc, noc, _cor, dynamic_friction());

            return *this;
        }
        
    private :
};
}; /* namespace raptor_physics */
