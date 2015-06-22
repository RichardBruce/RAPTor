#pragma once

#include "collider.h"

/* Common headers */
#include "physics_common.h"
#include "logging.h"

/* Physics headers */
#include "inertia_tensor.h"


namespace raptor_physics
{
/* Function to process an instantaneous, frictionless collision */
/* The function is templated for testing */
template<class Object>
void instantaneous_frictionless_collide(Object *const po_a, Object *const po_b, const point_t &poc, const point_t &noc, const float cor)
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

    /* Projected velocity */
    const point_t t_a_abso(po_a->get_velocity(poc));
    const point_t t_b_abso(po_b->get_velocity(poc));
    const point_t t_a_proj(project_vector(t_a_abso, -noc));
    const point_t t_b_proj(project_vector(t_b_abso,  noc));

    /* The closing speed */
    const point_t closing_v(t_b_proj - t_a_proj);
    BOOST_LOG_TRIVIAL(trace) << "Closing velocity: " << closing_v;

    /* Inertia matrices */
    const inertia_tensor_view inertia_a(po_a->get_orientated_tensor());
    const inertia_tensor_view inertia_b(po_b->get_orientated_tensor());

    /* Rotational component of impulse */
    const point_t ra(poc - po_a->get_center_of_mass());
    const point_t rb(poc - po_b->get_center_of_mass());
    const point_t ra_cross_n(cross_product(ra, noc));
    const point_t rb_cross_n(cross_product(rb, noc));

    const point_t a_cross_r(cross_product(ra_cross_n, ra) / inertia_a);
    const point_t b_cross_r(cross_product(rb_cross_n, rb) / inertia_b);
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
    const point_t a_mult_i(ra_cross_n / inertia_a);
    const point_t b_mult_i(rb_cross_n / inertia_b);
    po_a->apply_impulse(noc, a_mult_i,  impul);
    po_b->apply_impulse(noc, b_mult_i, -impul);

    /* Log state */
    BOOST_LOG_TRIVIAL(trace) << "Exit v a: " << po_a->get_velocity();
    BOOST_LOG_TRIVIAL(trace) << "Exit v b: " << po_b->get_velocity();
    BOOST_LOG_TRIVIAL(trace) << "Exit w a: " << po_a->get_angular_velocity();
    BOOST_LOG_TRIVIAL(trace) << "Exit w b: " << po_b->get_angular_velocity();
}

/* Function to process an instantaneous collision */
/* The function is templated for testing */
template<class Object>
void instantaneous_collide(Object *const po_a, Object *const po_b, const point_t &poc, const point_t &noc, const float cor, const float mu)
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
    BOOST_LOG_TRIVIAL(trace) << "Coefficient of friction: " << mu;
    
    /* Projected velocity */
    const point_t t_a_abso(po_a->get_velocity(poc));
    const point_t t_b_abso(po_b->get_velocity(poc));
    const point_t t_a_proj(project_vector(t_a_abso, -noc));
    const point_t t_b_proj(project_vector(t_b_abso,  noc));
    const point_t t_a_tanj(t_a_abso - t_a_proj);
    const point_t t_b_tanj(t_b_abso - t_b_proj);

    /* The closing speeds */
    const point_t proj_v_close(t_b_proj - t_a_proj);
    point_t tanj_v_close(t_b_tanj - t_a_tanj);
    
    const float tanj_v_magn = magnitude(tanj_v_close);
    if (tanj_v_magn > raptor_physics::EPSILON)
    {
        tanj_v_close /= tanj_v_magn;
    }
    BOOST_LOG_TRIVIAL(trace) << "Closing velocity projected: " << proj_v_close << ", tanjent: " << tanj_v_close;

    /* Inertia matrices */
    const inertia_tensor_view inertia_a(po_a->get_orientated_tensor());
    const inertia_tensor_view inertia_b(po_b->get_orientated_tensor());

    /* Rotational component */
    const point_t ra(poc - po_a->get_center_of_mass());
    const point_t rb(poc - po_b->get_center_of_mass());
    const point_t ra_cross_n(cross_product(ra, noc));
    const point_t rb_cross_n(cross_product(rb, noc));
    const point_t ra_cross_tanj(cross_product(ra, tanj_v_close));
    const point_t rb_cross_tanj(cross_product(rb, tanj_v_close));

    const point_t a_cross_r(cross_product(ra_cross_n, ra) / inertia_a);
    const point_t b_cross_r(cross_product(rb_cross_n, rb) / inertia_b);
    const float rot_comp = dot_product((a_cross_r + b_cross_r), noc);

    /* Linear component */
    const float total_m_inv = (1.0f / po_a->get_mass()) + (1.0f / po_b->get_mass());
    const float tra_comp = dot_product(noc, noc * total_m_inv);

    /* Limit friction to bringing point to a halt */
    float friction_max = 0.0f;
    const point_t a_mult_proj(ra_cross_n / inertia_a);
    const point_t b_mult_proj(rb_cross_n / inertia_b);
    const point_t a_mult_tanj(ra_cross_tanj / inertia_a);
    const point_t b_mult_tanj(rb_cross_tanj / inertia_b);
    const float tanj_coeff = magnitude(cross_product(a_mult_tanj, ra) + cross_product(b_mult_tanj, rb) + (tanj_v_close / po_a->get_mass()) +  (tanj_v_close / po_b->get_mass()));
    if (tanj_coeff > raptor_physics::EPSILON)
    {
        friction_max = tanj_v_magn / tanj_coeff;
    }
    BOOST_LOG_TRIVIAL(trace) << "Maximum friction impulse: " << friction_max;

    /* Impulses */
    const float denom = rot_comp + tra_comp;
    const float numer = dot_product(proj_v_close * (1.0f + cor), noc);
    const float reactive_impulse = numer / denom;
    const float friction_impulse = std::min(friction_max, -reactive_impulse * mu);
    BOOST_LOG_TRIVIAL(trace) << "Reactive impulse: " << reactive_impulse << ", Fricion impulse: " << friction_impulse;

    /* Apply the impulses */
    po_a->apply_impulse(noc, a_mult_proj,  reactive_impulse);
    po_b->apply_impulse(noc, b_mult_proj, -reactive_impulse);
    po_a->apply_impulse(tanj_v_close, a_mult_tanj,  friction_impulse);
    po_b->apply_impulse(tanj_v_close, b_mult_tanj, -friction_impulse);

    /* Log state */
    BOOST_LOG_TRIVIAL(trace) << "Exit v a: " << po_a->get_velocity();
    BOOST_LOG_TRIVIAL(trace) << "Exit v b: " << po_b->get_velocity();
    BOOST_LOG_TRIVIAL(trace) << "Exit w a: " << po_a->get_angular_velocity();
    BOOST_LOG_TRIVIAL(trace) << "Exit w b: " << po_b->get_angular_velocity();
}


/* Class to process rigid body collisions */
class rigid_body_collider : public collider
{
    public :
        rigid_body_collider(const float cor, const float mu)
            : _cor(cor), _mu(mu) {  };
        
        virtual const rigid_body_collider& collide(physics_object *const po_a, physics_object *const po_b, const point_t &poc, const point_t &noc, const collision_t type) const
        {
            METHOD_LOG;

            /* Only process these types of collision */
            assert(type == collision_t::COLLISION);

            /* Actual collision processing */
            instantaneous_collide(po_a, po_b, poc, noc, _cor, _mu);

            return *this;
        }
        
    private :
        const float _cor;    /* Coefficient of restitution   */
        const float _mu;     /* Coefficient of friction      */
};
}; /* namespace raptor_physics */
