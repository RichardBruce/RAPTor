#pragma once

/* Common headers */
#include "point_t.h"
#include "quaternion_t.h"

/* Physics headers */
#include "inertia_tensor.h"
#include "force.h"


namespace raptor_physics
{
class rk4_integrator
{
    public :
        point_t project_translation(const aggregate_force &f, const inertia_tensor &i, const point_t &v0, const float dt) const
        {
            point_t v1;
            return project_translation(f, i, &v1, v0, dt);
        }


        point_t project_translation(const aggregate_force &f, const inertia_tensor &i, point_t *const v1, const point_t &v0, const float dt) const
        {
            assert(dt >= 0.0f || !"Error: Negative time step");

            const float half_dt  = dt * 0.5f;
            const float sixth_dt = dt * (1.0f / 6.0f);

            /* A1 = G(T,S0), B1 = S0 + (DT/2)*A1 */
            const point_t f_1(f.get_force(i, i.center_of_mass(), v0, 0.0f));
            const point_t v_1(v0 + (half_dt * f_1 / i.mass()));

            /* A2 = G(T+DT/2,B1), B2 = S0 + (DT/2)*A2 */
            const point_t x_2(i.center_of_mass() + half_dt * v0);
            const point_t f_2(f.get_force(i, x_2, v_1, half_dt));
            const point_t v_2(v0 + (half_dt * f_2 / i.mass()));

            /* A3 = G(T+DT/2,B2), B3 = S0 + DT*A3 */
            const point_t x_3(i.center_of_mass() + half_dt * v_1);
            const point_t f_3(f.get_force(i, x_3, v_2, half_dt));
            const point_t v_3(v0 + (dt * f_3 / i.mass()));

            /* A4 = G(T+DT,B3), S1 = S0 + (DT/6)*(A1+2*(A2+A3)+A4) */
            const point_t x_4(i.center_of_mass() + dt * v_2);
            const point_t f_4(f.get_force(i, x_4, v_3, dt));
            *v1 = v0 + (sixth_dt * (f_1 + ((f_2 + f_3) * 2.0f) + f_4)) / i.mass();

            return sixth_dt * (v0 + ((v_1 + v_2) * 2.0f) + v_3);
        }


        quaternion_t project_rotation(const aggregate_force &t, const inertia_tensor &i, const quaternion_t &o0, const point_t &w0, const float dt) const
        {
            point_t w1;
            return project_rotation(t, i, &w1, o0, w0, dt);
        }


        quaternion_t project_rotation(const aggregate_force &t, const inertia_tensor &i, point_t *const w1, const quaternion_t &o0, const point_t &w0, const float dt) const
        {
            assert(dt >= 0.0 || !"Error: Negative time step");

            const float half_dt  = dt * 0.5f;
            const float sixth_dt = dt * (1.0f / 6.0f);

            // A1 = G(T,S0), B1 = S0 + (DT/2)*A1
            const quaternion_t q_1(0.5f * quaternion_t(0.0f, w0.x, w0.y, w0.z) * o0);
            const point_t t_1(t.get_torque(i, i.center_of_mass(), w0, 0.0f));

            const quaternion_t o_1(o0 + half_dt * q_1);
            const inertia_tensor_view i_1(i, o_1);
            const point_t l_1(half_dt * t_1);
            const point_t w_1(w0 + l_1 / i_1);

            // A2 = G(T+DT/2,B1), B2 = S0 + (DT/2)*A2
            const quaternion_t q_2(0.5f * quaternion_t(0.0f, w_1.x, w_1.y, w_1.z) * o_1);
            const point_t t_2(t.get_torque(i, i.center_of_mass(), w_1, half_dt));

            const quaternion_t o_2(o0 + half_dt * q_2);
            const inertia_tensor_view i_2(i, o_2);
            const point_t l_2(half_dt * t_2);
            const point_t w_2(w0 + l_2 / i_2);

            // A3 = G(T+DT/2,B2), B3 = S0 + DT*A3
            const quaternion_t q_3(0.5f * quaternion_t(0.0f, w_2.x, w_2.y, w_2.z) * o_2);
            const point_t t_3(t.get_torque(i, i.center_of_mass(), w_2, half_dt));

            const quaternion_t o_3(o0 + dt * q_3);
            const inertia_tensor_view i_3(i, o_3);
            const point_t l_3(dt * t_3);
            const point_t w_3(w0 + l_3 / i_3);

            // A4 = G(T+DT,B3), S1 = S0 + (DT/6)*(A1+2*(A2+A3)+A4)
            const quaternion_t q_4(0.5f * quaternion_t(0.0f, w_3.x, w_3.y, w_3.z) * o_3);
            const point_t t_4(t.get_torque(i, i.center_of_mass(), w_3, dt));

            const quaternion_t o_4(sixth_dt * (q_1 + ((q_2 + q_3) * 2.0f) + q_4));
            const inertia_tensor_view i_4(i, o0 + o_4);
            const point_t l_4(sixth_dt * (t_1 + ((t_2 + t_3) * 2.0f) + t_4));

            *w1 = w0 + (l_4 / i_4);
            return o_4;
        }

    private :
};
} /* namespace raptor_physics */
