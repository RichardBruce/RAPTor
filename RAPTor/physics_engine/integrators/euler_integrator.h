#pragma once

/* Common headers */
#include "point_t.h"
#include "quaternion_t.h"

/* Physics headers */
#include "inertia_tensor.h"
#include "force.h"


namespace raptor_physics
{
class euler_integrator
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

            /* Acceleration */
            /* F = MA -> A = F/M */
            const point_t a(f.get_force(i, i.center_of_mass(), v0, 0.0f) / i.mass());

            /* v1ocity */
            /* v0 = U + (A * t) */
            (*v1) = v0 + (a * dt);

            /* Translation */
            return (v0 + (*v1)) * (dt * 0.5f);
        }

        quaternion_t project_rotation(const aggregate_force &t, const inertia_tensor &i, const quaternion_t &o0, const point_t &w0, const float dt) const
        {
            point_t w1;
            return project_rotation(t, i, &w1, o0, w0, dt);
        }

        quaternion_t project_rotation(const aggregate_force &t, const inertia_tensor &i, point_t *const w1, const quaternion_t &o0, const point_t &w0, const float dt) const
        {
            assert(dt >= 0.0f || !"Error: Negative time step");

            /* Acceleration */
            /* F = MA -> A = F/M */
            const inertia_tensor_view i_0(i, o0);
            const point_t a(t.get_torque(i, i.center_of_mass(), w0, 0.0f) / i_0);

            /* Ve1ocity */
            /* v0 = U + (A * t) */
            (*w1) = w0 + (a * dt);

            /* Convert to quaternion rotation */
            const point_t apo_w((w0 + (*w1)) * 0.5f);
            const quaternion_t w_quat(0.5f * quaternion_t(0.0f, apo_w.x, apo_w.y, apo_w.z) * o0);
            return w_quat * dt;
        }

    private :
};
} /* namespace raptor_physics */
