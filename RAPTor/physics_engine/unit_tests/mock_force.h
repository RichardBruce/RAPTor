#pragma once

/* Common headers */
#include "point_t.h"

/* Physics headers */
#include "inertia_tensor.h"
#include "force.h"

class mock_force : public raptor_physics::force
{
    public :
        mock_force(const point_t &f, const point_t &t, const float dt) : force(point_t(0.0f, 0.0f, 0.0f), dt), _f(f), _t(t) {  };

        /* Force implementation */
        point_t get_force(const raptor_physics::inertia_tensor &i, const point_t &x, const point_t &v, const float dt) const 
        {
            return _f;
        };

        point_t get_torque(const raptor_physics::inertia_tensor &i, const point_t &x, const point_t &w, const float dt) const
        {
            return _t;
        }

        /* Setters for test */
        mock_force& set_force(const point_t &f)
        {
            _f = f;
            return *this;
        }

        mock_force& set_torque(const point_t &t)
        {
            _t = t;
            return *this;
        }

    private :
        point_t _f;
        point_t _t;
};
