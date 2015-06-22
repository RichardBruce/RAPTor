#pragma once

/* Common headers */
#include "point_t.h"
#include "quaternion_t.h"

/* Physics headers */
#include "inertia_tensor.h"
#include "force.h"


namespace raptor_physics
{
class verlet_integrator
{
    public :
// xi+1 = xi + ((xi - xi-1) * (dti / dti-1)) + (a * dti * dti)
    private :
};
} /* namespace raptor_physics */
