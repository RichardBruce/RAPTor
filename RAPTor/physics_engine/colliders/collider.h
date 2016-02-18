#pragma once

/* Phycsic headers */
#include "point_t.h"
#include "physics_object.h"


namespace raptor_physics
{
/* Abstract class to collide two objects (i.e. process the collision response). */
class collider
{
    public :
        /* Ctor */
        collider(const float cor, const float mu) : _cor(cor), _mu(mu) {  };
        
        /* Virtual DTOR just in case a base class needs a DTOR */
        virtual ~collider() {  };
        
        virtual const collider& collide(physics_object *const po_a, physics_object *const po_b, const point_t &poc, const point_t &noc, const collision_t type) const = 0;

        float dynamic_friction() const { return _mu; }

    protected :
        const float _cor;    /* Coefficient of restitution   */
        const float _mu;     /* Coefficient of friction      */
};
}; /* namespace raptor_physics */
