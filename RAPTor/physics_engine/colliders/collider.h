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
        collider(const float cor, const float mus, const float mud) : _cor(cor), _mus(mus), _mud(mud) {  };
        
        /* Virtual DTOR just in case a base class needs a DTOR */
        virtual ~collider() {  };
        
        virtual const collider& collide(physics_object *const po_a, physics_object *const po_b, const point_t &poc, const point_t &noc, const collision_t type) const = 0;

        float static_friction()     const { return _mus; }
        float dynamic_friction()    const { return _mud; }

    protected :
        const float _cor;    /* Coefficient of restitution      */
        const float _mus;    /* Coefficient of static friction  */
        const float _mud;    /* Coefficient of dynamic friction */
};
}; /* namespace raptor_physics */
