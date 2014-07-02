#ifndef __PHYSICS_OBJECT_H__
#define __PHYSICS_OBJECT_H__

/* Standard headers */
#include <algorithm>
#include <vector>

/* Boost headers */
#include "boost/noncopyable.hpp"

/* Common headers */
#include "point_t.h"
#include "quaternion_t.h"

/* Physics headers */
#include "vertex_group.h"
#include "force.h"


namespace raptor_physics
{
/* Forward declarations */
class simplex;
class physics_engine;

/* Enum to show the different ways objects can connect */
/* Possible is used for rotating object were only a conservative result can be given */
enum collision_t { NO_COLLISION = 0, SLIDING_COLLISION = 1, COLLISION = 2, 
    POSSIBLE_SLIDING_COLLISION = 5, POSSIBLE_COLLISION = 6 };

/* Check for a POSSIBLY_ prefix */
inline bool is_uncertain(const collision_t c)
{
    return (c & 0x4);
}

/* Convert to the certain form */
inline collision_t to_certain(const collision_t c)
{
    return static_cast<collision_t>(c & 0x3);
}

/* Class to wrap a vertex group with the physics engine specific functionality */
class physics_object : private boost::noncopyable
{
    public :
        typedef vertex_group inner_vg;

        /* Ownership isnt taken of vg */
        physics_object(vertex_group *const vg, const point_t &com, const fp_t density, const unsigned int t = 0)
            : physics_object(vg, quaternion_t(1.0, 0.0, 0.0, 0.0), com, density, t) {  };
              
        physics_object(vertex_group *const vg, const quaternion_t &o, const point_t &com, const fp_t density, const unsigned int t = 0)
            : physics_object(vg, o, com, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), density, t) {  };

        physics_object(vertex_group *const vg, const quaternion_t &o, const point_t &com, const point_t &v, const point_t &w, const fp_t density, const unsigned int t = 0)
            : _vg(vg),
              _forces(new std::vector<force *>()),
              _agg_force(*_forces),
              _i(_vg->build_inertia_tensor(density)),
              _w(w),
              _v(v),
              _o(o),
              _cur_t(0),
              _type(t)
              {
                    _i->move_center_of_mass(com - _i->center_of_mass());
              };

        ~physics_object()
        {
            /* Clean up outstanding forces */
            for (auto f : (*_forces))
            {
                delete f;
            }
            delete _forces;

            delete _i;
        }
        
        /* Register a force the will be applied for multiple time steps */
        physics_object& register_force(force *const f)
        {
            _forces->push_back(f);
            return *this;
        }
        
        /* Start a new time step, reset time and apply the forces */
        physics_object& begin_time_step()
        {
            _cur_t  = 0.0;
            return *this;
        }

        /* Physics calculations for applied forces */
        /* Note -- Technically the force should stop after f.t */
        /*         This is appoximated by weighting the force in proportion to the period it is applied */
        /* TODO - Is it worth checking that the force is applied on the object not in free space? */
        physics_object& apply_force(const point_t &at, const point_t &f, const fp_t t)
        {
            /* No point pushing an infinite mass object */
            if (get_mass() == numeric_limits<fp_t>::infinity())
            {
                return *this;
            }

            register_force(new const_force(at, f, t));

            return *this;
        }

        physics_object& apply_impulse(const point_t &impulse, const point_t &poc)
        {
            const point_t l(cross_product(poc - get_center_of_mass(), impulse));
            BOOST_LOG_TRIVIAL(trace) << "l: " << l;

            _v += (impulse / _i->mass());
            _w += (l / get_orientated_tensor());
            return *this;
        }

        physics_object& apply_impulse(const point_t &n, const point_t &angular_weight, const fp_t impulse)
        {
            _v += (n * (impulse / _i->mass()));
            _w += (angular_weight * impulse);
            return *this;
        }
        
        /* Collision detection */
        bool has_collided(physics_object *const po, simplex **const manifold_a, simplex **const manifold_b, point_t *const d, const fp_t t0, const fp_t t1);
        
        /* Check for collisions and separate objects as needed */
        collision_t resolve_collisions(physics_object *const po, simplex **const manifold_a, simplex **const manifold_b, fp_t *const t)
        {
            /* If no rotation use exact */
            if ((fabs(magnitude(    _w)) < raptor_physics::EPSILON) && (fabs(magnitude(    get_torque())) < raptor_physics::EPSILON) &&
                (fabs(magnitude(po->_w)) < raptor_physics::EPSILON) && (fabs(magnitude(po->get_torque())) < raptor_physics::EPSILON))
            {
                return exactly_resolve_collisions(po, manifold_a, manifold_b, t);
            }
            /* Else must be conservative */
            else
            {
                return conservatively_resolve_collisions(po, manifold_a, manifold_b, t);
            }
        }
        
        physics_object& commit_movement(const fp_t t);

        const physics_object& triangles(primitive_list *p) const
        {
            _vg->triangles(p, _o, _i->center_of_mass());
            return *this;
        }

        /* Access functions */
        /* Physics getters */
        unsigned int                get_physical_type()     const { return _type;                                                       }
        const point_t               get_force()             const { return _agg_force.get_force(*_i, get_center_of_mass(), _v, 0.0);    }
        const point_t               get_torque()            const { return _agg_force.get_torque(*_i, get_center_of_mass(), _w, 0.0);   }
        const fp_t                  get_speed()             const { return magnitude(_v);                                               }
        const fp_t                  get_mass()              const { return _i->mass();                                                  }
        const point_t&              get_center_of_mass()    const { return _i->center_of_mass();                                        }
        const quaternion_t&         get_orientation()       const { return _o;                                                          }
        const inertia_tensor&       get_inertia_tenor()     const { return *_i;                                                         }
        const inertia_tensor_view   get_orientated_tensor() const { return inertia_tensor_view(*_i, _o);                                }

        const point_t&  get_velocity()                  const { return _v; }
        const point_t&  get_angular_velocity()          const { return _w; }
        point_t         get_velocity(const point_t &p)  const
        {
            /* Get rotational velocity at p */
            const point_t r(p - _i->center_of_mass());
            const point_t rot_vel(cross_product(_w, r));

            return _v + rot_vel;
        }

        const point_t get_momentum() const
        {
            /* Infinite mass objects shouldnt be moving */
            return (_i->mass() == numeric_limits<fp_t>::infinity()) ? 0.0 : ( _i->mass() * _v);
        }

        const point_t get_angular_momentum() const
        {
            /* Infinite mass objects shouldnt be moving */
            return (_i->mass() == numeric_limits<fp_t>::infinity()) ? point_t(0.0, 0.0, 0.0) : (get_orientated_tensor() * _w);
        }

        /* Vertex getters */
        const std::shared_ptr<vertex_group>& get_vertex_group() const { return _vg; }
        point_t get_orientated_vertex(const int i) const
        {
            return _o.rotate(_vg->get_vertex(i));
        }

        point_t get_global_vertex(const int i) const
        {
            return get_orientated_vertex(i) + get_center_of_mass();
        }

        const physics_object& vertex_to_global(point_t *const p) const
        {
            _o.rotate(p);
            (*p) += get_center_of_mass();
            return *this;
        }

        /* Force getters */
        int                 number_of_registered_forces()       const { return _forces->size();  }
        const force *const  get_registered_force(const int i)   const
        {
            if (i < static_cast<int>(_forces->size()))
            {
                return (*_forces)[i];
            }
            else
            {
                return nullptr;
            }
        }


        /* Setters, only used in unit tests */
        physics_object& set_velocity(const point_t &v) 
        {
            _v = v;
            return *this;
        }

        physics_object& set_angular_velocity(const point_t &w)
        {
            _w = w;
            return *this;
        }
        
    private :
        /* Find the projection of the largest rotational movement in the direction n */
        fp_t project_maximum_rotation_onto(const point_t &n, const point_t &p, const fp_t t) const;

        /* Calculate the longest possible movement, assuming worst case rotation */
        /* of the vertices in the direction n and of v's vertices in the direction of -n */
        fp_t project_maximum_movement_onto(const physics_object &v, const point_t &p_a, const point_t &p_b, const point_t &n, const fp_t t) const;

        /* Check for collisions when the exact object movement can be calculated */
        collision_t exactly_resolve_collisions(physics_object *const vg, simplex **const manifold_a, simplex **const manifold_b, fp_t *const t);

        /* Check for collisions and conservatively separate objects as needed */
        /* Objects should be moved and retested before accepting a collision */
        collision_t conservatively_resolve_collisions(physics_object *const vg, simplex **const manifold_a, simplex **const manifold_b, fp_t *const t);

        /* Specialise algorithm for calculating toc when two objects are very close */
        collision_t close_contact_collision_detection(physics_object *const vg, simplex **const manifold_a, simplex **const manifold_b, fp_t *const t, const point_t &noc) const;
        

        std::shared_ptr<vertex_group>   _vg;
        std::vector<force *>   *        _forces;
        aggregate_force                 _agg_force;
        inertia_tensor      *const      _i;     /* The moment of inertia                                        */
        point_t                         _w;     /* Angular velocity                                             */
        point_t                         _v;     /* Velocity                                                     */
        quaternion_t                    _o;     /* Orientation                                                  */
        fp_t                            _cur_t; /* The committed time                                           */
        const unsigned int              _type;  /* The type of the material to associate it with a collider     */
};
}; /* namespace raptor_physics */

#endif /* #ifndef __PHYSICS_OBJECT_H__ */
