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
#include "object_bound.h"
#include "force.h"
#include "integrators.h"


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
        physics_object(vertex_group *const vg, const point_t &com, const float density, const unsigned int t = 0)
            : physics_object(vg, quaternion_t(1.0, 0.0, 0.0, 0.0), com, density, t) {  };
              
        physics_object(vertex_group *const vg, const quaternion_t &o, const point_t &com, const float density, const unsigned int t = 0)
            : physics_object(vg, o, com, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), density, t) {  };

        physics_object(vertex_group *const vg, const quaternion_t &o, const point_t &com, const point_t &v, const point_t &w, const float density, const unsigned int t = 0)
            : _vg(vg),
              _forces(new std::vector<force *>()),
              _agg_force(*_forces),
              _i(_vg->build_inertia_tensor(density)),
              _w(w),
              _v(v),
              _o(o),
              _cur_t(0.0),
              _t_step(0.0),
              _type(t)
              {
                    _i->move_center_of_mass(com - _i->center_of_mass());

                    /* Build bounds */
                    point_t hi;
                    point_t lo;
                    get_bounds(&hi, &lo);

                    _lower_bound[0] = new object_bound(this, lo.x, true);
                    _upper_bound[0] = new object_bound(this, hi.x, false);
                    _lower_bound[1] = new object_bound(this, lo.y, true);
                    _upper_bound[1] = new object_bound(this, hi.y, false);
                    _lower_bound[2] = new object_bound(this, lo.z, true);
                    _upper_bound[2] = new object_bound(this, hi.z, false);
              };

        ~physics_object()
        {
            /* Clean up outstanding forces */
            for (auto f : (*_forces))
            {
                delete f;
            }
            delete _forces;

            /* Clean up inertia tensor */
            delete _i;

            /* Clean up bounds */
            delete _upper_bound[0];
            delete _upper_bound[1];
            delete _upper_bound[2];

            delete _lower_bound[0];
            delete _lower_bound[1];
            delete _lower_bound[2];
        }
        
        /* Register a force the will be applied for multiple time steps */
        physics_object& register_force(force *const f)
        {
            _forces->push_back(f);
            return *this;
        }
        
        /* Start a new time step, reset time and apply the forces */
        physics_object& begin_time_step(const float t_step)
        {
            /* Set the step times */
            _cur_t  = 0.0;
            _t_step = t_step;

            /* Update the bounds for the full time step */
            update_bounds(_t_step);

            return *this;
        }

        /* Physics calculations for applied forces */
        /* Note -- Technically the force should stop after f.t */
        /*         This is appoximated by weighting the force in proportion to the period it is applied */
        /* TODO - Is it worth checking that the force is applied on the object not in free space? */
        physics_object& apply_force(const point_t &at, const point_t &f, const float t)
        {
            /* No point pushing an infinite mass object */
            if (get_mass() == numeric_limits<float>::infinity())
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

        physics_object& apply_impulse(const point_t &n, const point_t &angular_weight, const float impulse)
        {
            _v += (n * (impulse / _i->mass()));
            _w += (angular_weight * impulse);
            return *this;
        }
        
        /* Collision detection */
        bool has_collided(physics_object *const po, simplex **const manifold_a, simplex **const manifold_b, point_t *const d, const float t0, const float t1);
        
        /* Check for collisions and separate objects as needed */
        collision_t resolve_collisions(physics_object *const po, simplex **const manifold_a, simplex **const manifold_b, float *const t)
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
        
        physics_object& commit_movement(const float t)
        {
            /* Update time */
            if ((t < _cur_t) | (_i->mass() == std::numeric_limits<float>::infinity()))
            {
                return *this;
            }
            const float dt = t - _cur_t;
            _cur_t = t;

            /* Rotate */
            point_t vel;
            rk4_integrator integ;
            _o += integ.project_rotation(_agg_force, *_i, &vel, _o, _w, dt);
            _w = vel;
            normalise(&_o);

            /* Translate */
            _i->move_center_of_mass(integ.project_translation(_agg_force, *_i, &vel, _v, dt));
            _v = vel;

            /* Update bounds for the remainder of the time step */
            update_bounds(_t_step - _cur_t);

            /* Derement forces */
            for (auto& f : (*_forces))
            {
                if (f->commit(dt))
                {
                    delete f;
                    f = nullptr;
                }
            }
            
            /* Remove spent forces */
            _forces->erase(std::remove(_forces->begin(), _forces->end(), nullptr), _forces->end());
            
            return *this;
        }

        const physics_object& triangles(raptor_raytracer::primitive_list *p) const
        {
            _vg->triangles(p, _o, _i->center_of_mass());
            return *this;
        }

        /* Access functions */
        /* Physics getters */
        unsigned int                get_physical_type()     const { return _type;                                                       }
        const point_t               get_force()             const { return _agg_force.get_force(*_i, get_center_of_mass(), _v, 0.0);    }
        const point_t               get_torque()            const { return _agg_force.get_torque(*_i, get_center_of_mass(), _w, 0.0);   }
        const float                 get_speed()             const { return magnitude(_v);                                               }
        const float                 get_mass()              const { return _i->mass();                                                  }
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
            return (_i->mass() == numeric_limits<float>::infinity()) ? 0.0 : ( _i->mass() * _v);
        }

        const point_t get_angular_momentum() const
        {
            /* Infinite mass objects shouldnt be moving */
            return (_i->mass() == numeric_limits<float>::infinity()) ? point_t(0.0, 0.0, 0.0) : (get_orientated_tensor() * _w);
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

        /* Bounds access */
        object_bound *upper_bound(const axis_t axis) const
        {
            return _upper_bound[axis];
        }

        object_bound *lower_bound(const axis_t axis) const
        {
            return _lower_bound[axis];
        }

    private :
        /* Bounding box */
        const physics_object& get_bounds(point_t *const hi, point_t *const lo) const
        {
            /* Get the bounds of the vertex group */
            point_t local_hi;
            point_t local_lo;
            _vg->get_bounds(&local_hi, &local_lo);

            /* Move bounds into global co-ordinates, an OOB */
            const point_t p0(_o.rotate(local_lo) + _i->center_of_mass());
            const point_t p1(_o.rotate(point_t(local_lo.x, local_lo.y, local_hi.x)) + _i->center_of_mass());
            const point_t p2(_o.rotate(point_t(local_lo.x, local_hi.y, local_lo.x)) + _i->center_of_mass());
            const point_t p3(_o.rotate(point_t(local_lo.x, local_hi.y, local_hi.x)) + _i->center_of_mass());
            const point_t p4(_o.rotate(point_t(local_hi.x, local_lo.y, local_lo.x)) + _i->center_of_mass());
            const point_t p5(_o.rotate(point_t(local_hi.x, local_lo.y, local_hi.x)) + _i->center_of_mass());
            const point_t p6(_o.rotate(point_t(local_hi.x, local_hi.y, local_lo.x)) + _i->center_of_mass());
            const point_t p7(_o.rotate(local_hi) + _i->center_of_mass());

            /* Find the axis aligned bounds of the OOB */
            (*hi) = max(max(max(p0, p1), max(p2, p3)), max(max(p4, p5), max(p6, p7))) + WELD_DISTANCE;
            (*lo) = min(min(min(p0, p1), min(p2, p3)), min(min(p4, p5), min(p6, p7))) - WELD_DISTANCE;

            return *this;
        }

        /* Update of bounding box */
        physics_object& update_bounds(const float dt)
        {
            /* Get the bounds of the stationary object */
            point_t hi;
            point_t lo;
            get_bounds(&hi, &lo);

            /* Get the translation of the object */
            point_t vel;
            rk4_integrator integ;
            const point_t trans(integ.project_translation(_agg_force, *_i, &vel, _v, dt));

            /* Set bounds to current position plus any translation */
            _lower_bound[0]->bound(lo.x + std::min(trans.x, 0.0f));
            _upper_bound[0]->bound(hi.x + std::max(trans.x, 0.0f));
            _lower_bound[1]->bound(lo.y + std::min(trans.y, 0.0f));
            _upper_bound[1]->bound(hi.y + std::max(trans.y, 0.0f));
            _lower_bound[2]->bound(lo.z + std::min(trans.z, 0.0f));
            _upper_bound[2]->bound(hi.z + std::max(trans.z, 0.0f));

            return *this;
        }
        
        /* Find the projection of the largest rotational movement in the direction n */
        float project_maximum_rotation_onto(const point_t &n, const point_t &p, const float t) const;

        /* Calculate the longest possible movement, assuming worst case rotation */
        /* of the vertices in the direction n and of v's vertices in the direction of -n */
        float project_maximum_movement_onto(const physics_object &v, const point_t &p_a, const point_t &p_b, const point_t &n, const float t) const;

        /* Check for collisions when the exact object movement can be calculated */
        collision_t exactly_resolve_collisions(physics_object *const vg, simplex **const manifold_a, simplex **const manifold_b, float *const t);

        /* Check for collisions and conservatively separate objects as needed */
        /* Objects should be moved and retested before accepting a collision */
        collision_t conservatively_resolve_collisions(physics_object *const vg, simplex **const manifold_a, simplex **const manifold_b, float *const t);

        /* Specialise algorithm for calculating toc when two objects are very close */
        collision_t close_contact_collision_detection(physics_object *const vg, simplex **const manifold_a, simplex **const manifold_b, float *const t, const point_t &noc) const;
        

        std::shared_ptr<vertex_group>   _vg;
        std::vector<force *>   *        _forces;
        aggregate_force                 _agg_force;
        inertia_tensor      *const      _i;                 /* The moment of inertia                                        */
        object_bound *                  _upper_bound[3];    /* The upper bound of the object in this time step              */
        object_bound *                  _lower_bound[3];    /* The lower bound of the object in this time step              */
        point_t                         _w;                 /* Angular velocity                                             */
        point_t                         _v;                 /* Velocity                                                     */
        quaternion_t                    _o;                 /* Orientation                                                  */
        float                           _cur_t;             /* The committed time                                           */
        float                           _t_step;            /* The time we are simulating to                                */
        const unsigned int              _type;              /* The type of the material to associate it with a collider     */
};
}; /* namespace raptor_physics */

#endif /* #ifndef __PHYSICS_OBJECT_H__ */
