#pragma once

/* Standard headers */
#include <algorithm>
#include <vector>

/* Boost headers */
#include "boost/noncopyable.hpp"

/* Common headers */
#include "point_t.h"
#include "quaternion_t.h"

/* Physics headers */
#include "collision.h"
#include "vertex_group.h"
#include "object_bound.h"
#include "force.h"
#include "integrators.h"


namespace raptor_physics
{
/* Forward declarations */
class simplex;
class physics_engine;

/* Class to wrap a vertex group with the physics engine specific functionality */
class physics_object : private boost::noncopyable
{
    public :
        /* Ownership isnt taken of vg */
        physics_object(vertex_group *const vg, const point_t<> &com, const float density, const unsigned int t = 0) : 
            physics_object(vg, quaternion_t(1.0f, 0.0f, 0.0f, 0.0f), com, density, t) {  };
              
        physics_object(vertex_group *const vg, const quaternion_t &o, const point_t<> &com, const float density, const unsigned int t = 0) : 
            physics_object(vg, o, com, point_t<>(0.0f, 0.0f, 0.0f), point_t<>(0.0f, 0.0f, 0.0f), density, t) {  };

        physics_object(vertex_group *const vg, const quaternion_t &o, const point_t<> &com, const point_t<> &v, const point_t<> &w, const float density, const unsigned int t = 0) :
            _vg(vg),
            _forces(new std::vector<force *>()),
            _agg_force(*_forces),
            _i(_vg->build_inertia_tensor(density)),
            _w(w),
            _v(v),
            _lw(w),
            _lv(v),
            _o(o),
            _cur_t(0.0f),
            _t_step(0.0f),
            _type(t)
        {
            _i->move_center_of_mass(com - _i->center_of_mass());

            /* Build bounds */
            point_t<> hi;
            point_t<> lo;
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
            _cur_t  = 0.0f;
            _t_step = t_step;

            /* Update the bounds for the full time step */
            update_bounds();

            return *this;
        }

        /* Physics calculations for applied forces */
        /* Note -- Technically the force should stop after f.t */
        /*         This is appoximated by weighting the force in proportion to the period it is applied */
        /* TODO - Is it worth checking that the force is applied on the object not in free space? */
        physics_object& apply_force(const point_t<> &at, const point_t<> &f, const float t)
        {
            /* No point pushing an infinite mass object */
            if (get_mass() == std::numeric_limits<float>::infinity())
            {
                return *this;
            }

            register_force(new const_force(at, f, t));

            return *this;
        }

        physics_object& apply_internal_force(const point_t<> &at, const point_t<> &f)
        {
            _agg_force.apply_internal_force(at, f);
            return *this;
        }

        physics_object& clear_internal_forces()
        {
            _agg_force.clear_internal_forces();
            update_bounds();
            return *this;
        }

        physics_object& apply_impulse(const point_t<> &impulse, const point_t<> &poc, const bool ub = true)
        {
            const point_t<> l(cross_product(poc - get_center_of_mass(), impulse));
            // BOOST_LOG_TRIVIAL(trace) << "l: " << l;

            _v  = _lv + (impulse / _i->mass());
            _lv = _v;
            _w  = _lw + (l / get_orientated_tensor());
            _lw = _w;

            /* Update the bounds for the full time step */
            if (ub)
            {
                update_bounds();
            }

            return *this;
        }

        physics_object& apply_impulse(const point_t<> &n, const point_t<> &angular_weight, const float impulse, const bool ub = true)
        {
            _v  = _lv + (n * (impulse / _i->mass()));
            _lv = _v;
            _w  = _lw + (angular_weight * impulse);
            _lw = _w;
            BOOST_LOG_TRIVIAL(trace) << "Impulsed lv: " << _v;

            /* Update the bounds for the full time step */
            if (ub)
            {
                update_bounds();
            }

            return *this;
        }
        
        /* Collision detection */
        bool has_collided(physics_object *const po, simplex **const manifold_a, simplex **const manifold_b, point_t<> *const d, const float t0, const float t1);
        bool has_collided(physics_object *const po, simplex **const manifold_a, simplex **const manifold_b, point_t<> *const d, const quaternion_t &oa, const quaternion_t &ob, const float t0, const float t1);
        
        /* Check for collisions and separate objects as needed */
        collision_t resolve_collisions(physics_object *const po, simplex **const manifold_a, simplex **const manifold_b, float *const t, const bool sliding)
        {
            /* If no rotation use exact */
            // if ((fabs(magnitude(    _w)) < raptor_physics::EPSILON) && (fabs(magnitude(    get_torque())) < raptor_physics::EPSILON) &&
            //     (fabs(magnitude(po->_w)) < raptor_physics::EPSILON) && (fabs(magnitude(po->get_torque())) < raptor_physics::EPSILON))
            // {
            //     return exactly_resolve_collisions(po, manifold_a, manifold_b, t);
            // }
            // /* Else must be conservative */
            // else
            // {
                return conservatively_resolve_collisions(po, manifold_a, manifold_b, t, sliding);
            // }
        }
        
        physics_object& commit_movement(const float t);

        const physics_object& triangles(raptor_raytracer::primitive_store *p) const
        {
            _vg->triangles(p, _o, _i->center_of_mass());
            return *this;
        }

        /* Access functions */
        /* Physics getters */
        unsigned int                get_physical_type()     const { return _type;                                                       }
        const point_t<>               get_force()             const { return _agg_force.get_force(*_i, get_center_of_mass(), _lv, 0.0f);  }
        const point_t<>               get_torque()            const { return _agg_force.get_torque(*_i, get_center_of_mass(), _lw, 0.0f); }
        const float                 get_speed()             const { return magnitude(_v);                                               }
        const float                 get_mass()              const { return _i->mass();                                                  }
        const point_t<>&              get_center_of_mass()    const { return _i->center_of_mass();                                        }
        const quaternion_t&         get_orientation()       const { return _o;                                                          }
        const inertia_tensor&       get_inertia_tenor()     const { return *_i;                                                         }
        const inertia_tensor_view   get_orientated_tensor() const { return inertia_tensor_view(*_i, _o);                                }

        const point_t<>&  get_velocity_0()                const { return _v;  }
        const point_t<>&  get_velocity()                  const { return _lv; }
        const point_t<>&  get_angular_velocity_0()        const { return _w;  }
        const point_t<>&  get_angular_velocity()          const { return _lw; }
        point_t<>         get_velocity(const point_t<> &p)  const
        {
            /* Get rotational velocity at p */
            const point_t<> r(p - _i->center_of_mass());
            const point_t<> rot_vel(cross_product(_lw, r));

            return _lv + rot_vel;
        }

        point_t<>         get_velocity_0(const point_t<> &p)  const
        {
            /* Get rotational velocity at p */
            const point_t<> r(p - _i->center_of_mass());
            const point_t<> rot_vel(cross_product(_w, r));

            return _v + rot_vel;
        }

        point_t<> get_acceleration(const point_t<> &poc) const
        {
            const point_t<> lin(get_force() / _i->mass());
            const point_t<> rot(cross_product(get_torque() / get_orientated_tensor(), poc - _i->center_of_mass()));
            return lin + rot;
        }

        const point_t<> get_momentum() const
        {
            /* Infinite mass objects shouldnt be moving */
            return (_i->mass() == std::numeric_limits<float>::infinity()) ? 0.0f : ( _i->mass() * _lv);
        }

        const point_t<> get_angular_momentum() const
        {
            /* Infinite mass objects shouldnt be moving */
            return (_i->mass() == std::numeric_limits<float>::infinity()) ? point_t<>(0.0f, 0.0f, 0.0f) : (get_orientated_tensor() * _lw);
        }

        /* Vertex getters */
        const std::shared_ptr<vertex_group>& get_vertex_group() const { return _vg; }
        point_t<> get_orientated_vertex(const int i) const
        {
            return _o.rotate(_vg->get_vertex(i));
        }

        point_t<> get_global_vertex(const int i) const
        {
            return get_orientated_vertex(i) + get_center_of_mass();
        }

        const physics_object& vertex_to_global(point_t<> *const p) const
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
        physics_object& set_velocity(const point_t<> &v) 
        {
            _v  = v;
            _lv = v;
            return *this;
        }

        physics_object& set_angular_velocity(const point_t<> &w)
        {
            _w  = w;
            _lw = w;
            return *this;
        }

        /* Bounds access */
        object_bound *upper_bound(const axis_t axis) const
        {
            return _upper_bound[static_cast<int>(axis)];
        }

        object_bound *lower_bound(const axis_t axis) const
        {
            return _lower_bound[static_cast<int>(axis)];
        }

        /* Update of bounding box */
        physics_object& update_bounds()
        {
            // METHOD_LOG;

            /* Calculate remaining time step */
            const float dt = _t_step - _cur_t;

            /* Get the bounds of the stationary object */
            point_t<> hi;
            point_t<> lo;
            get_bounds(&hi, &lo);
            BOOST_LOG_TRIVIAL(trace) << "Input vel: " << _v << ", and: " << _w;

            /* Get the translation of the object */
            rk4_integrator integ;
            const point_t<> trans(integ.project_translation(_agg_force, *_i, _v, dt));

            /* Set bounds to current position plus any translation */
            _lower_bound[0]->bound(lo.x + std::min(trans.x, 0.0f));
            _upper_bound[0]->bound(hi.x + std::max(trans.x, 0.0f));
            _lower_bound[1]->bound(lo.y + std::min(trans.y, 0.0f));
            _upper_bound[1]->bound(hi.y + std::max(trans.y, 0.0f));
            _lower_bound[2]->bound(lo.z + std::min(trans.z, 0.0f));
            _upper_bound[2]->bound(hi.z + std::max(trans.z, 0.0f));

            /* Get the rotation of the object */
            const quaternion_t rot(integ.project_rotation(_agg_force, *_i, _o, _w, dt));

            /* Update the descrtised motion for this frame */
            if (dt > raptor_physics::EPSILON)
            {
                quaternion_t o_inv(_o);
                o_inv.inverse();
                const quaternion_t qlw((rot / (dt * 0.5f)) * o_inv);
                _lw.x = qlw.x;
                _lw.y = qlw.y;
                _lw.z = qlw.z;
                _lv = trans / dt;
                BOOST_LOG_TRIVIAL(trace) << "Re-integrated lv: " << _lv;
            }
            else
            {
                _lw = point_t<>(0.0f, 0.0f, 0.0f);
                _lv = point_t<>(0.0f, 0.0f, 0.0f);
            }

            return *this;
        }

    private :
        /* Bounding box */
        const physics_object& get_bounds(point_t<> *const hi, point_t<> *const lo) const
        {
            /* Get the bounds of the vertex group */
            point_t<> local_hi;
            point_t<> local_lo;
            _vg->get_bounds(&local_hi, &local_lo);

            /* Move bounds into global co-ordinates, an OOB */
            const point_t<> p0(_o.rotate(local_lo) + _i->center_of_mass());
            const point_t<> p1(_o.rotate(point_t<>(local_lo.x, local_lo.y, local_hi.x)) + _i->center_of_mass());
            const point_t<> p2(_o.rotate(point_t<>(local_lo.x, local_hi.y, local_lo.x)) + _i->center_of_mass());
            const point_t<> p3(_o.rotate(point_t<>(local_lo.x, local_hi.y, local_hi.x)) + _i->center_of_mass());
            const point_t<> p4(_o.rotate(point_t<>(local_hi.x, local_lo.y, local_lo.x)) + _i->center_of_mass());
            const point_t<> p5(_o.rotate(point_t<>(local_hi.x, local_lo.y, local_hi.x)) + _i->center_of_mass());
            const point_t<> p6(_o.rotate(point_t<>(local_hi.x, local_hi.y, local_lo.x)) + _i->center_of_mass());
            const point_t<> p7(_o.rotate(local_hi) + _i->center_of_mass());

            /* Find the axis aligned bounds of the OOB */
            (*hi) = max(max(max(p0, p1), max(p2, p3)), max(max(p4, p5), max(p6, p7))) + WELD_DISTANCE;
            (*lo) = min(min(min(p0, p1), min(p2, p3)), min(min(p4, p5), min(p6, p7))) - WELD_DISTANCE;


            return *this;
        }
        
        /* Find the projection of the largest rotational movement in the direction n */
        float project_maximum_rotation_onto(const point_t<> &n, const point_t<> &p, const float t) const;

        /* Calculate the longest possible movement, assuming worst case rotation */
        /* of the vertices in the direction n and of v's vertices in the direction of -n */
        float project_maximum_movement_onto(const physics_object &v, const point_t<> &p_a, const point_t<> &p_b, const point_t<> &n, const float t) const;

        /* Check for collisions when the exact object movement can be calculated */
        collision_t exactly_resolve_collisions(physics_object *const vg, simplex **const manifold_a, simplex **const manifold_b, float *const t);

        /* Check for collisions and conservatively separate objects as needed */
        /* Objects should be moved and retested before accepting a collision */
        collision_t conservatively_resolve_collisions(physics_object *const vg, simplex **const manifold_a, simplex **const manifold_b, float *const t, const bool sliding);

        /* Specialise algorithm for calculating toc when two objects are very close */
        collision_t close_contact_collision_detection(physics_object *const vg, simplex **const manifold_a, simplex **const manifold_b, float *const t, const point_t<> &noc, float min_t, const float d_t0, const bool sliding);

        /* Get the translation and rotation at a given time */
        void configuration_at_time(quaternion_t *const o, point_t<> *const tr, const float t) const;

        /* Find the point on vg that moves the futhest beyond the plane defined by plane_n and plane_w */
        int find_deepest_intersection(const vertex_group &vg, const quaternion_t &pl_r, const quaternion_t &ol_r, const point_t<> &pl_t, const point_t<> &ol_t, const point_t<> &plane_n, const point_t<> &plane_w, float *const d) const;
        

        std::shared_ptr<vertex_group>   _vg;
        std::vector<force *>   *        _forces;
        aggregate_force                 _agg_force;
        inertia_tensor      *const      _i;                 /* The moment of inertia                                        */
        object_bound *                  _upper_bound[3];    /* The upper bound of the object in this time step              */
        object_bound *                  _lower_bound[3];    /* The lower bound of the object in this time step              */
        point_t<>                       _w;                 /* Angular velocity                                             */
        point_t<>                       _v;                 /* Velocity                                                     */
        point_t<>                       _lw;                /* Descretised angular velocity for this frame                  */
        point_t<>                       _lv;                /* Descretised velocity for this frame                          */
        quaternion_t                    _o;                 /* Orientation                                                  */
        float                           _cur_t;             /* The committed time                                           */
        float                           _t_step;            /* The time we are simulating to                                */
        const unsigned int              _type;              /* The type of the material to associate it with a collider     */
};
}; /* namespace raptor_physics */
