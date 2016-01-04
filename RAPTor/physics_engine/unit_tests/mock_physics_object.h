#pragma once

/* Standard headers */
#include <deque>

/* Common headers */
#include "point_t.h"
#include "force_info.h"

/* Physics headers */
#include "inertia_tensor.h"
#include "vertex_group.h"
#include "physics_object.h"
#include "polygon.h"


namespace raptor_physics
{
namespace test
{
/* Class to act as a vertex group while testing */
class mock_physics_object
{
    public :
        typedef mock_physics_object inner_vg;

        mock_physics_object(inertia_tensor *const i, const point_t &v = point_t(0.0, 0.0, 0.0), const point_t &w = point_t(0.0, 0.0, 0.0)) 
            : _i(i), _v(v), _w(w), _f(point_t(0.0, 0.0, 0.0)), _tor(point_t(0.0, 0.0, 0.0)) {  };

        ~mock_physics_object()
        {
            delete _i;
        }

        /* Setters */
        mock_physics_object& set_force(const point_t &f)
        {
            _f = f;
            return *this;
        }

        mock_physics_object& set_torque(const point_t &tor)
        {
            _tor = tor;
            return *this;
        }

        mock_physics_object& set_velocity(const point_t &v)
        {
            _v = v;
            return *this;
        }

        mock_physics_object& set_angular_velocity(const point_t &w)
        {
            _w = w;
            return *this;
        }

        mock_physics_object& apply_force(const point_t &at, const point_t &f, const float t)
        {
            /* No point pushing an infinite mass object */
            if (get_mass() == std::numeric_limits<float>::infinity())
            {
                return *this;
            }

            /* Add to force */
            _f += f;

            /* Add to the torque */
            point_t tor;
            cross_product(at, f, &tor);
            _tor += tor;

            return *this;
        }

        mock_physics_object& apply_internal_force(const point_t &at, const point_t &f)
        {
            if (get_mass() != std::numeric_limits<float>::infinity())
            {
                _f += f;
                _tor += cross_product(at, f);
            }
            
            return *this;
        }

        mock_physics_object& apply_impulse(const point_t &impulse, const point_t &poc, const bool)
        {
            const point_t l(cross_product(poc - get_center_of_mass(), impulse));

            _v += (impulse / _i->mass());
            _w += (l / get_orientated_tensor());
            return *this;
        }


        /* Getters */
        mock_physics_object *   get_vertex_group()                    { return this;                    }
        point_t                 get_force()                     const { return _f;                      }
        point_t                 get_torque()                    const { return _tor;                    }
        point_t                 get_velocity()                  const { return _v;                      }
        point_t                 get_angular_velocity()          const { return _w;                      }
        point_t                 get_center_of_mass()            const { return _i->center_of_mass();    }
        inertia_tensor&         get_inertia_tenor()             const { return *_i;                     }
        float                   get_mass()                      const { return _i->mass();              }

        point_t get_velocity(const point_t &p) const
        {
            return _v + cross_product(_w, p - _i->center_of_mass());
        }

        point_t get_momentum() const
        {
            /* Infinite mass objects shouldnt be moving */
            return (_i->mass() == std::numeric_limits<float>::infinity()) ? 0.0f : ( _i->mass() * _v);
        }

        point_t get_angular_momentum() const
        {
            /* Infinite mass objects shouldnt be moving */
            return (_i->mass() == std::numeric_limits<float>::infinity()) ? point_t(0.0f, 0.0f, 0.0f) : (get_orientated_tensor() * _w);
        }

        const inertia_tensor_view get_orientated_tensor() const
        {
            return inertia_tensor_view(*_i, quaternion_t(1.0f, 0.0f, 0.0f, 0.0f));
        }

        mock_physics_object& update_bounds()
        {
            return *this;
        }
        
        /* Setters */
        mock_physics_object& apply_impulse(const point_t &n, const point_t &angular_weight, const float impulse, const bool ob = true)
        {
            _v += (n * (impulse / _i->mass()));
            _w += (angular_weight * impulse);

            _n.push_back(n);
            _angular_weight.push_back(angular_weight);
            _impulse.push_back(impulse);

            return *this;
        }

        /* Result getters */
        point_t get_collision_normal()
        {
            const point_t tmp = _n.front();
            _n.pop_front();
            return tmp;
        }

        point_t get_angular_weight()
        {
            const point_t tmp = _angular_weight.front();
            _angular_weight.pop_front();
            return tmp;
        }

        float   get_impulse()
        {
            const float tmp = _impulse.front();
            _impulse.pop_front();
            return tmp;
        }

        int number_of_impulses() const { return _impulse.size(); }


    private :
        inertia_tensor *const   _i;
        std::deque<point_t>     _n;
        std::deque<point_t>     _angular_weight;
        std::deque<float>       _impulse;
        point_t                 _v;
        point_t                 _w;
        point_t                 _f;
        point_t                 _tor;
};

/* Build a physics object that is just functional enough to build a simplex with */
inline raptor_physics::physics_object* physics_object_for_simplex_testing(const std::vector<point_t> *const verts)
{
    return new raptor_physics::physics_object(new raptor_physics::vertex_group(verts, std::vector<polygon>(), nullptr), point_t(0.0, 0.0, 0.0), 0.0);
}

inline raptor_physics::physics_object* physics_object_for_simplex_testing(const std::vector<point_t> *const verts, const quaternion_t &o, const point_t &t)
{
    return new raptor_physics::physics_object(new raptor_physics::vertex_group(verts, std::vector<polygon>(), nullptr), o, t, 0.0);
}

inline raptor_physics::physics_object* physics_object_for_simplex_testing(const std::vector<point_t> *const verts, const std::vector<int> &polys, const quaternion_t &o, const point_t &t)
{
    return new raptor_physics::physics_object(new raptor_physics::vertex_group(verts, { polygon(verts, polys) }, nullptr), o, t, 0.0);
}
} /* namespace test */
} /* namespace raptor_physics */
