#ifndef __REGRESSION_FIXTURE_H__
#define __REGRESSION_FIXTURE_H__

/* Standard headers */
#include <vector>

/* Common headers */
#include "point_t.h"

/* Boost headers */
#include "boost/test/unit_test.hpp"
#include "boost/noncopyable.hpp"

/* Raytracer headers */
#include "phong_shader.h"

/* Physics headers */
#include "physics_options.h"
#include "physics_engine.h"
#include "rigid_body_collider.h"
#include "simulation_environment.h"
#include "vertex_group.h"

/* Test headers */
#include "regression_checker.h"

/* Maximum number of objects allowed in the regression tests */
/* I feel a bit dirty having this, but while the physics engine has maps keyed off pointers it is needed */
/* I'm not prepared to add size and runtime to the engine for the sake of this */
const int max_test_objects = 10000;

/* Test data */
struct regression_fixture : private boost::noncopyable
{
    public :
        regression_fixture()
          : po(0.04, 0.04, -1, false, false),
            pe(new raptor_physics::rigid_body_collider(0.5, 0.75), false),
            se(&pe, &po),
            m(new phong_shader(ext_colour_t(255.0, 255.0, 255.0), 1.0)),
            _objects(),
            _objects_idx(0)
        {  };

        /* Clean up objects made with placement new */
        ~regression_fixture()
        {
            for (int i = 0; i < _objects_idx; ++i)
            {
                physics_object *p = reinterpret_cast<physics_object *>(&_objects[alignof(physics_object) + (i * sizeof(physics_object))]);
                p->~physics_object();
            }

            delete m;
        }

        /* Add objects for testing */
        physics_object* make_plane(material *m, const point_t &bl, const point_t &br, const point_t &tl, const point_t &tr, const point_t &com, const fp_t density)
        {
            /* Check there is space */
            assert(_objects_idx < max_test_objects);

            /* Build physics object */
            new (&_objects[alignof(physics_object) + (_objects_idx * sizeof(physics_object))]) raptor_physics::physics_object(raptor_physics::make_plane(m, bl, br, tl, tr), com, density);
            return reinterpret_cast<physics_object *>(&_objects[alignof(physics_object) + (_objects_idx++ * sizeof(physics_object))]);
        }


        /* Routine to make cubes out of triangles */
        physics_object* make_cube(material *m, const point_t &bl, const point_t &tr, const point_t &com, const fp_t density, const int t = 0)
        {
            return make_cube(m, bl, tr, com, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), density, t);
        }

        physics_object* make_cube(material *m, const point_t &bl, const point_t &tr, const point_t &com, const point_t &v, const point_t &w, const fp_t density, const int t = 0)
        {
            return make_cube(m, quaternion_t(1.0, 0.0, 0.0, 0.0), bl, tr, com, v, w, density, t);
        }

        physics_object* make_cube(material *m, const quaternion_t &o, const point_t &bl, const point_t &tr, const point_t &com, const point_t &v, const point_t &w, const fp_t density, const int t = 0)
        {
            /* Check there is space */
            assert(_objects_idx < max_test_objects);

            /* Build physics object */
            new (&_objects[alignof(physics_object) + (_objects_idx * sizeof(physics_object))]) raptor_physics::physics_object(raptor_physics::make_cube(m, bl, tr), o, com, v, w, density, t);
            return reinterpret_cast<physics_object *>(&_objects[alignof(physics_object) + (_objects_idx++ * sizeof(physics_object))]);
        }

        raptor_physics::physics_options po;
        raptor_physics::physics_engine pe;
        raptor_physics::simulation_environment se;
        material *m;

    private :
        char    _objects[alignof(physics_object) + (max_test_objects * sizeof(physics_object))];
        int     _objects_idx;
};

#endif /* #ifndef __REGRESSION_FIXTURE_H__ */
