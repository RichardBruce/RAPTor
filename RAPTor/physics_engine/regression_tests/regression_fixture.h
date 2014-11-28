#ifndef __REGRESSION_FIXTURE_H__
#define __REGRESSION_FIXTURE_H__

/* Standard headers */
#include <chrono>
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
          : po(0.04f, 0.04f, -1, false, false),
            pe(new raptor_physics::rigid_body_collider(0.5f, 0.75f), false),
            se(&pe, &po),
            m(new raptor_raytracer::phong_shader(raptor_raytracer::ext_colour_t(255.0f, 255.0f, 255.0f), 1.0f)),
            _objects(),
            _runtime(0.0f),
            _frames_inv(0.0f),
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

            /* Log run time */
            BOOST_LOG_TRIVIAL(fatal) << "PERF 2 - Physics Time us: " << _runtime;
        }

        /* Add objects for testing */
        physics_object* make_plane(raptor_raytracer::material *m, const point_t &bl, const point_t &br, const point_t &tl, const point_t &tr, const point_t &com, const float density)
        {
            /* Check there is space */
            assert(_objects_idx < max_test_objects);

            /* Build physics object */
            new (&_objects[alignof(physics_object) + (_objects_idx * sizeof(physics_object))]) raptor_physics::physics_object(raptor_physics::make_plane(m, bl, br, tl, tr), com, density);
            return reinterpret_cast<physics_object *>(&_objects[alignof(physics_object) + (_objects_idx++ * sizeof(physics_object))]);
        }


        /* Routine to make cubes out of triangles */
        physics_object* make_cube(raptor_raytracer::material *m, const point_t &bl, const point_t &tr, const point_t &com, const float density, const int t = 0)
        {
            return make_cube(m, bl, tr, com, point_t(0.0f, 0.0f, 0.0f), point_t(0.0f, 0.0f, 0.0f), density, t);
        }

        physics_object* make_cube(raptor_raytracer::material *m, const point_t &bl, const point_t &tr, const point_t &com, const point_t &v, const point_t &w, const float density, const int t = 0)
        {
            return make_cube(m, quaternion_t(1.0f, 0.0f, 0.0f, 0.0f), bl, tr, com, v, w, density, t);
        }

        physics_object* make_cube(raptor_raytracer::material *m, const quaternion_t &o, const point_t &bl, const point_t &tr, const point_t &com, const point_t &v, const point_t &w, const float density, const int t = 0)
        {
            /* Check there is space */
            assert(_objects_idx < max_test_objects);

            /* Build physics object */
            new (&_objects[alignof(physics_object) + (_objects_idx * sizeof(physics_object))]) raptor_physics::physics_object(raptor_physics::make_cube(m, bl, tr), o, com, v, w, density, t);
            return reinterpret_cast<physics_object *>(&_objects[alignof(physics_object) + (_objects_idx++ * sizeof(physics_object))]);
        }

        void run(regression_checker *const checker, const int total_frames, const int frames, const bool initial = true)
        {
            /* Check starting state */
            if (initial)
            {
                BOOST_LOG_TRIVIAL(fatal) << "PERF 1 - # of frames: " << total_frames;
                checker->check(pe, 0);
                _frames_inv = 1.0f / static_cast<float>(total_frames);
            }

            /* Run some frames and check */
            for (int i = 1; i <= frames; ++i)
            {
                po.frames_to_run(1);

                /* Time frame */
                const auto t0(std::chrono::system_clock::now());
                BOOST_CHECK(se.run() == 0);
                const auto t1(std::chrono::system_clock::now());

                /* Check */
                checker->check(pe, i);

                /* Keep running average frame time */
                _runtime += (std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count() * _frames_inv);
            }
        }

        void run(regression_checker *const checker, const int frames, const bool initial = true)
        {
            run(checker, frames, frames, initial);
        }

        raptor_physics::physics_options         po;
        raptor_physics::physics_engine          pe;
        raptor_physics::simulation_environment  se;
        raptor_raytracer::material *            m;

    private :
        char    _objects[alignof(physics_object) + (max_test_objects * sizeof(physics_object))];
        float   _runtime;
        float   _frames_inv;
        int     _objects_idx;
};

#endif /* #ifndef __REGRESSION_FIXTURE_H__ */
