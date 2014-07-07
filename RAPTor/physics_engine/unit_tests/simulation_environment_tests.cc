#ifdef STAND_ALONE
#define BOOST_TEST_MODULE simulation_environment test

/* Common headers */
#include "logging.h"

/* Initialise logger */
const raptor_physics::init_logger init_logger;
#endif /* #ifdef STAND_ALONE */


/* Boost headers */
#include "boost/test/unit_test.hpp"
#include "boost/noncopyable.hpp"

/* Raytracer */
#include "material.h"

/* Physics headers */
#include "simulation_environment.h"
#include "physics_engine.h"
#include "physics_options.h"
#include "rigid_body_collider.h"


using namespace raptor_physics;

/* Test data */
struct simulation_environment_fixture : private boost::noncopyable
{
    simulation_environment_fixture()
    :  pe(new rigid_body_collider(0.9, 0.75)),
       po(-1.0, 0.0, 1, false, false), /* Cant check the rendered output so make sure that code isnt even run */
       m(new phong_shader(ext_colour_t(255.0, 255.0, 255.0), 1.0))
    {  };

    ~simulation_environment_fixture()
    {
        delete m;
    }

    physics_engine pe;
    physics_options po;
    material *m;
};


BOOST_FIXTURE_TEST_SUITE( simulation_environment_tests, simulation_environment_fixture )

const float result_tolerance = 0.0005;


/* Test Construction */
BOOST_AUTO_TEST_CASE( ctor_test )
{
    simulation_environment uut(&pe, &po);
    BOOST_CHECK(uut.engine() == &pe);
    BOOST_CHECK(uut.time_run() == 0.0);
    BOOST_CHECK(uut.engine()->number_of_objects() == 0);
    BOOST_CHECK(uut.engine()->number_of_moving_objects() == 0);
    BOOST_CHECK(uut.number_of_lights() == 0);
    BOOST_CHECK(uut.screen_initialised() == false);
    BOOST_CHECK(uut.font_initialised() == false);
    BOOST_CHECK(uut.load_screen_initialised() == false);
}


/* Test add light */
BOOST_AUTO_TEST_CASE( add_light_test )
{
    simulation_environment uut(&pe, &po);
    BOOST_CHECK(uut.engine() == &pe);
    BOOST_CHECK(uut.time_run() == 0.0);
    BOOST_CHECK(uut.engine()->number_of_objects() == 0);
    BOOST_CHECK(uut.engine()->number_of_moving_objects() == 0);
    BOOST_CHECK(uut.number_of_lights() == 0);
    BOOST_CHECK(uut.screen_initialised() == false);
    BOOST_CHECK(uut.font_initialised() == false);
    BOOST_CHECK(uut.load_screen_initialised() == false);

    uut.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(0.0, 0.0, 1.0));
    BOOST_CHECK(uut.engine() == &pe);
    BOOST_CHECK(uut.time_run() == 0.0);
    BOOST_CHECK(uut.engine()->number_of_objects() == 0);
    BOOST_CHECK(uut.engine()->number_of_moving_objects() == 0);
    BOOST_CHECK(uut.number_of_lights() == 1);
    BOOST_CHECK(uut.screen_initialised() == false);
    BOOST_CHECK(uut.font_initialised() == false);
    BOOST_CHECK(uut.load_screen_initialised() == false);

    uut.add_light(ext_colour_t(125.0, 0.0, 64.0), point_t(-10.0, 0.0, 5.0));
    BOOST_CHECK(uut.engine() == &pe);
    BOOST_CHECK(uut.time_run() == 0.0);
    BOOST_CHECK(uut.engine()->number_of_objects() == 0);
    BOOST_CHECK(uut.engine()->number_of_moving_objects() == 0);
    BOOST_CHECK(uut.number_of_lights() == 2);
    BOOST_CHECK(uut.screen_initialised() == false);
    BOOST_CHECK(uut.font_initialised() == false);
    BOOST_CHECK(uut.load_screen_initialised() == false);
}


/* Test add object */
BOOST_AUTO_TEST_CASE( add_object_test )
{
    simulation_environment uut(&pe, &po);
    BOOST_CHECK(uut.engine() == &pe);
    BOOST_CHECK(uut.time_run() == 0.0);
    BOOST_CHECK(uut.engine()->number_of_objects() == 0);
    BOOST_CHECK(uut.engine()->number_of_moving_objects() == 0);
    BOOST_CHECK(uut.number_of_lights() == 0);
    BOOST_CHECK(uut.screen_initialised() == false);
    BOOST_CHECK(uut.font_initialised() == false);
    BOOST_CHECK(uut.load_screen_initialised() == false);

    uut.add_object(new physics_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5)), point_t(0.0, 9.5, 0.0), 10.0));
    BOOST_CHECK(uut.engine() == &pe);
    BOOST_CHECK(uut.time_run() == 0.0);
    BOOST_CHECK(uut.engine()->number_of_objects() == 1);
    BOOST_CHECK(uut.engine()->number_of_moving_objects() == 0);
    BOOST_CHECK(uut.number_of_lights() == 0);
    BOOST_CHECK(uut.screen_initialised() == false);
    BOOST_CHECK(uut.font_initialised() == false);
    BOOST_CHECK(uut.load_screen_initialised() == false);

    uut.add_object(new physics_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5)), point_t(0.0, -9.5, 0.0), 10.0));
    BOOST_CHECK(uut.engine() == &pe);
    BOOST_CHECK(uut.time_run() == 0.0);
    BOOST_CHECK(uut.engine()->number_of_objects() == 2);
    BOOST_CHECK(uut.engine()->number_of_moving_objects() == 0);
    BOOST_CHECK(uut.number_of_lights() == 0);
    BOOST_CHECK(uut.screen_initialised() == false);
    BOOST_CHECK(uut.font_initialised() == false);
    BOOST_CHECK(uut.load_screen_initialised() == false);
}


/* Test add moving object */
BOOST_AUTO_TEST_CASE( add_moving_object_test )
{
    simulation_environment uut(&pe, &po);
    BOOST_CHECK(uut.engine() == &pe);
    BOOST_CHECK(uut.time_run() == 0.0);
    BOOST_CHECK(uut.engine()->number_of_objects() == 0);
    BOOST_CHECK(uut.engine()->number_of_moving_objects() == 0);
    BOOST_CHECK(uut.number_of_lights() == 0);
    BOOST_CHECK(uut.screen_initialised() == false);
    BOOST_CHECK(uut.font_initialised() == false);
    BOOST_CHECK(uut.load_screen_initialised() == false);

    uut.add_moving_object(new physics_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5)), point_t(0.0, 9.5, 0.0), 10.0));
    BOOST_CHECK(uut.engine() == &pe);
    BOOST_CHECK(uut.time_run() == 0.0);
    BOOST_CHECK(uut.engine()->number_of_objects() == 1);
    BOOST_CHECK(uut.engine()->number_of_moving_objects() == 1);
    BOOST_CHECK(uut.number_of_lights() == 0);
    BOOST_CHECK(uut.screen_initialised() == false);
    BOOST_CHECK(uut.font_initialised() == false);
    BOOST_CHECK(uut.load_screen_initialised() == false);

    uut.add_moving_object(new physics_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5)), point_t(0.0, -9.5, 0.0), 10.0));
    BOOST_CHECK(uut.engine() == &pe);
    BOOST_CHECK(uut.time_run() == 0.0);
    BOOST_CHECK(uut.engine()->number_of_objects() == 2);
    BOOST_CHECK(uut.engine()->number_of_moving_objects() == 2);
    BOOST_CHECK(uut.number_of_lights() == 0);
    BOOST_CHECK(uut.screen_initialised() == false);
    BOOST_CHECK(uut.font_initialised() == false);
    BOOST_CHECK(uut.load_screen_initialised() == false);
}


/* Test run */
BOOST_AUTO_TEST_CASE( run_one_frame_no_fps_limit_test )
{
    simulation_environment uut(&pe, &po);
    uut.add_moving_object(new physics_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5)), point_t(0.0, 9.5, 0.0), 10.0));
    uut.add_moving_object(new physics_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5)), point_t(0.0, -9.5, 0.0), 10.0));
    BOOST_CHECK(uut.engine() == &pe);
    BOOST_CHECK(uut.time_run() == 0.0);
    BOOST_CHECK(uut.engine()->number_of_objects() == 2);
    BOOST_CHECK(uut.engine()->number_of_moving_objects() == 2);
    BOOST_CHECK(uut.number_of_lights() == 0);
    BOOST_CHECK(uut.screen_initialised() == false);
    BOOST_CHECK(uut.font_initialised() == false);
    BOOST_CHECK(uut.load_screen_initialised() == false);

    po.frames_to_run(1);
    BOOST_CHECK(uut.run() == 0);
    BOOST_CHECK(uut.time_run() >= 0.0);

    po.frames_to_run(1);
    BOOST_CHECK(uut.run() == 0);
    BOOST_CHECK(uut.time_run() >= 0.0);

    po.frames_to_run(1);
    BOOST_CHECK(uut.run() == 0);
    BOOST_CHECK(uut.time_run() >= 0.0);
}


BOOST_AUTO_TEST_CASE( run_one_frame_fps_limit_test )
{
    simulation_environment uut(&pe, &po);
    BOOST_CHECK(uut.engine() == &pe);
    BOOST_CHECK(uut.time_run() == 0.0);
    BOOST_CHECK(uut.engine()->number_of_objects() == 0);
    BOOST_CHECK(uut.engine()->number_of_moving_objects() == 0);
    BOOST_CHECK(uut.number_of_lights() == 0);
    BOOST_CHECK(uut.screen_initialised() == false);
    BOOST_CHECK(uut.font_initialised() == false);
    BOOST_CHECK(uut.load_screen_initialised() == false);

    po.max_timestep(0.5);
    po.min_timestep(0.5);
    po.frames_to_run(1);
    BOOST_CHECK(uut.run() == 0);
    BOOST_CHECK(uut.time_run() == 0.5);

    po.frames_to_run(1);
    BOOST_CHECK(uut.run() == 0);
    BOOST_CHECK(uut.time_run() == 0.5);

    po.frames_to_run(1);
    BOOST_CHECK(uut.run() == 0);
    BOOST_CHECK(uut.time_run() == 0.5);
}


BOOST_AUTO_TEST_CASE( run_multiple_frames_fps_limit_test )
{
    simulation_environment uut(&pe, &po);
    BOOST_CHECK(uut.engine() == &pe);
    BOOST_CHECK(uut.time_run() == 0.0);
    BOOST_CHECK(uut.engine()->number_of_objects() == 0);
    BOOST_CHECK(uut.engine()->number_of_moving_objects() == 0);
    BOOST_CHECK(uut.number_of_lights() == 0);
    BOOST_CHECK(uut.screen_initialised() == false);
    BOOST_CHECK(uut.font_initialised() == false);
    BOOST_CHECK(uut.load_screen_initialised() == false);

    po.max_timestep(0.25);
    po.min_timestep(0.25);
    po.frames_to_run(5);
    BOOST_CHECK(uut.run() == 0);
    BOOST_CHECK(uut.time_run() == 1.25);

    po.frames_to_run(10);
    BOOST_CHECK(uut.run() == 0);
    BOOST_CHECK(uut.time_run() == 2.5);

    po.frames_to_run(2);
    BOOST_CHECK(uut.run() == 0);
    BOOST_CHECK(uut.time_run() == 0.5);
}


/* Test load screen */
BOOST_AUTO_TEST_CASE (load_screen_test )
{
    simulation_environment uut(&pe, &po);
    BOOST_CHECK(uut.engine() == &pe);
    BOOST_CHECK(uut.time_run() == 0.0);
    BOOST_CHECK(uut.engine()->number_of_objects() == 0);
    BOOST_CHECK(uut.engine()->number_of_moving_objects() == 0);
    BOOST_CHECK(uut.number_of_lights() == 0);
    BOOST_CHECK(uut.screen_initialised() == false);
    BOOST_CHECK(uut.font_initialised() == false);
    BOOST_CHECK(uut.load_screen_initialised() == false);

    /* Set load screen */
    uut.load_screen("./load_screen.png");
    BOOST_CHECK(uut.load_screen_initialised() == false);    /* No renderer so cant load image */
}

BOOST_AUTO_TEST_SUITE_END()
