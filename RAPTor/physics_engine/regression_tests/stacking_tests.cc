#ifdef STAND_ALONE
#define BOOST_TEST_MODULE stacking test

/* Common headers */
#include "logging.h"

/* Initialise logger */
const raptor_physics::init_logger init_logger;
#endif /* #ifdef STAND_ALONE */

/* Test headers */
#include "regression_fixture.h"


namespace raptor_physics
{
namespace test
{
BOOST_FIXTURE_TEST_SUITE( stacking_tests, regression_fixture );

// BOOST_AUTO_TEST_CASE( four_straight_stack_10_fps_cor_0_1_test )
// {
//     /* Checker */
//     CREATE_REGRESSION_CHECKER(checker);

//     /* Enviroment set up */
//     po.max_timestep(0.1);
//     po.min_timestep(0.1);
//     pe.default_collider(new rigid_body_collider(0.1, 0.75));

//     /* Lights */
//     se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0,  10.0, -100.0));
//     se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0, -20.0, -100.0));

//     /* Add moving objects */
//     se.add_moving_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t(0.0, -4.5, 0.0), 10.0));
//     se.add_moving_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t(0.0,  0.5, 0.0), 20.0));
//     se.add_moving_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t(0.0,  5.5, 0.0), 10.0));
//     se.add_moving_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t(0.0, 10.5, 0.0), 20.0));

//     /* Add static objects */
//     se.add_object(make_plane(m, point_t(-10.0, 0.0, -10.0), point_t(10.0, 0.0, -10.0), point_t(-10.0, 0.0, 10.0), point_t(10.0, 0.0, 10.0), point_t(0.0, -10.0, 0.0), std::numeric_limits<float>::infinity()));

//     /* Run physics simulation */
//     run(&checker, 150);
// }


// BOOST_AUTO_TEST_CASE( four_straight_stack_10_fps_cor_0_5_test )
// {
//     /* Checker */
//     CREATE_REGRESSION_CHECKER(checker);

//     /* Enviroment set up */
//     po.max_timestep(0.1);
//     po.min_timestep(0.1);
//     pe.default_collider(new rigid_body_collider(0.5, 0.75));

//     /* Lights */
//     se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0,  10.0, -100.0));
//     se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0, -20.0, -100.0));

//     /* Add moving objects */
//     se.add_moving_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t(0.0, -4.5, 0.0), 10.0));
//     se.add_moving_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t(0.0,  0.5, 0.0), 20.0));
//     se.add_moving_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t(0.0,  5.5, 0.0), 10.0));
//     se.add_moving_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t(0.0, 10.5, 0.0), 20.0));

//     /* Add static objects */
//     se.add_object(make_plane(m, point_t(-10.0, 0.0, -10.0), point_t(10.0, 0.0, -10.0), point_t(-10.0, 0.0, 10.0), point_t(10.0, 0.0, 10.0), point_t(0.0, -10.0, 0.0), std::numeric_limits<float>::infinity()));

//     /* Run physics simulation */
//     run(&checker, 150);
// }


BOOST_AUTO_TEST_CASE( four_straight_stack_10_fps_cor_0_9_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    po.max_timestep(0.1);
    po.min_timestep(0.1);
    pe.default_collider(new rigid_body_collider(0.9, 0.75, 0.75));

    /* Lights */
    se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0,  10.0, -100.0));
    se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0, -20.0, -100.0));

    /* Add moving objects */
    se.add_moving_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t(0.0, -4.5, 0.0), 10.0));
    se.add_moving_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t(0.0,  0.5, 0.0), 20.0));
    se.add_moving_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t(0.0,  5.5, 0.0), 10.0));
    se.add_moving_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t(0.0, 10.5, 0.0), 20.0));

    /* Add static objects */
    se.add_object(make_plane(m, point_t(-10.0, 0.0, -10.0), point_t(10.0, 0.0, -10.0), point_t(-10.0, 0.0, 10.0), point_t(10.0, 0.0, 10.0), point_t(0.0, -10.0, 0.0), std::numeric_limits<float>::infinity()));

    /* Run physics simulation */
    run(&checker, 150);
}


// BOOST_AUTO_TEST_CASE( four_straight_stack_25_fps_cor_0_1_test )
// {
//     /* Checker */
//     CREATE_REGRESSION_CHECKER(checker);

//     /* Enviroment set up */
//     po.max_timestep(0.04);
//     po.min_timestep(0.04);
//     pe.default_collider(new rigid_body_collider(0.1, 0.75));

//     /* Lights */
//     se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0,  10.0, -100.0));
//     se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0, -20.0, -100.0));

//     /* Add moving objects */
//     se.add_moving_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t(0.0, -4.5, 0.0), 10.0));
//     se.add_moving_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t(0.0,  0.5, 0.0), 20.0));
//     se.add_moving_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t(0.0,  5.5, 0.0), 10.0));
//     se.add_moving_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t(0.0, 10.5, 0.0), 20.0));

//     /* Add static objects */
//     se.add_object(make_plane(m, point_t(-10.0, 0.0, -10.0), point_t(10.0, 0.0, -10.0), point_t(-10.0, 0.0, 10.0), point_t(10.0, 0.0, 10.0), point_t(0.0, -10.0, 0.0), std::numeric_limits<float>::infinity()));

//     /* Run physics simulation */
//     run(&checker, 200);
// }


// BOOST_AUTO_TEST_CASE( four_straight_stack_25_fps_cor_0_5_test )
// {
//     /* Checker */
//     CREATE_REGRESSION_CHECKER(checker);

//     /* Enviroment set up */
//     po.max_timestep(0.04);
//     po.min_timestep(0.04);
//     pe.default_collider(new rigid_body_collider(0.5, 0.75));

//     /* Lights */
//     se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0,  10.0, -100.0));
//     se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0, -20.0, -100.0));

//     /* Add moving objects */
//     se.add_moving_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t(0.0, -4.5, 0.0), 10.0));
//     se.add_moving_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t(0.0,  0.5, 0.0), 20.0));
//     se.add_moving_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t(0.0,  5.5, 0.0), 10.0));
//     se.add_moving_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t(0.0, 10.5, 0.0), 20.0));

//     /* Add static objects */
//     se.add_object(make_plane(m, point_t(-10.0, 0.0, -10.0), point_t(10.0, 0.0, -10.0), point_t(-10.0, 0.0, 10.0), point_t(10.0, 0.0, 10.0), point_t(0.0, -10.0, 0.0), std::numeric_limits<float>::infinity()));

//     /* Run physics simulation */
//     run(&checker, 200);
// }


// BOOST_AUTO_TEST_CASE( four_straight_stack_25_fps_cor_0_9_test )
// {
//     /* Checker */
//     CREATE_REGRESSION_CHECKER(checker);

//     /* Enviroment set up */
//     po.max_timestep(0.04);
//     po.min_timestep(0.04);
//     pe.default_collider(new rigid_body_collider(0.9, 0.75));

//     /* Lights */
//     se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0,  10.0, -100.0));
//     se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0, -20.0, -100.0));

//     /* Add moving objects */
//     se.add_moving_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t(0.0, -4.5, 0.0), 10.0));
//     se.add_moving_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t(0.0,  0.5, 0.0), 20.0));
//     se.add_moving_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t(0.0,  5.5, 0.0), 10.0));
//     se.add_moving_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t(0.0, 10.5, 0.0), 20.0));

//     /* Add static objects */
//     se.add_object(make_plane(m, point_t(-10.0, 0.0, -10.0), point_t(10.0, 0.0, -10.0), point_t(-10.0, 0.0, 10.0), point_t(10.0, 0.0, 10.0), point_t(0.0, -10.0, 0.0), std::numeric_limits<float>::infinity()));

//     /* Run physics simulation */
//     run(&checker, 250);
// }


// BOOST_AUTO_TEST_CASE( four_straight_stack_60_fps_cor_0_1_test )
// {
//     /* Checker */
//     CREATE_REGRESSION_CHECKER(checker);

//     /* Enviroment set up */
//     po.max_timestep(1.0 / 60.0);
//     po.min_timestep(1.0 / 60.0);
//     pe.default_collider(new rigid_body_collider(0.1, 0.75));

//     /* Lights */
//     se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0,  10.0, -100.0));
//     se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0, -20.0, -100.0));

//     /* Add moving objects */
//     se.add_moving_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t(0.0, -4.5, 0.0), 10.0));
//     se.add_moving_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t(0.0,  0.5, 0.0), 20.0));
//     se.add_moving_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t(0.0,  5.5, 0.0), 10.0));
//     se.add_moving_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t(0.0, 10.5, 0.0), 20.0));

//     /* Add static objects */
//     se.add_object(make_plane(m, point_t(-10.0, 0.0, -10.0), point_t(10.0, 0.0, -10.0), point_t(-10.0, 0.0, 10.0), point_t(10.0, 0.0, 10.0), point_t(0.0, -10.0, 0.0), std::numeric_limits<float>::infinity()));

//     /* Run physics simulation */
//     run(&checker, 450);
// }


// BOOST_AUTO_TEST_CASE( four_straight_stack_60_fps_cor_0_5_test )
// {
//     /* Checker */
//     CREATE_REGRESSION_CHECKER(checker);

//     /* Enviroment set up */
//     po.max_timestep(1.0 / 60.0);
//     po.min_timestep(1.0 / 60.0);
//     pe.default_collider(new rigid_body_collider(0.5, 0.75));

//     /* Lights */
//     se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0,  10.0, -100.0));
//     se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0, -20.0, -100.0));

//     /* Add moving objects */
//     se.add_moving_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t(0.0, -4.5, 0.0), 10.0));
//     se.add_moving_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t(0.0,  0.5, 0.0), 20.0));
//     se.add_moving_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t(0.0,  5.5, 0.0), 10.0));
//     se.add_moving_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t(0.0, 10.5, 0.0), 20.0));

//     /* Add static objects */
//     se.add_object(make_plane(m, point_t(-10.0, 0.0, -10.0), point_t(10.0, 0.0, -10.0), point_t(-10.0, 0.0, 10.0), point_t(10.0, 0.0, 10.0), point_t(0.0, -10.0, 0.0), std::numeric_limits<float>::infinity()));

//     /* Run physics simulation */
//     run(&checker, 450);
// }


// BOOST_AUTO_TEST_CASE( four_straight_stack_60_fps_cor_0_9_test )
// {
//     /* Checker */
//     CREATE_REGRESSION_CHECKER(checker);

//     /* Enviroment set up */
//     po.max_timestep(1.0 / 60.0);
//     po.min_timestep(1.0 / 60.0);
//     pe.default_collider(new rigid_body_collider(0.9, 0.75));

//     /* Lights */
//     se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0,  10.0, -100.0));
//     se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0, -20.0, -100.0));

//     /* Add moving objects */
//     se.add_moving_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t(0.0, -4.5, 0.0), 10.0));
//     se.add_moving_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t(0.0,  0.5, 0.0), 20.0));
//     se.add_moving_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t(0.0,  5.5, 0.0), 10.0));
//     se.add_moving_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t(0.0, 10.5, 0.0), 20.0));

//     /* Add static objects */
//     se.add_object(make_plane(m, point_t(-10.0, 0.0, -10.0), point_t(10.0, 0.0, -10.0), point_t(-10.0, 0.0, 10.0), point_t(10.0, 0.0, 10.0), point_t(0.0, -10.0, 0.0), std::numeric_limits<float>::infinity()));

//     /* Run physics simulation */
//     run(&checker, 800);
// }


/* Below here requires rotation to work properly */
// BOOST_AUTO_TEST_CASE( four_y_stack_25_fps_cor_0_5_test )
// {
//     /* Checker */
//     CREATE_REGRESSION_CHECKER(checker);

//     /* Enviroment set up */
//     po.max_timestep(0.04);
//     po.min_timestep(0.04);
//     pe.default_collider(new rigid_body_collider(0.5, 0.75));

//     /* Lights */
//     se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10,  10, -100));
//     se.add_light(raptor_raytracer::ext_colour_t(255, 255, 255), point_t(10, -20, -100));

//     /* Add moving objects */
//     se.add_moving_object(make_cube(m, point_t(0.0, -5.0, -0.5), point_t(1.0, -4.0, 0.5), 10.0));
//     se.add_moving_object(make_cube(m, point_t(-4.0, 5.0, -0.5), point_t(5.0, 6.0, 0.5), 20.0));
//     se.add_moving_object(make_cube(m, point_t(-4.0, 10.0, -0.5), point_t(-3.0, 11.0, 0.5), 10.0));
//     se.add_moving_object(make_cube(m, point_t(4.0, 10.0, -0.5), point_t(5.0, 11.0, 0.5), 10.0));

//     /* Add static objects */
//     se.add_object(make_plane(m, point_t(-10.0, 0.0, -10.0), point_t(10.0, 0.0, -10.0), point_t(-10.0, 0.0, 10.0), point_t(10.0, 0.0, 10.0), point_t(0.0, -10.0, 0.0), std::numeric_limits<float>::infinity()));

//     /* Run physics simulation */
    // run(&checker, 400);
// }


// BOOST_AUTO_TEST_CASE( seven_y_stack_25_fps_cor_0_5_test )
// {
//     /* Checker */
//     CREATE_REGRESSION_CHECKER(checker);

//     /* Enviroment set up */
//     po.max_timestep(0.04);
//     po.min_timestep(0.04);
//     pe.default_collider(new rigid_body_collider(0.5, 0.75));

//     /* Lights */
//     se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10,  10, -100));
//     se.add_light(raptor_raytracer::ext_colour_t(255, 255, 255), point_t(10, -20, -100));

//     /* Add moving objects */
//     se.add_moving_object(make_cube(m, point_t(0.0, -5.0, -0.5), point_t(1.0, -4.0, 0.5), 10.0));
//     se.add_moving_object(make_cube(m, point_t(0.0, 0.0, -0.5), point_t(1.0, 1.0, 0.5), 10.0));
//     se.add_moving_object(make_cube(m, point_t(-4.0, 5.0, -0.5), point_t(5.0, 6.0, 0.5), 20.0));
//     se.add_moving_object(make_cube(m, point_t(-4.0, 10.0, -0.5), point_t(-3.0, 11.0, 0.5), 10.0));
//     se.add_moving_object(make_cube(m, point_t(4.0, 10.0, -0.5), point_t(5.0, 11.0, 0.5), 10.0));
//     se.add_moving_object(make_cube(m, point_t(-4.0, 15.0, -0.5), point_t(-3.0, 16.0, 0.5), 10.0));
//     se.add_moving_object(make_cube(m, point_t(4.0, 15.0, -0.5), point_t(5.0, 16.0, 0.5), 10.0));

//     /* Add static objects */
//     se.add_object(make_plane(m, point_t(-10.0, 0.0, -10.0), point_t(10.0, 0.0, -10.0), point_t(-10.0, 0.0, 10.0), point_t(10.0, 0.0, 10.0), point_t(0.0, -10.0, 0.0), std::numeric_limits<float>::infinity()));

//     /* Run physics simulation */
    // run(&checker, 400);
// }


// BOOST_AUTO_TEST_CASE( four_bridge_stack_25_fps_cor_0_5_test )
// {
//     /* Checker */
//     CREATE_REGRESSION_CHECKER(checker);

//     /* Enviroment set up */
//     po.max_timestep(0.04);
//     po.min_timestep(0.04);
//     pe.default_collider(new rigid_body_collider(0.5, 0.75));

//     /* Lights */
//     se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10,  10, -100));
//     se.add_light(raptor_raytracer::ext_colour_t(255, 255, 255), point_t(10, -20, -100));

//     /* Add moving objects */
//     se.add_moving_object(make_cube(m, point_t(-5.0, -5.0, -0.5), point_t(-4.0, -3.0, 0.5), 10.0));
//     se.add_moving_object(make_cube(m, point_t(4.0, -5.0, -0.5), point_t(5.0, -4.0, 0.5), 10.0));
//     se.add_moving_object(make_cube(m, point_t(4.0, 0.0, -0.5), point_t(5.0, 1.0, 0.5), 10.0));
//     se.add_moving_object(make_cube(m, point_t(-5.0, 5.0, -0.5), point_t(5.0, 6.0, 0.5), 10.0));

//     /* Add static objects */
//     se.add_object(make_plane(m, point_t(-10.0, 0.0, -10.0), point_t(10.0, 0.0, -10.0), point_t(-10.0, 0.0, 10.0), point_t(10.0, 0.0, 10.0), point_t(0.0, -10.0, 0.0), std::numeric_limits<float>::infinity()));

//     /* Run physics simulation */
    // run(&checker, 400);
// }


// BOOST_AUTO_TEST_CASE( eight_double_bridge_stack_25_fps_cor_0_5_test )
// {
//     /* Checker */
//     CREATE_REGRESSION_CHECKER(checker);

//     /* Enviroment set up */
//     po.max_timestep(0.04);
//     po.min_timestep(0.04);
//     pe.default_collider(new rigid_body_collider(0.5, 0.75));

//     /* Lights */
//     se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10,  10, -100));
//     se.add_light(raptor_raytracer::ext_colour_t(255, 255, 255), point_t(10, -20, -100));

//     /* Add moving objects */
//     se.add_moving_object(make_cube(m, point_t(-5.0, -5.0, -0.5), point_t(-4.0, -3.0, 0.5), 10.0));
//     se.add_moving_object(make_cube(m, point_t(4.0, -5.0, -0.5), point_t(5.0, -4.0, 0.5), 10.0));
//     se.add_moving_object(make_cube(m, point_t(4.0, 0.0, -0.5), point_t(5.0, 1.0, 0.5), 10.0));
//     se.add_moving_object(make_cube(m, point_t(-5.0, 5.0, -0.5), point_t(5.0, 6.0, 0.5), 10.0));
//     se.add_moving_object(make_cube(m, point_t(-5.0, 10.0, -0.5), point_t(-4.0, 12.0, 0.5), 10.0));
//     se.add_moving_object(make_cube(m, point_t(4.0, 10.0, -0.5), point_t(5.0, 11.0, 0.5), 10.0));
//     se.add_moving_object(make_cube(m, point_t(4.0, 15.0, -0.5), point_t(5.0, 16.0, 0.5), 10.0));
//     se.add_moving_object(make_cube(m, point_t(-5.0, 20.0, -0.5), point_t(5.0, 21.0, 0.5), 10.0));

//     /* Add static objects */
//     se.add_object(make_plane(m, point_t(-10.0, 0.0, -10.0), point_t(10.0, 0.0, -10.0), point_t(-10.0, 0.0, 10.0), point_t(10.0, 0.0, 10.0), point_t(0.0, -10.0, 0.0), std::numeric_limits<float>::infinity()));

//     /* Run physics simulation */
    // run(&checker, 400);
// }

BOOST_AUTO_TEST_SUITE_END()
}; /* namespace test */
}; /* namespace raptor_physics */
