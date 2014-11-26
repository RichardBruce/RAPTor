#ifdef STAND_ALONE
#define BOOST_TEST_MODULE free_motion test

/* Common headers */
#include "logging.h"

/* Initialise logger */
const raptor_physics::init_logger init_logger;
#endif /* #ifdef STAND_ALONE */

/* Test headers */
#include "regression_fixture.h"


BOOST_FIXTURE_TEST_SUITE( free_motion_tests, regression_fixture );

BOOST_AUTO_TEST_CASE( sin_500_n_10_fps_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    po.max_timestep(0.1);
    po.min_timestep(0.1);

    /* Lights */
    se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0,  10.0, -100.0));
    se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0, -20.0, -100.0));

    /* Add moving objects */
    se.add_moving_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t(-5.0, -15.0, 0.0), 10.0), false);
    pe.apply_force(new sin_force(point_t(0.0, 0.0, 0.0), point_t(0.0, 500.0, 0.0), point_t(0.0, 0.0, 0.0), 0.1 * PI, 0.5 * PI, 20.0), 0);
    pe.apply_force(new const_force(point_t(0.0, 0.0, 0.25), point_t(5.0, 0.0, 0.0), 20.0), 0);

    /* Run physics simulation */
    run(&checker, 250);
}

BOOST_AUTO_TEST_CASE( sin_500_n_25_fps_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    po.max_timestep(0.04);
    po.min_timestep(0.04);

    /* Lights */
    se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0,  10.0, -100.0));
    se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0, -20.0, -100.0));

    /* Add moving objects */
    se.add_moving_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t(-5.0, -15.0, 0.0), 10.0), false);
    pe.apply_force(new sin_force(point_t(0.0, 0.0, 0.0), point_t(0.0, 500.0, 0.0), point_t(0.0, 0.0, 0.0), 0.1 * PI, 0.5 * PI, 20.0), 0);
    pe.apply_force(new const_force(point_t(0.0, 0.0, 0.25), point_t(5.0, 0.0, 0.0), 20.0), 0);

    /* Run physics simulation */
    run(&checker, 500);
}


BOOST_AUTO_TEST_CASE( sin_500_n_60_fps_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    po.max_timestep(1.0 / 60.0);
    po.min_timestep(1.0 / 60.0);

    /* Lights */
    se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0,  10.0, -100.0));
    se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0, -20.0, -100.0));

    /* Add moving objects */
    se.add_moving_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t(-5.0, -15.0, 0.0), 10.0), false);
    pe.apply_force(new sin_force(point_t(0.0, 0.0, 0.0), point_t(0.0, 500.0, 0.0), point_t(0.0, 0.0, 0.0), 0.1 * PI, 0.5 * PI, 20.0), 0);
    pe.apply_force(new const_force(point_t(0.0, 0.0, 0.25), point_t(5.0, 0.0, 0.0), 20.0), 0);

    /* Run physics simulation */
    run(&checker, 1000);
}


BOOST_AUTO_TEST_CASE( sin_10000_n_10_fps_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    po.max_timestep(0.1);
    po.min_timestep(0.1);

    /* Lights */
    se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0,  10.0, -100.0));
    se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0, -20.0, -100.0));

    /* Add moving objects */
    se.add_moving_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t(-5.0, -15.0, 0.0), 10.0), false);
    pe.apply_force(new sin_force(point_t(0.0, 0.0, 0.0), point_t(0.0, 10000.0, 0.0), point_t(0.0, 0.0, 0.0), 0.5 * PI, 0.5 * PI, 20.0), 0);
    pe.apply_force(new const_force(point_t(0.0, 0.0, 0.25), point_t(5.0, 0.0, 0.0), 20.0), 0);

    /* Run physics simulation */
    run(&checker, 250);
}


BOOST_AUTO_TEST_CASE( sin_10000_n_25_fps_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    po.max_timestep(0.04);
    po.min_timestep(0.04);

    /* Lights */
    se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0,  10.0, -100.0));
    se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0, -20.0, -100.0));

    /* Add moving objects */
    se.add_moving_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t(-5.0, -15.0, 0.0), 10.0), false);
    pe.apply_force(new sin_force(point_t(0.0, 0.0, 0.0), point_t(0.0, 10000.0, 0.0), point_t(0.0, 0.0, 0.0), 0.5 * PI, 0.5 * PI, 20.0), 0);
    pe.apply_force(new const_force(point_t(0.0, 0.0, 0.25), point_t(5.0, 0.0, 0.0), 20.0), 0);

    /* Run physics simulation */
    run(&checker, 500);
}


BOOST_AUTO_TEST_CASE( sin_10000_n_60_fps_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    po.max_timestep(1.0 / 60.0);
    po.min_timestep(1.0 / 60.0);

    /* Lights */
    se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0,  10.0, -100.0));
    se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0, -20.0, -100.0));

    /* Add moving objects */
    se.add_moving_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t(-5.0, -15.0, 0.0), 10.0), false);
    pe.apply_force(new sin_force(point_t(0.0, 0.0, 0.0), point_t(0.0, 10000.0, 0.0), point_t(0.0, 0.0, 0.0), 0.5 * PI, 0.5 * PI, 20.0), 0);
    pe.apply_force(new const_force(point_t(0.0, 0.0, 0.25), point_t(5.0, 0.0, 0.0), 20.0), 0);

    /* Run physics simulation */
    run(&checker, 1000);
}


BOOST_AUTO_TEST_CASE( viscous_10_fps_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    po.max_timestep(0.1);
    po.min_timestep(0.1);

    /* Lights */
    se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0,  10.0, -100.0));
    se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0, -20.0, -100.0));

    /* Add moving objects */
    se.add_moving_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t(-10.0, 5.0, 0.0), point_t(150.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 10.0), false);
    pe.apply_force(new viscous_force(point_t(0.0, 0.0, 0.0), 50.0, numeric_limits<float>::infinity()), 0);

    /* Run physics simulation */
    run(&checker, 150);
}


BOOST_AUTO_TEST_CASE( viscous_25_fps_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    po.max_timestep(0.04);
    po.min_timestep(0.04);

    /* Lights */
    se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0,  10.0, -100.0));
    se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0, -20.0, -100.0));

    /* Add moving objects */
    se.add_moving_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t(-10.0, 5.0, 0.0), point_t(150.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 10.0), false);
    pe.apply_force(new viscous_force(point_t(0.0, 0.0, 0.0), 50.0, numeric_limits<float>::infinity()), 0);

    /* Run physics simulation */
    run(&checker, 250);
}


BOOST_AUTO_TEST_CASE( viscous_60_fps_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    po.max_timestep(1.0 / 60.0);
    po.min_timestep(1.0 / 60.0);

    /* Lights */
    se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0,  10.0, -100.0));
    se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0, -20.0, -100.0));

    /* Add moving objects */
    se.add_moving_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t(-10.0, 5.0, 0.0), point_t(150.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 10.0), false);
    pe.apply_force(new viscous_force(point_t(0.0, 0.0, 0.0), 50.0, numeric_limits<float>::infinity()), 0);

    /* Run physics simulation */
    run(&checker, 400);
}


BOOST_AUTO_TEST_CASE( attract_10_fps_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    po.max_timestep(0.1);
    po.min_timestep(0.1);

    /* Lights */
    se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0,  10.0, -100.0));
    se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0, -20.0, -100.0));

    /* Add moving objects */
    /* Sun */
    se.add_moving_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t(0.0, 0.0, 0.0), 10.0), false);
    
    /* Planet */
    se.add_moving_object(make_cube(m, point_t(-0.1, -0.1, -0.1), point_t(0.1, 0.1, 0.1), point_t(0.0, -15.0, 0.0), point_t(10.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1250.0), false);
    pe.apply_force(new attract_force(point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1000.0, 0.0, numeric_limits<float>::infinity()), 1);

    se.add_moving_object(make_cube(m, point_t(-0.1, -0.1, -0.1), point_t(0.1, 0.1, 0.1), point_t(0.0, -5.0, 0.0), point_t(15.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1250.0), false);
    pe.apply_force(new attract_force(point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1000.0, 0.0, numeric_limits<float>::infinity()), 2);

    se.add_moving_object(make_cube(m, point_t(-0.1, -0.1, -0.1), point_t(0.1, 0.1, 0.1), point_t(0.0, -25.0, 0.0), point_t(5.0, 10.0, 0.0), point_t(0.0, 0.0, 0.0), 250.0), false);
    pe.apply_force(new attract_force(point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1000.0, 0.0, numeric_limits<float>::infinity()), 3);

    se.add_moving_object(make_cube(m, point_t(-0.1, -0.1, -0.1), point_t(0.1, 0.1, 0.1), point_t(0.0, 0.0, -15.0), point_t(3.0, 0.0, 15.0), point_t(0.0, 0.0, 0.0), 750.0), false);
    pe.apply_force(new attract_force(point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1000.0, 0.0, numeric_limits<float>::infinity()), 4);

    /* Run physics simulation */
    run(&checker, 500);
}


BOOST_AUTO_TEST_CASE( attract_25_fps_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    po.max_timestep(0.04);
    po.min_timestep(0.04);

    /* Lights */
    se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0,  10.0, -100.0));
    se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0, -20.0, -100.0));

    /* Add moving objects */
    /* Sun */
    se.add_moving_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t(0.0, 0.0, 0.0), 10.0), false);
    
    /* Planet */
    se.add_moving_object(make_cube(m, point_t(-0.1, -0.1, -0.1), point_t(0.1, 0.1, 0.1), point_t(0.0, -15.0, 0.0), point_t(10.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1250.0), false);
    pe.apply_force(new attract_force(point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1000.0, 0.0, numeric_limits<float>::infinity()), 1);

    se.add_moving_object(make_cube(m, point_t(-0.1, -0.1, -0.1), point_t(0.1, 0.1, 0.1), point_t(0.0, -5.0, 0.0), point_t(15.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1250.0), false);
    pe.apply_force(new attract_force(point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1000.0, 0.0, numeric_limits<float>::infinity()), 2);

    se.add_moving_object(make_cube(m, point_t(-0.1, -0.1, -0.1), point_t(0.1, 0.1, 0.1), point_t(0.0, -25.0, 0.0), point_t(5.0, 10.0, 0.0), point_t(0.0, 0.0, 0.0), 250.0), false);
    pe.apply_force(new attract_force(point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1000.0, 0.0, numeric_limits<float>::infinity()), 3);

    se.add_moving_object(make_cube(m, point_t(-0.1, -0.1, -0.1), point_t(0.1, 0.1, 0.1), point_t(0.0, 0.0, -15.0), point_t(3.0, 0.0, 15.0), point_t(0.0, 0.0, 0.0), 750.0), false);
    pe.apply_force(new attract_force(point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1000.0, 0.0, numeric_limits<float>::infinity()), 4);

    /* Run physics simulation */
    run(&checker, 1000);
}


BOOST_AUTO_TEST_CASE( attract_60_fps_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    po.max_timestep(1.0 / 60.0);
    po.min_timestep(1.0 / 60.0);

    /* Lights */
    se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0,  10.0, -100.0));
    se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0, -20.0, -100.0));

    /* Add moving objects */
    /* Sun */
    se.add_moving_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t(0.0, 0.0, 0.0), 10.0), false);
    
    /* Planet */
    se.add_moving_object(make_cube(m, point_t(-0.1, -0.1, -0.1), point_t(0.1, 0.1, 0.1), point_t(0.0, -15.0, 0.0), point_t(10.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1250.0), false);
    pe.apply_force(new attract_force(point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1000.0, 0.0, numeric_limits<float>::infinity()), 1);

    se.add_moving_object(make_cube(m, point_t(-0.1, -0.1, -0.1), point_t(0.1, 0.1, 0.1), point_t(0.0, -5.0, 0.0), point_t(15.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1250.0), false);
    pe.apply_force(new attract_force(point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1000.0, 0.0, numeric_limits<float>::infinity()), 2);

    se.add_moving_object(make_cube(m, point_t(-0.1, -0.1, -0.1), point_t(0.1, 0.1, 0.1), point_t(0.0, -25.0, 0.0), point_t(5.0, 10.0, 0.0), point_t(0.0, 0.0, 0.0), 250.0), false);
    pe.apply_force(new attract_force(point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1000.0, 0.0, numeric_limits<float>::infinity()), 3);

    se.add_moving_object(make_cube(m, point_t(-0.1, -0.1, -0.1), point_t(0.1, 0.1, 0.1), point_t(0.0, 0.0, -15.0), point_t(3.0, 0.0, 15.0), point_t(0.0, 0.0, 0.0), 750.0), false);
    pe.apply_force(new attract_force(point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1000.0, 0.0, numeric_limits<float>::infinity()), 4);


    /* Run physics simulation */
    run(&checker, 1500);
}


BOOST_AUTO_TEST_CASE( repulsive_10_fps_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    po.max_timestep(0.1);
    po.min_timestep(0.1);

    /* Lights */
    se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0,  10.0, -100.0));
    se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0, -20.0, -100.0));

    /* Add moving objects */
    se.add_moving_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t(-0.5,  10.0, 0.0), 10.0), false);
    se.add_moving_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t( 0.5, -10.0, 0.0), 10.0), false);
    
    se.add_moving_object(make_cube(m, point_t(-0.1, -0.1, -0.1), point_t(0.1, 0.1, 0.1), point_t( 0.0,  -1.5,  0.0), point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1250.0), false);
    se.add_moving_object(make_cube(m, point_t(-0.1, -0.1, -0.1), point_t(0.1, 0.1, 0.1), point_t(-1.5,   1.0,  0.0), point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0),  250.0), false);
    se.add_moving_object(make_cube(m, point_t(-0.1, -0.1, -0.1), point_t(0.1, 0.1, 0.1), point_t( 1.5,  -1.0, -1.0), point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0),   50.0), false);
    se.add_moving_object(make_cube(m, point_t(-0.1, -0.1, -0.1), point_t(0.1, 0.1, 0.1), point_t( 0.0,  -7.5,  0.0), point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1250.0), false);
    se.add_moving_object(make_cube(m, point_t(-0.1, -0.1, -0.1), point_t(0.1, 0.1, 0.1), point_t( 0.0, -10.0, -2.0), point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1000.0), false);
    se.add_moving_object(make_cube(m, point_t(-0.1, -0.1, -0.1), point_t(0.1, 0.1, 0.1), point_t( 2.0,  -1.5,  1.0), point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0),  500.0), false);
    se.add_moving_object(make_cube(m, point_t(-0.1, -0.1, -0.1), point_t(0.1, 0.1, 0.1), point_t( 0.5,   1.5, -1.0), point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0),  250.0), false);
    se.add_moving_object(make_cube(m, point_t(-0.1, -0.1, -0.1), point_t(0.1, 0.1, 0.1), point_t( 0.0,  10.5, -2.0), point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0),  500.0), false);
    se.add_moving_object(make_cube(m, point_t(-0.1, -0.1, -0.1), point_t(0.1, 0.1, 0.1), point_t( 0.0,   0.0, -2.0), point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0),   50.0), false);
    se.add_moving_object(make_cube(m, point_t(-0.1, -0.1, -0.1), point_t(0.1, 0.1, 0.1), point_t( 0.0,   7.0, -0.5), point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0),  250.0), false);
    se.add_moving_object(make_cube(m, point_t(-0.1, -0.1, -0.1), point_t(0.1, 0.1, 0.1), point_t( 0.5,   7.5,  0.0), point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1000.0), false);
    se.add_moving_object(make_cube(m, point_t(-0.1, -0.1, -0.1), point_t(0.1, 0.1, 0.1), point_t( 5.0,   0.0,  0.0), point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0),  250.0), false);

    /* Run physics simulation */
    run(&checker, 110, 10);

    /* Apply forces */
    for (int i = 2; i < 14; ++i)
    {
        pe.apply_force(new repel_force(point_t(0.0, 0.0, 0.0), point_t(-0.5,  10.0, 0.0), 750.0, 0.0, numeric_limits<float>::infinity()), i);
        pe.apply_force(new repel_force(point_t(0.0, 0.0, 0.0), point_t( 0.5, -10.0, 0.0), 1000.0, 0.0, numeric_limits<float>::infinity()), i);
        pe.apply_force(new viscous_force(point_t(0.0, 0.0, 0.0), 10.0, numeric_limits<float>::infinity()), i);
    }

    /* Run some frames and check */
    run(&checker, 100, false);
}


BOOST_AUTO_TEST_CASE( repulsive_25_fps_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    po.max_timestep(0.04);
    po.min_timestep(0.04);

    /* Lights */
    se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0,  10.0, -100.0));
    se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0, -20.0, -100.0));

    /* Add moving objects */
    se.add_moving_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t(-0.5,  10.0, 0.0), 10.0), false);
    se.add_moving_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t( 0.5, -10.0, 0.0), 10.0), false);
    
    se.add_moving_object(make_cube(m, point_t(-0.1, -0.1, -0.1), point_t(0.1, 0.1, 0.1), point_t( 0.0,  -1.5,  0.0), point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1250.0), false);
    se.add_moving_object(make_cube(m, point_t(-0.1, -0.1, -0.1), point_t(0.1, 0.1, 0.1), point_t(-1.5,   1.0,  0.0), point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0),  250.0), false);
    se.add_moving_object(make_cube(m, point_t(-0.1, -0.1, -0.1), point_t(0.1, 0.1, 0.1), point_t( 1.5,  -1.0, -1.0), point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0),   50.0), false);
    se.add_moving_object(make_cube(m, point_t(-0.1, -0.1, -0.1), point_t(0.1, 0.1, 0.1), point_t( 0.0,  -7.5,  0.0), point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1250.0), false);
    se.add_moving_object(make_cube(m, point_t(-0.1, -0.1, -0.1), point_t(0.1, 0.1, 0.1), point_t( 0.0, -10.0, -2.0), point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1000.0), false);
    se.add_moving_object(make_cube(m, point_t(-0.1, -0.1, -0.1), point_t(0.1, 0.1, 0.1), point_t( 2.0,  -1.5,  1.0), point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0),  500.0), false);
    se.add_moving_object(make_cube(m, point_t(-0.1, -0.1, -0.1), point_t(0.1, 0.1, 0.1), point_t( 0.5,   1.5, -1.0), point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0),  250.0), false);
    se.add_moving_object(make_cube(m, point_t(-0.1, -0.1, -0.1), point_t(0.1, 0.1, 0.1), point_t( 0.0,  10.5, -2.0), point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0),  500.0), false);
    se.add_moving_object(make_cube(m, point_t(-0.1, -0.1, -0.1), point_t(0.1, 0.1, 0.1), point_t( 0.0,   0.0, -2.0), point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0),   50.0), false);
    se.add_moving_object(make_cube(m, point_t(-0.1, -0.1, -0.1), point_t(0.1, 0.1, 0.1), point_t( 0.0,   7.0, -0.5), point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0),  250.0), false);
    se.add_moving_object(make_cube(m, point_t(-0.1, -0.1, -0.1), point_t(0.1, 0.1, 0.1), point_t( 0.5,   7.5,  0.0), point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1000.0), false);
    se.add_moving_object(make_cube(m, point_t(-0.1, -0.1, -0.1), point_t(0.1, 0.1, 0.1), point_t( 5.0,   0.0,  0.0), point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0),  250.0), false);

    /* Run physics simulation */
    run(&checker, 275, 25);

    /* Apply forces */
    for (int i = 2; i < 14; ++i)
    {
        pe.apply_force(new repel_force(point_t(0.0, 0.0, 0.0), point_t(-0.5,  10.0, 0.0), 750.0, 0.0, numeric_limits<float>::infinity()), i);
        pe.apply_force(new repel_force(point_t(0.0, 0.0, 0.0), point_t( 0.5, -10.0, 0.0), 1000.0, 0.0, numeric_limits<float>::infinity()), i);
        pe.apply_force(new viscous_force(point_t(0.0, 0.0, 0.0), 10.0, numeric_limits<float>::infinity()), i);
    }

    /* Run some frames and check */
    run(&checker, 250, false);
}


BOOST_AUTO_TEST_CASE( repulsive_60_fps_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    po.max_timestep(1.0 / 60.0);
    po.min_timestep(1.0 / 60.0);

    /* Lights */
    se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0,  10.0, -100.0));
    se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0, -20.0, -100.0));

    /* Add moving objects */
    se.add_moving_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t(-0.5,  10.0, 0.0), 10.0), false);
    se.add_moving_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t( 0.5, -10.0, 0.0), 10.0), false);
    
    se.add_moving_object(make_cube(m, point_t(-0.1, -0.1, -0.1), point_t(0.1, 0.1, 0.1), point_t( 0.0,  -1.5,  0.0), point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1250.0), false);
    se.add_moving_object(make_cube(m, point_t(-0.1, -0.1, -0.1), point_t(0.1, 0.1, 0.1), point_t(-1.5,   1.0,  0.0), point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0),  250.0), false);
    se.add_moving_object(make_cube(m, point_t(-0.1, -0.1, -0.1), point_t(0.1, 0.1, 0.1), point_t( 1.5,  -1.0, -1.0), point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0),   50.0), false);
    se.add_moving_object(make_cube(m, point_t(-0.1, -0.1, -0.1), point_t(0.1, 0.1, 0.1), point_t( 0.0,  -7.5,  0.0), point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1250.0), false);
    se.add_moving_object(make_cube(m, point_t(-0.1, -0.1, -0.1), point_t(0.1, 0.1, 0.1), point_t( 0.0, -10.0, -2.0), point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1000.0), false);
    se.add_moving_object(make_cube(m, point_t(-0.1, -0.1, -0.1), point_t(0.1, 0.1, 0.1), point_t( 2.0,  -1.5,  1.0), point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0),  500.0), false);
    se.add_moving_object(make_cube(m, point_t(-0.1, -0.1, -0.1), point_t(0.1, 0.1, 0.1), point_t( 0.5,   1.5, -1.0), point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0),  250.0), false);
    se.add_moving_object(make_cube(m, point_t(-0.1, -0.1, -0.1), point_t(0.1, 0.1, 0.1), point_t( 0.0,  10.5, -2.0), point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0),  500.0), false);
    se.add_moving_object(make_cube(m, point_t(-0.1, -0.1, -0.1), point_t(0.1, 0.1, 0.1), point_t( 0.0,   0.0, -2.0), point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0),   50.0), false);
    se.add_moving_object(make_cube(m, point_t(-0.1, -0.1, -0.1), point_t(0.1, 0.1, 0.1), point_t( 0.0,   7.0, -0.5), point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0),  250.0), false);
    se.add_moving_object(make_cube(m, point_t(-0.1, -0.1, -0.1), point_t(0.1, 0.1, 0.1), point_t( 0.5,   7.5,  0.0), point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1000.0), false);
    se.add_moving_object(make_cube(m, point_t(-0.1, -0.1, -0.1), point_t(0.1, 0.1, 0.1), point_t( 5.0,   0.0,  0.0), point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0),  250.0), false);

    /* Run physics simulation */
    run(&checker, 550, 50);

    /* Apply forces */
    for (int i = 2; i < 14; ++i)
    {
        pe.apply_force(new repel_force(point_t(0.0, 0.0, 0.0), point_t(-0.5,  10.0, 0.0), 750.0, 0.0, numeric_limits<float>::infinity()), i);
        pe.apply_force(new repel_force(point_t(0.0, 0.0, 0.0), point_t( 0.5, -10.0, 0.0), 1000.0, 0.0, numeric_limits<float>::infinity()), i);
        pe.apply_force(new viscous_force(point_t(0.0, 0.0, 0.0), 10.0, numeric_limits<float>::infinity()), i);
    }

    /* Run some frames and check */
    run(&checker, 500, false);
}


BOOST_AUTO_TEST_CASE( repulsive_drop_10_fps_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    po.max_timestep(0.1);
    po.min_timestep(0.1);

    /* Lights */
    se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0,  10.0, -100.0));
    se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0, -20.0, -100.0));

    /* Add repeler */
    se.add_moving_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t(0.0, -15.0, 0.0), 10.0), false);
    
    /* Add moving objects */
    se.add_moving_object(make_cube(m, point_t(-0.1, -0.1, -0.1), point_t(0.1, 0.1, 0.1), point_t(  0.0,  10.0,  0.0), point_t(  0.0, 0.0,  0.0), point_t(0.0, 1.0, 1.0), 1250.0));
    se.add_moving_object(make_cube(m, point_t(-0.1, -0.1, -0.1), point_t(0.1, 0.1, 0.1), point_t(  1.0,  10.0,  0.0), point_t(  0.0, 0.0,  0.0), point_t(0.0, 1.0, 0.0), 2000.0));
    se.add_moving_object(make_cube(m, point_t(-0.1, -0.1, -0.1), point_t(0.1, 0.1, 0.1), point_t( -1.0,  10.0,  0.0), point_t(  0.0, 0.0,  0.0), point_t(1.0, 0.0, 0.0), 1500.0));
    se.add_moving_object(make_cube(m, point_t(-0.1, -0.1, -0.1), point_t(0.1, 0.1, 0.1), point_t(-15.0,   7.0,  0.0), point_t(  8.0, 0.0,  0.0), point_t(0.0, 0.0, 1.0), 1250.0));
    se.add_moving_object(make_cube(m, point_t(-0.1, -0.1, -0.1), point_t(0.1, 0.1, 0.1), point_t(  0.0,   5.0, 12.0), point_t(  0.0, 0.0, -7.0), point_t(0.0, 9.0, 7.0), 1250.0));
    se.add_moving_object(make_cube(m, point_t(-0.1, -0.1, -0.1), point_t(0.1, 0.1, 0.1), point_t( 10.0, -14.0,  0.0), point_t(-45.0, 0.0,  0.0), point_t(0.0, 1.0, 1.0), 1250.0));

    /* Apply forces */
    for (int i = 1; i < 7; ++i)
    {
        pe.apply_force(new repel_force(point_t(0.0, 0.0, 0.0), point_t(0.0, -15.0, 0.0), 1000.0, 0.0, numeric_limits<float>::infinity()), i);
    }

    /* Run physics simulation */
    run(&checker, 100);
}


BOOST_AUTO_TEST_CASE( repulsive_drop_25_fps_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    po.max_timestep(0.04);
    po.min_timestep(0.04);

    /* Lights */
    se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0,  10.0, -100.0));
    se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0, -20.0, -100.0));

    /* Add repeler */
    se.add_moving_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t(0.0, -15.0, 0.0), 10.0), false);
    
    /* Add moving objects */
    se.add_moving_object(make_cube(m, point_t(-0.1, -0.1, -0.1), point_t(0.1, 0.1, 0.1), point_t(  0.0,  10.0,  0.0), point_t(  0.0, 0.0,  0.0), point_t(0.0, 1.0, 1.0), 1250.0));
    se.add_moving_object(make_cube(m, point_t(-0.1, -0.1, -0.1), point_t(0.1, 0.1, 0.1), point_t(  1.0,  10.0,  0.0), point_t(  0.0, 0.0,  0.0), point_t(0.0, 1.0, 0.0), 2000.0));
    se.add_moving_object(make_cube(m, point_t(-0.1, -0.1, -0.1), point_t(0.1, 0.1, 0.1), point_t( -1.0,  10.0,  0.0), point_t(  0.0, 0.0,  0.0), point_t(1.0, 0.0, 0.0), 1500.0));
    se.add_moving_object(make_cube(m, point_t(-0.1, -0.1, -0.1), point_t(0.1, 0.1, 0.1), point_t(-15.0,   7.0,  0.0), point_t(  8.0, 0.0,  0.0), point_t(0.0, 0.0, 1.0), 1250.0));
    se.add_moving_object(make_cube(m, point_t(-0.1, -0.1, -0.1), point_t(0.1, 0.1, 0.1), point_t(  0.0,   5.0, 12.0), point_t(  0.0, 0.0, -7.0), point_t(0.0, 9.0, 7.0), 1250.0));
    se.add_moving_object(make_cube(m, point_t(-0.1, -0.1, -0.1), point_t(0.1, 0.1, 0.1), point_t( 10.0, -14.0,  0.0), point_t(-45.0, 0.0,  0.0), point_t(0.0, 1.0, 1.0), 1250.0));

    /* Apply forces */
    for (int i = 1; i < 7; ++i)
    {
        pe.apply_force(new repel_force(point_t(0.0, 0.0, 0.0), point_t(0.0, -15.0, 0.0), 1000.0, 0.0, numeric_limits<float>::infinity()), i);
    }

    /* Run physics simulation */
    run(&checker, 250);
}


BOOST_AUTO_TEST_CASE( repulsive_drop_60_fps_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    po.max_timestep(1.0 / 60.0);
    po.min_timestep(1.0 / 60.0);

    /* Lights */
    se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0,  10.0, -100.0));
    se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0, -20.0, -100.0));

    /* Add repeler */
    se.add_moving_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t(0.0, -15.0, 0.0), 10.0), false);
    
    /* Add moving objects */
    se.add_moving_object(make_cube(m, point_t(-0.1, -0.1, -0.1), point_t(0.1, 0.1, 0.1), point_t(  0.0,  10.0,  0.0), point_t(  0.0, 0.0,  0.0), point_t(0.0, 1.0, 1.0), 1250.0));
    se.add_moving_object(make_cube(m, point_t(-0.1, -0.1, -0.1), point_t(0.1, 0.1, 0.1), point_t(  1.0,  10.0,  0.0), point_t(  0.0, 0.0,  0.0), point_t(0.0, 1.0, 0.0), 2000.0));
    se.add_moving_object(make_cube(m, point_t(-0.1, -0.1, -0.1), point_t(0.1, 0.1, 0.1), point_t( -1.0,  10.0,  0.0), point_t(  0.0, 0.0,  0.0), point_t(1.0, 0.0, 0.0), 1500.0));
    se.add_moving_object(make_cube(m, point_t(-0.1, -0.1, -0.1), point_t(0.1, 0.1, 0.1), point_t(-15.0,   7.0,  0.0), point_t(  8.0, 0.0,  0.0), point_t(0.0, 0.0, 1.0), 1250.0));
    se.add_moving_object(make_cube(m, point_t(-0.1, -0.1, -0.1), point_t(0.1, 0.1, 0.1), point_t(  0.0,   5.0, 12.0), point_t(  0.0, 0.0, -7.0), point_t(0.0, 9.0, 7.0), 1250.0));
    se.add_moving_object(make_cube(m, point_t(-0.1, -0.1, -0.1), point_t(0.1, 0.1, 0.1), point_t( 10.0, -14.0,  0.0), point_t(-45.0, 0.0,  0.0), point_t(0.0, 1.0, 1.0), 1250.0));

    /* Apply forces */
    for (int i = 1; i < 7; ++i)
    {
        pe.apply_force(new repel_force(point_t(0.0, 0.0, 0.0), point_t(0.0, -15.0, 0.0), 1000.0, 0.0, numeric_limits<float>::infinity()), i);
    }

    /* Run physics simulation */
    run(&checker, 500);
}

BOOST_AUTO_TEST_SUITE_END()
