#ifdef STAND_ALONE
#define BOOST_TEST_MODULE dropping test

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
BOOST_FIXTURE_TEST_SUITE( dropping_tests, regression_fixture );


BOOST_AUTO_TEST_CASE( no_force_drop_10_fps_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    po.max_timestep(0.1);
    po.min_timestep(0.1);
    pe.pair_collider(new rigid_body_collider(0.1, 0.75), 0, 1);
    pe.pair_collider(new rigid_body_collider(0.5, 0.75), 0, 2);
    pe.pair_collider(new rigid_body_collider(0.9, 0.75), 0, 3);

    /* Lights */
    se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0,  10.0, -100.0));
    se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0, -20.0, -100.0));

    /* Add moving objects */
    se.add_moving_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t(-5.0, -8.5, 0.0), 10.0, 1));
    se.add_moving_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t( 0.0, -8.5, 0.0), 10.0, 2));
    se.add_moving_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t( 5.0, -8.5, 0.0), 10.0, 3));

    /* Add static objects */
    se.add_object(make_plane(m, point_t(-10.0, 0.0, -10.0), point_t(10.0, 0.0, -10.0), point_t(-10.0, 0.0, 10.0), point_t(10.0, 0.0, 10.0), point_t(0.0, -10.0, 0.0), std::numeric_limits<float>::infinity()));

    /* Run physics simulation */
    run(&checker, 150);
}


BOOST_AUTO_TEST_CASE( no_force_drop_25_fps_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    po.max_timestep(0.04);
    po.min_timestep(0.04);
    pe.pair_collider(new rigid_body_collider(0.1, 0.75), 0, 1);
    pe.pair_collider(new rigid_body_collider(0.5, 0.75), 0, 2);
    pe.pair_collider(new rigid_body_collider(0.9, 0.75), 0, 3);

    /* Lights */
    se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0,  10.0, -100.0));
    se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0, -20.0, -100.0));

    /* Add moving objects */
    se.add_moving_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t(-5.0, -8.5, 0.0), 10.0, 1));
    se.add_moving_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t( 0.0, -8.5, 0.0), 10.0, 2));
    se.add_moving_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t( 5.0, -8.5, 0.0), 10.0, 3));

    /* Add static objects */
    se.add_object(make_plane(m, point_t(-10.0, 0.0, -10.0), point_t(10.0, 0.0, -10.0), point_t(-10.0, 0.0, 10.0), point_t(10.0, 0.0, 10.0), point_t(0.0, -10.0, 0.0), std::numeric_limits<float>::infinity()));

    /* Run physics simulation */
    run(&checker, 300);
}


BOOST_AUTO_TEST_CASE( no_force_drop_60_fps_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    po.max_timestep(1.0 / 60.0);
    po.min_timestep(1.0 / 60.0);
    pe.pair_collider(new rigid_body_collider(0.1, 0.75), 0, 1);
    pe.pair_collider(new rigid_body_collider(0.5, 0.75), 0, 2);
    pe.pair_collider(new rigid_body_collider(0.9, 0.75), 0, 3);

    /* Lights */
    se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0,  10.0, -100.0));
    se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0, -20.0, -100.0));

    /* Add moving objects */
    se.add_moving_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t(-5.0, -8.5, 0.0), 10.0, 1));
    se.add_moving_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t( 0.0, -8.5, 0.0), 10.0, 2));
    se.add_moving_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t( 5.0, -8.5, 0.0), 10.0, 3));

    /* Add static objects */
    se.add_object(make_plane(m, point_t(-10.0, 0.0, -10.0), point_t(10.0, 0.0, -10.0), point_t(-10.0, 0.0, 10.0), point_t(10.0, 0.0, 10.0), point_t(0.0, -10.0, 0.0), std::numeric_limits<float>::infinity()));

    /* Run physics simulation */
    run(&checker, 600);
}


BOOST_AUTO_TEST_CASE( up_force_drop_10_fps_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    po.max_timestep(0.1);
    po.min_timestep(0.1);
    pe.pair_collider(new rigid_body_collider(0.1, 0.75), 0, 1);
    pe.pair_collider(new rigid_body_collider(0.5, 0.75), 0, 2);
    pe.pair_collider(new rigid_body_collider(0.9, 0.75), 0, 3);

    /* Lights */
    se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0,  10.0, -100.0));
    se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0, -20.0, -100.0));

    /* Add moving objects */
    se.add_moving_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t(-5.0, -8.5, 0.0), 10.0, 1));
    se.add_moving_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t( 0.0, -8.5, 0.0), 10.0, 2));
    se.add_moving_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t( 5.0, -8.5, 0.0), 10.0, 3));

    /* Add static objects */
    se.add_object(make_plane(m, point_t(-10.0, 0.0, -10.0), point_t(10.0, 0.0, -10.0), point_t(-10.0, 0.0, 10.0), point_t(10.0, 0.0, 10.0), point_t(0.0, -10.0, 0.0), std::numeric_limits<float>::infinity()));
    
    /* Apply forces */
    pe.apply_force(new const_force(point_t(0.0, 0.0, 0.0), point_t(0.0, 200.0, 0.0), 0.5), 0);
    pe.apply_force(new const_force(point_t(0.0, 0.0, 0.0), point_t(0.0, 200.0, 0.0), 0.5), 1);
    pe.apply_force(new const_force(point_t(0.0, 0.0, 0.0), point_t(0.0, 200.0, 0.0), 0.5), 2);

    /* Run physics simulation */
    run(&checker, 250);
}


BOOST_AUTO_TEST_CASE( up_force_drop_25_fps_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    po.max_timestep(0.04);
    po.min_timestep(0.04);
    pe.pair_collider(new rigid_body_collider(0.1, 0.75), 0, 1);
    pe.pair_collider(new rigid_body_collider(0.5, 0.75), 0, 2);
    pe.pair_collider(new rigid_body_collider(0.9, 0.75), 0, 3);

    /* Lights */
    se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0,  10.0, -100.0));
    se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0, -20.0, -100.0));

    /* Add moving objects */
    se.add_moving_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t(-5.0, -8.5, 0.0), 10.0, 1));
    se.add_moving_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t( 0.0, -8.5, 0.0), 10.0, 2));
    se.add_moving_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t( 5.0, -8.5, 0.0), 10.0, 3));

    /* Add static objects */
    se.add_object(make_plane(m, point_t(-10.0, 0.0, -10.0), point_t(10.0, 0.0, -10.0), point_t(-10.0, 0.0, 10.0), point_t(10.0, 0.0, 10.0), point_t(0.0, -10.0, 0.0), std::numeric_limits<float>::infinity()));
    
    /* Apply forces */
    pe.apply_force(new const_force(point_t(0.0, 0.0, 0.0), point_t(0.0, 200.0, 0.0), 0.5), 0);
    pe.apply_force(new const_force(point_t(0.0, 0.0, 0.0), point_t(0.0, 200.0, 0.0), 0.5), 1);
    pe.apply_force(new const_force(point_t(0.0, 0.0, 0.0), point_t(0.0, 200.0, 0.0), 0.5), 2);

    /* Run physics simulation */
    run(&checker, 600);
}


BOOST_AUTO_TEST_CASE( up_force_drop_60_fps_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    po.max_timestep(1.0 / 60.0);
    po.min_timestep(1.0 / 60.0);
    pe.pair_collider(new rigid_body_collider(0.1, 0.75), 0, 1);
    pe.pair_collider(new rigid_body_collider(0.5, 0.75), 0, 2);
    pe.pair_collider(new rigid_body_collider(0.9, 0.75), 0, 3);

    /* Lights */
    se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0,  10.0, -100.0));
    se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0, -20.0, -100.0));

    /* Add moving objects */
    se.add_moving_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t(-5.0, -8.5, 0.0), 10.0, 1));
    se.add_moving_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t( 0.0, -8.5, 0.0), 10.0, 2));
    se.add_moving_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t( 5.0, -8.5, 0.0), 10.0, 3));

    /* Add static objects */
    se.add_object(make_plane(m, point_t(-10.0, 0.0, -10.0), point_t(10.0, 0.0, -10.0), point_t(-10.0, 0.0, 10.0), point_t(10.0, 0.0, 10.0), point_t(0.0, -10.0, 0.0), std::numeric_limits<float>::infinity()));
    
    /* Apply forces */
    pe.apply_force(new const_force(point_t(0.0, 0.0, 0.0), point_t(0.0, 200.0, 0.0), 0.5), 0);
    pe.apply_force(new const_force(point_t(0.0, 0.0, 0.0), point_t(0.0, 200.0, 0.0), 0.5), 1);
    pe.apply_force(new const_force(point_t(0.0, 0.0, 0.0), point_t(0.0, 200.0, 0.0), 0.5), 2);
    
    /* Run physics simulation */
    run(&checker, 1200);
}


BOOST_AUTO_TEST_CASE( down_force_drop_10_fps_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    po.max_timestep(0.1);
    po.min_timestep(0.1);
    pe.pair_collider(new rigid_body_collider(0.1, 0.75), 0, 1);
    pe.pair_collider(new rigid_body_collider(0.5, 0.75), 0, 2);
    pe.pair_collider(new rigid_body_collider(0.9, 0.75), 0, 3);

    /* Lights */
    se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0,  10.0, -100.0));
    se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0, -20.0, -100.0));

    /* Add moving objects */
    se.add_moving_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t(-5.0, -8.5, 0.0), 10.0, 1));
    se.add_moving_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t( 0.0, -8.5, 0.0), 10.0, 2));
    se.add_moving_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t( 5.0, -8.5, 0.0), 10.0, 3));

    /* Add static objects */
    se.add_object(make_plane(m, point_t(-10.0, 0.0, -10.0), point_t(10.0, 0.0, -10.0), point_t(-10.0, 0.0, 10.0), point_t(10.0, 0.0, 10.0), point_t(0.0, -10.0, 0.0), std::numeric_limits<float>::infinity()));
    
    /* Apply forces */
    pe.apply_force(new const_force(point_t(0.0, 0.0, 0.0), point_t(0.0, -200.0, 0.0), 0.5), 0);
    pe.apply_force(new const_force(point_t(0.0, 0.0, 0.0), point_t(0.0, -200.0, 0.0), 0.5), 1);
    pe.apply_force(new const_force(point_t(0.0, 0.0, 0.0), point_t(0.0, -200.0, 0.0), 0.5), 2);

    /* Run physics simulation */
    run(&checker, 150);
}


BOOST_AUTO_TEST_CASE( down_force_drop_25_fps_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    po.max_timestep(0.04);
    po.min_timestep(0.04);
    pe.pair_collider(new rigid_body_collider(0.1, 0.75), 0, 1);
    pe.pair_collider(new rigid_body_collider(0.5, 0.75), 0, 2);
    pe.pair_collider(new rigid_body_collider(0.9, 0.75), 0, 3);

    /* Lights */
    se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0,  10.0, -100.0));
    se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0, -20.0, -100.0));

    /* Add moving objects */
    se.add_moving_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t(-5.0, -8.5, 0.0), 10.0, 1));
    se.add_moving_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t( 0.0, -8.5, 0.0), 10.0, 2));
    se.add_moving_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t( 5.0, -8.5, 0.0), 10.0, 3));

    /* Add static objects */
    se.add_object(make_plane(m, point_t(-10.0, 0.0, -10.0), point_t(10.0, 0.0, -10.0), point_t(-10.0, 0.0, 10.0), point_t(10.0, 0.0, 10.0), point_t(0.0, -10.0, 0.0), std::numeric_limits<float>::infinity()));
    
    /* Apply forces */
    pe.apply_force(new const_force(point_t(0.0, 0.0, 0.0), point_t(0.0, -200.0, 0.0), 0.5), 0);
    pe.apply_force(new const_force(point_t(0.0, 0.0, 0.0), point_t(0.0, -200.0, 0.0), 0.5), 1);
    pe.apply_force(new const_force(point_t(0.0, 0.0, 0.0), point_t(0.0, -200.0, 0.0), 0.5), 2);

    /* Run physics simulation */
    run(&checker, 300);
}


BOOST_AUTO_TEST_CASE( down_force_drop_60_fps_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    po.max_timestep(1.0 / 60.0);
    po.min_timestep(1.0 / 60.0);
    pe.pair_collider(new rigid_body_collider(0.1, 0.75), 0, 1);
    pe.pair_collider(new rigid_body_collider(0.5, 0.75), 0, 2);
    pe.pair_collider(new rigid_body_collider(0.9, 0.75), 0, 3);

    /* Lights */
    se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0,  10.0, -100.0));
    se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0, -20.0, -100.0));

    /* Add moving objects */
    se.add_moving_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t(-5.0, -8.5, 0.0), 10.0, 1));
    se.add_moving_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t( 0.0, -8.5, 0.0), 10.0, 2));
    se.add_moving_object(make_cube(m, point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t( 5.0, -8.5, 0.0), 10.0, 3));

    /* Add static objects */
    se.add_object(make_plane(m, point_t(-10.0, 0.0, -10.0), point_t(10.0, 0.0, -10.0), point_t(-10.0, 0.0, 10.0), point_t(10.0, 0.0, 10.0), point_t(0.0, -10.0, 0.0), std::numeric_limits<float>::infinity()));
    
    /* Apply forces */
    pe.apply_force(new const_force(point_t(0.0, 0.0, 0.0), point_t(0.0, -200.0, 0.0), 0.5), 0);
    pe.apply_force(new const_force(point_t(0.0, 0.0, 0.0), point_t(0.0, -200.0, 0.0), 0.5), 1);
    pe.apply_force(new const_force(point_t(0.0, 0.0, 0.0), point_t(0.0, -200.0, 0.0), 0.5), 2);
    
    /* Run physics simulation */
    run(&checker, 600);
}

// BOOST_AUTO_TEST_CASE( friction_drop_25_fps_cor_0_5_test )
// {
//     /* Checker */
//     CREATE_REGRESSION_CHECKER(checker);

//     /* Enviroment set up */
//     po.max_timestep(0.04);
//     po.min_timestep(0.04);
//     pe.default_collider(new rigid_body_collider(0.5, 0.3));

//     /* Lights */
//     se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0,  10.0, -100.0));
//     se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0, -20.0, -100.0));

//     /* Add moving objects */
//     se.add_moving_object(make_cube(m, quaternion_t(0.9238795325, 0.0, 0.0, 0.3826834324), point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t(0.0, 5.0, 0.0), point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 5.0), 10.0));

//     /* Add static objects */
//     se.add_object(make_plane(m, point_t(-10.0, 0.0, -10.0), point_t(10.0, 0.0, -10.0), point_t(-10.0, 0.0, 10.0), point_t(10.0, 0.0, 10.0), point_t(0.0, -10.0, 0.0), std::numeric_limits<float>::infinity()));

//     /* Run physics simulation */
    // run(&checker, 200);
// }


// // BOOST_AUTO_TEST_CASE( friction_drop_10_fps_cor_0_1_test )
// // {
// //     /* Checker */
// //     CREATE_REGRESSION_CHECKER(checker);

// //     /* Enviroment set up */
// //     po.max_timestep(0.1);
// //     po.min_timestep(0.1);
// //     pe.default_collider(new rigid_body_collider(0.1, 0.75));

// //     /* Lights */
// //     se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0,  10.0, -100.0));
// //     se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0, -20.0, -100.0));

// //     /* Add moving objects */
// //     se.add_moving_object(make_cube(m, quaternion_t(0.9238795325, 0.0, 0.0, 0.3826834324), point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t(-10.0, 5.0, 0.0), point_t(20.0, -10.0, 0.0), point_t(0.0, 0.0, 0.0), 10.0));

// //     /* Add static objects */
// //     se.add_object(make_plane(m, point_t(-10.0, 0.0, -10.0), point_t(10.0, 0.0, -10.0), point_t(-10.0, 0.0, 10.0), point_t(10.0, 0.0, 10.0), point_t(0.0, -10.0, 0.0), std::numeric_limits<float>::infinity()));

// //     /* Run physics simulation */
    // run(&checker, 50);
// // }


// // BOOST_AUTO_TEST_CASE( friction_drop_10_fps_cor_0_5_test )
// // {
// //     /* Checker */
// //     CREATE_REGRESSION_CHECKER(checker);

// //     /* Enviroment set up */
// //     po.max_timestep(0.1);
// //     po.min_timestep(0.1);
// //     pe.default_collider(new rigid_body_collider(0.5, 0.3));

// //     /* Lights */
// //     se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0,  10.0, -100.0));
// //     se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0, -20.0, -100.0));

// //     /* Add moving objects */
// //     se.add_moving_object(make_cube(m, quaternion_t(0.9238795325, 0.0, 0.0, 0.3826834324), point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t(-10.0, 5.0, 0.0), point_t(20.0, -10.0, 0.0), point_t(0.0, 0.0, 0.0), 10.0));

// //     /* Add static objects */
// //     se.add_object(make_plane(m, point_t(-10.0, 0.0, -10.0), point_t(10.0, 0.0, -10.0), point_t(-10.0, 0.0, 10.0), point_t(10.0, 0.0, 10.0), point_t(0.0, -10.0, 0.0), std::numeric_limits<float>::infinity()));

// //     /* Run physics simulation */
// //     /* Check starting state */
// //     checker.check(pe, 0);

// //     /* Run some frames and check */
    // run(&checker, 50);
// // }


// // BOOST_AUTO_TEST_CASE( friction_drop_10_fps_cor_0_9_test )
// // {
// //     /* Checker */
// //     CREATE_REGRESSION_CHECKER(checker);

// //     /* Enviroment set up */
// //     po.max_timestep(0.1);
// //     po.min_timestep(0.1);
// //     pe.default_collider(new rigid_body_collider(0.9, 0.5));

// //     /* Lights */
// //     se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0,  10.0, -100.0));
// //     se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0, -20.0, -100.0));

// //     /* Add moving objects */
// //     se.add_moving_object(make_cube(m, quaternion_t(0.9238795325, 0.0, 0.0, 0.3826834324), point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t(-10.0, 5.0, 0.0), point_t(20.0, -10.0, 0.0), point_t(0.0, 0.0, 0.0), 10.0));

// //     /* Add static objects */
// //     se.add_object(make_plane(m, point_t(-10.0, 0.0, -10.0), point_t(10.0, 0.0, -10.0), point_t(-10.0, 0.0, 10.0), point_t(10.0, 0.0, 10.0), point_t(0.0, -10.0, 0.0), std::numeric_limits<float>::infinity()));

// //     /* Run physics simulation */
// //     /* Check starting state */
// //     checker.check(pe, 0);

// //     /* Run some frames and check */
    // run(&checker, 50);
// // }


// // BOOST_AUTO_TEST_CASE( friction_drop_25_fps_cor_0_1_test )
// // {
// //     /* Checker */
// //     CREATE_REGRESSION_CHECKER(checker);

// //     /* Enviroment set up */
// //     po.max_timestep(0.04);
// //     po.min_timestep(0.04);
// //     pe.default_collider(new rigid_body_collider(0.1, 0.75));

// //     /* Lights */
// //     se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0,  10.0, -100.0));
// //     se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0, -20.0, -100.0));

// //     /* Add moving objects */
// //     se.add_moving_object(make_cube(m, quaternion_t(0.9238795325, 0.0, 0.0, 0.3826834324), point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t(-10.0, 5.0, 0.0), point_t(20.0, -10.0, 0.0), point_t(0.0, 0.0, 0.0), 10.0));

// //     /* Add static objects */
// //     se.add_object(make_plane(m, point_t(-10.0, 0.0, -10.0), point_t(10.0, 0.0, -10.0), point_t(-10.0, 0.0, 10.0), point_t(10.0, 0.0, 10.0), point_t(0.0, -10.0, 0.0), std::numeric_limits<float>::infinity()));

// //     /* Run physics simulation */
    // run(&checker, 100);
// // }


// // BOOST_AUTO_TEST_CASE( friction_drop_25_fps_cor_0_5_test )
// // {
// //     /* Checker */
// //     CREATE_REGRESSION_CHECKER(checker);

// //     /* Enviroment set up */
// //     po.max_timestep(0.04);
// //     po.min_timestep(0.04);
// //     pe.default_collider(new rigid_body_collider(0.5, 0.3));

// //     /* Lights */
// //     se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0,  10.0, -100.0));
// //     se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0, -20.0, -100.0));

// //     /* Add moving objects */
// //     se.add_moving_object(make_cube(m, quaternion_t(0.9238795325, 0.0, 0.0, 0.3826834324), point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t(-10.0, 5.0, 0.0), point_t(20.0, -10.0, 0.0), point_t(0.0, 0.0, 0.0), 10.0));

// //     /* Add static objects */
// //     se.add_object(make_plane(m, point_t(-10.0, 0.0, -10.0), point_t(10.0, 0.0, -10.0), point_t(-10.0, 0.0, 10.0), point_t(10.0, 0.0, 10.0), point_t(0.0, -10.0, 0.0), std::numeric_limits<float>::infinity()));

// //     /* Run physics simulation */
    // run(&checker, 100);
// // }


// // BOOST_AUTO_TEST_CASE( friction_drop_25_fps_cor_0_9_test )
// // {
// //     /* Checker */
// //     CREATE_REGRESSION_CHECKER(checker);

// //     /* Enviroment set up */
// //     po.max_timestep(0.04);
// //     po.min_timestep(0.04);
// //     pe.default_collider(new rigid_body_collider(0.9, 0.5));

// //     /* Lights */
// //     se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0,  10.0, -100.0));
// //     se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0, -20.0, -100.0));

// //     /* Add moving objects */
// //     se.add_moving_object(make_cube(m, quaternion_t(0.9238795325, 0.0, 0.0, 0.3826834324), point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t(-10.0, 5.0, 0.0), point_t(20.0, -10.0, 0.0), point_t(0.0, 0.0, 0.0), 10.0));

// //     /* Add static objects */
// //     se.add_object(make_plane(m, point_t(-10.0, 0.0, -10.0), point_t(10.0, 0.0, -10.0), point_t(-10.0, 0.0, 10.0), point_t(10.0, 0.0, 10.0), point_t(0.0, -10.0, 0.0), std::numeric_limits<float>::infinity()));

// //     /* Run physics simulation */
    // run(&checker, 100);
// // }


// // BOOST_AUTO_TEST_CASE( friction_drop_60_fps_cor_0_1_test )
// // {
// //     /* Checker */
// //     CREATE_REGRESSION_CHECKER(checker);

// //     /* Enviroment set up */
// //     po.max_timestep(1.0 / 60.0);
// //     po.min_timestep(1.0 / 60.0);
// //     pe.default_collider(new rigid_body_collider(0.1, 0.75));

// //     /* Lights */
// //     se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0,  10.0, -100.0));
// //     se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0, -20.0, -100.0));

// //     /* Add moving objects */
// //     se.add_moving_object(make_cube(m, quaternion_t(0.9238795325, 0.0, 0.0, 0.3826834324), point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t(-10.0, 5.0, 0.0), point_t(20.0, -10.0, 0.0), point_t(0.0, 0.0, 0.0), 10.0));

// //     /* Add static objects */
// //     se.add_object(make_plane(m, point_t(-10.0, 0.0, -10.0), point_t(10.0, 0.0, -10.0), point_t(-10.0, 0.0, 10.0), point_t(10.0, 0.0, 10.0), point_t(0.0, -10.0, 0.0), std::numeric_limits<float>::infinity()));

// //     /* Run physics simulation */
    // run(&checker, 200);
// // }


// // BOOST_AUTO_TEST_CASE( friction_drop_60_fps_cor_0_5_test )
// // {
// //     /* Checker */
// //     CREATE_REGRESSION_CHECKER(checker);

// //     /* Enviroment set up */
// //     po.max_timestep(1.0 / 60.0);
// //     po.min_timestep(1.0 / 60.0);
// //     pe.default_collider(new rigid_body_collider(0.5, 0.3));

// //     /* Lights */
// //     se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0,  10.0, -100.0));
// //     se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0, -20.0, -100.0));

// //     /* Add moving objects */
// //     se.add_moving_object(make_cube(m, quaternion_t(0.9238795325, 0.0, 0.0, 0.3826834324), point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t(-10.0, 5.0, 0.0), point_t(20.0, -10.0, 0.0), point_t(0.0, 0.0, 0.0), 10.0));

// //     /* Add static objects */
// //     se.add_object(make_plane(m, point_t(-10.0, 0.0, -10.0), point_t(10.0, 0.0, -10.0), point_t(-10.0, 0.0, 10.0), point_t(10.0, 0.0, 10.0), point_t(0.0, -10.0, 0.0), std::numeric_limits<float>::infinity()));

// //     /* Run physics simulation */
    // run(&checker, 200);
// // }


// // BOOST_AUTO_TEST_CASE( friction_drop_60_fps_cor_0_9_test )
// // {
// //     /* Checker */
// //     CREATE_REGRESSION_CHECKER(checker);

// //     /* Enviroment set up */
// //     po.max_timestep(1.0 / 60.0);
// //     po.min_timestep(1.0 / 60.0);
// //     pe.default_collider(new rigid_body_collider(0.9, 0.5));

// //     /* Lights */
// //     se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0,  10.0, -100.0));
// //     se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0, -20.0, -100.0));

// //     /* Add moving objects */
// //     se.add_moving_object(make_cube(m, quaternion_t(0.9238795325, 0.0, 0.0, 0.3826834324), point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t(-10.0, 5.0, 0.0), point_t(20.0, -10.0, 0.0), point_t(0.0, 0.0, 0.0), 10.0));

// //     /* Add static objects */
// //     se.add_object(make_plane(m, point_t(-10.0, 0.0, -10.0), point_t(10.0, 0.0, -10.0), point_t(-10.0, 0.0, 10.0), point_t(10.0, 0.0, 10.0), point_t(0.0, -10.0, 0.0), std::numeric_limits<float>::infinity()));

// //     /* Run physics simulation */
    // run(&checker, 200);
// // }


// // BOOST_AUTO_TEST_CASE( friction_drop_60_fps_cor_0_9_test )
// // {
// //     /* Checker */
// //     CREATE_REGRESSION_CHECKER(checker);

// //     /* Enviroment set up */
// //     po.max_timestep(1.0 / 60.0);
// //     po.min_timestep(1.0 / 60.0);
// //     pe.default_collider(new rigid_body_collider(0.9, 0.75));

// //     /* Lights */
// //     se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0,  10.0, -100.0));
// //     se.add_light(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), point_t(10.0, -20.0, -100.0));

// //     /* Add moving objects */
// //     se.add_moving_object(make_cube(m, quaternion_t(0.9238795325, 0.0, 0.0, 0.3826834324), point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5), point_t(0.0, 5.0, 0.0), point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 20.0), 10.0));

// //     /* Add static objects */
// //     se.add_object(make_plane(m, point_t(-10.0, 0.0, -10.0), point_t(10.0, 0.0, -10.0), point_t(-10.0, 0.0, 10.0), point_t(10.0, 0.0, 10.0), point_t(0.0, -10.0, 0.0), std::numeric_limits<float>::infinity()));

// //     /* Run physics simulation */
    // run(&checker, 200);
// // }

BOOST_AUTO_TEST_SUITE_END()

// #define SCENE 12

// #if (SCENE == 1)
//     phy_obj = make_cube(m.get(), point_t(-5.0, -10.0, -0.5), point_t(-4.0, -9.0, 0.5), 10.0);
//     pe.add_object(phy_obj);
//     pe.apply_force(point_t(20.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 10.0, 0);

//     phy_obj = make_cube(m.get(), point_t(0.0, -10.0, -0.5), point_t(1.0, -9.0, 0.5), 10.0);
//     pe.add_object(phy_obj);
//     pe.apply_force(point_t(-20.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 9.0, 1);
// #endif

// #if (SCENE == 2)
//     const float half_weld_dist = 0.5 * raptor_physics::WELD_DISTANCE;
//     phy_obj = make_cube(m.get(), point_t(-1.0 - half_weld_dist, -10.0, -0.5), point_t(-half_weld_dist, -9.0, 0.5), 10.0);
//     pe.add_object(phy_obj);
//     pe.apply_force(point_t(-5.1, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 10.0, 0);

//     phy_obj = make_cube(m.get(), point_t(0.0, -10.0, -0.5), point_t(1.0, -9.0, 0.5), 10.0);
//     pe.add_object(phy_obj);
//     pe.apply_force(point_t(-5.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 10.0, 1);
// #endif

// #if (SCENE == 3)
//     phy_obj = make_cube(m.get(), point_t(-5.0, -10.0, -0.5), point_t(-4.0, -9.0, 0.5), 10.0);
//     pe.add_object(phy_obj);
//     pe.apply_force(point_t(40.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 50.0, 0);

//     phy_obj = make_cube(m.get(), point_t(0.0, -10.0, -0.5), point_t(1.0, -9.0, 0.5), 10.0);
//     pe.add_object(phy_obj);
// //    pe.apply_force(point_t(-5.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 10.0, 1);

//     phy_obj = make_cube(m.get(), point_t(4.0, -10.0, -0.5), point_t(5.0, -9.0, 0.5), 10.0);
//     pe.add_object(phy_obj);
// //    pe.apply_force(point_t(-5.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 10.0, 1);
// #endif


// BOOST_AUTO_TEST_CASE( slide_and_hit_25_fps_cor_0_5_test )
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
//     se.add_moving_object(make_cube(m, point_t(-10.0, -5.0, -0.5), point_t(-9.0, -4.0, 0.5), 10.0));
//     se.add_moving_object(make_cube(m, point_t(-5.0, -5.0, -0.5), point_t(-4.0, -4.0, 0.5), 10.0));
//     se.add_moving_object(make_cube(m, point_t(0.0, -5.0, -0.5), point_t(1.0, -4.0, 0.5), 10.0));

//     /* Add static objects */
//     se.add_object(make_plane(m, point_t(-10.0, 0.0, -10.0), point_t(10.0, 0.0, -10.0), point_t(-10.0, 0.0, 10.0), point_t(10.0, 0.0, 10.0), point_t(0.0, -10.0, 0.0), std::numeric_limits<float>::infinity()));

//     /* Apply forces */
//     pe.apply_force(point_t(50.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 100.0, 0);

//     /* Run physics simulation */
//     /* Check starting state */
//     checker.check(pe, 0);

//     /* Run some frames and check */
//     for (int i = 1; i <= 400; ++i)
//     {
//         po.frames_to_run(1);
//         BOOST_CHECK(se.run() == 0);
//         checker.check(pe, i);
//     }
// }

// /* Pathalogically rotating object */
// #if (SCENE == 10)
//     phy_obj = make_cube(m.get(), point_t(-0.1, -6.0, -0.5), point_t(0.1, 4.0, 0.0), 10.0);
//     pe.add_object(phy_obj);
//     phy_obj->set_angular_velocity(point_t(0.0,0.0, PI * -3.0));

//     phy_obj = make_plane(m.get(), point_t(-10.0, -7.0, -10.0), point_t(10.0, -7.0, -10.0), point_t(-10.0, -7.0, 10.0), point_t(10.0, -7.0, 10.0), std::numeric_limits<float>::infinity());
//     pe.add_object(phy_obj);
// #endif

// /* Rotating bouncing object */
// #if (SCENE == 11)
//     phy_obj = make_cube(m.get(), point_t(-0.5, 9.0, -0.5), point_t(0.5, 10.0, 0.5), 10.0);
//     pe.add_object(phy_obj);
//     moving_objs.push_back(phy_obj);
//     point_t at(0.0, 0.0, 0.0);
// //    at.x -= 0.5;
//     at.y += 0.5;
// //    at.z += 0.5;
//     pe.apply_force(point_t(10.0, 0.0, 0.0), at, 0.5, 0);

//     phy_obj = make_plane(m.get(), point_t(-10.0, -10.0, -10.0), point_t(10.0, -10.0, -10.0), point_t(-10.0, -10.0, 10.0), point_t(10.0, -10.0, 10.0), std::numeric_limits<float>::infinity());
//     pe.add_object(phy_obj);
// #endif

// #if (SCENE == 12)
//     phy_obj = make_cube(m.get(), point_t(-0.5, 9.0, -0.5), point_t(0.5, 10.0, 0.5), 10.0);
//     pe.add_object(phy_obj);
//     moving_objs.push_back(phy_obj);
//     point_t at(0.0, 0.0, 0.0);
// //    at.x -= 0.5;
//     at.y += 0.5;
// //    at.z += 0.5;
//     pe.apply_force(point_t(10.0, 0.0, 0.0), at, 0.5, 0);

//     phy_obj = make_plane(m.get(), point_t(-10.0, -10.0, -10.0), point_t(10.0, -10.0, -10.0), point_t(-10.0, -10.0, 10.0), point_t(10.0, -10.0, 10.0), std::numeric_limits<float>::infinity());
//     pe.add_object(phy_obj);
// #endif
}; /* namespace test */
}; /* namespace raptor_physics */
