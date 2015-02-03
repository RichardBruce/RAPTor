#ifdef STAND_ALONE
#define BOOST_TEST_MODULE obj test

/* Common headers */
#include "logging.h"

/* Initialise logger */
const raptor_physics::init_logger init_logger;
#endif /* #ifdef STAND_ALONE */

/* Test headers */
#include "regression_fixture.h"


namespace raptor_raytracer
{
namespace test
{
BOOST_AUTO_TEST_SUITE( obj_tests );

#ifndef VALGRIND_TESTS
BOOST_AUTO_TEST_CASE( obj_isd_imperator_0_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/obj_scenes/isd_imperator/isd_imperator.obj", model_format_t::obj, point_t(0.0, -85000.0, -650000.0), point_t(1.0, 0.0, 0.0), point_t(0.0, 1.0, 0.0), point_t(0.0, 0.0, 1.0), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(500000.0, 200000.0, -700000.0), 0.0, 10.0);

    /* Ray trace the scene using kd tree */
    /* Builder throws assert */
    // fixture.render<kd_tree>();
    // checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( obj_isd_imperator_1_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/obj_scenes/isd_imperator/isd_imperator.obj", model_format_t::obj, point_t(-200000.0, -200000.0, 350000.0), point_t(0.880037, -0.0196184, 0.4745), point_t(-0.139384, 0.944474, 0.29756), point_t(0.45399, 0.328001, -0.828437), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-500000.0, -500000.0, 700000.0), 0.0, 10.0);

    /* Ray trace the scene using kd tree */
    /* Builder throws assert */
    // fixture.render<kd_tree>();
    // checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( obj_isd_imperator_2_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/obj_scenes/isd_imperator/isd_imperator.obj", model_format_t::obj, point_t(500000.0, 250000.0, -650000.0), point_t(-0.639625, 0.000893619, -0.768686), point_t(-0.36828, 0.877402, 0.307467), point_t(-0.674721, -0.479756, 0.56088), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(0.0, 400000.0, -1000000.0), 0.0, 10.0);

    /* Ray trace the scene using kd tree */
    /* Builder throws assert */
    // fixture.render<kd_tree>();
    // checker.check(fixture.get_camera(), "kdt");
    
    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( obj_eg07_dragon_original_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/obj_scenes/phlegmatic_dragon/eg07_dragon_original.obj", model_format_t::obj, point_t(25.0, 300.0, -725.0), point_t(1.0, 0.0, 0.0), point_t(0.0, 0.0, -1.0), point_t(0.0, -1.0, 0.0), ext_colour_t(255.0, 255.0, 255.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t( 500.0, 750.0, 1000.0), 0.0, 10.0);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-500.0, 500.0, -750.0), 0.0, 10.0);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( obj_eg07_dragon_smoothed_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/obj_scenes/phlegmatic_dragon/eg07_dragon_smoothed.obj", model_format_t::obj, point_t(3.33962, -225.0, 152.5), point_t(0.998058, -0.0153803, -0.0603799), point_t(0.0603797, 0.477955, 0.876307), point_t(-0.0153811, 0.878252, -0.477956), ext_colour_t(255.0, 255.0, 255.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t( 250.0, -350.0,  500.0), 0.0, 10.0);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-250.0, -250.0, -350.0), 0.0, 10.0);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( obj_sponza_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/obj_scenes/sponza/sponza.obj", model_format_t::obj, point_t(-13.7643, -6.92507, 1.32953), point_t(-0.105038, -0.00870562, -0.99443), point_t(-0.238403, 0.971023, 0.0166812), point_t(0.965468, 0.238826, -0.104071), ext_colour_t(255.0, 255.0, 255.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-5.0, -5.0, -3.0), 0.0, 0.0001);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-6.0, 10.0,  0.0), 0.0, 0.0001);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}
#endif /* #ifndef VALGRIND_TESTS */

BOOST_AUTO_TEST_SUITE_END()
}; /* namespace raptor_raytracer */
}; /* namespace test */
