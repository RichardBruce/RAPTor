#ifdef STAND_ALONE
#define BOOST_TEST_MODULE ply test

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
BOOST_AUTO_TEST_SUITE( ply_tests );

#ifndef VALGRIND_TESTS
BOOST_AUTO_TEST_CASE( ply_bun_zipper_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/ply_scenes/bunny/bun_zipper.ply", model_format_t::ply, point_t(-0.12, 0.29, 0.28), point_t(0.960294, 0.06086, 0.272272), point_t(0.0945048, 0.847262, -0.522701), point_t(  0.262497, -0.527678, -0.807869), ext_colour_t(255.0, 255.0, 255.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(0.0,   1.0, 100.0), 0.0, 10.0);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(0.0, 100.0,   1.0), 0.0, 10.0);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( ply_bun_zipper_res2_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/ply_scenes/bunny/bun_zipper_res2.ply", model_format_t::ply, point_t(-0.12, 0.29, 0.28), point_t(0.960294, 0.06086, 0.272272), point_t(0.0945048, 0.847262, -0.522701), point_t(  0.262497, -0.527678, -0.807869), ext_colour_t(255.0, 255.0, 255.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(0.0,   1.0, 100.0), 0.0, 10.0);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(0.0, 100.0,   1.0), 0.0, 10.0);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( ply_bun_zipper_res3_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/ply_scenes/bunny/bun_zipper_res3.ply", model_format_t::ply, point_t(-0.12, 0.29, 0.28), point_t(0.960294, 0.06086, 0.272272), point_t(0.0945048, 0.847262, -0.522701), point_t(  0.262497, -0.527678, -0.807869), ext_colour_t(255.0, 255.0, 255.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(0.0,   1.0, 100.0), 0.0, 10.0);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(0.0, 100.0,   1.0), 0.0, 10.0);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( ply_bun_zipper_res4_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/ply_scenes/bunny/bun_zipper_res4.ply", model_format_t::ply, point_t(-0.12, 0.29, 0.28), point_t(0.960294, 0.06086, 0.272272), point_t(0.0945048, 0.847262, -0.522701), point_t(  0.262497, -0.527678, -0.807869), ext_colour_t(255.0, 255.0, 255.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(0.0,   1.0, 100.0), 0.0, 10.0);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(0.0, 100.0,   1.0), 0.0, 10.0);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( ply_dragon_vrip_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/ply_scenes/dragon/dragon_vrip.ply", model_format_t::ply, point_t(-0.12, 0.29, 0.25), point_t(0.960294, 0.06086, 0.272272), point_t(0.0945048, 0.847262, -0.522701), point_t(0.262497, -0.527678, -0.807869), ext_colour_t(255.0, 255.0, 255.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(0.0,   1.0, 100.0), 0.0, 10.0);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(0.0, 100.0,   1.0), 0.0, 10.0);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( ply_dragon_vrip_res2_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/ply_scenes/dragon/dragon_vrip_res2.ply", model_format_t::ply, point_t(-0.12, 0.29, 0.25), point_t(0.960294, 0.06086, 0.272272), point_t(0.0945048, 0.847262, -0.522701), point_t(0.262497, -0.527678, -0.807869), ext_colour_t(255.0, 255.0, 255.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(0.0,   1.0, 100.0), 0.0, 10.0);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(0.0, 100.0,   1.0), 0.0, 10.0);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( ply_dragon_vrip_res3_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/ply_scenes/dragon/dragon_vrip_res3.ply", model_format_t::ply, point_t(-0.12, 0.29, 0.25), point_t(0.960294, 0.06086, 0.272272), point_t(0.0945048, 0.847262, -0.522701), point_t(0.262497, -0.527678, -0.807869), ext_colour_t(255.0, 255.0, 255.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(0.0,   1.0, 100.0), 0.0, 10.0);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(0.0, 100.0,   1.0), 0.0, 10.0);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( ply_dragon_vrip_res4_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/ply_scenes/dragon/dragon_vrip_res4.ply", model_format_t::ply, point_t(-0.12, 0.29, 0.25), point_t(0.960294, 0.06086, 0.272272), point_t(0.0945048, 0.847262, -0.522701), point_t(0.262497, -0.527678, -0.807869), ext_colour_t(255.0, 255.0, 255.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(0.0,   1.0, 100.0), 0.0, 10.0);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(0.0, 100.0,   1.0), 0.0, 10.0);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( ply_happy_vrip_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/ply_scenes/happy_budha/happy_vrip.ply", model_format_t::ply, point_t(-0.08, 0.3, 0.2), point_t(0.184456, 0.849229, -0.494759), point_t(0.947139, 0.0191441, -0.320253), point_t(0.262497, -0.527678, -0.807869), ext_colour_t(255.0, 255.0, 255.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(0.0,   1.0, 100.0), 0.0, 10.0);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(0.0, 100.0,   1.0), 0.0, 10.0);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( ply_happy_vrip_res2_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/ply_scenes/happy_budha/happy_vrip_res2.ply", model_format_t::ply, point_t(-0.08, 0.3, 0.2), point_t(0.184456, 0.849229, -0.494759), point_t(0.947139, 0.0191441, -0.320253), point_t(0.262497, -0.527678, -0.807869), ext_colour_t(255.0, 255.0, 255.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(0.0,   1.0, 100.0), 0.0, 10.0);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(0.0, 100.0,   1.0), 0.0, 10.0);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( ply_happy_vrip_res3_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/ply_scenes/happy_budha/happy_vrip_res3.ply", model_format_t::ply, point_t(-0.08, 0.3, 0.2), point_t(0.184456, 0.849229, -0.494759), point_t(0.947139, 0.0191441, -0.320253), point_t(0.262497, -0.527678, -0.807869), ext_colour_t(255.0, 255.0, 255.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(0.0,   1.0, 100.0), 0.0, 10.0);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(0.0, 100.0,   1.0), 0.0, 10.0);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( ply_happy_vrip_res4_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/ply_scenes/happy_budha/happy_vrip_res4.ply", model_format_t::ply, point_t(-0.08, 0.3, 0.2), point_t(0.184456, 0.849229, -0.494759), point_t(0.947139, 0.0191441, -0.320253), point_t(0.262497, -0.527678, -0.807869), ext_colour_t(255.0, 255.0, 255.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(0.0,   1.0, 100.0), 0.0, 10.0);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(0.0, 100.0,   1.0), 0.0, 10.0);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( ply_armadillo_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/ply_scenes/armadillo/Armadillo.ply", model_format_t::ply, point_t(50.0, 140.0, -225.0), point_t(0.951057, 0.0, 0.309017), point_t(0.140291, 0.891007, 0.431771), point_t(-0.275337, -0.453991, 0.847397), ext_colour_t(255.0, 255.0, 255.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(500.0,   0.0,  -100.0), 0.0, 10.0);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(  0.0, 100.0, -5000.0), 0.0, 10.0);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( ply_drill_shaft_vrip_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/ply_scenes/drill/drill_shaft_vrip.ply", model_format_t::ply, point_t(-0.06, 0.18, 0.08), point_t(0.754168, -0.00684085, 0.656646), point_t(0.332036, 0.866678, -0.37232), point_t(0.566554, -0.498822, -0.655892), ext_colour_t(255.0, 255.0, 255.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(10.0,   1.0, 100.0), 0.0, 10.0);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t( 0.0, 100.0,   1.0), 0.0, 10.0);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( ply_drill_shaft_zip_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/ply_scenes/drill/drill_shaft_zip.ply", model_format_t::ply, point_t(-0.06, 0.18, 0.08), point_t(0.754168, -0.00684085, 0.656646), point_t(0.332036, 0.866678, -0.37232), point_t(0.566554, -0.498822, -0.655892), ext_colour_t(255.0, 255.0, 255.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(10.0,   1.0, 100.0), 0.0, 10.0);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t( 0.0, 100.0,   1.0), 0.0, 10.0);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( ply_xyzrgb_dragon_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/ply_scenes/xyzrgb_dragon/xyzrgb_dragon.ply", model_format_t::ply, point_t(110.0, -50.0, -175.0), point_t(0.873063, -0.0552301, 0.48447), point_t(0.187223, 0.955376, -0.22848), point_t(-0.450232, 0.290181, 0.844444), ext_colour_t(255.0, 255.0, 255.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(100.0,   0.0, -1000.0), 0.0, 10.0);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(  0.0, 500.0,   100.0), 0.0, 10.0);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( ply_xyzrgb_manuscript_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/ply_scenes/xyzrgb_manuscript/xyzrgb_manuscript.ply", model_format_t::ply, point_t(125.0, -90.0, -275.0), point_t(0.873063, -0.0552301, 0.48447), point_t(0.187223, 0.955376, -0.22848), point_t(-0.450232, 0.290181, 0.844444), ext_colour_t(255.0, 255.0, 255.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(100.0,   0.0, -1000.0), 0.0, 10.0);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(  0.0, 500.0,   100.0), 0.0, 10.0);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( ply_xyzrgb_statuette_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/ply_scenes/xyzrgb_statuette/xyzrgb_statuette.ply", model_format_t::ply, point_t(-15.0, 10.0, -500.0), point_t(0.000986636, 0.999507, 0.0313953), point_t(0.999507, 0.0, -0.0314108), point_t(0.0313953, -0.0314108, 0.999013), ext_colour_t(255.0, 255.0, 255.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(100.0,   0.0, -1000.0), 0.0, 10.0);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(  0.0, 500.0,   100.0), 0.0, 10.0);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( ply_blade_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/ply_scenes/blade/blade.ply", model_format_t::ply, point_t(-150.0, -150.0, -500.0), point_t(0.982723, -0.031024, 0.182462), point_t(0.00155596, 0.987201, 0.159474), point_t(-0.185074, -0.156434, 0.970194), ext_colour_t(255.0, 255.0, 255.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(100.0,   0.0, -1000.0), 0.0, 10.0);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(  0.0, 500.0,   100.0), 0.0, 10.0);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( ply_hand_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/ply_scenes/hand/hand.ply", model_format_t::ply, point_t(4.78399, 2.74896, -10.0388), point_t(0.12625, 0.991625, 0.0272109), point_t(0.987618, 0.12822, -0.0903884), point_t(-0.0931204, -0.0154624, 0.995535), ext_colour_t(255.0, 255.0, 255.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(100.0,   0.0, -1000.0), 0.0, 10.0);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(  0.0, 500.0,   100.0), 0.0, 10.0);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( ply_lucy_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/ply_scenes/lucy/lucy.ply", model_format_t::ply, point_t(625.0, 1700.0, 150.0), point_t(0.0, 0.0314108, 0.999507), point_t(0.999507, 0.0313953, -0.000986636), point_t(0.0314108, -0.999013, 0.0313953), ext_colour_t(255.0, 255.0, 255.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(100.0,   0.0, -1000.0), 0.0, 10.0);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(  0.0, 500.0,   100.0), 0.0, 10.0);

    /* Ray trace the scene using kd tree */
    // fixture.render<kd_tree>();
    // checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( ply_horse_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/ply_scenes/horse/horse.ply", model_format_t::ply, point_t(-0.252297, 0.074832, 0.11), point_t(0.298892, 0.953555, -0.0373727), point_t(0.364136, -0.0777636, 0.928094), point_t(0.882082, -0.291008, -0.370466), ext_colour_t(255.0, 255.0, 255.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-100.0, 100.0, 200.0), 0.0, 10.0);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(   0.0, -50.0, 100.0), 0.0, 10.0);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}
#endif /* #ifndef VALGRIND_TESTS */

BOOST_AUTO_TEST_CASE( ply_happy_vrip_res4_low_res_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/ply_scenes/happy_budha/happy_vrip_res4.ply", model_format_t::ply, point_t(-0.08, 0.3, 0.2), point_t(0.184456, 0.849229, -0.494759), point_t(0.947139, 0.0191441, -0.320253), point_t(0.262497, -0.527678, -0.807869), ext_colour_t(255.0, 255.0, 255.0), 0.0, 0.0, 0.0, 640, 480);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(0.0,   1.0, 100.0), 0.0, 10.0);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(0.0, 100.0,   1.0), 0.0, 10.0);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_SUITE_END()
}; /* namespace raptor_raytracer */
}; /* namespace test */
