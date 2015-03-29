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
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

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
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

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
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");
    
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

BOOST_AUTO_TEST_CASE( obj_art_studio_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/obj_scenes/artistry/artStudio.obj", model_format_t::obj, point_t(-1550.0, 240.0, -182.0), point_t(-0.338738, 0.0, 0.940881), point_t(0.233987, 0.968583, 0.0842407), point_t(0.911321, -0.24869, 0.328096), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-1250.0, 250.0, -182.0), 0.0, 0.0001);
    // fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-100.0, 0.0,  2000.0), 0.0, 0.0001);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( obj_bedroom_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/obj_scenes/bedroom/Bedroom.obj", model_format_t::obj, point_t(155.0, 265.0, -125.0), point_t(-0.724664, -0.119806, -0.678599), point_t(-0.398447, 0.876307, 0.270784), point_t(-0.56222, -0.466613, 0.682764), ext_colour_t(255.0, 255.0, 255.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(155.0, 265.0, -125.0), 0.0, 0.0001);
    // fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-6.0, 10.0,  0.0), 0.0, 0.0001);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( obj_bottle_collection_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/obj_scenes/bottle_collection/BottleCollection.obj", model_format_t::obj, point_t(0.0, 50.0, 450.0), point_t(1.0, 0.0, 0.0), point_t(0.0, 1.0, 0.0), point_t(0.0, 0.0, -1.0), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(0.0, 50.0, 500.0), 0.0, 0.0001);
    // fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-6.0, 10.0,  0.0), 0.0, 0.0001);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( obj_candles_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/obj_scenes/candles/candles.obj", model_format_t::obj, point_t(17.8887, 65.0, 250.0), point_t(0.951055, 0.0, 0.309017), point_t(0.0483409, 0.987688, -0.148778), point_t(0.305212, -0.156434, -0.939346), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(0.0, 50.0, 500.0), 0.0, 0.0001);
    // fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-6.0, 10.0,  0.0), 0.0, 0.0001);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( obj_conference_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/obj_scenes/conference/conference.obj", model_format_t::obj, point_t(-575.0, 500.0, -1000.0), point_t(-0.612902, 0.0, 0.790148), point_t(0.0743595, 0.995562, 0.0576792), point_t(0.786641, -0.0941067, 0.610182), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(500.0, 500.0, -200.0), 0.0, 0.0001);
    // fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-6.0, 10.0,  0.0), 0.0, 0.0001);

    /* Ray trace the scene using kd tree */
    // fixture.render<kd_tree>();
    // checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( obj_cornell_box_empty_co_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/obj_scenes/cornell_box/CornellBox-Empty-CO.obj", model_format_t::obj, point_t(0.0, 1.0, 4.0), point_t(1.0, 0.0, 0.0), point_t(0.0, 1.0, 0.0), point_t(0.0, 0.0, -1.0), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(0.0, 0.0, 10.0), 0.0, 0.0001);
    // fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-6.0, 10.0,  0.0), 0.0, 0.0001);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( obj_cornell_box_empty_rg_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/obj_scenes/cornell_box/CornellBox-Empty-RG.obj", model_format_t::obj, point_t(0.0, 1.0, 4.0), point_t(1.0, 0.0, 0.0), point_t(0.0, 1.0, 0.0), point_t(0.0, 0.0, -1.0), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(0.0, 0.0, 10.0), 0.0, 0.0001);
    // fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-6.0, 10.0,  0.0), 0.0, 0.0001);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( obj_cornell_box_empty_squashed_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/obj_scenes/cornell_box/CornellBox-Empty-Squashed.obj", model_format_t::obj, point_t(0.0, 0.8, 3.5), point_t(1.0, 0.0, 0.0), point_t(0.0, 1.0, 0.0), point_t(0.0, 0.0, -1.0), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(0.0, 0.0, 10.0), 0.0, 0.0001);
    // fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-6.0, 10.0,  0.0), 0.0, 0.0001);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( obj_cornell_box_empty_white_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/obj_scenes/cornell_box/CornellBox-Empty-White.obj", model_format_t::obj, point_t(0.0, 1.0, 4.0), point_t(1.0, 0.0, 0.0), point_t(0.0, 1.0, 0.0), point_t(0.0, 0.0, -1.0), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(0.0, 0.0, 10.0), 0.0, 0.0001);
    // fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-6.0, 10.0,  0.0), 0.0, 0.0001);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( obj_cornell_box_glossy_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/obj_scenes/cornell_box/CornellBox-Glossy.obj", model_format_t::obj, point_t(0.0, 0.8, 4.0), point_t(1.0, 0.0, 0.0), point_t(0.0, 1.0, 0.0), point_t(0.0, 0.0, -1.0), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(0.0, 0.0, 10.0), 0.0, 0.0001);
    // fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-6.0, 10.0,  0.0), 0.0, 0.0001);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( obj_cornell_box_mirror_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/obj_scenes/cornell_box/CornellBox-Mirror.obj", model_format_t::obj, point_t(0.0, 1.0, 4.0), point_t(1.0, 0.0, 0.0), point_t(0.0, 1.0, 0.0), point_t(0.0, 0.0, -1.0), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(0.0, 0.0, 10.0), 0.0, 0.0001);
    // fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-6.0, 10.0,  0.0), 0.0, 0.0001);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( obj_cornell_box_original_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/obj_scenes/cornell_box/CornellBox-Original.obj", model_format_t::obj, point_t(0.0, 1.0, 4.0), point_t(1.0, 0.0, 0.0), point_t(0.0, 1.0, 0.0), point_t(0.0, 0.0, -1.0), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(0.0, 0.0, 10.0), 0.0, 0.0001);
    // fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-6.0, 10.0,  0.0), 0.0, 0.0001);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( obj_cornell_box_sphere_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/obj_scenes/cornell_box/CornellBox-Sphere.obj", model_format_t::obj, point_t(0.0, 0.8, 4.0), point_t(1.0, 0.0, 0.0), point_t(0.0, 1.0, 0.0), point_t(0.0, 0.0, -1.0), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(0.0, 0.0, 10.0), 0.0, 0.0001);
    // fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-6.0, 10.0,  0.0), 0.0, 0.0001);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( obj_cornell_box_water_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/obj_scenes/cornell_box/CornellBox-Water.obj", model_format_t::obj, point_t(0.0, 0.8, 4.0), point_t(1.0, 0.0, 0.0), point_t(0.0, 1.0, 0.0), point_t(0.0, 0.0, -1.0), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(0.0, 0.0, 10.0), 0.0, 0.0001);
    // fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-6.0, 10.0,  0.0), 0.0, 0.0001);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( obj_crytek_sponza_banner_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/obj_scenes/crytek_sponza/banner.obj", model_format_t::obj, point_t(-1000.0, -10.0, -730.0), point_t(-0.587778, 0.0, 0.809006), point_t(0.0, -1.0, 0.0), point_t(0.809013, 0.0, 0.587783), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-1000.0, -100.0, -730.0), 0.0, 0.0001);
    // fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-6.0, 10.0,  0.0), 0.0, 0.0001);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( obj_crytek_sponza_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/obj_scenes/crytek_sponza/sponza.obj", model_format_t::obj, point_t(600.0, 300.0, -30.0), point_t(0.0, 0.0, -1.0), point_t(0.0, 1.0, 0.0), point_t(-1.0, 0.0, 0.0), ext_colour_t(255.0, 255.0, 255.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(300.0, 500.0, -30.0), 0.0, 0.0001);
    // fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-6.0, 10.0,  0.0), 0.0, 0.0001);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( obj_cube_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/obj_scenes/cube/cube.obj", model_format_t::obj, point_t(2.21519, 2.0, 3.87274), point_t(0.860741, 0.0, -0.509041), point_t(-0.172431, 0.94088, -0.291566), point_t(-0.478947, -0.338737, -0.809855), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(0.0, -500.0, -30.0), 0.0, 0.0001);
    // fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-6.0, 10.0,  0.0), 0.0, 0.0001);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( obj_dabrovic_sponza_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/obj_scenes/dabrovic_sponza/sponza.obj", model_format_t::obj, point_t(15.947, 8.04172, -5.92896), point_t(-0.327855, 0.0395989, -0.943865), point_t(-0.0597337, 0.996246, 0.0625436), point_t(-0.942806, -0.0768859, 0.324262), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(4.0, 9.0, 0.0), 0.0, 0.0001);
    // fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-6.0, 10.0,  0.0), 0.0, 0.0001);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( obj_eye_polygonal_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/obj_scenes/eye/eyePolygonal.obj", model_format_t::obj, point_t(18.0, -1.0, -0.811631), point_t(-0.0941073, 0.0, -0.995558), point_t(0.0, 1.0, 0.0), point_t(-0.995558, 0.0, 0.0941073), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(20.0, 0.0, -1.0), 0.0, 0.0001);
    // fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-6.0, 10.0,  0.0), 0.0, 0.0001);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( obj_flying_spaghetti_monster_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/obj_scenes/flying_spaghetti_monster/FlyingSpaghettiMonster.obj", model_format_t::obj, point_t(235.0, 20.0, 19.0), point_t(0.208997, 0.0586416, -0.97613), point_t(-0.130897, 0.990888, 0.0315028), point_t(-0.969088, -0.12119, -0.214768), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(18.0, 0.0, -1.0), 0.0, 0.0001);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(235.0, 200.0, 19.0), 0.0, 0.0001);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( obj_fruit_v2_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/obj_scenes/fruit/fruit_v2.obj", model_format_t::obj, point_t(30.0, 13.0, 10.5), point_t(0.486089, -0.0980145, -0.868389), point_t(0.0929807, 0.965818, -0.24197), point_t(-0.814677, -0.410414, -0.40969), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(9.0,  14.0, 9.5), 0.0, 0.0001);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(49.0, 14.0, 9.5), 0.0, 0.0001);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( obj_hairball_0_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/obj_scenes/hairball/hairball.obj", model_format_t::obj, point_t(0.0, 0.0, 13.0), point_t(1.0, 0.0, 0.0), point_t(0.0, 1.0, 0.0), point_t(0.0, 0.0, -1.0), ext_colour_t(255.0, 255.0, 255.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(0.0, 0.0, 10.0), 0.0, 0.0001);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(10.0, 0.0, 10.0), 0.0, 0.0001);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( obj_hairball_1_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/obj_scenes/hairball/hairball.obj", model_format_t::obj, point_t(4.0, 0.0, 3.0), point_t(1.0, 0.0, 0.0), point_t(0.0, 1.0, 0.0), point_t(0.0, 0.0, -1.0), ext_colour_t(255.0, 255.0, 255.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(0.0, 0.0, 10.0), 0.0, 0.0001);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(10.0, 0.0, 10.0), 0.0, 0.0001);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( obj_haunted_hallway_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/obj_scenes/haunted_hallway/hauntedhallway.obj", model_format_t::obj, point_t(-1.98371, 21.9048, -52.3404), point_t(-0.956286, -0.0136459, 0.292099), point_t(0.159485, 0.812922, 0.560107), point_t(0.245097, -0.582208, 0.77521), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(0.0, 14.0, -37.0), 0.0, 0.0001);
    // fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-6.0, 10.0,  0.0), 0.0, 0.0001);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( obj_fs_kitchen_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/obj_scenes/kitchen/FS_Kitchen.obj", model_format_t::obj, point_t(-65.0, 67.0, 135.0), point_t(0.876302, 0.0, 0.48175), point_t(0.0, 1.0, 0.0), point_t(0.48175, 0.0, -0.876302), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-65.0, 70.0, 135.0), 0.0, 0.0001);
    // fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-6.0, 10.0,  0.0), 0.0, 0.0001);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( obj_head_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/obj_scenes/head/head.OBJ", model_format_t::obj, point_t(0.0, -0.0156976, 0.750493), point_t(1.0, 0.0, 0.0), point_t(0.0, 0.996917, -0.0784591), point_t(0.0, -0.0784591, -0.996917), ext_colour_t(255.0, 255.0, 255.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(0.5, 0.5, 1.0), 0.0, 0.0001);
    // fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-6.0, 10.0,  0.0), 0.0, 0.0001);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( obj_lost_empire_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/obj_scenes/lost_empire/lost_empire.obj", model_format_t::obj, point_t(3.10559, 40.7329, 21.77), point_t(0.983681, -0.0487641, 0.173058), point_t(0.0918722, 0.963702, -0.250664), point_t(0.154554, -0.262474, -0.952458), ext_colour_t(0.0, 0.0, 255.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(45.0, 75.0, 30.0), 0.0, 0.0001);
    // fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-6.0, 10.0,  0.0), 0.0, 0.0001);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( obj_mad_science_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/obj_scenes/mad_science/madScience.obj", model_format_t::obj, point_t(0.0, 115.0, -210.0), point_t(1.0, 0.0, 0.0), point_t(0.0, 0.876306, 0.481753), point_t(0.0, -0.481753, 0.876306), ext_colour_t(255.0, 255.0, 255.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(7.0, 115.0, -210.0), 0.0, 0.0001);
    // fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-6.0, 10.0,  0.0), 0.0, 0.0001);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( obj_matinee_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/obj_scenes/matinee/Matinee.obj", model_format_t::obj, point_t(1050.0, 475.0, -590.0), point_t(0.0860035, 0.00228097, 0.99629), point_t(-0.207885, 0.978026, 0.0157076), point_t(-0.974363, -0.208464, 0.0845876), ext_colour_t(255.0, 255.0, 255.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(90.0, 720.0, -820.0), 0.0, 0.0001);
    // fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-6.0, 10.0,  0.0), 0.0, 0.0001);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( obj_mitsuba_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/obj_scenes/mitsuba/mitsuba.obj", model_format_t::obj, point_t(0.0, 2.0, 4.0), point_t(1.0, 0.0, 0.0), point_t(0.0, 0.960293, -0.278991), point_t(0.0, -0.278991, -0.960293), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(0.0, 0.0, 10.0), 0.0, 0.0001);
    // fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-6.0, 10.0,  0.0), 0.0, 0.0001);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( obj_mitsuba_sphere_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/obj_scenes/mitsuba/mitsuba-sphere.obj", model_format_t::obj, point_t(0.0, 2.0, 4.0), point_t(1.0, 0.0, 0.0), point_t(0.0, 0.960293, -0.278991), point_t(0.0, -0.278991, -0.960293), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(0.0, 0.0, 10.0), 0.0, 0.0001);
    // fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-6.0, 10.0,  0.0), 0.0, 0.0001);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( obj_rungholt_house_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/obj_scenes/rungholt/house.obj", model_format_t::obj, point_t(85.0, 40.0, -59.0), point_t(0.587784, 0.0, 0.809015), point_t(-0.201194, 0.968583, 0.146176), point_t(-0.783598, -0.248689, 0.569318), ext_colour_t(255.0, 255.0, 255.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(15.0, 24.0, -105.0), 0.0, 0.0001);
    // fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-6.0, 10.0,  0.0), 0.0, 0.0001);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( obj_rungholt_0_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/obj_scenes/rungholt/rungholt.obj", model_format_t::obj, point_t(-251.0, 63.0, 81.0), point_t(-0.181559, 0.00417275, -0.983368), point_t(0.294922, 0.954189, -0.0504024), point_t(0.93811, -0.299168, -0.174472), ext_colour_t(0.0, 0.0, 255.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(0.0, 300.0, 0.0), 0.0, 0.0001);
    // fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-6.0, 10.0,  0.0), 0.0, 0.0001);

    /* Ray trace the scene using kd tree */
    // fixture.render<kd_tree>();
    // checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( obj_rungholt_1_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/obj_scenes/rungholt/rungholt.obj", model_format_t::obj, point_t(15.0, 7.0, -71.0), point_t(-0.201094, 0.00221717, 0.979556), point_t(0.09992, 0.994821, 0.0182609), point_t(-0.974444, 0.101549, -0.200271), ext_colour_t(0.0, 0.0, 255.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(0.0, 300.0, 0.0), 0.0, 0.0001);
    // fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-6.0, 10.0,  0.0), 0.0, 0.0001);

    /* Ray trace the scene using kd tree */
    // fixture.render<kd_tree>();
    // checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( obj_san_miguel_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/obj_scenes/san_miguel/san-miguel.obj", model_format_t::obj, point_t(9.0, 1.0, 5.0), point_t(-0.453989, 0.0, -0.891005), point_t(-0.0838509, 0.995562, 0.0427242), point_t(0.88705, 0.0941079, -0.451974), ext_colour_t(255.0, 255.0, 255.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(10.0, 5.0, 3.0), 0.0, 0.0001);
    // fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-6.0, 10.0,  0.0), 0.0, 0.0001);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( obj_sibenik_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/obj_scenes/sibenik/sibenik.obj", model_format_t::obj, point_t(-17.0, -13.0, 1.0), point_t(-0.0941056, 0.0, -0.995544), point_t(0.0, 1.0, 0.0), point_t(0.995544, 0.0, -0.0941056), ext_colour_t(255.0, 255.0, 255.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(125.0, 125.0, 125.0), point_t(-8.0, -9.0, 0.0), 0.0, 0.0001);
    fixture.add_light(ext_colour_t(125.0, 125.0, 125.0), point_t(10.0, -9.0, 0.0), 0.0, 0.0001);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( obj_teapot_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/obj_scenes/teapot/teapot.obj", model_format_t::obj, point_t(69.9687, 95.9084, 167.297), point_t(-0.933909, -0.0487743, 0.354163), point_t(-0.159631, 0.943301, -0.29103), point_t(-0.319888, -0.328331, -0.888745), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(70.0, 150.0, 140.0), 0.0, 0.0001);
    // fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-6.0, 10.0,  0.0), 0.0, 0.0001);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( obj_the_cabin_0_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/obj_scenes/the_cabin/Lighting_Challenge_24_theCabin.obj", model_format_t::obj, point_t(5.73536, -1.0, 11.7759), point_t(0.982275, 0.0, -0.187379), point_t(0.0, 1.0, 0.0), point_t(-0.187379, 0.0, -0.982275), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(0.0, 50.0, 30.0), 0.0, 0.0001);
    // fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-6.0, 10.0,  0.0), 0.0, 0.0001);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( obj_the_cabin_1_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/obj_scenes/the_cabin/Lighting_Challenge_24_theCabin.obj", model_format_t::obj, point_t(5.73536, -1.0, 11.7759), point_t(0.982275, 0.0, -0.187379), point_t(0.0, 1.0, 0.0), point_t(-0.187379, 0.0, -0.982275), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-10.0, 5.0, 30.0), 0.0, 0.0001);
    // fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-6.0, 10.0,  0.0), 0.0, 0.0001);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( obj_the_carnival_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/obj_scenes/the_carnival/TheCarnival.obj", model_format_t::obj, point_t(2858.0, 2234.0, -17052.0), point_t(-0.992112, 0.0, -0.125333), point_t(-0.0311691, 0.968583, 0.246728), point_t(-0.121396, -0.248688, 0.960943), ext_colour_t(255.0, 255.0, 255.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(1000.0, 5000.0, -15000.0), 0.0, 0.0001);
    // fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-6.0, 10.0,  0.0), 0.0, 0.0001);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( obj_A10_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/obj_scenes/A10/A10.obj", model_format_t::obj, point_t(7.68375, 3.44892, -15.2514), point_t(-0.917753, 0.0, -0.397147), point_t(-0.0866349, 0.975917, 0.200202), point_t(-0.387582, -0.218142, 0.8956), ext_colour_t(255.0, 255.0, 255.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(10.0, 50.0, -150.0), 0.0, 0.0001);
    // fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-6.0, 10.0,  0.0), 0.0, 0.0001);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( obj_b_17_silver_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/obj_scenes/B-17/B17SILVER.obj", model_format_t::obj, point_t(2.63233, 4.5, 5.5), point_t(0.951056, 0.0, -0.309017), point_t(-0.131573, 0.904826, -0.40494), point_t(-0.279606, -0.425779, -0.860541), ext_colour_t(0.0, 0.0, 200.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(10.0, 50.0, 150.0), 0.0, 0.0001);
    // fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-6.0, 10.0,  0.0), 0.0, 0.0001);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( obj_b2_spirit_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/obj_scenes/b2_spirit/B2_Spirit.obj", model_format_t::obj, point_t(22.0, 14.0, -34.0), point_t(0.929776, 0.0, 0.368124), point_t(-0.135515, 0.929776, 0.342273), point_t(-0.342273, -0.368124, 0.864484), ext_colour_t(0.0, 0.0, 200.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(10.0, 5.0, -160.0), 0.0, 0.0001);
    // fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-6.0, 10.0,  0.0), 0.0, 0.0001);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( obj_blackhawk_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/obj_scenes/black_hawk/Blackhawk.obj", model_format_t::obj, point_t(8.45907, 1.0, -9.32057), point_t(0.707106, 0.0, 0.707106), point_t(0.0, 1.0, 0.0), point_t(-0.707106, 0.0, 0.707106), ext_colour_t(0.0, 0.0, 200.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(200.0, 200.0, 200.0), point_t(1000.0, 100.0, -900.0), 0.0, 0.0001);
    // fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-6.0, 10.0,  0.0), 0.0, 0.0001);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( obj_Downtown_Damage_0_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/obj_scenes/damaged_downtown/Downtown_Damage_0.obj", model_format_t::obj, point_t(-76.0, 24.5, -127.5), point_t(-0.770505, 0.0, -0.637418), point_t(-0.0, 1.0, 0.0), point_t(0.637418, 0.0, -0.770505), ext_colour_t(0.0, 0.0, 200.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-76.0, 100.0, -127.5), 0.0, 0.0001);
    // fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-6.0, 10.0,  0.0), 0.0, 0.0001);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( obj_Downtown_Damage_1_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/obj_scenes/damaged_downtown/Downtown_Damage_1.obj", model_format_t::obj, point_t(-105.0, 25.0, 78.0), point_t(0.968581, 0.0, 0.248689), point_t(0.0, 1.0, 0.0), point_t(-0.248689, 0.0, 0.9685), ext_colour_t(0.0, 0.0, 200.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-75.0, 200.0, 78.0), 0.0, 0.0001);
    // fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-6.0, 10.0,  0.0), 0.0, 0.0001);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( obj_Downtown_Damage_2_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/obj_scenes/damaged_downtown/Downtown_Damage_2.obj", model_format_t::obj, point_t(-1.16788, 25.9646, 69.6975), point_t(0.311482, -0.0302841, -0.949754), point_t(-0.166501, 0.98229, -0.0859272), point_t(0.935535, 0.184899, 0.300923), ext_colour_t(0.0, 0.0, 200.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-72.0, 100.0, -35.0), 0.0, 0.0001);
    // fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-6.0, 10.0,  0.0), 0.0, 0.0001);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( obj_desert_city_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/obj_scenes/desert_city/desert_city.obj", model_format_t::obj, point_t(42.8173, 31.9754, 300.0), point_t(-0.992109, 0.0, 0.125333), point_t(-0.0196063, 0.987688, -0.1552), point_t(-0.12379, -0.156433, -0.979895), ext_colour_t(0.0, 0.0, 200.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(40.0, 300.0, 250.0), 0.0, 0.0001);
    // fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-6.0, 10.0,  0.0), 0.0, 0.0001);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( obj_elsa_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/obj_scenes/elsa/Elsa2.obj", model_format_t::obj, point_t(1.0, 1.75, 3.5), point_t(-0.939191, -0.0351117, 0.341594), point_t(-0.0953847, 0.982287, -0.161287), point_t(-0.329881, -0.184062, -0.925904), ext_colour_t(255.0, 255.0, 255.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(1.0, 2.5, 5.0), 0.0, 0.0001);
    // fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-6.0, 10.0,  0.0), 0.0, 0.0001);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( obj_ironman_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/obj_scenes/iron_man/IronMan.obj", model_format_t::obj, point_t(326.662, 276.885, 322.31), point_t(-0.70313, -0.0389036, 0.709995), point_t(-0.24362, 0.951249, -0.189142), point_t(-0.668023, -0.305961, -0.67833), ext_colour_t(255.0, 255.0, 255.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(200.0, 300.0, 300.0), 0.0, 0.0001);
    // fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-6.0, 10.0,  0.0), 0.0, 0.0001);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( obj_mig21_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/obj_scenes/mig_21/Mig21.obj", model_format_t::obj, point_t(2.72193, 1.16851, 10.4699), point_t(-0.940318, 0.0126159, 0.34006), point_t(-0.0396971, 0.988423, -0.146438), point_t(-0.33797, -0.151198, -0.928931), ext_colour_t(0.0, 0.0, 200.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(3.0, 5.0, 10.0), 0.0, 0.0001);
    // fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-6.0, 10.0,  0.0), 0.0, 0.0001);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( obj_spirit_of_fire_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/obj_scenes/sof/sof.obj", model_format_t::obj, point_t(64.9104, 27.3028, -17.1907), point_t(0.23036, -0.285895, 0.930159), point_t(-0.168811, 0.929634, 0.327541), point_t(-0.958351, -0.232474, 0.165888), ext_colour_t(255.0, 255.0, 255.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(0.0, 100.0, 0.0), 0.0, 0.0001);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(100.0, 0.0, 0.0), 0.0, 0.0001);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( obj_war_machine_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/obj_scenes/war_machine/iron-man-v1_&_v2_(+war_machine).obj", model_format_t::obj, point_t(63.0277, 84.5532, 120.0), point_t(-0.998022, 0.0, -0.0627896), point_t(0.00590902, 0.995562, -0.0939222), point_t(0.0625109, -0.0941075, -0.993593), ext_colour_t(255.0, 255.0, 255.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(10.0, 100.0, -15.0), 0.0, 0.0001);
    // fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-6.0, 10.0,  0.0), 0.0, 0.0001);

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
