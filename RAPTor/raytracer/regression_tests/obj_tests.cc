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
#endif /* #ifndef VALGRIND_TESTS */
// obj_scenes/A10/A10.3ds
// obj_scenes/B-17/.DS_Store
// obj_scenes/Humvee/Tex_0023_1.png
// obj_scenes/b2_spirit/455deb7c.png
// obj_scenes/black_hawk/0783dfef.png
// obj_scenes/damaged_downtown/Downtown_Damage_0.obj
// obj_scenes/damaged_downtown/Downtown_Damage_1.obj
// obj_scenes/damaged_downtown/Downtown_Damage_2.obj
// obj_scenes/desert_city/desert city.obj
// obj_scenes/elsa/Elsa2.obj
// obj_scenes/iron_man/IronMan.obj
// obj_scenes/mig_21/Mig21.obj
// obj_scenes/sof/sof.obj
// obj_scenes/t-90a/t-90a(Elements_of_war).obj
// obj_scenes/war_machine/iron-man-v1_&_v2_(+war_machine).obj
// obj_scenes/war_machine/war_machine-v2.obj
// obj_scenes/white_house/Runtime/libraries/character/WhiteHouse/WhiteHouse.obj

BOOST_AUTO_TEST_SUITE_END()
}; /* namespace raptor_raytracer */
}; /* namespace test */
