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
// BOOST_AUTO_TEST_CASE( obj_isd_imperator_0_test )
// {
//     /* Checker */
//     CREATE_REGRESSION_CHECKER(checker);

//     /* Enviroment set up */
//     regression_fixture fixture("/obj_scenes/isd_imperator/isd_imperator.obj", model_format_t::obj, point_t(0.0, -85000.0, -650000.0), point_t(1.0, 0.0, 0.0), point_t(0.0, 1.0, 0.0), point_t(0.0, 0.0, 1.0), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(500000.0, 200000.0, -700000.0), 0.0, 10.0);

//     /* Ray trace the scene using kd tree */
//     /* Builder throws assert */
//     // fixture.render<kd_tree>();
//     // checker.check(fixture.get_camera(), "kdt");

//     /* Ray trace the scene using bih */
//     fixture.render<bih>();
//     checker.check(fixture.get_camera(), "bih");
// }

// BOOST_AUTO_TEST_CASE( obj_isd_imperator_1_test )
// {
//     /* Checker */
//     CREATE_REGRESSION_CHECKER(checker);

//     /* Enviroment set up */
//     regression_fixture fixture("/obj_scenes/isd_imperator/isd_imperator.obj", model_format_t::obj, point_t(-200000.0, -200000.0, 350000.0), point_t(0.880037, -0.0196184, 0.4745), point_t(-0.139384, 0.944474, 0.29756), point_t(0.45399, 0.328001, -0.828437), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-500000.0, -500000.0, 700000.0), 0.0, 10.0);

//     /* Ray trace the scene using kd tree */
//     /* Builder throws assert */
//     // fixture.render<kd_tree>();
//     // checker.check(fixture.get_camera(), "kdt");

//     /* Ray trace the scene using bih */
//     fixture.render<bih>();
//     checker.check(fixture.get_camera(), "bih");
// }

// BOOST_AUTO_TEST_CASE( obj_isd_imperator_2_test )
// {
//     /* Checker */
//     CREATE_REGRESSION_CHECKER(checker);

//     /* Enviroment set up */
//     regression_fixture fixture("/obj_scenes/isd_imperator/isd_imperator.obj", model_format_t::obj, point_t(500000.0, 250000.0, -650000.0), point_t(-0.639625, 0.000893619, -0.768686), point_t(-0.36828, 0.877402, 0.307467), point_t(-0.674721, -0.479756, 0.56088), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(0.0, 400000.0, -1000000.0), 0.0, 10.0);

//     /* Ray trace the scene using kd tree */
//     /* Builder throws assert */
//     // fixture.render<kd_tree>();
//     // checker.check(fixture.get_camera(), "kdt");
    
//     /* Ray trace the scene using bih */
//     fixture.render<bih>();
//     checker.check(fixture.get_camera(), "bih");
// }

// BOOST_AUTO_TEST_CASE( obj_eg07_dragon_original_test )
// {
//     /* Checker */
//     CREATE_REGRESSION_CHECKER(checker);

//     /* Enviroment set up */
//     regression_fixture fixture("/obj_scenes/phlegmatic_dragon/eg07_dragon_original.obj", model_format_t::obj, point_t(25.0, 300.0, -725.0), point_t(1.0, 0.0, 0.0), point_t(0.0, 0.0, -1.0), point_t(0.0, -1.0, 0.0), ext_colour_t(255.0, 255.0, 255.0), 0.0, 0.0, 0.0, 1920, 1080);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t( 500.0, 750.0, 1000.0), 0.0, 10.0);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-500.0, 500.0, -750.0), 0.0, 10.0);

//     /* Ray trace the scene using kd tree */
//     fixture.render<kd_tree>();
//     checker.check(fixture.get_camera(), "kdt");

//     /* Ray trace the scene using bih */
//     fixture.render<bih>();
//     checker.check(fixture.get_camera(), "bih");
// }

// BOOST_AUTO_TEST_CASE( obj_eg07_dragon_smoothed_test )
// {
//     /* Checker */
//     CREATE_REGRESSION_CHECKER(checker);

//     /* Enviroment set up */
//     regression_fixture fixture("/obj_scenes/phlegmatic_dragon/eg07_dragon_smoothed.obj", model_format_t::obj, point_t(3.33962, -225.0, 152.5), point_t(0.998058, -0.0153803, -0.0603799), point_t(0.0603797, 0.477955, 0.876307), point_t(-0.0153811, 0.878252, -0.477956), ext_colour_t(255.0, 255.0, 255.0), 0.0, 0.0, 0.0, 1920, 1080);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t( 250.0, -350.0,  500.0), 0.0, 10.0);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-250.0, -250.0, -350.0), 0.0, 10.0);

//     /* Ray trace the scene using kd tree */
//     fixture.render<kd_tree>();
//     checker.check(fixture.get_camera(), "kdt");

//     /* Ray trace the scene using bih */
//     fixture.render<bih>();
//     checker.check(fixture.get_camera(), "bih");
// }

// BOOST_AUTO_TEST_CASE( obj_sponza_test )
// {
//     /* Checker */
//     CREATE_REGRESSION_CHECKER(checker);

//     /* Enviroment set up */
//     regression_fixture fixture("/obj_scenes/sponza/sponza.obj", model_format_t::obj, point_t(-13.7643, -6.92507, 1.32953), point_t(-0.105038, -0.00870562, -0.99443), point_t(-0.238403, 0.971023, 0.0166812), point_t(0.965468, 0.238826, -0.104071), ext_colour_t(255.0, 255.0, 255.0), 0.0, 0.0, 0.0, 1920, 1080);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-5.0, -5.0, -3.0), 0.0, 0.0001);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-6.0, 10.0,  0.0), 0.0, 0.0001);

//     /* Ray trace the scene using kd tree */
//     fixture.render<kd_tree>();
//     checker.check(fixture.get_camera(), "kdt");

//     /* Ray trace the scene using bih */
//     fixture.render<bih>();
//     checker.check(fixture.get_camera(), "bih");
// }

// BOOST_AUTO_TEST_CASE( obj_art_studio_test )
// {
//     /* Checker */
//     CREATE_REGRESSION_CHECKER(checker);

//     /* Enviroment set up */
//     regression_fixture fixture("/obj_scenes/artistry/artStudio.obj", model_format_t::obj, point_t(-1550.0, 240.0, -182.0), point_t(-0.338738, 0.0, 0.940881), point_t(0.233987, 0.968583, 0.0842407), point_t(0.911321, -0.24869, 0.328096), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-1250.0, 250.0, -182.0), 0.0, 0.0001);
//     // fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-100.0, 0.0,  2000.0), 0.0, 0.0001);

//     /* Ray trace the scene using kd tree */
//     fixture.render<kd_tree>();
//     checker.check(fixture.get_camera(), "kdt");

//     /* Ray trace the scene using bih */
//     fixture.render<bih>();
//     checker.check(fixture.get_camera(), "bih");
// }

// BOOST_AUTO_TEST_CASE( obj_bedroom_test )
// {
//     /* Checker */
//     CREATE_REGRESSION_CHECKER(checker);

//     /* Enviroment set up */
//     regression_fixture fixture("/obj_scenes/bedroom/Bedroom.obj", model_format_t::obj, point_t(155.0, 265.0, -125.0), point_t(-0.724664, -0.119806, -0.678599), point_t(-0.398447, 0.876307, 0.270784), point_t(-0.56222, -0.466613, 0.682764), ext_colour_t(255.0, 255.0, 255.0), 0.0, 0.0, 0.0, 1920, 1080);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(155.0, 265.0, -125.0), 0.0, 0.0001);
//     // fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-6.0, 10.0,  0.0), 0.0, 0.0001);

//     /* Ray trace the scene using kd tree */
//     fixture.render<kd_tree>();
//     checker.check(fixture.get_camera(), "kdt");

//     /* Ray trace the scene using bih */
//     fixture.render<bih>();
//     checker.check(fixture.get_camera(), "bih");
// }

// BOOST_AUTO_TEST_CASE( obj_bottle_collection_test )
// {
//     /* Checker */
//     CREATE_REGRESSION_CHECKER(checker);

//     /* Enviroment set up */
//     regression_fixture fixture("/obj_scenes/bottle_collection/BottleCollection.obj", model_format_t::obj, point_t(0.0, 50.0, 450.0), point_t(1.0, 0.0, 0.0), point_t(0.0, 1.0, 0.0), point_t(0.0, 0.0, -1.0), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(0.0, 50.0, 500.0), 0.0, 0.0001);
//     // fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-6.0, 10.0,  0.0), 0.0, 0.0001);

//     /* Ray trace the scene using kd tree */
//     fixture.render<kd_tree>();
//     checker.check(fixture.get_camera(), "kdt");

//     /* Ray trace the scene using bih */
//     fixture.render<bih>();
//     checker.check(fixture.get_camera(), "bih");
// }

// BOOST_AUTO_TEST_CASE( obj_candles_test )
// {
//     /* Checker */
//     CREATE_REGRESSION_CHECKER(checker);

//     /* Enviroment set up */
//     regression_fixture fixture("/obj_scenes/candles/candles.obj", model_format_t::obj, point_t(17.8887, 65.0, 250.0), point_t(0.951055, 0.0, 0.309017), point_t(0.0483409, 0.987688, -0.148778), point_t(0.305212, -0.156434, -0.939346), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(0.0, 50.0, 500.0), 0.0, 0.0001);
//     // fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-6.0, 10.0,  0.0), 0.0, 0.0001);

//     /* Ray trace the scene using kd tree */
//     fixture.render<kd_tree>();
//     checker.check(fixture.get_camera(), "kdt");

//     /* Ray trace the scene using bih */
//     fixture.render<bih>();
//     checker.check(fixture.get_camera(), "bih");
// }

// BOOST_AUTO_TEST_CASE( obj_conference_test )
// {
//     /* Checker */
//     CREATE_REGRESSION_CHECKER(checker);

//     /* Enviroment set up */
//     regression_fixture fixture("/obj_scenes/conference/conference.obj", model_format_t::obj, point_t(-575.0, 500.0, -1000.0), point_t(-0.612902, 0.0, 0.790148), point_t(0.0743595, 0.995562, 0.0576792), point_t(0.786641, -0.0941067, 0.610182), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(500.0, 500.0, -200.0), 0.0, 0.0001);
//     // fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-6.0, 10.0,  0.0), 0.0, 0.0001);

//     /* Ray trace the scene using kd tree */
//     // fixture.render<kd_tree>();
//     // checker.check(fixture.get_camera(), "kdt");

//     /* Ray trace the scene using bih */
//     fixture.render<bih>();
//     checker.check(fixture.get_camera(), "bih");
// }

// BOOST_AUTO_TEST_CASE( obj_cornell_box_empty_co_test )
// {
//     /* Checker */
//     CREATE_REGRESSION_CHECKER(checker);

//     /* Enviroment set up */
//     regression_fixture fixture("/obj_scenes/cornell_box/CornellBox-Empty-CO.obj", model_format_t::obj, point_t(-13.7643, -6.92507, 1.32953), point_t(-0.105038, -0.00870562, -0.99443), point_t(-0.238403, 0.971023, 0.0166812), point_t(0.965468, 0.238826, -0.104071), ext_colour_t(255.0, 255.0, 255.0), 0.0, 0.0, 0.0, 1920, 1080);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-5.0, -5.0, -3.0), 0.0, 0.0001);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-6.0, 10.0,  0.0), 0.0, 0.0001);

//     /* Ray trace the scene using kd tree */
//     fixture.render<kd_tree>();
//     checker.check(fixture.get_camera(), "kdt");

//     /* Ray trace the scene using bih */
//     fixture.render<bih>();
//     checker.check(fixture.get_camera(), "bih");
// }

// BOOST_AUTO_TEST_CASE( obj_cornell_box_empty_rg_test )
// {
//     /* Checker */
//     CREATE_REGRESSION_CHECKER(checker);

//     /* Enviroment set up */
//     regression_fixture fixture("/obj_scenes/cornell_box/CornellBox-Empty-RG.obj", model_format_t::obj, point_t(-13.7643, -6.92507, 1.32953), point_t(-0.105038, -0.00870562, -0.99443), point_t(-0.238403, 0.971023, 0.0166812), point_t(0.965468, 0.238826, -0.104071), ext_colour_t(255.0, 255.0, 255.0), 0.0, 0.0, 0.0, 1920, 1080);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-5.0, -5.0, -3.0), 0.0, 0.0001);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-6.0, 10.0,  0.0), 0.0, 0.0001);

//     /* Ray trace the scene using kd tree */
//     fixture.render<kd_tree>();
//     checker.check(fixture.get_camera(), "kdt");

//     /* Ray trace the scene using bih */
//     fixture.render<bih>();
//     checker.check(fixture.get_camera(), "bih");
// }

// BOOST_AUTO_TEST_CASE( obj_cornell_box_empty_squashed_test )
// {
//     /* Checker */
//     CREATE_REGRESSION_CHECKER(checker);

//     /* Enviroment set up */
//     regression_fixture fixture("/obj_scenes/cornell_box/CornellBox-Empty-Squashed.obj", model_format_t::obj, point_t(-13.7643, -6.92507, 1.32953), point_t(-0.105038, -0.00870562, -0.99443), point_t(-0.238403, 0.971023, 0.0166812), point_t(0.965468, 0.238826, -0.104071), ext_colour_t(255.0, 255.0, 255.0), 0.0, 0.0, 0.0, 1920, 1080);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-5.0, -5.0, -3.0), 0.0, 0.0001);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-6.0, 10.0,  0.0), 0.0, 0.0001);

//     /* Ray trace the scene using kd tree */
//     fixture.render<kd_tree>();
//     checker.check(fixture.get_camera(), "kdt");

//     /* Ray trace the scene using bih */
//     fixture.render<bih>();
//     checker.check(fixture.get_camera(), "bih");
// }

// BOOST_AUTO_TEST_CASE( obj_cornell_box_empty_white_test )
// {
//     /* Checker */
//     CREATE_REGRESSION_CHECKER(checker);

//     /* Enviroment set up */
//     regression_fixture fixture("/obj_scenes/cornell_box/CornellBox-Empty-White.obj", model_format_t::obj, point_t(-13.7643, -6.92507, 1.32953), point_t(-0.105038, -0.00870562, -0.99443), point_t(-0.238403, 0.971023, 0.0166812), point_t(0.965468, 0.238826, -0.104071), ext_colour_t(255.0, 255.0, 255.0), 0.0, 0.0, 0.0, 1920, 1080);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-5.0, -5.0, -3.0), 0.0, 0.0001);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-6.0, 10.0,  0.0), 0.0, 0.0001);

//     /* Ray trace the scene using kd tree */
//     fixture.render<kd_tree>();
//     checker.check(fixture.get_camera(), "kdt");

//     /* Ray trace the scene using bih */
//     fixture.render<bih>();
//     checker.check(fixture.get_camera(), "bih");
// }

// BOOST_AUTO_TEST_CASE( obj_cornell_box_glossy_test )
// {
//     /* Checker */
//     CREATE_REGRESSION_CHECKER(checker);

//     /* Enviroment set up */
//     regression_fixture fixture("/obj_scenes/cornell_box/CornellBox-Glossy.obj", model_format_t::obj, point_t(-13.7643, -6.92507, 1.32953), point_t(-0.105038, -0.00870562, -0.99443), point_t(-0.238403, 0.971023, 0.0166812), point_t(0.965468, 0.238826, -0.104071), ext_colour_t(255.0, 255.0, 255.0), 0.0, 0.0, 0.0, 1920, 1080);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-5.0, -5.0, -3.0), 0.0, 0.0001);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-6.0, 10.0,  0.0), 0.0, 0.0001);

//     /* Ray trace the scene using kd tree */
//     fixture.render<kd_tree>();
//     checker.check(fixture.get_camera(), "kdt");

//     /* Ray trace the scene using bih */
//     fixture.render<bih>();
//     checker.check(fixture.get_camera(), "bih");
// }

// BOOST_AUTO_TEST_CASE( obj_cornell_box_mirror_test )
// {
//     /* Checker */
//     CREATE_REGRESSION_CHECKER(checker);

//     /* Enviroment set up */
//     regression_fixture fixture("/obj_scenes/cornell_box/CornellBox-Mirror.obj", model_format_t::obj, point_t(-13.7643, -6.92507, 1.32953), point_t(-0.105038, -0.00870562, -0.99443), point_t(-0.238403, 0.971023, 0.0166812), point_t(0.965468, 0.238826, -0.104071), ext_colour_t(255.0, 255.0, 255.0), 0.0, 0.0, 0.0, 1920, 1080);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-5.0, -5.0, -3.0), 0.0, 0.0001);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-6.0, 10.0,  0.0), 0.0, 0.0001);

//     /* Ray trace the scene using kd tree */
//     fixture.render<kd_tree>();
//     checker.check(fixture.get_camera(), "kdt");

//     /* Ray trace the scene using bih */
//     fixture.render<bih>();
//     checker.check(fixture.get_camera(), "bih");
// }

// BOOST_AUTO_TEST_CASE( obj_cornell_box_original_test )
// {
//     /* Checker */
//     CREATE_REGRESSION_CHECKER(checker);

//     /* Enviroment set up */
//     regression_fixture fixture("/obj_scenes/cornell_box/CornellBox-Original.obj", model_format_t::obj, point_t(-13.7643, -6.92507, 1.32953), point_t(-0.105038, -0.00870562, -0.99443), point_t(-0.238403, 0.971023, 0.0166812), point_t(0.965468, 0.238826, -0.104071), ext_colour_t(255.0, 255.0, 255.0), 0.0, 0.0, 0.0, 1920, 1080);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-5.0, -5.0, -3.0), 0.0, 0.0001);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-6.0, 10.0,  0.0), 0.0, 0.0001);

//     /* Ray trace the scene using kd tree */
//     fixture.render<kd_tree>();
//     checker.check(fixture.get_camera(), "kdt");

//     /* Ray trace the scene using bih */
//     fixture.render<bih>();
//     checker.check(fixture.get_camera(), "bih");
// }

// BOOST_AUTO_TEST_CASE( obj_cornell_box_sphere_test )
// {
//     /* Checker */
//     CREATE_REGRESSION_CHECKER(checker);

//     /* Enviroment set up */
//     regression_fixture fixture("/obj_scenes/cornell_box/CornellBox-Sphere.obj", model_format_t::obj, point_t(-13.7643, -6.92507, 1.32953), point_t(-0.105038, -0.00870562, -0.99443), point_t(-0.238403, 0.971023, 0.0166812), point_t(0.965468, 0.238826, -0.104071), ext_colour_t(255.0, 255.0, 255.0), 0.0, 0.0, 0.0, 1920, 1080);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-5.0, -5.0, -3.0), 0.0, 0.0001);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-6.0, 10.0,  0.0), 0.0, 0.0001);

//     /* Ray trace the scene using kd tree */
//     fixture.render<kd_tree>();
//     checker.check(fixture.get_camera(), "kdt");

//     /* Ray trace the scene using bih */
//     fixture.render<bih>();
//     checker.check(fixture.get_camera(), "bih");
// }

// BOOST_AUTO_TEST_CASE( obj_cornell_box_water_test )
// {
//     /* Checker */
//     CREATE_REGRESSION_CHECKER(checker);

//     /* Enviroment set up */
//     regression_fixture fixture("/obj_scenes/cornell_box/CornellBox-Water.obj", model_format_t::obj, point_t(-13.7643, -6.92507, 1.32953), point_t(-0.105038, -0.00870562, -0.99443), point_t(-0.238403, 0.971023, 0.0166812), point_t(0.965468, 0.238826, -0.104071), ext_colour_t(255.0, 255.0, 255.0), 0.0, 0.0, 0.0, 1920, 1080);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-5.0, -5.0, -3.0), 0.0, 0.0001);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-6.0, 10.0,  0.0), 0.0, 0.0001);

//     /* Ray trace the scene using kd tree */
//     fixture.render<kd_tree>();
//     checker.check(fixture.get_camera(), "kdt");

//     /* Ray trace the scene using bih */
//     fixture.render<bih>();
//     checker.check(fixture.get_camera(), "bih");
// }

// BOOST_AUTO_TEST_CASE( obj_water_test )
// {
//     /* Checker */
//     CREATE_REGRESSION_CHECKER(checker);

//     /* Enviroment set up */
//     regression_fixture fixture("/obj_scenes/cornell_box/water.obj", model_format_t::obj, point_t(-13.7643, -6.92507, 1.32953), point_t(-0.105038, -0.00870562, -0.99443), point_t(-0.238403, 0.971023, 0.0166812), point_t(0.965468, 0.238826, -0.104071), ext_colour_t(255.0, 255.0, 255.0), 0.0, 0.0, 0.0, 1920, 1080);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-5.0, -5.0, -3.0), 0.0, 0.0001);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-6.0, 10.0,  0.0), 0.0, 0.0001);

//     /* Ray trace the scene using kd tree */
//     fixture.render<kd_tree>();
//     checker.check(fixture.get_camera(), "kdt");

//     /* Ray trace the scene using bih */
//     fixture.render<bih>();
//     checker.check(fixture.get_camera(), "bih");
// }

// BOOST_AUTO_TEST_CASE( obj_crytek_sponza_banner_test )
// {
//     /* Checker */
//     CREATE_REGRESSION_CHECKER(checker);

//     /* Enviroment set up */
//     regression_fixture fixture("/obj_scenes/crytek_sponza/banner.obj", model_format_t::obj, point_t(-1000.0, -10.0, -730.0), point_t(-0.587778, 0.0, 0.809006), point_t(0.0, -1.0, 0.0), point_t(0.809013, 0.0, 0.587783), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-1000.0, -100.0, -730.0), 0.0, 0.0001);
//     // fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-6.0, 10.0,  0.0), 0.0, 0.0001);

//     /* Ray trace the scene using kd tree */
//     fixture.render<kd_tree>();
//     checker.check(fixture.get_camera(), "kdt");

//     /* Ray trace the scene using bih */
//     fixture.render<bih>();
//     checker.check(fixture.get_camera(), "bih");
// }

// BOOST_AUTO_TEST_CASE( obj_crytek_sponza_test )
// {
//     /* Checker */
//     CREATE_REGRESSION_CHECKER(checker);

//     /* Enviroment set up */
//     regression_fixture fixture("/obj_scenes/crytek_sponza/sponza.obj", model_format_t::obj, point_t(600.0, 300.0, -30.0), point_t(0.0, 0.0, -1.0), point_t(0.0, 1.0, 0.0), point_t(-1.0, 0.0, 0.0), ext_colour_t(255.0, 255.0, 255.0), 0.0, 0.0, 0.0, 1920, 1080);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(300.0, 500.0, -30.0), 0.0, 0.0001);
//     // fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-6.0, 10.0,  0.0), 0.0, 0.0001);

//     /* Ray trace the scene using kd tree */
//     fixture.render<kd_tree>();
//     checker.check(fixture.get_camera(), "kdt");

//     /* Ray trace the scene using bih */
//     fixture.render<bih>();
//     checker.check(fixture.get_camera(), "bih");
// }

// BOOST_AUTO_TEST_CASE( obj_cube_test )
// {
//     /* Checker */
//     CREATE_REGRESSION_CHECKER(checker);

//     /* Enviroment set up */
//     regression_fixture fixture("/obj_scenes/cube/cube.obj", model_format_t::obj, point_t(2.21519, 2.0, 3.87274), point_t(0.860741, 0.0, -0.509041), point_t(-0.172431, 0.94088, -0.291566), point_t(-0.478947, -0.338737, -0.809855), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(0.0, -500.0, -30.0), 0.0, 0.0001);
//     // fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-6.0, 10.0,  0.0), 0.0, 0.0001);

//     /* Ray trace the scene using kd tree */
//     fixture.render<kd_tree>();
//     checker.check(fixture.get_camera(), "kdt");

//     /* Ray trace the scene using bih */
//     fixture.render<bih>();
//     checker.check(fixture.get_camera(), "bih");
// }

// BOOST_AUTO_TEST_CASE( obj_dabrovic_sponza_test )
// {
//     /* Checker */
//     CREATE_REGRESSION_CHECKER(checker);

//     /* Enviroment set up */
//     regression_fixture fixture("/obj_scenes/dabrovic_sponza/sponza.obj", model_format_t::obj, point_t(15.947, 8.04172, -5.92896), point_t(-0.327855, 0.0395989, -0.943865), point_t(-0.0597337, 0.996246, 0.0625436), point_t(-0.942806, -0.0768859, 0.324262), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(4.0, 9.0, 0.0), 0.0, 0.0001);
//     // fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-6.0, 10.0,  0.0), 0.0, 0.0001);

//     /* Ray trace the scene using kd tree */
//     fixture.render<kd_tree>();
//     checker.check(fixture.get_camera(), "kdt");

//     /* Ray trace the scene using bih */
//     fixture.render<bih>();
//     checker.check(fixture.get_camera(), "bih");
// }

// BOOST_AUTO_TEST_CASE( obj_eye_polygonal_test )
// {
//     /* Checker */
//     CREATE_REGRESSION_CHECKER(checker);

//     /* Enviroment set up */
//     regression_fixture fixture("/obj_scenes/eye/eyePolygonal.obj", model_format_t::obj, point_t(18.0, -1.0, -0.811631), point_t(-0.0941073, 0.0, -0.995558), point_t(0.0, 1.0, 0.0), point_t(-0.995558, 0.0, 0.0941073), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(20.0, 0.0, -1.0), 0.0, 0.0001);
//     // fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-6.0, 10.0,  0.0), 0.0, 0.0001);

//     /* Ray trace the scene using kd tree */
//     fixture.render<kd_tree>();
//     checker.check(fixture.get_camera(), "kdt");

//     /* Ray trace the scene using bih */
//     fixture.render<bih>();
//     checker.check(fixture.get_camera(), "bih");
// }

// BOOST_AUTO_TEST_CASE( obj_flying_spaghetti_monster_test )
// {
//     /* Checker */
//     CREATE_REGRESSION_CHECKER(checker);

//     /* Enviroment set up */
//     regression_fixture fixture("/obj_scenes/flying_spaghetti_monster/FlyingSpaghettiMonster.obj", model_format_t::obj, point_t(235.0, 20.0, 19.0), point_t(0.208997, 0.0586416, -0.97613), point_t(-0.130897, 0.990888, 0.0315028), point_t(-0.969088, -0.12119, -0.214768), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(18.0, 0.0, -1.0), 0.0, 0.0001);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(235.0, 200.0, 19.0), 0.0, 0.0001);

//     /* Ray trace the scene using kd tree */
//     fixture.render<kd_tree>();
//     checker.check(fixture.get_camera(), "kdt");

//     /* Ray trace the scene using bih */
//     fixture.render<bih>();
//     checker.check(fixture.get_camera(), "bih");
// }

// BOOST_AUTO_TEST_CASE( obj_fruit_v2_test )
// {
//     /* Checker */
//     CREATE_REGRESSION_CHECKER(checker);

//     /* Enviroment set up */
//     regression_fixture fixture("/obj_scenes/fruit/fruit_v2.obj", model_format_t::obj, point_t(30.0, 13.0, 10.5), point_t(0.486089, -0.0980145, -0.868389), point_t(0.0929807, 0.965818, -0.24197), point_t(-0.814677, -0.410414, -0.40969), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(9.0,  14.0, 9.5), 0.0, 0.0001);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(49.0, 14.0, 9.5), 0.0, 0.0001);

//     /* Ray trace the scene using kd tree */
//     fixture.render<kd_tree>();
//     checker.check(fixture.get_camera(), "kdt");

//     /* Ray trace the scene using bih */
//     fixture.render<bih>();
//     checker.check(fixture.get_camera(), "bih");
// }

// BOOST_AUTO_TEST_CASE( obj_hairball_0_test )
// {
//     /* Checker */
//     CREATE_REGRESSION_CHECKER(checker);

//     /* Enviroment set up */
//     regression_fixture fixture("/obj_scenes/hairball/hairball.obj", model_format_t::obj, point_t(0.0, 0.0, 13.0), point_t(1.0, 0.0, 0.0), point_t(0.0, 1.0, 0.0), point_t(0.0, 0.0, -1.0), ext_colour_t(255.0, 255.0, 255.0), 0.0, 0.0, 0.0, 1920, 1080);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(0.0, 0.0, 10.0), 0.0, 0.0001);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(10.0, 0.0, 10.0), 0.0, 0.0001);

//     /* Ray trace the scene using kd tree */
//     fixture.render<kd_tree>();
//     checker.check(fixture.get_camera(), "kdt");

//     /* Ray trace the scene using bih */
//     fixture.render<bih>();
//     checker.check(fixture.get_camera(), "bih");
// }

// BOOST_AUTO_TEST_CASE( obj_hairball_1_test )
// {
//     /* Checker */
//     CREATE_REGRESSION_CHECKER(checker);

//     /* Enviroment set up */
//     regression_fixture fixture("/obj_scenes/hairball/hairball.obj", model_format_t::obj, point_t(4.0, 0.0, 3.0), point_t(1.0, 0.0, 0.0), point_t(0.0, 1.0, 0.0), point_t(0.0, 0.0, -1.0), ext_colour_t(255.0, 255.0, 255.0), 0.0, 0.0, 0.0, 1920, 1080);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(0.0, 0.0, 10.0), 0.0, 0.0001);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(10.0, 0.0, 10.0), 0.0, 0.0001);

//     /* Ray trace the scene using kd tree */
//     fixture.render<kd_tree>();
//     checker.check(fixture.get_camera(), "kdt");

//     /* Ray trace the scene using bih */
//     fixture.render<bih>();
//     checker.check(fixture.get_camera(), "bih");
// }

// BOOST_AUTO_TEST_CASE( obj_haunted_hallway_test )
// {
//     /* Checker */
//     CREATE_REGRESSION_CHECKER(checker);

//     /* Enviroment set up */
//     regression_fixture fixture("/obj_scenes/haunted_hallway/hauntedhallway.obj", model_format_t::obj, point_t(-1.98371, 21.9048, -52.3404), point_t(-0.956286, -0.0136459, 0.292099), point_t(0.159485, 0.812922, 0.560107), point_t(0.245097, -0.582208, 0.77521), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(0.0, 14.0, -37.0), 0.0, 0.0001);
//     // fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-6.0, 10.0,  0.0), 0.0, 0.0001);

//     /* Ray trace the scene using kd tree */
//     fixture.render<kd_tree>();
//     checker.check(fixture.get_camera(), "kdt");

//     /* Ray trace the scene using bih */
//     fixture.render<bih>();
//     checker.check(fixture.get_camera(), "bih");
// }

// BOOST_AUTO_TEST_CASE( obj_fs_kitchen_test )
// {
//     /* Checker */
//     CREATE_REGRESSION_CHECKER(checker);

//     /* Enviroment set up */
//     regression_fixture fixture("/obj_scenes/kitchen/FS_Kitchen.obj", model_format_t::obj, point_t(-65.0, 67.0, 135.0), point_t(0.876302, 0.0, 0.48175), point_t(0.0, 1.0, 0.0), point_t(0.48175, 0.0, -0.876302), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-65.0, 70.0, 135.0), 0.0, 0.0001);
//     // fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-6.0, 10.0,  0.0), 0.0, 0.0001);

//     /* Ray trace the scene using kd tree */
//     fixture.render<kd_tree>();
//     checker.check(fixture.get_camera(), "kdt");

//     /* Ray trace the scene using bih */
//     fixture.render<bih>();
//     checker.check(fixture.get_camera(), "bih");
// }

BOOST_AUTO_TEST_CASE( obj_lost_empire_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/obj_scenes/lost_empire/lost_empire.obj", model_format_t::obj, point_t(-13.7643, -6.92507, 1.32953), point_t(-0.105038, -0.00870562, -0.99443), point_t(-0.238403, 0.971023, 0.0166812), point_t(0.965468, 0.238826, -0.104071), ext_colour_t(255.0, 255.0, 255.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-5.0, -5.0, -3.0), 0.0, 0.0001);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-6.0, 10.0,  0.0), 0.0, 0.0001);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

// BOOST_AUTO_TEST_CASE( obj_mad_science_test )
// {
//     /* Checker */
//     CREATE_REGRESSION_CHECKER(checker);

//     /* Enviroment set up */
//     regression_fixture fixture("/obj_scenes/mad_science/madScience.obj", model_format_t::obj, point_t(-13.7643, -6.92507, 1.32953), point_t(-0.105038, -0.00870562, -0.99443), point_t(-0.238403, 0.971023, 0.0166812), point_t(0.965468, 0.238826, -0.104071), ext_colour_t(255.0, 255.0, 255.0), 0.0, 0.0, 0.0, 1920, 1080);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-5.0, -5.0, -3.0), 0.0, 0.0001);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-6.0, 10.0,  0.0), 0.0, 0.0001);

//     /* Ray trace the scene using kd tree */
//     fixture.render<kd_tree>();
//     checker.check(fixture.get_camera(), "kdt");

//     /* Ray trace the scene using bih */
//     fixture.render<bih>();
//     checker.check(fixture.get_camera(), "bih");
// }

// BOOST_AUTO_TEST_CASE( obj_matinee_test )
// {
//     /* Checker */
//     CREATE_REGRESSION_CHECKER(checker);

//     /* Enviroment set up */
//     regression_fixture fixture("/obj_scenes/matinee/Matinee.obj", model_format_t::obj, point_t(-13.7643, -6.92507, 1.32953), point_t(-0.105038, -0.00870562, -0.99443), point_t(-0.238403, 0.971023, 0.0166812), point_t(0.965468, 0.238826, -0.104071), ext_colour_t(255.0, 255.0, 255.0), 0.0, 0.0, 0.0, 1920, 1080);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-5.0, -5.0, -3.0), 0.0, 0.0001);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-6.0, 10.0,  0.0), 0.0, 0.0001);

//     /* Ray trace the scene using kd tree */
//     fixture.render<kd_tree>();
//     checker.check(fixture.get_camera(), "kdt");

//     /* Ray trace the scene using bih */
//     fixture.render<bih>();
//     checker.check(fixture.get_camera(), "bih");
// }

// BOOST_AUTO_TEST_CASE( obj_mitsuba_test )
// {
//     /* Checker */
//     CREATE_REGRESSION_CHECKER(checker);

//     /* Enviroment set up */
//     regression_fixture fixture("/obj_scenes/mitsuba/mitsuba.obj", model_format_t::obj, point_t(-13.7643, -6.92507, 1.32953), point_t(-0.105038, -0.00870562, -0.99443), point_t(-0.238403, 0.971023, 0.0166812), point_t(0.965468, 0.238826, -0.104071), ext_colour_t(255.0, 255.0, 255.0), 0.0, 0.0, 0.0, 1920, 1080);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-5.0, -5.0, -3.0), 0.0, 0.0001);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-6.0, 10.0,  0.0), 0.0, 0.0001);

//     /* Ray trace the scene using kd tree */
//     fixture.render<kd_tree>();
//     checker.check(fixture.get_camera(), "kdt");

//     /* Ray trace the scene using bih */
//     fixture.render<bih>();
//     checker.check(fixture.get_camera(), "bih");
// }

// BOOST_AUTO_TEST_CASE( obj_mitsuba_sphere_test )
// {
//     /* Checker */
//     CREATE_REGRESSION_CHECKER(checker);

//     /* Enviroment set up */
//     regression_fixture fixture("/obj_scenes/mitsuba/mitsuba-sphere.obj", model_format_t::obj, point_t(-13.7643, -6.92507, 1.32953), point_t(-0.105038, -0.00870562, -0.99443), point_t(-0.238403, 0.971023, 0.0166812), point_t(0.965468, 0.238826, -0.104071), ext_colour_t(255.0, 255.0, 255.0), 0.0, 0.0, 0.0, 1920, 1080);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-5.0, -5.0, -3.0), 0.0, 0.0001);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-6.0, 10.0,  0.0), 0.0, 0.0001);

//     /* Ray trace the scene using kd tree */
//     fixture.render<kd_tree>();
//     checker.check(fixture.get_camera(), "kdt");

//     /* Ray trace the scene using bih */
//     fixture.render<bih>();
//     checker.check(fixture.get_camera(), "bih");
// }

// BOOST_AUTO_TEST_CASE( obj_powerplant_test )
// {
//     /* Checker */
//     CREATE_REGRESSION_CHECKER(checker);

//     /* Enviroment set up */
//     regression_fixture fixture("/obj_scenes/powerplant/powerplant.obj", model_format_t::obj, point_t(-13.7643, -6.92507, 1.32953), point_t(-0.105038, -0.00870562, -0.99443), point_t(-0.238403, 0.971023, 0.0166812), point_t(0.965468, 0.238826, -0.104071), ext_colour_t(255.0, 255.0, 255.0), 0.0, 0.0, 0.0, 1920, 1080);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-5.0, -5.0, -3.0), 0.0, 0.0001);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-6.0, 10.0,  0.0), 0.0, 0.0001);

//     /* Ray trace the scene using kd tree */
//     fixture.render<kd_tree>();
//     checker.check(fixture.get_camera(), "kdt");

//     /* Ray trace the scene using bih */
//     fixture.render<bih>();
//     checker.check(fixture.get_camera(), "bih");
// }

// BOOST_AUTO_TEST_CASE( obj_rungholt_house_test )
// {
//     /* Checker */
//     CREATE_REGRESSION_CHECKER(checker);

//     /* Enviroment set up */
//     regression_fixture fixture("/obj_scenes/rungholt/house.obj", model_format_t::obj, point_t(-13.7643, -6.92507, 1.32953), point_t(-0.105038, -0.00870562, -0.99443), point_t(-0.238403, 0.971023, 0.0166812), point_t(0.965468, 0.238826, -0.104071), ext_colour_t(255.0, 255.0, 255.0), 0.0, 0.0, 0.0, 1920, 1080);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-5.0, -5.0, -3.0), 0.0, 0.0001);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-6.0, 10.0,  0.0), 0.0, 0.0001);

//     /* Ray trace the scene using kd tree */
//     fixture.render<kd_tree>();
//     checker.check(fixture.get_camera(), "kdt");

//     /* Ray trace the scene using bih */
//     fixture.render<bih>();
//     checker.check(fixture.get_camera(), "bih");
// }

// BOOST_AUTO_TEST_CASE( obj_rungholt_test )
// {
//     /* Checker */
//     CREATE_REGRESSION_CHECKER(checker);

//     /* Enviroment set up */
//     regression_fixture fixture("/obj_scenes/rungholt/rungholt.obj", model_format_t::obj, point_t(-13.7643, -6.92507, 1.32953), point_t(-0.105038, -0.00870562, -0.99443), point_t(-0.238403, 0.971023, 0.0166812), point_t(0.965468, 0.238826, -0.104071), ext_colour_t(255.0, 255.0, 255.0), 0.0, 0.0, 0.0, 1920, 1080);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-5.0, -5.0, -3.0), 0.0, 0.0001);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-6.0, 10.0,  0.0), 0.0, 0.0001);

//     /* Ray trace the scene using kd tree */
//     fixture.render<kd_tree>();
//     checker.check(fixture.get_camera(), "kdt");

//     /* Ray trace the scene using bih */
//     fixture.render<bih>();
//     checker.check(fixture.get_camera(), "bih");
// }

// BOOST_AUTO_TEST_CASE( obj_san_miguel_test )
// {
//     /* Checker */
//     CREATE_REGRESSION_CHECKER(checker);

//     /* Enviroment set up */
//     regression_fixture fixture("/obj_scenes/san_miguel/san-miguel.obj", model_format_t::obj, point_t(-13.7643, -6.92507, 1.32953), point_t(-0.105038, -0.00870562, -0.99443), point_t(-0.238403, 0.971023, 0.0166812), point_t(0.965468, 0.238826, -0.104071), ext_colour_t(255.0, 255.0, 255.0), 0.0, 0.0, 0.0, 1920, 1080);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-5.0, -5.0, -3.0), 0.0, 0.0001);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-6.0, 10.0,  0.0), 0.0, 0.0001);

//     /* Ray trace the scene using kd tree */
//     fixture.render<kd_tree>();
//     checker.check(fixture.get_camera(), "kdt");

//     /* Ray trace the scene using bih */
//     fixture.render<bih>();
//     checker.check(fixture.get_camera(), "bih");
// }

// BOOST_AUTO_TEST_CASE( obj_sibenik_test )
// {
//     /* Checker */
//     CREATE_REGRESSION_CHECKER(checker);

//     /* Enviroment set up */
//     regression_fixture fixture("/obj_scenes/sibenik/sibenik.obj", model_format_t::obj, point_t(-13.7643, -6.92507, 1.32953), point_t(-0.105038, -0.00870562, -0.99443), point_t(-0.238403, 0.971023, 0.0166812), point_t(0.965468, 0.238826, -0.104071), ext_colour_t(255.0, 255.0, 255.0), 0.0, 0.0, 0.0, 1920, 1080);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-5.0, -5.0, -3.0), 0.0, 0.0001);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-6.0, 10.0,  0.0), 0.0, 0.0001);

//     /* Ray trace the scene using kd tree */
//     fixture.render<kd_tree>();
//     checker.check(fixture.get_camera(), "kdt");

//     /* Ray trace the scene using bih */
//     fixture.render<bih>();
//     checker.check(fixture.get_camera(), "bih");
// }

// BOOST_AUTO_TEST_CASE( obj_teapot_test )
// {
//     /* Checker */
//     CREATE_REGRESSION_CHECKER(checker);

//     /* Enviroment set up */
//     regression_fixture fixture("/obj_scenes/teapot/teapot.obj", model_format_t::obj, point_t(-13.7643, -6.92507, 1.32953), point_t(-0.105038, -0.00870562, -0.99443), point_t(-0.238403, 0.971023, 0.0166812), point_t(0.965468, 0.238826, -0.104071), ext_colour_t(255.0, 255.0, 255.0), 0.0, 0.0, 0.0, 1920, 1080);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-5.0, -5.0, -3.0), 0.0, 0.0001);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-6.0, 10.0,  0.0), 0.0, 0.0001);

//     /* Ray trace the scene using kd tree */
//     fixture.render<kd_tree>();
//     checker.check(fixture.get_camera(), "kdt");

//     /* Ray trace the scene using bih */
//     fixture.render<bih>();
//     checker.check(fixture.get_camera(), "bih");
// }

// BOOST_AUTO_TEST_CASE( obj_lighting_challenge_24_the_cabin_test )
// {
//     /* Checker */
//     CREATE_REGRESSION_CHECKER(checker);

//     /* Enviroment set up */
//     regression_fixture fixture("/obj_scenes/the_cabin/Lighting_Challenge_24_theCabin.obj", model_format_t::obj, point_t(-13.7643, -6.92507, 1.32953), point_t(-0.105038, -0.00870562, -0.99443), point_t(-0.238403, 0.971023, 0.0166812), point_t(0.965468, 0.238826, -0.104071), ext_colour_t(255.0, 255.0, 255.0), 0.0, 0.0, 0.0, 1920, 1080);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-5.0, -5.0, -3.0), 0.0, 0.0001);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-6.0, 10.0,  0.0), 0.0, 0.0001);

//     /* Ray trace the scene using kd tree */
//     fixture.render<kd_tree>();
//     checker.check(fixture.get_camera(), "kdt");

//     /* Ray trace the scene using bih */
//     fixture.render<bih>();
//     checker.check(fixture.get_camera(), "bih");
// }

// BOOST_AUTO_TEST_CASE( obj_the_carnival_test )
// {
//     /* Checker */
//     CREATE_REGRESSION_CHECKER(checker);

//     /* Enviroment set up */
//     regression_fixture fixture("/obj_scenes/the_carnival/TheCarnival.obj", model_format_t::obj, point_t(-13.7643, -6.92507, 1.32953), point_t(-0.105038, -0.00870562, -0.99443), point_t(-0.238403, 0.971023, 0.0166812), point_t(0.965468, 0.238826, -0.104071), ext_colour_t(255.0, 255.0, 255.0), 0.0, 0.0, 0.0, 1920, 1080);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-5.0, -5.0, -3.0), 0.0, 0.0001);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-6.0, 10.0,  0.0), 0.0, 0.0001);

//     /* Ray trace the scene using kd tree */
//     fixture.render<kd_tree>();
//     checker.check(fixture.get_camera(), "kdt");

//     /* Ray trace the scene using bih */
//     fixture.render<bih>();
//     checker.check(fixture.get_camera(), "bih");
// }

// BOOST_AUTO_TEST_CASE( obj_under_the_boardwalk_test )
// {
//     /* Checker */
//     CREATE_REGRESSION_CHECKER(checker);

//     /* Enviroment set up */
//     regression_fixture fixture("/obj_scenes/under_the_boardwalk/UnderTheBoardwalk.obj", model_format_t::obj, point_t(-13.7643, -6.92507, 1.32953), point_t(-0.105038, -0.00870562, -0.99443), point_t(-0.238403, 0.971023, 0.0166812), point_t(0.965468, 0.238826, -0.104071), ext_colour_t(255.0, 255.0, 255.0), 0.0, 0.0, 0.0, 1920, 1080);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-5.0, -5.0, -3.0), 0.0, 0.0001);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-6.0, 10.0,  0.0), 0.0, 0.0001);

//     /* Ray trace the scene using kd tree */
//     fixture.render<kd_tree>();
//     checker.check(fixture.get_camera(), "kdt");

//     /* Ray trace the scene using bih */
//     fixture.render<bih>();
//     checker.check(fixture.get_camera(), "bih");
// }
#endif /* #ifndef VALGRIND_TESTS */

BOOST_AUTO_TEST_SUITE_END()
}; /* namespace raptor_raytracer */
}; /* namespace test */
