#ifdef STAND_ALONE
#define BOOST_TEST_MODULE cfg test

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
BOOST_AUTO_TEST_SUITE( cfg_tests );

// #ifndef VALGRIND_TESTS
// BOOST_AUTO_TEST_CASE( cfg_mgftree_test )
// {
//     /* Checker */
//     CREATE_REGRESSION_CHECKER(checker);

//     /* Enviroment set up */
//     regression_fixture fixture(test_name, "/mgf_scenes/mgftree/mgftree.cfg", model_format_t::cfg, point_t(1.5, -4.0, 2.37), point_t(0.929776, 0.368124, 0.0), point_t(-0.0915488, 0.231226, 0.968583), point_t(-0.356559, 0.900565, -0.24869), ext_colour_t(255.0, 255.0, 255.0), 0.0, 0.0, 0.0, 1920, 1080);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(5.0, -25.0, 5.0), 0.0, 10.0);

//     /* Ray trace the scene using bvh */
//     fixture.render<bvh>();
//     checker.check(fixture.get_camera(), "bvh");

//     /* Ray trace the scene using kd tree */
//     fixture.render<kd_tree>();
//     checker.check(fixture.get_camera(), "kdt");

//     /* Ray trace the scene using bih */
//     fixture.render<bih>();
//     checker.check(fixture.get_camera(), "bih");
// }

// BOOST_AUTO_TEST_CASE( cfg_design1_test )
// {
//     /* Checker */
//     CREATE_REGRESSION_CHECKER(checker);

//     /* Enviroment set up */
//     regression_fixture fixture(test_name, "/mgf_scenes/nrcoff/design1.cfg", model_format_t::cfg, point_t(0.82453, 5.66974, 2.70777), point_t(0.505084, 0.860968, 0.0601528), point_t(0.314033, -0.24825, 0.916379), point_t(0.803909, -0.443958, -0.39576), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(1.0, 6.0, 2.7), 0.0, 0.001);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(2.5, 6.0, 2.6), 0.0, 0.001);

//     /* Ray trace the scene using bvh */
//     fixture.render<bvh>();
//     checker.check(fixture.get_camera(), "bvh");

//     /* Ray trace the scene using kd tree */
//     fixture.render<kd_tree>();
//     checker.check(fixture.get_camera(), "kdt");

//     /* Ray trace the scene using bih */
//     fixture.render<bih>();
//     checker.check(fixture.get_camera(), "bih");
// }

// BOOST_AUTO_TEST_CASE( cfg_df_dl_ww_test )
// {
//     /* Checker */
//     CREATE_REGRESSION_CHECKER(checker);

//     /* Enviroment set up */
//     regression_fixture fixture(test_name, "/mgf_scenes/model/df_dl_ww.cfg", model_format_t::cfg, point_t(1.99039, 6.84602, 2.85339), point_t(0.993631, -0.110235, 0.0240561), point_t(-0.0497237, -0.236433, 0.970391), point_t(-0.101282, -0.965395, -0.240404), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(1.0, 1.1, 2.0), 0.0, 0.1);

//     /* Ray trace the scene using bvh */
//     fixture.render<bvh>();
//     checker.check(fixture.get_camera(), "bvh");

//     /* Ray trace the scene using kd tree */
//     fixture.render<kd_tree>();
//     checker.check(fixture.get_camera(), "kdt");

//     /* Ray trace the scene using bih */
//     fixture.render<bih>();
//     checker.check(fixture.get_camera(), "bih");
// }

// BOOST_AUTO_TEST_CASE( cfg_if_ww_test )
// {
//     /* Checker */
//     CREATE_REGRESSION_CHECKER(checker);

//     /* Enviroment set up */
//     regression_fixture fixture(test_name, "/mgf_scenes/model/if_ww.cfg", model_format_t::cfg, point_t(1.99039, 6.84602, 2.85339), point_t(0.993631, -0.110235, 0.0240561), point_t(-0.0497237, -0.236433, 0.970391), point_t(-0.101282, -0.965395, -0.240404), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(1.0, 1.1, 2.0), 0.0, 0.1);

//     /* Ray trace the scene using bvh */
//     fixture.render<bvh>();
//     checker.check(fixture.get_camera(), "bvh");

//     /* Ray trace the scene using kd tree */
//     fixture.render<kd_tree>();
//     checker.check(fixture.get_camera(), "kdt");

//     /* Ray trace the scene using bih */
//     fixture.render<bih>();
//     checker.check(fixture.get_camera(), "bih");
// }

// BOOST_AUTO_TEST_CASE( cfg_dif_ww_test )
// {
//     /* Checker */
//     CREATE_REGRESSION_CHECKER(checker);

//     /* Enviroment set up */
//     regression_fixture fixture(test_name, "/mgf_scenes/model/dif_ww.cfg", model_format_t::cfg, point_t(1.99039, 6.84602, 2.85339), point_t(0.993631, -0.110235, 0.0240561), point_t(-0.0497237, -0.236433, 0.970391), point_t(-0.101282, -0.965395, -0.240404), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(1.0, 1.1, 2.0), 0.0, 0.1);

//     /* Ray trace the scene using bvh */
//     fixture.render<bvh>();
//     checker.check(fixture.get_camera(), "bvh");

//     /* Ray trace the scene using kd tree */
//     fixture.render<kd_tree>();
//     checker.check(fixture.get_camera(), "kdt");

//     /* Ray trace the scene using bih */
//     fixture.render<bih>();
//     checker.check(fixture.get_camera(), "bih");
// }

BOOST_AUTO_TEST_CASE( cfg_unc_powerplant_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture(test_name, "/ply_scenes/unc_powerplant/unc_powerplant.cfg", model_format_t::cfg, point_t(500000.0, 75000.0, 50000.0), point_t(0.0314108, 0.0, 0.999507), point_t(0.0313953, 0.999507, -0.000986636), point_t(-0.999013, 0.0314108, 0.0313953) , ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(500000.0, 500000.0, 100000.0), 0.0, 10.0);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(100000.0, 500000.0, 500000.0), 0.0, 10.0);

    // /* Ray trace the scene using bvh */
    // fixture.render<bvh>();
    // checker.check(fixture.get_camera(), "bvh");

    // /* Ray trace the scene using kd tree */
    // fixture.render<kd_tree>();
    // checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

// BOOST_AUTO_TEST_CASE( cfg_unc_powerplant_2_test )
// {
//     /* Checker */
//     CREATE_REGRESSION_CHECKER(checker);

//     /* Enviroment set up */
//     regression_fixture fixture(test_name, "/ply_scenes/unc_powerplant/unc_powerplant.cfg", model_format_t::cfg, point_t(-98371.6, 14684.6, 90946), point_t(-0.512434, 0.0539994, -0.857031), point_t(-0.21419, 0.95844, 0.188457), point_t(0.831589, 0.280139, -0.479571), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-500000.0, 50000.0, 100000.0), 0.0, 10.0);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-100000.0, 50000.0, 500000.0), 0.0, 10.0);

//     /* Ray trace the scene using bvh */
//     fixture.render<bvh>();
//     checker.check(fixture.get_camera(), "bvh");

//     /* Ray trace the scene using kd tree */
//     fixture.render<kd_tree>();
//     checker.check(fixture.get_camera(), "kdt");

//     /* Ray trace the scene using bih */
//     fixture.render<bih>();
//     checker.check(fixture.get_camera(), "bih");
// }
// #endif /* #ifndef VALGRIND_TESTS */

// BOOST_AUTO_TEST_CASE( cfg_dif_ww_low_res_test )
// {
//     /* Checker */
//     CREATE_REGRESSION_CHECKER(checker);

//     /* Enviroment set up */
//     regression_fixture fixture(test_name, "/mgf_scenes/model/dif_ww.cfg", model_format_t::cfg, point_t(1.99039, 6.84602, 2.85339), point_t(0.993631, -0.110235, 0.0240561), point_t(-0.0497237, -0.236433, 0.970391), point_t(-0.101282, -0.965395, -0.240404), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 640, 480);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(1.0, 1.1, 2.0), 0.0, 0.1);

//     /* Ray trace the scene using bvh */
//     fixture.render<bvh>();
//     checker.check(fixture.get_camera(), "bvh");

//     /* Ray trace the scene using kd tree */
//     fixture.render<kd_tree>();
//     checker.check(fixture.get_camera(), "kdt");

//     /* Ray trace the scene using bih */
//     fixture.render<bih>();
//     checker.check(fixture.get_camera(), "bih");
// }

BOOST_AUTO_TEST_SUITE_END()
}; /* namespace raptor_raytracer */
}; /* namespace test */
