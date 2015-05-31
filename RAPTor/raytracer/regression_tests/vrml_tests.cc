#ifdef STAND_ALONE
#define BOOST_TEST_MODULE vrml test

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
BOOST_AUTO_TEST_SUITE( vrml_tests );

BOOST_AUTO_TEST_CASE( vrml_haniwa_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/vrml_scenes/haniwa.wrl", model_format_t::vrml, point_t(1.0, -1.5, 5.5), point_t(1.0, 0.0, 0.0), point_t(0.0, 1.0, 0.0), point_t(0.0, 0.0, -1.0), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080, 1, 1, "default");
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(50.0, 20.0, -7.0), 0.0, 10.0);

    /* Ray trace the scene using bvh */
    // fixture.render<bvh>();
    // checker.check(fixture.get_camera(), "bvh");

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    // fixture.render<bih>();
    // checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( vrml_unicorn_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/vrml_scenes/unicorn.wrl", model_format_t::vrml, point_t(0.0471165, -0.3, -7.0), point_t(-0.999504, 0.0, 0.031411), point_t(0.0, 1.0, 0.0), point_t(0.031411, 0.0, 0.999504), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080, 1, 1, "default");
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(50.0, -20.0, -7.0), 0.0, 10.0);

    /* Ray trace the scene using bvh */
    // fixture.render<bvh>();
    // checker.check(fixture.get_camera(), "bvh");

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    // fixture.render<bih>();
    // checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_SUITE_END()
}; /* namespace raptor_raytracer */
}; /* namespace test */
