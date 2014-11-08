#ifdef STAND_ALONE
#define BOOST_TEST_MODULE nff test

/* Common headers */
#include "logging.h"

/* Initialise logger */
const raptor_physics::init_logger init_logger;
#endif /* #ifdef STAND_ALONE */

/* Test headers */
#include "regression_fixture.h"


BOOST_AUTO_TEST_SUITE( nff_tests );

BOOST_AUTO_TEST_CASE( nff_balls_1_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/spd3_14/balls_1", model_format_t::nff);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

BOOST_AUTO_TEST_CASE( nff_balls_2_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/spd3_14/balls_2", model_format_t::nff);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

BOOST_AUTO_TEST_CASE( nff_balls_3_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/spd3_14/balls_3", model_format_t::nff);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

BOOST_AUTO_TEST_CASE( nff_balls_4_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/spd3_14/balls_4", model_format_t::nff);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

BOOST_AUTO_TEST_CASE( nff_gears_2_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/spd3_14/gears_2", model_format_t::nff);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

BOOST_AUTO_TEST_CASE( nff_gears_5_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/spd3_14/gears_5", model_format_t::nff);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

BOOST_AUTO_TEST_CASE( nff_gears_12_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/spd3_14/gears_12", model_format_t::nff);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

BOOST_AUTO_TEST_CASE( nff_gears_25_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/spd3_14/gears_25", model_format_t::nff);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

BOOST_AUTO_TEST_CASE( nff_jacks_2_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/spd3_14/jacks_2", model_format_t::nff);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

BOOST_AUTO_TEST_CASE( nff_jacks_3_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/spd3_14/jacks_3", model_format_t::nff);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

BOOST_AUTO_TEST_CASE( nff_jacks_4_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/spd3_14/jacks_4", model_format_t::nff);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

BOOST_AUTO_TEST_CASE( nff_jacks_5_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/spd3_14/jacks_5", model_format_t::nff);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

BOOST_AUTO_TEST_CASE( nff_lattice_1_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/spd3_14/lattice_1", model_format_t::nff);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

BOOST_AUTO_TEST_CASE( nff_lattice_3_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/spd3_14/lattice_3", model_format_t::nff);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

BOOST_AUTO_TEST_CASE( nff_lattice_8_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/spd3_14/lattice_8", model_format_t::nff);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

BOOST_AUTO_TEST_CASE( nff_lattice_19_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/spd3_14/lattice_19", model_format_t::nff);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

BOOST_AUTO_TEST_CASE( nff_mount_4_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/spd3_14/mount_4", model_format_t::nff);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

BOOST_AUTO_TEST_CASE( nff_mount_7_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/spd3_14/mount_7", model_format_t::nff);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

BOOST_AUTO_TEST_CASE( nff_mount_9_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/spd3_14/mount_9", model_format_t::nff);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

BOOST_AUTO_TEST_CASE( nff_mount_11_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/spd3_14/mount_11", model_format_t::nff);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

BOOST_AUTO_TEST_CASE( nff_nurbtst_1_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/spd3_14/nurbtst_1", model_format_t::nff);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

BOOST_AUTO_TEST_CASE( nff_rings_1_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/spd3_14/rings_1", model_format_t::nff);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

BOOST_AUTO_TEST_CASE( nff_rings_2_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/spd3_14/rings_2", model_format_t::nff);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

BOOST_AUTO_TEST_CASE( nff_rings_4_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/spd3_14/rings_4", model_format_t::nff);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

BOOST_AUTO_TEST_CASE( nff_rings_9_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/spd3_14/rings_9", model_format_t::nff);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

BOOST_AUTO_TEST_CASE( nff_sample_1_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/spd3_14/sample_1", model_format_t::nff);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

BOOST_AUTO_TEST_CASE( nff_shells_1_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/spd3_14/shells_1", model_format_t::nff);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

BOOST_AUTO_TEST_CASE( nff_shells_2_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/spd3_14/shells_2", model_format_t::nff);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

BOOST_AUTO_TEST_CASE( nff_shells_5_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/spd3_14/shells_5", model_format_t::nff);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

BOOST_AUTO_TEST_CASE( nff_shells_9_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/spd3_14/shells_9", model_format_t::nff);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

BOOST_AUTO_TEST_CASE( nff_sombrero_2_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/spd3_14/sombrero_2", model_format_t::nff);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

BOOST_AUTO_TEST_CASE( nff_sombrero_3_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/spd3_14/sombrero_3", model_format_t::nff);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

BOOST_AUTO_TEST_CASE( nff_sombrero_5_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/spd3_14/sombrero_5", model_format_t::nff);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

BOOST_AUTO_TEST_CASE( nff_sombrero_7_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/spd3_14/sombrero_7", model_format_t::nff);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

BOOST_AUTO_TEST_CASE( nff_teapot_12_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/spd3_14/teapot_12", model_format_t::nff);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

BOOST_AUTO_TEST_CASE( nff_teapot_38_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/spd3_14/teapot_38", model_format_t::nff);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

BOOST_AUTO_TEST_CASE( nff_teapot_123_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/spd3_14/teapot_123", model_format_t::nff);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

BOOST_AUTO_TEST_CASE( nff_teapot_389_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/spd3_14/teapot_389", model_format_t::nff);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

BOOST_AUTO_TEST_CASE( nff_tetra_6_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/spd3_14/tetra_6", model_format_t::nff);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

BOOST_AUTO_TEST_CASE( nff_tetra_8_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/spd3_14/tetra_8", model_format_t::nff);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

BOOST_AUTO_TEST_CASE( nff_tetra_9_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/spd3_14/tetra_9", model_format_t::nff);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

BOOST_AUTO_TEST_CASE( nff_tetra_11_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/spd3_14/tetra_11", model_format_t::nff);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

BOOST_AUTO_TEST_CASE( nff_tree_4_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/spd3_14/tree_4", model_format_t::nff);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

BOOST_AUTO_TEST_CASE( nff_tree_7_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/spd3_14/tree_7", model_format_t::nff);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

BOOST_AUTO_TEST_CASE( nff_tree_10_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/spd3_14/tree_10", model_format_t::nff);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

BOOST_AUTO_TEST_CASE( nff_tree_14_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/spd3_14/tree_14", model_format_t::nff);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

BOOST_AUTO_TEST_SUITE_END()
