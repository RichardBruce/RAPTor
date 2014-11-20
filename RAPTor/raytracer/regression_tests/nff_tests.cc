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

#ifndef VALGRIND_TESTS
BOOST_AUTO_TEST_CASE( nff_balls_1_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/spd3_14/balls_scenes/balls_1.nff", model_format_t::nff);

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
    regression_fixture fixture("/spd3_14/balls_scenes/balls_2.nff", model_format_t::nff);

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
    regression_fixture fixture("/spd3_14/balls_scenes/balls_3.nff", model_format_t::nff);

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
    regression_fixture fixture("/spd3_14/balls_scenes/balls_4.nff", model_format_t::nff);

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
    regression_fixture fixture("/spd3_14/gears_scenes/gears_2.nff", model_format_t::nff);

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
    regression_fixture fixture("/spd3_14/gears_scenes/gears_5.nff", model_format_t::nff);

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
    regression_fixture fixture("/spd3_14/gears_scenes/gears_12.nff", model_format_t::nff);

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
    regression_fixture fixture("/spd3_14/gears_scenes/gears_25.nff", model_format_t::nff);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}
#endif /* #ifndef VALGRIND_TESTS */

BOOST_AUTO_TEST_CASE( nff_jacks_2_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/spd3_14/jacks_scenes/jacks_2.nff", model_format_t::nff);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

#ifndef VALGRIND_TESTS
BOOST_AUTO_TEST_CASE( nff_jacks_3_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/spd3_14/jacks_scenes/jacks_3.nff", model_format_t::nff);

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
    regression_fixture fixture("/spd3_14/jacks_scenes/jacks_4.nff", model_format_t::nff);

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
    regression_fixture fixture("/spd3_14/jacks_scenes/jacks_5.nff", model_format_t::nff);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}
#endif /* #ifndef VALGRIND_TESTS */

BOOST_AUTO_TEST_CASE( nff_lattice_1_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/spd3_14/lattice_scenes/lattice_1.nff", model_format_t::nff);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

#ifndef VALGRIND_TESTS
BOOST_AUTO_TEST_CASE( nff_lattice_3_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/spd3_14/lattice_scenes/lattice_3.nff", model_format_t::nff);

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
    regression_fixture fixture("/spd3_14/lattice_scenes/lattice_8.nff", model_format_t::nff);

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
    regression_fixture fixture("/spd3_14/lattice_scenes/lattice_19.nff", model_format_t::nff);

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
    regression_fixture fixture("/spd3_14/mount_scenes/mount_4.nff", model_format_t::nff);

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
    regression_fixture fixture("/spd3_14/mount_scenes/mount_7.nff", model_format_t::nff);

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
    regression_fixture fixture("/spd3_14/mount_scenes/mount_9.nff", model_format_t::nff);

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
    regression_fixture fixture("/spd3_14/mount_scenes/mount_11.nff", model_format_t::nff);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}
#endif /* #ifndef VALGRIND_TESTS */

BOOST_AUTO_TEST_CASE( nff_nurbtst_1_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/spd3_14/nurbtst_scenes/nurbtst_1.nff", model_format_t::nff);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

#ifndef VALGRIND_TESTS
BOOST_AUTO_TEST_CASE( nff_rings_1_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/spd3_14/rings_scenes/rings_1.nff", model_format_t::nff);

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
    regression_fixture fixture("/spd3_14/rings_scenes/rings_2.nff", model_format_t::nff);

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
    regression_fixture fixture("/spd3_14/rings_scenes/rings_4.nff", model_format_t::nff);

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
    regression_fixture fixture("/spd3_14/rings_scenes/rings_9.nff", model_format_t::nff);

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
    regression_fixture fixture("/spd3_14/sample_scenes/sample_1.nff", model_format_t::nff);

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
    regression_fixture fixture("/spd3_14/shells_scenes/shells_1.nff", model_format_t::nff);

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
    regression_fixture fixture("/spd3_14/shells_scenes/shells_2.nff", model_format_t::nff);

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
    regression_fixture fixture("/spd3_14/shells_scenes/shells_5.nff", model_format_t::nff);

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
    regression_fixture fixture("/spd3_14/shells_scenes/shells_9.nff", model_format_t::nff);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}
#endif /* #ifndef VALGRIND_TESTS */

BOOST_AUTO_TEST_CASE( nff_sombrero_2_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/spd3_14/sombrero_scenes/sombrero_2.nff", model_format_t::nff);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

#ifndef VALGRIND_TESTS
BOOST_AUTO_TEST_CASE( nff_sombrero_3_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/spd3_14/sombrero_scenes/sombrero_3.nff", model_format_t::nff);

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
    regression_fixture fixture("/spd3_14/sombrero_scenes/sombrero_5.nff", model_format_t::nff);

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
    regression_fixture fixture("/spd3_14/sombrero_scenes/sombrero_7.nff", model_format_t::nff);

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
    regression_fixture fixture("/spd3_14/teapot_scenes/teapot_12.nff", model_format_t::nff);

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
    regression_fixture fixture("/spd3_14/teapot_scenes/teapot_38.nff", model_format_t::nff);

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
    regression_fixture fixture("/spd3_14/teapot_scenes/teapot_123.nff", model_format_t::nff);

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
    regression_fixture fixture("/spd3_14/teapot_scenes/teapot_389.nff", model_format_t::nff);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}
#endif /* #ifndef VALGRIND_TESTS */

BOOST_AUTO_TEST_CASE( nff_tetra_6_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/spd3_14/tetra_scenes/tetra_6.nff", model_format_t::nff);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}

#ifndef VALGRIND_TESTS
BOOST_AUTO_TEST_CASE( nff_tetra_8_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/spd3_14/tetra_scenes/tetra_8.nff", model_format_t::nff);

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
    regression_fixture fixture("/spd3_14/tetra_scenes/tetra_9.nff", model_format_t::nff);

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
    regression_fixture fixture("/spd3_14/tetra_scenes/tetra_11.nff", model_format_t::nff);

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
    regression_fixture fixture("/spd3_14/tree_scenes/tree_4.nff", model_format_t::nff);

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
    regression_fixture fixture("/spd3_14/tree_scenes/tree_7.nff", model_format_t::nff);

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
    regression_fixture fixture("/spd3_14/tree_scenes/tree_10.nff", model_format_t::nff);

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
    regression_fixture fixture("/spd3_14/tree_scenes/tree_14.nff", model_format_t::nff);

    /* Ray trace the scene */
    fixture.render();

    /* Check image */
    checker.check(fixture.get_camera());
}
#endif /* #ifndef VALGRIND_TESTS */

BOOST_AUTO_TEST_SUITE_END()
