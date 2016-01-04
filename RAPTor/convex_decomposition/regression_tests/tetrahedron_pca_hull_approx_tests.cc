#ifdef STAND_ALONE
#define BOOST_TEST_MODULE regression test
#endif /* #ifdef STAND_ALONE */


/* Standard headers */

/* Boost headers */
#include "boost/test/unit_test.hpp"

/* Convex Decomposition headers */
#include "convex_decomposition.h"

/* Test headers */
#include "regression_fixture.h"


namespace raptor_convex_decomposition
{
namespace test
{
/* Test data */
struct tetrahedron_pca_hull_approx_regression_fixture : public regression_fixture
{
    tetrahedron_pca_hull_approx_regression_fixture() : 
        regression_fixture(convex_decomposition_options(0.001f, 0.05f, 0.05f, 0.0005f, 0.05f, 0.0001f, 100000, 0, 20, 4, 4, discretisation_type_t::tetrahedron, true, true))
    {  }
};


BOOST_FIXTURE_TEST_SUITE( regression_tests, tetrahedron_pca_hull_approx_regression_fixture )

const float result_tolerance = 0.0005f;


/* Tests */
// BOOST_AUTO_TEST_CASE( block_tetrahedron_pca_hull_approx_test )
// {
//     /* Load file */
//     BOOST_REQUIRE(load_off("../../v-hacd/data/block.off", &points, &triangles));

//     /* Decompose */
//     const convex_decomposition uut(points, triangles, options);

//     /* Save resutls */
//     check(uut, "block_tetrahedron_pca_hull_approx");
// }

// BOOST_AUTO_TEST_CASE( bunny_tetrahedron_pca_hull_approx_test )
// {
//     /* Load file */
//     BOOST_REQUIRE(load_off("../../v-hacd/data/bunny.off", &points, &triangles));

//     /* Decompose */
//     const convex_decomposition uut(points, triangles, options);

//     /* Save resutls */
//     check(uut, "bunny_tetrahedron_pca_hull_approx");
// }

// BOOST_AUTO_TEST_CASE( camel_tetrahedron_pca_hull_approx_test )
// {
//     /* Load file */
//     BOOST_REQUIRE(load_off("../../v-hacd/data/camel.off", &points, &triangles));

//     /* Decompose */
//     const convex_decomposition uut(points, triangles, options);

//     /* Save resutls */
//     check(uut, "camel_tetrahedron_pca_hull_approx");
// }

// BOOST_AUTO_TEST_CASE( casting_tetrahedron_pca_hull_approx_test )
// {
//     /* Load file */
//     BOOST_REQUIRE(load_off("../../v-hacd/data/casting.off", &points, &triangles));

//     /* Decompose */
//     const convex_decomposition uut(points, triangles, options);

//     /* Save resutls */
//     check(uut, "casting_tetrahedron_pca_hull_approx");
// }

// BOOST_AUTO_TEST_CASE( chair_tetrahedron_pca_hull_approx_test )
// {
//     /* Load file */
//     BOOST_REQUIRE(load_off("../../v-hacd/data/chair.off", &points, &triangles));

//     /* Decompose */
//     const convex_decomposition uut(points, triangles, options);

//     /* Save resutls */
//     check(uut, "chair_tetrahedron_pca_hull_approx");
// }

// BOOST_AUTO_TEST_CASE( cow1_tetrahedron_pca_hull_approx_test )
// {
//     /* Load file */
//     BOOST_REQUIRE(load_off("../../v-hacd/data/cow1.off", &points, &triangles));

//     /* Decompose */
//     const convex_decomposition uut(points, triangles, options);

//     /* Save resutls */
//     check(uut, "cow1_tetrahedron_pca_hull_approx");
// }

// BOOST_AUTO_TEST_CASE( cow2_tetrahedron_pca_hull_approx_test )
// {
//     /* Load file */
//     BOOST_REQUIRE(load_off("../../v-hacd/data/cow2.off", &points, &triangles));

//     /* Decompose */
//     const convex_decomposition uut(points, triangles, options);

//     /* Save resutls */
//     check(uut, "cow2_tetrahedron_pca_hull_approx");
// }

// BOOST_AUTO_TEST_CASE( crank_tetrahedron_pca_hull_approx_test )
// {
//     /* Load file */
//     BOOST_REQUIRE(load_off("../../v-hacd/data/crank.off", &points, &triangles));

//     /* Decompose */
//     const convex_decomposition uut(points, triangles, options);

//     /* Save resutls */
//     check(uut, "crank_tetrahedron_pca_hull_approx");
// }

// BOOST_AUTO_TEST_CASE( cup_tetrahedron_pca_hull_approx_test )
// {
//     /* Load file */
//     BOOST_REQUIRE(load_off("../../v-hacd/data/cup.off", &points, &triangles));

//     /* Decompose */
//     const convex_decomposition uut(points, triangles, options);

//     /* Save resutls */
//     check(uut, "cup_tetrahedron_pca_hull_approx");
// }

// BOOST_AUTO_TEST_CASE( dancer2_tetrahedron_pca_hull_approx_test )
// {
//     /* Load file */
//     BOOST_REQUIRE(load_off("../../v-hacd/data/dancer2.off", &points, &triangles));

//     /* Decompose */
//     const convex_decomposition uut(points, triangles, options);

//     /* Save resutls */
//     check(uut, "dancer2_tetrahedron_pca_hull_approx");
// }

// BOOST_AUTO_TEST_CASE( deer_bound_tetrahedron_pca_hull_approx_test )
// {
//     /* Load file */
//     BOOST_REQUIRE(load_off("../../v-hacd/data/deer_bound.off", &points, &triangles));

//     /* Decompose */
//     const convex_decomposition uut(points, triangles, options);

//     /* Save resutls */
//     check(uut, "deer_bound_tetrahedron_pca_hull_approx");
// }

// BOOST_AUTO_TEST_CASE( dilo_tetrahedron_pca_hull_approx_test )
// {
//     /* Load file */
//     BOOST_REQUIRE(load_off("../../v-hacd/data/dilo.off", &points, &triangles));

//     /* Decompose */
//     const convex_decomposition uut(points, triangles, options);

//     /* Save resutls */
//     check(uut, "dilo_tetrahedron_pca_hull_approx");
// }

// BOOST_AUTO_TEST_CASE( dino_tetrahedron_pca_hull_approx_test )
// {
//     /* Load file */
//     BOOST_REQUIRE(load_off("../../v-hacd/data/dino.off", &points, &triangles));

//     /* Decompose */
//     const convex_decomposition uut(points, triangles, options);

//     /* Save resutls */
//     check(uut, "dino_tetrahedron_pca_hull_approx");
// }

// BOOST_AUTO_TEST_CASE( DRAGON_F_tetrahedron_pca_hull_approx_test )
// {
//     /* Load file */
//     BOOST_REQUIRE(load_off("../../v-hacd/data/DRAGON_F.off", &points, &triangles));

//     /* Decompose */
//     const convex_decomposition uut(points, triangles, options);

//     /* Save resutls */
//     check(uut, "DRAGON_F_tetrahedron_pca_hull_approx");
// }

// BOOST_AUTO_TEST_CASE( drum_tetrahedron_pca_hull_approx_test )
// {
//     /* Load file */
//     BOOST_REQUIRE(load_off("../../v-hacd/data/drum.off", &points, &triangles));

//     /* Decompose */
//     const convex_decomposition uut(points, triangles, options);

//     /* Save resutls */
//     check(uut, "drum_tetrahedron_pca_hull_approx");
// }

// BOOST_AUTO_TEST_CASE( eight_tetrahedron_pca_hull_approx_test )
// {
//     /* Load file */
//     BOOST_REQUIRE(load_off("../../v-hacd/data/eight.off", &points, &triangles));

//     /* Decompose */
//     const convex_decomposition uut(points, triangles, options);

//     /* Save resutls */
//     check(uut, "eight_tetrahedron_pca_hull_approx");
// }

// BOOST_AUTO_TEST_CASE( elephant_tetrahedron_pca_hull_approx_test )
// {
//     /* Load file */
//     BOOST_REQUIRE(load_off("../../v-hacd/data/elephant.off", &points, &triangles));

//     /* Decompose */
//     const convex_decomposition uut(points, triangles, options);

//     /* Save resutls */
//     check(uut, "elephant_tetrahedron_pca_hull_approx");
// }

// BOOST_AUTO_TEST_CASE( elk_tetrahedron_pca_hull_approx_test )
// {
//     /* Load file */
//     BOOST_REQUIRE(load_off("../../v-hacd/data/elk.off", &points, &triangles));

//     /* Decompose */
//     const convex_decomposition uut(points, triangles, options);

//     /* Save resutls */
//     check(uut, "elk_tetrahedron_pca_hull_approx");
// }

// BOOST_AUTO_TEST_CASE( egea_tetrahedron_pca_hull_approx_test )
// {
//     /* Load file */
//     BOOST_REQUIRE(load_off("../../v-hacd/data/egea.off", &points, &triangles));

//     /* Decompose */
//     const convex_decomposition uut(points, triangles, options);

//     /* Save resutls */
//     check(uut, "egea_tetrahedron_pca_hull_approx");
// }

// BOOST_AUTO_TEST_CASE( face_yh_tetrahedron_pca_hull_approx_test_test )
// {
//     /* Load file */
//     BOOST_REQUIRE(load_off("../../v-hacd/data/face-YH.off", &points, &triangles));

//     /* Decompose */
//     const convex_decomposition uut(points, triangles, options);

//     /* Save resutls */
//     check(uut, "face-YH_tetrahedron_pca_hull_approx");
// }

// BOOST_AUTO_TEST_CASE( feline_tetrahedron_pca_hull_approx_test )
// {
//     /* Load file */
//     BOOST_REQUIRE(load_off("../../v-hacd/data/feline.off", &points, &triangles));

//     /* Decompose */
//     const convex_decomposition uut(points, triangles, options);

//     /* Save resutls */
//     check(uut, "feline_tetrahedron_pca_hull_approx");
// }

// BOOST_AUTO_TEST_CASE( fish_tetrahedron_pca_hull_approx_test )
// {
//     /* Load file */
//     BOOST_REQUIRE(load_off("../../v-hacd/data/fish.off", &points, &triangles));

//     /* Decompose */
//     const convex_decomposition uut(points, triangles, options);

//     /* Save resutls */
//     check(uut, "fish_tetrahedron_pca_hull_approx");
// }

// BOOST_AUTO_TEST_CASE( foot_tetrahedron_pca_hull_approx_test )
// {
//     /* Load file */
//     BOOST_REQUIRE(load_off("../../v-hacd/data/foot.off", &points, &triangles));

//     /* Decompose */
//     const convex_decomposition uut(points, triangles, options);

//     /* Save resutls */
//     check(uut, "foot_tetrahedron_pca_hull_approx");
// }

// BOOST_AUTO_TEST_CASE( genus3_tetrahedron_pca_hull_approx_test )
// {
//     /* Load file */
//     BOOST_REQUIRE(load_off("../../v-hacd/data/genus3.off", &points, &triangles));

//     /* Decompose */
//     const convex_decomposition uut(points, triangles, options);

//     /* Save resutls */
//     check(uut, "genus3_tetrahedron_pca_hull_approx");
// }

// BOOST_AUTO_TEST_CASE( greek_sculpture_tetrahedron_pca_hull_approx_test )
// {
//     /* Load file */
//     BOOST_REQUIRE(load_off("../../v-hacd/data/greek_sculpture.off", &points, &triangles));

//     /* Decompose */
//     const convex_decomposition uut(points, triangles, options);

//     /* Save resutls */
//     check(uut, "greek_sculpture_tetrahedron_pca_hull_approx");
// }

// BOOST_AUTO_TEST_CASE( Hand1_tetrahedron_pca_hull_approx_test )
// {
//     /* Load file */
//     BOOST_REQUIRE(load_off("../../v-hacd/data/Hand1.off", &points, &triangles));

//     /* Decompose */
//     const convex_decomposition uut(points, triangles, options);

//     /* Save resutls */
//     check(uut, "Hand1_tetrahedron_pca_hull_approx");
// }

// BOOST_AUTO_TEST_CASE( hand2_tetrahedron_pca_hull_approx_test )
// {
//     /* Load file */
//     BOOST_REQUIRE(load_off("../../v-hacd/data/hand2.off", &points, &triangles));

//     /* Decompose */
//     const convex_decomposition uut(points, triangles, options);

//     /* Save resutls */
//     check(uut, "hand2_tetrahedron_pca_hull_approx");
// }

// BOOST_AUTO_TEST_CASE( horse_tetrahedron_pca_hull_approx_test )
// {
//     /* Load file */
//     BOOST_REQUIRE(load_off("../../v-hacd/data/horse.off", &points, &triangles));

//     /* Decompose */
//     const convex_decomposition uut(points, triangles, options);

//     /* Save resutls */
//     check(uut, "horse_tetrahedron_pca_hull_approx");
// }

// BOOST_AUTO_TEST_CASE( helix_tetrahedron_pca_hull_approx_test )
// {
//     /* Load file */
//     BOOST_REQUIRE(load_off("../../v-hacd/data/helix.off", &points, &triangles));

//     /* Decompose */
//     const convex_decomposition uut(points, triangles, options);

//     /* Save resutls */
//     check(uut, "helix_tetrahedron_pca_hull_approx");
// }

// BOOST_AUTO_TEST_CASE( helmet_tetrahedron_pca_hull_approx_test )
// {
//     /* Load file */
//     BOOST_REQUIRE(load_off("../../v-hacd/data/helmet.off", &points, &triangles));

//     /* Decompose */
//     const convex_decomposition uut(points, triangles, options);

//     /* Save resutls */
//     check(uut, "helmet_tetrahedron_pca_hull_approx");
// }

// BOOST_AUTO_TEST_CASE( hero_tetrahedron_pca_hull_approx_test )
// {
//     /* Load file */
//     BOOST_REQUIRE(load_off("../../v-hacd/data/hero.off", &points, &triangles));

//     /* Decompose */
//     const convex_decomposition uut(points, triangles, options);

//      Save resutls 
//     check(uut, "hero_tetrahedron_pca_hull_approx");
// }

// BOOST_AUTO_TEST_CASE( homer_tetrahedron_pca_hull_approx_test )
// {
//     /* Load file */
//     BOOST_REQUIRE(load_off("../../v-hacd/data/homer.off", &points, &triangles));

//     /* Decompose */
//     const convex_decomposition uut(points, triangles, options);

//     /* Save resutls */
//     check(uut, "homer_tetrahedron_pca_hull_approx");
// }

// BOOST_AUTO_TEST_CASE( hornbug_tetrahedron_pca_hull_approx_test )
// {
//     /* Load file */
//     BOOST_REQUIRE(load_off("../../v-hacd/data/hornbug.off", &points, &triangles));

//     /* Decompose */
//     const convex_decomposition uut(points, triangles, options);

//     /* Save resutls */
//     check(uut, "hornbug_tetrahedron_pca_hull_approx");
// }

// BOOST_AUTO_TEST_CASE( maneki_neko_tetrahedron_pca_hull_approx_test )
// {
//     /* Load file */
//     BOOST_REQUIRE(load_off("../../v-hacd/data/maneki-neko.off", &points, &triangles));

//     /* Decompose */
//     const convex_decomposition uut(points, triangles, options);

//     /* Save resutls */
//     check(uut, "maneki-neko_tetrahedron_pca_hull_approx");
// }

// BOOST_AUTO_TEST_CASE( mannequin_devil_tetrahedron_pca_hull_approx_test )
// {
//     /* Load file */
//     BOOST_REQUIRE(load_off("../../v-hacd/data/mannequin-devil.off", &points, &triangles));

//     /* Decompose */
//     const convex_decomposition uut(points, triangles, options);

//     /* Save resutls */
//     check(uut, "mannequin-devil_tetrahedron_pca_hull_approx");
// }

// BOOST_AUTO_TEST_CASE( mannequin_tetrahedron_pca_hull_approx_test )
// {
//     /* Load file */
//     BOOST_REQUIRE(load_off("../../v-hacd/data/mannequin.off", &points, &triangles));

//     /* Decompose */
//     const convex_decomposition uut(points, triangles, options);

//     /* Save resutls */
//     check(uut, "mannequin_tetrahedron_pca_hull_approx");
// }

// BOOST_AUTO_TEST_CASE( mask_tetrahedron_pca_hull_approx_test )
// {
//     /* Load file */
//     BOOST_REQUIRE(load_off("../../v-hacd/data/mask.off", &points, &triangles));

//     /* Decompose */
//     const convex_decomposition uut(points, triangles, options);

//     /* Save resutls */
//     check(uut, "mask_tetrahedron_pca_hull_approx");
// }

// BOOST_AUTO_TEST_CASE( moaimoai_tetrahedron_pca_hull_approx_test )
// {
//     /* Load file */
//     BOOST_REQUIRE(load_off("../../v-hacd/data/moaimoai.off", &points, &triangles));

//     /* Decompose */
//     const convex_decomposition uut(points, triangles, options);

//     /* Save resutls */
//     check(uut, "moaimoai_tetrahedron_pca_hull_approx");
// }

// BOOST_AUTO_TEST_CASE( monk_tetrahedron_pca_hull_approx_test )
// {
//     /* Load file */
//     BOOST_REQUIRE(load_off("../../v-hacd/data/monk.off", &points, &triangles));

//     /* Decompose */
//     const convex_decomposition uut(points, triangles, options);

//     /* Save resutls */
//     check(uut, "monk_tetrahedron_pca_hull_approx");
// }

// BOOST_AUTO_TEST_CASE( octopus_tetrahedron_pca_hull_approx_test )
// {
//     /* Load file */
//     BOOST_REQUIRE(load_off("../../v-hacd/data/octopus.off", &points, &triangles));

//     /* Decompose */
//     const convex_decomposition uut(points, triangles, options);

//     /* Save resutls */
//     check(uut, "octopus_tetrahedron_pca_hull_approx");
// }

// BOOST_AUTO_TEST_CASE( pig_tetrahedron_pca_hull_approx_test )
// {
//     /* Load file */
//     BOOST_REQUIRE(load_off("../../v-hacd/data/pig.off", &points, &triangles));

//     /* Decompose */
//     const convex_decomposition uut(points, triangles, options);

//     /* Save resutls */
//     check(uut, "pig_tetrahedron_pca_hull_approx");
// }

// BOOST_AUTO_TEST_CASE( pinocchio_b_tetrahedron_pca_hull_approx_test )
// {
//     /* Load file */
//     BOOST_REQUIRE(load_off("../../v-hacd/data/pinocchio_b.off", &points, &triangles));

//     /* Decompose */
//     const convex_decomposition uut(points, triangles, options);

//     /* Save resutls */
//     check(uut, "pinocchio_b_tetrahedron_pca_hull_approx");
// }

// BOOST_AUTO_TEST_CASE( polygirl_tetrahedron_pca_hull_approx_test )
// {
//     /* Load file */
//     BOOST_REQUIRE(load_off("../../v-hacd/data/polygirl.off", &points, &triangles));

//     /* Decompose */
//     const convex_decomposition uut(points, triangles, options);

//     /* Save resutls */
//     check(uut, "polygirl_tetrahedron_pca_hull_approx");
// }

// BOOST_AUTO_TEST_CASE( rabbit_tetrahedron_pca_hull_approx_test )
// {
//     /* Load file */
//     BOOST_REQUIRE(load_off("../../v-hacd/data/rabbit.off", &points, &triangles));

//     /* Decompose */
//     const convex_decomposition uut(points, triangles, options);

//     /* Save resutls */
//     check(uut, "rabbit_tetrahedron_pca_hull_approx");
// }

// BOOST_AUTO_TEST_CASE( rocker_arm_tetrahedron_pca_hull_approx_test )
// {
//     /* Load file */
//     BOOST_REQUIRE(load_off("../../v-hacd/data/rocker-arm.off", &points, &triangles));

//     /* Decompose */
//     const convex_decomposition uut(points, triangles, options);

//     /* Save resutls */
//     check(uut, "rocker-arm_tetrahedron_pca_hull_approx");
// }

// BOOST_AUTO_TEST_CASE( screwdriver_tetrahedron_pca_hull_approx_test )
// {
//     /* Load file */
//     BOOST_REQUIRE(load_off("../../v-hacd/data/screwdriver.off", &points, &triangles));

//     /* Decompose */
//     const convex_decomposition uut(points, triangles, options);

//     /* Save resutls */
//     check(uut, "screwdriver_tetrahedron_pca_hull_approx");
// }

// BOOST_AUTO_TEST_CASE( shark_b_tetrahedron_pca_hull_approx_test )
// {
//     /* Load file */
//     BOOST_REQUIRE(load_off("../../v-hacd/data/shark_b.off", &points, &triangles));

//     /* Decompose */
//     const convex_decomposition uut(points, triangles, options);

//     /* Save resutls */
//     check(uut, "shark_b_tetrahedron_pca_hull_approx");
// }

// BOOST_AUTO_TEST_CASE( sketched_brunnen_tetrahedron_pca_hull_approx_test )
// {
//     /* Load file */
//     BOOST_REQUIRE(load_off("../../v-hacd/data/Sketched-Brunnen.off", &points, &triangles));

//     /* Decompose */
//     const convex_decomposition uut(points, triangles, options);

//     /* Save resutls */
//     check(uut, "Sketched-Brunnen_tetrahedron_pca_hull_approx");
// }

// BOOST_AUTO_TEST_CASE( sledge_tetrahedron_pca_hull_approx_test )
// {
//     /* Load file */
//     BOOST_REQUIRE(load_off("../../v-hacd/data/sledge.off", &points, &triangles));

//     /* Decompose */
//     const convex_decomposition uut(points, triangles, options);

//     /* Save resutls */
//     check(uut, "sledge_tetrahedron_pca_hull_approx");
// }

// BOOST_AUTO_TEST_CASE( squirrel_tetrahedron_pca_hull_approx_test )
// {
//     /* Load file */
//     BOOST_REQUIRE(load_off("../../v-hacd/data/squirrel.off", &points, &triangles));

//     /* Decompose */
//     const convex_decomposition uut(points, triangles, options);

//     /* Save resutls */
//     check(uut, "squirrel_tetrahedron_pca_hull_approx");
// }

// BOOST_AUTO_TEST_CASE( sword_tetrahedron_pca_hull_approx_test )
// {
//     /* Load file */
//     BOOST_REQUIRE(load_off("../../v-hacd/data/sword.off", &points, &triangles));

//     /* Decompose */
//     const convex_decomposition uut(points, triangles, options);

//     /* Save resutls */
//     check(uut, "sword_tetrahedron_pca_hull_approx");
// }

// BOOST_AUTO_TEST_CASE( table_tetrahedron_pca_hull_approx_test )
// {
//     /* Load file */
//     BOOST_REQUIRE(load_off("../../v-hacd/data/table.off", &points, &triangles));

//     /* Decompose */
//     const convex_decomposition uut(points, triangles, options);

//     /* Save resutls */
//     check(uut, "table_tetrahedron_pca_hull_approx");
// }

// BOOST_AUTO_TEST_CASE( Teapot_tetrahedron_pca_hull_approx_test )
// {
//     /* Load file */
//     BOOST_REQUIRE(load_off("../../v-hacd/data/Teapot.off", &points, &triangles));

//     /* Decompose */
//     const convex_decomposition uut(points, triangles, options);

//     /* Save resutls */
//     check(uut, "Teapot_tetrahedron_pca_hull_approx");
// }

// BOOST_AUTO_TEST_CASE( test2_tetrahedron_pca_hull_approx_test )
// {
//     /* Load file */
//     BOOST_REQUIRE(load_off("../../v-hacd/data/test2.off", &points, &triangles));

//     /* Decompose */
//     const convex_decomposition uut(points, triangles, options);

//     /* Save resutls */
//     check(uut, "test2_tetrahedron_pca_hull_approx");
// }

// BOOST_AUTO_TEST_CASE( test_tetrahedron_pca_hull_approx_test )
// {
//     /* Load file */
//     BOOST_REQUIRE(load_off("../../v-hacd/data/test.off", &points, &triangles));

//     /* Decompose */
//     const convex_decomposition uut(points, triangles, options);

//     /* Save resutls */
//     check(uut, "test_tetrahedron_pca_hull_approx");
// }

// BOOST_AUTO_TEST_CASE( torus_tetrahedron_pca_hull_approx_test )
// {
//     /* Load file */
//     BOOST_REQUIRE(load_off("../../v-hacd/data/torus.off", &points, &triangles));

//     /* Decompose */
//     const convex_decomposition uut(points, triangles, options);

//     /* Save resutls */
//     check(uut, "torus_tetrahedron_pca_hull_approx");
// }

// BOOST_AUTO_TEST_CASE( tstTorusModel3_tetrahedron_pca_hull_approx_test )
// {
//     /* Load file */
//     BOOST_REQUIRE(load_off("../../v-hacd/data/tstTorusModel3.off", &points, &triangles));

//     /* Decompose */
//     const convex_decomposition uut(points, triangles, options);

//     /* Save resutls */
//     check(uut, "tstTorusModel3_tetrahedron_pca_hull_approx");
// }

// BOOST_AUTO_TEST_CASE( tstTorusModel_tetrahedron_pca_hull_approx_test )
// {
//     /* Load file */
//     BOOST_REQUIRE(load_off("../../v-hacd/data/tstTorusModel.off", &points, &triangles));

//     /* Decompose */
//     const convex_decomposition uut(points, triangles, options);

//     /* Save resutls */
//     check(uut, "tstTorusModel_tetrahedron_pca_hull_approx");
// }

// BOOST_AUTO_TEST_CASE( tube1_tetrahedron_pca_hull_approx_test )
// {
//     /* Load file */
//     BOOST_REQUIRE(load_off("../../v-hacd/data/tube1.off", &points, &triangles));

//     /* Decompose */
//     const convex_decomposition uut(points, triangles, options);

//     /* Save resutls */
//     check(uut, "tube1_tetrahedron_pca_hull_approx");
// }

// BOOST_AUTO_TEST_CASE( venus_tetrahedron_pca_hull_approx_test )
// {
//     /* Load file */
//     BOOST_REQUIRE(load_off("../../v-hacd/data/venus.off", &points, &triangles));

//     /* Decompose */
//     const convex_decomposition uut(points, triangles, options);

//     /* Save resutls */
//     check(uut, "venus_tetrahedron_pca_hull_approx");
// }

// BOOST_AUTO_TEST_CASE( venus_original_tetrahedron_pca_hull_approx_test )
// {
//     /* Load file */
//     BOOST_REQUIRE(load_off("../../v-hacd/data/venus-original.off", &points, &triangles));

//     /* Decompose */
//     const convex_decomposition uut(points, triangles, options);

//     /* Save resutls */
//     check(uut, "venus-original_tetrahedron_pca_hull_approx");
// }

BOOST_AUTO_TEST_SUITE_END()
}; /* namespace test */
}; /* namespace raptor_convex_decomposition */
