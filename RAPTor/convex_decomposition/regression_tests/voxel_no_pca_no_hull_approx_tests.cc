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
struct voxel_no_pca_no_hull_approx_regression_fixture : public regression_fixture
{
    voxel_no_pca_no_hull_approx_regression_fixture() : 
        regression_fixture(convex_decomposition_options(0.001f, 0.05f, 0.05f, 0.0005f, 0.0001f, 100000, 0, 20, 4, 4, discretisation_type_t::voxel, false, false))
    {  }
};


BOOST_FIXTURE_TEST_SUITE( regression_tests, voxel_no_pca_no_hull_approx_regression_fixture )

const float result_tolerance = 0.0005f;


/* Tests */
#ifndef VALGRIND_TESTS
BOOST_AUTO_TEST_CASE( block_voxel_no_pca_no_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/block.off");

    /* Decompose */
    const convex_decomposition uut(points, triangles, options);

    /* Save resutls */
    check(uut, "block_voxel_no_pca_no_hull_approx");
}
#endif /* #ifndef VALGRIND_TESTS */

BOOST_AUTO_TEST_CASE( bunny_voxel_no_pca_no_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/bunny.off");

    /* Decompose */
    const convex_decomposition uut(points, triangles, options);

    /* Save resutls */
    check(uut, "bunny_voxel_no_pca_no_hull_approx");
}

#ifndef VALGRIND_TESTS
BOOST_AUTO_TEST_CASE( camel_voxel_no_pca_no_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/camel.off");

    /* Decompose */
    const convex_decomposition uut(points, triangles, options);

    /* Save resutls */
    check(uut, "camel_voxel_no_pca_no_hull_approx");
}

BOOST_AUTO_TEST_CASE( casting_voxel_no_pca_no_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/casting.off");

    /* Decompose */
    const convex_decomposition uut(points, triangles, options);

    /* Save resutls */
    check(uut, "casting_voxel_no_pca_no_hull_approx");
}

BOOST_AUTO_TEST_CASE( chair_voxel_no_pca_no_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/chair.off");

    /* Decompose */
    const convex_decomposition uut(points, triangles, options);

    /* Save resutls */
    check(uut, "chair_voxel_no_pca_no_hull_approx");
}

BOOST_AUTO_TEST_CASE( cow1_voxel_no_pca_no_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/cow1.off");

    /* Decompose */
    const convex_decomposition uut(points, triangles, options);

    /* Save resutls */
    check(uut, "cow1_voxel_no_pca_no_hull_approx");
}

BOOST_AUTO_TEST_CASE( cow2_voxel_no_pca_no_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/cow2.off");

    /* Decompose */
    const convex_decomposition uut(points, triangles, options);

    /* Save resutls */
    check(uut, "cow2_voxel_no_pca_no_hull_approx");
}

BOOST_AUTO_TEST_CASE( crank_voxel_no_pca_no_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/crank.off");

    /* Decompose */
    const convex_decomposition uut(points, triangles, options);

    /* Save resutls */
    check(uut, "crank_voxel_no_pca_no_hull_approx");
}

BOOST_AUTO_TEST_CASE( cup_voxel_no_pca_no_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/cup.off");

    /* Decompose */
    const convex_decomposition uut(points, triangles, options);

    /* Save resutls */
    check(uut, "cup_voxel_no_pca_no_hull_approx");
}

BOOST_AUTO_TEST_CASE( dancer2_voxel_no_pca_no_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/dancer2.off");

    /* Decompose */
    const convex_decomposition uut(points, triangles, options);

    /* Save resutls */
    check(uut, "dancer2_voxel_no_pca_no_hull_approx");
}

BOOST_AUTO_TEST_CASE( deer_bound_voxel_no_pca_no_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/deer_bound.off");

    /* Decompose */
    const convex_decomposition uut(points, triangles, options);

    /* Save resutls */
    check(uut, "deer_bound_voxel_no_pca_no_hull_approx");
}

BOOST_AUTO_TEST_CASE( dilo_voxel_no_pca_no_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/dilo.off");

    /* Decompose */
    const convex_decomposition uut(points, triangles, options);

    /* Save resutls */
    check(uut, "dilo_voxel_no_pca_no_hull_approx");
}

BOOST_AUTO_TEST_CASE( dino_voxel_no_pca_no_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/dino.off");

    /* Decompose */
    const convex_decomposition uut(points, triangles, options);

    /* Save resutls */
    check(uut, "dino_voxel_no_pca_no_hull_approx");
}

BOOST_AUTO_TEST_CASE( DRAGON_F_voxel_no_pca_no_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/DRAGON_F.off");

    /* Decompose */
    const convex_decomposition uut(points, triangles, options);

    /* Save resutls */
    check(uut, "DRAGON_F_voxel_no_pca_no_hull_approx");
}

BOOST_AUTO_TEST_CASE( drum_voxel_no_pca_no_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/drum.off");

    /* Decompose */
    const convex_decomposition uut(points, triangles, options);

    /* Save resutls */
    check(uut, "drum_voxel_no_pca_no_hull_approx");
}

BOOST_AUTO_TEST_CASE( eight_voxel_no_pca_no_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/eight.off");

    /* Decompose */
    const convex_decomposition uut(points, triangles, options);

    /* Save resutls */
    check(uut, "eight_voxel_no_pca_no_hull_approx");
}

BOOST_AUTO_TEST_CASE( elephant_voxel_no_pca_no_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/elephant.off");

    /* Decompose */
    const convex_decomposition uut(points, triangles, options);

    /* Save resutls */
    check(uut, "elephant_voxel_no_pca_no_hull_approx");
}

BOOST_AUTO_TEST_CASE( elk_voxel_no_pca_no_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/elk.off");

    /* Decompose */
    const convex_decomposition uut(points, triangles, options);

    /* Save resutls */
    check(uut, "elk_voxel_no_pca_no_hull_approx");
}

BOOST_AUTO_TEST_CASE( egea_voxel_no_pca_no_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/egea.off");

    /* Decompose */
    const convex_decomposition uut(points, triangles, options);

    /* Save resutls */
    check(uut, "egea_voxel_no_pca_no_hull_approx");
}

BOOST_AUTO_TEST_CASE( face_yh_voxel_no_pca_no_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/face-YH.off");

    /* Decompose */
    const convex_decomposition uut(points, triangles, options);

    /* Save resutls */
    check(uut, "face-YH_voxel_no_pca_no_hull_approx");
}

BOOST_AUTO_TEST_CASE( feline_voxel_no_pca_no_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/feline.off");

    /* Decompose */
    const convex_decomposition uut(points, triangles, options);

    /* Save resutls */
    check(uut, "feline_voxel_no_pca_no_hull_approx");
}

BOOST_AUTO_TEST_CASE( fish_voxel_no_pca_no_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/fish.off");

    /* Decompose */
    const convex_decomposition uut(points, triangles, options);

    /* Save resutls */
    check(uut, "fish_voxel_no_pca_no_hull_approx");
}

BOOST_AUTO_TEST_CASE( foot_voxel_no_pca_no_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/foot.off");

    /* Decompose */
    const convex_decomposition uut(points, triangles, options);

    /* Save resutls */
    check(uut, "foot_voxel_no_pca_no_hull_approx");
}

BOOST_AUTO_TEST_CASE( genus3_voxel_no_pca_no_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/genus3.off");

    /* Decompose */
    const convex_decomposition uut(points, triangles, options);

    /* Save resutls */
    check(uut, "genus3_voxel_no_pca_no_hull_approx");
}

BOOST_AUTO_TEST_CASE( greek_sculpture_voxel_no_pca_no_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/greek_sculpture.off");

    /* Decompose */
    const convex_decomposition uut(points, triangles, options);

    /* Save resutls */
    check(uut, "greek_sculpture_voxel_no_pca_no_hull_approx");
}

BOOST_AUTO_TEST_CASE( Hand1_voxel_no_pca_no_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/Hand1.off");

    /* Decompose */
    const convex_decomposition uut(points, triangles, options);

    /* Save resutls */
    check(uut, "Hand1_voxel_no_pca_no_hull_approx");
}

BOOST_AUTO_TEST_CASE( hand2_voxel_no_pca_no_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/hand2.off");

    /* Decompose */
    const convex_decomposition uut(points, triangles, options);

    /* Save resutls */
    check(uut, "hand2_voxel_no_pca_no_hull_approx");
}

BOOST_AUTO_TEST_CASE( horse_voxel_no_pca_no_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/horse.off");

    /* Decompose */
    const convex_decomposition uut(points, triangles, options);

    /* Save resutls */
    check(uut, "horse_voxel_no_pca_no_hull_approx");
}

BOOST_AUTO_TEST_CASE( helix_voxel_no_pca_no_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/helix.off");

    /* Decompose */
    const convex_decomposition uut(points, triangles, options);

    /* Save resutls */
    check(uut, "helix_voxel_no_pca_no_hull_approx");
}

BOOST_AUTO_TEST_CASE( helmet_voxel_no_pca_no_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/helmet.off");

    /* Decompose */
    const convex_decomposition uut(points, triangles, options);

    /* Save resutls */
    check(uut, "helmet_voxel_no_pca_no_hull_approx");
}

BOOST_AUTO_TEST_CASE( hero_voxel_no_pca_no_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/hero.off");

    /* Decompose */
    const convex_decomposition uut(points, triangles, options);

    /* Save resutls */
    check(uut, "hero_voxel_no_pca_no_hull_approx");
}

BOOST_AUTO_TEST_CASE( homer_voxel_no_pca_no_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/homer.off");

    /* Decompose */
    const convex_decomposition uut(points, triangles, options);

    /* Save resutls */
    check(uut, "homer_voxel_no_pca_no_hull_approx");
}

BOOST_AUTO_TEST_CASE( hornbug_voxel_no_pca_no_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/hornbug.off");

    /* Decompose */
    const convex_decomposition uut(points, triangles, options);

    /* Save resutls */
    check(uut, "hornbug_voxel_no_pca_no_hull_approx");
}

BOOST_AUTO_TEST_CASE( maneki_neko_voxel_no_pca_no_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/maneki-neko.off");

    /* Decompose */
    const convex_decomposition uut(points, triangles, options);

    /* Save resutls */
    check(uut, "maneki-neko_voxel_no_pca_no_hull_approx");
}

BOOST_AUTO_TEST_CASE( mannequin_devil_voxel_no_pca_no_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/mannequin-devil.off");

    /* Decompose */
    const convex_decomposition uut(points, triangles, options);

    /* Save resutls */
    check(uut, "mannequin-devil_voxel_no_pca_no_hull_approx");
}

BOOST_AUTO_TEST_CASE( mannequin_voxel_no_pca_no_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/mannequin.off");

    /* Decompose */
    const convex_decomposition uut(points, triangles, options);

    /* Save resutls */
    check(uut, "mannequin_voxel_no_pca_no_hull_approx");
}

BOOST_AUTO_TEST_CASE( mask_voxel_no_pca_no_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/mask.off");

    /* Decompose */
    const convex_decomposition uut(points, triangles, options);

    /* Save resutls */
    check(uut, "mask_voxel_no_pca_no_hull_approx");
}

BOOST_AUTO_TEST_CASE( moaimoai_voxel_no_pca_no_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/moaimoai.off");

    /* Decompose */
    const convex_decomposition uut(points, triangles, options);

    /* Save resutls */
    check(uut, "moaimoai_voxel_no_pca_no_hull_approx");
}

BOOST_AUTO_TEST_CASE( monk_voxel_no_pca_no_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/monk.off");

    /* Decompose */
    const convex_decomposition uut(points, triangles, options);

    /* Save resutls */
    check(uut, "monk_voxel_no_pca_no_hull_approx");
}

BOOST_AUTO_TEST_CASE( octopus_voxel_no_pca_no_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/octopus.off");

    /* Decompose */
    const convex_decomposition uut(points, triangles, options);

    /* Save resutls */
    check(uut, "octopus_voxel_no_pca_no_hull_approx");
}

BOOST_AUTO_TEST_CASE( pig_voxel_no_pca_no_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/pig.off");

    /* Decompose */
    const convex_decomposition uut(points, triangles, options);

    /* Save resutls */
    check(uut, "pig_voxel_no_pca_no_hull_approx");
}

BOOST_AUTO_TEST_CASE( pinocchio_b_voxel_no_pca_no_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/pinocchio_b.off");

    /* Decompose */
    const convex_decomposition uut(points, triangles, options);

    /* Save resutls */
    check(uut, "pinocchio_b_voxel_no_pca_no_hull_approx");
}

BOOST_AUTO_TEST_CASE( polygirl_voxel_no_pca_no_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/polygirl.off");

    /* Decompose */
    const convex_decomposition uut(points, triangles, options);

    /* Save resutls */
    check(uut, "polygirl_voxel_no_pca_no_hull_approx");
}

BOOST_AUTO_TEST_CASE( rabbit_voxel_no_pca_no_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/rabbit.off");

    /* Decompose */
    const convex_decomposition uut(points, triangles, options);

    /* Save resutls */
    check(uut, "rabbit_voxel_no_pca_no_hull_approx");
}

BOOST_AUTO_TEST_CASE( rocker_arm_voxel_no_pca_no_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/rocker-arm.off");

    /* Decompose */
    const convex_decomposition uut(points, triangles, options);

    /* Save resutls */
    check(uut, "rocker-arm_voxel_no_pca_no_hull_approx");
}

BOOST_AUTO_TEST_CASE( screwdriver_voxel_no_pca_no_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/screwdriver.off");

    /* Decompose */
    const convex_decomposition uut(points, triangles, options);

    /* Save resutls */
    check(uut, "screwdriver_voxel_no_pca_no_hull_approx");
}

BOOST_AUTO_TEST_CASE( shark_b_voxel_no_pca_no_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/shark_b.off");

    /* Decompose */
    const convex_decomposition uut(points, triangles, options);

    /* Save resutls */
    check(uut, "shark_b_voxel_no_pca_no_hull_approx");
}

BOOST_AUTO_TEST_CASE( sketched_brunnen_voxel_no_pca_no_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/Sketched-Brunnen.off");

    /* Decompose */
    const convex_decomposition uut(points, triangles, options);

    /* Save resutls */
    check(uut, "Sketched-Brunnen_voxel_no_pca_no_hull_approx");
}

BOOST_AUTO_TEST_CASE( sledge_voxel_no_pca_no_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/sledge.off");

    /* Decompose */
    const convex_decomposition uut(points, triangles, options);

    /* Save resutls */
    check(uut, "sledge_voxel_no_pca_no_hull_approx");
}

BOOST_AUTO_TEST_CASE( squirrel_voxel_no_pca_no_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/squirrel.off");

    /* Decompose */
    const convex_decomposition uut(points, triangles, options);

    /* Save resutls */
    check(uut, "squirrel_voxel_no_pca_no_hull_approx");
}

BOOST_AUTO_TEST_CASE( sword_voxel_no_pca_no_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/sword.off");

    /* Decompose */
    const convex_decomposition uut(points, triangles, options);

    /* Save resutls */
    check(uut, "sword_voxel_no_pca_no_hull_approx");
}

BOOST_AUTO_TEST_CASE( table_voxel_no_pca_no_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/table.off");

    /* Decompose */
    const convex_decomposition uut(points, triangles, options);

    /* Save resutls */
    check(uut, "table_voxel_no_pca_no_hull_approx");
}

BOOST_AUTO_TEST_CASE( Teapot_voxel_no_pca_no_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/Teapot.off");

    /* Decompose */
    const convex_decomposition uut(points, triangles, options);

    /* Save resutls */
    check(uut, "Teapot_voxel_no_pca_no_hull_approx");
}

BOOST_AUTO_TEST_CASE( test2_voxel_no_pca_no_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/test2.off");

    /* Decompose */
    const convex_decomposition uut(points, triangles, options);

    /* Save resutls */
    check(uut, "test2_voxel_no_pca_no_hull_approx");
}

BOOST_AUTO_TEST_CASE( test_voxel_no_pca_no_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/test.off");

    /* Decompose */
    const convex_decomposition uut(points, triangles, options);

    /* Save resutls */
    check(uut, "test_voxel_no_pca_no_hull_approx");
}

BOOST_AUTO_TEST_CASE( torus_voxel_no_pca_no_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/torus.off");

    /* Decompose */
    const convex_decomposition uut(points, triangles, options);

    /* Save resutls */
    check(uut, "torus_voxel_no_pca_no_hull_approx");
}

BOOST_AUTO_TEST_CASE( tstTorusModel3_voxel_no_pca_no_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/tstTorusModel3.off");

    /* Decompose */
    const convex_decomposition uut(points, triangles, options);

    /* Save resutls */
    check(uut, "tstTorusModel3_voxel_no_pca_no_hull_approx");
}

BOOST_AUTO_TEST_CASE( tstTorusModel_voxel_no_pca_no_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/tstTorusModel.off");

    /* Decompose */
    const convex_decomposition uut(points, triangles, options);

    /* Save resutls */
    check(uut, "tstTorusModel_voxel_no_pca_no_hull_approx");
}

BOOST_AUTO_TEST_CASE( tube1_voxel_no_pca_no_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/tube1.off");

    /* Decompose */
    const convex_decomposition uut(points, triangles, options);

    /* Save resutls */
    check(uut, "tube1_voxel_no_pca_no_hull_approx");
}

BOOST_AUTO_TEST_CASE( venus_voxel_no_pca_no_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/venus.off");

    /* Decompose */
    const convex_decomposition uut(points, triangles, options);

    /* Save resutls */
    check(uut, "venus_voxel_no_pca_no_hull_approx");
}

BOOST_AUTO_TEST_CASE( venus_original_voxel_no_pca_no_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/venus-original.off");

    /* Decompose */
    const convex_decomposition uut(points, triangles, options);

    /* Save resutls */
    check(uut, "venus-original_voxel_no_pca_no_hull_approx");
}
#endif /* #ifndef VALGRIND_TESTS */

BOOST_AUTO_TEST_SUITE_END()
}; /* namespace test */
}; /* namespace raptor_convex_decomposition */
