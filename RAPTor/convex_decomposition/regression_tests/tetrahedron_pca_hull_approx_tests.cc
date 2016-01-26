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
        regression_fixture(convex_decomposition_options(0.001f, 0.05f, 0.05f, 0.0005f, 0.0001f, 100000, 0, 20, 4, 4, discretisation_type_t::tetrahedron, true, true))
    {  }
};


BOOST_FIXTURE_TEST_SUITE( regression_tests, tetrahedron_pca_hull_approx_regression_fixture )

const float result_tolerance = 0.0005f;


/* Tests */
#ifndef VALGRIND_TESTS
BOOST_AUTO_TEST_CASE( block_tetrahedron_pca_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/block.off");

    /* Decompose */
    run();

    /* Save results */
    check("block_tetrahedron_pca_hull_approx");
}

BOOST_AUTO_TEST_CASE( bunny_tetrahedron_pca_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/bunny.off");

    /* Decompose */
    run();

    /* Save results */
    check("bunny_tetrahedron_pca_hull_approx");
}

BOOST_AUTO_TEST_CASE( camel_tetrahedron_pca_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/camel.off");

    /* Decompose */
    run();

    /* Save results */
    check("camel_tetrahedron_pca_hull_approx");
}

BOOST_AUTO_TEST_CASE( casting_tetrahedron_pca_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/casting.off");

    /* Decompose */
    run();

    /* Save results */
    check("casting_tetrahedron_pca_hull_approx");
}
#endif /* #ifndef VALGRIND_TESTS */

BOOST_AUTO_TEST_CASE( chair_tetrahedron_pca_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/chair.off");

    /* Decompose */
    run();

    /* Save results */
    check("chair_tetrahedron_pca_hull_approx");
}

#ifndef VALGRIND_TESTS
BOOST_AUTO_TEST_CASE( cow1_tetrahedron_pca_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/cow1.off");

    /* Decompose */
    run();

    /* Save results */
    check("cow1_tetrahedron_pca_hull_approx");
}

BOOST_AUTO_TEST_CASE( cow2_tetrahedron_pca_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/cow2.off");

    /* Decompose */
    run();

    /* Save results */
    check("cow2_tetrahedron_pca_hull_approx");
}

BOOST_AUTO_TEST_CASE( crank_tetrahedron_pca_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/crank.off");

    /* Decompose */
    run();

    /* Save results */
    check("crank_tetrahedron_pca_hull_approx");
}

BOOST_AUTO_TEST_CASE( cup_tetrahedron_pca_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/cup.off");

    /* Decompose */
    run();

    /* Save results */
    check("cup_tetrahedron_pca_hull_approx");
}

BOOST_AUTO_TEST_CASE( dancer2_tetrahedron_pca_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/dancer2.off");

    /* Decompose */
    run();

    /* Save results */
    check("dancer2_tetrahedron_pca_hull_approx");
}

BOOST_AUTO_TEST_CASE( deer_bound_tetrahedron_pca_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/deer_bound.off");

    /* Decompose */
    run();

    /* Save results */
    check("deer_bound_tetrahedron_pca_hull_approx");
}

BOOST_AUTO_TEST_CASE( dilo_tetrahedron_pca_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/dilo.off");

    /* Decompose */
    run();

    /* Save results */
    check("dilo_tetrahedron_pca_hull_approx");
}

BOOST_AUTO_TEST_CASE( dino_tetrahedron_pca_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/dino.off");

    /* Decompose */
    run();

    /* Save results */
    check("dino_tetrahedron_pca_hull_approx");
}

BOOST_AUTO_TEST_CASE( DRAGON_F_tetrahedron_pca_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/DRAGON_F.off");

    /* Decompose */
    run();

    /* Save results */
    check("DRAGON_F_tetrahedron_pca_hull_approx");
}

BOOST_AUTO_TEST_CASE( drum_tetrahedron_pca_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/drum.off");

    /* Decompose */
    run();

    /* Save results */
    check("drum_tetrahedron_pca_hull_approx");
}

BOOST_AUTO_TEST_CASE( eight_tetrahedron_pca_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/eight.off");

    /* Decompose */
    run();

    /* Save results */
    check("eight_tetrahedron_pca_hull_approx");
}

BOOST_AUTO_TEST_CASE( elephant_tetrahedron_pca_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/elephant.off");

    /* Decompose */
    run();

    /* Save results */
    check("elephant_tetrahedron_pca_hull_approx");
}

BOOST_AUTO_TEST_CASE( elk_tetrahedron_pca_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/elk.off");

    /* Decompose */
    run();

    /* Save results */
    check("elk_tetrahedron_pca_hull_approx");
}

BOOST_AUTO_TEST_CASE( egea_tetrahedron_pca_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/egea.off");

    /* Decompose */
    run();

    /* Save results */
    check("egea_tetrahedron_pca_hull_approx");
}

BOOST_AUTO_TEST_CASE( face_yh_tetrahedron_pca_hull_approx_test_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/face-YH.off");

    /* Decompose */
    run();

    /* Save results */
    check("face-YH_tetrahedron_pca_hull_approx");
}

BOOST_AUTO_TEST_CASE( feline_tetrahedron_pca_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/feline.off");

    /* Decompose */
    run();

    /* Save results */
    check("feline_tetrahedron_pca_hull_approx");
}

BOOST_AUTO_TEST_CASE( fish_tetrahedron_pca_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/fish.off");

    /* Decompose */
    run();

    /* Save results */
    check("fish_tetrahedron_pca_hull_approx");
}

BOOST_AUTO_TEST_CASE( foot_tetrahedron_pca_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/foot.off");

    /* Decompose */
    run();

    /* Save results */
    check("foot_tetrahedron_pca_hull_approx");
}

BOOST_AUTO_TEST_CASE( genus3_tetrahedron_pca_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/genus3.off");

    /* Decompose */
    run();

    /* Save results */
    check("genus3_tetrahedron_pca_hull_approx");
}

BOOST_AUTO_TEST_CASE( greek_sculpture_tetrahedron_pca_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/greek_sculpture.off");

    /* Decompose */
    run();

    /* Save results */
    check("greek_sculpture_tetrahedron_pca_hull_approx");
}

BOOST_AUTO_TEST_CASE( Hand1_tetrahedron_pca_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/Hand1.off");

    /* Decompose */
    run();

    /* Save results */
    check("Hand1_tetrahedron_pca_hull_approx");
}

BOOST_AUTO_TEST_CASE( hand2_tetrahedron_pca_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/hand2.off");

    /* Decompose */
    run();

    /* Save results */
    check("hand2_tetrahedron_pca_hull_approx");
}

BOOST_AUTO_TEST_CASE( horse_tetrahedron_pca_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/horse.off");

    /* Decompose */
    run();

    /* Save results */
    check("horse_tetrahedron_pca_hull_approx");
}

BOOST_AUTO_TEST_CASE( helix_tetrahedron_pca_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/helix.off");

    /* Decompose */
    run();

    /* Save results */
    check("helix_tetrahedron_pca_hull_approx");
}

BOOST_AUTO_TEST_CASE( helmet_tetrahedron_pca_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/helmet.off");

    /* Decompose */
    run();

    /* Save results */
    check("helmet_tetrahedron_pca_hull_approx");
}

BOOST_AUTO_TEST_CASE( hero_tetrahedron_pca_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/hero.off");

    /* Decompose */
    run();

    /* Save results */
    check("hero_tetrahedron_pca_hull_approx");
}

BOOST_AUTO_TEST_CASE( homer_tetrahedron_pca_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/homer.off");

    /* Decompose */
    run();

    /* Save results */
    check("homer_tetrahedron_pca_hull_approx");
}

BOOST_AUTO_TEST_CASE( hornbug_tetrahedron_pca_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/hornbug.off");

    /* Decompose */
    run();

    /* Save results */
    check("hornbug_tetrahedron_pca_hull_approx");
}

BOOST_AUTO_TEST_CASE( maneki_neko_tetrahedron_pca_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/maneki-neko.off");

    /* Decompose */
    run();

    /* Save results */
    check("maneki-neko_tetrahedron_pca_hull_approx");
}

BOOST_AUTO_TEST_CASE( mannequin_devil_tetrahedron_pca_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/mannequin-devil.off");

    /* Decompose */
    run();

    /* Save results */
    check("mannequin-devil_tetrahedron_pca_hull_approx");
}

BOOST_AUTO_TEST_CASE( mannequin_tetrahedron_pca_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/mannequin.off");

    /* Decompose */
    run();

    /* Save results */
    check("mannequin_tetrahedron_pca_hull_approx");
}

BOOST_AUTO_TEST_CASE( mask_tetrahedron_pca_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/mask.off");

    /* Decompose */
    run();

    /* Save results */
    check("mask_tetrahedron_pca_hull_approx");
}

BOOST_AUTO_TEST_CASE( moaimoai_tetrahedron_pca_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/moaimoai.off");

    /* Decompose */
    run();

    /* Save results */
    check("moaimoai_tetrahedron_pca_hull_approx");
}

BOOST_AUTO_TEST_CASE( monk_tetrahedron_pca_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/monk.off");

    /* Decompose */
    run();

    /* Save results */
    check("monk_tetrahedron_pca_hull_approx");
}

BOOST_AUTO_TEST_CASE( octopus_tetrahedron_pca_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/octopus.off");

    /* Decompose */
    run();

    /* Save results */
    check("octopus_tetrahedron_pca_hull_approx");
}

BOOST_AUTO_TEST_CASE( pig_tetrahedron_pca_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/pig.off");

    /* Decompose */
    run();

    /* Save results */
    check("pig_tetrahedron_pca_hull_approx");
}

BOOST_AUTO_TEST_CASE( pinocchio_b_tetrahedron_pca_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/pinocchio_b.off");

    /* Decompose */
    run();

    /* Save results */
    check("pinocchio_b_tetrahedron_pca_hull_approx");
}

BOOST_AUTO_TEST_CASE( polygirl_tetrahedron_pca_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/polygirl.off");

    /* Decompose */
    run();

    /* Save results */
    check("polygirl_tetrahedron_pca_hull_approx");
}

BOOST_AUTO_TEST_CASE( rabbit_tetrahedron_pca_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/rabbit.off");

    /* Decompose */
    run();

    /* Save results */
    check("rabbit_tetrahedron_pca_hull_approx");
}

BOOST_AUTO_TEST_CASE( rocker_arm_tetrahedron_pca_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/rocker-arm.off");

    /* Decompose */
    run();

    /* Save results */
    check("rocker-arm_tetrahedron_pca_hull_approx");
}

BOOST_AUTO_TEST_CASE( screwdriver_tetrahedron_pca_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/screwdriver.off");

    /* Decompose */
    run();

    /* Save results */
    check("screwdriver_tetrahedron_pca_hull_approx");
}

BOOST_AUTO_TEST_CASE( shark_b_tetrahedron_pca_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/shark_b.off");

    /* Decompose */
    run();

    /* Save results */
    check("shark_b_tetrahedron_pca_hull_approx");
}

BOOST_AUTO_TEST_CASE( sketched_brunnen_tetrahedron_pca_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/Sketched-Brunnen.off");

    /* Decompose */
    run();

    /* Save results */
    check("Sketched-Brunnen_tetrahedron_pca_hull_approx");
}

BOOST_AUTO_TEST_CASE( sledge_tetrahedron_pca_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/sledge.off");

    /* Decompose */
    run();

    /* Save results */
    check("sledge_tetrahedron_pca_hull_approx");
}

BOOST_AUTO_TEST_CASE( squirrel_tetrahedron_pca_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/squirrel.off");

    /* Decompose */
    run();

    /* Save results */
    check("squirrel_tetrahedron_pca_hull_approx");
}

BOOST_AUTO_TEST_CASE( sword_tetrahedron_pca_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/sword.off");

    /* Decompose */
    run();

    /* Save results */
    check("sword_tetrahedron_pca_hull_approx");
}

BOOST_AUTO_TEST_CASE( table_tetrahedron_pca_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/table.off");

    /* Decompose */
    run();

    /* Save results */
    check("table_tetrahedron_pca_hull_approx");
}

BOOST_AUTO_TEST_CASE( Teapot_tetrahedron_pca_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/Teapot.off");

    /* Decompose */
    run();

    /* Save results */
    check("Teapot_tetrahedron_pca_hull_approx");
}

BOOST_AUTO_TEST_CASE( test2_tetrahedron_pca_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/test2.off");

    /* Decompose */
    run();

    /* Save results */
    check("test2_tetrahedron_pca_hull_approx");
}

BOOST_AUTO_TEST_CASE( test_tetrahedron_pca_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/test.off");

    /* Decompose */
    run();

    /* Save results */
    check("test_tetrahedron_pca_hull_approx");
}

BOOST_AUTO_TEST_CASE( torus_tetrahedron_pca_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/torus.off");

    /* Decompose */
    run();

    /* Save results */
    check("torus_tetrahedron_pca_hull_approx");
}

BOOST_AUTO_TEST_CASE( tstTorusModel3_tetrahedron_pca_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/tstTorusModel3.off");

    /* Decompose */
    run();

    /* Save results */
    check("tstTorusModel3_tetrahedron_pca_hull_approx");
}

BOOST_AUTO_TEST_CASE( tstTorusModel_tetrahedron_pca_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/tstTorusModel.off");

    /* Decompose */
    run();

    /* Save results */
    check("tstTorusModel_tetrahedron_pca_hull_approx");
}

BOOST_AUTO_TEST_CASE( tube1_tetrahedron_pca_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/tube1.off");

    /* Decompose */
    run();

    /* Save results */
    check("tube1_tetrahedron_pca_hull_approx");
}

BOOST_AUTO_TEST_CASE( venus_tetrahedron_pca_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/venus.off");

    /* Decompose */
    run();

    /* Save results */
    check("venus_tetrahedron_pca_hull_approx");
}

BOOST_AUTO_TEST_CASE( venus_original_tetrahedron_pca_hull_approx_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/venus-original.off");

    /* Decompose */
    run();

    /* Save results */
    check("venus-original_tetrahedron_pca_hull_approx");
}
#endif /* #ifndef VALGRIND_TESTS */

BOOST_AUTO_TEST_SUITE_END()
}; /* namespace test */
}; /* namespace raptor_convex_decomposition */
