#ifdef STAND_ALONE
#define BOOST_TEST_MODULE regression test
#endif /* #ifdef STAND_ALONE */


/* Standard headers */

/* Boost headers */
#include "boost/test/unit_test.hpp"

/* Mesh decimation headers */
#include "mesh_decimation.h"
#include "mesh_decimation_options.h"

/* Test headers */
#include "regression_fixture.h"


namespace raptor_mesh_decimation
{
namespace test
{
/* Test data */
struct vertex_target_0_95_regression_fixture : public regression_fixture
{
    vertex_target_0_95_regression_fixture() :
        regression_fixture(mesh_decimation_options(100.0f, 100.0f, 0.95f))
    {  }
};


BOOST_FIXTURE_TEST_SUITE( regression_tests, vertex_target_0_95_regression_fixture )


/* Tests */
#ifndef VALGRIND_TESTS
BOOST_AUTO_TEST_CASE( block_vertex_target_0_95_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/block.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("vertex_target_0_95_tests_block");
}

BOOST_AUTO_TEST_CASE( bunny_vertex_target_0_95_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/bunny.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("vertex_target_0_95_tests_bunny");
}

BOOST_AUTO_TEST_CASE( camel_vertex_target_0_95_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/camel.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("vertex_target_0_95_tests_camel");
}

BOOST_AUTO_TEST_CASE( casting_vertex_target_0_95_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/casting.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("vertex_target_0_95_tests_casting");
}

BOOST_AUTO_TEST_CASE( chair_vertex_target_0_95_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/chair.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("vertex_target_0_95_tests_chair");
}

BOOST_AUTO_TEST_CASE( cow1_vertex_target_0_95_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/cow1.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("vertex_target_0_95_tests_cow1");
}

BOOST_AUTO_TEST_CASE( cow2_vertex_target_0_95_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/cow2.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("vertex_target_0_95_tests_cow2");
}
#endif /* #ifndef VALGRIND_TESTS */

BOOST_AUTO_TEST_CASE( crank_vertex_target_0_95_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/crank.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("vertex_target_0_95_tests_crank");
}

#ifndef VALGRIND_TESTS
BOOST_AUTO_TEST_CASE( cup_vertex_target_0_95_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/cup.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("vertex_target_0_95_tests_cup");
}

BOOST_AUTO_TEST_CASE( dancer2_vertex_target_0_95_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/dancer2.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("vertex_target_0_95_tests_dancer2");
}

BOOST_AUTO_TEST_CASE( deer_bound_vertex_target_0_95_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/deer_bound.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("vertex_target_0_95_tests_deer_bound");
}

BOOST_AUTO_TEST_CASE( dilo_vertex_target_0_95_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/dilo.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("vertex_target_0_95_tests_dilo");
}

BOOST_AUTO_TEST_CASE( dino_vertex_target_0_95_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/dino.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("vertex_target_0_95_tests_dino");
}

BOOST_AUTO_TEST_CASE( DRAGON_F_vertex_target_0_95_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/DRAGON_F.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("vertex_target_0_95_tests_DRAGON_F");
}

BOOST_AUTO_TEST_CASE( drum_vertex_target_0_95_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/drum.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("vertex_target_0_95_tests_drum");
}

BOOST_AUTO_TEST_CASE( eight_vertex_target_0_95_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/eight.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("vertex_target_0_95_tests_eight");
}

BOOST_AUTO_TEST_CASE( elephant_vertex_target_0_95_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/elephant.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("vertex_target_0_95_tests_elephant");
}

BOOST_AUTO_TEST_CASE( elk_vertex_target_0_95_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/elk.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("vertex_target_0_95_tests_elk");
}

BOOST_AUTO_TEST_CASE( egea_vertex_target_0_95_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/egea.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("vertex_target_0_95_tests_egea");
}

BOOST_AUTO_TEST_CASE( face_yh_vertex_target_0_95_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/face-YH.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("vertex_target_0_95_tests_face-YH");
}

BOOST_AUTO_TEST_CASE( feline_vertex_target_0_95_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/feline.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("vertex_target_0_95_tests_feline");
}

BOOST_AUTO_TEST_CASE( fish_vertex_target_0_95_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/fish.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("vertex_target_0_95_tests_fish");
}

BOOST_AUTO_TEST_CASE( foot_vertex_target_0_95_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/foot.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("vertex_target_0_95_tests_foot");
}

BOOST_AUTO_TEST_CASE( genus3_vertex_target_0_95_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/genus3.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("vertex_target_0_95_tests_genus3");
}

BOOST_AUTO_TEST_CASE( greek_sculpture_vertex_target_0_95_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/greek_sculpture.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("vertex_target_0_95_tests_greek_sculpture");
}

BOOST_AUTO_TEST_CASE( hand1_vertex_target_0_95_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/Hand1.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("vertex_target_0_95_tests_Hand1");
}

BOOST_AUTO_TEST_CASE( hand2_vertex_target_0_95_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/hand2.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("vertex_target_0_95_tests_hand2");
}

BOOST_AUTO_TEST_CASE( horse_vertex_target_0_95_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/horse.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("vertex_target_0_95_tests_horse");
}

BOOST_AUTO_TEST_CASE( helix_vertex_target_0_95_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/helix.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("vertex_target_0_95_tests_helix");
}

BOOST_AUTO_TEST_CASE( helmet_vertex_target_0_95_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/helmet.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("vertex_target_0_95_tests_helmet");
}

BOOST_AUTO_TEST_CASE( hero_vertex_target_0_95_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/hero.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("vertex_target_0_95_tests_hero");
}

BOOST_AUTO_TEST_CASE( homer_vertex_target_0_95_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/homer.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("vertex_target_0_95_tests_homer");
}

BOOST_AUTO_TEST_CASE( hornbug_vertex_target_0_95_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/hornbug.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("vertex_target_0_95_tests_hornbug");
}

BOOST_AUTO_TEST_CASE( maneki_neko_vertex_target_0_95_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/maneki-neko.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("vertex_target_0_95_tests_maneki-neko");
}

BOOST_AUTO_TEST_CASE( mannequin_devil_vertex_target_0_95_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/mannequin-devil.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("vertex_target_0_95_tests_mannequin-devil");
}

BOOST_AUTO_TEST_CASE( mannequin_vertex_target_0_95_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/mannequin.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("vertex_target_0_95_tests_mannequin");
}

BOOST_AUTO_TEST_CASE( mask_vertex_target_0_95_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/mask.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("vertex_target_0_95_tests_mask");
}

BOOST_AUTO_TEST_CASE( moaimoai_vertex_target_0_95_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/moaimoai.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("vertex_target_0_95_tests_moaimoai");
}

BOOST_AUTO_TEST_CASE( monk_vertex_target_0_95_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/monk.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("vertex_target_0_95_tests_monk");
}

BOOST_AUTO_TEST_CASE( octopus_vertex_target_0_95_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/octopus.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("vertex_target_0_95_tests_octopus");
}

BOOST_AUTO_TEST_CASE( pig_vertex_target_0_95_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/pig.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("vertex_target_0_95_tests_pig");
}

BOOST_AUTO_TEST_CASE( pinocchio_b_vertex_target_0_95_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/pinocchio_b.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("vertex_target_0_95_tests_pinocchio_b");
}

BOOST_AUTO_TEST_CASE( polygirl_vertex_target_0_95_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/polygirl.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("vertex_target_0_95_tests_polygirl");
}

BOOST_AUTO_TEST_CASE( rabbit_vertex_target_0_95_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/rabbit.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("vertex_target_0_95_tests_rabbit");
}

BOOST_AUTO_TEST_CASE( rocker_arm_vertex_target_0_95_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/rocker-arm.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("vertex_target_0_95_tests_rocker-arm");
}

BOOST_AUTO_TEST_CASE( screwdriver_vertex_target_0_95_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/screwdriver.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("vertex_target_0_95_tests_screwdriver");
}

BOOST_AUTO_TEST_CASE( shark_b_vertex_target_0_95_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/shark_b.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("vertex_target_0_95_tests_shark_b");
}

BOOST_AUTO_TEST_CASE( sketched_brunnen_vertex_target_0_95_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/Sketched-Brunnen.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("vertex_target_0_95_tests_Sketched-Brunnen");
}

BOOST_AUTO_TEST_CASE( sledge_vertex_target_0_95_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/sledge.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("vertex_target_0_95_tests_sledge");
}

BOOST_AUTO_TEST_CASE( squirrel_vertex_target_0_95_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/squirrel.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("vertex_target_0_95_tests_squirrel");
}

BOOST_AUTO_TEST_CASE( sword_vertex_target_0_95_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/sword.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("vertex_target_0_95_tests_sword");
}

BOOST_AUTO_TEST_CASE( table_vertex_target_0_95_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/table.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("vertex_target_0_95_tests_table");
}

BOOST_AUTO_TEST_CASE( teapot_vertex_target_0_95_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/Teapot.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("vertex_target_0_95_tests_Teapot");

}

BOOST_AUTO_TEST_CASE( test2_vertex_target_0_95_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/test2.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("vertex_target_0_95_tests_test2");
}

BOOST_AUTO_TEST_CASE( test_vertex_target_0_95_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/test.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("vertex_target_0_95_tests_test");
}

BOOST_AUTO_TEST_CASE( torus_vertex_target_0_95_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/tstTorusModel2.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("vertex_target_0_95_tests_tstTorusModel2");
}

BOOST_AUTO_TEST_CASE( tstTorusModel3_vertex_target_0_95_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/tstTorusModel3.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("vertex_target_0_95_tests_tstTorusModel3");
}

BOOST_AUTO_TEST_CASE( tstTorusModel_vertex_target_0_95_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/tstTorusModel.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("vertex_target_0_95_tests_tstTorusModel");
}

BOOST_AUTO_TEST_CASE( tube1_vertex_target_0_95_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/tube1.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("vertex_target_0_95_tests_tube1");
}

BOOST_AUTO_TEST_CASE( venus_vertex_target_0_95_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/venus.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("vertex_target_0_95_tests_venus");
}

BOOST_AUTO_TEST_CASE( venus_original_vertex_target_0_95_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/venus-original.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("vertex_target_0_95_tests_venus-original");
}
#endif /* #ifndef VALGRIND_TESTS */

BOOST_AUTO_TEST_SUITE_END()
} /* namespace test */
} /* namespace raptor_mesh_decimation */
