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
struct mesh_decimation_regression_fixture : public regression_fixture
{
    mesh_decimation_regression_fixture() :
        regression_fixture(mesh_decimation_options(1000.0f, 0.8f))
        // regression_fixture(mesh_decimation_options(5000.0f, 5504))
    {  }
};


BOOST_FIXTURE_TEST_SUITE( vertex_target_0_8_tests, mesh_decimation_regression_fixture )


/* Tests */
#ifndef VALGRIND_TESTS
BOOST_AUTO_TEST_CASE( block_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/block.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("block");
}

BOOST_AUTO_TEST_CASE( bunny_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/bunny.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("bunny");
}

BOOST_AUTO_TEST_CASE( camel_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/camel.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("camel");
}

BOOST_AUTO_TEST_CASE( casting_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/casting.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("casting");
}

BOOST_AUTO_TEST_CASE( chair_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/chair.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("chair");
}

BOOST_AUTO_TEST_CASE( cow1_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/cow1.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("cow1");
}

BOOST_AUTO_TEST_CASE( cow2_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/cow2.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("cow2");
}
#endif /* #ifndef VALGRIND_TESTS */

BOOST_AUTO_TEST_CASE( crank_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/crank.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("crank");
}

#ifndef VALGRIND_TESTS
BOOST_AUTO_TEST_CASE( cup_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/cup.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("cup");
}

BOOST_AUTO_TEST_CASE( dancer2_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/dancer2.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("dancer2");
}

BOOST_AUTO_TEST_CASE( deer_bound_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/deer_bound.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("deer_bound");
}

BOOST_AUTO_TEST_CASE( dilo_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/dilo.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("dilo");
}

BOOST_AUTO_TEST_CASE( dino_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/dino.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("dino");
}

BOOST_AUTO_TEST_CASE( DRAGON_F_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/DRAGON_F.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("DRAGON_F");
}

BOOST_AUTO_TEST_CASE( drum_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/drum.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("drum");
}

BOOST_AUTO_TEST_CASE( eight_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/eight.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("eight");
}

BOOST_AUTO_TEST_CASE( elephant_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/elephant.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("elephant");
}

BOOST_AUTO_TEST_CASE( elk_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/elk.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("elk");
}

BOOST_AUTO_TEST_CASE( egea_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/egea.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("egea");
}

BOOST_AUTO_TEST_CASE( face_yh_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/face-YH.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("face-YH");
}

BOOST_AUTO_TEST_CASE( feline_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/feline.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("feline");
}

BOOST_AUTO_TEST_CASE( fish_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/fish.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("fish");
}

BOOST_AUTO_TEST_CASE( foot_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/foot.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("foot");
}

BOOST_AUTO_TEST_CASE( genus3_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/genus3.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("genus3");
}

BOOST_AUTO_TEST_CASE( greek_sculpture_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/greek_sculpture.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("greek_sculpture");
}

BOOST_AUTO_TEST_CASE( hand1_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/Hand1.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("Hand1");
}

BOOST_AUTO_TEST_CASE( hand2_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/hand2.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("hand2");
}

BOOST_AUTO_TEST_CASE( horse_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/horse.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("horse");
}

BOOST_AUTO_TEST_CASE( helix_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/helix.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("helix");
}

BOOST_AUTO_TEST_CASE( helmet_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/helmet.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("helmet");
}

BOOST_AUTO_TEST_CASE( hero_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/hero.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("hero");
}

BOOST_AUTO_TEST_CASE( homer_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/homer.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("homer");
}

BOOST_AUTO_TEST_CASE( hornbug_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/hornbug.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("hornbug");
}

BOOST_AUTO_TEST_CASE( maneki_neko_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/maneki-neko.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("maneki-neko");
}

BOOST_AUTO_TEST_CASE( mannequin_devil_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/mannequin-devil.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("mannequin-devil");
}

BOOST_AUTO_TEST_CASE( mannequin_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/mannequin.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("mannequin");
}

BOOST_AUTO_TEST_CASE( mask_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/mask.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("mask");
}

BOOST_AUTO_TEST_CASE( moaimoai_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/moaimoai.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("moaimoai");
}

BOOST_AUTO_TEST_CASE( monk_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/monk.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("monk");
}

BOOST_AUTO_TEST_CASE( octopus_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/octopus.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("octopus");
}

BOOST_AUTO_TEST_CASE( pig_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/pig.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("pig");
}

BOOST_AUTO_TEST_CASE( pinocchio_b_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/pinocchio_b.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("pinocchio_b");
}

BOOST_AUTO_TEST_CASE( polygirl_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/polygirl.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("polygirl");
}

BOOST_AUTO_TEST_CASE( rabbit_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/rabbit.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("rabbit");
}

BOOST_AUTO_TEST_CASE( rocker_arm_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/rocker-arm.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("rocker-arm");
}

BOOST_AUTO_TEST_CASE( screwdriver_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/screwdriver.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("screwdriver");
}

BOOST_AUTO_TEST_CASE( shark_b_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/shark_b.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("shark_b");
}

BOOST_AUTO_TEST_CASE( sketched_brunnen_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/Sketched-Brunnen.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("Sketched-Brunnen");
}

BOOST_AUTO_TEST_CASE( sledge_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/sledge.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("sledge");
}

BOOST_AUTO_TEST_CASE( squirrel_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/squirrel.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("squirrel");
}

BOOST_AUTO_TEST_CASE( sword_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/sword.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("sword");
}

BOOST_AUTO_TEST_CASE( table_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/table.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("table");
}

BOOST_AUTO_TEST_CASE( teapot_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/Teapot.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("Teapot");

}

BOOST_AUTO_TEST_CASE( test2_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/test2.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("test2");
}

BOOST_AUTO_TEST_CASE( test_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/test.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("test");
}

BOOST_AUTO_TEST_CASE( torus_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/tstTorusModel2.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("tstTorusModel2");
}

BOOST_AUTO_TEST_CASE( tstTorusModel3_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/tstTorusModel3.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("tstTorusModel3");
}

BOOST_AUTO_TEST_CASE( tstTorusModel_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/tstTorusModel.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("tstTorusModel");
}

BOOST_AUTO_TEST_CASE( tube1_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/tube1.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("tube1");
}

BOOST_AUTO_TEST_CASE( venus_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/venus.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("venus");
}

BOOST_AUTO_TEST_CASE( venus_original_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/venus-original.off");

    /* Decimate */
    run();

    /* Save resutls */
    check("venus-original");
}
#endif /* #ifndef VALGRIND_TESTS */

BOOST_AUTO_TEST_SUITE_END()
} /* namespace test */
} /* namespace raptor_mesh_decimation */
