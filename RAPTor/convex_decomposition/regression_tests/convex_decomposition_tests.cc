
#ifdef STAND_ALONE
#define BOOST_TEST_MODULE convex_decomposition test

/* Common headers */
#include "logging.h"

/* Initialise logger */
const raptor_physics::init_logger init_logger;
#endif /* #ifdef STAND_ALONE */

/* Test headers */
#include "convex_decomposition_fixture.h"


BOOST_FIXTURE_TEST_SUITE( convex_decomposition_tests, convex_decomposition_fixture );


BOOST_AUTO_TEST_CASE( block_convex_decomposition_test )
{
    /* Load data */
    load_data("block", 1.0, 30.0, 2000.0, 2, false, true, true);

    /* Run convex decomposition */
    cd->Compute();

    /* Check results */
    check("block");
}


BOOST_AUTO_TEST_CASE( bowl_convex_decomposition_test )
{
    /* Load data */
    load_data("bowl", 1.0, 30.0, 2000.0, 2, false, true, true);

    /* Run convex decomposition */
    cd->Compute();

    /* Check results */
    check("bowl");
}


BOOST_AUTO_TEST_CASE( bunny_convex_decomposition_test )
{
    /* Load data */
    load_data("bunny", 1.0, 30.0, 2000.0, 2, false, true, true);

    /* Run convex decomposition */
    cd->Compute();

    /* Check results */
    check("bunny");
}


BOOST_AUTO_TEST_CASE( camel_convex_decomposition_test )
{
    /* Load data */
    load_data("camel", 1.0, 30.0, 2000.0, 2, false, true, true);

    /* Run convex decomposition */
    cd->Compute();

    /* Check results */
    check("camel");
}


BOOST_AUTO_TEST_CASE( casting_convex_decomposition_test )
{
    /* Load data */
    load_data("casting", 1.0, 30.0, 2000.0, 2, false, true, true);

    /* Run convex decomposition */
    cd->Compute();

    /* Check results */
    check("casting");
}


BOOST_AUTO_TEST_CASE( chair_convex_decomposition_test )
{
    /* Load data */
    load_data("chair", 1.0, 30.0, 2000.0, 2, false, true, true);

    /* Run convex decomposition */
    cd->Compute();

    /* Check results */
    check("chair");
}


BOOST_AUTO_TEST_CASE( cow1_convex_decomposition_test )
{
    /* Load data */
    load_data("cow1", 1.0, 30.0, 2000.0, 2, false, true, true);

    /* Run convex decomposition */
    cd->Compute();

    /* Check results */
    check("cow1");
}


BOOST_AUTO_TEST_CASE( cow2_convex_decomposition_test )
{
    /* Load data */
    load_data("cow2", 1.0, 30.0, 2000.0, 2, false, true, true);

    /* Run convex decomposition */
    cd->Compute();

    /* Check results */
    check("cow2");
}


BOOST_AUTO_TEST_CASE( crank_convex_decomposition_test )
{
    /* Load data */
    load_data("crank", 1.0, 30.0, 2000.0, 2, false, true, true);

    /* Run convex decomposition */
    cd->Compute();

    /* Check results */
    check("crank");
}


BOOST_AUTO_TEST_CASE( CRATERS_F_convex_decomposition_test )
{
    /* Load data */
    load_data("CRATERS_F", 1.0, 30.0, 2000.0, 2, false, true, true);

    /* Run convex decomposition */
    cd->Compute();

    /* Check results */
    check("CRATERS_F");
}


BOOST_AUTO_TEST_CASE( cup_convex_decomposition_test )
{
    /* Load data */
    load_data("cup", 1.0, 30.0, 2000.0, 2, false, true, true);

    /* Run convex decomposition */
    cd->Compute();

    /* Check results */
    check("cup");
}


BOOST_AUTO_TEST_CASE( dancer2_convex_decomposition_test )
{
    /* Load data */
    load_data("dancer2", 1.0, 30.0, 2000.0, 2, false, true, true);

    /* Run convex decomposition */
    cd->Compute();

    /* Check results */
    check("dancer2");
}


BOOST_AUTO_TEST_CASE( deer_bound_convex_decomposition_test )
{
    /* Load data */
    load_data("deer_bound", 1.0, 30.0, 2000.0, 2, false, true, true);

    /* Run convex decomposition */
    cd->Compute();

    /* Check results */
    check("deer_bound");
}


BOOST_AUTO_TEST_CASE( dilo_convex_decomposition_test )
{
    /* Load data */
    load_data("dilo", 1.0, 30.0, 2000.0, 2, false, true, true);

    /* Run convex decomposition */
    cd->Compute();

    /* Check results */
    check("dilo");
}


BOOST_AUTO_TEST_CASE( dino_convex_decomposition_test )
{
    /* Load data */
    load_data("dino", 1.0, 30.0, 2000.0, 2, false, true, true);

    /* Run convex decomposition */
    cd->Compute();

    /* Check results */
    check("dino");
}


BOOST_AUTO_TEST_CASE( DRAGON_F_convex_decomposition_test )
{
    /* Load data */
    load_data("DRAGON_F", 1.0, 30.0, 2000.0, 2, false, true, true);

    /* Run convex decomposition */
    cd->Compute();

    /* Check results */
    check("DRAGON_F");
}


BOOST_AUTO_TEST_CASE( drum_convex_decomposition_test )
{
    /* Load data */
    load_data("drum", 1.0, 30.0, 2000.0, 2, false, true, true);

    /* Run convex decomposition */
    cd->Compute();

    /* Check results */
    check("drum");
}


BOOST_AUTO_TEST_CASE( egea_convex_decomposition_test )
{
    /* Load data */
    load_data("egea", 1.0, 30.0, 2000.0, 2, false, true, true);

    /* Run convex decomposition */
    cd->Compute();

    /* Check results */
    check("egea");
}


BOOST_AUTO_TEST_CASE( eight_convex_decomposition_test )
{
    /* Load data */
    load_data("eight", 1.0, 30.0, 2000.0, 2, false, true, true);

    /* Run convex decomposition */
    cd->Compute();

    /* Check results */
    check("eight");
}


BOOST_AUTO_TEST_CASE( elephant_convex_decomposition_test )
{
    /* Load data */
    load_data("elephant", 1.0, 30.0, 2000.0, 2, false, true, true);

    /* Run convex decomposition */
    cd->Compute();

    /* Check results */
    check("elephant");
}


BOOST_AUTO_TEST_CASE( elk_convex_decomposition_test )
{
    /* Load data */
    load_data("elk", 1.0, 30.0, 2000.0, 2, false, true, true);

    /* Run convex decomposition */
    cd->Compute();

    /* Check results */
    check("elk");
}


BOOST_AUTO_TEST_CASE( face_YH_convex_decomposition_test )
{
    /* Load data */
    load_data("face-YH", 1.0, 30.0, 2000.0, 2, false, true, true);

    /* Run convex decomposition */
    cd->Compute();

    /* Check results */
    check("face-YH");
}


BOOST_AUTO_TEST_CASE( feline_convex_decomposition_test )
{
    /* Load data */
    load_data("feline", 1.0, 30.0, 2000.0, 2, false, true, true);

    /* Run convex decomposition */
    cd->Compute();

    /* Check results */
    check("feline");
}


BOOST_AUTO_TEST_CASE( fish_convex_decomposition_test )
{
    /* Load data */
    load_data("fish", 1.0, 30.0, 2000.0, 2, false, true, true);

    /* Run convex decomposition */
    cd->Compute();

    /* Check results */
    check("fish");
}


BOOST_AUTO_TEST_CASE( foot_convex_decomposition_test )
{
    /* Load data */
    load_data("foot", 1.0, 30.0, 2000.0, 2, false, true, true);

    /* Run convex decomposition */
    cd->Compute();

    /* Check results */
    check("foot");
}


BOOST_AUTO_TEST_CASE( GARGOYLE_F_convex_decomposition_test )
{
    /* Load data */
    load_data("GARGOYLE_F", 1.0, 30.0, 2000.0, 2, false, true, true);

    /* Run convex decomposition */
    cd->Compute();

    /* Check results */
    check("GARGOYLE_F");
}


BOOST_AUTO_TEST_CASE( genus3_convex_decomposition_test )
{
    /* Load data */
    load_data("genus3", 1.0, 30.0, 2000.0, 2, false, true, true);

    /* Run convex decomposition */
    cd->Compute();

    /* Check results */
    check("genus3");
}


BOOST_AUTO_TEST_CASE( greek_sculpture_convex_decomposition_test )
{
    /* Load data */
    load_data("greek_sculpture", 1.0, 30.0, 2000.0, 2, false, true, true);

    /* Run convex decomposition */
    cd->Compute();

    /* Check results */
    check("greek_sculpture");
}


BOOST_AUTO_TEST_CASE( Hand1_convex_decomposition_test )
{
    /* Load data */
    load_data("Hand1", 1.0, 30.0, 2000.0, 2, false, true, true);

    /* Run convex decomposition */
    cd->Compute();

    /* Check results */
    check("Hand1");
}


BOOST_AUTO_TEST_CASE( hand2_convex_decomposition_test )
{
    /* Load data */
    load_data("hand2", 1.0, 30.0, 2000.0, 2, false, true, true);

    /* Run convex decomposition */
    cd->Compute();

    /* Check results */
    check("hand2");
}


BOOST_AUTO_TEST_CASE( hand_convex_decomposition_test )
{
    /* Load data */
    load_data("hand", 1.0, 30.0, 2000.0, 2, false, true, true);

    /* Run convex decomposition */
    cd->Compute();

    /* Check results */
    check("hand");
}


BOOST_AUTO_TEST_CASE( helix_convex_decomposition_test )
{
    /* Load data */
    load_data("helix", 1.0, 30.0, 2000.0, 3, false, true, true);

    /* Run convex decomposition */
    cd->Compute();

    /* Check results */
    check("helix");
}


BOOST_AUTO_TEST_CASE( helmet_convex_decomposition_test )
{
    /* Load data */
    load_data("helmet", 1.0, 30.0, 2000.0, 3, false, true, true);

    /* Run convex decomposition */
    cd->Compute();

    /* Check results */
    check("helmet");
}


BOOST_AUTO_TEST_CASE( hero_convex_decomposition_test )
{
    /* Load data */
    load_data("hero", 1.0, 30.0, 2000.0, 3, false, true, true);

    /* Run convex decomposition */
    cd->Compute();

    /* Check results */
    check("hero");
}


BOOST_AUTO_TEST_CASE( homer_convex_decomposition_test )
{
    /* Load data */
    load_data("homer", 1.0, 30.0, 2000.0, 3, false, true, true);

    /* Run convex decomposition */
    cd->Compute();

    /* Check results */
    check("homer");
}


BOOST_AUTO_TEST_CASE( hornbug_convex_decomposition_test )
{
    /* Load data */
    load_data("hornbug", 1.0, 30.0, 2000.0, 3, false, true, true);

    /* Run convex decomposition */
    cd->Compute();

    /* Check results */
    check("hornbug");
}


BOOST_AUTO_TEST_CASE( horse_convex_decomposition_test )
{
    /* Load data */
    load_data("horse", 1.0, 30.0, 2000.0, 3, false, true, true);

    /* Run convex decomposition */
    cd->Compute();

    /* Check results */
    check("horse");
}


BOOST_AUTO_TEST_CASE( maneki_neko_convex_decomposition_test )
{
    /* Load data */
    load_data("maneki-neko", 1.0, 30.0, 2000.0, 3, false, true, true);

    /* Run convex decomposition */
    cd->Compute();

    /* Check results */
    check("maneki-neko");
}


BOOST_AUTO_TEST_CASE( mannequin_devil_convex_decomposition_test )
{
    /* Load data */
    load_data("mannequin-devil", 1.0, 30.0, 2000.0, 3, false, true, true);

    /* Run convex decomposition */
    cd->Compute();

    /* Check results */
    check("mannequin-devil");
}


BOOST_AUTO_TEST_CASE( mannequin_convex_decomposition_test )
{
    /* Load data */
    load_data("mannequin", 1.0, 30.0, 2000.0, 3, false, true, true);

    /* Run convex decomposition */
    cd->Compute();

    /* Check results */
    check("mannequin");
}


BOOST_AUTO_TEST_CASE( mask_convex_decomposition_test )
{
    /* Load data */
    load_data("mask", 1.0, 30.0, 2000.0, 3, false, true, true);

    /* Run convex decomposition */
    cd->Compute();

    /* Check results */
    check("mask");
}


BOOST_AUTO_TEST_CASE( moaimoai_convex_decomposition_test )
{
    /* Load data */
    load_data("moaimoai", 1.0, 30.0, 2000.0, 3, false, true, true);

    /* Run convex decomposition */
    cd->Compute();

    /* Check results */
    check("moaimoai");
}


BOOST_AUTO_TEST_CASE( monk_convex_decomposition_test )
{
    /* Load data */
    load_data("monk", 1.0, 30.0, 2000.0, 3, false, true, true);

    /* Run convex decomposition */
    cd->Compute();

    /* Check results */
    check("monk");
}


BOOST_AUTO_TEST_CASE( octopus_convex_decomposition_test )
{
    /* Load data */
    load_data("octopus", 1.0, 30.0, 2000.0, 3, false, true, true);

    /* Run convex decomposition */
    cd->Compute();

    /* Check results */
    check("octopus");
}


BOOST_AUTO_TEST_CASE( pig_convex_decomposition_test )
{
    /* Load data */
    load_data("pig", 1.0, 30.0, 2000.0, 3, false, true, true);

    /* Run convex decomposition */
    cd->Compute();

    /* Check results */
    check("pig");
}


BOOST_AUTO_TEST_CASE( pinocchio_b_convex_decomposition_test )
{
    /* Load data */
    load_data("pinocchio_b", 1.0, 30.0, 2000.0, 3, false, true, true);

    /* Run convex decomposition */
    cd->Compute();

    /* Check results */
    check("pinocchio_b");
}


BOOST_AUTO_TEST_CASE( polygirl_convex_decomposition_test )
{
    /* Load data */
    load_data("polygirl", 1.0, 30.0, 2000.0, 3, false, true, true);

    /* Run convex decomposition */
    cd->Compute();

    /* Check results */
    check("polygirl");
}


BOOST_AUTO_TEST_CASE( rabbit_convex_decomposition_test )
{
    /* Load data */
    load_data("rabbit", 1.0, 30.0, 2000.0, 3, false, true, true);

    /* Run convex decomposition */
    cd->Compute();

    /* Check results */
    check("rabbit");
}


BOOST_AUTO_TEST_CASE( rocker_arm_convex_decomposition_test )
{
    /* Load data */
    load_data("rocker-arm", 1.0, 30.0, 2000.0, 3, false, true, true);

    /* Run convex decomposition */
    cd->Compute();

    /* Check results */
    check("rocker-arm");
}


BOOST_AUTO_TEST_CASE( RSCREATURE_F_convex_decomposition_test )
{
    /* Load data */
    load_data("RSCREATURE_F", 1.0, 30.0, 2000.0, 3, false, true, true);

    /* Run convex decomposition */
    cd->Compute();

    /* Check results */
    check("RSCREATURE_F");
}


BOOST_AUTO_TEST_CASE( screwdriver_convex_decomposition_test )
{
    /* Load data */
    load_data("screwdriver", 1.0, 30.0, 2000.0, 3, false, true, true);

    /* Run convex decomposition */
    cd->Compute();

    /* Check results */
    check("screwdriver");
}


BOOST_AUTO_TEST_CASE( screw_remeshed_convex_decomposition_test )
{
    /* Load data */
    load_data("screw-remeshed", 1.0, 30.0, 2000.0, 3, false, true, true);

    /* Run convex decomposition */
    cd->Compute();

    /* Check results */
    check("screw-remeshed");
}


BOOST_AUTO_TEST_CASE( shark_b_convex_decomposition_test )
{
    /* Load data */
    load_data("shark_b", 1.0, 30.0, 2000.0, 3, false, true, true);

    /* Run convex decomposition */
    cd->Compute();

    /* Check results */
    check("shark_b");
}


BOOST_AUTO_TEST_CASE( Sketched_Brunnen_convex_decomposition_test )
{
    /* Load data */
    load_data("Sketched-Brunnen", 1.0, 30.0, 2000.0, 3, false, true, true);

    /* Run convex decomposition */
    cd->Compute();

    /* Check results */
    check("Sketched-Brunnen");
}


BOOST_AUTO_TEST_CASE( skull_original_convex_decomposition_test )
{
    /* Load data */
    load_data("skull-original", 1.0, 30.0, 2000.0, 3, false, true, true);

    /* Run convex decomposition */
    cd->Compute();

    /* Check results */
    check("skull-original");
}


BOOST_AUTO_TEST_CASE( sledge_convex_decomposition_test )
{
    /* Load data */
    load_data("sledge", 1.0, 30.0, 2000.0, 3, false, true, true);

    /* Run convex decomposition */
    cd->Compute();

    /* Check results */
    check("sledge");
}


BOOST_AUTO_TEST_CASE( squirrel_convex_decomposition_test )
{
    /* Load data */
    load_data("squirrel", 1.0, 30.0, 2000.0, 3, false, true, true);

    /* Run convex decomposition */
    cd->Compute();

    /* Check results */
    check("squirrel");
}


BOOST_AUTO_TEST_CASE( sword_convex_decomposition_test )
{
    /* Load data */
    load_data("sword", 1.0, 30.0, 2000.0, 3, false, true, true);

    /* Run convex decomposition */
    cd->Compute();

    /* Check results */
    check("sword");
}


BOOST_AUTO_TEST_CASE( table_convex_decomposition_test )
{
    /* Load data */
    load_data("table", 1.0, 30.0, 2000.0, 3, false, true, true);

    /* Run convex decomposition */
    cd->Compute();

    /* Check results */
    check("table");
}


BOOST_AUTO_TEST_CASE( Teapot_convex_decomposition_test )
{
    /* Load data */
    load_data("Teapot", 1.0, 30.0, 2000.0, 3, false, true, true);

    /* Run convex decomposition */
    cd->Compute();

    /* Check results */
    check("Teapot");
}


BOOST_AUTO_TEST_CASE( test2_convex_decomposition_test )
{
    /* Load data */
    load_data("test2", 1.0, 30.0, 2000.0, 3, false, true, true);

    /* Run convex decomposition */
    cd->Compute();

    /* Check results */
    check("test2");
}


BOOST_AUTO_TEST_CASE( test_convex_decomposition_test )
{
    /* Load data */
    load_data("test", 1.0, 30.0, 2000.0, 3, false, true, true);

    /* Run convex decomposition */
    cd->Compute();

    /* Check results */
    check("test");
}


BOOST_AUTO_TEST_CASE( tstTorusModel2_convex_decomposition_test )
{
    /* Load data */
    load_data("tstTorusModel2", 1.0, 30.0, 2000.0, 3, false, true, true);

    /* Run convex decomposition */
    cd->Compute();

    /* Check results */
    check("tstTorusModel2");
}


BOOST_AUTO_TEST_CASE( tstTorusModel3_convex_decomposition_test )
{
    /* Load data */
    load_data("tstTorusModel3", 1.0, 30.0, 2000.0, 3, false, true, true);

    /* Run convex decomposition */
    cd->Compute();

    /* Check results */
    check("tstTorusModel3");
}


BOOST_AUTO_TEST_CASE( tstTorusModel_convex_decomposition_test )
{
    /* Load data */
    load_data("tstTorusModel", 1.0, 30.0, 2000.0, 3, false, true, true);

    /* Run convex decomposition */
    cd->Compute();

    /* Check results */
    check("tstTorusModel");
}


BOOST_AUTO_TEST_CASE( tube1_convex_decomposition_test )
{
    /* Load data */
    load_data("tube1", 1.0, 30.0, 2000.0, 3, false, true, true);

    /* Run convex decomposition */
    cd->Compute();

    /* Check results */
    check("tube1");
}


BOOST_AUTO_TEST_CASE( venus_convex_decomposition_test )
{
    /* Load data */
    load_data("venus", 1.0, 30.0, 2000.0, 3, false, true, true);

    /* Run convex decomposition */
    cd->Compute();

    /* Check results */
    check("venus");
}


BOOST_AUTO_TEST_CASE( venus_original_convex_decomposition_test )
{
    /* Load data */
    load_data("venus-original", 1.0, 30.0, 2000.0, 3, false, true, true);

    /* Run convex decomposition */
    cd->Compute();

    /* Check results */
    check("venus-original");
}

BOOST_AUTO_TEST_SUITE_END()
