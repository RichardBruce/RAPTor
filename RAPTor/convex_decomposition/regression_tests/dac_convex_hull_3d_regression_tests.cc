#ifdef STAND_ALONE
#define BOOST_TEST_MODULE regression test
#endif /* #ifdef STAND_ALONE */


/* Standard headers */
#include <chrono>

/* Boost headers */
#include "boost/test/unit_test.hpp"
#include "boost/noncopyable.hpp"

/* Common headers */
#include "logging.h"

/* Parser headers */
#include "parser.h"

/* Convex decomposition headers */
#include "dac_convex_hull_3d.h"

/* Test headers */


namespace raptor_convex_decomposition
{
int dac_convex_hull_3d::_merge_num = 0;

namespace test
{
/* Test data */
const float result_tolerance = 0.00001f;

#define LOAD_TEST_DATA(FILE)   const std::string test_name(boost::unit_test::framework::current_test_case().p_name); \
    BOOST_LOG_TRIVIAL(fatal) << "PERF 0 - Test: " << test_name; \
    const std::string data_dir(getenv("RAPTOR_DATA")); \
    BOOST_REQUIRE(raptor_parsers::load_off(data_dir + "/" + (FILE), &points, &triangles));


struct dac_convex_hull_3d_regression_fixture : private boost::noncopyable
{
    dac_convex_hull_3d_regression_fixture() {  }

    void run()
    {
        BOOST_LOG_TRIVIAL(fatal) << "PERF 1 - # Vertices: " << points.size();
        const auto t0(std::chrono::system_clock::now());
        triangles_act = build(points, &points_act);
        const auto t1(std::chrono::system_clock::now());
        BOOST_LOG_TRIVIAL(fatal) << "PERF 2 - Runtime us: " << std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
    }

    void check(const std::string &file)
    {
        /* Save actual results */
        raptor_parsers::save_off(points_act, triangles_act, "test_data/" + file + "_act.off");
        raptor_parsers::save_vrml2("test_data/" + file + "_act.wrl", points_act, triangles_act, 1.0f, 1.0f, 1.0f);
        
        /* Load expected data */
        std::vector<point_t<>>  points_exp;
        std::vector<point_ti<>> triangles_exp;
        BOOST_REQUIRE(raptor_parsers::load_off("test_data/" + file + "_exp.off", &points_exp, &triangles_exp));

        /* Check points */
        BOOST_REQUIRE(points_act.size() == points_exp.size());
        for (size_t i = 0; i < points_exp.size(); ++i)
        {
            if (std::fabs(magnitude(points_act[i] - points_exp[i])) > result_tolerance)
            {
                BOOST_CHECK(std::fabs(magnitude(points_act[i] - points_exp[i])) < result_tolerance);
            }
        }

        /* Check triangles */
        BOOST_REQUIRE(triangles_act.size() == triangles_exp.size());
        for (size_t i = 0; i < triangles_exp.size(); ++i)
        {
            BOOST_CHECK(triangles_act[i] == triangles_exp[i]);
        }
    }

    std::vector<point_t<>>  points;
    std::vector<point_ti<>> triangles;
    std::vector<point_t<>>  points_act;
    std::vector<point_ti<>> triangles_act;
};


BOOST_FIXTURE_TEST_SUITE( regression_tests, dac_convex_hull_3d_regression_fixture )


/* Tests */
#ifndef VALGRIND_TESTS
BOOST_AUTO_TEST_CASE( block_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/block.off");

    /* Hull */
    run();

    /* Save resutls */
    check("block");
}

BOOST_AUTO_TEST_CASE( bunny_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/bunny.off");

    /* Hull */
    run();

    /* Save resutls */
    check("bunny");
}

BOOST_AUTO_TEST_CASE( camel_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/camel.off");

    /* Hull */
    run();

    /* Save resutls */
    check("camel");
}

BOOST_AUTO_TEST_CASE( casting_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/casting.off");

    /* Hull */
    run();

    /* Save resutls */
    check("casting");
}

BOOST_AUTO_TEST_CASE( chair_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/chair.off");

    /* Hull */
    run();

    /* Save resutls */
    check("chair");
}

BOOST_AUTO_TEST_CASE( cow1_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/cow1.off");

    /* Hull */
    run();

    /* Save resutls */
    check("cow1");
}

BOOST_AUTO_TEST_CASE( cow2_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/cow2.off");

    /* Hull */
    run();

    /* Save resutls */
    check("cow2");
}
#endif /* #ifndef VALGRIND_TESTS */

BOOST_AUTO_TEST_CASE( crank_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/crank.off");

    /* Hull */
    run();

    /* Save resutls */
    check("crank");
}

#ifndef VALGRIND_TESTS
BOOST_AUTO_TEST_CASE( cup_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/cup.off");

    /* Hull */
    run();

    /* Save resutls */
    check("cup");
}

BOOST_AUTO_TEST_CASE( dancer2_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/dancer2.off");

    /* Hull */
    run();

    /* Save resutls */
    check("dancer2");
}

BOOST_AUTO_TEST_CASE( deer_bound_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/deer_bound.off");

    /* Hull */
    run();

    /* Save resutls */
    check("deer_bound");
}

BOOST_AUTO_TEST_CASE( dilo_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/dilo.off");

    /* Hull */
    run();

    /* Save resutls */
    check("dilo");
}

BOOST_AUTO_TEST_CASE( dino_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/dino.off");

    /* Hull */
    run();

    /* Save resutls */
    check("dino");
}

BOOST_AUTO_TEST_CASE( DRAGON_F_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/DRAGON_F.off");

    /* Hull */
    run();

    /* Save resutls */
    check("DRAGON_F");
}

BOOST_AUTO_TEST_CASE( drum_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/drum.off");

    /* Hull */
    run();

    /* Save resutls */
    check("drum");
}

BOOST_AUTO_TEST_CASE( eight_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/eight.off");

    /* Hull */
    run();

    /* Save resutls */
    check("eight");
}

BOOST_AUTO_TEST_CASE( elephant_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/elephant.off");

    /* Hull */
    run();

    /* Save resutls */
    check("elephant");
}

BOOST_AUTO_TEST_CASE( elk_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/elk.off");

    /* Hull */
    run();

    /* Save resutls */
    check("elk");
}

BOOST_AUTO_TEST_CASE( egea_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/egea.off");

    /* Hull */
    run();

    /* Save resutls */
    check("egea");
}

BOOST_AUTO_TEST_CASE( face_yh_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/face-YH.off");

    /* Hull */
    run();

    /* Save resutls */
    check("face-YH");
}

BOOST_AUTO_TEST_CASE( feline_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/feline.off");

    /* Hull */
    run();

    /* Save resutls */
    check("feline");
}

BOOST_AUTO_TEST_CASE( fish_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/fish.off");

    /* Hull */
    run();

    /* Save resutls */
    check("fish");
}

BOOST_AUTO_TEST_CASE( foot_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/foot.off");

    /* Hull */
    run();

    /* Save resutls */
    check("foot");
}

BOOST_AUTO_TEST_CASE( genus3_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/genus3.off");

    /* Hull */
    run();

    /* Save resutls */
    check("genus3");
}

BOOST_AUTO_TEST_CASE( greek_sculpture_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/greek_sculpture.off");

    /* Hull */
    run();

    /* Save resutls */
    check("greek_sculpture");
}

BOOST_AUTO_TEST_CASE( hand1_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/Hand1.off");

    /* Hull */
    run();

    /* Save resutls */
    check("Hand1");
}

BOOST_AUTO_TEST_CASE( hand2_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/hand2.off");

    /* Hull */
    run();

    /* Save resutls */
    check("hand2");
}

BOOST_AUTO_TEST_CASE( horse_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/horse.off");

    /* Hull */
    run();

    /* Save resutls */
    check("horse");
}

BOOST_AUTO_TEST_CASE( helix_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/helix.off");

    /* Hull */
    run();

    /* Save resutls */
    check("helix");
}

BOOST_AUTO_TEST_CASE( helmet_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/helmet.off");

    /* Hull */
    run();

    /* Save resutls */
    check("helmet");
}

BOOST_AUTO_TEST_CASE( hero_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/hero.off");

    /* Hull */
    run();

    /* Save resutls */
    check("hero");
}

BOOST_AUTO_TEST_CASE( homer_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/homer.off");

    /* Hull */
    run();

    /* Save resutls */
    check("homer");
}

BOOST_AUTO_TEST_CASE( hornbug_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/hornbug.off");

    /* Hull */
    run();

    /* Save resutls */
    check("hornbug");
}

BOOST_AUTO_TEST_CASE( maneki_neko_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/maneki-neko.off");

    /* Hull */
    run();

    /* Save resutls */
    check("maneki-neko");
}

BOOST_AUTO_TEST_CASE( mannequin_devil_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/mannequin-devil.off");

    /* Hull */
    run();

    /* Save resutls */
    check("mannequin-devil");
}

BOOST_AUTO_TEST_CASE( mannequin_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/mannequin.off");

    /* Hull */
    run();

    /* Save resutls */
    check("mannequin");
}

BOOST_AUTO_TEST_CASE( mask_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/mask.off");

    /* Hull */
    run();

    /* Save resutls */
    check("mask");
}

BOOST_AUTO_TEST_CASE( moaimoai_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/moaimoai.off");

    /* Hull */
    run();

    /* Save resutls */
    check("moaimoai");
}

BOOST_AUTO_TEST_CASE( monk_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/monk.off");

    /* Hull */
    run();

    /* Save resutls */
    check("monk");
}

BOOST_AUTO_TEST_CASE( octopus_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/octopus.off");

    /* Hull */
    run();

    /* Save resutls */
    check("octopus");
}

BOOST_AUTO_TEST_CASE( pig_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/pig.off");

    /* Hull */
    run();

    /* Save resutls */
    check("pig");
}

BOOST_AUTO_TEST_CASE( pinocchio_b_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/pinocchio_b.off");

    /* Hull */
    run();

    /* Save resutls */
    check("pinocchio_b");
}

BOOST_AUTO_TEST_CASE( polygirl_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/polygirl.off");

    /* Hull */
    run();

    /* Save resutls */
    check("polygirl");
}

BOOST_AUTO_TEST_CASE( rabbit_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/rabbit.off");

    /* Hull */
    run();

    /* Save resutls */
    check("rabbit");
}

BOOST_AUTO_TEST_CASE( rocker_arm_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/rocker-arm.off");

    /* Hull */
    run();

    /* Save resutls */
    check("rocker-arm");
}

BOOST_AUTO_TEST_CASE( screwdriver_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/screwdriver.off");

    /* Hull */
    run();

    /* Save resutls */
    check("screwdriver");
}

BOOST_AUTO_TEST_CASE( shark_b_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/shark_b.off");

    /* Hull */
    run();

    /* Save resutls */
    check("shark_b");
}

BOOST_AUTO_TEST_CASE( sketched_brunnen_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/Sketched-Brunnen.off");

    /* Hull */
    run();

    /* Save resutls */
    check("Sketched-Brunnen");
}

BOOST_AUTO_TEST_CASE( sledge_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/sledge.off");

    /* Hull */
    run();

    /* Save resutls */
    check("sledge");
}

BOOST_AUTO_TEST_CASE( squirrel_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/squirrel.off");

    /* Hull */
    run();

    /* Save resutls */
    check("squirrel");
}

BOOST_AUTO_TEST_CASE( sword_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/sword.off");

    /* Hull */
    run();

    /* Save resutls */
    check("sword");
}

BOOST_AUTO_TEST_CASE( table_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/table.off");

    /* Hull */
    run();

    /* Save resutls */
    check("table");
}

BOOST_AUTO_TEST_CASE( teapot_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/Teapot.off");

    /* Hull */
    run();

    /* Save resutls */
    check("Teapot");

}

BOOST_AUTO_TEST_CASE( test2_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/test2.off");

    /* Hull */
    run();

    /* Save resutls */
    check("test2");
}

BOOST_AUTO_TEST_CASE( test_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/test.off");

    /* Hull */
    run();

    /* Save resutls */
    check("test");
}

BOOST_AUTO_TEST_CASE( torus_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/tstTorusModel2.off");

    /* Hull */
    run();

    /* Save resutls */
    check("tstTorusModel2");
}

BOOST_AUTO_TEST_CASE( tstTorusModel3_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/tstTorusModel3.off");

    /* Hull */
    run();

    /* Save resutls */
    check("tstTorusModel3");
}

BOOST_AUTO_TEST_CASE( tstTorusModel_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/tstTorusModel.off");

    /* Hull */
    run();

    /* Save resutls */
    check("tstTorusModel");
}

BOOST_AUTO_TEST_CASE( tube1_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/tube1.off");

    /* Hull */
    run();

    /* Save resutls */
    check("tube1");
}

BOOST_AUTO_TEST_CASE( venus_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/venus.off");

    /* Hull */
    run();

    /* Save resutls */
    check("venus");
}

BOOST_AUTO_TEST_CASE( venus_original_test )
{
    /* Load file */
    LOAD_TEST_DATA("off_scenes/venus-original.off");

    /* Hull */
    run();

    /* Save resutls */
    check("venus-original");
}
#endif /* #ifndef VALGRIND_TESTS */

BOOST_AUTO_TEST_SUITE_END()
} /* namespace test */
} /* namespace raptor_convex_decomposition */
