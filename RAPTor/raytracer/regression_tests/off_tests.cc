#ifdef STAND_ALONE
#define BOOST_TEST_MODULE off test

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
BOOST_AUTO_TEST_SUITE( off_tests );

BOOST_AUTO_TEST_CASE( off_block_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/off_scenes/block.off", model_format_t::off, point_t(41.3208, 21.393, 19.2579), point_t(-0.392289, -0.0249371, 0.919504), point_t(-0.425708, 0.891055, -0.157455), point_t(-0.815402, -0.453208, -0.360167), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(50.0, 20.0, -7.0), 0.0, 10.0);

    /* Ray trace the scene using bvh */
    fixture.render<bvh>();
    checker.check(fixture.get_camera(), "bvh");

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( off_tst_torus_model_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/off_scenes/tstTorusModel.off", model_format_t::off, point_t(75.0, -100.0, -11.0), point_t(-0.0176554, 0.0259792, 0.999507), point_t(0.82708, 0.562083, 0.0), point_t(-0.561805, 0.826672, -0.0314107), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(50.0, -200.0, -70.0), 0.0, 10.0);

    /* Ray trace the scene using bvh */
    fixture.render<bvh>();
    checker.check(fixture.get_camera(), "bvh");

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

#ifndef VALGRIND_TESTS
BOOST_AUTO_TEST_CASE( off_bowl_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/off_scenes/bowl.off", model_format_t::off, point_t(220.0, 210.0, 43.0), point_t(0.0, 0.0, 1.0), point_t(-0.481753, 0.876306, 0.0), point_t(-0.876306, -0.481753, 0.0), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(500.0, 300.0, 300.0), 0.0, 10.0);

    /* Ray trace the scene using bvh */
    fixture.render<bvh>();
    checker.check(fixture.get_camera(), "bvh");

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( off_bunny_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/off_scenes/bunny.off", model_format_t::off, point_t(0.193065, -0.215, -0.229911), point_t(-0.728834, 0.0196274, -0.684416), point_t(-0.248479, -0.939028, 0.237677), point_t(-0.638019, 0.343288, 0.689266), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(5.0, 2.0, -7.0), 0.0, 10.0);

    /* Ray trace the scene using bvh */
    fixture.render<bvh>();
    checker.check(fixture.get_camera(), "bvh");

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( off_camel_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/off_scenes/camel.off", model_format_t::off, point_t(2.0, 0.0, 0.0), point_t(0.0, 0.0, 1.0), point_t(0.0, 1.0, 0.0), point_t(-1.0, 0.0, 0.0), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(5.0, 20.0, -4.0), 0.0, 10.0);

    /* Ray trace the scene using bvh */
    fixture.render<bvh>();
    checker.check(fixture.get_camera(), "bvh");

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( off_casting_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/off_scenes/casting.off", model_format_t::off, point_t(1.12369, 0.518247, 0.0), point_t(0.0, 0.0, 1.0), point_t(-0.509041, 0.860741, 0.0), point_t(-0.860741, -0.509041, 0.0), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(5.0, 20.0, -7.0), 0.0, 10.0);

    /* Ray trace the scene using bvh */
    fixture.render<bvh>();
    checker.check(fixture.get_camera(), "bvh");

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( off_chair_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/off_scenes/chair.off", model_format_t::off, point_t(3.5, 1.35, 1.5), point_t(-0.425569, -0.013374, 0.904827), point_t(-0.109329, 0.993327, -0.0367389), point_t(-0.898297, -0.114559, -0.42419), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(5.0, 20.0, -7.0), 0.0, 10.0);

    /* Ray trace the scene using bvh */
    fixture.render<bvh>();
    checker.check(fixture.get_camera(), "bvh");

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( off_cow_1_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/off_scenes/cow1.off", model_format_t::off, point_t(2.08752, 0.722247, 0.699606), point_t(-0.30537, -0.0262554, 0.951871), point_t(-0.356935, 0.929893, -0.0888594), point_t(-0.882805, -0.366892, -0.293333), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(5.0, 20.0, -7.0), 0.0, 10.0);

    /* Ray trace the scene using bvh */
    fixture.render<bvh>();
    checker.check(fixture.get_camera(), "bvh");

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( off_cow_2_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/off_scenes/cow2.off", model_format_t::off, point_t(1.57886, 0.72, 0.817758), point_t(-0.410329, -0.00623812, 0.911916), point_t(-0.364484, 0.917754, -0.157726), point_t(-0.835931, -0.397098, -0.37885), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(5.0, 20.0, -7.0), 0.0, 10.0);

    /* Ray trace the scene using bvh */
    fixture.render<bvh>();
    checker.check(fixture.get_camera(), "bvh");

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( off_crank_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/off_scenes/crank.off", model_format_t::off, point_t(1.94639, 0.952701, 0.856727), point_t(-0.362183, -0.0387301, 0.931302), point_t(-0.387723, 0.914856, -0.112739), point_t(-0.84764, -0.401919, -0.34636), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(5.0, 20.0, -7.0), 0.0, 10.0);

    /* Ray trace the scene using bvh */
    fixture.render<bvh>();
    checker.check(fixture.get_camera(), "bvh");

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( off_craters_f_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/off_scenes/CRATERS_F.off", model_format_t::off, point_t(711.71, -12.8372, 522.743), point_t(-0.281943, -0.958773, -0.0353766), point_t(-0.609648, 0.150561, 0.778235), point_t(-0.740831, 0.240988, -0.62696), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(700.0, 800.0, 500.0), 0.0, 10.0);

    /* Ray trace the scene using bvh */
    fixture.render<bvh>();
    checker.check(fixture.get_camera(), "bvh");

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( off_cup_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/off_scenes/cup.off", model_format_t::off, point_t(40.2872, 19.3066, 18.7816), point_t(-0.389472, -0.0804029, 0.917522), point_t(-0.405798, 0.909262, -0.0925751), point_t(-0.826824, -0.408384, -0.386759), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(50.0, 20.0, -7.0), 0.0, 10.0);

    /* Ray trace the scene using bvh */
    fixture.render<bvh>();
    checker.check(fixture.get_camera(), "bvh");

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( off_dancer_2_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/off_scenes/dancer2.off", model_format_t::off, point_t(260.0, 1.82375, -675.0), point_t(-0.647372, 0.550893, 0.526708), point_t(0.464634, 0.833053, -0.300228), point_t(-0.604168, 0.0503669, -0.79525), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(5.0, 20.0, -7.0), 0.0, 10.0);

    /* Ray trace the scene using bvh */
    fixture.render<bvh>();
    checker.check(fixture.get_camera(), "bvh");

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( off_deer_bound_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/off_scenes/deer_bound.off", model_format_t::off, point_t(3.15191, 1.62542, 1.33347), point_t(-0.338209, -0.00588578, 0.941052), point_t(-0.207147, 0.975919, -0.0683436), point_t(-0.917989, -0.218051, -0.331284), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(5.0, 20.0, -7.0), 0.0, 10.0);

    /* Ray trace the scene using bvh */
    fixture.render<bvh>();
    checker.check(fixture.get_camera(), "bvh");

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( off_dilo_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/off_scenes/dilo.off", model_format_t::off, point_t(-0.75, -0.32, 0.7), point_t(0.683897, -0.0155865, 0.729405), point_t(0.175107, -0.967038, -0.184847), point_t(0.708247, 0.254142, -0.658629), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-5.0, -5.0, 7.0), 0.0, 10.0);

    /* Ray trace the scene using bvh */
    fixture.render<bvh>();
    checker.check(fixture.get_camera(), "bvh");

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( off_dino_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/off_scenes/dino.off", model_format_t::off, point_t(0.673974, 0.9, -1.5), point_t(0.916211, -0.198166, 0.348258), point_t(0.00180433, 0.871174, 0.490969), point_t(-0.400688, -0.449204, 0.798538), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(5.0, 20.0, -7.0), 0.0, 10.0);

    /* Ray trace the scene using bvh */
    fixture.render<bvh>();
    checker.check(fixture.get_camera(), "bvh");

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( off_dragon_f_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/off_scenes/DRAGON_F.off", model_format_t::off, point_t(650.0, 120.639, 320.0), point_t(0.148673, -0.986936, 0.0620931), point_t(-0.215719, 0.0289108, 0.976028), point_t(-0.965071, -0.158503, -0.208603), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(550.0, -120.0, 0.0), 0.0, 10.0);

    /* Ray trace the scene using bvh */
    fixture.render<bvh>();
    checker.check(fixture.get_camera(), "bvh");

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( off_drum_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/off_scenes/drum.off", model_format_t::off, point_t(4.0, 2.5, 4.0), point_t(-0.739334, -0.00366679, 0.673328), point_t(-0.180271, 0.964558, -0.19269), point_t(-0.648757, -0.263844, -0.713791), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(70.0, 200.0, 50.0), 0.0, 10.0);

    /* Ray trace the scene using bvh */
    fixture.render<bvh>();
    checker.check(fixture.get_camera(), "bvh");

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( off_egea_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/off_scenes/egea.off", model_format_t::off, point_t(2.48955, 0.95, -0.850761), point_t(0.315956, -0.0690505, 0.946257), point_t(-0.19083, 0.972341, 0.134672), point_t(-0.929384, -0.223125, 0.294), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(5.0, 20.0, -7.0), 0.0, 10.0);

    /* Ray trace the scene using bvh */
    fixture.render<bvh>();
    checker.check(fixture.get_camera(), "bvh");

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( off_eight_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/off_scenes/eight.off", model_format_t::off, point_t(0.863258, 0.883035, 0.0), point_t(0.0, 0.0, 1.0), point_t(-0.734326, 0.678804, 0.0), point_t(-0.678804, -0.734326, 0.0), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(5.0, 20.0, -7.0), 0.0, 10.0);

    /* Ray trace the scene using bvh */
    fixture.render<bvh>();
    checker.check(fixture.get_camera(), "bvh");

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( off_elephant_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/off_scenes/elephant.off", model_format_t::off, point_t(65.0, 1.0, -1.9681), point_t(0.0314108, 0.0, 0.999506), point_t(0.0, 1.0, 0.0), point_t(-0.999506, 0.0, 0.0314108), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(55.0, 0.0, 8.0), 0.0, 10.0);

    /* Ray trace the scene using bvh */
    fixture.render<bvh>();
    checker.check(fixture.get_camera(), "bvh");

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( off_elk_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/off_scenes/elk.off", model_format_t::off, point_t(350.0, 75.0, -102.36), point_t(0.24869, 0.0, 0.968583), point_t(-0.226111, 0.97237, 0.0580555), point_t(-0.94182, -0.233445, 0.241818), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(80.0, 200.0, 1000.0), 0.0, 10.0);

    /* Ray trace the scene using bvh */
    fixture.render<bvh>();
    checker.check(fixture.get_camera(), "bvh");

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( off_face_yh_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/off_scenes/face-YH.off", model_format_t::off, point_t(107.979, 56.0, -854.313), point_t(-0.968579, 0.0, 0.248689), point_t(-0.031169, 0.992113, -0.121395), point_t(-0.246728, -0.125332, -0.960939), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(5.0, 20.0, -7.0), 0.0, 10.0);

    /* Ray trace the scene using bvh */
    fixture.render<bvh>();
    checker.check(fixture.get_camera(), "bvh");

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( off_feline_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/off_scenes/feline.off", model_format_t::off, point_t(1.75, 1.75, -2.5), point_t(-0.770511, 0.637423, 0.0), point_t(-0.243931, -0.294862, -0.923878), point_t(-0.588903, -0.71186, 0.382682), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(5.0, 20.0, -7.0), 0.0, 10.0);


    /* Ray trace the scene using bvh */
    fixture.render<bvh>();
    checker.check(fixture.get_camera(), "bvh");

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( off_fish_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/off_scenes/fish.off", model_format_t::off, point_t(42.6612, -5.0, -759.0), point_t(-0.99211, 0.0, 0.125333), point_t(0.0, 1.0, 0.0), point_t(-0.125333, 0.0, -0.99211), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(5.0, 20.0, -7.0), 0.0, 10.0);

    /* Ray trace the scene using bvh */
    fixture.render<bvh>();
    checker.check(fixture.get_camera(), "bvh");

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( off_foot_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/off_scenes/foot.off", model_format_t::off, point_t(2.97234, -0.871201, 0.127428), point_t(-0.248662, -0.968577, 0.00399815), point_t(-0.298219, 0.0804875, 0.951093), point_t(-0.921536, 0.235311, -0.308865), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(5.0, 0.0, -5.0), 0.0, 10.0);

    /* Ray trace the scene using bvh */
    fixture.render<bvh>();
    checker.check(fixture.get_camera(), "bvh");

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( off_gargoyle_f_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/off_scenes/GARGOYLE_F.off", model_format_t::off, point_t(17.0, 0.5, -9.0), point_t(-0.313335, -0.473085, -0.82341), point_t(0.623628, -0.756408, 0.197278), point_t(-0.716168, -0.451691, 0.532), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-5.0, -10.0, -15.0), 0.0, 10.0);

    /* Ray trace the scene using bvh */
    fixture.render<bvh>();
    checker.check(fixture.get_camera(), "bvh");

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( off_genus_3_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/off_scenes/genus3.off", model_format_t::off, point_t(1.0, -0.65, 0.40), point_t(-0.45399, 0.0, 0.891006), point_t(0.385692, 0.901455, 0.19652), point_t(-0.803202, 0.432872, -0.409252), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(5.0, -20.0, 7.0), 0.0, 10.0);

    /* Ray trace the scene using bvh */
    fixture.render<bvh>();
    checker.check(fixture.get_camera(), "bvh");

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( off_greek_sculpture_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/off_scenes/greek_sculpture.off", model_format_t::off, point_t(110.299, 3.0, -200.0), point_t(-0.975912, 0.0, 0.218143), point_t(0.0, 1.0, 0.0), point_t(-0.218143, 0.0, -0.975912), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(5.0, 20.0, -7.0), 0.0, 10.0);

    /* Ray trace the scene using bvh */
    fixture.render<bvh>();
    checker.check(fixture.get_camera(), "bvh");

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( off_hand_1_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/off_scenes/Hand1.off", model_format_t::off, point_t(-150.0, 1.0, -525.0), point_t(-0.852636, 0.0, -0.522495), point_t(0.0, 1.0, 0.0), point_t(0.522495, 0.0, -0.852636), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(5.0, 20.0, -7.0), 0.0, 10.0);

    /* Ray trace the scene using bvh */
    fixture.render<bvh>();
    checker.check(fixture.get_camera(), "bvh");

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( off_hand_2_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/off_scenes/hand2.off", model_format_t::off, point_t(160.0, 150.0, -575.0), point_t(-0.82846, 0.0124746, 0.5599), point_t(-0.220912, 0.911407, -0.347181), point_t(-0.514628, -0.411314, -0.752308), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(5.0, 20.0, -7.0), 0.0, 10.0);

    /* Ray trace the scene using bvh */
    fixture.render<bvh>();
    checker.check(fixture.get_camera(), "bvh");

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( off_hand_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/off_scenes/hand.off", model_format_t::off, point_t(1.5, 0.8, 1.25), point_t(-0.613161, -0.0820631, 0.785684), point_t(-0.353861, 0.917754, -0.180301), point_t(-0.706269, -0.388577, -0.59177), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(5.0, 20.0, 7.0), 0.0, 10.0);

    /* Ray trace the scene using bvh */
    fixture.render<bvh>();
    checker.check(fixture.get_camera(), "bvh");

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( off_helix_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/off_scenes/helix.off", model_format_t::off, point_t(45.0, 6.0, 55.8754), point_t(0.157185, -0.0302495, 0.987103), point_t(-0.0496142, 0.998027, 0.0384847), point_t(-0.98632, -0.0550235, 0.155374), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(5.0, 20.0, -7.0), 0.0, 10.0);

    /* Ray trace the scene using bvh */
    fixture.render<bvh>();
    checker.check(fixture.get_camera(), "bvh");

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( off_helmet_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/off_scenes/helmet.off", model_format_t::off, point_t(-1.5, 0.0, 1.2), point_t(-0.103453, 0.987684, -0.117346), point_t(0.653168, 0.156437, 0.740875), point_t(0.75011, 0.0, -0.6613), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-5.0, 20.0, 7.0), 0.0, 10.0);

    /* Ray trace the scene using bvh */
    fixture.render<bvh>();
    checker.check(fixture.get_camera(), "bvh");

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( off_hero_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/off_scenes/hero.off", model_format_t::off, point_t(275.0, 150.0, 275.0), point_t(-0.684546, 0.0, 0.728967), point_t(-0.0913638, 0.992115, -0.0857963), point_t(-0.723219, -0.125333, -0.679148), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(200.0, 125.0, 200.0), 0.0, 10.0);

    /* Ray trace the scene using bvh */
    fixture.render<bvh>();
    checker.check(fixture.get_camera(), "bvh");

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( off_homer_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/off_scenes/homer.off", model_format_t::off, point_t(1.25, 0.7, 1.25), point_t(-0.680231, -0.0101181, 0.732927), point_t(-0.276134, 0.929776, -0.243445), point_t(-0.678995, -0.367985, -0.635257), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(5.0, 20.0, 7.0), 0.0, 10.0);

    /* Ray trace the scene using bvh */
    fixture.render<bvh>();
    checker.check(fixture.get_camera(), "bvh");

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( off_hornbug_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/off_scenes/hornbug.off", model_format_t::off, point_t(2.54503, 1.44781, 1.72043), point_t(-0.509041, 0.0, 0.860741), point_t(-0.31686, 0.929776, -0.187391), point_t(-0.800297, -0.368124, -0.47329), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(5.0, 20.0, -7.0), 0.0, 10.0);

    /* Ray trace the scene using bvh */
    fixture.render<bvh>();
    checker.check(fixture.get_camera(), "bvh");

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( off_horse_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/off_scenes/horse.off", model_format_t::off, point_t(-1.07136, 0.657388, -0.496045), point_t(0.348318, 0.0267266, -0.936992), point_t(0.458956, 0.86672, 0.195335), point_t(0.81733, -0.498076, 0.28962), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(5.0, 20.0, -7.0), 0.0, 10.0);

    /* Ray trace the scene using bvh */
    fixture.render<bvh>();
    checker.check(fixture.get_camera(), "bvh");

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( off_maneki_neko_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/off_scenes/maneki-neko.off", model_format_t::off, point_t(1.5, 19.75, 42.0), point_t(-0.980458, -0.162342, 0.111101), point_t(-0.196428, 0.838627, -0.508054), point_t(-0.0106944, -0.519949, -0.854127), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(5.0, 20.0, 70.0), 0.0, 10.0);

    /* Ray trace the scene using bvh */
    fixture.render<bvh>();
    checker.check(fixture.get_camera(), "bvh");

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( off_mannequin_devil_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/off_scenes/mannequin-devil.off", model_format_t::off, point_t(43.0, -57.0, 24.0), point_t(-0.788129, -0.612379, -0.0619777), point_t(-0.243031, 0.217096, 0.94541), point_t(-0.565495, 0.76017, -0.31992), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(50.0, -20.0, 70.0), 0.0, 10.0);

    /* Ray trace the scene using bvh */
    fixture.render<bvh>();
    checker.check(fixture.get_camera(), "bvh");

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( off_mannequin_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/off_scenes/mannequin.off", model_format_t::off, point_t(4.7, -3.5, 2.3), point_t(-0.620862, -0.782801, 0.0418044), point_t(-0.278037, 0.269753, 0.921913), point_t(-0.732954, 0.56076, -0.385129), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(5.0, -20.0, 7.0), 0.0, 10.0);

    /* Ray trace the scene using bvh */
    fixture.render<bvh>();
    checker.check(fixture.get_camera(), "bvh");

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( off_mask_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/off_scenes/mask.off", model_format_t::off, point_t(2.0, 1.0, 3.0), point_t(-0.823748, -0.0194411, 0.566623), point_t(-0.166175, 0.963799, -0.208515), point_t(-0.542056, -0.265922, -0.797158), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(5.0, 20.0, 7.0), 0.0, 10.0);

    /* Ray trace the scene using bvh */
    fixture.render<bvh>();
    checker.check(fixture.get_camera(), "bvh");

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( off_moaimoai_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/off_scenes/moaimoai.off", model_format_t::off, point_t(13.5, 4.1, 13.5), point_t(-0.750111, 0.0, 0.661311), point_t(-0.144261, 0.975917, -0.163632), point_t(-0.645384, -0.218143, -0.732046), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(5.0, 20.0, 7.0), 0.0, 10.0);

    /* Ray trace the scene using bvh */
    fixture.render<bvh>();
    checker.check(fixture.get_camera(), "bvh");

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( off_monk_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/off_scenes/monk.off", model_format_t::off, point_t(9.0, -5.8, -26.0), point_t(-0.982282, 0.0176346, -0.186552), point_t(-0.0466003, -0.987271, 0.152047), point_t(-0.181498, 0.158047, 0.970606), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(5.0, -20.0, -7.0), 0.0, 10.0);

    /* Ray trace the scene using bvh */
    fixture.render<bvh>();
    checker.check(fixture.get_camera(), "bvh");

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( off_octopus_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/off_scenes/octopus.off", model_format_t::off, point_t(1.375, 0.0, 0.0), point_t(0.0, 0.0, 1.0), point_t(0.0, 1.0, 0.0), point_t(-1.0, 0.0, 0.0), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(5.0, 20.0, -7.0), 0.0, 10.0);

    /* Ray trace the scene using bvh */
    fixture.render<bvh>();
    checker.check(fixture.get_camera(), "bvh");

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( off_pig_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/off_scenes/pig.off", model_format_t::off, point_t(0.8, -0.5, -1.1), point_t(-0.770509, 0.0200222, -0.637109), point_t(-0.266863, -0.917825, 0.293895), point_t(-0.578874, 0.396472, 0.71254), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(5.0, -20.0, -7.0), 0.0, 10.0);

    /* Ray trace the scene using bvh */
    fixture.render<bvh>();
    checker.check(fixture.get_camera(), "bvh");

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( off_pinocchio_b_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/off_scenes/pinocchio_b.off", model_format_t::off, point_t(60000.0, 28000.0, 105000.0), point_t(-0.784658, -0.240871, 0.571208), point_t(-0.339035, 0.938142, -0.0701245), point_t(-0.518984, -0.248684, -0.81778), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(50000.0, 20000.0, 80000.0), 0.0, 10.0);

    /* Ray trace the scene using bvh */
    fixture.render<bvh>();
    checker.check(fixture.get_camera(), "bvh");

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( off_polygirl_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/off_scenes/polygirl.off", model_format_t::off, point_t(1.75, 0.219686, 0.78), point_t(0.164187, -0.982051, -0.0928314), point_t(-0.381065, -0.149948, 0.912308), point_t(-0.909853, -0.114414, -0.398846), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(5.0, 20.0, 7.0), 0.0, 10.0);

    /* Ray trace the scene using bvh */
    fixture.render<bvh>();
    checker.check(fixture.get_camera(), "bvh");

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( off_rabbit_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/off_scenes/rabbit.off", model_format_t::off, point_t(0.16, -0.04, 0.06), point_t(0.0, -0.892802, -0.450498), point_t(-0.416002, -0.40967, 0.811888), point_t(-0.90937, 0.1874, -0.371391), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(5.0, -20.0, 7.0), 0.0, 10.0);

    /* Ray trace the scene using bvh */
    fixture.render<bvh>();
    checker.check(fixture.get_camera(), "bvh");

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( off_rocker_arm_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/off_scenes/rocker-arm.off", model_format_t::off, point_t(0.951274, 0.687012, -0.226903), point_t(0.159414, 0.0909281, 0.983016), point_t(-0.49546, 0.868634, 0.0), point_t(-0.853883, -0.487045, 0.183524), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(5.0, 20.0, -7.0), 0.0, 10.0);

    /* Ray trace the scene using bvh */
    fixture.render<bvh>();
    checker.check(fixture.get_camera(), "bvh");

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( off_rs_creature_f_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/off_scenes/RSCREATURE_F.off", model_format_t::off, point_t(-7000.0, -1120.0, 2300.0), point_t(-0.128829, 0.991621, 0.00908751), point_t(0.338311, 0.0353345, 0.94037), point_t(0.932171, 0.124222, -0.34003), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-5000.0, -1000.0, 1800.0), 0.0, 10.0);

    /* Ray trace the scene using bvh */
    fixture.render<bvh>();
    checker.check(fixture.get_camera(), "bvh");

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( off_screwdriver_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/off_scenes/screwdriver.off", model_format_t::off, point_t(0.615127, 0.597009, 0.703515), point_t(-0.707103, 0.0, 0.707104), point_t(-0.397451, 0.827082, -0.397451), point_t(-0.584832, -0.562078, -0.58483), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(5.0, 20.0, -7.0), 0.0, 10.0);

    /* Ray trace the scene using bvh */
    fixture.render<bvh>();
    checker.check(fixture.get_camera(), "bvh");

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( off_screw_remeshed_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/off_scenes/screw-remeshed.off", model_format_t::off, point_t(0.0597028, 0.0654053, 0.0606354), point_t(-0.655468, -0.0436217, 0.753968), point_t(-0.480031, 0.794784, -0.371337), point_t(-0.583043, -0.605329, -0.541897), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(5.0, 20.0, -7.0), 0.0, 10.0);

    /* Ray trace the scene using bvh */
    fixture.render<bvh>();
    checker.check(fixture.get_camera(), "bvh");

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( off_shark_b_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/off_scenes/shark_b.off", model_format_t::off, point_t(88060.6, 26781.7, 116182.0), point_t(-0.844328, 0.0, 0.535827), point_t(-0.116887, 0.975917, -0.184185), point_t(-0.522923, -0.218143, -0.82399), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(80000.0, 20000.0, 110000.0), 0.0, 10.0);

    /* Ray trace the scene using bvh */
    fixture.render<bvh>();
    checker.check(fixture.get_camera(), "bvh");

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( off_sketched_brunnen_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/off_scenes/Sketched-Brunnen.off", model_format_t::off, point_t(6.25, -13.25, 12.0), point_t(0.140556, -0.736823, -0.66131), point_t(-0.631226, -0.581274, 0.513486), point_t(-0.762754, 0.345264, -0.546807), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(5.0, -20.0, 7.0), 0.0, 10.0);

    /* Ray trace the scene using bvh */
    fixture.render<bvh>();
    checker.check(fixture.get_camera(), "bvh");

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( off_skull_original_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/off_scenes/skull-original.off", model_format_t::off, point_t(225.0, 75.0, 125.0), point_t(0.307988, -0.949125, -0.065513), point_t(-0.617895, -0.251916, 0.744803), point_t(-0.723422, -0.188912, -0.664053), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(200.0, 75.0, 100.0), 0.0, 10.0);

    /* Ray trace the scene using bvh */
    fixture.render<bvh>();
    checker.check(fixture.get_camera(), "bvh");

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( off_sledge_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/off_scenes/sledge.off", model_format_t::off, point_t(8.0, 7.5, 10.0), point_t(-0.724795, -0.031878, 0.688226), point_t(-0.38767, 0.844655, -0.369145), point_t(-0.569546, -0.534359, -0.62456), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(5.0, 20.0, 7.0), 0.0, 10.0);

    /* Ray trace the scene using bvh */
    fixture.render<bvh>();
    checker.check(fixture.get_camera(), "bvh");

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( off_squirrel_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/off_scenes/squirrel.off", model_format_t::off, point_t(14.0, 4.0, 15.0), point_t(-0.713604, -0.184062, 0.675935), point_t(-0.262279, 0.964888, -0.0141498), point_t(-0.649597, -0.187381, -0.736824), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(4.0, 20.0, 8.0), 0.0, 10.0);

    /* Ray trace the scene using bvh */
    fixture.render<bvh>();
    checker.check(fixture.get_camera(), "bvh");

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( off_sword_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/off_scenes/sword.off", model_format_t::off, point_t(15.8429, 0.0, -1.24345), point_t(0.24869, 0.0, 0.968582), point_t(0.0, 1.0, 0.0), point_t(-0.968582, 0.0, 0.24869), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(5.0, 20.0, -7.0), 0.0, 10.0);

    /* Ray trace the scene using bvh */
    fixture.render<bvh>();
    checker.check(fixture.get_camera(), "bvh");

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( off_table_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/off_scenes/table.off", model_format_t::off, point_t(11.0, 5.0, 5.0), point_t(-0.425779, 0.0, 0.904827), point_t(-0.410783, 0.891006, -0.1933), point_t(-0.806206, -0.45399, -0.379372), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(5.0, 20.0, 7.0), 0.0, 10.0);

    /* Ray trace the scene using bvh */
    fixture.render<bvh>();
    checker.check(fixture.get_camera(), "bvh");

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( off_teapot_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/off_scenes/Teapot.off", model_format_t::off, point_t(2.19939, -2.88243, -0.664387), point_t(-0.591811, 0.0890611, -0.801137), point_t(-0.638906, -0.657813, 0.398841), point_t(-0.491481, 0.747894, 0.44620), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(5.0, -20.0, -7.0), 0.0, 10.0);

    /* Ray trace the scene using bvh */
    fixture.render<bvh>();
    checker.check(fixture.get_camera(), "bvh");

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( off_test_2_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/off_scenes/test2.off", model_format_t::off, point_t(220.0, 235.0, -120.0), point_t(0.637424, 0.0, 0.770513), point_t(-0.392223, 0.860742, 0.324475), point_t(-0.663212, -0.509041, 0.548657), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(200.0, 200.0, -100.0), 0.0, 10.0);

    /* Ray trace the scene using bvh */
    fixture.render<bvh>();
    checker.check(fixture.get_camera(), "bvh");

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( off_test_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/off_scenes/test.off", model_format_t::off, point_t(-1.2, -3.0, -2.0), point_t(-0.0828844, -0.656096, 0.750111), point_t(0.995441, -0.0901378, 0.031152), point_t(0.0471747, 0.749273, 0.660576), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-5.0, -20.0, -7.0), 0.0, 10.0);

    /* Ray trace the scene using bvh */
    fixture.render<bvh>();
    checker.check(fixture.get_camera(), "bvh");

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( off_tst_torus_model_2_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/off_scenes/tstTorusModel2.off", model_format_t::off, point_t(46.4853, 0.0, 175.0), point_t(-0.951056, 0.0, 0.309016), point_t(0.0, 1.0, 0.0), point_t(-0.309016, 0.0, -0.951056), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(50.0, 20.0, 700.0), 0.0, 10.0);

    /* Ray trace the scene using bvh */
    fixture.render<bvh>();
    checker.check(fixture.get_camera(), "bvh");

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( off_tst_torus_model_3_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/off_scenes/tstTorusModel3.off", model_format_t::off, point_t(100.0, 33.0, 100.0), point_t(-0.728968, 0.0, 0.684546), point_t(-0.149329, 0.975917, -0.15902), point_t(-0.66806, -0.218143, -0.711412), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(50.0, 200.0, 70.0), 0.0, 10.0);

    /* Ray trace the scene using bvh */
    fixture.render<bvh>();
    checker.check(fixture.get_camera(), "bvh");

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( off_tube_1_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/off_scenes/tube1.off", model_format_t::off, point_t(-75.0, -200.0, 0.0), point_t(0.0, 0.0, 1.0), point_t(0.951055, -0.309017, 0.0), point_t(0.309017, 0.951055, 0.0), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-50.0, -200.0, -70.0), 0.0, 10.0);

    /* Ray trace the scene using bvh */
    fixture.render<bvh>();
    checker.check(fixture.get_camera(), "bvh");

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( off_venus_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/off_scenes/venus.off", model_format_t::off, point_t(4.5, 3.2, 8.8), point_t(-0.891006, 0.0, 0.45399), point_t(-0.140291, 0.951056, -0.275336), point_t(-0.43177, -0.309016, -0.847397), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(5.0, 20.0, 7.0), 0.0, 10.0);

    /* Ray trace the scene using bvh */
    fixture.render<bvh>();
    checker.check(fixture.get_camera(), "bvh");

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( off_venus_original_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/off_scenes/venus-original.off", model_format_t::off, point_t(1.8, 1.3, 0.5), point_t(0.637422, -0.770132, -0.0242026), point_t(-0.176929, -0.176867, 0.9682), point_t(-0.749925, -0.612872, -0.248998), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(5.0, 20.0, 2.0), 0.0, 10.0);

    /* Ray trace the scene using bvh */
    fixture.render<bvh>();
    checker.check(fixture.get_camera(), "bvh");

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}
#endif /* #ifndef VALGRIND_TESTS */

BOOST_AUTO_TEST_SUITE_END()
}; /* namespace raptor_raytracer */
}; /* namespace test */
