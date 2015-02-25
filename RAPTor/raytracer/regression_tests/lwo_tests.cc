#ifdef STAND_ALONE
#define BOOST_TEST_MODULE lwo test

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
BOOST_AUTO_TEST_SUITE( lwo_tests );

/* LWO 1 tests */
#ifndef VALGRIND_TESTS
BOOST_AUTO_TEST_CASE( lwo_caterpillar_t530_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/lwo_scenes/vehicles/trucks/caterpillar/Caterpillar_T530.lwo", model_format_t::lwo, point_t(16.2813, 7.38379, -8.46192), point_t(0.427311, 0.0183088, 0.903919), point_t(-0.373232, 0.914198, 0.157922), point_t(-0.82347, -0.404853, 0.397481), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(100.0, 100.0, -100.0), 0.0, 10.0);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( lwo_freightliner_aerodyne_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/lwo_scenes/vehicles/trucks/freightliner/Freightliner_Aerodyne.lwo", model_format_t::lwo, point_t(-11.8461, 5.51688, -15.9438), point_t(0.727965, 0.0198696, -0.685326), point_t(0.223434, 0.938136,0.264534), point_t(0.648185, -0.345697, 0.67849), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-100, 100, -100), 0.0, 10.0);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( lwo_hummer_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/lwo_scenes/vehicles/cars/hummer/Hummer.lwo", model_format_t::lwo, point_t(10.7323, 2.44797, -13.1218), point_t(0.728969, 0.0, 0.684547), point_t(-0.107087, 0.987688, 0.114036), point_t(-0.676119, -0.156434, 0.719994), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(100.0, 100.0, -100.0), 0.0, 10.0);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(100.0, -100.0, 100.0), 0.0, 10.0);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( lwo_scud_launcher_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/lwo_scenes/vehicles/tanks/scud/scud_launcher.lwo", model_format_t::lwo, point_t(14.1448, 9.99139, -22.98), point_t(0.750989, -0.00278201, 0.66031), point_t(-0.297723, 0.89115, 0.342362), point_t(-0.589388, -0.453698, 0.668415) , ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(100.0, 100.0, -100.0), 0.0, 10.0);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(100.0, -100.0, 100.0), 0.0, 10.0);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( lwo_subaru_impreza_0_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/lwo_scenes/vehicles/race_cars/subaru/Subaru_Impreza.lwo", model_format_t::lwo, point_t(9.9388, 8.90993, -13.7376), point_t(0.728969, 0.0, 0.684547), point_t(-0.384773, 0.82708, 0.409741), point_t(-0.566176, -0.562083, 0.602916), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(100.0,  100.0, -100.0), 0.0, 10.0);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(100.0, -100.0,  100.0), 0.0, 10.0);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( lwo_subaru_impreza_1_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/lwo_scenes/vehicles/race_cars/subaru/Subaru_Impreza.lwo", model_format_t::lwo, point_t(-8.18096, -0.839894, 14.0719), point_t(-0.808254, -0.0146934, -0.588651), point_t(-0.0112015, 0.999891, -0.00957785), point_t(0.588727, -0.00114707, -0.808331), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(-100.0,  100.0, 100.0), 0.0, 10.0);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t( 100.0, -100.0, 100.0), 0.0, 10.0);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( lwo_t_62_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/lwo_scenes/vehicles/tanks/t62/t_62.lwo", model_format_t::lwo, point_t(12.9077, 8.18881, -19.4989), point_t(0.750648, -0.00087924, 0.660703), point_t(-0.22302, 0.940969, 0.254633), point_t(-0.621925, -0.338488, 0.706141), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(100.0,  100.0, -100.0), 0.0, 10.0);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(100.0, -100.0,  100.0), 0.0, 10.0);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( lwo_t_62_spotlit_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/lwo_scenes/vehicles/tanks/t62/t_62.lwo", model_format_t::lwo, point_t(12.9077, 8.18881, -19.4989), point_t(0.750648, -0.00087924, 0.660703), point_t(-0.22302, 0.940969, 0.254633), point_t(-0.621925, -0.338488, 0.706141), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_spotlight(ext_colour_t(255.0, 255.0, 255.0), point_t(0.0, 15.0, 0.0), point_t(0.0, 0.0, 0.0), 10.0, 0.0, 10.0, 45.0);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( lwo_t_62_directlit_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/lwo_scenes/vehicles/tanks/t62/t_62.lwo", model_format_t::lwo, point_t(12.9077, 8.18881, -19.4989), point_t(0.750648, -0.00087924, 0.660703), point_t(-0.22302, 0.940969, 0.254633), point_t(-0.621925, -0.338488, 0.706141), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_directional_light(ext_colour_t(255.0, 255.0, 255.0), point_t(15.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.0);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( lwo_trailer_chemical_tanker_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/lwo_scenes/vehicles/trucks/trailer_tanker/trailer_chemical_tanker.lwo", model_format_t::lwo, point_t(15.1995, 5.0, -22.2613), point_t(0.728969, 0.0, 0.684547), point_t(-0.107087, 0.987688, 0.114036), point_t(-0.676119, -0.156434, 0.719994), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(100.0,  100.0, -100.0), 0.0, 10.0);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(100.0, -100.0,  100.0), 0.0, 10.0);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( lwo_trailer_container_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/lwo_scenes/vehicles/trucks/trailer_container/Trailer_container.lwo", model_format_t::lwo, point_t(15.1995, 5.0, -22.2613), point_t(0.728969, 0.0, 0.684547), point_t(-0.107087, 0.987688, 0.114036), point_t(-0.676119, -0.156434, 0.719994), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(100.0,  100.0, -100.0), 0.0, 10.0);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(100.0, -100.0,  100.0), 0.0, 10.0);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( lwo_diablo_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/lwo_scenes/vehicles/cars/diablo/Lamborghini_Diablo.lwo", model_format_t::lwo, point_t(15.1995, 5.0, -22.2613), point_t(0.728969, 0.0, 0.684547), point_t(-0.107087, 0.987688, 0.114036), point_t(-0.676119, -0.156434, 0.719994), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(100.0,  100.0, -100.0), 0.0, 10.0);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(100.0, -100.0,  100.0), 0.0, 10.0);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( lwo_thunderbird_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/lwo_scenes/vehicles/cars/thunderbird/Ford_Thunderbird_1957.lwo", model_format_t::lwo, point_t(15.1995, 5.0, -22.2613), point_t(0.728969, 0.0, 0.684547), point_t(-0.107087, 0.987688, 0.114036), point_t(-0.676119, -0.156434, 0.719994), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(100.0,  100.0, -100.0), 0.0, 10.0);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(100.0, -100.0,  100.0), 0.0, 10.0);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( lwo_viper_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/lwo_scenes/vehicles/cars/viper/Chrysler_Viper.lwo", model_format_t::lwo, point_t(15.1995, 5.0, -22.2613), point_t(0.728969, 0.0, 0.684547), point_t(-0.107087, 0.987688, 0.114036), point_t(-0.676119, -0.156434, 0.719994), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(100.0,  100.0, -100.0), 0.0, 10.0);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(100.0, -100.0,  100.0), 0.0, 10.0);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( lwo_dragster_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/lwo_scenes/vehicles/race_cars/dragster/Dragster.lwo", model_format_t::lwo, point_t(15.1995, 5.0, -22.2613), point_t(0.728969, 0.0, 0.684547), point_t(-0.107087, 0.987688, 0.114036), point_t(-0.676119, -0.156434, 0.719994), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(100.0,  100.0, -100.0), 0.0, 10.0);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(100.0, -100.0,  100.0), 0.0, 10.0);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( lwo_panzer2_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/lwo_scenes/vehicles/tanks/panzer2/Panzer_II_AUSF.lwo", model_format_t::lwo, point_t(15.1995, 5.0, -22.2613), point_t(0.728969, 0.0, 0.684547), point_t(-0.107087, 0.987688, 0.114036), point_t(-0.676119, -0.156434, 0.719994), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(100.0,  100.0, -100.0), 0.0, 10.0);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(100.0, -100.0,  100.0), 0.0, 10.0);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( lwo_mack_dumper_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/lwo_scenes/vehicles/trucks/mack_dumper/Mack_dumper.lwo", model_format_t::lwo, point_t(15.1995, 5.0, -22.2613), point_t(0.728969, 0.0, 0.684547), point_t(-0.107087, 0.987688, 0.114036), point_t(-0.676119, -0.156434, 0.719994), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(100.0,  100.0, -100.0), 0.0, 10.0);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(100.0, -100.0,  100.0), 0.0, 10.0);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( lwo_mack_superliner_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/lwo_scenes/vehicles/trucks/mack_superliner/Mack_Superliner.lwo", model_format_t::lwo, point_t(15.1995, 5.0, -22.2613), point_t(0.728969, 0.0, 0.684547), point_t(-0.107087, 0.987688, 0.114036), point_t(-0.676119, -0.156434, 0.719994), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(100.0,  100.0, -100.0), 0.0, 10.0);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(100.0, -100.0,  100.0), 0.0, 10.0);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( lwo_trailer_lowboy_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/lwo_scenes/vehicles/trucks/trailer_lowboy/Trailer_lowboy.lwo", model_format_t::lwo, point_t(15.1995, 5.0, -22.2613), point_t(0.728969, 0.0, 0.684547), point_t(-0.107087, 0.987688, 0.114036), point_t(-0.676119, -0.156434, 0.719994), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(100.0,  100.0, -100.0), 0.0, 10.0);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(100.0, -100.0,  100.0), 0.0, 10.0);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}
#endif /* #ifndef VALGRIND_TESTS */

BOOST_AUTO_TEST_CASE( lwo_caterpillar_t530_low_res_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/lwo_scenes/vehicles/trucks/caterpillar/Caterpillar_T530.lwo", model_format_t::lwo, point_t(16.2813, 7.38379, -8.46192), point_t(0.427311, 0.0183088, 0.903919), point_t(-0.373232, 0.914198, 0.157922), point_t(-0.82347, -0.404853, 0.397481), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 640, 480);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(100.0, 100.0, -100.0), 0.0, 10.0);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

/* LWO 2 tests */
// #ifndef VALGRIND_TESTS
// BOOST_AUTO_TEST_CASE( lwo_mercedes_clk_dtm_test )
// {
//     /* Checker */
//     CREATE_REGRESSION_CHECKER(checker);

//     /* Enviroment set up */
//     regression_fixture fixture("/lwo_scenes/vehicles/tanks/panzer2/Panzer_II_AUSF.lwo", model_format_t::lwo, point_t(15.1995, 5.0, -22.2613), point_t(0.728969, 0.0, 0.684547), point_t(-0.107087, 0.987688, 0.114036), point_t(-0.676119, -0.156434, 0.719994), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(100.0,  100.0, -100.0), 0.0, 10.0);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(100.0, -100.0,  100.0), 0.0, 10.0);

//     /* Ray trace the scene using kd tree */
//     fixture.render<kd_tree>();
//     checker.check(fixture.get_camera(), "kdt");

//     /* Ray trace the scene using bih */
//     fixture.render<bih>();
//     checker.check(fixture.get_camera(), "bih");
// }

// BOOST_AUTO_TEST_CASE( lwo_mack_dumper_test )
// {
//     /* Checker */
//     CREATE_REGRESSION_CHECKER(checker);

//     /* Enviroment set up */
//     regression_fixture fixture("/lwo_scenes/vehicles/tanks/abrams/M1_Abrams.lwo", model_format_t::lwo, point_t(15.1995, 5.0, -22.2613), point_t(0.728969, 0.0, 0.684547), point_t(-0.107087, 0.987688, 0.114036), point_t(-0.676119, -0.156434, 0.719994), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(200.0,  100.0, -100.0), 0.0, 10.0);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(100.0, -100.0,  100.0), 0.0, 10.0);

//     /* Ray trace the scene using kd tree */
//     fixture.render<kd_tree>();
//     checker.check(fixture.get_camera(), "kdt");

//     /* Ray trace the scene using bih */
//     fixture.render<bih>();
//     checker.check(fixture.get_camera(), "bih");
// }

// BOOST_AUTO_TEST_CASE( lwo_weisel_2_ozelot_test )
// {
//     /* Checker */
//     CREATE_REGRESSION_CHECKER(checker);

//     /* Enviroment set up */
//     regression_fixture fixture("/lwo_scenes/vehicles/tanks/weisel_2_ozelot/ozelot.lwo", model_format_t::lwo, point_t(350.0, 175.0, -350.0), point_t(0.728969, 0.0, 0.684547), point_t(-0.107087, 0.987688, 0.114036), point_t(-0.676119, -0.156434, 0.719994), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(500.0,  500.0, -500.0), 0.0, 10.0);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(500.0, -500.0,  500.0), 0.0, 10.0);

//     /* Ray trace the scene using kd tree */
//     fixture.render<kd_tree>();
//     checker.check(fixture.get_camera(), "kdt");

//     /* Ray trace the scene using bih */
//     fixture.render<bih>();
//     checker.check(fixture.get_camera(), "bih");
// }

// BOOST_AUTO_TEST_CASE( lwo_peterbilt_379_4axle_test )
// {
//     /* Checker */
//     CREATE_REGRESSION_CHECKER(checker);

//     /* Enviroment set up */
//     regression_fixture fixture("/lwo_scenes/vehicles/trucks/peterbilt_379_4axle/Peterbilt_379_4_axle.lwo", model_format_t::lwo, point_t(15.1995, 5.0, -22.2613), point_t(0.728969, 0.0, 0.684547), point_t(-0.107087, 0.987688, 0.114036), point_t(-0.676119, -0.156434, 0.719994), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(100.0,  100.0, -100.0), 0.0, 10.0);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(100.0, -100.0,  100.0), 0.0, 10.0);

//     /* Ray trace the scene using kd tree */
//     fixture.render<kd_tree>();
//     checker.check(fixture.get_camera(), "kdt");

//     /* Ray trace the scene using bih */
//     fixture.render<bih>();
//     checker.check(fixture.get_camera(), "bih");
// }

// BOOST_AUTO_TEST_CASE( lwo_trailer_doepker_logger_test )
// {
//     /* Checker */
//     CREATE_REGRESSION_CHECKER(checker);

//     /* Enviroment set up */
//     regression_fixture fixture("/lwo_scenes/vehicles/trucks/trailer_lowboy/Trailer_lowboy.lwo", model_format_t::lwo, point_t(15.1995, 5.0, -22.2613), point_t(0.728969, 0.0, 0.684547), point_t(-0.107087, 0.987688, 0.114036), point_t(-0.676119, -0.156434, 0.719994), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(100.0,  100.0, -100.0), 0.0, 10.0);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(100.0, -100.0,  100.0), 0.0, 10.0);

//     /* Ray trace the scene using kd tree */
//     fixture.render<kd_tree>();
//     checker.check(fixture.get_camera(), "kdt");

//     /* Ray trace the scene using bih */
//     fixture.render<bih>();
//     checker.check(fixture.get_camera(), "bih");
// }
// #endif /* #ifndef VALGRIND_TESTS */

// BOOST_AUTO_TEST_CASE( lwo_caterpillar_t530_low_res_test )
// {
//     /* Checker */
//     CREATE_REGRESSION_CHECKER(checker);

//     /* Enviroment set up */
//     regression_fixture fixture("/lwo_scenes/vehicles/trucks/caterpillar/Caterpillar_T530.lwo", model_format_t::lwo, point_t(16.2813, 7.38379, -8.46192), point_t(0.427311, 0.0183088, 0.903919), point_t(-0.373232, 0.914198, 0.157922), point_t(-0.82347, -0.404853, 0.397481), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 640, 480);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(100.0, 100.0, -100.0), 0.0, 10.0);

//     /* Ray trace the scene using kd tree */
//     fixture.render<kd_tree>();
//     checker.check(fixture.get_camera(), "kdt");

//     /* Ray trace the scene using bih */
//     fixture.render<bih>();
//     checker.check(fixture.get_camera(), "bih");
// }

/* LWO 2 tests */
#ifndef VALGRIND_TESTS
BOOST_AUTO_TEST_CASE( lwo_mercedes_clk_dtm_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/lwo_scenes/vehicles/race_cars/mercedes_clk_dtm/Mercedes_CLK_DTM.lwo", model_format_t::lwo, point_t(15.1995, 5.0, -22.2613), point_t(0.728969, 0.0, 0.684547), point_t(-0.107087, 0.987688, 0.114036), point_t(-0.676119, -0.156434, 0.719994), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(100.0,  100.0, -100.0), 0.0, 10.0);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(100.0, -100.0,  100.0), 0.0, 10.0);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( lwo_abrams_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/lwo_scenes/vehicles/tanks/abrams/M1_Abrams.lwo", model_format_t::lwo, point_t(15.1995, 5.0, -22.2613), point_t(0.728969, 0.0, 0.684547), point_t(-0.107087, 0.987688, 0.114036), point_t(-0.676119, -0.156434, 0.719994), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(200.0,  100.0, -100.0), 0.0, 10.0);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(100.0, -100.0,  100.0), 0.0, 10.0);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( lwo_weisel_2_ozelot_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/lwo_scenes/vehicles/tanks/weisel_2_ozelot/ozelot.lwo", model_format_t::lwo, point_t(350.0, 175.0, -350.0), point_t(0.728969, 0.0, 0.684547), point_t(-0.107087, 0.987688, 0.114036), point_t(-0.676119, -0.156434, 0.719994), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(500.0,  500.0, -500.0), 0.0, 10.0);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(500.0, -500.0,  500.0), 0.0, 10.0);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( lwo_peterbilt_379_4axle_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/lwo_scenes/vehicles/trucks/peterbilt_379_4axle/Peterbilt_379_4_axle.lwo", model_format_t::lwo, point_t(15.1995, 5.0, -22.2613), point_t(0.728969, 0.0, 0.684547), point_t(-0.107087, 0.987688, 0.114036), point_t(-0.676119, -0.156434, 0.719994), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(100.0,  100.0, -100.0), 0.0, 10.0);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(100.0, -100.0,  100.0), 0.0, 10.0);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

// BOOST_AUTO_TEST_CASE( lwo_trailer_3_axle_mechanical_detachable_gooseneck_jeep_test )
// {
//     /* Checker */
//     CREATE_REGRESSION_CHECKER(checker);

//     /* Enviroment set up */
//     regression_fixture fixture("/lwo_scenes/vehicles/trucks/trailers_trailking/3_axle_mechanical_detachable_gooseneck_jeep.lwo", model_format_t::lwo, point_t(17.0, 4.0, -38.0), point_t(0.728969, 0.0, 0.684547), point_t(-0.107087, 0.987688, 0.114036), point_t(-0.676119, -0.156434, 0.719994), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(100.0,  100.0, -100.0), 0.0, 10.0);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(100.0, -100.0,  100.0), 0.0, 10.0);

//     /* Ray trace the scene using kd tree */
//     fixture.render<kd_tree>();
//     checker.check(fixture.get_camera(), "kdt");

//     /* Ray trace the scene using bih */
//     fixture.render<bih>();
//     checker.check(fixture.get_camera(), "bih");
// }

BOOST_AUTO_TEST_CASE( lwo_trailer_2_axle_mechanical_detachable_gooseneck_jeep_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/lwo_scenes/vehicles/trucks/trailers_trailking/2_axle_Mechanical_Detachable_Gooseneck_Jeep.lwo", model_format_t::lwo, point_t(17.0, 4.0, -38.0), point_t(0.728969, 0.0, 0.684547), point_t(-0.107087, 0.987688, 0.114036), point_t(-0.676119, -0.156434, 0.719994), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(100.0,  100.0, -100.0), 0.0, 10.0);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(100.0, -100.0,  100.0), 0.0, 10.0);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( lwo_trailer_3_axle_jeep_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/lwo_scenes/vehicles/trucks/trailers_trailking/3_axle_jeep.lwo", model_format_t::lwo, point_t(17.0, 4.0, -38.0), point_t(0.728969, 0.0, 0.684547), point_t(-0.107087, 0.987688, 0.114036), point_t(-0.676119, -0.156434, 0.719994), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(100.0,  100.0, -100.0), 0.0, 10.0);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(100.0, -100.0,  100.0), 0.0, 10.0);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

BOOST_AUTO_TEST_CASE( lwo_trailer_3_axle_lowboy_trailer_with_short_goosneck_test )
{
    /* Checker */
    CREATE_REGRESSION_CHECKER(checker);

    /* Enviroment set up */
    regression_fixture fixture("/lwo_scenes/vehicles/trucks/trailers_trailking/3_axle_lowboy_trailer_with_short_goosneck.lwo", model_format_t::lwo, point_t(17.0, 4.0, -38.0), point_t(0.728969, 0.0, 0.684547), point_t(-0.107087, 0.987688, 0.114036), point_t(-0.676119, -0.156434, 0.719994), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(100.0,  100.0, -100.0), 0.0, 10.0);
    fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(100.0, -100.0,  100.0), 0.0, 10.0);

    /* Ray trace the scene using kd tree */
    fixture.render<kd_tree>();
    checker.check(fixture.get_camera(), "kdt");

    /* Ray trace the scene using bih */
    fixture.render<bih>();
    checker.check(fixture.get_camera(), "bih");
}

// // BOOST_AUTO_TEST_CASE( lwo_jcb_135b_test )
// // {
// //     /* Checker */
// //     CREATE_REGRESSION_CHECKER(checker);

// //     /* Enviroment set up */
// //     regression_fixture fixture("/lwo_scenes/vehicles/trucks/jcb_135b/JCB_435_B.lwo", model_format_t::lwo, point_t(15.1995, 5.0, -22.2613), point_t(0.728969, 0.0, 0.684547), point_t(-0.107087, 0.987688, 0.114036), point_t(-0.676119, -0.156434, 0.719994), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1920, 1080);
// //     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(100.0,  100.0, -100.0), 0.0, 10.0);
// //     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(100.0, -100.0,  100.0), 0.0, 10.0);

// //     /* Ray trace the scene using kd tree */
// //     fixture.render<kd_tree>();
// //     checker.check(fixture.get_camera(), "kdt");

// //     /* Ray trace the scene using bih */
// //     fixture.render<bih>();
// //     checker.check(fixture.get_camera(), "bih");
// // }
#endif /* #ifndef VALGRIND_TESTS */

// BOOST_AUTO_TEST_CASE( lwo_trailer_3_axle_mechanical_detachable_gooseneck_jeep_low_res_test )

// {
//     /* Checker */
//     CREATE_REGRESSION_CHECKER(checker);

//     /* Enviroment set up */
//     regression_fixture fixture("/lwo_scenes/vehicles/trucks/trailers_trailking/3_axle_mechanical_detachable_gooseneck_jeep.lwo", model_format_t::lwo, point_t(17.0, 4.0, -38.0), point_t(0.728969, 0.0, 0.684547), point_t(-0.107087, 0.987688, 0.114036), point_t(-0.676119, -0.156434, 0.719994), ext_colour_t(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 640, 480);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(100.0,  100.0, -100.0), 0.0, 10.0);
//     fixture.add_light(ext_colour_t(255.0, 255.0, 255.0), point_t(100.0, -100.0,  100.0), 0.0, 10.0);

//     /* Ray trace the scene using kd tree */
//     fixture.render<kd_tree>();
//     checker.check(fixture.get_camera(), "kdt");

//     /* Ray trace the scene using bih */
//     fixture.render<bih>();
//     checker.check(fixture.get_camera(), "bih");
// }
BOOST_AUTO_TEST_SUITE_END()
}; /* namespace raptor_raytracer */
}; /* namespace test */
