#ifdef STAND_ALONE
#define BOOST_TEST_MODULE physics_options test

/* Common headers */
#include "logging.h"

/* Initialise logger */
const raptor_physics::init_logger init_logger;
#endif /* #ifdef STAND_ALONE */

/* Standard headers */
#include <string>

/* Boost headers */
#include "boost/test/unit_test.hpp"

/* Common headers */
#include "physics_options.h"


namespace raptor_physics
{
namespace test
{
BOOST_AUTO_TEST_SUITE( physics_options_tests )

const float result_tolerance = 0.0001;

/* CTOR tests */
BOOST_AUTO_TEST_CASE( default_ctor_test )
{
    physics_options po;
    BOOST_CHECK(po.max_timestep()  == -1.0);
    BOOST_CHECK(po.min_timestep()  ==  0.0);
    BOOST_CHECK(po.frames_to_run() == -1);
    BOOST_CHECK(po.pause_on_last() == false);
    BOOST_CHECK(po.render()        == true);
}

BOOST_AUTO_TEST_CASE( non_default_ctor_test )
{
    physics_options po(0.5, 0.25, 17, true, false);
    BOOST_CHECK(po.max_timestep()  == 0.5);
    BOOST_CHECK(po.min_timestep()  == 0.25);
    BOOST_CHECK(po.frames_to_run() == 17);
    BOOST_CHECK(po.pause_on_last() == true);
    BOOST_CHECK(po.render()        == false);
}

BOOST_AUTO_TEST_CASE( args_ctor_test )
{
    std::string arg0("physics_engine");
    std::string arg1("-max_fps");
    std::string arg2("2.0");
    std::string arg3("--min_fps");
    std::string arg4("4.0");
    std::string arg5("--frames_to_run");
    std::string arg6("17");
    std::string arg7("-pause_on_last");
    std::string arg8("--dont_render");
    const char *argv[] = { arg0.c_str(), arg1.c_str(), arg2.c_str(), arg3.c_str(), arg4.c_str(), arg5.c_str(), arg6.c_str(), arg7.c_str(), arg8.c_str() };

    physics_options po(argv, 9);
    BOOST_CHECK(po.max_timestep()  == 0.5);
    BOOST_CHECK(po.min_timestep()  == 0.25);
    BOOST_CHECK(po.frames_to_run() == 17);
    BOOST_CHECK(po.pause_on_last() == true);
    BOOST_CHECK(po.render()        == false);
}

BOOST_AUTO_TEST_CASE( args_ctor_max_fps_error_test )
{
    std::string arg0("physics_engine");
    std::string arg1("-max_fps");
    const char *argv[] = { arg0.c_str(), arg1.c_str() };

    physics_options po(argv, 2);
    BOOST_CHECK(po.max_timestep()  == -1.0);
    BOOST_CHECK(po.min_timestep()  ==  0.0);
    BOOST_CHECK(po.frames_to_run() == -1);
    BOOST_CHECK(po.pause_on_last() == false);
    BOOST_CHECK(po.render()        == true);
    BOOST_CHECK(po.error() == true);
}

BOOST_AUTO_TEST_CASE( args_ctor_min_fps_error_test )
{
    std::string arg0("physics_engine");
    std::string arg1("-max_fps");
    std::string arg2("2.0");
    std::string arg3("--min_fps");
    const char *argv[] = { arg0.c_str(), arg1.c_str(), arg2.c_str(), arg3.c_str() };

    physics_options po(argv, 4);
    BOOST_CHECK(po.max_timestep()  ==  0.5);
    BOOST_CHECK(po.min_timestep()  ==  0.0);
    BOOST_CHECK(po.frames_to_run() == -1);
    BOOST_CHECK(po.pause_on_last() == false);
    BOOST_CHECK(po.render()        == true);
    BOOST_CHECK(po.error() == true);
}

BOOST_AUTO_TEST_CASE( args_ctor_frames_to_run_error_test )
{
    std::string arg0("physics_engine");
    std::string arg1("-max_fps");
    std::string arg2("2.0");
    std::string arg3("--min_fps");
    std::string arg4("4.0");
    std::string arg5("--frames_to_run");
    const char *argv[] = { arg0.c_str(), arg1.c_str(), arg2.c_str(), arg3.c_str(), arg4.c_str(), arg5.c_str() };

    physics_options po(argv, 6);
    BOOST_CHECK(po.max_timestep()  == 0.5);
    BOOST_CHECK(po.min_timestep()  == 0.25);
    BOOST_CHECK(po.frames_to_run() == -1);
    BOOST_CHECK(po.pause_on_last() == false);
    BOOST_CHECK(po.render()        == true);
    BOOST_CHECK(po.error() == true);
}

BOOST_AUTO_TEST_CASE( args_ctor_unknown_option_error_test )
{
    std::string arg0("physics_engine");
    std::string arg1("-max_fps");
    std::string arg2("2.0");
    std::string arg3("--min_fps");
    std::string arg4("4.0");
    std::string arg5("--unknown_option");
    const char *argv[] = { arg0.c_str(), arg1.c_str(), arg2.c_str(), arg3.c_str(), arg4.c_str(), arg5.c_str() };

    physics_options po(argv, 6);
    BOOST_CHECK(po.max_timestep()  == 0.5);
    BOOST_CHECK(po.min_timestep()  == 0.25);
    BOOST_CHECK(po.frames_to_run() == -1);
    BOOST_CHECK(po.pause_on_last() == false);
    BOOST_CHECK(po.render()        == true);
    BOOST_CHECK(po.error() == true);
}


/* Check max timestep */
BOOST_AUTO_TEST_CASE( max_timestep_test )
{
    physics_options po;
    BOOST_CHECK(po.max_timestep()  == -1.0);
    BOOST_CHECK(po.min_timestep()  ==  0.0);
    BOOST_CHECK(po.frames_to_run() == -1);
    BOOST_CHECK(po.pause_on_last() == false);
    BOOST_CHECK(po.render()        == true);

    po.max_timestep(1.5);
    BOOST_CHECK(po.max_timestep()  ==  1.5);
    BOOST_CHECK(po.min_timestep()  ==  0.0);
    BOOST_CHECK(po.frames_to_run() == -1);
    BOOST_CHECK(po.pause_on_last() == false);
    BOOST_CHECK(po.render()        == true);
}


/* Check min timestep */
BOOST_AUTO_TEST_CASE( min_timestep_test )
{
    physics_options po;
    BOOST_CHECK(po.max_timestep()  == -1.0);
    BOOST_CHECK(po.min_timestep()  ==  0.0);
    BOOST_CHECK(po.frames_to_run() == -1);
    BOOST_CHECK(po.pause_on_last() == false);
    BOOST_CHECK(po.render()        == true);

    po.min_timestep(7.25);
    BOOST_CHECK(po.max_timestep()  == -1.0);
    BOOST_CHECK(po.min_timestep()  ==  7.25);
    BOOST_CHECK(po.frames_to_run() == -1);
    BOOST_CHECK(po.pause_on_last() == false);
    BOOST_CHECK(po.render()        == true);
}


/* Check frames to run */
BOOST_AUTO_TEST_CASE( frames_to_run_test )
{
    physics_options po;
    BOOST_CHECK(po.max_timestep()  == -1.0);
    BOOST_CHECK(po.min_timestep()  ==  0.0);
    BOOST_CHECK(po.frames_to_run() == -1);
    BOOST_CHECK(po.pause_on_last() == false);
    BOOST_CHECK(po.render()        == true);

    po.frames_to_run(5);
    BOOST_CHECK(po.max_timestep()  == -1.0);
    BOOST_CHECK(po.min_timestep()  ==  0.0);
    BOOST_CHECK(po.frames_to_run() ==  5);
    BOOST_CHECK(po.pause_on_last() == false);
    BOOST_CHECK(po.render()        == true);
}


/* Check frame done */
BOOST_AUTO_TEST_CASE( frame_done_test )
{
    physics_options po(0.5, 0.25, 3, true, false);
    BOOST_CHECK(po.max_timestep()  == 0.5);
    BOOST_CHECK(po.min_timestep()  == 0.25);
    BOOST_CHECK(po.frames_to_run() == 3);
    BOOST_CHECK(po.pause_on_last() == true);
    BOOST_CHECK(po.render()        == false);

    po.frame_done();
    BOOST_CHECK(po.frames_to_run() == 2);

    po.frame_done();
    BOOST_CHECK(po.frames_to_run() == 1);

    po.frame_done();
    BOOST_CHECK(po.frames_to_run() == 0);

    po.frame_done();
    BOOST_CHECK(po.frames_to_run() == 0);

    po.frame_done();
    BOOST_CHECK(po.frames_to_run() == 0);

    po.frame_done();
    BOOST_CHECK(po.frames_to_run() == 0);
}

BOOST_AUTO_TEST_SUITE_END()
}; /* namespace test */
}; /* namespace raptor_physics */
