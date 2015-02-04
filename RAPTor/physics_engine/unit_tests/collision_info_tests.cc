#ifdef STAND_ALONE
#define BOOST_TEST_MODULE collision_info test

/* Common headers */
#include "logging.h"

/* Initialise logger */
const raptor_physics::init_logger init_logger;
#endif /* #ifdef STAND_ALONE */


/* Boost headers */
#include "boost/test/unit_test.hpp"

/* Physics headers */
#include "simplex.h"
#include "vertex_group.h"
#include "physics_object.h"
#include "collision_info.h"

/* Test headers */
#include "mock_physics_object.h"


namespace raptor_physics
{
namespace test
{
/* Test data */
struct collision_info_fixture
{
    collision_info_fixture()
    : data0(physics_object_for_simplex_testing(std::vector<point_t>(
        {
            point_t( 1.5,  1.5, 1.0),
            point_t( 1.5, -0.5, 1.0),
            point_t(-0.5, -0.5, 1.0),
            point_t(-0.5,  1.5, 1.0)
        }))),
    data1(physics_object_for_simplex_testing(std::vector<point_t>(
        {
            point_t( 1.5,  1.6,  1.0),
            point_t(-1.5,  0.5,  5.0),
            point_t(-3.5, -0.8,  1.0),
            point_t(-0.5,  1.5, -2.0)
        })))
    {  };

    std::unique_ptr<physics_object> data0;
    std::unique_ptr<physics_object> data1;
};


BOOST_FIXTURE_TEST_SUITE( collision_info_tests, collision_info_fixture )

const float result_tolerance = 0.0005;


/* Test Construction */
BOOST_AUTO_TEST_CASE( ctor_test )
{
    /* New simplices, the collision infos will delete these */
    simplex *s0 = new simplex(*data0);
    simplex *s1 = new simplex(*data1);
    simplex *s2 = new simplex(*data1);

    /* Test certain collision */
    collision_info uut0(s0, *s1, 0.1, COLLISION);
    BOOST_CHECK(uut0.get_simplex().get() == s0);
    BOOST_CHECK(uut0.get_normal_of_collision() == point_t(0.0, -1.0, 0.0));
    BOOST_CHECK(uut0.get_point_of_collision() == point_t(1.5, 1.55, 1.0));
    BOOST_CHECK(fabs(uut0.get_time() - 0.1) < result_tolerance);
    BOOST_CHECK(uut0.get_type() == COLLISION);
    
    /* Test uncertain collision */
    collision_info uut1(s1, *s0, 1.5, POSSIBLE_COLLISION);
    BOOST_CHECK(uut1.get_simplex().get() == s1);
    BOOST_CHECK(uut1.get_normal_of_collision() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(uut1.get_point_of_collision() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(uut1.get_time() == 1.5);
    BOOST_CHECK(uut1.get_type() == POSSIBLE_COLLISION);
    
    /* Test certain collision, but with flipped simplices */
    collision_info uut2(s2, *s0, 0.5, COLLISION);
    BOOST_CHECK(uut2.get_simplex().get() == s2);
    BOOST_CHECK(uut2.get_normal_of_collision() == point_t(0.0, 1.0, 0.0));
    BOOST_CHECK(uut2.get_point_of_collision() == point_t(1.5, 1.55, 1.0));
    BOOST_CHECK(uut2.get_time() == 0.5);
    BOOST_CHECK(uut2.get_type() == COLLISION);
}


/* Voiding a collision */
BOOST_AUTO_TEST_CASE( void_collsion_test )
{
    /* New simplices, the collision infos will delete these */
    simplex *s0 = new simplex(*data0);
    std::unique_ptr<simplex> s1(new simplex(*data1));

    /* Construct and test */
    collision_info uut(s0, *s1, 0.1, COLLISION);
    BOOST_CHECK(uut.get_simplex().get() == s0);
    BOOST_CHECK(uut.get_normal_of_collision() == point_t(0.0, -1.0, 0.0));
    BOOST_CHECK(uut.get_point_of_collision() == point_t(1.5, 1.55, 1.0));
    BOOST_CHECK(fabs(uut.get_time() - 0.1) < result_tolerance);
    BOOST_CHECK(uut.get_type() == COLLISION);

    /* Void and test */
    uut.void_collision();
    BOOST_CHECK(uut.get_simplex().get() == s0);
    BOOST_CHECK(uut.get_normal_of_collision() == point_t(0.0, -1.0, 0.0));
    BOOST_CHECK(uut.get_point_of_collision() == point_t(1.5, 1.55, 1.0));
    BOOST_CHECK(uut.get_time() == std::numeric_limits<fp_t>::max());
    BOOST_CHECK(uut.get_type() == NO_COLLISION);
}


/* Successfully retest a collision */
BOOST_AUTO_TEST_CASE( successful_retest_update_test )
{
    /* New simplices, the collision infos will delete these */
    std::unique_ptr<simplex> s0(new simplex(*data0));
    simplex *s1 = new simplex(*data1);
    simplex *s2 = new simplex(*data1);
    
    /* Construct and test */
    collision_info uut(s1, *s0, 1.5, POSSIBLE_COLLISION);
    BOOST_CHECK(uut.get_simplex().get() == s1);
    BOOST_CHECK(uut.get_normal_of_collision() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(uut.get_point_of_collision() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(uut.get_time() == 1.5);
    BOOST_CHECK(uut.get_type() == POSSIBLE_COLLISION);

    /* Update and test */
    uut.successful_retest_update(s2, *s0);
    BOOST_CHECK(uut.get_simplex().get() == s2);
    BOOST_CHECK(uut.get_normal_of_collision() == point_t(0.0, 1.0, 0.0));
    BOOST_CHECK(uut.get_point_of_collision() == point_t(1.5, 1.55, 1.0));
    BOOST_CHECK(uut.get_time() == 1.5);
    BOOST_CHECK(uut.get_type() == COLLISION);
}


/* Update a collision */
BOOST_AUTO_TEST_CASE( update_test )
{
    /* New simplices, the collision infos will delete these */
    simplex *s0 = new simplex(*data0);
    simplex *s1 = new simplex(*data1);
    simplex *s2 = new simplex(*data1);
    simplex *s3 = new simplex(*data0);

    /* Construct certain collision and test */
    collision_info uut0(s0, *s1, 0.1, COLLISION);
    BOOST_CHECK(uut0.get_simplex().get() == s0);
    BOOST_CHECK(uut0.get_normal_of_collision() == point_t(0.0, -1.0, 0.0));
    BOOST_CHECK(uut0.get_point_of_collision()  == point_t(1.5, 1.55, 1.0));
    BOOST_CHECK(fabs(uut0.get_time() - 0.1) < result_tolerance);
    BOOST_CHECK(uut0.get_type() == COLLISION);

    /* Update and test */
    uut0.update(s1, *s0, 0.2, POSSIBLE_COLLISION);
    BOOST_CHECK(uut0.get_simplex().get() == s1);
    BOOST_CHECK(uut0.get_normal_of_collision() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(uut0.get_point_of_collision()  == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(fabs(uut0.get_time() - 0.2) < result_tolerance);
    BOOST_CHECK(uut0.get_type() == POSSIBLE_COLLISION);
    
    /* Construct uncertain collsion and test */
    collision_info uut1(s2, *s0, 1.5, POSSIBLE_COLLISION);
    BOOST_CHECK(uut1.get_simplex().get() == s2);
    BOOST_CHECK(uut1.get_normal_of_collision() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(uut1.get_point_of_collision()  == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(uut1.get_time() == 1.5);
    BOOST_CHECK(uut1.get_type() == POSSIBLE_COLLISION);

    /* Update and test */
    uut1.update(s3, *s1, 1.2, COLLISION);
    BOOST_CHECK(uut1.get_simplex().get() == s3);
    BOOST_CHECK(uut1.get_normal_of_collision() == point_t(0.0, -1.0, 0.0));
    BOOST_CHECK(uut1.get_point_of_collision()  == point_t(1.5, 1.55, 1.0));
    BOOST_CHECK(fabs(uut1.get_time() - 1.2) < result_tolerance);
    BOOST_CHECK(uut1.get_type() == COLLISION);
    
}


/* Switch a collision to sliding */
BOOST_AUTO_TEST_CASE( switch_to_sliding )
{
    /* New simplices, the collision infos will delete these */
    simplex *s0 = new simplex(*data0);
    simplex *s1 = new simplex(*data1);
    simplex *s2 = new simplex(*data1);

    /* Construct certain collision and test */
    collision_info uut(s0, *s1, 0.1, COLLISION);
    BOOST_CHECK(uut.get_simplex().get() == s0);
    BOOST_CHECK(uut.get_normal_of_collision() == point_t(0.0, -1.0, 0.0));
    BOOST_CHECK(uut.get_point_of_collision()  == point_t(1.5, 1.55, 1.0));
    BOOST_CHECK(fabs(uut.get_time() - 0.1) < result_tolerance);
    BOOST_CHECK(uut.get_type() == COLLISION);
    BOOST_CHECK(!uut.switch_to_sliding());
    BOOST_CHECK(uut.switch_to_sliding());

    /* Update to reset sliding and test */
    uut.update(s1, *s0, 0.2, POSSIBLE_COLLISION);
    BOOST_CHECK(!uut.switch_to_sliding());
    BOOST_CHECK(uut.switch_to_sliding());

    /* Update not resetting sliding and test */
    /* Note - Collision type doesnt affect sliding, if no collision then switch_to_sliding should never be called */
    uut.update(s2, *s0, 0.2, NO_COLLISION);
    uut.void_collision();
    BOOST_CHECK(uut.switch_to_sliding());
}

BOOST_AUTO_TEST_SUITE_END()
}; /* namespace test */
}; /* namespace raptor_physics */
