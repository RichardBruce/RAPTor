#ifdef STAND_ALONE
#define BOOST_TEST_MODULE collision_info<> test

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
    collision_info_fixture() :
        data0(physics_object_for_simplex_testing(new std::vector<point_t>(
            {
                point_t( 1.5f,  1.5f, 1.0f),
                point_t( 1.5f, -0.5f, 1.0f),
                point_t(-0.5f, -0.5f, 1.0f),
                point_t(-0.5f,  1.5f, 1.0f)
            }), { 0, 1, 2, 3 }, quaternion_t(1.0f, 0.0f, 0.0f, 0.0f), point_t(0.0f, 0.0f, 0.0f))),
        data1(physics_object_for_simplex_testing(new std::vector<point_t>(
            {
                point_t( 1.5f,  1.6f,  1.0f),
                point_t(-1.5f,  0.5f,  5.0f),
                point_t(-3.5f, -0.8f,  1.0f),
                point_t(-0.5f,  1.5f, -2.0f)
            }), { 0, 1, 2, 3 }, quaternion_t(1.0f, 0.0f, 0.0f, 0.0f), point_t(0.0f, 0.0f, 0.0f)))
    {  };

    std::unique_ptr<physics_object> data0;
    std::unique_ptr<physics_object> data1;
};


BOOST_FIXTURE_TEST_SUITE( collision_info_tests, collision_info_fixture )

const float result_tolerance = 0.0005f;


/* Test Construction */
BOOST_AUTO_TEST_CASE( ctor_test )
{
    /* New simplices, the collision infos will delete these */
    simplex *s0 = new simplex(*data0, true);
    simplex *s1 = new simplex(*data1, true);
    simplex *s2 = new simplex(*data1, true);

    /* Test certain collision */
    collision_info<> uut0(s0, *s1, 0.1f, collision_t::COLLISION);
    BOOST_CHECK(uut0.get_simplex().get()        == s0);
    BOOST_CHECK(uut0.get_normal_of_collision()  == point_t(0.0f, -1.0f,  0.0f));
    BOOST_CHECK(uut0.get_point_of_collision()   == point_t(0.5f,  1.5f, 1.0f));
    BOOST_CHECK(uut0.get_type()                 == collision_t::COLLISION);
    BOOST_CHECK(fabs(uut0.get_time() - 0.1f) < result_tolerance);
    
    /* Test uncertain collision */
    collision_info<> uut1(s1, *s0, 1.5f, collision_t::POSSIBLE_COLLISION);
    BOOST_CHECK(uut1.get_simplex().get()        == s1);
    BOOST_CHECK(uut1.get_normal_of_collision()  == point_t(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(uut1.get_point_of_collision()   == point_t(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(uut1.get_type()                 == collision_t::POSSIBLE_COLLISION);
    BOOST_CHECK(uut1.get_time()                 == 1.5f);
    
    /* Test certain collision, but with flipped simplices */
    collision_info<> uut2(s2, *s0, 0.5f, collision_t::COLLISION);
    BOOST_CHECK(uut2.get_simplex().get()        == s2);
    BOOST_CHECK(uut2.get_normal_of_collision()  == point_t(0.0f, 1.0f,  0.0f));
    BOOST_CHECK(uut2.get_point_of_collision()   == point_t(0.5f, 1.5f, 1.0f));
    BOOST_CHECK(uut2.get_type()                 == collision_t::COLLISION);
    BOOST_CHECK(uut2.get_time()                 == 0.5f);
}

/* Voiding a collision */
BOOST_AUTO_TEST_CASE( void_collsion_test )
{
    /* New simplices, the collision infos will delete these */
    simplex *s0 = new simplex(*data0, true);
    std::unique_ptr<simplex> s1(new simplex(*data1, true));

    /* Construct and test */
    collision_info<> uut(s0, *s1, 0.1f, collision_t::COLLISION);
    BOOST_CHECK(uut.get_simplex().get()         == s0);
    BOOST_CHECK(uut.get_normal_of_collision()   == point_t(0.0f, -1.0f, 0.0f));
    BOOST_CHECK(uut.get_point_of_collision()    == point_t(0.5f,  1.5f, 1.0f));
    BOOST_CHECK(uut.get_type()                  == collision_t::COLLISION);
    BOOST_CHECK(fabs(uut.get_time() - 0.1f) < result_tolerance);

    /* Void */
    uut.void_collision();

    /* Checks */
    BOOST_CHECK(uut.get_simplex().get()         == s0);
    BOOST_CHECK(uut.get_normal_of_collision()   == point_t(0.0f, -1.0f, 0.0f));
    BOOST_CHECK(uut.get_point_of_collision()    == point_t(0.5f,  1.5f, 1.0f));
    BOOST_CHECK(uut.get_type()                  == collision_t::COLLISION);
    BOOST_CHECK(uut.get_time()                  == std::numeric_limits<float>::max());
}

/* Successfully retest a collision */
BOOST_AUTO_TEST_CASE( successful_retest_update_test )
{
    /* New simplices, the collision infos will delete these */
    std::unique_ptr<simplex> s0(new simplex(*data0, true));
    simplex *s1 = new simplex(*data1, true);
    simplex *s2 = new simplex(*data1, true);
    
    /* Construct and test */
    collision_info<> uut(s1, *s0, 1.5f, collision_t::POSSIBLE_COLLISION);
    BOOST_CHECK(uut.get_simplex().get()         == s1);
    BOOST_CHECK(uut.get_normal_of_collision()   == point_t(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(uut.get_point_of_collision()    == point_t(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(uut.get_type()                  == collision_t::POSSIBLE_COLLISION);
    BOOST_CHECK(uut.get_time()                  == 1.5f);

    /* Update */
    uut.successful_retest_update(s2, *s0);

    /* Checks */
    BOOST_CHECK(uut.get_simplex().get()         == s2);
    BOOST_CHECK(uut.get_normal_of_collision()   == point_t(0.0f, 1.0f, 0.0f));
    BOOST_CHECK(uut.get_point_of_collision()    == point_t(0.5f, 1.5f, 1.0f));
    BOOST_CHECK(uut.get_type()                  == collision_t::COLLISION);
    BOOST_CHECK(uut.get_time()                  == 1.5f);
}

/* Update a collision */
BOOST_AUTO_TEST_CASE( update_test )
{
    /* New simplices, the collision infos will delete these */
    simplex *s0 = new simplex(*data0, true);
    simplex *s1 = new simplex(*data1, true);
    simplex *s2 = new simplex(*data1, true);
    simplex *s3 = new simplex(*data0, true);

    /* Construct certain collision and test */
    collision_info<> uut0(s0, *s1, 0.1f, collision_t::COLLISION);
    BOOST_CHECK(uut0.get_simplex().get()        == s0);
    BOOST_CHECK(uut0.get_normal_of_collision()  == point_t(0.0f, -1.0f, 0.0f));
    BOOST_CHECK(uut0.get_point_of_collision()   == point_t(0.5f,  1.5f, 1.0f));
    BOOST_CHECK(uut0.get_type()                 == collision_t::COLLISION);
    BOOST_CHECK(fabs(uut0.get_time() - 0.1f) < result_tolerance);

    /* Update */
    uut0.update(s1, *s0, 0.2, collision_t::POSSIBLE_COLLISION);

    /* Checks */
    BOOST_CHECK(uut0.get_simplex().get()        == s1);
    BOOST_CHECK(uut0.get_normal_of_collision()  == point_t(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(uut0.get_point_of_collision()   == point_t(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(uut0.get_type()                 == collision_t::POSSIBLE_COLLISION);
    BOOST_CHECK(fabs(uut0.get_time() - 0.2f) < result_tolerance);
    
    /* Construct uncertain collsion and test */
    collision_info<> uut1(s2, *s0, 1.5f, collision_t::POSSIBLE_COLLISION);
    BOOST_CHECK(uut1.get_simplex().get()        == s2);
    BOOST_CHECK(uut1.get_normal_of_collision()  == point_t(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(uut1.get_point_of_collision()   == point_t(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(uut1.get_type()                 == collision_t::POSSIBLE_COLLISION);
    BOOST_CHECK(uut1.get_time()                 == 1.5f);

    /* Update */
    uut1.update(s3, *s1, 1.2f, collision_t::COLLISION);

    /* Checks */
    BOOST_CHECK(uut1.get_simplex().get()        == s3);
    BOOST_CHECK(uut1.get_normal_of_collision()  == point_t(0.0f, -1.0f, 0.0f));
    BOOST_CHECK(uut1.get_point_of_collision()   == point_t(0.5f,  1.5f, 1.0f));
    BOOST_CHECK(uut1.get_type()                 == collision_t::COLLISION);
    BOOST_CHECK(fabs(uut1.get_time() - 1.2f) < result_tolerance);
    
}

/* Switch a collision to sliding */
BOOST_AUTO_TEST_CASE( switch_to_sliding )
{
    /* New simplices, the collision infos will delete these */
    simplex *s0 = new simplex(*data0, true);
    simplex *s1 = new simplex(*data1, true);
    simplex *s2 = new simplex(*data1, true);

    /* Construct certain collision and test */
    collision_info<> uut(s0, *s1, 0.1f, collision_t::COLLISION);
    BOOST_CHECK(uut.get_simplex().get()         == s0);
    BOOST_CHECK(uut.get_normal_of_collision()   == point_t(0.0f, -1.0f, 0.0f));
    BOOST_CHECK(uut.get_point_of_collision()    == point_t(0.5f,  1.5f, 1.0f));
    BOOST_CHECK(uut.get_type()                  == collision_t::COLLISION);
    BOOST_CHECK(fabs(uut.get_time() - 0.1f) < result_tolerance);
    BOOST_CHECK(!uut.switch_to_sliding());
    BOOST_CHECK(uut.switch_to_sliding());

    /* Update to reset sliding and test */
    uut.update(s1, *s0, 0.2f, collision_t::POSSIBLE_COLLISION);
    BOOST_CHECK(!uut.switch_to_sliding());
    BOOST_CHECK(uut.switch_to_sliding());

    /* Update not resetting sliding and test */
    /* Note - Collision type doesnt affect sliding, if no collision then switch_to_sliding should never be called */
    uut.update(s2, *s0, 0.2f, collision_t::NO_COLLISION);
    uut.void_collision();
    BOOST_CHECK(uut.switch_to_sliding());
}

BOOST_AUTO_TEST_SUITE_END()
}; /* namespace test */
}; /* namespace raptor_physics */
