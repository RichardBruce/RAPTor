#ifdef STAND_ALONE
#define BOOST_TEST_MODULE particle test

/* Common headers */
#include "logging.h"

/* Initialise logger */
const raptor_physics::init_logger init_logger;
#endif /* #ifdef STAND_ALONE */


/* Boost headers */
#include "boost/test/unit_test.hpp"
#include "boost/noncopyable.hpp"


/* Physics headers */
#include "point_t.h"
#include "particle.h"

/* Test headers */
#include "mock_force.h"


namespace raptor_physics
{
namespace test
{
/* Test data */
struct particle_fixture : private boost::noncopyable
{
    particle_fixture() :
        p0(point_t(0.0f, 0.0f, 0.0f), 1.0f, 0),
        p1(point_t(0.0f, 2.1f, 0.0f), 1.0f, 0),
        p2(point_t(0.0f, 2.0f, 0.0f), 1.0f, 0),
        p3(point_t(0.0f, 0.0f, 0.0f), 2.8f, 0),
        p4(point_t(0.1f, 0.0f, 6.9f), 4.1f, 0),
        p5(point_t(8.3f, 0.0f, 0.0f), 5.5f, 0),
        p6(point_t(0.2f, 1.0f, 2.7f), 2.8f, 0),
        p7(point_t(0.3f, 1.0f, 9.6f), 4.1f, 0),
        p8(point_t(8.5f, 1.0f, 2.7f), 5.5f, 0),
        t(5.0f)
      {  };

    ~particle_fixture()
    {
    }

    particle    p0;
    particle    p1;
    particle    p2;
    particle    p3;
    particle    p4;
    particle    p5;
    particle    p6;
    particle    p7;
    particle    p8;
    simplex *   manifold_a;
    simplex *   manifold_b;
    float       t;
};


BOOST_FIXTURE_TEST_SUITE( particle_tests, particle_fixture )

const float result_tolerance = 0.0005f;

BOOST_AUTO_TEST_CASE( no_self_collision_test )
{
    BOOST_CHECK(p0.resolve_collisions(&p0, &manifold_a, &manifold_b, &t, false) == collision_t::NO_COLLISION);
    BOOST_CHECK(t == 5.0f);
}

BOOST_AUTO_TEST_CASE( origin_unit_radius_static_no_collision_test )
{
    BOOST_CHECK(p0.resolve_collisions(&p1, &manifold_a, &manifold_b, &t, false) == collision_t::NO_COLLISION);
    BOOST_CHECK(t == 5.0f);

    BOOST_CHECK(p1.resolve_collisions(&p0, &manifold_a, &manifold_b, &t, false) == collision_t::NO_COLLISION);
    BOOST_CHECK(t == 5.0f);
}

BOOST_AUTO_TEST_CASE( origin_unit_radius_static_collision_test )
{
    BOOST_CHECK(p0.resolve_collisions(&p2, &manifold_a, &manifold_b, &t, false) == collision_t::COLLISION);
    BOOST_CHECK(t == 5.0f);

    BOOST_CHECK(p2.resolve_collisions(&p0, &manifold_a, &manifold_b, &t, false) == collision_t::COLLISION);
    BOOST_CHECK(t == 5.0f);
}

BOOST_AUTO_TEST_CASE( origin_static_no_collision_test )
{
    BOOST_CHECK(p3.resolve_collisions(&p4, &manifold_a, &manifold_b, &t, false) == collision_t::NO_COLLISION);
    BOOST_CHECK(t == 5.0f);

    BOOST_CHECK(p4.resolve_collisions(&p3, &manifold_a, &manifold_b, &t, false) == collision_t::NO_COLLISION);
    BOOST_CHECK(t == 5.0f);
}

BOOST_AUTO_TEST_CASE( origin_static_collision_test )
{
    BOOST_CHECK(p3.resolve_collisions(&p5, &manifold_a, &manifold_b, &t, false) == collision_t::COLLISION);
    BOOST_CHECK(t == 5.0f);

    BOOST_CHECK(p5.resolve_collisions(&p3, &manifold_a, &manifold_b, &t, false) == collision_t::COLLISION);
    BOOST_CHECK(t == 5.0f);
}

BOOST_AUTO_TEST_CASE( static_no_collision_test )
{
    BOOST_CHECK(p6.resolve_collisions(&p7, &manifold_a, &manifold_b, &t, false) == collision_t::NO_COLLISION);
    BOOST_CHECK(t == 5.0f);

    BOOST_CHECK(p7.resolve_collisions(&p6, &manifold_a, &manifold_b, &t, false) == collision_t::NO_COLLISION);
    BOOST_CHECK(t == 5.0f);
}

BOOST_AUTO_TEST_CASE( static_collision_test )
{
    BOOST_CHECK(p6.resolve_collisions(&p8, &manifold_a, &manifold_b, &t, false) == collision_t::COLLISION);
    BOOST_CHECK(t == 5.0f);

    BOOST_CHECK(p8.resolve_collisions(&p6, &manifold_a, &manifold_b, &t, false) == collision_t::COLLISION);
    BOOST_CHECK(t == 5.0f);
}

BOOST_AUTO_TEST_SUITE_END()
}; /* namespace test */
}; /* namespace raptor_physics */
