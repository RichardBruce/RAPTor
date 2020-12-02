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
        p0 (point_t(0.0f, 0.0f, 0.0f), 1.0f, 0),
        p1 (point_t(0.0f, 2.1f, 0.0f), 1.0f, 0),
        p2 (point_t(0.0f, 2.0f, 0.0f), 1.0f, 0),
        p3 (point_t(0.0f, 0.0f, 0.0f), 2.8f, 0),
        p4 (point_t(0.2f, 0.0f, 6.9f), 4.1f, 0),
        p5 (point_t(8.3f, 0.0f, 0.0f), 5.5f, 0),
        p6 (point_t(0.2f, 1.0f, 2.7f), 2.8f, 0),
        p7 (point_t(0.4f, 1.0f, 9.6f), 4.1f, 0),
        p8 (point_t(8.5f, 1.0f, 2.7f), 5.5f, 0),
        p9 (point_t(7.8f, 2.2f, 0.1f), 1.0f, 0),
        p10(point_t(7.8f, 4.2f, 0.2f), 1.0f, 0),
        p11(point_t(7.8f, 4.2f, 0.1f), 1.0f, 0),
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
    particle    p9;
    particle    p10;
    particle    p11;
    simplex *   manifold_a;
    simplex *   manifold_b;
    float       t;
};


BOOST_FIXTURE_TEST_SUITE( particle_tests, particle_fixture )

const float result_tolerance = 0.05f;

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

BOOST_AUTO_TEST_CASE( unit_radius_static_no_collision_test )
{
    BOOST_CHECK(p9.resolve_collisions(&p10, &manifold_a, &manifold_b, &t, false) == collision_t::NO_COLLISION);
    BOOST_CHECK(t == 5.0f);

    BOOST_CHECK(p10.resolve_collisions(&p9, &manifold_a, &manifold_b, &t, false) == collision_t::NO_COLLISION);
    BOOST_CHECK(t == 5.0f);
}

BOOST_AUTO_TEST_CASE( unit_radius_static_collision_test )
{
    BOOST_CHECK(p9.resolve_collisions(&p11, &manifold_a, &manifold_b, &t, false) == collision_t::COLLISION);
    BOOST_CHECK(t == 5.0f);

    BOOST_CHECK(p11.resolve_collisions(&p9, &manifold_a, &manifold_b, &t, false) == collision_t::COLLISION);
    BOOST_CHECK(t == 5.0f);
}

BOOST_AUTO_TEST_CASE( origin_unit_radius_no_collision_test )
{
    p0.set_velocity(point_t(0.0f, 0.019f, 0.0f));
    BOOST_CHECK(p0.resolve_collisions(&p1, &manifold_a, &manifold_b, &t, false) == collision_t::NO_COLLISION);
    BOOST_CHECK(t == 5.0f);

    BOOST_CHECK(p1.resolve_collisions(&p0, &manifold_a, &manifold_b, &t, false) == collision_t::NO_COLLISION);
    BOOST_CHECK(t == 5.0f);
}

BOOST_AUTO_TEST_CASE( origin_unit_radius_collision_test )
{
    p0.set_velocity(point_t(0.0f, 0.02f, 0.0f));
    BOOST_CHECK(p0.resolve_collisions(&p1, &manifold_a, &manifold_b, &t, false) == collision_t::COLLISION);
    BOOST_CHECK_CLOSE(t, 5.0f, result_tolerance);

    t = 5.0f;
    BOOST_CHECK(p1.resolve_collisions(&p0, &manifold_a, &manifold_b, &t, false) == collision_t::COLLISION);
    BOOST_CHECK_CLOSE(t, 5.0f, result_tolerance);
}

BOOST_AUTO_TEST_CASE( away_origin_unit_radius_no_collision_test )
{
    p0.set_velocity(point_t(0.0f, -1.0f, 0.0f));
    BOOST_CHECK(p0.resolve_collisions(&p1, &manifold_a, &manifold_b, &t, false) == collision_t::NO_COLLISION);
    BOOST_CHECK(t == 5.0f);

    BOOST_CHECK(p1.resolve_collisions(&p0, &manifold_a, &manifold_b, &t, false) == collision_t::NO_COLLISION);
    BOOST_CHECK(t == 5.0f);
}

BOOST_AUTO_TEST_CASE( fast_origin_unit_radius_collision_test )
{
    p0.set_velocity(point_t(0.0f, 0.2f, 0.0f));
    BOOST_CHECK(p0.resolve_collisions(&p1, &manifold_a, &manifold_b, &t, false) == collision_t::COLLISION);
    BOOST_CHECK_CLOSE(t, 0.5f, result_tolerance);

    t = 5.0f;
    BOOST_CHECK(p1.resolve_collisions(&p0, &manifold_a, &manifold_b, &t, false) == collision_t::COLLISION);
    BOOST_CHECK_CLOSE(t, 0.5f, result_tolerance);
}

BOOST_AUTO_TEST_CASE( origin_no_collision_test )
{
    p3.set_velocity(point_t(0.019f, 0.0f, 0.0f));
    BOOST_CHECK(p3.resolve_collisions(&p4, &manifold_a, &manifold_b, &t, false) == collision_t::NO_COLLISION);
    BOOST_CHECK(t == 5.0f);

    BOOST_CHECK(p4.resolve_collisions(&p3, &manifold_a, &manifold_b, &t, false) == collision_t::NO_COLLISION);
    BOOST_CHECK(t == 5.0f);
}

BOOST_AUTO_TEST_CASE( origin_collision_test )
{
    p3.set_velocity(point_t(0.04f, 0.0f, 0.000001f));
    BOOST_CHECK(p3.resolve_collisions(&p4, &manifold_a, &manifold_b, &t, false) == collision_t::COLLISION);
    BOOST_CHECK_CLOSE(t, 4.809f, result_tolerance);

    t = 5.0f;
    BOOST_CHECK(p4.resolve_collisions(&p3, &manifold_a, &manifold_b, &t, false) == collision_t::COLLISION);
    BOOST_CHECK_CLOSE(t, 4.809f, result_tolerance);
}

BOOST_AUTO_TEST_CASE( away_origin_no_collision_test )
{
    p3.set_velocity(point_t(-0.02f, 0.0f, 0.000001f));
    BOOST_CHECK(p3.resolve_collisions(&p4, &manifold_a, &manifold_b, &t, false) == collision_t::NO_COLLISION);
    BOOST_CHECK(t == 5.0f);

    BOOST_CHECK(p4.resolve_collisions(&p3, &manifold_a, &manifold_b, &t, false) == collision_t::NO_COLLISION);
    BOOST_CHECK(t == 5.0f);
}

BOOST_AUTO_TEST_CASE( fast_origin_collision_test )
{
    p3.set_velocity(point_t(0.4f, 0.0f, 0.000001f));
    BOOST_CHECK(p3.resolve_collisions(&p4, &manifold_a, &manifold_b, &t, false) == collision_t::COLLISION);
    BOOST_CHECK_CLOSE(t, 0.5f, result_tolerance);

    t = 5.0f;
    BOOST_CHECK(p4.resolve_collisions(&p3, &manifold_a, &manifold_b, &t, false) == collision_t::COLLISION);
    BOOST_CHECK_CLOSE(t, 0.5f, result_tolerance);
}

BOOST_AUTO_TEST_CASE( no_collision_test )
{
    p6.set_velocity(point_t(0.019f, 0.0f, 0.0f));
    BOOST_CHECK(p6.resolve_collisions(&p7, &manifold_a, &manifold_b, &t, false) == collision_t::NO_COLLISION);
    BOOST_CHECK(t == 5.0f);

    BOOST_CHECK(p7.resolve_collisions(&p6, &manifold_a, &manifold_b, &t, false) == collision_t::NO_COLLISION);
    BOOST_CHECK(t == 5.0f);
}

BOOST_AUTO_TEST_CASE( collision_test )
{
    p6.set_velocity(point_t(0.04f, 0.0f, 0.000001f));
    BOOST_CHECK(p6.resolve_collisions(&p7, &manifold_a, &manifold_b, &t, false) == collision_t::COLLISION);
    BOOST_CHECK_CLOSE(t, 4.821f, result_tolerance);

    BOOST_CHECK(p7.resolve_collisions(&p6, &manifold_a, &manifold_b, &t, false) == collision_t::COLLISION);
    BOOST_CHECK_CLOSE(t, 4.821f, result_tolerance);
}

BOOST_AUTO_TEST_CASE( away_no_collision_test )
{
    p6.set_velocity(point_t(-0.02f, 0.0f, 0.0f));
    BOOST_CHECK(p6.resolve_collisions(&p7, &manifold_a, &manifold_b, &t, false) == collision_t::NO_COLLISION);
    BOOST_CHECK(t == 5.0f);

    BOOST_CHECK(p7.resolve_collisions(&p6, &manifold_a, &manifold_b, &t, false) == collision_t::NO_COLLISION);
    BOOST_CHECK(t == 5.0f);
}

BOOST_AUTO_TEST_CASE( fast_collision_test )
{
    p6.set_velocity(point_t(0.4f, 0.0f, 0.00001f));
    BOOST_CHECK(p6.resolve_collisions(&p7, &manifold_a, &manifold_b, &t, false) == collision_t::COLLISION);
    BOOST_CHECK_CLOSE(t, 0.4821f, result_tolerance);

    BOOST_CHECK(p7.resolve_collisions(&p6, &manifold_a, &manifold_b, &t, false) == collision_t::COLLISION);
    BOOST_CHECK_CLOSE(t, 0.4821f, result_tolerance);
}

BOOST_AUTO_TEST_CASE( unit_radius_no_collision_test )
{
    p9.set_velocity(point_t(0.0f, 0.0f, 0.019f));
    BOOST_CHECK(p9.resolve_collisions(&p10, &manifold_a, &manifold_b, &t, false) == collision_t::NO_COLLISION);
    BOOST_CHECK(t == 5.0f);

    BOOST_CHECK(p10.resolve_collisions(&p9, &manifold_a, &manifold_b, &t, false) == collision_t::NO_COLLISION);
    BOOST_CHECK(t == 5.0f);
}

BOOST_AUTO_TEST_CASE( unit_radius_collision_test )
{
    p9.set_velocity(point_t(0.0f, 0.0f, 0.02f));
    BOOST_CHECK(p9.resolve_collisions(&p10, &manifold_a, &manifold_b, &t, false) == collision_t::COLLISION);
    BOOST_CHECK_CLOSE(t, 4.958f, result_tolerance);

    t = 5.0f;
    BOOST_CHECK(p10.resolve_collisions(&p9, &manifold_a, &manifold_b, &t, false) == collision_t::COLLISION);
    BOOST_CHECK_CLOSE(t, 4.958f, result_tolerance);
}

BOOST_AUTO_TEST_CASE( away_unit_radius_no_collision_test )
{
    p9.set_velocity(point_t(0.0f, 0.0f, -0.02f));
    BOOST_CHECK(p9.resolve_collisions(&p10, &manifold_a, &manifold_b, &t, false) == collision_t::NO_COLLISION);
    BOOST_CHECK(t == 5.0f);

    BOOST_CHECK(p10.resolve_collisions(&p9, &manifold_a, &manifold_b, &t, false) == collision_t::NO_COLLISION);
    BOOST_CHECK(t == 5.0f);
}

BOOST_AUTO_TEST_CASE( fast_unit_radius_collision_test )
{
    p9.set_velocity(point_t(0.0f, 0.0f, 0.2f));
    BOOST_CHECK(p9.resolve_collisions(&p10, &manifold_a, &manifold_b, &t, false) == collision_t::COLLISION);
    BOOST_CHECK_CLOSE(t, 0.4958f, result_tolerance);

    t = 5.0f;
    BOOST_CHECK(p10.resolve_collisions(&p9, &manifold_a, &manifold_b, &t, false) == collision_t::COLLISION);
    BOOST_CHECK_CLOSE(t, 0.4958f, result_tolerance);
}

BOOST_AUTO_TEST_CASE( both_moving_origin_unit_radius_no_collision_test )
{
    p0.set_velocity(point_t(0.0f,  0.009f, 3.8f));
    p1.set_velocity(point_t(0.0f, -0.01f,  3.8f));
    BOOST_CHECK(p0.resolve_collisions(&p1, &manifold_a, &manifold_b, &t, false) == collision_t::NO_COLLISION);
    BOOST_CHECK(t == 5.0f);

    BOOST_CHECK(p1.resolve_collisions(&p0, &manifold_a, &manifold_b, &t, false) == collision_t::NO_COLLISION);
    BOOST_CHECK(t == 5.0f);
}

BOOST_AUTO_TEST_CASE( both_moving_origin_unit_radius_collision_test )
{
    p0.set_velocity(point_t(3.8f,  0.005f, 0.0f));
    p1.set_velocity(point_t(3.8f, -0.015f, 0.0f));
    BOOST_CHECK(p0.resolve_collisions(&p1, &manifold_a, &manifold_b, &t, false) == collision_t::COLLISION);
    BOOST_CHECK_CLOSE(t, 5.0f, result_tolerance);

    t = 5.0f;
    BOOST_CHECK(p1.resolve_collisions(&p0, &manifold_a, &manifold_b, &t, false) == collision_t::COLLISION);
    BOOST_CHECK_CLOSE(t, 5.0f, result_tolerance);
}

BOOST_AUTO_TEST_CASE( both_moving_away_origin_unit_radius_no_collision_test )
{
    p0.set_velocity(point_t(0.0f, -1.0f, 0.0f));
    p1.set_velocity(point_t(0.0f,  8.0f, 0.0f));
    BOOST_CHECK(p0.resolve_collisions(&p1, &manifold_a, &manifold_b, &t, false) == collision_t::NO_COLLISION);
    BOOST_CHECK(t == 5.0f);

    BOOST_CHECK(p1.resolve_collisions(&p0, &manifold_a, &manifold_b, &t, false) == collision_t::NO_COLLISION);
    BOOST_CHECK(t == 5.0f);
}

BOOST_AUTO_TEST_CASE( both_moving_fast_origin_unit_radius_collision_test )
{
    p0.set_velocity(point_t(0.0f,  0.15f, 0.0f));
    p1.set_velocity(point_t(0.0f, -0.05f, 0.0f));
    BOOST_CHECK(p0.resolve_collisions(&p1, &manifold_a, &manifold_b, &t, false) == collision_t::COLLISION);
    BOOST_CHECK_CLOSE(t, 0.5f, result_tolerance);

    t = 5.0f;
    BOOST_CHECK(p1.resolve_collisions(&p0, &manifold_a, &manifold_b, &t, false) == collision_t::COLLISION);
    BOOST_CHECK_CLOSE(t, 0.5f, result_tolerance);
}

BOOST_AUTO_TEST_CASE( both_moving_origin_no_collision_test )
{
    p3.set_velocity(point_t( 0.010f, 0.0f, 0.0f));
    p4.set_velocity(point_t(-0.009f, 0.0f, 0.0f));
    BOOST_CHECK(p3.resolve_collisions(&p4, &manifold_a, &manifold_b, &t, false) == collision_t::NO_COLLISION);
    BOOST_CHECK(t == 5.0f);

    BOOST_CHECK(p4.resolve_collisions(&p3, &manifold_a, &manifold_b, &t, false) == collision_t::NO_COLLISION);
    BOOST_CHECK(t == 5.0f);
}

BOOST_AUTO_TEST_CASE( both_moving_origin_collision_test )
{
    p3.set_velocity(point_t( 0.01f, 0.0f, 0.000001f));
    p4.set_velocity(point_t(-0.03f, 0.0f, 0.0f));
    BOOST_CHECK(p3.resolve_collisions(&p4, &manifold_a, &manifold_b, &t, false) == collision_t::COLLISION);
    BOOST_CHECK_CLOSE(t, 4.809f, result_tolerance);

    t = 5.0f;
    BOOST_CHECK(p4.resolve_collisions(&p3, &manifold_a, &manifold_b, &t, false) == collision_t::COLLISION);
    BOOST_CHECK_CLOSE(t, 4.809f, result_tolerance);
}

BOOST_AUTO_TEST_CASE( both_moving_away_origin_no_collision_test )
{
    p3.set_velocity(point_t(-0.02f, 0.0f, 0.000001f));
    p4.set_velocity(point_t( 0.02f, 0.0f, 0.000001f));
    BOOST_CHECK(p3.resolve_collisions(&p4, &manifold_a, &manifold_b, &t, false) == collision_t::NO_COLLISION);
    BOOST_CHECK(t == 5.0f);

    BOOST_CHECK(p4.resolve_collisions(&p3, &manifold_a, &manifold_b, &t, false) == collision_t::NO_COLLISION);
    BOOST_CHECK(t == 5.0f);
}

BOOST_AUTO_TEST_CASE( both_moving_fast_origin_collision_test )
{
    p3.set_velocity(point_t( 0.25f, 0.0f, 0.000001f));
    p4.set_velocity(point_t(-0.15f, 0.0f, 0.0f));
    BOOST_CHECK(p3.resolve_collisions(&p4, &manifold_a, &manifold_b, &t, false) == collision_t::COLLISION);
    BOOST_CHECK_CLOSE(t, 0.5f, result_tolerance);

    t = 5.0f;
    BOOST_CHECK(p4.resolve_collisions(&p3, &manifold_a, &manifold_b, &t, false) == collision_t::COLLISION);
    BOOST_CHECK_CLOSE(t, 0.5f, result_tolerance);
}

BOOST_AUTO_TEST_CASE( both_moving_no_collision_test )
{
    p6.set_velocity(point_t( 0.009f, 0.0f, 0.0f));
    p7.set_velocity(point_t(-0.010f, 0.0f, 0.0f));
    BOOST_CHECK(p6.resolve_collisions(&p7, &manifold_a, &manifold_b, &t, false) == collision_t::NO_COLLISION);
    BOOST_CHECK(t == 5.0f);

    BOOST_CHECK(p7.resolve_collisions(&p6, &manifold_a, &manifold_b, &t, false) == collision_t::NO_COLLISION);
    BOOST_CHECK(t == 5.0f);
}

BOOST_AUTO_TEST_CASE( both_moving_collision_test )
{
    p6.set_velocity(point_t( 0.01f, 0.0f, 0.000001f));
    p7.set_velocity(point_t(-0.03f, 0.0f, 0.0f));
    BOOST_CHECK(p6.resolve_collisions(&p7, &manifold_a, &manifold_b, &t, false) == collision_t::COLLISION);
    BOOST_CHECK_CLOSE(t, 4.821f, result_tolerance);

    BOOST_CHECK(p7.resolve_collisions(&p6, &manifold_a, &manifold_b, &t, false) == collision_t::COLLISION);
    BOOST_CHECK_CLOSE(t, 4.821f, result_tolerance);
}

BOOST_AUTO_TEST_CASE( both_moving_away_no_collision_test )
{
    p6.set_velocity(point_t(-0.01f, 0.0f, 0.0f));
    p7.set_velocity(point_t( 0.01f, 0.0f, 0.0f));
    BOOST_CHECK(p6.resolve_collisions(&p7, &manifold_a, &manifold_b, &t, false) == collision_t::NO_COLLISION);
    BOOST_CHECK(t == 5.0f);

    BOOST_CHECK(p7.resolve_collisions(&p6, &manifold_a, &manifold_b, &t, false) == collision_t::NO_COLLISION);
    BOOST_CHECK(t == 5.0f);
}

BOOST_AUTO_TEST_CASE( both_moving_fast_collision_test )
{
    p6.set_velocity(point_t( 0.15f, 0.0f, 0.00001f));
    p7.set_velocity(point_t(-0.25f, 0.0f, 0.0f));
    BOOST_CHECK(p6.resolve_collisions(&p7, &manifold_a, &manifold_b, &t, false) == collision_t::COLLISION);
    BOOST_CHECK_CLOSE(t, 0.4821f, result_tolerance);

    BOOST_CHECK(p7.resolve_collisions(&p6, &manifold_a, &manifold_b, &t, false) == collision_t::COLLISION);
    BOOST_CHECK_CLOSE(t, 0.4821f, result_tolerance);
}

BOOST_AUTO_TEST_CASE( both_moving_unit_radius_no_collision_test )
{
    p9.set_velocity( point_t(0.0f, 0.0f,  0.010f));
    p10.set_velocity(point_t(0.0f, 0.0f, -0.009f));
    BOOST_CHECK(p9.resolve_collisions(&p10, &manifold_a, &manifold_b, &t, false) == collision_t::NO_COLLISION);
    BOOST_CHECK(t == 5.0f);

    BOOST_CHECK(p10.resolve_collisions(&p9, &manifold_a, &manifold_b, &t, false) == collision_t::NO_COLLISION);
    BOOST_CHECK(t == 5.0f);
}

BOOST_AUTO_TEST_CASE( both_moving_unit_radius_collision_test )
{
    p9.set_velocity( point_t(0.0f, 0.0f,  0.005f));
    p10.set_velocity(point_t(0.0f, 0.0f, -0.015f));
    BOOST_CHECK(p9.resolve_collisions(&p10, &manifold_a, &manifold_b, &t, false) == collision_t::COLLISION);
    BOOST_CHECK_CLOSE(t, 4.958f, result_tolerance);

    t = 5.0f;
    BOOST_CHECK(p10.resolve_collisions(&p9, &manifold_a, &manifold_b, &t, false) == collision_t::COLLISION);
    BOOST_CHECK_CLOSE(t, 4.958f, result_tolerance);
}

BOOST_AUTO_TEST_CASE( both_moving_away_unit_radius_no_collision_test )
{
    p9.set_velocity( point_t(0.0f, 0.0f, -0.01f));
    p10.set_velocity(point_t(0.0f, 0.0f,  0.01f));
    BOOST_CHECK(p9.resolve_collisions(&p10, &manifold_a, &manifold_b, &t, false) == collision_t::NO_COLLISION);
    BOOST_CHECK(t == 5.0f);

    BOOST_CHECK(p10.resolve_collisions(&p9, &manifold_a, &manifold_b, &t, false) == collision_t::NO_COLLISION);
    BOOST_CHECK(t == 5.0f);
}

BOOST_AUTO_TEST_CASE( both_moving_fast_unit_radius_collision_test )
{
    p9.set_velocity( point_t(0.0f, 0.0f,  0.15f));
    p10.set_velocity(point_t(0.0f, 0.0f, -0.05f));
    BOOST_CHECK(p9.resolve_collisions(&p10, &manifold_a, &manifold_b, &t, false) == collision_t::COLLISION);
    BOOST_CHECK_CLOSE(t, 0.4958f, result_tolerance);

    t = 5.0f;
    BOOST_CHECK(p10.resolve_collisions(&p9, &manifold_a, &manifold_b, &t, false) == collision_t::COLLISION);
    BOOST_CHECK_CLOSE(t, 0.4958f, result_tolerance);
}

BOOST_AUTO_TEST_SUITE_END()
}; /* namespace test */
}; /* namespace raptor_physics */
