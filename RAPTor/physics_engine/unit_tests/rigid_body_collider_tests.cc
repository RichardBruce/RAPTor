#ifdef STAND_ALONE
#define BOOST_TEST_MODULE rigid_body_collider test

/* Common headers */
#include "logging.h"

/* Initialise logger */
const raptor_physics::init_logger init_logger;
#endif /* #ifdef STAND_ALONE */


/* Standard headers */
#include <limits>

/* Boost headers */
#include "boost/test/unit_test.hpp"

/* Raytracer headers */
#include "phong_shader.h"

/* Physics headers */
#include "inertia_tensor.h"
#include "vertex_group.h"
#include "rigid_body_collider.h"

/* Test headers */
#include "mock_physics_object.h"


namespace raptor_physics
{
namespace test
{
struct rigid_body_collider_fixture
{
    const float inf = std::numeric_limits<float>::infinity();
};

BOOST_FIXTURE_TEST_SUITE( rigid_body_collider_tests, rigid_body_collider_fixture )

const float result_tolerance = 0.001f;


/* Test instantaneous frictionless collide with no rotation */
BOOST_AUTO_TEST_CASE( instantaneous_frictionless_rotationless_collide_test )
{
    const float cor = 1.0f;
    const point_t<> poc(0.0f, 0.0f, 0.0f);
    const point_t<> noc(1.0f, 0.0f, 0.0f);
    mock_physics_object po_a(new inertia_tensor(new float[6]{ 1.0f, 1.1f, 1.2f, 0.0f, 0.0f, 0.0f }, point_t<>( 1.0f, 0.0f, 0.0f), 1.0f), point_t<>(-5.0f, 0.0f, 0.0f), point_t<>(0.0f, 0.0f, 0.0f));
    mock_physics_object po_b(new inertia_tensor(new float[6]{ 2.9f, 2.8f, 2.7f, 0.0f, 0.0f, 0.0f }, point_t<>(-4.3f, 0.0f, 0.0f), 2.5f), point_t<>( 7.0f, 0.0f, 0.0f), point_t<>(0.0f, 0.0f, 0.0f));
    instantaneous_frictionless_collide(&po_a, &po_b, poc, noc, cor);

    BOOST_CHECK(po_a.get_collision_normal() == noc);
    BOOST_CHECK(po_a.get_angular_weight() == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 17.1429f, result_tolerance);

    BOOST_CHECK(po_b.get_collision_normal() == noc);
    BOOST_CHECK(po_b.get_angular_weight() == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK_CLOSE(po_b.get_impulse(), -17.1429f, result_tolerance);
}

BOOST_AUTO_TEST_CASE( instantaneous_frictionless_rotationless_collide_infinite_mass_a_test )
{
    const float cor = 0.5f;
    const point_t<> poc(0.0f, 0.0f, 0.0f);
    const point_t<> noc(1.0f, 0.0f, 0.0f);
    mock_physics_object po_a(new inertia_tensor(new float[6]{ inf, inf, inf, inf, inf, inf }, point_t<>( 1.0f, 0.0f, 0.0f), inf), point_t<>(-5.0f, 0.0f, 0.0f), point_t<>(0.0f, 0.0f, 0.0f));
    mock_physics_object po_b(new inertia_tensor(new float[6]{ 2.9f, 2.8f, 2.7f, 0.0f, 0.0f, 0.0f }, point_t<>(-4.3f, 0.0f, 0.0f), 2.5f), point_t<>( 7.0f, 0.0f, 0.0f), point_t<>(0.0f, 0.0f, 0.0f));
    instantaneous_frictionless_collide(&po_a, &po_b, poc, noc, cor);

    BOOST_CHECK(po_a.get_collision_normal() == noc);
    BOOST_CHECK(po_a.get_angular_weight() == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 45.0f, result_tolerance);

    BOOST_CHECK(po_b.get_collision_normal() == noc);
    BOOST_CHECK(po_b.get_angular_weight() == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK_CLOSE(po_b.get_impulse(), -45.0f, result_tolerance);
}

BOOST_AUTO_TEST_CASE( instantaneous_frictionless_rotationless_collide_infinite_mass_b_test )
{
    const float cor = 0.75f;
    const point_t<> poc(0.0f, 0.0f, 0.0f);
    const point_t<> noc(0.0f, 1.0f, 0.0f);
    mock_physics_object po_a(new inertia_tensor(new float[6]{ 1.0f, 1.1f, 1.2f, 0.0f, 0.0f, 0.0f }, point_t<>(0.0f,  1.0f, 0.0f), 1.0f), point_t<>(0.0f, -5.0f, 0.0f), point_t<>(0.0f, 0.0f, 0.0f));
    mock_physics_object po_b(new inertia_tensor(new float[6]{ inf, inf, inf, inf, inf, inf }, point_t<>(0.0f, -4.3f, 0.0f), inf), point_t<>(0.0f, 7.0f, 0.0f), point_t<>(0.0f, 0.0f, 0.0f));
    instantaneous_frictionless_collide(&po_a, &po_b, poc, noc, cor);

    BOOST_CHECK(po_a.get_collision_normal() == noc);
    BOOST_CHECK(po_a.get_angular_weight() == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 21.0f, result_tolerance);

    BOOST_CHECK(po_b.get_collision_normal() == noc);
    BOOST_CHECK(po_b.get_angular_weight() == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK_CLOSE(po_b.get_impulse(), -21.0f, result_tolerance);
}

/* Test instantaneous frictionless collide with off center impact (i.e. imparting some rotation) */
BOOST_AUTO_TEST_CASE( instantaneous_frictionless_off_center_collide_test )
{
    const float cor = 0.81f;
    const point_t<> poc(-1.0f, 3.6f, 0.0f);
    const point_t<> noc( 0.7071067812f, 0.7071067812f, 0.0f);
    mock_physics_object po_a(new inertia_tensor(new float[6]{ 9.0f, 4.6f, 5.6f, 4.1f, -0.5f, 4.6f }, point_t<>( 1.0f, 0.0f, -2.3f), 9.0f), point_t<>(-5.0f, 0.0f, 0.0f), point_t<>(0.0f, 0.0f, 0.0f));
    mock_physics_object po_b(new inertia_tensor(new float[6]{ 2.0f, 1.2f, 8.2f, 0.0f,  0.0f, 0.0f }, point_t<>(-4.3f, 5.7f,  0.0f), 5.5f), point_t<>( 7.0f, 0.0f, 0.0f), point_t<>(0.0f, 0.0f, 0.0f));
    instantaneous_frictionless_collide(&po_a, &po_b, poc, noc, cor);

    BOOST_CHECK(po_a.get_collision_normal() == noc);
    BOOST_CHECK(fabs(magnitude(po_a.get_angular_weight() - point_t<>(0.484855f, -0.200698f, -1.1233f))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 2.84199667f, result_tolerance);

    BOOST_CHECK(po_b.get_collision_normal() == noc);
    BOOST_CHECK(fabs(magnitude(po_b.get_angular_weight() - point_t<>(0.0f, 0.0f, 0.465656f))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_b.get_impulse(), -2.84199667f, result_tolerance);
}

BOOST_AUTO_TEST_CASE( instantaneous_frictionless_off_center_collide_infinite_mass_a_test )
{
    const float cor = 0.57f;
    const point_t<> poc(-2.0f, 0.0f, 0.0f);
    const point_t<> noc(0.0f, -0.7071067812f, -0.7071067812f);
    mock_physics_object po_a(new inertia_tensor(new float[6]{ inf, inf, inf, inf, inf, inf }, point_t<>( 1.0f, 0.0f, -2.3f), inf), point_t<>(0.0f, 10.7f, 1.3f), point_t<>(0.0f, 0.0f, 0.0f));
    mock_physics_object po_b(new inertia_tensor(new float[6]{ 2.0f, 1.2f, 8.2f, -0.2f, 0.7f, -3.5f }, point_t<>(-4.3f, 5.7f,  0.0f), 5.5f), point_t<>(0.0f, -4.5f, -22.3f), point_t<>(0.0f, 0.0f, 0.0f));
    instantaneous_frictionless_collide(&po_a, &po_b, poc, noc, cor);

    BOOST_CHECK(po_a.get_collision_normal() == noc);
    BOOST_CHECK(po_a.get_angular_weight() == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 1.82475f, result_tolerance);

    BOOST_CHECK(po_b.get_collision_normal() == noc);
    BOOST_CHECK(fabs(magnitude(po_b.get_angular_weight() - point_t<>(6.39831f, 0.99378f, 2.44782f))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_b.get_impulse(), -1.82475f, result_tolerance);
}

BOOST_AUTO_TEST_CASE( instantaneous_frictionless_off_center_collide_infinite_mass_b_test )
{
    const float cor = 0.35f;
    const point_t<> poc(-2.0f, 0.0f, -1.2f);
    const point_t<> noc(-0.4082482905f, -0.8164965809f, 0.4082482905f);
    mock_physics_object po_a(new inertia_tensor(new float[6]{ 1.0f, 1.1f, 1.2f, 0.0f, 0.0f, 0.0f }, point_t<>( 1.0f, 0.0f, -2.3f), 1.9f), point_t<>(0.0f, 10.7f, 1.3f), point_t<>(0.0f, 0.0f, 0.0f));
    mock_physics_object po_b(new inertia_tensor(new float[6]{ inf, inf, inf, inf, inf, inf }, point_t<>(-4.3f, 5.7f,  0.0f), inf), point_t<>(0.0f, 0.0f, 0.0f), point_t<>(0.0f, 0.0f, 0.0f));
    instantaneous_frictionless_collide(&po_a, &po_b, poc, noc, cor);

    BOOST_CHECK(po_a.get_collision_normal() == noc);
    BOOST_CHECK(fabs(magnitude(po_a.get_angular_weight() - point_t<>(0.898146f, 0.705156f, 2.04124f))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 1.61016f, result_tolerance);

    BOOST_CHECK(po_b.get_collision_normal() == noc);
    BOOST_CHECK(po_b.get_angular_weight() == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK_CLOSE(po_b.get_impulse(), -1.61016f, result_tolerance);
}

/* Test instantaneous frictionless collide with rotation */
BOOST_AUTO_TEST_CASE( instantaneous_frictionless_rotating_collide_test )
{
    const float cor = 0.1f;
    const point_t<> poc(-1.0f, 3.6f, 0.0f);
    const point_t<> noc( 0.7071067812f, 0.7071067812f, 0.0f);
    mock_physics_object po_a(new inertia_tensor(new float[6]{ 9.0f, 4.6f, 5.6f, 4.1f, -0.5f, 4.6f }, point_t<>( 1.0f, 0.0f, -2.3f), 9.0f), point_t<>(-5.0f, 0.0f, 0.0f), point_t<>( 0.0f, 3.3f, 0.0f));
    mock_physics_object po_b(new inertia_tensor(new float[6]{ 2.0f, 1.2f, 8.2f, -0.2f, 0.7f, -3.5f }, point_t<>(-4.3f, 5.7f,  0.0f), 5.5f), point_t<>( 7.0f, 0.0f, 0.0f), point_t<>(-1.5f, 0.0f, 0.0f));
    instantaneous_frictionless_collide(&po_a, &po_b, poc, noc, cor);

    BOOST_CHECK(po_a.get_collision_normal() == noc);
    BOOST_CHECK(fabs(magnitude(po_a.get_angular_weight() - point_t<>(0.484855f, -0.200698f, -1.1233f))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 0.311196f, result_tolerance);

    BOOST_CHECK(po_b.get_collision_normal() == noc);
    BOOST_CHECK(fabs(magnitude(po_b.get_angular_weight() - point_t<>(3.33246f, -0.574562f, 1.9371f))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_b.get_impulse(), -0.311196f, result_tolerance);
}

BOOST_AUTO_TEST_CASE( instantaneous_frictionless_rotating_collide_infinite_mass_a_test )
{
    const float cor = 0.46f;
    const point_t<> poc(-2.0f, 0.0f, 0.0f);
    const point_t<> noc(0.0f, -0.7071067812f, -0.7071067812f);
    mock_physics_object po_a(new inertia_tensor(new float[6]{ inf, inf, inf, inf, inf, inf }, point_t<>( 1.0f, 0.0f, -2.3f), inf), point_t<>(0.0f, 10.7f, 1.3f), point_t<>(-7.9f, 0.0f, 4.0f));
    mock_physics_object po_b(new inertia_tensor(new float[6]{ 2.0f, 1.2f, 8.2f, -0.2f, 0.7f, -3.5f }, point_t<>(-4.3f, 5.7f,  0.0f), 5.5f), point_t<>(0.0f, -4.5f, -22.3f), point_t<>(0.0f, 10.0f, 0.0f));
    instantaneous_frictionless_collide(&po_a, &po_b, poc, noc, cor);

    BOOST_CHECK(po_a.get_collision_normal() == noc);
    BOOST_CHECK(po_a.get_angular_weight() == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 2.97264f, result_tolerance);

    BOOST_CHECK(po_b.get_collision_normal() == noc);
    BOOST_CHECK(fabs(magnitude(po_b.get_angular_weight() - point_t<>(6.39831f, 0.99378f, 2.44782f))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_b.get_impulse(), -2.97264f, result_tolerance);
}

BOOST_AUTO_TEST_CASE( instantaneous_frictionless_rotating_collide_infinite_mass_b_test )
{
    const float cor = 0.85f;
    const point_t<> poc(-2.0f, 0.0f, -1.2f);
    const point_t<> noc(0.4082482905f, 0.8164965809f, -0.4082482905f);
    mock_physics_object po_a(new inertia_tensor(new float[6]{ 1.0f, 1.1f, 1.2f, 0.0f, 0.0f, 0.0f }, point_t<>( 1.0f, 0.0f, -2.3f), 1.9f), point_t<>(-15.0f, 10.7f, 1.3f), point_t<>(-7.9f, 11.0f, 4.0f));
    mock_physics_object po_b(new inertia_tensor(new float[6]{ inf, inf, inf, inf, inf, inf }, point_t<>(-4.3f, 5.7f,  0.0f), inf), point_t<>(-5.6f, -4.5f, -22.3f), point_t<>(-1.9f, 10.0f, 14.0f));
    instantaneous_frictionless_collide(&po_a, &po_b, poc, noc, cor);

    BOOST_CHECK(po_a.get_collision_normal() == noc);
    BOOST_CHECK(fabs(magnitude(po_a.get_angular_weight() - point_t<>(-0.898146f, -0.705156f, -2.04124f))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 18.6544f, result_tolerance);

    BOOST_CHECK(po_b.get_collision_normal() == noc);
    BOOST_CHECK(po_b.get_angular_weight() == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK_CLOSE(po_b.get_impulse(), -18.6544f, result_tolerance);
}

/* Collider with friction */
/* Test instantaneous collide with no rotation */
BOOST_AUTO_TEST_CASE( instantaneous_0_mu_rotationless_collide_test )
{
    const float mu = 0.0f;
    const float cor = 1.0f;
    const point_t<> poc(0.0f, 0.0f, 0.0f);
    const point_t<> noc(1.0f, 0.0f, 0.0f);
    mock_physics_object po_a(new inertia_tensor(new float[6]{ 1.0f, 1.1f, 1.2f, 0.0f, 0.0f, 0.0f }, point_t<>( 1.0f, 0.0f, 0.0f), 1.0f), point_t<>(-5.0f, 0.0f, 0.0f), point_t<>(0.0f, 0.0f, 0.0f));
    mock_physics_object po_b(new inertia_tensor(new float[6]{ 2.9f, 2.8f, 2.7f, 0.0f, 0.0f, 0.0f }, point_t<>(-4.3f, 0.0f, 0.0f), 2.5f), point_t<>( 7.0f, 0.0f, 0.0f), point_t<>(0.0f, 0.0f, 0.0f));
    instantaneous_collide(&po_a, &po_b, poc, noc, cor, mu);

    /* Checks */
    BOOST_REQUIRE(po_a.number_of_impulses() == 2);
    BOOST_REQUIRE(po_b.number_of_impulses() == 2);

    BOOST_CHECK(po_a.get_collision_normal() == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(po_a.get_angular_weight() == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 0.0f, result_tolerance);

    BOOST_CHECK(po_a.get_collision_normal() == noc);
    BOOST_CHECK(po_a.get_angular_weight() == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 17.1429f, result_tolerance);

    BOOST_CHECK(po_b.get_collision_normal() == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(po_b.get_angular_weight() == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK_CLOSE(po_b.get_impulse(), 0.0f, result_tolerance);

    BOOST_CHECK(po_b.get_collision_normal() == noc);
    BOOST_CHECK(po_b.get_angular_weight() == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK_CLOSE(po_b.get_impulse(), -17.1429f, result_tolerance);
}

BOOST_AUTO_TEST_CASE( instantaneous_0_mu_rotationless_collide_infinite_mass_a_test )
{
    const float mu = 0.0f;
    const float cor = 0.5f;
    const point_t<> poc(0.0f, 0.0f, 0.0f);
    const point_t<> noc(1.0f, 0.0f, 0.0f);
    mock_physics_object po_a(new inertia_tensor(new float[6]{ inf, inf, inf, inf, inf, inf }, point_t<>( 1.0f, 0.0f, 0.0f), inf), point_t<>(-5.0f, 0.0f, 0.0f), point_t<>(0.0f, 0.0f, 0.0f));
    mock_physics_object po_b(new inertia_tensor(new float[6]{ 2.9f, 2.8f, 2.7f, 0.0f, 0.0f, 0.0f }, point_t<>(-4.3f, 0.0f, 0.0f), 2.5f), point_t<>( 7.0f, 0.0f, 0.0f), point_t<>(0.0f, 0.0f, 0.0f));
    instantaneous_collide(&po_a, &po_b, poc, noc, cor, mu);

    /* Checks */
    BOOST_REQUIRE(po_a.number_of_impulses() == 2);
    BOOST_REQUIRE(po_b.number_of_impulses() == 2);

    BOOST_CHECK(po_a.get_collision_normal() == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(po_a.get_angular_weight() == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 0.0f, result_tolerance);

    BOOST_CHECK(po_a.get_collision_normal() == noc);
    BOOST_CHECK(po_a.get_angular_weight() == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 45.0f, result_tolerance);

    BOOST_CHECK(po_b.get_collision_normal() == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(po_b.get_angular_weight() == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK_CLOSE(po_b.get_impulse(), 0.0f, result_tolerance);

    BOOST_CHECK(po_b.get_collision_normal() == noc);
    BOOST_CHECK(po_b.get_angular_weight() == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK_CLOSE(po_b.get_impulse(), -45.0f, result_tolerance);
}

BOOST_AUTO_TEST_CASE( instantaneous_0_mu_rotationless_collide_infinite_mass_b_test )
{
    const float mu = 0.0f;
    const float cor = 0.75f;
    const point_t<> poc(0.0f, 0.0f, 0.0f);
    const point_t<> noc(0.0f, 1.0f, 0.0f);
    mock_physics_object po_a(new inertia_tensor(new float[6]{ 1.0f, 1.1f, 1.2f, 0.0f, 0.0f, 0.0f }, point_t<>(0.0f,  1.0f, 0.0f), 1.0f), point_t<>(0.0f, -5.0f, 0.0f), point_t<>(0.0f, 0.0f, 0.0f));
    mock_physics_object po_b(new inertia_tensor(new float[6]{ inf, inf, inf, inf, inf, inf }, point_t<>(0.0f, -4.3f, 0.0f), inf), point_t<>(0.0f, 7.0f, 0.0f), point_t<>(0.0f, 0.0f, 0.0f));
    instantaneous_collide(&po_a, &po_b, poc, noc, cor, mu);

    /* Checks */
    BOOST_REQUIRE(po_a.number_of_impulses() == 2);
    BOOST_REQUIRE(po_b.number_of_impulses() == 2);

    BOOST_CHECK(po_a.get_collision_normal() == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(po_a.get_angular_weight() == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 0.0f, result_tolerance);

    BOOST_CHECK(po_a.get_collision_normal() == noc);
    BOOST_CHECK(po_a.get_angular_weight() == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 21.0f, result_tolerance);

    BOOST_CHECK(po_b.get_collision_normal() == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(po_b.get_angular_weight() == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK_CLOSE(po_b.get_impulse(), 0.0f, result_tolerance);

    BOOST_CHECK(po_b.get_collision_normal() == noc);
    BOOST_CHECK(po_b.get_angular_weight() == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK_CLOSE(po_b.get_impulse(), -21.0f, result_tolerance);
}

/* Test instantaneous collide with off center impact (i.e. imparting some rotation) */
BOOST_AUTO_TEST_CASE( instantaneous_0_mu_off_center_collide_test )
{
    const float mu = 0.0f;
    const float cor = 0.81f;
    const point_t<> poc(-1.0f, 3.6f, 0.0f);
    const point_t<> noc( 0.7071067812f, 0.7071067812f, 0.0f);
    mock_physics_object po_a(new inertia_tensor(new float[6]{ 9.0f, 4.6f, 5.6f, 4.1f, -0.5f, 4.6f }, point_t<>( 1.0f, 0.0f, -2.3f), 9.0f), point_t<>(-5.0f, 0.0f, 0.0f), point_t<>(0.0f, 0.0f, 0.0f));
    mock_physics_object po_b(new inertia_tensor(new float[6]{ 2.0f, 1.2f, 8.2f, 0.0f,  0.0f, 0.0f }, point_t<>(-4.3f, 5.7f,  0.0f), 5.5f), point_t<>( 7.0f, 0.0f, 0.0f), point_t<>(0.0f, 0.0f, 0.0f));
    instantaneous_collide(&po_a, &po_b, poc, noc, cor, mu);

    /* Checks */
    BOOST_REQUIRE(po_a.number_of_impulses() == 2);
    BOOST_REQUIRE(po_b.number_of_impulses() == 2);

    BOOST_CHECK(fabs(magnitude(po_a.get_collision_normal() - point_t<>(0.707107f, -0.707107f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_a.get_angular_weight() - point_t<>(1.38466f, -1.03624f, -1.43195f))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 0.0f, result_tolerance);

    BOOST_CHECK(po_a.get_collision_normal() == noc);
    BOOST_CHECK(fabs(magnitude(po_a.get_angular_weight() - point_t<>(0.484855f, -0.200698f, -1.1233f))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 2.84199667f, result_tolerance);

    BOOST_CHECK(fabs(magnitude(po_b.get_collision_normal() - point_t<>(0.707107f, -0.707107f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b.get_angular_weight() - point_t<>(0.0f, 0.0f, -0.103479f))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_b.get_impulse(), 0.0f, result_tolerance);

    BOOST_CHECK(po_b.get_collision_normal() == noc);
    BOOST_CHECK(fabs(magnitude(po_b.get_angular_weight() - point_t<>(0.0f, 0.0f, 0.465656f))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_b.get_impulse(), -2.84199667f, result_tolerance);
}

BOOST_AUTO_TEST_CASE( instantaneous_0_mu_off_center_collide_infinite_mass_a_test )
{
    const float mu = 0.0f;
    const float cor = 0.57f;
    const point_t<> poc(-2.0f, 0.0f, 0.0f);
    const point_t<> noc(0.0f, -0.7071067812f, -0.7071067812f);
    mock_physics_object po_a(new inertia_tensor(new float[6]{ inf, inf, inf, inf, inf, inf }, point_t<>( 1.0f, 0.0f, -2.3f), inf), point_t<>(0.0f, 10.7f, 1.3f), point_t<>(0.0f, 0.0f, 0.0f));
    mock_physics_object po_b(new inertia_tensor(new float[6]{ 2.0f, 1.2f, 8.2f, -0.2f, 0.7f, -3.5f }, point_t<>(-4.3f, 5.7f,  0.0f), 5.5f), point_t<>(0.0f, -4.5f, -22.3f), point_t<>(0.0f, 0.0f, 0.0f));
    instantaneous_collide(&po_a, &po_b, poc, noc, cor, mu);

    /* Checks */
    BOOST_REQUIRE(po_a.number_of_impulses() == 3);
    BOOST_REQUIRE(po_b.number_of_impulses() == 3);

    BOOST_CHECK(fabs(magnitude(po_a.get_collision_normal() - point_t<>(0.0f, 0.707107f, -0.707107f))) < result_tolerance);
    BOOST_CHECK(po_a.get_angular_weight() == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 0.0f, result_tolerance);

    BOOST_CHECK(po_a.get_collision_normal() == noc);
    BOOST_CHECK(po_a.get_angular_weight() == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 0.18925f, result_tolerance);

    BOOST_CHECK(po_a.get_collision_normal() == noc);
    BOOST_CHECK(po_a.get_angular_weight() == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 1.6355f, result_tolerance);

    BOOST_CHECK(fabs(magnitude(po_b.get_collision_normal() - point_t<>(0.0f, 0.707107f, -0.707107f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b.get_angular_weight() - point_t<>(9.23708f, 0.504338f, 4.09794f))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_b.get_impulse(), 0.0f, result_tolerance);

    BOOST_CHECK(po_b.get_collision_normal() == noc);
    BOOST_CHECK(fabs(magnitude(po_b.get_angular_weight() - point_t<>(6.39831f, 0.99378f, 2.44782f))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_b.get_impulse(), -0.18925f, result_tolerance);

    BOOST_CHECK(po_b.get_collision_normal() == noc);
    BOOST_CHECK(fabs(magnitude(po_b.get_angular_weight() - point_t<>(6.39831f, 0.99378f, 2.44782f))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_b.get_impulse(), -1.6355f, result_tolerance);
}

BOOST_AUTO_TEST_CASE( instantaneous_0_mu_off_center_collide_infinite_mass_b_test )
{
    const float mu = 0.0f;
    const float cor = 0.35f;
    const point_t<> poc(-2.0f, 0.0f, -1.2f);
    const point_t<> noc(-0.4082482905f, -0.8164965809f, 0.4082482905f);
    mock_physics_object po_a(new inertia_tensor(new float[6]{ 1.0f, 1.1f, 1.2f, 0.0f, 0.0f, 0.0f }, point_t<>( 1.0f, 0.0f, -2.3f), 1.9f), point_t<>(0.0f, 10.7f, 1.3f), point_t<>(0.0f, 0.0f, 0.0f));
    mock_physics_object po_b(new inertia_tensor(new float[6]{ inf, inf, inf, inf, inf, inf }, point_t<>(-4.3f, 5.7f,  0.0f), inf), point_t<>(0.0f, 0.0f, 0.0f), point_t<>(0.0f, 0.0f, 0.0f));
    instantaneous_collide(&po_a, &po_b, poc, noc, cor, mu);

    /* Checks */
    BOOST_REQUIRE(po_a.number_of_impulses() == 2);
    BOOST_REQUIRE(po_b.number_of_impulses() == 2);

    BOOST_CHECK(fabs(magnitude(po_a.get_collision_normal() - point_t<>(0.47933f, -0.572335f, -0.665339f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_a.get_angular_weight() - point_t<>(0.629568f, -1.33523f, 1.43084f))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 0.0f, result_tolerance);

    BOOST_CHECK(po_a.get_collision_normal() == noc);
    BOOST_CHECK(fabs(magnitude(po_a.get_angular_weight() - point_t<>(0.898146f, 0.705156f, 2.04124f))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 1.61016f, result_tolerance);

    BOOST_CHECK(fabs(magnitude(po_b.get_collision_normal() - point_t<>(0.47933f, -0.572335f, -0.665339f))) < result_tolerance);
    BOOST_CHECK(po_b.get_angular_weight() == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK_CLOSE(po_b.get_impulse(), 0.0f, result_tolerance);

    BOOST_CHECK(po_b.get_collision_normal() == noc);
    BOOST_CHECK(po_b.get_angular_weight() == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK_CLOSE(po_b.get_impulse(), -1.61016f, result_tolerance);
}

/* Test instantaneous collide with rotation */
BOOST_AUTO_TEST_CASE( instantaneous_0_mu_rotating_collide_test )
{
    const float mu = 0.0f;
    const float cor = 0.1f;
    const point_t<> poc(-1.0f, 3.6f, 0.0f);
    const point_t<> noc( 0.7071067812f, 0.7071067812f, 0.0f);
    mock_physics_object po_a(new inertia_tensor(new float[6]{ 9.0f, 4.6f, 5.6f, 4.1f, -0.5f, 4.6f }, point_t<>( 1.0f, 0.0f, -2.3f), 9.0f), point_t<>(-5.0f, 0.0f, 0.0f), point_t<>( 0.0f, 3.3f, 0.0f));
    mock_physics_object po_b(new inertia_tensor(new float[6]{ 2.0f, 1.2f, 8.2f, -0.2f, 0.7f, -3.5f }, point_t<>(-4.3f, 5.7f,  0.0f), 5.5f), point_t<>( 7.0f, 0.0f, 0.0f), point_t<>(-1.5f, 0.0f, 0.0f));
    instantaneous_collide(&po_a, &po_b, poc, noc, cor, mu);

    /* Checks */
    BOOST_REQUIRE(po_a.number_of_impulses() == 2);
    BOOST_REQUIRE(po_b.number_of_impulses() == 2);

    BOOST_CHECK(fabs(magnitude(po_a.get_collision_normal() - point_t<>(0.474149f, -0.474149f, -0.741866f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_a.get_angular_weight() - point_t<>(-0.663117f, 0.555435f, 0.458824f))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 0.0f, result_tolerance);

    BOOST_CHECK(po_a.get_collision_normal() == noc);
    BOOST_CHECK(fabs(magnitude(po_a.get_angular_weight() - point_t<>(0.484855f, -0.200698f, -1.1233f))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 0.311196f, result_tolerance);

    BOOST_CHECK(fabs(magnitude(po_b.get_collision_normal() - point_t<>(0.474149f, -0.474149f, -0.741866f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b.get_angular_weight() - point_t<>(2.2084f, 1.99833f, 0.702633f))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_b.get_impulse(), 0.0f, result_tolerance);

    BOOST_CHECK(po_b.get_collision_normal() == noc);
    BOOST_CHECK(fabs(magnitude(po_b.get_angular_weight() - point_t<>(3.33246f, -0.574562f, 1.9371f))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_b.get_impulse(), -0.311196f, result_tolerance);
}

BOOST_AUTO_TEST_CASE( instantaneous_0_mu_rotating_collide_infinite_mass_a_test )
{
    const float mu = 0.0f;
    const float cor = 0.46f;
    const point_t<> poc(-2.0f, 0.0f, 0.0f);
    const point_t<> noc(0.0f, -0.7071067812f, -0.7071067812f);
    mock_physics_object po_a(new inertia_tensor(new float[6]{ inf, inf, inf, inf, inf, inf }, point_t<>( 1.0f, 0.0f, -2.3f), inf), point_t<>(0.0f, 10.7f, 1.3f), point_t<>(-7.9f, 0.0f, 4.0f));
    mock_physics_object po_b(new inertia_tensor(new float[6]{ 2.0f, 1.2f, 8.2f, -0.2f, 0.7f, -3.5f }, point_t<>(-4.3f, 5.7f,  0.0f), 5.5f), point_t<>(0.0f, -4.5f, -22.3f), point_t<>(0.0f, 10.0f, 0.0f));
    instantaneous_collide(&po_a, &po_b, poc, noc, cor, mu);

    /* Checks */
    BOOST_REQUIRE(po_a.number_of_impulses() == 3);
    BOOST_REQUIRE(po_b.number_of_impulses() == 3);

    BOOST_CHECK(fabs(magnitude(po_a.get_collision_normal() - point_t<>(0.0f, 0.707107f, -0.707107f))) < result_tolerance);
    BOOST_CHECK(po_a.get_angular_weight() == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 0.0f, result_tolerance);

    BOOST_CHECK(po_a.get_collision_normal() == noc);
    BOOST_CHECK(po_a.get_angular_weight() == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 0.568422f, result_tolerance);

    BOOST_CHECK(po_a.get_collision_normal() == noc);
    BOOST_CHECK(po_a.get_angular_weight() == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 2.40422f, result_tolerance);

    BOOST_CHECK(fabs(magnitude(po_b.get_collision_normal() - point_t<>(0.0f, 0.707107f, -0.707107f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b.get_angular_weight() - point_t<>(9.23708f, 0.504338f, 4.09794f))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_b.get_impulse(), 0.0f, result_tolerance);

    BOOST_CHECK(po_b.get_collision_normal() == noc);
    BOOST_CHECK(fabs(magnitude(po_b.get_angular_weight() - point_t<>(6.39831f, 0.99378f, 2.44782f))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_b.get_impulse(), -0.568422f, result_tolerance);

    BOOST_CHECK(po_b.get_collision_normal() == noc);
    BOOST_CHECK(fabs(magnitude(po_b.get_angular_weight() - point_t<>(6.39831f, 0.99378f, 2.44782f))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_b.get_impulse(), -2.40422f, result_tolerance);
}

BOOST_AUTO_TEST_CASE( instantaneous_0_mu_rotating_collide_infinite_mass_b_test )
{
    const float mu = 0.0f;
    const float cor = 0.85f;
    const point_t<> poc(-2.0f, 0.0f, -1.2f);
    const point_t<> noc(0.4082482905f, 0.8164965809f, -0.4082482905f);
    mock_physics_object po_a(new inertia_tensor(new float[6]{ 1.0f, 1.1f, 1.2f, 0.0f, 0.0f, 0.0f }, point_t<>( 1.0f, 0.0f, -2.3f), 1.9f), point_t<>(-15.0f, 10.7f, 1.3f), point_t<>(-7.9f, 11.0f, 4.0f));
    mock_physics_object po_b(new inertia_tensor(new float[6]{ inf, inf, inf, inf, inf, inf }, point_t<>(-4.3f, 5.7f,  0.0f), inf), point_t<>(-5.6f, -4.5f, -22.3f), point_t<>(-1.9f, 10.0f, 14.0f));
    instantaneous_collide(&po_a, &po_b, poc, noc, cor, mu);

    /* Checks */
    BOOST_REQUIRE(po_a.number_of_impulses() == 2);
    BOOST_REQUIRE(po_b.number_of_impulses() == 2);

    BOOST_CHECK(fabs(magnitude(po_a.get_collision_normal() - point_t<>(0.5495f, -0.576916f, -0.604333f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_a.get_angular_weight() - point_t<>(0.634608f, -1.09868f, 1.44229f))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 0.0f, result_tolerance);

    BOOST_CHECK(po_a.get_collision_normal() == noc);
    BOOST_CHECK(fabs(magnitude(po_a.get_angular_weight() - point_t<>(-0.898146f, -0.705156f, -2.04124f))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 18.6544f, result_tolerance);

    BOOST_CHECK(fabs(magnitude(po_b.get_collision_normal() - point_t<>(0.5495f, -0.576916f, -0.604333f))) < result_tolerance);
    BOOST_CHECK(po_b.get_angular_weight() == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK_CLOSE(po_b.get_impulse(), 0.0f, result_tolerance);

    BOOST_CHECK(po_b.get_collision_normal() == noc);
    BOOST_CHECK(po_b.get_angular_weight() == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK_CLOSE(po_b.get_impulse(), -18.6544f, result_tolerance);
}

BOOST_AUTO_TEST_CASE( instantaneous_no_tangent_rotationless_collide_test )
{
    const float mu = 0.75f;
    const float cor = 1.0f;
    const point_t<> poc(0.0f, 0.0f, 0.0f);
    const point_t<> noc(1.0f, 0.0f, 0.0f);
    mock_physics_object po_a(new inertia_tensor(new float[6]{ 1.0f, 1.1f, 1.2f, 0.0f, 0.0f, 0.0f }, point_t<>( 1.0f, 0.0f, 0.0f), 1.0f), point_t<>(-5.0f, 0.0f, 0.0f), point_t<>(0.0f, 0.0f, 0.0f));
    mock_physics_object po_b(new inertia_tensor(new float[6]{ 2.9f, 2.8f, 2.7f, 0.0f, 0.0f, 0.0f }, point_t<>(-4.3f, 0.0f, 0.0f), 2.5f), point_t<>( 7.0f, 0.0f, 0.0f), point_t<>(0.0f, 0.0f, 0.0f));
    instantaneous_collide(&po_a, &po_b, poc, noc, cor, mu);

    /* Checks */
    BOOST_REQUIRE(po_a.number_of_impulses() == 3);
    BOOST_REQUIRE(po_b.number_of_impulses() == 3);

    BOOST_CHECK(po_a.get_collision_normal() == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(po_a.get_angular_weight() == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 0.0f, result_tolerance);

    BOOST_CHECK(po_a.get_collision_normal() == noc);
    BOOST_CHECK(po_a.get_angular_weight() == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 0.0f, result_tolerance);

    BOOST_CHECK(po_a.get_collision_normal() == noc);
    BOOST_CHECK(po_a.get_angular_weight() == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 17.1429f, result_tolerance);

    BOOST_CHECK(po_b.get_collision_normal() == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(po_b.get_angular_weight() == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK_CLOSE(po_b.get_impulse(), 0.0f, result_tolerance);

    BOOST_CHECK(po_b.get_collision_normal() == noc);
    BOOST_CHECK(po_b.get_angular_weight() == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK_CLOSE(po_b.get_impulse(), 0.0f, result_tolerance);

    BOOST_CHECK(po_b.get_collision_normal() == noc);
    BOOST_CHECK(po_b.get_angular_weight() == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK_CLOSE(po_b.get_impulse(), -17.1429f, result_tolerance);
}

BOOST_AUTO_TEST_CASE( instantaneous_only_tangent_rotationless_collide_test )
{
    const float mu = 0.75f;
    const float cor = 1.0f;
    const point_t<> poc(0.0f, 0.0f, 0.0f);
    const point_t<> noc(1.0f, 0.0f, 0.0f);
    mock_physics_object po_a(new inertia_tensor(new float[6]{ 1.0f, 1.1f, 1.2f, 0.0f, 0.0f, 0.0f }, point_t<>( 1.0f, 0.0f, 0.0f), 1.0f), point_t<>(0.0f, -5.0f, 0.0f), point_t<>(0.0f, 0.0f, 0.0f));
    mock_physics_object po_b(new inertia_tensor(new float[6]{ 2.9f, 2.8f, 2.7f, 0.0f, 0.0f, 0.0f }, point_t<>(-4.3f, 0.0f, 0.0f), 2.5f), point_t<>(0.0f,  7.0f, 0.0f), point_t<>(0.0f, 0.0f, 0.0f));
    instantaneous_collide(&po_a, &po_b, poc, noc, cor, mu);

    /* Checks */
    BOOST_REQUIRE(po_a.number_of_impulses() == 2);
    BOOST_REQUIRE(po_b.number_of_impulses() == 2);

    BOOST_CHECK(po_a.get_collision_normal() == point_t<>(0.0f, 1.0f, 0.0f));
    BOOST_CHECK(std::fabs(magnitude(po_a.get_angular_weight() - point_t<>(0.0f, 0.0f, -0.833333f))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 0.0f, result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po_a.get_collision_normal() - noc)) < result_tolerance);
    BOOST_CHECK(po_a.get_angular_weight() == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 0.0f, result_tolerance);

    BOOST_CHECK(po_b.get_collision_normal() == point_t<>(0.0f, 1.0f, 0.0f));
    BOOST_CHECK(std::fabs(magnitude(po_b.get_angular_weight() - point_t<>(0.0f, 0.0f, 1.59259f))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_b.get_impulse(), 0.0f, result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po_b.get_collision_normal() - noc)) < result_tolerance);
    BOOST_CHECK(po_b.get_angular_weight() == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK_CLOSE(po_b.get_impulse(), 0.0f, result_tolerance);
}

BOOST_AUTO_TEST_CASE( instantaneous_mu_tangent_lt_reaction_rotationless_collide_test )
{
    const float mu = 0.75f;
    const float cor = 1.0f;
    const point_t<> poc(0.0f, 0.0f, 0.0f);
    const point_t<> noc(1.0f, 0.0f, 0.0f);
    mock_physics_object po_a(new inertia_tensor(new float[6]{ 1.0f, 1.1f, 1.2f, 0.0f, 0.0f, 0.0f }, point_t<>( 1.0f, 0.0f, 0.0f), 1.0f), point_t<>(-5.0f, -5.0f, 0.0f), point_t<>(0.0f, 0.0f, 0.0f));
    mock_physics_object po_b(new inertia_tensor(new float[6]{ 2.9f, 2.8f, 2.7f, 0.0f, 0.0f, 0.0f }, point_t<>(-4.3f, 0.0f, 0.0f), 2.5f), point_t<>( 7.0f,  7.0f, 0.0f), point_t<>(0.0f, 0.0f, 0.0f));
    instantaneous_collide(&po_a, &po_b, poc, noc, cor, mu);

    /* Checks */
    BOOST_REQUIRE(po_a.number_of_impulses() == 3);
    BOOST_REQUIRE(po_b.number_of_impulses() == 3);

    BOOST_CHECK(po_a.get_collision_normal() == point_t<>(0.0f, 1.0f, 0.0f));
    BOOST_CHECK(std::fabs(magnitude(po_a.get_angular_weight() - point_t<>(0.0f, 0.0f, -0.833333f))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 1.32137f, result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po_a.get_collision_normal() - noc)) < result_tolerance);
    BOOST_CHECK(po_a.get_angular_weight() == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 1.76182f, result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po_a.get_collision_normal() - noc)) < result_tolerance);
    BOOST_CHECK(po_a.get_angular_weight() == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 15.381f, result_tolerance);

    BOOST_CHECK(po_b.get_collision_normal() == point_t<>(0.0f, 1.0f, 0.0f));
    BOOST_CHECK(std::fabs(magnitude(po_b.get_angular_weight() - point_t<>(0.0f, 0.0f, 1.59259f))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_b.get_impulse(), -1.32137f, result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po_b.get_collision_normal() - noc)) < result_tolerance);
    BOOST_CHECK(po_b.get_angular_weight() == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK_CLOSE(po_b.get_impulse(), -1.76182f, result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po_b.get_collision_normal() - noc)) < result_tolerance);
    BOOST_CHECK(po_b.get_angular_weight() == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK_CLOSE(po_b.get_impulse(), -15.381f, result_tolerance);
}

BOOST_AUTO_TEST_CASE( instantaneous_mu_tangent_gt_reaction_rotationless_collide_test )
{
    const float mu = 0.5f;
    const float cor = 1.0f;
    const point_t<> poc(0.0f, 0.0f, 0.0f);
    const point_t<> noc(1.0f, 0.0f, 0.0f);
    mock_physics_object po_a(new inertia_tensor(new float[6]{ 1.0f, 1.1f, 1.2f, 0.0f, 0.0f, 0.0f }, point_t<>( 1.0f, 0.0f, 0.0f), 1.0f), point_t<>(-5.0f, -50.0f, 0.0f), point_t<>(0.0f, 0.0f, 0.0f));
    mock_physics_object po_b(new inertia_tensor(new float[6]{ 2.9f, 2.8f, 2.7f, 0.0f, 0.0f, 0.0f }, point_t<>(-4.3f, 0.0f, 0.0f), 2.5f), point_t<>( 7.0f,  70.0f, 0.0f), point_t<>(0.0f, 0.0f, 0.0f));
    instantaneous_collide(&po_a, &po_b, poc, noc, cor, mu);

    /* Checks */
    BOOST_REQUIRE(po_a.number_of_impulses() == 2);
    BOOST_REQUIRE(po_b.number_of_impulses() == 2);

    BOOST_CHECK(po_a.get_collision_normal() == point_t<>(0.0f, 1.0f, 0.0f));
    BOOST_CHECK(std::fabs(magnitude(po_a.get_angular_weight() - point_t<>(0.0f, 0.0f, -0.833333f))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 8.5714f, result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po_a.get_collision_normal() - noc)) < result_tolerance);
    BOOST_CHECK(po_a.get_angular_weight() == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 17.1428f, result_tolerance);

    BOOST_CHECK(po_b.get_collision_normal() == point_t<>(0.0f, 1.0f, 0.0f));
    BOOST_CHECK(std::fabs(magnitude(po_b.get_angular_weight() - point_t<>(0.0f, 0.0f, 1.59259f))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_b.get_impulse(), -8.5714f, result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po_b.get_collision_normal() - noc)) < result_tolerance);
    BOOST_CHECK(po_b.get_angular_weight() == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK_CLOSE(po_b.get_impulse(), -17.1428f, result_tolerance);
}

BOOST_AUTO_TEST_CASE( instantaneous_rotationless_collide_infinite_mass_a_test )
{
    const float mu = 0.5f;
    const float cor = 0.5f;
    const point_t<> poc(0.0f, 0.0f, 0.0f);
    const point_t<> noc(1.0f, 0.0f, 0.0f);
    mock_physics_object po_a(new inertia_tensor(new float[6]{ inf, inf, inf, inf, inf, inf }, point_t<>( 1.0f, 0.0f, 0.0f), inf), point_t<>(-5.0f, 4.0f, 0.0f), point_t<>(0.0f, 0.0f, 0.0f));
    mock_physics_object po_b(new inertia_tensor(new float[6]{ 2.9f, 2.8f, 2.7f, 0.0f, 0.0f, 0.0f }, point_t<>(-4.3f, 0.0f, 0.0f), 2.5f), point_t<>( 7.0f, 8.0f, 0.0f), point_t<>(0.0f, 0.0f, 0.0f));
    instantaneous_collide(&po_a, &po_b, poc, noc, cor, mu);

    /* Checks */
    BOOST_REQUIRE(po_a.number_of_impulses() == 3);
    BOOST_REQUIRE(po_b.number_of_impulses() == 3);

    BOOST_CHECK(po_a.get_collision_normal() == point_t<>(0.0f, 1.0f, 0.0f));
    BOOST_CHECK(po_a.get_angular_weight() == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 0.5518651f, result_tolerance);

    BOOST_CHECK(po_a.get_collision_normal() == noc);
    BOOST_CHECK(po_a.get_angular_weight() == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 1.10373f, result_tolerance);

    BOOST_CHECK(po_a.get_collision_normal() == noc);
    BOOST_CHECK(po_a.get_angular_weight() == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 43.89627f, result_tolerance);

    BOOST_CHECK(po_b.get_collision_normal() == point_t<>(0.0f, 1.0f, 0.0f));
    BOOST_CHECK(std::fabs(magnitude(po_b.get_angular_weight() - point_t<>(0.0f, 0.0f, 1.59259f))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_b.get_impulse(), -0.5518651f, result_tolerance);

    BOOST_CHECK(po_b.get_collision_normal() == noc);
    BOOST_CHECK(po_b.get_angular_weight() == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK_CLOSE(po_b.get_impulse(), -1.10373f, result_tolerance);

    BOOST_CHECK(po_b.get_collision_normal() == noc);
    BOOST_CHECK(po_b.get_angular_weight() == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK_CLOSE(po_b.get_impulse(), -43.89627f, result_tolerance);
}

BOOST_AUTO_TEST_CASE( instantaneous_rotationless_collide_infinite_mass_b_test )
{
    const float mu = 0.333f;
    const float cor = 0.75f;
    const point_t<> poc(0.0f, 0.0f, 0.0f);
    const point_t<> noc(0.0f, 1.0f, 0.0f);
    mock_physics_object po_a(new inertia_tensor(new float[6]{ 1.0f, 1.1f, 1.2f, 0.0f, 0.0f, 0.0f }, point_t<>(0.0f,  1.0f, 0.0f), 1.0f), point_t<>(0.0f, -5.0f, -7.0f), point_t<>(0.0f, 0.0f, 0.0f));
    mock_physics_object po_b(new inertia_tensor(new float[6]{ inf, inf, inf, inf, inf, inf }, point_t<>(0.0f, -4.3f, 0.0f), inf), point_t<>(0.0f, 7.0f, -4.0f), point_t<>(0.0f, 0.0f, 0.0f));
    instantaneous_collide(&po_a, &po_b, poc, noc, cor, mu);

    /* Checks */
    BOOST_REQUIRE(po_a.number_of_impulses() == 3);
    BOOST_REQUIRE(po_b.number_of_impulses() == 3);

    BOOST_CHECK(std::fabs(magnitude(po_a.get_collision_normal() - point_t<>(0.0f, 0.0f, 1.0f))) < result_tolerance);
    BOOST_CHECK(po_a.get_angular_weight() == point_t<>(-1.0f, 0.0f, 0.0f));
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 1.5f, result_tolerance);

    BOOST_CHECK(po_a.get_collision_normal() == noc);
    BOOST_CHECK(po_a.get_angular_weight() == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 4.5045f, result_tolerance);

    BOOST_CHECK(po_a.get_collision_normal() == noc);
    BOOST_CHECK(po_a.get_angular_weight() == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 16.4954f, result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(po_b.get_collision_normal() - point_t<>(0.0f, 0.0f, 1.0f))) < result_tolerance);
    BOOST_CHECK(po_b.get_angular_weight() == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK_CLOSE(po_b.get_impulse(), -1.5f, result_tolerance);

    BOOST_CHECK(po_b.get_collision_normal() == noc);
    BOOST_CHECK(po_b.get_angular_weight() == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK_CLOSE(po_b.get_impulse(), -4.5045f, result_tolerance);

    BOOST_CHECK(po_b.get_collision_normal() == noc);
    BOOST_CHECK(po_b.get_angular_weight() == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK_CLOSE(po_b.get_impulse(), -16.4954f, result_tolerance);
}

/* Test instantaneous collide with off center impact (i.e. imparting some rotation) */
BOOST_AUTO_TEST_CASE( instantaneous_off_center_collide_test )
{
    const float mu = 0.75f;
    const float cor = 0.81f;
    const point_t<> poc(-1.0f, 3.6f, 0.0f);
    const point_t<> noc( 0.7071067812f, 0.7071067812f, 0.0f);
    mock_physics_object po_a(new inertia_tensor(new float[6]{ 9.0f, 4.6f, 5.6f, 4.1f, -0.5f, 4.6f }, point_t<>( 1.0f, 0.0f, -2.3f), 9.0f), point_t<>(-5.0f, 0.0f, 0.0f), point_t<>(0.0f, 0.0f, 0.0f));
    mock_physics_object po_b(new inertia_tensor(new float[6]{ 2.0f, 1.2f, 8.2f, 0.0f,  0.0f, 0.0f }, point_t<>(-4.3f, 5.7f,  0.0f), 5.5f), point_t<>( 7.0f, 0.0f, 0.0f), point_t<>(0.0f, 0.0f, 0.0f));
    instantaneous_collide(&po_a, &po_b, poc, noc, cor, mu);

    /* Checks */
    BOOST_REQUIRE(po_a.number_of_impulses() == 2);
    BOOST_REQUIRE(po_b.number_of_impulses() == 2);

    BOOST_CHECK(fabs(magnitude(po_a.get_collision_normal() - point_t<>(0.707107f, -0.707107f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_a.get_angular_weight() - point_t<>(1.38466f, -1.03624f, -1.43195f))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 1.7977f, result_tolerance);

    BOOST_CHECK(po_a.get_collision_normal() == noc);
    BOOST_CHECK(fabs(magnitude(po_a.get_angular_weight() - point_t<>(0.484855f, -0.200698f, -1.1233f))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 2.39693f, result_tolerance);

    BOOST_CHECK(fabs(magnitude(po_b.get_collision_normal() - point_t<>(0.707107f, -0.707107f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b.get_angular_weight() - point_t<>(0.0f, 0.0f, -0.103479f))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_b.get_impulse(), -1.7977f, result_tolerance);

    BOOST_CHECK(po_b.get_collision_normal() == noc);
    BOOST_CHECK(fabs(magnitude(po_b.get_angular_weight() - point_t<>(0.0f, 0.0f, 0.465656f))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_b.get_impulse(), -2.39693f, result_tolerance);
}

BOOST_AUTO_TEST_CASE( instantaneous_off_center_collide_infinite_mass_a_test )
{
    const float mu = 0.5f;
    const float cor = 0.57f;
    const point_t<> poc(-2.0f, 0.0f, 0.0f);
    const point_t<> noc(0.0f, -0.7071067812f, -0.7071067812f);
    mock_physics_object po_a(new inertia_tensor(new float[6]{ inf, inf, inf, inf, inf, inf }, point_t<>( 1.0f, 0.0f, -2.3f), inf), point_t<>(0.0f, 10.7f, 1.3f), point_t<>(0.0f, 0.0f, 0.0f));
    mock_physics_object po_b(new inertia_tensor(new float[6]{ 2.0f, 1.2f, 8.2f, -0.2f, 0.7f, -3.5f }, point_t<>(-4.3f, 5.7f,  0.0f), 5.5f), point_t<>(0.0f, -4.5f, -22.3f), point_t<>(0.0f, 0.0f, 0.0f));
    instantaneous_collide(&po_a, &po_b, poc, noc, cor, mu);

    /* Checks */
    BOOST_REQUIRE(po_a.number_of_impulses() == 3);
    BOOST_REQUIRE(po_b.number_of_impulses() == 3);

    BOOST_CHECK(fabs(magnitude(po_a.get_collision_normal() - point_t<>(0., 0.707107f, -0.707107f))) < result_tolerance);
    BOOST_CHECK(po_a.get_angular_weight() == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 0.0551666f, result_tolerance);

    BOOST_CHECK(po_a.get_collision_normal() == noc);
    BOOST_CHECK(po_a.get_angular_weight() == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 0.110333f, result_tolerance);

    BOOST_CHECK(po_a.get_collision_normal() == noc);
    BOOST_CHECK(po_a.get_angular_weight() == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 1.64106f, result_tolerance);

    BOOST_CHECK(fabs(magnitude(po_b.get_collision_normal() - point_t<>(0., 0.707107f, -0.707107f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b.get_angular_weight() - point_t<>(9.23708f, 0.504338f, 4.09794f))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_b.get_impulse(), -0.0551666f, result_tolerance);

    BOOST_CHECK(po_b.get_collision_normal() == noc);
    BOOST_CHECK(fabs(magnitude(po_b.get_angular_weight() - point_t<>(6.39831f, 0.99378f, 2.44782f))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_b.get_impulse(), -0.110333f, result_tolerance);

    BOOST_CHECK(po_b.get_collision_normal() == noc);
    BOOST_CHECK(fabs(magnitude(po_b.get_angular_weight() - point_t<>(6.39831f, 0.99378f, 2.44782f))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_b.get_impulse(), -1.64106f, result_tolerance);
}

BOOST_AUTO_TEST_CASE( instantaneous_off_center_collide_infinite_mass_b_test )
{
    const float mu = 0.333f;
    const float cor = 0.35f;
    const point_t<> poc(-2.0f, 0.0f, -1.2f);
    const point_t<> noc(-0.4082482905f, -0.8164965809f, 0.4082482905f);
    mock_physics_object po_a(new inertia_tensor(new float[6]{ 1.0f, 1.1f, 1.2f, 0.0f, 0.0f, 0.0f }, point_t<>( 1.0f, 0.0f, -2.3f), 1.9f), point_t<>(0.0f, 10.7f, 1.3f), point_t<>(0.0f, 0.0f, 0.0f));
    mock_physics_object po_b(new inertia_tensor(new float[6]{ inf, inf, inf, inf, inf, inf }, point_t<>(-4.3f, 5.7f,  0.0f), inf), point_t<>(0.0f, 0.0f, 0.0f), point_t<>(0.0f, 0.0f, 0.0f));
    instantaneous_collide(&po_a, &po_b, poc, noc, cor, mu);

    /* Checks */
    BOOST_REQUIRE(po_a.number_of_impulses() == 2);
    BOOST_REQUIRE(po_b.number_of_impulses() == 2);

    BOOST_CHECK(fabs(magnitude(po_a.get_collision_normal() - point_t<>(0.47933f, -0.572335f, -0.665339f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_a.get_angular_weight() - point_t<>(0.629568f, -1.33523f, 1.43084f))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 0.467515f, result_tolerance);

    BOOST_CHECK(po_a.get_collision_normal() == noc);
    BOOST_CHECK(fabs(magnitude(po_a.get_angular_weight() - point_t<>(0.898146f, 0.705156f, 2.04124f))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 1.40395f, result_tolerance);

    BOOST_CHECK(fabs(magnitude(po_b.get_collision_normal() - point_t<>(0.47933f, -0.572335f, -0.665339f))) < result_tolerance);
    BOOST_CHECK(po_b.get_angular_weight() == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK_CLOSE(po_b.get_impulse(), -0.467515f, result_tolerance);

    BOOST_CHECK(po_b.get_collision_normal() == noc);
    BOOST_CHECK(po_b.get_angular_weight() == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK_CLOSE(po_b.get_impulse(), -1.40395f, result_tolerance);
}

/* Test instantaneous collide with rotation */
BOOST_AUTO_TEST_CASE( instantaneous_rotating_collide_test )
{
    const float mu = 0.75f;
    const float cor = 0.1f;
    const point_t<> poc(-1.0f, 3.6f, 0.0f);
    const point_t<> noc( 0.7071067812f, 0.7071067812f, 0.0f);
    mock_physics_object po_a(new inertia_tensor(new float[6]{ 9.0f, 4.6f, 5.6f, 4.1f, -0.5f, 4.6f }, point_t<>( 1.0f, 0.0f, -2.3f), 9.0f), point_t<>(-5.0f, 0.0f, 0.0f), point_t<>( 0.0f, 3.3f, 0.0f));
    mock_physics_object po_b(new inertia_tensor(new float[6]{ 2.0f, 1.2f, 8.2f, -0.2f, 0.7f, -3.5f }, point_t<>(-4.3f, 5.7f,  0.0f), 5.5f), point_t<>( 7.0f, 0.0f, 0.0f), point_t<>(-1.5f, 0.0f, 0.0f));
    instantaneous_collide(&po_a, &po_b, poc, noc, cor, mu);

    /* Checks */
    BOOST_REQUIRE(po_a.number_of_impulses() == 2);
    BOOST_REQUIRE(po_b.number_of_impulses() == 2);

    BOOST_CHECK(fabs(magnitude(po_a.get_collision_normal() - point_t<>(0.474149f, -0.474149f, -0.741866f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_a.get_angular_weight() - point_t<>(-0.663117f, 0.555435f, 0.458824f))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 0.19551f, result_tolerance);

    BOOST_CHECK(po_a.get_collision_normal() == noc);
    BOOST_CHECK(fabs(magnitude(po_a.get_angular_weight() - point_t<>(0.484855f, -0.200698f, -1.1233f))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 0.26068f, result_tolerance);

    BOOST_CHECK(fabs(magnitude(po_b.get_collision_normal() - point_t<>(0.474149f, -0.474149f, -0.741866f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b.get_angular_weight() - point_t<>(2.2084f, 1.99833f, 0.702633f))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_b.get_impulse(), -0.19551f, result_tolerance);

    BOOST_CHECK(po_b.get_collision_normal() == noc);
    BOOST_CHECK(fabs(magnitude(po_b.get_angular_weight() - point_t<>(3.33246f, -0.574562f, 1.9371f))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_b.get_impulse(), -0.26068f, result_tolerance);
}

BOOST_AUTO_TEST_CASE( instantaneous_rotating_collide_infinite_mass_a_test )
{
    const float mu = 0.5f;
    const float cor = 0.46f;
    const point_t<> poc(-2.0f, 0.0f, 0.0f);
    const point_t<> noc(0.0f, -0.7071067812f, -0.7071067812f);
    mock_physics_object po_a(new inertia_tensor(new float[6]{ inf, inf, inf, inf, inf, inf }, point_t<>( 1.0f, 0.0f, -2.3f), inf), point_t<>(0.0f, 10.7f, 1.3f), point_t<>(-7.9f, 0.0f, 4.0f));
    mock_physics_object po_b(new inertia_tensor(new float[6]{ 2.0f, 1.2f, 8.2f, -0.2f, 0.7f, -3.5f }, point_t<>(-4.3f, 5.7f,  0.0f), 5.5f), point_t<>(0.0f, -4.5f, -22.3f), point_t<>(0.0f, 10.0f, 0.0f));
    instantaneous_collide(&po_a, &po_b, poc, noc, cor, mu);

    /* Checks */
    BOOST_REQUIRE(po_a.number_of_impulses() == 3);
    BOOST_REQUIRE(po_b.number_of_impulses() == 3);

    BOOST_CHECK(fabs(magnitude(po_a.get_collision_normal() - point_t<>(0.0f, 0.707107f, -0.707107f))) < result_tolerance);
    BOOST_CHECK(po_a.get_angular_weight() == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 0.165697f, result_tolerance);

    BOOST_CHECK(po_a.get_collision_normal() == noc);
    BOOST_CHECK(po_a.get_angular_weight() == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 0.331394f, result_tolerance);

    BOOST_CHECK(po_a.get_collision_normal() == noc);
    BOOST_CHECK(po_a.get_angular_weight() == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 2.42093f, result_tolerance);

    BOOST_CHECK(fabs(magnitude(po_b.get_collision_normal() - point_t<>(0.0f, 0.707107f, -0.707107f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b.get_angular_weight() - point_t<>(9.23708f, 0.504338f, 4.09794f))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_b.get_impulse(), -0.165697f, result_tolerance);

    BOOST_CHECK(po_b.get_collision_normal() == noc);
    BOOST_CHECK(fabs(magnitude(po_b.get_angular_weight() - point_t<>(6.39831f, 0.99378f, 2.44782f))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_b.get_impulse(), -0.331394f, result_tolerance);

    BOOST_CHECK(po_b.get_collision_normal() == noc);
    BOOST_CHECK(fabs(magnitude(po_b.get_angular_weight() - point_t<>(6.39831f, 0.99378f, 2.44782f))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_b.get_impulse(), -2.42093f, result_tolerance);
}

BOOST_AUTO_TEST_CASE( instantaneous_rotating_collide_infinite_mass_b_test )
{
    const float mu = 0.333f;
    const float cor = 0.85f;
    const point_t<> poc(-2.0f, 0.0f, -1.2f);
    const point_t<> noc(0.4082482905f, 0.8164965809f, -0.4082482905f);
    mock_physics_object po_a(new inertia_tensor(new float[6]{ 1.0f, 1.1f, 1.2f, 0.0f, 0.0f, 0.0f }, point_t<>( 1.0f, 0.0f, -2.3f), 1.9f), point_t<>(-15.0f, 10.7f, 1.3f), point_t<>(-7.9f, 11.0f, 4.0f));
    mock_physics_object po_b(new inertia_tensor(new float[6]{ inf, inf, inf, inf, inf, inf }, point_t<>(-4.3f, 5.7f,  0.0f), inf), point_t<>(-5.6f, -4.5f, -22.3f), point_t<>(-1.9f, 10.0f, 14.0f));
    instantaneous_collide(&po_a, &po_b, poc, noc, cor, mu);

    /* Checks */
    BOOST_REQUIRE(po_a.number_of_impulses() == 2);
    BOOST_REQUIRE(po_b.number_of_impulses() == 2);

    BOOST_CHECK(fabs(magnitude(po_a.get_collision_normal() - point_t<>(0.5495f, -0.576916f, -0.604333f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_a.get_angular_weight() - point_t<>(0.634608f, -1.09868f, 1.44229f))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 7.37174f, result_tolerance);

    BOOST_CHECK(po_a.get_collision_normal() == noc);
    BOOST_CHECK(fabs(magnitude(po_a.get_angular_weight() - point_t<>(-0.898146f, -0.705156f, -2.04124f))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 22.1373f, result_tolerance);

    BOOST_CHECK(fabs(magnitude(po_b.get_collision_normal() - point_t<>(0.5495f, -0.576916f, -0.604333f))) < result_tolerance);
    BOOST_CHECK(po_b.get_angular_weight() == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK_CLOSE(po_b.get_impulse(), -7.37174f, result_tolerance);

    BOOST_CHECK(po_b.get_collision_normal() == noc);
    BOOST_CHECK(po_b.get_angular_weight() == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK_CLOSE(po_b.get_impulse(), -22.1373f, result_tolerance);
}

/* Test instantaneous collide with various friction/reaction components */
BOOST_AUTO_TEST_CASE( instantaneous_reaction_beating_friction_collide_test )
{
    const float mu = 0.75f;
    const float cor = 0.5f;
    const point_t<> poc(-1.0f, 0.0f, 0.0f);
    const point_t<> noc( 0.0f, 1.0f, 0.0f);
    mock_physics_object po_a(new inertia_tensor(new float[6]{ 1.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f }, point_t<>(0.0f,  1.0f, 0.0f), 9.0f), point_t<>(-5.0f, 0.0f, 0.0f), point_t<>(0.0f, 0.0f, 4.0f));
    mock_physics_object po_b(new inertia_tensor(new float[6]{ inf, inf, inf, inf, inf, inf }, point_t<>(0.0f, -1.0f, 0.0f), inf), point_t<>(0.0f, 0.0f, 0.0f), point_t<>(0.0f, 0.0f, 0.0f));
    instantaneous_collide(&po_a, &po_b, poc, noc, cor, mu);

    /* Checks */
    BOOST_REQUIRE(po_a.number_of_impulses() == 2);
    BOOST_REQUIRE(po_b.number_of_impulses() == 2);

    BOOST_CHECK(fabs(magnitude(po_a.get_collision_normal() - point_t<>(1.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_a.get_angular_weight() - point_t<>(0.0f, 0.0f, 1.0f))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 12.4615f, result_tolerance);

    BOOST_CHECK(po_a.get_collision_normal() == noc);
    BOOST_CHECK(fabs(magnitude(po_a.get_angular_weight() - point_t<>(0.0f, 0.0f, -1.0f))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 16.6153f, result_tolerance);

    BOOST_CHECK(fabs(magnitude(po_b.get_collision_normal() - point_t<>(1.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b.get_angular_weight() - point_t<>(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_b.get_impulse(), -12.4615f, result_tolerance);

    BOOST_CHECK(po_b.get_collision_normal() == noc);
    BOOST_CHECK(fabs(magnitude(po_b.get_angular_weight() - point_t<>(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_b.get_impulse(), -16.6153f, result_tolerance);
}

BOOST_AUTO_TEST_CASE( instantaneous_reaction_fighting_friction_collide_test )
{
    const float mu = 0.75f;
    const float cor = 0.5f;
    const point_t<> poc(-0.1f, 0.0f, 0.0f);
    const point_t<> noc( 0.0f, 1.0f, 0.0f);
    mock_physics_object po_a(new inertia_tensor(new float[6]{ 1.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f }, point_t<>(0.0f,  1.0f, 0.0f), 9.0f), point_t<>(-5.0f, 0.0f, 0.0f), point_t<>(0.0f, 0.0f, 4.0f));
    mock_physics_object po_b(new inertia_tensor(new float[6]{ inf, inf, inf, inf, inf, inf }, point_t<>(0.0f, -1.0f, 0.0f), inf), point_t<>(0.0f, 0.0f, 0.0f), point_t<>(0.0f, 0.0f, 0.0f));
    instantaneous_collide(&po_a, &po_b, poc, noc, cor, mu);

    /* Checks */
    BOOST_REQUIRE(po_a.number_of_impulses() == 3);
    BOOST_REQUIRE(po_b.number_of_impulses() == 3);

    BOOST_CHECK(fabs(magnitude(po_a.get_collision_normal() - point_t<>(1.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_a.get_angular_weight() - point_t<>(0.0f, 0.0f, 1.0f))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 1.02272f, result_tolerance);

    BOOST_CHECK(po_a.get_collision_normal() == noc);
    BOOST_CHECK(fabs(magnitude(po_a.get_angular_weight() - point_t<>(0.0f, 0.0f, -0.1f))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 1.36363f, result_tolerance);

    BOOST_CHECK(po_a.get_collision_normal() == noc);
    BOOST_CHECK(fabs(magnitude(po_a.get_angular_weight() - point_t<>(0.0f, 0.0f, -0.1f))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 4.43494f, result_tolerance);

    BOOST_CHECK(fabs(magnitude(po_b.get_collision_normal() - point_t<>(1.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b.get_angular_weight() - point_t<>(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_b.get_impulse(), -1.02272f, result_tolerance);

    BOOST_CHECK(po_b.get_collision_normal() == noc);
    BOOST_CHECK(fabs(magnitude(po_b.get_angular_weight() - point_t<>(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_b.get_impulse(), -1.36363f, result_tolerance);

    BOOST_CHECK(po_b.get_collision_normal() == noc);
    BOOST_CHECK(fabs(magnitude(po_b.get_angular_weight() - point_t<>(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_b.get_impulse(), -4.43494f, result_tolerance);
}

BOOST_AUTO_TEST_CASE( instantaneous_reaction_helping_friction_collide_test )
{
    const float mu = 0.75f;
    const float cor = 0.5f;
    const point_t<> poc( 0.1f, 0.0f, 0.0f);
    const point_t<> noc( 0.0f, 1.0f, 0.0f);
    mock_physics_object po_a(new inertia_tensor(new float[6]{ 1.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f }, point_t<>(0.0f,  1.0f, 0.0f), 9.0f), point_t<>(-8.0f, -1.0f, 0.0f), point_t<>(0.0f, 0.0f, 2.0f));
    mock_physics_object po_b(new inertia_tensor(new float[6]{ inf, inf, inf, inf, inf, inf }, point_t<>(0.0f, -1.0f, 0.0f), inf), point_t<>(0.0f, 0.0f, 0.0f), point_t<>(0.0f, 0.0f, 0.0f));
    instantaneous_collide(&po_a, &po_b, poc, noc, cor, mu);

    /* Checks */
    BOOST_REQUIRE(po_a.number_of_impulses() == 2);
    BOOST_REQUIRE(po_b.number_of_impulses() == 2);

    BOOST_CHECK(fabs(magnitude(po_a.get_collision_normal() - point_t<>(1.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_a.get_angular_weight() - point_t<>(0.0f, 0.0f, 1.0f))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 4.58923f, result_tolerance);

    BOOST_CHECK(po_a.get_collision_normal() == noc);
    BOOST_CHECK(fabs(magnitude(po_a.get_angular_weight() - point_t<>(0.0f, 0.0f, 0.1f))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 6.11898f, result_tolerance);

    BOOST_CHECK(fabs(magnitude(po_b.get_collision_normal() - point_t<>(1.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b.get_angular_weight() - point_t<>(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_b.get_impulse(), -4.58923f, result_tolerance);

    BOOST_CHECK(po_b.get_collision_normal() == noc);
    BOOST_CHECK(fabs(magnitude(po_b.get_angular_weight() - point_t<>(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_b.get_impulse(), -6.11898f, result_tolerance);
}

BOOST_AUTO_TEST_CASE( instantaneous_reaction_stopping_friction_collide_test )
{
    const float mu = 0.75f;
    const float cor = 0.5f;
    const point_t<> poc( 1.0f, 0.0f, 0.0f);
    const point_t<> noc( 0.0f, 1.0f, 0.0f);
    mock_physics_object po_a(new inertia_tensor(new float[6]{ 1.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f }, point_t<>(0.0f,  1.0f, 0.0f), 9.0f), point_t<>(-6.0f, -5.0f, 0.0f), point_t<>(0.0f, 0.0f, 2.0f));
    mock_physics_object po_b(new inertia_tensor(new float[6]{ inf, inf, inf, inf, inf, inf }, point_t<>(0.0f, -1.0f, 0.0f), inf), point_t<>(0.0f, 0.0f, 0.0f), point_t<>(0.0f, 0.0f, 0.0f));
    instantaneous_collide(&po_a, &po_b, poc, noc, cor, mu);

    /* Checks */
    BOOST_REQUIRE(po_a.number_of_impulses() == 3);
    BOOST_REQUIRE(po_b.number_of_impulses() == 3);

    BOOST_CHECK(fabs(magnitude(po_a.get_collision_normal() - point_t<>(1.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_a.get_angular_weight() - point_t<>(0.0f, 0.0f, 1.0f))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 1.63636f, result_tolerance);

    BOOST_CHECK(po_a.get_collision_normal() == noc);
    BOOST_CHECK(fabs(magnitude(po_a.get_angular_weight() - point_t<>(0.0f, 0.0f, 1.0f))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 2.18181f, result_tolerance);

    BOOST_CHECK(po_a.get_collision_normal() == noc);
    BOOST_CHECK(fabs(magnitude(po_a.get_angular_weight() - point_t<>(0.0f, 0.0f, 1.0f))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 0.395454f, result_tolerance);

    BOOST_CHECK(fabs(magnitude(po_b.get_collision_normal() - point_t<>(1.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b.get_angular_weight() - point_t<>(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_b.get_impulse(), -1.63636f, result_tolerance);

    BOOST_CHECK(po_b.get_collision_normal() == noc);
    BOOST_CHECK(fabs(magnitude(po_b.get_angular_weight() - point_t<>(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_b.get_impulse(), -2.18181f, result_tolerance);

    BOOST_CHECK(po_b.get_collision_normal() == noc);
    BOOST_CHECK(fabs(magnitude(po_b.get_angular_weight() - point_t<>(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_b.get_impulse(), -0.395454f, result_tolerance);
}

/* Test rigid body collider class */
BOOST_AUTO_TEST_CASE( rigid_body_collider_class_test )
{
    const point_t<> poc(0.5f,  1.0f, 0.5f);
    const point_t<> noc(0.0f, -1.0f, 0.0f);
    std::unique_ptr<raptor_raytracer::material> m(new raptor_raytracer::phong_shader(raptor_raytracer::ext_colour_t(255, 255, 255), 1.0f));
    std::unique_ptr<physics_object> po_a(new physics_object(make_cube(m.get(), point_t<>(-0.5f, -0.5f, -0.5f), point_t<>(0.5f, 0.5f, 0.5f)), point_t<>(0.5f, 0.5f, 0.5f), 1.0f));
    std::unique_ptr<physics_object> po_b(new physics_object(make_cube(m.get(), point_t<>(-0.5f, -0.5f, -0.5f), point_t<>(0.5f, 0.5f, 0.5f)), point_t<>(0.5f, 1.5f, 0.5f), 1.0f));

    std::unique_ptr<collider> uut(new rigid_body_collider(0.1f, 0.0f, 0.3f));

    /* Collide and check */
    po_a->set_velocity(point_t<>(-10.0f,  10.0f, 0.0f));
    po_b->set_velocity(point_t<>( 10.0f, -10.0f, 0.0f));

    uut->collide(po_a.get(), po_b.get(), poc, noc, collision_t::COLLISION);
    BOOST_CHECK(fabs(magnitude(po_a->get_velocity_0()         - point_t<>(-6.7f,  -1.0f,  0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_a->get_angular_velocity_0() - point_t<>( 0.0f,   0.0f, -9.9f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_a->get_velocity_0(poc)      - point_t<>(-1.75f, -1.0f,  0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b->get_velocity_0()         - point_t<>( 6.7f,   1.0f,  0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b->get_angular_velocity_0() - point_t<>( 0.0f,   0.0f, -9.9f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b->get_velocity_0(poc)      - point_t<>( 1.75f,  1.0f,  0.0f))) < result_tolerance);

    /* Collide and check */
    po_a->set_velocity(point_t<>(-10.0f,  10.0f, 0.0f));
    po_b->set_velocity(point_t<>( 10.0f, -10.0f, 0.0f));
    po_a->set_angular_velocity(point_t<>(0.0f, 0.0f, 0.0f));
    po_b->set_angular_velocity(point_t<>(0.0f, 0.0f, 0.0f));
    
    uut->collide(po_b.get(), po_a.get(), poc, -noc, collision_t::COLLISION);
    BOOST_CHECK(fabs(magnitude(po_a->get_velocity_0()         - point_t<>(-6.7f,  -1.0f,  0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_a->get_angular_velocity_0() - point_t<>( 0.0f,   0.0f, -9.9f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_a->get_velocity_0(poc)      - point_t<>(-1.75f, -1.0f,  0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b->get_velocity_0()         - point_t<>( 6.7f,   1.0f,  0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b->get_angular_velocity_0() - point_t<>( 0.0f,   0.0f, -9.9f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b->get_velocity_0(poc)      - point_t<>( 1.75f,  1.0f,  0.0f))) < result_tolerance);
}

BOOST_AUTO_TEST_CASE( rigid_body_collider_class_reverse_test )
{
    const point_t<> poc(0.5f,  1.0f, 0.5f);
    const point_t<> noc(0.0f, -1.0f, 0.0f);
    std::unique_ptr<raptor_raytracer::material> m(new raptor_raytracer::phong_shader(raptor_raytracer::ext_colour_t(255, 255, 255), 1.0f));
    std::unique_ptr<physics_object> po_a(new physics_object(make_cube(m.get(), point_t<>(-0.5f, -0.5f, -0.5f), point_t<>(0.5f, 0.5f, 0.5f)), point_t<>(0.5f, 0.5f, 0.5f), 1.0f));
    std::unique_ptr<physics_object> po_b(new physics_object(make_cube(m.get(), point_t<>(-0.5f, -0.5f, -0.5f), point_t<>(0.5f, 0.5f, 0.5f)), point_t<>(0.5f, 1.5f, 0.5f), 1.0f));

    std::unique_ptr<collider> uut(new rigid_body_collider(0.1f, 0.0f, 0.3f));

    /* Collide and check */
    po_a->set_velocity(point_t<>( 10.0f,  10.0f, 0.0f));
    po_b->set_velocity(point_t<>(-10.0f, -10.0f, 0.0f));

    uut->collide(po_a.get(), po_b.get(), poc, noc, collision_t::COLLISION);
    BOOST_CHECK(fabs(magnitude(po_a->get_velocity_0()         - point_t<>( 6.7f,  -1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_a->get_angular_velocity_0() - point_t<>( 0.0f,   0.0f, 9.9f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_a->get_velocity_0(poc)      - point_t<>( 1.75f, -1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b->get_velocity_0()         - point_t<>(-6.7f,   1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b->get_angular_velocity_0() - point_t<>( 0.0f,   0.0f, 9.9f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b->get_velocity_0(poc)      - point_t<>(-1.75f,  1.0f, 0.0f))) < result_tolerance);

    /* Collide and check */
    po_a->set_velocity(point_t<>( 10.0f,  10.0f, 0.0f));
    po_b->set_velocity(point_t<>(-10.0f, -10.0f, 0.0f));
    po_a->set_angular_velocity(point_t<>(0.0f, 0.0f, 0.0f));
    po_b->set_angular_velocity(point_t<>(0.0f, 0.0f, 0.0f));
    
    uut->collide(po_b.get(), po_a.get(), poc, -noc, collision_t::COLLISION);
    BOOST_CHECK(fabs(magnitude(po_a->get_velocity_0()         - point_t<>(6.7f, -1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_a->get_angular_velocity_0() - point_t<>(0.0f,  0.0f, 9.9f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_a->get_velocity_0(poc)      - point_t<>(1.75f, -1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b->get_velocity_0()         - point_t<>(-6.7f, 1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b->get_angular_velocity_0() - point_t<>(0.0f, 0.0f, 9.9f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b->get_velocity_0(poc)      - point_t<>(-1.75f, 1.0f, 0.0f))) < result_tolerance);
}

BOOST_AUTO_TEST_CASE( rigid_body_collider_class_more_friction_test )
{
    const point_t<> poc(0.5f,  1.0f, 0.5f);
    const point_t<> noc(0.0f, -1.0f, 0.0f);
    std::unique_ptr<raptor_raytracer::material> m(new raptor_raytracer::phong_shader(raptor_raytracer::ext_colour_t(255, 255, 255), 1.0f));
    std::unique_ptr<physics_object> po_a(new physics_object(make_cube(m.get(), point_t<>(-0.5f, -0.5f, -0.5f), point_t<>(0.5f, 0.5f, 0.5f)), point_t<>(0.5f, 0.5f, 0.5f), 1.0f));
    std::unique_ptr<physics_object> po_b(new physics_object(make_cube(m.get(), point_t<>(-0.5f, -0.5f, -0.5f), point_t<>(0.5f, 0.5f, 0.5f)), point_t<>(0.5f, 1.5f, 0.5f), 1.0f));
    
    std::unique_ptr<collider> uut(new rigid_body_collider(0.1f, 0.0f, 0.35f));

    /* Collide and check */
    po_a->set_velocity(point_t<>(-10.0f,  10.0f, 0.0f));
    po_b->set_velocity(point_t<>( 10.0f, -10.0f, 0.0f));

    uut->collide(po_a.get(), po_b.get(), poc, noc, collision_t::COLLISION);
    BOOST_CHECK(fabs(magnitude(po_a->get_velocity_0()         - point_t<>(-6.15f,  -1.0f,   0.0f)))  < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_a->get_angular_velocity_0() - point_t<>( 0.0f,    0.0f, -11.55f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_a->get_velocity_0(poc)      - point_t<>(-0.375f, -1.0f,   0.0f)))  < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b->get_velocity_0()         - point_t<>( 6.15f,   1.0f,   0.0f)))  < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b->get_angular_velocity_0() - point_t<>( 0.0f,    0.0f, -11.55f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b->get_velocity_0(poc)      - point_t<>( 0.375f,  1.0f,   0.0f)))  < result_tolerance);

    /* Collide and check */
    po_a->set_velocity(point_t<>(-10.0f,  10.0f, 0.0f));
    po_b->set_velocity(point_t<>( 10.0f, -10.0f, 0.0f));
    po_a->set_angular_velocity(point_t<>(0.0f, 0.0f, 0.0f));
    po_b->set_angular_velocity(point_t<>(0.0f, 0.0f, 0.0f));
    
    uut->collide(po_b.get(), po_a.get(), poc, -noc, collision_t::COLLISION);
    BOOST_CHECK(fabs(magnitude(po_a->get_velocity_0()         - point_t<>(-6.15f, -1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_a->get_angular_velocity_0() - point_t<>(0.0f,  0.0f, -11.55f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_a->get_velocity_0(poc)      - point_t<>(-0.375f, -1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b->get_velocity_0()         - point_t<>(6.15f, 1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b->get_angular_velocity_0() - point_t<>(0.0f, 0.0f, -11.55f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b->get_velocity_0(poc)      - point_t<>(0.375f, 1.0f, 0.0f))) < result_tolerance);
}

BOOST_AUTO_TEST_CASE( rigid_body_collider_class_max_friction_test )
{
    const point_t<> poc(0.5f,  1.0f, 0.5f);
    const point_t<> noc(0.0f, -1.0f, 0.0f);
    std::unique_ptr<raptor_raytracer::material> m(new raptor_raytracer::phong_shader(raptor_raytracer::ext_colour_t(255, 255, 255), 1.0f));
    std::unique_ptr<physics_object> po_a(new physics_object(make_cube(m.get(), point_t<>(-0.5f, -0.5f, -0.5f), point_t<>(0.5f, 0.5f, 0.5f)), point_t<>(0.5f, 0.5f, 0.5f), 1.0f));
    std::unique_ptr<physics_object> po_b(new physics_object(make_cube(m.get(), point_t<>(-0.5f, -0.5f, -0.5f), point_t<>(0.5f, 0.5f, 0.5f)), point_t<>(0.5f, 1.5f, 0.5f), 1.0f));

    std::unique_ptr<collider> uut(new rigid_body_collider(0.1f, 0.0f, 0.9f));

    /* Collide and check */
    po_a->set_velocity(point_t<>(-10.0f,  10.0f, 0.0f));
    po_b->set_velocity(point_t<>( 10.0f, -10.0f, 0.0f));
    uut->collide(po_a.get(), po_b.get(), poc, noc, collision_t::COLLISION);
    BOOST_CHECK(fabs(magnitude(po_a->get_velocity_0()         - point_t<>(-6.00007f,     -1.0f,   0.0f   ))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_a->get_angular_velocity_0() - point_t<>( 0.0f,          0.0f, -11.9998f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_a->get_velocity_0(poc)      - point_t<>(-0.000175476f, -1.0f,   0.0f   ))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b->get_velocity_0()         - point_t<>( 6.00007f,      1.0f,   0.0f   ))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b->get_angular_velocity_0() - point_t<>( 0.0f,          0.0f, -11.9998f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b->get_velocity_0(poc)      - point_t<>( 0.000175476f,  1.0f,   0.0f   ))) < result_tolerance);

    /* Collide and check */
    po_a->set_velocity(point_t<>(-10.0f,  10.0f, 0.0f));
    po_b->set_velocity(point_t<>( 10.0f, -10.0f, 0.0f));
    po_a->set_angular_velocity(point_t<>(0.0f, 0.0f, 0.0f));
    po_b->set_angular_velocity(point_t<>(0.0f, 0.0f, 0.0f));
    
    uut->collide(po_b.get(), po_a.get(), poc, -noc, collision_t::COLLISION);
    BOOST_CHECK(fabs(magnitude(po_a->get_velocity_0()         - point_t<>(-6.00007f, -1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_a->get_angular_velocity_0() - point_t<>(0.0f,  0.0f, -11.9998f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_a->get_velocity_0(poc)      - point_t<>(-0.000175476f, -1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b->get_velocity_0()         - point_t<>(6.00007f, 1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b->get_angular_velocity_0() - point_t<>(0.0f, 0.0f, -11.9998f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b->get_velocity_0(poc)      - point_t<>(0.000175476f, 1.0f, 0.0f))) < result_tolerance);
}

BOOST_AUTO_TEST_CASE( rigid_body_collider_class_zero_tanjent_velocity_test )
{
    const point_t<> poc(0.5f,  1.0f, 0.5f);
    const point_t<> noc(0.0f, -1.0f, 0.0f);
    std::unique_ptr<raptor_raytracer::material> m(new raptor_raytracer::phong_shader(raptor_raytracer::ext_colour_t(255, 255, 255), 1.0f));
    std::unique_ptr<physics_object> po_a(new physics_object(make_cube(m.get(), point_t<>(-0.5f, -0.5f, -0.5f), point_t<>(0.5f, 0.5f, 0.5f)), point_t<>(0.5f, 0.5f, 0.5f), 1.0f));
    std::unique_ptr<physics_object> po_b(new physics_object(make_cube(m.get(), point_t<>(-0.5f, -0.5f, -0.5f), point_t<>(0.5f, 0.5f, 0.5f)), point_t<>(0.5f, 1.5f, 0.5f), 1.0f));

    std::unique_ptr<collider> uut(new rigid_body_collider(0.1f, 0.0f, 0.3f));
    
    /* Collide and check */
    po_a->set_velocity(point_t<>(0.0f,  10.0f, 0.0f));
    po_b->set_velocity(point_t<>(0.0f, -10.0f, 0.0f));

    uut->collide(po_a.get(), po_b.get(), poc, noc, collision_t::COLLISION);
    BOOST_CHECK(fabs(magnitude(po_a->get_velocity_0()         - point_t<>(0.0f, -1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_a->get_angular_velocity_0() - point_t<>(0.0f,  0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_a->get_velocity_0(poc)      - point_t<>(0.0f, -1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b->get_velocity_0()         - point_t<>(0.0f,  1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b->get_angular_velocity_0() - point_t<>(0.0f,  0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b->get_velocity_0(poc)      - point_t<>(0.0f,  1.0f, 0.0f))) < result_tolerance);

    /* Collide and check */
    po_a->set_velocity(point_t<>(0.0f,  10.0f, 0.0f));
    po_b->set_velocity(point_t<>(0.0f, -10.0f, 0.0f));
    po_a->set_angular_velocity(point_t<>(0.0f, 0.0f, 0.0f));
    po_b->set_angular_velocity(point_t<>(0.0f, 0.0f, 0.0f));

    uut->collide(po_b.get(), po_a.get(), poc, -noc, collision_t::COLLISION);
    BOOST_CHECK(fabs(magnitude(po_a->get_velocity_0()         - point_t<>(0.0f, -1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_a->get_angular_velocity_0() - point_t<>(0.0f,  0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_a->get_velocity_0(poc)      - point_t<>(0.0f, -1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b->get_velocity_0()         - point_t<>(0.0f,  1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b->get_angular_velocity_0() - point_t<>(0.0f,  0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b->get_velocity_0(poc)      - point_t<>(0.0f,  1.0f, 0.0f))) < result_tolerance);
}

BOOST_AUTO_TEST_CASE( rigid_body_collider_class_zero_mu_test )
{
    const point_t<> poc(0.5f,  1.0f, 0.5f);
    const point_t<> noc(0.0f, -1.0f, 0.0f);
    std::unique_ptr<raptor_raytracer::material> m(new raptor_raytracer::phong_shader(raptor_raytracer::ext_colour_t(255, 255, 255), 1.0f));
    std::unique_ptr<physics_object> po_a(new physics_object(make_cube(m.get(), point_t<>(-0.5f, -0.5f, -0.5f), point_t<>(0.5f, 0.5f, 0.5f)), point_t<>(0.5f, 0.5f, 0.5f), 1.0f));
    std::unique_ptr<physics_object> po_b(new physics_object(make_cube(m.get(), point_t<>(-0.5f, -0.5f, -0.5f), point_t<>(0.5f, 0.5f, 0.5f)), point_t<>(0.5f, 1.5f, 0.5f), 1.0f));

    std::unique_ptr<collider> uut(new rigid_body_collider(0.0f, 0.0f, 0.3f));
    
    /* Collide and check */
    po_a->begin_time_step(1.0f);
    po_b->begin_time_step(1.0f);
    po_a->set_velocity(point_t<>(-10.0f,  10.0f, 0.0f));
    po_b->set_velocity(point_t<>( 10.0f, -10.0f, 0.0f));

    uut->collide(po_a.get(), po_b.get(), poc, noc, collision_t::COLLISION);
    BOOST_CHECK(fabs(magnitude(po_a->get_velocity_0()         - point_t<>(-7.0f, 0.0f,  0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_a->get_angular_velocity_0() - point_t<>( 0.0f, 0.0f, -9.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_a->get_velocity_0(poc)      - point_t<>(-2.5f, 0.0f,  0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b->get_velocity_0()         - point_t<>( 7.0f, 0.0f,  0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b->get_angular_velocity_0() - point_t<>( 0.0f, 0.0f, -9.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b->get_velocity_0(poc)      - point_t<>( 2.5f, 0.0f,  0.0f))) < result_tolerance);

    /* Collide and check */
    po_a->set_velocity(point_t<>(-10.0f,  10.0f, 0.0f));
    po_b->set_velocity(point_t<>( 10.0f, -10.0f, 0.0f));
    po_a->set_angular_velocity(point_t<>(0.0f, 0.0f, 0.0f));
    po_b->set_angular_velocity(point_t<>(0.0f, 0.0f, 0.0f));

    uut->collide(po_b.get(), po_a.get(), poc, -noc, collision_t::COLLISION);
    BOOST_CHECK(fabs(magnitude(po_a->get_velocity_0()         - point_t<>(-7.0f, 0.0f,  0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_a->get_angular_velocity_0() - point_t<>( 0.0f, 0.0f, -9.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_a->get_velocity_0(poc)      - point_t<>(-2.5f, 0.0f,  0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b->get_velocity_0()         - point_t<>( 7.0f, 0.0f,  0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b->get_angular_velocity_0() - point_t<>( 0.0f, 0.0f, -9.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b->get_velocity_0(poc)      - point_t<>( 2.5f, 0.0f,  0.0f))) < result_tolerance);
}

BOOST_AUTO_TEST_SUITE_END()
}; /* namespace test */
}; /* namespace raptor_physics */
