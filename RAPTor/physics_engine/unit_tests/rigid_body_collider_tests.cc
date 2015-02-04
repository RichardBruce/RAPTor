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
BOOST_AUTO_TEST_SUITE( rigid_body_collider_tests )

const float result_tolerance = 0.001;


/* Test instantaneous frictionless collide with no rotation */
BOOST_AUTO_TEST_CASE( instantaneous_frictionless_rotationless_collide_test )
{
    const fp_t cor = 1.0;
    const point_t poc(0.0, 0.0, 0.0);
    const point_t noc(1.0, 0.0, 0.0);
    mock_physics_object po_a(new inertia_tensor(new fp_t[6]{ 1.0, 1.1, 1.2, 0.0, 0.0, 0.0 }, point_t( 1.0, 0.0, 0.0), 1.0), point_t(-5.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0));
    mock_physics_object po_b(new inertia_tensor(new fp_t[6]{ 2.9, 2.8, 2.7, 0.0, 0.0, 0.0 }, point_t(-4.3, 0.0, 0.0), 2.5), point_t( 7.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0));
    instantaneous_frictionless_collide(&po_a, &po_b, poc, noc, cor);

    BOOST_CHECK(po_a.get_collision_normal() == noc);
    BOOST_CHECK(po_a.get_angular_weight() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 17.1429, result_tolerance);

    BOOST_CHECK(po_b.get_collision_normal() == noc);
    BOOST_CHECK(po_b.get_angular_weight() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK_CLOSE(po_b.get_impulse(), -17.1429, result_tolerance);
}


BOOST_AUTO_TEST_CASE( instantaneous_frictionless_rotationless_collide_infinite_mass_a_test )
{
    const fp_t cor = 0.5;
    const fp_t inf = std::numeric_limits<float>::infinity();
    const point_t poc(0.0, 0.0, 0.0);
    const point_t noc(1.0, 0.0, 0.0);
    mock_physics_object po_a(new inertia_tensor(new fp_t[6]{ inf, inf, inf, inf, inf, inf }, point_t( 1.0, 0.0, 0.0), inf), point_t(-5.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0));
    mock_physics_object po_b(new inertia_tensor(new fp_t[6]{ 2.9, 2.8, 2.7, 0.0, 0.0, 0.0 }, point_t(-4.3, 0.0, 0.0), 2.5), point_t( 7.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0));
    instantaneous_frictionless_collide(&po_a, &po_b, poc, noc, cor);

    BOOST_CHECK(po_a.get_collision_normal() == noc);
    BOOST_CHECK(po_a.get_angular_weight() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 45.0, result_tolerance);

    BOOST_CHECK(po_b.get_collision_normal() == noc);
    BOOST_CHECK(po_b.get_angular_weight() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK_CLOSE(po_b.get_impulse(), -45.0, result_tolerance);
}


BOOST_AUTO_TEST_CASE( instantaneous_frictionless_rotationless_collide_infinite_mass_b_test )
{
    const fp_t cor = 0.75;
    const fp_t inf = std::numeric_limits<float>::infinity();
    const point_t poc(0.0, 0.0, 0.0);
    const point_t noc(0.0, 1.0, 0.0);
    mock_physics_object po_a(new inertia_tensor(new fp_t[6]{ 1.0, 1.1, 1.2, 0.0, 0.0, 0.0 }, point_t(0.0,  1.0, 0.0), 1.0), point_t(0.0, -5.0, 0.0), point_t(0.0, 0.0, 0.0));
    mock_physics_object po_b(new inertia_tensor(new fp_t[6]{ inf, inf, inf, inf, inf, inf }, point_t(0.0, -4.3, 0.0), inf), point_t(0.0, 7.0, 0.0), point_t(0.0, 0.0, 0.0));
    instantaneous_frictionless_collide(&po_a, &po_b, poc, noc, cor);

    BOOST_CHECK(po_a.get_collision_normal() == noc);
    BOOST_CHECK(po_a.get_angular_weight() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 21.0, result_tolerance);

    BOOST_CHECK(po_b.get_collision_normal() == noc);
    BOOST_CHECK(po_b.get_angular_weight() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK_CLOSE(po_b.get_impulse(), -21.0, result_tolerance);
}


/* Test instantaneous frictionless collide with off center impact (i.e. imparting some rotation) */
BOOST_AUTO_TEST_CASE( instantaneous_frictionless_off_center_collide_test )
{
    const fp_t cor = 0.81;
    const point_t poc(-1.0, 3.6, 0.0);
    const point_t noc( 0.7071067812, 0.7071067812, 0.0);
    mock_physics_object po_a(new inertia_tensor(new fp_t[6]{ 9.0, 4.6, 5.6, 4.1, -0.5, 4.6 }, point_t( 1.0, 0.0, -2.3), 9.0), point_t(-5.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0));
    mock_physics_object po_b(new inertia_tensor(new fp_t[6]{ 2.0, 1.2, 8.2, 0.0,  0.0, 0.0 }, point_t(-4.3, 5.7,  0.0), 5.5), point_t( 7.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0));
    instantaneous_frictionless_collide(&po_a, &po_b, poc, noc, cor);

    BOOST_CHECK(po_a.get_collision_normal() == noc);
    BOOST_CHECK(fabs(magnitude(po_a.get_angular_weight() - point_t(0.484855, -0.200698, -1.1233))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 1.2378, result_tolerance);

    BOOST_CHECK(po_b.get_collision_normal() == noc);
    BOOST_CHECK(fabs(magnitude(po_b.get_angular_weight() - point_t(0.0, 0.0, 0.465656))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_b.get_impulse(), -1.2378, result_tolerance);
}


BOOST_AUTO_TEST_CASE( instantaneous_frictionless_off_center_collide_infinite_mass_a_test )
{
    const fp_t cor = 0.57;
    const fp_t inf = std::numeric_limits<float>::infinity();
    const point_t poc(-2.0, 0.0, 0.0);
    const point_t noc(0.0, -0.7071067812, -0.7071067812);
    mock_physics_object po_a(new inertia_tensor(new fp_t[6]{ inf, inf, inf, inf, inf, inf }, point_t( 1.0, 0.0, -2.3), inf), point_t(0.0, 10.7, 1.3), point_t(0.0, 0.0, 0.0));
    mock_physics_object po_b(new inertia_tensor(new fp_t[6]{ 2.0, 1.2, 8.2, -0.2, 0.7, -3.5 }, point_t(-4.3, 5.7,  0.0), 5.5), point_t(0.0, -4.5, -22.3), point_t(0.0, 0.0, 0.0));
    instantaneous_frictionless_collide(&po_a, &po_b, poc, noc, cor);

    BOOST_CHECK(po_a.get_collision_normal() == noc);
    BOOST_CHECK(po_a.get_angular_weight() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 3.19932, result_tolerance);

    BOOST_CHECK(po_b.get_collision_normal() == noc);
    BOOST_CHECK(fabs(magnitude(po_b.get_angular_weight() - point_t(6.39831, 0.99378, 2.44782))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_b.get_impulse(), -3.19932, result_tolerance);
}


BOOST_AUTO_TEST_CASE( instantaneous_frictionless_off_center_collide_infinite_mass_b_test )
{
    const fp_t cor = 0.35;
    const fp_t inf = std::numeric_limits<float>::infinity();
    const point_t poc(-2.0, 0.0, -1.2);
    const point_t noc(-0.4082482905, -0.8164965809, 0.4082482905);
    mock_physics_object po_a(new inertia_tensor(new fp_t[6]{ 1.0, 1.1, 1.2, 0.0, 0.0, 0.0 }, point_t( 1.0, 0.0, -2.3), 1.9), point_t(0.0, 10.7, 1.3), point_t(0.0, 0.0, 0.0));
    mock_physics_object po_b(new inertia_tensor(new fp_t[6]{ inf, inf, inf, inf, inf, inf }, point_t(-4.3, 5.7,  0.0), inf), point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0));
    instantaneous_frictionless_collide(&po_a, &po_b, poc, noc, cor);

    BOOST_CHECK(po_a.get_collision_normal() == noc);
    BOOST_CHECK(fabs(magnitude(po_a.get_angular_weight() - point_t(0.898146, 0.705156, 2.04124))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 1.54772, result_tolerance);

    BOOST_CHECK(po_b.get_collision_normal() == noc);
    BOOST_CHECK(po_b.get_angular_weight() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK_CLOSE(po_b.get_impulse(), -1.54772, result_tolerance);
}


/* Test instantaneous frictionless collide with rotation */
BOOST_AUTO_TEST_CASE( instantaneous_frictionless_rotating_collide_test )
{
    const fp_t cor = 0.1;
    const point_t poc(-1.0, 3.6, 0.0);
    const point_t noc( 0.7071067812, 0.7071067812, 0.0);
    mock_physics_object po_a(new inertia_tensor(new fp_t[6]{ 9.0, 4.6, 5.6, 4.1, -0.5, 4.6 }, point_t( 1.0, 0.0, -2.3), 9.0), point_t(-5.0, 0.0, 0.0), point_t( 0.0, 3.3, 0.0));
    mock_physics_object po_b(new inertia_tensor(new fp_t[6]{ 2.0, 1.2, 8.2, -0.2, 0.7, -3.5 }, point_t(-4.3, 5.7,  0.0), 5.5), point_t( 7.0, 0.0, 0.0), point_t(-1.5, 0.0, 0.0));
    instantaneous_frictionless_collide(&po_a, &po_b, poc, noc, cor);

    BOOST_CHECK(po_a.get_collision_normal() == noc);
    BOOST_CHECK(fabs(magnitude(po_a.get_angular_weight() - point_t(0.484855, -0.200698, -1.1233))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 0.180977, result_tolerance);

    BOOST_CHECK(po_b.get_collision_normal() == noc);
    BOOST_CHECK(fabs(magnitude(po_b.get_angular_weight() - point_t(3.33246, -0.574562, 1.9371))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_b.get_impulse(), -0.180977, result_tolerance);
}


BOOST_AUTO_TEST_CASE( instantaneous_frictionless_rotating_collide_infinite_mass_a_test )
{
    const fp_t cor = 0.46;
    const fp_t inf = std::numeric_limits<float>::infinity();
    const point_t poc(-2.0, 0.0, 0.0);
    const point_t noc(0.0, -0.7071067812, -0.7071067812);
    mock_physics_object po_a(new inertia_tensor(new fp_t[6]{ inf, inf, inf, inf, inf, inf }, point_t( 1.0, 0.0, -2.3), inf), point_t(0.0, 10.7, 1.3), point_t(-7.9, 0.0, 4.0));
    mock_physics_object po_b(new inertia_tensor(new fp_t[6]{ 2.0, 1.2, 8.2, -0.2, 0.7, -3.5 }, point_t(-4.3, 5.7,  0.0), 5.5), point_t(0.0, -4.5, -22.3), point_t(0.0, 10.0, 0.0));
    instantaneous_frictionless_collide(&po_a, &po_b, poc, noc, cor);

    BOOST_CHECK(po_a.get_collision_normal() == noc);
    BOOST_CHECK(po_a.get_angular_weight() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 5.21191, result_tolerance);

    BOOST_CHECK(po_b.get_collision_normal() == noc);
    BOOST_CHECK(fabs(magnitude(po_b.get_angular_weight() - point_t(6.39831, 0.99378, 2.44782))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_b.get_impulse(), -5.21191, result_tolerance);
}


BOOST_AUTO_TEST_CASE( instantaneous_frictionless_rotating_collide_infinite_mass_b_test )
{
    const fp_t cor = 0.85;
    const fp_t inf = std::numeric_limits<float>::infinity();
    const point_t poc(-2.0, 0.0, -1.2);
    const point_t noc(0.4082482905, 0.8164965809, -0.4082482905);
    mock_physics_object po_a(new inertia_tensor(new fp_t[6]{ 1.0, 1.1, 1.2, 0.0, 0.0, 0.0 }, point_t( 1.0, 0.0, -2.3), 1.9), point_t(-15.0, 10.7, 1.3), point_t(-7.9, 11.0, 4.0));
    mock_physics_object po_b(new inertia_tensor(new fp_t[6]{ inf, inf, inf, inf, inf, inf }, point_t(-4.3, 5.7,  0.0), inf), point_t(-5.6, -4.5, -22.3), point_t(-1.9, 10.0, 14.0));
    instantaneous_frictionless_collide(&po_a, &po_b, poc, noc, cor);

    BOOST_CHECK(po_a.get_collision_normal() == noc);
    BOOST_CHECK(fabs(magnitude(po_a.get_angular_weight() - point_t(-0.898146, -0.705156, -2.04124))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 17.9309, result_tolerance);

    BOOST_CHECK(po_b.get_collision_normal() == noc);
    BOOST_CHECK(po_b.get_angular_weight() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK_CLOSE(po_b.get_impulse(), -17.9309, result_tolerance);
}


/* Collider with friction */
/* Test instantaneous collide with no rotation */
BOOST_AUTO_TEST_CASE( instantaneous_0_mu_rotationless_collide_test )
{
    const fp_t mu = 0.0;
    const fp_t cor = 1.0;
    const point_t poc(0.0, 0.0, 0.0);
    const point_t noc(1.0, 0.0, 0.0);
    mock_physics_object po_a(new inertia_tensor(new fp_t[6]{ 1.0, 1.1, 1.2, 0.0, 0.0, 0.0 }, point_t( 1.0, 0.0, 0.0), 1.0), point_t(-5.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0));
    mock_physics_object po_b(new inertia_tensor(new fp_t[6]{ 2.9, 2.8, 2.7, 0.0, 0.0, 0.0 }, point_t(-4.3, 0.0, 0.0), 2.5), point_t( 7.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0));
    instantaneous_collide(&po_a, &po_b, poc, noc, cor, mu);

    BOOST_CHECK(po_a.get_collision_normal() == noc);
    BOOST_CHECK(po_a.get_angular_weight() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 17.1429, result_tolerance);

    BOOST_CHECK(po_a.get_collision_normal() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(po_a.get_angular_weight() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 0.0, result_tolerance);

    BOOST_CHECK(po_b.get_collision_normal() == noc);
    BOOST_CHECK(po_b.get_angular_weight() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK_CLOSE(po_b.get_impulse(), -17.1429, result_tolerance);

    BOOST_CHECK(po_b.get_collision_normal() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(po_b.get_angular_weight() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK_CLOSE(po_b.get_impulse(), 0.0, result_tolerance);
}


BOOST_AUTO_TEST_CASE( instantaneous_0_mu_rotationless_collide_infinite_mass_a_test )
{
    const fp_t mu = 0.0;
    const fp_t cor = 0.5;
    const fp_t inf = std::numeric_limits<float>::infinity();
    const point_t poc(0.0, 0.0, 0.0);
    const point_t noc(1.0, 0.0, 0.0);
    mock_physics_object po_a(new inertia_tensor(new fp_t[6]{ inf, inf, inf, inf, inf, inf }, point_t( 1.0, 0.0, 0.0), inf), point_t(-5.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0));
    mock_physics_object po_b(new inertia_tensor(new fp_t[6]{ 2.9, 2.8, 2.7, 0.0, 0.0, 0.0 }, point_t(-4.3, 0.0, 0.0), 2.5), point_t( 7.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0));
    instantaneous_collide(&po_a, &po_b, poc, noc, cor, mu);

    BOOST_CHECK(po_a.get_collision_normal() == noc);
    BOOST_CHECK(po_a.get_angular_weight() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 45.0, result_tolerance);

    BOOST_CHECK(po_a.get_collision_normal() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(po_a.get_angular_weight() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 0.0, result_tolerance);

    BOOST_CHECK(po_b.get_collision_normal() == noc);
    BOOST_CHECK(po_b.get_angular_weight() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK_CLOSE(po_b.get_impulse(), -45.0, result_tolerance);

    BOOST_CHECK(po_b.get_collision_normal() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(po_b.get_angular_weight() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK_CLOSE(po_b.get_impulse(), 0.0, result_tolerance);
}


BOOST_AUTO_TEST_CASE( instantaneous_0_mu_rotationless_collide_infinite_mass_b_test )
{
    const fp_t mu = 0.0;
    const fp_t cor = 0.75;
    const fp_t inf = std::numeric_limits<float>::infinity();
    const point_t poc(0.0, 0.0, 0.0);
    const point_t noc(0.0, 1.0, 0.0);
    mock_physics_object po_a(new inertia_tensor(new fp_t[6]{ 1.0, 1.1, 1.2, 0.0, 0.0, 0.0 }, point_t(0.0,  1.0, 0.0), 1.0), point_t(0.0, -5.0, 0.0), point_t(0.0, 0.0, 0.0));
    mock_physics_object po_b(new inertia_tensor(new fp_t[6]{ inf, inf, inf, inf, inf, inf }, point_t(0.0, -4.3, 0.0), inf), point_t(0.0, 7.0, 0.0), point_t(0.0, 0.0, 0.0));
    instantaneous_collide(&po_a, &po_b, poc, noc, cor, mu);

    BOOST_CHECK(po_a.get_collision_normal() == noc);
    BOOST_CHECK(po_a.get_angular_weight() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 21.0, result_tolerance);

    BOOST_CHECK(po_a.get_collision_normal() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(po_a.get_angular_weight() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 0.0, result_tolerance);

    BOOST_CHECK(po_b.get_collision_normal() == noc);
    BOOST_CHECK(po_b.get_angular_weight() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK_CLOSE(po_b.get_impulse(), -21.0, result_tolerance);

    BOOST_CHECK(po_b.get_collision_normal() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(po_b.get_angular_weight() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK_CLOSE(po_b.get_impulse(), 0.0, result_tolerance);
}


/* Test instantaneous collide with off center impact (i.e. imparting some rotation) */
BOOST_AUTO_TEST_CASE( instantaneous_0_mu_off_center_collide_test )
{
    const fp_t mu = 0.0;
    const fp_t cor = 0.81;
    const point_t poc(-1.0, 3.6, 0.0);
    const point_t noc( 0.7071067812, 0.7071067812, 0.0);
    mock_physics_object po_a(new inertia_tensor(new fp_t[6]{ 9.0, 4.6, 5.6, 4.1, -0.5, 4.6 }, point_t( 1.0, 0.0, -2.3), 9.0), point_t(-5.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0));
    mock_physics_object po_b(new inertia_tensor(new fp_t[6]{ 2.0, 1.2, 8.2, 0.0,  0.0, 0.0 }, point_t(-4.3, 5.7,  0.0), 5.5), point_t( 7.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0));
    instantaneous_collide(&po_a, &po_b, poc, noc, cor, mu);

    BOOST_CHECK(po_a.get_collision_normal() == noc);
    BOOST_CHECK(fabs(magnitude(po_a.get_angular_weight() - point_t(0.484855, -0.200698, -1.1233))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 1.2378, result_tolerance);

    BOOST_CHECK(fabs(magnitude(po_a.get_collision_normal() - point_t(0.707107, -0.707107, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_a.get_angular_weight() - point_t(1.38466, -1.03624, -1.43195))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 0.0, result_tolerance);

    BOOST_CHECK(po_b.get_collision_normal() == noc);
    BOOST_CHECK(fabs(magnitude(po_b.get_angular_weight() - point_t(0.0, 0.0, 0.465656))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_b.get_impulse(), -1.2378, result_tolerance);

    BOOST_CHECK(fabs(magnitude(po_b.get_collision_normal() - point_t(0.707107, -0.707107, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b.get_angular_weight() - point_t(0.0, 0.0, -0.103479))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_b.get_impulse(), 0.0, result_tolerance);
}


BOOST_AUTO_TEST_CASE( instantaneous_0_mu_off_center_collide_infinite_mass_a_test )
{
    const fp_t mu = 0.0;
    const fp_t cor = 0.57;
    const fp_t inf = std::numeric_limits<float>::infinity();
    const point_t poc(-2.0, 0.0, 0.0);
    const point_t noc(0.0, -0.7071067812, -0.7071067812);
    mock_physics_object po_a(new inertia_tensor(new fp_t[6]{ inf, inf, inf, inf, inf, inf }, point_t( 1.0, 0.0, -2.3), inf), point_t(0.0, 10.7, 1.3), point_t(0.0, 0.0, 0.0));
    mock_physics_object po_b(new inertia_tensor(new fp_t[6]{ 2.0, 1.2, 8.2, -0.2, 0.7, -3.5 }, point_t(-4.3, 5.7,  0.0), 5.5), point_t(0.0, -4.5, -22.3), point_t(0.0, 0.0, 0.0));
    instantaneous_collide(&po_a, &po_b, poc, noc, cor, mu);

    BOOST_CHECK(po_a.get_collision_normal() == noc);
    BOOST_CHECK(po_a.get_angular_weight() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 3.19932, result_tolerance);

    BOOST_CHECK(fabs(magnitude(po_a.get_collision_normal() - point_t(0., 0.707107, -0.707107))) < result_tolerance);
    BOOST_CHECK(po_a.get_angular_weight() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 0.0, result_tolerance);

    BOOST_CHECK(po_b.get_collision_normal() == noc);
    BOOST_CHECK(fabs(magnitude(po_b.get_angular_weight() - point_t(6.39831, 0.99378, 2.44782))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_b.get_impulse(), -3.19932, result_tolerance);

    BOOST_CHECK(fabs(magnitude(po_b.get_collision_normal() - point_t(0., 0.707107, -0.707107))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b.get_angular_weight() - point_t(9.23708, 0.504338, 4.09794))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_b.get_impulse(), 0.0, result_tolerance);
}


BOOST_AUTO_TEST_CASE( instantaneous_0_mu_off_center_collide_infinite_mass_b_test )
{
    const fp_t mu = 0.0;
    const fp_t cor = 0.35;
    const fp_t inf = std::numeric_limits<float>::infinity();
    const point_t poc(-2.0, 0.0, -1.2);
    const point_t noc(-0.4082482905, -0.8164965809, 0.4082482905);
    mock_physics_object po_a(new inertia_tensor(new fp_t[6]{ 1.0, 1.1, 1.2, 0.0, 0.0, 0.0 }, point_t( 1.0, 0.0, -2.3), 1.9), point_t(0.0, 10.7, 1.3), point_t(0.0, 0.0, 0.0));
    mock_physics_object po_b(new inertia_tensor(new fp_t[6]{ inf, inf, inf, inf, inf, inf }, point_t(-4.3, 5.7,  0.0), inf), point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0));
    instantaneous_collide(&po_a, &po_b, poc, noc, cor, mu);

    BOOST_CHECK(po_a.get_collision_normal() == noc);
    BOOST_CHECK(fabs(magnitude(po_a.get_angular_weight() - point_t(0.898146, 0.705156, 2.04124))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 1.54772, result_tolerance);

    BOOST_CHECK(fabs(magnitude(po_a.get_collision_normal() - point_t(0.47933, -0.572335, -0.665339))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_a.get_angular_weight() - point_t(0.629568, -1.33523, 1.43084))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 0.0, result_tolerance);

    BOOST_CHECK(po_b.get_collision_normal() == noc);
    BOOST_CHECK(po_b.get_angular_weight() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK_CLOSE(po_b.get_impulse(), -1.54772, result_tolerance);

    BOOST_CHECK(fabs(magnitude(po_b.get_collision_normal() - point_t(0.47933, -0.572335, -0.665339))) < result_tolerance);
    BOOST_CHECK(po_b.get_angular_weight() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK_CLOSE(po_b.get_impulse(), 0.0, result_tolerance);
}


/* Test instantaneous collide with rotation */
BOOST_AUTO_TEST_CASE( instantaneous_0_mu_rotating_collide_test )
{
    const fp_t mu = 0.0;
    const fp_t cor = 0.1;
    const point_t poc(-1.0, 3.6, 0.0);
    const point_t noc( 0.7071067812, 0.7071067812, 0.0);
    mock_physics_object po_a(new inertia_tensor(new fp_t[6]{ 9.0, 4.6, 5.6, 4.1, -0.5, 4.6 }, point_t( 1.0, 0.0, -2.3), 9.0), point_t(-5.0, 0.0, 0.0), point_t( 0.0, 3.3, 0.0));
    mock_physics_object po_b(new inertia_tensor(new fp_t[6]{ 2.0, 1.2, 8.2, -0.2, 0.7, -3.5 }, point_t(-4.3, 5.7,  0.0), 5.5), point_t( 7.0, 0.0, 0.0), point_t(-1.5, 0.0, 0.0));
    instantaneous_collide(&po_a, &po_b, poc, noc, cor, mu);

    BOOST_CHECK(po_a.get_collision_normal() == noc);
    BOOST_CHECK(fabs(magnitude(po_a.get_angular_weight() - point_t(0.484855, -0.200698, -1.1233))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 0.180977, result_tolerance);

    BOOST_CHECK(fabs(magnitude(po_a.get_collision_normal() - point_t(0.474149, -0.474149, -0.741866))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_a.get_angular_weight() - point_t(-0.663117, 0.555435, 0.458824))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 0.0, result_tolerance);

    BOOST_CHECK(po_b.get_collision_normal() == noc);
    BOOST_CHECK(fabs(magnitude(po_b.get_angular_weight() - point_t(3.33246, -0.574562, 1.9371))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_b.get_impulse(), -0.180977, result_tolerance);

    BOOST_CHECK(fabs(magnitude(po_b.get_collision_normal() - point_t(0.474149, -0.474149, -0.741866))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b.get_angular_weight() - point_t(2.2084, 1.99833, 0.702633))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_b.get_impulse(), 0.0, result_tolerance);
}


BOOST_AUTO_TEST_CASE( instantaneous_0_mu_rotating_collide_infinite_mass_a_test )
{
    const fp_t mu = 0.0;
    const fp_t cor = 0.46;
    const fp_t inf = std::numeric_limits<float>::infinity();
    const point_t poc(-2.0, 0.0, 0.0);
    const point_t noc(0.0, -0.7071067812, -0.7071067812);
    mock_physics_object po_a(new inertia_tensor(new fp_t[6]{ inf, inf, inf, inf, inf, inf }, point_t( 1.0, 0.0, -2.3), inf), point_t(0.0, 10.7, 1.3), point_t(-7.9, 0.0, 4.0));
    mock_physics_object po_b(new inertia_tensor(new fp_t[6]{ 2.0, 1.2, 8.2, -0.2, 0.7, -3.5 }, point_t(-4.3, 5.7,  0.0), 5.5), point_t(0.0, -4.5, -22.3), point_t(0.0, 10.0, 0.0));
    instantaneous_collide(&po_a, &po_b, poc, noc, cor, mu);

    BOOST_CHECK(po_a.get_collision_normal() == noc);
    BOOST_CHECK(po_a.get_angular_weight() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 5.21191, result_tolerance);

    BOOST_CHECK(fabs(magnitude(po_a.get_collision_normal() - point_t(0.0, 0.707107, -0.707107))) < result_tolerance);
    BOOST_CHECK(po_a.get_angular_weight() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 0.0, result_tolerance);

    BOOST_CHECK(po_b.get_collision_normal() == noc);
    BOOST_CHECK(fabs(magnitude(po_b.get_angular_weight() - point_t(6.39831, 0.99378, 2.44782))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_b.get_impulse(), -5.21191, result_tolerance);

    BOOST_CHECK(fabs(magnitude(po_b.get_collision_normal() - point_t(0.0, 0.707107, -0.707107))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b.get_angular_weight() - point_t(9.23708, 0.504338, 4.09794))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_b.get_impulse(), 0.0, result_tolerance);
}


BOOST_AUTO_TEST_CASE( instantaneous_0_mu_rotating_collide_infinite_mass_b_test )
{
    const fp_t mu = 0.0;
    const fp_t cor = 0.85;
    const fp_t inf = std::numeric_limits<float>::infinity();
    const point_t poc(-2.0, 0.0, -1.2);
    const point_t noc(0.4082482905, 0.8164965809, -0.4082482905);
    mock_physics_object po_a(new inertia_tensor(new fp_t[6]{ 1.0, 1.1, 1.2, 0.0, 0.0, 0.0 }, point_t( 1.0, 0.0, -2.3), 1.9), point_t(-15.0, 10.7, 1.3), point_t(-7.9, 11.0, 4.0));
    mock_physics_object po_b(new inertia_tensor(new fp_t[6]{ inf, inf, inf, inf, inf, inf }, point_t(-4.3, 5.7,  0.0), inf), point_t(-5.6, -4.5, -22.3), point_t(-1.9, 10.0, 14.0));
    instantaneous_collide(&po_a, &po_b, poc, noc, cor, mu);

    BOOST_CHECK(po_a.get_collision_normal() == noc);
    BOOST_CHECK(fabs(magnitude(po_a.get_angular_weight() - point_t(-0.898146, -0.705156, -2.04124))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 17.9309, result_tolerance);

    BOOST_CHECK(fabs(magnitude(po_a.get_collision_normal() - point_t(0.5495, -0.576916, -0.604333))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_a.get_angular_weight() - point_t(0.634608, -1.09868, 1.44229))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 0.0, result_tolerance);

    BOOST_CHECK(po_b.get_collision_normal() == noc);
    BOOST_CHECK(po_b.get_angular_weight() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK_CLOSE(po_b.get_impulse(), -17.9309, result_tolerance);

    BOOST_CHECK(fabs(magnitude(po_b.get_collision_normal() - point_t(0.5495, -0.576916, -0.604333))) < result_tolerance);
    BOOST_CHECK(po_b.get_angular_weight() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK_CLOSE(po_b.get_impulse(), 0.0, result_tolerance);
}


BOOST_AUTO_TEST_CASE( instantaneous_rotationless_collide_test )
{
    const fp_t mu = 0.75;
    const fp_t cor = 1.0;
    const point_t poc(0.0, 0.0, 0.0);
    const point_t noc(1.0, 0.0, 0.0);
    mock_physics_object po_a(new inertia_tensor(new fp_t[6]{ 1.0, 1.1, 1.2, 0.0, 0.0, 0.0 }, point_t( 1.0, 0.0, 0.0), 1.0), point_t(-5.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0));
    mock_physics_object po_b(new inertia_tensor(new fp_t[6]{ 2.9, 2.8, 2.7, 0.0, 0.0, 0.0 }, point_t(-4.3, 0.0, 0.0), 2.5), point_t( 7.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0));
    instantaneous_collide(&po_a, &po_b, poc, noc, cor, mu);

    BOOST_CHECK(po_a.get_collision_normal() == noc);
    BOOST_CHECK(po_a.get_angular_weight() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 17.1429, result_tolerance);

    BOOST_CHECK(po_a.get_collision_normal() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(po_a.get_angular_weight() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK_CLOSE(po_a.get_impulse(), -12.8572, result_tolerance);

    BOOST_CHECK(po_b.get_collision_normal() == noc);
    BOOST_CHECK(po_b.get_angular_weight() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK_CLOSE(po_b.get_impulse(), -17.1429, result_tolerance);

    BOOST_CHECK(po_b.get_collision_normal() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(po_b.get_angular_weight() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK_CLOSE(po_b.get_impulse(), 12.8572, result_tolerance);
}


BOOST_AUTO_TEST_CASE( instantaneous_rotationless_collide_infinite_mass_a_test )
{
    const fp_t mu = 0.5;
    const fp_t cor = 0.5;
    const fp_t inf = std::numeric_limits<float>::infinity();
    const point_t poc(0.0, 0.0, 0.0);
    const point_t noc(1.0, 0.0, 0.0);
    mock_physics_object po_a(new inertia_tensor(new fp_t[6]{ inf, inf, inf, inf, inf, inf }, point_t( 1.0, 0.0, 0.0), inf), point_t(-5.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0));
    mock_physics_object po_b(new inertia_tensor(new fp_t[6]{ 2.9, 2.8, 2.7, 0.0, 0.0, 0.0 }, point_t(-4.3, 0.0, 0.0), 2.5), point_t( 7.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0));
    instantaneous_collide(&po_a, &po_b, poc, noc, cor, mu);

    BOOST_CHECK(po_a.get_collision_normal() == noc);
    BOOST_CHECK(po_a.get_angular_weight() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 45.0, result_tolerance);

    BOOST_CHECK(po_a.get_collision_normal() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(po_a.get_angular_weight() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK_CLOSE(po_a.get_impulse(), -22.5, result_tolerance);

    BOOST_CHECK(po_b.get_collision_normal() == noc);
    BOOST_CHECK(po_b.get_angular_weight() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK_CLOSE(po_b.get_impulse(), -45.0, result_tolerance);

    BOOST_CHECK(po_b.get_collision_normal() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(po_b.get_angular_weight() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK_CLOSE(po_b.get_impulse(), 22.5, result_tolerance);
}


BOOST_AUTO_TEST_CASE( instantaneous_rotationless_collide_infinite_mass_b_test )
{
    const fp_t mu = 0.333;
    const fp_t cor = 0.75;
    const fp_t inf = std::numeric_limits<float>::infinity();
    const point_t poc(0.0, 0.0, 0.0);
    const point_t noc(0.0, 1.0, 0.0);
    mock_physics_object po_a(new inertia_tensor(new fp_t[6]{ 1.0, 1.1, 1.2, 0.0, 0.0, 0.0 }, point_t(0.0,  1.0, 0.0), 1.0), point_t(0.0, -5.0, 0.0), point_t(0.0, 0.0, 0.0));
    mock_physics_object po_b(new inertia_tensor(new fp_t[6]{ inf, inf, inf, inf, inf, inf }, point_t(0.0, -4.3, 0.0), inf), point_t(0.0, 7.0, 0.0), point_t(0.0, 0.0, 0.0));
    instantaneous_collide(&po_a, &po_b, poc, noc, cor, mu);

    BOOST_CHECK(po_a.get_collision_normal() == noc);
    BOOST_CHECK(po_a.get_angular_weight() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 21.0, result_tolerance);

    BOOST_CHECK(po_a.get_collision_normal() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(po_a.get_angular_weight() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK_CLOSE(po_a.get_impulse(), -6.993, result_tolerance);

    BOOST_CHECK(po_b.get_collision_normal() == noc);
    BOOST_CHECK(po_b.get_angular_weight() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK_CLOSE(po_b.get_impulse(), -21.0, result_tolerance);

    BOOST_CHECK(po_b.get_collision_normal() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(po_b.get_angular_weight() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK_CLOSE(po_b.get_impulse(), 6.993, result_tolerance);
}


/* Test instantaneous collide with off center impact (i.e. imparting some rotation) */
BOOST_AUTO_TEST_CASE( instantaneous_off_center_collide_test )
{
    const fp_t mu = 0.75;
    const fp_t cor = 0.81;
    const point_t poc(-1.0, 3.6, 0.0);
    const point_t noc( 0.7071067812, 0.7071067812, 0.0);
    mock_physics_object po_a(new inertia_tensor(new fp_t[6]{ 9.0, 4.6, 5.6, 4.1, -0.5, 4.6 }, point_t( 1.0, 0.0, -2.3), 9.0), point_t(-5.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0));
    mock_physics_object po_b(new inertia_tensor(new fp_t[6]{ 2.0, 1.2, 8.2, 0.0,  0.0, 0.0 }, point_t(-4.3, 5.7,  0.0), 5.5), point_t( 7.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0));
    instantaneous_collide(&po_a, &po_b, poc, noc, cor, mu);

    BOOST_CHECK(po_a.get_collision_normal() == noc);
    BOOST_CHECK(fabs(magnitude(po_a.get_angular_weight() - point_t(0.484855, -0.200698, -1.1233))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 1.2378, result_tolerance);

    BOOST_CHECK(fabs(magnitude(po_a.get_collision_normal() - point_t(0.707107, -0.707107, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_a.get_angular_weight() - point_t(1.38466, -1.03624, -1.43195))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_a.get_impulse(), -0.92835, result_tolerance);

    BOOST_CHECK(po_b.get_collision_normal() == noc);
    BOOST_CHECK(fabs(magnitude(po_b.get_angular_weight() - point_t(0.0, 0.0, 0.465656))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_b.get_impulse(), -1.2378, result_tolerance);

    BOOST_CHECK(fabs(magnitude(po_b.get_collision_normal() - point_t(0.707107, -0.707107, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b.get_angular_weight() - point_t(0.0, 0.0, -0.103479))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_b.get_impulse(), 0.92835, result_tolerance);
}


BOOST_AUTO_TEST_CASE( instantaneous_off_center_collide_infinite_mass_a_test )
{
    const fp_t mu = 0.5;
    const fp_t cor = 0.57;
    const fp_t inf = std::numeric_limits<float>::infinity();
    const point_t poc(-2.0, 0.0, 0.0);
    const point_t noc(0.0, -0.7071067812, -0.7071067812);
    mock_physics_object po_a(new inertia_tensor(new fp_t[6]{ inf, inf, inf, inf, inf, inf }, point_t( 1.0, 0.0, -2.3), inf), point_t(0.0, 10.7, 1.3), point_t(0.0, 0.0, 0.0));
    mock_physics_object po_b(new inertia_tensor(new fp_t[6]{ 2.0, 1.2, 8.2, -0.2, 0.7, -3.5 }, point_t(-4.3, 5.7,  0.0), 5.5), point_t(0.0, -4.5, -22.3), point_t(0.0, 0.0, 0.0));
    instantaneous_collide(&po_a, &po_b, poc, noc, cor, mu);

    BOOST_CHECK(po_a.get_collision_normal() == noc);
    BOOST_CHECK(po_a.get_angular_weight() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 3.19932, result_tolerance);

    BOOST_CHECK(fabs(magnitude(po_a.get_collision_normal() - point_t(0., 0.707107, -0.707107))) < result_tolerance);
    BOOST_CHECK(po_a.get_angular_weight() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK_CLOSE(po_a.get_impulse(), -1.59966, result_tolerance);

    BOOST_CHECK(po_b.get_collision_normal() == noc);
    BOOST_CHECK(fabs(magnitude(po_b.get_angular_weight() - point_t(6.39831, 0.99378, 2.44782))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_b.get_impulse(), -3.19932, result_tolerance);

    BOOST_CHECK(fabs(magnitude(po_b.get_collision_normal() - point_t(0., 0.707107, -0.707107))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b.get_angular_weight() - point_t(9.23708, 0.504338, 4.09794))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_b.get_impulse(), 1.59966, result_tolerance);
}


BOOST_AUTO_TEST_CASE( instantaneous_off_center_collide_infinite_mass_b_test )
{
    const fp_t mu = 0.333;
    const fp_t cor = 0.35;
    const fp_t inf = std::numeric_limits<float>::infinity();
    const point_t poc(-2.0, 0.0, -1.2);
    const point_t noc(-0.4082482905, -0.8164965809, 0.4082482905);
    mock_physics_object po_a(new inertia_tensor(new fp_t[6]{ 1.0, 1.1, 1.2, 0.0, 0.0, 0.0 }, point_t( 1.0, 0.0, -2.3), 1.9), point_t(0.0, 10.7, 1.3), point_t(0.0, 0.0, 0.0));
    mock_physics_object po_b(new inertia_tensor(new fp_t[6]{ inf, inf, inf, inf, inf, inf }, point_t(-4.3, 5.7,  0.0), inf), point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0));
    instantaneous_collide(&po_a, &po_b, poc, noc, cor, mu);

    BOOST_CHECK(po_a.get_collision_normal() == noc);
    BOOST_CHECK(fabs(magnitude(po_a.get_angular_weight() - point_t(0.898146, 0.705156, 2.04124))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 1.54772, result_tolerance);

    BOOST_CHECK(fabs(magnitude(po_a.get_collision_normal() - point_t(0.47933, -0.572335, -0.665339))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_a.get_angular_weight() - point_t(0.629568, -1.33523, 1.43084))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_a.get_impulse(), -0.515391, result_tolerance);

    BOOST_CHECK(po_b.get_collision_normal() == noc);
    BOOST_CHECK(po_b.get_angular_weight() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK_CLOSE(po_b.get_impulse(), -1.54772, result_tolerance);

    BOOST_CHECK(fabs(magnitude(po_b.get_collision_normal() - point_t(0.47933, -0.572335, -0.665339))) < result_tolerance);
    BOOST_CHECK(po_b.get_angular_weight() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK_CLOSE(po_b.get_impulse(), 0.515391, result_tolerance);
}


/* Test instantaneous collide with rotation */
BOOST_AUTO_TEST_CASE( instantaneous_rotating_collide_test )
{
    const fp_t mu = 0.75;
    const fp_t cor = 0.1;
    const point_t poc(-1.0, 3.6, 0.0);
    const point_t noc( 0.7071067812, 0.7071067812, 0.0);
    mock_physics_object po_a(new inertia_tensor(new fp_t[6]{ 9.0, 4.6, 5.6, 4.1, -0.5, 4.6 }, point_t( 1.0, 0.0, -2.3), 9.0), point_t(-5.0, 0.0, 0.0), point_t( 0.0, 3.3, 0.0));
    mock_physics_object po_b(new inertia_tensor(new fp_t[6]{ 2.0, 1.2, 8.2, -0.2, 0.7, -3.5 }, point_t(-4.3, 5.7,  0.0), 5.5), point_t( 7.0, 0.0, 0.0), point_t(-1.5, 0.0, 0.0));
    instantaneous_collide(&po_a, &po_b, poc, noc, cor, mu);

    BOOST_CHECK(po_a.get_collision_normal() == noc);
    BOOST_CHECK(fabs(magnitude(po_a.get_angular_weight() - point_t(0.484855, -0.200698, -1.1233))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 0.180977, result_tolerance);

    BOOST_CHECK(fabs(magnitude(po_a.get_collision_normal() - point_t(0.474149, -0.474149, -0.741866))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_a.get_angular_weight() - point_t(-0.663117, 0.555435, 0.458824))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_a.get_impulse(), -0.135733, result_tolerance);

    BOOST_CHECK(po_b.get_collision_normal() == noc);
    BOOST_CHECK(fabs(magnitude(po_b.get_angular_weight() - point_t(3.33246, -0.574562, 1.9371))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_b.get_impulse(), -0.180977, result_tolerance);

    BOOST_CHECK(fabs(magnitude(po_b.get_collision_normal() - point_t(0.474149, -0.474149, -0.741866))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b.get_angular_weight() - point_t(2.2084, 1.99833, 0.702633))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_b.get_impulse(), 0.135733, result_tolerance);
}


BOOST_AUTO_TEST_CASE( instantaneous_rotating_collide_infinite_mass_a_test )
{
    const fp_t mu = 0.5;
    const fp_t cor = 0.46;
    const fp_t inf = std::numeric_limits<float>::infinity();
    const point_t poc(-2.0, 0.0, 0.0);
    const point_t noc(0.0, -0.7071067812, -0.7071067812);
    mock_physics_object po_a(new inertia_tensor(new fp_t[6]{ inf, inf, inf, inf, inf, inf }, point_t( 1.0, 0.0, -2.3), inf), point_t(0.0, 10.7, 1.3), point_t(-7.9, 0.0, 4.0));
    mock_physics_object po_b(new inertia_tensor(new fp_t[6]{ 2.0, 1.2, 8.2, -0.2, 0.7, -3.5 }, point_t(-4.3, 5.7,  0.0), 5.5), point_t(0.0, -4.5, -22.3), point_t(0.0, 10.0, 0.0));
    instantaneous_collide(&po_a, &po_b, poc, noc, cor, mu);

    BOOST_CHECK(po_a.get_collision_normal() == noc);
    BOOST_CHECK(po_a.get_angular_weight() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 5.21191, result_tolerance);

    BOOST_CHECK(fabs(magnitude(po_a.get_collision_normal() - point_t(0.0, 0.707107, -0.707107))) < result_tolerance);
    BOOST_CHECK(po_a.get_angular_weight() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK_CLOSE(po_a.get_impulse(), -2.60596, result_tolerance);

    BOOST_CHECK(po_b.get_collision_normal() == noc);
    BOOST_CHECK(fabs(magnitude(po_b.get_angular_weight() - point_t(6.39831, 0.99378, 2.44782))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_b.get_impulse(), -5.21191, result_tolerance);

    BOOST_CHECK(fabs(magnitude(po_b.get_collision_normal() - point_t(0.0, 0.707107, -0.707107))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b.get_angular_weight() - point_t(9.23708, 0.504338, 4.09794))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_b.get_impulse(), 2.60596, result_tolerance);
}


BOOST_AUTO_TEST_CASE( instantaneous_rotating_collide_infinite_mass_b_test )
{
    const fp_t mu = 0.333;
    const fp_t cor = 0.85;
    const fp_t inf = std::numeric_limits<float>::infinity();
    const point_t poc(-2.0, 0.0, -1.2);
    const point_t noc(0.4082482905, 0.8164965809, -0.4082482905);
    mock_physics_object po_a(new inertia_tensor(new fp_t[6]{ 1.0, 1.1, 1.2, 0.0, 0.0, 0.0 }, point_t( 1.0, 0.0, -2.3), 1.9), point_t(-15.0, 10.7, 1.3), point_t(-7.9, 11.0, 4.0));
    mock_physics_object po_b(new inertia_tensor(new fp_t[6]{ inf, inf, inf, inf, inf, inf }, point_t(-4.3, 5.7,  0.0), inf), point_t(-5.6, -4.5, -22.3), point_t(-1.9, 10.0, 14.0));
    instantaneous_collide(&po_a, &po_b, poc, noc, cor, mu);

    BOOST_CHECK(po_a.get_collision_normal() == noc);
    BOOST_CHECK(fabs(magnitude(po_a.get_angular_weight() - point_t(-0.898146, -0.705156, -2.04124))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_a.get_impulse(), 17.9309, result_tolerance);

    BOOST_CHECK(fabs(magnitude(po_a.get_collision_normal() - point_t(0.5495, -0.576916, -0.604333))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_a.get_angular_weight() - point_t(0.634608, -1.09868, 1.44229))) < result_tolerance);
    BOOST_CHECK_CLOSE(po_a.get_impulse(), -5.97099, result_tolerance);

    BOOST_CHECK(po_b.get_collision_normal() == noc);
    BOOST_CHECK(po_b.get_angular_weight() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK_CLOSE(po_b.get_impulse(), -17.9309, result_tolerance);

    BOOST_CHECK(fabs(magnitude(po_b.get_collision_normal() - point_t(0.5495, -0.576916, -0.604333))) < result_tolerance);
    BOOST_CHECK(po_b.get_angular_weight() == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK_CLOSE(po_b.get_impulse(), 5.97099, result_tolerance);
}


/* Test rigid body collider class */
BOOST_AUTO_TEST_CASE( rigid_body_collider_class_test )
{
    const point_t poc(0.5, 1.0, 0.5);
    const point_t noc(0.0, 1.0, 0.0);
    std::unique_ptr<raptor_raytracer::material> m(new raptor_raytracer::phong_shader(raptor_raytracer::ext_colour_t(255, 255, 255), 1.0));
    std::unique_ptr<physics_object> po_a(new physics_object(make_cube(m.get(), point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5)), point_t(0.5, 0.5, 0.5), 1.0));
    std::unique_ptr<physics_object> po_b(new physics_object(make_cube(m.get(), point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5)), point_t(0.5, 1.5, 0.5), 1.0));

    std::unique_ptr<collider> uut(new rigid_body_collider(0.1, 0.3));

    /* Collide and check */
    po_a->set_velocity(point_t(-10.0,  10.0, 0.0));
    po_b->set_velocity(point_t( 10.0, -10.0, 0.0));

    uut->collide(po_a.get(), po_b.get(), poc, noc, COLLISION);
    BOOST_CHECK(fabs(magnitude(po_a->get_velocity() - point_t(-6.7, -1.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_a->get_angular_velocity() - point_t(0.0,  0.0, -9.9))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_a->get_velocity(poc) - point_t(-1.75, -1.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b->get_velocity() - point_t(6.7, 1.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b->get_angular_velocity() - point_t(0.0, 0.0, -9.9))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b->get_velocity(poc) - point_t(1.75, 1.0, 0.0))) < result_tolerance);

    /* Collide and check */
    po_a->set_velocity(point_t(-10.0,  10.0, 0.0));
    po_b->set_velocity(point_t( 10.0, -10.0, 0.0));
    po_a->set_angular_velocity(point_t(0.0, 0.0, 0.0));
    po_b->set_angular_velocity(point_t(0.0, 0.0, 0.0));
    
    uut->collide(po_b.get(), po_a.get(), poc, -noc, COLLISION);
    BOOST_CHECK(fabs(magnitude(po_a->get_velocity() - point_t(-6.7, -1.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_a->get_angular_velocity() - point_t(0.0,  0.0, -9.9))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_a->get_velocity(poc) - point_t(-1.75, -1.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b->get_velocity() - point_t(6.7, 1.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b->get_angular_velocity() - point_t(0.0, 0.0, -9.9))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b->get_velocity(poc) - point_t(1.75, 1.0, 0.0))) < result_tolerance);
}


BOOST_AUTO_TEST_CASE( rigid_body_collider_class_reverse_test )
{
    const point_t poc(0.5, 1.0, 0.5);
    const point_t noc(0.0, 1.0, 0.0);
    std::unique_ptr<raptor_raytracer::material> m(new raptor_raytracer::phong_shader(raptor_raytracer::ext_colour_t(255, 255, 255), 1.0));
    std::unique_ptr<physics_object> po_a(new physics_object(make_cube(m.get(), point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5)), point_t(0.5, 0.5, 0.5), 1.0));
    std::unique_ptr<physics_object> po_b(new physics_object(make_cube(m.get(), point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5)), point_t(0.5, 1.5, 0.5), 1.0));

    std::unique_ptr<collider> uut(new rigid_body_collider(0.1, 0.3));

    /* Collide and check */
    po_a->set_velocity(point_t( 10.0,  10.0, 0.0));
    po_b->set_velocity(point_t(-10.0, -10.0, 0.0));

    uut->collide(po_a.get(), po_b.get(), poc, noc, COLLISION);
    BOOST_CHECK(fabs(magnitude(po_a->get_velocity() - point_t(6.7, -1.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_a->get_angular_velocity() - point_t(0.0,  0.0, 9.9))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_a->get_velocity(poc) - point_t(1.75, -1.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b->get_velocity() - point_t(-6.7, 1.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b->get_angular_velocity() - point_t(0.0, 0.0, 9.9))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b->get_velocity(poc) - point_t(-1.75, 1.0, 0.0))) < result_tolerance);

    /* Collide and check */
    po_a->set_velocity(point_t( 10.0,  10.0, 0.0));
    po_b->set_velocity(point_t(-10.0, -10.0, 0.0));
    po_a->set_angular_velocity(point_t(0.0, 0.0, 0.0));
    po_b->set_angular_velocity(point_t(0.0, 0.0, 0.0));
    
    uut->collide(po_b.get(), po_a.get(), poc, -noc, COLLISION);
    BOOST_CHECK(fabs(magnitude(po_a->get_velocity() - point_t(6.7, -1.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_a->get_angular_velocity() - point_t(0.0,  0.0, 9.9))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_a->get_velocity(poc) - point_t(1.75, -1.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b->get_velocity() - point_t(-6.7, 1.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b->get_angular_velocity() - point_t(0.0, 0.0, 9.9))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b->get_velocity(poc) - point_t(-1.75, 1.0, 0.0))) < result_tolerance);
}


BOOST_AUTO_TEST_CASE( rigid_body_collider_class_more_friction_test )
{
    const point_t poc(0.5, 1.0, 0.5);
    const point_t noc(0.0, 1.0, 0.0);
    std::unique_ptr<raptor_raytracer::material> m(new raptor_raytracer::phong_shader(raptor_raytracer::ext_colour_t(255, 255, 255), 1.0));
    std::unique_ptr<physics_object> po_a(new physics_object(make_cube(m.get(), point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5)), point_t(0.5, 0.5, 0.5), 1.0));
    std::unique_ptr<physics_object> po_b(new physics_object(make_cube(m.get(), point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5)), point_t(0.5, 1.5, 0.5), 1.0));
    
    std::unique_ptr<collider> uut(new rigid_body_collider(0.1, 0.35));

    /* Collide and check */
    po_a->set_velocity(point_t(-10.0,  10.0, 0.0));
    po_b->set_velocity(point_t( 10.0, -10.0, 0.0));

    uut->collide(po_a.get(), po_b.get(), poc, noc, COLLISION);
    BOOST_CHECK(fabs(magnitude(po_a->get_velocity() - point_t(-6.15, -1.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_a->get_angular_velocity() - point_t(0.0,  0.0, -11.55))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_a->get_velocity(poc) - point_t(-0.375, -1.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b->get_velocity() - point_t(6.15, 1.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b->get_angular_velocity() - point_t(0.0, 0.0, -11.55))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b->get_velocity(poc) - point_t(0.375, 1.0, 0.0))) < result_tolerance);

    /* Collide and check */
    po_a->set_velocity(point_t(-10.0,  10.0, 0.0));
    po_b->set_velocity(point_t( 10.0, -10.0, 0.0));
    po_a->set_angular_velocity(point_t(0.0, 0.0, 0.0));
    po_b->set_angular_velocity(point_t(0.0, 0.0, 0.0));
    
    uut->collide(po_b.get(), po_a.get(), poc, -noc, COLLISION);
    BOOST_CHECK(fabs(magnitude(po_a->get_velocity() - point_t(-6.15, -1.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_a->get_angular_velocity() - point_t(0.0,  0.0, -11.55))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_a->get_velocity(poc) - point_t(-0.375, -1.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b->get_velocity() - point_t(6.15, 1.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b->get_angular_velocity() - point_t(0.0, 0.0, -11.55))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b->get_velocity(poc) - point_t(0.375, 1.0, 0.0))) < result_tolerance);
}


BOOST_AUTO_TEST_CASE( rigid_body_collider_class_max_friction_test )
{
    const point_t poc(0.5, 1.0, 0.5);
    const point_t noc(0.0, 1.0, 0.0);
    std::unique_ptr<raptor_raytracer::material> m(new raptor_raytracer::phong_shader(raptor_raytracer::ext_colour_t(255, 255, 255), 1.0));
    std::unique_ptr<physics_object> po_a(new physics_object(make_cube(m.get(), point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5)), point_t(0.5, 0.5, 0.5), 1.0));
    std::unique_ptr<physics_object> po_b(new physics_object(make_cube(m.get(), point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5)), point_t(0.5, 1.5, 0.5), 1.0));

    std::unique_ptr<collider> uut(new rigid_body_collider(0.1, 0.36363));

    /* Collide and check */
    po_a->set_velocity(point_t(-10.0,  10.0, 0.0));
    po_b->set_velocity(point_t( 10.0, -10.0, 0.0));
    uut->collide(po_a.get(), po_b.get(), poc, noc, COLLISION);
    BOOST_CHECK(fabs(magnitude(po_a->get_velocity() - point_t(-6.00007, -1.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_a->get_angular_velocity() - point_t(0.0,  0.0, -11.9998))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_a->get_velocity(poc) - point_t(-0.000175476, -1.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b->get_velocity() - point_t(6.00007, 1.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b->get_angular_velocity() - point_t(0.0, 0.0, -11.9998))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b->get_velocity(poc) - point_t(0.000175476, 1.0, 0.0))) < result_tolerance);

    /* Collide and check */
    po_a->set_velocity(point_t(-10.0,  10.0, 0.0));
    po_b->set_velocity(point_t( 10.0, -10.0, 0.0));
    po_a->set_angular_velocity(point_t(0.0, 0.0, 0.0));
    po_b->set_angular_velocity(point_t(0.0, 0.0, 0.0));
    
    uut->collide(po_b.get(), po_a.get(), poc, -noc, COLLISION);
    BOOST_CHECK(fabs(magnitude(po_a->get_velocity() - point_t(-6.00007, -1.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_a->get_angular_velocity() - point_t(0.0,  0.0, -11.9998))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_a->get_velocity(poc) - point_t(-0.000175476, -1.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b->get_velocity() - point_t(6.00007, 1.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b->get_angular_velocity() - point_t(0.0, 0.0, -11.9998))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b->get_velocity(poc) - point_t(0.000175476, 1.0, 0.0))) < result_tolerance);
}


BOOST_AUTO_TEST_CASE( rigid_body_collider_class_capped_max_friction_test )
{
    const point_t poc(0.5, 1.0, 0.5);
    const point_t noc(0.0, 1.0, 0.0);
    std::unique_ptr<raptor_raytracer::material> m(new raptor_raytracer::phong_shader(raptor_raytracer::ext_colour_t(255, 255, 255), 1.0));
    std::unique_ptr<physics_object> po_a(new physics_object(make_cube(m.get(), point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5)), point_t(0.5, 0.5, 0.5), 1.0));
    std::unique_ptr<physics_object> po_b(new physics_object(make_cube(m.get(), point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5)), point_t(0.5, 1.5, 0.5), 1.0));

    std::unique_ptr<collider> uut(new rigid_body_collider(0.1, 0.5));
    
    /* Collide and check */
    po_a->set_velocity(point_t(-10.0,  10.0, 0.0));
    po_b->set_velocity(point_t( 10.0, -10.0, 0.0));

    uut->collide(po_a.get(), po_b.get(), poc, noc, COLLISION);
    BOOST_CHECK(fabs(magnitude(po_a->get_velocity() - point_t(-6.0, -1.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_a->get_angular_velocity() - point_t(0.0,  0.0, -12.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_a->get_velocity(poc) - point_t(0.0, -1.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b->get_velocity() - point_t(6.0, 1.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b->get_angular_velocity() - point_t(0.0, 0.0, -12.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b->get_velocity(poc) - point_t(0.0, 1.0, 0.0))) < result_tolerance);

    /* Collide and check */
    po_a->set_velocity(point_t(-10.0,  10.0, 0.0));
    po_b->set_velocity(point_t( 10.0, -10.0, 0.0));
    po_a->set_angular_velocity(point_t(0.0, 0.0, 0.0));
    po_b->set_angular_velocity(point_t(0.0, 0.0, 0.0));

    uut->collide(po_b.get(), po_a.get(), poc, -noc, COLLISION);
    BOOST_CHECK(fabs(magnitude(po_a->get_velocity() - point_t(-6.0, -1.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_a->get_angular_velocity() - point_t(0.0,  0.0, -12.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_a->get_velocity(poc) - point_t(0.0, -1.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b->get_velocity() - point_t(6.0, 1.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b->get_angular_velocity() - point_t(0.0, 0.0, -12.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b->get_velocity(poc) - point_t(0.0, 1.0, 0.0))) < result_tolerance);
}


BOOST_AUTO_TEST_CASE( rigid_body_collider_class_zero_tanjent_velocity_test )
{
    const point_t poc(0.5, 1.0, 0.5);
    const point_t noc(0.0, 1.0, 0.0);
    std::unique_ptr<raptor_raytracer::material> m(new raptor_raytracer::phong_shader(raptor_raytracer::ext_colour_t(255, 255, 255), 1.0));
    std::unique_ptr<physics_object> po_a(new physics_object(make_cube(m.get(), point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5)), point_t(0.5, 0.5, 0.5), 1.0));
    std::unique_ptr<physics_object> po_b(new physics_object(make_cube(m.get(), point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5)), point_t(0.5, 1.5, 0.5), 1.0));

    std::unique_ptr<collider> uut(new rigid_body_collider(0.1, 0.3));
    
    /* Collide and check */
    po_a->set_velocity(point_t(0.0,  10.0, 0.0));
    po_b->set_velocity(point_t(0.0, -10.0, 0.0));

    uut->collide(po_a.get(), po_b.get(), poc, noc, COLLISION);
    BOOST_CHECK(fabs(magnitude(po_a->get_velocity() - point_t(0.0, -1.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_a->get_angular_velocity() - point_t(0.0,  0.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_a->get_velocity(poc) - point_t(0.0, -1.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b->get_velocity() - point_t(0.0, 1.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b->get_angular_velocity() - point_t(0.0, 0.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b->get_velocity(poc) - point_t(0.0, 1.0, 0.0))) < result_tolerance);

    /* Collide and check */
    po_a->set_velocity(point_t(0.0,  10.0, 0.0));
    po_b->set_velocity(point_t(0.0, -10.0, 0.0));
    po_a->set_angular_velocity(point_t(0.0, 0.0, 0.0));
    po_b->set_angular_velocity(point_t(0.0, 0.0, 0.0));

    uut->collide(po_b.get(), po_a.get(), poc, -noc, COLLISION);
    BOOST_CHECK(fabs(magnitude(po_a->get_velocity() - point_t(0.0, -1.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_a->get_angular_velocity() - point_t(0.0,  0.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_a->get_velocity(poc) - point_t(0.0, -1.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b->get_velocity() - point_t(0.0, 1.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b->get_angular_velocity() - point_t(0.0, 0.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b->get_velocity(poc) - point_t(0.0, 1.0, 0.0))) < result_tolerance);
}


BOOST_AUTO_TEST_CASE( rigid_body_collider_class_zero_mu_test )
{
    const point_t poc(0.5, 1.0, 0.5);
    const point_t noc(0.0, 1.0, 0.0);
    std::unique_ptr<raptor_raytracer::material> m(new raptor_raytracer::phong_shader(raptor_raytracer::ext_colour_t(255, 255, 255), 1.0));
    std::unique_ptr<physics_object> po_a(new physics_object(make_cube(m.get(), point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5)), point_t(0.5, 0.5, 0.5), 1.0));
    std::unique_ptr<physics_object> po_b(new physics_object(make_cube(m.get(), point_t(-0.5, -0.5, -0.5), point_t(0.5, 0.5, 0.5)), point_t(0.5, 1.5, 0.5), 1.0));

    std::unique_ptr<collider> uut(new rigid_body_collider(0.0, 0.3));
    
    /* Collide and check */
    po_a->set_velocity(point_t(-10.0,  10.0, 0.0));
    po_b->set_velocity(point_t( 10.0, -10.0, 0.0));

    uut->collide(po_a.get(), po_b.get(), poc, noc, COLLISION);
    BOOST_CHECK(fabs(magnitude(po_a->get_velocity() - point_t(-7.0, 0.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_a->get_angular_velocity() - point_t(0.0, 0.0, -9.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_a->get_velocity(poc) - point_t(-2.5, 0.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b->get_velocity() - point_t(7.0, 0.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b->get_angular_velocity() - point_t(0.0, 0.0, -9.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b->get_velocity(poc) - point_t(2.5, 0.0, 0.0))) < result_tolerance);

    /* Collide and check */
    po_a->set_velocity(point_t(-10.0,  10.0, 0.0));
    po_b->set_velocity(point_t( 10.0, -10.0, 0.0));
    po_a->set_angular_velocity(point_t(0.0, 0.0, 0.0));
    po_b->set_angular_velocity(point_t(0.0, 0.0, 0.0));

    uut->collide(po_b.get(), po_a.get(), poc, -noc, COLLISION);
    BOOST_CHECK(fabs(magnitude(po_a->get_velocity() - point_t(-7.0, 0.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_a->get_angular_velocity() - point_t(0.0, 0.0, -9.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_a->get_velocity(poc) - point_t(-2.5, 0.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b->get_velocity() - point_t(7.0, 0.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b->get_angular_velocity() - point_t(0.0, 0.0, -9.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(po_b->get_velocity(poc) - point_t(2.5, 0.0, 0.0))) < result_tolerance);
}

BOOST_AUTO_TEST_SUITE_END()
}; /* namespace test */
}; /* namespace raptor_physics */
