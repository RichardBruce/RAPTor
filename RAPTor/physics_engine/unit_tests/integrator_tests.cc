#ifdef STAND_ALONE
#define BOOST_TEST_MODULE integrator test

/* Common headers */
#include "logging.h"

/* Initialise logger */
const raptor_physics::init_logger init_logger;
#endif /* #ifdef STAND_ALONE */

/* Boost headers */
#include "boost/test/unit_test.hpp"

/* Physics headers */
#include "point_t.h"
#include "inertia_tensor.h"
#include "integrators.h"
#include "force.h"


namespace raptor_physics
{
namespace test
{
/* Test data */
struct integrator_fixture
{
    integrator_fixture()
    : i0(new fp_t[6]{ 2.66667, 2.66667, 2.66667, 0.0,  0.0, 0.0 }, point_t(            0.0,  -0.5,            0.0  ), 4.0),
      i1(new fp_t[6]{ 1.33333, 1.33333, 1.33333, 0.0,  0.0, 0.0 }, point_t( 1.0 / sqrt(2.0),  0.0, 1.0 / sqrt(2.0) ), 2.0),
      i2(new fp_t[6]{ std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity(), 0.0,  0.0, 0.0 }, point_t( 0.0, 0.0, 0.0), std::numeric_limits<float>::infinity()),
      i3(new fp_t[6]{ 4.0, 4.0, 4.0, 0.0,  0.0, 0.0 }, point_t( -1.0, 1.0, -1.0 ), 6.0),
      no_force_vec(),
      no_force(no_force_vec)
    {  };

    point_t             v1;
    inertia_tensor      i0;
    inertia_tensor      i1;
    inertia_tensor      i2;
    inertia_tensor      i3;
    std::vector<force*> no_force_vec;
    aggregate_force     no_force;
    euler_integrator    euler_uut;
    rk4_integrator      rk4_uut;
};


BOOST_FIXTURE_TEST_SUITE( integrator_tests, integrator_fixture )

const float result_tolerance = 0.0005;


/* Test with velocity, no applied forces */
BOOST_AUTO_TEST_CASE( basic_velocity_euler_test )
{
    BOOST_CHECK(euler_uut.project_translation(no_force, i0, &v1, point_t(1.0, 1.0, 1.0), 1.0) == point_t(1.0, 1.0, 1.0));
    BOOST_CHECK(v1 == point_t(1.0, 1.0, 1.0));

    BOOST_CHECK(euler_uut.project_translation(no_force, i1, &v1, point_t(1.0, -1.0, 1.0), 1.0) == point_t(1.0, -1.0, 1.0));
    BOOST_CHECK(v1 == point_t(1.0, -1.0, 1.0));

    BOOST_CHECK(euler_uut.project_translation(no_force, i2, &v1, point_t(-1.0, 1.0, -1.0), 1.0) == point_t(-1.0, 1.0, -1.0));
    BOOST_CHECK(v1 == point_t(-1.0, 1.0, -1.0));
}


BOOST_AUTO_TEST_CASE( timestep_velocity_euler_test )
{
    BOOST_CHECK(euler_uut.project_translation(no_force, i0, &v1, point_t(1.0, 1.0, 1.0), 0.5) == point_t(0.5, 0.5, 0.5));
    BOOST_CHECK(v1 == point_t(1.0, 1.0, 1.0));

    BOOST_CHECK(euler_uut.project_translation(no_force, i1, &v1, point_t(1.0, -1.0, 1.0), 0.0) == point_t(0.0, -0.0, 0.0));
    BOOST_CHECK(v1 == point_t(1.0, -1.0, 1.0));

    BOOST_CHECK(euler_uut.project_translation(no_force, i2, &v1, point_t(-1.0, 1.0, -1.0), 0.75) == point_t(-0.75, 0.75, -0.75));
    BOOST_CHECK(v1 == point_t(-1.0, 1.0, -1.0));
}


BOOST_AUTO_TEST_CASE( basic_angular_velocity_euler_test )
{
    /* These rotations are all much to big for the linear approximation used, hence we see some strange quaternions */
    const quaternion_t start(1.0, 0.0, 0.0, 0.0);
    BOOST_CHECK(fabs(magnitude(euler_uut.project_rotation(no_force, i0, &v1, start, point_t(PI, 0.0, 0.0), 1.0) - quaternion_t(1.0, 1.5708, 0.0, 0.0) + start)) < result_tolerance);
    BOOST_CHECK(v1 == point_t(PI, 0.0, 0.0));

    BOOST_CHECK(fabs(magnitude(euler_uut.project_rotation(no_force, i1, &v1, start, point_t(0.0, -PI, 0.0), 1.0) - quaternion_t(1.0, 0.0, -1.5708, 0.0) + start)) < result_tolerance);
    BOOST_CHECK(v1 == point_t(0.0, -PI, 0.0));

    BOOST_CHECK(fabs(magnitude(euler_uut.project_rotation(no_force, i2, &v1, start, point_t(0.0, 0.0, -PI), 1.0) - quaternion_t(1.0, 0.0, 0.0, -1.5708) + start)) < result_tolerance);
    BOOST_CHECK(v1 == point_t(0.0, 0.0, -PI));
}


BOOST_AUTO_TEST_CASE( timestep_angular_velocity_euler_test )
{
    const quaternion_t start(1.0, 0.0, 0.0, 0.0);
    BOOST_CHECK(fabs(magnitude(euler_uut.project_rotation(no_force, i0, &v1, start, point_t(PI, 0.0, 0.0), 0.5) - quaternion_t(1.0, 0.785398, 0.0, 0.0) + start)) < result_tolerance);
    BOOST_CHECK(v1 == point_t(PI, 0.0, 0.0));

    BOOST_CHECK(fabs(magnitude(euler_uut.project_rotation(no_force, i1, &v1, start, point_t(0.0, -PI, 0.0), 0.0) - quaternion_t(1.0, 0.0, 0.0, 0.0) + start)) < result_tolerance);
    BOOST_CHECK(v1 == point_t(0.0, -PI, 0.0));

    BOOST_CHECK(fabs(magnitude(euler_uut.project_rotation(no_force, i2, &v1, start, point_t(0.0, -PI), 0.75) - quaternion_t(1.0, 0.0, -1.1781, 0.0) + start)) < result_tolerance);
    BOOST_CHECK(v1 == point_t(0.0, -PI));
}


/* Constant force tests */
BOOST_AUTO_TEST_CASE( basic_const_force_euler_test )
{
    std::unique_ptr<const_force> cf0(new const_force(point_t(0.0, 0.0, 0.0), point_t(2.0, -4.0, 10.0), 5.0));
    std::vector<force *> forces({cf0.get()});
    aggregate_force f(forces);

    BOOST_CHECK(euler_uut.project_translation(f, i0, &v1, point_t(1.0, 1.0, 1.0), 1.0) == point_t(1.25, 0.5, 2.25));
    BOOST_CHECK(v1 == point_t(1.5, 0.0, 3.5));

    BOOST_CHECK(euler_uut.project_translation(f, i1, &v1, point_t(1.0, -1.0, 1.0), 1.0) == point_t(1.5, -2.0, 3.5));
    BOOST_CHECK(v1 == point_t(2.0, -3.0, 6.0));

    BOOST_CHECK(euler_uut.project_translation(f, i2, &v1, point_t(-1.0, 1.0, -1.0), 1.0) == point_t(-1.0, 1.0, -1.0));
    BOOST_CHECK(v1 == point_t(-1.0, 1.0, -1.0));
}


BOOST_AUTO_TEST_CASE( timestep_const_force_euler_test )
{
    std::unique_ptr<const_force> cf0(new const_force(point_t(0.0, 0.0, 0.0), point_t(2.0, -4.0, 10.0), 5.0));
    std::vector<force *> forces({cf0.get()});
    aggregate_force f(forces);

    BOOST_CHECK(euler_uut.project_translation(f, i0, &v1, point_t(1.0, 1.0, 1.0), 0.5) == point_t(0.5625, 0.375, 0.8125));
    BOOST_CHECK(v1 == point_t(1.25, 0.5, 2.25));

    BOOST_CHECK(euler_uut.project_translation(f, i1, &v1, point_t(1.0, -1.0, 1.0), 0.0) == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(v1 == point_t(1.0, -1.0, 1.0));

    BOOST_CHECK(euler_uut.project_translation(f, i2, &v1, point_t(-1.0, 1.0, -1.0), 0.75) == point_t(-0.75, 0.75, -0.75));
    BOOST_CHECK(v1 == point_t(-1.0, 1.0, -1.0));
}


BOOST_AUTO_TEST_CASE( basic_const_torque_euler_test )
{
    std::unique_ptr<const_force> cf0(new const_force(point_t(1.0, 0.0, 0.0), point_t(0.0, -0.5 * PI, 0.0), 5.0));
    std::vector<force *> forces({cf0.get()});
    aggregate_force f(forces);

    /* These rotations are all much to big for the linear approximation used, hence we see some strange quaternions */
    const quaternion_t start(1.0, 0.0, 0.0, 0.0);
    BOOST_CHECK(fabs(magnitude(euler_uut.project_rotation(f, i0, &v1, start, point_t(PI, 0.0, 0.0), 1.0) - quaternion_t(1.0, 1.5708, 0.0, -0.147262) + start)) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(v1 - point_t(PI, 0.0, -0.589))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(euler_uut.project_rotation(f, i1, &v1, start, point_t(0.0, -PI, 0.0), 1.0) - quaternion_t(1.0, 0.0, -1.5708, -0.294525) + start)) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(v1 - point_t(0.0, -PI, -1.1781))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(euler_uut.project_rotation(f, i2, &v1, start, point_t(0.0, 0.0, -PI), 1.0) - quaternion_t(1.0, 0.0, 0.0, -1.5708) + start)) < result_tolerance);
    BOOST_CHECK(v1 == point_t(0.0, 0.0, -PI));
}


BOOST_AUTO_TEST_CASE( timestep_const_torque_euler_test )
{
    std::unique_ptr<const_force> cf0(new const_force(point_t(1.0, 0.0, 0.0), point_t(0.0, -0.5 * PI, 0.0), 5.0));
    std::vector<force *> forces({cf0.get()});
    aggregate_force f(forces);

    const quaternion_t start(1.0, 0.0, 0.0, 0.0);
    BOOST_CHECK(fabs(magnitude(euler_uut.project_rotation(f, i0, &v1, start, point_t(PI, 0.0, 0.0), 0.5) - quaternion_t(1, 0.785398, 0, -0.0368155) + start)) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(v1 - point_t(3.14159, 0, -0.294524))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(euler_uut.project_rotation(f, i1, &v1, start, point_t(0.0, -PI, 0.0), 0.0) - quaternion_t(1.0, 0.0, 0.0, 0.0) + start)) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(v1 - point_t(0.0, -PI, 0.0))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(euler_uut.project_rotation(f, i2, &v1, start, point_t(0.0, -PI), 0.75) - quaternion_t(1.0, 0.0, -1.1781, 0.0) + start)) < result_tolerance);
    BOOST_CHECK(v1 == point_t(0.0, -PI));
}


/* Linear force tests */
/* Note -- Euler is no longer exact */
/* Note -- Dropped test on infinite mass object, too boring */
BOOST_AUTO_TEST_CASE( basic_linear_force_euler_test )
{
    std::unique_ptr<linear_force> lf0(new linear_force(point_t(0.0, 0.0, 0.0), point_t(-1.0, 4.0, -10.0), point_t(2.0, -4.0, 10.0), 5.0));
    std::vector<force *> forces({lf0.get()});
    aggregate_force f(forces);

    BOOST_CHECK_MESSAGE(fabs(magnitude(euler_uut.project_translation(f, i0, &v1, point_t(1.0, 1.0, 1.0), 1.0) - point_t(1.25, 0.5, 2.25))) < result_tolerance, euler_uut.project_translation(f, i0, &v1, point_t(1.0, 1.0, 1.0), 1.0));
    BOOST_CHECK_MESSAGE(fabs(magnitude(v1 - point_t(1.5, 0.0, 3.5))) < result_tolerance, v1);

    BOOST_CHECK_MESSAGE(fabs(magnitude(euler_uut.project_translation(f, i1, &v1, point_t(1.0, -1.0, 1.0), 1.0) - point_t(1.5, -2.0, 3.5))) < result_tolerance, euler_uut.project_translation(f, i1, &v1, point_t(1.0, -1.0, 1.0), 1.0));
    BOOST_CHECK_MESSAGE(fabs(magnitude(v1 - point_t(2.0, -3.0, 6.0))) < result_tolerance, v1);

    BOOST_CHECK_MESSAGE(fabs(magnitude(euler_uut.project_translation(f, i3, &v1, point_t(-1.0, 1.0, -1.0), 1.0) - point_t(-0.833333, 0.666667, -0.166667))) < result_tolerance, euler_uut.project_translation(f, i3, &v1, point_t(-1.0, 1.0, -1.0), 1.0));
    BOOST_CHECK_MESSAGE(fabs(magnitude(v1 - point_t(-0.666667, 0.333333, 0.666667))) < result_tolerance, v1);
}


BOOST_AUTO_TEST_CASE( timestep_linear_force_euler_test )
{
    std::unique_ptr<linear_force> lf0(new linear_force(point_t(0.0, 0.0, 0.0), point_t(-1.0, 4.0, -10.0), point_t(2.0, -4.0, 10.0), 5.0));
    std::vector<force *> forces({lf0.get()});
    aggregate_force f(forces);

    BOOST_CHECK_MESSAGE(fabs(magnitude(euler_uut.project_translation(f, i0, &v1, point_t(1.0, 1.0, 1.0), 0.5) - point_t(0.5625, 0.375, 0.8125))) < result_tolerance, euler_uut.project_translation(f, i0, &v1, point_t(1.0, 1.0, 1.0), 0.5));
    BOOST_CHECK_MESSAGE(fabs(magnitude(v1 - point_t(1.25, 0.5, 2.25))) < result_tolerance, v1);

    BOOST_CHECK(euler_uut.project_translation(f, i1, &v1, point_t(1.0, -1.0, 1.0), 0.0) == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(v1 == point_t(1.0, -1.0, 1.0));

    BOOST_CHECK_MESSAGE(fabs(magnitude(euler_uut.project_translation(f, i3, &v1, point_t(-1.0, 1.0, -1.0), 0.75) - point_t(-0.65625, 0.5625, -0.28125))) < result_tolerance, euler_uut.project_translation(f, i3, &v1, point_t(-1.0, 1.0, -1.0), 0.75));
    BOOST_CHECK_MESSAGE(fabs(magnitude(v1 - point_t(-0.75, 0.5, 0.25))) < result_tolerance, v1);
}


BOOST_AUTO_TEST_CASE( basic_linear_torque_euler_test )
{
    std::unique_ptr<linear_force> lf0(new linear_force(point_t(0.0, 2.0, 0.0), point_t(-0.5 * PI, 4.0, -10.0), point_t(PI, -4.0, 10.0), 5.0));
    std::vector<force *> forces({lf0.get()});
    aggregate_force f(forces);

    /* These rotations are all much to big for the linear approximation used, hence we see some strange quaternions */
    const quaternion_t start(1.0, 0.0, 0.0, 0.0);
    BOOST_CHECK_MESSAGE(fabs(magnitude(euler_uut.project_rotation(f, i0, &v1, start, point_t(PI, 0.0, 0.0), 1.0) - quaternion_t(1.0, 3.44579, 0.0, -0.589048) + start)) < result_tolerance, euler_uut.project_rotation(f, i0, &v1, start, point_t(PI, 0.0, 0.0), 1.0) + start);
    BOOST_CHECK_MESSAGE(fabs(magnitude(v1 - point_t(10.6416, 0.0, -2.35619))) < result_tolerance, v1);

    BOOST_CHECK_MESSAGE(fabs(magnitude(euler_uut.project_rotation(f, i1, &v1, start, point_t(0.0, -PI, 0.0), 1.0) - quaternion_t(1.0, 3.75001, -1.5708, -1.1781) + start)) < result_tolerance, euler_uut.project_rotation(f, i1, &v1, start, point_t(0.0, -PI, 0.0), 1.0) + start);
    BOOST_CHECK_MESSAGE(fabs(magnitude(v1 - point_t(15.0, -3.14159, -4.7124))) < result_tolerance, v1);

    BOOST_CHECK_MESSAGE(fabs(magnitude(euler_uut.project_rotation(f, i2, &v1, start, point_t(0.0, 0.0, -PI), 1.0) - quaternion_t(1.0, 0.0, 0.0, -1.5708) + start)) < result_tolerance, euler_uut.project_rotation(f, i2, &v1, start, point_t(0.0, 0.0, -PI), 1.0) + start);
    BOOST_CHECK_MESSAGE(v1 == point_t(0.0, 0.0, -PI), v1);
}


BOOST_AUTO_TEST_CASE( timestep_linear_torque_euler_test )
{
    std::unique_ptr<linear_force> lf0(new linear_force(point_t(0.0, 2.0, 0.0), point_t(-0.5 * PI, 4.0, -10.0), point_t(PI, -4.0, 10.0), 5.0));
    std::vector<force *> forces({lf0.get()});
    aggregate_force f(forces);

    const quaternion_t start(1.0, 0.0, 0.0, 0.0);
    BOOST_CHECK_MESSAGE(fabs(magnitude(euler_uut.project_rotation(f, i0, &v1, start, point_t(PI, 0.0, 0.0), 0.5) - quaternion_t(1, 1.25415, 0.0, -0.147262) + start)) < result_tolerance, euler_uut.project_rotation(f, i0, &v1, start, point_t(PI, 0.0, 0.0), 0.5) + start);
    BOOST_CHECK_MESSAGE(fabs(magnitude(v1 - point_t(6.89159, 0.0, -1.1781))) < result_tolerance, v1);

    BOOST_CHECK_MESSAGE(fabs(magnitude(euler_uut.project_rotation(f, i1, &v1, start, point_t(0.0, -PI, 0.0), 0.0) - quaternion_t(1.0, 0.0, 0.0, 0.0) + start)) < result_tolerance, euler_uut.project_rotation(f, i1, &v1, start, point_t(0.0, -PI, 0.0), 0.0) + start);
    BOOST_CHECK_MESSAGE(v1 == point_t(0.0, -PI, 0.0), v1);

    BOOST_CHECK_MESSAGE(fabs(magnitude(euler_uut.project_rotation(f, i2, &v1, start, point_t(0.0, -PI), 0.75) - quaternion_t(1.0, 0.0, -1.1781, 0.0) + start)) < result_tolerance, euler_uut.project_rotation(f, i2, &v1, start, point_t(0.0, -PI), 0.75) + start);
    BOOST_CHECK_MESSAGE(v1 == point_t(0.0, -PI, 0.0), v1);
}


/* Squared force tests */
/* Note -- Dropped test on infinite mass object, too boring */
BOOST_AUTO_TEST_CASE( basic_squared_force_euler_test )
{
    std::unique_ptr<squared_force> sf0(new squared_force(point_t(0.0, 0.0, 0.0), point_t(-1.0, 4.0, -10.0), point_t(2.0, -4.0, 10.0), 5.0));
    std::vector<force *> forces({sf0.get()});
    aggregate_force f(forces);
    
    BOOST_CHECK_MESSAGE(fabs(magnitude(euler_uut.project_translation(f, i0, &v1, point_t(1.0, 1.0, 1.0), 1.0) - point_t(1.25, 0.5, 2.25))) < result_tolerance, euler_uut.project_translation(f, i0, &v1, point_t(1.0, 1.0, 1.0), 1.0));
    BOOST_CHECK_MESSAGE(fabs(magnitude(v1 - point_t(1.5, 0.0, 3.5))) < result_tolerance, v1);

    BOOST_CHECK_MESSAGE(fabs(magnitude(euler_uut.project_translation(f, i1, &v1, point_t(1.0, -1.0, 1.0), 1.0) - point_t(1.5, -2.0, 3.5))) < result_tolerance, euler_uut.project_translation(f, i1, &v1, point_t(1.0, -1.0, 1.0), 1.0));
    BOOST_CHECK_MESSAGE(fabs(magnitude(v1 - point_t(2.0, -3.0, 6.0))) < result_tolerance, v1);

    BOOST_CHECK_MESSAGE(fabs(magnitude(euler_uut.project_translation(f, i3, &v1, point_t(-1.0, 1.0, -1.0), 1.0) - point_t(-0.833333, 0.666667, -0.166667))) < result_tolerance, euler_uut.project_translation(f, i3, &v1, point_t(-1.0, 1.0, -1.0), 1.0));
    BOOST_CHECK_MESSAGE(fabs(magnitude(v1 - point_t(-0.666667, 0.333333, 0.666667))) < result_tolerance, v1);
}


/* Note -- Dropped zero time test, too boring */
BOOST_AUTO_TEST_CASE( timestep_squared_force_euler_test )
{
    std::unique_ptr<squared_force> sf0(new squared_force(point_t(0.0, 0.0, 0.0), point_t(-1.0, 4.0, -10.0), point_t(2.0, -4.0, 10.0), 5.0));
    std::vector<force *> forces({sf0.get()});
    aggregate_force f(forces);

    BOOST_CHECK_MESSAGE(fabs(magnitude(euler_uut.project_translation(f, i0, &v1, point_t(1.0, 1.0, 1.0), 0.5) - point_t(0.5625, 0.375, 0.8125))) < result_tolerance, euler_uut.project_translation(f, i0, &v1, point_t(1.0, 1.0, 1.0), 0.5));
    BOOST_CHECK_MESSAGE(fabs(magnitude(v1 - point_t(1.25, 0.5, 2.25))) < result_tolerance, v1);

    BOOST_CHECK_MESSAGE(fabs(magnitude(euler_uut.project_translation(f, i1, &v1, point_t(1.0, -1.0, 1.0), 0.75) - point_t(1.03125, -1.3125, 2.15625))) < result_tolerance, euler_uut.project_translation(f, i1, &v1, point_t(1.0, -1.0, 1.0), 0.75));
    BOOST_CHECK_MESSAGE(fabs(magnitude(v1 - point_t(1.75, -2.5, 4.75))) < result_tolerance, v1);

    BOOST_CHECK_MESSAGE(fabs(magnitude(euler_uut.project_translation(f, i3, &v1, point_t(-1.0, 1.0, -1.0), 0.9) - point_t(-0.765, 0.63, -0.225))) < result_tolerance, euler_uut.project_translation(f, i3, &v1, point_t(-1.0, 1.0, -1.0), 0.9));
    BOOST_CHECK_MESSAGE(fabs(magnitude(v1 - point_t(-0.7, 0.4, 0.5))) < result_tolerance, v1);
}


BOOST_AUTO_TEST_CASE( basic_squared_torque_euler_test )
{
    std::unique_ptr<squared_force> sf0(new squared_force(point_t(0.0, 0.0, 1.0), point_t(-0.5 * PI, 4.0, -10.0), point_t(PI, -4.0, 10.0), 5.0));
    std::vector<force *> forces({sf0.get()});
    aggregate_force f(forces);

    /* These rotations are all much to big for the linear approximation used, hence we see some strange quaternions */
    const quaternion_t start(1.0, 0.0, 0.0, 0.0);
    BOOST_CHECK_MESSAGE(fabs(magnitude(euler_uut.project_rotation(f, i0, &v1, start, point_t(PI, 0.0, 0.0), 1.0) - quaternion_t(1.0, 1.9458, 0.294524, 0.0) + start)) < result_tolerance, euler_uut.project_rotation(f, i0, &v1, start, point_t(PI, 0.0, 0.0), 1.0) + start);
    BOOST_CHECK_MESSAGE(fabs(magnitude(v1 - point_t(4.64159, 1.1781, 0.0))) < result_tolerance, v1);

    BOOST_CHECK_MESSAGE(fabs(magnitude(euler_uut.project_rotation(f, i1, &v1, start, point_t(0.0, -PI, 0.0), 1.0) - quaternion_t(1.0, 0.750002, -0.981746, 0.0) + start)) < result_tolerance, euler_uut.project_rotation(f, i1, &v1, start, point_t(0.0, -PI, 0.0), 1.0) + start);
    BOOST_CHECK_MESSAGE(fabs(magnitude(v1 - point_t(3.00001, -0.785392, 0.0))) < result_tolerance, v1);

    BOOST_CHECK_MESSAGE(fabs(magnitude(euler_uut.project_rotation(f, i3, &v1, start, point_t(0.0, 0.0, -PI), 1.0) - quaternion_t(1.0, 0.25, 0.19635, -1.570808) + start)) < result_tolerance, euler_uut.project_rotation(f, i3, &v1, start, point_t(0.0, 0.0, -PI), 1.0) + start);
    BOOST_CHECK_MESSAGE(fabs(magnitude(v1 - point_t(1.0, 0.785398, -3.14159))) < result_tolerance, v1);
}


BOOST_AUTO_TEST_CASE( timestep_squared_torque_euler_test )
{
    std::unique_ptr<squared_force> sf0(new squared_force(point_t(0.0, 0.0, 1.0), point_t(-0.5 * PI, 4.0, -10.0), point_t(PI, -4.0, 10.0), 5.0));
    std::vector<force *> forces({sf0.get()});
    aggregate_force f(forces);

    const quaternion_t start(1.0, 0.0, 0.0, 0.0);
    BOOST_CHECK_MESSAGE(fabs(magnitude(euler_uut.project_rotation(f, i0, &v1, start, point_t(PI, 0.0, 0.0), 0.5) - quaternion_t(1.0, 0.879148, 0.073631, 0.0) + start)) < result_tolerance, euler_uut.project_rotation(f, i0, &v1, start, point_t(PI, 0.0, 0.0), 0.5) + start);
    BOOST_CHECK_MESSAGE(fabs(magnitude(v1 - point_t(3.89159, 0.589048, 0.0))) < result_tolerance, v1);

    BOOST_CHECK_MESSAGE(fabs(magnitude(euler_uut.project_rotation(f, i1, &v1, start, point_t(0.0, -PI, 0.0), 0.75) - quaternion_t(1.0, 0.421876, -0.846757, 0.0) + start)) < result_tolerance, euler_uut.project_rotation(f, i1, &v1, start, point_t(0.0, -PI, 0.0), 0.75) + start);
    BOOST_CHECK_MESSAGE(fabs(magnitude(v1 - point_t(2.25001, -1.37444, 0.0))) < result_tolerance, v1);

    BOOST_CHECK_MESSAGE(fabs(magnitude(euler_uut.project_rotation(f, i3, &v1, start, point_t(0.0, -PI), 0.9) - quaternion_t(1.0, 0.2025, -1.25467, 0.0) + start)) < result_tolerance, euler_uut.project_rotation(f, i3, &v1, start, point_t(0.0, -PI), 0.9) + start);
    BOOST_CHECK_MESSAGE(fabs(magnitude(v1 - point_t(0.9, -2.43473, 0.0))) < result_tolerance, v1);
}


/* Sinosoidal force tests */
BOOST_AUTO_TEST_CASE( basic_sinusoidal_force_euler_test )
{
    std::unique_ptr<sin_force> sf0(new sin_force(point_t(0.0, 0.0, 0.0), point_t(-1.0, 4.0, -10.0), point_t(2.0, -4.0, 10.0), 0.75, 1.5 * PI, 5.0));
    std::vector<force *> forces({sf0.get()});
    aggregate_force f(forces);

    BOOST_CHECK_MESSAGE(fabs(magnitude(euler_uut.project_translation(f, i0, &v1, point_t(1.0, 1.0, 1.0), 1.0) - point_t(1.375, 0.0, 3.5))) < result_tolerance, euler_uut.project_translation(f, i0, &v1, point_t(1.0, 1.0, 1.0), 1.0));
    BOOST_CHECK_MESSAGE(fabs(magnitude(v1 - point_t(1.75, -1.0, 6.0))) < result_tolerance, v1);

    BOOST_CHECK_MESSAGE(fabs(magnitude(euler_uut.project_translation(f, i1, &v1, point_t(1.0, -1.0, 1.0), 1.0) - point_t(1.75, -3.0, 6.0))) < result_tolerance, euler_uut.project_translation(f, i1, &v1, point_t(1.0, -1.0, 1.0), 1.0));
    BOOST_CHECK_MESSAGE(fabs(magnitude(v1 - point_t(2.5, -5.0, 11.0))) < result_tolerance, v1);

    BOOST_CHECK_MESSAGE(fabs(magnitude(euler_uut.project_translation(f, i3, &v1, point_t(-1.0, 1.0, -1.0), 1.0) - point_t(-0.75, 0.333333, 0.666667))) < result_tolerance, euler_uut.project_translation(f, i3, &v1, point_t(-1.0, 1.0, -1.0), 1.0));
    BOOST_CHECK_MESSAGE(fabs(magnitude(v1 - point_t(-0.5, -0.333333, 2.33333))) < result_tolerance, v1);
}


/* Note -- Dropped zero time test, too boring */
BOOST_AUTO_TEST_CASE( timestep_sinusoidal_force_euler_test )
{
    std::unique_ptr<sin_force> sf0(new sin_force(point_t(0.0, 0.0, 0.0), point_t(-1.0, 4.0, -10.0), point_t(2.0, -4.0, 10.0), 0.75, 1.5 * PI, 5.0));
    std::vector<force *> forces({sf0.get()});
    aggregate_force f(forces);

    BOOST_CHECK_MESSAGE(fabs(magnitude(euler_uut.project_translation(f, i0, &v1, point_t(1.0, 1.0, 1.0), 0.5) - point_t(0.59375, 0.25, 1.125))) < result_tolerance, euler_uut.project_translation(f, i0, &v1, point_t(1.0, 1.0, 1.0), 0.5));
    BOOST_CHECK_MESSAGE(fabs(magnitude(v1 - point_t(1.375, 0.0, 3.5))) < result_tolerance, v1);

    BOOST_CHECK_MESSAGE(fabs(magnitude(euler_uut.project_translation(f, i1, &v1, point_t(1.0, -1.0, 1.0), 0.75) - point_t(1.17188, -1.875, 3.5625))) < result_tolerance, euler_uut.project_translation(f, i1, &v1, point_t(1.0, -1.0, 1.0), 0.75));
    BOOST_CHECK_MESSAGE(fabs(magnitude(v1 - point_t(2.125, -4.0, 8.5))) < result_tolerance, v1);

    BOOST_CHECK_MESSAGE(fabs(magnitude(euler_uut.project_translation(f, i3, &v1, point_t(-1.0, 1.0, -1.0), 0.9) - point_t(-0.6975, 0.36, 0.45))) < result_tolerance, euler_uut.project_translation(f, i3, &v1, point_t(-1.0, 1.0, -1.0), 0.9));
    BOOST_CHECK_MESSAGE(fabs(magnitude(v1 - point_t(-0.55, -0.2, 2.0))) < result_tolerance, v1);
}


BOOST_AUTO_TEST_CASE( basic_sinusoidal_torque_euler_test )
{
    std::unique_ptr<sin_force> sf0(new sin_force(point_t(0.0, 1.0, 0.0), point_t(-1.0, 4.0, -10.0), point_t(2.0, -4.0, 10.0), 0.75, 1.5 * PI, 5.0));
    std::vector<force *> forces({sf0.get()});
    aggregate_force f(forces);

    /* These rotations are all much to big for the linear approximation used, hence we see some strange quaternions */
    const quaternion_t start(1.0, 0.0, 0.0, 0.0);
    BOOST_CHECK_MESSAGE(fabs(magnitude(euler_uut.project_rotation(f, i0, &v1, start, point_t(PI, 0.0, 0.0), 1.0) - quaternion_t(1.0, 3.44579, 0.0, -0.28125) + start)) < result_tolerance, euler_uut.project_rotation(f, i0, &v1, start, point_t(PI, 0.0, 0.0), 1.0) + start);
    BOOST_CHECK_MESSAGE(fabs(magnitude(v1 - point_t(10.6416, 0.0, -1.125))) < result_tolerance, v1);

    BOOST_CHECK_MESSAGE(fabs(magnitude(euler_uut.project_rotation(f, i1, &v1, start, point_t(0.0, -PI, 0.0), 1.0) - quaternion_t(1.0, 3.75001, -1.5708, -0.562501) + start)) < result_tolerance, euler_uut.project_rotation(f, i1, &v1, start, point_t(0.0, -PI, 0.0), 1.0) + start);
    BOOST_CHECK_MESSAGE(fabs(magnitude(v1 - point_t(15.0, -3.14159, -2.25001))) < result_tolerance, v1);

    BOOST_CHECK_MESSAGE(fabs(magnitude(euler_uut.project_rotation(f, i3, &v1, start, point_t(0.0, 0.0, -PI), 1.0) - quaternion_t(1.0, 1.25, 0.0, -1.7583) + start)) < result_tolerance, euler_uut.project_rotation(f, i3, &v1, start, point_t(0.0, 0.0, -PI), 1.0) + start);
    BOOST_CHECK_MESSAGE(fabs(magnitude(v1 - point_t(5.0, 0.0, -3.89159))) < result_tolerance, v1);
}


BOOST_AUTO_TEST_CASE( timestep_sinusoidal_torque_euler_test )
{
    std::unique_ptr<sin_force> sf0(new sin_force(point_t(0.0, 1.0, 0.0), point_t(-1.0, 4.0, -10.0), point_t(2.0, -4.0, 10.0), 0.75, 1.5 * PI, 5.0));
    std::vector<force *> forces({sf0.get()});
    aggregate_force f(forces);

    const quaternion_t start(1.0, 0.0, 0.0, 0.0);
    BOOST_CHECK_MESSAGE(fabs(magnitude(euler_uut.project_rotation(f, i0, &v1, start, point_t(PI, 0.0, 0.0), 0.5) - quaternion_t(1.0, 1.25415, 0.0, -0.0703124) + start)) < result_tolerance, euler_uut.project_rotation(f, i0, &v1, start, point_t(PI, 0.0, 0.0), 0.5) + start);
    BOOST_CHECK_MESSAGE(fabs(magnitude(v1 - point_t(6.89159, 0.0, -0.562499))) < result_tolerance, v1);

    BOOST_CHECK_MESSAGE(fabs(magnitude(euler_uut.project_rotation(f, i1, &v1, start, point_t(0.0, -PI, 0.0), 0.75) - quaternion_t(1.0, 2.10938, -1.1781, -0.316407) + start)) < result_tolerance, euler_uut.project_rotation(f, i1, &v1, start, point_t(0.0, -PI, 0.0), 0.75) + start);
    BOOST_CHECK_MESSAGE(fabs(magnitude(v1 - point_t(11.25, -3.14159, -1.6875))) < result_tolerance, v1);

    BOOST_CHECK_MESSAGE(fabs(magnitude(euler_uut.project_rotation(f, i3, &v1, start, point_t(0.0, -PI), 0.9) - quaternion_t(1.0, 1.0125, -1.41372, -0.151875) + start)) < result_tolerance, euler_uut.project_rotation(f, i3, &v1, start, point_t(0.0, -PI), 0.9) + start);
    BOOST_CHECK_MESSAGE(fabs(magnitude(v1 - point_t(4.5, -3.14159, -0.675))) < result_tolerance, v1);
}


/* Attractive force tests */
/* Note -- RK4 and Euler do very badly on these tests */
BOOST_AUTO_TEST_CASE( basic_attractive_force_euler_test )
{
    std::unique_ptr<attract_force> af0(new attract_force(point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 12.3, -1.2, 5.0));
    std::vector<force *> forces({af0.get()});
    aggregate_force f(forces);

    BOOST_CHECK(fabs(magnitude(euler_uut.project_translation(f, i0, &v1, point_t(1.0, 1.0, 1.0), 1.0) - point_t(1.0, 3.925, 1.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(v1 - point_t(1.0, 6.85, 1.0))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(euler_uut.project_translation(f, i1, &v1, point_t(1.0, -1.0, 1.0), 1.0) - point_t(-0.962221, -1.0, -0.962221))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(v1 - point_t(-2.92444, -1.0, -2.92444))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(euler_uut.project_translation(f, i3, &v1, point_t(-1.0, 1.0, -1.0), 1.0) - point_t(-0.716068, 0.716068, -0.716068))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(v1 - point_t(-0.432137, 0.432137, -0.432137))) < result_tolerance);
}


BOOST_AUTO_TEST_CASE( timestep_attractive_force_euler_test )
{
    std::unique_ptr<attract_force> af0(new attract_force(point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 12.3, -1.2, 5.0));
    std::vector<force *> forces({af0.get()});
    aggregate_force f(forces);

    BOOST_CHECK(fabs(magnitude(euler_uut.project_translation(f, i0, &v1, point_t(1.0, 1.0, 1.0), 0.5) - point_t(0.5, 1.23125, 0.5))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(v1 - point_t(1.0, 3.925, 1.0))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(euler_uut.project_translation(f, i1, &v1, point_t(1.0, -1.0, 1.0), 0.75) - point_t(-0.353749, -0.75, -0.353749))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(v1 - point_t(-1.94333, -1.0, -1.94333))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(euler_uut.project_translation(f, i3, &v1, point_t(-1.0, 1.0, -1.0), 0.9) - point_t(-0.670015, 0.670015, -0.670015))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(v1 - point_t(-0.488923, 0.488923, -0.488923))) < result_tolerance);
}


BOOST_AUTO_TEST_CASE( basic_attractive_torque_euler_test )
{
    std::unique_ptr<attract_force> af0(new attract_force(point_t(0.0, 0.0, 1.5), point_t(0.0, 0.0, 0.0), 12.3, -1.2, 5.0));
    std::vector<force *> forces({af0.get()});
    aggregate_force f(forces);

    /* These rotations are all much to big for the linear approximation used, hence we see some strange quaternions */
    const quaternion_t start(1.0, 0.0, 0.0, 0.0);
    BOOST_CHECK_MESSAGE(fabs(magnitude(euler_uut.project_rotation(f, i0, &v1, start, point_t(PI, 0.0, 0.0), 1.0) - quaternion_t(1.0, -1.71982, 0.0, 0.0) + start)) < result_tolerance, euler_uut.project_rotation(f, i0, &v1, start, point_t(PI, 0.0, 0.0), 1.0) + start);
    BOOST_CHECK_MESSAGE(fabs(magnitude(v1 - point_t(-10.0209, 0.0, 0.0))) < result_tolerance, v1);

    BOOST_CHECK_MESSAGE(fabs(magnitude(euler_uut.project_rotation(f, i1, &v1, start, point_t(0.0, -PI, 0.0), 1.0) - quaternion_t(1.0, 0.0, -3.7783, 0.0) + start)) < result_tolerance, euler_uut.project_rotation(f, i1, &v1, start, point_t(0.0, -PI, 0.0), 1.0) + start);
    BOOST_CHECK_MESSAGE(fabs(magnitude(v1 - point_t(0.0, -11.9716, 0.0))) < result_tolerance, v1);

    BOOST_CHECK_MESSAGE(fabs(magnitude(euler_uut.project_rotation(f, i3, &v1, start, point_t(0.0, 0.0, -PI), 1.0) - quaternion_t(1.0, 0.319423, 0.319423, -1.5708) + start)) < result_tolerance, euler_uut.project_rotation(f, i3, &v1, start, point_t(0.0, 0.0, -PI), 1.0) + start);
    BOOST_CHECK_MESSAGE(fabs(magnitude(v1 - point_t(1.27769, 1.27769, -3.14159))) < result_tolerance, v1);
}


BOOST_AUTO_TEST_CASE( timestep_attractive_torque_euler_test )
{
    std::unique_ptr<attract_force> af0(new attract_force(point_t(0.0, 0.0, 1.5), point_t(0.0, 0.0, 0.0), 12.3, -1.2, 5.0));
    std::vector<force *> forces({af0.get()});
    aggregate_force f(forces);

    const quaternion_t start(1.0, 0.0, 0.0, 0.0);
    BOOST_CHECK_MESSAGE(fabs(magnitude(euler_uut.project_rotation(f, i0, &v1, start, point_t(PI, 0.0, 0.0), 0.5) - quaternion_t(1.0, -0.037257, 0.0, 0.0) + start)) < result_tolerance, euler_uut.project_rotation(f, i0, &v1, start, point_t(PI, 0.0, 0.0), 0.5) + start);
    BOOST_CHECK_MESSAGE(fabs(magnitude(v1 - point_t(-3.43965, 0.0, 0.0))) < result_tolerance, v1);

    BOOST_CHECK_MESSAGE(fabs(magnitude(euler_uut.project_rotation(f, i1, &v1, start, point_t(0.0, -PI, 0.0), 0.75) - quaternion_t(1.0, 0, -2.41982, 0.0) + start)) < result_tolerance, euler_uut.project_rotation(f, i1, &v1, start, point_t(0.0, -PI, 0.0), 0.75) + start);
    BOOST_CHECK_MESSAGE(fabs(magnitude(v1 - point_t(0.0, -9.76411, 0.0))) < result_tolerance, v1);

    BOOST_CHECK_MESSAGE(fabs(magnitude(euler_uut.project_rotation(f, i3, &v1, start, point_t(0.0, -PI), 0.9) - quaternion_t(1.0, 0.258733, -1.15498, 0.0) + start)) < result_tolerance, euler_uut.project_rotation(f, i3, &v1, start, point_t(0.0, -PI), 0.9) + start);
    BOOST_CHECK_MESSAGE(fabs(magnitude(v1 - point_t(1.14992, -1.99167, 0.0))) < result_tolerance, v1);
}


/* Viscous force tests */
/* Note -- RK4 is pretty good on these tests, Euler over estimates the force so v and x are too small */
BOOST_AUTO_TEST_CASE( basic_viscous_force_euler_test )
{
    std::unique_ptr<viscous_force> vf0(new viscous_force(point_t(0.0, 0.0, 0.0), 2.1, 5.0));
    std::vector<force *> forces({vf0.get()});
    aggregate_force f(forces);

    BOOST_CHECK(fabs(magnitude(euler_uut.project_translation(f, i0, &v1, point_t(1.0, 1.0, 1.0), 1.0) - point_t(0.7375, 0.7375, 0.7375))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(v1 - point_t(0.475, 0.475, 0.475))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(euler_uut.project_translation(f, i1, &v1, point_t(1.0, -1.0, 1.0), 1.0) - point_t(0.475, -0.475, 0.475))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(v1 - point_t(-0.05, 0.05, -0.05))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(euler_uut.project_translation(f, i3, &v1, point_t(-1.0, 1.0, -1.0), 1.0) - point_t(-0.825, 0.825, -0.825))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(v1 - point_t(-0.65, 0.65, -0.65))) < result_tolerance);
}


BOOST_AUTO_TEST_CASE( timestep_viscous_force_euler_test )
{
    std::unique_ptr<viscous_force> vf0(new viscous_force(point_t(0.0, 0.0, 0.0), 2.1, 5.0));
    std::vector<force *> forces({vf0.get()});
    aggregate_force f(forces);

    BOOST_CHECK(fabs(magnitude(euler_uut.project_translation(f, i0, &v1, point_t(1.0, 1.0, 1.0), 0.5) - point_t(0.434375, 0.434375, 0.434375))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(v1 - point_t(0.7375, 0.7375, 0.7375))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(euler_uut.project_translation(f, i1, &v1, point_t(1.0, -1.0, 1.0), 0.75) - point_t(0.454687, -0.454687, 0.454687))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(v1 - point_t(0.2125, -0.2125, 0.2125))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(euler_uut.project_translation(f, i3, &v1, point_t(-1.0, 1.0, -1.0), 0.9) - point_t(-0.75825, 0.75825, -0.75825))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(v1 - point_t(-0.685, 0.685, -0.685))) < result_tolerance);
}


BOOST_AUTO_TEST_CASE( basic_viscous_torque_euler_test )
{
    std::unique_ptr<viscous_force> vf0(new viscous_force(point_t(2.1, 0.0, 0.0), 2.1, 5.0));
    std::vector<force *> forces({vf0.get()});
    aggregate_force f(forces);

    /* These rotations are all much to big for the linear approximation used, hence we see some strange quaternions */
    const quaternion_t start(1.0, 0.0, 0.0, 0.0);
    BOOST_CHECK_MESSAGE(fabs(magnitude(euler_uut.project_rotation(f, i0, &v1, start, point_t(PI, 0.0, 0.0), 1.0) - quaternion_t(1.0, 1.5708, 0.0, 0.0) + start)) < result_tolerance, euler_uut.project_rotation(f, i0, &v1, start, point_t(PI, 0.0, 0.0), 1.0) + start);
    BOOST_CHECK_MESSAGE(fabs(magnitude(v1 - point_t(3.14159, 0.0, 0.0))) < result_tolerance, v1);

    BOOST_CHECK_MESSAGE(fabs(magnitude(euler_uut.project_rotation(f, i1, &v1, start, point_t(0.0, -PI, 0.0), 1.0) - quaternion_t(1.0, 0.0, -1.5708, 2.59771) + start)) < result_tolerance, euler_uut.project_rotation(f, i1, &v1, start, point_t(0.0, -PI, 0.0), 1.0) + start);
    BOOST_CHECK_MESSAGE(fabs(magnitude(v1 - point_t(0.0, -3.14159, 10.3908))) < result_tolerance, v1);

    BOOST_CHECK_MESSAGE(fabs(magnitude(euler_uut.project_rotation(f, i3, &v1, start, point_t(0.0, 0.0, -PI), 1.0) - quaternion_t(1.0, 0.0, -0.865901, -1.5708) + start)) < result_tolerance, euler_uut.project_rotation(f, i3, &v1, start, point_t(0.0, 0.0, -PI), 1.0) + start);
    BOOST_CHECK_MESSAGE(fabs(magnitude(v1 - point_t(0.0, -3.46361, -3.14159))) < result_tolerance, v1);
}


BOOST_AUTO_TEST_CASE( timestep_viscous_torque_euler_test )
{
    std::unique_ptr<viscous_force> vf0(new viscous_force(point_t(2.1, 0.0, 0.0), 2.1, 5.0));
    std::vector<force *> forces({vf0.get()});
    aggregate_force f(forces);

    const quaternion_t start(1.0, 0.0, 0.0, 0.0);
    BOOST_CHECK_MESSAGE(fabs(magnitude(euler_uut.project_rotation(f, i0, &v1, start, point_t(PI, 0.0, 0.0), 0.5) - quaternion_t(1.0, 0.785398, 0.0, 0.0) + start)) < result_tolerance, euler_uut.project_rotation(f, i0, &v1, start, point_t(PI, 0.0, 0.0), 0.5) + start);
    BOOST_CHECK_MESSAGE(fabs(magnitude(v1 - point_t(3.14159, 0.0, 0.0))) < result_tolerance, v1);

    BOOST_CHECK_MESSAGE(fabs(magnitude(euler_uut.project_rotation(f, i1, &v1, start, point_t(0.0, -PI, 0.0), 0.75) - quaternion_t(1.0, 0.0, -1.1781, 1.46121) + start)) < result_tolerance, euler_uut.project_rotation(f, i1, &v1, start, point_t(0.0, -PI, 0.0), 0.75) + start);
    BOOST_CHECK_MESSAGE(fabs(magnitude(v1 - point_t(0.0, -3.14159, 7.79313))) < result_tolerance, v1);

    BOOST_CHECK_MESSAGE(fabs(magnitude(euler_uut.project_rotation(f, i3, &v1, start, point_t(0.0, -PI), 0.9) - quaternion_t(1.0, 0.0, -1.41372, 0.70138) + start)) < result_tolerance, euler_uut.project_rotation(f, i3, &v1, start, point_t(0.0, -PI), 0.9) + start);
    BOOST_CHECK_MESSAGE(fabs(magnitude(v1 - point_t(0.0, -3.14159, 3.11724))) < result_tolerance, v1);
}


/* RK4 Tests */
/* Test with velocity, no applied forces */
BOOST_AUTO_TEST_CASE( basic_velocity_rk4_test )
{
    BOOST_CHECK(rk4_uut.project_translation(no_force, i0, &v1, point_t(1.0, 1.0, 1.0), 1.0) == point_t(1.0, 1.0, 1.0));
    BOOST_CHECK(v1 == point_t(1.0, 1.0, 1.0));

    BOOST_CHECK(rk4_uut.project_translation(no_force, i1, &v1, point_t(1.0, -1.0, 1.0), 1.0) == point_t(1.0, -1.0, 1.0));
    BOOST_CHECK(v1 == point_t(1.0, -1.0, 1.0));

    BOOST_CHECK(rk4_uut.project_translation(no_force, i2, &v1, point_t(-1.0, 1.0, -1.0), 1.0) == point_t(-1.0, 1.0, -1.0));
    BOOST_CHECK(v1 == point_t(-1.0, 1.0, -1.0));
}


BOOST_AUTO_TEST_CASE( timestep_velocity_rk4_test )
{
    BOOST_CHECK(rk4_uut.project_translation(no_force, i0, &v1, point_t(1.0, 1.0, 1.0), 0.5) == point_t(0.5, 0.5, 0.5));
    BOOST_CHECK(v1 == point_t(1.0, 1.0, 1.0));

    BOOST_CHECK(rk4_uut.project_translation(no_force, i1, &v1, point_t(1.0, -1.0, 1.0), 0.0) == point_t(0.0, -0.0, 0.0));
    BOOST_CHECK(v1 == point_t(1.0, -1.0, 1.0));

    BOOST_CHECK(rk4_uut.project_translation(no_force, i2, &v1, point_t(-1.0, 1.0, -1.0), 0.75) == point_t(-0.75, 0.75, -0.75));
    BOOST_CHECK(v1 == point_t(-1.0, 1.0, -1.0));
}


BOOST_AUTO_TEST_CASE( basic_angular_velocity_rk4_test )
{
    /* These rotations are all much to big for the linear approximation used, hence we see some strange quaternions */
    const quaternion_t start(1.0, 0.0, 0.0, 0.0);
    BOOST_CHECK(fabs(magnitude(rk4_uut.project_rotation(no_force, i0, &v1, start, point_t(PI, 0.0, 0.0), 1.0) - quaternion_t(0.0199689, 0.924832, 0.0, 0.0) + start)) < result_tolerance);
    BOOST_CHECK(v1 == point_t(PI, 0.0, 0.0));

    BOOST_CHECK(fabs(magnitude(rk4_uut.project_rotation(no_force, i1, &v1, start, point_t(0.0, -PI, 0.0), 1.0) - quaternion_t(0.0199689, 0.0, -0.924832, 0.0) + start)) < result_tolerance);
    BOOST_CHECK(v1 == point_t(0.0, -PI, 0.0));

    BOOST_CHECK(fabs(magnitude(rk4_uut.project_rotation(no_force, i2, &v1, start, point_t(0.0, 0.0, -PI), 1.0) - quaternion_t(0.0199689, 0.0, 0.0, -0.924832) + start)) < result_tolerance);
    BOOST_CHECK(v1 == point_t(0.0, 0.0, -PI));
}


BOOST_AUTO_TEST_CASE( timestep_angular_velocity_rk4_test )
{
    const quaternion_t start(1.0, 0.0, 0.0, 0.0);
    BOOST_CHECK(fabs(magnitude(rk4_uut.project_rotation(no_force, i0, &v1, start, point_t(PI, 0.0, 0.0), 0.5) - quaternion_t(0.707429, 0.704653, 0.0, 0.0) + start)) < result_tolerance);
    BOOST_CHECK(v1 == point_t(PI, 0.0, 0.0));

    BOOST_CHECK(fabs(magnitude(rk4_uut.project_rotation(no_force, i1, &v1, start, point_t(0.0, -PI, 0.0), 0.0) - quaternion_t(1.0, 0.0, 0.0, 0.0) + start)) < result_tolerance);
    BOOST_CHECK(v1 == point_t(0.0, -PI, 0.0));

    BOOST_CHECK(fabs(magnitude(rk4_uut.project_rotation(no_force, i2, &v1, start, point_t(0.0, -PI), 0.75) - quaternion_t(0.386306, 0.0, -0.905581, 0.0) + start)) < result_tolerance);
    BOOST_CHECK(v1 == point_t(0.0, -PI));
}


/* Constant force tests */
BOOST_AUTO_TEST_CASE( basic_const_force_rk4_test )
{
    std::unique_ptr<const_force> cf0(new const_force(point_t(0.0, 0.0, 0.0), point_t(2.0, -4.0, 10.0), 5.0));
    std::vector<force *> forces({cf0.get()});
    aggregate_force f(forces);

    BOOST_CHECK(rk4_uut.project_translation(f, i0, &v1, point_t(1.0, 1.0, 1.0), 1.0) == point_t(1.25, 0.5, 2.25));
    BOOST_CHECK(v1 == point_t(1.5, 0.0, 3.5));

    BOOST_CHECK(rk4_uut.project_translation(f, i1, &v1, point_t(1.0, -1.0, 1.0), 1.0) == point_t(1.5, -2.0, 3.5));
    BOOST_CHECK(v1 == point_t(2.0, -3.0, 6.0));

    BOOST_CHECK(rk4_uut.project_translation(f, i2, &v1, point_t(-1.0, 1.0, -1.0), 1.0) == point_t(-1.0, 1.0, -1.0));
    BOOST_CHECK(v1 == point_t(-1.0, 1.0, -1.0));
}


BOOST_AUTO_TEST_CASE( timestep_const_force_rk4_test )
{
    std::unique_ptr<const_force> cf0(new const_force(point_t(0.0, 0.0, 0.0), point_t(2.0, -4.0, 10.0), 5.0));
    std::vector<force *> forces({cf0.get()});
    aggregate_force f(forces);
    
    BOOST_CHECK(rk4_uut.project_translation(f, i0, &v1, point_t(1.0, 1.0, 1.0), 0.5) == point_t(0.5625, 0.375, 0.8125));
    BOOST_CHECK(v1 == point_t(1.25, 0.5, 2.25));

    BOOST_CHECK(rk4_uut.project_translation(f, i1, &v1, point_t(1.0, -1.0, 1.0), 0.0) == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(v1 == point_t(1.0, -1.0, 1.0));

    BOOST_CHECK(rk4_uut.project_translation(f, i2, &v1, point_t(-1.0, 1.0, -1.0), 0.75) == point_t(-0.75, 0.75, -0.75));
    BOOST_CHECK(v1 == point_t(-1.0, 1.0, -1.0));
}


/* Note -- Using small time step for RK4 or the result is too full of error */
BOOST_AUTO_TEST_CASE( basic_const_torque_rk4_test )
{
    std::unique_ptr<const_force> cf0(new const_force(point_t(1.0, 0.0, 0.0), point_t(0.0, -0.5 * PI, 0.0), 5.0));
    std::vector<force *> forces({cf0.get()});
    aggregate_force f(forces);

    /* These rotations are all much to big for the linear approximation used, hence we see some strange quaternions */
    const quaternion_t start(1.0, 0.0, 0.0, 0.0);
    BOOST_CHECK(fabs(magnitude(rk4_uut.project_rotation(f, i0, &v1, start, point_t(PI, 0.0, 0.0), 0.1) - quaternion_t(0.987687, 0.156434, 0.0, -0.00146657) + start)) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(v1 - point_t(3.14159, 0.0, -0.0589048))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(rk4_uut.project_rotation(f, i1, &v1, start, point_t(0.0, -PI, 0.0), 0.1) - quaternion_t(0.987684, -0.000153264, -0.156433, -0.00293315) + start)) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(v1 - point_t(0.0, -3.14159, -0.11781))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(rk4_uut.project_rotation(f, i2, &v1, start, point_t(0.0, 0.0, -PI), 1.0) - quaternion_t(0.0199689, 0.0, 0.0, -0.924832) + start)) < result_tolerance);
    BOOST_CHECK(v1 == point_t(0.0, 0.0, -PI));
}


BOOST_AUTO_TEST_CASE( timestep_const_torque_rk4_test )
{
    std::unique_ptr<const_force> cf0(new const_force(point_t(1.0, 0.0, 0.0), point_t(0.0, -0.5 * PI, 0.0), 5.0));
    std::vector<force *> forces({cf0.get()});
    aggregate_force f(forces);

    const quaternion_t start(1.0, 0.0, 0.0, 0.0);
    BOOST_CHECK(fabs(magnitude(rk4_uut.project_rotation(f, i0, &v1, start, point_t(PI, 0.0, 0.0), 0.05) - quaternion_t(0.996917, 0.0784591, 0.0, -0.000367776) + start)) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(v1 - point_t(3.14159, -3.29156e-10, -0.0294524))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(rk4_uut.project_rotation(f, i1, &v1, start, point_t(0.0, -PI, 0.0), 0.0) - quaternion_t(1.0, 0.0, 0.0, 0.0) + start)) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(v1 - point_t(0.0, -PI, 0.0))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(rk4_uut.project_rotation(f, i2, &v1, start, point_t(0.0, -PI), 0.75) - quaternion_t(0.386306, 0.0, -0.905581, 0.0) + start)) < result_tolerance);
    BOOST_CHECK(v1 == point_t(0.0, -PI));
}


/* Linear force tests */
/* Note -- Dropped test on infinite mass object, too boring */
BOOST_AUTO_TEST_CASE( basic_linear_force_rk4_test )
{
    std::unique_ptr<linear_force> lf0(new linear_force(point_t(0.0, 0.0, 0.0), point_t(-1.0, 4.0, -10.0), point_t(2.0, -4.0, 10.0), 5.0));
    std::vector<force *> forces({lf0.get()});
    aggregate_force f(forces);
    
    BOOST_CHECK(fabs(magnitude(rk4_uut.project_translation(f, i0, &v1, point_t(1.0, 1.0, 1.0), 1.0) - point_t(1.20833, 0.666667, 1.83333))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(v1 - point_t(1.375, 0.5, 2.25))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(rk4_uut.project_translation(f, i1, &v1, point_t(1.0, -1.0, 1.0), 1.0) - point_t(1.41667, -1.66667, 2.66667))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(v1 - point_t(1.75, -2.0, 3.5))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(rk4_uut.project_translation(f, i3, &v1, point_t(-1.0, 1.0, -1.0), 1.0) - point_t(-0.861111, 0.777778, -0.444444))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(v1 - point_t(-0.75, 0.666667, -0.166667))) < result_tolerance);
}


BOOST_AUTO_TEST_CASE( timestep_linear_force_rk4_test )
{
    std::unique_ptr<linear_force> lf0(new linear_force(point_t(0.0, 0.0, 0.0), point_t(-1.0, 4.0, -10.0), point_t(2.0, -4.0, 10.0), 5.0));
    std::vector<force *> forces({lf0.get()});
    aggregate_force f(forces);

    BOOST_CHECK(fabs(magnitude(rk4_uut.project_translation(f, i0, &v1, point_t(1.0, 1.0, 1.0), 0.5) - point_t(0.557292, 0.395833, 0.760417))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(v1 - point_t(1.21875, 0.625, 1.9375))) < result_tolerance);

    BOOST_CHECK(rk4_uut.project_translation(f, i1, &v1, point_t(1.0, -1.0, 1.0), 0.0) == point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(v1 == point_t(1.0, -1.0, 1.0));

    BOOST_CHECK(fabs(magnitude(rk4_uut.project_translation(f, i3, &v1, point_t(-1.0, 1.0, -1.0), 0.75) - point_t(-0.667969, 0.609375, -0.398437))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(v1 - point_t(-0.796875, 0.6875, -0.21875))) < result_tolerance);
}


BOOST_AUTO_TEST_CASE( basic_linear_torque_rk4_test )
{
    std::unique_ptr<linear_force> lf0(new linear_force(point_t(0.0, 2.0, 0.0), point_t(-0.5 * PI, 4.0, -10.0), point_t(PI, -4.0, 10.0), 5.0));
    std::vector<force *> forces({lf0.get()});
    aggregate_force f(forces);

    /* These rotations are all much to big for the linear approximation used, hence we see some strange quaternions */
    const quaternion_t start(1.0, 0.0, 0.0, 0.0);
    BOOST_CHECK_MESSAGE(fabs(magnitude(rk4_uut.project_rotation(f, i0, &v1, start, point_t(PI, 0.0, 0.0), 0.1) - quaternion_t(0.984677, 0.174307, -0.000298844, -0.00576287) + start)) < result_tolerance, rk4_uut.project_rotation(f, i0, &v1, start, point_t(PI, 0.0, 0.0), 0.1) + start);
    BOOST_CHECK(fabs(magnitude(v1 - point_t(3.85409, 0.0, -0.229729))) < result_tolerance);

    BOOST_CHECK_MESSAGE(fabs(magnitude(rk4_uut.project_rotation(f, i1, &v1, start, point_t(0.0, -PI, 0.0), 0.1) - quaternion_t(0.986978, 0.0354974, -0.156398, -0.0133875) + start)) < result_tolerance, rk4_uut.project_rotation(f, i1, &v1, start, point_t(0.0, -PI, 0.0), 0.1) + start);
    BOOST_CHECK(fabs(magnitude(v1 - point_t(1.425, -3.14159, -0.459458))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(rk4_uut.project_rotation(f, i2, &v1, start, point_t(0.0, 0.0, -PI), 1.0) - quaternion_t(0.0199689, 0.0, 0.0, -0.924832) + start)) < result_tolerance);
    BOOST_CHECK(v1 == point_t(0.0, 0.0, -PI));
}


BOOST_AUTO_TEST_CASE( timestep_linear_torque_rk4_test )
{
    std::unique_ptr<linear_force> lf0(new linear_force(point_t(0.0, 2.0, 0.0), point_t(-0.5 * PI, 4.0, -10.0), point_t(PI, -4.0, 10.0), 5.0));
    std::vector<force *> forces({lf0.get()});
    aggregate_force f(forces);
    
    const quaternion_t start(1.0, 0.0, 0.0, 0.0);
    BOOST_CHECK_MESSAGE(fabs(magnitude(rk4_uut.project_rotation(f, i0, &v1, start, point_t(PI, 0.0, 0.0), 0.05) - quaternion_t(0.996544, 0.0830534, 0.0, -0.00145867) + start)) < result_tolerance, rk4_uut.project_rotation(f, i0, &v1, start, point_t(PI, 0.0, 0.0), 0.05) + start);
    BOOST_CHECK_MESSAGE(fabs(magnitude(v1 - point_t(3.50722, 0.0, -0.116337))) < result_tolerance, v1);

    BOOST_CHECK(fabs(magnitude(rk4_uut.project_rotation(f, i1, &v1, start, point_t(0.0, -PI, 0.0), 0.0) - quaternion_t(1.0, 0.0, 0.0, 0.0) + start)) < result_tolerance);
    BOOST_CHECK(v1 == point_t(0.0, -PI, 0.0));

    BOOST_CHECK_MESSAGE(fabs(magnitude(rk4_uut.project_rotation(f, i2, &v1, start, point_t(0.0, -PI), 0.075) - quaternion_t(0.993068, 0, -0.117537, 0.0) + start)) < result_tolerance, rk4_uut.project_rotation(f, i2, &v1, start, point_t(0.0, -PI), 0.075) + start);
    BOOST_CHECK(v1 == point_t(0.0, -PI, 0.0));
}


/* Squared force tests */
/* Note -- Dropped test on infinite mass object, too boring */
BOOST_AUTO_TEST_CASE( basic_squared_force_rk4_test )
{
    std::unique_ptr<squared_force> sf0(new squared_force(point_t(0.0, 0.0, 0.0), point_t(-1.0, 4.0, -10.0), point_t(2.0, -4.0, 10.0), 5.0));
    std::vector<force *> forces({sf0.get()});
    aggregate_force f(forces);

    BOOST_CHECK(fabs(magnitude(rk4_uut.project_translation(f, i0, &v1, point_t(1.0, 1.0, 1.0), 1.0) - point_t(1.22917, 0.583333, 2.04167))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(v1 - point_t(1.41667, 0.333333, 2.66667))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(rk4_uut.project_translation(f, i1, &v1, point_t(1.0, -1.0, 1.0), 1.0) - point_t(1.45833, -1.83333, 3.08333))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(v1 - point_t(1.83333, -2.33333, 4.33333))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(rk4_uut.project_translation(f, i3, &v1, point_t(-1.0, 1.0, -1.0), 1.0) - point_t(-0.847222, 0.722222, -0.305556))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(v1 - point_t(-0.722222, 0.555556, 0.111111))) < result_tolerance);
}


/* Note -- Dropped zero time test, too boring */
BOOST_AUTO_TEST_CASE( timestep_squared_force_rk4_test )
{
    std::unique_ptr<squared_force> sf0(new squared_force(point_t(0.0, 0.0, 0.0), point_t(-1.0, 4.0, -10.0), point_t(2.0, -4.0, 10.0), 5.0));
    std::vector<force *> forces({sf0.get()});
    aggregate_force f(forces);

    BOOST_CHECK(fabs(magnitude(rk4_uut.project_translation(f, i0, &v1, point_t(1.0, 1.0, 1.0), 0.5) - point_t(0.561198, 0.380208, 0.799479))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(v1 - point_t(1.23958, 0.541667, 2.14583))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(rk4_uut.project_translation(f, i1, &v1, point_t(1.0, -1.0, 1.0), 0.75) - point_t(1.01807, -1.25977, 2.02441))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(v1 - point_t(1.67969, -2.21875, 4.04687))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(rk4_uut.project_translation(f, i3, &v1, point_t(-1.0, 1.0, -1.0), 0.9) - point_t(-0.774112, 0.66645, -0.316125))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(v1 - point_t(-0.7405, 0.562, 0.095))) < result_tolerance);
}


BOOST_AUTO_TEST_CASE( basic_squared_torque_rk4_test )
{
    std::unique_ptr<squared_force> sf0(new squared_force(point_t(0.0, 0.0, 1.0), point_t(-0.5 * PI, 4.0, -10.0), point_t(PI, -4.0, 10.0), 5.0));
    std::vector<force *> forces({sf0.get()});
    aggregate_force f(forces);

    /* These rotations are all much to big for the linear approximation used, hence we see some strange quaternions */
    const quaternion_t start(1.0, 0.0, 0.0, 0.0);
    BOOST_CHECK_MESSAGE(fabs(magnitude(rk4_uut.project_rotation(f, i0, &v1, start, point_t(PI, 0.0, 0.0), 0.1) - quaternion_t(0.987091, 0.16013, 0.00293012, -0.000153027) + start)) < result_tolerance, rk4_uut.project_rotation(f, i0, &v1, start, point_t(PI, 0.0, 0.0), 0.1) + start);
    BOOST_CHECK_MESSAGE(fabs(magnitude(v1 - point_t(3.29109, 0.117613, 0.0))) < result_tolerance, v1);

    BOOST_CHECK_MESSAGE(fabs(magnitude(rk4_uut.project_rotation(f, i1, &v1, start, point_t(0.0, -PI, 0.0), 0.1) - quaternion_t(0.988564, 0.00745896, -0.150617, -0.000389487) + start)) < result_tolerance, rk4_uut.project_rotation(f, i1, &v1, start, point_t(0.0, -PI, 0.0), 0.1) + start);
    BOOST_CHECK_MESSAGE(fabs(magnitude(v1 - point_t(0.299001, -2.90637, 0.0))) < result_tolerance, v1);

    BOOST_CHECK_MESSAGE(fabs(magnitude(rk4_uut.project_rotation(f, i3, &v1, start, point_t(0.0, 0.0, -PI), 0.1) - quaternion_t(0.987683, 0.00238354, 0.00208357, -0.156433) + start)) < result_tolerance, rk4_uut.project_rotation(f, i3, &v1, start, point_t(0.0, 0.0, -PI), 0.1) + start);
    BOOST_CHECK_MESSAGE(fabs(magnitude(v1 - point_t(0.0996667, 0.0784089, -3.14159))) < result_tolerance, v1);
}


BOOST_AUTO_TEST_CASE( timestep_squared_torque_rk4_test )
{
    std::unique_ptr<squared_force> sf0(new squared_force(point_t(0.0, 0.0, 1.0), point_t(-0.5 * PI, 4.0, -10.0), point_t(PI, -4.0, 10.0), 5.0));
    std::vector<force *> forces({sf0.get()});
    aggregate_force f(forces);

    const quaternion_t start(1.0, 0.0, 0.0, 0.0);
    BOOST_CHECK_MESSAGE(fabs(magnitude(rk4_uut.project_rotation(f, i0, &v1, start, point_t(PI, 0.0, 0.0), 0.05) - quaternion_t(0.996843, 0.0793933, 0.000735382, 0.0) + start)) < result_tolerance, rk4_uut.project_rotation(f, i0, &v1, start, point_t(PI, 0.0, 0.0), 0.05) + start);
    BOOST_CHECK_MESSAGE(fabs(magnitude(v1 - point_t(3.21653, 0.0588802, 0.0))) < result_tolerance, v1);

    BOOST_CHECK_MESSAGE(fabs(magnitude(rk4_uut.project_rotation(f, i1, &v1, start, point_t(0.0, -PI, 0.0), 0.075) - quaternion_t(0.993443, 0.00420559, -0.114247, -0.000164896) + start)) < result_tolerance, rk4_uut.project_rotation(f, i1, &v1, start, point_t(0.0, -PI, 0.0), 0.075) + start);
    BOOST_CHECK_MESSAGE(fabs(magnitude(v1 - point_t(0.224579, -2.96504, 0.0))) < result_tolerance, v1);

    BOOST_CHECK_MESSAGE(fabs(magnitude(rk4_uut.project_rotation(f, i3, &v1, start, point_t(0.0, -PI), 0.09) - quaternion_t(0.990244, 0.00201569, -0.139327, 0.0) + start)) < result_tolerance, rk4_uut.project_rotation(f, i3, &v1, start, point_t(0.0, -PI), 0.09) + start);
    BOOST_CHECK_MESSAGE(fabs(magnitude(v1 - point_t(0.089757, -3.071, 0.0))) < result_tolerance, v1);
}


/* Sinosoidal force tests */
/* Note -- RK4 is no longer exact, but not that bad either  */
BOOST_AUTO_TEST_CASE( basic_sinusoidal_force_rk4_test )
{
    std::unique_ptr<sin_force> sf0(new sin_force(point_t(0.0, 0.0, 0.0), point_t(-1.0, 4.0, -10.0), point_t(2.0, -4.0, 10.0), 0.75, 1.5 * PI, 5.0));
    std::vector<force *> forces({sf0.get()});
    aggregate_force f(forces);
    
    BOOST_CHECK(fabs(magnitude(rk4_uut.project_translation(f, i0, &v1, point_t(1.0, 1.0, 1.0), 1.0) - point_t(1.23274, 0.569036, 2.07741))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(v1 - point_t(1.42382, 0.304738, 2.73816))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(rk4_uut.project_translation(f, i1, &v1, point_t(1.0, -1.0, 1.0), 1.0) - point_t(1.46548, -1.86193, 3.15482))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(v1 - point_t(1.84763, -2.39052, 4.47631))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(rk4_uut.project_translation(f, i3, &v1, point_t(-1.0, 1.0, -1.0), 1.0) - point_t(-0.844839, 0.71269, -0.281726))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(v1 - point_t(-0.717456, 0.536492, 0.15877))) < result_tolerance);
}


/* Note -- Dropped zero time test, too boring */
BOOST_AUTO_TEST_CASE( timestep_sinusoidal_force_rk4_test )
{
    std::unique_ptr<sin_force> sf0(new sin_force(point_t(0.0, 0.0, 0.0), point_t(-1.0, 4.0, -10.0), point_t(2.0, -4.0, 10.0), 0.75, 1.5 * PI, 5.0));
    std::vector<force *> forces({sf0.get()});
    aggregate_force f(forces);

    BOOST_CHECK(fabs(magnitude(rk4_uut.project_translation(f, i0, &v1, point_t(1.0, 1.0, 1.0), 0.5) - point_t(0.580889, 0.301443, 0.996392))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(v1 - point_t(1.28799, 0.348031, 2.62992))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(rk4_uut.project_translation(f, i1, &v1, point_t(1.0, -1.0, 1.0), 0.75) - point_t(1.05984, -1.42684, 2.4421))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(v1 - point_t(1.70598, -2.32394, 4.30985))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(rk4_uut.project_translation(f, i3, &v1, point_t(-1.0, 1.0, -1.0), 0.9) - point_t(-0.766012, 0.63405, -0.235124))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(v1 - point_t(-0.7386, 0.554398, 0.114004))) < result_tolerance);
}


BOOST_AUTO_TEST_CASE( basic_sinusoidal_torque_rk4_test )
{
    std::unique_ptr<sin_force> sf0(new sin_force(point_t(0.0, 1.0, 0.0), point_t(-1.0, 4.0, -10.0), point_t(2.0, -4.0, 10.0), 0.75, 1.5 * PI, 5.0));
    std::vector<force *> forces({sf0.get()});
    aggregate_force f(forces);

    /* These rotations are all much to big for the linear approximation used, hence we see some strange quaternions */
    const quaternion_t start(1.0, 0.0, 0.0, 0.0);
    BOOST_CHECK_MESSAGE(fabs(magnitude(rk4_uut.project_rotation(f, i0, &v1, start, point_t(PI, 0.0, 0.0), 0.1) - quaternion_t(0.984608, 0.174753, -0.000144818, -0.00278091) + start)) < result_tolerance, rk4_uut.project_rotation(f, i0, &v1, start, point_t(PI, 0.0, 0.0), 0.1) + start);
    BOOST_CHECK_MESSAGE(fabs(magnitude(v1 - point_t(3.87787, 0.0, -0.111128))) < result_tolerance, v1);

    BOOST_CHECK_MESSAGE(fabs(magnitude(rk4_uut.project_rotation(f, i1, &v1, start, point_t(0.0, -PI, 0.0), 0.1) - quaternion_t(0.986987, 0.036704, -0.156397, -0.00748994) + start)) < result_tolerance, rk4_uut.project_rotation(f, i1, &v1, start, point_t(0.0, -PI, 0.0), 0.1) + start);
    BOOST_CHECK_MESSAGE(fabs(magnitude(v1 - point_t(1.47256, -3.14159, -0.222257))) < result_tolerance, v1);

    BOOST_CHECK_MESSAGE(fabs(magnitude(rk4_uut.project_rotation(f, i3, &v1, start, point_t(0.0, 0.0, -PI), 0.1) - quaternion_t(0.987319, 0.0123327, 0.000641318, -0.15827) + start)) < result_tolerance, rk4_uut.project_rotation(f, i3, &v1, start, point_t(0.0, 0.0, -PI), 0.1) + start);
    BOOST_CHECK_MESSAGE(fabs(magnitude(v1 - point_t(0.490854, 0.0, -3.21568))) < result_tolerance, v1);
}


BOOST_AUTO_TEST_CASE( timestep_sinusoidal_torque_rk4_test )
{
    std::unique_ptr<sin_force> sf0(new sin_force(point_t(0.0, 1.0, 0.0), point_t(-1.0, 4.0, -10.0), point_t(2.0, -4.0, 10.0), 0.75, 1.5 * PI, 5.0));
    std::vector<force *> forces({sf0.get()});
    aggregate_force f(forces);
    
    const quaternion_t start(1.0, 0.0, 0.0, 0.0);
    BOOST_CHECK_MESSAGE(fabs(magnitude(rk4_uut.project_rotation(f, i0, &v1, start, point_t(PI, 0.0, 0.0), 0.05) - quaternion_t(0.996539, 0.0831204, 0.0, -0.000701231) + start)) < result_tolerance, rk4_uut.project_rotation(f, i0, &v1, start, point_t(PI, 0.0, 0.0), 0.05) + start);
    BOOST_CHECK_MESSAGE(fabs(magnitude(v1 - point_t(3.51486, 0.0, -0.0560769))) < result_tolerance, v1);

    BOOST_CHECK_MESSAGE(fabs(magnitude(rk4_uut.project_rotation(f, i1, &v1, start, point_t(0.0, -PI, 0.0), 0.075) - quaternion_t(0.992844, 0.0208112, -0.117528, -0.00396455) + start)) < result_tolerance, rk4_uut.project_rotation(f, i1, &v1, start, point_t(0.0, -PI, 0.0), 0.075) + start);
    BOOST_CHECK_MESSAGE(fabs(magnitude(v1 - point_t(1.11337, -3.14159, -0.167587))) < result_tolerance, v1);

    BOOST_CHECK_MESSAGE(fabs(magnitude(rk4_uut.project_rotation(f, i3, &v1, start, point_t(0.0, -PI, 0.0), 0.09) - quaternion_t(0.989972, 0.00994522, -0.140898, -0.00197555) + start)) < result_tolerance, rk4_uut.project_rotation(f, i3, &v1, start, point_t(0.0, -PI), 0.09) + start);
    BOOST_CHECK_MESSAGE(fabs(magnitude(v1 - point_t(0.443318, -3.14159, -0.0668317))) < result_tolerance, v1);
}


/* Attractive force tests */
/* Note -- RK4 and Euler do very badly on these tests */
BOOST_AUTO_TEST_CASE( basic_attractive_force_rk4_test )
{
    std::unique_ptr<attract_force> af0(new attract_force(point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 12.3, -1.2, 5.0));
    std::vector<force *> forces({af0.get()});
    aggregate_force f(forces);

    BOOST_CHECK_MESSAGE(fabs(magnitude(rk4_uut.project_translation(f, i0, &v1, point_t(1.0, 1.0, 1.0), 1.0) - point_t(0.44114, 1.73598, 0.44114))) < result_tolerance, rk4_uut.project_translation(f, i0, &v1, point_t(1.0, 1.0, 1.0), 1.0));
    BOOST_CHECK_MESSAGE(fabs(magnitude(v1 - point_t(0.210262, 1.11686, 0.210262))) < result_tolerance, v1);

    BOOST_CHECK_MESSAGE(fabs(magnitude(rk4_uut.project_translation(f, i1, &v1, point_t(1.0, -1.0, 1.0), 1.0) - point_t(-0.596962, 0.504954, -0.596962))) < result_tolerance, rk4_uut.project_translation(f, i1, &v1, point_t(1.0, -1.0, 1.0), 1.0));
    BOOST_CHECK_MESSAGE(fabs(magnitude(v1 - point_t(-2.00128, 2.38427, -2.00128))) < result_tolerance, v1);

    BOOST_CHECK_MESSAGE(fabs(magnitude(rk4_uut.project_translation(f, i3, &v1, point_t(-1.0, 1.0, -1.0), 1.0) - point_t(-0.784057, 0.784057, -0.784057))) < result_tolerance, rk4_uut.project_translation(f, i3, &v1, point_t(-1.0, 1.0, -1.0), 1.0));
    BOOST_CHECK_MESSAGE(fabs(magnitude(v1 - point_t(-0.619767, 0.619767, -0.619767))) < result_tolerance, v1);
}


BOOST_AUTO_TEST_CASE( timestep_attractive_force_rk4_test )
{
    std::unique_ptr<attract_force> af0(new attract_force(point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 12.3, -1.2, 5.0));
    std::vector<force *> forces({af0.get()});
    aggregate_force f(forces);

    BOOST_CHECK_MESSAGE(fabs(magnitude(rk4_uut.project_translation(f, i0, &v1, point_t(1.0, 1.0, 1.0), 0.5) - point_t(0.113293, 0.804187, 0.113293))) < result_tolerance, rk4_uut.project_translation(f, i0, &v1, point_t(1.0, 1.0, 1.0), 0.5));
    BOOST_CHECK_MESSAGE(fabs(magnitude(v1 - point_t(-0.556087, 1.23255, -0.556087))) < result_tolerance, v1);

    BOOST_CHECK_MESSAGE(fabs(magnitude(rk4_uut.project_translation(f, i1, &v1, point_t(1.0, -1.0, 1.0), 0.75) - point_t(-0.229946, -0.393863, -0.229946))) < result_tolerance, rk4_uut.project_translation(f, i1, &v1, point_t(1.0, -1.0, 1.0), 0.75));
    BOOST_CHECK_MESSAGE(fabs(magnitude(v1 - point_t(-1.46272, 0.169303, -1.46272))) < result_tolerance, v1);

    BOOST_CHECK_MESSAGE(fabs(magnitude(rk4_uut.project_translation(f, i3, &v1, point_t(-1.0, 1.0, -1.0), 0.9) - point_t(-0.721794, 0.721794, -0.721794))) < result_tolerance, rk4_uut.project_translation(f, i3, &v1, point_t(-1.0, 1.0, -1.0), 0.9));
    BOOST_CHECK_MESSAGE(fabs(magnitude(v1 - point_t(-0.648112, 0.648112, -0.64811))) < result_tolerance, v1);
}


BOOST_AUTO_TEST_CASE( basic_attractive_torque_rk4_test )
{
    std::unique_ptr<attract_force> af0(new attract_force(point_t(0.0, 0.0, 1.5), point_t(0.0, 0.0, 0.0), 12.3, -1.2, 5.0));
    std::vector<force *> forces({af0.get()});
    aggregate_force f(forces);

    /* These rotations are all much to big for the linear approximation used, hence we see some strange quaternions */
    const quaternion_t start(1.0, 0.0, 0.0, 0.0);
    BOOST_CHECK_MESSAGE(fabs(magnitude(rk4_uut.project_rotation(f, i0, &v1, start, point_t(PI, 0.0, 0.0), 0.1) - quaternion_t(0.9923, 0.123854, 0.0, 0.0) + start)) < result_tolerance, rk4_uut.project_rotation(f, i0, &v1, start, point_t(PI, 0.0, 0.0), 0.1) + start);
    BOOST_CHECK_MESSAGE(fabs(magnitude(v1 - point_t(1.82534, 0.0, 0.0))) < result_tolerance, v1);

    BOOST_CHECK_MESSAGE(fabs(magnitude(rk4_uut.project_rotation(f, i1, &v1, start, point_t(0.0, -PI, 0.0), 0.1) - quaternion_t(0.983994, 0.0, -0.178196, 0.0) + start)) < result_tolerance, rk4_uut.project_rotation(f, i1, &v1, start, point_t(0.0, -PI, 0.0), 0.1) + start);
    BOOST_CHECK_MESSAGE(fabs(magnitude(v1 - point_t(0.0, -4.02459, 0.0))) < result_tolerance, v1);

    BOOST_CHECK_MESSAGE(fabs(magnitude(rk4_uut.project_rotation(f, i3, &v1, start, point_t(0.0, 0.0, -PI), 0.1) - quaternion_t(0.987678, 0.00301488, 0.00334732, -0.156433) + start)) < result_tolerance, rk4_uut.project_rotation(f, i3, &v1, start, point_t(0.0, 0.0, -PI), 0.1) + start);
    BOOST_CHECK_MESSAGE(fabs(magnitude(v1 - point_t(0.127769, 0.127769, -3.14159))) < result_tolerance, v1);
}


BOOST_AUTO_TEST_CASE( timestep_attractive_torque_rk4_test )
{
    std::unique_ptr<attract_force> af0(new attract_force(point_t(0.0, 0.0, 1.5), point_t(0.0, 0.0, 0.0), 12.3, -1.2, 5.0));
    std::vector<force *> forces({af0.get()});
    aggregate_force f(forces);

    const quaternion_t start(1.0, 0.0, 0.0, 0.0);
    BOOST_CHECK_MESSAGE(fabs(magnitude(rk4_uut.project_rotation(f, i0, &v1, start, point_t(PI, 0.0, 0.0), 0.05) - quaternion_t(0.997529, 0.0702553, 0.0, 0.0) + start)) < result_tolerance, rk4_uut.project_rotation(f, i0, &v1, start, point_t(PI, 0.0, 0.0), 0.05) + start);
    BOOST_CHECK_MESSAGE(fabs(magnitude(v1 - point_t(2.48347, 0.0, 0.0))) < result_tolerance, v1);

    BOOST_CHECK_MESSAGE(fabs(magnitude(rk4_uut.project_rotation(f, i1, &v1, start, point_t(0.0, -PI, 0.0), 0.075) - quaternion_t(0.991532, 0.0, -0.129859, 0.0) + start)) < result_tolerance, rk4_uut.project_rotation(f, i1, &v1, start, point_t(0.0, -PI, 0.0), 0.075) + start);
    BOOST_CHECK_MESSAGE(fabs(magnitude(v1 - point_t(0.0, -3.80384, 0.0))) < result_tolerance, v1);

    BOOST_CHECK_MESSAGE(fabs(magnitude(rk4_uut.project_rotation(f, i3, &v1, start, point_t(0.0, -PI, 0.0), 0.09) - quaternion_t(0.990382, 0.00257902, -0.138339, -0.000121339) + start)) < result_tolerance, rk4_uut.project_rotation(f, i3, &v1, start, point_t(0.0, -PI), 0.09) + start);
    BOOST_CHECK_MESSAGE(fabs(magnitude(v1 - point_t(0.114992, -3.0266, 0.0))) < result_tolerance, v1);
}


/* Viscous force tests */
/* Note -- RK4 is pretty good on these tests, Euler over estimates the force so v and x are too small */
BOOST_AUTO_TEST_CASE( basic_viscous_force_rk4_test )
{
    std::unique_ptr<viscous_force> vf0(new viscous_force(point_t(0.0, 0.0, 0.0), 2.1, 5.0));
    std::vector<force *> forces({vf0.get()});
    aggregate_force f(forces);

    BOOST_CHECK_MESSAGE(fabs(magnitude(rk4_uut.project_translation(f, i0, &v1, point_t(1.0, 1.0, 1.0), 1.0) - point_t(0.777408, 0.777408, 0.777408))) < result_tolerance, rk4_uut.project_translation(f, i0, &v1, point_t(1.0, 1.0, 1.0), 1.0));
    BOOST_CHECK_MESSAGE(fabs(magnitude(v1 - point_t(0.591861, 0.591861, 0.591861))) < result_tolerance, v1);

    BOOST_CHECK_MESSAGE(fabs(magnitude(rk4_uut.project_translation(f, i1, &v1, point_t(1.0, -1.0, 1.0), 1.0) - point_t(0.610516, -0.610516, 0.610516))) < result_tolerance, rk4_uut.project_translation(f, i1, &v1, point_t(1.0, -1.0, 1.0), 1.0));
    BOOST_CHECK_MESSAGE(fabs(magnitude(v1 - point_t(0.358959, -0.358959, 0.358959))) < result_tolerance, v1);

    BOOST_CHECK_MESSAGE(fabs(magnitude(rk4_uut.project_translation(f, i3, &v1, point_t(-1.0, 1.0, -1.0), 1.0) - point_t(-0.843748, 0.843748, -0.843748))) < result_tolerance, rk4_uut.project_translation(f, i3, &v1, point_t(-1.0, 1.0, -1.0), 1.0));
    BOOST_CHECK_MESSAGE(fabs(magnitude(v1 - point_t(-0.704688, 0.704688, -0.704688))) < result_tolerance, v1);
}


BOOST_AUTO_TEST_CASE( timestep_viscous_force_rk4_test )
{
    std::unique_ptr<viscous_force> vf0(new viscous_force(point_t(0.0, 0.0, 0.0), 2.1, 5.0));
    std::vector<force *> forces({vf0.get()});
    aggregate_force f(forces);

    BOOST_CHECK_MESSAGE(fabs(magnitude(rk4_uut.project_translation(f, i0, &v1, point_t(1.0, 1.0, 1.0), 0.5) - point_t(0.439759, 0.439759, 0.439759))) < result_tolerance, rk4_uut.project_translation(f, i0, &v1, point_t(1.0, 1.0, 1.0), 0.5));
    BOOST_CHECK_MESSAGE(fabs(magnitude(v1 - point_t(0.769126, 0.769126, 0.769126))) < result_tolerance, v1);

    BOOST_CHECK_MESSAGE(fabs(magnitude(rk4_uut.project_translation(f, i1, &v1, point_t(1.0, -1.0, 1.0), 0.75) - point_t(0.516945, -0.516945, 0.516945))) < result_tolerance, rk4_uut.project_translation(f, i1, &v1, point_t(1.0, -1.0, 1.0), 0.75));
    BOOST_CHECK_MESSAGE(fabs(magnitude(v1 - point_t(0.457207, -0.457207, 0.457207))) < result_tolerance, v1);

    BOOST_CHECK_MESSAGE(fabs(magnitude(rk4_uut.project_translation(f, i3, &v1, point_t(-1.0, 1.0, -1.0), 0.9) - point_t(-0.772032, 0.772032, -0.772032))) < result_tolerance, rk4_uut.project_translation(f, i3, &v1, point_t(-1.0, 1.0, -1.0), 0.9));
    BOOST_CHECK_MESSAGE(fabs(magnitude(v1 - point_t(-0.729789, 0.729789, -0.729789))) < result_tolerance, v1);
}


BOOST_AUTO_TEST_CASE( basic_viscous_torque_rk4_test )
{
    std::unique_ptr<viscous_force> vf0(new viscous_force(point_t(2.1, 0.0, 0.0), 2.1, 5.0));
    std::vector<force *> forces({vf0.get()});
    aggregate_force f(forces);

    /* These rotations are all much to big for the linear approximation used, hence we see some strange quaternions */
    const quaternion_t start(1.0, 0.0, 0.0, 0.0);
    BOOST_CHECK_MESSAGE(fabs(magnitude(rk4_uut.project_rotation(f, i0, &v1, start, point_t(PI, 0.0, 0.0), 0.1) - quaternion_t(0.987688, 0.156434, 0.0, 0.0) + start)) < result_tolerance, rk4_uut.project_rotation(f, i0, &v1, start, point_t(PI, 0.0, 0.0), 0.1) + start);
    BOOST_CHECK_MESSAGE(fabs(magnitude(v1 - point_t(3.14159, 0.0, 0.0))) < result_tolerance, v1);

    BOOST_CHECK_MESSAGE(fabs(magnitude(rk4_uut.project_rotation(f, i1, &v1, start, point_t(0.0, -PI, 0.0), 0.1) - quaternion_t(0.987799, 0.00133463, -0.153587, 0.0256369) + start)) < result_tolerance, rk4_uut.project_rotation(f, i1, &v1, start, point_t(0.0, -PI, 0.0), 0.1) + start);
    BOOST_CHECK_MESSAGE(fabs(magnitude(v1 - point_t(0.0, -2.97132, 1.02014))) < result_tolerance, v1);

    BOOST_CHECK_MESSAGE(fabs(magnitude(rk4_uut.project_rotation(f, i3, &v1, start, point_t(0.0, 0.0, -PI), 0.1) - quaternion_t(0.987701, 0.000449927, -0.0086148, -0.156117) + start)) < result_tolerance, rk4_uut.project_rotation(f, i3, &v1, start, point_t(0.0, 0.0, -PI), 0.1) + start);
    BOOST_CHECK_MESSAGE(fabs(magnitude(v1 - point_t(0.0, -0.345659, -3.12252))) < result_tolerance, v1);
}


BOOST_AUTO_TEST_CASE( timestep_viscous_torque_rk4_test )
{
    std::unique_ptr<viscous_force> vf0(new viscous_force(point_t(2.1, 0.0, 0.0), 2.1, 5.0));
    std::vector<force *> forces({vf0.get()});
    aggregate_force f(forces);

    const quaternion_t start(1.0, 0.0, 0.0, 0.0);
    BOOST_CHECK_MESSAGE(fabs(magnitude(rk4_uut.project_rotation(f, i0, &v1, start, point_t(PI, 0.0, 0.0), 0.05) - quaternion_t(0.996917, 0.0784591, 0.0, 0.0) + start)) < result_tolerance, rk4_uut.project_rotation(f, i0, &v1, start, point_t(PI, 0.0, 0.0), 0.05) + start);
    BOOST_CHECK_MESSAGE(fabs(magnitude(v1 - point_t(3.14159, 0.0, 0.0))) < result_tolerance, v1);

    BOOST_CHECK_MESSAGE(fabs(magnitude(rk4_uut.project_rotation(f, i1, &v1, start, point_t(0.0, -PI, 0.0), 0.075) - quaternion_t(0.993104, 0.000567602, -0.116333, 0.014504) + start)) < result_tolerance, rk4_uut.project_rotation(f, i1, &v1, start, point_t(0.0, -PI, 0.0), 0.075) + start);
    BOOST_CHECK_MESSAGE(fabs(magnitude(v1 - point_t(0.0, -3.04543, 0.771321))) < result_tolerance, v1);

    BOOST_CHECK_MESSAGE(fabs(magnitude(rk4_uut.project_rotation(f, i3, &v1, start, point_t(0.0, -PI, 0.0), 0.09) - quaternion_t(0.990032, 0.000328472, -0.14067, 0.00698477) + start)) < result_tolerance, rk4_uut.project_rotation(f, i3, &v1, start, point_t(0.0, -PI), 0.09) + start);
    BOOST_CHECK_MESSAGE(fabs(magnitude(v1 - point_t(0.0, -3.12614, 0.311213))) < result_tolerance, v1);
}

BOOST_AUTO_TEST_SUITE_END()
}; /* namespace test */
}; /* namespace raptor_physics */
