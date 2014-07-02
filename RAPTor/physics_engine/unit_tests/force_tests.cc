#ifdef STAND_ALONE
#define BOOST_TEST_MODULE force test

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
#include "force.h"


using namespace raptor_physics;

/* Test data */
struct force_fixture
{
    force_fixture()
    : i0(new fp_t[6]{ 0.0, 0.0, 0.0, 0.0,  0.0, 0.0 }, point_t(0.0, 0.0,    0.0 ), 0.0),
      i1(new fp_t[6]{ 0.0, 0.0, 0.0, 0.0,  0.0, 0.0 }, point_t(3.0, 1.5,   -7.5 ), 0.0),
      i2(new fp_t[6]{ 0.0, 0.0, 0.0, 0.0,  0.0, 0.0 }, point_t(6.0, 3.0,  -15.0 ), 0.0),
      i3(new fp_t[6]{ 0.0, 0.0, 0.0, 0.0,  0.0, 0.0 }, point_t(8.0, 4.0,  -20.0 ), 0.0),
      i4(new fp_t[6]{ 0.0, 0.0, 0.0, 0.0,  0.0, 0.0 }, point_t(9.0, 4.5,  -22.5 ), 0.0),
      i5(new fp_t[6]{ 0.0, 0.0, 0.0, 0.0,  0.0, 0.0 }, point_t(9.5, 4.75, -23.75), 0.0)
    {  };

    inertia_tensor      i0;
    inertia_tensor      i1;
    inertia_tensor      i2;
    inertia_tensor      i3;
    inertia_tensor      i4;
    inertia_tensor      i5;
};


BOOST_FIXTURE_TEST_SUITE( force_tests, force_fixture )

const float result_tolerance = 0.0005;


/* Const force tests */
BOOST_AUTO_TEST_CASE( const_force_test )
{
    const_force uut(point_t(1.5, 0.0, 0.0), point_t(1.2, -7.8, 4.8), 5.0);
    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.0) - point_t(1.2, -7.8, 4.8))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.0) - point_t(0.0, -7.2, -11.7))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0) - point_t(1.2, -7.8, 4.8))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0) - point_t(0.0, -7.2, -11.7))) < result_tolerance);
    BOOST_CHECK(!uut.commit(1.0));

    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0) - point_t(1.2, -7.8, 4.8))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0) - point_t(0.0, -7.2, -11.7))) < result_tolerance);
    BOOST_CHECK(!uut.commit(1.0));

    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0) - point_t(1.2, -7.8, 4.8))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0) - point_t(0.0, -7.2, -11.7))) < result_tolerance);
    BOOST_CHECK(!uut.commit(1.0));

    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0) - point_t(1.2, -7.8, 4.8))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0) - point_t(0.0, -7.2, -11.7))) < result_tolerance);
    BOOST_CHECK(!uut.commit(1.0));

    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0) - point_t(1.2, -7.8, 4.8))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0) - point_t(0.0, -7.2, -11.7))) < result_tolerance);
    BOOST_CHECK(uut.commit(1.0));
}


/* Linear force tests */
BOOST_AUTO_TEST_CASE( linear_force_no_c_test )
{
    linear_force uut(point_t(0.0, -6.7, 0.0), point_t(1.2, -7.8, 4.8), point_t(0.0, 0.0, 0.0), 5.0);
    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.0) - point_t(0.0, 0.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.0) - point_t(0.0, 0.0, 0.0))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0) - point_t(1.2, -7.8, 4.8))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0) - point_t(-32.16, 0.0, 8.04))) < result_tolerance);
    BOOST_CHECK(!uut.commit(1.0));

    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0) - point_t(2.4, -15.6, 9.6))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0) - point_t(-64.32, 0.0, 16.08))) < result_tolerance);
    BOOST_CHECK(!uut.commit(1.0));

    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0) - point_t(3.6, -23.4, 14.4))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0) - point_t(-96.48, 0.0, 24.12))) < result_tolerance);
    BOOST_CHECK(!uut.commit(1.0));

    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0) - point_t(4.8, -31.2, 19.2))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0) - point_t(-128.64, 0.0, 32.16))) < result_tolerance);
    BOOST_CHECK(!uut.commit(1.0));

    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0) - point_t(6.0, -39.0, 24.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0) - point_t(-160.8, 0.0, 40.2))) < result_tolerance);
    BOOST_CHECK(uut.commit(1.0));
}


BOOST_AUTO_TEST_CASE( linear_force_test )
{
    linear_force uut(point_t(0.0, -6.7, 0.0), point_t(1.2, -7.8, 4.8), point_t(1.7, 9.4, -6.3), 5.0);
    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.0) - point_t(1.7, 9.4, -6.3))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.0) - point_t(42.21, 0.0, 11.39))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0) - point_t(2.9, 1.6, -1.5))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0) - point_t(10.05, 0.0, 19.43))) < result_tolerance);
    BOOST_CHECK(!uut.commit(1.0));

    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0) - point_t(4.1, -6.2, 3.3))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0) - point_t(-22.11, 0.0, 27.47))) < result_tolerance);
    BOOST_CHECK(!uut.commit(1.0));

    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0) - point_t(5.3, -14.0, 8.1))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0) - point_t(-54.27, 0.0, 35.51))) < result_tolerance);
    BOOST_CHECK(!uut.commit(1.0));

    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0) - point_t(6.5, -21.8, 12.9))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0) - point_t(-86.43, 0.0, 43.55))) < result_tolerance);
    BOOST_CHECK(!uut.commit(1.0));

    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0) - point_t(7.7, -29.6, 17.7))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0) - point_t(-118.59, 0.0, 51.59))) < result_tolerance);
    BOOST_CHECK(uut.commit(1.0));
}


/* Squared force tests */
BOOST_AUTO_TEST_CASE( squared_force_no_c_test )
{
    squared_force uut(point_t(0.0, 0.0, -1.9), point_t(1.2, -1.8, 1.8), point_t(0.0, 0.0, 0.0), 5.0);
    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.0) - point_t(0.0, 0.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.0) - point_t(0.0, 0.0, 0.0))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0) - point_t(1.2, -1.8, 1.8))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0) - point_t(-3.42, -2.28, 0.0))) < result_tolerance);
    BOOST_CHECK(!uut.commit(1.0));

    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0) - point_t(4.8, -7.2, 7.2))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0) - point_t(-13.68, -9.12, 0.0))) < result_tolerance);
    BOOST_CHECK(!uut.commit(1.0));

    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0) - point_t(10.8, -16.2, 16.2))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0) - point_t(-30.78, -20.52, 0.0))) < result_tolerance);
    BOOST_CHECK(!uut.commit(1.0));

    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0) - point_t(19.2, -28.8, 28.8))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0) - point_t(-54.72, -36.48, 0.0))) < result_tolerance);
    BOOST_CHECK(!uut.commit(1.0));

    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0) - point_t(30.0, -45.0, 45.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0) - point_t(-85.5, -57.0, 0.0))) < result_tolerance);
    BOOST_CHECK(uut.commit(1.0));
}


BOOST_AUTO_TEST_CASE( squared_force_test )
{
    squared_force uut(point_t(0.0, 0.0, -1.9), point_t(1.2, -1.8, 1.8), point_t(-1.7, -9.4, 6.3), 5.0);
    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.0) - point_t(-1.7, -9.4, 6.3))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.0) - point_t(-17.86, 3.23, 0.0))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0) - point_t(-0.5, -11.2, 8.1))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0) - point_t(-21.28, 0.95, 0.0))) < result_tolerance);
    BOOST_CHECK(!uut.commit(1.0));

    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0) - point_t(3.1, -16.6, 13.5))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0) - point_t(-31.54, -5.89, 0.0))) < result_tolerance);
    BOOST_CHECK(!uut.commit(1.0));

    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0) - point_t(9.1, -25.6, 22.5))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0) - point_t(-48.64, -17.29, 0.0))) < result_tolerance);
    BOOST_CHECK(!uut.commit(1.0));

    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0) - point_t(17.5, -38.2, 35.1))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0) - point_t(-72.58, -33.25, 0.0))) < result_tolerance);
    BOOST_CHECK(!uut.commit(1.0));

    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0) - point_t(28.3, -54.4, 51.3))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0) - point_t(-103.36, -53.77, 0.0))) < result_tolerance);
    BOOST_CHECK(uut.commit(1.0));
}


/* Atractive force tests */
BOOST_AUTO_TEST_CASE( attractive_force_no_c_test )
{
    attract_force uut(point_t(0.0, 2.0, -1.9), point_t(10.0, 5.0, -25.0), 5.8, 0.0, 5.0);
    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.0) - point_t(0.0773333, 0.0386667, -0.193333))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.0) - point_t(-0.3132, -0.146933, -0.154667))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(uut.get_force(i1, point_t(3.0, 1.5, -7.5), point_t(0.0, 0.0, 0.0), 1.0) - point_t(0.110476, 0.0552381, -0.27619))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i1, point_t(3.0, 1.5, -7.5), point_t(0.0, 0.0, 0.0), 1.0) - point_t(-0.447429, -0.209905, -0.220952))) < result_tolerance);
    BOOST_CHECK(!uut.commit(1.0));

    BOOST_CHECK(fabs(magnitude(uut.get_force(i2, point_t(6.0, 3.0, -15.0), point_t(0.0, 0.0, 0.0), 1.0) - point_t(0.193333, 0.0966667, -0.483333))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i2, point_t(6.0, 3.0, -15.0), point_t(0.0, 0.0, 0.0), 1.0) - point_t(-0.783, -0.367333, -0.386667))) < result_tolerance);
    BOOST_CHECK(!uut.commit(1.0));

    BOOST_CHECK(fabs(magnitude(uut.get_force(i3, point_t(8.0, 4.0, -20.0), point_t(0.0, 0.0, 0.0), 1.0) - point_t(0.386667, 0.193333, -0.966667))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i3, point_t(8.0, 4.0, -20.0), point_t(0.0, 0.0, 0.0), 1.0) - point_t(-1.566, -0.734667, -0.773333))) < result_tolerance);
    BOOST_CHECK(!uut.commit(1.0));

    BOOST_CHECK(fabs(magnitude(uut.get_force(i4, point_t(9.0, 4.5, -22.5), point_t(0.0, 0.0, 0.0), 1.0) - point_t(0.773333, 0.386667, -1.93333))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i4, point_t(9.0, 4.5, -22.5), point_t(0.0, 0.0, 0.0), 1.0) - point_t(-3.132, -1.46933, -1.54667))) < result_tolerance);
    BOOST_CHECK(!uut.commit(1.0));

    BOOST_CHECK(fabs(magnitude(uut.get_force(i5, point_t(9.5, 4.75, -23.75), point_t(0.0, 0.0, 0.0), 1.0) - point_t(1.54667, 0.773333, -3.86667))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i5, point_t(9.5, 4.75, -23.75), point_t(0.0, 0.0, 0.0), 1.0) - point_t(-6.264, -2.93867, -3.09333))) < result_tolerance);
    BOOST_CHECK(uut.commit(1.0));
}


BOOST_AUTO_TEST_CASE( attractive_force_test )
{
    attract_force uut(point_t(0.0, 2.0, -1.9), point_t(10.0, 5.0, -25.0), 5.8, -2.7, 5.0);
    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.0) - point_t(-0.908567, -0.454284, 2.27142))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.0) - point_t(3.6797, 1.72628, 1.81713))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(uut.get_force(i1, point_t(3.0, 1.5, -7.5), point_t(0.0, 0.0, 0.0), 1.0) - point_t(-0.875424, -0.437712, 2.18856))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i1, point_t(3.0, 1.5, -7.5), point_t(0.0, 0.0, 0.0), 1.0) - point_t(3.54547, 1.66331, 1.75085))) < result_tolerance);
    BOOST_CHECK(!uut.commit(1.0));

    BOOST_CHECK(fabs(magnitude(uut.get_force(i2, point_t(6.0, 3.0, -15.0), point_t(0.0, 0.0, 0.0), 1.0) - point_t(-0.792567, -0.396284, 1.98142))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i2, point_t(6.0, 3.0, -15.0), point_t(0.0, 0.0, 0.0), 1.0) - point_t(3.2099, 1.50588, 1.58513))) < result_tolerance);
    BOOST_CHECK(!uut.commit(1.0));

    BOOST_CHECK(fabs(magnitude(uut.get_force(i3, point_t(8.0, 4.0, -20.0), point_t(0.0, 0.0, 0.0), 1.0) - point_t(-0.599234, -0.299617, 1.49808))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i3, point_t(8.0, 4.0, -20.0), point_t(0.0, 0.0, 0.0), 1.0) - point_t(2.4269, 1.13854, 1.19847))) < result_tolerance);
    BOOST_CHECK(!uut.commit(1.0));

    BOOST_CHECK(fabs(magnitude(uut.get_force(i4, point_t(9.0, 4.5, -22.5), point_t(0.0, 0.0, 0.0), 1.0) - point_t(-0.212567, -0.106284, 0.531418))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i4, point_t(9.0, 4.5, -22.5), point_t(0.0, 0.0, 0.0), 1.0) - point_t(0.860897, 0.403878, 0.425135))) < result_tolerance);
    BOOST_CHECK(!uut.commit(1.0));

    BOOST_CHECK(fabs(magnitude(uut.get_force(i5, point_t(9.5, 4.75, -23.75), point_t(0.0, 0.0, 0.0), 1.0) - point_t(0.560766, 0.280383, -1.40192))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i5, point_t(9.5, 4.75, -23.75), point_t(0.0, 0.0, 0.0), 1.0) - point_t(-2.2711, -1.06546, -1.12153))) < result_tolerance);
    BOOST_CHECK(uut.commit(1.0));
}


/* Repulsive force tests */
BOOST_AUTO_TEST_CASE( repelusive_force_no_c_test )
{
    repel_force uut(point_t(0.0, 2.0, -1.9), point_t(10.0, 5.0, -25.0), 5.8, 0.0, 5.0);
    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.0) - point_t(-0.0773333, -0.0386667, 0.193333))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.0) - point_t(0.3132, 0.146933, 0.154667))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(uut.get_force(i1, point_t(3.0, 1.5, -7.5), point_t(0.0, 0.0, 0.0), 1.0) - point_t(-0.110476, -0.0552381, 0.27619))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i1, point_t(3.0, 1.5, -7.5), point_t(0.0, 0.0, 0.0), 1.0) - point_t(0.447429, 0.209905, 0.220952))) < result_tolerance);
    BOOST_CHECK(!uut.commit(1.0));

    BOOST_CHECK(fabs(magnitude(uut.get_force(i2, point_t(6.0, 3.0, -15.0), point_t(0.0, 0.0, 0.0), 1.0) - point_t(-0.193333, -0.0966667, 0.483333))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i2, point_t(6.0, 3.0, -15.0), point_t(0.0, 0.0, 0.0), 1.0) - point_t(0.783, 0.367333, 0.386667))) < result_tolerance);
    BOOST_CHECK(!uut.commit(1.0));

    BOOST_CHECK(fabs(magnitude(uut.get_force(i3, point_t(8.0, 4.0, -20.0), point_t(0.0, 0.0, 0.0), 1.0) - point_t(-0.386667, -0.193333, 0.966667))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i3, point_t(8.0, 4.0, -20.0), point_t(0.0, 0.0, 0.0), 1.0) - point_t(1.566, 0.734667, 0.773333))) < result_tolerance);
    BOOST_CHECK(!uut.commit(1.0));

    BOOST_CHECK(fabs(magnitude(uut.get_force(i4, point_t(9.0, 4.5, -22.5), point_t(0.0, 0.0, 0.0), 1.0) - point_t(-0.773333, -0.386667, 1.93333))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i4, point_t(9.0, 4.5, -22.5), point_t(0.0, 0.0, 0.0), 1.0) - point_t(3.132, 1.46933, 1.54667))) < result_tolerance);
    BOOST_CHECK(!uut.commit(1.0));

    BOOST_CHECK(fabs(magnitude(uut.get_force(i5, point_t(9.5, 4.75, -23.75), point_t(0.0, 0.0, 0.0), 1.0) - point_t(-1.54667, -0.773333, 3.86667))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i5, point_t(9.5, 4.75, -23.75), point_t(0.0, 0.0, 0.0), 1.0) - point_t(6.264, 2.93867, 3.09333))) < result_tolerance);
    BOOST_CHECK(uut.commit(1.0));
}


BOOST_AUTO_TEST_CASE( repelusive_force_test )
{
    repel_force uut(point_t(0.0, 2.0, -1.9), point_t(10.0, 5.0, -25.0), 5.8, -2.7, 5.0);
    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.0) - point_t(0.908567, 0.454284, -2.27142))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.0) - point_t(-3.6797, -1.72628, -1.81713))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(uut.get_force(i1, point_t(3.0, 1.5, -7.5), point_t(0.0, 0.0, 0.0), 1.0) - point_t(0.875424, 0.437712, -2.18856))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i1, point_t(3.0, 1.5, -7.5), point_t(0.0, 0.0, 0.0), 1.0) - point_t(-3.54547, -1.66331, -1.75085))) < result_tolerance);
    BOOST_CHECK(!uut.commit(1.0));

    BOOST_CHECK(fabs(magnitude(uut.get_force(i2, point_t(6.0, 3.0, -15.0), point_t(0.0, 0.0, 0.0), 1.0) - point_t(0.792567, 0.396284, -1.98142))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i2, point_t(6.0, 3.0, -15.0), point_t(0.0, 0.0, 0.0), 1.0) - point_t(-3.2099, -1.50588, -1.58513))) < result_tolerance);
    BOOST_CHECK(!uut.commit(1.0));

    BOOST_CHECK(fabs(magnitude(uut.get_force(i3, point_t(8.0, 4.0, -20.0), point_t(0.0, 0.0, 0.0), 1.0) - point_t(0.599234, 0.299617, -1.49808))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i3, point_t(8.0, 4.0, -20.0), point_t(0.0, 0.0, 0.0), 1.0) - point_t(-2.4269, -1.13854, -1.19847))) < result_tolerance);
    BOOST_CHECK(!uut.commit(1.0));

    BOOST_CHECK(fabs(magnitude(uut.get_force(i4, point_t(9.0, 4.5, -22.5), point_t(0.0, 0.0, 0.0), 1.0) - point_t(0.212567, 0.106284, -0.531418))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i4, point_t(9.0, 4.5, -22.5), point_t(0.0, 0.0, 0.0), 1.0) - point_t(-0.860897, -0.403878, -0.425135))) < result_tolerance);
    BOOST_CHECK(!uut.commit(1.0));

    BOOST_CHECK(fabs(magnitude(uut.get_force(i5, point_t(9.5, 4.75, -23.75), point_t(0.0, 0.0, 0.0), 1.0) - point_t(-0.560766, -0.280383, 1.40192))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i5, point_t(9.5, 4.75, -23.75), point_t(0.0, 0.0, 0.0), 1.0) - point_t(2.2711, 1.06546, 1.12153))) < result_tolerance);
    BOOST_CHECK(uut.commit(1.0));
}


/* Sin force tests */
BOOST_AUTO_TEST_CASE( sin_force_no_f_no_k_no_c_test )
{
    sin_force uut(point_t(5.0, 0.0, -1.9), point_t(3.2, -1.8, 5.8), point_t(0.0, 0.0, 0.0), 1.0, 0.0, 1.0);
    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.0) - point_t(0.0, 0.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.0) - point_t(0.0, 0.0, 0.0))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.2) - point_t(3.04338, -1.7119, 5.51613))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.2) - point_t(-3.25261, -33.3631, -8.55951))) < result_tolerance);
    BOOST_CHECK(!uut.commit(0.2));

    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.2) - point_t(1.88091, -1.05801, 3.40915))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.2) - point_t(-2.01023, -20.6195, -5.29007))) < result_tolerance);
    BOOST_CHECK(!uut.commit(0.2));

    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.2) - point_t(-1.88091, 1.05801, -3.40915))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.2) - point_t(2.01023, 20.6195, 5.29007))) < result_tolerance);
    BOOST_CHECK(!uut.commit(0.2));

    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.2) - point_t(-3.04338, 1.7119, -5.51613))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.2) - point_t(3.25261, 33.3631, 8.55951))) < result_tolerance);
    BOOST_CHECK(!uut.commit(0.2));

    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.2) - point_t(0.0, 0.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.2) - point_t(0.0, 0.0, 0.0))) < result_tolerance);
    BOOST_CHECK(uut.commit(0.201));
}


BOOST_AUTO_TEST_CASE( sin_force_no_k_no_c_test )
{
    sin_force uut(point_t(5.0, 0.0, -1.9), point_t(3.2, -1.8, 5.8), point_t(0.0, 0.0, 0.0), 4.0, 0.0, 0.25);
    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.0) - point_t(0.0, 0.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.0) - point_t(0.0, 0.0, 0.0))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.05) - point_t(3.04338, -1.7119, 5.51613))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.05) - point_t(-3.25261, -33.3631, -8.55951))) < result_tolerance);
    BOOST_CHECK(!uut.commit(0.05));

    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.05) - point_t(1.88091, -1.05801, 3.40915))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.05) - point_t(-2.01023, -20.6195, -5.29007))) < result_tolerance);
    BOOST_CHECK(!uut.commit(0.05));

    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.05) - point_t(-1.88091, 1.05801, -3.40915))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.05) - point_t(2.01023, 20.6195, 5.29007))) < result_tolerance);
    BOOST_CHECK(!uut.commit(0.05));

    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.05) - point_t(-3.04338, 1.7119, -5.51613))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.05) - point_t(3.25261, 33.3631, 8.55951))) < result_tolerance);
    BOOST_CHECK(!uut.commit(0.05));

    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.05) - point_t(0.0, 0.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.05) - point_t(0.0, 0.0, 0.0))) < result_tolerance);
    BOOST_CHECK(uut.commit(0.0501));
}


BOOST_AUTO_TEST_CASE( sin_force_no_c_test )
{
    sin_force uut(point_t(5.0, 0.0, -1.9), point_t(3.2, -1.8, 5.8), point_t(0.0, 0.0, 0.0), 4.0, 0.4 * PI, 0.25);
    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.0) - point_t(3.04338, -1.7119, 5.51613))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.0) - point_t(-3.25261, -33.3631, -8.55951))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.05) - point_t(1.88091, -1.05801, 3.40915))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.05) - point_t(-2.01023, -20.6195, -5.29007))) < result_tolerance);
    BOOST_CHECK(!uut.commit(0.05));

    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.05) - point_t(-1.88091, 1.05801, -3.40915))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.05) - point_t(2.01023, 20.6195, 5.29007))) < result_tolerance);
    BOOST_CHECK(!uut.commit(0.05));

    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.05) - point_t(-3.04338, 1.7119, -5.51613))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.05) - point_t(3.25261, 33.3631, 8.55951))) < result_tolerance);
    BOOST_CHECK(!uut.commit(0.05));

    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.05) - point_t(0.0, 0.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.05) - point_t(0.0, 0.0, 0.0))) < result_tolerance);
    BOOST_CHECK(!uut.commit(0.05));

    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.05) - point_t(3.04338, -1.7119, 5.51613))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.05) - point_t(-3.25261, -33.3631, -8.55951))) < result_tolerance);
    BOOST_CHECK(uut.commit(0.0501));
}


BOOST_AUTO_TEST_CASE( sin_force_test )
{
    sin_force uut(point_t(5.0, 0.0, -1.9), point_t(3.2, -1.8, 5.8), point_t(0.4, -7.1, 5.1), 4.0, 0.4 * PI, 0.25);
    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.0) - point_t(3.44338, -8.8119, 10.6161))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.0) - point_t(-16.7426, -59.6231, -44.0595))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.05) - point_t(2.28091, -8.15801, 8.50915))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.05) - point_t(-15.5002, -46.8795, -40.7901))) < result_tolerance);
    BOOST_CHECK(!uut.commit(0.05));

    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.05) - point_t(-1.48091, -6.04199, 1.69085))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.05) - point_t(-11.4798, -5.64049, -30.2099))) < result_tolerance);
    BOOST_CHECK(!uut.commit(0.05));

    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.05) - point_t(-2.64338, -5.3881, -0.416128))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.05) - point_t(-10.2374, 7.10306, -26.9405))) < result_tolerance);
    BOOST_CHECK(!uut.commit(0.05));

    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.05) - point_t(0.4, -7.1, 5.1))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.05) - point_t(-13.49, -26.26, -35.5))) < result_tolerance);
    BOOST_CHECK(!uut.commit(0.05));

    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.05) - point_t(3.44338, -8.8119, 10.6161))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.05) - point_t(-16.7426, -59.6231, -44.0595))) < result_tolerance);
    BOOST_CHECK(uut.commit(0.0501));
}


/* Cos force tests */
BOOST_AUTO_TEST_CASE( cos_force_no_f_no_k_no_c_test )
{
    cos_force uut(point_t(5.0, 0.0, -1.9), point_t(3.2, -1.8, 5.8), point_t(0.0, 0.0, 0.0), 1.0, 0.0, 1.0);
    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.0) - point_t(3.2, -1.8, 5.8))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.0) - point_t(-3.42, -35.08, -9.0))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.2) - point_t(0.988854, -0.556231, 1.7923))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.2) - point_t(-1.05684, -10.8403, -2.78115))) < result_tolerance);
    BOOST_CHECK(!uut.commit(0.2));

    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.2) - point_t(-2.58885, 1.45623, -4.6923))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.2) - point_t(2.76684, 28.3803, 7.28115))) < result_tolerance);
    BOOST_CHECK(!uut.commit(0.2));

    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.2) - point_t(-2.58885, 1.45623, -4.6923))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.2) - point_t(2.76684, 28.3803, 7.28115))) < result_tolerance);
    BOOST_CHECK(!uut.commit(0.2));

    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.2) - point_t(0.988854, -0.556231, 1.7923))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.2) - point_t(-1.05684, -10.8403, -2.78115))) < result_tolerance);
    BOOST_CHECK(!uut.commit(0.2));

    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.2) - point_t(3.2, -1.8, 5.8))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.2) - point_t(-3.42, -35.08, -9.0))) < result_tolerance);
    BOOST_CHECK(uut.commit(0.201));
}


BOOST_AUTO_TEST_CASE( cos_force_no_k_no_c_test )
{
    cos_force uut(point_t(5.0, 0.0, -1.9), point_t(3.2, -1.8, 5.8), point_t(0.0, 0.0, 0.0), 4.0, 0.0, 0.25);
    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.0) - point_t(3.2, -1.8, 5.8))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.0) - point_t(-3.42, -35.08, -9.0))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.05) - point_t(0.988854, -0.556231, 1.7923))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.05) - point_t(-1.05684, -10.8403, -2.78115))) < result_tolerance);
    BOOST_CHECK(!uut.commit(0.05));

    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.05) - point_t(-2.58885, 1.45623, -4.6923))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.05) - point_t(2.76684, 28.3803, 7.28115))) < result_tolerance);
    BOOST_CHECK(!uut.commit(0.05));

    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.05) - point_t(-2.58885, 1.45623, -4.6923))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.05) - point_t(2.76684, 28.3803, 7.28115))) < result_tolerance);
    BOOST_CHECK(!uut.commit(0.05));

    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.05) - point_t(0.988854, -0.556231, 1.7923))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.05) - point_t(-1.05684, -10.8403, -2.78115))) < result_tolerance);
    BOOST_CHECK(!uut.commit(0.05));

    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.05) - point_t(3.2, -1.8, 5.8))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.05) - point_t(-3.42, -35.08, -9.0))) < result_tolerance);
    BOOST_CHECK(uut.commit(0.0501));
}


BOOST_AUTO_TEST_CASE( cos_force_no_c_test )
{
    cos_force uut(point_t(5.0, 0.0, -1.9), point_t(3.2, -1.8, 5.8), point_t(0.0, 0.0, 0.0), 4.0, 0.4 * PI, 0.25);
    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.0) - point_t(0.988854, -0.556231, 1.7923))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.0) - point_t(-1.05684, -10.8403, -2.78115))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.05) - point_t(-2.58885, 1.45623, -4.6923))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.05) - point_t(2.76684, 28.3803, 7.28115))) < result_tolerance);
    BOOST_CHECK(!uut.commit(0.05));

    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.05) - point_t(-2.58885, 1.45623, -4.6923))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.05) - point_t(2.76684, 28.3803, 7.28115))) < result_tolerance);
    BOOST_CHECK(!uut.commit(0.05));

    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.05) - point_t(0.988854, -0.556231, 1.7923))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.05) - point_t(-1.05684, -10.8403, -2.78115))) < result_tolerance);
    BOOST_CHECK(!uut.commit(0.05));

    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.05) - point_t(3.2, -1.8, 5.8))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.05) - point_t(-3.42, -35.08, -9.0))) < result_tolerance);
    BOOST_CHECK(!uut.commit(0.05));

    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.05) - point_t(0.988854, -0.556231, 1.7923))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.05) - point_t(-1.05684, -10.8403, -2.78115))) < result_tolerance);
    BOOST_CHECK(uut.commit(0.0501));
}


BOOST_AUTO_TEST_CASE( cos_force_test )
{
    cos_force uut(point_t(5.0, 0.0, -1.9), point_t(3.2, -1.8, 5.8), point_t(0.5, 8.4, 7.8), 4.0, 0.4 * PI, 0.25);
    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.0) - point_t(1.48885, 7.84377, 9.5923))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.0) - point_t(14.9032, -50.7903, 39.2188))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.05) - point_t(-2.08885, 9.85623, 3.1077))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.05) - point_t(18.7268, -11.5697, 49.2812))) < result_tolerance);
    BOOST_CHECK(!uut.commit(0.05));

    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.05) - point_t(-2.08885, 9.85623, 3.1077))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.05) - point_t(18.7268, -11.5697, 49.2812))) < result_tolerance);
    BOOST_CHECK(!uut.commit(0.05));

    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.05) - point_t(1.48885, 7.84377, 9.5923))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.05) - point_t(14.9032, -50.7903, 39.2188))) < result_tolerance);
    BOOST_CHECK(!uut.commit(0.05));

    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.05) - point_t(3.7, 6.6, 13.6))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.05) - point_t(12.54, -75.03, 33.0))) < result_tolerance);
    BOOST_CHECK(!uut.commit(0.05));

    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.05) - point_t(1.48885, 7.84377, 9.5923))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.05) - point_t(14.9032, -50.7903, 39.2188))) < result_tolerance);
    BOOST_CHECK(uut.commit(0.0501));
}

/* Viscous force tests */
BOOST_AUTO_TEST_CASE( viscous_force_test )
{
    viscous_force uut(point_t(5.0, 0.0, -1.9), 0.25, 1.0);
    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 1.0), 0.0) - point_t(0.0, 0.0, -0.25))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 1.0), 0.0) - point_t(0.0, 1.25, 0.0))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(-1.0, 2.5, -3.7), 0.2) - point_t(0.25, -0.625, 0.925))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(-1.0, 2.5, -3.7), 0.2) - point_t(-1.1875, -5.1, -3.125))) < result_tolerance);
    BOOST_CHECK(!uut.commit(0.2));

    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(3.7, 6.6, 13.6), 0.2) - point_t(-0.925, -1.65, -3.4))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(3.7, 6.6, 13.6), 0.2) - point_t(-3.135, 18.7575, -8.25))) < result_tolerance);
    BOOST_CHECK(!uut.commit(0.2));

    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(-1.4, 7.8, 9.5), 0.2) - point_t(0.35, -1.95, -2.375))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(-1.4, 7.8, 9.5), 0.2) - point_t(-3.705, 11.21, -9.75))) < result_tolerance);
    BOOST_CHECK(!uut.commit(0.2));

    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(14.9, -50.7, 39.2), 0.2) - point_t(-3.725, 12.675, -9.8))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(14.9, -50.7, 39.2), 0.2) - point_t(24.0825, 56.0775, 63.375))) < result_tolerance);
    BOOST_CHECK(!uut.commit(0.2));

    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(-3.725, 12.675, -9.8), 0.2) - point_t(0.93125, -3.16875, 2.45))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(-3.725, 12.675, -9.8), 0.2) - point_t(-6.02063, -14.0194, -15.8438))) < result_tolerance);
    BOOST_CHECK(uut.commit(0.2001));
}


/* Indefinate force tests */
BOOST_AUTO_TEST_CASE( indefinite_force_test )
{
    const_force uut(point_t(1.5, 0.0, 0.0), point_t(1.2, -7.8, 4.8), std::numeric_limits<fp_t>::infinity());
    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.0) - point_t(1.2, -7.8, 4.8))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.0) - point_t(0.0, -7.2, -11.7))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0e308) - point_t(1.2, -7.8, 4.8))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0e308) - point_t(0.0, -7.2, -11.7))) < result_tolerance);
    BOOST_CHECK(!uut.commit(1.0e308));

    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0e308) - point_t(1.2, -7.8, 4.8))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0e308) - point_t(0.0, -7.2, -11.7))) < result_tolerance);
    BOOST_CHECK(!uut.commit(1.0e308));

    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0e308) - point_t(1.2, -7.8, 4.8))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0e308) - point_t(0.0, -7.2, -11.7))) < result_tolerance);
    BOOST_CHECK(!uut.commit(1.0e308));

    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0e308) - point_t(1.2, -7.8, 4.8))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0e308) - point_t(0.0, -7.2, -11.7))) < result_tolerance);
    BOOST_CHECK(!uut.commit(1.0e308));

    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0e308) - point_t(1.2, -7.8, 4.8))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0e308) - point_t(0.0, -7.2, -11.7))) < result_tolerance);
    BOOST_CHECK(!uut.commit(1.0e308));

    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0e308) - point_t(1.2, -7.8, 4.8))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0e308) - point_t(0.0, -7.2, -11.7))) < result_tolerance);
    BOOST_CHECK(!uut.commit(1.0e308));

    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0e308) - point_t(1.2, -7.8, 4.8))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0e308) - point_t(0.0, -7.2, -11.7))) < result_tolerance);
    BOOST_CHECK(!uut.commit(1.0e308));

    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0e308) - point_t(1.2, -7.8, 4.8))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0e308) - point_t(0.0, -7.2, -11.7))) < result_tolerance);
    BOOST_CHECK(!uut.commit(1.0e308));

    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0e308) - point_t(1.2, -7.8, 4.8))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0e308) - point_t(0.0, -7.2, -11.7))) < result_tolerance);
    BOOST_CHECK(!uut.commit(1.0e308));

    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0e308) - point_t(1.2, -7.8, 4.8))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0e308) - point_t(0.0, -7.2, -11.7))) < result_tolerance);
    BOOST_CHECK(!uut.commit(1.0e308));
}


/* Aggregate force tests */
BOOST_AUTO_TEST_CASE( aggregate_force_test )
{
    std::unique_ptr<const_force>   cf0(new const_force(point_t(1.5, 0.0, 0.0), point_t(0.2,  7.8, 1.2), 5.0));
    std::unique_ptr<const_force>   cf1(new const_force(point_t(1.5, 0.0, 0.0), point_t(0.6, -3.4, 0.0), 5.0));
    std::unique_ptr<const_force>   cf2(new const_force(point_t(1.5, 0.0, 0.0), point_t(0.3, -4.4, 3.0), 5.0));
    std::unique_ptr<const_force>   cf3(new const_force(point_t(1.5, 0.0, 0.0), point_t(0.1, -7.8, 0.6), 5.0));
    std::unique_ptr<attract_force> af0(new attract_force(point_t(0.0, 2.0, -1.9), point_t(10.0, 5.0, -25.0), 5.8, 0.0, 5.0));
    std::vector<force *> forces({cf0.get(), cf1.get(), cf2.get(), cf3.get(), af0.get()});
    aggregate_force uut(forces);


    BOOST_CHECK(fabs(magnitude(uut.get_force(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.0) - point_t(1.27733, -7.76133, 4.60667))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i0, point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 0.0) - point_t(-0.3132, -7.34693, -11.8547))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(uut.get_force(i1, point_t(3.0, 1.5, -7.5 ), point_t(0.0, 0.0, 0.0), 1.0) - point_t(1.31048, -7.74476, 4.52381))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i1, point_t(3.0, 1.5, -7.5 ), point_t(0.0, 0.0, 0.0), 1.0) - point_t(-0.447429, -7.40991, -11.921))) < result_tolerance);
    BOOST_CHECK(!cf0->commit(1.0));
    BOOST_CHECK(!cf1->commit(1.0));
    BOOST_CHECK(!cf2->commit(1.0));
    BOOST_CHECK(!cf3->commit(1.0));
    BOOST_CHECK(!af0->commit(1.0));

    BOOST_CHECK(fabs(magnitude(uut.get_force(i2, point_t(6.0, 3.0, -15.0), point_t(0.0, 0.0, 0.0), 1.0) - point_t(1.39333, -7.70333, 4.31667))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i2, point_t(6.0, 3.0, -15.0), point_t(0.0, 0.0, 0.0), 1.0) - point_t(-0.783, -7.56733, -12.0867))) < result_tolerance);
    BOOST_CHECK(!cf0->commit(1.0));
    BOOST_CHECK(!cf1->commit(1.0));
    BOOST_CHECK(!cf2->commit(1.0));
    BOOST_CHECK(!cf3->commit(1.0));
    BOOST_CHECK(!af0->commit(1.0));

    BOOST_CHECK(fabs(magnitude(uut.get_force(i3, point_t(8.0, 4.0, -20.0), point_t(0.0, 0.0, 0.0), 1.0) - point_t(1.58667, -7.60667, 3.83333))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i3, point_t(8.0, 4.0, -20.0), point_t(0.0, 0.0, 0.0), 1.0) - point_t(-1.566, -7.93467, -12.4733))) < result_tolerance);
    BOOST_CHECK(!cf0->commit(1.0));
    BOOST_CHECK(!cf1->commit(1.0));
    BOOST_CHECK(!cf2->commit(1.0));
    BOOST_CHECK(!cf3->commit(1.0));
    BOOST_CHECK(!af0->commit(1.0));

    BOOST_CHECK(fabs(magnitude(uut.get_force(i4, point_t(9.0, 4.5, -22.5), point_t(0.0, 0.0, 0.0), 1.0) - point_t(1.97333, -7.41333, 2.86667))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i4, point_t(9.0, 4.5, -22.5), point_t(0.0, 0.0, 0.0), 1.0) - point_t(-3.132, -8.66933, -13.2467))) < result_tolerance);
    BOOST_CHECK(!cf0->commit(1.0));
    BOOST_CHECK(!cf1->commit(1.0));
    BOOST_CHECK(!cf2->commit(1.0));
    BOOST_CHECK(!cf3->commit(1.0));
    BOOST_CHECK(!af0->commit(1.0));

    BOOST_CHECK(fabs(magnitude(uut.get_force(i5, point_t(9.5, 4.75, -23.75), point_t(0.0, 0.0, 0.0), 1.0) - point_t(2.74667, -7.02667, 0.93333))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(uut.get_torque(i5, point_t(9.5, 4.75, -23.75), point_t(0.0, 0.0, 0.0), 1.0) - point_t(-6.264, -10.1387, -14.7933))) < result_tolerance);
    BOOST_CHECK(cf0->commit(1.0));
    BOOST_CHECK(cf1->commit(1.0));
    BOOST_CHECK(cf2->commit(1.0));
    BOOST_CHECK(cf3->commit(1.0));
    BOOST_CHECK(af0->commit(1.0));
}

BOOST_AUTO_TEST_SUITE_END()
