#ifdef STAND_ALONE
#define BOOST_TEST_MODULE exact_collision_time test

/* Common headers */
#include "logging.h"

/* Initialise logger */
const raptor_physics::init_logger init_logger;
#endif /* #ifdef STAND_ALONE */


/* Boost headers */
#include "boost/test/unit_test.hpp"

/* Physics headers */
#include "point_t.h"
#include "physics_common.h"


using namespace raptor_physics;


BOOST_AUTO_TEST_SUITE( exact_collision_time_tests )

const float result_tolerance = 0.0005;


/* Point intersecting plane tests */
/* Test with no motion */
BOOST_AUTO_TEST_CASE( static_point_away_from_plane_test )
{
    const point_t pa(0.0, 1.0, 0.0);
    const point_t pb(0.0, 0.0, 1.0);
    const point_t nb(0.0, 0.0, 1.0);
    const point_t x0(0.0, 0.0, 0.0);
    const point_t x1(0.0, 0.0, 0.0);
    const point_t q0(0.0, 0.0, 0.0);
    const point_t q1(0.0, 0.0, 0.0);
    const fp_t r0 = 1.0;
    const fp_t r1 = 1.0;

    const fp_t s0 = find_exact_collision_time(pa, pb, nb, x0, x1, q0, q1, r0, r1);
    BOOST_CHECK(s0 == numeric_limits<fp_t>::infinity());

    const fp_t s1 = find_exact_none_translating_collision_time(pa, pb, nb, x0, q0, q1, r0, r1);
    BOOST_CHECK(s1 == numeric_limits<fp_t>::infinity());
}


BOOST_AUTO_TEST_CASE( static_point_in_plane_test )
{
    const point_t pa(1.0, 0.0, 1.0);
    const point_t pb(0.0, 0.0, 1.0);
    const point_t nb(0.0, 0.0, 1.0);
    const point_t x0(0.0, 0.0, 0.0);
    const point_t x1(0.0, 0.0, 0.0);
    const point_t q0(0.0, 0.0, 0.0);
    const point_t q1(0.0, 0.0, 0.0);
    const fp_t r0 = 1.0;
    const fp_t r1 = 1.0;
    
    const fp_t s0 = find_exact_collision_time(pa, pb, nb, x0, x1, q0, q1, r0, r1);
    BOOST_CHECK(fabs(s0) < raptor_physics::EPSILON);

    const fp_t s1 = find_exact_none_translating_collision_time(pa, pb, nb, x0, q0, q1, r0, r1);
    BOOST_CHECK(fabs(s1) < raptor_physics::EPSILON);
}


/* Test with translation only */
BOOST_AUTO_TEST_CASE( point_translating_straight_through_plane_test )
{
    const point_t pa(0.0, 0.0, 0.0);
    const point_t pb(0.0, 0.0, 1.0);
    const point_t nb(0.0, 0.0, 1.0);
    const point_t x0(0.0, 0.0, 0.0);
    const point_t x1(0.0, 0.0, 2.0);
    const point_t q0(0.0, 0.0, 0.0);
    const point_t q1(0.0, 0.0, 0.0);
    const fp_t r0 = 1.0;
    const fp_t r1 = 1.0;
    
    const fp_t s0 = find_exact_collision_time(pa, pb, nb, x0, x1, q0, q1, r0, r1);
    BOOST_CHECK(s0 == 0.5);
}


BOOST_AUTO_TEST_CASE( point_translating_diagonal_through_plane_test )
{
    const point_t pa(0.0, 0.0, 0.0);
    const point_t pb(0.0, 0.0, 1.0);
    const point_t nb(0.0, 0.0, 1.0);
    const point_t x0(0.0, 0.0, 0.0);
    const point_t x1(0.0, 2.0, 2.0);
    const point_t q0(0.0, 0.0, 0.0);
    const point_t q1(0.0, 0.0, 0.0);
    const fp_t r0 = 1.0;
    const fp_t r1 = 1.0;
    
    const fp_t s0 = find_exact_collision_time(pa, pb, nb, x0, x1, q0, q1, r0, r1);
    BOOST_CHECK(s0 == 0.5);
}


BOOST_AUTO_TEST_CASE( point_translating_not_through_plane_test )
{
    const point_t pa(0.0, 0.0,  0.0);
    const point_t pb(0.0, 0.0,  1.0);
    const point_t nb(0.0, 0.0,  1.0);
    const point_t x0(0.0, 0.0,  2.0);
    const point_t x1(0.0, 0.0, 10.0);
    const point_t q0(0.0, 0.0,  0.0);
    const point_t q1(0.0, 0.0,  0.0);
    const fp_t r0 = 1.0;
    const fp_t r1 = 1.0;

    const fp_t s0 = find_exact_collision_time(pa, pb, nb, x0, x1, q0, q1, r0, r1);
    BOOST_CHECK(s0 == numeric_limits<fp_t>::infinity());
}


/* Test with rotation only */
BOOST_AUTO_TEST_CASE( point_rotating_through_plane_x_test )
{
    const point_t pa(0.0, 0.0, 2.0);
    const point_t pb(0.0, 0.0, 1.0);
    const point_t nb(0.0, 0.0, 1.0);
    const point_t x0(0.0, 0.0, 0.0);
    const point_t x1(0.0, 0.0, 0.0);
    const point_t q0(0.0, 0.0, 0.0);
    const point_t q1(1.0, 0.0, 0.0);
    const fp_t r0 =  1.0;
    const fp_t r1 =  0.0;
    
    const fp_t s0 = find_exact_collision_time(pa, pb, nb, x0, x1, q0, q1, r0, r1);
    BOOST_CHECK(s0 == 0.5);

    const fp_t s1 = find_exact_none_translating_collision_time(pa, pb, nb, x0, q0, q1, r0, r1);
    BOOST_CHECK(s1 == 0.5);
}


BOOST_AUTO_TEST_CASE( point_rotating_through_plane_y_test )
{
    const point_t pa(0.0, 0.0, 2.0);
    const point_t pb(0.0, 0.0, 1.0);
    const point_t nb(0.0, 0.0, 1.0);
    const point_t x0(0.0, 0.0, 0.0);
    const point_t x1(0.0, 0.0, 0.0);
    const point_t q0(0.0, 0.0, 0.0);
    const point_t q1(0.0, 1.0, 0.0);
    const fp_t r0 =  1.0;
    const fp_t r1 =  0.0;

    const fp_t s0 = find_exact_collision_time(pa, pb, nb, x0, x1, q0, q1, r0, r1);
    BOOST_CHECK(s0 == 0.5);

    const fp_t s1 = find_exact_none_translating_collision_time(pa, pb, nb, x0, q0, q1, r0, r1);
    BOOST_CHECK(s1 == 0.5);
}


BOOST_AUTO_TEST_CASE( point_rotating_in_plane_test )
{
    const point_t pa(0.0, 0.0, 2.0);
    const point_t pb(0.0, 0.0, 1.0);
    const point_t nb(0.0, 0.0, 1.0);
    const point_t x0(0.0, 0.0, 0.0);
    const point_t x1(0.0, 0.0, 0.0);
    const point_t q0(0.0, 0.0, 0.0);
    const point_t q1(0.0, 0.0, 1.0);
    const fp_t r0 =  1.0;
    const fp_t r1 =  0.0;

    const fp_t s0 = find_exact_collision_time(pa, pb, nb, x0, x1, q0, q1, r0, r1);
    BOOST_CHECK(s0 == numeric_limits<fp_t>::infinity());

    const fp_t s1 = find_exact_none_translating_collision_time(pa, pb, nb, x0, q0, q1, r0, r1);
    BOOST_CHECK(s1 == numeric_limits<fp_t>::infinity());
}


BOOST_AUTO_TEST_CASE( point_rotating_through_plane_xy_test )
{
    const fp_t sin_45_d_root_2 = 0.3826834324 * 0.7071067812;
    const fp_t sin_135_d_root_2 = 0.9238795325 * 0.7071067812;
    const point_t pa(0.0, 0.0, 2.0);
    const point_t pb(0.0, 0.0, 1.0);
    const point_t nb(0.0, 0.0, 1.0);
    const point_t x0(0.0, 0.0, 0.0);
    const point_t x1(0.0, 0.0, 0.0);
    const point_t q0(sin_45_d_root_2, sin_45_d_root_2, 0.0);
    const point_t q1(sin_135_d_root_2, sin_135_d_root_2, 0.0);
    const fp_t r0 = 0.9238795325;
    const fp_t r1 = 0.3826834324;

    const fp_t s0 = find_exact_collision_time(pa, pb, nb, x0, x1, q0, q1, r0, r1);
    BOOST_CHECK(fabs(s0 - 0.216773) < result_tolerance);

    const fp_t s1 = find_exact_none_translating_collision_time(pa, pb, nb, x0, q0, q1, r0, r1);
    BOOST_CHECK(fabs(s1 - 0.216773) < result_tolerance);
}


/* Test with translation and rotation */
BOOST_AUTO_TEST_CASE( point_rotating_through_plane_x_with_strong_translation_up_test )
{
    const point_t pa(0.0, 0.0, 2.0);
    const point_t pb(0.0, 0.0, 1.0);
    const point_t nb(0.0, 0.0, 1.0);
    const point_t x0(0.0, 0.0, 0.0);
    const point_t x1(0.0, 0.0, 2.0);
    const point_t q0(0.0, 0.0, 0.0);
    const point_t q1(1.0, 0.0, 0.0);
    const fp_t r0 =  1.0;
    const fp_t r1 =  0.0;

    const fp_t s0 = find_exact_collision_time(pa, pb, nb, x0, x1, q0, q1, r0, r1);
    BOOST_CHECK(fabs(s0 - 0.809017) < result_tolerance);
}


BOOST_AUTO_TEST_CASE( point_rotating_through_plane_x_with_strong_translation_down_test )
{
    const point_t pa(0.0, 0.0,  2.0);
    const point_t pb(0.0, 0.0,  1.0);
    const point_t nb(0.0, 0.0,  1.0);
    const point_t x0(0.0, 0.0,  0.0);
    const point_t x1(0.0, 0.0, -2.0);
    const point_t q0(0.0, 0.0,  0.0);
    const point_t q1(1.0, 0.0,  0.0);
    const fp_t r0 =  1.0;
    const fp_t r1 =  0.0;

    const fp_t s0 = find_exact_collision_time(pa, pb, nb, x0, x1, q0, q1, r0, r1);
    BOOST_CHECK(fabs(s0 - 0.309017) < result_tolerance);
}


BOOST_AUTO_TEST_CASE( point_rotating_through_plane_xy_with_translation_up_test )
{
    const fp_t sin_45_d_root_2 = 0.3826834324 * 0.7071067812;
    const fp_t sin_135_d_root_2 = 0.9238795325 * 0.7071067812;
    const point_t pa(0.0, 0.0, 2.0);
    const point_t pb(0.0, 0.0, 1.0);
    const point_t nb(0.0, 0.0, 1.0);
    const point_t x0(0.0, 0.0, 0.0);
    const point_t x1(0.0, 0.0, 0.5);
    const point_t q0(sin_45_d_root_2, sin_45_d_root_2, 0.0);
    const point_t q1(sin_135_d_root_2, sin_135_d_root_2, 0.0);
    const fp_t r0 = 0.9238795325;
    const fp_t r1 = 0.3826834324;

    const fp_t s0 = find_exact_collision_time(pa, pb, nb, x0, x1, q0, q1, r0, r1);
    BOOST_CHECK(fabs(s0 - 0.279141) < result_tolerance);
}


BOOST_AUTO_TEST_CASE( point_rotating_through_plane_xy_with_translation_down_test )
{
    const fp_t sin_45_d_root_2 = 0.3826834324 * 0.7071067812;
    const fp_t sin_135_d_root_2 = 0.9238795325 * 0.7071067812;
    const point_t pa(0.0, 0.0,  2.0);
    const point_t pb(0.0, 0.0,  1.0);
    const point_t nb(0.0, 0.0,  1.0);
    const point_t x0(0.0, 0.0,  0.0);
    const point_t x1(0.0, 0.0, -0.5);
    const point_t q0(sin_45_d_root_2, sin_45_d_root_2, 0.0);
    const point_t q1(sin_135_d_root_2, sin_135_d_root_2, 0.0);
    const fp_t r0 = 0.9238795325;
    const fp_t r1 = 0.3826834324;

    const fp_t s0 = find_exact_collision_time(pa, pb, nb, x0, x1, q0, q1, r0, r1);
    BOOST_CHECK(fabs(s0 - 0.175345) < result_tolerance);
}


BOOST_AUTO_TEST_CASE( point_rotating_through_plane_x_with_translation_to_miss_test )
{
    const point_t pa(0.0, 0.0,  2.0);
    const point_t pb(0.0, 0.0,  1.0);
    const point_t nb(0.0, 0.0,  1.0);
    const point_t x0(0.0, 0.0,  0.0);
    const point_t x1(0.0, 0.0, 10.0);
    const point_t q0(0.0, 0.0,  0.0);
    const point_t q1(1.0, 0.0,  0.0);
    const fp_t r0 =  1.0;
    const fp_t r1 =  0.0;

    const fp_t s0 = find_exact_collision_time(pa, pb, nb, x0, x1, q0, q1, r0, r1);
    BOOST_CHECK(fabs(s0 - 2.59629) < result_tolerance);
}


/* Special tests */
/* This processes a quadratic with no roots */
BOOST_AUTO_TEST_CASE( point_plane_quadratic_no_root_found_test )
{
    const point_t pa( -0.659,   -0.255,  0.5);
    const point_t pb(-10.0,      0.0,  -10.0);
    const point_t nb(  0.0,   -400.0,    0.0);
    const point_t x0(  0.0,      0.659,  0.0);
    const point_t x1(  0.0,      0.921,  0.0);
    const point_t q0(  0.0,      0.0,    0.0);
    const point_t q1(  0.724,    0.0,    0.0);
    const fp_t r0 =  1.0;
    const fp_t r1 =  0.688;

    const fp_t s0 = find_exact_collision_time(pa, pb, nb, x0, x1, q0, q1, r0, r1);
    BOOST_CHECK(s0 == numeric_limits<fp_t>::infinity());
}


/* Edge intersecting edge tests */
/* Test with no motion */
BOOST_AUTO_TEST_CASE( static_edge_away_from_edge_test )
{
    const point_t pa(0.0, 1.0, 0.0);
    const point_t pb(1.0, 0.0, 0.0);
    const point_t ea(0.0, 0.0, 1.0);
    const point_t eb(0.0, 1.0, 0.0);
    const point_t x0(0.0, 0.0, 0.0);
    const point_t x1(0.0, 0.0, 0.0);
    const point_t q0(0.0, 0.0, 0.0);
    const point_t q1(0.0, 0.0, 0.0);
    const fp_t r0 = 1.0;
    const fp_t r1 = 1.0;

    const fp_t s0 = find_exact_collision_time(pa, pb, ea, eb, x0, x1, q0, q1, r0, r1);
    BOOST_CHECK(s0 == numeric_limits<fp_t>::infinity());

    const fp_t s1 = find_exact_none_translating_collision_time(pa, pb, ea, eb, x0, q0, q1, r0, r1);
    BOOST_CHECK(s1 == numeric_limits<fp_t>::infinity());
}


BOOST_AUTO_TEST_CASE( static_edge_touching_edge_test )
{
    const point_t pa(0.0, 1.0, 0.0);
    const point_t pb(1.0, 0.0, 0.0);
    const point_t ea(1.0, 0.0, 0.0);
    const point_t eb(0.0, 1.0, 0.0);
    const point_t x0(0.0, 0.0, 0.0);
    const point_t x1(0.0, 0.0, 0.0);
    const point_t q0(0.0, 0.0, 0.0);
    const point_t q1(0.0, 0.0, 0.0);
    const fp_t r0 = 1.0;
    const fp_t r1 = 1.0;

    const fp_t s0 = find_exact_collision_time(pa, pb, ea, eb, x0, x1, q0, q1, r0, r1);
    BOOST_CHECK(fabs(s0) < raptor_physics::EPSILON);

    const fp_t s1 = find_exact_none_translating_collision_time(pa, pb, ea, eb, x0, q0, q1, r0, r1);
    BOOST_CHECK(fabs(s1) < raptor_physics::EPSILON);
}


BOOST_AUTO_TEST_CASE( static_parallel_edge_touching_edge_test )
{
    const point_t pa(0.0, 1.0, 0.0);
    const point_t pb(0.0, 2.0, 0.0);
    const point_t ea(0.0, 1.0, 0.0);
    const point_t eb(0.0, 1.0, 0.0);
    const point_t x0(0.0, 0.0, 0.0);
    const point_t x1(0.0, 0.0, 0.0);
    const point_t q0(0.0, 0.0, 0.0);
    const point_t q1(0.0, 0.0, 0.0);
    const fp_t r0 = 1.0;
    const fp_t r1 = 1.0;

    const fp_t s0 = find_exact_collision_time(pa, pb, ea, eb, x0, x1, q0, q1, r0, r1);
    BOOST_CHECK(s0 == numeric_limits<fp_t>::infinity());

    const fp_t s1 = find_exact_none_translating_collision_time(pa, pb, ea, eb, x0, q0, q1, r0, r1);
    BOOST_CHECK(s1 == numeric_limits<fp_t>::infinity());
}


BOOST_AUTO_TEST_CASE( static_parallel_edge_away_from_edge_test )
{
    const point_t pa(0.0, 0.0, 1.0);
    const point_t pb(1.0, 0.0, 0.0);
    const point_t ea(0.0, 1.0, 0.0);
    const point_t eb(0.0, 1.0, 0.0);
    const point_t x0(0.0, 0.0, 0.0);
    const point_t x1(0.0, 0.0, 0.0);
    const point_t q0(0.0, 0.0, 0.0);
    const point_t q1(0.0, 0.0, 0.0);
    const fp_t r0 = 1.0;
    const fp_t r1 = 1.0;

    const fp_t s0 = find_exact_collision_time(pa, pb, ea, eb, x0, x1, q0, q1, r0, r1);
    BOOST_CHECK(s0 == numeric_limits<fp_t>::infinity());

    const fp_t s1 = find_exact_none_translating_collision_time(pa, pb, ea, eb, x0, q0, q1, r0, r1);
    BOOST_CHECK(s1 == numeric_limits<fp_t>::infinity());
}


/* Test with translation only */
BOOST_AUTO_TEST_CASE( edge_translating_straight_through_edge_test )
{
    const point_t pa(0.0, 1.0, 0.0);
    const point_t pb(1.0, 0.0, 0.0);
    const point_t ea(0.0, 0.0, 1.0);
    const point_t eb(0.0, 1.0, 0.0);
    const point_t x0(0.0, 0.0, 0.0);
    const point_t x1(2.0, 0.0, 0.0);
    const point_t q0(0.0, 0.0, 0.0);
    const point_t q1(0.0, 0.0, 0.0);
    const fp_t r0 = 1.0;
    const fp_t r1 = 1.0;

    const fp_t s0 = find_exact_collision_time(pa, pb, ea, eb, x0, x1, q0, q1, r0, r1);
    BOOST_CHECK(s0 == 0.5);
}


BOOST_AUTO_TEST_CASE( edge_translating_diagonal_through_edge_test )
{
    const point_t pa(0.0,  1.0, 0.0);
    const point_t pb(1.0,  0.0, 0.0);
    const point_t ea(0.0,  0.0, 1.0);
    const point_t eb(0.0,  1.0, 0.0);
    const point_t x0(0.0,  0.0, 0.0);
    const point_t x1(2.0, -2.0, 0.0);
    const point_t q0(0.0,  0.0, 0.0);
    const point_t q1(0.0,  0.0, 0.0);
    const fp_t r0 = 1.0;
    const fp_t r1 = 1.0;

    const fp_t s0 = find_exact_collision_time(pa, pb, ea, eb, x0, x1, q0, q1, r0, r1);
    BOOST_CHECK(s0 == 0.5);
}


BOOST_AUTO_TEST_CASE( edge_translating_away_from_edge_test )
{
    const point_t pa( 0.0, 0.1, 0.0);
    const point_t pb( 1.0, 0.0, 0.0);
    const point_t ea( 0.0, 0.0, 1.0);
    const point_t eb( 0.0, 1.0, 0.0);
    const point_t x0( 0.0, 0.0, 0.0);
    const point_t x1(-2.0, 2.0, 0.0);
    const point_t q0( 0.0, 0.0, 0.0);
    const point_t q1( 0.0, 0.0, 0.0);
    const fp_t r0 = 1.0;
    const fp_t r1 = 1.0;

    const fp_t s0 = find_exact_collision_time(pa, pb, ea, eb, x0, x1, q0, q1, r0, r1);
    BOOST_CHECK(s0 == numeric_limits<fp_t>::infinity());
}


BOOST_AUTO_TEST_CASE( parallel_edge_translating_to_edge_0_test )
{
    const point_t pa(1.0, 0.0,  0.0);
    const point_t pb(1.0, 0.0,  0.0);
    const point_t ea(0.0, 1.0,  0.0);
    const point_t eb(0.0, 1.0,  0.0);
    const point_t x0(0.0, 0.0,  0.0);
    const point_t x1(2.0, 0.0, -2.0);
    const point_t q0(0.0, 0.0,  0.0);
    const point_t q1(0.0, 0.0,  0.0);
    const fp_t r0 = 1.0;
    const fp_t r1 = 1.0;

    const fp_t s0 = find_exact_collision_time(pa, pb, ea, eb, x0, x1, q0, q1, r0, r1);
    BOOST_CHECK(s0 == numeric_limits<fp_t>::infinity());
}


BOOST_AUTO_TEST_CASE( parallel_edge_translating_to_edge_0_5_test )
{
    const point_t pa(0.0, -0.1,  1.0);
    const point_t pb(1.0,  0.0,  0.0);
    const point_t ea(0.0,  1.0,  0.0);
    const point_t eb(0.0,  1.0,  0.0);
    const point_t x0(0.0,  0.0,  0.0);
    const point_t x1(2.0,  0.0, -2.0);
    const point_t q0(0.0,  0.0,  0.0);
    const point_t q1(0.0,  0.0,  0.0);
    const fp_t r0 = 1.0;
    const fp_t r1 = 1.0;

    const fp_t s0 = find_exact_collision_time(pa, pb, ea, eb, x0, x1, q0, q1, r0, r1);
    BOOST_CHECK(s0 == numeric_limits<fp_t>::infinity());
}


/* Test with rotation only */
BOOST_AUTO_TEST_CASE( edge_rotating_through_edge_z_test )
{
    const point_t pa(0.0, 1.0, 0.0);
    const point_t pb(1.0, 0.0, 0.0);
    const point_t ea(0.0, 0.0, 1.0);
    const point_t eb(0.0, 1.0, 0.0);
    const point_t x0(0.0, 0.0, 0.0);
    const point_t x1(0.0, 0.0, 0.0);
    const point_t q0(0.0, 0.0, 0.0);
    const point_t q1(0.0, 0.0, 1.0);
    const fp_t r0 =  1.0;
    const fp_t r1 =  0.0;

    const fp_t s0 = find_exact_collision_time(pa, pb, ea, eb, x0, x1, q0, q1, r0, r1);
    BOOST_CHECK(fabs(s0 - 1.36603) < result_tolerance);

    const fp_t s1 = find_exact_none_translating_collision_time(pa, pb, ea, eb, x0, q0, q1, r0, r1);
    BOOST_CHECK(fabs(s1 - 1.36603) < result_tolerance);
}


BOOST_AUTO_TEST_CASE( edge_rotating_through_edge_y_test )
{
    const point_t pa(0.0, 1.0, 0.0);
    const point_t pb(1.0, 0.0, 0.0);
    const point_t ea(0.0, 0.0, 1.0);
    const point_t eb(0.0, 1.0, 0.0);
    const point_t x0(0.0, 0.0, 0.0);
    const point_t x1(0.0, 0.0, 0.0);
    const point_t q0(0.0, 0.0, 0.0);
    const point_t q1(0.0, 1.0, 0.0);
    const fp_t r0 =  1.0;
    const fp_t r1 =  0.0;

    const fp_t s0 = find_exact_collision_time(pa, pb, ea, eb, x0, x1, q0, q1, r0, r1);
    BOOST_CHECK(fabs(s0 - 0.707107) < result_tolerance);

    const fp_t s1 = find_exact_none_translating_collision_time(pa, pb, ea, eb, x0, q0, q1, r0, r1);
    BOOST_CHECK(fabs(s1 - 0.707107) < result_tolerance);
}


BOOST_AUTO_TEST_CASE( edge_rotating_in_plane_test )
{
    const point_t pa(0.0, 1.0, 0.0);
    const point_t pb(1.0, 0.0, 0.0);
    const point_t ea(0.0, 0.0, 1.0);
    const point_t eb(0.0, 1.0, 0.0);
    const point_t x0(0.0, 0.0, 0.0);
    const point_t x1(0.0, 0.0, 0.0);
    const point_t q0(0.0, 0.0, 0.0);
    const point_t q1(1.0, 0.0, 0.0);
    const fp_t r0 =  1.0;
    const fp_t r1 =  0.0;

    const fp_t s0 = find_exact_collision_time(pa, pb, ea, eb, x0, x1, q0, q1, r0, r1);
    BOOST_CHECK(s0 == numeric_limits<fp_t>::infinity());

    const fp_t s1 = find_exact_none_translating_collision_time(pa, pb, ea, eb, x0, q0, q1, r0, r1);
    BOOST_CHECK(s1 == numeric_limits<fp_t>::infinity());
}


BOOST_AUTO_TEST_CASE( parallel_edge_rotating_in_plane_test )
{
    const point_t pa(0.0,  1.0, 0.0);
    const point_t pb(1.0,  0.0, 0.0);
    const point_t ea(0.0, -1.0, 0.0);
    const point_t eb(0.0,  1.0, 0.0);
    const point_t x0(0.0,  0.0, 0.0);
    const point_t x1(0.0,  0.0, 0.0);
    const point_t q0(0.0,  0.0, 0.0);
    const point_t q1(1.0,  0.0, 0.0);
    const fp_t r0 =  1.0;
    const fp_t r1 =  0.0;

    const fp_t s0 = find_exact_collision_time(pa, pb, ea, eb, x0, x1, q0, q1, r0, r1);
    BOOST_CHECK(s0 == numeric_limits<fp_t>::infinity());

    const fp_t s1 = find_exact_none_translating_collision_time(pa, pb, ea, eb, x0, q0, q1, r0, r1);
    BOOST_CHECK(s1 == numeric_limits<fp_t>::infinity());
}


BOOST_AUTO_TEST_CASE( edge_rotating_through_edge_xy_test )
{
    const fp_t q0_ele = 0.08715574275 * 0.7071067812;
    const fp_t q1_ele = -0.7660444431 * 0.7071067812;
    const point_t pa(0.0, 1.0,  0.0);
    const point_t pb(0.0, 1.1, -2.0);
    const point_t ea(0.0, 0.0,  1.0);
    const point_t eb(1.0, 0.0,  0.0);
    const point_t x0(0.0, 0.0,  0.0);
    const point_t x1(0.0, 0.0,  0.0);
    const point_t q0(q0_ele, q0_ele, 0.0);
    const point_t q1(q1_ele, q1_ele, 0.0);
    const fp_t r0 = 0.9961946981;
    const fp_t r1 = 0.6427876097;

    const fp_t s0 = find_exact_collision_time(pa, pb, ea, eb, x0, x1, q0, q1, r0, r1);
    BOOST_CHECK(fabs(s0 - 0.060296) < result_tolerance);

    const fp_t s1 = find_exact_none_translating_collision_time(pa, pb, ea, eb, x0, q0, q1, r0, r1);
    BOOST_CHECK(fabs(s1 - 0.060296) < result_tolerance);
}


BOOST_AUTO_TEST_CASE( edge_rotating_not_through_edge_xy_test )
{
    const fp_t q0_ele = 0.08715574275 * 0.7071067812;
    const fp_t q1_ele = -0.7660444431 * 0.7071067812;
    const point_t pa(0.0, 1.0,  0.0);
    const point_t pb(0.0, 2.0, -2.0);
    const point_t ea(1.0, 0.0,  0.0);
    const point_t eb(0.0, 0.0,  1.0);
    const point_t x0(0.0, 0.0,  0.0);
    const point_t x1(0.0, 0.0,  0.0);
    const point_t q0(q0_ele, q0_ele, 0.0);
    const point_t q1(q1_ele, q1_ele, 0.0);
    const fp_t r0 = 0.9961946981;
    const fp_t r1 = 0.6427876097;

    const fp_t s0 = find_exact_collision_time(pa, pb, ea, eb, x0, x1, q0, q1, r0, r1);
    BOOST_CHECK(s0 == numeric_limits<fp_t>::infinity());

    const fp_t s1 = find_exact_none_translating_collision_time(pa, pb, ea, eb, x0, q0, q1, r0, r1);
    BOOST_CHECK(s1 == numeric_limits<fp_t>::infinity());
}


/* Test with translation and rotation */
BOOST_AUTO_TEST_CASE( edge_rotating_through_edge_xy_with_translation_down_test )
{
    const fp_t q0_ele = 0.08715574275 * 0.7071067812;
    const fp_t q1_ele = -0.7660444431 * 0.7071067812;
    const point_t pa(0.0, 1.0,  0.0);
    const point_t pb(0.0, 1.1, -2.0);
    const point_t ea(0.0, 0.0,  1.0);
    const point_t eb(1.0, 0.0,  0.0);
    const point_t x0(0.0, 0.0,  0.0);
    const point_t x1(0.0, 0.0, -1.0);
    const point_t q0(q0_ele, q0_ele, 0.0);
    const point_t q1(q1_ele, q1_ele, 0.0);
    const fp_t r0 = 0.9961946981;
    const fp_t r1 = 0.6427876097;

    const fp_t s0 = find_exact_collision_time(pa, pb, ea, eb, x0, x1, q0, q1, r0, r1);
    BOOST_CHECK(fabs(s0 - 0.0590805) < result_tolerance);
}


BOOST_AUTO_TEST_CASE( edge_rotating_through_edge_xy_with_strong_translation_down_test )
{
    const fp_t q0_ele = 0.08715574275 * 0.7071067812;
    const fp_t q1_ele = -0.7660444431 * 0.7071067812;
    const point_t pa(0.0, 1.0,   0.0);
    const point_t pb(0.0, 1.1,  -2.0);
    const point_t ea(0.0, 0.0,   1.0);
    const point_t eb(1.0, 0.0,   0.0);
    const point_t x0(0.0, 0.0,   1.0);
    const point_t x1(0.0, 0.0, -10.0);
    const point_t q0(q0_ele, q0_ele, 0.0);
    const point_t q1(q1_ele, q1_ele, 0.0);
    const fp_t r0 = 0.9961946981;
    const fp_t r1 = 0.6427876097;

    const fp_t s0 = find_exact_collision_time(pa, pb, ea, eb, x0, x1, q0, q1, r0, r1);
    BOOST_CHECK(fabs(s0 - 0.0652741) < result_tolerance);
}


BOOST_AUTO_TEST_CASE( edge_rotating_through_edge_xy_with_strong_translation_up_test )
{
    const fp_t q0_ele = 0.08715574275 * 0.7071067812;
    const fp_t q1_ele = -0.7660444431 * 0.7071067812;
    const point_t pa(0.0, 1.0,   0.0);
    const point_t pb(0.0, 1.1,  -2.0);
    const point_t ea(0.0, 0.0,   1.0);
    const point_t eb(1.0, 0.0,   0.0);
    const point_t x0(0.0, 0.0,   1.0);
    const point_t x1(0.0, 0.0,  25.0);
    const point_t q0(q0_ele, q0_ele, 0.0);
    const point_t q1(q1_ele, q1_ele, 0.0);
    const fp_t r0 = 0.9961946981;
    const fp_t r1 = 0.6427876097;

    const fp_t s0 = find_exact_collision_time(pa, pb, ea, eb, x0, x1, q0, q1, r0, r1);
    BOOST_CHECK(fabs(s0 - 0.0851885) < result_tolerance);
}


BOOST_AUTO_TEST_CASE( edge_rotating_in_plane_with_translation_test )
{
    const point_t pa( 0.0,  1.0, 0.0);
    const point_t pb( 1.0,  0.0, 0.0);
    const point_t ea( 0.0,  0.0, 1.0);
    const point_t eb( 0.0,  1.0, 0.0);
    const point_t x0(-1.0, -2.0, 0.0);
    const point_t x1( 2.0,  0.0, 0.0);
    const point_t q0(0.6427876097, 0.0, 0.0);
    const point_t q1(0.7660444431, 0.0, 0.0);
    const fp_t r0 =  0.7660444431;
    const fp_t r1 =  0.6427876097;

    const fp_t s0 = find_exact_collision_time(pa, pb, ea, eb, x0, x1, q0, q1, r0, r1);
    BOOST_CHECK(fabs(s0 - 0.666667) < result_tolerance);
}

BOOST_AUTO_TEST_SUITE_END()
