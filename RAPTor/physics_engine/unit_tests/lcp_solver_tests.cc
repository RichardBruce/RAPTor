#ifdef STAND_ALONE
#define BOOST_TEST_MODULE lcp_solver test

/* Common headers */
#include "logging.h"

/* Initialise logger */
const raptor_physics::init_logger init_logger;
#endif /* #ifde STAND_ALONE */


/* Boost headers */
#include "boost/test/unit_test.hpp"

/* Physics headers */
#include "matrix_3d.h"
#include "lcp_solver.h"


using namespace raptor_physics;


BOOST_AUTO_TEST_SUITE( lcp_solver_tests )

const float result_tolerance = 0.0005;

/* Test construct and set up */
BOOST_AUTO_TEST_CASE ( ctor_test )
{
    /* Construct */
    float q[3] = { -2.0, -1.0,  3.0 };
    float m[9] = {  0.0,  0.0, -1.0, 
                    0.0,  0.0, -1.0, 
                    1.0,  1.0,  0.0 };
    lcp_solver uut(3, 4);

    /* Initialise */
    memcpy(uut.initialise_m(), m, 9 * sizeof(float));
    memcpy(uut.initialise_q(), q, 3 * sizeof(float));
    
    /* Check */
    BOOST_CHECK(std::equal(&uut.initialise_m()[0], &uut.initialise_m()[9], m));
    BOOST_CHECK(std::equal(&uut.initialise_q()[0], &uut.initialise_q()[3], q));
}


BOOST_AUTO_TEST_CASE ( copy_ctor_test )
{
    /* Construct, initialise and check */
    float q[3] = { -2.0, -1.0,  3.0 };
    float m[9] = {  0.0,  0.0, -1.0, 
                    0.0,  0.0, -1.0, 
                    1.0,  1.0,  0.0 };
    lcp_solver uut0(3, 4);
    memcpy(uut0.initialise_m(), m, 9 * sizeof(float));
    memcpy(uut0.initialise_q(), q, 3 * sizeof(float));
    
    BOOST_CHECK(std::equal(&uut0.initialise_m()[0], &uut0.initialise_m()[9], m));
    BOOST_CHECK(std::equal(&uut0.initialise_q()[0], &uut0.initialise_q()[3], q));

    /* Copy */
    lcp_solver uut1(uut0);
    
    /* Check */
    BOOST_CHECK(std::equal(&uut1.initialise_m()[0], &uut1.initialise_m()[9], m));
    BOOST_CHECK(std::equal(&uut1.initialise_q()[0], &uut1.initialise_q()[3], q));
}


BOOST_AUTO_TEST_CASE ( move_ctor_test )
{
    /* Construct, initialise and check */
    float q[3] = { -2.0, -1.0,  3.0 };
    float m[9] = {  0.0,  0.0, -1.0, 
                    0.0,  0.0, -1.0, 
                    1.0,  1.0,  0.0 };
    lcp_solver uut0(3, 4);
    memcpy(uut0.initialise_m(), m, 9 * sizeof(float));
    memcpy(uut0.initialise_q(), q, 3 * sizeof(float));
    
    BOOST_CHECK(std::equal(&uut0.initialise_m()[0], &uut0.initialise_m()[9], m));
    BOOST_CHECK(std::equal(&uut0.initialise_q()[0], &uut0.initialise_q()[3], q));

    /* Copy */
    lcp_solver uut1(std::move(uut0));
    
    /* Check */
    BOOST_CHECK(std::equal(&uut1.initialise_m()[0], &uut1.initialise_m()[9], m));
    BOOST_CHECK(std::equal(&uut1.initialise_q()[0], &uut1.initialise_q()[3], q));
}


/* Test solve */
BOOST_AUTO_TEST_CASE( solve_linear_test )
{
    /* Example 14.2 from Eberly */
    float q[3] = { -2.0, -1.0,  3.0 };
    float m[9] = {  0.0,  0.0, -1.0, 
                    0.0,  0.0, -1.0, 
                    1.0,  1.0,  0.0 };
    float z_exp[3] = { 3.0, 0.0, 2.0 };
    lcp_solver uut(3, 4);
    memcpy(uut.initialise_m(), m, 9 * sizeof(float));
    memcpy(uut.initialise_q(), q, 3 * sizeof(float));
    
    float z[3];
    BOOST_CHECK(uut.solve(z));
    BOOST_CHECK(std::equal(&z_exp[0], &z_exp[3], z));
}


BOOST_AUTO_TEST_CASE( solve_zero_coeff0_test )
{
    /* Example 14.3 from Eberly */
    float q[ 5] = { -1.0, -1.0, 2.0, -1.0, 3.0 };
    float m[25] = {  0.0,  0.0,  1.0, -2.0, -3.0, 
                     0.0,  0.0, -1.0,  1.0, -1.0, 
                    -1.0,  1.0,  0.0,  0.0,  0.0, 
                     2.0, -1.0,  0.0,  0.0,  0.0, 
                     3.0,  1.0,  0.0,  0.0,  0.0 };
    float z_exp[5] = { 0.25, 2.25, 0.5, 0.0, 0.5 };
    lcp_solver uut(5, 6);
    memcpy(uut.initialise_m(), m, 25 * sizeof(float));
    memcpy(uut.initialise_q(), q,  5 * sizeof(float));
    
    float z[5];
    BOOST_CHECK(uut.solve(z));
    BOOST_CHECK(std::equal(&z_exp[0], &z_exp[5], z));
}


BOOST_AUTO_TEST_CASE( solve_zero_coeff1_test )
{
    /* Example 14.4 from Eberly */
    float q[3] = { -1.0, -1.0,  2.0 };
    float m[9] = {  0.0,  0.0, -1.0,
                    0.0,  0.0, -1.0,
                    1.0,  1.0,  0.0 };
    float z_exp[3] = { 0.0, 2.0, 1.0 };
    lcp_solver uut(3, 4);
    memcpy(uut.initialise_m(), m, 9 * sizeof(float));
    memcpy(uut.initialise_q(), q, 3 * sizeof(float));
    
    float z[3];
    BOOST_CHECK(uut.solve(z));
    BOOST_CHECK(std::equal(&z_exp[0], &z_exp[3], z));
}


BOOST_AUTO_TEST_CASE( solve_quadratic_test )
{
    /* Example 14.9 from Eberly */
    float q[3] = { -2.0, -4.0,  1.0 };
    float m[9] = {  2.0,  0.0, -1.0,
                    0.0,  2.0, -1.0,
                    1.0,  1.0,  0.0 };
    float z_exp[3] = { 0.0, 1.0, 2.0 };
    lcp_solver uut(3, 4);
    memcpy(uut.initialise_m(), m, 9 * sizeof(float));
    memcpy(uut.initialise_q(), q, 3 * sizeof(float));
    
    float z[3];
    BOOST_CHECK(uut.solve(z));
    BOOST_CHECK(std::equal(&z_exp[0], &z_exp[3], z));
}


BOOST_AUTO_TEST_CASE( tetrahedron_distance_test )
{
    /* Squared distance between two tetrahedron */
    matrix_3d p0(std::vector<point_t>(
        {
            point_t(1.0, 1.0, 1.0), 
            point_t(2.0, 1.0, 1.0),
            point_t(1.0, 2.0, 1.0),
            point_t(1.0, 1.0, 2.0)
        }));

    matrix_3d p1(std::vector<point_t>(
        {
            point_t(3.0, 1.0, 1.0), 
            point_t(4.0, 1.0, 1.0),
            point_t(4.0, 2.0, 1.0),
            point_t(4.0, 1.0, 2.0)
        }));

    std::vector<int> e(
        {
            0, 1, 2, 0, 3, 1, 
            1, 3, 2, 2, 3, 0
        });

    float z_exp[6] = { 2.0, 1.0, 1.0, 3.0, 1.0, 1.0 };

    lcp_solver uut(squared_distance_solver(p0, p1, e, e));
    
    float z[14];
    BOOST_CHECK(uut.solve(z));
    BOOST_CHECK(std::equal(&z_exp[0], &z_exp[6], z, [] (const float exp, const float act) { return fabs(exp - act) < 0.000001; }));
}


BOOST_AUTO_TEST_CASE( cube_distance_test )
{
    /* Squared distance between two cubes */
    matrix_3d p0(std::vector<point_t>(
        {
            point_t(1.0, 1.0, 1.0), 
            point_t(2.0, 1.0, 1.0),
            point_t(2.0, 2.0, 1.0),
            point_t(1.0, 2.0, 1.0),
            point_t(1.0, 1.0, 2.0), 
            point_t(2.0, 1.0, 2.0),
            point_t(2.0, 2.0, 2.0),
            point_t(1.0, 2.0, 2.0)
        }));

    matrix_3d p1(std::vector<point_t>(
        {
            point_t(1.0, 4.0, 1.0), 
            point_t(2.0, 4.0, 1.0),
            point_t(2.0, 5.0, 1.0),
            point_t(1.0, 5.0, 1.0),
            point_t(1.0, 4.0, 2.0), 
            point_t(2.0, 4.0, 2.0),
            point_t(2.0, 5.0, 2.0),
            point_t(1.0, 5.0, 2.0)
        }));


    std::vector<int> e(
        {
            0, 1, 2, 0, 2, 3, /* Front face */
            4, 6, 5, 4, 7, 6, /* Back face */
            4, 0, 7, 7, 0, 3, /* Left face */
            1, 6, 2, 1, 5, 6, /* Right face */
            3, 6, 7, 3, 2, 6, /* Top face */
            0, 4, 1, 1, 4, 5 /* Bottom face */
        });

    float z_exp[6] = { 1.0, 2.0, 1.0, 1.0, 4.0, 1.0 };

    lcp_solver uut(squared_distance_solver(p0, p1, e, e));
    
    float z[30];
    BOOST_CHECK(uut.solve(z));
    BOOST_CHECK(std::equal(&z_exp[0], &z_exp[6], z, [] (const float exp, const float act) { return fabs(exp - act) < 0.000001; }));
}


/* No solution tests */
BOOST_AUTO_TEST_CASE( instant_no_solution_test )
{
    float q[ 5] = {  1.0,  1.0,  2.0,  1.0,  3.0 };
    float m[25] = {  0.0,  0.0,  1.0, -2.0, -3.0, 
                     0.0,  0.0, -1.0,  1.0, -1.0, 
                    -1.0,  1.0,  0.0,  0.0,  0.0, 
                     2.0, -1.0,  0.0,  0.0,  0.0, 
                     3.0,  1.0,  0.0,  0.0,  0.0 };
    lcp_solver uut(5, 6);
    memcpy(uut.initialise_m(), m, 25 * sizeof(float));
    memcpy(uut.initialise_q(), q,  5 * sizeof(float));
    
    float z[5];
    BOOST_CHECK(!uut.solve(z));
}

BOOST_AUTO_TEST_SUITE_END()
