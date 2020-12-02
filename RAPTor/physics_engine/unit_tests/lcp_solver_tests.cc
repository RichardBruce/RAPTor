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


namespace raptor_physics
{
namespace test
{
BOOST_AUTO_TEST_SUITE( lcp_solver_tests )

const float result_tolerance = 0.0005f;

/* Test construct and set up */
BOOST_AUTO_TEST_CASE ( ctor_test )
{
    /* Construct */
    float q[3] = { -2.0f, -1.0f,  3.0f };
    float m[9] = {  0.0f,  0.0f, -1.0f, 
                    0.0f,  0.0f, -1.0f, 
                    1.0f,  1.0f,  0.0f };
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
    float q[3] = { -2.0f, -1.0f,  3.0f };
    float m[9] = {  0.0f,  0.0f, -1.0f, 
                    0.0f,  0.0f, -1.0f, 
                    1.0f,  1.0f,  0.0f };
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
    float q[3] = { -2.0f, -1.0f,  3.0f };
    float m[9] = {  0.0f,  0.0f, -1.0f, 
                    0.0f,  0.0f, -1.0f, 
                    1.0f,  1.0f,  0.0f };
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
    float q[3] = { -2.0f, -1.0f,  3.0f };
    float m[9] = {  0.0f,  0.0f, -1.0f, 
                    0.0f,  0.0f, -1.0f, 
                    1.0f,  1.0f,  0.0f };
    float z_exp[3] = { 3.0f, 0.0f, 2.0f };
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
    float q[ 5] = { -1.0f, -1.0f,  2.0f, -1.0f,  3.0f };
    float m[25] = {  0.0f,  0.0f,  1.0f, -2.0f, -3.0f, 
                     0.0f,  0.0f, -1.0f,  1.0f, -1.0f, 
                    -1.0f,  1.0f,  0.0f,  0.0f,  0.0f, 
                     2.0f, -1.0f,  0.0f,  0.0f,  0.0f, 
                     3.0f,  1.0f,  0.0f,  0.0f,  0.0f };
    float z_exp[5] = { 0.25f, 2.25f, 0.5f, 0.0f, 0.5f };
    lcp_solver uut(5, 6);
    memcpy(uut.initialise_m(), m, 25 * sizeof(float));
    memcpy(uut.initialise_q(), q,  5 * sizeof(float));
    
    float z[5];
    BOOST_CHECK(uut.solve(z));
    BOOST_CHECK_CLOSE(z[0], z_exp[0], result_tolerance);
    BOOST_CHECK_CLOSE(z[1], z_exp[1], result_tolerance);
    BOOST_CHECK_CLOSE(z[2], z_exp[2], result_tolerance);
    BOOST_CHECK_CLOSE(z[3], z_exp[3], result_tolerance);
    BOOST_CHECK_CLOSE(z[4], z_exp[4], result_tolerance);
}

BOOST_AUTO_TEST_CASE( solve_zero_coeff1_test )
{
    /* Example 14.4 from Eberly */
    float q[3] = { -1.0f, -1.0f,  2.0f };
    float m[9] = {  0.0f,  0.0f, -1.0f,
                    0.0f,  0.0f, -1.0f,
                    1.0f,  1.0f,  0.0f };
    float z_exp[3] = { 2.0f, 0.0f, 1.0f };
    lcp_solver uut(3, 4);
    memcpy(uut.initialise_m(), m, 9 * sizeof(float));
    memcpy(uut.initialise_q(), q, 3 * sizeof(float));
    
    float z[3];
    BOOST_CHECK(uut.solve(z));
    BOOST_CHECK_CLOSE(z[0], z_exp[0], result_tolerance);
    BOOST_CHECK_CLOSE(z[1], z_exp[1], result_tolerance);
    BOOST_CHECK_CLOSE(z[2], z_exp[2], result_tolerance);
}

BOOST_AUTO_TEST_CASE( solve_quadratic_test )
{
    /* Example 14.9 from Eberly */
    float q[3] = { -2.0f, -4.0f,  1.0f };
    float m[9] = {  2.0f,  0.0f, -1.0f,
                    0.0f,  2.0f, -1.0f,
                    1.0f,  1.0f,  0.0f };
    float z_exp[3] = { 0.0f, 1.0f, 2.0f };
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
    matrix_3d p0(std::vector<point_t<>>(
        {
            point_t<>(1.0f, 1.0f, 1.0f), 
            point_t<>(2.0f, 1.0f, 1.0f),
            point_t<>(1.0f, 2.0f, 1.0f),
            point_t<>(1.0f, 1.0f, 2.0f)
        }));

    matrix_3d p1(std::vector<point_t<>>(
        {
            point_t<>(3.0f, 1.0f, 1.0f), 
            point_t<>(4.0f, 1.0f, 1.0f),
            point_t<>(4.0f, 2.0f, 1.0f),
            point_t<>(4.0f, 1.0f, 2.0f)
        }));

    std::vector<int> e(
        {
            0, 1, 2, 0, 3, 1, 
            1, 3, 2, 2, 3, 0
        });

    float z_exp[6] = { 2.0f, 1.0f, 1.0f, 3.0f, 1.0f, 1.0f };

    lcp_solver uut(squared_distance_solver(p0, p1, e, e));
    
    float z[14];
    BOOST_CHECK(uut.solve(z));
    BOOST_CHECK(std::equal(&z_exp[0], &z_exp[6], z, [] (const float exp, const float act) { return fabs(exp - act) < 0.000001; }));
}

BOOST_AUTO_TEST_CASE( cube_distance_test )
{
    /* Squared distance between two cubes */
    matrix_3d p0(std::vector<point_t<>>(
        {
            point_t<>(1.0f, 1.0f, 1.0f), 
            point_t<>(2.0f, 1.0f, 1.0f),
            point_t<>(2.0f, 2.0f, 1.0f),
            point_t<>(1.0f, 2.0f, 1.0f),
            point_t<>(1.0f, 1.0f, 2.0f), 
            point_t<>(2.0f, 1.0f, 2.0f),
            point_t<>(2.0f, 2.0f, 2.0f),
            point_t<>(1.0f, 2.0f, 2.0f)
        }));

    matrix_3d p1(std::vector<point_t<>>(
        {
            point_t<>(1.0f, 4.0f, 1.0f), 
            point_t<>(2.0f, 4.0f, 1.0f),
            point_t<>(2.0f, 5.0f, 1.0f),
            point_t<>(1.0f, 5.0f, 1.0f),
            point_t<>(1.0f, 4.0f, 2.0f), 
            point_t<>(2.0f, 4.0f, 2.0f),
            point_t<>(2.0f, 5.0f, 2.0f),
            point_t<>(1.0f, 5.0f, 2.0f)
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

    float z_exp[6] = { 2.0f, 2.0f, 2.0f, 2.0f, 4.0f, 2.0f };
    lcp_solver uut(squared_distance_solver(p0, p1, e, e));

    float z[30];
    BOOST_CHECK(uut.solve(z));
    // BOOST_CHECK_CLOSE(z[0], z_exp[0], result_tolerance);
    // BOOST_CHECK_CLOSE(z[1], z_exp[1], result_tolerance);
    // BOOST_CHECK_CLOSE(z[2], z_exp[2], result_tolerance);
    // BOOST_CHECK_CLOSE(z[3], z_exp[3], result_tolerance);
    // BOOST_CHECK_CLOSE(z[4], z_exp[4], result_tolerance);
    // BOOST_CHECK_CLOSE(z[5], z_exp[5], result_tolerance);
}

/* No solution tests */
BOOST_AUTO_TEST_CASE( instant_no_solution_test )
{
    float q[ 5] = {  1.0f,  1.0f,  2.0f,  1.0f,  3.0f };
    float m[25] = {  0.0f,  0.0f,  1.0f, -2.0f, -3.0f, 
                     0.0f,  0.0f, -1.0f,  1.0f, -1.0f, 
                    -1.0f,  1.0f,  0.0f,  0.0f,  0.0f, 
                     2.0f, -1.0f,  0.0f,  0.0f,  0.0f, 
                     3.0f,  1.0f,  0.0f,  0.0f,  0.0f };
    lcp_solver uut(5, 6);
    memcpy(uut.initialise_m(), m, 25 * sizeof(float));
    memcpy(uut.initialise_q(), q,  5 * sizeof(float));
    
    float z[5];
    BOOST_CHECK(!uut.solve(z));
}

BOOST_AUTO_TEST_CASE( infinite_loop_regression_test )
{
    float q[12] = { -9.8f, -9.8f, -9.8f, -9.8f, 3.22f, 2.82f, 1.97f, 2.51f, 0.0f, 0.0f, 0.0f, 0.0f };
    float m[144] = 
    {
         0.4000034928f,     0.1000010148f, -0.2000000179f,    0.10000249f,   -0.2690843046f,   0.0f,           0.0f,            0.0f,         -0.5f,  0.0f,  0.0f,  0.0f,
         0.1000010297f,     0.3999985158f,  0.099997513f,    -0.1999999583f,  0.0f,           -0.1563823968f,  0.0f,            0.0f,          0.0f, -0.5f,  0.0f,  0.0f,
        -0.2000000179f,     0.099997513f,   0.3999964893f,    0.09999898821f, 0.0f,            0.0f,          -0.2590998411f,   0.0f,          0.0f,  0.0f, -0.5f,  0.0f,
         0.1000024602f,    -0.1999999583f,  0.09999900311f,   0.4000014365f,  0.0f,            0.0f,           0.0f,           -0.1250377744f, 0.0f,  0.0f,  0.0f, -0.5f,
         0.0f,              0.0f,           0.0f,             0.0f,           0.5381686091f,   0.0f,           0.0f,            0.0f,          1.0f,  0.0f,  0.0f,  0.0f,
         0.0f,              0.0f,           0.0f,             0.0f,           0.0f,            0.3127647936f,  0.0f,            0.0f,          0.0f,  1.0f,  0.0f,  0.0f,
         0.0f,              0.0f,           0.0f,             0.0f,           0.0f,            0.0f,           0.5181996822f,   0.0f,          0.0f,  0.0f,  1.0f,  0.0f,
         0.0f,              0.0f,           0.0f,             0.0f,           0.0f,            0.0f,           0.0f,            0.2500755489f, 0.0f,  0.0f,  0.0f,  1.0f,
         0.04227176309f,   -0.2079290152f, -0.04227025062f,   0.2079305351f,  0.0f,           -0.3751211166f,  0.03001190722f, -0.2456464618f, 1.0f,  0.0f,  0.0f,  0.0f,
         0.09699514508f,   -0.1887187362f, -0.09699308872f,   0.1887207627f, -0.3751211166f,   0.0f, -         0.1178897247f,  -0.2224733233f, 0.0f,  1.0f,  0.0f,  0.0f,
         0.06916732341f,   -0.2005974352f, -0.06916551292f,   0.2005992085f,  0.03001190722f, -0.1178897172f,  0.0f,           -0.235539943f,  0.0f,  0.0f,  1.0f,  0.0f,
        -0.0005080550909f, -0.2121830881f,  0.0005090907216f, 0.2121841311f, -0.2456464618f,  -0.2224733233f, -0.235539943f,    0.0f,          0.0f,  0.0f,  0.0f,  1.0f
     };
    float z_exp[12] = { 44.536705f, 7.09078026f, 46.3725128f, 0.0f, 16.0530186f, 3.55215216f, 19.0246677f, 0.0f, 6.21534109f, 0.0f, 4.16158676f, 0.0f };
    lcp_solver uut(12, 13);
    memcpy(uut.initialise_m(), m, 144 * sizeof(float));
    memcpy(uut.initialise_q(), q,  12 * sizeof(float));

    float z[12];
    BOOST_CHECK(uut.solve(z));
    BOOST_CHECK_CLOSE(z[ 0], z_exp[ 0], result_tolerance);
    BOOST_CHECK_CLOSE(z[ 1], z_exp[ 1], result_tolerance);
    BOOST_CHECK_CLOSE(z[ 2], z_exp[ 2], result_tolerance);
    BOOST_CHECK_CLOSE(z[ 3], z_exp[ 3], result_tolerance);
    BOOST_CHECK_CLOSE(z[ 4], z_exp[ 4], result_tolerance);
    BOOST_CHECK_CLOSE(z[ 5], z_exp[ 5], result_tolerance);
    BOOST_CHECK_CLOSE(z[ 6], z_exp[ 6], result_tolerance);
    BOOST_CHECK_CLOSE(z[ 7], z_exp[ 7], result_tolerance);
    BOOST_CHECK_CLOSE(z[ 8], z_exp[ 8], result_tolerance);
    BOOST_CHECK_CLOSE(z[ 9], z_exp[ 9], result_tolerance);
    BOOST_CHECK_CLOSE(z[10], z_exp[10], result_tolerance);
    BOOST_CHECK_CLOSE(z[11], z_exp[11], result_tolerance);
}

BOOST_AUTO_TEST_SUITE_END()
}; /* namespace test */
}; /* namespace raptor_physics */
