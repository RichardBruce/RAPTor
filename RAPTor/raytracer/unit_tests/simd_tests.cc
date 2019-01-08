#ifdef STAND_ALONE
#define BOOST_TEST_MODULE simd test

/* Common headers */
#include "logging.h"

/* Initialise logger */
const raptor_physics::init_logger init_logger;
#endif /* #ifdef STAND_ALONE */

/* Standard headers */
#include <chrono>
#include <random>

/* Common headers */
#include "logging.h"

/* Boost headers */
#include "boost/test/unit_test.hpp"

/* Raytracer headers */
#include "simd.h"


namespace raptor_raytracer
{
namespace test
{
const float result_tolerance = 0.00001;

BOOST_AUTO_TEST_SUITE( simd_tests );


// CTOR tests
BOOST_AUTO_TEST_CASE( simd_ctor_from_floats_test )
{
    // Contruct from 4 floats
    vfp_t uut0(-100.0, -0.5, 0.0, 100.0);

    // Checks
    BOOST_CHECK_CLOSE(uut0[0], -100.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut0[1],   -0.5, result_tolerance);
    BOOST_CHECK_CLOSE(uut0[2],    0.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut0[3],  100.0, result_tolerance);

    // Contruct from 3 floats
    vfp_t uut1(0.5, -0.5, 0.0);

    // Checks
    BOOST_CHECK_CLOSE(uut1[0],  0.5, result_tolerance);
    BOOST_CHECK_CLOSE(uut1[1], -0.5, result_tolerance);
    BOOST_CHECK_CLOSE(uut1[2],  0.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut1[3],  0.0, result_tolerance);

    // Contruct from 2 floats
    vfp_t uut2(0.5, -0.5);

    // Checks
    BOOST_CHECK_CLOSE(uut2[0],  0.5, result_tolerance);
    BOOST_CHECK_CLOSE(uut2[1], -0.5, result_tolerance);
    BOOST_CHECK_CLOSE(uut2[2], -0.5, result_tolerance);
    BOOST_CHECK_CLOSE(uut2[3], -0.5, result_tolerance);

    // Contruct from 1 floats
    vfp_t uut3(0.5);

    // Checks
    BOOST_CHECK_CLOSE(uut3[0], 0.5, result_tolerance);
    BOOST_CHECK_CLOSE(uut3[1], 0.5, result_tolerance);
    BOOST_CHECK_CLOSE(uut3[2], 0.5, result_tolerance);
    BOOST_CHECK_CLOSE(uut3[3], 0.5, result_tolerance);
    
    // Contruct from array of floats
    float a[] = { 100.0, 0.5, 0.0, -100.0 };
    vfp_t uut4(a);

    // Checks
    BOOST_CHECK_CLOSE(uut4[0],  100.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut4[1],    0.5, result_tolerance);
    BOOST_CHECK_CLOSE(uut4[2],    0.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut4[3], -100.0, result_tolerance);
}

BOOST_AUTO_TEST_CASE( copy_ctor_test )
{
    vfp_t uut0(-100.0, -0.5, 0.0, 100.0);
    vfp_t uut1(uut0);

    // Checks
    BOOST_CHECK_CLOSE(uut1[0], -100.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut1[1],   -0.5, result_tolerance);
    BOOST_CHECK_CLOSE(uut1[2],    0.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut1[3],  100.0, result_tolerance);
}

// Member access operator tests
BOOST_AUTO_TEST_CASE( operator_float_ptr_test )
{
    vfp_t uut0(-100.0, -0.5, 0.0, 100.0);
    float *a = uut0;

    // Checks
    BOOST_CHECK_CLOSE(a[0], -100.0, result_tolerance);
    BOOST_CHECK_CLOSE(a[1],   -0.5, result_tolerance);
    BOOST_CHECK_CLOSE(a[2],    0.0, result_tolerance);
    BOOST_CHECK_CLOSE(a[3],  100.0, result_tolerance);
}

BOOST_AUTO_TEST_CASE( operator_const_float_ptr_test )
{
    const vfp_t uut0(-100.0, -0.5, 0.0, 100.0);
    const float *a = uut0;

    // Checks
    BOOST_CHECK_CLOSE(a[0], -100.0, result_tolerance);
    BOOST_CHECK_CLOSE(a[1],   -0.5, result_tolerance);
    BOOST_CHECK_CLOSE(a[2],    0.0, result_tolerance);
    BOOST_CHECK_CLOSE(a[3],  100.0, result_tolerance);
}

// Operator tests
// Unary operator tests
BOOST_AUTO_TEST_CASE( operator_equals_test )
{
    vfp_t uut0(-100.0, -0.5, 0.0, 100.0);
    vfp_t uut1 = uut0;

    // Checks
    BOOST_CHECK_CLOSE(uut0[0], -100.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut0[1],   -0.5, result_tolerance);
    BOOST_CHECK_CLOSE(uut0[2],    0.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut0[3],  100.0, result_tolerance);
}

BOOST_AUTO_TEST_CASE( operator_plus_equals_test )
{
    // Versus vfp_t
    vfp_t uut0(-100.0, -0.5, 10.0,  100.0);
    vfp_t uut1(   5.0, -0.5,  1.0, -100.0);
    uut0 += uut1;

    // Checks
    BOOST_CHECK_CLOSE(uut0[0], -95.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut0[1],  -1.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut0[2],  11.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut0[3],   0.0, result_tolerance);

    // Versus float
    vfp_t uut2(-100.0, -0.5, 10.0,  100.0);
    uut2 += 5.0;

    // Checks
    BOOST_CHECK_CLOSE(uut2[0], -95.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut2[1],   4.5, result_tolerance);
    BOOST_CHECK_CLOSE(uut2[2],  15.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut2[3], 105.0, result_tolerance);
}

BOOST_AUTO_TEST_CASE( operator_minus_equals_test )
{
    // Versus vfp_t
    vfp_t uut0(-100.0, -0.5, 10.0,  100.0);
    vfp_t uut1(   5.0, -0.5,  1.0, -100.0);
    uut0 -= uut1;

    // Checks
    BOOST_CHECK_CLOSE(uut0[0], -105.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut0[1],    0.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut0[2],    9.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut0[3],  200.0, result_tolerance);

    // Versus float
    vfp_t uut2(-100.0, -0.5, 10.0,  100.0);
    uut2 -= 5.0;

    // Checks
    BOOST_CHECK_CLOSE(uut2[0], -105.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut2[1],   -5.5, result_tolerance);
    BOOST_CHECK_CLOSE(uut2[2],    5.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut2[3],   95.0, result_tolerance);
}

BOOST_AUTO_TEST_CASE( operator_times_equals_test )
{
    // Versus vfp_t
    vfp_t uut0(-100.0, -0.5, 10.0,  100.0);
    vfp_t uut1(   5.0, -0.5,  1.0, -100.0);
    uut0 *= uut1;

    // Checks
    BOOST_CHECK_CLOSE(uut0[0],   -500.0 , result_tolerance);
    BOOST_CHECK_CLOSE(uut0[1],      0.25, result_tolerance);
    BOOST_CHECK_CLOSE(uut0[2],     10.0 , result_tolerance);
    BOOST_CHECK_CLOSE(uut0[3], -10000.0 , result_tolerance);

    // Versus float
    vfp_t uut2(-100.0, -0.5, 10.0,  100.0);
    uut2 *= 5.0;

    // Checks
    BOOST_CHECK_CLOSE(uut2[0], -500.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut2[1],   -2.5, result_tolerance);
    BOOST_CHECK_CLOSE(uut2[2],   50.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut2[3],  500.0, result_tolerance);
}

BOOST_AUTO_TEST_CASE( operator_divide_equals_test )
{
    // Versus vfp_t
    vfp_t uut0(-100.0, -0.5, 10.0,  100.0);
    vfp_t uut1(   5.0, -0.5,  1.0, -100.0);
    uut0 /= uut1;

    // Checks
    BOOST_CHECK_CLOSE(uut0[0], -20.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut0[1],   1.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut0[2],  10.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut0[3],  -1.0, result_tolerance);

    // Versus float
    vfp_t uut2(-100.0, -0.5, 10.0,  100.0);
    uut2 /= 5.0;

    // Checks
    BOOST_CHECK_CLOSE(uut2[0], -20.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut2[1],  -0.1, result_tolerance);
    BOOST_CHECK_CLOSE(uut2[2],   2.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut2[3],  20.0, result_tolerance);
}

BOOST_AUTO_TEST_CASE( operator_negate_test )
{
    vfp_t uut0(-100.0, -0.5, 10.0, 100.0);
    vfp_t uut1(-uut0);

    // Checks
    BOOST_CHECK_CLOSE(uut1[0],  100.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut1[1],    0.5, result_tolerance);
    BOOST_CHECK_CLOSE(uut1[2],  -10.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut1[3], -100.0, result_tolerance);
}

BOOST_AUTO_TEST_CASE( operator_and_equals_test )
{
    // Versus vfp_t
    vfp_t uut0(bit_cast<unsigned int, float>(0xffffffff), bit_cast<unsigned int, float>(0x00000000), bit_cast<unsigned int, float>(0xa5a5a5a5), bit_cast<unsigned int, float>(0x3c3c3c3c));
    vfp_t uut1(bit_cast<unsigned int, float>(0xa5a53c3c), bit_cast<unsigned int, float>(0xffff0000), bit_cast<unsigned int, float>(0xffffa5a5), bit_cast<unsigned int, float>(0x00003c3c));
    uut0 &= uut1;

    // Checks
    BOOST_CHECK((bit_cast<float, unsigned int>(uut0[0])) == 0xa5a53c3c);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut0[1])) == 0x00000000);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut0[2])) == 0xa5a5a5a5);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut0[3])) == 0x00003c3c);

    // Versus float
    vfp_t uut2(bit_cast<unsigned int, float>(0xffffffff), bit_cast<unsigned int, float>(0x00000000), bit_cast<unsigned int, float>(0xa5a5a5a5), bit_cast<unsigned int, float>(0x3c3c3c3c));
    uut2 &= bit_cast<unsigned int, float>(0xa5a5a5a5);

    // Checks
    BOOST_CHECK((bit_cast<float, unsigned int>(uut2[0])) == 0xa5a5a5a5);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut2[1])) == 0x00000000);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut2[2])) == 0xa5a5a5a5);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut2[3])) == 0x24242424);
}

BOOST_AUTO_TEST_CASE( operator_or_equals_test )
{
    // Versus vfp_t
    vfp_t uut0(bit_cast<unsigned int, float>(0xffffffff), bit_cast<unsigned int, float>(0x00000000), bit_cast<unsigned int, float>(0xa5a5a5a5), bit_cast<unsigned int, float>(0x3c3c3c3c));
    vfp_t uut1(bit_cast<unsigned int, float>(0xa5a53c3c), bit_cast<unsigned int, float>(0xffff0000), bit_cast<unsigned int, float>(0xffffa5a5), bit_cast<unsigned int, float>(0x00003c3c));
    uut0 |= uut1;

    // Checks
    BOOST_CHECK((bit_cast<float, unsigned int>(uut0[0])) == 0xffffffff);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut0[1])) == 0xffff0000);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut0[2])) == 0xffffa5a5);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut0[3])) == 0x3c3c3c3c);

    // Versus float
    vfp_t uut2(bit_cast<unsigned int, float>(0xffffffff), bit_cast<unsigned int, float>(0x00000000), bit_cast<unsigned int, float>(0xa5a5a5a5), bit_cast<unsigned int, float>(0x3c3c3c3c));
    uut2 |= bit_cast<unsigned int, float>(0xa5a5a5a5);

    // Checks
    BOOST_CHECK((bit_cast<float, unsigned int>(uut2[0])) == 0xffffffff);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut2[1])) == 0xa5a5a5a5);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut2[2])) == 0xa5a5a5a5);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut2[3])) == 0xbdbdbdbd);
}

BOOST_AUTO_TEST_CASE( operator_xor_equals_test )
{
    // Versus vfp_t
    vfp_t uut0(bit_cast<unsigned int, float>(0xffffffff), bit_cast<unsigned int, float>(0x00000000), bit_cast<unsigned int, float>(0xa5a5a5a5), bit_cast<unsigned int, float>(0x3c3c3c3c));
    vfp_t uut1(bit_cast<unsigned int, float>(0xa5a53c3c), bit_cast<unsigned int, float>(0xffff0000), bit_cast<unsigned int, float>(0xffffa5a5), bit_cast<unsigned int, float>(0x00003c3c));
    uut0 ^= uut1;

    // Checks
    BOOST_CHECK((bit_cast<float, unsigned int>(uut0[0])) == 0x5a5ac3c3);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut0[1])) == 0xffff0000);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut0[2])) == 0x5a5a0000);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut0[3])) == 0x3c3c0000);

    // Versus float
    vfp_t uut2(bit_cast<unsigned int, float>(0xffffffff), bit_cast<unsigned int, float>(0x00000000), bit_cast<unsigned int, float>(0xa5a5a5a5), bit_cast<unsigned int, float>(0x3c3c3c3c));
    uut2 ^= bit_cast<unsigned int, float>(0xa5a5a5a5);

    // Checks
    BOOST_CHECK((bit_cast<float, unsigned int>(uut2[0])) == 0x5a5a5a5a);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut2[1])) == 0xa5a5a5a5);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut2[2])) == 0x00000000);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut2[3])) == 0x99999999);
}


// Friendly operator tests
BOOST_AUTO_TEST_CASE( operator_plus_test )
{
    // Versus vfp_t
    vfp_t uut0(-100.0, -0.5, 10.0,  100.0);
    vfp_t uut1(   5.0, -0.5,  1.0, -100.0);
    vfp_t uut2(uut0 + uut1);

    // Checks
    BOOST_CHECK_CLOSE(uut2[0], -95.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut2[1],  -1.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut2[2],  11.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut2[3],   0.0, result_tolerance);

    // Versus float
    vfp_t uut3(uut0 + 5.1);

    // Checks
    BOOST_CHECK_CLOSE(uut3[0], -94.9, result_tolerance);
    BOOST_CHECK_CLOSE(uut3[1],   4.6, result_tolerance);
    BOOST_CHECK_CLOSE(uut3[2],  15.1, result_tolerance);
    BOOST_CHECK_CLOSE(uut3[3], 105.1, result_tolerance);

    // Versus float reverse
    vfp_t uut4(5.1 + uut0);

    // Checks
    BOOST_CHECK_CLOSE(uut4[0], -94.9, result_tolerance);
    BOOST_CHECK_CLOSE(uut4[1],   4.6, result_tolerance);
    BOOST_CHECK_CLOSE(uut4[2],  15.1, result_tolerance);
    BOOST_CHECK_CLOSE(uut4[3], 105.1, result_tolerance);
}

BOOST_AUTO_TEST_CASE( operator_minus_test )
{
    // Versus vfp_t
    vfp_t uut0(-100.0, -0.5, 10.0,  100.0);
    vfp_t uut1(   5.0, -0.5,  1.0, -100.0);
    vfp_t uut2(uut0 - uut1);

    // Checks
    BOOST_CHECK_CLOSE(uut2[0], -105.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut2[1],    0.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut2[2],    9.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut2[3],  200.0, result_tolerance);

    // Versus float
    vfp_t uut3(uut0 - 5.1);

    // Checks
    BOOST_CHECK_CLOSE(uut3[0], -105.1, result_tolerance);
    BOOST_CHECK_CLOSE(uut3[1],   -5.6, result_tolerance);
    BOOST_CHECK_CLOSE(uut3[2],    4.9, result_tolerance);
    BOOST_CHECK_CLOSE(uut3[3],   94.9, result_tolerance);

    // Versus float reverse
    vfp_t uut4(5.1 - uut0);

    // Checks
    BOOST_CHECK_CLOSE(uut4[0],  105.1, result_tolerance);
    BOOST_CHECK_CLOSE(uut4[1],    5.6, result_tolerance);
    BOOST_CHECK_CLOSE(uut4[2],   -4.9, result_tolerance);
    BOOST_CHECK_CLOSE(uut4[3],  -94.9, result_tolerance);
}

BOOST_AUTO_TEST_CASE( operator_times_test )
{
    // Versus vfp_t
    vfp_t uut0(-100.0, -0.5, 10.0,  100.0);
    vfp_t uut1(   5.0, -0.5,  1.0, -100.0);
    vfp_t uut2(uut0 * uut1);

    // Checks
    BOOST_CHECK_CLOSE(uut2[0],   -500.0 , result_tolerance);
    BOOST_CHECK_CLOSE(uut2[1],      0.25, result_tolerance);
    BOOST_CHECK_CLOSE(uut2[2],     10.0 , result_tolerance);
    BOOST_CHECK_CLOSE(uut2[3], -10000.0 , result_tolerance);

    // Versus float
    vfp_t uut3(uut0 * 5.1);

    // Checks
    BOOST_CHECK_CLOSE(uut3[0], -510.0 , result_tolerance);
    BOOST_CHECK_CLOSE(uut3[1],   -2.55, result_tolerance);
    BOOST_CHECK_CLOSE(uut3[2],   51.0 , result_tolerance);
    BOOST_CHECK_CLOSE(uut3[3],  510.0 , result_tolerance);

    // Versus float reverse
    vfp_t uut4(5.1 * uut0);

    // Checks
    BOOST_CHECK_CLOSE(uut4[0], -510.0 , result_tolerance);
    BOOST_CHECK_CLOSE(uut4[1],   -2.55, result_tolerance);
    BOOST_CHECK_CLOSE(uut4[2],   51.0 , result_tolerance);
    BOOST_CHECK_CLOSE(uut4[3],  510.0 , result_tolerance);
}

BOOST_AUTO_TEST_CASE( operator_divide_test )
{
    // Versus vfp_t
    vfp_t uut0(-100.0, -0.5, 10.0,  100.0);
    vfp_t uut1(   5.0, -0.5,  1.0, -100.0);
    vfp_t uut2(uut0 / uut1);

    // Checks
    BOOST_CHECK_CLOSE(uut2[0], -20.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut2[1],   1.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut2[2],  10.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut2[3],  -1.0, result_tolerance);

    // Versus float
    vfp_t uut3(uut0 / 5.0);

    // Checks
    BOOST_CHECK_CLOSE(uut3[0], -20.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut3[1],  -0.1, result_tolerance);
    BOOST_CHECK_CLOSE(uut3[2],   2.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut3[3],  20.0, result_tolerance);

    // Versus float reverse
    vfp_t uut4(5.0 / uut0);

    // Checks
    BOOST_CHECK_CLOSE(uut4[0],  -0.05, result_tolerance);
    BOOST_CHECK_CLOSE(uut4[1], -10.0 , result_tolerance);
    BOOST_CHECK_CLOSE(uut4[2],   0.5 , result_tolerance);
    BOOST_CHECK_CLOSE(uut4[3],   0.05, result_tolerance);
}

BOOST_AUTO_TEST_CASE( operator_and_test )
{
    // Versus vfp_t
    vfp_t uut0(bit_cast<unsigned int, float>(0xffffffff), bit_cast<unsigned int, float>(0x00000000), bit_cast<unsigned int, float>(0xa5a5a5a5), bit_cast<unsigned int, float>(0x3c3c3c3c));
    vfp_t uut1(bit_cast<unsigned int, float>(0xa5a53c3c), bit_cast<unsigned int, float>(0xffff0000), bit_cast<unsigned int, float>(0xffffa5a5), bit_cast<unsigned int, float>(0x00003c3c));
    vfp_t uut2(uut0 & uut1);

    // Checks
    BOOST_CHECK((bit_cast<float, unsigned int>(uut2[0])) == 0xa5a53c3c);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut2[1])) == 0x00000000);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut2[2])) == 0xa5a5a5a5);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut2[3])) == 0x00003c3c);

    // Versus float
    vfp_t uut3(uut0 & bit_cast<unsigned int, float>(0xa5a53c3c));

    // Checks
    BOOST_CHECK((bit_cast<float, unsigned int>(uut3[0])) == 0xa5a53c3c);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut3[1])) == 0x00000000);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut3[2])) == 0xa5a52424);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut3[3])) == 0x24243c3c);

    // Versus float reverse
    vfp_t uut4(bit_cast<unsigned int, float>(0xa5a53c3c) & uut0);

    // Checks
    BOOST_CHECK((bit_cast<float, unsigned int>(uut4[0])) == 0xa5a53c3c);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut4[1])) == 0x00000000);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut4[2])) == 0xa5a52424);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut4[3])) == 0x24243c3c);
}

BOOST_AUTO_TEST_CASE( operator_or_test )
{
    // Versus vfp_t
    vfp_t uut0(bit_cast<unsigned int, float>(0xffffffff), bit_cast<unsigned int, float>(0x00000000), bit_cast<unsigned int, float>(0xa5a5a5a5), bit_cast<unsigned int, float>(0x3c3c3c3c));
    vfp_t uut1(bit_cast<unsigned int, float>(0xa5a53c3c), bit_cast<unsigned int, float>(0xffff0000), bit_cast<unsigned int, float>(0xffffa5a5), bit_cast<unsigned int, float>(0x00003c3c));
    vfp_t uut2(uut0 | uut1);

    // Checks
    BOOST_CHECK((bit_cast<float, unsigned int>(uut2[0])) == 0xffffffff);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut2[1])) == 0xffff0000);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut2[2])) == 0xffffa5a5);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut2[3])) == 0x3c3c3c3c);

    // Versus float
    vfp_t uut3(uut0 | bit_cast<unsigned int, float>(0xa5a53c3c));

    // Checks
    BOOST_CHECK((bit_cast<float, unsigned int>(uut3[0])) == 0xffffffff);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut3[1])) == 0xa5a53c3c);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut3[2])) == 0xa5a5bdbd);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut3[3])) == 0xbdbd3c3c);

    // Versus float reverse
    vfp_t uut4(bit_cast<unsigned int, float>(0xa5a53c3c) | uut0);

    // Checks
    BOOST_CHECK((bit_cast<float, unsigned int>(uut4[0])) == 0xffffffff);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut4[1])) == 0xa5a53c3c);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut4[2])) == 0xa5a5bdbd);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut4[3])) == 0xbdbd3c3c);
}

BOOST_AUTO_TEST_CASE( operator_xor_test )
{
    // Versus vfp_t
    vfp_t uut0(bit_cast<unsigned int, float>(0xffffffff), bit_cast<unsigned int, float>(0x00000000), bit_cast<unsigned int, float>(0xa5a5a5a5), bit_cast<unsigned int, float>(0x3c3c3c3c));
    vfp_t uut1(bit_cast<unsigned int, float>(0xa5a53c3c), bit_cast<unsigned int, float>(0xffff0000), bit_cast<unsigned int, float>(0xffffa5a5), bit_cast<unsigned int, float>(0x00003c3c));
    vfp_t uut2(uut0 ^ uut1);

    // Checks
    BOOST_CHECK((bit_cast<float, unsigned int>(uut2[0])) == 0x5a5ac3c3);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut2[1])) == 0xffff0000);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut2[2])) == 0x5a5a0000);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut2[3])) == 0x3c3c0000);

    // Versus float
    vfp_t uut3(uut0 ^ bit_cast<unsigned int, float>(0xa5a53c3c));

    // Checks
    BOOST_CHECK((bit_cast<float, unsigned int>(uut3[0])) == 0x5a5ac3c3);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut3[1])) == 0xa5a53c3c);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut3[2])) == 0x00009999);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut3[3])) == 0x99990000);

    // Versus float reverse
    vfp_t uut4(bit_cast<unsigned int, float>(0xa5a53c3c) ^ uut0);

    // Checks
    BOOST_CHECK((bit_cast<float, unsigned int>(uut4[0])) == 0x5a5ac3c3);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut4[1])) == 0xa5a53c3c);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut4[2])) == 0x00009999);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut4[3])) == 0x99990000);
}


// Comparison tests
BOOST_AUTO_TEST_CASE( operator_equality_test )
{
    // Versus vfp_t
    vfp_t uut0(-100.0, -0.5, 10.0,  100.0);
    vfp_t uut1(   5.0, -0.5,  1.0, -100.0);
    vfp_t uut2(uut0 == uut1);

    // Checks
    BOOST_CHECK((bit_cast<float, unsigned int>(uut2[0])) == 0x00000000);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut2[1])) == 0xffffffff);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut2[2])) == 0x00000000);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut2[3])) == 0x00000000);

    // Versus float
    vfp_t uut3(uut0 == -100.0);

    // Checks
    BOOST_CHECK((bit_cast<float, unsigned int>(uut3[0])) == 0xffffffff);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut3[1])) == 0x00000000);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut3[2])) == 0x00000000);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut3[3])) == 0x00000000);

    // Versus float reverse
    vfp_t uut4(-100.0 == uut0);

    // Checks
    BOOST_CHECK((bit_cast<float, unsigned int>(uut4[0])) == 0xffffffff);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut4[1])) == 0x00000000);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut4[2])) == 0x00000000);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut4[3])) == 0x00000000);
}

BOOST_AUTO_TEST_CASE( operator_not_equality_test )
{
    // Versus vfp_t
    vfp_t uut0(-100.0, -0.5, 10.0,  100.0);
    vfp_t uut1(   5.0, -0.5,  1.0, -100.0);
    vfp_t uut2(uut0 != uut1);

    // Checks
    BOOST_CHECK((bit_cast<float, unsigned int>(uut2[0])) == 0xffffffff);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut2[1])) == 0x00000000);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut2[2])) == 0xffffffff);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut2[3])) == 0xffffffff);

    // Versus float
    vfp_t uut3(uut0 != -100.0);

    // Checks
    BOOST_CHECK((bit_cast<float, unsigned int>(uut3[0])) == 0x00000000);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut3[1])) == 0xffffffff);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut3[2])) == 0xffffffff);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut3[3])) == 0xffffffff);

    // Versus float reverse
    vfp_t uut4(-100.0 != uut0);

    // Checks
    BOOST_CHECK((bit_cast<float, unsigned int>(uut4[0])) == 0x00000000);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut4[1])) == 0xffffffff);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut4[2])) == 0xffffffff);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut4[3])) == 0xffffffff);
}

BOOST_AUTO_TEST_CASE( operator_greater_test )
{
    // Versus vfp_t
    vfp_t uut0(-100.0, -0.5, 10.0,  100.0);
    vfp_t uut1(   5.0, -0.5,  1.0, -100.0);
    vfp_t uut2(uut0 > uut1);

    // Checks
    BOOST_CHECK((bit_cast<float, unsigned int>(uut2[0])) == 0x00000000);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut2[1])) == 0x00000000);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut2[2])) == 0xffffffff);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut2[3])) == 0xffffffff);

    // Versus float
    vfp_t uut3(uut0 > 10.0);

    // Checks
    BOOST_CHECK((bit_cast<float, unsigned int>(uut3[0])) == 0x00000000);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut3[1])) == 0x00000000);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut3[2])) == 0x00000000);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut3[3])) == 0xffffffff);

    // Versus float reverse
    vfp_t uut4(10.0 > uut0);

    // Checks
    BOOST_CHECK((bit_cast<float, unsigned int>(uut4[0])) == 0xffffffff);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut4[1])) == 0xffffffff);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut4[2])) == 0x00000000);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut4[3])) == 0x00000000);
}

BOOST_AUTO_TEST_CASE( operator_greater_equal_test )
{
    // Versus vfp_t
    vfp_t uut0(-100.0, -0.5, 10.0,  100.0);
    vfp_t uut1(   5.0, -0.5,  1.0, -100.0);
    vfp_t uut2(uut0 >= uut1);

    // Checks
    BOOST_CHECK((bit_cast<float, unsigned int>(uut2[0])) == 0x00000000);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut2[1])) == 0xffffffff);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut2[2])) == 0xffffffff);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut2[3])) == 0xffffffff);

    // Versus float
    vfp_t uut3(uut0 >= 10.0);

    // Checks
    BOOST_CHECK((bit_cast<float, unsigned int>(uut3[0])) == 0x00000000);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut3[1])) == 0x00000000);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut3[2])) == 0xffffffff);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut3[3])) == 0xffffffff);

    // Versus float reverse
    vfp_t uut4(10.0 >= uut0);

    // Checks
    BOOST_CHECK((bit_cast<float, unsigned int>(uut4[0])) == 0xffffffff);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut4[1])) == 0xffffffff);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut4[2])) == 0xffffffff);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut4[3])) == 0x00000000);
}

BOOST_AUTO_TEST_CASE( operator_lesser_test )
{
    // Versus vfp_t
    vfp_t uut0(-100.0, -0.5, 10.0,  100.0);
    vfp_t uut1(   5.0, -0.5,  1.0, -100.0);
    vfp_t uut2(uut0 < uut1);

    // Checks
    BOOST_CHECK((bit_cast<float, unsigned int>(uut2[0])) == 0xffffffff);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut2[1])) == 0x00000000);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut2[2])) == 0x00000000);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut2[3])) == 0x00000000);

    // Versus float
    vfp_t uut3(uut0 < 10.0);

    // Checks
    BOOST_CHECK((bit_cast<float, unsigned int>(uut3[0])) == 0xffffffff);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut3[1])) == 0xffffffff);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut3[2])) == 0x00000000);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut3[3])) == 0x00000000);

    // Versus float reverse
    vfp_t uut4(10.0 < uut0);

    // Checks
    BOOST_CHECK((bit_cast<float, unsigned int>(uut4[0])) == 0x00000000);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut4[1])) == 0x00000000);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut4[2])) == 0x00000000);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut4[3])) == 0xffffffff);
}

BOOST_AUTO_TEST_CASE( operator_lesser_equal_test )
{
    // Versus vfp_t
    vfp_t uut0(-100.0, -0.5, 10.0,  100.0);
    vfp_t uut1(   5.0, -0.5,  1.0, -100.0);
    vfp_t uut2(uut0 <= uut1);

    // Checks
    BOOST_CHECK((bit_cast<float, unsigned int>(uut2[0])) == 0xffffffff);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut2[1])) == 0xffffffff);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut2[2])) == 0x00000000);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut2[3])) == 0x00000000);

    // Versus float
    vfp_t uut3(uut0 <= 10.0);

    // Checks
    BOOST_CHECK((bit_cast<float, unsigned int>(uut3[0])) == 0xffffffff);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut3[1])) == 0xffffffff);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut3[2])) == 0xffffffff);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut3[3])) == 0x00000000);

    // Versus float reverse
    vfp_t uut4(10.0 <= uut0);

    // Checks
    BOOST_CHECK((bit_cast<float, unsigned int>(uut4[0])) == 0x00000000);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut4[1])) == 0x00000000);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut4[2])) == 0xffffffff);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut4[3])) == 0xffffffff);
}


// Friendly function tests
BOOST_AUTO_TEST_CASE( sqrt_test )
{
    // Versus vfp_t
    vfp_t uut0(49.0, 9.0, 25.0, 100.0);
    vfp_t uut1(sqrt(uut0));

    // Checks
    BOOST_CHECK_CLOSE(uut1[0],  7.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut1[1],  3.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut1[2],  5.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut1[3], 10.0, result_tolerance);

    // Versus float
    vfp_t uut2(sqrt(25.0));

    // Checks
    BOOST_CHECK_CLOSE(uut2[0], 5.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut2[1], 5.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut2[2], 5.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut2[3], 5.0, result_tolerance);
}

BOOST_AUTO_TEST_CASE( inverse_test )
{
    // Versus vfp_t
    vfp_t uut0(-100.0, -10.0, 25.0, 100.0);
    vfp_t uut1(inverse(uut0));

    // Checks
    BOOST_CHECK_CLOSE(uut1[0], -0.01, result_tolerance);
    BOOST_CHECK_CLOSE(uut1[1], -0.1 , result_tolerance);
    BOOST_CHECK_CLOSE(uut1[2],  0.04, result_tolerance);
    BOOST_CHECK_CLOSE(uut1[3],  0.01, result_tolerance);

    // Versus float
    vfp_t uut2(inverse(25.0));

    // Checks
    BOOST_CHECK_CLOSE(uut2[0], 0.04, result_tolerance);
    BOOST_CHECK_CLOSE(uut2[1], 0.04, result_tolerance);
    BOOST_CHECK_CLOSE(uut2[2], 0.04, result_tolerance);
    BOOST_CHECK_CLOSE(uut2[3], 0.04, result_tolerance);
}

BOOST_AUTO_TEST_CASE( inverse_sqrt_test )
{
    // Versus vfp_t
    vfp_t uut0(49.0, 9.0, 25.0, 100.0);
    vfp_t uut1(inverse_sqrt(uut0));

    // Checks
    BOOST_CHECK_CLOSE(uut1[0], 0.142857143, result_tolerance);
    BOOST_CHECK_CLOSE(uut1[1], 0.333333333, result_tolerance);
    BOOST_CHECK_CLOSE(uut1[2], 0.2        , result_tolerance);
    BOOST_CHECK_CLOSE(uut1[3], 0.1        , result_tolerance);

    // Versus float
    vfp_t uut2(inverse_sqrt(25.0));

    // Checks
    BOOST_CHECK_CLOSE(uut2[0], 0.2, result_tolerance);
    BOOST_CHECK_CLOSE(uut2[1], 0.2, result_tolerance);
    BOOST_CHECK_CLOSE(uut2[2], 0.2, result_tolerance);
    BOOST_CHECK_CLOSE(uut2[3], 0.2, result_tolerance);
}

BOOST_AUTO_TEST_CASE( approx_inverse_test )
{
    // Versus vfp_t
    vfp_t uut0(-100.0, -9.0, 25.0, 100.0);
    vfp_t uut1(approx_inverse(uut0));

    // Checks
    BOOST_CHECK_CLOSE(uut1[0], -0.01       , result_tolerance * 3000.0);
    BOOST_CHECK_CLOSE(uut1[1], -0.111111111, result_tolerance * 3000.0);
    BOOST_CHECK_CLOSE(uut1[2],  0.04       , result_tolerance * 3000.0);
    BOOST_CHECK_CLOSE(uut1[3],  0.01       , result_tolerance * 3000.0);

    // Versus float
    vfp_t uut2(approx_inverse(25.0));

    // Checks
    BOOST_CHECK_CLOSE(uut2[0], 0.04, result_tolerance * 3000.0);
    BOOST_CHECK_CLOSE(uut2[1], 0.04, result_tolerance * 3000.0);
    BOOST_CHECK_CLOSE(uut2[2], 0.04, result_tolerance * 3000.0);
    BOOST_CHECK_CLOSE(uut2[3], 0.04, result_tolerance * 3000.0);
}

BOOST_AUTO_TEST_CASE( approx_inverse_sqrt_test )
{
    // Versus vfp_t
    vfp_t uut0(49.0, 9.0, 25.0, 100.0);
    vfp_t uut1(approx_inverse_sqrt(uut0));

    // Checks
    BOOST_CHECK_CLOSE(uut1[0], 0.142857143, result_tolerance * 3000.0);
    BOOST_CHECK_CLOSE(uut1[1], 0.333333333, result_tolerance * 3000.0);
    BOOST_CHECK_CLOSE(uut1[2], 0.2        , result_tolerance * 3000.0);
    BOOST_CHECK_CLOSE(uut1[3], 0.1        , result_tolerance * 3000.0);

    // Versus float
    vfp_t uut2(approx_inverse_sqrt(25.0));

    // Checks
    BOOST_CHECK_CLOSE(uut2[0], 0.2, result_tolerance * 3000.0);
    BOOST_CHECK_CLOSE(uut2[1], 0.2, result_tolerance * 3000.0);
    BOOST_CHECK_CLOSE(uut2[2], 0.2, result_tolerance * 3000.0);
    BOOST_CHECK_CLOSE(uut2[3], 0.2, result_tolerance * 3000.0);
}

BOOST_AUTO_TEST_CASE( abs_test )
{
    // Versus vfp_t
    vfp_t uut0(-100.0, -9.0, 25.0, 100.0);
    vfp_t uut1(abs(uut0));

    // Checks
    BOOST_CHECK_CLOSE(uut1[0], 100.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut1[1],   9.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut1[2],  25.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut1[3], 100.0, result_tolerance);

    // Versus float
    vfp_t uut2(abs(25.0));

    // Checks
    BOOST_CHECK_CLOSE(uut2[0], 25.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut2[1], 25.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut2[2], 25.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut2[3], 25.0, result_tolerance);

    vfp_t uut3(abs(-25.0));

    // Checks
    BOOST_CHECK_CLOSE(uut3[0], 25.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut3[1], 25.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut3[2], 25.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut3[3], 25.0, result_tolerance);
}

BOOST_AUTO_TEST_CASE( max_test )
{
    // Versus vfp_t
    vfp_t uut0(-100.0, -0.5, 10.0,  100.0);
    vfp_t uut1(   5.0, -0.5,  1.0, -100.0);
    vfp_t uut2(max(uut0, uut1));

    // Checks
    BOOST_CHECK_CLOSE(uut2[0],   5.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut2[1],  -0.5, result_tolerance);
    BOOST_CHECK_CLOSE(uut2[2],  10.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut2[3], 100.0, result_tolerance);

    // Versus float
    vfp_t uut3(max(uut0, 10.0));

    // Checks
    BOOST_CHECK_CLOSE(uut3[0],  10.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut3[1],  10.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut3[2],  10.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut3[3], 100.0, result_tolerance);

    // Versus float reverse
    vfp_t uut4(max(10.0, uut0));

    // Checks
    BOOST_CHECK_CLOSE(uut4[0],  10.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut4[1],  10.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut4[2],  10.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut4[3], 100.0, result_tolerance);
}

BOOST_AUTO_TEST_CASE( min_test )
{
    // Versus vfp_t
    vfp_t uut0(-100.0, -0.5, 10.0,  100.0);
    vfp_t uut1(   5.0, -0.5,  1.0, -100.0);
    vfp_t uut2(min(uut0, uut1));

    // Checks
    BOOST_CHECK_CLOSE(uut2[0], -100.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut2[1],   -0.5, result_tolerance);
    BOOST_CHECK_CLOSE(uut2[2],    1.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut2[3], -100.0, result_tolerance);

    // Versus float
    vfp_t uut3(min(uut0, 10.0));

    // Checks
    BOOST_CHECK_CLOSE(uut3[0], -100.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut3[1],   -0.5, result_tolerance);
    BOOST_CHECK_CLOSE(uut3[2],   10.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut3[3],   10.0, result_tolerance);

    // Versus float reverse
    vfp_t uut4(min(10.0, uut0));

    // Checks
    BOOST_CHECK_CLOSE(uut4[0], -100.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut4[1],   -0.5, result_tolerance);
    BOOST_CHECK_CLOSE(uut4[2],   10.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut4[3],   10.0, result_tolerance);
}
        

// Non standard friendly function tests
BOOST_AUTO_TEST_CASE( move_mask_test )
{
    // Check 0
    vfp_t uut0(bit_cast<unsigned int, float>(0x00000000), bit_cast<unsigned int, float>(0x00000000), bit_cast<unsigned int, float>(0x00000000), bit_cast<unsigned int, float>(0x00000000));
    BOOST_CHECK(move_mask(uut0) == 0x0);

    // Check 1
    vfp_t uut1_0(bit_cast<unsigned int, float>(0x00000000), bit_cast<unsigned int, float>(0x00000000), bit_cast<unsigned int, float>(0x00000000), bit_cast<unsigned int, float>(0xffffffff));
    vfp_t uut1_1(bit_cast<unsigned int, float>(0x00000000), bit_cast<unsigned int, float>(0x00000000), bit_cast<unsigned int, float>(0xffffffff), bit_cast<unsigned int, float>(0x00000000));
    vfp_t uut1_2(bit_cast<unsigned int, float>(0x00000000), bit_cast<unsigned int, float>(0xffffffff), bit_cast<unsigned int, float>(0x00000000), bit_cast<unsigned int, float>(0x00000000));
    vfp_t uut1_3(bit_cast<unsigned int, float>(0xffffffff), bit_cast<unsigned int, float>(0x00000000), bit_cast<unsigned int, float>(0x00000000), bit_cast<unsigned int, float>(0x00000000));

    BOOST_CHECK(move_mask(uut1_0) == 0x8);
    BOOST_CHECK(move_mask(uut1_1) == 0x4);
    BOOST_CHECK(move_mask(uut1_2) == 0x2);
    BOOST_CHECK(move_mask(uut1_3) == 0x1);

    // Check 2
    vfp_t uut2_0(bit_cast<unsigned int, float>(0x00000000), bit_cast<unsigned int, float>(0x00000000), bit_cast<unsigned int, float>(0xffffffff), bit_cast<unsigned int, float>(0xffffffff));
    vfp_t uut2_1(bit_cast<unsigned int, float>(0x00000000), bit_cast<unsigned int, float>(0xffffffff), bit_cast<unsigned int, float>(0x00000000), bit_cast<unsigned int, float>(0xffffffff));
    vfp_t uut2_2(bit_cast<unsigned int, float>(0xffffffff), bit_cast<unsigned int, float>(0x00000000), bit_cast<unsigned int, float>(0x00000000), bit_cast<unsigned int, float>(0xffffffff));
    vfp_t uut2_3(bit_cast<unsigned int, float>(0x00000000), bit_cast<unsigned int, float>(0xffffffff), bit_cast<unsigned int, float>(0xffffffff), bit_cast<unsigned int, float>(0x00000000));
    vfp_t uut2_4(bit_cast<unsigned int, float>(0xffffffff), bit_cast<unsigned int, float>(0x00000000), bit_cast<unsigned int, float>(0xffffffff), bit_cast<unsigned int, float>(0x00000000));
    vfp_t uut2_5(bit_cast<unsigned int, float>(0xffffffff), bit_cast<unsigned int, float>(0xffffffff), bit_cast<unsigned int, float>(0x00000000), bit_cast<unsigned int, float>(0x00000000));

    BOOST_CHECK(move_mask(uut2_0) == 0xc);
    BOOST_CHECK(move_mask(uut2_1) == 0xa);
    BOOST_CHECK(move_mask(uut2_2) == 0x9);
    BOOST_CHECK(move_mask(uut2_3) == 0x6);
    BOOST_CHECK(move_mask(uut2_4) == 0x5);
    BOOST_CHECK(move_mask(uut2_5) == 0x3);

    // Check 3
    vfp_t uut3_0(bit_cast<unsigned int, float>(0xffffffff), bit_cast<unsigned int, float>(0xffffffff), bit_cast<unsigned int, float>(0xffffffff), bit_cast<unsigned int, float>(0x00000000));
    vfp_t uut3_1(bit_cast<unsigned int, float>(0xffffffff), bit_cast<unsigned int, float>(0xffffffff), bit_cast<unsigned int, float>(0x00000000), bit_cast<unsigned int, float>(0xffffffff));
    vfp_t uut3_2(bit_cast<unsigned int, float>(0xffffffff), bit_cast<unsigned int, float>(0x00000000), bit_cast<unsigned int, float>(0xffffffff), bit_cast<unsigned int, float>(0xffffffff));
    vfp_t uut3_3(bit_cast<unsigned int, float>(0x00000000), bit_cast<unsigned int, float>(0xffffffff), bit_cast<unsigned int, float>(0xffffffff), bit_cast<unsigned int, float>(0xffffffff));

    BOOST_CHECK(move_mask(uut3_0) == 0x7);
    BOOST_CHECK(move_mask(uut3_1) == 0xb);
    BOOST_CHECK(move_mask(uut3_2) == 0xd);
    BOOST_CHECK(move_mask(uut3_3) == 0xe);

    // Check 4
    vfp_t uut4(bit_cast<unsigned int, float>(0xffffffff), bit_cast<unsigned int, float>(0xffffffff), bit_cast<unsigned int, float>(0xffffffff), bit_cast<unsigned int, float>(0xffffffff));
    BOOST_CHECK(move_mask(uut4) == 0xf);
}

BOOST_AUTO_TEST_CASE( andnot_test )
{
    // Versus vfp_t
    vfp_t uut0(bit_cast<unsigned int, float>(0xffffffff), bit_cast<unsigned int, float>(0x00000000), bit_cast<unsigned int, float>(0xa5a5a5a5), bit_cast<unsigned int, float>(0x3c3c3c3c));
    vfp_t uut1(bit_cast<unsigned int, float>(0xa5a53c3c), bit_cast<unsigned int, float>(0xffff0000), bit_cast<unsigned int, float>(0xffffa5a5), bit_cast<unsigned int, float>(0x00003c3c));
    vfp_t uut2(andnot(uut0, uut1));

    // Checks
    BOOST_CHECK((bit_cast<float, unsigned int>(uut2[0])) == 0x00000000);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut2[1])) == 0xffff0000);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut2[2])) == 0x5a5a0000);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut2[3])) == 0x00000000);

    // Versus float
    vfp_t uut3(andnot(uut0, bit_cast<unsigned int, float>(0xa5a53c3c)));

    // Checks
    BOOST_CHECK((bit_cast<float, unsigned int>(uut3[0])) == 0x00000000);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut3[1])) == 0xa5a53c3c);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut3[2])) == 0x00001818);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut3[3])) == 0x81810000);

    // Versus float reverse
    vfp_t uut4(andnot(bit_cast<unsigned int, float>(0xa5a53c3c), uut0));

    // Checks
    BOOST_CHECK((bit_cast<float, unsigned int>(uut4[0])) == 0x5a5ac3c3);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut4[1])) == 0x00000000);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut4[2])) == 0x00008181);
    BOOST_CHECK((bit_cast<float, unsigned int>(uut4[3])) == 0x18180000);
}

BOOST_AUTO_TEST_CASE( mov_p_test )
{
    // Predicated move
    vfp_t uut0(-100.0, -0.5, 10.0,  100.0);
    vfp_t uut1(   5.0, -1.5,  1.0, -100.0);
    vfp_t uut2(bit_cast<unsigned int, float>(0xffffffff), bit_cast<unsigned int, float>(0x00000000), bit_cast<unsigned int, float>(0x00000000), bit_cast<unsigned int, float>(0xffffffff));

    vfp_t uut3(mov_p(uut2, uut0, uut1));

    // Checks
    BOOST_CHECK_CLOSE(uut3[0], -100.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut3[1],   -1.5, result_tolerance);
    BOOST_CHECK_CLOSE(uut3[2],    1.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut3[3],  100.0, result_tolerance);
}

BOOST_AUTO_TEST_CASE( shuffle_test )
{
    // Shuffle
    vfp_t uut0(-100.0, -0.5, 10.0,  100.0);
    vfp_t uut1(   5.0, -1.5,  1.0, -100.0);

    vfp_t uut3(shuffle<0, 2, 3, 1>(uut0, uut1));

    // Checks
    BOOST_CHECK_CLOSE(uut3[0],  -0.5, result_tolerance);
    BOOST_CHECK_CLOSE(uut3[1], 100.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut3[2],   1.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut3[3],   5.0, result_tolerance);

    // Shuffle again
    vfp_t uut4(shuffle<3, 3, 0, 0>(uut0, uut1));

    // Checks
    BOOST_CHECK_CLOSE(uut4[0], -100.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut4[1], -100.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut4[2], -100.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut4[3], -100.0, result_tolerance);

    // Last one, just for good measure
    vfp_t uut5(shuffle<1, 3, 2, 0>(uut0, uut1));

    // Checks
    BOOST_CHECK_CLOSE(uut5[0], -100.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut5[1],   10.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut5[2], -100.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut5[3],   -1.5, result_tolerance);
}

BOOST_AUTO_TEST_CASE( transpose_test )
{
    // Transpose
    vfp_t uut0(-100.0, -0.5, 10.0,  100.0);
    vfp_t uut1(   5.0, -0.5,  1.0, -100.0);
    vfp_t uut2(  50.0, -0.5, 11.0,  -10.0);
    vfp_t uut3( -50.0, -0.5,  9.0,   10.0);

    transpose(uut0, uut1, uut2, uut3);

    // Checks
    BOOST_CHECK_CLOSE(uut0[0], -100.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut0[1],    5.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut0[2],   50.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut0[3],  -50.0, result_tolerance);

    BOOST_CHECK_CLOSE(uut1[0], -0.5, result_tolerance);
    BOOST_CHECK_CLOSE(uut1[1], -0.5, result_tolerance);
    BOOST_CHECK_CLOSE(uut1[2], -0.5, result_tolerance);
    BOOST_CHECK_CLOSE(uut1[3], -0.5, result_tolerance);

    BOOST_CHECK_CLOSE(uut2[0], 10.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut2[1],  1.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut2[2], 11.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut2[3],  9.0, result_tolerance);

    BOOST_CHECK_CLOSE(uut3[0],  100.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut3[1], -100.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut3[2],  -10.0, result_tolerance);
    BOOST_CHECK_CLOSE(uut3[3],   10.0, result_tolerance);
}

BOOST_AUTO_TEST_CASE( min_element_in_vector_test )
{
    float min = -1.0f;
    const float d0[] = { 0.0f, 1.0f, 2.0f, 3.0f };
    BOOST_CHECK(min_element(d0, &min, 4) == 0);
    BOOST_CHECK(min == 0.0f);

    min = -1.0f;
    const float d1[] = { 5.0f, 1.0f, 2.0f, 3.0f };
    BOOST_CHECK(min_element(d1, &min, 4) == 1);
    BOOST_CHECK(min == 1.0f);

    min = -1.0f;
    const float d2[] = { 5.0f, 6.0f, 2.0f, 3.0f };
    BOOST_CHECK(min_element(d2, &min, 4) == 2);
    BOOST_CHECK(min == 2.0f);

    min = -1.0f;
    const float d3[] = { 5.0f, 6.0f, 7.0f, 3.0f };
    BOOST_CHECK(min_element(d3, &min, 4) == 3);
    BOOST_CHECK(min == 3.0f);
}

BOOST_AUTO_TEST_CASE( min_element_test )
{
    float min = -1.0f;
    const float d0[] = { 7.0f, 8.0f, 9.0f, 10.0f, 4.0f, 5.0f, 6.0f, 7.0f, 0.0f, 1.0f, 2.0f, 3.0f };
    BOOST_CHECK(min_element(d0, &min, 12) == 8);
    BOOST_CHECK(min == 0.0f);
}

BOOST_AUTO_TEST_CASE( min_element_duplicate_test )
{
    float min = -1.0f;
    const float d0[] = { 7.0f, 8.0f, 0.0f, 10.0f, 4.0f, 5.0f, 6.0f, 7.0f, 0.0f, 1.0f, 2.0f, 3.0f };
    BOOST_CHECK(min_element(d0, &min, 12) == 8);
    BOOST_CHECK(min == 0.0f);

    min = -1.0f;
    const float d1[] = { 7.0f, 0.0f, 9.0f, 10.0f, 4.0f, 5.0f, 6.0f, 7.0f, 3.0f, 1.0f, 0.0f, 3.0f };
    BOOST_CHECK(min_element(d1, &min, 12) == 1);
    BOOST_CHECK(min == 0.0f);
}

BOOST_AUTO_TEST_CASE( min_element_non_vector_width_test )
{
    float min = -1.0f;
    const float d0[] = { 7.0f, 8.0f, 2.0f, 10.0f, 4.0f, 5.0f, 6.0f, 7.0f, 1.0f, 1.0f, 2.0f, 3.0f, 0.0f };
    BOOST_CHECK(min_element(d0, &min, 13) == 12);
    BOOST_CHECK(min == 0.0f);

    min = -1.0f;
    const float d1[] = { 7.0f, 5.0f, 9.0f, 10.0f, 4.0f, 5.0f, 6.0f, 7.0f, 3.0f, 1.0f, 5.0f, 3.0f, 1.0f, 2.0f, 0.0f };
    BOOST_CHECK(min_element(d1, &min, 15) == 14);
    BOOST_CHECK(min == 0.0f);

    min = -1.0f;
    const float d2[] = { 7.0f, 5.0f, 9.0f, 10.0f, 4.0f, 5.0f, 6.0f, 7.0f, 3.0f, 1.0f, 5.0f, 3.0f, 1.0f, 0.0f, 2.0f };
    BOOST_CHECK(min_element(d2, &min, 15) == 13);
    BOOST_CHECK(min == 0.0f);

    min = -1.0f;
    const float d3[] = { 7.0f, 5.0f, 9.0f, 0.0f, 4.0f, 5.0f, 6.0f, 7.0f, 3.0f, 1.0f, 5.0f, 3.0f, 1.0f, 10.0f, 2.0f };
    BOOST_CHECK(min_element(d3, &min, 15) == 3);
    BOOST_CHECK(min == 0.0f);

    min = -1.0f;
    const float d4[] = { 7.0f, 5.0f, 9.0f, 0.0f, 4.0f, 5.0f, 6.0f, 7.0f, 3.0f, 1.0f, 5.0f, 3.0f, 1.0f, 0.0f, 2.0f };
    BOOST_CHECK(min_element(d4, &min, 15) == 3);
    BOOST_CHECK(min == 0.0f);
}

BOOST_AUTO_TEST_CASE( min_element_unaligned_in_vector_test )
{
    float min = -1.0f;
    const float d0[] = { 0.0f, 1.0f, 2.0f, 3.0f };
    BOOST_CHECK(min_element(d0, &min, 0, 4) == 0);
    BOOST_CHECK(min == 0.0f);

    min = -1.0f;
    const float d1[] = { 5.0f, 1.0f, 2.0f, 3.0f };
    BOOST_CHECK(min_element(d1, &min, 0, 4) == 1);
    BOOST_CHECK(min == 1.0f);

    min = -1.0f;
    const float d2[] = { 5.0f, 6.0f, 2.0f, 3.0f };
    BOOST_CHECK(min_element(d2, &min, 0, 4) == 2);
    BOOST_CHECK(min == 2.0f);

    min = -1.0f;
    const float d3[] = { 5.0f, 6.0f, 7.0f, 3.0f };
    BOOST_CHECK(min_element(d3, &min, 0, 4) == 3);
    BOOST_CHECK(min == 3.0f);
}

BOOST_AUTO_TEST_CASE( min_element_unaligned_no_vector_test )
{
    float min = -1.0f;
    const float d0[] = { 0.0f, 1.0f, 2.0f, 3.0f };
    BOOST_CHECK(min_element(d0, &min, 0, 4) == 0);
    BOOST_CHECK(min == 0.0f);

    min = -1.0f;
    const float d1[] = { 5.0f, 1.0f, 2.0f, 3.0f };
    BOOST_CHECK(min_element(d1, &min, 1, 4) == 1);
    BOOST_CHECK(min == 1.0f);

    min = -1.0f;
    const float d2[] = { 5.0f, 6.0f, 2.0f, 3.0f };
    BOOST_CHECK(min_element(d2, &min, 2, 4) == 2);
    BOOST_CHECK(min == 2.0f);

    min = -1.0f;
    const float d3[] = { 5.0f, 6.0f, 7.0f, 3.0f };
    BOOST_CHECK(min_element(d3, &min, 3, 4) == 3);
    BOOST_CHECK(min == 3.0f);
}

BOOST_AUTO_TEST_CASE( min_element_unaligned_in_align_test )
{
    float min = -1.0f;
    const float d0[] = { 0.0f, 2.0f, 1.0f, 3.0f };
    BOOST_CHECK(min_element(d0, &min, 1, 4) == 2);
    BOOST_CHECK(min == 1.0f);

    min = -1.0f;
    const float d1[] = { 5.0f, 1.0f, 2.0f, 3.0f };
    BOOST_CHECK(min_element(d1, &min, 2, 4) == 2);
    BOOST_CHECK(min == 2.0f);

    min = -1.0f;
    const float d2[] = { 5.0f, 6.0f, 2.0f, 3.0f };
    BOOST_CHECK(min_element(d2, &min, 3, 4) == 3);
    BOOST_CHECK(min == 3.0f);
}

BOOST_AUTO_TEST_CASE( min_element_unaligned_test )
{
    float min = -1.0f;
    const float d0[] = { 7.0f, 8.0f, 9.0f, 10.0f, 4.0f, 5.0f, 6.0f, 7.0f, 0.0f, 1.0f, 2.0f, 3.0f };
    BOOST_CHECK(min_element(d0, &min, 0, 12) == 8);
    BOOST_CHECK(min == 0.0f);

    min = -1.0f;
    const float d1[] = { 7.0f, 8.0f, 9.0f, 10.0f, 4.0f, 5.0f, 6.0f, 7.0f, 0.0f, 1.0f, 2.0f, 3.0f };
    BOOST_CHECK(min_element(d1, &min, 1, 12) == 8);
    BOOST_CHECK(min == 0.0f);

    min = -1.0f;
    const float d2[] = { 7.0f, 8.0f, 9.0f, 10.0f, 4.0f, 5.0f, 6.0f, 7.0f, 0.0f, 1.0f, 2.0f, 3.0f };
    BOOST_CHECK(min_element(d2, &min, 2, 12) == 8);
    BOOST_CHECK(min == 0.0f);

    min = -1.0f;
    const float d3[] = { 7.0f, 8.0f, 9.0f, 10.0f, 4.0f, 5.0f, 6.0f, 7.0f, 0.0f, 1.0f, 2.0f, 3.0f };
    BOOST_CHECK(min_element(d3, &min, 3, 12) == 8);
    BOOST_CHECK(min == 0.0f);

    min = -1.0f;
    const float d4[] = { 7.0f, 8.0f, 9.0f, 10.0f, 4.0f, 5.0f, 6.0f, 7.0f, 0.0f, 1.0f, 2.0f, 3.0f };
    BOOST_CHECK(min_element(d4, &min, 4, 12) == 8);
    BOOST_CHECK(min == 0.0f);
}

BOOST_AUTO_TEST_CASE( min_element_unaligned_duplicate_test )
{
    float min = -1.0f;
    const float d0[] = { 7.0f, 8.0f, 0.0f, 10.0f, 4.0f, 5.0f, 6.0f, 7.0f, 0.0f, 1.0f, 2.0f, 3.0f };
    BOOST_CHECK(min_element(d0, &min, 2, 12) == 2);
    BOOST_CHECK(min == 0.0f);

    min = -1.0f;
    const float d1[] = { 7.0f, 0.0f, 9.0f, 10.0f, 4.0f, 5.0f, 6.0f, 7.0f, 3.0f, 1.0f, 0.0f, 3.0f };
    BOOST_CHECK(min_element(d1, &min, 1, 12) == 1);
    BOOST_CHECK(min == 0.0f);
}

BOOST_AUTO_TEST_CASE( min_element_unaligned_non_vector_width_test )
{
    float min = -1.0f;
    const float d0[] = { 7.0f, 8.0f, 2.0f, 10.0f, 4.0f, 5.0f, 6.0f, 7.0f, 1.0f, 1.0f, 2.0f, 3.0f, 0.0f };
    BOOST_CHECK(min_element(d0, &min, 0, 13) == 12);
    BOOST_CHECK(min == 0.0f);

    min = -1.0f;
    const float d1[] = { 7.0f, 5.0f, 9.0f, 10.0f, 4.0f, 5.0f, 6.0f, 7.0f, 3.0f, 1.0f, 5.0f, 3.0f, 1.0f, 2.0f, 0.0f };
    BOOST_CHECK(min_element(d1, &min, 1, 15) == 14);
    BOOST_CHECK(min == 0.0f);

    min = -1.0f;
    const float d2[] = { 7.0f, 5.0f, 9.0f, 10.0f, 4.0f, 5.0f, 6.0f, 7.0f, 3.0f, 1.0f, 5.0f, 3.0f, 1.0f, 0.0f, 2.0f };
    BOOST_CHECK(min_element(d2, &min, 2, 15) == 13);
    BOOST_CHECK(min == 0.0f);

    min = -1.0f;
    const float d3[] = { 7.0f, 5.0f, 9.0f, 0.0f, 4.0f, 5.0f, 6.0f, 7.0f, 3.0f, 1.0f, 5.0f, 3.0f, 1.0f, 10.0f, 2.0f };
    BOOST_CHECK(min_element(d3, &min, 3, 15) == 3);
    BOOST_CHECK(min == 0.0f);

    min = -1.0f;
    const float d4[] = { 7.0f, 5.0f, 9.0f, 0.0f, 4.0f, 5.0f, 6.0f, 7.0f, 3.0f, 1.0f, 5.0f, 3.0f, 1.0f, 0.0f, 2.0f };
    BOOST_CHECK(min_element(d4, &min, 4, 15) == 13);
    BOOST_CHECK(min == 0.0f);
}

#ifndef VALGRIND_TESTS
BOOST_AUTO_TEST_CASE( min_element_performance_test )
{
    /* Get an array of the right size */
    std::vector<float> data(16777216);

    /* Fill it */
    std::default_random_engine gen;
    std::uniform_real_distribution<float> dist(-100.0f, 100.0f);
    std::generate(data.begin(), data.end(), [&dist, &gen]()
        {
            return dist(gen);
        });

    /* Run scalar code */
    auto s0(std::chrono::system_clock::now());
    float min_v = std::numeric_limits<float>::infinity();
    int min_i = -1;
    for (int i = 0; i < static_cast<int>(data.size()); ++i)
    {
        if (data[i] < min_v)
        {
            min_v = data[i];
            min_i = i;
        }
    }
    auto s1(std::chrono::system_clock::now());
    BOOST_CHECK(min_i > -1);
    BOOST_CHECK(min_v < std::numeric_limits<float>::infinity());

    BOOST_LOG_TRIVIAL(fatal) << "PERF 0 - Test: min_element_performance_test scalar";
    BOOST_LOG_TRIVIAL(fatal) << "PERF 1 - Runtime us: " << std::chrono::duration_cast<std::chrono::microseconds>(s1 - s0).count();

    /* Run vector code */
    float min = std::numeric_limits<float>::infinity();
    auto v0(std::chrono::system_clock::now());
    BOOST_CHECK(min_element(data.data(), &min, data.size()) == min_i);
    auto v1(std::chrono::system_clock::now());
    BOOST_CHECK(min == min_v);

    BOOST_LOG_TRIVIAL(fatal) << "PERF 0 - Test: min_element_performance_test vector";
    BOOST_LOG_TRIVIAL(fatal) << "PERF 1 - Runtime us: " << std::chrono::duration_cast<std::chrono::microseconds>(v1 - v0).count();
}
#endif /* #ifndef VALGRIND_TESTS */

BOOST_AUTO_TEST_SUITE_END()
}; /* namespace test */
}; /* namespace raptor_raytracer */
