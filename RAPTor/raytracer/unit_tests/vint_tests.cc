#ifdef STAND_ALONE
#define BOOST_TEST_MODULE vint test

/* Common headers */
#include "logging.h"

/* Initialise logger */
const raptor_physics::init_logger init_logger;
#endif /* #ifdef STAND_ALONE */

/* Standard headers */
#include <chrono>

/* Boost headers */
#include "boost/test/unit_test.hpp"

/* Ray tracer headers */
#include "simd.h"


namespace raptor_raytracer
{
namespace test
{
const int result_tolerance = 0.00001;

BOOST_AUTO_TEST_SUITE( vint_tests );


/* CTOR tests */
BOOST_AUTO_TEST_CASE( vint_ctor_from_ints_test )
{
    /* Contruct from 4 ints */
    vint_t uut0(-100, 5, 0, 100);

    /* Checks */
    BOOST_CHECK(uut0[0] == -100);
    BOOST_CHECK(uut0[1] ==    5);
    BOOST_CHECK(uut0[2] ==    0);
    BOOST_CHECK(uut0[3] ==  100);

    /* Contruct from 3 ints */
    vint_t uut1(5, -5, 0);

    /* Checks */
    BOOST_CHECK(uut1[0] ==  5);
    BOOST_CHECK(uut1[1] == -5);
    BOOST_CHECK(uut1[2] ==  0);
    BOOST_CHECK(uut1[3] ==  0);

    /* Contruct from 2 ints */
    vint_t uut2(5, -5);

    /* Checks */
    BOOST_CHECK(uut2[0] ==  5);
    BOOST_CHECK(uut2[1] == -5);
    BOOST_CHECK(uut2[2] == -5);
    BOOST_CHECK(uut2[3] == -5);

    /* Contruct from 1 ints */
    vint_t uut3(5);

    /* Checks */
    BOOST_CHECK(uut3[0] == 5);
    BOOST_CHECK(uut3[1] == 5);
    BOOST_CHECK(uut3[2] == 5);
    BOOST_CHECK(uut3[3] == 5);
    
    /* Contruct from array of ints */
    int a[] = { 100, 5, 0, -100 };
    vint_t uut4(a);

    /* Checks */
    BOOST_CHECK(uut4[0] ==  100);
    BOOST_CHECK(uut4[1] ==    5);
    BOOST_CHECK(uut4[2] ==    0);
    BOOST_CHECK(uut4[3] == -100);
}

BOOST_AUTO_TEST_CASE( copy_ctor_test )
{
    vint_t uut0(-100, -5, 0, 100);
    vint_t uut1(uut0);

    /* Checks */
    BOOST_CHECK(uut1[0] == -100);
    BOOST_CHECK(uut1[1] ==   -5);
    BOOST_CHECK(uut1[2] ==    0);
    BOOST_CHECK(uut1[3] ==  100);
}

BOOST_AUTO_TEST_CASE( float_copy_ctor_test )
{
    vfp_t uut0(-100.5, -0.51, 0.51, 100.9);
    vint_t uut1(uut0);

    /* Checks */
    /* Note -- It rounds nearest with 0.5 getting round down */
    BOOST_CHECK(uut1[0] == -100);
    BOOST_CHECK(uut1[1] ==    0);
    BOOST_CHECK(uut1[2] ==    0);
    BOOST_CHECK(uut1[3] ==  100);
}

/* Element access */
BOOST_AUTO_TEST_CASE( store_test )
{
    int to[4];
    const vint_t uut0(-100, -5, 0, 100);
    uut0.store(to);

    /* Checks */
    BOOST_CHECK(to[0] == -100);
    BOOST_CHECK(to[1] ==   -5);
    BOOST_CHECK(to[2] ==    0);
    BOOST_CHECK(to[3] ==  100);
}

BOOST_AUTO_TEST_CASE( extract_test )
{
    const vint_t uut0(-100, -5, 0, 100);

    /* Checks */
    BOOST_CHECK(uut0.extract(0) == -100);
    BOOST_CHECK(uut0.extract(1) ==   -5);
    BOOST_CHECK(uut0.extract(2) ==    0);
    BOOST_CHECK(uut0.extract(3) ==  100);
}


/* Operator tests */
/* Unary operator tests */
BOOST_AUTO_TEST_CASE( operator_equals_test )
{
    vint_t uut0(-100, -5, 0, 100);
    vint_t uut1 = uut0;

    /* Checks */
    BOOST_CHECK(uut0[0] == -100);
    BOOST_CHECK(uut0[1] ==   -5);
    BOOST_CHECK(uut0[2] ==    0);
    BOOST_CHECK(uut0[3] ==  100);
}

BOOST_AUTO_TEST_CASE( operator_plus_equals_test )
{
    /* Versus vint_t */
    vint_t uut0(-100, -5, 10,  100);
    vint_t uut1(   5, -5,  1, -100);
    uut0 += uut1;

    /* Checks */
    BOOST_CHECK(uut0[0] == -95);
    BOOST_CHECK(uut0[1] == -10);
    BOOST_CHECK(uut0[2] ==  11);
    BOOST_CHECK(uut0[3] ==   0);

    /* Versus int */
    vint_t uut2(-100, -5, 10,  100);
    uut2 += 5;

    /* Checks */
    BOOST_CHECK(uut2[0] == -95);
    BOOST_CHECK(uut2[1] ==   0);
    BOOST_CHECK(uut2[2] ==  15);
    BOOST_CHECK(uut2[3] == 105);
}

BOOST_AUTO_TEST_CASE( operator_left_shift_equals_test )
{
    /* Versus vint_t */
    vint_t uut0(-100, -5, 10,  100);
    vint_t uut1(   5, -5,  1, -100);
    uut0 <<= uut1;

    /* Checks */
    /* Note -- This just shifts by the lower lane */
    BOOST_CHECK(uut0[0] == -3200);
    BOOST_CHECK(uut0[1] ==  -160);
    BOOST_CHECK(uut0[2] ==   320);
    BOOST_CHECK(uut0[3] ==  3200);

    /* Versus int */
    vint_t uut2(-100, -5, 10,  100);
    uut2 <<= 4;

    /* Checks */
    BOOST_CHECK(uut2[0] == -1600);
    BOOST_CHECK(uut2[1] ==   -80);
    BOOST_CHECK(uut2[2] ==   160);
    BOOST_CHECK(uut2[3] ==  1600);
}

BOOST_AUTO_TEST_CASE( operator_right_shift_equals_test )
{
    /* Versus vint_t */
    vint_t uut0(-100, -5, 10,  100);
    vint_t uut1(   5, -5,  1, -100);
    uut0 >>= uut1;

    /* Checks */
    /* Note -- This just shifts by the lower lane */
    BOOST_CHECK(uut0[0] == -4);
    BOOST_CHECK(uut0[1] == -1);
    BOOST_CHECK(uut0[2] ==  0);
    BOOST_CHECK(uut0[3] ==  3);

    /* Versus int */
    /* Note - Negative shifts are interpreted as positive shift by int max*/
    vint_t uut2(-100, -5, 10,  100);
    uut2 >>= -4;

    /* Checks */
    BOOST_CHECK(uut2[0] == -1);
    BOOST_CHECK(uut2[1] == -1);
    BOOST_CHECK(uut2[2] ==  0);
    BOOST_CHECK(uut2[3] ==  0);
}

BOOST_AUTO_TEST_CASE( operator_minus_equals_test )
{
    /* Versus vint_t */
    vint_t uut0(-100, -5, 10,  100);
    vint_t uut1(   5, -5,  1, -100);
    uut0 -= uut1;

    /* Checks */
    BOOST_CHECK(uut0[0] == -105);
    BOOST_CHECK(uut0[1] ==    0);
    BOOST_CHECK(uut0[2] ==    9);
    BOOST_CHECK(uut0[3] ==  200);

    /* Versus int */
    vint_t uut2(-100, -5, 10,  100);
    uut2 -= 5.0;

    /* Checks */
    BOOST_CHECK(uut2[0] == -105);
    BOOST_CHECK(uut2[1] ==  -10);
    BOOST_CHECK(uut2[2] ==    5);
    BOOST_CHECK(uut2[3] ==   95);
}

BOOST_AUTO_TEST_CASE( operator_negate_test )
{
    vint_t uut0(-100, -5, 10, 100);
    vint_t uut1(-uut0);

    /* Checks */
    BOOST_CHECK(uut1[0] ==  100);
    BOOST_CHECK(uut1[1] ==    5);
    BOOST_CHECK(uut1[2] ==  -10);
    BOOST_CHECK(uut1[3] == -100);
}

BOOST_AUTO_TEST_CASE( operator_and_equals_test )
{
    /* Versus vint_t */
    vint_t uut0(0xffffffff, 0x00000000, 0xa5a5a5a5, 0x3c3c3c3c);
    vint_t uut1(0xa5a53c3c, 0xffff0000, 0xffffa5a5, 0x00003c3c);
    uut0 &= uut1;

    /* Checks */
    BOOST_CHECK(uut0[0] == static_cast<int>(0xa5a53c3c));
    BOOST_CHECK(uut0[1] == static_cast<int>(0x00000000));
    BOOST_CHECK(uut0[2] == static_cast<int>(0xa5a5a5a5));
    BOOST_CHECK(uut0[3] == static_cast<int>(0x00003c3c));

    /* Versus int */
    vint_t uut2(0xffffffff, 0x00000000, 0xa5a5a5a5, 0x3c3c3c3c);
    uut2 &= 0xa5a5a5a5;

    /* Checks */
    BOOST_CHECK(uut2[0] == static_cast<int>(0xa5a5a5a5));
    BOOST_CHECK(uut2[1] == static_cast<int>(0x00000000));
    BOOST_CHECK(uut2[2] == static_cast<int>(0xa5a5a5a5));
    BOOST_CHECK(uut2[3] == static_cast<int>(0x24242424));
}

BOOST_AUTO_TEST_CASE( operator_or_equals_test )
{
    /* Versus vint_t */
    vint_t uut0(0xffffffff, 0x00000000, 0xa5a5a5a5, 0x3c3c3c3c);
    vint_t uut1(0xa5a53c3c, 0xffff0000, 0xffffa5a5, 0x00003c3c);
    uut0 |= uut1;

    /* Checks */
    BOOST_CHECK(uut0[0] == static_cast<int>(0xffffffff));
    BOOST_CHECK(uut0[1] == static_cast<int>(0xffff0000));
    BOOST_CHECK(uut0[2] == static_cast<int>(0xffffa5a5));
    BOOST_CHECK(uut0[3] == static_cast<int>(0x3c3c3c3c));

    /* Versus int */
    vint_t uut2(0xffffffff, 0x00000000, 0xa5a5a5a5, 0x3c3c3c3c);
    uut2 |= 0xa5a5a5a5;

    /* Checks */
    BOOST_CHECK(uut2[0] == static_cast<int>(0xffffffff));
    BOOST_CHECK(uut2[1] == static_cast<int>(0xa5a5a5a5));
    BOOST_CHECK(uut2[2] == static_cast<int>(0xa5a5a5a5));
    BOOST_CHECK(uut2[3] == static_cast<int>(0xbdbdbdbd));
}

BOOST_AUTO_TEST_CASE( operator_xor_equals_test )
{
    /* Versus vint_t */
    vint_t uut0(0xffffffff, 0x00000000, 0xa5a5a5a5, 0x3c3c3c3c);
    vint_t uut1(0xa5a53c3c, 0xffff0000, 0xffffa5a5, 0x00003c3c);
    uut0 ^= uut1;

    /* Checks */
    BOOST_CHECK(uut0[0] == static_cast<int>(0x5a5ac3c3));
    BOOST_CHECK(uut0[1] == static_cast<int>(0xffff0000));
    BOOST_CHECK(uut0[2] == static_cast<int>(0x5a5a0000));
    BOOST_CHECK(uut0[3] == static_cast<int>(0x3c3c0000));

    /* Versus int */
    vint_t uut2(0xffffffff, 0x00000000, 0xa5a5a5a5, 0x3c3c3c3c);
    uut2 ^= 0xa5a5a5a5;

    /* Checks */
    BOOST_CHECK(uut2[0] == static_cast<int>(0x5a5a5a5a));
    BOOST_CHECK(uut2[1] == static_cast<int>(0xa5a5a5a5));
    BOOST_CHECK(uut2[2] == static_cast<int>(0x00000000));
    BOOST_CHECK(uut2[3] == static_cast<int>(0x99999999));
}


/* Friendly operator tests */
BOOST_AUTO_TEST_CASE( operator_plus_test )
{
    /* Versus vint_t */
    vint_t uut0(-100, -5, 10,  100);
    vint_t uut1(   5, -5,  1, -100);
    vint_t uut2(uut0 + uut1);

    /* Checks */
    BOOST_CHECK(uut2[0] == -95);
    BOOST_CHECK(uut2[1] == -10);
    BOOST_CHECK(uut2[2] ==  11);
    BOOST_CHECK(uut2[3] ==   0);

    /* Versus int */
    vint_t uut3(uut0 + 5);

    /* Checks */
    BOOST_CHECK(uut3[0] == -95);
    BOOST_CHECK(uut3[1] ==   0);
    BOOST_CHECK(uut3[2] ==  15);
    BOOST_CHECK(uut3[3] == 105);

    /* Versus int reverse */
    vint_t uut4(5 + uut0);

    /* Checks */
    BOOST_CHECK(uut4[0] == -95);
    BOOST_CHECK(uut4[1] ==   0);
    BOOST_CHECK(uut4[2] ==  15);
    BOOST_CHECK(uut4[3] == 105);
}

BOOST_AUTO_TEST_CASE( operator_minus_test )
{
    /* Versus vint_t */
    vint_t uut0(-100, -5, 10,  100);
    vint_t uut1(   5, -5,  1, -100);
    vint_t uut2(uut0 - uut1);

    /* Checks */
    BOOST_CHECK(uut2[0] == -105);
    BOOST_CHECK(uut2[1] ==    0);
    BOOST_CHECK(uut2[2] ==    9);
    BOOST_CHECK(uut2[3] ==  200);

    /* Versus int */
    vint_t uut3(uut0 - 5);

    /* Checks */
    BOOST_CHECK(uut3[0] == -105);
    BOOST_CHECK(uut3[1] ==  -10);
    BOOST_CHECK(uut3[2] ==    5);
    BOOST_CHECK(uut3[3] ==   95);

    /* Versus int reverse */
    vint_t uut4(5 - uut0);

    /* Checks */
    BOOST_CHECK(uut4[0] ==  105);
    BOOST_CHECK(uut4[1] ==   10);
    BOOST_CHECK(uut4[2] ==   -5);
    BOOST_CHECK(uut4[3] ==  -95);
}

BOOST_AUTO_TEST_CASE( operator_left_shift_test )
{
    /* Versus vint_t */
    vint_t uut0(-100, -5, 10,  100);
    vint_t uut1(   5, -5,  1, -100);
    vint_t uut2(uut0 << uut1);

    /* Checks */
    BOOST_CHECK(uut2[0] == -3200);
    BOOST_CHECK(uut2[1] ==  -160);
    BOOST_CHECK(uut2[2] ==   320);
    BOOST_CHECK(uut2[3] ==  3200);

    /* Versus int */
    vint_t uut3(uut0 << 4);

    /* Checks */
    BOOST_CHECK(uut3[0] == -1600);
    BOOST_CHECK(uut3[1] ==   -80);
    BOOST_CHECK(uut3[2] ==   160);
    BOOST_CHECK(uut3[3] ==  1600);

    /* Versus int reverse */
    vint_t uut4(3, -5, 10,  100);
    vint_t uut5(4 << uut4);

    /* Checks */
    BOOST_CHECK(uut5[0] == 32);
    BOOST_CHECK(uut5[1] == 32);
    BOOST_CHECK(uut5[2] == 32);
    BOOST_CHECK(uut5[3] == 32);
}

BOOST_AUTO_TEST_CASE( operator_right_shift_test )
{
    /* Versus vint_t */
    vint_t uut0(-100, -5, 10,  100);
    vint_t uut1(   5, -5,  1, -100);
    vint_t uut2(uut0 >> uut1);

    /* Checks */
    /* Note -- This just shifts by the lower lane */
    BOOST_CHECK(uut2[0] == -4);
    BOOST_CHECK(uut2[1] == -1);
    BOOST_CHECK(uut2[2] ==  0);
    BOOST_CHECK(uut2[3] ==  3);

    /* Versus int */
    vint_t uut3(uut0 >> 4);

    /* Checks */
    BOOST_CHECK(uut3[0] == -7);
    BOOST_CHECK(uut3[1] == -1);
    BOOST_CHECK(uut3[2] ==  0);
    BOOST_CHECK(uut3[3] ==  6);

    /* Versus int reverse */
    vint_t uut4(3, -5, 10,  100);
    vint_t uut5(79 >> uut4);

    /* Checks */
    BOOST_CHECK(uut5[0] == 9);
    BOOST_CHECK(uut5[1] == 9);
    BOOST_CHECK(uut5[2] == 9);
    BOOST_CHECK(uut5[3] == 9);
}

BOOST_AUTO_TEST_CASE( operator_and_test )
{
    /* Versus vint_t */
    vint_t uut0(0xffffffff, 0x00000000, 0xa5a5a5a5, 0x3c3c3c3c);
    vint_t uut1(0xa5a53c3c, 0xffff0000, 0xffffa5a5, 0x00003c3c);
    vint_t uut2(uut0 & uut1);

    /* Checks */
    BOOST_CHECK(uut2[0] == static_cast<int>(0xa5a53c3c));
    BOOST_CHECK(uut2[1] == static_cast<int>(0x00000000));
    BOOST_CHECK(uut2[2] == static_cast<int>(0xa5a5a5a5));
    BOOST_CHECK(uut2[3] == static_cast<int>(0x00003c3c));

    /* Versus int */
    vint_t uut3(uut0 & 0xa5a53c3c);

    /* Checks */
    BOOST_CHECK(uut3[0] == static_cast<int>(0xa5a53c3c));
    BOOST_CHECK(uut3[1] == static_cast<int>(0x00000000));
    BOOST_CHECK(uut3[2] == static_cast<int>(0xa5a52424));
    BOOST_CHECK(uut3[3] == static_cast<int>(0x24243c3c));

    /* Versus int reverse */
    vint_t uut4(0xa5a53c3c & uut0);

    /* Checks */
    BOOST_CHECK(uut4[0] == static_cast<int>(0xa5a53c3c));
    BOOST_CHECK(uut4[1] == static_cast<int>(0x00000000));
    BOOST_CHECK(uut4[2] == static_cast<int>(0xa5a52424));
    BOOST_CHECK(uut4[3] == static_cast<int>(0x24243c3c));
}

BOOST_AUTO_TEST_CASE( operator_or_test )
{
    /* Versus vint_t */
    vint_t uut0(0xffffffff, 0x00000000, 0xa5a5a5a5, 0x3c3c3c3c);
    vint_t uut1(0xa5a53c3c, 0xffff0000, 0xffffa5a5, 0x00003c3c);
    vint_t uut2(uut0 | uut1);

    /* Checks */
    BOOST_CHECK(uut2[0] == static_cast<int>(0xffffffff));
    BOOST_CHECK(uut2[1] == static_cast<int>(0xffff0000));
    BOOST_CHECK(uut2[2] == static_cast<int>(0xffffa5a5));
    BOOST_CHECK(uut2[3] == static_cast<int>(0x3c3c3c3c));

    /* Versus int */
    vint_t uut3(uut0 | 0xa5a53c3c);

    /* Checks */
    BOOST_CHECK(uut3[0] == static_cast<int>(0xffffffff));
    BOOST_CHECK(uut3[1] == static_cast<int>(0xa5a53c3c));
    BOOST_CHECK(uut3[2] == static_cast<int>(0xa5a5bdbd));
    BOOST_CHECK(uut3[3] == static_cast<int>(0xbdbd3c3c));

    /* Versus int reverse */
    vint_t uut4(0xa5a53c3c | uut0);

    /* Checks */
    BOOST_CHECK(uut4[0] == static_cast<int>(0xffffffff));
    BOOST_CHECK(uut4[1] == static_cast<int>(0xa5a53c3c));
    BOOST_CHECK(uut4[2] == static_cast<int>(0xa5a5bdbd));
    BOOST_CHECK(uut4[3] == static_cast<int>(0xbdbd3c3c));
}

BOOST_AUTO_TEST_CASE( operator_xor_test )
{
    /* Versus vint_t */
    vint_t uut0(0xffffffff, 0x00000000, 0xa5a5a5a5, 0x3c3c3c3c);
    vint_t uut1(0xa5a53c3c, 0xffff0000, 0xffffa5a5, 0x00003c3c);
    vint_t uut2(uut0 ^ uut1);

    /* Checks */
    BOOST_CHECK(uut2[0] == static_cast<int>(0x5a5ac3c3));
    BOOST_CHECK(uut2[1] == static_cast<int>(0xffff0000));
    BOOST_CHECK(uut2[2] == static_cast<int>(0x5a5a0000));
    BOOST_CHECK(uut2[3] == static_cast<int>(0x3c3c0000));

    /* Versus int */
    vint_t uut3(uut0 ^ 0xa5a53c3c);

    /* Checks */
    BOOST_CHECK(uut3[0] == static_cast<int>(0x5a5ac3c3));
    BOOST_CHECK(uut3[1] == static_cast<int>(0xa5a53c3c));
    BOOST_CHECK(uut3[2] == static_cast<int>(0x00009999));
    BOOST_CHECK(uut3[3] == static_cast<int>(0x99990000));

    /* Versus int reverse */
    vint_t uut4(0xa5a53c3c ^ uut0);

    /* Checks */
    BOOST_CHECK(uut4[0] == static_cast<int>(0x5a5ac3c3));
    BOOST_CHECK(uut4[1] == static_cast<int>(0xa5a53c3c));
    BOOST_CHECK(uut4[2] == static_cast<int>(0x00009999));
    BOOST_CHECK(uut4[3] == static_cast<int>(0x99990000));
}


/* Comparison tests */
BOOST_AUTO_TEST_CASE( operator_equality_test )
{
    /* Versus vint_t */
    vint_t uut0(-100, -5, 10,  100);
    vint_t uut1(   5, -5,  1, -100);
    vint_t uut2(uut0 == uut1);

    /* Checks */
    BOOST_CHECK(uut2[0] == static_cast<int>(0x00000000));
    BOOST_CHECK(uut2[1] == static_cast<int>(0xffffffff));
    BOOST_CHECK(uut2[2] == static_cast<int>(0x00000000));
    BOOST_CHECK(uut2[3] == static_cast<int>(0x00000000));

    /* Versus int */
    vint_t uut3(uut0 == -100);

    /* Checks */
    BOOST_CHECK(uut3[0] == static_cast<int>(0xffffffff));
    BOOST_CHECK(uut3[1] == static_cast<int>(0x00000000));
    BOOST_CHECK(uut3[2] == static_cast<int>(0x00000000));
    BOOST_CHECK(uut3[3] == static_cast<int>(0x00000000));

    /* Versus int reverse */
    vint_t uut4(-100 == uut0);

    /* Checks */
    BOOST_CHECK(uut4[0] == static_cast<int>(0xffffffff));
    BOOST_CHECK(uut4[1] == static_cast<int>(0x00000000));
    BOOST_CHECK(uut4[2] == static_cast<int>(0x00000000));
    BOOST_CHECK(uut4[3] == static_cast<int>(0x00000000));
}

BOOST_AUTO_TEST_CASE( operator_greater_test )
{
    /* Versus vint_t */
    vint_t uut0(-100, -5, 10,  100);
    vint_t uut1(   5, -5,  1, -100);
    vint_t uut2(uut0 > uut1);

    /* Checks */
    BOOST_CHECK(uut2[0] == static_cast<int>(0x00000000));
    BOOST_CHECK(uut2[1] == static_cast<int>(0x00000000));
    BOOST_CHECK(uut2[2] == static_cast<int>(0xffffffff));
    BOOST_CHECK(uut2[3] == static_cast<int>(0xffffffff));

    /* Versus int */
    vint_t uut3(uut0 > 10);

    /* Checks */
    BOOST_CHECK(uut3[0] == static_cast<int>(0x00000000));
    BOOST_CHECK(uut3[1] == static_cast<int>(0x00000000));
    BOOST_CHECK(uut3[2] == static_cast<int>(0x00000000));
    BOOST_CHECK(uut3[3] == static_cast<int>(0xffffffff));

    /* Versus int reverse */
    vint_t uut4(10 > uut0);

    /* Checks */
    BOOST_CHECK(uut4[0] == static_cast<int>(0xffffffff));
    BOOST_CHECK(uut4[1] == static_cast<int>(0xffffffff));
    BOOST_CHECK(uut4[2] == static_cast<int>(0x00000000));
    BOOST_CHECK(uut4[3] == static_cast<int>(0x00000000));
}

BOOST_AUTO_TEST_CASE( operator_lesser_test )
{
    /* Versus vint_t */
    vint_t uut0(-100, -5, 10,  100);
    vint_t uut1(   5, -5,  1, -100);
    vint_t uut2(uut0 < uut1);

    /* Checks */
    BOOST_CHECK(uut2[0] == static_cast<int>(0xffffffff));
    BOOST_CHECK(uut2[1] == static_cast<int>(0x00000000));
    BOOST_CHECK(uut2[2] == static_cast<int>(0x00000000));
    BOOST_CHECK(uut2[3] == static_cast<int>(0x00000000));

    /* Versus int */
    vint_t uut3(uut0 < 10.0);

    /* Checks */
    BOOST_CHECK(uut3[0] == static_cast<int>(0xffffffff));
    BOOST_CHECK(uut3[1] == static_cast<int>(0xffffffff));
    BOOST_CHECK(uut3[2] == static_cast<int>(0x00000000));
    BOOST_CHECK(uut3[3] == static_cast<int>(0x00000000));

    /* Versus int reverse */
    vint_t uut4(10.0 < uut0);

    /* Checks */
    BOOST_CHECK(uut4[0] == static_cast<int>(0x00000000));
    BOOST_CHECK(uut4[1] == static_cast<int>(0x00000000));
    BOOST_CHECK(uut4[2] == static_cast<int>(0x00000000));
    BOOST_CHECK(uut4[3] == static_cast<int>(0xffffffff));
}


/* Friendly function tests */

/* Non standard friendly function tests */
BOOST_AUTO_TEST_CASE( move_mask_test )
{
    /* Check 0 */
    vint_t uut0(0x00000000, 0x00000000, 0x00000000, 0x00000000);
    BOOST_CHECK(move_mask(uut0) == 0x0);

    /* Check 1 */
    vint_t uut1_0(0x00000000, 0x00000000, 0x00000000, 0xffffffff);
    vint_t uut1_1(0x00000000, 0x00000000, 0xffffffff, 0x00000000);
    vint_t uut1_2(0x00000000, 0xffffffff, 0x00000000, 0x00000000);
    vint_t uut1_3(0xffffffff, 0x00000000, 0x00000000, 0x00000000);

    BOOST_CHECK(move_mask(uut1_0) == 0xf000);
    BOOST_CHECK(move_mask(uut1_1) == 0x0f00);
    BOOST_CHECK(move_mask(uut1_2) == 0x00f0);
    BOOST_CHECK(move_mask(uut1_3) == 0x000f);

    /* Check 2 */
    vint_t uut2_0(0x00000000, 0x00000000, 0xffffffff, 0xffffffff);
    vint_t uut2_1(0x00000000, 0xffffffff, 0x00000000, 0xffffffff);
    vint_t uut2_2(0xffffffff, 0x00000000, 0x00000000, 0xffffffff);
    vint_t uut2_3(0x00000000, 0xffffffff, 0xffffffff, 0x00000000);
    vint_t uut2_4(0xffffffff, 0x00000000, 0xffffffff, 0x00000000);
    vint_t uut2_5(0xffffffff, 0xffffffff, 0x00000000, 0x00000000);

    BOOST_CHECK(move_mask(uut2_0) == 0xff00);
    BOOST_CHECK(move_mask(uut2_1) == 0xf0f0);
    BOOST_CHECK(move_mask(uut2_2) == 0xf00f);
    BOOST_CHECK(move_mask(uut2_3) == 0x0ff0);
    BOOST_CHECK(move_mask(uut2_4) == 0x0f0f);
    BOOST_CHECK(move_mask(uut2_5) == 0x00ff);

    /* Check 3 */
    vint_t uut3_0(0xffffffff, 0xffffffff, 0xffffffff, 0x00000000);
    vint_t uut3_1(0xffffffff, 0xffffffff, 0x00000000, 0xffffffff);
    vint_t uut3_2(0xffffffff, 0x00000000, 0xffffffff, 0xffffffff);
    vint_t uut3_3(0x00000000, 0xffffffff, 0xffffffff, 0xffffffff);

    BOOST_CHECK(move_mask(uut3_0) == 0x0fff);
    BOOST_CHECK(move_mask(uut3_1) == 0xf0ff);
    BOOST_CHECK(move_mask(uut3_2) == 0xff0f);
    BOOST_CHECK(move_mask(uut3_3) == 0xfff0);

    /* Check 4 */
    vint_t uut4(0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff);
    BOOST_CHECK(move_mask(uut4) == 0xffff);
}

BOOST_AUTO_TEST_CASE( andnot_test )
{
    /* Versus vint_t */
    vint_t uut0(0xffffffff, 0x00000000, 0xa5a5a5a5, 0x3c3c3c3c);
    vint_t uut1(0xa5a53c3c, 0xffff0000, 0xffffa5a5, 0x00003c3c);
    vint_t uut2(andnot(uut0, uut1));

    /* Checks */
    BOOST_CHECK(uut2[0] == static_cast<int>(0x00000000));
    BOOST_CHECK(uut2[1] == static_cast<int>(0xffff0000));
    BOOST_CHECK(uut2[2] == static_cast<int>(0x5a5a0000));
    BOOST_CHECK(uut2[3] == static_cast<int>(0x00000000));

    /* Versus int */
    vint_t uut3(andnot(uut0, 0xa5a53c3c));

    /* Checks */
    BOOST_CHECK(uut3[0] == static_cast<int>(0x00000000));
    BOOST_CHECK(uut3[1] == static_cast<int>(0xa5a53c3c));
    BOOST_CHECK(uut3[2] == static_cast<int>(0x00001818));
    BOOST_CHECK(uut3[3] == static_cast<int>(0x81810000));

    /* Versus int reverse */
    vint_t uut4(andnot(0xa5a53c3c, uut0));

    /* Checks */
    BOOST_CHECK(uut4[0] == static_cast<int>(0x5a5ac3c3));
    BOOST_CHECK(uut4[1] == static_cast<int>(0x00000000));
    BOOST_CHECK(uut4[2] == static_cast<int>(0x00008181));
    BOOST_CHECK(uut4[3] == static_cast<int>(0x18180000));
}

BOOST_AUTO_TEST_CASE( mov_p_test )
{
    /* Predicated move */
    vint_t uut0(-100,  0, 10,  100);
    vint_t uut1(   5, -1,  1, -100);
    vint_t uut2(0xffffffff, 0x00000000, 0x00000000, 0xffffffff);

    vint_t uut3(mov_p(uut2, uut0, uut1));

    /* Checks */
    BOOST_CHECK(uut3[0] == -100);
    BOOST_CHECK(uut3[1] ==   -1);
    BOOST_CHECK(uut3[2] ==    1);
    BOOST_CHECK(uut3[3] ==  100);
}

BOOST_AUTO_TEST_CASE( shuffle_test )
{
    /* Shuffle */
    vint_t uut0(-100, -5, 10,  100);

    vint_t uut3(shuffle<0, 2, 3, 1>(uut0));

    /* Checks */
    BOOST_CHECK(uut3[0] ==   -5);
    BOOST_CHECK(uut3[1] ==  100);
    BOOST_CHECK(uut3[2] ==   10);
    BOOST_CHECK(uut3[3] == -100);

    /* Shuffle again */
    vint_t uut4(shuffle<3, 3, 0, 0>(uut0));

    /* Checks */
    BOOST_CHECK(uut4[0] == -100);
    BOOST_CHECK(uut4[1] == -100);
    BOOST_CHECK(uut4[2] ==  100);
    BOOST_CHECK(uut4[3] ==  100);

    /* Last one, just for good measure */
    vint_t uut5(shuffle<1, 3, 2, 0>(uut0));

    /* Checks */
    BOOST_CHECK(uut5[0] == -100);
    BOOST_CHECK(uut5[1] ==   10);
    BOOST_CHECK(uut5[2] ==  100);
    BOOST_CHECK(uut5[3] ==   -5);
}

BOOST_AUTO_TEST_CASE( transpose_test )
{
    /* Transpose */
    vint_t uut0(-100, -5, 10,  100);
    vint_t uut1(   5, -5,  1, -100);
    vint_t uut2(  50, -5, 11,  -10);
    vint_t uut3( -50, -5,  9,   10);

    transpose(uut0, uut1, uut2, uut3);

    /* Checks */
    BOOST_CHECK(uut0[0] == -100);
    BOOST_CHECK(uut0[1] ==    5);
    BOOST_CHECK(uut0[2] ==   50);
    BOOST_CHECK(uut0[3] ==  -50);

    BOOST_CHECK(uut1[0] == -5);
    BOOST_CHECK(uut1[1] == -5);
    BOOST_CHECK(uut1[2] == -5);
    BOOST_CHECK(uut1[3] == -5);

    BOOST_CHECK(uut2[0] == 10);
    BOOST_CHECK(uut2[1] ==  1);
    BOOST_CHECK(uut2[2] == 11);
    BOOST_CHECK(uut2[3] ==  9);

    BOOST_CHECK(uut3[0] ==  100);
    BOOST_CHECK(uut3[1] == -100);
    BOOST_CHECK(uut3[2] ==  -10);
    BOOST_CHECK(uut3[3] ==   10);
}

BOOST_AUTO_TEST_CASE( simple_divisior_morton_code_test )
{
    const vfp_t x(1.0f, 2.0f, 3.0f, 4.0f);
    const vfp_t y(1.1f, 2.1f, 3.1f, 4.1f);
    const vfp_t z(1.2f, 2.2f, 3.2f, 4.2f);
    const vfp_t x_mul(1.0f);
    const vfp_t y_mul(1.0f);
    const vfp_t z_mul(1.0f);

    const vint_t mc(morton_code(x, y, z, x_mul, y_mul, z_mul));
    BOOST_CHECK(mc[0] == 0x007);
    BOOST_CHECK(mc[1] == 0x038);
    BOOST_CHECK(mc[2] == 0x03f);
    BOOST_CHECK(mc[3] == 0x1c0);
}

BOOST_AUTO_TEST_CASE( max_bit_range_morton_code_test )
{
    const vfp_t x(1023.0f, 1023.0f, 1023.0f, 1023.0f);
    const vfp_t y(2046.1f, 2046.1f, 2046.1f, 2046.1f);
    const vfp_t z(3069.2f, 3069.2f, 3069.2f, 3069.2f);
    const vfp_t x_mul(1.0f / 1.0f);
    const vfp_t y_mul(1.0f / 2.0f);
    const vfp_t z_mul(1.0f / 3.0f);

    const vint_t mc(morton_code(x, y, z, x_mul, y_mul, z_mul));
    BOOST_CHECK(mc[0] == 0x3fffffff);
    BOOST_CHECK(mc[1] == 0x3fffffff);
    BOOST_CHECK(mc[2] == 0x3fffffff);
    BOOST_CHECK(mc[3] == 0x3fffffff);
}

BOOST_AUTO_TEST_CASE( morton_code_test )
{
    const vfp_t x(1023.0f,  10.0f,  523.0f,   6.0f);
    const vfp_t y( 246.1f,  26.1f,  146.1f,  92.1f);
    const vfp_t z(  39.2f, 369.2f, 1569.2f, 184.2f);
    const vfp_t x_mul(1.0f / 1.0f, 1.0f / 4.5f, 1.0f / 8.3f, 1.0f / 1.4f);
    const vfp_t y_mul(1.0f / 2.0f, 1.0f / 3.6f, 1.0f / 7.1f, 1.0f / 2.7f);
    const vfp_t z_mul(1.0f / 3.0f, 1.0f / 2.7f, 1.0f / 6.0f, 1.0f / 3.3f);

    const vint_t mc(morton_code(x, y, z, x_mul, y_mul, z_mul));
    BOOST_CHECK(mc[0] == 0x92dbf5f);
    BOOST_CHECK(mc[1] == 0x080089a);
    BOOST_CHECK(mc[2] == 0x400b3cd);
    BOOST_CHECK(mc[3] == 0x0034174);
}

BOOST_AUTO_TEST_CASE( morton_code_performance_test )
{
    /* Generate some input data */
    const int test_size = 262144;
    std::vector<float> x(test_size);
    std::vector<float> y(test_size);
    std::vector<float> z(test_size);
    std::vector<float> x_mul(test_size);
    std::vector<float> y_mul(test_size);
    std::vector<float> z_mul(test_size);

    std::default_random_engine gen;
    std::uniform_real_distribution<float> scaled_dist(-511.0f, 511.0f);
    std::uniform_real_distribution<float> unscaled_dist(-50000.0f, 50000.0f);
    for (int i = 0; i < test_size; ++i)
    {
        x[i] = unscaled_dist(gen);
        y[i] = unscaled_dist(gen);
        z[i] = unscaled_dist(gen);
        x_mul[i] = scaled_dist(gen) / x[i];
        y_mul[i] = scaled_dist(gen) / y[i];
        z_mul[i] = scaled_dist(gen) / z[i];
    }

    /* Run scalar code */
    int scalar_mc[test_size];
    auto scalar_t0(std::chrono::system_clock::now());
    for (int i = 0; i < test_size; ++i)
    {
        scalar_mc[i] = morton_code(x[i], y[i], z[i], x_mul[i], y_mul[i], z_mul[i]);
    }
    auto scalar_t1(std::chrono::system_clock::now());
    BOOST_LOG_TRIVIAL(fatal) << "PERF 0 - Test: Scalar Morton Code";
    BOOST_LOG_TRIVIAL(fatal) << "PERF 1 - Runtime us: " << std::chrono::duration_cast<std::chrono::microseconds>(scalar_t1 - scalar_t0).count();

    /* Run vector code */
    int vector_mc[test_size];
    auto vector_t0(std::chrono::system_clock::now());
    for (int i = 0; i < test_size; i += SIMD_WIDTH)
    {
        const vint_t mc(morton_code(vfp_t(&x[i]), vfp_t(&y[i]), vfp_t(&z[i]), vfp_t(&x_mul[i]), vfp_t(&y_mul[i]), vfp_t(&z_mul[i])));
        mc.store(&vector_mc[i]);
    }
    auto vector_t1(std::chrono::system_clock::now());
    BOOST_LOG_TRIVIAL(fatal) << "PERF 0 - Test: Vector Morton Code";
    BOOST_LOG_TRIVIAL(fatal) << "PERF 1 - Runtime us: " << std::chrono::duration_cast<std::chrono::microseconds>(vector_t1 - vector_t0).count();
    
    /* Checks */
    bool passed = true;
    for (int i = 0; i < test_size; ++i)
    {
        passed &= (scalar_mc[i] = vector_mc[i]);
    }
    BOOST_CHECK(passed);
}

BOOST_AUTO_TEST_SUITE_END()
}; // namespace test
}; // namespace raptor_raytracer
