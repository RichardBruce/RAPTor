#ifdef STAND_ALONE
#define BOOST_TEST_MODULE voxel test

/* Common headers */
#include "logging.h"

/* Initialise logger */
const raptor_physics::init_logger init_logger;
#endif /* #ifdef STAND_ALONE */

/* Standard headers */
#include <vector>

/* Common headers */
#include "logging.h"

/* Boost headers */
#include "boost/test/unit_test.hpp"

/* Ray tracer headers */
#include "voxel.h"


namespace raptor_raytracer
{
namespace test
{
const float result_tolerance = 0.0005f;

BOOST_AUTO_TEST_SUITE( voxel_tests );


/* approximate_sah_minima tests */
BOOST_AUTO_TEST_CASE( voxel_approx_minima_x0_sensitivity_test )
{
    BOOST_CHECK(approximate_sah_minima(0.0f, 10.0f, 10.0f, 0.0f, 5.0f, 0.1f, 1.0f, 1.0f, 1.0f) + 5.0f == 2.775f);
    BOOST_CHECK(approximate_sah_minima(0.0f, 10.0f, 10.0f, 0.0f, 4.0f, 0.1f, 1.0f, 1.0f, 1.0f) + 4.0f == 2.275f);
    BOOST_CHECK(approximate_sah_minima(0.0f, 10.0f, 10.0f, 0.0f, 3.0f, 0.1f, 1.0f, 1.0f, 1.0f) + 3.0f == 1.775f);
    BOOST_CHECK(approximate_sah_minima(0.0f, 10.0f, 10.0f, 0.0f, 2.0f, 0.1f, 1.0f, 1.0f, 1.0f) + 2.0f == 1.275f);
    BOOST_CHECK(approximate_sah_minima(0.0f, 10.0f, 10.0f, 0.0f, 1.0f, 0.1f, 1.0f, 1.0f, 1.0f) + 1.0f == 0.775f);
}

BOOST_AUTO_TEST_CASE( voxel_approx_minima_xw_sensitivity_test )
{
    BOOST_CHECK(approximate_sah_minima(0.0f, 10.0f, 10.0f, 0.0f, 5.0f, 0.1f, 1.0f, 1.0f, 1.0f) + 5.0f == 2.775f);
    BOOST_CHECK(approximate_sah_minima(0.0f, 10.0f, 10.0f, 0.0f, 5.0f, 0.1f, 2.0f, 1.0f, 1.0f) + 5.0f == 3.025f);
    BOOST_CHECK(approximate_sah_minima(0.0f, 10.0f, 10.0f, 0.0f, 5.0f, 0.1f, 3.0f, 1.0f, 1.0f) + 5.0f == 3.275f);
    BOOST_CHECK(approximate_sah_minima(0.0f, 10.0f, 10.0f, 0.0f, 5.0f, 0.1f, 4.0f, 1.0f, 1.0f) + 5.0f == 3.525f);
    BOOST_CHECK(approximate_sah_minima(0.0f, 10.0f, 10.0f, 0.0f, 5.0f, 0.1f, 5.0f, 1.0f, 1.0f) + 5.0f == 3.775f);
}

BOOST_AUTO_TEST_CASE( voxel_approx_minima_yw_sensitivity_test )
{
    BOOST_CHECK(approximate_sah_minima(0.0f, 10.0f, 10.0f, 0.0f, 5.0f, 0.1f, 1.0f, 1.0f, 1.0f) + 5.0f == 2.775f);
    BOOST_CHECK(approximate_sah_minima(0.0f, 10.0f, 10.0f, 0.0f, 5.0f, 0.1f, 1.0f, 2.0f, 1.0f) + 5.0f == 2.775f);
    BOOST_CHECK(approximate_sah_minima(0.0f, 10.0f, 10.0f, 0.0f, 5.0f, 0.1f, 1.0f, 3.0f, 1.0f) + 5.0f == 2.775f);
    BOOST_CHECK(approximate_sah_minima(0.0f, 10.0f, 10.0f, 0.0f, 5.0f, 0.1f, 1.0f, 4.0f, 1.0f) + 5.0f == 2.775f);
    BOOST_CHECK(approximate_sah_minima(0.0f, 10.0f, 10.0f, 0.0f, 5.0f, 0.1f, 1.0f, 5.0f, 1.0f) + 5.0f == 2.775f);
    
    BOOST_CHECK(approximate_sah_minima(0.0f, 10.0f, 10.0f, 0.0f, 5.0f, 0.1f, 3.0f, 1.0f, 1.0f) + 5.0f == 3.275f);
    BOOST_CHECK(approximate_sah_minima(0.0f, 10.0f, 10.0f, 0.0f, 5.0f, 0.1f, 3.0f, 2.0f, 1.0f) + 5.0f == 3.275f);
    BOOST_CHECK(approximate_sah_minima(0.0f, 10.0f, 10.0f, 0.0f, 5.0f, 0.1f, 3.0f, 3.0f, 1.0f) + 5.0f == 3.275f);
    BOOST_CHECK(approximate_sah_minima(0.0f, 10.0f, 10.0f, 0.0f, 5.0f, 0.1f, 3.0f, 4.0f, 1.0f) + 5.0f == 3.275f);
    BOOST_CHECK(approximate_sah_minima(0.0f, 10.0f, 10.0f, 0.0f, 5.0f, 0.1f, 3.0f, 5.0f, 1.0f) + 5.0f == 3.275f);
}

BOOST_AUTO_TEST_CASE( voxel_approx_minima_zw_sensitivity_test )
{
    BOOST_CHECK(approximate_sah_minima(0.0f, 10.0f, 10.0f, 0.0f, 5.0f, 0.1f, 1.0f, 1.0f, 1.0f) + 5.0f == 2.775f);
    BOOST_CHECK(approximate_sah_minima(0.0f, 10.0f, 10.0f, 0.0f, 5.0f, 0.1f, 1.0f, 1.0f, 2.0f) + 5.0f == 2.775f);
    BOOST_CHECK(approximate_sah_minima(0.0f, 10.0f, 10.0f, 0.0f, 5.0f, 0.1f, 1.0f, 1.0f, 3.0f) + 5.0f == 2.775f);
    BOOST_CHECK(approximate_sah_minima(0.0f, 10.0f, 10.0f, 0.0f, 5.0f, 0.1f, 1.0f, 1.0f, 4.0f) + 5.0f == 2.775f);
    BOOST_CHECK(approximate_sah_minima(0.0f, 10.0f, 10.0f, 0.0f, 5.0f, 0.1f, 1.0f, 1.0f, 5.0f) + 5.0f == 2.775f);
    
    BOOST_CHECK(approximate_sah_minima(0.0f, 10.0f, 10.0f, 0.0f, 5.0f, 0.1f, 3.0f, 1.0f, 1.0f) + 5.0f == 3.275f);
    BOOST_CHECK(approximate_sah_minima(0.0f, 10.0f, 10.0f, 0.0f, 5.0f, 0.1f, 3.0f, 1.0f, 2.0f) + 5.0f == 3.275f);
    BOOST_CHECK(approximate_sah_minima(0.0f, 10.0f, 10.0f, 0.0f, 5.0f, 0.1f, 3.0f, 1.0f, 3.0f) + 5.0f == 3.275f);
    BOOST_CHECK(approximate_sah_minima(0.0f, 10.0f, 10.0f, 0.0f, 5.0f, 0.1f, 3.0f, 1.0f, 4.0f) + 5.0f == 3.275f);
    BOOST_CHECK(approximate_sah_minima(0.0f, 10.0f, 10.0f, 0.0f, 5.0f, 0.1f, 3.0f, 1.0f, 5.0f) + 5.0f == 3.275f);
}

BOOST_AUTO_TEST_CASE( voxel_approx_minima_d_sensitivity_test )
{
    BOOST_CHECK(approximate_sah_minima(0.0f, 10.0f, 10.0f, 0.0f, 5.0f, 0.1f, 1.0f, 1.0f, 1.0f) + 5.0f == 2.775f);
    BOOST_CHECK(approximate_sah_minima(0.0f, 10.0f, 10.0f, 0.0f, 5.0f, 0.2f, 1.0f, 1.0f, 1.0f) + 5.0f == 2.8f);
    BOOST_CHECK(approximate_sah_minima(0.0f, 10.0f, 10.0f, 0.0f, 5.0f, 0.3f, 1.0f, 1.0f, 1.0f) + 5.0f == 2.825f);
    BOOST_CHECK(approximate_sah_minima(0.0f, 10.0f, 10.0f, 0.0f, 5.0f, 0.4f, 1.0f, 1.0f, 1.0f) + 5.0f == 2.85f);
    BOOST_CHECK(approximate_sah_minima(0.0f, 10.0f, 10.0f, 0.0f, 5.0f, 0.5f, 1.0f, 1.0f, 1.0f) + 5.0f == 2.875f);
    
    BOOST_CHECK(approximate_sah_minima(0.0f, 10.0f, 10.0f, 0.0f, 5.0f, 0.1f, 2.0f, 1.0f, 1.0f) + 5.0f == 3.025f);
    BOOST_CHECK(approximate_sah_minima(0.0f, 10.0f, 10.0f, 0.0f, 5.0f, 0.2f, 2.0f, 1.0f, 1.0f) + 5.0f == 3.05f);
    BOOST_CHECK(approximate_sah_minima(0.0f, 10.0f, 10.0f, 0.0f, 5.0f, 0.3f, 2.0f, 1.0f, 1.0f) + 5.0f == 3.075f);
    BOOST_CHECK(approximate_sah_minima(0.0f, 10.0f, 10.0f, 0.0f, 5.0f, 0.4f, 2.0f, 1.0f, 1.0f) + 5.0f == 3.1f);
    BOOST_CHECK(approximate_sah_minima(0.0f, 10.0f, 10.0f, 0.0f, 5.0f, 0.5f, 2.0f, 1.0f, 1.0f) + 5.0f == 3.125f);
}

BOOST_AUTO_TEST_CASE( voxel_approx_minima_cl0_sensitivity_test )
{
    BOOST_CHECK_CLOSE(approximate_sah_minima(0.0f, 10.0f, 10.0f, 0.0f, 5.0f, 0.1f, 1.0f, 1.0f, 1.0f) + 5.0f, 2.775f,    result_tolerance);
    BOOST_CHECK_CLOSE(approximate_sah_minima(1.0f, 10.0f, 10.0f, 0.0f, 5.0f, 0.1f, 1.0f, 1.0f, 1.0f) + 5.0f, 2.8f,      result_tolerance);
    BOOST_CHECK_CLOSE(approximate_sah_minima(2.0f, 10.0f, 10.0f, 0.0f, 5.0f, 0.1f, 1.0f, 1.0f, 1.0f) + 5.0f, 2.82778f,  result_tolerance);
    BOOST_CHECK_CLOSE(approximate_sah_minima(3.0f, 10.0f, 10.0f, 0.0f, 5.0f, 0.1f, 1.0f, 1.0f, 1.0f) + 5.0f, 2.85882f,  result_tolerance);
    BOOST_CHECK_CLOSE(approximate_sah_minima(4.0f, 10.0f, 10.0f, 0.0f, 5.0f, 0.1f, 1.0f, 1.0f, 1.0f) + 5.0f, 2.89375f,  result_tolerance);
}

BOOST_AUTO_TEST_CASE( voxel_approx_minima_cl1_sensitivity_test )
{
    BOOST_CHECK_CLOSE(approximate_sah_minima(0.0f, 10.0f, 10.0f, 0.0f, 5.0f, 0.1f, 1.0f, 1.0f, 1.0f) + 5.0f, 2.775f,    result_tolerance);
    BOOST_CHECK_CLOSE(approximate_sah_minima(0.0f,  9.0f, 10.0f, 0.0f, 5.0f, 0.1f, 1.0f, 1.0f, 1.0f) + 5.0f, 2.80263f,  result_tolerance);
    BOOST_CHECK_CLOSE(approximate_sah_minima(0.0f,  8.0f, 10.0f, 0.0f, 5.0f, 0.1f, 1.0f, 1.0f, 1.0f) + 5.0f, 2.83333f,  result_tolerance);
    BOOST_CHECK_CLOSE(approximate_sah_minima(0.0f,  7.0f, 10.0f, 0.0f, 5.0f, 0.1f, 1.0f, 1.0f, 1.0f) + 5.0f, 2.86765f,  result_tolerance);
    BOOST_CHECK_CLOSE(approximate_sah_minima(0.0f,  6.0f, 10.0f, 0.0f, 5.0f, 0.1f, 1.0f, 1.0f, 1.0f) + 5.0f, 2.90625f,  result_tolerance);
}

BOOST_AUTO_TEST_CASE( voxel_approx_minima_cr0_sensitivity_test )
{
    BOOST_CHECK_CLOSE(approximate_sah_minima(0.0f, 10.0f, 10.0f, 0.0f, 5.0f, 0.1f, 1.0f, 1.0f, 1.0f) + 5.0f, 2.775f,    result_tolerance);
    BOOST_CHECK_CLOSE(approximate_sah_minima(0.0f, 10.0f,  9.0f, 0.0f, 5.0f, 0.1f, 1.0f, 1.0f, 1.0f) + 5.0f, 2.74737f,  result_tolerance);
    BOOST_CHECK_CLOSE(approximate_sah_minima(0.0f, 10.0f,  8.0f, 0.0f, 5.0f, 0.1f, 1.0f, 1.0f, 1.0f) + 5.0f, 2.71667f,  result_tolerance);
    BOOST_CHECK_CLOSE(approximate_sah_minima(0.0f, 10.0f,  7.0f, 0.0f, 5.0f, 0.1f, 1.0f, 1.0f, 1.0f) + 5.0f, 2.68235f,  result_tolerance);
    BOOST_CHECK_CLOSE(approximate_sah_minima(0.0f, 10.0f,  6.0f, 0.0f, 5.0f, 0.1f, 1.0f, 1.0f, 1.0f) + 5.0f, 2.64375f,  result_tolerance);
}

BOOST_AUTO_TEST_CASE( voxel_approx_minima_cr1_sensitivity_test )
{
    BOOST_CHECK_CLOSE(approximate_sah_minima(0.0f, 10.0f, 10.0f, 0.0f, 5.0f, 0.1f, 1.0f, 1.0f, 1.0f) + 5.0f, 2.775f,    result_tolerance);
    BOOST_CHECK_CLOSE(approximate_sah_minima(0.0f, 10.0f, 10.0f, 1.0f, 5.0f, 0.1f, 1.0f, 1.0f, 1.0f) + 5.0f, 2.75f,     result_tolerance);
    BOOST_CHECK_CLOSE(approximate_sah_minima(0.0f, 10.0f, 10.0f, 2.0f, 5.0f, 0.1f, 1.0f, 1.0f, 1.0f) + 5.0f, 2.72222f,  result_tolerance);
    BOOST_CHECK_CLOSE(approximate_sah_minima(0.0f, 10.0f, 10.0f, 3.0f, 5.0f, 0.1f, 1.0f, 1.0f, 1.0f) + 5.0f, 2.69118f,  result_tolerance);
    BOOST_CHECK_CLOSE(approximate_sah_minima(0.0f, 10.0f, 10.0f, 4.0f, 5.0f, 0.1f, 1.0f, 1.0f, 1.0f) + 5.0f, 2.65625f,  result_tolerance);
}

/* fix_adaptive_samples tests */
struct fix_adaptive_samples_fixture
{
    std::vector<float>  cl;
    std::vector<float>  cr;
    float               nr_samples[9];
    float               widths[9];
    float               samples[9];
};

BOOST_FIXTURE_TEST_CASE( regular_samples_fix_adaptive_samples_test, fix_adaptive_samples_fixture )
{
    cl = {  0,  5, 10, 15, 20, 25, 30, 35 };
    cr = { 35, 30, 25, 20, 15, 10,  5,  0 };

    /* Fix the adaptive sample locations */
    fix_adaptive_samples(&nr_samples[0], &widths[0], &samples[0], cl.data(), cr.data(), 2.0f, 1.0f, 35.0f);

    /* Checks */
    BOOST_CHECK(nr_samples[0] == 0);
    BOOST_CHECK(nr_samples[1] == 1);
    BOOST_CHECK(nr_samples[2] == 1);
    BOOST_CHECK(nr_samples[3] == 1);
    BOOST_CHECK(nr_samples[4] == 2);
    BOOST_CHECK(nr_samples[5] == 1);
    BOOST_CHECK(nr_samples[6] == 1);
    BOOST_CHECK(nr_samples[7] == 1);
    BOOST_CHECK(nr_samples[8] == 0);

    BOOST_CHECK_CLOSE(widths[0], 1.0f,      result_tolerance);
    BOOST_CHECK_CLOSE(widths[1], 0.5f,      result_tolerance);
    BOOST_CHECK_CLOSE(widths[2], 0.5f,      result_tolerance);
    BOOST_CHECK_CLOSE(widths[3], 0.5f,      result_tolerance);
    BOOST_CHECK_CLOSE(widths[4], 0.333333f, result_tolerance);
    BOOST_CHECK_CLOSE(widths[5], 0.5f,      result_tolerance);
    BOOST_CHECK_CLOSE(widths[6], 0.5f,      result_tolerance);
    BOOST_CHECK_CLOSE(widths[7], 0.5f,      result_tolerance);
    BOOST_CHECK_CLOSE(widths[8], 1.0f,      result_tolerance);

    BOOST_CHECK_CLOSE(samples[0], 3.5f,     result_tolerance);
    BOOST_CHECK_CLOSE(samples[1], 4.5f,     result_tolerance);
    BOOST_CHECK_CLOSE(samples[2], 5.5f,     result_tolerance);
    BOOST_CHECK_CLOSE(samples[3], 6.33333f, result_tolerance);
    BOOST_CHECK_CLOSE(samples[4], 6.66667f, result_tolerance);
    BOOST_CHECK_CLOSE(samples[5], 7.5f,     result_tolerance);
    BOOST_CHECK_CLOSE(samples[6], 8.5f,     result_tolerance);
    BOOST_CHECK_CLOSE(samples[7], 9.5f,     result_tolerance);
}

BOOST_FIXTURE_TEST_CASE( regular_samples_x0_fix_adaptive_samples_test, fix_adaptive_samples_fixture )
{
    cl = {  0,  5, 10, 15, 20, 25, 30, 35 };
    cr = { 35, 30, 25, 20, 15, 10,  5,  0 };

    /* Up x0 and fix the adaptive sample locations */
    fix_adaptive_samples(&nr_samples[0], &widths[0], &samples[0], cl.data(), cr.data(), 5.0f, 1.0f, 35.0f);

    /* Checks */
    BOOST_CHECK(nr_samples[0] == 0);
    BOOST_CHECK(nr_samples[1] == 1);
    BOOST_CHECK(nr_samples[2] == 1);
    BOOST_CHECK(nr_samples[3] == 1);
    BOOST_CHECK(nr_samples[4] == 2);
    BOOST_CHECK(nr_samples[5] == 1);
    BOOST_CHECK(nr_samples[6] == 1);
    BOOST_CHECK(nr_samples[7] == 1);
    BOOST_CHECK(nr_samples[8] == 0);

    BOOST_CHECK_CLOSE(widths[0], 1.0f,      result_tolerance);
    BOOST_CHECK_CLOSE(widths[1], 0.5f,      result_tolerance);
    BOOST_CHECK_CLOSE(widths[2], 0.5f,      result_tolerance);
    BOOST_CHECK_CLOSE(widths[3], 0.5f,      result_tolerance);
    BOOST_CHECK_CLOSE(widths[4], 0.333333f, result_tolerance);
    BOOST_CHECK_CLOSE(widths[5], 0.5f,      result_tolerance);
    BOOST_CHECK_CLOSE(widths[6], 0.5f,      result_tolerance);
    BOOST_CHECK_CLOSE(widths[7], 0.5f,      result_tolerance);
    BOOST_CHECK_CLOSE(widths[8], 1.0f,      result_tolerance);

    BOOST_CHECK_CLOSE(samples[0],  6.5f,        result_tolerance);
    BOOST_CHECK_CLOSE(samples[1],  7.5f,        result_tolerance);
    BOOST_CHECK_CLOSE(samples[2],  8.5f,        result_tolerance);
    BOOST_CHECK_CLOSE(samples[3],  9.33333f,    result_tolerance);
    BOOST_CHECK_CLOSE(samples[4],  9.66667f,    result_tolerance);
    BOOST_CHECK_CLOSE(samples[5], 10.5f,        result_tolerance);
    BOOST_CHECK_CLOSE(samples[6], 11.5f,        result_tolerance);
    BOOST_CHECK_CLOSE(samples[7], 12.5f,        result_tolerance);
}

BOOST_FIXTURE_TEST_CASE( regular_samples_dx_fix_adaptive_samples_test, fix_adaptive_samples_fixture )
{
    cl = {  0,  5, 10, 15, 20, 25, 30, 35 };
    cr = { 35, 30, 25, 20, 15, 10,  5,  0 };
    /* Up the bin width and fix the adaptive sample locations */
    fix_adaptive_samples(&nr_samples[0], &widths[0], &samples[0], cl.data(), cr.data(), 2.0f, 3.0f, 35.0f);

    /* Checks */
    BOOST_CHECK(nr_samples[0] == 0);
    BOOST_CHECK(nr_samples[1] == 1);
    BOOST_CHECK(nr_samples[2] == 1);
    BOOST_CHECK(nr_samples[3] == 1);
    BOOST_CHECK(nr_samples[4] == 2);
    BOOST_CHECK(nr_samples[5] == 1);
    BOOST_CHECK(nr_samples[6] == 1);
    BOOST_CHECK(nr_samples[7] == 1);
    BOOST_CHECK(nr_samples[8] == 0);

    BOOST_CHECK_CLOSE(widths[0], 3.0f,  result_tolerance);
    BOOST_CHECK_CLOSE(widths[1], 1.5f,  result_tolerance);
    BOOST_CHECK_CLOSE(widths[2], 1.5f,  result_tolerance);
    BOOST_CHECK_CLOSE(widths[3], 1.5f,  result_tolerance);
    BOOST_CHECK_CLOSE(widths[4], 1.0f,  result_tolerance);
    BOOST_CHECK_CLOSE(widths[5], 1.5f,  result_tolerance);
    BOOST_CHECK_CLOSE(widths[6], 1.5f,  result_tolerance);
    BOOST_CHECK_CLOSE(widths[7], 1.5f,  result_tolerance);
    BOOST_CHECK_CLOSE(widths[8], 3.0f,  result_tolerance);

    BOOST_CHECK_CLOSE(samples[0],  6.5f,    result_tolerance);
    BOOST_CHECK_CLOSE(samples[1],  9.5f,    result_tolerance);
    BOOST_CHECK_CLOSE(samples[2], 12.5f,    result_tolerance);
    BOOST_CHECK_CLOSE(samples[3], 15.0f,    result_tolerance);
    BOOST_CHECK_CLOSE(samples[4], 16.0f,    result_tolerance);
    BOOST_CHECK_CLOSE(samples[5], 18.5f,    result_tolerance);
    BOOST_CHECK_CLOSE(samples[6], 21.5f,    result_tolerance);
    BOOST_CHECK_CLOSE(samples[7], 24.5f,    result_tolerance);
}

BOOST_FIXTURE_TEST_CASE( discontinious_samples_fix_adaptive_samples_test, fix_adaptive_samples_fixture )
{
    /* Set up data */
    cl = {  0,  0,  0, 35, 35, 35, 35, 35 };
    cr = { 35, 35, 35,  0,  0,  0,  0,  0 };

    /* Fix the adaptive sample locations */
    fix_adaptive_samples(&nr_samples[0], &widths[0], &samples[0], cl.data(), cr.data(), 2.0f, 1.0f, 35.0f);

    /* Checks */
    BOOST_CHECK(nr_samples[0] == 0);
    BOOST_CHECK(nr_samples[1] == 0);
    BOOST_CHECK(nr_samples[2] == 0);
    BOOST_CHECK(nr_samples[3] == 8);
    BOOST_CHECK(nr_samples[4] == 0);
    BOOST_CHECK(nr_samples[5] == 0);
    BOOST_CHECK(nr_samples[6] == 0);
    BOOST_CHECK(nr_samples[7] == 0);
    BOOST_CHECK(nr_samples[8] == 0);

    BOOST_CHECK_CLOSE(widths[0], 1.0f,      result_tolerance);
    BOOST_CHECK_CLOSE(widths[1], 1.0f,      result_tolerance);
    BOOST_CHECK_CLOSE(widths[2], 1.0f,      result_tolerance);
    BOOST_CHECK_CLOSE(widths[3], 0.111111f, result_tolerance);
    BOOST_CHECK_CLOSE(widths[4], 1.0f,      result_tolerance);
    BOOST_CHECK_CLOSE(widths[5], 1.0f,      result_tolerance);
    BOOST_CHECK_CLOSE(widths[6], 1.0f,      result_tolerance);
    BOOST_CHECK_CLOSE(widths[7], 1.0f,      result_tolerance);
    BOOST_CHECK_CLOSE(widths[8], 1.0f,      result_tolerance);

    BOOST_CHECK_CLOSE(samples[0], 5.11111f, result_tolerance);
    BOOST_CHECK_CLOSE(samples[1], 5.22222f, result_tolerance);
    BOOST_CHECK_CLOSE(samples[2], 5.33333f, result_tolerance);
    BOOST_CHECK_CLOSE(samples[3], 5.44444f, result_tolerance);
    BOOST_CHECK_CLOSE(samples[4], 5.55555f, result_tolerance);
    BOOST_CHECK_CLOSE(samples[5], 5.66666f, result_tolerance);
    BOOST_CHECK_CLOSE(samples[6], 5.77777f, result_tolerance);
    BOOST_CHECK_CLOSE(samples[7], 5.88888f, result_tolerance);
}

BOOST_FIXTURE_TEST_CASE( fix_cl_fix_adaptive_samples_test, fix_adaptive_samples_fixture )
{
    /* Set up data */
    cl = { 35, 35, 35, 35, 35, 35, 35, 35 };
    cr = { 35, 30, 25, 20, 15, 10,  5,  0 };

    /* Fix the adaptive sample locations */
    fix_adaptive_samples(&nr_samples[0], &widths[0], &samples[0], cl.data(), cr.data(), 2.0f, 1.0f, 35.0f);

    /* Checks */
    BOOST_CHECK(nr_samples[0] == 4);
    BOOST_CHECK(nr_samples[1] == 1);
    BOOST_CHECK(nr_samples[2] == 0);
    BOOST_CHECK(nr_samples[3] == 1);
    BOOST_CHECK(nr_samples[4] == 1);
    BOOST_CHECK(nr_samples[5] == 0);
    BOOST_CHECK(nr_samples[6] == 1);
    BOOST_CHECK(nr_samples[7] == 0);
    BOOST_CHECK(nr_samples[8] == 0);

    BOOST_CHECK_CLOSE(widths[0], 0.2f,  result_tolerance);
    BOOST_CHECK_CLOSE(widths[1], 0.5f,  result_tolerance);
    BOOST_CHECK_CLOSE(widths[2], 1.0f,  result_tolerance);
    BOOST_CHECK_CLOSE(widths[3], 0.5f,  result_tolerance);
    BOOST_CHECK_CLOSE(widths[4], 0.5f,  result_tolerance);
    BOOST_CHECK_CLOSE(widths[5], 1.0f,  result_tolerance);
    BOOST_CHECK_CLOSE(widths[6], 0.5f,  result_tolerance);
    BOOST_CHECK_CLOSE(widths[7], 1.0f,  result_tolerance);
    BOOST_CHECK_CLOSE(widths[8], 1.0f,  result_tolerance);

    BOOST_CHECK_CLOSE(samples[0], 2.2f, result_tolerance);
    BOOST_CHECK_CLOSE(samples[1], 2.4f, result_tolerance);
    BOOST_CHECK_CLOSE(samples[2], 2.6f, result_tolerance);
    BOOST_CHECK_CLOSE(samples[3], 2.8f, result_tolerance);
    BOOST_CHECK_CLOSE(samples[4], 3.5f, result_tolerance);
    BOOST_CHECK_CLOSE(samples[5], 5.5f, result_tolerance);
    BOOST_CHECK_CLOSE(samples[6], 6.5f, result_tolerance);
    BOOST_CHECK_CLOSE(samples[7], 8.5f, result_tolerance);
}

BOOST_FIXTURE_TEST_CASE( fix_cr_fix_adaptive_samples_test, fix_adaptive_samples_fixture )
{
    /* Set up data */
    cl = {  0,  5, 10, 15, 20, 25, 30, 35 };
    cr = { 35, 35, 35, 35, 35, 35, 35, 35 };

    /* Fix the adaptive sample locations */
    fix_adaptive_samples(&nr_samples[0], &widths[0], &samples[0], cl.data(), cr.data(), 2.0f, 1.0f, 35.0f);

    /* Checks */
    BOOST_CHECK(nr_samples[0] == 0);
    BOOST_CHECK(nr_samples[1] == 0);
    BOOST_CHECK(nr_samples[2] == 1);
    BOOST_CHECK(nr_samples[3] == 0);
    BOOST_CHECK(nr_samples[4] == 1);
    BOOST_CHECK(nr_samples[5] == 1);
    BOOST_CHECK(nr_samples[6] == 0);
    BOOST_CHECK(nr_samples[7] == 1);
    BOOST_CHECK(nr_samples[8] == 4);

    BOOST_CHECK_CLOSE(widths[0], 1.0f,  result_tolerance);
    BOOST_CHECK_CLOSE(widths[1], 1.0f,  result_tolerance);
    BOOST_CHECK_CLOSE(widths[2], 0.5f,  result_tolerance);
    BOOST_CHECK_CLOSE(widths[3], 1.0f,  result_tolerance);
    BOOST_CHECK_CLOSE(widths[4], 0.5f,  result_tolerance);
    BOOST_CHECK_CLOSE(widths[5], 0.5f,  result_tolerance);
    BOOST_CHECK_CLOSE(widths[6], 1.0f,  result_tolerance);
    BOOST_CHECK_CLOSE(widths[7], 0.5f,  result_tolerance);
    BOOST_CHECK_CLOSE(widths[8], 0.2f,  result_tolerance);

    BOOST_CHECK_CLOSE(samples[0],  4.5f,    result_tolerance);
    BOOST_CHECK_CLOSE(samples[1],  6.5f,    result_tolerance);
    BOOST_CHECK_CLOSE(samples[2],  7.5f,    result_tolerance);
    BOOST_CHECK_CLOSE(samples[3],  9.5f,    result_tolerance);
    BOOST_CHECK_CLOSE(samples[4], 10.2f,    result_tolerance);
    BOOST_CHECK_CLOSE(samples[5], 10.4f,    result_tolerance);
    BOOST_CHECK_CLOSE(samples[6], 10.6f,    result_tolerance);
    BOOST_CHECK_CLOSE(samples[7], 10.8f,    result_tolerance);
}

/* Clip triangle test */
BOOST_AUTO_TEST_CASE( clip_triangle_x_no_parent_bounds )
{
    point_t r_bl;
    point_t r_tr;
    point_t bl(-MAX_DIST, -MAX_DIST, -MAX_DIST);
    point_t tr( MAX_DIST,  MAX_DIST,  MAX_DIST);
    triangle tri(nullptr, point_t(0.0f, 0.0f, 0.0f), point_t(1.0f, 0.0f, 0.0f), point_t(1.0f, 1.0f, 0.0f));
    clip_triangle(&tri, &bl, &tr, &r_bl, &r_tr, 0.5f, axis_t::x_axis);

    BOOST_CHECK(fabs(magnitude(bl   - point_t(0.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(tr   - point_t(0.5f, 0.5f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(r_bl - point_t(0.5f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(r_tr - point_t(1.0f, 1.0f, 0.0f))) < result_tolerance);
}

BOOST_AUTO_TEST_CASE( clip_triangle_y_no_parent_bounds )
{
    point_t r_bl;
    point_t r_tr;
    point_t bl(-MAX_DIST, -MAX_DIST, -MAX_DIST);
    point_t tr( MAX_DIST,  MAX_DIST,  MAX_DIST);
    triangle tri(nullptr, point_t(0.0f, 0.0f, 0.0f), point_t(1.0f, 0.0f, 0.0f), point_t(1.0f, 1.0f, 0.0f));
    clip_triangle(&tri, &bl, &tr, &r_bl, &r_tr, 0.25f, axis_t::y_axis);

    BOOST_CHECK(fabs(magnitude(bl   - point_t(0.0f,  0.0f,  0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(tr   - point_t(1.0f,  0.25f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(r_bl - point_t(0.25f, 0.25f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(r_tr - point_t(1.0f,  1.0f,  0.0f))) < result_tolerance);
}

BOOST_AUTO_TEST_CASE( clip_triangle_z_no_parent_bounds )
{
    point_t r_bl;
    point_t r_tr;
    point_t bl(-MAX_DIST, -MAX_DIST, -MAX_DIST);
    point_t tr( MAX_DIST,  MAX_DIST,  MAX_DIST);
    triangle tri(nullptr, point_t(0.0f, 0.0f, 0.0f), point_t(1.0f, 0.0f, 0.0f), point_t(1.0f, 0.0f, 5.0f));
    clip_triangle(&tri, &bl, &tr, &r_bl, &r_tr, 0.75f, axis_t::z_axis);

    BOOST_CHECK(fabs(magnitude(bl   - point_t(0.0f,  0.0f, 0.0f)))  < result_tolerance);
    BOOST_CHECK(fabs(magnitude(tr   - point_t(1.0f,  0.0f, 0.75f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(r_bl - point_t(0.15f, 0.0f, 0.75f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(r_tr - point_t(1.0f,  0.0f, 5.0f)))  < result_tolerance);
}

BOOST_AUTO_TEST_CASE( clip_triangle_x )
{
    point_t r_bl;
    point_t r_tr;
    point_t bl(0.1f, 0.2f, 0.0f);
    point_t tr(0.9f, 0.8f, 0.7f);
    triangle tri(nullptr, point_t(0.0f, 0.0f, 0.0f), point_t(1.0f, 0.0f, 0.0f), point_t(1.0f, 1.0f, 0.0f));
    clip_triangle(&tri, &bl, &tr, &r_bl, &r_tr, 0.5f, axis_t::x_axis);

    BOOST_CHECK(fabs(magnitude(bl   - point_t(0.1f, 0.2f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(tr   - point_t(0.5f, 0.5f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(r_bl - point_t(0.5f, 0.2f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(r_tr - point_t(0.9f, 0.8f, 0.0f))) < result_tolerance);
}

BOOST_AUTO_TEST_CASE( clip_triangle_y )
{
    point_t r_bl;
    point_t r_tr;
    point_t bl(0.2f, 0.4f, 0.0f);
    point_t tr(0.6f, 0.7f, 0.0f);
    triangle tri(nullptr, point_t(0.0f, 0.0f, 0.0f), point_t(1.0f, 0.0f, 0.0f), point_t(1.0f, 1.0f, 0.0f));
    clip_triangle(&tri, &bl, &tr, &r_bl, &r_tr, 0.25f, axis_t::y_axis);

    BOOST_CHECK(fabs(magnitude(bl   - point_t(0.2f,  0.4f,  0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(tr   - point_t(0.6f,  0.25f, 0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(r_bl - point_t(0.25f, 0.4f,  0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(r_tr - point_t(0.6f,  0.7f,  0.0f))) < result_tolerance);
}

BOOST_AUTO_TEST_CASE( clip_triangle_z )
{
    point_t r_bl;
    point_t r_tr;
    point_t bl(0.2f, 0.0f, 0.4f);
    point_t tr(0.6f, 0.0f, 0.7f);
    triangle tri(nullptr, point_t(0.0f, 0.0f, 0.0f), point_t(1.0f, 0.0f, 0.0f), point_t(1.0f, 0.0f, 5.0f));
    clip_triangle(&tri, &bl, &tr, &r_bl, &r_tr, 0.75f, axis_t::z_axis);

    BOOST_CHECK(fabs(magnitude(bl   - point_t(0.2f,  0.0f, 0.4f)))  < result_tolerance);
    BOOST_CHECK(fabs(magnitude(tr   - point_t(0.6f,  0.0f, 0.7f)))  < result_tolerance);
    BOOST_CHECK(fabs(magnitude(r_bl - point_t(0.2f,  0.0f, 0.75f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(r_tr - point_t(0.6f,  0.0f, 0.7f)))  < result_tolerance);
}

BOOST_AUTO_TEST_SUITE_END()
}; // namespace test
}; // namespace raptor_raytracer
