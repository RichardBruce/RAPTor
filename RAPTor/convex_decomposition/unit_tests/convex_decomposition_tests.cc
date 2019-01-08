#ifdef STAND_ALONE
#define BOOST_TEST_MODULE convex_decomposition test
#endif /* #ifdef STAND_ALONE */


/* Standard headers */
#include <chrono>
#include <iostream>
#include <random>

/* Boost headers */
#include "boost/noncopyable.hpp"
#include "boost/test/unit_test.hpp"

/* Convex Decomposition headers */
#include "convex_decomposition.h"


namespace raptor_convex_decomposition
{
namespace test
{
/* Test data */
struct convex_decomposition_fixture : private boost::noncopyable
{
    convex_decomposition_fixture() {  }
};


BOOST_FIXTURE_TEST_SUITE( convex_decomposition_tests, convex_decomposition_fixture )

const float result_tolerance = 0.0005f;


/* Test Ctor */
BOOST_AUTO_TEST_CASE( empty_ctor_test )
{

}

BOOST_AUTO_TEST_SUITE_END()
}; /* namespace test */
}; /* namespace raptor_convex_decomposition */
