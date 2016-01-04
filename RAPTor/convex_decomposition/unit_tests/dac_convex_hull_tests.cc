#ifdef STAND_ALONE
#define BOOST_TEST_MODULE dac_convex_hull test
#endif /* #ifdef STAND_ALONE */


/* Standard headers */
#include <chrono>
#include <iostream>
#include <random>

/* Boost headers */
#include "boost/noncopyable.hpp"
#include "boost/test/unit_test.hpp"

/* Convex Decomposition headers */
#include "dac_convex_hull.h"


namespace raptor_convex_decomposition
{
namespace test
{
/* Test data */
struct dac_convex_hull_fixture : private boost::noncopyable
{
    dac_convex_hull_fixture() {  }
};


BOOST_FIXTURE_TEST_SUITE( dac_convex_hull_tests, dac_convex_hull_fixture )

const float result_tolerance = 0.0005f;


/* Test Ctor */
BOOST_AUTO_TEST_CASE( empty_ctor_test )
{
    const std::vector<point_t> points;

    const dac_convex_hull uut(points);
    BOOST_REQUIRE(uut.vertices().size() == 0);
    BOOST_REQUIRE(uut.faces().size()    == 0);
}

BOOST_AUTO_TEST_CASE( line_x_ctor_test )
{
    /* Add points */
    const std::vector<point_t> points(
    {
        point_t(0.0f, 0.0f, 0.0f),
        point_t(0.0f, 0.0f, 0.0f),
        point_t(1.0f, 0.0f, 0.0f),
        point_t(1.0f, 0.0f, 0.0f),
        point_t(3.0f, 0.0f, 0.0f),
        point_t(4.0f, 0.0f, 0.0f),
        point_t(5.0f, 0.0f, 0.0f),
        point_t(6.0f, 0.0f, 0.0f),
        point_t(8.0f, 0.0f, 0.0f),
        point_t(9.0f, 0.0f, 0.0f),
        point_t(2.0f, 0.0f, 0.0f)
    });

    const dac_convex_hull uut(points);

    /* Checks */
    BOOST_REQUIRE(uut.vertices().size() == 2);
    BOOST_CHECK(uut.vertices()[0] == point_t(9.0f, 0.0f, 0.0f));
    BOOST_CHECK(uut.vertices()[1] == point_t(0.0f, 0.0f, 0.0f));

    BOOST_REQUIRE(uut.faces().size() == 1);

    BOOST_REQUIRE(uut.faces()[0].size() == 2);
    BOOST_CHECK(uut.faces()[0][0] == 1);
    BOOST_CHECK(uut.faces()[0][1] == 0);
}

BOOST_AUTO_TEST_CASE( line_y_ctor_test )
{
    /* Add points */
    const std::vector<point_t> points(
    {
        point_t(0.0f, 0.0f, 0.0f),
        point_t(0.0f, 0.0f, 0.0f),
        point_t(0.0f, 1.0f, 0.0f),
        point_t(0.0f, 1.0f, 0.0f),
        point_t(0.0f, 3.0f, 0.0f),
        point_t(0.0f, 4.0f, 0.0f),
        point_t(0.0f, 5.0f, 0.0f),
        point_t(0.0f, 6.0f, 0.0f),
        point_t(0.0f, 8.0f, 0.0f),
        point_t(0.0f, 9.0f, 0.0f),
        point_t(0.0f, 2.0f, 0.0f)
    });

    const dac_convex_hull uut(points);

    /* Checks */
    BOOST_REQUIRE(uut.vertices().size() == 2);
    BOOST_CHECK(uut.vertices()[0] == point_t(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(uut.vertices()[1] == point_t(0.0f, 9.0f, 0.0f));

    BOOST_REQUIRE(uut.faces().size() == 1);

    BOOST_REQUIRE(uut.faces()[0].size() == 2);
    BOOST_CHECK(uut.faces()[0][0] == 1);
    BOOST_CHECK(uut.faces()[0][1] == 0);
}

BOOST_AUTO_TEST_CASE( line_z_ctor_test )
{
    /* Add points */
    const std::vector<point_t> points(
    {
        point_t(0.0f, 0.0f, 0.0f),
        point_t(0.0f, 0.0f, 0.0f),
        point_t(0.0f, 0.0f, 1.0f),
        point_t(0.0f, 0.0f, 1.0f),
        point_t(0.0f, 0.0f, 3.0f),
        point_t(0.0f, 0.0f, 4.0f),
        point_t(0.0f, 0.0f, 5.0f),
        point_t(0.0f, 0.0f, 6.0f),
        point_t(0.0f, 0.0f, 8.0f),
        point_t(0.0f, 0.0f, 9.0f),
        point_t(0.0f, 0.0f, 2.0f)
    });

    const dac_convex_hull uut(points);

    /* Checks */
    BOOST_REQUIRE(uut.vertices().size() == 2);
    BOOST_CHECK(uut.vertices()[0] == point_t(0.0f, 0.0f, 9.0f));
    BOOST_CHECK(uut.vertices()[1] == point_t(0.0f, 0.0f, 0.0f));

    BOOST_REQUIRE(uut.faces().size() == 1);

    BOOST_REQUIRE(uut.faces()[0].size() == 2);
    BOOST_CHECK(uut.faces()[0][0] == 1);
    BOOST_CHECK(uut.faces()[0][1] == 0);
}

BOOST_AUTO_TEST_CASE( triangle_ctor_test )
{
    /* Add points */
    const std::vector<point_t> points(
    {
        point_t(0.0f, 0.0f, 0.0f),
        point_t(1.0f, 0.0f, 0.0f),
        point_t(0.0f, 1.0f, 0.0f)
    });

    const dac_convex_hull uut(points);

    /* Checks */
    BOOST_REQUIRE(uut.vertices().size() == 3);
    BOOST_CHECK(uut.vertices()[0] == point_t(0.0f, 1.0f, 0.0f));
    BOOST_CHECK(uut.vertices()[1] == point_t(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(uut.vertices()[2] == point_t(1.0f, 0.0f, 0.0f));

    BOOST_REQUIRE(uut.faces().size() == 2);

    BOOST_REQUIRE(uut.faces()[0].size() == 3);
    BOOST_CHECK(uut.faces()[0][0] == 1);
    BOOST_CHECK(uut.faces()[0][1] == 2);
    BOOST_CHECK(uut.faces()[0][2] == 0);

    BOOST_REQUIRE(uut.faces()[1].size() == 3);
    BOOST_CHECK(uut.faces()[1][0] == 2);
    BOOST_CHECK(uut.faces()[1][1] == 1);
    BOOST_CHECK(uut.faces()[1][2] == 0);
}

BOOST_AUTO_TEST_CASE( co_linear_edge_triangle_ctor_test )
{
    /* Add points */
    const std::vector<point_t> points(
    {
        point_t(0.0f, 0.0f, 0.0f),
        point_t(0.2f, 0.0f, 0.0f),
        point_t(0.4f, 0.0f, 0.0f),
        point_t(0.6f, 0.0f, 0.0f),
        point_t(0.8f, 0.0f, 0.0f),
        point_t(0.0f, 0.2f, 0.0f),
        point_t(0.0f, 0.4f, 0.0f),
        point_t(0.0f, 0.6f, 0.0f),
        point_t(0.0f, 0.8f, 0.0f),
        point_t(1.0f, 0.0f, 0.0f),
        point_t(0.0f, 1.0f, 0.0f)
    });

    const dac_convex_hull uut(points);

    /* Checks */
    BOOST_REQUIRE(uut.vertices().size() == 3);
    BOOST_CHECK(uut.vertices()[0] == point_t(0.0f, 1.0f, 0.0f));
    BOOST_CHECK(uut.vertices()[1] == point_t(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(uut.vertices()[2] == point_t(1.0f, 0.0f, 0.0f));

    BOOST_REQUIRE(uut.faces().size() == 2);

    BOOST_REQUIRE(uut.faces()[0].size() == 3);
    BOOST_CHECK(uut.faces()[0][0] == 1);
    BOOST_CHECK(uut.faces()[0][1] == 2);
    BOOST_CHECK(uut.faces()[0][2] == 0);

    BOOST_REQUIRE(uut.faces()[1].size() == 3);
    BOOST_CHECK(uut.faces()[1][0] == 2);
    BOOST_CHECK(uut.faces()[1][1] == 1);
    BOOST_CHECK(uut.faces()[1][2] == 0);
}

BOOST_AUTO_TEST_CASE( octagon_ctor_test )
{
    /* Add points */
    const std::vector<point_t> points(
    {
        point_t(1.0f, 0.0f, 0.0f),
        point_t(0.0f, 1.0f, 0.0f),
        point_t(0.0f, 2.0f, 0.0f),
        point_t(1.0f, 3.0f, 0.0f),
        point_t(2.0f, 3.0f, 0.0f),
        point_t(3.0f, 2.0f, 0.0f),
        point_t(3.0f, 1.0f, 0.0f),
        point_t(2.0f, 0.0f, 0.0f)
    });

    const dac_convex_hull uut(points);

    /* Checks */
    BOOST_REQUIRE(uut.vertices().size() == 8);
    BOOST_CHECK(std::fabs(magnitude(uut.vertices()[0] - point_t(2.0f, 3.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(uut.vertices()[1] - point_t(3.0f, 2.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(uut.vertices()[2] - point_t(1.0f, 3.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(uut.vertices()[3] - point_t(3.0f, 1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(uut.vertices()[4] - point_t(0.0f, 2.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(uut.vertices()[5] - point_t(2.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(uut.vertices()[6] - point_t(0.0f, 1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(uut.vertices()[7] - point_t(1.0f, 0.0f, 0.0f))) < result_tolerance);

    BOOST_REQUIRE(uut.faces().size() == 2);

    BOOST_REQUIRE(uut.faces()[0].size() == 8);
    BOOST_CHECK(uut.faces()[0][0] == 1);
    BOOST_CHECK(uut.faces()[0][1] == 3);
    BOOST_CHECK(uut.faces()[0][2] == 5);
    BOOST_CHECK(uut.faces()[0][3] == 7);
    BOOST_CHECK(uut.faces()[0][4] == 6);
    BOOST_CHECK(uut.faces()[0][5] == 4);
    BOOST_CHECK(uut.faces()[0][6] == 2);
    BOOST_CHECK(uut.faces()[0][7] == 0);

    BOOST_REQUIRE(uut.faces()[1].size() == 8);
    BOOST_CHECK(uut.faces()[1][0] == 2);
    BOOST_CHECK(uut.faces()[1][1] == 4);
    BOOST_CHECK(uut.faces()[1][2] == 6);
    BOOST_CHECK(uut.faces()[1][3] == 7);
    BOOST_CHECK(uut.faces()[1][4] == 5);
    BOOST_CHECK(uut.faces()[1][5] == 3);
    BOOST_CHECK(uut.faces()[1][6] == 1);
    BOOST_CHECK(uut.faces()[1][7] == 0);
}

BOOST_AUTO_TEST_CASE( tetrahedron_ctor_test )
{
    /* Add points */
    const std::vector<point_t> points(
    {
        point_t(0.0f, 0.0f, 0.0f),
        point_t(1.0f, 0.0f, 0.0f),
        point_t(0.0f, 1.0f, 0.0f),
        point_t(0.0f, 0.0f, 1.0f)
    });

    const dac_convex_hull uut(points);

    /* Checks */
    BOOST_REQUIRE(uut.vertices().size() == 4);
    BOOST_CHECK(uut.vertices()[0] == point_t(0.0f, 1.0f, 0.0f));
    BOOST_CHECK(uut.vertices()[1] == point_t(1.0f, 0.0f, 0.0f));
    BOOST_CHECK(uut.vertices()[2] == point_t(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(uut.vertices()[3] == point_t(0.0f, 0.0f, 1.0f));

    BOOST_REQUIRE(uut.faces().size() == 4);

    BOOST_REQUIRE(uut.faces()[0].size() == 3);
    BOOST_CHECK(uut.faces()[0][0] == 1);
    BOOST_CHECK(uut.faces()[0][1] == 2);
    BOOST_CHECK(uut.faces()[0][2] == 0);

    BOOST_REQUIRE(uut.faces()[1].size() == 3);
    BOOST_CHECK(uut.faces()[1][0] == 2);
    BOOST_CHECK(uut.faces()[1][1] == 3);
    BOOST_CHECK(uut.faces()[1][2] == 0);

    BOOST_REQUIRE(uut.faces()[2].size() == 3);
    BOOST_CHECK(uut.faces()[2][0] == 3);
    BOOST_CHECK(uut.faces()[2][1] == 1);
    BOOST_CHECK(uut.faces()[2][2] == 0);

    BOOST_REQUIRE(uut.faces()[3].size() == 3);
    BOOST_CHECK(uut.faces()[3][0] == 3);
    BOOST_CHECK(uut.faces()[3][1] == 2);
    BOOST_CHECK(uut.faces()[3][2] == 1);
}

BOOST_AUTO_TEST_CASE( pyramid_ctor_test )
{
    /* Add points */
    const std::vector<point_t> points(
    {
        point_t(-0.5f, -0.5f,  0.0f),
        point_t( 0.5f, -0.5f,  0.0f),
        point_t(-0.5f,  0.5f,  0.0f),
        point_t( 0.5f,  0.5f,  0.0f),
        point_t( 0.0f,  0.0f,  1.0f)
    });

    const dac_convex_hull uut(points);

    /* Checks */
    BOOST_REQUIRE(uut.vertices().size() == 5);
    BOOST_CHECK(uut.vertices()[0] == point_t( 0.5f,  0.5f, 0.0f));
    BOOST_CHECK(uut.vertices()[1] == point_t( 0.5f, -0.5f, 0.0f));
    BOOST_CHECK(uut.vertices()[2] == point_t(-0.5f,  0.5f, 0.0f));
    BOOST_CHECK(uut.vertices()[3] == point_t( 0.0f,  0.0f, 1.0f));
    BOOST_CHECK(uut.vertices()[4] == point_t(-0.5f, -0.5f, 0.0f));

    BOOST_REQUIRE(uut.faces().size() == 5);

    BOOST_REQUIRE(uut.faces()[0].size() == 4);
    BOOST_CHECK(uut.faces()[0][0] == 1);
    BOOST_CHECK(uut.faces()[0][1] == 4);
    BOOST_CHECK(uut.faces()[0][2] == 2);
    BOOST_CHECK(uut.faces()[0][3] == 0);

    BOOST_REQUIRE(uut.faces()[1].size() == 3);
    BOOST_CHECK(uut.faces()[1][0] == 2);
    BOOST_CHECK(uut.faces()[1][1] == 3);
    BOOST_CHECK(uut.faces()[1][2] == 0);

    BOOST_REQUIRE(uut.faces()[2].size() == 3);
    BOOST_CHECK(uut.faces()[2][0] == 3);
    BOOST_CHECK(uut.faces()[2][1] == 1);
    BOOST_CHECK(uut.faces()[2][2] == 0);

    BOOST_REQUIRE(uut.faces()[3].size() == 3);
    BOOST_CHECK(uut.faces()[3][0] == 3);
    BOOST_CHECK(uut.faces()[3][1] == 4);
    BOOST_CHECK(uut.faces()[3][2] == 1);

    BOOST_REQUIRE(uut.faces()[4].size() == 3);
    BOOST_CHECK(uut.faces()[4][0] == 4);
    BOOST_CHECK(uut.faces()[4][1] == 3);
    BOOST_CHECK(uut.faces()[4][2] == 2);
}

BOOST_AUTO_TEST_CASE( triangular_prism_ctor_test )
{
    /* Add points */
    const std::vector<point_t> points(
    {
        point_t(-0.5f, -0.5f,  0.0f),
        point_t( 0.5f, -0.5f,  0.0f),
        point_t(-0.5f,  0.5f,  0.0f),
        point_t( 0.5f,  0.5f,  0.0f),
        point_t(-0.5f,  0.0f,  1.0f),
        point_t( 0.5f,  0.0f,  1.0f)
    });

    const dac_convex_hull uut(points);

    /* Checks */
    BOOST_REQUIRE(uut.vertices().size() == 6);
    BOOST_CHECK(uut.vertices()[0] == point_t( 0.5f,  0.5f, 0.0f));
    BOOST_CHECK(uut.vertices()[1] == point_t( 0.5f, -0.5f, 0.0f));
    BOOST_CHECK(uut.vertices()[2] == point_t(-0.5f,  0.5f, 0.0f));
    BOOST_CHECK(uut.vertices()[3] == point_t( 0.5f,  0.0f, 1.0f));
    BOOST_CHECK(uut.vertices()[4] == point_t(-0.5f, -0.5f, 0.0f));
    BOOST_CHECK(uut.vertices()[5] == point_t(-0.5f,  0.0f, 1.0f));

    BOOST_REQUIRE(uut.faces().size() == 5);

    BOOST_REQUIRE(uut.faces()[0].size() == 4);
    BOOST_CHECK(uut.faces()[0][0] == 1);
    BOOST_CHECK(uut.faces()[0][1] == 4);
    BOOST_CHECK(uut.faces()[0][2] == 2);
    BOOST_CHECK(uut.faces()[0][3] == 0);

    BOOST_REQUIRE(uut.faces()[1].size() == 4);
    BOOST_CHECK(uut.faces()[1][0] == 2);
    BOOST_CHECK(uut.faces()[1][1] == 5);
    BOOST_CHECK(uut.faces()[1][2] == 3);
    BOOST_CHECK(uut.faces()[1][3] == 0);

    BOOST_REQUIRE(uut.faces()[2].size() == 3);
    BOOST_CHECK(uut.faces()[2][0] == 3);
    BOOST_CHECK(uut.faces()[2][1] == 1);
    BOOST_CHECK(uut.faces()[2][2] == 0);

    BOOST_REQUIRE(uut.faces()[3].size() == 4);
    BOOST_CHECK(uut.faces()[3][0] == 3);
    BOOST_CHECK(uut.faces()[3][1] == 5);
    BOOST_CHECK(uut.faces()[3][2] == 4);
    BOOST_CHECK(uut.faces()[3][3] == 1);

    BOOST_REQUIRE(uut.faces()[4].size() == 3);
    BOOST_CHECK(uut.faces()[4][0] == 4);
    BOOST_CHECK(uut.faces()[4][1] == 5);
    BOOST_CHECK(uut.faces()[4][2] == 2);
}

BOOST_AUTO_TEST_CASE( octagon_pyramid_ctor_test )
{
    /* Add points */
    const std::vector<point_t> points(
    {
        point_t(1.0f, 0.0f, 0.0f),
        point_t(0.0f, 1.0f, 0.0f),
        point_t(0.0f, 2.0f, 0.0f),
        point_t(1.0f, 3.0f, 0.0f),
        point_t(2.0f, 3.0f, 0.0f),
        point_t(3.0f, 2.0f, 0.0f),
        point_t(3.0f, 1.0f, 0.0f),
        point_t(1.5f, 2.5f, 0.5f),
        point_t(2.0f, 0.0f, 0.0f)
    });

    const dac_convex_hull uut(points);

    /* Checks */
    BOOST_REQUIRE(uut.vertices().size() == 9);
    BOOST_CHECK(std::fabs(magnitude(uut.vertices()[0] - point_t(2.0f, 3.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(uut.vertices()[1] - point_t(3.0f, 2.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(uut.vertices()[2] - point_t(1.0f, 3.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(uut.vertices()[3] - point_t(1.5f, 2.5f, 0.5f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(uut.vertices()[4] - point_t(3.0f, 1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(uut.vertices()[5] - point_t(0.0f, 2.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(uut.vertices()[6] - point_t(0.0f, 1.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(uut.vertices()[7] - point_t(1.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(uut.vertices()[8] - point_t(2.0f, 0.0f, 0.0f))) < result_tolerance);

    BOOST_REQUIRE(uut.faces().size() == 9);

    BOOST_REQUIRE(uut.faces()[0].size() == 8);
    BOOST_CHECK(uut.faces()[0][0] == 1);
    BOOST_CHECK(uut.faces()[0][1] == 4);
    BOOST_CHECK(uut.faces()[0][2] == 8);
    BOOST_CHECK(uut.faces()[0][3] == 7);
    BOOST_CHECK(uut.faces()[0][4] == 6);
    BOOST_CHECK(uut.faces()[0][5] == 5);
    BOOST_CHECK(uut.faces()[0][6] == 2);
    BOOST_CHECK(uut.faces()[0][7] == 0);

    BOOST_REQUIRE(uut.faces()[1].size() == 3);
    BOOST_CHECK(uut.faces()[1][0] == 2);
    BOOST_CHECK(uut.faces()[1][1] == 3);
    BOOST_CHECK(uut.faces()[1][2] == 0);

    BOOST_REQUIRE(uut.faces()[2].size() == 3);
    BOOST_CHECK(uut.faces()[2][0] == 3);
    BOOST_CHECK(uut.faces()[2][1] == 1);
    BOOST_CHECK(uut.faces()[2][2] == 0);

    BOOST_REQUIRE(uut.faces()[3].size() == 3);
    BOOST_CHECK(uut.faces()[3][0] == 3);
    BOOST_CHECK(uut.faces()[3][1] == 4);
    BOOST_CHECK(uut.faces()[3][2] == 1);

    BOOST_REQUIRE(uut.faces()[4].size() == 3);
    BOOST_CHECK(uut.faces()[4][0] == 5);
    BOOST_CHECK(uut.faces()[4][1] == 3);
    BOOST_CHECK(uut.faces()[4][2] == 2);

    BOOST_REQUIRE(uut.faces()[4].size() == 3);
    BOOST_CHECK(uut.faces()[4][0] == 5);
    BOOST_CHECK(uut.faces()[4][1] == 3);
    BOOST_CHECK(uut.faces()[4][2] == 2);

    BOOST_REQUIRE(uut.faces()[5].size() == 3);
    BOOST_CHECK(uut.faces()[5][0] == 5);
    BOOST_CHECK(uut.faces()[5][1] == 6);
    BOOST_CHECK(uut.faces()[5][2] == 3);

    BOOST_REQUIRE(uut.faces()[6].size() == 3);
    BOOST_CHECK(uut.faces()[6][0] == 6);
    BOOST_CHECK(uut.faces()[6][1] == 7);
    BOOST_CHECK(uut.faces()[6][2] == 3);

    BOOST_REQUIRE(uut.faces()[7].size() == 3);
    BOOST_CHECK(uut.faces()[7][0] == 7);
    BOOST_CHECK(uut.faces()[7][1] == 8);
    BOOST_CHECK(uut.faces()[7][2] == 3);
}

BOOST_AUTO_TEST_CASE( cube_ctor_test )
{
    const std::vector<point_t> points(
    {
        point_t(-0.5f, -0.5f,  0.5f),
        point_t( 0.5f, -0.5f,  0.5f),
        point_t(-0.5f,  0.5f,  0.5f),
        point_t( 0.5f,  0.5f,  0.5f),
        point_t(-0.5f, -0.5f, -0.5f),
        point_t( 0.5f, -0.5f, -0.5f),
        point_t(-0.5f,  0.5f, -0.5f),
        point_t( 0.5f,  0.5f, -0.5f)
    });

    const dac_convex_hull uut(points);

    /* Checks */
    BOOST_REQUIRE(uut.vertices().size() == 8);
    BOOST_CHECK(uut.vertices()[0] == point_t( 0.5f,  0.5f,  0.5f));
    BOOST_CHECK(uut.vertices()[1] == point_t( 0.5f,  0.5f, -0.5f));
    BOOST_CHECK(uut.vertices()[2] == point_t(-0.5f,  0.5f,  0.5f));
    BOOST_CHECK(uut.vertices()[3] == point_t( 0.5f, -0.5f,  0.5f));
    BOOST_CHECK(uut.vertices()[4] == point_t( 0.5f, -0.5f, -0.5f));
    BOOST_CHECK(uut.vertices()[5] == point_t(-0.5f,  0.5f, -0.5f));
    BOOST_CHECK(uut.vertices()[6] == point_t(-0.5f, -0.5f,  0.5f));
    BOOST_CHECK(uut.vertices()[7] == point_t(-0.5f, -0.5f, -0.5f));

    BOOST_REQUIRE(uut.faces().size() == 6);

    BOOST_REQUIRE(uut.faces()[0].size() == 4);
    BOOST_CHECK(uut.faces()[0][0] == 1);
    BOOST_CHECK(uut.faces()[0][1] == 5);
    BOOST_CHECK(uut.faces()[0][2] == 2);
    BOOST_CHECK(uut.faces()[0][3] == 0);

    BOOST_REQUIRE(uut.faces()[1].size() == 4);
    BOOST_CHECK(uut.faces()[1][0] == 2);
    BOOST_CHECK(uut.faces()[1][1] == 6);
    BOOST_CHECK(uut.faces()[1][2] == 3);
    BOOST_CHECK(uut.faces()[1][3] == 0);

    BOOST_REQUIRE(uut.faces()[2].size() == 4);
    BOOST_CHECK(uut.faces()[2][0] == 3);
    BOOST_CHECK(uut.faces()[2][1] == 4);
    BOOST_CHECK(uut.faces()[2][2] == 1);
    BOOST_CHECK(uut.faces()[2][3] == 0);

    BOOST_REQUIRE(uut.faces()[3].size() == 4);
    BOOST_CHECK(uut.faces()[3][0] == 4);
    BOOST_CHECK(uut.faces()[3][1] == 7);
    BOOST_CHECK(uut.faces()[3][2] == 5);
    BOOST_CHECK(uut.faces()[3][3] == 1);

    BOOST_REQUIRE(uut.faces()[4].size() == 4);
    BOOST_CHECK(uut.faces()[4][0] == 5);
    BOOST_CHECK(uut.faces()[4][1] == 7);
    BOOST_CHECK(uut.faces()[4][2] == 6);
    BOOST_CHECK(uut.faces()[4][3] == 2);

    BOOST_REQUIRE(uut.faces()[5].size() == 4);
    BOOST_CHECK(uut.faces()[5][0] == 6);
    BOOST_CHECK(uut.faces()[5][1] == 7);
    BOOST_CHECK(uut.faces()[5][2] == 4);
    BOOST_CHECK(uut.faces()[5][3] == 3);
}

BOOST_AUTO_TEST_CASE( cube_points_twice_ctor_test )
{
    const std::vector<point_t> points(
    {
        point_t(-0.5f, -0.5f,  0.5f),
        point_t( 0.5f, -0.5f,  0.5f),
        point_t(-0.5f,  0.5f,  0.5f),
        point_t( 0.5f,  0.5f,  0.5f),
        point_t(-0.5f, -0.5f, -0.5f),
        point_t( 0.5f, -0.5f, -0.5f),
        point_t(-0.5f,  0.5f, -0.5f),
        point_t( 0.5f,  0.5f, -0.5f),
        point_t(-0.5f, -0.5f,  0.5f),
        point_t( 0.5f, -0.5f,  0.5f),
        point_t(-0.5f,  0.5f,  0.5f),
        point_t( 0.5f,  0.5f,  0.5f),
        point_t(-0.5f, -0.5f, -0.5f),
        point_t( 0.5f, -0.5f, -0.5f),
        point_t(-0.5f,  0.5f, -0.5f),
        point_t( 0.5f,  0.5f, -0.5f)
    });

    const dac_convex_hull uut(points);

    /* Checks */
    BOOST_REQUIRE(uut.vertices().size() == 8);
    BOOST_CHECK(uut.vertices()[0] == point_t( 0.5f,  0.5f,  0.5f));
    BOOST_CHECK(uut.vertices()[1] == point_t( 0.5f,  0.5f, -0.5f));
    BOOST_CHECK(uut.vertices()[2] == point_t(-0.5f,  0.5f,  0.5f));
    BOOST_CHECK(uut.vertices()[3] == point_t( 0.5f, -0.5f,  0.5f));
    BOOST_CHECK(uut.vertices()[4] == point_t( 0.5f, -0.5f, -0.5f));
    BOOST_CHECK(uut.vertices()[5] == point_t(-0.5f,  0.5f, -0.5f));
    BOOST_CHECK(uut.vertices()[6] == point_t(-0.5f, -0.5f,  0.5f));
    BOOST_CHECK(uut.vertices()[7] == point_t(-0.5f, -0.5f, -0.5f));

    BOOST_REQUIRE(uut.faces().size() == 6);

    BOOST_REQUIRE(uut.faces()[0].size() == 4);
    BOOST_CHECK(uut.faces()[0][0] == 1);
    BOOST_CHECK(uut.faces()[0][1] == 5);
    BOOST_CHECK(uut.faces()[0][2] == 2);
    BOOST_CHECK(uut.faces()[0][3] == 0);

    BOOST_REQUIRE(uut.faces()[1].size() == 4);
    BOOST_CHECK(uut.faces()[1][0] == 2);
    BOOST_CHECK(uut.faces()[1][1] == 6);
    BOOST_CHECK(uut.faces()[1][2] == 3);
    BOOST_CHECK(uut.faces()[1][3] == 0);

    BOOST_REQUIRE(uut.faces()[2].size() == 4);
    BOOST_CHECK(uut.faces()[2][0] == 3);
    BOOST_CHECK(uut.faces()[2][1] == 4);
    BOOST_CHECK(uut.faces()[2][2] == 1);
    BOOST_CHECK(uut.faces()[2][3] == 0);

    BOOST_REQUIRE(uut.faces()[3].size() == 4);
    BOOST_CHECK(uut.faces()[3][0] == 4);
    BOOST_CHECK(uut.faces()[3][1] == 7);
    BOOST_CHECK(uut.faces()[3][2] == 5);
    BOOST_CHECK(uut.faces()[3][3] == 1);

    BOOST_REQUIRE(uut.faces()[4].size() == 4);
    BOOST_CHECK(uut.faces()[4][0] == 5);
    BOOST_CHECK(uut.faces()[4][1] == 7);
    BOOST_CHECK(uut.faces()[4][2] == 6);
    BOOST_CHECK(uut.faces()[4][3] == 2);

    BOOST_REQUIRE(uut.faces()[5].size() == 4);
    BOOST_CHECK(uut.faces()[5][0] == 6);
    BOOST_CHECK(uut.faces()[5][1] == 7);
    BOOST_CHECK(uut.faces()[5][2] == 4);
    BOOST_CHECK(uut.faces()[5][3] == 3);
}

BOOST_AUTO_TEST_CASE( co_planar_face_cube_ctor_test )
{
    const std::vector<point_t> points(
    {
        point_t( 0.5f,  0.0f,  0.0f),
        point_t( 0.0f,  0.5f,  0.0f),
        point_t( 0.0f,  0.0f,  0.5f),
        point_t(-0.5f,  0.0f,  0.0f),
        point_t( 0.0f, -0.5f,  0.0f),
        point_t( 0.0f,  0.0f, -0.5f),
        point_t(-0.5f, -0.5f,  0.5f),
        point_t( 0.5f, -0.5f,  0.5f),
        point_t(-0.5f,  0.5f,  0.5f),
        point_t( 0.5f,  0.5f,  0.5f),
        point_t(-0.5f, -0.5f, -0.5f),
        point_t( 0.5f, -0.5f, -0.5f),
        point_t(-0.5f,  0.5f, -0.5f),
        point_t( 0.5f,  0.5f, -0.5f)
    });

    const dac_convex_hull uut(points);

    /* Checks */
    BOOST_REQUIRE(uut.vertices().size() == 8);
    BOOST_CHECK(uut.vertices()[0] == point_t( 0.5f,  0.5f,  0.5f));
    BOOST_CHECK(uut.vertices()[1] == point_t( 0.5f, -0.5f,  0.5f));
    BOOST_CHECK(uut.vertices()[2] == point_t( 0.5f,  0.5f, -0.5f));
    BOOST_CHECK(uut.vertices()[3] == point_t(-0.5f,  0.5f,  0.5f));
    BOOST_CHECK(uut.vertices()[4] == point_t( 0.5f, -0.5f, -0.5f));
    BOOST_CHECK(uut.vertices()[5] == point_t(-0.5f, -0.5f,  0.5f));
    BOOST_CHECK(uut.vertices()[6] == point_t(-0.5f,  0.5f, -0.5f));
    BOOST_CHECK(uut.vertices()[7] == point_t(-0.5f, -0.5f, -0.5f));

    BOOST_REQUIRE(uut.faces().size() == 6);

    BOOST_REQUIRE(uut.faces()[0].size() == 4);
    BOOST_CHECK(uut.faces()[0][0] == 1);
    BOOST_CHECK(uut.faces()[0][1] == 4);
    BOOST_CHECK(uut.faces()[0][2] == 2);
    BOOST_CHECK(uut.faces()[0][3] == 0);

    BOOST_REQUIRE(uut.faces()[1].size() == 4);
    BOOST_CHECK(uut.faces()[1][0] == 2);
    BOOST_CHECK(uut.faces()[1][1] == 6);
    BOOST_CHECK(uut.faces()[1][2] == 3);
    BOOST_CHECK(uut.faces()[1][3] == 0);

    BOOST_REQUIRE(uut.faces()[2].size() == 4);
    BOOST_CHECK(uut.faces()[2][0] == 3);
    BOOST_CHECK(uut.faces()[2][1] == 5);
    BOOST_CHECK(uut.faces()[2][2] == 1);
    BOOST_CHECK(uut.faces()[2][3] == 0);

    BOOST_REQUIRE(uut.faces()[3].size() == 4);
    BOOST_CHECK(uut.faces()[3][0] == 5);
    BOOST_CHECK(uut.faces()[3][1] == 7);
    BOOST_CHECK(uut.faces()[3][2] == 4);
    BOOST_CHECK(uut.faces()[3][3] == 1);

    BOOST_REQUIRE(uut.faces()[4].size() == 4);
    BOOST_CHECK(uut.faces()[4][0] == 4);
    BOOST_CHECK(uut.faces()[4][1] == 7);
    BOOST_CHECK(uut.faces()[4][2] == 6);
    BOOST_CHECK(uut.faces()[4][3] == 2);

    BOOST_REQUIRE(uut.faces()[5].size() == 4);
    BOOST_CHECK(uut.faces()[5][0] == 6);
    BOOST_CHECK(uut.faces()[5][1] == 7);
    BOOST_CHECK(uut.faces()[5][2] == 5);
    BOOST_CHECK(uut.faces()[5][3] == 3);
}

BOOST_AUTO_TEST_CASE( cube_points_inside_ctor_test )
{
    /* Add points */
    const std::vector<point_t> points(
    {
        point_t(-0.5f, -0.5f,  0.5f),
        point_t( 0.0f,  0.0f,  0.0f),
        point_t( 0.5f, -0.5f,  0.5f),
        point_t(-0.5f,  0.5f,  0.5f),
        point_t( 0.0f,  0.0f,  0.0f),
        point_t( 0.5f,  0.5f,  0.5f),
        point_t(-0.5f, -0.5f, -0.5f),
        point_t( 0.0f,  0.0f,  0.0f),
        point_t( 0.5f, -0.5f, -0.5f),
        point_t(-0.5f,  0.5f, -0.5f),
        point_t( 0.5f,  0.5f, -0.5f)
    });

    const dac_convex_hull uut(points);

    /* Checks */
    BOOST_REQUIRE(uut.vertices().size() == 8);
    BOOST_CHECK(uut.vertices()[0] == point_t( 0.5f,  0.5f,  0.5f));
    BOOST_CHECK(uut.vertices()[1] == point_t( 0.5f, -0.5f,  0.5f));
    BOOST_CHECK(uut.vertices()[2] == point_t( 0.5f,  0.5f, -0.5f));
    BOOST_CHECK(uut.vertices()[3] == point_t(-0.5f,  0.5f,  0.5f));
    BOOST_CHECK(uut.vertices()[4] == point_t( 0.5f, -0.5f, -0.5f));
    BOOST_CHECK(uut.vertices()[5] == point_t(-0.5f, -0.5f,  0.5f));
    BOOST_CHECK(uut.vertices()[6] == point_t(-0.5f,  0.5f, -0.5f));
    BOOST_CHECK(uut.vertices()[7] == point_t(-0.5f, -0.5f, -0.5f));

    BOOST_REQUIRE(uut.faces().size() == 6);

    BOOST_REQUIRE(uut.faces()[0].size() == 4);
    BOOST_CHECK(uut.faces()[0][0] == 1);
    BOOST_CHECK(uut.faces()[0][1] == 4);
    BOOST_CHECK(uut.faces()[0][2] == 2);
    BOOST_CHECK(uut.faces()[0][3] == 0);

    BOOST_REQUIRE(uut.faces()[1].size() == 4);
    BOOST_CHECK(uut.faces()[1][0] == 2);
    BOOST_CHECK(uut.faces()[1][1] == 6);
    BOOST_CHECK(uut.faces()[1][2] == 3);
    BOOST_CHECK(uut.faces()[1][3] == 0);

    BOOST_REQUIRE(uut.faces()[2].size() == 4);
    BOOST_CHECK(uut.faces()[2][0] == 3);
    BOOST_CHECK(uut.faces()[2][1] == 5);
    BOOST_CHECK(uut.faces()[2][2] == 1);
    BOOST_CHECK(uut.faces()[2][3] == 0);

    BOOST_REQUIRE(uut.faces()[3].size() == 4);
    BOOST_CHECK(uut.faces()[3][0] == 5);
    BOOST_CHECK(uut.faces()[3][1] == 7);
    BOOST_CHECK(uut.faces()[3][2] == 4);
    BOOST_CHECK(uut.faces()[3][3] == 1);

    BOOST_REQUIRE(uut.faces()[4].size() == 4);
    BOOST_CHECK(uut.faces()[4][0] == 4);
    BOOST_CHECK(uut.faces()[4][1] == 7);
    BOOST_CHECK(uut.faces()[4][2] == 6);
    BOOST_CHECK(uut.faces()[4][3] == 2);

    BOOST_REQUIRE(uut.faces()[5].size() == 4);
    BOOST_CHECK(uut.faces()[5][0] == 6);
    BOOST_CHECK(uut.faces()[5][1] == 7);
    BOOST_CHECK(uut.faces()[5][2] == 5);
    BOOST_CHECK(uut.faces()[5][3] == 3);
}

BOOST_AUTO_TEST_CASE( cube_pair_ctor_test )
{
    const std::vector<point_t> points(
    {
        point_t(-0.5f, -0.5f,  0.5f),
        point_t( 0.5f, -0.5f,  0.5f),
        point_t(-0.5f,  0.5f,  0.5f),
        point_t( 0.5f,  0.5f,  0.5f),
        point_t(-0.5f, -0.5f, -0.5f),
        point_t( 0.5f, -0.5f, -0.5f),
        point_t(-0.5f,  0.5f, -0.5f),
        point_t( 0.5f,  0.5f, -0.5f),

        point_t( 2.5f,  2.5f,  3.5f),
        point_t( 3.5f,  2.5f,  3.5f),
        point_t( 2.5f,  3.5f,  3.5f),
        point_t( 3.5f,  3.5f,  3.5f),
        point_t( 2.5f,  2.5f,  2.5f),
        point_t( 3.5f,  2.5f,  2.5f),
        point_t( 2.5f,  3.5f,  2.5f),
        point_t( 3.5f,  3.5f,  2.5f)
    });

    const dac_convex_hull uut(points);

    /* Checks */
    BOOST_REQUIRE_MESSAGE(uut.vertices().size() == 14, uut.vertices().size());
    BOOST_CHECK(uut.vertices()[ 0] == point_t( 3.5f,  3.5f,  3.5f));
    BOOST_CHECK(uut.vertices()[ 1] == point_t( 3.5f,  3.5f,  2.5f));
    BOOST_CHECK(uut.vertices()[ 2] == point_t( 2.5f,  3.5f,  3.5f));
    BOOST_CHECK(uut.vertices()[ 3] == point_t( 3.5f,  2.5f,  3.5f));
    BOOST_CHECK(uut.vertices()[ 4] == point_t( 3.5f,  2.5f,  2.5f));
    BOOST_CHECK(uut.vertices()[ 5] == point_t( 0.5f,  0.5f, -0.5f));
    BOOST_CHECK(uut.vertices()[ 6] == point_t( 2.5f,  3.5f,  2.5f));
    BOOST_CHECK(uut.vertices()[ 7] == point_t(-0.5f,  0.5f,  0.5f));
    BOOST_CHECK(uut.vertices()[ 8] == point_t( 2.5f,  2.5f,  3.5f));
    BOOST_CHECK(uut.vertices()[ 9] == point_t( 0.5f, -0.5f,  0.5f));
    BOOST_CHECK(uut.vertices()[10] == point_t( 0.5f, -0.5f, -0.5f));
    BOOST_CHECK(uut.vertices()[11] == point_t(-0.5f,  0.5f, -0.5f));
    BOOST_CHECK(uut.vertices()[12] == point_t(-0.5f, -0.5f,  0.5f));
    BOOST_CHECK(uut.vertices()[13] == point_t(-0.5f, -0.5f, -0.5f));

    BOOST_REQUIRE_MESSAGE(uut.faces().size() == 12, uut.faces().size());

    BOOST_REQUIRE(uut.faces()[0].size() == 4);
    BOOST_CHECK(uut.faces()[0][0] ==  1);
    BOOST_CHECK(uut.faces()[0][1] ==  6);
    BOOST_CHECK(uut.faces()[0][2] ==  2);
    BOOST_CHECK(uut.faces()[0][3] ==  0);

    BOOST_REQUIRE(uut.faces()[1].size() == 4);
    BOOST_CHECK(uut.faces()[1][0] ==  2);
    BOOST_CHECK(uut.faces()[1][1] ==  8);
    BOOST_CHECK(uut.faces()[1][2] ==  3);
    BOOST_CHECK(uut.faces()[1][3] ==  0);

    BOOST_REQUIRE(uut.faces()[2].size() == 4);
    BOOST_CHECK(uut.faces()[2][0] ==  3);
    BOOST_CHECK(uut.faces()[2][1] ==  4);
    BOOST_CHECK(uut.faces()[2][2] ==  1);
    BOOST_CHECK(uut.faces()[2][3] ==  0);

    BOOST_REQUIRE(uut.faces()[3].size() == 4);
    BOOST_CHECK(uut.faces()[3][0] ==  4);
    BOOST_CHECK(uut.faces()[3][1] == 10);
    BOOST_CHECK(uut.faces()[3][2] ==  5);
    BOOST_CHECK(uut.faces()[3][3] ==  1);

    BOOST_REQUIRE(uut.faces()[4].size() == 4);
    BOOST_CHECK(uut.faces()[4][0] ==  5);
    BOOST_CHECK(uut.faces()[4][1] == 11);
    BOOST_CHECK(uut.faces()[4][2] ==  6);
    BOOST_CHECK(uut.faces()[4][3] ==  1);

    BOOST_REQUIRE(uut.faces()[5].size() == 4);
    BOOST_CHECK(uut.faces()[5][0] ==  6);
    BOOST_CHECK(uut.faces()[5][1] == 11);
    BOOST_CHECK(uut.faces()[5][2] ==  7);
    BOOST_CHECK(uut.faces()[5][3] ==  2);

    BOOST_REQUIRE(uut.faces()[6].size() == 4);
    BOOST_CHECK(uut.faces()[6][0] ==  7);
    BOOST_CHECK(uut.faces()[6][1] == 12);
    BOOST_CHECK(uut.faces()[6][2] ==  8);
    BOOST_CHECK(uut.faces()[6][3] ==  2);

    BOOST_REQUIRE(uut.faces()[7].size() == 4);
    BOOST_CHECK(uut.faces()[7][0] ==  8);
    BOOST_CHECK(uut.faces()[7][1] == 12);
    BOOST_CHECK(uut.faces()[7][2] ==  9);
    BOOST_CHECK(uut.faces()[7][3] ==  3);

    BOOST_REQUIRE(uut.faces()[8].size() == 4);
    BOOST_CHECK(uut.faces()[8][0] ==  9);
    BOOST_CHECK(uut.faces()[8][1] == 10);
    BOOST_CHECK(uut.faces()[8][2] ==  4);
    BOOST_CHECK(uut.faces()[8][3] ==  3);

    BOOST_REQUIRE(uut.faces()[9].size() == 4);
    BOOST_CHECK(uut.faces()[9][0] == 10);
    BOOST_CHECK(uut.faces()[9][1] == 13);
    BOOST_CHECK(uut.faces()[9][2] == 11);
    BOOST_CHECK(uut.faces()[9][3] ==  5);

    BOOST_REQUIRE(uut.faces()[10].size() == 4);
    BOOST_CHECK(uut.faces()[10][0] == 11);
    BOOST_CHECK(uut.faces()[10][1] == 13);
    BOOST_CHECK(uut.faces()[10][2] == 12);
    BOOST_CHECK(uut.faces()[10][3] ==  7);

    BOOST_REQUIRE(uut.faces()[11].size() == 4);
    BOOST_CHECK(uut.faces()[11][0] == 12);
    BOOST_CHECK(uut.faces()[11][1] == 13);
    BOOST_CHECK(uut.faces()[11][2] == 10);
    BOOST_CHECK(uut.faces()[11][3] ==  9);
}

BOOST_AUTO_TEST_CASE( tetrahedron_triple_ctor_test )
{
    /* Add points */
    const std::vector<point_t> points(
    {
        point_t( 0.5f, -0.5f, -0.5f),
        point_t(-0.5f,  0.5f, -0.5f),
        point_t( 0.5f,  0.5f,  0.5f),
        point_t(-0.5f, -0.5f,  0.5f),
        point_t( 5.5f,  0.5f, -0.5f),
        point_t( 4.5f,  0.5f, -0.5f),
        point_t( 5.5f,  0.5f,  0.5f),
        point_t( 1.5f, -0.5f,  0.5f),
        point_t( 1.5f, -0.5f, -0.5f),
        point_t( 1.5f,  0.5f,  0.5f),
        point_t( 0.5f, -0.5f,  0.5f),
        point_t( 5.5f, -0.5f, -0.5f)
    });

    const dac_convex_hull uut(points);

    /* Checks */
    BOOST_REQUIRE(uut.vertices().size() == 8);
    BOOST_CHECK(std::fabs(magnitude(uut.vertices()[0] - point_t( 5.5f,  0.5f,  0.5f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(uut.vertices()[1] - point_t( 1.5f, -0.5f,  0.5f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(uut.vertices()[2] - point_t( 5.5f, -0.5f, -0.5f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(uut.vertices()[3] - point_t( 5.5f,  0.5f, -0.5f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(uut.vertices()[4] - point_t( 0.5f,  0.5f,  0.5f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(uut.vertices()[5] - point_t(-0.5f, -0.5f,  0.5f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(uut.vertices()[6] - point_t( 0.5f, -0.5f, -0.5f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(uut.vertices()[7] - point_t(-0.5f,  0.5f, -0.5f))) < result_tolerance);

    BOOST_REQUIRE(uut.faces().size() == 8);

    BOOST_REQUIRE(uut.faces()[0].size() == 3);
    BOOST_CHECK(uut.faces()[0][0] == 1);
    BOOST_CHECK(uut.faces()[0][1] == 2);
    BOOST_CHECK(uut.faces()[0][2] == 0);
    
    BOOST_REQUIRE(uut.faces()[1].size() == 3);
    BOOST_CHECK(uut.faces()[1][0] == 2);
    BOOST_CHECK(uut.faces()[1][1] == 3);
    BOOST_CHECK(uut.faces()[1][2] == 0);

    BOOST_REQUIRE(uut.faces()[2].size() == 4);
    BOOST_CHECK(uut.faces()[2][0] == 3);
    BOOST_CHECK(uut.faces()[2][1] == 7);
    BOOST_CHECK(uut.faces()[2][2] == 4);
    BOOST_CHECK(uut.faces()[2][3] == 0);

    BOOST_REQUIRE(uut.faces()[3].size() == 4);
    BOOST_CHECK(uut.faces()[3][0] == 4);
    BOOST_CHECK(uut.faces()[3][1] == 5);
    BOOST_CHECK(uut.faces()[3][2] == 1);
    BOOST_CHECK(uut.faces()[3][3] == 0);

    BOOST_REQUIRE(uut.faces()[4].size() == 4);
    BOOST_CHECK(uut.faces()[4][0] == 5);
    BOOST_CHECK(uut.faces()[4][1] == 6);
    BOOST_CHECK(uut.faces()[4][2] == 2);
    BOOST_CHECK(uut.faces()[4][3] == 1);

    BOOST_REQUIRE(uut.faces()[5].size() == 4);
    BOOST_CHECK(uut.faces()[5][0] == 6);
    BOOST_CHECK(uut.faces()[5][1] == 7);
    BOOST_CHECK(uut.faces()[5][2] == 3);
    BOOST_CHECK(uut.faces()[5][3] == 2);

    BOOST_REQUIRE(uut.faces()[6].size() == 3);
    BOOST_CHECK(uut.faces()[6][0] == 7);
    BOOST_CHECK(uut.faces()[6][1] == 5);
    BOOST_CHECK(uut.faces()[6][2] == 4);

    BOOST_REQUIRE(uut.faces()[7].size() == 3);
    BOOST_CHECK(uut.faces()[7][0] == 7);
    BOOST_CHECK(uut.faces()[7][1] == 6);
    BOOST_CHECK(uut.faces()[7][2] == 5);
}

/* Test volume */
BOOST_AUTO_TEST_CASE( tetrahedron_volume_test )
{
    /* Add points */
    const std::vector<point_t> points(
    {
        point_t(0.0f, 0.0f, 0.0f),
        point_t(1.0f, 0.0f, 0.0f),
        point_t(0.0f, 1.0f, 0.0f),
        point_t(0.0f, 0.0f, 1.0f)
    });

    const dac_convex_hull uut(points);

    /* Checks */
    BOOST_CHECK_CLOSE(uut.volume(), 0.1666667f, result_tolerance);
}

BOOST_AUTO_TEST_CASE( cube_volume_test )
{
    const std::vector<point_t> points(
    {
        point_t(-0.5f, -0.5f,  0.5f),
        point_t( 0.5f, -0.5f,  0.5f),
        point_t(-0.5f,  0.5f,  0.5f),
        point_t( 0.5f,  0.5f,  0.5f),
        point_t(-0.5f, -0.5f, -0.5f),
        point_t( 0.5f, -0.5f, -0.5f),
        point_t(-0.5f,  0.5f, -0.5f),
        point_t( 0.5f,  0.5f, -0.5f)
    });

    const dac_convex_hull uut(points);

    /* Checks */
    BOOST_CHECK_MESSAGE(uut.volume() == 1.0f, uut.volume());
}

BOOST_AUTO_TEST_CASE( octagon_pyramid_volume_test )
{
    /* Add points */
    const std::vector<point_t> points(
    {
        point_t(1.0f, 0.0f, 0.0f),
        point_t(0.0f, 1.0f, 0.0f),
        point_t(0.0f, 2.0f, 0.0f),
        point_t(1.0f, 3.0f, 0.0f),
        point_t(2.0f, 3.0f, 0.0f),
        point_t(3.0f, 2.0f, 0.0f),
        point_t(3.0f, 1.0f, 0.0f),
        point_t(1.5f, 2.5f, 0.5f),
        point_t(2.0f, 0.0f, 0.0f)
    });

    const dac_convex_hull uut(points);

    /* Checks */
    BOOST_CHECK_CLOSE(uut.volume(), 1.16654f, result_tolerance);
}

// BOOST_AUTO_TEST_CASE( performance_ctor_test )
// {
//     const int number_runs   = 100;
//     const int number_points = 100000;

//     /* Memory to hold points */
//     std::vector<point_t> points;
//     points.reserve(number_points);

//     /* Generate points */
//     std::default_random_engine gen;
//     std::uniform_real_distribution<float> dist(-100.0f, 100.0f);
//     for (int i = 0; i< number_points; ++i)
//     {
//         points.emplace_back(dist(gen), dist(gen), dist(gen));
//     }

//     /* Build */
//     const auto t0(std::chrono::system_clock::now());
//     for (int i = 0; i < number_runs; ++i)
//     {
//         dac_convex_hull uut(points);
//         BOOST_REQUIRE(!uut.faces().empty());
//     }
//     const auto t1(std::chrono::system_clock::now());
//     std::cout << "Test took: " << (std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count() / number_runs) << " ms per hull" << std::endl;
// }

BOOST_AUTO_TEST_SUITE_END()
}; /* namespace test */
}; /* namespace raptor_convex_decomposition */
