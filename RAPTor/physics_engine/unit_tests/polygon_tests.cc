#ifdef STAND_ALONE
#define BOOST_TEST_MODULE polygon test

/* Common headers */
#include "logging.h"

/* Initialise logger */
const raptor_physics::init_logger init_logger;
#endif /* #ifdef STAND_ALONE */


/* Standard headers */
#include <list>
#include <memory>
#include <vector>

/* Boost headers */
#include "boost/test/unit_test.hpp"

/* Common headers */
#include "common.h"
#include "point_t.h"

/* Physics headers */
#include "polygon.h"


namespace raptor_physics
{
namespace test
{
struct polygon_fixture
{
    polygon_fixture() : 
        pts{ point_t<>( 1.5f,  1.5f, 1.0f), point_t<>( 1.5f, -0.5f, 1.0f), point_t<>(-0.5f, -0.5f, 1.0f), point_t<>(-0.5f,  1.5f, 1.0f) },
        square_poly(&pts, { 0, 1, 2, 3 })
    {  }

    std::vector<point_t<>>  pts;
    polygon                 square_poly;
};

struct world_polygon_fixture
{
    world_polygon_fixture() : 
        uut({  point_t<>( 1.5f,  1.5f, 1.0f), point_t<>( 1.5f, -0.5f, 1.0f), point_t<>(-0.5f, -0.5f, 1.0f), point_t<>(-0.5f,  1.5f,  1.0f),
               point_t<>( 1.5f,  3.5f, 1.0f), point_t<>(-2.5f, -0.5f, 1.0f), point_t<>(-0.5f, -0.5f, 6.0f), point_t<>(-0.5f,  1.5f, -3.0f) })
    {  }

    world_polygon   uut;
};

BOOST_AUTO_TEST_SUITE( polygon_tests )

const float result_tolerance = 0.0001;

BOOST_FIXTURE_TEST_CASE( polygon_to_world_polygon_test, polygon_fixture )
{
    auto wp(square_poly.to_world_polygon(quaternion_t(1.0f, 0.0f, 0.0f, 0.0f), point_t<>(0.0f, 0.0f, 0.0f)));

    /* Checks */
    BOOST_REQUIRE(wp.number_of_vertices() == 4);
    BOOST_CHECK(wp.vertex(0) == point_t<>( 1.5f,  1.5f, 1.0f));
    BOOST_CHECK(wp.vertex(1) == point_t<>( 1.5f, -0.5f, 1.0f));
    BOOST_CHECK(wp.vertex(2) == point_t<>(-0.5f, -0.5f, 1.0f));
    BOOST_CHECK(wp.vertex(3) == point_t<>(-0.5f,  1.5f, 1.0f));
}

BOOST_FIXTURE_TEST_CASE( polygon_to_world_polygon_with_translate_test, polygon_fixture )
{
    auto wp(square_poly.to_world_polygon(quaternion_t(1.0f, 0.0f, 0.0f, 0.0f), point_t<>(-2.0f, 1.0f, 3.0f)));

    /* Checks */
    BOOST_REQUIRE(wp.number_of_vertices() == 4);
    BOOST_CHECK(wp.vertex(0) == point_t<>(-0.5f, 2.5f, 4.0f));
    BOOST_CHECK(wp.vertex(1) == point_t<>(-0.5f, 0.5f, 4.0f));
    BOOST_CHECK(wp.vertex(2) == point_t<>(-2.5f, 0.5f, 4.0f));
    BOOST_CHECK(wp.vertex(3) == point_t<>(-2.5f, 2.5f, 4.0f));
}

BOOST_FIXTURE_TEST_CASE( polygon_to_world_polygon_with_rotate_test, polygon_fixture )
{
    auto wp0(square_poly.to_world_polygon(quaternion_t(1.0f / std::sqrt(2.0f), 1.0f / std::sqrt(2.0f), 0.0f, 0.0f), point_t<>(0.0f, 0.0f, 0.0f)));

    /* Checks */
    BOOST_REQUIRE(wp0.number_of_vertices() == 4);
    BOOST_CHECK(fabs(magnitude(wp0.vertex(0) - point_t<>( 1.5f, -1.0f,  1.5f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(wp0.vertex(1) - point_t<>( 1.5f, -1.0f, -0.5f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(wp0.vertex(2) - point_t<>(-0.5f, -1.0f, -0.5f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(wp0.vertex(3) - point_t<>(-0.5f, -1.0f,  1.5f))) < result_tolerance);

    auto wp1(square_poly.to_world_polygon(quaternion_t(1.0f / std::sqrt(2.0f), -1.0f / std::sqrt(2.0f), 0.0f, 0.0f), point_t<>(0.0f, 0.0f, 0.0f)));

    /* Checks */
    BOOST_REQUIRE(wp1.number_of_vertices() == 4);
    BOOST_CHECK(fabs(magnitude(wp1.vertex(0) - point_t<>( 1.5f, 1.0f, -1.5f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(wp1.vertex(1) - point_t<>( 1.5f, 1.0f,  0.5f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(wp1.vertex(2) - point_t<>(-0.5f, 1.0f,  0.5f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(wp1.vertex(3) - point_t<>(-0.5f, 1.0f, -1.5f))) < result_tolerance);
}

BOOST_FIXTURE_TEST_CASE( polygon_to_world_polygon_with_translate_and_rotate_test, polygon_fixture )
{
    auto wp0(square_poly.to_world_polygon(quaternion_t(1.0f / std::sqrt(2.0f), 1.0f / std::sqrt(2.0f), 0.0f, 0.0f), point_t<>(1.0f, -2.0f, 3.0f)));

    /* Checks */
    BOOST_REQUIRE(wp0.number_of_vertices() == 4);
    BOOST_CHECK(fabs(magnitude(wp0.vertex(0) - point_t<>(2.5f, -3.0f, 4.5f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(wp0.vertex(1) - point_t<>(2.5f, -3.0f, 2.5f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(wp0.vertex(2) - point_t<>(0.5f, -3.0f, 2.5f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(wp0.vertex(3) - point_t<>(0.5f, -3.0f, 4.5f))) < result_tolerance);

    auto wp1(square_poly.to_world_polygon(quaternion_t(1.0f / std::sqrt(2.0f), -1.0f / std::sqrt(2.0f), 0.0f, 0.0f), point_t<>(-1.0f, -1.0f, 3.0f)));

    /* Checks */
    BOOST_REQUIRE(wp1.number_of_vertices() == 4);
    BOOST_CHECK(fabs(magnitude(wp1.vertex(0) - point_t<>( 0.5f, 0.0f, 1.5f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(wp1.vertex(1) - point_t<>( 0.5f, 0.0f, 3.5f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(wp1.vertex(2) - point_t<>(-1.5f, 0.0f, 3.5f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(wp1.vertex(3) - point_t<>(-1.5f, 0.0f, 1.5f))) < result_tolerance);
}

BOOST_FIXTURE_TEST_CASE( polygon_contains_vertices_test, polygon_fixture )
{
    polygon uut(&pts, { 0, 2, 4, 6, 8, 3, 5, 9 });

    /* Single value tests */
    int contain1_0[] = { 1 };
    int contain1_1[] = { 3 };
    BOOST_CHECK(!uut.contains_vertices(&contain1_0[0], &contain1_0[1]));
    BOOST_CHECK( uut.contains_vertices(&contain1_1[0], &contain1_1[1]));

    /* 2 value tests */
    int contain2_0[] = {  3, 10 };
    int contain2_1[] = { 10,  3 };
    int contain2_2[] = {  3,  4 };
    int contain2_3[] = {  4,  3 };
    BOOST_CHECK(!uut.contains_vertices(&contain2_0[0], &contain2_0[2]));
    BOOST_CHECK(!uut.contains_vertices(&contain2_1[0], &contain2_1[2]));
    BOOST_CHECK( uut.contains_vertices(&contain2_2[0], &contain2_2[2]));
    BOOST_CHECK( uut.contains_vertices(&contain2_3[0], &contain2_3[2]));

    /* 3 value tests */
    int contain3_0[] = { 7, 4, 3 };
    int contain3_1[] = { 4, 3, 7 };
    int contain3_2[] = { 4, 3, 9 };
    int contain3_3[] = { 4, 3, 3 };
    BOOST_CHECK(!uut.contains_vertices(&contain3_0[0], &contain3_0[3]));
    BOOST_CHECK(!uut.contains_vertices(&contain3_1[0], &contain3_1[3]));
    BOOST_CHECK( uut.contains_vertices(&contain3_2[0], &contain3_2[3]));
    BOOST_CHECK( uut.contains_vertices(&contain3_3[0], &contain3_3[3]));
}

BOOST_FIXTURE_TEST_CASE( polygon_normal_test, polygon_fixture )
{
    polygon rev_square_poly(&pts, { 3, 2, 1, 0 });
    BOOST_CHECK(square_poly.normal()     == point_t<>(0.0f, 0.0f, -1.0f));
    BOOST_CHECK(rev_square_poly.normal() == point_t<>(0.0f, 0.0f,  1.0f));
}

BOOST_AUTO_TEST_CASE( polygon_to_triangles_line_test )
{
    std::vector<point_t<>> pts{ point_t<>( 1.5f,  1.5f, 1.0f), point_t<>( 1.5f, -0.5f, 1.0f) };
    polygon uut(&pts, { 0, 1 });

    std::vector<int> tris;
    uut.to_triangles(&tris);

    /* Checks */
    BOOST_REQUIRE(tris.size() == 0);
}

BOOST_AUTO_TEST_CASE( polygon_to_triangles_triangle_test )
{
    std::vector<point_t<>> pts{ point_t<>( 1.5f,  1.5f, 1.0f), point_t<>( 1.5f, -0.5f, 1.0f), point_t<>(-0.5f, -0.5f, 1.0f) };
    polygon uut(&pts, { 0, 1, 2 });

    std::vector<int> tris;
    uut.to_triangles(&tris);

    /* Checks */
    BOOST_REQUIRE(tris.size() == 3);
    BOOST_CHECK(tris[0] == 2);
    BOOST_CHECK(tris[1] == 0);
    BOOST_CHECK(tris[2] == 1);
}

BOOST_AUTO_TEST_CASE( polygon_to_triangles_square_test )
{
    std::vector<point_t<>> pts{ point_t<>( 1.5f,  1.5f, 1.0f), point_t<>( 1.5f, -0.5f, 1.0f), point_t<>(-0.5f, -0.5f, 1.0f), point_t<>(-0.5f,  1.5f, 1.0f) };
    polygon uut(&pts, { 0, 1, 2, 3 });

    std::vector<int> tris;
    uut.to_triangles(&tris);

    /* Checks */
    BOOST_REQUIRE(tris.size() == 6);
    BOOST_CHECK(tris[0] == 3);
    BOOST_CHECK(tris[1] == 0);
    BOOST_CHECK(tris[2] == 1);
    BOOST_CHECK(tris[3] == 3);
    BOOST_CHECK(tris[4] == 1);
    BOOST_CHECK(tris[5] == 2);
}

BOOST_AUTO_TEST_CASE( polygon_to_triangles_obvious_extreme_test )
{
    std::vector<point_t<>> pts{ point_t<>( 1.5f,  1.5f, 1.0f), point_t<>( 1.5f, -0.5f, 1.0f), point_t<>(-0.5f, -0.5f, 1.0f), point_t<>(-0.5f,  7.5f, 1.0f) };
    polygon uut(&pts, { 0, 1, 2, 3 });

    std::vector<int> tris;
    uut.to_triangles(&tris);

    /* Checks */
    BOOST_REQUIRE(tris.size() == 6);
    BOOST_CHECK(tris[0] == 2);
    BOOST_CHECK(tris[1] == 3);
    BOOST_CHECK(tris[2] == 0);
    BOOST_CHECK(tris[3] == 2);
    BOOST_CHECK(tris[4] == 0);
    BOOST_CHECK(tris[5] == 1);
}

BOOST_AUTO_TEST_CASE( polygon_to_triangles_octogon_test )
{
    std::vector<point_t<>> pts{   point_t<>(0.0f, 1.0f, 0.0f), point_t<>(0.0f, 2.0f, 0.0f), point_t<>(1.0f, 3.0f, 0.0f), point_t<>(2.0f, 3.0f, 0.0f),
                                point_t<>(3.0f, 2.0f, 0.0f), point_t<>(3.0f, 1.0f, 0.0f), point_t<>(2.0f, 0.0f, 0.0f), point_t<>(1.0f, 0.0f, 0.0f) };
    polygon uut(&pts, { 0, 1, 2, 3, 4, 5, 6, 7 });

    std::vector<int> tris;
    uut.to_triangles(&tris);

    /* Checks */
    BOOST_REQUIRE(tris.size() == 18);
    BOOST_CHECK(tris[ 0] == 7);
    BOOST_CHECK(tris[ 1] == 0);
    BOOST_CHECK(tris[ 2] == 1);
    BOOST_CHECK(tris[ 3] == 7);
    BOOST_CHECK(tris[ 4] == 1);
    BOOST_CHECK(tris[ 5] == 2);
    BOOST_CHECK(tris[ 6] == 7);
    BOOST_CHECK(tris[ 7] == 2);
    BOOST_CHECK(tris[ 8] == 3);
    BOOST_CHECK(tris[ 9] == 7);
    BOOST_CHECK(tris[10] == 3);
    BOOST_CHECK(tris[11] == 4);
    BOOST_CHECK(tris[12] == 7);
    BOOST_CHECK(tris[13] == 4);
    BOOST_CHECK(tris[14] == 5);
    BOOST_CHECK(tris[15] == 7);
    BOOST_CHECK(tris[16] == 5);
    BOOST_CHECK(tris[17] == 6);
}

BOOST_AUTO_TEST_CASE( polygon_to_triangles_e_test )
{
    std::vector<point_t<>> pts{   point_t<>(0.0f, 0.0f, 0.0f), point_t<>(0.0f, 5.0f, 0.0f), point_t<>(2.0f, 5.0f, 0.0f), point_t<>(2.0f, 4.0f, 0.0f),
                                point_t<>(1.0f, 4.0f, 0.0f), point_t<>(1.0f, 3.0f, 0.0f), point_t<>(2.0f, 3.0f, 0.0f), point_t<>(2.0f, 2.0f, 0.0f),
                                point_t<>(1.0f, 2.0f, 0.0f), point_t<>(1.0f, 1.0f, 0.0f), point_t<>(2.0f, 1.0f, 0.0f), point_t<>(2.0f, 0.0f, 0.0f) };
    polygon uut(&pts, { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 });

    std::vector<int> tris;
    uut.to_triangles(&tris);

    /* Checks */
    BOOST_REQUIRE(tris.size() == 27);
    BOOST_CHECK(tris[ 0] ==  1);
    BOOST_CHECK(tris[ 1] ==  2);
    BOOST_CHECK(tris[ 2] ==  3);
    BOOST_CHECK(tris[ 3] ==  1);
    BOOST_CHECK(tris[ 4] ==  3);
    BOOST_CHECK(tris[ 5] ==  4);
    BOOST_CHECK(tris[ 6] ==  1);
    BOOST_CHECK(tris[ 7] ==  4);
    BOOST_CHECK(tris[ 8] ==  5);
    BOOST_CHECK(tris[ 9] ==  5);
    BOOST_CHECK(tris[10] ==  6);
    BOOST_CHECK(tris[11] ==  7);
    BOOST_CHECK(tris[12] ==  5);
    BOOST_CHECK(tris[13] ==  7);
    BOOST_CHECK(tris[14] ==  8);
    BOOST_CHECK(tris[15] ==  9);
    BOOST_CHECK(tris[16] == 10);
    BOOST_CHECK(tris[17] == 11);
    BOOST_CHECK(tris[18] ==  9);
    BOOST_CHECK(tris[19] == 11);
    BOOST_CHECK(tris[20] ==  0);
    BOOST_CHECK(tris[21] ==  9);
    BOOST_CHECK(tris[22] ==  0);
    BOOST_CHECK(tris[23] ==  1);
    BOOST_CHECK(tris[24] ==  9);
    BOOST_CHECK(tris[25] ==  1);
    BOOST_CHECK(tris[26] ==  5);
}

BOOST_AUTO_TEST_CASE( polygon_to_triangles_octogon_hole_in_square_test )
{
    std::vector<point_t<>> pts{   point_t<>( 0.0f,  1.0f, 0.0f), point_t<>( 0.0f, 2.0f, 0.0f), point_t<>(1.0f, 3.0f, 0.0f), point_t<>(2.0f,  3.0f, 0.0f),
                                point_t<>( 3.0f,  2.0f, 0.0f), point_t<>( 3.0f, 1.0f, 0.0f), point_t<>(2.0f, 0.0f, 0.0f), point_t<>(1.0f,  0.0f, 0.0f),
                                point_t<>(-1.0f, -1.0f, 0.0f), point_t<>(-1.0f, 4.0f, 0.0f), point_t<>(4.0f, 4.0f, 0.0f), point_t<>(4.0f, -1.0f, 0.0f) };
    polygon uut(&pts, { 8, 9, 10, 11, 8, 7, 6, 5, 4, 3, 2, 1, 0, 7 });

    std::vector<int> tris;
    uut.to_triangles(&tris);

    /* Checks */
    BOOST_REQUIRE(tris.size() == 36);
    BOOST_CHECK(tris[ 0] == 11);
    BOOST_CHECK(tris[ 1] ==  8);
    BOOST_CHECK(tris[ 2] ==  7);
    BOOST_CHECK(tris[ 3] == 11);
    BOOST_CHECK(tris[ 4] ==  7);
    BOOST_CHECK(tris[ 5] ==  6);
    BOOST_CHECK(tris[ 6] == 11);
    BOOST_CHECK(tris[ 7] ==  6);
    BOOST_CHECK(tris[ 8] ==  5);
    BOOST_CHECK(tris[ 9] == 11);
    BOOST_CHECK(tris[10] ==  5);
    BOOST_CHECK(tris[11] ==  4);
    BOOST_CHECK(tris[12] ==  0);
    BOOST_CHECK(tris[13] ==  7);
    BOOST_CHECK(tris[14] ==  8);
    BOOST_CHECK(tris[15] ==  0);
    BOOST_CHECK(tris[16] ==  8);
    BOOST_CHECK(tris[17] ==  9);
    BOOST_CHECK(tris[18] == 10);
    BOOST_CHECK(tris[19] == 11);
    BOOST_CHECK(tris[20] ==  4);
    BOOST_CHECK(tris[21] == 10);
    BOOST_CHECK(tris[22] ==  4);
    BOOST_CHECK(tris[23] ==  3);
    BOOST_CHECK(tris[24] == 10);
    BOOST_CHECK(tris[25] ==  3);
    BOOST_CHECK(tris[26] ==  2);
    BOOST_CHECK(tris[27] ==  1);
    BOOST_CHECK(tris[28] ==  0);
    BOOST_CHECK(tris[29] ==  9);
    BOOST_CHECK(tris[30] ==  9);
    BOOST_CHECK(tris[31] == 10);
    BOOST_CHECK(tris[32] ==  2);
    BOOST_CHECK(tris[33] ==  9);
    BOOST_CHECK(tris[34] ==  2);
    BOOST_CHECK(tris[35] ==  1);
}

BOOST_AUTO_TEST_CASE( polygon_to_triangles_h_test )
{
    std::vector<point_t<>> pts{   point_t<>(0.0f, 0.0f, 0.0f), point_t<>(0.0f, 3.0f, 0.0f), point_t<>(1.0f, 3.0f, 0.0f), point_t<>(1.0f, 2.0f, 0.0f),
                                point_t<>(2.0f, 2.0f, 0.0f), point_t<>(2.0f, 3.0f, 0.0f), point_t<>(3.0f, 3.0f, 0.0f), point_t<>(3.0f, 0.0f, 0.0f),
                                point_t<>(2.0f, 0.0f, 0.0f), point_t<>(2.0f, 1.0f, 0.0f), point_t<>(1.0f, 1.0f, 0.0f), point_t<>(1.0f, 0.0f, 0.0f) };
    polygon uut(&pts, { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 });

    std::vector<int> tris;
    uut.to_triangles(&tris);

    /* Checks */
    BOOST_REQUIRE(tris.size() == 21);
    BOOST_CHECK(tris[ 0] == 11);
    BOOST_CHECK(tris[ 1] ==  0);
    BOOST_CHECK(tris[ 2] ==  1);
    BOOST_CHECK(tris[ 3] == 11);
    BOOST_CHECK(tris[ 4] ==  1);
    BOOST_CHECK(tris[ 5] ==  2);
    BOOST_CHECK(tris[ 6] ==  4);
    BOOST_CHECK(tris[ 7] ==  5);
    BOOST_CHECK(tris[ 8] ==  6);
    BOOST_CHECK(tris[ 9] ==  4);
    BOOST_CHECK(tris[10] ==  6);
    BOOST_CHECK(tris[11] ==  7);
    BOOST_CHECK(tris[12] ==  4);
    BOOST_CHECK(tris[13] ==  7);
    BOOST_CHECK(tris[14] ==  8);
    BOOST_CHECK(tris[15] ==  4);
    BOOST_CHECK(tris[16] ==  9);
    BOOST_CHECK(tris[17] == 10);
    BOOST_CHECK(tris[18] == 10);
    BOOST_CHECK(tris[19] ==  3);
    BOOST_CHECK(tris[20] ==  4);
}

BOOST_AUTO_TEST_CASE( polygon_to_triangles_star_test )
{
    std::vector<point_t<>> pts{   point_t<>(0.0f, 3.0f, 0.0f), point_t<>(0.0f, 2.0f, 1.0f), point_t<>(0.0f, 0.0f, 1.0f), point_t<>(0.0f, 1.0f, 2.0f),
                                point_t<>(0.0f, 0.0f, 3.0f), point_t<>(0.0f, 2.0f, 3.0f), point_t<>(0.0f, 3.0f, 4.0f), point_t<>(0.0f, 4.0f, 3.0f),
                                point_t<>(0.0f, 6.0f, 3.0f), point_t<>(0.0f, 5.0f, 2.0f), point_t<>(0.0f, 6.0f, 1.0f), point_t<>(0.0f, 4.0f, 1.0f) };
    polygon uut(&pts, { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 });

    std::vector<int> tris;
    uut.to_triangles(&tris);

    /* Checks */
    BOOST_REQUIRE(tris.size() == 24);
    BOOST_CHECK(tris[ 0] ==  1);
    BOOST_CHECK(tris[ 1] ==  2);
    BOOST_CHECK(tris[ 2] ==  3);
    BOOST_CHECK(tris[ 3] ==  1);
    BOOST_CHECK(tris[ 4] ==  4);
    BOOST_CHECK(tris[ 5] ==  5);
    BOOST_CHECK(tris[ 6] ==  1);
    BOOST_CHECK(tris[ 7] ==  5);
    BOOST_CHECK(tris[ 8] ==  6);
    BOOST_CHECK(tris[ 9] ==  1);
    BOOST_CHECK(tris[10] ==  6);
    BOOST_CHECK(tris[11] ==  7);
    BOOST_CHECK(tris[12] ==  1);
    BOOST_CHECK(tris[13] ==  7);
    BOOST_CHECK(tris[14] ==  8);
    BOOST_CHECK(tris[15] ==  1);
    BOOST_CHECK(tris[16] ==  8);
    BOOST_CHECK(tris[17] ==  9);
    BOOST_CHECK(tris[18] ==  1);
    BOOST_CHECK(tris[19] ==  9);
    BOOST_CHECK(tris[20] == 10);
    BOOST_CHECK(tris[21] ==  1);
    BOOST_CHECK(tris[22] == 11);
    BOOST_CHECK(tris[23] ==  0);
}

BOOST_FIXTURE_TEST_CASE( world_polygon_points_above_plane_x_1_test, world_polygon_fixture )
{
    uut.points_above_plane(point_t<>(1.0f, 0.0f, 0.0f), point_t<>(-1.0f, 0.0f, 0.0f));

    std::unique_ptr<std::vector<point_t<>>> exp(new std::vector<point_t<>>({
        point_t<>(-0.5f, -0.5f,  1.0f),
        point_t<>(-0.5f,  1.5f,  1.0f),
        point_t<>(-2.5f, -0.5f,  1.0f),
        point_t<>(-0.5f, -0.5f,  6.0f),
        point_t<>(-0.5f,  1.5f, -3.0f) 
    }));

    /* Checks */
    BOOST_REQUIRE(uut.number_of_vertices() == static_cast<int>(exp->size()));
    for (int i = 0; i < uut.number_of_vertices(); ++i)
    {
        BOOST_CHECK(fabs(exp->at(i).x - uut.vertex(i).x) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).y - uut.vertex(i).y) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).z - uut.vertex(i).z) < result_tolerance);
    }
}

BOOST_FIXTURE_TEST_CASE( world_polygon_points_above_plane_my_1_test, world_polygon_fixture )
{
    uut.points_above_plane(point_t<>(0.0f, 1.0f, 0.0f), point_t<>(0.0f, 1.0f, 0.0f));

    std::unique_ptr<std::vector<point_t<>>> exp(new std::vector<point_t<>>({
        point_t<>( 1.5f,  1.5f,  1.0f),
        point_t<>(-0.5f,  1.5f,  1.0f),
        point_t<>( 1.5f,  3.5f,  1.0f),
        point_t<>(-0.5f,  1.5f, -3.0f) 
    }));

    /* Checks */
    BOOST_REQUIRE(uut.number_of_vertices() == static_cast<int>(exp->size()));
    for (int i = 0; i < uut.number_of_vertices(); ++i)
    {
        BOOST_CHECK(fabs(exp->at(i).x - uut.vertex(i).x) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).y - uut.vertex(i).y) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).z - uut.vertex(i).z) < result_tolerance);
    }
}

BOOST_FIXTURE_TEST_CASE( world_polygon_points_above_plane_z_0_test, world_polygon_fixture )
{
    uut.points_above_plane(point_t<>(0.0f, 0.0f, 0.0f), point_t<>(0.0f, 0.0f, -1.0f));

    std::unique_ptr<std::vector<point_t<>>> exp(new std::vector<point_t<>>({
        point_t<>(-0.5f,  1.5f, -3.0f)
    }));

    /* Checks */
    BOOST_REQUIRE(uut.number_of_vertices() == static_cast<int>(exp->size()));
    for (int i = 0; i < uut.number_of_vertices(); ++i)
    {
        BOOST_CHECK(fabs(exp->at(i).x - uut.vertex(i).x) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).y - uut.vertex(i).y) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).z - uut.vertex(i).z) < result_tolerance);
    }
}

BOOST_FIXTURE_TEST_CASE( world_polygon_points_above_point_on_plane_test, world_polygon_fixture )
{
    uut.points_above_plane(point_t<>(-0.5f, -0.5f, 6.0f), point_t<>(0.0f, 0.0f, 1.0f));

    std::unique_ptr<std::vector<point_t<>>> exp(new std::vector<point_t<>>({
        point_t<>(-0.5f, -0.5f, 6.0f) 
    }));

    /* Checks */
    BOOST_REQUIRE(uut.number_of_vertices() == static_cast<int>(exp->size()));
    for (int i = 0; i < uut.number_of_vertices(); ++i)
    {
        BOOST_CHECK(fabs(exp->at(i).x - uut.vertex(i).x) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).y - uut.vertex(i).y) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).z - uut.vertex(i).z) < result_tolerance);
    }
}

BOOST_FIXTURE_TEST_CASE( world_polygon_extreme_vertices_test, world_polygon_fixture )
{
    point_t<> neg_vert;
    BOOST_CHECK(uut.extreme_vertices(&neg_vert, point_t<>(1.0f, 0.0f, 0.0f)) == point_t<>( 1.5f, -0.5f, 1.0f));
    BOOST_CHECK(neg_vert == point_t<>(-2.5f, -0.5f, 1.0f));

    BOOST_CHECK(uut.extreme_vertices(&neg_vert, point_t<>(1.0f, 1.0f, 1.0f)) == point_t<>( 1.5f,  3.5f, 1.0f));
    BOOST_CHECK(neg_vert == point_t<>(-2.5f, -0.5f, 1.0f));

    BOOST_CHECK(uut.extreme_vertices(&neg_vert, point_t<>(-1.0f, -1.0f, -1.0f)) == point_t<>(-2.5f, -0.5f, 1.0f));
    BOOST_CHECK(neg_vert == point_t<>( 1.5f,  3.5f, 1.0f));
}

BOOST_FIXTURE_TEST_CASE( world_polygon_center_test, world_polygon_fixture )
{
    BOOST_CHECK(uut.center() == point_t<>(0.0f, 0.75f, 1.125f));
}

BOOST_AUTO_TEST_CASE( world_polygon_intersection_vertex_vertex_test )
{
    /* Square test 0 */
    world_polygon p0({
        point_t<>( 1.0f,  1.0f, 0.0f)
    });

    world_polygon p1({
        point_t<>( 1.0f,  1.0f, 0.5f)
    });

    std::unique_ptr<std::vector<point_t<>>> exp(new std::vector<point_t<>>({
        point_t<>( 1.0f,  1.0f, 0.0f)
    }));

    p0.intersection(p1, point_t<>(0.0f, 0.0f, 1.0f), point_t<>(0.0f, 0.0f, 0.5f));

    /* Checks */
    BOOST_REQUIRE(p0.number_of_vertices() == static_cast<int>(exp->size()));
    for (int i = 0; i < p0.number_of_vertices(); ++i)
    {
        BOOST_CHECK(fabs(exp->at(i).x - p0.vertex(i).x) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).y - p0.vertex(i).y) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).z - p0.vertex(i).z) < result_tolerance);
    }
}

BOOST_AUTO_TEST_CASE( world_polygon_intersection_vertex_line_test )
{
    /* Square test 0 */
    world_polygon p0({
        point_t<>( 1.0f,  1.0f, 0.0f)
    });

    world_polygon p1({
        point_t<>( 1.0f,  1.0f, 0.5f),
        point_t<>( 1.0f,  1.5f, 0.5f)
    });

    std::unique_ptr<std::vector<point_t<>>> exp(new std::vector<point_t<>>({
        point_t<>( 1.0f,  1.0f, 0.0f)
    }));

    p0.intersection(p1, point_t<>(0.0f, 0.0f, 1.0f), point_t<>(0.0f, 0.0f, 0.5f));

    /* Checks */
    BOOST_REQUIRE(p0.number_of_vertices() == static_cast<int>(exp->size()));
    for (int i = 0; i < p0.number_of_vertices(); ++i)
    {
        BOOST_CHECK(fabs(exp->at(i).x - p0.vertex(i).x) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).y - p0.vertex(i).y) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).z - p0.vertex(i).z) < result_tolerance);
    }
}

BOOST_AUTO_TEST_CASE( world_polygon_intersection_vertex_polygon_test )
{
    /* Square test 0 */
    world_polygon p0({
        point_t<>( 1.0f,  1.0f, 0.0f)
    });

    world_polygon p1({
        point_t<>( 1.0f,  1.0f, 0.5f),
        point_t<>( 1.0f,  1.5f, 0.5f),
        point_t<>( 1.5f,  1.5f, 0.5f),
    });

    std::unique_ptr<std::vector<point_t<>>> exp(new std::vector<point_t<>>({
        point_t<>( 1.0f,  1.0f, 0.0f)
    }));

    p0.intersection(p1, point_t<>(0.0f, 0.0f, 1.0f), point_t<>(0.0f, 0.0f, 0.5f));

    /* Checks */
    BOOST_REQUIRE(p0.number_of_vertices() == static_cast<int>(exp->size()));
    for (int i = 0; i < p0.number_of_vertices(); ++i)
    {
        BOOST_CHECK(fabs(exp->at(i).x - p0.vertex(i).x) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).y - p0.vertex(i).y) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).z - p0.vertex(i).z) < result_tolerance);
    }
}

BOOST_AUTO_TEST_CASE( world_polygon_intersection_line_vertex_test )
{
    /* Square test 0 */
    world_polygon p0({
        point_t<>( 1.0f,  1.0f, 0.5f),
        point_t<>( 1.0f,  1.5f, 0.5f)
    });

    world_polygon p1({
        point_t<>( 1.0f,  1.0f, 0.0f)
    });

    std::unique_ptr<std::vector<point_t<>>> exp(new std::vector<point_t<>>({
        point_t<>( 1.0f,  1.0f, 0.5f)
    }));

    p0.intersection(p1, point_t<>(0.0f, 0.0f, -1.0f), point_t<>(0.0f, 0.0f, -0.5f));

    /* Checks */
    BOOST_REQUIRE(p0.number_of_vertices() == static_cast<int>(exp->size()));
    for (int i = 0; i < p0.number_of_vertices(); ++i)
    {
        BOOST_CHECK(fabs(exp->at(i).x - p0.vertex(i).x) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).y - p0.vertex(i).y) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).z - p0.vertex(i).z) < result_tolerance);
    }
}

BOOST_AUTO_TEST_CASE( world_polygon_intersection_polygon_vertex_test )
{
    /* Square test 0 */
    world_polygon p0({
        point_t<>( 1.0f,  1.0f, 0.5f),
        point_t<>( 1.0f,  1.5f, 0.5f),
        point_t<>( 1.5f,  1.5f, 0.5f),
    });

    world_polygon p1({
        point_t<>( 1.0f,  1.0f, 0.0f)
    });

    std::unique_ptr<std::vector<point_t<>>> exp(new std::vector<point_t<>>({
        point_t<>( 1.0f,  1.0f, 0.5f)
    }));

    p0.intersection(p1, point_t<>(0.0f, 0.0f, -1.0f), point_t<>(0.0f, 0.0f, -0.5f));

    /* Checks */
    BOOST_REQUIRE(p0.number_of_vertices() == static_cast<int>(exp->size()));
    for (int i = 0; i < p0.number_of_vertices(); ++i)
    {
        BOOST_CHECK(fabs(exp->at(i).x - p0.vertex(i).x) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).y - p0.vertex(i).y) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).z - p0.vertex(i).z) < result_tolerance);
    }
}

BOOST_AUTO_TEST_CASE( world_polygon_intersection_line_line_skew_0_test )
{
    /* Square test 0 */
    world_polygon p0({
        point_t<>( 1.0f,  1.0f, 0.5f),
        point_t<>( 1.0f,  2.0f, 0.5f)
    });

    world_polygon p1({
        point_t<>( 0.0f,  1.5f, 0.0f),
        point_t<>( 2.0f,  1.5f, 0.0f)
    });

    std::unique_ptr<std::vector<point_t<>>> exp(new std::vector<point_t<>>({
        point_t<>( 1.0f,  1.5f, 0.5f)
    }));

    p0.intersection(p1, point_t<>(0.0f, 0.0f, -1.0f), point_t<>(0.0f, 0.0f, -0.5f));

    /* Checks */
    BOOST_REQUIRE(p0.number_of_vertices() == static_cast<int>(exp->size()));
    for (int i = 0; i < p0.number_of_vertices(); ++i)
    {
        BOOST_CHECK(fabs(exp->at(i).x - p0.vertex(i).x) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).y - p0.vertex(i).y) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).z - p0.vertex(i).z) < result_tolerance);
    }
}

BOOST_AUTO_TEST_CASE( world_polygon_intersection_line_line_skew_1_test )
{
    /* Square test 0 */
    world_polygon p0({
        point_t<>( 1.0f,  1.5f, 0.5f),
        point_t<>( 1.0f,  2.0f, 0.5f)
    });

    world_polygon p1({
        point_t<>( 0.0f,  1.5f, 0.0f),
        point_t<>( 1.0f,  1.5f, 0.0f)
    });

    std::unique_ptr<std::vector<point_t<>>> exp(new std::vector<point_t<>>({
        point_t<>( 1.0f,  1.5f, 0.5f)
    }));

    p0.intersection(p1, point_t<>(0.0f, 0.0f, -1.0f), point_t<>(0.0f, 0.0f, -0.5f));

    /* Checks */
    BOOST_REQUIRE(p0.number_of_vertices() == static_cast<int>(exp->size()));
    for (int i = 0; i < p0.number_of_vertices(); ++i)
    {
        BOOST_CHECK(fabs(exp->at(i).x - p0.vertex(i).x) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).y - p0.vertex(i).y) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).z - p0.vertex(i).z) < result_tolerance);
    }
}

BOOST_AUTO_TEST_CASE( world_polygon_intersection_line_line_skew_2_test )
{
    /* Square test 0 */
    world_polygon p0({
        point_t<>( 1.0f,  1.5f, 0.5f),
        point_t<>( 1.0f,  2.0f, 0.5f)
    });

    world_polygon p1({
        point_t<>( 0.0f,  2.0f, 0.0f),
        point_t<>( 1.0f,  2.0f, 0.0f)
    });

    std::unique_ptr<std::vector<point_t<>>> exp(new std::vector<point_t<>>({
        point_t<>( 1.0f,  2.0f, 0.5f)
    }));

    p0.intersection(p1, point_t<>(0.0f, 0.0f, -1.0f), point_t<>(0.0f, 0.0f, -0.5f));

    /* Checks */
    BOOST_REQUIRE(p0.number_of_vertices() == static_cast<int>(exp->size()));
    for (int i = 0; i < p0.number_of_vertices(); ++i)
    {
        BOOST_CHECK(fabs(exp->at(i).x - p0.vertex(i).x) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).y - p0.vertex(i).y) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).z - p0.vertex(i).z) < result_tolerance);
    }
}

BOOST_AUTO_TEST_CASE( world_polygon_intersection_line_line_skew_3_test )
{
    /* Square test 0 */
    world_polygon p0({
        point_t<>( 1.0f,  1.5f, 0.0f),
        point_t<>( 0.0f,  1.5f, 0.0f)
    });

    world_polygon p1({
        point_t<>( 1.0f,  1.5f, 0.5f),
        point_t<>( 1.0f,  2.0f, 0.5f)
    });

    std::unique_ptr<std::vector<point_t<>>> exp(new std::vector<point_t<>>({
        point_t<>( 1.0f,  1.5f, 0.0f)
    }));

    p0.intersection(p1, point_t<>(0.0f, 0.0f, 1.0f), point_t<>(0.0f, 0.0f, 0.5f));

    /* Checks */
    BOOST_REQUIRE(p0.number_of_vertices() == static_cast<int>(exp->size()));
    for (int i = 0; i < p0.number_of_vertices(); ++i)
    {
        BOOST_CHECK(fabs(exp->at(i).x - p0.vertex(i).x) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).y - p0.vertex(i).y) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).z - p0.vertex(i).z) < result_tolerance);
    }
}

BOOST_AUTO_TEST_CASE( world_polygon_intersection_line_line_skew_4_test )
{
    /* Square test 0 */
    world_polygon p0({
        point_t<>( 1.0f,  2.0f, 0.0f),
        point_t<>( 0.0f,  2.0f, 0.0f)
    });

    world_polygon p1({
        point_t<>( 1.0f,  1.5f, 0.5f),
        point_t<>( 1.0f,  2.0f, 0.5f)
    });

    std::unique_ptr<std::vector<point_t<>>> exp(new std::vector<point_t<>>({
        point_t<>( 1.0f,  2.0f, 0.0f)
    }));

    p0.intersection(p1, point_t<>(0.0f, 0.0f, 1.0f), point_t<>(0.0f, 0.0f, 0.5f));

    /* Checks */
    BOOST_REQUIRE(p0.number_of_vertices() == static_cast<int>(exp->size()));
    for (int i = 0; i < p0.number_of_vertices(); ++i)
    {
        BOOST_CHECK(fabs(exp->at(i).x - p0.vertex(i).x) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).y - p0.vertex(i).y) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).z - p0.vertex(i).z) < result_tolerance);
    }
}

BOOST_AUTO_TEST_CASE( world_polygon_intersection_line_line_parallel_0_overlap_test )
{
    /* Square test 0 */
    world_polygon p0({
        point_t<>( 1.0f,  1.0f, 0.5f),
        point_t<>( 1.0f,  2.0f, 0.5f)
    });

    world_polygon p1({
        point_t<>( 1.0f,  1.5f, 0.0f),
        point_t<>( 1.0f,  2.5f, 0.0f)
    });

    std::unique_ptr<std::vector<point_t<>>> exp(new std::vector<point_t<>>({
        point_t<>( 1.0f,  1.5f, 0.5f),
        point_t<>( 1.0f,  2.0f, 0.5f)
    }));

    p0.intersection(p1, point_t<>(0.0f, 0.0f, -1.0f), point_t<>(0.0f, 0.0f, -0.5f));

    /* Checks */
    BOOST_REQUIRE(p0.number_of_vertices() == static_cast<int>(exp->size()));
    for (int i = 0; i < p0.number_of_vertices(); ++i)
    {
        BOOST_CHECK(fabs(exp->at(i).x - p0.vertex(i).x) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).y - p0.vertex(i).y) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).z - p0.vertex(i).z) < result_tolerance);
    }
}

BOOST_AUTO_TEST_CASE( world_polygon_intersection_line_line_parallel_1_overlap_test )
{
    /* Square test 0 */
    world_polygon p0({
        point_t<>( 1.0f,  1.0f, 0.5f),
        point_t<>( 1.0f,  2.0f, 0.5f)
    });

    world_polygon p1({
        point_t<>( 1.0f,  0.5f, 0.0f),
        point_t<>( 1.0f,  1.5f, 0.0f)
    });

    std::unique_ptr<std::vector<point_t<>>> exp(new std::vector<point_t<>>({
        point_t<>( 1.0f,  1.5f, 0.5f),
        point_t<>( 1.0f,  1.0f, 0.5f)
    }));

    p0.intersection(p1, point_t<>(0.0f, 0.0f, -1.0f), point_t<>(0.0f, 0.0f, -0.5f));

    /* Checks */
    BOOST_REQUIRE(p0.number_of_vertices() == static_cast<int>(exp->size()));
    for (int i = 0; i < p0.number_of_vertices(); ++i)
    {
        BOOST_CHECK(fabs(exp->at(i).x - p0.vertex(i).x) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).y - p0.vertex(i).y) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).z - p0.vertex(i).z) < result_tolerance);
    }
}

BOOST_AUTO_TEST_CASE( world_polygon_intersection_line_line_parallel_3_overlap_test )
{
    /* Square test 0 */
    world_polygon p0({
        point_t<>( 1.0f,  1.0f, 0.5f),
        point_t<>( 1.0f,  3.0f, 0.5f)
    });

    world_polygon p1({
        point_t<>( 1.0f,  1.5f, 0.0f),
        point_t<>( 1.0f,  2.5f, 0.0f)
    });

    std::unique_ptr<std::vector<point_t<>>> exp(new std::vector<point_t<>>({
        point_t<>( 1.0f,  1.5f, 0.5f),
        point_t<>( 1.0f,  2.5f, 0.5f)
    }));

    p0.intersection(p1, point_t<>(0.0f, 0.0f, -1.0f), point_t<>(0.0f, 0.0f, -0.5f));

    /* Checks */
    BOOST_REQUIRE(p0.number_of_vertices() == static_cast<int>(exp->size()));
    for (int i = 0; i < p0.number_of_vertices(); ++i)
    {
        BOOST_CHECK(fabs(exp->at(i).x - p0.vertex(i).x) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).y - p0.vertex(i).y) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).z - p0.vertex(i).z) < result_tolerance);
    }
}

BOOST_AUTO_TEST_CASE( world_polygon_intersection_line_line_parallel_4_overlap_test )
{
    /* Square test 0 */
    world_polygon p0({
        point_t<>( 1.0f,  1.5f, 0.5f),
        point_t<>( 1.0f,  2.5f, 0.5f)
    });

    world_polygon p1({
        point_t<>( 1.0f,  1.0f, 0.0f),
        point_t<>( 1.0f,  3.0f, 0.0f)
    });

    std::unique_ptr<std::vector<point_t<>>> exp(new std::vector<point_t<>>({
        point_t<>( 1.0f,  1.5f, 0.5f),
        point_t<>( 1.0f,  2.5f, 0.5f)
    }));

    p0.intersection(p1, point_t<>(0.0f, 0.0f, -1.0f), point_t<>(0.0f, 0.0f, -0.5f));

    /* Checks */
    BOOST_REQUIRE(p0.number_of_vertices() == static_cast<int>(exp->size()));
    for (int i = 0; i < p0.number_of_vertices(); ++i)
    {
        BOOST_CHECK(fabs(exp->at(i).x - p0.vertex(i).x) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).y - p0.vertex(i).y) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).z - p0.vertex(i).z) < result_tolerance);
    }
}

BOOST_AUTO_TEST_CASE( world_polygon_intersection_line_line_parallel_5_overlap_test )
{
    /* Square test 0 */
    world_polygon p0({
        point_t<>( 1.0f,  1.5f, 0.5f),
        point_t<>( 1.0f,  2.5f, 0.5f)
    });

    world_polygon p1({
        point_t<>( 1.0f,  1.5f, 0.0f),
        point_t<>( 1.0f,  2.5f, 0.0f)
    });

    std::unique_ptr<std::vector<point_t<>>> exp(new std::vector<point_t<>>({
        point_t<>( 1.0f,  1.5f, 0.5f),
        point_t<>( 1.0f,  2.5f, 0.5f)
    }));

    p0.intersection(p1, point_t<>(0.0f, 0.0f, -1.0f), point_t<>(0.0f, 0.0f, -0.5f));

    /* Checks */
    BOOST_REQUIRE(p0.number_of_vertices() == static_cast<int>(exp->size()));
    for (int i = 0; i < p0.number_of_vertices(); ++i)
    {
        BOOST_CHECK(fabs(exp->at(i).x - p0.vertex(i).x) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).y - p0.vertex(i).y) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).z - p0.vertex(i).z) < result_tolerance);
    }
}

BOOST_AUTO_TEST_CASE( world_polygon_intersection_line_polygon_0_test )
{
    /* Square test 0 */
    world_polygon p0({
        point_t<>( 1.0f,  1.5f, 0.5f),
        point_t<>( 1.0f,  2.5f, 0.5f)
    });

    world_polygon p1({
        point_t<>( 0.0f,  1.0f, 0.0f),
        point_t<>( 0.0f,  3.0f, 0.0f),
        point_t<>( 2.0f,  3.0f, 0.0f),
        point_t<>( 2.0f,  1.0f, 0.0f)
    });

    std::unique_ptr<std::vector<point_t<>>> exp(new std::vector<point_t<>>({
        point_t<>( 1.0f,  1.5f, 0.5f),
        point_t<>( 1.0f,  2.5f, 0.5f)
    }));

    p0.intersection(p1, point_t<>(0.0f, 0.0f, -1.0f), point_t<>(0.0f, 0.0f, -0.5f));

    /* Checks */
    BOOST_REQUIRE(p0.number_of_vertices() == static_cast<int>(exp->size()));
    for (int i = 0; i < p0.number_of_vertices(); ++i)
    {
        BOOST_CHECK(fabs(exp->at(i).x - p0.vertex(i).x) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).y - p0.vertex(i).y) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).z - p0.vertex(i).z) < result_tolerance);
    }
}

BOOST_AUTO_TEST_CASE( world_polygon_intersection_line_polygon_1_test )
{
    /* Square test 0 */
    world_polygon p0({
        point_t<>( 1.0f,  1.5f, 0.5f),
        point_t<>( 1.0f,  3.5f, 0.5f)
    });

    world_polygon p1({
        point_t<>( 0.0f,  1.0f, 0.0f),
        point_t<>( 0.0f,  3.0f, 0.0f),
        point_t<>( 2.0f,  3.0f, 0.0f),
        point_t<>( 2.0f,  1.0f, 0.0f)
    });

    std::unique_ptr<std::vector<point_t<>>> exp(new std::vector<point_t<>>({
        point_t<>( 1.0f,  1.5f, 0.5f),
        point_t<>( 1.0f,  3.0f, 0.5f)
    }));

    p0.intersection(p1, point_t<>(0.0f, 0.0f, -1.0f), point_t<>(0.0f, 0.0f, -0.5f));

    /* Checks */
    BOOST_REQUIRE(p0.number_of_vertices() == static_cast<int>(exp->size()));
    for (int i = 0; i < p0.number_of_vertices(); ++i)
    {
        BOOST_CHECK(fabs(exp->at(i).x - p0.vertex(i).x) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).y - p0.vertex(i).y) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).z - p0.vertex(i).z) < result_tolerance);
    }
}

BOOST_AUTO_TEST_CASE( world_polygon_intersection_line_polygon_2_test )
{
    /* Square test 0 */
    world_polygon p0({
        point_t<>( 1.0f,  0.5f, 0.5f),
        point_t<>( 1.0f,  2.5f, 0.5f)
    });

    world_polygon p1({
        point_t<>( 0.0f,  1.0f, 0.0f),
        point_t<>( 0.0f,  3.0f, 0.0f),
        point_t<>( 2.0f,  3.0f, 0.0f),
        point_t<>( 2.0f,  1.0f, 0.0f)
    });

    std::unique_ptr<std::vector<point_t<>>> exp(new std::vector<point_t<>>({
        point_t<>( 1.0f,  2.5f, 0.5f),
        point_t<>( 1.0f,  1.0f, 0.5f)
    }));

    p0.intersection(p1, point_t<>(0.0f, 0.0f, -1.0f), point_t<>(0.0f, 0.0f, -0.5f));

    /* Checks */
    BOOST_REQUIRE(p0.number_of_vertices() == static_cast<int>(exp->size()));
    for (int i = 0; i < p0.number_of_vertices(); ++i)
    {
        BOOST_CHECK(fabs(exp->at(i).x - p0.vertex(i).x) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).y - p0.vertex(i).y) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).z - p0.vertex(i).z) < result_tolerance);
    }
}

BOOST_AUTO_TEST_CASE( world_polygon_intersection_line_polygon_3_test )
{
    /* Square test 0 */
    world_polygon p0({
        point_t<>( 1.0f,  0.5f, 0.5f),
        point_t<>( 1.0f,  3.5f, 0.5f)
    });

    world_polygon p1({
        point_t<>( 0.0f,  1.0f, 0.0f),
        point_t<>( 0.0f,  3.0f, 0.0f),
        point_t<>( 2.0f,  3.0f, 0.0f),
        point_t<>( 2.0f,  1.0f, 0.0f)
    });

    std::unique_ptr<std::vector<point_t<>>> exp(new std::vector<point_t<>>({
        point_t<>( 1.0f,  1.0f, 0.5f),
        point_t<>( 1.0f,  3.0f, 0.5f)
    }));

    p0.intersection(p1, point_t<>(0.0f, 0.0f, -1.0f), point_t<>(0.0f, 0.0f, -0.5f));

    /* Checks */
    BOOST_REQUIRE(p0.number_of_vertices() == static_cast<int>(exp->size()));
    for (int i = 0; i < p0.number_of_vertices(); ++i)
    {
        BOOST_CHECK(fabs(exp->at(i).x - p0.vertex(i).x) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).y - p0.vertex(i).y) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).z - p0.vertex(i).z) < result_tolerance);
    }
}



BOOST_AUTO_TEST_CASE( world_polygon_intersection_line_polygon_4_test )
{
    /* Square test 0 */
    world_polygon p0({
        point_t<>( 0.0f,  1.0f, 0.0f),
        point_t<>( 0.0f,  3.0f, 0.0f),
        point_t<>( 2.0f,  3.0f, 0.0f),
        point_t<>( 2.0f,  1.0f, 0.0f)
    });

    world_polygon p1({
        point_t<>( 1.0f,  1.5f, 0.5f),
        point_t<>( 1.0f,  2.5f, 0.5f)
    });

    std::unique_ptr<std::vector<point_t<>>> exp(new std::vector<point_t<>>({
        point_t<>( 1.0f,  1.5f, 0.0f),
        point_t<>( 1.0f,  2.5f, 0.0f)
    }));

    p0.intersection(p1, point_t<>(0.0f, 0.0f, 1.0f), point_t<>(0.0f, 0.0f, 0.5f));

    /* Checks */
    BOOST_REQUIRE(p0.number_of_vertices() == static_cast<int>(exp->size()));
    for (int i = 0; i < p0.number_of_vertices(); ++i)
    {
        BOOST_CHECK(fabs(exp->at(i).x - p0.vertex(i).x) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).y - p0.vertex(i).y) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).z - p0.vertex(i).z) < result_tolerance);
    }
}

BOOST_AUTO_TEST_CASE( world_polygon_intersection_line_polygon_5_test )
{
    /* Square test 0 */
    world_polygon p0({
        point_t<>( 0.0f,  1.0f, 0.0f),
        point_t<>( 0.0f,  3.0f, 0.0f),
        point_t<>( 2.0f,  3.0f, 0.0f),
        point_t<>( 2.0f,  1.0f, 0.0f)
    });

    world_polygon p1({
        point_t<>( 1.0f,  1.5f, 0.5f),
        point_t<>( 1.0f,  3.5f, 0.5f)
    });

    std::unique_ptr<std::vector<point_t<>>> exp(new std::vector<point_t<>>({
        point_t<>( 1.0f,  1.5f, 0.0f),
        point_t<>( 1.0f,  3.0f, 0.0f)
    }));

    p0.intersection(p1, point_t<>(0.0f, 0.0f, 1.0f), point_t<>(0.0f, 0.0f, 0.5f));

    /* Checks */
    BOOST_REQUIRE(p0.number_of_vertices() == static_cast<int>(exp->size()));
    for (int i = 0; i < p0.number_of_vertices(); ++i)
    {
        BOOST_CHECK(fabs(exp->at(i).x - p0.vertex(i).x) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).y - p0.vertex(i).y) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).z - p0.vertex(i).z) < result_tolerance);
    }
}

BOOST_AUTO_TEST_CASE( world_polygon_intersection_line_polygon_6_test )
{
    /* Square test 0 */
    world_polygon p0({
        point_t<>( 0.0f,  1.0f, 0.0f),
        point_t<>( 0.0f,  3.0f, 0.0f),
        point_t<>( 2.0f,  3.0f, 0.0f),
        point_t<>( 2.0f,  1.0f, 0.0f)
    });

    world_polygon p1({
        point_t<>( 1.0f,  0.5f, 0.5f),
        point_t<>( 1.0f,  2.5f, 0.5f)
    });

    std::unique_ptr<std::vector<point_t<>>> exp(new std::vector<point_t<>>({
        point_t<>( 1.0f,  2.5f, 0.0f),
        point_t<>( 1.0f,  1.0f, 0.0f)
    }));

    p0.intersection(p1, point_t<>(0.0f, 0.0f, 1.0f), point_t<>(0.0f, 0.0f, 0.5f));

    /* Checks */
    BOOST_REQUIRE(p0.number_of_vertices() == static_cast<int>(exp->size()));
    for (int i = 0; i < p0.number_of_vertices(); ++i)
    {
        BOOST_CHECK(fabs(exp->at(i).x - p0.vertex(i).x) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).y - p0.vertex(i).y) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).z - p0.vertex(i).z) < result_tolerance);
    }
}

BOOST_AUTO_TEST_CASE( world_polygon_intersection_line_polygon_7_test )
{
    /* Square test 0 */
    world_polygon p0({
        point_t<>( 0.0f,  1.0f, 0.0f),
        point_t<>( 0.0f,  3.0f, 0.0f),
        point_t<>( 2.0f,  3.0f, 0.0f),
        point_t<>( 2.0f,  1.0f, 0.0f)
    });

    world_polygon p1({
        point_t<>( 1.0f,  0.5f, 0.5f),
        point_t<>( 1.0f,  3.5f, 0.5f)
    });

    std::unique_ptr<std::vector<point_t<>>> exp(new std::vector<point_t<>>({
        point_t<>( 1.0f,  1.0f, 0.0f),
        point_t<>( 1.0f,  3.0f, 0.0f)
    }));

    p0.intersection(p1, point_t<>(0.0f, 0.0f, 1.0f), point_t<>(0.0f, 0.0f, 0.5f));

    /* Checks */
    BOOST_REQUIRE(p0.number_of_vertices() == static_cast<int>(exp->size()));
    for (int i = 0; i < p0.number_of_vertices(); ++i)
    {
        BOOST_CHECK(fabs(exp->at(i).x - p0.vertex(i).x) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).y - p0.vertex(i).y) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).z - p0.vertex(i).z) < result_tolerance);
    }
}

BOOST_AUTO_TEST_CASE( world_polygon_intersection_line_polygon_8_test )
{
    /* Square test 0 */
    world_polygon p0({
        point_t<>( 0.0f,  1.0f, 0.0f),
        point_t<>( 0.0f,  3.0f, 0.0f),
        point_t<>( 2.0f,  3.0f, 0.0f),
        point_t<>( 2.0f,  1.0f, 0.0f)
    });

    world_polygon p1({
        point_t<>( 1.0f,  0.5f, 0.5f),
        point_t<>( 1.5f,  3.5f, 0.5f)
    });

    std::unique_ptr<std::vector<point_t<>>> exp(new std::vector<point_t<>>({
        point_t<>( 1.08333f,  1.0f, 0.0f),
        point_t<>( 1.41667f,  3.0f, 0.0f)
    }));

    p0.intersection(p1, point_t<>(0.0f, 0.0f, 1.0f), point_t<>(0.0f, 0.0f, 0.5f));

    /* Checks */
    BOOST_REQUIRE(p0.number_of_vertices() == static_cast<int>(exp->size()));
    for (int i = 0; i < p0.number_of_vertices(); ++i)
    {
        BOOST_CHECK(fabs(exp->at(i).x - p0.vertex(i).x) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).y - p0.vertex(i).y) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).z - p0.vertex(i).z) < result_tolerance);
    }
}

BOOST_AUTO_TEST_CASE( world_polygon_intersection_square0_test )
{
    /* Square test 0 */
    world_polygon p0({
        point_t<>( 1.5f,  1.5f, 1.0f),
        point_t<>( 1.5f, -0.5f, 1.0f),
        point_t<>(-0.5f, -0.5f, 1.0f),
        point_t<>(-0.5f,  1.5f, 1.0f)
    });

    world_polygon p1({
        point_t<>(-1.0f,  1.0f, 0.0f),
        point_t<>(-1.0f, -1.0f, 0.0f),
        point_t<>( 1.0f, -1.0f, 0.0f),
        point_t<>( 1.0f,  1.0f, 0.0f)
    });

    std::unique_ptr<std::vector<point_t<>>> exp(new std::vector<point_t<>>({
        point_t<>( 1.0f,  1.0f, 1.0f),
        point_t<>( 1.0f, -0.5f, 1.0f),
        point_t<>(-0.5f, -0.5f, 1.0f),
        point_t<>(-0.5f,  1.0f, 1.0f)
    }));

    p0.intersection(p1, point_t<>(0.0f, 0.0f, 1.0f), point_t<>(0.0f, 0.0f, 1.0f));

    /* Checks */
    BOOST_REQUIRE(p0.number_of_vertices() == static_cast<int>(exp->size()));
    for (int i = 0; i < p0.number_of_vertices(); ++i)
    {
        BOOST_CHECK(fabs(exp->at(i).x - p0.vertex(i).x) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).y - p0.vertex(i).y) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).z - p0.vertex(i).z) < result_tolerance);
    }
}

BOOST_AUTO_TEST_CASE( world_polygon_intersection_square1_test )
{
    /* Square test 1 */
    world_polygon p0({
        point_t<>( 0.0f,  1.0f, 1.0f),
        point_t<>( 0.0f, -1.0f, 1.0f),
        point_t<>(-2.0f, -1.0f, 1.0f),
        point_t<>(-2.0f,  1.0f, 1.0f)
    });

    world_polygon p1({
        point_t<>(-1.0f,  1.0f, 0.0f),
        point_t<>(-1.0f, -1.0f, 0.0f),
        point_t<>( 1.0f, -1.0f, 0.0f),
        point_t<>( 1.0f,  1.0f, 0.0f)
    });

    std::unique_ptr<std::vector<point_t<>>> exp(new std::vector<point_t<>>({
        point_t<>(-1.0f,  1.0f, 1.0f),
        point_t<>( 0.0f,  1.0f, 1.0f),
        point_t<>( 0.0f, -1.0f, 1.0f),
        point_t<>(-1.0f, -1.0f, 1.0f)
    }));

    p0.intersection(p1, point_t<>(0.0f, 0.0f, 1.0f), point_t<>(0.0f, 0.0f, 1.0f));

    /* Checks */
    BOOST_REQUIRE(p0.number_of_vertices() == static_cast<int>(exp->size()));
    for (int i = 0; i < p0.number_of_vertices(); ++i)
    {
        BOOST_CHECK(fabs(exp->at(i).x - p0.vertex(i).x) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).y - p0.vertex(i).y) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).z - p0.vertex(i).z) < result_tolerance);
    }
}

BOOST_AUTO_TEST_CASE( world_polygon_intersection_square2_test )
{
    /* Square test 2 */
    world_polygon p0({
        point_t<>( 0.0f,  1.1f, 1.0f),
        point_t<>( 0.0f, -1.1f, 1.0f),
        point_t<>(-2.0f, -1.1f, 1.0f),
        point_t<>(-2.0f,  1.1f, 1.0f)
    });

    world_polygon p1({
        point_t<>(-1.0f,  1.0f, 0.0f),
        point_t<>(-1.0f, -1.0f, 0.0f),
        point_t<>( 1.0f, -1.0f, 0.0f),
        point_t<>( 1.0f,  1.0f, 0.0f)
    });

    std::unique_ptr<std::vector<point_t<>>> exp(new std::vector<point_t<>>({
        point_t<>(-1.0f,  1.0f, 1.0f),
        point_t<>( 0.0f,  1.0f, 1.0f),
        point_t<>( 0.0f, -1.0f, 1.0f),
        point_t<>(-1.0f, -1.0f, 1.0f)
    }));

    p0.intersection(p1, point_t<>(0.0f, 0.0f, 1.0f), point_t<>(0.0f, 0.0f, 1.0f));

    /* Checks */
    BOOST_REQUIRE(p0.number_of_vertices() == static_cast<int>(exp->size()));
    for (int i = 0; i < p0.number_of_vertices(); ++i)
    {
        BOOST_CHECK(fabs(exp->at(i).x - p0.vertex(i).x) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).y - p0.vertex(i).y) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).z - p0.vertex(i).z) < result_tolerance);
    }
}

BOOST_AUTO_TEST_CASE( world_polygon_intersection_square3_test )
{
    /* Square test 3 */
    world_polygon p0({
        point_t<>( 0.5f,  1.5f, 1.0f),
        point_t<>( 0.5f, -1.5f, 1.0f),
        point_t<>(-0.5f, -1.5f, 1.0f),
        point_t<>(-0.5f,  1.5f, 1.0f)
    });

    world_polygon p1({
        point_t<>(-1.0f,  1.0f, 0.0f),
        point_t<>(-1.0f, -1.0f, 0.0f),
        point_t<>( 1.0f, -1.0f, 0.0f),
        point_t<>( 1.0f,  1.0f, 0.0f)
    });

    std::unique_ptr<std::vector<point_t<>>> exp(new std::vector<point_t<>>({
        point_t<>( 0.5f,  1.0f, 1.0f),
        point_t<>( 0.5f, -1.0f, 1.0f),
        point_t<>(-0.5f, -1.0f, 1.0f),
        point_t<>(-0.5f,  1.0f, 1.0f)
    }));

    p0.intersection(p1, point_t<>(0.0f, 0.0f, 1.0f), point_t<>(0.0f, 0.0f, 1.0f));

    /* Checks */
    BOOST_REQUIRE(p0.number_of_vertices() == static_cast<int>(exp->size()));
    for (int i = 0; i < p0.number_of_vertices(); ++i)
    {
        BOOST_CHECK(fabs(exp->at(i).x - p0.vertex(i).x) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).y - p0.vertex(i).y) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).z - p0.vertex(i).z) < result_tolerance);
    }
}

BOOST_AUTO_TEST_CASE( world_polygon_intersection_square4_test )
{
    /* Square test 4, poly a inside poly b */
    world_polygon p0({
        point_t<>( 0.5f,  0.5f, 1.0f),
        point_t<>( 0.5f, -0.5f, 1.0f),
        point_t<>(-0.5f, -0.5f, 1.0f),
        point_t<>(-0.5f,  0.5f, 1.0f)
    });

    world_polygon p1({
        point_t<>(-1.0f,  1.0f, 0.0f),
        point_t<>(-1.0f, -1.0f, 0.0f),
        point_t<>( 1.0f, -1.0f, 0.0f),
        point_t<>( 1.0f,  1.0f, 0.0f)
    });

    std::unique_ptr<std::vector<point_t<>>> exp(new std::vector<point_t<>>({
        point_t<>( 0.5f,  0.5f, 1.0f),
        point_t<>( 0.5f, -0.5f, 1.0f),
        point_t<>(-0.5f, -0.5f, 1.0f),
        point_t<>(-0.5f,  0.5f, 1.0f)
    }));

    p0.intersection(p1, point_t<>(0.0f, 0.0f, 1.0f), point_t<>(0.0f, 0.0f, 1.0f));

    /* Checks */
    BOOST_REQUIRE(p0.number_of_vertices() == static_cast<int>(exp->size()));
    for (int i = 0; i < p0.number_of_vertices(); ++i)
    {
        BOOST_CHECK(fabs(exp->at(i).x - p0.vertex(i).x) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).y - p0.vertex(i).y) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).z - p0.vertex(i).z) < result_tolerance);
    }
}

BOOST_AUTO_TEST_CASE( world_polygon_intersection_square5_test )
{
    /* Square test 5, poly b inside poly a */
    world_polygon p0({
        point_t<>( 1.0f,  1.0f, 1.0f),
        point_t<>( 1.0f, -1.0f, 1.0f),
        point_t<>(-1.0f, -1.0f, 1.0f),
        point_t<>(-1.0f,  1.0f, 1.0f)
    });

    world_polygon p1({
        point_t<>(-0.5f,  0.5f, 0.0f),
        point_t<>(-0.5f, -0.5f, 0.0f),
        point_t<>( 0.5f, -0.5f, 0.0f),
        point_t<>( 0.5f,  0.5f, 0.0f)
    });

    std::unique_ptr<std::vector<point_t<>>> exp(new std::vector<point_t<>>({
        point_t<>(-0.5f,  0.5f, 1.0f),
        point_t<>( 0.5f,  0.5f, 1.0f),
        point_t<>( 0.5f, -0.5f, 1.0f),
        point_t<>(-0.5f, -0.5f, 1.0f)
    }));

    p0.intersection(p1, point_t<>(0.0f, 0.0f, 1.0f), point_t<>(0.0f, 0.0f, 1.0f));

    /* Checks */
    BOOST_REQUIRE(p0.number_of_vertices() == static_cast<int>(exp->size()));
    for (int i = 0; i < p0.number_of_vertices(); ++i)
    {
        BOOST_CHECK(fabs(exp->at(i).x - p0.vertex(i).x) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).y - p0.vertex(i).y) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).z - p0.vertex(i).z) < result_tolerance);
    }
}

BOOST_AUTO_TEST_CASE( world_polygon_intersection_square6_test )
{
    /* Square test 5, poly b inside poly a */
    world_polygon p0({
        point_t<>( 1.0f,  1.0f, 1.0f),
        point_t<>( 1.0f, -1.0f, 1.0f),
        point_t<>(-1.0f, -1.0f, 1.0f),
        point_t<>(-1.0f,  1.0f, 1.0f)
    });

    world_polygon p1({
        point_t<>(-1.1f,  0.0f, 0.0f),
        point_t<>( 0.0f, -1.1f, 0.0f),
        point_t<>( 1.1f,  0.0f, 0.0f),
        point_t<>( 0.0f,  1.1f, 0.0f)
    });

    std::unique_ptr<std::vector<point_t<>>> exp(new std::vector<point_t<>>({
        point_t<>(-0.1f,  1.0f, 1.0f),
        point_t<>( 0.1f,  1.0f, 1.0f),
        point_t<>( 1.0f,  0.1f, 1.0f),
        point_t<>( 1.0f, -0.1f, 1.0f),
        point_t<>( 0.1f, -1.0f, 1.0f),
        point_t<>(-0.1f, -1.0f, 1.0f),
        point_t<>(-1.0f, -0.1f, 1.0f),
        point_t<>(-1.0f,  0.1f, 1.0f)
    }));

    p0.intersection(p1, point_t<>(0.0f, 0.0f, 1.0f), point_t<>(0.0f, 0.0f, 1.0f));

    /* Checks */
    BOOST_REQUIRE(p0.number_of_vertices() == static_cast<int>(exp->size()));
    for (int i = 0; i < p0.number_of_vertices(); ++i)
    {
        BOOST_CHECK(fabs(exp->at(i).x - p0.vertex(i).x) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).y - p0.vertex(i).y) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).z - p0.vertex(i).z) < result_tolerance);
    }

}

BOOST_AUTO_TEST_CASE( world_polygon_intersection_triangle_square0_test )
{
    /* Square test 3 */
    world_polygon p0({
        point_t<>( 0.5f,  1.5f, 1.0f),
        point_t<>( 0.5f, -1.5f, 1.0f),
        point_t<>(-0.5f, -1.5f, 1.0f),
        point_t<>(-0.5f,  1.5f, 1.0f)
    });

    world_polygon p1({
        point_t<>(-0.5f, -2.0f, 0.0f),
        point_t<>( 0.5f, -2.0f, 0.0f),
        point_t<>( 0.0f,  1.0f, 0.0f)
    });

    std::unique_ptr<std::vector<point_t<>>> exp(new std::vector<point_t<>>({
        point_t<>(  0.0f,                   1.0f, 1.0f),
        point_t<>(  0.5f - (1.0f / 12.0f), -1.5f, 1.0f),
        point_t<>( -0.5f + (1.0f / 12.0f), -1.5f, 1.0f)
    }));

    p0.intersection(p1, point_t<>(0.0f, 0.0f, 1.0f), point_t<>(0.0f, 0.0f, 1.0f));

    /* Checks */
    BOOST_REQUIRE(p0.number_of_vertices() == static_cast<int>(exp->size()));
    for (int i = 0; i < p0.number_of_vertices(); ++i)
    {
        BOOST_CHECK(fabs(exp->at(i).x - p0.vertex(i).x) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).y - p0.vertex(i).y) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).z - p0.vertex(i).z) < result_tolerance);
    }
}

BOOST_AUTO_TEST_CASE( world_polygon_intersection_octogon0_test )
{
    /* Octogon test 0 */
    world_polygon p0({
        point_t<>( 2.0f,  5.0f, 1.0f),
        point_t<>( 3.0f,  4.0f, 1.0f),
        point_t<>( 3.0f,  2.0f, 1.0f),
        point_t<>( 2.0f,  1.0f, 1.0f),
        point_t<>( 0.0f,  1.0f, 1.0f),
        point_t<>(-1.0f,  2.0f, 1.0f),
        point_t<>(-1.0f,  4.0f, 1.0f),
        point_t<>( 0.0f,  5.0f, 1.0f)
    });

    world_polygon p1({
        point_t<>(-1.0f,  2.0f, 1.0f),
        point_t<>(-2.0f,  1.0f, 1.0f),
        point_t<>(-2.0f, -1.0f, 1.0f),
        point_t<>(-1.0f, -2.0f, 1.0f),
        point_t<>( 1.0f, -2.0f, 1.0f), 
        point_t<>( 2.0f, -1.0f, 1.0f),
        point_t<>( 2.0f,  1.0f, 1.0f),
        point_t<>( 1.0f,  2.0f, 1.0f)
    });

    std::unique_ptr<std::vector<point_t<>>> exp(new std::vector<point_t<>>({
        point_t<>(-1.0f,  2.0f, 1.0f),
        point_t<>( 1.0f,  2.0f, 1.0f),
        point_t<>( 2.0f,  1.0f, 1.0f),
        point_t<>( 0.0f,  1.0f, 1.0f)
    }));

    p0.intersection(p1, point_t<>(0.0f, 0.0f, 1.0f), point_t<>(0.0f, 0.0f, 1.0f));

    /* Checks */
    BOOST_REQUIRE(p0.number_of_vertices() == static_cast<int>(exp->size()));
    for (int i = 0; i < p0.number_of_vertices(); ++i)
    {
        BOOST_CHECK(fabs(exp->at(i).x - p0.vertex(i).x) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).y - p0.vertex(i).y) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).z - p0.vertex(i).z) < result_tolerance);
    }
}

BOOST_AUTO_TEST_CASE( world_polygon_intersection_octogon1_test )
{
    /* Octogon test 1 */
    world_polygon p0({
        point_t<>( 0.25f,  4.0f, 2.0f), 
        point_t<>( 0.50f,  3.0f, 2.0f), 
        point_t<>( 0.50f, -3.0f, 2.0f), 
        point_t<>( 0.25f, -4.0f, 2.0f),
        point_t<>(-0.25f, -4.0f, 2.0f), 
        point_t<>(-0.50f, -3.0f, 2.0f), 
        point_t<>(-0.50f,  3.0f, 2.0f), 
        point_t<>(-0.25f,  4.0f, 2.0f)
    });

    world_polygon p1({
        point_t<>(-1.0f,  2.0f, 1.0f),
        point_t<>(-2.0f,  1.0f, 1.0f), 
        point_t<>(-2.0f, -1.0f, 1.0f), 
        point_t<>(-1.0f, -2.0f, 1.0f), 
        point_t<>( 1.0f, -2.0f, 1.0f),
        point_t<>( 2.0f, -1.0f, 1.0f), 
        point_t<>( 2.0f,  1.0f, 1.0f), 
        point_t<>( 1.0f,  2.0f, 1.0f) 
    });

    std::unique_ptr<std::vector<point_t<>>> exp(new std::vector<point_t<>>({
        point_t<>( 0.5f,  2.0f, 2.0f),
        point_t<>( 0.5f, -2.0f, 2.0f),
        point_t<>(-0.5f, -2.0f, 2.0f),
        point_t<>(-0.5f,  2.0f, 2.0f)
    }));

    p0.intersection(p1, point_t<>(0.0f, 0.0f, 2.0f), point_t<>(0.0f, 0.0f, 2.0f));

    /* Checks */
    BOOST_REQUIRE(p0.number_of_vertices() == static_cast<int>(exp->size()));
    for (int i = 0; i < p0.number_of_vertices(); ++i)
    {
        BOOST_CHECK(fabs(exp->at(i).x - p0.vertex(i).x) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).y - p0.vertex(i).y) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).z - p0.vertex(i).z) < result_tolerance);
    }
}

BOOST_AUTO_TEST_CASE( world_polygon_intersection_octogon2_test )
{
    /* Octogon test 2 */
    world_polygon p0({
        point_t<>( 4.25f,  4.0f, 2.0f),
        point_t<>( 4.50f,  3.0f, 2.0f),
        point_t<>( 4.50f, -3.0f, 2.0f),
        point_t<>( 4.25f, -4.0f, 2.0f),
        point_t<>(-0.25f, -4.0f, 2.0f),
        point_t<>(-0.50f, -3.0f, 2.0f),
        point_t<>(-0.50f,  3.0f, 2.0f),
        point_t<>(-0.25f,  4.0f, 2.0f)
    });

    world_polygon p1({
        point_t<>(-1.0f,  2.0f, 1.0f),
        point_t<>(-2.0f,  1.0f, 1.0f),
        point_t<>(-2.0f, -1.0f, 1.0f),
        point_t<>(-1.0f, -2.0f, 1.0f),
        point_t<>( 1.0f, -2.0f, 1.0f),
        point_t<>( 2.0f, -1.0f, 1.0f),
        point_t<>( 2.0f,  1.0f, 1.0f),
        point_t<>( 1.0f,  2.0f, 1.0f)
    });

    std::unique_ptr<std::vector<point_t<>>> exp(new std::vector<point_t<>>({
        point_t<>( 1.0f,  2.0f, 2.0f),
        point_t<>( 2.0f,  1.0f, 2.0f),
        point_t<>( 2.0f, -1.0f, 2.0f),
        point_t<>( 1.0f, -2.0f, 2.0f),
        point_t<>(-0.5f, -2.0f, 2.0f),
        point_t<>(-0.5f,  2.0f, 2.0f)
    }));

    p0.intersection(p1, point_t<>(0.0f, 0.0f, 1.0f), point_t<>(0.0f, 0.0f, 1.0f));

    /* Checks */
    BOOST_REQUIRE(p0.number_of_vertices() == static_cast<int>(exp->size()));
    for (int i = 0; i < p0.number_of_vertices(); ++i)
    {
        BOOST_CHECK(fabs(exp->at(i).x - p0.vertex(i).x) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).y - p0.vertex(i).y) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).z - p0.vertex(i).z) < result_tolerance);
    }
}

BOOST_AUTO_TEST_CASE( world_polygon_intersection_octogon3_test )
{
    /* Octogon test 3, poly a inside poly b */
    world_polygon p0({
        point_t<>( 4.25f,  4.0f, 2.0f),
        point_t<>( 4.50f,  3.0f, 2.0f),
        point_t<>( 4.50f, -3.0f, 2.0f),
        point_t<>( 4.25f, -4.0f, 2.0f),
        point_t<>(-0.25f, -4.0f, 2.0f),
        point_t<>(-0.50f, -3.0f, 2.0f),
        point_t<>(-0.50f,  3.0f, 2.0f),
        point_t<>(-0.25f,  4.0f, 2.0f)
    });

    world_polygon p1({
        point_t<>(-1.25f,  5.0f, 1.0f),
        point_t<>(-1.50f,  4.0f, 1.0f),
        point_t<>(-1.50f, -4.0f, 1.0f),
        point_t<>(-1.25f, -5.0f, 1.0f),
        point_t<>( 5.25f, -5.0f, 1.0f),
        point_t<>( 5.50f, -4.0f, 1.0f),
        point_t<>( 5.50f,  4.0f, 1.0f),
        point_t<>( 5.25f,  5.0f, 1.0f)
    });

    std::unique_ptr<std::vector<point_t<>>> exp(new std::vector<point_t<>>({
        point_t<>( 4.25f,  4.0f, 2.0f),
        point_t<>( 4.50f,  3.0f, 2.0f),
        point_t<>( 4.50f, -3.0f, 2.0f),
        point_t<>( 4.25f, -4.0f, 2.0f),
        point_t<>(-0.25f, -4.0f, 2.0f),
        point_t<>(-0.50f, -3.0f, 2.0f),
        point_t<>(-0.50f,  3.0f, 2.0f),
        point_t<>(-0.25f,  4.0f, 2.0f)
    }));

    p0.intersection(p1, point_t<>(0.0f, 0.0f, 1.0f), point_t<>(0.0f, 0.0f, 1.0f));

    /* Checks */
    BOOST_REQUIRE(p0.number_of_vertices() == static_cast<int>(exp->size()));
    for (int i = 0; i < p0.number_of_vertices(); ++i)
    {
        BOOST_CHECK(fabs(exp->at(i).x - p0.vertex(i).x) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).y - p0.vertex(i).y) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).z - p0.vertex(i).z) < result_tolerance);
    }
}

BOOST_AUTO_TEST_CASE( world_polygon_intersection_octogon4_test )
{
    /* Octogon test 4, poly b inside poly a */
    world_polygon p0({
        point_t<>( 5.25f,  5.0f, 2.0f),
        point_t<>( 5.50f,  4.0f, 2.0f),
        point_t<>( 5.50f, -4.0f, 2.0f),
        point_t<>( 5.25f, -5.0f, 2.0f),
        point_t<>(-1.25f, -5.0f, 2.0f),
        point_t<>(-1.50f, -4.0f, 2.0f),
        point_t<>(-1.50f,  4.0f, 2.0f),
        point_t<>(-1.25f,  5.0f, 2.0f)
    });

    world_polygon p1({
        point_t<>(-0.25f,  4.0f, 1.0f),
        point_t<>(-0.50f,  3.0f, 1.0f),
        point_t<>(-0.50f, -3.0f, 1.0f),
        point_t<>(-0.25f, -4.0f, 1.0f),
        point_t<>( 4.25f, -4.0f, 1.0f),
        point_t<>( 4.50f, -3.0f, 1.0f),
        point_t<>( 4.50f,  3.0f, 1.0f),
        point_t<>( 4.25f,  4.0f, 1.0f)
    });

    std::unique_ptr<std::vector<point_t<>>> exp(new std::vector<point_t<>>({
        point_t<>(-0.50f,  3.0f, 2.0f),
        point_t<>(-0.25f,  4.0f, 2.0f),
        point_t<>( 4.25f,  4.0f, 2.0f),
        point_t<>( 4.50f,  3.0f, 2.0f),
        point_t<>( 4.50f, -3.0f, 2.0f),
        point_t<>( 4.25f, -4.0f, 2.0f),
        point_t<>(-0.25f, -4.0f, 2.0f),
        point_t<>(-0.50f, -3.0f, 2.0f)
    }));

    p0.intersection(p1, point_t<>(0.0f, 0.0f, 1.0f), point_t<>(0.0f, 0.0f, 1.0f));

    /* Checks */
    BOOST_REQUIRE(p0.number_of_vertices() == static_cast<int>(exp->size()));
    for (int i = 0; i < p0.number_of_vertices(); ++i)
    {
        BOOST_CHECK(fabs(exp->at(i).x - p0.vertex(i).x) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).y - p0.vertex(i).y) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).z - p0.vertex(i).z) < result_tolerance);
    }
}

BOOST_AUTO_TEST_CASE( world_polygon_intersection_octogon_square0_test )
{
    /* Octogon test 4, poly b inside poly a */
    world_polygon p0({
        point_t<>( 5.25f,  5.0f, 2.0f),
        point_t<>( 5.50f,  4.0f, 2.0f),
        point_t<>( 5.50f, -4.0f, 2.0f),
        point_t<>( 5.25f, -5.0f, 2.0f),
        point_t<>(-1.25f, -5.0f, 2.0f),
        point_t<>(-1.50f, -4.0f, 2.0f),
        point_t<>(-1.50f,  4.0f, 2.0f),
        point_t<>(-1.25f,  5.0f, 2.0f)
    });

    world_polygon p1({
        point_t<>(-1.0f,  1.0f, 1.0f),
        point_t<>(-1.0f, -1.0f, 1.0f),
        point_t<>( 1.0f, -1.0f, 1.0f),
        point_t<>( 1.0f,  1.0f, 1.0f)
    });

    std::unique_ptr<std::vector<point_t<>>> exp(new std::vector<point_t<>>({
        point_t<>(-1.0f,  1.0f, 2.0f),
        point_t<>( 1.0f,  1.0f, 2.0f),
        point_t<>( 1.0f, -1.0f, 2.0f),
        point_t<>(-1.0f, -1.0f, 2.0f)
    }));

    p0.intersection(p1, point_t<>(0.0f, 0.0f, 1.0f), point_t<>(0.0f, 0.0f, 1.0f));

    /* Checks */
    BOOST_REQUIRE(p0.number_of_vertices() == static_cast<int>(exp->size()));
    for (int i = 0; i < p0.number_of_vertices(); ++i)
    {
        BOOST_CHECK(fabs(exp->at(i).x - p0.vertex(i).x) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).y - p0.vertex(i).y) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).z - p0.vertex(i).z) < result_tolerance);
    }
}

BOOST_AUTO_TEST_CASE( world_polygon_intersection_octogon_square1_test )
{
    /* Octogon test 4, poly b inside poly a */
    world_polygon p0({
        point_t<>( 5.25f,  5.0f, 2.0f),
        point_t<>( 5.50f,  4.0f, 2.0f),
        point_t<>( 5.50f, -4.0f, 2.0f),
        point_t<>( 5.25f, -5.0f, 2.0f),
        point_t<>(-1.25f, -5.0f, 2.0f),
        point_t<>(-1.50f, -4.0f, 2.0f),
        point_t<>(-1.50f,  4.0f, 2.0f),
        point_t<>(-1.25f,  5.0f, 2.0f)
    });

    world_polygon p1({
        point_t<>(-6.0f,  6.0f, 1.0f),
        point_t<>(-6.0f, -6.0f, 1.0f),
        point_t<>( 6.0f, -6.0f, 1.0f),
        point_t<>( 6.0f,  6.0f, 1.0f)
    });

    std::unique_ptr<std::vector<point_t<>>> exp(new std::vector<point_t<>>({
        point_t<>( 5.25f,  5.0f, 2.0f),
        point_t<>( 5.50f,  4.0f, 2.0f),
        point_t<>( 5.50f, -4.0f, 2.0f),
        point_t<>( 5.25f, -5.0f, 2.0f),
        point_t<>(-1.25f, -5.0f, 2.0f),
        point_t<>(-1.50f, -4.0f, 2.0f),
        point_t<>(-1.50f,  4.0f, 2.0f),
        point_t<>(-1.25f,  5.0f, 2.0f)
    }));

    p0.intersection(p1, point_t<>(0.0f, 0.0f, 1.0f), point_t<>(0.0f, 0.0f, 1.0f));

    /* Checks */
    BOOST_REQUIRE(p0.number_of_vertices() == static_cast<int>(exp->size()));
    for (int i = 0; i < p0.number_of_vertices(); ++i)
    {
        BOOST_CHECK(fabs(exp->at(i).x - p0.vertex(i).x) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).y - p0.vertex(i).y) < result_tolerance);
        BOOST_CHECK(fabs(exp->at(i).z - p0.vertex(i).z) < result_tolerance);
    }
}

BOOST_AUTO_TEST_SUITE_END()
}; /* namespace test */
}; /* namespace raptor_physics */
