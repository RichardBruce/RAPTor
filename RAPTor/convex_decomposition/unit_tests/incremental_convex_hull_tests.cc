
#ifdef STAND_ALONE
#define BOOST_TEST_MODULE incremental_convex_hull test
#endif /* #ifdef STAND_ALONE */


/* Standard headers */
#include <chrono>
#include <iostream>

/* Boost headers */
#include "boost/noncopyable.hpp"
#include "boost/test/unit_test.hpp"

/* Convex Decomposition headers */
#include "incremental_convex_hull.h"


namespace raptor_convex_decomposition
{
namespace test
{
/* Test data */
struct incremental_convex_hull_fixture : private boost::noncopyable
{
    incremental_convex_hull_fixture()
    {
        /* Add points */
        const std::vector<point_t> points_0(
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

        cube_uut.add_points(points_0);
        cube_uut.process();

        /* Add points */
        const std::vector<point_t> points_1(
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

        octagon_uut.add_points(points_1);
        octagon_uut.process();
    }

    incremental_convex_hull uut;
    incremental_convex_hull cube_uut;
    incremental_convex_hull octagon_uut;
};

BOOST_FIXTURE_TEST_SUITE( incremental_convex_hull_tests, incremental_convex_hull_fixture )

const float result_tolerance = 0.0005f;


BOOST_AUTO_TEST_CASE( hull_ctor_test )
{
    BOOST_CHECK(uut.number_of_vertices()    == 0);
    BOOST_CHECK(uut.number_of_triangles()   == 0);
    BOOST_CHECK(!uut.flat());
}

BOOST_AUTO_TEST_CASE( hull_add_points_test )
{
    /* Add points */
    const std::vector<point_t> points(
    {
        point_t(1.0f, 2.0f, 3.0f),
        point_t(2.0f, 3.0f, 7.0f),
        point_t(3.0f, 4.0f, 8.0f),
        point_t(4.0f, 5.0f, 9.0f),
        point_t(5.0f, 6.0f, 8.0f)
    });

    uut.add_points(points);

    /* Checks */
    BOOST_CHECK(uut.number_of_vertices()    == 5);
    BOOST_CHECK(uut.number_of_triangles()   == 0);
    BOOST_CHECK(!uut.flat());
}

// BOOST_AUTO_TEST_CASE( hull_add_point_test )
// {
//     /* Add points */
//     uut.add_point(point_t(1.0f, 2.0f, 3.0f), 0);
//     uut.add_point(point_t(2.0f, 3.0f, 7.0f), 1);
//     uut.add_point(point_t(3.0f, 4.0f, 8.0f), 2);
//     uut.add_point(point_t(4.0f, 5.0f, 9.0f), 3);
//     uut.add_point(point_t(5.0f, 6.0f, 8.0f), 4);

//     /* Checks */
//     BOOST_CHECK(uut.number_of_vertices()    == 5);
//     BOOST_CHECK(uut.number_of_triangles()   == 0);
//     BOOST_CHECK(!uut.flat());
// }

BOOST_AUTO_TEST_CASE( hull_clear_test )
{
    /* Add points */
    const std::vector<point_t> points(
    {
        point_t(1.0f, 2.0f, 3.0f),
        point_t(2.0f, 3.0f, 7.0f),
        point_t(3.0f, 4.0f, 8.0f),
        point_t(4.0f, 5.0f, 9.0f),
        point_t(5.0f, 6.0f, 8.0f)
    });

    uut.add_points(points);

    /* Checks */
    BOOST_CHECK(uut.number_of_vertices()    == 5);
    BOOST_CHECK(uut.number_of_triangles()   == 0);
    BOOST_CHECK(!uut.flat());

    uut.clear();

    /* Checks */
    BOOST_CHECK(uut.number_of_vertices()    == 0);
    BOOST_CHECK(uut.number_of_triangles()   == 0);
    BOOST_CHECK(!uut.flat());
}

/* Process tests */
BOOST_AUTO_TEST_CASE( hull_process_not_enough_points_test )
{
    /* Add points */
    const std::vector<point_t> points(
    {
        point_t(1.0f, 2.0f, 3.0f),
        point_t(2.0f, 3.0f, 7.0f)
    });

    uut.add_points(points);

    /* Process */
    BOOST_CHECK(uut.process() == convex_hull_error_t::not_enough_points);

    /* Checks */
    BOOST_CHECK(uut.number_of_vertices()    == 2);
    BOOST_CHECK(uut.number_of_triangles()   == 0);
    BOOST_CHECK(!uut.flat());

    std::vector<point_t>    mesh_points;
    std::vector<point_ti<>> mesh_triangles;
    uut.mesh().points_and_triangles(&mesh_points, &mesh_triangles);
    BOOST_CHECK(mesh_points[0] == point_t(1.0f, 2.0f, 3.0f));
    BOOST_CHECK(mesh_points[1] == point_t(2.0f, 3.0f, 7.0f));
}

BOOST_AUTO_TEST_CASE( hull_process_no_triangles_test )
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

    uut.add_points(points);

    /* Process */
    BOOST_CHECK(uut.process() == convex_hull_error_t::inconsistent);

    /* Checks */
    BOOST_CHECK(uut.number_of_vertices()    == 11);
    BOOST_CHECK(uut.number_of_triangles()   == 0);
    BOOST_CHECK(!uut.flat());

    std::vector<point_t>    mesh_points;
    std::vector<point_ti<>> mesh_triangles;
    uut.mesh().points_and_triangles(&mesh_points, &mesh_triangles);
    BOOST_CHECK(mesh_points[ 0] == point_t(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(mesh_points[ 1] == point_t(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(mesh_points[ 2] == point_t(1.0f, 0.0f, 0.0f));
    BOOST_CHECK(mesh_points[ 3] == point_t(1.0f, 0.0f, 0.0f));
    BOOST_CHECK(mesh_points[ 4] == point_t(3.0f, 0.0f, 0.0f));
    BOOST_CHECK(mesh_points[ 5] == point_t(4.0f, 0.0f, 0.0f));
    BOOST_CHECK(mesh_points[ 6] == point_t(5.0f, 0.0f, 0.0f));
    BOOST_CHECK(mesh_points[ 7] == point_t(6.0f, 0.0f, 0.0f));
    BOOST_CHECK(mesh_points[ 8] == point_t(8.0f, 0.0f, 0.0f));
    BOOST_CHECK(mesh_points[ 9] == point_t(9.0f, 0.0f, 0.0f));
    BOOST_CHECK(mesh_points[10] == point_t(2.0f, 0.0f, 0.0f));
}

BOOST_AUTO_TEST_CASE( hull_process_triangle_test )
{
    /* Add points */
    const std::vector<point_t> points(
    {
        point_t(0.0f, 0.0f, 0.0f),
        point_t(1.0f, 0.0f, 0.0f),
        point_t(0.0f, 1.0f, 0.0f)
    });

    uut.add_points(points);

    /* Process */
    BOOST_CHECK(uut.process() == convex_hull_error_t::ok);

    /* Checks */
    BOOST_CHECK(uut.number_of_vertices()    == 3);
    BOOST_CHECK(uut.number_of_triangles()   == 2);
    BOOST_CHECK(uut.flat());

    std::vector<point_t>    mesh_points;
    std::vector<point_ti<>> mesh_triangles;
    uut.mesh().points_and_triangles(&mesh_points, &mesh_triangles);
    BOOST_CHECK(mesh_points[0] == point_t(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(mesh_points[1] == point_t(1.0f, 0.0f, 0.0f));
    BOOST_CHECK(mesh_points[2] == point_t(0.0f, 1.0f, 0.0f));

    BOOST_CHECK(mesh_triangles[0] == point_ti<>(0, 1, 2));
    BOOST_CHECK(mesh_triangles[1] == point_ti<>(2, 1, 0));
}

BOOST_AUTO_TEST_CASE( hull_process_triangle_co_linear_start_test )
{
    /* Add points */
    const std::vector<point_t> points(
    {
        point_t(0.0f, 0.0f, 0.0f),
        point_t(0.2f, 0.0f, 0.0f),
        point_t(0.4f, 0.0f, 0.0f),
        point_t(0.6f, 0.0f, 0.0f),
        point_t(0.8f, 0.0f, 0.0f),
        point_t(1.0f, 0.0f, 0.0f),
        point_t(0.0f, 1.0f, 0.0f)
    });

    uut.add_points(points);

    /* Process */
    BOOST_CHECK(uut.process() == convex_hull_error_t::ok);

    /* Checks */
    BOOST_REQUIRE(uut.number_of_vertices()  == 4);
    BOOST_REQUIRE(uut.number_of_triangles() == 4);
    BOOST_CHECK(uut.flat());

    std::vector<point_t>    mesh_points;
    std::vector<point_ti<>> mesh_triangles;
    uut.mesh().points_and_triangles(&mesh_points, &mesh_triangles);
    BOOST_CHECK(mesh_points[0] == point_t(0.8f, 0.0f, 0.0f));
    BOOST_CHECK(mesh_points[1] == point_t(1.0f, 0.0f, 0.0f));
    BOOST_CHECK(mesh_points[2] == point_t(0.0f, 1.0f, 0.0f));
    BOOST_CHECK(mesh_points[3] == point_t(0.0f, 0.0f, 0.0f));

    BOOST_CHECK(mesh_triangles[0] == point_ti<>(2, 1, 0));
    BOOST_CHECK(mesh_triangles[1] == point_ti<>(2, 0, 3));
    BOOST_CHECK(mesh_triangles[2] == point_ti<>(1, 2, 0));
    BOOST_CHECK(mesh_triangles[3] == point_ti<>(0, 2, 3));
}

BOOST_AUTO_TEST_CASE( hull_process_octagon_test )
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

    uut.add_points(points);

    /* Process */
    BOOST_CHECK(uut.process() == convex_hull_error_t::ok);

    /* Checks */
    BOOST_REQUIRE(uut.number_of_vertices()  == 8);
    BOOST_REQUIRE(uut.number_of_triangles() == 12);
    BOOST_CHECK(uut.flat());

    std::vector<point_t>    mesh_points;
    std::vector<point_ti<>> mesh_triangles;
    uut.mesh().points_and_triangles(&mesh_points, &mesh_triangles);
    BOOST_CHECK(mesh_points[0] == point_t(1.0f, 0.0f, 0.0f));
    BOOST_CHECK(mesh_points[1] == point_t(0.0f, 1.0f, 0.0f));
    BOOST_CHECK(mesh_points[2] == point_t(0.0f, 2.0f, 0.0f));
    BOOST_CHECK(mesh_points[3] == point_t(1.0f, 3.0f, 0.0f));
    BOOST_CHECK(mesh_points[4] == point_t(2.0f, 3.0f, 0.0f));
    BOOST_CHECK(mesh_points[5] == point_t(3.0f, 2.0f, 0.0f));
    BOOST_CHECK(mesh_points[6] == point_t(3.0f, 1.0f, 0.0f));
    BOOST_CHECK(mesh_points[7] == point_t(2.0f, 0.0f, 0.0f));

    BOOST_CHECK(mesh_triangles[ 0] == point_ti<>(2, 1, 0));
    BOOST_CHECK(mesh_triangles[ 1] == point_ti<>(2, 0, 3));
    BOOST_CHECK(mesh_triangles[ 2] == point_ti<>(3, 0, 4));
    BOOST_CHECK(mesh_triangles[ 3] == point_ti<>(4, 0, 5));
    BOOST_CHECK(mesh_triangles[ 4] == point_ti<>(5, 0, 6));
    BOOST_CHECK(mesh_triangles[ 5] == point_ti<>(6, 0, 7));
    BOOST_CHECK(mesh_triangles[ 6] == point_ti<>(1, 2, 0));
    BOOST_CHECK(mesh_triangles[ 7] == point_ti<>(0, 2, 3));
    BOOST_CHECK(mesh_triangles[ 8] == point_ti<>(0, 3, 4));
    BOOST_CHECK(mesh_triangles[ 9] == point_ti<>(0, 4, 5));
    BOOST_CHECK(mesh_triangles[10] == point_ti<>(0, 5, 6));
    BOOST_CHECK(mesh_triangles[11] == point_ti<>(0, 6, 7));
}

BOOST_AUTO_TEST_CASE( hull_process_tetrahedron_test )
{
    /* Add points */
    const std::vector<point_t> points(
    {
        point_t(0.0f, 0.0f, 0.0f),
        point_t(1.0f, 0.0f, 0.0f),
        point_t(0.0f, 1.0f, 0.0f),
        point_t(0.0f, 0.0f, 1.0f)
    });

    uut.add_points(points);

    /* Process */
    BOOST_CHECK(uut.process() == convex_hull_error_t::ok);

    /* Checks */
    BOOST_CHECK(uut.number_of_vertices()    == 4);
    BOOST_CHECK(uut.number_of_triangles()   == 4);
    BOOST_CHECK(!uut.flat());

    std::vector<point_t>     mesh_points;
    std::vector<point_ti<>>  mesh_triangles;
    uut.mesh().points_and_triangles(&mesh_points, &mesh_triangles);
    BOOST_CHECK(mesh_points[0] == point_t(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(mesh_points[1] == point_t(1.0f, 0.0f, 0.0f));
    BOOST_CHECK(mesh_points[2] == point_t(0.0f, 1.0f, 0.0f));
    BOOST_CHECK(mesh_points[3] == point_t(0.0f, 0.0f, 1.0f));

    BOOST_CHECK(mesh_triangles[0] == point_ti<>(2, 1, 0));
    BOOST_CHECK(mesh_triangles[1] == point_ti<>(2, 0, 3));
    BOOST_CHECK(mesh_triangles[2] == point_ti<>(0, 1, 3));
    BOOST_CHECK(mesh_triangles[3] == point_ti<>(1, 2, 3));
}

BOOST_AUTO_TEST_CASE( hull_process_tetrahedron_co_linear_start_test )
{
    /* Add points */
    const std::vector<point_t> points(
    {
        point_t(0.0f, 0.0f, 0.0f),
        point_t(0.2f, 0.0f, 0.0f),
        point_t(0.4f, 0.0f, 0.0f),
        point_t(0.6f, 0.0f, 0.0f),
        point_t(0.8f, 0.0f, 0.0f),
        point_t(1.0f, 0.0f, 0.0f),
        point_t(0.0f, 1.0f, 0.0f),
        point_t(0.0f, 0.0f, 1.0f)
    });

    uut.add_points(points);

    /* Process */
    BOOST_CHECK(uut.process() == convex_hull_error_t::ok);

    /* Checks */
    BOOST_CHECK(uut.number_of_vertices()    == 5);
    BOOST_CHECK(uut.number_of_triangles()   == 6);
    BOOST_CHECK(!uut.flat());

    std::vector<point_t>    mesh_points;
    std::vector<point_ti<>> mesh_triangles;
    uut.mesh().points_and_triangles(&mesh_points, &mesh_triangles);
    BOOST_CHECK(mesh_points[0] == point_t(0.8f, 0.0f, 0.0f));
    BOOST_CHECK(mesh_points[1] == point_t(1.0f, 0.0f, 0.0f));
    BOOST_CHECK(mesh_points[2] == point_t(0.0f, 1.0f, 0.0f));
    BOOST_CHECK(mesh_points[3] == point_t(0.0f, 0.0f, 1.0f));
    BOOST_CHECK(mesh_points[4] == point_t(0.0f, 0.0f, 0.0f));

    BOOST_CHECK(mesh_triangles[0] == point_ti<>(2, 1, 0));
    BOOST_CHECK(mesh_triangles[1] == point_ti<>(0, 1, 3));
    BOOST_CHECK(mesh_triangles[2] == point_ti<>(1, 2, 3));
    BOOST_CHECK(mesh_triangles[3] == point_ti<>(2, 0, 4));
    BOOST_CHECK(mesh_triangles[4] == point_ti<>(0, 3, 4));
    BOOST_CHECK(mesh_triangles[5] == point_ti<>(3, 2, 4));
}

BOOST_AUTO_TEST_CASE( hull_process_pyramid_test )
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

    uut.add_points(points);

    /* Process */
    BOOST_CHECK(uut.process() == convex_hull_error_t::ok);

    /* Checks */
    BOOST_CHECK(uut.number_of_vertices()    == 5);
    BOOST_CHECK(uut.number_of_triangles()   == 6);
    BOOST_CHECK(!uut.flat());

    std::vector<point_t>    mesh_points;
    std::vector<point_ti<>> mesh_triangles;
    uut.mesh().points_and_triangles(&mesh_points, &mesh_triangles);
    BOOST_CHECK(mesh_points[0] == point_t(-0.5f, -0.5f,  0.0f));
    BOOST_CHECK(mesh_points[1] == point_t( 0.5f, -0.5f,  0.0f));
    BOOST_CHECK(mesh_points[2] == point_t(-0.5f,  0.5f,  0.0f));
    BOOST_CHECK(mesh_points[3] == point_t( 0.0f,  0.0f,  1.0f));
    BOOST_CHECK(mesh_points[4] == point_t( 0.5f,  0.5f,  0.0f));

    BOOST_CHECK(mesh_triangles[0] == point_ti<>(2, 1, 0));
    BOOST_CHECK(mesh_triangles[1] == point_ti<>(2, 0, 3));
    BOOST_CHECK(mesh_triangles[2] == point_ti<>(0, 1, 3));
    BOOST_CHECK(mesh_triangles[3] == point_ti<>(1, 2, 4));
    BOOST_CHECK(mesh_triangles[4] == point_ti<>(2, 3, 4));
    BOOST_CHECK(mesh_triangles[5] == point_ti<>(3, 1, 4));
}

BOOST_AUTO_TEST_CASE( hull_process_triangular_prism_test )
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

    uut.add_points(points);

    /* Process */
    BOOST_CHECK(uut.process() == convex_hull_error_t::ok);

    /* Checks */
    BOOST_CHECK(uut.number_of_vertices()    == 6);
    BOOST_CHECK(uut.number_of_triangles()   == 8);
    BOOST_CHECK(!uut.flat());

    std::vector<point_t>    mesh_points;
    std::vector<point_ti<>> mesh_triangles;
    uut.mesh().points_and_triangles(&mesh_points, &mesh_triangles);
    BOOST_CHECK(mesh_points[0] == point_t(-0.5f, -0.5f,  0.0f));
    BOOST_CHECK(mesh_points[1] == point_t( 0.5f, -0.5f,  0.0f));
    BOOST_CHECK(mesh_points[2] == point_t(-0.5f,  0.5f,  0.0f));
    BOOST_CHECK(mesh_points[3] == point_t(-0.5f,  0.0f,  1.0f));
    BOOST_CHECK(mesh_points[4] == point_t( 0.5f,  0.5f,  0.0f));
    BOOST_CHECK(mesh_points[5] == point_t( 0.5f,  0.0f,  1.0f));

    BOOST_CHECK(mesh_triangles[0] == point_ti<>(2, 1, 0));
    BOOST_CHECK(mesh_triangles[1] == point_ti<>(2, 0, 3));
    BOOST_CHECK(mesh_triangles[2] == point_ti<>(0, 1, 3));
    BOOST_CHECK(mesh_triangles[3] == point_ti<>(1, 2, 4));
    BOOST_CHECK(mesh_triangles[4] == point_ti<>(2, 3, 4));
    BOOST_CHECK(mesh_triangles[5] == point_ti<>(3, 1, 5));
    BOOST_CHECK(mesh_triangles[6] == point_ti<>(1, 4, 5));
    BOOST_CHECK(mesh_triangles[7] == point_ti<>(4, 3, 5));
}

BOOST_AUTO_TEST_CASE( hull_process_cube_test )
{
    /* Add points */
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

    uut.add_points(points);

    /* Process */
    BOOST_CHECK(uut.process() == convex_hull_error_t::ok);

    /* Checks */
    BOOST_CHECK(uut.number_of_vertices()    == 8);
    BOOST_CHECK(uut.number_of_triangles()   == 12);
    BOOST_CHECK(!uut.flat());

    std::vector<point_t>    mesh_points;
    std::vector<point_ti<>> mesh_triangles;
    uut.mesh().points_and_triangles(&mesh_points, &mesh_triangles);
    BOOST_CHECK(mesh_points[0] == point_t(-0.5f, -0.5f,  0.5f));
    BOOST_CHECK(mesh_points[1] == point_t( 0.5f, -0.5f,  0.5f));
    BOOST_CHECK(mesh_points[2] == point_t(-0.5f,  0.5f,  0.5f));
    BOOST_CHECK(mesh_points[3] == point_t(-0.5f, -0.5f, -0.5f));
    BOOST_CHECK(mesh_points[4] == point_t( 0.5f,  0.5f,  0.5f));
    BOOST_CHECK(mesh_points[5] == point_t( 0.5f, -0.5f, -0.5f));
    BOOST_CHECK(mesh_points[6] == point_t(-0.5f,  0.5f, -0.5f));
    BOOST_CHECK(mesh_points[7] == point_t( 0.5f,  0.5f, -0.5f));

    BOOST_CHECK(mesh_triangles[ 0] == point_ti<>(0, 1, 2));
    BOOST_CHECK(mesh_triangles[ 1] == point_ti<>(0, 2, 3));
    BOOST_CHECK(mesh_triangles[ 2] == point_ti<>(1, 0, 3));
    BOOST_CHECK(mesh_triangles[ 3] == point_ti<>(2, 1, 4));
    BOOST_CHECK(mesh_triangles[ 4] == point_ti<>(1, 3, 5));
    BOOST_CHECK(mesh_triangles[ 5] == point_ti<>(4, 1, 5));
    BOOST_CHECK(mesh_triangles[ 6] == point_ti<>(3, 2, 6));
    BOOST_CHECK(mesh_triangles[ 7] == point_ti<>(2, 4, 6));
    BOOST_CHECK(mesh_triangles[ 8] == point_ti<>(5, 3, 6));
    BOOST_CHECK(mesh_triangles[ 9] == point_ti<>(4, 5, 7));
    BOOST_CHECK(mesh_triangles[10] == point_ti<>(6, 4, 7));
    BOOST_CHECK(mesh_triangles[11] == point_ti<>(5, 6, 7));
}

BOOST_AUTO_TEST_CASE( hull_clear_and_process_cube_test )
{
    /* Clear */
    cube_uut.clear();

    /* Checks */
    BOOST_CHECK(cube_uut.number_of_vertices()    == 0);
    BOOST_CHECK(cube_uut.number_of_triangles()   == 0);
    BOOST_CHECK(!cube_uut.flat());

    /* Add points */
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

    cube_uut.add_points(points);

    /* Process */
    BOOST_CHECK(cube_uut.process() == convex_hull_error_t::ok);

    /* Checks */
    BOOST_CHECK(cube_uut.number_of_vertices()    == 8);
    BOOST_CHECK(cube_uut.number_of_triangles()   == 12);
    BOOST_CHECK(!cube_uut.flat());

    std::vector<point_t>    mesh_points;
    std::vector<point_ti<>> mesh_triangles;
    cube_uut.mesh().points_and_triangles(&mesh_points, &mesh_triangles);
    BOOST_CHECK(mesh_points[0] == point_t(-0.5f, -0.5f,  0.5f));
    BOOST_CHECK(mesh_points[1] == point_t( 0.5f, -0.5f,  0.5f));
    BOOST_CHECK(mesh_points[2] == point_t(-0.5f,  0.5f,  0.5f));
    BOOST_CHECK(mesh_points[3] == point_t(-0.5f, -0.5f, -0.5f));
    BOOST_CHECK(mesh_points[4] == point_t( 0.5f,  0.5f,  0.5f));
    BOOST_CHECK(mesh_points[5] == point_t( 0.5f, -0.5f, -0.5f));
    BOOST_CHECK(mesh_points[6] == point_t(-0.5f,  0.5f, -0.5f));
    BOOST_CHECK(mesh_points[7] == point_t( 0.5f,  0.5f, -0.5f));

    BOOST_CHECK(mesh_triangles[ 0] == point_ti<>(0, 1, 2));
    BOOST_CHECK(mesh_triangles[ 1] == point_ti<>(0, 2, 3));
    BOOST_CHECK(mesh_triangles[ 2] == point_ti<>(1, 0, 3));
    BOOST_CHECK(mesh_triangles[ 3] == point_ti<>(2, 1, 4));
    BOOST_CHECK(mesh_triangles[ 4] == point_ti<>(1, 3, 5));
    BOOST_CHECK(mesh_triangles[ 5] == point_ti<>(4, 1, 5));
    BOOST_CHECK(mesh_triangles[ 6] == point_ti<>(3, 2, 6));
    BOOST_CHECK(mesh_triangles[ 7] == point_ti<>(2, 4, 6));
    BOOST_CHECK(mesh_triangles[ 8] == point_ti<>(5, 3, 6));
    BOOST_CHECK(mesh_triangles[ 9] == point_ti<>(4, 5, 7));
    BOOST_CHECK(mesh_triangles[10] == point_ti<>(6, 4, 7));
    BOOST_CHECK(mesh_triangles[11] == point_ti<>(5, 6, 7));
}

BOOST_AUTO_TEST_CASE( hull_process_cube_points_inside_test )
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

    uut.add_points(points);

    /* Process */
    BOOST_CHECK(uut.process() == convex_hull_error_t::ok);

    /* Checks */
    BOOST_CHECK(uut.number_of_vertices()    == 8);
    BOOST_CHECK(uut.number_of_triangles()   == 12);
    BOOST_CHECK(!uut.flat());

    std::vector<point_t>    mesh_points;
    std::vector<point_ti<>> mesh_triangles;
    uut.mesh().points_and_triangles(&mesh_points, &mesh_triangles);
    BOOST_CHECK(mesh_points[0] == point_t(-0.5f, -0.5f,  0.5f));
    BOOST_CHECK(mesh_points[1] == point_t( 0.5f, -0.5f,  0.5f));
    BOOST_CHECK(mesh_points[2] == point_t(-0.5f,  0.5f,  0.5f));
    BOOST_CHECK(mesh_points[3] == point_t( 0.5f,  0.5f,  0.5f));
    BOOST_CHECK(mesh_points[4] == point_t(-0.5f, -0.5f, -0.5f));
    BOOST_CHECK(mesh_points[5] == point_t( 0.5f, -0.5f, -0.5f));
    BOOST_CHECK(mesh_points[6] == point_t(-0.5f,  0.5f, -0.5f));
    BOOST_CHECK(mesh_points[7] == point_t( 0.5f,  0.5f, -0.5f));

    BOOST_CHECK(mesh_triangles[ 0] == point_ti<>(0, 1, 2));
    BOOST_CHECK(mesh_triangles[ 1] == point_ti<>(2, 1, 3));
    BOOST_CHECK(mesh_triangles[ 2] == point_ti<>(1, 0, 4));
    BOOST_CHECK(mesh_triangles[ 3] == point_ti<>(0, 2, 4));
    BOOST_CHECK(mesh_triangles[ 4] == point_ti<>(3, 1, 5));
    BOOST_CHECK(mesh_triangles[ 5] == point_ti<>(1, 4, 5));
    BOOST_CHECK(mesh_triangles[ 6] == point_ti<>(2, 3, 6));
    BOOST_CHECK(mesh_triangles[ 7] == point_ti<>(4, 2, 6));
    BOOST_CHECK(mesh_triangles[ 8] == point_ti<>(5, 4, 6));
    BOOST_CHECK(mesh_triangles[ 9] == point_ti<>(3, 5, 7));
    BOOST_CHECK(mesh_triangles[10] == point_ti<>(6, 3, 7));
    BOOST_CHECK(mesh_triangles[11] == point_ti<>(5, 6, 7));
}

BOOST_AUTO_TEST_CASE( hull_process_tetrahedron_2_stage_test )
{
    /* Add points */
    const std::vector<point_t> points_0(
    {
        point_t(0.0f, 0.0f, 0.0f),
        point_t(1.0f, 0.0f, 0.0f),
        point_t(0.0f, 1.0f, 0.0f)
    });

    uut.add_points(points_0);

    /* Process */
    BOOST_CHECK(uut.process() == convex_hull_error_t::ok);

    /* Checks */
    BOOST_CHECK(uut.number_of_vertices()    == 3);
    BOOST_CHECK(uut.number_of_triangles()   == 2);
    BOOST_CHECK(uut.flat());

    /* Add more points */
    const std::vector<point_t> points_1(
    {
        point_t(0.0f, 0.0f, 1.0f)
    });

    uut.add_points(points_1);

    /* Process */
    BOOST_CHECK(uut.process() == convex_hull_error_t::ok);


    /* Checks */
    BOOST_CHECK(uut.number_of_vertices()    == 4);
    BOOST_CHECK(uut.number_of_triangles()   == 4);
    BOOST_CHECK(!uut.flat());

    std::vector<point_t>    mesh_points;
    std::vector<point_ti<>> mesh_triangles;
    uut.mesh().points_and_triangles(&mesh_points, &mesh_triangles);
    BOOST_CHECK(mesh_points[0] == point_t(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(mesh_points[1] == point_t(1.0f, 0.0f, 0.0f));
    BOOST_CHECK(mesh_points[2] == point_t(0.0f, 1.0f, 0.0f));
    BOOST_CHECK(mesh_points[3] == point_t(0.0f, 0.0f, 1.0f));

    BOOST_CHECK(mesh_triangles[0] == point_ti<>(2, 1, 0));
    BOOST_CHECK(mesh_triangles[1] == point_ti<>(2, 0, 3));
    BOOST_CHECK(mesh_triangles[2] == point_ti<>(0, 1, 3));
    BOOST_CHECK(mesh_triangles[3] == point_ti<>(1, 2, 3));
}

BOOST_AUTO_TEST_CASE( hull_process_octagon_pyramid_2_stages_test )
{
    /* Add points */
    const std::vector<point_t> points_0(
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

    uut.add_points(points_0);

    /* Process */
    BOOST_CHECK(uut.process() == convex_hull_error_t::ok);

    /* Checks */
    BOOST_CHECK(uut.number_of_vertices()    == 8);
    BOOST_CHECK(uut.number_of_triangles()   == 12);
    BOOST_CHECK(uut.flat());

    /* Add more points */
    const std::vector<point_t> points_1(
    {
        point_t( 1.5f,  2.5f,  0.5f)
    });

    uut.add_points(points_1);

    /* Process */
    BOOST_CHECK(uut.process() == convex_hull_error_t::ok);

    /* Checks */
    BOOST_CHECK(uut.number_of_vertices()    == 9);
    BOOST_CHECK(uut.number_of_triangles()   == 14);
    BOOST_CHECK(!uut.flat());

    std::vector<point_t>    mesh_points;
    std::vector<point_ti<>> mesh_triangles;
    uut.mesh().points_and_triangles(&mesh_points, &mesh_triangles);
    BOOST_CHECK(mesh_points[0] == point_t( 1.0f,  0.0f,  0.0f));
    BOOST_CHECK(mesh_points[1] == point_t( 0.0f,  1.0f,  0.0f));
    BOOST_CHECK(mesh_points[2] == point_t( 0.0f,  2.0f,  0.0f));
    BOOST_CHECK(mesh_points[3] == point_t( 1.5f,  2.5f,  0.5f));
    BOOST_CHECK(mesh_points[4] == point_t( 1.0f,  3.0f,  0.0f));
    BOOST_CHECK(mesh_points[5] == point_t( 2.0f,  3.0f,  0.0f));
    BOOST_CHECK(mesh_points[6] == point_t( 3.0f,  2.0f,  0.0f));
    BOOST_CHECK(mesh_points[7] == point_t( 3.0f,  1.0f,  0.0f));
    BOOST_CHECK(mesh_points[8] == point_t( 2.0f,  0.0f,  0.0f));

    BOOST_CHECK(mesh_triangles[ 0] == point_ti<>(0, 1, 2));
    BOOST_CHECK(mesh_triangles[ 1] == point_ti<>(1, 0, 3));
    BOOST_CHECK(mesh_triangles[ 2] == point_ti<>(2, 1, 3));
    BOOST_CHECK(mesh_triangles[ 3] == point_ti<>(0, 2, 4));
    BOOST_CHECK(mesh_triangles[ 4] == point_ti<>(2, 3, 4));
    BOOST_CHECK(mesh_triangles[ 5] == point_ti<>(0, 4, 5));
    BOOST_CHECK(mesh_triangles[ 6] == point_ti<>(4, 3, 5));
    BOOST_CHECK(mesh_triangles[ 7] == point_ti<>(0, 5, 6));
    BOOST_CHECK(mesh_triangles[ 8] == point_ti<>(5, 3, 6));
    BOOST_CHECK(mesh_triangles[ 9] == point_ti<>(0, 6, 7));
    BOOST_CHECK(mesh_triangles[10] == point_ti<>(6, 3, 7));
    BOOST_CHECK(mesh_triangles[11] == point_ti<>(3, 0, 8));
    BOOST_CHECK(mesh_triangles[12] == point_ti<>(0, 7, 8));
    BOOST_CHECK(mesh_triangles[13] == point_ti<>(7, 3, 8));
}

BOOST_AUTO_TEST_CASE( hull_process_cube_2_stages_test )
{
    /* Add points */
    const std::vector<point_t> points_0(
    {
        point_t(-0.5f, -0.5f,  0.0f),
        point_t( 0.5f, -0.5f,  0.0f),
        point_t(-0.5f,  0.5f,  0.0f),
        point_t( 0.5f,  0.5f,  0.0f),
        point_t(-0.5f,  0.0f,  0.5f),
        point_t( 0.5f,  0.0f,  0.5f)
    });

    uut.add_points(points_0);

    /* Process */
    BOOST_CHECK(uut.process() == convex_hull_error_t::ok);

    /* Checks */
    BOOST_CHECK(uut.number_of_vertices()    == 6);
    BOOST_CHECK(uut.number_of_triangles()   == 8);
    BOOST_CHECK(!uut.flat());

    /* Add more points */
    const std::vector<point_t> points_1(
    {
        point_t(-0.6f, -0.6f,  0.6f),
        point_t( 0.6f, -0.6f,  0.6f),
        point_t(-0.6f,  0.6f,  0.6f),
        point_t( 0.6f,  0.6f,  0.6f),
        point_t(-0.6f, -0.6f, -0.6f),
        point_t( 0.6f, -0.6f, -0.6f),
        point_t(-0.6f,  0.6f, -0.6f),
        point_t( 0.6f,  0.6f, -0.6f)
    });

    uut.add_points(points_1);

    /* Process */
    BOOST_CHECK(uut.process() == convex_hull_error_t::ok);

    /* Checks */
    BOOST_CHECK(uut.number_of_vertices()    == 8);
    BOOST_CHECK(uut.number_of_triangles()   == 12);
    BOOST_CHECK(!uut.flat());

    std::vector<point_t>    mesh_points;
    std::vector<point_ti<>> mesh_triangles;
    uut.mesh().points_and_triangles(&mesh_points, &mesh_triangles);
    BOOST_CHECK(mesh_points[0] == point_t(-0.6f, -0.6f,  0.6f));
    BOOST_CHECK(mesh_points[1] == point_t( 0.6f, -0.6f,  0.6f));
    BOOST_CHECK(mesh_points[2] == point_t(-0.6f,  0.6f,  0.6f));
    BOOST_CHECK(mesh_points[3] == point_t( 0.6f,  0.6f,  0.6f));
    BOOST_CHECK(mesh_points[4] == point_t(-0.6f, -0.6f, -0.6f));
    BOOST_CHECK(mesh_points[5] == point_t( 0.6f, -0.6f, -0.6f));
    BOOST_CHECK(mesh_points[6] == point_t(-0.6f,  0.6f, -0.6f));
    BOOST_CHECK(mesh_points[7] == point_t( 0.6f,  0.6f, -0.6f));

    BOOST_CHECK(mesh_triangles[ 0] == point_ti<>(0, 1, 2));
    BOOST_CHECK(mesh_triangles[ 1] == point_ti<>(2, 1, 3));
    BOOST_CHECK(mesh_triangles[ 2] == point_ti<>(1, 0, 4));
    BOOST_CHECK(mesh_triangles[ 3] == point_ti<>(0, 2, 4));
    BOOST_CHECK(mesh_triangles[ 4] == point_ti<>(3, 1, 5));
    BOOST_CHECK(mesh_triangles[ 5] == point_ti<>(1, 4, 5));
    BOOST_CHECK(mesh_triangles[ 6] == point_ti<>(2, 3, 6));
    BOOST_CHECK(mesh_triangles[ 7] == point_ti<>(4, 2, 6));
    BOOST_CHECK(mesh_triangles[ 8] == point_ti<>(5, 4, 6));
    BOOST_CHECK(mesh_triangles[ 9] == point_ti<>(3, 5, 7));
    BOOST_CHECK(mesh_triangles[10] == point_ti<>(6, 3, 7));
    BOOST_CHECK(mesh_triangles[11] == point_ti<>(5, 6, 7));
}

BOOST_AUTO_TEST_CASE( hull_min_process_not_enough_points_test )
{
    /* Add points */
    const std::vector<point_t> points(
    {
        point_t(1.0f, 2.0f, 3.0f),
        point_t(2.0f, 3.0f, 7.0f)
    });

    uut.add_points(points);

    /* Process */
    BOOST_CHECK(uut.process(5, 5.0f) == convex_hull_error_t::not_enough_points);

    /* Checks */
    BOOST_CHECK(uut.number_of_vertices()    == 2);
    BOOST_CHECK(uut.number_of_triangles()   == 0);
    BOOST_CHECK(!uut.flat());

    std::vector<point_t>    mesh_points;
    std::vector<point_ti<>> mesh_triangles;
    uut.mesh().points_and_triangles(&mesh_points, &mesh_triangles);
    BOOST_CHECK(mesh_points[0] == point_t(1.0f, 2.0f, 3.0f));
    BOOST_CHECK(mesh_points[1] == point_t(2.0f, 3.0f, 7.0f));
}

BOOST_AUTO_TEST_CASE( hull_min_process_not_enough_points_requested_test )
{
    /* Add points */
    const std::vector<point_t> points(
    {
        point_t(1.0f, 2.0f, 3.0f),
        point_t(1.0f, 2.0f, 3.0f),
        point_t(1.0f, 2.0f, 3.0f),
        point_t(1.0f, 2.0f, 3.0f),
        point_t(2.0f, 3.0f, 7.0f)
    });

    uut.add_points(points);

    /* Process */
    BOOST_CHECK(uut.process(2, 5.0f) == convex_hull_error_t::not_enough_points);

    /* Checks */
    BOOST_CHECK(uut.number_of_vertices()    == 5);
    BOOST_CHECK(uut.number_of_triangles()   == 0);
    BOOST_CHECK(!uut.flat());

    std::vector<point_t>    mesh_points;
    std::vector<point_ti<>> mesh_triangles;
    uut.mesh().points_and_triangles(&mesh_points, &mesh_triangles);
    BOOST_CHECK(mesh_points[0] == point_t(1.0f, 2.0f, 3.0f));
    BOOST_CHECK(mesh_points[1] == point_t(1.0f, 2.0f, 3.0f));
    BOOST_CHECK(mesh_points[2] == point_t(1.0f, 2.0f, 3.0f));
    BOOST_CHECK(mesh_points[3] == point_t(1.0f, 2.0f, 3.0f));
    BOOST_CHECK(mesh_points[4] == point_t(2.0f, 3.0f, 7.0f));
}

BOOST_AUTO_TEST_CASE( hull_min_process_no_triangles_test )
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

    uut.add_points(points);

    /* Process */
    BOOST_CHECK(uut.process(10, 5.0f) == convex_hull_error_t::inconsistent);

    /* Checks */
    BOOST_CHECK(uut.number_of_vertices()    == 11);
    BOOST_CHECK(uut.number_of_triangles()   == 0);
    BOOST_CHECK(!uut.flat());

    std::vector<point_t>    mesh_points;
    std::vector<point_ti<>> mesh_triangles;
    uut.mesh().points_and_triangles(&mesh_points, &mesh_triangles);
    BOOST_CHECK(mesh_points[ 0] == point_t(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(mesh_points[ 1] == point_t(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(mesh_points[ 2] == point_t(1.0f, 0.0f, 0.0f));
    BOOST_CHECK(mesh_points[ 3] == point_t(1.0f, 0.0f, 0.0f));
    BOOST_CHECK(mesh_points[ 4] == point_t(3.0f, 0.0f, 0.0f));
    BOOST_CHECK(mesh_points[ 5] == point_t(4.0f, 0.0f, 0.0f));
    BOOST_CHECK(mesh_points[ 6] == point_t(5.0f, 0.0f, 0.0f));
    BOOST_CHECK(mesh_points[ 7] == point_t(6.0f, 0.0f, 0.0f));
    BOOST_CHECK(mesh_points[ 8] == point_t(8.0f, 0.0f, 0.0f));
    BOOST_CHECK(mesh_points[ 9] == point_t(9.0f, 0.0f, 0.0f));
    BOOST_CHECK(mesh_points[10] == point_t(2.0f, 0.0f, 0.0f));
}

BOOST_AUTO_TEST_CASE( hull_min_process_triangle_test )
{
    /* Add points */
    const std::vector<point_t> points(
    {
        point_t(0.0f, 0.0f, 0.0f),
        point_t(1.0f, 0.0f, 0.0f),
        point_t(0.0f, 1.0f, 0.0f)
    });

    uut.add_points(points);

    /* Process */
    BOOST_CHECK(uut.process(10, 5.0f) == convex_hull_error_t::ok);

    /* Checks */
    BOOST_CHECK(uut.number_of_vertices()    == 3);
    BOOST_CHECK(uut.number_of_triangles()   == 2);
    BOOST_CHECK(uut.flat());

    std::vector<point_t>    mesh_points;
    std::vector<point_ti<>> mesh_triangles;
    uut.mesh().points_and_triangles(&mesh_points, &mesh_triangles);
    BOOST_CHECK(mesh_points[0] == point_t(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(mesh_points[1] == point_t(1.0f, 0.0f, 0.0f));
    BOOST_CHECK(mesh_points[2] == point_t(0.0f, 1.0f, 0.0f));

    BOOST_CHECK(mesh_triangles[0] == point_ti<>(0, 1, 2));
    BOOST_CHECK(mesh_triangles[1] == point_ti<>(2, 1, 0));
}

BOOST_AUTO_TEST_CASE( hull_min_process_triangle_co_linear_start_test )
{
    /* Add points */
    const std::vector<point_t> points(
    {
        point_t(0.0f, 0.0f, 0.0f),
        point_t(0.2f, 0.0f, 0.0f),
        point_t(0.4f, 0.0f, 0.0f),
        point_t(0.6f, 0.0f, 0.0f),
        point_t(0.8f, 0.0f, 0.0f),
        point_t(1.0f, 0.0f, 0.0f),
        point_t(0.0f, 1.0f, 0.0f)
    });

    uut.add_points(points);

    /* Process */
    BOOST_CHECK(uut.process(15, 0.0001f) == convex_hull_error_t::ok);

    /* Checks */
    BOOST_CHECK(uut.number_of_vertices()    == 4);
    BOOST_CHECK(uut.number_of_triangles()   == 4);
    BOOST_CHECK(uut.flat());

    std::vector<point_t>    mesh_points;
    std::vector<point_ti<>> mesh_triangles;
    uut.mesh().points_and_triangles(&mesh_points, &mesh_triangles);
    BOOST_CHECK(mesh_points[0] == point_t(0.8f, 0.0f, 0.0f));
    BOOST_CHECK(mesh_points[1] == point_t(1.0f, 0.0f, 0.0f));
    BOOST_CHECK(mesh_points[2] == point_t(0.0f, 1.0f, 0.0f));
    BOOST_CHECK(mesh_points[3] == point_t(0.0f, 0.0f, 0.0f));

    BOOST_CHECK(mesh_triangles[0] == point_ti<>(2, 1, 0));
    BOOST_CHECK(mesh_triangles[1] == point_ti<>(2, 0, 3));
    BOOST_CHECK(mesh_triangles[2] == point_ti<>(1, 2, 0));
    BOOST_CHECK(mesh_triangles[3] == point_ti<>(0, 2, 3));
}

BOOST_AUTO_TEST_CASE( hull_min_process_octagon_test )
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

    uut.add_points(points);

    /* Process */
    BOOST_CHECK(uut.process(15, 0.0001f) == convex_hull_error_t::ok);

    /* Checks */
    BOOST_CHECK(uut.number_of_vertices()    == 8);
    BOOST_CHECK(uut.number_of_triangles()   == 12);
    BOOST_CHECK(uut.flat());

    std::vector<point_t>    mesh_points;
    std::vector<point_ti<>> mesh_triangles;
    uut.mesh().points_and_triangles(&mesh_points, &mesh_triangles);
    BOOST_CHECK(mesh_points[0] == point_t(1.0f, 0.0f, 0.0f));
    BOOST_CHECK(mesh_points[1] == point_t(0.0f, 1.0f, 0.0f));
    BOOST_CHECK(mesh_points[2] == point_t(0.0f, 2.0f, 0.0f));
    BOOST_CHECK(mesh_points[3] == point_t(3.0f, 2.0f, 0.0f));
    BOOST_CHECK(mesh_points[4] == point_t(1.0f, 3.0f, 0.0f));
    BOOST_CHECK(mesh_points[5] == point_t(3.0f, 1.0f, 0.0f));
    BOOST_CHECK(mesh_points[6] == point_t(2.0f, 3.0f, 0.0f));
    BOOST_CHECK(mesh_points[7] == point_t(2.0f, 0.0f, 0.0f));

    BOOST_CHECK(mesh_triangles[ 0] == point_ti<>(2, 1, 0));
    BOOST_CHECK(mesh_triangles[ 1] == point_ti<>(2, 0, 3));
    BOOST_CHECK(mesh_triangles[ 2] == point_ti<>(2, 3, 4));
    BOOST_CHECK(mesh_triangles[ 3] == point_ti<>(3, 0, 5));
    BOOST_CHECK(mesh_triangles[ 4] == point_ti<>(4, 3, 6));
    BOOST_CHECK(mesh_triangles[ 5] == point_ti<>(5, 0, 7));
    BOOST_CHECK(mesh_triangles[ 6] == point_ti<>(1, 2, 0));
    BOOST_CHECK(mesh_triangles[ 7] == point_ti<>(0, 2, 3));
    BOOST_CHECK(mesh_triangles[ 8] == point_ti<>(3, 2, 4));
    BOOST_CHECK(mesh_triangles[ 9] == point_ti<>(0, 3, 5));
    BOOST_CHECK(mesh_triangles[10] == point_ti<>(3, 4, 6));
    BOOST_CHECK(mesh_triangles[11] == point_ti<>(0, 5, 7));
}

BOOST_AUTO_TEST_CASE( hull_min_process_tetrahedron_test )
{
    /* Add points */
    const std::vector<point_t> points(
    {
        point_t(0.0f, 0.0f, 0.0f),
        point_t(1.0f, 0.0f, 0.0f),
        point_t(0.0f, 1.0f, 0.0f),
        point_t(0.0f, 0.0f, 1.0f)
    });

    uut.add_points(points);

    /* Process */
    BOOST_CHECK(uut.process(15, 0.0001f) == convex_hull_error_t::ok);

    /* Checks */
    BOOST_CHECK(uut.number_of_vertices()    == 4);
    BOOST_CHECK(uut.number_of_triangles()   == 4);
    BOOST_CHECK(!uut.flat());

    std::vector<point_t>    mesh_points;
    std::vector<point_ti<>> mesh_triangles;
    uut.mesh().points_and_triangles(&mesh_points, &mesh_triangles);
    BOOST_CHECK(mesh_points[0] == point_t(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(mesh_points[1] == point_t(1.0f, 0.0f, 0.0f));
    BOOST_CHECK(mesh_points[2] == point_t(0.0f, 1.0f, 0.0f));
    BOOST_CHECK(mesh_points[3] == point_t(0.0f, 0.0f, 1.0f));

    BOOST_CHECK(mesh_triangles[0] == point_ti<>(2, 1, 0));
    BOOST_CHECK(mesh_triangles[1] == point_ti<>(2, 0, 3));
    BOOST_CHECK(mesh_triangles[2] == point_ti<>(0, 1, 3));
    BOOST_CHECK(mesh_triangles[3] == point_ti<>(1, 2, 3));
}

BOOST_AUTO_TEST_CASE( hull_min_process_tetrahedron_co_linear_start_test )
{
    /* Add points */
    const std::vector<point_t> points(
    {
        point_t(0.0f, 0.0f, 0.0f),
        point_t(0.2f, 0.0f, 0.0f),
        point_t(0.4f, 0.0f, 0.0f),
        point_t(0.6f, 0.0f, 0.0f),
        point_t(0.8f, 0.0f, 0.0f),
        point_t(1.0f, 0.0f, 0.0f),
        point_t(0.0f, 1.0f, 0.0f),
        point_t(0.0f, 0.0f, 1.0f)
    });

    uut.add_points(points);

    /* Process */
    BOOST_CHECK(uut.process(15, 0.0001f) == convex_hull_error_t::ok);

    /* Checks */
    BOOST_CHECK(uut.number_of_vertices()    == 5);
    BOOST_CHECK(uut.number_of_triangles()   == 6);
    BOOST_CHECK(!uut.flat());

    std::vector<point_t>    mesh_points;
    std::vector<point_ti<>> mesh_triangles;
    uut.mesh().points_and_triangles(&mesh_points, &mesh_triangles);
    BOOST_CHECK(mesh_points[0] == point_t(0.8f, 0.0f, 0.0f));
    BOOST_CHECK(mesh_points[1] == point_t(1.0f, 0.0f, 0.0f));
    BOOST_CHECK(mesh_points[2] == point_t(0.0f, 1.0f, 0.0f));
    BOOST_CHECK(mesh_points[3] == point_t(0.0f, 0.0f, 1.0f));
    BOOST_CHECK(mesh_points[4] == point_t(0.0f, 0.0f, 0.0f));

    BOOST_CHECK(mesh_triangles[0] == point_ti<>(2, 1, 0));
    BOOST_CHECK(mesh_triangles[1] == point_ti<>(0, 1, 3));
    BOOST_CHECK(mesh_triangles[2] == point_ti<>(1, 2, 3));
    BOOST_CHECK(mesh_triangles[3] == point_ti<>(2, 0, 4));
    BOOST_CHECK(mesh_triangles[4] == point_ti<>(0, 3, 4));
    BOOST_CHECK(mesh_triangles[5] == point_ti<>(3, 2, 4));
}

BOOST_AUTO_TEST_CASE( hull_min_process_pyramid_test )
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

    uut.add_points(points);

    /* Process */
    BOOST_CHECK(uut.process(15, 0.0001f) == convex_hull_error_t::ok);

    /* Checks */
    BOOST_CHECK(uut.number_of_vertices()    == 5);
    BOOST_CHECK(uut.number_of_triangles()   == 6);
    BOOST_CHECK(!uut.flat());

    std::vector<point_t>    mesh_points;
    std::vector<point_ti<>> mesh_triangles;
    uut.mesh().points_and_triangles(&mesh_points, &mesh_triangles);
    BOOST_CHECK(mesh_points[0] == point_t(-0.5f, -0.5f,  0.0f));
    BOOST_CHECK(mesh_points[1] == point_t( 0.5f, -0.5f,  0.0f));
    BOOST_CHECK(mesh_points[2] == point_t(-0.5f,  0.5f,  0.0f));
    BOOST_CHECK(mesh_points[3] == point_t( 0.0f,  0.0f,  1.0f));
    BOOST_CHECK(mesh_points[4] == point_t( 0.5f,  0.5f,  0.0f));

    BOOST_CHECK(mesh_triangles[0] == point_ti<>(2, 1, 0));
    BOOST_CHECK(mesh_triangles[1] == point_ti<>(2, 0, 3));
    BOOST_CHECK(mesh_triangles[2] == point_ti<>(0, 1, 3));
    BOOST_CHECK(mesh_triangles[3] == point_ti<>(1, 2, 4));
    BOOST_CHECK(mesh_triangles[4] == point_ti<>(2, 3, 4));
    BOOST_CHECK(mesh_triangles[5] == point_ti<>(3, 1, 4));
}

BOOST_AUTO_TEST_CASE( hull_min_process_triangular_prism_test )
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

    uut.add_points(points);

    /* Process */
    BOOST_CHECK(uut.process(15, 0.0001f) == convex_hull_error_t::ok);

    /* Checks */
    BOOST_CHECK(uut.number_of_vertices()    == 6);
    BOOST_CHECK(uut.number_of_triangles()   == 8);
    BOOST_CHECK(!uut.flat());

    std::vector<point_t>    mesh_points;
    std::vector<point_ti<>> mesh_triangles;
    uut.mesh().points_and_triangles(&mesh_points, &mesh_triangles);
    BOOST_CHECK(mesh_points[0] == point_t(-0.5f, -0.5f,  0.0f));
    BOOST_CHECK(mesh_points[1] == point_t( 0.5f, -0.5f,  0.0f));
    BOOST_CHECK(mesh_points[2] == point_t(-0.5f,  0.5f,  0.0f));
    BOOST_CHECK(mesh_points[3] == point_t(-0.5f,  0.0f,  1.0f));
    BOOST_CHECK(mesh_points[4] == point_t( 0.5f,  0.5f,  0.0f));
    BOOST_CHECK(mesh_points[5] == point_t( 0.5f,  0.0f,  1.0f));

    BOOST_CHECK(mesh_triangles[0] == point_ti<>(2, 1, 0));
    BOOST_CHECK(mesh_triangles[1] == point_ti<>(2, 0, 3));
    BOOST_CHECK(mesh_triangles[2] == point_ti<>(0, 1, 3));
    BOOST_CHECK(mesh_triangles[3] == point_ti<>(1, 2, 4));
    BOOST_CHECK(mesh_triangles[4] == point_ti<>(2, 3, 4));
    BOOST_CHECK(mesh_triangles[5] == point_ti<>(3, 1, 5));
    BOOST_CHECK(mesh_triangles[6] == point_ti<>(1, 4, 5));
    BOOST_CHECK(mesh_triangles[7] == point_ti<>(4, 3, 5));
}

BOOST_AUTO_TEST_CASE( hull_min_process_cube_test )
{
    /* Add points */
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

    uut.add_points(points);

    /* Process */
    BOOST_CHECK(uut.process(15, 0.0001f) == convex_hull_error_t::ok);

    /* Checks */
    BOOST_CHECK(uut.number_of_vertices()    == 8);
    BOOST_CHECK(uut.number_of_triangles()   == 12);
    BOOST_CHECK(!uut.flat());

    std::vector<point_t>    mesh_points;
    std::vector<point_ti<>> mesh_triangles;
    uut.mesh().points_and_triangles(&mesh_points, &mesh_triangles);
    BOOST_CHECK(mesh_points[0] == point_t(-0.5f, -0.5f,  0.5f));
    BOOST_CHECK(mesh_points[1] == point_t( 0.5f, -0.5f,  0.5f));
    BOOST_CHECK(mesh_points[2] == point_t(-0.5f,  0.5f,  0.5f));
    BOOST_CHECK(mesh_points[3] == point_t(-0.5f, -0.5f, -0.5f));
    BOOST_CHECK(mesh_points[4] == point_t( 0.5f,  0.5f, -0.5f));
    BOOST_CHECK(mesh_points[5] == point_t( 0.5f,  0.5f,  0.5f));
    BOOST_CHECK(mesh_points[6] == point_t( 0.5f, -0.5f, -0.5f));
    BOOST_CHECK(mesh_points[7] == point_t(-0.5f,  0.5f, -0.5f));

    BOOST_CHECK(mesh_triangles[ 0] == point_ti<>(0, 1, 2));
    BOOST_CHECK(mesh_triangles[ 1] == point_ti<>(0, 2, 3));
    BOOST_CHECK(mesh_triangles[ 2] == point_ti<>(1, 0, 3));
    BOOST_CHECK(mesh_triangles[ 3] == point_ti<>(2, 1, 5));
    BOOST_CHECK(mesh_triangles[ 4] == point_ti<>(4, 2, 5));
    BOOST_CHECK(mesh_triangles[ 5] == point_ti<>(1, 4, 5));
    BOOST_CHECK(mesh_triangles[ 6] == point_ti<>(1, 3, 6));
    BOOST_CHECK(mesh_triangles[ 7] == point_ti<>(4, 1, 6));
    BOOST_CHECK(mesh_triangles[ 8] == point_ti<>(3, 4, 6));
    BOOST_CHECK(mesh_triangles[ 9] == point_ti<>(3, 2, 7));
    BOOST_CHECK(mesh_triangles[10] == point_ti<>(2, 4, 7));
    BOOST_CHECK(mesh_triangles[11] == point_ti<>(4, 3, 7));
}

BOOST_AUTO_TEST_CASE( hull_clear_and_min_process_cube_test )
{
    /* Clear */
    cube_uut.clear();

    /* Checks */
    BOOST_CHECK(cube_uut.number_of_vertices()    == 0);
    BOOST_CHECK(cube_uut.number_of_triangles()   == 0);
    BOOST_CHECK(!cube_uut.flat());

    /* Add points */
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

    cube_uut.add_points(points);

    /* Process */
    BOOST_CHECK(cube_uut.process(15, 0.0001f) == convex_hull_error_t::ok);

    /* Checks */
    BOOST_CHECK(cube_uut.number_of_vertices()    == 8);
    BOOST_CHECK(cube_uut.number_of_triangles()   == 12);
    BOOST_CHECK(!cube_uut.flat());

    std::vector<point_t>    mesh_points;
    std::vector<point_ti<>> mesh_triangles;
    cube_uut.mesh().points_and_triangles(&mesh_points, &mesh_triangles);
    BOOST_CHECK(mesh_points[0] == point_t(-0.5f, -0.5f,  0.5f));
    BOOST_CHECK(mesh_points[1] == point_t( 0.5f, -0.5f,  0.5f));
    BOOST_CHECK(mesh_points[2] == point_t(-0.5f,  0.5f,  0.5f));
    BOOST_CHECK(mesh_points[3] == point_t(-0.5f, -0.5f, -0.5f));
    BOOST_CHECK(mesh_points[4] == point_t( 0.5f,  0.5f, -0.5f));
    BOOST_CHECK(mesh_points[5] == point_t( 0.5f,  0.5f,  0.5f));
    BOOST_CHECK(mesh_points[6] == point_t( 0.5f, -0.5f, -0.5f));
    BOOST_CHECK(mesh_points[7] == point_t(-0.5f,  0.5f, -0.5f));

    BOOST_CHECK(mesh_triangles[ 0] == point_ti<>(0, 1, 2));
    BOOST_CHECK(mesh_triangles[ 1] == point_ti<>(0, 2, 3));
    BOOST_CHECK(mesh_triangles[ 2] == point_ti<>(1, 0, 3));
    BOOST_CHECK(mesh_triangles[ 3] == point_ti<>(2, 1, 5));
    BOOST_CHECK(mesh_triangles[ 4] == point_ti<>(4, 2, 5));
    BOOST_CHECK(mesh_triangles[ 5] == point_ti<>(1, 4, 5));
    BOOST_CHECK(mesh_triangles[ 6] == point_ti<>(1, 3, 6));
    BOOST_CHECK(mesh_triangles[ 7] == point_ti<>(4, 1, 6));
    BOOST_CHECK(mesh_triangles[ 8] == point_ti<>(3, 4, 6));
    BOOST_CHECK(mesh_triangles[ 9] == point_ti<>(3, 2, 7));
    BOOST_CHECK(mesh_triangles[10] == point_ti<>(2, 4, 7));
    BOOST_CHECK(mesh_triangles[11] == point_ti<>(4, 3, 7));
}

BOOST_AUTO_TEST_CASE( hull_min_process_cube_points_inside_test )
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

    uut.add_points(points);

    /* Process */
    BOOST_CHECK(uut.process(15, 0.0001f) == convex_hull_error_t::ok);

    /* Checks */
    BOOST_CHECK(uut.number_of_vertices()    == 8);
    BOOST_CHECK(uut.number_of_triangles()   == 12);
    BOOST_CHECK(!uut.flat());

    std::vector<point_t>    mesh_points;
    std::vector<point_ti<>> mesh_triangles;
    uut.mesh().points_and_triangles(&mesh_points, &mesh_triangles);
    BOOST_CHECK(mesh_points[0] == point_t(-0.5f, -0.5f,  0.5f));
    BOOST_CHECK(mesh_points[1] == point_t( 0.5f, -0.5f,  0.5f));
    BOOST_CHECK(mesh_points[2] == point_t(-0.5f,  0.5f,  0.5f));
    BOOST_CHECK(mesh_points[3] == point_t(-0.5f, -0.5f, -0.5f));
    BOOST_CHECK(mesh_points[4] == point_t( 0.5f,  0.5f, -0.5f));
    BOOST_CHECK(mesh_points[5] == point_t( 0.5f,  0.5f,  0.5f));
    BOOST_CHECK(mesh_points[6] == point_t( 0.5f, -0.5f, -0.5f));
    BOOST_CHECK(mesh_points[7] == point_t(-0.5f,  0.5f, -0.5f));

    BOOST_CHECK(mesh_triangles[ 0] == point_ti<>(0, 1, 2));
    BOOST_CHECK(mesh_triangles[ 1] == point_ti<>(1, 0, 3));
    BOOST_CHECK(mesh_triangles[ 2] == point_ti<>(0, 2, 3));
    BOOST_CHECK(mesh_triangles[ 3] == point_ti<>(2, 1, 5));
    BOOST_CHECK(mesh_triangles[ 4] == point_ti<>(1, 4, 5));
    BOOST_CHECK(mesh_triangles[ 5] == point_ti<>(4, 2, 5));
    BOOST_CHECK(mesh_triangles[ 6] == point_ti<>(1, 3, 6));
    BOOST_CHECK(mesh_triangles[ 7] == point_ti<>(4, 1, 6));
    BOOST_CHECK(mesh_triangles[ 8] == point_ti<>(3, 4, 6));
    BOOST_CHECK(mesh_triangles[ 9] == point_ti<>(3, 2, 7));
    BOOST_CHECK(mesh_triangles[10] == point_ti<>(2, 4, 7));
    BOOST_CHECK(mesh_triangles[11] == point_ti<>(4, 3, 7));
}

BOOST_AUTO_TEST_CASE( hull_min_process_tetrahedron_2_stage_test )
{
    /* Add points */
    const std::vector<point_t> points_0(
    {
        point_t(0.0f, 0.0f, 0.0f),
        point_t(1.0f, 0.0f, 0.0f),
        point_t(0.0f, 1.0f, 0.0f)
    });

    uut.add_points(points_0);

    /* Process */
    BOOST_CHECK(uut.process(15, 0.0001f) == convex_hull_error_t::ok);

    /* Checks */
    BOOST_CHECK(uut.number_of_vertices()    == 3);
    BOOST_CHECK(uut.number_of_triangles()   == 2);
    BOOST_CHECK(uut.flat());

    /* Add more points */
    const std::vector<point_t> points_1(
    {
        point_t(0.0f, 0.0f, 1.0f)
    });

    uut.add_points(points_1);

    /* Process */
    BOOST_CHECK(uut.process(15, 0.0001f) == convex_hull_error_t::ok);


    /* Checks */
    BOOST_CHECK(uut.number_of_vertices()    == 4);
    BOOST_CHECK(uut.number_of_triangles()   == 4);
    BOOST_CHECK(!uut.flat());

    std::vector<point_t>    mesh_points;
    std::vector<point_ti<>> mesh_triangles;
    uut.mesh().points_and_triangles(&mesh_points, &mesh_triangles);
    BOOST_CHECK(mesh_points[0] == point_t(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(mesh_points[1] == point_t(1.0f, 0.0f, 0.0f));
    BOOST_CHECK(mesh_points[2] == point_t(0.0f, 1.0f, 0.0f));
    BOOST_CHECK(mesh_points[3] == point_t(0.0f, 0.0f, 1.0f));

    BOOST_CHECK(mesh_triangles[0] == point_ti<>(2, 1, 0));
    BOOST_CHECK(mesh_triangles[1] == point_ti<>(2, 0, 3));
    BOOST_CHECK(mesh_triangles[2] == point_ti<>(0, 1, 3));
    BOOST_CHECK(mesh_triangles[3] == point_ti<>(1, 2, 3));
}

BOOST_AUTO_TEST_CASE( hull_min_process_octagon_pyramid_2_stages_test )
{
    /* Add points */
    const std::vector<point_t> points_0(
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

    uut.add_points(points_0);

    /* Process */
    BOOST_CHECK(uut.process(15, 0.0001f) == convex_hull_error_t::ok);

    /* Checks */
    BOOST_CHECK(uut.number_of_vertices()    == 8);
    BOOST_CHECK(uut.number_of_triangles()   == 12);
    BOOST_CHECK(uut.flat());

    /* Add more points */
    const std::vector<point_t> points_1(
    {
        point_t( 1.5f,  2.5f,  0.5f)
    });

    uut.add_points(points_1);

    /* Process */
    BOOST_CHECK(uut.process(15, 0.0001f) == convex_hull_error_t::ok);

    /* Checks */
    BOOST_CHECK(uut.number_of_vertices()    == 9);
    BOOST_CHECK(uut.number_of_triangles()   == 14);
    BOOST_CHECK(!uut.flat());

    std::vector<point_t>    mesh_points;
    std::vector<point_ti<>> mesh_triangles;
    uut.mesh().points_and_triangles(&mesh_points, &mesh_triangles);
    BOOST_CHECK(mesh_points[0] == point_t( 1.0f,  0.0f,  0.0f));
    BOOST_CHECK(mesh_points[1] == point_t( 0.0f,  1.0f,  0.0f));
    BOOST_CHECK(mesh_points[2] == point_t( 0.0f,  2.0f,  0.0f));
    BOOST_CHECK(mesh_points[3] == point_t( 1.5f,  2.5f,  0.5f));
    BOOST_CHECK(mesh_points[4] == point_t( 3.0f,  2.0f,  0.0f));
    BOOST_CHECK(mesh_points[5] == point_t( 1.0f,  3.0f,  0.0f));
    BOOST_CHECK(mesh_points[6] == point_t( 3.0f,  1.0f,  0.0f));
    BOOST_CHECK(mesh_points[7] == point_t( 2.0f,  3.0f,  0.0f));
    BOOST_CHECK(mesh_points[8] == point_t( 2.0f,  0.0f,  0.0f));

    BOOST_CHECK(mesh_triangles[ 0] == point_ti<>(0, 1, 2));
    BOOST_CHECK(mesh_triangles[ 1] == point_ti<>(1, 0, 3));
    BOOST_CHECK(mesh_triangles[ 2] == point_ti<>(2, 1, 3));
    BOOST_CHECK(mesh_triangles[ 3] == point_ti<>(0, 2, 4));
    BOOST_CHECK(mesh_triangles[ 4] == point_ti<>(2, 3, 5));
    BOOST_CHECK(mesh_triangles[ 5] == point_ti<>(4, 2, 5));
    BOOST_CHECK(mesh_triangles[ 6] == point_ti<>(0, 4, 6));
    BOOST_CHECK(mesh_triangles[ 7] == point_ti<>(4, 3, 6));
    BOOST_CHECK(mesh_triangles[ 8] == point_ti<>(3, 4, 7));
    BOOST_CHECK(mesh_triangles[ 9] == point_ti<>(5, 3, 7));
    BOOST_CHECK(mesh_triangles[10] == point_ti<>(4, 5, 7));
    BOOST_CHECK(mesh_triangles[11] == point_ti<>(3, 0, 8));
    BOOST_CHECK(mesh_triangles[12] == point_ti<>(0, 6, 8));
    BOOST_CHECK(mesh_triangles[13] == point_ti<>(6, 3, 8));
}

BOOST_AUTO_TEST_CASE( hull_min_process_cube_2_stages_test )
{
    /* Add points */
    const std::vector<point_t> points_0(
    {
        point_t(-0.5f, -0.5f,  0.0f),
        point_t( 0.5f, -0.5f,  0.0f),
        point_t(-0.5f,  0.5f,  0.0f),
        point_t( 0.5f,  0.5f,  0.0f),
        point_t(-0.5f,  0.0f,  0.5f),
        point_t( 0.5f,  0.0f,  0.5f)
    });

    uut.add_points(points_0);

    /* Process */
    BOOST_CHECK(uut.process(15, 0.0001f) == convex_hull_error_t::ok);

    /* Checks */
    BOOST_CHECK(uut.number_of_vertices()    == 6);
    BOOST_CHECK(uut.number_of_triangles()   == 8);
    BOOST_CHECK(!uut.flat());

    /* Add more points */
    const std::vector<point_t> points_1(
    {
        point_t(-0.6f, -0.6f,  0.6f),
        point_t( 0.6f, -0.6f,  0.6f),
        point_t(-0.6f,  0.6f,  0.6f),
        point_t( 0.6f,  0.6f,  0.6f),
        point_t(-0.6f, -0.6f, -0.6f),
        point_t( 0.6f, -0.6f, -0.6f),
        point_t(-0.6f,  0.6f, -0.6f),
        point_t( 0.6f,  0.6f, -0.6f)
    });

    uut.add_points(points_1);

    /* Process */
    BOOST_CHECK(uut.process(15, 0.0001f) == convex_hull_error_t::ok);

    /* Checks */
    BOOST_CHECK(uut.number_of_vertices()    == 8);
    BOOST_CHECK(uut.number_of_triangles()   == 12);
    BOOST_CHECK(!uut.flat());

    std::vector<point_t>    mesh_points;
    std::vector<point_ti<>> mesh_triangles;
    uut.mesh().points_and_triangles(&mesh_points, &mesh_triangles);
    BOOST_CHECK(mesh_points[0] == point_t(-0.6f, -0.6f, -0.6f));
    BOOST_CHECK(mesh_points[1] == point_t( 0.6f,  0.6f, -0.6f));
    BOOST_CHECK(mesh_points[2] == point_t(-0.6f, -0.6f,  0.6f));
    BOOST_CHECK(mesh_points[3] == point_t(-0.6f,  0.6f,  0.6f));
    BOOST_CHECK(mesh_points[4] == point_t(-0.6f,  0.6f, -0.6f));
    BOOST_CHECK(mesh_points[5] == point_t( 0.6f, -0.6f, -0.6f));
    BOOST_CHECK(mesh_points[6] == point_t( 0.6f, -0.6f,  0.6f));
    BOOST_CHECK(mesh_points[7] == point_t( 0.6f,  0.6f,  0.6f));

    BOOST_CHECK(mesh_triangles[ 0] == point_ti<>(0, 2, 3));
    BOOST_CHECK(mesh_triangles[ 1] == point_ti<>(1, 0, 4));
    BOOST_CHECK(mesh_triangles[ 2] == point_ti<>(0, 3, 4));
    BOOST_CHECK(mesh_triangles[ 3] == point_ti<>(3, 1, 4));
    BOOST_CHECK(mesh_triangles[ 4] == point_ti<>(0, 1, 5));
    BOOST_CHECK(mesh_triangles[ 5] == point_ti<>(2, 0, 5));
    BOOST_CHECK(mesh_triangles[ 6] == point_ti<>(3, 2, 6));
    BOOST_CHECK(mesh_triangles[ 7] == point_ti<>(5, 1, 6));
    BOOST_CHECK(mesh_triangles[ 8] == point_ti<>(2, 5, 6));
    BOOST_CHECK(mesh_triangles[ 9] == point_ti<>(1, 3, 7));
    BOOST_CHECK(mesh_triangles[10] == point_ti<>(6, 1, 7));
    BOOST_CHECK(mesh_triangles[11] == point_ti<>(3, 6, 7));
}

BOOST_AUTO_TEST_CASE( hull_min_process_tetrahedron_point_limit_test )
{
    /* Add points */
    const std::vector<point_t> points(
    {
        point_t(0.0f, 0.0f, 0.0f),
        point_t(1.0f, 0.0f, 0.0f),
        point_t(0.0f, 1.0f, 0.0f),
        point_t(0.0f, 0.0f, 1.0f),
        point_t(1.0f, 0.0f, 0.5f),
        point_t(0.0f, 1.0f, 0.5f),
        point_t(1.0f, 0.0f, 0.6f),
        point_t(0.0f, 1.0f, 0.6f)
    });

    uut.add_points(points);

    /* Process */
    BOOST_CHECK(uut.process(4, 0.0001f) == convex_hull_error_t::ok);

    /* Checks */
    BOOST_CHECK(uut.number_of_vertices()    == 4);
    BOOST_CHECK(uut.number_of_triangles()   == 4);
    BOOST_CHECK(!uut.flat());

    std::vector<point_t>    mesh_points;
    std::vector<point_ti<>> mesh_triangles;
    uut.mesh().points_and_triangles(&mesh_points, &mesh_triangles);
    BOOST_CHECK(mesh_points[0] == point_t(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(mesh_points[1] == point_t(1.0f, 0.0f, 0.0f));
    BOOST_CHECK(mesh_points[2] == point_t(0.0f, 1.0f, 0.0f));
    BOOST_CHECK(mesh_points[3] == point_t(0.0f, 0.0f, 1.0f));

    BOOST_CHECK(mesh_triangles[0] == point_ti<>(2, 1, 0));
    BOOST_CHECK(mesh_triangles[1] == point_ti<>(2, 0, 3));
    BOOST_CHECK(mesh_triangles[2] == point_ti<>(0, 1, 3));
    BOOST_CHECK(mesh_triangles[3] == point_ti<>(1, 2, 3));
}

BOOST_AUTO_TEST_CASE( hull_min_process_tetrahedron_volume_limit_test )
{
    /* Add points */
    const std::vector<point_t> points(
    {
        point_t(0.0f, 0.0f, 0.0f),
        point_t(1.0f, 0.0f, 0.0f),
        point_t(0.0f, 1.0f, 0.0f),
        point_t(0.0f, 0.0f, 1.0f),
        point_t(1.0f, 0.0f, 0.5f),
        point_t(0.0f, 1.0f, 0.5f),
        point_t(1.0f, 0.0f, 0.6f),
        point_t(0.0f, 1.0f, 0.6f)
    });

    uut.add_points(points);

    /* Process */
    BOOST_CHECK(uut.process(15, 0.7f) == convex_hull_error_t::ok);

    /* Checks */
    BOOST_CHECK(uut.number_of_vertices()    == 4);
    BOOST_CHECK(uut.number_of_triangles()   == 4);
    BOOST_CHECK(!uut.flat());

    std::vector<point_t>    mesh_points;
    std::vector<point_ti<>> mesh_triangles;
    uut.mesh().points_and_triangles(&mesh_points, &mesh_triangles);
    BOOST_CHECK(mesh_points[0] == point_t(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(mesh_points[1] == point_t(1.0f, 0.0f, 0.0f));
    BOOST_CHECK(mesh_points[2] == point_t(0.0f, 1.0f, 0.0f));
    BOOST_CHECK(mesh_points[3] == point_t(0.0f, 0.0f, 1.0f));

    BOOST_CHECK(mesh_triangles[0] == point_ti<>(2, 1, 0));
    BOOST_CHECK(mesh_triangles[1] == point_ti<>(2, 0, 3));
    BOOST_CHECK(mesh_triangles[2] == point_ti<>(0, 1, 3));
    BOOST_CHECK(mesh_triangles[3] == point_ti<>(1, 2, 3));
}

BOOST_AUTO_TEST_CASE( process_regression_fail_0_test )
{
    /* Add points */
    const std::vector<point_t> points(
    {
        point_t(0.0559701f, -0.15963f,  -0.238758f),
        point_t(0.0559701f, -0.15963f,  -0.231284f),
        point_t(0.0559701f, -0.107412f, -0.238758f),
        point_t(0.212687f,  -0.107412f, -0.238758f),
        point_t(0.138041f,  -0.107412f, -0.179056f),
        point_t(0.182819f,  -0.15963f,  -0.179056f),
        point_t(0.19776f,   -0.159635f, -0.238758f),
        point_t(0.212687f,  -0.107412f, -0.186537f),
        point_t(0.108235f,  -0.15963f,  -0.186531f),
        point_t(0.182834f,  -0.159635f, -0.238758f),
        point_t(0.0634409f, -0.15963f,  -0.223827f),
        point_t(0.0559701f, -0.107412f, -0.231284f),
        point_t(0.182834f,  -0.159635f, -0.186537f),
        point_t(0.0709116f, -0.15963f,  -0.216376f),
        point_t(0.0634409f, -0.107412f, -0.223827f),
        point_t(0.19029f,   -0.159635f, -0.186537f),
        point_t(0.138041f,  -0.15963f,  -0.179056f),
        point_t(0.212687f,  -0.114876f, -0.238758f),
        point_t(0.19776f,   -0.159635f, -0.193982f),
        point_t(0.0709116f, -0.107412f, -0.216376f),
        point_t(0.108235f,  -0.107412f, -0.186531f),
        point_t(0.212687f,  -0.114876f, -0.186537f),
        point_t(0.19776f,   -0.107412f, -0.179056f),
        point_t(0.19776f,   -0.152166f, -0.186537f),
        point_t(0.19776f,   -0.114876f, -0.179056f)
    });

    uut.add_points(points);

    /* Process */
    BOOST_CHECK(uut.process() == convex_hull_error_t::ok);

    /* Checks */
    BOOST_CHECK(uut.number_of_vertices()    == 25);
    BOOST_CHECK(uut.number_of_triangles()   == 46);
    BOOST_CHECK(!uut.flat());

    std::vector<point_t>    mesh_points;
    std::vector<point_ti<>> mesh_triangles;
    uut.mesh().points_and_triangles(&mesh_points, &mesh_triangles);
    BOOST_CHECK(mesh_points[ 0] == point_t(0.0559701f, -0.15963f,  -0.238758f));
    BOOST_CHECK(mesh_points[ 1] == point_t(0.0559701f, -0.15963f,  -0.231284f));
    BOOST_CHECK(mesh_points[ 2] == point_t(0.0559701f, -0.107412f, -0.238758f));
    BOOST_CHECK(mesh_points[ 3] == point_t(0.212687f,  -0.107412f, -0.238758f));
    BOOST_CHECK(mesh_points[ 4] == point_t(0.138041f,  -0.107412f, -0.179056f));
    BOOST_CHECK(mesh_points[ 5] == point_t(0.182819f,  -0.15963f,  -0.179056f));
    BOOST_CHECK(mesh_points[ 6] == point_t(0.19776f,   -0.159635f, -0.238758f));
    BOOST_CHECK(mesh_points[ 7] == point_t(0.212687f,  -0.107412f, -0.186537f));
    BOOST_CHECK(mesh_points[ 8] == point_t(0.108235f,  -0.15963f,  -0.186531f));
    BOOST_CHECK(mesh_points[ 9] == point_t(0.182834f,  -0.159635f, -0.238758f));
    BOOST_CHECK(mesh_points[10] == point_t(0.0634409f, -0.15963f,  -0.223827f));
    BOOST_CHECK(mesh_points[11] == point_t(0.0559701f, -0.107412f, -0.231284f));
    BOOST_CHECK(mesh_points[12] == point_t(0.182834f,  -0.159635f, -0.186537f));
    BOOST_CHECK(mesh_points[13] == point_t(0.0709116f, -0.15963f,  -0.216376f));
    BOOST_CHECK(mesh_points[14] == point_t(0.0634409f, -0.107412f, -0.223827f));
    BOOST_CHECK(mesh_points[15] == point_t(0.19029f,   -0.159635f, -0.186537f));
    BOOST_CHECK(mesh_points[16] == point_t(0.138041f,  -0.15963f,  -0.179056f));
    BOOST_CHECK(mesh_points[17] == point_t(0.212687f,  -0.114876f, -0.238758f));
    BOOST_CHECK(mesh_points[18] == point_t(0.19776f,   -0.159635f, -0.193982f));
    BOOST_CHECK(mesh_points[19] == point_t(0.0709116f, -0.107412f, -0.216376f));
    BOOST_CHECK(mesh_points[20] == point_t(0.108235f,  -0.107412f, -0.186531f));
    BOOST_CHECK(mesh_points[21] == point_t(0.212687f,  -0.114876f, -0.186537f));
    BOOST_CHECK(mesh_points[22] == point_t(0.19776f,   -0.107412f, -0.179056f));
    BOOST_CHECK(mesh_points[23] == point_t(0.19776f,   -0.152166f, -0.186537f));
    BOOST_CHECK(mesh_points[24] == point_t(0.19776f,   -0.114876f, -0.179056f));

    BOOST_CHECK(mesh_triangles[ 0] == point_ti<>( 0,  1,  2));
    BOOST_CHECK(mesh_triangles[ 1] == point_ti<>( 0,  2,  3));
    BOOST_CHECK(mesh_triangles[ 2] == point_ti<>( 3,  2,  4));
    BOOST_CHECK(mesh_triangles[ 3] == point_ti<>( 0,  3,  6));
    BOOST_CHECK(mesh_triangles[ 4] == point_ti<>( 3,  4,  7));
    BOOST_CHECK(mesh_triangles[ 5] == point_ti<>( 1,  0,  9));
    BOOST_CHECK(mesh_triangles[ 6] == point_ti<>( 0,  6,  9));
    BOOST_CHECK(mesh_triangles[ 7] == point_ti<>( 2,  1, 11));
    BOOST_CHECK(mesh_triangles[ 8] == point_ti<>( 4,  2, 11));
    BOOST_CHECK(mesh_triangles[ 9] == point_ti<>( 1, 10, 11));
    BOOST_CHECK(mesh_triangles[10] == point_ti<>( 1,  9, 12));
    BOOST_CHECK(mesh_triangles[11] == point_ti<>( 9,  6, 12));
    BOOST_CHECK(mesh_triangles[12] == point_ti<>(10,  1, 12));
    BOOST_CHECK(mesh_triangles[13] == point_ti<>(12,  8, 13));
    BOOST_CHECK(mesh_triangles[14] == point_ti<>(10, 12, 13));
    BOOST_CHECK(mesh_triangles[15] == point_ti<>( 4, 11, 14));
    BOOST_CHECK(mesh_triangles[16] == point_ti<>(11, 10, 14));
    BOOST_CHECK(mesh_triangles[17] == point_ti<>(10, 13, 14));
    BOOST_CHECK(mesh_triangles[18] == point_ti<>( 5, 12, 15));
    BOOST_CHECK(mesh_triangles[19] == point_ti<>(12,  6, 15));
    BOOST_CHECK(mesh_triangles[20] == point_ti<>( 5,  4, 16));
    BOOST_CHECK(mesh_triangles[21] == point_ti<>( 4,  8, 16));
    BOOST_CHECK(mesh_triangles[22] == point_ti<>(12,  5, 16));
    BOOST_CHECK(mesh_triangles[23] == point_ti<>( 8, 12, 16));
    BOOST_CHECK(mesh_triangles[24] == point_ti<>( 6,  3, 17));
    BOOST_CHECK(mesh_triangles[25] == point_ti<>( 3,  7, 17));
    BOOST_CHECK(mesh_triangles[26] == point_ti<>( 5, 15, 18));
    BOOST_CHECK(mesh_triangles[27] == point_ti<>(15,  6, 18));
    BOOST_CHECK(mesh_triangles[28] == point_ti<>( 6, 17, 18));
    BOOST_CHECK(mesh_triangles[29] == point_ti<>(13,  8, 19));
    BOOST_CHECK(mesh_triangles[30] == point_ti<>( 4, 14, 19));
    BOOST_CHECK(mesh_triangles[31] == point_ti<>(14, 13, 19));
    BOOST_CHECK(mesh_triangles[32] == point_ti<>( 8,  4, 20));
    BOOST_CHECK(mesh_triangles[33] == point_ti<>( 4, 19, 20));
    BOOST_CHECK(mesh_triangles[34] == point_ti<>(19,  8, 20));
    BOOST_CHECK(mesh_triangles[35] == point_ti<>(17,  7, 21));
    BOOST_CHECK(mesh_triangles[36] == point_ti<>(18, 17, 21));
    BOOST_CHECK(mesh_triangles[37] == point_ti<>( 4,  5, 22));
    BOOST_CHECK(mesh_triangles[38] == point_ti<>( 7,  4, 22));
    BOOST_CHECK(mesh_triangles[39] == point_ti<>(21,  7, 22));
    BOOST_CHECK(mesh_triangles[40] == point_ti<>( 5, 18, 23));
    BOOST_CHECK(mesh_triangles[41] == point_ti<>(21,  5, 23));
    BOOST_CHECK(mesh_triangles[42] == point_ti<>(18, 21, 23));
    BOOST_CHECK(mesh_triangles[43] == point_ti<>( 5, 21, 24));
    BOOST_CHECK(mesh_triangles[44] == point_ti<>(22,  5, 24));
    BOOST_CHECK(mesh_triangles[45] == point_ti<>(21, 22, 24));
}

BOOST_AUTO_TEST_CASE( process_regression_fail_1_test )
{
    /* Add points */
    const std::vector<point_t> points(
    {
        point_t(-2.7423725f, 38.2191810f, 20.7063713f),
        point_t(-3.5658187f, 38.2191810f, 20.7063713f),
        point_t(-2.7423725f, 11.1007184f, 20.7063713f),
        point_t(15.3382873f, 38.2191810f,  1.8022613f),
        // point_t(13.6913948f,  8.6301374f,  1.8022613f),
        point_t(15.3382873f,  8.6301374f,  2.6257076f),
        // point_t(-3.5658187f, 38.2191810f, 19.0613288f),
        // point_t(-3.5658187f, 11.1007184f, 19.0613288f),
        // point_t(13.6932449f, 38.2191810f,  1.8022613f),
        point_t(15.3382873f, 38.2191810f,  2.6257076f),
        // point_t(-1.0991806f,  8.6301374f, 18.2397327f),
        // point_t(-1.0991806f,  8.6301374f, 16.5946903f),
        // point_t(15.3382873f,  8.6301374f,  1.8022613f),
        // point_t(-3.5658187f, 11.1007184f, 20.7063713f),
        point_t(-0.2757344f,  8.6301374f, 18.2397327f),
        // point_t(13.6913948f, 11.0978221f,  1.8022613f)
    });

    uut.add_points(points);

    /* Process */
    BOOST_CHECK(uut.process(64, 31.0658f) == convex_hull_error_t::ok);

    /* Checks */
    // BOOST_CHECK(uut.number_of_vertices()    == 25);
    // BOOST_CHECK(uut.number_of_triangles()   == 46);
    // BOOST_CHECK(!uut.flat());

    // std::vector<point_t>    mesh_points;
    // std::vector<point_ti<>> mesh_triangles;
    // uut.mesh().points_and_triangles(&mesh_points, &mesh_triangles);
    // BOOST_CHECK(mesh_points[ 0] == point_t(0.0559701f, -0.15963f,  -0.238758f));
    // BOOST_CHECK(mesh_points[ 1] == point_t(0.0559701f, -0.15963f,  -0.231284f));
    // BOOST_CHECK(mesh_points[ 2] == point_t(0.0559701f, -0.107412f, -0.238758f));
    // BOOST_CHECK(mesh_points[ 3] == point_t(0.212687f,  -0.107412f, -0.238758f));
    // BOOST_CHECK(mesh_points[ 4] == point_t(0.138041f,  -0.107412f, -0.179056f));
    // BOOST_CHECK(mesh_points[ 5] == point_t(0.182819f,  -0.15963f,  -0.179056f));
    // BOOST_CHECK(mesh_points[ 6] == point_t(0.19776f,   -0.159635f, -0.238758f));
    // BOOST_CHECK(mesh_points[ 7] == point_t(0.212687f,  -0.107412f, -0.186537f));
    // BOOST_CHECK(mesh_points[ 8] == point_t(0.108235f,  -0.15963f,  -0.186531f));
    // BOOST_CHECK(mesh_points[ 9] == point_t(0.182834f,  -0.159635f, -0.238758f));
    // BOOST_CHECK(mesh_points[10] == point_t(0.0634409f, -0.15963f,  -0.223827f));
    // BOOST_CHECK(mesh_points[11] == point_t(0.0559701f, -0.107412f, -0.231284f));
    // BOOST_CHECK(mesh_points[12] == point_t(0.182834f,  -0.159635f, -0.186537f));
    // BOOST_CHECK(mesh_points[13] == point_t(0.0709116f, -0.15963f,  -0.216376f));
    // BOOST_CHECK(mesh_points[14] == point_t(0.0634409f, -0.107412f, -0.223827f));
    // BOOST_CHECK(mesh_points[15] == point_t(0.19029f,   -0.159635f, -0.186537f));
    // BOOST_CHECK(mesh_points[16] == point_t(0.138041f,  -0.15963f,  -0.179056f));
    // BOOST_CHECK(mesh_points[17] == point_t(0.212687f,  -0.114876f, -0.238758f));
    // BOOST_CHECK(mesh_points[18] == point_t(0.19776f,   -0.159635f, -0.193982f));
    // BOOST_CHECK(mesh_points[19] == point_t(0.0709116f, -0.107412f, -0.216376f));
    // BOOST_CHECK(mesh_points[20] == point_t(0.108235f,  -0.107412f, -0.186531f));
    // BOOST_CHECK(mesh_points[21] == point_t(0.212687f,  -0.114876f, -0.186537f));
    // BOOST_CHECK(mesh_points[22] == point_t(0.19776f,   -0.107412f, -0.179056f));
    // BOOST_CHECK(mesh_points[23] == point_t(0.19776f,   -0.152166f, -0.186537f));
    // BOOST_CHECK(mesh_points[24] == point_t(0.19776f,   -0.114876f, -0.179056f));

    // BOOST_CHECK(mesh_triangles[ 0] == point_ti<>( 0,  1,  2));
    // BOOST_CHECK(mesh_triangles[ 1] == point_ti<>( 0,  2,  3));
    // BOOST_CHECK(mesh_triangles[ 2] == point_ti<>( 3,  2,  4));
    // BOOST_CHECK(mesh_triangles[ 3] == point_ti<>( 0,  3,  6));
    // BOOST_CHECK(mesh_triangles[ 4] == point_ti<>( 3,  4,  7));
    // BOOST_CHECK(mesh_triangles[ 5] == point_ti<>( 1,  0,  9));
    // BOOST_CHECK(mesh_triangles[ 6] == point_ti<>( 0,  6,  9));
    // BOOST_CHECK(mesh_triangles[ 7] == point_ti<>( 2,  1, 11));
    // BOOST_CHECK(mesh_triangles[ 8] == point_ti<>( 4,  2, 11));
    // BOOST_CHECK(mesh_triangles[ 9] == point_ti<>( 1, 10, 11));
    // BOOST_CHECK(mesh_triangles[10] == point_ti<>( 1,  9, 12));
    // BOOST_CHECK(mesh_triangles[11] == point_ti<>( 9,  6, 12));
    // BOOST_CHECK(mesh_triangles[12] == point_ti<>(10,  1, 12));
    // BOOST_CHECK(mesh_triangles[13] == point_ti<>(12,  8, 13));
    // BOOST_CHECK(mesh_triangles[14] == point_ti<>(10, 12, 13));
    // BOOST_CHECK(mesh_triangles[15] == point_ti<>( 4, 11, 14));
    // BOOST_CHECK(mesh_triangles[16] == point_ti<>(11, 10, 14));
    // BOOST_CHECK(mesh_triangles[17] == point_ti<>(10, 13, 14));
    // BOOST_CHECK(mesh_triangles[18] == point_ti<>( 5, 12, 15));
    // BOOST_CHECK(mesh_triangles[19] == point_ti<>(12,  6, 15));
    // BOOST_CHECK(mesh_triangles[20] == point_ti<>( 5,  4, 16));
    // BOOST_CHECK(mesh_triangles[21] == point_ti<>( 4,  8, 16));
    // BOOST_CHECK(mesh_triangles[22] == point_ti<>(12,  5, 16));
    // BOOST_CHECK(mesh_triangles[23] == point_ti<>( 8, 12, 16));
    // BOOST_CHECK(mesh_triangles[24] == point_ti<>( 6,  3, 17));
    // BOOST_CHECK(mesh_triangles[25] == point_ti<>( 3,  7, 17));
    // BOOST_CHECK(mesh_triangles[26] == point_ti<>( 5, 15, 18));
    // BOOST_CHECK(mesh_triangles[27] == point_ti<>(15,  6, 18));
    // BOOST_CHECK(mesh_triangles[28] == point_ti<>( 6, 17, 18));
    // BOOST_CHECK(mesh_triangles[29] == point_ti<>(13,  8, 19));
    // BOOST_CHECK(mesh_triangles[30] == point_ti<>( 4, 14, 19));
    // BOOST_CHECK(mesh_triangles[31] == point_ti<>(14, 13, 19));
    // BOOST_CHECK(mesh_triangles[32] == point_ti<>( 8,  4, 20));
    // BOOST_CHECK(mesh_triangles[33] == point_ti<>( 4, 19, 20));
    // BOOST_CHECK(mesh_triangles[34] == point_ti<>(19,  8, 20));
    // BOOST_CHECK(mesh_triangles[35] == point_ti<>(17,  7, 21));
    // BOOST_CHECK(mesh_triangles[36] == point_ti<>(18, 17, 21));
    // BOOST_CHECK(mesh_triangles[37] == point_ti<>( 4,  5, 22));
    // BOOST_CHECK(mesh_triangles[38] == point_ti<>( 7,  4, 22));
    // BOOST_CHECK(mesh_triangles[39] == point_ti<>(21,  7, 22));
    // BOOST_CHECK(mesh_triangles[40] == point_ti<>( 5, 18, 23));
    // BOOST_CHECK(mesh_triangles[41] == point_ti<>(21,  5, 23));
    // BOOST_CHECK(mesh_triangles[42] == point_ti<>(18, 21, 23));
    // BOOST_CHECK(mesh_triangles[43] == point_ti<>( 5, 21, 24));
    // BOOST_CHECK(mesh_triangles[44] == point_ti<>(22,  5, 24));
    // BOOST_CHECK(mesh_triangles[45] == point_ti<>(21, 22, 24));
}

/* Test is inside */
BOOST_AUTO_TEST_CASE( is_inside_cube_test )
{
    BOOST_CHECK( cube_uut.is_inside(point_t( 0.0f,  0.0f,  0.0f), 0.0001f));
    BOOST_CHECK( cube_uut.is_inside(point_t( 0.4f,  0.4f,  0.4f), 0.0001f));
    BOOST_CHECK( cube_uut.is_inside(point_t(-0.4f, -0.4f, -0.4f), 0.0001f));
    BOOST_CHECK(!cube_uut.is_inside(point_t(-0.6f, -0.6f, -0.6f), 0.0001f));
}

BOOST_AUTO_TEST_CASE( is_inside_cube_big_tolerance_test )
{
    BOOST_CHECK(!cube_uut.is_inside(point_t(-0.4f, -0.4f, -0.4f), 0.1f));
    BOOST_CHECK( cube_uut.is_inside(point_t(-0.3f, -0.3f, -0.3f), 0.1f));
}

BOOST_AUTO_TEST_CASE( is_inside_octagon_test )
{
    BOOST_CHECK( octagon_uut.is_inside(point_t( 1.5f,  1.5f,  0.0f), 0.0001f));
    BOOST_CHECK( octagon_uut.is_inside(point_t( 1.5f,  1.5f,  1.0f), 0.0001f));
    BOOST_CHECK( octagon_uut.is_inside(point_t( 1.5f,  1.5f, -1.0f), 0.0001f));
    BOOST_CHECK(!octagon_uut.is_inside(point_t( 0.0f,  0.0f,  0.0f), 0.0001f));
    BOOST_CHECK( octagon_uut.is_inside(point_t( 2.0f,  0.0f,  0.0f), 0.0001f));
}

/* Performance tests */
BOOST_AUTO_TEST_CASE( process_hull_performance_test )
{
    const int number_runs   = 100;
    const int number_points = 100000;

    /* Memory to hold points */
    std::vector<point_t> points;
    points.reserve(number_points);

    /* Generate points */
    std::default_random_engine gen;
    std::uniform_real_distribution<float> dist(-100.0f, 100.0f);
    for (int i = 0; i< number_points; ++i)
    {
        points.emplace_back(dist(gen), dist(gen), dist(gen));
    }

    /* Build */
    const auto t0(std::chrono::system_clock::now());
    for (int i = 0; i < number_runs; ++i)
    {
        incremental_convex_hull uut;
        uut.add_points(points);
        uut.process();
        BOOST_REQUIRE(uut.number_of_triangles() > 0);
    }
    const auto t1(std::chrono::system_clock::now());
    std::cout << "Test took: " << (std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count() / number_runs) << " ms per hull" << std::endl;
}

BOOST_AUTO_TEST_SUITE_END()
}; /* namespace test */
}; /* namespace raptor_convex_decomposition */
