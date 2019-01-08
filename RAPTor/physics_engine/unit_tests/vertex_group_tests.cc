#ifdef STAND_ALONE
#define BOOST_TEST_MODULE vertex_group test

/* Common headers */
#include "logging.h"

/* Initialise logger */
const raptor_physics::init_logger init_logger;
#endif /* #ifdef STAND_ALONE */


/* Standard headers */
#include <limits>

/* Boost headers */
#include "boost/noncopyable.hpp"
#include "boost/test/unit_test.hpp"

/* Raytracer headers */
#include "phong_shader.h"

/* Physics headers */
#include "simplex.h"
#include "vertex_group.h"


namespace raptor_physics
{
namespace test
{
/* Test data */
struct vertex_group_fixture : private boost::noncopyable
{
    vertex_group_fixture()
    : mat(new raptor_raytracer::phong_shader(raptor_raytracer::ext_colour_t(255.0, 255.0, 255.0), 1.0)),
      plane_vg(make_plane(mat, point_t(-10.0, 0.0, -10.0), point_t(10.0, 0.0, -10.0), point_t(-10.0, 0.0, 10.0), point_t(10.0, 0.0, 10.0))),
      cube_vg0(make_cube(mat, point_t(-0.5, -0.5, -0.5), point_t(0.5,  0.5, 0.5))),
      cube_vg1(make_cube(mat, point_t(-0.5,  9.0, -0.5), point_t(0.5, 10.0, 0.5)))
      {  };

    ~vertex_group_fixture()
    {
        delete mat;
        delete plane_vg;
        delete cube_vg0;
        delete cube_vg1;
    }

    raptor_raytracer::material *    mat;
    vertex_group *                  plane_vg;
    vertex_group *                  cube_vg0;
    vertex_group *                  cube_vg1;
};

BOOST_FIXTURE_TEST_SUITE( vertex_group_tests, vertex_group_fixture )

const float result_tolerance = 0.0005;


/* Test ctor */
BOOST_AUTO_TEST_CASE( ctor_test )
{
    BOOST_CHECK(plane_vg->get_number_of_vertices() == 4);
    BOOST_CHECK(plane_vg->get_vertex(0) == point_t(-10.0, 0.0, -10.0));
    BOOST_CHECK(plane_vg->get_vertex(1) == point_t(-10.0, 0.0,  10.0));
    BOOST_CHECK(plane_vg->get_vertex(2) == point_t( 10.0, 0.0,  10.0));
    BOOST_CHECK(plane_vg->get_vertex(3) == point_t( 10.0, 0.0, -10.0));

    BOOST_CHECK(cube_vg0->get_number_of_vertices() == 8);
    BOOST_CHECK(cube_vg0->get_vertex(0) == point_t(-0.5, -0.5, -0.5));
    BOOST_CHECK(cube_vg0->get_vertex(1) == point_t( 0.5, -0.5, -0.5));
    BOOST_CHECK(cube_vg0->get_vertex(2) == point_t( 0.5,  0.5, -0.5));
    BOOST_CHECK(cube_vg0->get_vertex(3) == point_t(-0.5,  0.5, -0.5));
    BOOST_CHECK(cube_vg0->get_vertex(4) == point_t(-0.5, -0.5,  0.5));
    BOOST_CHECK(cube_vg0->get_vertex(5) == point_t( 0.5, -0.5,  0.5));
    BOOST_CHECK(cube_vg0->get_vertex(6) == point_t( 0.5,  0.5,  0.5));
    BOOST_CHECK(cube_vg0->get_vertex(7) == point_t(-0.5,  0.5,  0.5));

    BOOST_CHECK(cube_vg1->get_number_of_vertices() == 8);
    BOOST_CHECK(cube_vg1->get_vertex(0) == point_t(-0.5,  9.0, -0.5));
    BOOST_CHECK(cube_vg1->get_vertex(1) == point_t( 0.5,  9.0, -0.5));
    BOOST_CHECK(cube_vg1->get_vertex(2) == point_t( 0.5, 10.0, -0.5));
    BOOST_CHECK(cube_vg1->get_vertex(3) == point_t(-0.5, 10.0, -0.5));
    BOOST_CHECK(cube_vg1->get_vertex(4) == point_t(-0.5,  9.0,  0.5));
    BOOST_CHECK(cube_vg1->get_vertex(5) == point_t( 0.5,  9.0,  0.5));
    BOOST_CHECK(cube_vg1->get_vertex(6) == point_t( 0.5, 10.0,  0.5));
    BOOST_CHECK(cube_vg1->get_vertex(7) == point_t(-0.5, 10.0,  0.5));
}


/* Test get bounds */
BOOST_AUTO_TEST_CASE( get_bounds_test )
{
    point_t hi;
    point_t lo;
    plane_vg->get_bounds(&hi, &lo);
    BOOST_CHECK(hi == point_t( 10.0, 0.0,  10.0));
    BOOST_CHECK(lo == point_t(-10.0, 0.0, -10.0));

    cube_vg0->get_bounds(&hi, &lo);
    BOOST_CHECK(hi == point_t( 0.5,  0.5,  0.5));
    BOOST_CHECK(lo == point_t(-0.5, -0.5, -0.5));
      
    cube_vg1->get_bounds(&hi, &lo);
    BOOST_CHECK(hi == point_t( 0.5, 10.0,  0.5));
    BOOST_CHECK(lo == point_t(-0.5,  9.0, -0.5));
}


/* Test build inertia tensor */
BOOST_AUTO_TEST_CASE( build_inertia_tensor_test )
{
    std::unique_ptr<inertia_tensor> i0(plane_vg->build_inertia_tensor(std::numeric_limits<float>::infinity()));
    BOOST_CHECK(i0->mass() == std::numeric_limits<float>::infinity());
    BOOST_CHECK(i0->center_of_mass() == point_t(0.0, 0.0, 0.0));

    std::unique_ptr<inertia_tensor> i1(cube_vg0->build_inertia_tensor(2.0));
    BOOST_CHECK(i1->mass() == 2.0);
    BOOST_CHECK(i1->center_of_mass() == point_t(0.0, 0.0, 0.0));

    std::unique_ptr<inertia_tensor> i2(cube_vg1->build_inertia_tensor(4.0));
    BOOST_CHECK(i2->mass() == 4.0);
    BOOST_CHECK(i2->center_of_mass() == point_t(0.0, 9.5, 0.0));
}


/* Find support vertex tests */
BOOST_AUTO_TEST_CASE( find_support_vertex_test )
{
    /* This just forwards to physic common which is tested else where */
    BOOST_CHECK(cube_vg0->find_support_vertex(point_t(1.0f, 2.0f, -1.0f)) == 2);
}

BOOST_AUTO_TEST_CASE( find_support_vertex_with_optional_displacement_test )
{
    BOOST_CHECK(cube_vg0->find_support_vertex(point_t(1.0f, 2.0f, -1.0f), point_t(-1.0f, -2.0f,  1.0f)) == 2);
    BOOST_CHECK(cube_vg0->find_support_vertex(point_t(1.0f, 2.0f, -1.0f), point_t( 1.0f,  2.0f, -1.0f)) == static_cast<int>(0x80000002));
    BOOST_CHECK(cube_vg0->find_support_vertex(point_t(1.0f, 2.0f, -1.0f), point_t(-1.0f,  0.0f,  2.0f)) == 2);
    BOOST_CHECK(cube_vg0->find_support_vertex(point_t(1.0f, 2.0f, -1.0f), point_t( 0.0f,  2.0f, -1.0f)) == static_cast<int>(0x80000002));
}


BOOST_AUTO_TEST_CASE( find_support_penalised_vertex_test )
{
    /* This just forwards to physic common which is tested else where */
    float v;
    BOOST_CHECK(cube_vg0->find_support_vertex(point_t(0.0f, 1.0f, 0.0f), point_t(0.0f, 0.0f, 0.0f), point_t(10.0f, 0.0f, 0.0f), point_t(1.0f, 0.0f, 0.0f), &v) == 0);
    BOOST_CHECK(fabs(v - 0.707107f) < result_tolerance);
}

/* Find intersection time tests */
BOOST_AUTO_TEST_CASE( find_intersection_time_test )
{
    BOOST_CHECK(cube_vg0->find_intersection_time(*cube_vg1, point_t(-1.0, 0.0, 0.0), point_t(0.0, 15.6, 0.0), point_t(0.0, -12.2, 0.0), point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0, 1.0, 1.0) == 0.0);
}

/* Find polygon tests */
BOOST_AUTO_TEST_CASE( find_polygon_not_there_test )
{
    int contain0[] = { 5 };
    int contain1[] = { 12 };
    BOOST_CHECK(plane_vg->find_polygon(&contain0[0], &contain0[1], point_t(1.0f, 0.0f, 0.0f)) == nullptr);
    BOOST_CHECK(cube_vg0->find_polygon(&contain1[0], &contain1[1], point_t(0.0f, 1.0f, 0.0f)) == nullptr);
}

BOOST_AUTO_TEST_CASE( find_polygon_face_match_test )
{
    int contain0[] = { 0, 1, 3 };
    const auto *const p0 = plane_vg->find_polygon(&contain0[0], &contain0[3], point_t(1.0f, 0.0f, 0.0f));
    BOOST_REQUIRE(p0 != nullptr);
    BOOST_REQUIRE(p0->number_of_vertices()  == 4);
    BOOST_CHECK(p0->vertex(0)   == point_t(-10.0f, 0.0f, -10.0f));
    BOOST_CHECK(p0->vertex(1)   == point_t(-10.0f, 0.0f,  10.0f));
    BOOST_CHECK(p0->vertex(2)   == point_t( 10.0f, 0.0f,  10.0f));
    BOOST_CHECK(p0->vertex(3)   == point_t( 10.0f, 0.0f, -10.0f));

    int contain1[] = { 0, 1, 2 };
    const auto *const p1 = cube_vg0->find_polygon(&contain1[0], &contain1[3], point_t(0.0f, 1.0f, 0.0f));
    BOOST_REQUIRE(p1 != nullptr);
    BOOST_REQUIRE(p1->number_of_vertices()  == 4);
    BOOST_CHECK(p1->vertex(0)   == point_t(-0.5f,  0.5f, -0.5f));
    BOOST_CHECK(p1->vertex(1)   == point_t( 0.5f,  0.5f, -0.5f));
    BOOST_CHECK(p1->vertex(2)   == point_t( 0.5f, -0.5f, -0.5f));
    BOOST_CHECK(p1->vertex(3)   == point_t(-0.5f, -0.5f, -0.5f));
}

BOOST_AUTO_TEST_CASE( find_polygon_edge_match_test )
{
    int contain0[] = { 0, 1 };
    const auto *const p0 = cube_vg0->find_polygon(&contain0[0], &contain0[1], point_t(0.0f, 0.0f, -1.0f));
    BOOST_REQUIRE(p0 != nullptr);
    BOOST_REQUIRE(p0->number_of_vertices()  == 4);
    BOOST_CHECK(p0->vertex(0)   == point_t(-0.5f,  0.5f, -0.5f));
    BOOST_CHECK(p0->vertex(1)   == point_t( 0.5f,  0.5f, -0.5f));
    BOOST_CHECK(p0->vertex(2)   == point_t( 0.5f, -0.5f, -0.5f));
    BOOST_CHECK(p0->vertex(3)   == point_t(-0.5f, -0.5f, -0.5f));

    int contain1[] = { 0, 1 };
    const auto *const p1 = cube_vg0->find_polygon(&contain1[0], &contain1[1], point_t(0.0f, -1.0f, 0.0f));
    BOOST_REQUIRE(p1 != nullptr);
    BOOST_REQUIRE(p1->number_of_vertices()  == 4);
    BOOST_CHECK(p1->vertex(0)   == point_t(-0.5f, -0.5f, -0.5f));
    BOOST_CHECK(p1->vertex(1)   == point_t( 0.5f, -0.5f, -0.5f));
    BOOST_CHECK(p1->vertex(2)   == point_t( 0.5f, -0.5f,  0.5f));
    BOOST_CHECK(p1->vertex(3)   == point_t(-0.5f, -0.5f,  0.5f));

    /* Perpendicular is better than back face */
    int contain2[] = { 0, 1 };
    const auto *const p2 = cube_vg0->find_polygon(&contain2[0], &contain2[1], point_t(0.0f, 1.0f, 0.0f));
    BOOST_REQUIRE(p2 != nullptr);
    BOOST_REQUIRE(p2->number_of_vertices()  == 4);
    BOOST_CHECK(p2->vertex(0)   == point_t(-0.5f,  0.5f, -0.5f));
    BOOST_CHECK(p2->vertex(1)   == point_t( 0.5f,  0.5f, -0.5f));
    BOOST_CHECK(p2->vertex(2)   == point_t( 0.5f, -0.5f, -0.5f));
    BOOST_CHECK(p2->vertex(3)   == point_t(-0.5f, -0.5f, -0.5f));
}

BOOST_AUTO_TEST_CASE( find_polygon_vertex_match_test )
{
    int contain0[] = { 0 };
    const auto *const p0 = cube_vg0->find_polygon(&contain0[0], &contain0[0], point_t(0.0f, 0.0f, -1.0f));
    BOOST_REQUIRE(p0 != nullptr);
    BOOST_REQUIRE(p0->number_of_vertices()  == 4);
    BOOST_CHECK(p0->vertex(0)   == point_t(-0.5f,  0.5f, -0.5f));
    BOOST_CHECK(p0->vertex(1)   == point_t( 0.5f,  0.5f, -0.5f));
    BOOST_CHECK(p0->vertex(2)   == point_t( 0.5f, -0.5f, -0.5f));
    BOOST_CHECK(p0->vertex(3)   == point_t(-0.5f, -0.5f, -0.5f));

    int contain1[] = { 0 };
    const auto *const p1 = cube_vg0->find_polygon(&contain1[0], &contain1[0], point_t(0.0f, -1.0f, 0.0f));
    BOOST_REQUIRE(p1 != nullptr);
    BOOST_REQUIRE(p1->number_of_vertices()  == 4);
    BOOST_CHECK(p1->vertex(0)   == point_t(-0.5f, -0.5f, -0.5f));
    BOOST_CHECK(p1->vertex(1)   == point_t( 0.5f, -0.5f, -0.5f));
    BOOST_CHECK(p1->vertex(2)   == point_t( 0.5f, -0.5f,  0.5f));
    BOOST_CHECK(p1->vertex(3)   == point_t(-0.5f, -0.5f,  0.5f));

    int contain2[] = { 0 };
    const auto *const p2 = cube_vg0->find_polygon(&contain2[0], &contain2[0], point_t(-1.0f, 0.0f, 0.0f));
    BOOST_REQUIRE(p2 != nullptr);
    BOOST_REQUIRE(p2->number_of_vertices()  == 4);
    BOOST_CHECK(p2->vertex(0)   == point_t(-0.5f, -0.5f, -0.5f));
    BOOST_CHECK(p2->vertex(1)   == point_t(-0.5f, -0.5f,  0.5f));
    BOOST_CHECK(p2->vertex(2)   == point_t(-0.5f,  0.5f,  0.5f));
    BOOST_CHECK(p2->vertex(3)   == point_t(-0.5f,  0.5f, -0.5f));
}

/* Test build triangles */
BOOST_AUTO_TEST_CASE( static_build_triangles_test )
{
    /* Test plane */
    raptor_raytracer::primitive_store p0;
    plane_vg->triangles(&p0, quaternion_t(1.0f, 0.0f, 0.0f, 0.0f), point_t(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(p0.size() == 2);

    BOOST_CHECK(p0.primitive(0)->get_vertex_a() == point_t( 10.0f, 0.0f, -10.0f));
    BOOST_CHECK(p0.primitive(0)->get_vertex_b() == point_t(-10.0f, 0.0f, -10.0f));
    BOOST_CHECK(p0.primitive(0)->get_vertex_c() == point_t(-10.0f, 0.0f,  10.0f));

    BOOST_CHECK(p0.primitive(1)->get_vertex_a() == point_t( 10.0f, 0.0f, -10.0f));
    BOOST_CHECK(p0.primitive(1)->get_vertex_b() == point_t(-10.0f, 0.0f,  10.0f));
    BOOST_CHECK(p0.primitive(1)->get_vertex_c() == point_t( 10.0f, 0.0f,  10.0f));

    /* Test cube */
    raptor_raytracer::primitive_store p1;
    cube_vg0->triangles(&p1, quaternion_t(1.0f, 0.0f, 0.0f, 0.0f), point_t(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(p1.size() == 12);

    BOOST_CHECK(p1.primitive( 0)->get_vertex_a()    == point_t(-0.5f, -0.5f, -0.5f));
    BOOST_CHECK(p1.primitive( 0)->get_vertex_b()    == point_t(-0.5f,  0.5f, -0.5f));
    BOOST_CHECK(p1.primitive( 0)->get_vertex_c()    == point_t( 0.5f,  0.5f, -0.5f));

    BOOST_CHECK(p1.primitive( 1)->get_vertex_a()    == point_t(-0.5f, -0.5f, -0.5f));
    BOOST_CHECK(p1.primitive( 1)->get_vertex_b()    == point_t( 0.5f,  0.5f, -0.5f));
    BOOST_CHECK(p1.primitive( 1)->get_vertex_c()    == point_t( 0.5f, -0.5f, -0.5f));

    BOOST_CHECK(p1.primitive( 2)->get_vertex_a()    == point_t(-0.5f,  0.5f,  0.5f));
    BOOST_CHECK(p1.primitive( 2)->get_vertex_b()    == point_t(-0.5f, -0.5f,  0.5f));
    BOOST_CHECK(p1.primitive( 2)->get_vertex_c()    == point_t( 0.5f, -0.5f,  0.5f));

    BOOST_CHECK(p1.primitive( 3)->get_vertex_a()    == point_t(-0.5f,  0.5f,  0.5f));
    BOOST_CHECK(p1.primitive( 3)->get_vertex_b()    == point_t( 0.5f, -0.5f,  0.5f));
    BOOST_CHECK(p1.primitive( 3)->get_vertex_c()    == point_t( 0.5f,  0.5f,  0.5f));

    BOOST_CHECK(p1.primitive( 4)->get_vertex_a()    == point_t(-0.5f,  0.5f, -0.5f));
    BOOST_CHECK(p1.primitive( 4)->get_vertex_b()    == point_t(-0.5f, -0.5f, -0.5f));
    BOOST_CHECK(p1.primitive( 4)->get_vertex_c()    == point_t(-0.5f, -0.5f,  0.5f));

    BOOST_CHECK(p1.primitive( 5)->get_vertex_a()    == point_t(-0.5f,  0.5f, -0.5f));
    BOOST_CHECK(p1.primitive( 5)->get_vertex_b()    == point_t(-0.5f, -0.5f,  0.5f));
    BOOST_CHECK(p1.primitive( 5)->get_vertex_c()    == point_t(-0.5f,  0.5f,  0.5f));

    BOOST_CHECK(p1.primitive( 6)->get_vertex_a()    == point_t( 0.5f, -0.5f,  0.5f));
    BOOST_CHECK(p1.primitive( 6)->get_vertex_b()    == point_t( 0.5f, -0.5f, -0.5f));
    BOOST_CHECK(p1.primitive( 6)->get_vertex_c()    == point_t( 0.5f,  0.5f, -0.5f));

    BOOST_CHECK(p1.primitive( 7)->get_vertex_a()    == point_t( 0.5f, -0.5f,  0.5f));
    BOOST_CHECK(p1.primitive( 7)->get_vertex_b()    == point_t( 0.5f,  0.5f, -0.5f));
    BOOST_CHECK(p1.primitive( 7)->get_vertex_c()    == point_t( 0.5f,  0.5f,  0.5f));

    BOOST_CHECK(p1.primitive( 8)->get_vertex_a()    == point_t( 0.5f,  0.5f,  0.5f));
    BOOST_CHECK(p1.primitive( 8)->get_vertex_b()    == point_t( 0.5f,  0.5f, -0.5f));
    BOOST_CHECK(p1.primitive( 8)->get_vertex_c()    == point_t(-0.5f,  0.5f, -0.5f));

    BOOST_CHECK(p1.primitive( 9)->get_vertex_a()    == point_t( 0.5f,  0.5f,  0.5f));
    BOOST_CHECK(p1.primitive( 9)->get_vertex_b()    == point_t(-0.5f,  0.5f, -0.5f));
    BOOST_CHECK(p1.primitive( 9)->get_vertex_c()    == point_t(-0.5f,  0.5f,  0.5f));

    BOOST_CHECK(p1.primitive(10)->get_vertex_a()    == point_t(-0.5f, -0.5f,  0.5f));
    BOOST_CHECK(p1.primitive(10)->get_vertex_b()    == point_t(-0.5f, -0.5f, -0.5f));
    BOOST_CHECK(p1.primitive(10)->get_vertex_c()    == point_t( 0.5f, -0.5f, -0.5f));

    BOOST_CHECK(p1.primitive(11)->get_vertex_a()    == point_t(-0.5f, -0.5f,  0.5f));
    BOOST_CHECK(p1.primitive(11)->get_vertex_b()    == point_t( 0.5f, -0.5f, -0.5f));
    BOOST_CHECK(p1.primitive(11)->get_vertex_c()    == point_t( 0.5f, -0.5f,  0.5f));
}

BOOST_AUTO_TEST_CASE( translated_build_triangles_test )
{
    /* Test plane */
    raptor_raytracer::primitive_store p0;
    plane_vg->triangles(&p0, quaternion_t(1.0f, 0.0f, 0.0f, 0.0f), point_t(-3.0f, 1.0f, -2.0f));
    BOOST_CHECK(p0.size() == 2);

    BOOST_CHECK(p0.primitive(0)->get_vertex_a() == point_t(  7.0f, 1.0f, -12.0f));
    BOOST_CHECK(p0.primitive(0)->get_vertex_b() == point_t(-13.0f, 1.0f, -12.0f));
    BOOST_CHECK(p0.primitive(0)->get_vertex_c() == point_t(-13.0f, 1.0f,   8.0f));

    BOOST_CHECK(p0.primitive(1)->get_vertex_a() == point_t(  7.0f, 1.0f, -12.0f));
    BOOST_CHECK(p0.primitive(1)->get_vertex_b() == point_t(-13.0f, 1.0f,   8.0f));
    BOOST_CHECK(p0.primitive(1)->get_vertex_c() == point_t(  7.0f, 1.0f,   8.0f));

    /* Test cube */
    raptor_raytracer::primitive_store p1;
    cube_vg0->triangles(&p1, quaternion_t(1.0f, 0.0f, 0.0f, 0.0f), point_t(-2.3f, -1.2f, 1.5f));
    BOOST_CHECK(p1.size() == 12);

    BOOST_CHECK(fabs(magnitude(p1.primitive( 0)->get_vertex_a() - point_t(-2.8f, -1.7f,  1.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive( 0)->get_vertex_b() - point_t(-2.8f, -0.7f,  1.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive( 0)->get_vertex_c() - point_t(-1.8f, -0.7f,  1.0f))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(p1.primitive( 1)->get_vertex_a() - point_t(-2.8f, -1.7f,  1.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive( 1)->get_vertex_b() - point_t(-1.8f, -0.7f,  1.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive( 1)->get_vertex_c() - point_t(-1.8f, -1.7f,  1.0f))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(p1.primitive( 2)->get_vertex_a() - point_t(-2.8f, -0.7f,  2.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive( 2)->get_vertex_b() - point_t(-2.8f, -1.7f,  2.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive( 2)->get_vertex_c() - point_t(-1.8f, -1.7f,  2.0f))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(p1.primitive( 3)->get_vertex_a() - point_t(-2.8f, -0.7f,  2.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive( 3)->get_vertex_b() - point_t(-1.8f, -1.7f,  2.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive( 3)->get_vertex_c() - point_t(-1.8f, -0.7f,  2.0f))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(p1.primitive( 4)->get_vertex_a() - point_t(-2.8f, -0.7f,  1.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive( 4)->get_vertex_b() - point_t(-2.8f, -1.7f,  1.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive( 4)->get_vertex_c() - point_t(-2.8f, -1.7f,  2.0f))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(p1.primitive( 5)->get_vertex_a() - point_t(-2.8f, -0.7f,  1.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive( 5)->get_vertex_b() - point_t(-2.8f, -1.7f,  2.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive( 5)->get_vertex_c() - point_t(-2.8f, -0.7f,  2.0f))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(p1.primitive( 6)->get_vertex_a() - point_t(-1.8f, -1.7f,  2.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive( 6)->get_vertex_b() - point_t(-1.8f, -1.7f,  1.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive( 6)->get_vertex_c() - point_t(-1.8f, -0.7f,  1.0f))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(p1.primitive( 7)->get_vertex_a() - point_t(-1.8f, -1.7f,  2.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive( 7)->get_vertex_b() - point_t(-1.8f, -0.7f,  1.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive( 7)->get_vertex_c() - point_t(-1.8f, -0.7f,  2.0f))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(p1.primitive( 8)->get_vertex_a() - point_t(-1.8f, -0.7f,  2.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive( 8)->get_vertex_b() - point_t(-1.8f, -0.7f,  1.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive( 8)->get_vertex_c() - point_t(-2.8f, -0.7f,  1.0f))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(p1.primitive( 9)->get_vertex_a() - point_t(-1.8f, -0.7f,  2.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive( 9)->get_vertex_b() - point_t(-2.8f, -0.7f,  1.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive( 9)->get_vertex_c() - point_t(-2.8f, -0.7f,  2.0f))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(p1.primitive(10)->get_vertex_a() - point_t(-2.8f, -1.7f,  2.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive(10)->get_vertex_b() - point_t(-2.8f, -1.7f,  1.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive(10)->get_vertex_c() - point_t(-1.8f, -1.7f,  1.0f))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(p1.primitive(11)->get_vertex_a() - point_t(-2.8f, -1.7f,  2.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive(11)->get_vertex_b() - point_t(-1.8f, -1.7f,  1.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive(11)->get_vertex_c() - point_t(-1.8f, -1.7f,  2.0f))) < result_tolerance);
}

BOOST_AUTO_TEST_CASE( translated_rotated_build_triangles_test )
{
    /* Test plane */
    raptor_raytracer::primitive_store p0;
    plane_vg->triangles(&p0, quaternion_t(0.931655f, 0.0186331f, -0.316763f, -0.177014f), point_t(2.04f, -9.1f, -1.44f));
    BOOST_CHECK(p0.size() == 2);

    BOOST_CHECK(fabs(magnitude(p0.primitive(0)->get_vertex_a() - point_t( 15.3748f,   -13.2906f,   -3.58997f ))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p0.primitive(0)->get_vertex_b() - point_t(  0.641699f,  -6.45787f, -15.2626f  ))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p0.primitive(0)->get_vertex_c() - point_t(-11.2948f,    -4.9094f,    0.709969f))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(p0.primitive(1)->get_vertex_a() - point_t( 15.3748f,   -13.2906f,   -3.58997f ))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p0.primitive(1)->get_vertex_b() - point_t(-11.2948f,    -4.9094f,    0.709969f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p0.primitive(1)->get_vertex_c() - point_t(  3.4383f,   -11.7421f,   12.3826f  ))) < result_tolerance);

    /* Test cube */
    raptor_raytracer::primitive_store p1;
    cube_vg0->triangles(&p1, quaternion_t(0.0647662f, 0.450449f, -0.867219f, 0.20207f), point_t(-1.71f, -6.51f, 1.43f));
    BOOST_CHECK(p1.size() == 12);

    BOOST_CHECK(fabs(magnitude(p1.primitive( 0)->get_vertex_a() - point_t(-1.04823f,  -6.1843f,  1.88385f ))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive( 0)->get_vertex_b() - point_t(-1.85568f,  -5.67177f, 1.59172f ))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive( 0)->get_vertex_c() - point_t(-2.44148f,  -6.42687f, 1.8861f  ))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(p1.primitive( 1)->get_vertex_a() - point_t(-1.04823f,  -6.1843f,  1.88385f ))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive( 1)->get_vertex_b() - point_t(-2.44148f,  -6.42687f, 1.8861f  ))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive( 1)->get_vertex_c() - point_t(-1.63403f,  -6.9394f,  2.17823f ))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(p1.primitive( 2)->get_vertex_a() - point_t(-1.78597f,  -6.0806f,  0.681773f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive( 2)->get_vertex_b() - point_t(-0.978518f, -6.59313f, 0.973903f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive( 2)->get_vertex_c() - point_t(-1.56432f,  -7.34823f, 1.26828f ))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(p1.primitive( 3)->get_vertex_a() - point_t(-1.78597f,  -6.0806f,  0.681773f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive( 3)->get_vertex_b() - point_t(-1.56432f,  -7.34823f, 1.26828f ))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive( 3)->get_vertex_c() - point_t(-2.37177f,  -6.8357f,  0.976151f))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(p1.primitive( 4)->get_vertex_a() - point_t(-1.85568f,  -5.67177f, 1.59172f ))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive( 4)->get_vertex_b() - point_t(-1.04823f,  -6.1843f,  1.88385f ))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive( 4)->get_vertex_c() - point_t(-0.978518f, -6.59313f, 0.973903f))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(p1.primitive( 5)->get_vertex_a() - point_t(-1.85568f,  -5.67177f, 1.59172f ))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive( 5)->get_vertex_b() - point_t(-0.978518f, -6.59313f, 0.973903f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive( 5)->get_vertex_c() - point_t(-1.78597f,  -6.0806f,  0.681773f))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(p1.primitive( 6)->get_vertex_a() - point_t(-1.56432f,  -7.34823f, 1.26828f ))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive( 6)->get_vertex_b() - point_t(-1.63403f,  -6.9394f,  2.17823f ))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive( 6)->get_vertex_c() - point_t(-2.44148f,  -6.42687f, 1.8861f  ))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(p1.primitive( 7)->get_vertex_a() - point_t(-1.56432f,  -7.34823f, 1.26828f ))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive( 7)->get_vertex_b() - point_t(-2.44148f,  -6.42687f, 1.8861f  ))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive( 7)->get_vertex_c() - point_t(-2.37177f,  -6.8357f,  0.976151f))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(p1.primitive( 8)->get_vertex_a() - point_t(-2.37177f,  -6.8357f,  0.976151f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive( 8)->get_vertex_b() - point_t(-2.44148f,  -6.42687f, 1.8861f  ))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive( 8)->get_vertex_c() - point_t(-1.85568f,  -5.67177f, 1.59172f ))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(p1.primitive( 9)->get_vertex_a() - point_t(-2.37177f,  -6.8357f,  0.976151f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive( 9)->get_vertex_b() - point_t(-1.85568f,  -5.67177f, 1.59172f ))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive( 9)->get_vertex_c() - point_t(-1.78597f,  -6.0806f,  0.681773f))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(p1.primitive(10)->get_vertex_a() - point_t(-0.978518f, -6.59313f, 0.973903f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive(10)->get_vertex_b() - point_t(-1.04823f,  -6.1843f,  1.88385f ))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive(10)->get_vertex_c() - point_t(-1.63403f,  -6.9394f,  2.17823f ))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(p1.primitive(11)->get_vertex_a() - point_t(-0.978518f, -6.59313f, 0.973903f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive(11)->get_vertex_b() - point_t(-1.63403f,  -6.9394f,  2.17823f ))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive(11)->get_vertex_c() - point_t(-1.56432f,  -7.34823f, 1.26828f ))) < result_tolerance);
}

BOOST_AUTO_TEST_SUITE_END()
}; /* namespace test */
}; /* namespace raptor_physics */
