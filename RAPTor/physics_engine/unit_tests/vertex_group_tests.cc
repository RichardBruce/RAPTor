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
    : mat(new raptor_raytracer::phong_shader(raptor_raytracer::ext_colour_t(255, 255, 255), 1.0)),
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
    BOOST_CHECK(cube_vg0->find_support_vertex(point_t(1.0, 2.0, -1.0)) == 2);
}


BOOST_AUTO_TEST_CASE( find_support_vertex_with_optional_displacement_test )
{
    BOOST_CHECK(cube_vg0->find_support_vertex(point_t(1.0, 2.0, -1.0), point_t(-1.0, -2.0,  1.0)) == 2);
    BOOST_CHECK(cube_vg0->find_support_vertex(point_t(1.0, 2.0, -1.0), point_t( 1.0,  2.0, -1.0)) == static_cast<int>(0x80000002));
    BOOST_CHECK(cube_vg0->find_support_vertex(point_t(1.0, 2.0, -1.0), point_t(-1.0,  0.0,  2.0)) == 2);
    BOOST_CHECK(cube_vg0->find_support_vertex(point_t(1.0, 2.0, -1.0), point_t( 0.0,  2.0, -1.0)) == static_cast<int>(0x80000002));
}


BOOST_AUTO_TEST_CASE( find_support_penalised_vertex_test )
{
    /* This just forwards to physic common which is tested else where */
    float v;
    BOOST_CHECK(cube_vg0->find_support_vertex(point_t(0.0, 1.0, 0.0), point_t(0.0, 0.0, 0.0), point_t(10.0, 0.0, 0.0), point_t(1.0, 0.0, 0.0), &v) == 0);
    BOOST_CHECK(fabs(v - 11.2071) < result_tolerance);
}


/* Find intersection time tests */
BOOST_AUTO_TEST_CASE( find_intersection_time_test )
{
    BOOST_CHECK(cube_vg0->find_intersection_time(*cube_vg1, point_t(-1.0, 0.0, 0.0), point_t(0.0, 15.6, 0.0), point_t(0.0, -12.2, 0.0), point_t(0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0), 1.0, 1.0, 1.0) == 0.0);
}


/* Test build triangles */
BOOST_AUTO_TEST_CASE( static_build_triangles_test )
{
    /* Test plane */
    raptor_raytracer::primitive_store p0;
    plane_vg->triangles(&p0, quaternion_t(1.0, 0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(p0.size() == 2);

    BOOST_CHECK(p0.primitive(0)->get_vertex_a() == point_t(-10.0, 0.0, -10.0));
    BOOST_CHECK(p0.primitive(0)->get_vertex_b() == point_t( 10.0, 0.0,  10.0));
    BOOST_CHECK(p0.primitive(0)->get_vertex_c() == point_t(-10.0, 0.0,  10.0));

    BOOST_CHECK(p0.primitive(1)->get_vertex_a() == point_t(-10.0, 0.0, -10.0));
    BOOST_CHECK(p0.primitive(1)->get_vertex_b() == point_t( 10.0, 0.0, -10.0));
    BOOST_CHECK(p0.primitive(1)->get_vertex_c() == point_t( 10.0, 0.0,  10.0));

    /* Test cube */
    raptor_raytracer::primitive_store p1;
    cube_vg0->triangles(&p1, quaternion_t(1.0, 0.0, 0.0, 0.0), point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(p1.size() == 12);

    BOOST_CHECK(p1.primitive(0)->get_vertex_a()    == point_t(-0.5, -0.5, -0.5));
    BOOST_CHECK(p1.primitive(0)->get_vertex_b()    == point_t( 0.5,  0.5, -0.5));
    BOOST_CHECK(p1.primitive(0)->get_vertex_c()    == point_t( 0.5, -0.5, -0.5));

    BOOST_CHECK(p1.primitive(1)->get_vertex_a()    == point_t(-0.5, -0.5, -0.5));
    BOOST_CHECK(p1.primitive(1)->get_vertex_b()    == point_t(-0.5,  0.5, -0.5));
    BOOST_CHECK(p1.primitive(1)->get_vertex_c()    == point_t( 0.5,  0.5, -0.5));

    BOOST_CHECK(p1.primitive(2)->get_vertex_a()    == point_t(-0.5, -0.5,  0.5));
    BOOST_CHECK(p1.primitive(2)->get_vertex_b()    == point_t( 0.5, -0.5,  0.5));
    BOOST_CHECK(p1.primitive(2)->get_vertex_c()    == point_t( 0.5,  0.5,  0.5));

    BOOST_CHECK(p1.primitive(3)->get_vertex_a()    == point_t(-0.5, -0.5,  0.5));
    BOOST_CHECK(p1.primitive(3)->get_vertex_b()    == point_t( 0.5,  0.5,  0.5));
    BOOST_CHECK(p1.primitive(3)->get_vertex_c()    == point_t(-0.5,  0.5,  0.5));

    BOOST_CHECK(p1.primitive(4)->get_vertex_a()    == point_t(-0.5, -0.5,  0.5));
    BOOST_CHECK(p1.primitive(4)->get_vertex_b()    == point_t(-0.5,  0.5,  0.5));
    BOOST_CHECK(p1.primitive(4)->get_vertex_c()    == point_t(-0.5, -0.5, -0.5));

    BOOST_CHECK(p1.primitive(5)->get_vertex_a()    == point_t(-0.5,  0.5,  0.5));
    BOOST_CHECK(p1.primitive(5)->get_vertex_b()    == point_t(-0.5,  0.5, -0.5));
    BOOST_CHECK(p1.primitive(5)->get_vertex_c()    == point_t(-0.5, -0.5, -0.5));

    BOOST_CHECK(p1.primitive(6)->get_vertex_a()    == point_t( 0.5, -0.5, -0.5));
    BOOST_CHECK(p1.primitive(6)->get_vertex_b()    == point_t( 0.5,  0.5, -0.5));
    BOOST_CHECK(p1.primitive(6)->get_vertex_c()    == point_t( 0.5,  0.5,  0.5));

    BOOST_CHECK(p1.primitive(7)->get_vertex_a()    == point_t( 0.5, -0.5, -0.5));
    BOOST_CHECK(p1.primitive(7)->get_vertex_b()    == point_t( 0.5,  0.5,  0.5));
    BOOST_CHECK(p1.primitive(7)->get_vertex_c()    == point_t( 0.5, -0.5,  0.5));

    BOOST_CHECK(p1.primitive(8)->get_vertex_a()    == point_t(-0.5,  0.5, -0.5));
    BOOST_CHECK(p1.primitive(8)->get_vertex_b()    == point_t(-0.5,  0.5,  0.5));
    BOOST_CHECK(p1.primitive(8)->get_vertex_c()    == point_t( 0.5,  0.5,  0.5));

    BOOST_CHECK(p1.primitive(9)->get_vertex_a()    == point_t(-0.5,  0.5, -0.5));
    BOOST_CHECK(p1.primitive(9)->get_vertex_b()    == point_t( 0.5,  0.5,  0.5));
    BOOST_CHECK(p1.primitive(9)->get_vertex_c()    == point_t( 0.5,  0.5, -0.5));

    BOOST_CHECK(p1.primitive(10)->get_vertex_a()   == point_t(-0.5, -0.5, -0.5));
    BOOST_CHECK(p1.primitive(10)->get_vertex_b()   == point_t( 0.5, -0.5, -0.5));
    BOOST_CHECK(p1.primitive(10)->get_vertex_c()   == point_t(-0.5, -0.5,  0.5));

    BOOST_CHECK(p1.primitive(11)->get_vertex_a()   == point_t( 0.5, -0.5, -0.5));
    BOOST_CHECK(p1.primitive(11)->get_vertex_b()   == point_t( 0.5, -0.5,  0.5));
    BOOST_CHECK(p1.primitive(11)->get_vertex_c()   == point_t(-0.5, -0.5,  0.5));
}


BOOST_AUTO_TEST_CASE( translated_build_triangles_test )
{
    /* Test plane */
    raptor_raytracer::primitive_store p0;
    plane_vg->triangles(&p0, quaternion_t(1.0, 0.0, 0.0, 0.0), point_t(-3.0, 1.0, -2.0));
    BOOST_CHECK(p0.size() == 2);

    BOOST_CHECK(p0.primitive(0)->get_vertex_a() == point_t(-13.0, 1.0, -12.0));
    BOOST_CHECK(p0.primitive(0)->get_vertex_b() == point_t(  7.0, 1.0,   8.0));
    BOOST_CHECK(p0.primitive(0)->get_vertex_c() == point_t(-13.0, 1.0,   8.0));

    BOOST_CHECK(p0.primitive(1)->get_vertex_a() == point_t(-13.0, 1.0, -12.0));
    BOOST_CHECK(p0.primitive(1)->get_vertex_b() == point_t(  7.0, 1.0, -12.0));
    BOOST_CHECK(p0.primitive(1)->get_vertex_c() == point_t(  7.0, 1.0,   8.0));

    /* Test cube */
    raptor_raytracer::primitive_store p1;
    cube_vg0->triangles(&p1, quaternion_t(1.0, 0.0, 0.0, 0.0), point_t(-2.3, -1.2, 1.5));
    BOOST_CHECK(p1.size() == 12);

    BOOST_CHECK(fabs(magnitude(p1.primitive(0)->get_vertex_a() - point_t(-2.8, -1.7, 1.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive(0)->get_vertex_b() - point_t(-1.8, -0.7, 1.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive(0)->get_vertex_c() - point_t(-1.8, -1.7, 1.0))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(p1.primitive(1)->get_vertex_a() - point_t(-2.8, -1.7, 1.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive(1)->get_vertex_b() - point_t(-2.8, -0.7, 1.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive(1)->get_vertex_c() - point_t(-1.8, -0.7, 1.0))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(p1.primitive(2)->get_vertex_a() - point_t(-2.8, -1.7, 2.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive(2)->get_vertex_b() - point_t(-1.8, -1.7, 2.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive(2)->get_vertex_c() - point_t(-1.8, -0.7, 2.0))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(p1.primitive(3)->get_vertex_a() - point_t(-2.8, -1.7, 2.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive(3)->get_vertex_b() - point_t(-1.8, -0.7, 2.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive(3)->get_vertex_c() - point_t(-2.8, -0.7, 2.0))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(p1.primitive(4)->get_vertex_a() - point_t(-2.8, -1.7, 2.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive(4)->get_vertex_b() - point_t(-2.8, -0.7, 2.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive(4)->get_vertex_c() - point_t(-2.8, -1.7, 1.0))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(p1.primitive(5)->get_vertex_a() - point_t(-2.8, -0.7, 2.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive(5)->get_vertex_b() - point_t(-2.8, -0.7, 1.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive(5)->get_vertex_c() - point_t(-2.8, -1.7, 1.0))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(p1.primitive(6)->get_vertex_a() - point_t(-1.8, -1.7, 1.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive(6)->get_vertex_b() - point_t(-1.8, -0.7, 1.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive(6)->get_vertex_c() - point_t(-1.8, -0.7, 2.0))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(p1.primitive(7)->get_vertex_a() - point_t(-1.8, -1.7, 1.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive(7)->get_vertex_b() - point_t(-1.8, -0.7, 2.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive(7)->get_vertex_c() - point_t(-1.8, -1.7, 2.0))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(p1.primitive(8)->get_vertex_a() - point_t(-2.8, -0.7, 1.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive(8)->get_vertex_b() - point_t(-2.8, -0.7, 2.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive(8)->get_vertex_c() - point_t(-1.8, -0.7, 2.0))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(p1.primitive(9)->get_vertex_a() - point_t(-2.8, -0.7, 1.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive(9)->get_vertex_b() - point_t(-1.8, -0.7, 2.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive(9)->get_vertex_c() - point_t(-1.8, -0.7, 1.0))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(p1.primitive(10)->get_vertex_a() - point_t(-2.8, -1.7, 1.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive(10)->get_vertex_b() - point_t(-1.8, -1.7, 1.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive(10)->get_vertex_c() - point_t(-2.8, -1.7, 2.0))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(p1.primitive(11)->get_vertex_a() - point_t(-1.8, -1.7, 1.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive(11)->get_vertex_b() - point_t(-1.8, -1.7, 2.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive(11)->get_vertex_c() - point_t(-2.8, -1.7, 2.0))) < result_tolerance);
}


BOOST_AUTO_TEST_CASE( translated_rotated_build_triangles_test )
{
    /* Test plane */
    raptor_raytracer::primitive_store p0;
    plane_vg->triangles(&p0, quaternion_t(0.931655, 0.0186331, -0.316763, -0.177014), point_t(2.04, -9.1, -1.44));
    BOOST_CHECK(p0.size() == 2);

    BOOST_CHECK(fabs(magnitude(p0.primitive(0)->get_vertex_a() - point_t( 0.641683,  -6.45787, -15.2626  ))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p0.primitive(0)->get_vertex_b() - point_t( 3.43832,  -11.7421,   12.3826  ))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p0.primitive(0)->get_vertex_c() - point_t(-11.2948,   -4.90939,   0.709988))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(p0.primitive(1)->get_vertex_a() - point_t( 0.641683,  -6.45787, -15.2626 ))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p0.primitive(1)->get_vertex_b() - point_t(15.3748,   -13.2906,   -3.58999))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p0.primitive(1)->get_vertex_c() - point_t( 3.43832,  -11.7421,   12.3826 ))) < result_tolerance);

    /* Test cube */
    raptor_raytracer::primitive_store p1;
    cube_vg0->triangles(&p1, quaternion_t(0.0647662, 0.450449, -0.867219, 0.20207), point_t(-1.71, -6.51, 1.43));
    BOOST_CHECK(p1.size() == 12);

    BOOST_CHECK(fabs(magnitude(p1.primitive(0)->get_vertex_a() - point_t(-1.04823, -6.1843,    1.88385))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive(0)->get_vertex_b() - point_t(-2.44148, -6.42687,   1.8861 ))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive(0)->get_vertex_c() - point_t(-1.63403, -6.9394,    2.17823))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(p1.primitive(1)->get_vertex_a() - point_t(-1.04823, -6.1843,    1.88385))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive(1)->get_vertex_b() - point_t(-1.85568, -5.67177,   1.59172))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive(1)->get_vertex_c() - point_t(-2.44148, -6.42687,   1.8861 ))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(p1.primitive(2)->get_vertex_a() - point_t(-0.978518, -6.59313,  0.973904))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive(2)->get_vertex_b() - point_t(-1.56432,  -7.34823,  1.26828 ))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive(2)->get_vertex_c() - point_t(-2.37177,  -6.8357,   0.97615 ))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(p1.primitive(3)->get_vertex_a() - point_t(-0.978518, -6.59313,  0.973904))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive(3)->get_vertex_b() - point_t(-2.37177,  -6.8357,   0.97615 ))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive(3)->get_vertex_c() - point_t(-1.78597,  -6.0806,   0.681773))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(p1.primitive(4)->get_vertex_a() - point_t(-0.978518, -6.59313,  0.973904))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive(4)->get_vertex_b() - point_t(-1.78597,  -6.0806,   0.681773))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive(4)->get_vertex_c() - point_t(-1.04823,  -6.1843,   1.88385 ))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(p1.primitive(5)->get_vertex_a() - point_t(-1.78597,  -6.0806,   0.681773))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive(5)->get_vertex_b() - point_t(-1.85568,  -5.67177,  1.59172 ))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive(5)->get_vertex_c() - point_t(-1.04823,  -6.1843,   1.88385 ))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(p1.primitive(6)->get_vertex_a() - point_t(-1.63403,  -6.9394,   2.17823 ))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive(6)->get_vertex_b() - point_t(-2.44148,  -6.42687,  1.8861  ))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive(6)->get_vertex_c() - point_t(-2.37177,  -6.8357,   0.97615 ))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(p1.primitive(7)->get_vertex_a() - point_t(-1.63403,  -6.9394,   2.17823 ))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive(7)->get_vertex_b() - point_t(-2.37177,  -6.8357,   0.97615 ))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive(7)->get_vertex_c() - point_t(-1.56432,  -7.34823,  1.26828 ))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(p1.primitive(8)->get_vertex_a() - point_t(-1.85568,  -5.67177,  1.59172 ))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive(8)->get_vertex_b() - point_t(-1.78597,  -6.0806,   0.681773))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive(8)->get_vertex_c() - point_t(-2.37177,  -6.8357,   0.97615 ))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(p1.primitive(9)->get_vertex_a() - point_t(-1.85568,  -5.67177,  1.59172 ))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive(9)->get_vertex_b() - point_t(-2.37177,  -6.8357,   0.97615 ))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive(9)->get_vertex_c() - point_t(-2.44148,  -6.42687,  1.8861  ))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(p1.primitive(10)->get_vertex_a() - point_t(-1.04823,  -6.1843,  1.88385 ))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive(10)->get_vertex_b() - point_t(-1.63403,  -6.9394,  2.17823 ))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive(10)->get_vertex_c() - point_t(-0.978518, -6.59313, 0.973904))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(p1.primitive(11)->get_vertex_a() - point_t(-1.63403,  -6.9394,  2.17823 ))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive(11)->get_vertex_b() - point_t(-1.56432,  -7.34823, 1.26828 ))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(p1.primitive(11)->get_vertex_c() - point_t(-0.978518, -6.59313, 0.973904))) < result_tolerance);
}

BOOST_AUTO_TEST_SUITE_END()
}; /* namespace test */
}; /* namespace raptor_physics */
