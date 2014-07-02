#ifdef STAND_ALONE
#define BOOST_TEST_MODULE simplex test

/* Common headers */
#include "logging.h"

/* Initialise logger */
const raptor_physics::init_logger init_logger;
#endif /* #ifdef STAND_ALONE */

/* Standard headers */
#include <vector>

/* Boost headers */
#include "boost/test/unit_test.hpp"
#include "boost/noncopyable.hpp"

/* Physics headers */
#include "point_t.h"
#include "matrix_3d.h"
#include "simplex.h"

/* Test headers */
#include "mock_physics_object.h"


using namespace raptor_physics;

/* Test data */
struct simplex_fixture : private boost::noncopyable
{
    simplex_fixture()
    : data0(
        {
            point_t( 1.0, 0.0,  1.0),
            point_t( 1.0, 0.0, -1.0),
            point_t(-1.0, 0.0, -1.0),
            point_t(-1.0, 0.0,  1.0)
        }),
    data1(
        {
            point_t( 2.5,  0.5,  1.5),
            point_t(-1.5,  0.5,  5.0),
            point_t(-3.5, -0.8,  1.0),
            point_t(-0.5,  1.5, -2.0)
        }),
    data2(    /* Plane above data 0 */
        {
            point_t(0.0,  1.0,  1.0),
            point_t(0.0, -1.0,  1.0),
            point_t(0.0, -1.0, -1.0),
            point_t(0.0,  1.0, -1.0)
        }),
    data3(    /* v0 above data 0, then everything else above that */
        {
            point_t(-1.0,  1.0, -0.05),
            point_t( 1.0,  1.0,  0.05),
            point_t( 1.0, -1.0,  0.05),
            point_t(-1.0, -1.0,  0.05)
        }),
    data4(    /* v0 and v1 above data 0, then everything else above that */
        {
            point_t( 1.0, -1.0, -0.05),
            point_t(-1.0, -1.0, -0.05),
            point_t(-1.0,  1.0,  0.05),
            point_t( 1.0,  1.0,  0.05)
        }),
    data5(
        {
            point_t( 1.0, 0.0,  1.0),
            point_t( 1.0, 0.0, -1.0),
            point_t(-1.0, 0.0, -1.0),
            point_t(-1.0, 0.0,  1.0)
        }),
    data6(
        {
            point_t(1.0, 0.0, 0.0)
        }),
    data7(
        {
            point_t(0.0, 1.1, 0.0)
        }),
    data8(
        {
            point_t(-0.5, 0.0, 0.0)
        }),
    data9(
        {
            point_t( 1.0, -1.0, 0.0),
            point_t(-1.0,  1.0, 0.0)
        }),
    data10(
        {
            point_t( 1.0, 0.0, -1.0),
            point_t(-1.0, 0.0,  1.0)
        }),
    data11(
        {
            point_t(0.0,  1.0, -1.0),
            point_t(0.0, -1.0,  1.0)
        }),
    data12(
        {
            point_t(0.0, 0.0,  1.0),
            point_t(0.0, 0.0, -1.0)
        }),
    data13(
        {
            point_t(0.0, 1.0,  0.0),
            point_t(0.0, 0.0, -1.0),
            point_t(0.0, 0.0,  1.0)
        }),
    data14(
        {
            point_t(0.0, -1.0, 0.0)
        }),
    p0( physics_object_for_simplex_testing(data0,  quaternion_t(sqrt(0.5), -sqrt(0.5),      0.0,       0.0),  point_t(0.5, 0.5, 1.0 ))),
    p1( physics_object_for_simplex_testing(data1,  quaternion_t(     1.0,        0.0,       0.0,       0.0),  point_t(0.0, 0.0, 0.0 ))),
    p2( physics_object_for_simplex_testing(data2,  quaternion_t(sqrt(0.5),       0.0,  sqrt(0.5),      0.0),  point_t(0.5, 0.5, 1.1 ))),
    p3( physics_object_for_simplex_testing(data3,  quaternion_t(sqrt(0.5),       0.0,       0.0, -sqrt(0.5)), point_t(0.5, 0.5, 1.15))),
    p4( physics_object_for_simplex_testing(data4,  quaternion_t(sqrt(0.5),       0.0,       0.0,  sqrt(0.5)), point_t(0.5, 0.5, 1.15))),
    p5( physics_object_for_simplex_testing(data5,  quaternion_t(sqrt(0.5),  sqrt(0.5),      0.0,       0.0),  point_t(1.0, 1.0, 1.1 ))),
    p6( physics_object_for_simplex_testing(data6,  quaternion_t(sqrt(0.5),       0.0, -sqrt(0.5),      0.0),  point_t(1.5, 1.5, 0.0 ))),
    p7( physics_object_for_simplex_testing(data7,  quaternion_t(sqrt(0.5),  sqrt(0.5),      0.0,       0.0),  point_t(1.5, 1.5, 0.0 ))),
    p8( physics_object_for_simplex_testing(data8,  quaternion_t(sqrt(0.5),       0.0,       0.0,  sqrt(0.5)), point_t(1.5, 0.0, 1.1 ))),
    p9( physics_object_for_simplex_testing(data9,  quaternion_t(sqrt(0.5),       0.0,       0.0,  sqrt(0.5)), point_t(0.5, 0.5, 1.0 ))),
    p10(physics_object_for_simplex_testing(data10, quaternion_t(sqrt(0.5), -sqrt(0.5),      0.0,       0.0),  point_t(0.5, 0.5, 1.1 ))),
    p11(physics_object_for_simplex_testing(data11, quaternion_t(sqrt(0.5),       0.0,  sqrt(0.5),      0.0),  point_t(0.5, 0.5, 1.1 ))),
    p12(physics_object_for_simplex_testing(data12, quaternion_t(sqrt(0.5),  sqrt(0.5),      0.0,       0.0),  point_t(1.5, 0.5, 1.0 ))),
    p13(physics_object_for_simplex_testing(data13, quaternion_t(sqrt(0.5),       0.0,  sqrt(0.5),      0.0),  point_t(0.0, 0.0, 0.0 ))),
    p14(physics_object_for_simplex_testing(data14, quaternion_t(sqrt(0.5),  sqrt(0.5),      0.0,       0.0),  point_t(0.0, 0.0, 2.0 ))),
    s0(*p0),
    s1(*p1),
    s2(*p2),
    s3(*p3),
    s4(*p4),
    s5(*p5),
    s6(*p6),
    s7(*p7),
    s8(*p8),
    s9(*p9),
    s10(*p10),
    s11(*p11),
    s12(*p12),
    s13(*p13),
    s14(*p14)
    {  };

    // cppcheck-suppress unsafeClassCanLeak
    std::vector<point_t> data0;
    std::vector<point_t> data1;
    std::vector<point_t> data2;
    std::vector<point_t> data3;
    std::vector<point_t> data4;
    std::vector<point_t> data5;
    std::vector<point_t> data6;
    std::vector<point_t> data7;
    std::vector<point_t> data8;
    std::vector<point_t> data9;
    std::vector<point_t> data10;
    std::vector<point_t> data11;
    std::vector<point_t> data12;
    std::vector<point_t> data13;
    std::vector<point_t> data14;
    std::unique_ptr<physics_object> p0;
    std::unique_ptr<physics_object> p1;
    std::unique_ptr<physics_object> p2;
    std::unique_ptr<physics_object> p3;
    std::unique_ptr<physics_object> p4;
    std::unique_ptr<physics_object> p5;
    std::unique_ptr<physics_object> p6;
    std::unique_ptr<physics_object> p7;
    std::unique_ptr<physics_object> p8;
    std::unique_ptr<physics_object> p9;
    std::unique_ptr<physics_object> p10;
    std::unique_ptr<physics_object> p11;
    std::unique_ptr<physics_object> p12;
    std::unique_ptr<physics_object> p13;
    std::unique_ptr<physics_object> p14;
    simplex s0;
    simplex s1;
    simplex s2;
    simplex s3;
    simplex s4;
    simplex s5;
    simplex s6;     /* Point simplex        */
    simplex s7;     /* Above point simplex  */
    simplex s8;     /* Point at other end   */
    simplex s9;     /* Line simplex         */
    simplex s10;    /* Skew line simplex    */
    simplex s11;    /* Anti skew simplex    */
    simplex s12;    /* Straight line        */
    simplex s13;
    simplex s14;
};


BOOST_FIXTURE_TEST_SUITE( simplex_tests, simplex_fixture )

const float result_tolerance = 0.0005;


BOOST_AUTO_TEST_CASE( ctor_test )
{
    BOOST_CHECK(s0.size() == 1);
    BOOST_CHECK(s0.get_vertex(0) == point_t( 1.0, 0.0,  1.0));

    BOOST_CHECK(s3.size() == 1);
    BOOST_CHECK(fabs(magnitude(s3.get_vertex(0) - point_t(-1.0,  1.0, -0.05))) < result_tolerance);
}


BOOST_AUTO_TEST_CASE( copy_ctor_test )
{
    simplex s_copy0(s0);
    BOOST_CHECK(s_copy0.size() == 1);
    BOOST_CHECK(fabs(magnitude(s_copy0.get_displaced_vertex(point_t(1.0, 1.5, -2.0), 0) - point_t(1.0, 1.0, 0.0))) < result_tolerance);

    const int retain[] = { 0 };
    s0.retain_vertices(retain, 1);

    simplex s_copy1(s0);
    BOOST_CHECK(s_copy1.size() == 1);
    BOOST_CHECK(fabs(magnitude(s_copy1.get_displaced_vertex(point_t(1.0, 1.5, -2.0), 0) - point_t(1.0, 1.0, 0.0))) < result_tolerance);
}


BOOST_AUTO_TEST_CASE( simplex_add_test )
{
    BOOST_CHECK(s0.size() == 1);
    const point_t disp0(1.0, 1.5, -2.0);
    BOOST_CHECK(fabs(magnitude(s0.get_displaced_vertex(disp0, 0) - point_t( 1.0,  1.0, 0.0))) < result_tolerance);

    s0.add(2);
    BOOST_CHECK(s0.size() == 2);
    const point_t disp1(1.0, 1.5, -2.0);
    BOOST_CHECK(fabs(magnitude(s0.get_displaced_vertex(disp1, 0) - point_t( 1.0,  1.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s0.get_displaced_vertex(disp1, 1) - point_t(-1.0, -1.0, 0.0))) < result_tolerance);

    s0.add(1 | 0x80000000);
    BOOST_CHECK(s0.size() == 3);
    const point_t disp2(1.0, 1.5, -2.0);
    BOOST_CHECK(fabs(magnitude(s0.get_displaced_vertex(disp2, 0) - point_t( 1.0,  1.0,  0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s0.get_displaced_vertex(disp2, 1) - point_t(-1.0, -1.0,  0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s0.get_displaced_vertex(disp2, 2) - point_t( 2.0,  0.5, -2.0))) < result_tolerance);

    s0.add(3 | 0x80000000);
    BOOST_CHECK(s0.size() == 4);
    const point_t disp3(-1.0, -1.5, 2.0);
    BOOST_CHECK(fabs(magnitude(s0.get_displaced_vertex(disp3, 0) - point_t( 1.0,  1.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s0.get_displaced_vertex(disp3, 1) - point_t(-1.0, -1.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s0.get_displaced_vertex(disp3, 2) - point_t( 0.0, -2.5, 2.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s0.get_displaced_vertex(disp3, 3) - point_t(-2.0, -0.5, 2.0))) < result_tolerance);
}


BOOST_AUTO_TEST_CASE( get_displaced_vertex_test )
{
    s0.add(0x80000000);
    BOOST_CHECK(s0.get_vertex(0) == point_t( 1.0, 0.0,  1.0));
    BOOST_CHECK(fabs(magnitude(s0.get_displaced_vertex(point_t(1.0, 1.5, -2.0), 0) - point_t( 1.0, 1.0, 0.0))) < result_tolerance);

    BOOST_CHECK(s0.get_vertex(1) == point_t( 1.0, 0.0,  1.0));
    BOOST_CHECK(fabs(magnitude(s0.get_displaced_vertex(point_t(1.0, 1.5, -2.0), 1) - point_t( 2.0, 2.5, -2.0))) < result_tolerance);

    s3.add(0x80000000);
    BOOST_CHECK(fabs(magnitude(s3.get_vertex(0) - point_t(-1.0,  1.0, -0.05))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s3.get_displaced_vertex(point_t(1.5, 2.0, -0.85), 0) - point_t(1.0, 1.0, -0.05))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(s3.get_vertex(1) - point_t(-1.0,  1.0, -0.05))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s3.get_displaced_vertex(point_t(1.5, 2.0, -0.85), 1) - point_t( 2.5,  3.0, -0.9))) < result_tolerance);
}


BOOST_AUTO_TEST_CASE( compute_c_space_test )
{
    /* C space data */
    point_t c[4] = 
    {
        point_t(0.0, 0.0, 0.0),
        point_t(0.0, 0.0, 0.0),
        point_t(0.0, 0.0, 0.0),
        point_t(0.0, 0.0, 0.0)
    };
    const point_t fixed_rel_disp(0.0, 0.0, 0.0);
    const point_t float_rel_disp(0.0, 0.0, 0.0);
    s0.compute_c_space(s1, c, fixed_rel_disp, float_rel_disp);

    BOOST_CHECK(fabs(magnitude(c[0] - point_t(-1.5, 0.5, -1.5))) < result_tolerance);
    BOOST_CHECK(c[1] == point_t( 0.0, 0.0,  0.0));
    BOOST_CHECK(c[2] == point_t( 0.0, 0.0,  0.0));
    BOOST_CHECK(c[3] == point_t( 0.0, 0.0,  0.0));

    /* Increase the simplex size */
    s0.add(2);
    s1.add(0);
    s0.compute_c_space(s1, c, fixed_rel_disp, float_rel_disp);

    BOOST_CHECK(fabs(magnitude(c[0] - point_t(-1.5,  0.5, -1.5))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(c[1] - point_t(-3.5, -1.5, -1.5))) < result_tolerance);
    BOOST_CHECK(c[2] == point_t( 0.0,  0.0,  0.0));
    BOOST_CHECK(c[3] == point_t( 0.0,  0.0,  0.0));

    s0.add(1);
    s1.add(3);
    s0.compute_c_space(s1, c, fixed_rel_disp, float_rel_disp);

    BOOST_CHECK(fabs(magnitude(c[0] - point_t(-1.5,  0.5, -1.5))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(c[1] - point_t(-3.5, -1.5, -1.5))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(c[2] - point_t( 1.5, -2.5,  2.0))) < result_tolerance);
    BOOST_CHECK(c[3] == point_t( 0.0,  0.0,  0.0));

    s0.add(3);
    s1.add(1);
    s0.compute_c_space(s1, c, fixed_rel_disp, float_rel_disp);

    BOOST_CHECK(fabs(magnitude(c[0] - point_t(-1.5,  0.5, -1.5))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(c[1] - point_t(-3.5, -1.5, -1.5))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(c[2] - point_t( 1.5, -2.5,  2.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(c[3] - point_t( 0.5,  0.5, -5.0))) < result_tolerance);
}


BOOST_AUTO_TEST_CASE( compute_c_space_with_fixed_displacement_test )
{
    /* C space data */
    point_t c[4] = 
    {
        point_t(0.0, 0.0, 0.0),
        point_t(0.0, 0.0, 0.0),
        point_t(0.0, 0.0, 0.0),
        point_t(0.0, 0.0, 0.0)
    };
    const point_t float_rel_disp(0.0, 0.0, 0.0);
    s0.compute_c_space(s1, c, point_t(-0.5, -0.5, 2.5), float_rel_disp);

    BOOST_CHECK(fabs(magnitude(c[0] - point_t(-2.0, 0.0,  1.0))) < result_tolerance);
    BOOST_CHECK(c[1] == point_t( 0.0, 0.0,  0.0));
    BOOST_CHECK(c[2] == point_t( 0.0, 0.0,  0.0));
    BOOST_CHECK(c[3] == point_t( 0.0, 0.0,  0.0));

    /* Increase the simplex size */
    s0.add(2);
    s1.add(0);
    s0.compute_c_space(s1, c, point_t(2.5, 1.5, 2.0), float_rel_disp);

    BOOST_CHECK(fabs(magnitude(c[0] - point_t( 1.0,  2.0,  0.5))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(c[1] - point_t(-1.0,  0.0,  0.5))) < result_tolerance);
    BOOST_CHECK(c[2] == point_t( 0.0,  0.0,  0.0));
    BOOST_CHECK(c[3] == point_t( 0.0,  0.0,  0.0));

    s0.add(1);
    s1.add(3);
    s0.compute_c_space(s1, c, point_t(-0.5, 3.0, 2.0), float_rel_disp);

    BOOST_CHECK(fabs(magnitude(c[0] - point_t(-2.0,  3.5,  0.5))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(c[1] - point_t(-4.0,  1.5,  0.5))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(c[2] - point_t( 1.0,  0.5,  4.0))) < result_tolerance);
    BOOST_CHECK(c[3] == point_t( 0.0,  0.0,  0.0));

    s0.add(0);
    s1.add(2);
    s0.compute_c_space(s1, c, point_t(-4.5, -1.8, 1.0), float_rel_disp);

    BOOST_CHECK(magnitude(fabs(c[0] - point_t(-6.0, -1.3, -0.5))) < result_tolerance);
    BOOST_CHECK(magnitude(fabs(c[1] - point_t(-8.0, -3.3, -0.5))) < result_tolerance);
    BOOST_CHECK(magnitude(fabs(c[2] - point_t(-3.0, -4.3,  3.0))) < result_tolerance);
    BOOST_CHECK(magnitude(fabs(c[3] - point_t( 0.0,  0.0,  0.0))) < result_tolerance);
}


BOOST_AUTO_TEST_CASE( compute_c_space_with_floating_displacement_test )
{
    /* C space data */
    point_t c[4] = 
    {
        point_t(0.0, 0.0, 0.0),
        point_t(0.0, 0.0, 0.0),
        point_t(0.0, 0.0, 0.0),
        point_t(0.0, 0.0, 0.0)
    };
    s0.compute_c_space(s1, c, point_t(-0.5, -0.5, 2.5), point_t(1.0, 0.5, -1.5));

    BOOST_CHECK(fabs(magnitude(c[0] - point_t(-2.0, 0.0,  1.0))) < result_tolerance);   /* No displaced vertices used */
    BOOST_CHECK(c[1] == point_t( 0.0, 0.0,  0.0));
    BOOST_CHECK(c[2] == point_t( 0.0, 0.0,  0.0));
    BOOST_CHECK(c[3] == point_t( 0.0, 0.0,  0.0));

    /* Increase the simplex size */
    s0.add(2 | 0x80000000);
    s1.add(0);
    s0.compute_c_space(s1, c, point_t(2.5, 1.5, 2.0), point_t(2.0, 1.5, -1.0));

    BOOST_CHECK(fabs(magnitude(c[0] - point_t( 1.0,  2.0,  0.5))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(c[1] - point_t( 1.0,  1.5, -0.5))) < result_tolerance);
    BOOST_CHECK(c[2] == point_t( 0.0,  0.0,  0.0));
    BOOST_CHECK(c[3] == point_t( 0.0,  0.0,  0.0));

    s0.add(1);
    s1.add(3 | 0x80000000);
    s0.compute_c_space(s1, c, point_t(-0.5, 3.0, 2.0), point_t(-1.0, -0.5, 3.0));

    BOOST_CHECK(fabs(magnitude(c[0] - point_t(-2.0,  3.5,  0.5))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(c[1] - point_t(-5.0,  1.0,  3.5))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(c[2] - point_t( 2.0,  1.0,  1.0))) < result_tolerance);
    BOOST_CHECK(c[3] == point_t( 0.0,  0.0,  0.0));

    s0.add(0 | 0x80000000);
    s1.add(2 | 0x80000000);
    s0.compute_c_space(s1, c, point_t(-4.5, -1.8, 1.0), point_t(1.0, 0.5, -1.5));

    BOOST_CHECK(fabs(magnitude(c[0] - point_t(-6.0, -1.3, -0.5))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(c[1] - point_t(-7.0, -2.8, -2.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(c[2] - point_t(-4.0, -4.8,  4.5))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(c[3] - point_t( 0.0,  0.0,  0.0))) < result_tolerance);   /* Both vertices displaced */
}


BOOST_AUTO_TEST_CASE( is_new_pair_test )
{
    /* Test on base simplex */
    BOOST_CHECK(!s0.is_new_pair(s1, 0, 0));
    BOOST_CHECK(s0.is_new_pair(s1, 1, 0));
    BOOST_CHECK(s0.is_new_pair(s1, 0, 2));
    BOOST_CHECK(s0.is_new_pair(s1, 3, 3));

    /* Add a vert and check again */
    s0.add(1);
    s1.add(2);
    BOOST_CHECK(!s0.is_new_pair(s1, 0, 0));
    BOOST_CHECK(!s0.is_new_pair(s1, 1, 2));
    BOOST_CHECK(s0.is_new_pair(s1, 2, 1));
    BOOST_CHECK(s0.is_new_pair(s1, 2, 2));
    BOOST_CHECK(s0.is_new_pair(s1, 1, 1));
    BOOST_CHECK(s0.is_new_pair(s1, 0, 2));
    BOOST_CHECK(s0.is_new_pair(s1, 2, 0));

    /* Check with displaced vertex */
    s0.add(3 | 0x80000000);
    s1.add(3);
    BOOST_CHECK(!s0.is_new_pair(s1, 0, 0));
    BOOST_CHECK(!s0.is_new_pair(s1, 3 | 0x80000000, 3));
    BOOST_CHECK(s0.is_new_pair(s1, 3, 3 | 0x80000000));
    BOOST_CHECK(s0.is_new_pair(s1, 3, 3));
    BOOST_CHECK(s0.is_new_pair(s1, 3 | 0x80000000, 3 | 0x80000000));

    /* And one more for the complete set */
    s0.add(0);
    s1.add(3);
    BOOST_CHECK(!s0.is_new_pair(s1, 0, 0));
    BOOST_CHECK(!s0.is_new_pair(s1, 1, 2));
    BOOST_CHECK(!s0.is_new_pair(s1, 0, 3));
    BOOST_CHECK(s0.is_new_pair(s1, 2, 1));
    BOOST_CHECK(s0.is_new_pair(s1, 3, 0));
    BOOST_CHECK(s0.is_new_pair(s1, 1, 3));
    BOOST_CHECK(s0.is_new_pair(s1, 0, 2));
    BOOST_CHECK(s0.is_new_pair(s1, 2, 3));
}


BOOST_AUTO_TEST_CASE( retain_vertices_test )
{
    s0.add(3);
    s0.add(2);
    s0.add(1);

    /* Test flip order and keep half */
    const int retain_verts0[2] = { 3, 1 };
    s0.retain_vertices(retain_verts0, 2);
    BOOST_CHECK(s0.size() == 2);
    BOOST_CHECK(fabs(magnitude(s0.get_displaced_vertex(point_t(0.0, 0.0, 0.0), 0) - point_t( 1.0, -1.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s0.get_displaced_vertex(point_t(0.0, 0.0, 0.0), 1) - point_t(-1.0,  1.0, 0.0))) < result_tolerance);

    /* Test drop first */
    s0.add(2);
    s0.add(0);

    const int retain_verts1[3] = { 1, 2, 3 };
    s0.retain_vertices(retain_verts1, 3);
    BOOST_CHECK(s0.size() == 3);
    BOOST_CHECK(fabs(magnitude(s0.get_displaced_vertex(point_t(0.0, 0.0, 0.0), 0) - point_t(-1.0,  1.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s0.get_displaced_vertex(point_t(0.0, 0.0, 0.0), 1) - point_t(-1.0, -1.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s0.get_displaced_vertex(point_t(0.0, 0.0, 0.0), 2) - point_t( 1.0,  1.0, 0.0))) < result_tolerance);

    /* Test reversing the order */
    s0.add(1);

    const int retain_verts3[4] = { 3, 2, 1, 0 };
    s0.retain_vertices(retain_verts3, 4);
    BOOST_CHECK(s0.size() == 4);
    BOOST_CHECK(fabs(magnitude(s0.get_displaced_vertex(point_t(0.0, 0.0, 0.0), 0) - point_t( 1.0, -1.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s0.get_displaced_vertex(point_t(0.0, 0.0, 0.0), 1) - point_t( 1.0,  1.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s0.get_displaced_vertex(point_t(0.0, 0.0, 0.0), 2) - point_t(-1.0, -1.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s0.get_displaced_vertex(point_t(0.0, 0.0, 0.0), 3) - point_t(-1.0,  1.0, 0.0))) < result_tolerance);

    /* Test remove all, but last */
    const int retain_verts4[1] = { 3 };
    s0.retain_vertices(retain_verts4, 1);
    BOOST_CHECK(s0.size() == 1);
    BOOST_CHECK(fabs(magnitude(s0.get_displaced_vertex(point_t(0.0, 0.0, 0.0), 0) - point_t(-1.0,  1.0, 0.0))) < result_tolerance);
}


BOOST_AUTO_TEST_CASE ( vertex_normal_of_impact_tests )
{
    /* Vertex simplices */
    BOOST_CHECK(fabs(magnitude(s0.normal_of_impact(s1) - normalise(point_t(-1.0,  1.0, -0.5)))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s1.normal_of_impact(s0) - normalise(point_t( 1.0, -1.0,  0.5)))) < result_tolerance);
}


BOOST_AUTO_TEST_CASE ( line_normal_of_impact_tests )
{
    /* Parallel line */
    s0.add(1);  /* 0, 1 */
    s2.add(1);  /* 0, 1 */
    BOOST_CHECK(fabs(magnitude(s0.normal_of_impact(s2) - point_t(0.0, 0.0, -1.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s2.normal_of_impact(s0) - point_t(0.0, 0.0,  1.0))) < result_tolerance);

    /* Anti-parallel line */
    const int retain0[] = { 1, 0 };
    s2.retain_vertices(retain0, 2); /* 1, 0 */
    BOOST_CHECK(fabs(magnitude(s0.normal_of_impact(s2) - point_t(0.0, 0.0, -1.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s2.normal_of_impact(s0) - point_t(0.0, 0.0,  1.0))) < result_tolerance);

    /* Skew lines */
    s0.add(2);  /* 0, 1, 2 */
    const int retain1[] = { 0, 2 };
    s0.retain_vertices(retain1, 2);   /* 0, 2 */

    s2.add(3);  /* 1, 0, 3 */
    const int retain2[] = { 0, 2 };
    s2.retain_vertices(retain2, 2); /* 1, 3 */
    BOOST_CHECK(fabs(magnitude(s0.normal_of_impact(s2) - point_t(0.0, 0.0, -1.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s2.normal_of_impact(s0) - point_t(0.0, 0.0,  1.0))) < result_tolerance);

    /* Anti-skew line */
    const int retain3[] = { 1, 0 };
    s2.retain_vertices(retain3, 2); /* 3, 1 */
    BOOST_CHECK(fabs(magnitude(s0.normal_of_impact(s2) - point_t(0.0, 0.0, -1.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s2.normal_of_impact(s0) - point_t(0.0, 0.0,  1.0))) < result_tolerance);

    /* Line versus point */
    s0.add(1);  /* 0, 2, 1 */
    s0.retain_vertices(retain1, 2);   /* 0, 1 */
    s2.add(0);  /* 3, 1, 0 */
    s2.add(0);  /* 3, 1, 0, 0 */
    const int retain4[] = { 2, 3 };
    s2.retain_vertices(retain4, 2); /* 0, 0 */
    BOOST_CHECK(fabs(magnitude(s0.normal_of_impact(s2) - point_t(0.0, 0.0, -1.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s2.normal_of_impact(s0) - point_t(0.0, 0.0,  1.0))) < result_tolerance);

    /* Point at the other end of the line */
    s2.add(1);
    const int retain5[] = { 1, 1 };
    s2.retain_vertices(retain5, 2); /* 1, 1 */
    BOOST_CHECK(fabs(magnitude(s0.normal_of_impact(s2) - point_t(0.0, 0.0, -1.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s2.normal_of_impact(s0) - point_t(0.0, 0.0,  1.0))) < result_tolerance);
}


BOOST_AUTO_TEST_CASE ( face_normal_of_impact_tests )
{
    /* Face versus anything */
    s0.add(1);
    s0.add(2);
    s3.add(0);
    s3.add(0);
    BOOST_CHECK(fabs(magnitude(s0.normal_of_impact(s3) - point_t(0.0, 0.0, -1.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s3.normal_of_impact(s0) - point_t(0.0, 0.0,  1.0))) < result_tolerance);

    /* Cause flip because of general plane direction */
    s13.add(1);
    s13.add(2);
    s14.add(0);
    s14.add(0);
    BOOST_CHECK(fabs(magnitude(s13.normal_of_impact(s14) - point_t(0.0, 0.0, -1.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s14.normal_of_impact(s13) - point_t(0.0, 0.0,  1.0))) < result_tolerance);
}


BOOST_AUTO_TEST_CASE ( vertex_rate_of_change_of_normal_of_impact_tests )
{
    /* Vertex simplices */
    /* Static and moving about normal */
    p6->set_angular_velocity(point_t(0.0, 0.0, 1.0));
    p7->set_angular_velocity(point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(fabs(magnitude(s6.rate_of_change_of_normal_of_impact(s7, point_t(0.0, 0.0, -1.0)))) < result_tolerance);

    p6->set_angular_velocity(point_t(0.0, 0.0, 0.0));
    p7->set_angular_velocity(point_t(0.0, 0.0, 1.0));
    BOOST_CHECK(fabs(magnitude(s7.rate_of_change_of_normal_of_impact(s6, point_t(0.0, 0.0,  1.0)))) < result_tolerance);

    /* Angular movement */
    p6->set_angular_velocity(point_t(0.0,  1.0, 0.0));
    p7->set_angular_velocity(point_t(0.0, -1.0, 0.0));
    BOOST_CHECK(fabs(magnitude(s6.rate_of_change_of_normal_of_impact(s7, point_t(0.0, 0.0, -1.0)) - point_t( 21.0, 0.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s7.rate_of_change_of_normal_of_impact(s6, point_t(0.0, 0.0,  1.0)) - point_t(-21.0, 0.0, 0.0))) < result_tolerance);

    p6->set_angular_velocity(point_t(-1.0, 0.0, 0.0));
    p7->set_angular_velocity(point_t( 1.0, 0.0, 0.0));
    BOOST_CHECK(fabs(magnitude(s6.rate_of_change_of_normal_of_impact(s7, point_t(0.0, 0.0, -1.0)) - point_t( 0.0,  21.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s7.rate_of_change_of_normal_of_impact(s6, point_t(0.0, 0.0,  1.0)) - point_t( 0.0, -21.0, 0.0))) < result_tolerance);

    p6->set_angular_velocity(point_t(-1.0,  1.0, 0.0));
    p7->set_angular_velocity(point_t(-1.0, -1.0, 0.0));
    BOOST_CHECK(fabs(magnitude(s6.rate_of_change_of_normal_of_impact(s7, point_t(0.0, 0.0, -1.0)) - point_t( 21.0, -1.0, 0.0))) < result_tolerance);

    p6->set_angular_velocity(point_t(-1.0, -1.0, 0.0));
    p7->set_angular_velocity(point_t(-1.0,  1.0, 0.0));
    BOOST_CHECK(fabs(magnitude(s7.rate_of_change_of_normal_of_impact(s6, point_t(0.0, 0.0,  1.0)) - point_t( 21.0, 1.0, 0.0))) < result_tolerance);

    /* Translating in the normal and in sync */
    p6->set_velocity(point_t(0.0, 1.0,  5.0));
    p7->set_velocity(point_t(0.0, 1.0, -8.0));
    p6->set_angular_velocity(point_t(0.0, -1.0, 0.0));
    p7->set_angular_velocity(point_t(0.0, 1.0, 0.0));
    BOOST_CHECK(fabs(magnitude(s6.rate_of_change_of_normal_of_impact(s7, point_t(0.0, 0.0, -1.0)) - point_t(-21.0, 0.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s7.rate_of_change_of_normal_of_impact(s6, point_t(0.0, 0.0,  1.0)) - point_t( 21.0, 0.0, 0.0))) < result_tolerance);

    /* Translating */
    p6->set_velocity(point_t(0.0, 1.0, 0.0));
    p7->set_velocity(point_t(0.0, 0.0, 0.0));
    p6->set_angular_velocity(point_t(0.0, 1.0, 0.0));
    p7->set_angular_velocity(point_t(0.0, -1.0, 0.0));
    BOOST_CHECK(fabs(magnitude(s6.rate_of_change_of_normal_of_impact(s7, point_t(0.0, 0.0, -1.0)) - point_t( 21.0,  10.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s7.rate_of_change_of_normal_of_impact(s6, point_t(0.0, 0.0,  1.0)) - point_t(-21.0, -10.0, 0.0))) < result_tolerance);

    p6->set_velocity(point_t( 0.0,  1.0, 0.0));
    p7->set_velocity(point_t(-1.0, -1.0, 0.0));
    p6->set_angular_velocity(point_t(0.0,  1.0, 0.0));
    p7->set_angular_velocity(point_t(0.0, -1.0, 0.0));
    BOOST_CHECK(fabs(magnitude(s6.rate_of_change_of_normal_of_impact(s7, point_t(0.0, 0.0, -1.0)) - point_t( 31.0,  20.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s7.rate_of_change_of_normal_of_impact(s6, point_t(0.0, 0.0,  1.0)) - point_t(-31.0, -20.0, 0.0))) < result_tolerance);
}


BOOST_AUTO_TEST_CASE ( line_rate_of_change_of_normal_of_impact_tests )
{
    const point_t zero(0.0, 0.0, 0.0);

    /* Parallel line */
    // s0.add(1);  /* 0, 1 */
    // BOOST_CHECK(fabs(magnitude(s0.rate_of_change_of_normal_of_impact(s2, point_t(0.0, 0.0, -1.0), zero, point_t(1.0, 0.0, 0.0), com0, com2) - zero)) < result_tolerance);
    // BOOST_CHECK(fabs(magnitude(s2.rate_of_change_of_normal_of_impact(s0, point_t(0.0, 0.0,  1.0), point_t(0.0, 0.0, 1.0), zero, com2, com0) - zero)) < result_tolerance);

    // BOOST_CHECK(fabs(magnitude(s0.rate_of_change_of_normal_of_impact(s2, point_t(0.0, 0.0, -1.0), zero, point_t(0.0, 1.0, 0.0), com0, com2) - point_t(-10.0, 0.0, 0.0))) < result_tolerance);
    // BOOST_CHECK(fabs(magnitude(s2.rate_of_change_of_normal_of_impact(s0, point_t(0.0, 0.0,  1.0), point_t(0.0, 1.0, 0.0), zero, com2, com0) - point_t( 10.0, 0.0, 0.0))) < result_tolerance);

    // /* Anti-parallel line */
    // const int retain0[] = { 1, 0 };
    // s2.retain_vertices(retain0, 2); /* 1, 0 */
    // BOOST_CHECK(fabs(magnitude(s0.rate_of_change_of_normal_of_impact(s2, point_t(0.0, 0.0, -1.0), zero, point_t(1.0, 0.0, 0.0), com0, com2) - zero)) < result_tolerance);
    // BOOST_CHECK(fabs(magnitude(s2.rate_of_change_of_normal_of_impact(s0, point_t(0.0, 0.0,  1.0), point_t(0.0, 0.0, 1.0), zero, com2, com0) - zero)) < result_tolerance);
//shouldnt matter that com isnt above point[0]
    
    // BOOST_CHECK(fabs(magnitude(s0.rate_of_change_of_normal_of_impact(s2, point_t(0.0, 0.0, -1.0), zero, point_t(0.0, 1.0, 0.0), com0, com2) - point_t(-10.0, 0.0, 0.0))) < result_tolerance);
    // BOOST_CHECK(fabs(magnitude(s2.rate_of_change_of_normal_of_impact(s0, point_t(0.0, 0.0,  1.0), point_t(0.0, 1.0, 0.0), zero, com2, com0) - point_t( 10.0, 0.0, 0.0))) < result_tolerance);

    /* Skew lines */
    p9->set_angular_velocity(point_t(1.0, 1.0, 0.0));
    p10->set_angular_velocity(point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(fabs(magnitude(s9.rate_of_change_of_normal_of_impact(s10, point_t(0.0, 0.0, -1.0)) - zero)) < result_tolerance);

    p9->set_angular_velocity(point_t(1.0,  1.0, 0.0));
    p10->set_angular_velocity(point_t(1.0, -1.0, 0.0));
    BOOST_CHECK(fabs(magnitude(s10.rate_of_change_of_normal_of_impact(s9, point_t(0.0, 0.0,  1.0)) - zero)) < result_tolerance);

    p9->set_angular_velocity(point_t(0.0, 0.0, 1.0));
    p10->set_angular_velocity(point_t(0.0, 0.0, 1.0));
    BOOST_CHECK(fabs(magnitude(s9.rate_of_change_of_normal_of_impact(s10, point_t(0.0, 0.0, -1.0)) - zero)) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s10.rate_of_change_of_normal_of_impact(s9, point_t(0.0, 0.0,  1.0)) - zero)) < result_tolerance);

    p9->set_angular_velocity(point_t(2.0, 0.0, 0.0));
    p10->set_angular_velocity(point_t(0.0, 3.0, 0.0));
    BOOST_CHECK(fabs(magnitude(s9.rate_of_change_of_normal_of_impact(s10, point_t(0.0, 0.0, -1.0)) - point_t(-0.5, -2.5, 0.0))) < result_tolerance);

    p9->set_angular_velocity(point_t(0.0, 3.0, 0.0));
    p10->set_angular_velocity(point_t(2.0, 0.0, 0.0));
    BOOST_CHECK(fabs(magnitude(s10.rate_of_change_of_normal_of_impact(s9, point_t(0.0, 0.0,  1.0)) - point_t( 2.5, -0.5, 0.0))) < result_tolerance);

    /* Anti-skew line */
    p9->set_angular_velocity(point_t(1.0, 1.0, 0.0));
    p11->set_angular_velocity(point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(fabs(magnitude(s9.rate_of_change_of_normal_of_impact(s11, point_t(0.0, 0.0, -1.0)) - zero)) < result_tolerance);

    p9->set_angular_velocity(point_t(1.0,  1.0, 0.0));
    p11->set_angular_velocity(point_t(1.0, -1.0, 0.0));
    BOOST_CHECK(fabs(magnitude(s11.rate_of_change_of_normal_of_impact(s9, point_t(0.0, 0.0,  1.0)) - zero)) < result_tolerance);

    p9->set_angular_velocity(point_t(0.0, 0.0, 1.0));
    p11->set_angular_velocity(point_t(0.0, 0.0, 1.0));
    BOOST_CHECK(fabs(magnitude(s9.rate_of_change_of_normal_of_impact(s11, point_t(0.0, 0.0, -1.0)) - zero)) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s11.rate_of_change_of_normal_of_impact(s9, point_t(0.0, 0.0,  1.0)) - zero)) < result_tolerance);

    p9->set_angular_velocity(point_t(2.0, 0.0, 0.0));
    p11->set_angular_velocity(point_t(0.0, 3.0, 0.0));
    BOOST_CHECK(fabs(magnitude(s9.rate_of_change_of_normal_of_impact(s11, point_t(0.0, 0.0, -1.0)) - point_t(-0.5, -2.5, 0.0))) < result_tolerance);

    p9->set_angular_velocity(point_t(0.0, 3.0, 0.0));
    p11->set_angular_velocity(point_t(2.0, 0.0, 0.0));
    BOOST_CHECK(fabs(magnitude(s11.rate_of_change_of_normal_of_impact(s9, point_t(0.0, 0.0,  1.0)) - point_t( 2.5, -0.5, 0.0))) < result_tolerance);

    /* Line versus point */
    p7->set_angular_velocity(point_t(1.0, 0.0, 0.0));
    p12->set_angular_velocity(point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(fabs(magnitude(s12.rate_of_change_of_normal_of_impact(s7, point_t(0.0, 0.0, -1.0)) - zero)) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s7.rate_of_change_of_normal_of_impact(s12, point_t(0.0, 0.0,  1.0)) - zero)) < result_tolerance);

    p7->set_angular_velocity(point_t(0.0, 0.0, 0.0));
    p12->set_angular_velocity(point_t(1.0, 0.0, 0.0));
    BOOST_CHECK(fabs(magnitude(s12.rate_of_change_of_normal_of_impact(s7, point_t(0.0, 0.0, -1.0)) - point_t(0.0, -1.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s7.rate_of_change_of_normal_of_impact(s12, point_t(0.0, 0.0,  1.0)) - point_t(0.0,  1.0, 0.0))) < result_tolerance);

    p7->set_angular_velocity(point_t(0.0, 1.0, 0.0));
    p12->set_angular_velocity(point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(fabs(magnitude(s12.rate_of_change_of_normal_of_impact(s7, point_t(0.0, 0.0, -1.0)) - point_t( 11.0, 0.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s7.rate_of_change_of_normal_of_impact(s12, point_t(0.0, 0.0,  1.0)) - point_t(-11.0, 0.0, 0.0))) < result_tolerance);

    /* Point at the other end of the line */
    p8->set_angular_velocity(point_t(1.0, 0.0, 0.0));
    p12->set_angular_velocity(point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(fabs(magnitude(s12.rate_of_change_of_normal_of_impact(s8, point_t(0.0, 0.0, -1.0)) - zero)) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s8.rate_of_change_of_normal_of_impact(s12, point_t(0.0, 0.0,  1.0)) - zero)) < result_tolerance);

    p8->set_angular_velocity(point_t(0.0, 0.0, 0.0));
    p12->set_angular_velocity(point_t(1.0, 0.0, 0.0));
    BOOST_CHECK(fabs(magnitude(s12.rate_of_change_of_normal_of_impact(s8, point_t(0.0, 0.0, -1.0)) - point_t(0.0, -1.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s8.rate_of_change_of_normal_of_impact(s12, point_t(0.0, 0.0,  1.0)) - point_t(0.0,  1.0, 0.0))) < result_tolerance);

    p8->set_angular_velocity(point_t(0.0, 0.0, 1.0));
    p12->set_angular_velocity(point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(fabs(magnitude(s12.rate_of_change_of_normal_of_impact(s8, point_t(0.0, 0.0, -1.0)) - point_t( 5.0, 0.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s8.rate_of_change_of_normal_of_impact(s12, point_t(0.0, 0.0,  1.0)) - point_t(-5.0, 0.0, 0.0))) < result_tolerance);
}


BOOST_AUTO_TEST_CASE ( face_rate_of_change_of_normal_of_impact_tests )
{
    /* Face versus anything */
    /* Static and moving about normal */
    p0->set_angular_velocity(point_t(0.0, 0.0, 1.0));
    p3->set_angular_velocity(point_t(0.0, 0.0, 0.0));
    BOOST_CHECK(fabs(magnitude(s0.rate_of_change_of_normal_of_impact(s3, point_t(0.0, 0.0, -1.0)))) < result_tolerance);

    p0->set_angular_velocity(point_t(0.0, 0.0, 0.0));
    p3->set_angular_velocity(point_t(0.0, 0.0, 1.0));
    BOOST_CHECK(fabs(magnitude(s3.rate_of_change_of_normal_of_impact(s0, point_t(0.0, 0.0,  1.0)))) < result_tolerance);

    /* Moving */
    p0->set_angular_velocity(point_t(0.0, 1.0, 0.0));
    p3->set_angular_velocity(point_t(1.0, 0.0, 0.0));
    BOOST_CHECK(fabs(magnitude(s0.rate_of_change_of_normal_of_impact(s3, point_t(0.0, 0.0, -1.0)) - point_t(-1.0,  0.0, 0.0))) < result_tolerance);

    p0->set_angular_velocity(point_t(1.0, 0.0, 0.0));
    p3->set_angular_velocity(point_t(0.0, 1.0, 0.0));
    BOOST_CHECK(fabs(magnitude(s3.rate_of_change_of_normal_of_impact(s0, point_t(0.0, 0.0, 1.0)) - point_t(0.0, 1.0, 0.0))) < result_tolerance);
}


BOOST_AUTO_TEST_CASE ( vertex_center_of_impact_tests )
{
    BOOST_CHECK(fabs(magnitude(s0.center_of_impact(s3, s0.normal_of_impact(s3)) - point_t( 1.5,  1.5, 1.05))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s3.center_of_impact(s0, s3.normal_of_impact(s0)) - point_t( 1.5,  1.5, 1.05))) < result_tolerance);
}


BOOST_AUTO_TEST_CASE ( line_center_of_impact_tests )
{
    BOOST_CHECK(fabs(magnitude(s0.center_of_impact(s4, s0.normal_of_impact(s4)) - point_t( 1.5,  0.5, 1.05))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s4.center_of_impact(s0, s4.normal_of_impact(s0)) - point_t( 1.5,  0.5, 1.05))) < result_tolerance);
}


BOOST_AUTO_TEST_CASE ( face_center_of_impact_tests )
{
    BOOST_CHECK(fabs(magnitude(s0.center_of_impact(s2, s0.normal_of_impact(s2)) - point_t( 0.5,  0.5, 1.05))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s2.center_of_impact(s0, s2.normal_of_impact(s0)) - point_t( 0.5,  0.5, 1.05))) < result_tolerance);
}


BOOST_AUTO_TEST_CASE ( off_center_face_center_of_impact_tests )
{
    s0.add(1);
    s0.add(2);
    s5.add(0);
    s5.add(0);
    BOOST_CHECK(fabs(magnitude(s0.center_of_impact(s5, s0.normal_of_impact(s5)) - point_t( 0.75,  0.75, 1.05))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s5.center_of_impact(s0, s5.normal_of_impact(s0)) - point_t( 0.75,  0.75, 1.05))) < result_tolerance);
}

BOOST_AUTO_TEST_SUITE_END()
