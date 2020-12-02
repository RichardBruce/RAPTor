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


namespace raptor_physics
{
namespace test
{
/* Test data */
struct simplex_fixture : private boost::noncopyable
{
    simplex_fixture() :
    data0(new std::vector<point_t<>>(
        {
            point_t<>( 1.0f, 0.0f,  1.0f),
            point_t<>( 1.0f, 0.0f, -1.0f),
            point_t<>(-1.0f, 0.0f, -1.0f),
            point_t<>(-1.0f, 0.0f,  1.0f)
        })),
    data1(new std::vector<point_t<>>(
        {
            point_t<>( 2.5f,  0.5f,  1.5f),
            point_t<>(-1.5f,  0.5f,  5.0f),
            point_t<>(-3.5f, -0.8f,  1.0f),
            point_t<>(-0.5f,  1.5f, -2.0f)
        })),
    data2( new std::vector<point_t<>>(   /* Plane above data 0 */
        {
            point_t<>(0.0f,  1.0f,  1.0f),
            point_t<>(0.0f, -1.0f,  1.0f),
            point_t<>(0.0f, -1.0f, -1.0f),
            point_t<>(0.0f,  1.0f, -1.0f)
        })),
    data3(new std::vector<point_t<>>(    /* v0 above data 0, then everything else above that */
        {
            point_t<>(-1.0f,  1.0f, -0.05f),
            point_t<>( 1.0f,  1.0f,  0.05f),
            point_t<>( 1.0f, -1.0f,  0.05f),
            point_t<>(-1.0f, -1.0f,  0.05f)
        })),
    data4(new std::vector<point_t<>>(    /* v0 and v1 above data 0, then everything else above that */
        {
            point_t<>( 1.0f, -1.0f, -0.05f),
            point_t<>(-1.0f, -1.0f, -0.05f),
            point_t<>(-1.0f,  1.0f,  0.05f),
            point_t<>( 1.0f,  1.0f,  0.05f)
        })),
    data5(new std::vector<point_t<>>(
        {
            point_t<>( 1.0f, 0.0f,  1.0f),
            point_t<>( 1.0f, 0.0f, -1.0f),
            point_t<>(-1.0f, 0.0f, -1.0f),
            point_t<>(-1.0f, 0.0f,  1.0f)
        })),
    data6(new std::vector<point_t<>>(
        {
            point_t<>(1.0f, 0.0f, 0.0f)
        })),
    data7(new std::vector<point_t<>>(
        {
            point_t<>(0.0f, 1.1f, 0.0f)
        })),
    data8(new std::vector<point_t<>>(
        {
            point_t<>(-0.5f, 0.0f, 0.0f)
        })),
    data9(new std::vector<point_t<>>(
        {
            point_t<>( 1.0f, -1.0f, 0.0f),
            point_t<>(-1.0f,  1.0f, 0.0f)
        })),
    data10(new std::vector<point_t<>>(
        {
            point_t<>( 1.0f, 0.0f, -1.0f),
            point_t<>(-1.0f, 0.0f,  1.0f)
        })),
    data11(new std::vector<point_t<>>(
        {
            point_t<>(0.0f,  1.0f, -1.0f),
            point_t<>(0.0f, -1.0f,  1.0f)
        })),
    data12(new std::vector<point_t<>>(
        {
            point_t<>(0.0f, 0.0f,  1.0f),
            point_t<>(0.0f, 0.0f, -1.0f)
        })),
    data13(new std::vector<point_t<>>(
        {
            point_t<>(0.0f, 1.0f,  0.0f),
            point_t<>(0.0f, 0.0f, -1.0f),
            point_t<>(0.0f, 0.0f,  1.0f)
        })),
    data14(new std::vector<point_t<>>(
        {
            point_t<>(0.0f, -1.0f, 0.0f)
        })),
    data15(new std::vector<point_t<>>(
        {
            point_t<>(0.0f, 1.0f, 0.0f),
            point_t<>(0.0f, 2.0f, 0.0f),
            point_t<>(1.0f, 3.0f, 0.0f),
            point_t<>(2.0f, 3.0f, 0.0f),
            point_t<>(3.0f, 2.0f, 0.0f),
            point_t<>(3.0f, 1.0f, 0.0f),
            point_t<>(2.0f, 0.0f, 0.0f),
            point_t<>(1.0f, 0.0f, 0.0f)
        })),
    data16(new std::vector<point_t<>>(
        {
            point_t<>(0.0f, 1.0f, 0.1f),
            point_t<>(0.0f, 2.0f, 0.1f),
            point_t<>(1.0f, 3.0f, 0.1f),
            point_t<>(2.0f, 3.0f, 0.1f),
            point_t<>(3.0f, 2.0f, 0.1f),
            point_t<>(3.0f, 1.0f, 0.1f),
            point_t<>(2.0f, 0.0f, 0.1f),
            point_t<>(1.0f, 0.0f, 0.1f)
        })),
    p0( physics_object_for_simplex_testing(data0,  { 3, 2, 1, 0 }, quaternion_t(sqrt(0.5), -sqrt(0.5),      0.0,       0.0),  point_t<>(0.5, 0.5, 1.0 ))),
    p1( physics_object_for_simplex_testing(data1,  quaternion_t(     1.0,        0.0,       0.0,       0.0),  point_t<>(0.0, 0.0, 0.0 ))),
    p2( physics_object_for_simplex_testing(data2,  { 0, 1, 2, 3 }, quaternion_t(sqrt(0.5),       0.0,  sqrt(0.5),      0.0),  point_t<>(0.5, 0.5, 1.1 ))),
    p3( physics_object_for_simplex_testing(data3,  { 0, 1, 2, 3 }, quaternion_t(sqrt(0.5),       0.0,       0.0, -sqrt(0.5)), point_t<>(0.5, 0.5, 1.15))),
    p4( physics_object_for_simplex_testing(data4,  { 0, 1, 2, 3 }, quaternion_t(sqrt(0.5),       0.0,       0.0,  sqrt(0.5)), point_t<>(0.5, 0.5, 1.15))),
    p5( physics_object_for_simplex_testing(data5,  { 3, 2, 1, 0 }, quaternion_t(sqrt(0.5),  sqrt(0.5),      0.0,       0.0),  point_t<>(1.0, 1.0, 1.1 ))),
    p6( physics_object_for_simplex_testing(data6,  quaternion_t(sqrt(0.5),       0.0, -sqrt(0.5),      0.0),  point_t<>(1.5, 1.5, 0.0 ))),
    p7( physics_object_for_simplex_testing(data7,  quaternion_t(sqrt(0.5),  sqrt(0.5),      0.0,       0.0),  point_t<>(1.5, 1.5, 0.0 ))),
    p8( physics_object_for_simplex_testing(data8,  quaternion_t(sqrt(0.5),       0.0,       0.0,  sqrt(0.5)), point_t<>(1.5, 0.0, 1.1 ))),
    p9( physics_object_for_simplex_testing(data9,  quaternion_t(sqrt(0.5),       0.0,       0.0,  sqrt(0.5)), point_t<>(0.5, 0.5, 1.0 ))),
    p10(physics_object_for_simplex_testing(data10, quaternion_t(sqrt(0.5), -sqrt(0.5),      0.0,       0.0),  point_t<>(0.5, 0.5, 1.1 ))),
    p11(physics_object_for_simplex_testing(data11, quaternion_t(sqrt(0.5),       0.0,  sqrt(0.5),      0.0),  point_t<>(0.5, 0.5, 1.1 ))),
    p12(physics_object_for_simplex_testing(data12, quaternion_t(sqrt(0.5),  sqrt(0.5),      0.0,       0.0),  point_t<>(1.5, 0.5, 1.0 ))),
    p13(physics_object_for_simplex_testing(data13, quaternion_t(sqrt(0.5),       0.0,  sqrt(0.5),      0.0),  point_t<>(0.0, 0.0, 0.0 ))),
    p14(physics_object_for_simplex_testing(data14, quaternion_t(sqrt(0.5),  sqrt(0.5),      0.0,       0.0),  point_t<>(0.0, 0.0, 2.0 ))),
    p15(physics_object_for_simplex_testing(data15, { 7, 6, 5, 4, 3, 2, 1, 0 }, quaternion_t(1.0, 0.0, 0.0, 0.0), point_t<>(0.0, 0.0, 0.0))),
    p16(physics_object_for_simplex_testing(data16, { 0, 1, 2, 3, 4, 5, 6, 7 }, quaternion_t(1.0, 0.0, 0.0, 0.0), point_t<>(1.0, 1.0, 0.0))),
    s0(*p0, true),
    s1(*p1, true),
    s2(*p2, false),
    s3(*p3, true),
    s4(*p4, true),
    s5(*p5, false),
    s6(*p6, true),
    s7(*p7, true),
    s8(*p8, true),
    s9(*p9, true),
    s10(*p10, true),
    s11(*p11, true),
    s12(*p12, true),
    s13(*p13, true),
    s14(*p14, true),
    s15(*p15, true),
    s16(*p16, false)
    {  };

    std::vector<point_t<>> *const data0;
    std::vector<point_t<>> *const data1;
    std::vector<point_t<>> *const data2;
    std::vector<point_t<>> *const data3;
    std::vector<point_t<>> *const data4;
    std::vector<point_t<>> *const data5;
    std::vector<point_t<>> *const data6;
    std::vector<point_t<>> *const data7;
    std::vector<point_t<>> *const data8;
    std::vector<point_t<>> *const data9;
    std::vector<point_t<>> *const data10;
    std::vector<point_t<>> *const data11;
    std::vector<point_t<>> *const data12;
    std::vector<point_t<>> *const data13;
    std::vector<point_t<>> *const data14;
    std::vector<point_t<>> *const data15;
    std::vector<point_t<>> *const data16;
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
    std::unique_ptr<physics_object> p15;
    std::unique_ptr<physics_object> p16;
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
    simplex s15;
    simplex s16;
};


BOOST_FIXTURE_TEST_SUITE( simplex_tests, simplex_fixture )

const float result_tolerance = 0.0005f;


BOOST_AUTO_TEST_CASE( ctor_test )
{
    BOOST_CHECK(s0.size() == 1);
    BOOST_CHECK(s0.get_vertex(0) == point_t<>( 1.0f, 0.0f,  1.0f));

    BOOST_CHECK(s3.size() == 1);
    BOOST_CHECK(fabs(magnitude(s3.get_vertex(0) - point_t<>(-1.0f,  1.0f, -0.05f))) < result_tolerance);
}

BOOST_AUTO_TEST_CASE( copy_ctor_test )
{
    /* Copy */
    simplex s_copy0(s0);

    /* Checks */
    BOOST_CHECK(s_copy0.size()          == 1);
    BOOST_CHECK(s_copy0.get_vertex(0)   == point_t<>( 1.0f, 0.0f,  1.0f));

    /* Flip verts buffer*/
    const int retain[] = { 0 };
    s0.retain_vertices(retain, 1);
 
    /* Copy */
    simplex s_copy1(s0);

    /* Checks */
    BOOST_CHECK(s_copy1.size()          == 1);
    BOOST_CHECK(s_copy1.get_vertex(0)   == point_t<>( 1.0f, 0.0f,  1.0f));
}

BOOST_AUTO_TEST_CASE( simplex_add_test )
{
    BOOST_CHECK(s0.size() == 1);
    BOOST_CHECK(fabs(magnitude(s0.get_vertex(0) - point_t<>( 1.0f, 0.0f,  1.0f))) < result_tolerance);

    s0.add(2);
    BOOST_CHECK(s0.size() == 2);
    BOOST_CHECK(fabs(magnitude(s0.get_vertex(0) - point_t<>( 1.0f, 0.0f,  1.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s0.get_vertex(1) - point_t<>(-1.0f, 0.0f, -1.0f))) < result_tolerance);

    s0.add(1 | 0x80000000);
    BOOST_CHECK(s0.size() == 3);
    BOOST_CHECK(fabs(magnitude(s0.get_vertex(0) - point_t<>( 1.0f, 0.0f,  1.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s0.get_vertex(1) - point_t<>(-1.0f, 0.0f, -1.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s0.get_vertex(2) - point_t<>( 1.0f, 0.0f, -1.0f))) < result_tolerance);

    s0.add(3 | 0x80000000);
    BOOST_CHECK(s0.size() == 4);
    BOOST_CHECK(fabs(magnitude(s0.get_vertex(0) - point_t<>( 1.0f, 0.0f,  1.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s0.get_vertex(1) - point_t<>(-1.0f, 0.0f, -1.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s0.get_vertex(2) - point_t<>( 1.0f, 0.0f, -1.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s0.get_vertex(3) - point_t<>(-1.0f, 0.0f,  1.0f))) < result_tolerance);
}

BOOST_AUTO_TEST_CASE( get_displaced_vertex_test )
{
    s0.add(0x80000000);
    BOOST_CHECK(s0.get_vertex(0) == point_t<>( 1.0, 0.0,  1.0));
    BOOST_CHECK(fabs(magnitude(s0.get_displaced_vertex(quaternion_t(1.0f, 0.0f, 0.0f, 0.0f), point_t<>(1.0, 1.5, -2.0), 0) - point_t<>( 1.0, 0.0, 1.0))) < result_tolerance);

    BOOST_CHECK(s0.get_vertex(1) == point_t<>( 1.0, 0.0,  1.0));
    BOOST_CHECK(fabs(magnitude(s0.get_displaced_vertex(quaternion_t(1.0f, 0.0f, 0.0f, 0.0f), point_t<>(1.0, 1.5, -2.0), 1) - point_t<>( 2.0, 1.5, -1.0))) < result_tolerance);

    s3.add(0x80000000);
    BOOST_CHECK(fabs(magnitude(s3.get_vertex(0) - point_t<>(-1.0,  1.0, -0.05))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s3.get_displaced_vertex(quaternion_t(1.0f, 0.0f, 0.0f, 0.0f), point_t<>(1.5, 2.0, -0.85), 0) - point_t<>(-1.0, 1.0, -0.05))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(s3.get_vertex(1) - point_t<>(-1.0,  1.0, -0.05))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s3.get_displaced_vertex(quaternion_t(1.0f, 0.0f, 0.0f, 0.0f), point_t<>(1.5, 2.0, -0.85), 1) - point_t<>( 0.5,  3.0, -0.9))) < result_tolerance);
}

BOOST_AUTO_TEST_CASE( get_displaced_vertex_rotated_test )
{
    s0.add(0x80000000);
    BOOST_CHECK(s0.get_vertex(0) == point_t<>( 1.0, 0.0,  1.0));
    BOOST_CHECK(fabs(magnitude(s0.get_displaced_vertex(quaternion_t(1.0f / std::sqrt(2.0f), -1.0f / std::sqrt(2.0f), 0.0f, 0.0f), point_t<>(1.0, 1.5, -2.0), 0) - point_t<>( 1.0, 1.0, 0.0))) < result_tolerance);

    BOOST_CHECK(s0.get_vertex(1) == point_t<>( 1.0, 0.0,  1.0));
    BOOST_CHECK(fabs(magnitude(s0.get_displaced_vertex(quaternion_t(1.0f / std::sqrt(2.0f), -1.0f / std::sqrt(2.0f), 0.0f, 0.0f), point_t<>(1.0, 1.5, -2.0), 1) - point_t<>( 2.0, 2.5, -2.0))) < result_tolerance);

    s3.add(0x80000000);
    BOOST_CHECK(fabs(magnitude(s3.get_vertex(0) - point_t<>(-1.0,  1.0, -0.05))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s3.get_displaced_vertex(quaternion_t(1.0f / std::sqrt(2.0f), 0.0f, 0.0f, -1.0f / std::sqrt(2.0f)), point_t<>(1.5, 2.0, -0.85), 0) - point_t<>(1.0, 1.0, -0.05))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(s3.get_vertex(1) - point_t<>(-1.0,  1.0, -0.05))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s3.get_displaced_vertex(quaternion_t(1.0f / std::sqrt(2.0f), 0.0f, 0.0f, -1.0f / std::sqrt(2.0f)), point_t<>(1.5, 2.0, -0.85), 1) - point_t<>( 2.5,  3.0, -0.9))) < result_tolerance);
}

BOOST_AUTO_TEST_CASE( compute_c_space_test )
{
    /* C space data */
    point_t<> c[4] = 
    {
        point_t<>(0.0f, 0.0f, 0.0f),
        point_t<>(0.0f, 0.0f, 0.0f),
        point_t<>(0.0f, 0.0f, 0.0f),
        point_t<>(0.0f, 0.0f, 0.0f)
    };
    const point_t<> fixed_rel_disp(0.0, 0.0, 0.0);
    const point_t<> float_rel_disp(0.0, 0.0, 0.0);
    s0.compute_c_space(s1, c, quaternion_t(1.0f, 0.0f, 0.0f, 0.0f), quaternion_t(1.0f, 0.0f, 0.0f, 0.0f), fixed_rel_disp, float_rel_disp);

    BOOST_CHECK(fabs(magnitude(c[0] - point_t<>(-1.5f, -0.5f, -0.5f))) < result_tolerance);
    BOOST_CHECK(c[1] == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(c[2] == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(c[3] == point_t<>(0.0f, 0.0f, 0.0f));

    /* Increase the simplex size */
    s0.add(2);
    s1.add(0);
    s0.compute_c_space(s1, c, quaternion_t(1.0f, 0.0f, 0.0f, 0.0f), quaternion_t(1.0f, 0.0f, 0.0f, 0.0f), fixed_rel_disp, float_rel_disp);

    BOOST_CHECK(fabs(magnitude(c[0] - point_t<>(-1.5f, -0.5f, -0.5f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(c[1] - point_t<>(-3.5f, -0.5f, -2.5f))) < result_tolerance);
    BOOST_CHECK(c[2] == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(c[3] == point_t<>(0.0f, 0.0f, 0.0f));

    s0.add(1);
    s1.add(3);
    s0.compute_c_space(s1, c, quaternion_t(1.0f, 0.0f, 0.0f, 0.0f), quaternion_t(1.0f, 0.0f, 0.0f, 0.0f), fixed_rel_disp, float_rel_disp);

    BOOST_CHECK(fabs(magnitude(c[0] - point_t<>(-1.5f, -0.5f, -0.5f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(c[1] - point_t<>(-3.5f, -0.5f, -2.5f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(c[2] - point_t<>( 1.5f, -1.5f,  1.0f))) < result_tolerance);
    BOOST_CHECK(c[3] == point_t<>(0.0f, 0.0f, 0.0f));

    s0.add(3);
    s1.add(1);
    s0.compute_c_space(s1, c, quaternion_t(1.0f, 0.0f, 0.0f, 0.0f), quaternion_t(1.0f, 0.0f, 0.0f, 0.0f), fixed_rel_disp, float_rel_disp);

    BOOST_CHECK(fabs(magnitude(c[0] - point_t<>(-1.5f, -0.5f, -0.5f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(c[1] - point_t<>(-3.5f, -0.5f, -2.5f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(c[2] - point_t<>( 1.5f, -1.5f,  1.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(c[3] - point_t<>( 0.5f, -0.5f, -4.0f))) < result_tolerance);
}

BOOST_AUTO_TEST_CASE( compute_c_space_fixed_disp_test )
{
    /* C space data */
    point_t<> c[4] = 
    {
        point_t<>(0.0f, 0.0f, 0.0f),
        point_t<>(0.0f, 0.0f, 0.0f),
        point_t<>(0.0f, 0.0f, 0.0f),
        point_t<>(0.0f, 0.0f, 0.0f)
    };
    const point_t<> float_rel_disp(0.0, 0.0, 0.0);
    s0.compute_c_space(s1, c, quaternion_t(1.0f, 0.0f, 0.0f, 0.0f), quaternion_t(1.0f, 0.0f, 0.0f, 0.0f), point_t<>(0.5f, 0.5f, 0.5f), float_rel_disp);

    BOOST_CHECK(fabs(magnitude(c[0] - point_t<>(-1.0f, 0.0f, 0.0f))) < result_tolerance);
    BOOST_CHECK(c[1] == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(c[2] == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(c[3] == point_t<>(0.0f, 0.0f, 0.0f));

    /* Increase the simplex size */
    s0.add(2);
    s1.add(0);
    s0.compute_c_space(s1, c, quaternion_t(1.0f, 0.0f, 0.0f, 0.0f), quaternion_t(1.0f, 0.0f, 0.0f, 0.0f), point_t<>(1.5f, 0.0f, 0.0f), float_rel_disp);

    BOOST_CHECK(fabs(magnitude(c[0] - point_t<>( 0.0f, -0.5f, -0.5f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(c[1] - point_t<>(-2.0f, -0.5f, -2.5f))) < result_tolerance);
    BOOST_CHECK(c[2] == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(c[3] == point_t<>(0.0f, 0.0f, 0.0f));

    s0.add(1);
    s1.add(3);
    s0.compute_c_space(s1, c, quaternion_t(1.0f, 0.0f, 0.0f, 0.0f), quaternion_t(1.0f, 0.0f, 0.0f, 0.0f), point_t<>(0.0f, -1.5f, -1.0f), float_rel_disp);

    BOOST_CHECK(fabs(magnitude(c[0] - point_t<>(-1.5f, -2.0f, -1.5f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(c[1] - point_t<>(-3.5f, -2.0f, -3.5f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(c[2] - point_t<>( 1.5f, -3.0f,  0.0f))) < result_tolerance);
    BOOST_CHECK(c[3] == point_t<>(0.0f, 0.0f, 0.0f));

    s0.add(3);
    s1.add(1);
    s0.compute_c_space(s1, c, quaternion_t(1.0f, 0.0f, 0.0f, 0.0f), quaternion_t(1.0f, 0.0f, 0.0f, 0.0f), point_t<>(-3.5f, 0.5f, 4.0f), float_rel_disp);

    BOOST_CHECK(fabs(magnitude(c[0] - point_t<>(-5.0f,  0.0f,  3.5f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(c[1] - point_t<>(-7.0f,  0.0f,  1.5f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(c[2] - point_t<>(-2.0f, -1.0f,  5.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(c[3] - point_t<>(-3.0f,  0.0f, -0.0f))) < result_tolerance);
}

BOOST_AUTO_TEST_CASE( compute_c_space_floating_disp_test )
{
    /* C space data */
    point_t<> c[4] = 
    {
        point_t<>(0.0f, 0.0f, 0.0f),
        point_t<>(0.0f, 0.0f, 0.0f),
        point_t<>(0.0f, 0.0f, 0.0f),
        point_t<>(0.0f, 0.0f, 0.0f)
    };
    const point_t<> fixed_rel_disp(0.0, 0.0, 0.0);
    const point_t<> float_rel_disp(0.0, 0.0, 0.0);
    s0.compute_c_space(s1, c, quaternion_t(1.0f, 0.0f, 0.0f, 0.0f), quaternion_t(1.0f, 0.0f, 0.0f, 0.0f), fixed_rel_disp, point_t<>(0.5f, 0.5f, 0.5f));

    BOOST_CHECK(fabs(magnitude(c[0] - point_t<>(-1.5f, -0.5f, -0.5f))) < result_tolerance);
    BOOST_CHECK(c[1] == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(c[2] == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(c[3] == point_t<>(0.0f, 0.0f, 0.0f));

    /* Increase the simplex size */
    s0.add(0x80000002);
    s1.add(0x80000000);
    s0.compute_c_space(s1, c, quaternion_t(1.0f, 0.0f, 0.0f, 0.0f), quaternion_t(1.0f, 0.0f, 0.0f, 0.0f), fixed_rel_disp, point_t<>(1.5f, 0.0f, 0.0f));

    BOOST_CHECK(fabs(magnitude(c[0] - point_t<>(-1.5f, -0.5f, -0.5f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(c[1] - point_t<>(-3.5f, -0.5f, -2.5f))) < result_tolerance);
    BOOST_CHECK(c[2] == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(c[3] == point_t<>(0.0f, 0.0f, 0.0f));

    s0.add(0x80000001);
    s1.add(3);
    s0.compute_c_space(s1, c, quaternion_t(1.0f, 0.0f, 0.0f, 0.0f), quaternion_t(1.0f, 0.0f, 0.0f, 0.0f), fixed_rel_disp, point_t<>(0.0f, -1.5f, -1.0f));

    BOOST_CHECK(fabs(magnitude(c[0] - point_t<>(-1.5f, -0.5f, -0.5f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(c[1] - point_t<>(-3.5f, -0.5f, -2.5f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(c[2] - point_t<>( 1.5f, -3.0f,  0.0f))) < result_tolerance);
    BOOST_CHECK(c[3] == point_t<>(0.0f, 0.0f, 0.0f));

    s0.add(3);
    s1.add(0x80000001);
    s0.compute_c_space(s1, c, quaternion_t(1.0f, 0.0f, 0.0f, 0.0f), quaternion_t(1.0f, 0.0f, 0.0f, 0.0f), fixed_rel_disp, point_t<>(-3.5f, 0.5f, 4.0f));

    BOOST_CHECK(fabs(magnitude(c[0] - point_t<>(-1.5f, -0.5f, -0.5f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(c[1] - point_t<>(-3.5f, -0.5f, -2.5f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(c[2] - point_t<>(-2.0f, -1.0f,  5.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(c[3] - point_t<>( 4.0f, -1.0f, -8.0f))) < result_tolerance);
}

BOOST_AUTO_TEST_CASE( compute_c_space_rotated_test )
{
    /* C space data */
    point_t<> c[4] = 
    {
        point_t<>(0.0f, 0.0f, 0.0f),
        point_t<>(0.0f, 0.0f, 0.0f),
        point_t<>(0.0f, 0.0f, 0.0f),
        point_t<>(0.0f, 0.0f, 0.0f)
    };
    const point_t<> fixed_rel_disp(0.0, 0.0, 0.0);
    const point_t<> float_rel_disp(0.0, 0.0, 0.0);
    s0.compute_c_space(s1, c, quaternion_t(std::sqrt(0.5f), -std::sqrt(0.5f), 0.0f, 0.0f), quaternion_t(1.0f, 0.0f, 0.0f, 0.0f), fixed_rel_disp, float_rel_disp);

    BOOST_CHECK(fabs(magnitude(c[0] - point_t<>(-1.5f, 0.5f, -1.5f))) < result_tolerance);
    BOOST_CHECK(c[1] == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(c[2] == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(c[3] == point_t<>(0.0f, 0.0f, 0.0f));

    /* Increase the simplex size */
    s0.add(2);
    s1.add(0);
    s0.compute_c_space(s1, c, quaternion_t(std::sqrt(0.5f), -std::sqrt(0.5f), 0.0f, 0.0f), quaternion_t(1.0f, 0.0f, 0.0f, 0.0f), fixed_rel_disp, float_rel_disp);

    BOOST_CHECK(fabs(magnitude(c[0] - point_t<>(-1.5f,  0.5f, -1.5f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(c[1] - point_t<>(-3.5f, -1.5f, -1.5f))) < result_tolerance);
    BOOST_CHECK(c[2] == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(c[3] == point_t<>(0.0f, 0.0f, 0.0f));

    s0.add(1);
    s1.add(3);
    s0.compute_c_space(s1, c, quaternion_t(std::sqrt(0.5f), -std::sqrt(0.5f), 0.0f, 0.0f), quaternion_t(1.0f, 0.0f, 0.0f, 0.0f), fixed_rel_disp, float_rel_disp);

    BOOST_CHECK(fabs(magnitude(c[0] - point_t<>(-1.5f,  0.5f, -1.5f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(c[1] - point_t<>(-3.5f, -1.5f, -1.5f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(c[2] - point_t<>( 1.5f, -2.5f,  2.0f))) < result_tolerance);
    BOOST_CHECK(c[3] == point_t<>(0.0f, 0.0f, 0.0f));

    s0.add(3);
    s1.add(1);
    s0.compute_c_space(s1, c, quaternion_t(std::sqrt(0.5f), -std::sqrt(0.5f), 0.0f, 0.0f), quaternion_t(1.0f, 0.0f, 0.0f, 0.0f), fixed_rel_disp, float_rel_disp);

    BOOST_CHECK(fabs(magnitude(c[0] - point_t<>(-1.5f,  0.5f, -1.5f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(c[1] - point_t<>(-3.5f, -1.5f, -1.5f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(c[2] - point_t<>( 1.5f, -2.5f,  2.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(c[3] - point_t<>( 0.5f,  0.5f, -5.0f))) < result_tolerance);
}

BOOST_AUTO_TEST_CASE( compute_c_space_rotated_fixed_disp_test )
{
    /* C space data */
    point_t<> c[4] = 
    {
        point_t<>(0.0f, 0.0f, 0.0f),
        point_t<>(0.0f, 0.0f, 0.0f),
        point_t<>(0.0f, 0.0f, 0.0f),
        point_t<>(0.0f, 0.0f, 0.0f)
    };
    const point_t<> float_rel_disp(0.0f, 0.0f, 0.0f);
    s0.compute_c_space(s1, c, quaternion_t(std::sqrt(0.5f), -std::sqrt(0.5f), 0.0f, 0.0f), quaternion_t(1.0f, 0.0f, 0.0f, 0.0f), point_t<>(-0.5f, -0.5f, 2.5f), float_rel_disp);

    BOOST_CHECK(fabs(magnitude(c[0] - point_t<>(-2.0f,  0.0f,  1.0f))) < result_tolerance);
    BOOST_CHECK(c[1] == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(c[2] == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(c[3] == point_t<>(0.0f, 0.0f, 0.0f));

    /* Increase the simplex size */
    s0.add(2);
    s1.add(0);
    s0.compute_c_space(s1, c, quaternion_t(std::sqrt(0.5f), -std::sqrt(0.5f), 0.0f, 0.0f), quaternion_t(1.0f, 0.0f, 0.0f, 0.0f), point_t<>(2.5f, 1.5f, 2.0f), float_rel_disp);

    BOOST_CHECK(fabs(magnitude(c[0] - point_t<>( 1.0f,  2.0f,  0.5f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(c[1] - point_t<>(-1.0f,  0.0f,  0.5f))) < result_tolerance);
    BOOST_CHECK(c[2] == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(c[3] == point_t<>(0.0f, 0.0f, 0.0f));

    s0.add(1);
    s1.add(3);
    s0.compute_c_space(s1, c, quaternion_t(std::sqrt(0.5f), -std::sqrt(0.5f), 0.0f, 0.0f), quaternion_t(1.0f, 0.0f, 0.0f, 0.0f), point_t<>(-0.5f, 3.0f, 2.0f), float_rel_disp);

    BOOST_CHECK(fabs(magnitude(c[0] - point_t<>(-2.0f,  3.5f,  0.5f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(c[1] - point_t<>(-4.0f,  1.5f,  0.5f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(c[2] - point_t<>( 1.0f,  0.5f,  4.0f))) < result_tolerance);
    BOOST_CHECK(c[3] == point_t<>(0.0f, 0.0f, 0.0f));

    s0.add(0);
    s1.add(2);
    s0.compute_c_space(s1, c, quaternion_t(std::sqrt(0.5f), -std::sqrt(0.5f), 0.0f, 0.0f), quaternion_t(1.0f, 0.0f, 0.0f, 0.0f), point_t<>(-4.5f, -1.8f, 1.0f), float_rel_disp);

    BOOST_CHECK(magnitude(fabs(c[0] - point_t<>(-6.0f, -1.3f, -0.5f))) < result_tolerance);
    BOOST_CHECK(magnitude(fabs(c[1] - point_t<>(-8.0f, -3.3f, -0.5f))) < result_tolerance);
    BOOST_CHECK(magnitude(fabs(c[2] - point_t<>(-3.0f, -4.3f,  3.0f))) < result_tolerance);
    BOOST_CHECK(magnitude(fabs(c[3] - point_t<>( 0.0f,  0.0f,  0.0f))) < result_tolerance);
}

BOOST_AUTO_TEST_CASE( compute_c_space_rotated_fixed_and_floating_disp_test )
{
    /* C space data */
    point_t<> c[4] = 
    {
        point_t<>(0.0f, 0.0f, 0.0f),
        point_t<>(0.0f, 0.0f, 0.0f),
        point_t<>(0.0f, 0.0f, 0.0f),
        point_t<>(0.0f, 0.0f, 0.0f)
    };
    s0.compute_c_space(s1, c, quaternion_t(std::sqrt(0.5f), -std::sqrt(0.5f), 0.0f, 0.0f), quaternion_t(1.0f, 0.0f, 0.0f, 0.0f), point_t<>(-0.5, -0.5, 2.5), point_t<>(1.0, 0.5, -1.5));

    BOOST_CHECK(fabs(magnitude(c[0] - point_t<>(-2.0f, 0.0f,  1.0f))) < result_tolerance);   /* No displaced vertices used */
    BOOST_CHECK(c[1] == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(c[2] == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(c[3] == point_t<>(0.0f, 0.0f, 0.0f));

    /* Increase the simplex size */
    s0.add(2 | 0x80000000);
    s1.add(0);
    s0.compute_c_space(s1, c, quaternion_t(std::sqrt(0.5f), -std::sqrt(0.5f), 0.0f, 0.0f), quaternion_t(1.0f, 0.0f, 0.0f, 0.0f), point_t<>(2.5, 1.5, 2.0), point_t<>(2.0, 1.5, -1.0));

    BOOST_CHECK(fabs(magnitude(c[0] - point_t<>( 1.0f,  2.0f,  0.5f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(c[1] - point_t<>( 1.0f,  1.5f, -0.5f))) < result_tolerance);
    BOOST_CHECK(c[2] == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(c[3] == point_t<>(0.0f, 0.0f, 0.0f));

    s0.add(1);
    s1.add(3 | 0x80000000);
    s0.compute_c_space(s1, c, quaternion_t(std::sqrt(0.5f), -std::sqrt(0.5f), 0.0f, 0.0f), quaternion_t(1.0f, 0.0f, 0.0f, 0.0f), point_t<>(-0.5, 3.0, 2.0), point_t<>(-1.0, -0.5, 3.0));

    BOOST_CHECK(fabs(magnitude(c[0] - point_t<>(-2.0f,  3.5f,  0.5f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(c[1] - point_t<>(-5.0f,  1.0f,  3.5f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(c[2] - point_t<>( 2.0f,  1.0f,  1.0f))) < result_tolerance);
    BOOST_CHECK(c[3] == point_t<>(0.0f, 0.0f, 0.0f));

    s0.add(0 | 0x80000000);
    s1.add(2 | 0x80000000);
    s0.compute_c_space(s1, c, quaternion_t(std::sqrt(0.5f), -std::sqrt(0.5f), 0.0f, 0.0f), quaternion_t(1.0f, 0.0f, 0.0f, 0.0f), point_t<>(-4.5, -1.8, 1.0), point_t<>(1.0, 0.5, -1.5));

    BOOST_CHECK(fabs(magnitude(c[0] - point_t<>(-6.0f, -1.3f, -0.5f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(c[1] - point_t<>(-7.0f, -2.8f, -2.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(c[2] - point_t<>(-4.0f, -4.8f,  4.5f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(c[3] - point_t<>( 0.0f,  0.0f,  0.0f))) < result_tolerance);   /* Both vertices displaced */
}


BOOST_AUTO_TEST_CASE( unique_size_test )
{
    BOOST_CHECK(s0.unique_size() == 1);
    BOOST_CHECK(s1.unique_size() == 1);

    s0.add(0x80000000);
    s1.add(3);
    BOOST_CHECK(s0.unique_size() == 1);
    BOOST_CHECK(s1.unique_size() == 2);

    s0.add(0x80000001);
    s1.add(0x80000003);
    BOOST_CHECK(s0.unique_size() == 2);
    BOOST_CHECK(s1.unique_size() == 2);

    s0.add(2);
    s1.add(0x80000000);
    BOOST_CHECK(s0.unique_size() == 3);
    BOOST_CHECK(s1.unique_size() == 2);
}

BOOST_AUTO_TEST_CASE( is_new_pair_test )
{
    /* Test on base simplex */
    BOOST_CHECK(!s0.is_new_pair(s1, 0, 0));
    BOOST_CHECK( s0.is_new_pair(s1, 1, 0));
    BOOST_CHECK( s0.is_new_pair(s1, 0, 2));
    BOOST_CHECK( s0.is_new_pair(s1, 3, 3));

    /* Add a vert and check again */
    s0.add(1);
    s1.add(2);
    BOOST_CHECK(!s0.is_new_pair(s1, 0, 0));
    BOOST_CHECK(!s0.is_new_pair(s1, 1, 2));
    BOOST_CHECK( s0.is_new_pair(s1, 2, 1));
    BOOST_CHECK( s0.is_new_pair(s1, 2, 2));
    BOOST_CHECK( s0.is_new_pair(s1, 1, 1));
    BOOST_CHECK( s0.is_new_pair(s1, 0, 2));
    BOOST_CHECK( s0.is_new_pair(s1, 2, 0));

    /* Check with displaced vertex */
    s0.add(3 | 0x80000000);
    s1.add(3);
    BOOST_CHECK(!s0.is_new_pair(s1, 0, 0));
    BOOST_CHECK(!s0.is_new_pair(s1, 3 | 0x80000000, 3));
    BOOST_CHECK( s0.is_new_pair(s1, 3, 3 | 0x80000000));
    BOOST_CHECK( s0.is_new_pair(s1, 3, 3));
    BOOST_CHECK( s0.is_new_pair(s1, 3 | 0x80000000, 3 | 0x80000000));

    /* And one more for the complete set */
    s0.add(0);
    s1.add(3);
    BOOST_CHECK(!s0.is_new_pair(s1, 0, 0));
    BOOST_CHECK(!s0.is_new_pair(s1, 1, 2));
    BOOST_CHECK(!s0.is_new_pair(s1, 0, 3));
    BOOST_CHECK( s0.is_new_pair(s1, 2, 1));
    BOOST_CHECK( s0.is_new_pair(s1, 3, 0));
    BOOST_CHECK( s0.is_new_pair(s1, 1, 3));
    BOOST_CHECK( s0.is_new_pair(s1, 0, 2));
    BOOST_CHECK( s0.is_new_pair(s1, 2, 3));
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
    BOOST_CHECK(fabs(magnitude(s0.get_vertex(0) - point_t<>( 1.0f, 0.0f, -1.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s0.get_vertex(1) - point_t<>(-1.0f, 0.0f,  1.0f))) < result_tolerance);

    /* Test drop first */
    s0.add(2);
    s0.add(0);

    const int retain_verts1[3] = { 1, 2, 3 };
    s0.retain_vertices(retain_verts1, 3);
    BOOST_CHECK(s0.size() == 3);
    BOOST_CHECK(fabs(magnitude(s0.get_vertex(0) - point_t<>(-1.0f, 0.0f,  1.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s0.get_vertex(1) - point_t<>(-1.0f, 0.0f, -1.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s0.get_vertex(2) - point_t<>( 1.0f, 0.0f,  1.0f))) < result_tolerance);

    /* Test reversing the order */
    s0.add(1);

    const int retain_verts3[4] = { 3, 2, 1, 0 };
    s0.retain_vertices(retain_verts3, 4);
    BOOST_CHECK(s0.size() == 4);
    BOOST_CHECK(fabs(magnitude(s0.get_vertex(0) - point_t<>( 1.0f, 0.0f, -1.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s0.get_vertex(1) - point_t<>( 1.0f, 0.0f,  1.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s0.get_vertex(2) - point_t<>(-1.0f, 0.0f, -1.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s0.get_vertex(3) - point_t<>(-1.0f, 0.0f,  1.0f))) < result_tolerance);

    /* Test remove all, but last */
    const int retain_verts4[1] = { 3 };
    s0.retain_vertices(retain_verts4, 1);
    BOOST_CHECK(s0.size() == 1);
    BOOST_CHECK(fabs(magnitude(s0.get_vertex(0) - point_t<>(-1.0f, 0.0f,  1.0f))) < result_tolerance);
}

BOOST_AUTO_TEST_CASE ( vertex_rate_of_change_of_normal_of_impact_tests )
{
    /* Vertex simplices */
    /* Static and moving about normal */
    p6->set_angular_velocity(point_t<>(0.0, 0.0, 1.0));
    p7->set_angular_velocity(point_t<>(0.0, 0.0, 0.0));
    BOOST_CHECK(fabs(magnitude(s6.rate_of_change_of_normal_of_impact(s7, point_t<>(0.0, 0.0, -1.0)))) < result_tolerance);

    p6->set_angular_velocity(point_t<>(0.0, 0.0, 0.0));
    p7->set_angular_velocity(point_t<>(0.0, 0.0, 1.0));
    BOOST_CHECK(fabs(magnitude(s7.rate_of_change_of_normal_of_impact(s6, point_t<>(0.0, 0.0,  1.0)))) < result_tolerance);

    /* Angular movement */
    p6->set_angular_velocity(point_t<>(0.0,  1.0, 0.0));
    p7->set_angular_velocity(point_t<>(0.0, -1.0, 0.0));
    BOOST_CHECK(fabs(magnitude(s6.rate_of_change_of_normal_of_impact(s7, point_t<>(0.0, 0.0, -1.0)) - point_t<>( 21.0, 0.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s7.rate_of_change_of_normal_of_impact(s6, point_t<>(0.0, 0.0,  1.0)) - point_t<>(-21.0, 0.0, 0.0))) < result_tolerance);

    p6->set_angular_velocity(point_t<>(-1.0, 0.0, 0.0));
    p7->set_angular_velocity(point_t<>( 1.0, 0.0, 0.0));
    BOOST_CHECK(fabs(magnitude(s6.rate_of_change_of_normal_of_impact(s7, point_t<>(0.0, 0.0, -1.0)) - point_t<>( 0.0,  21.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s7.rate_of_change_of_normal_of_impact(s6, point_t<>(0.0, 0.0,  1.0)) - point_t<>( 0.0, -21.0, 0.0))) < result_tolerance);

    p6->set_angular_velocity(point_t<>(-1.0,  1.0, 0.0));
    p7->set_angular_velocity(point_t<>(-1.0, -1.0, 0.0));
    BOOST_CHECK(fabs(magnitude(s6.rate_of_change_of_normal_of_impact(s7, point_t<>(0.0, 0.0, -1.0)) - point_t<>( 21.0, -1.0, 0.0))) < result_tolerance);

    p6->set_angular_velocity(point_t<>(-1.0, -1.0, 0.0));
    p7->set_angular_velocity(point_t<>(-1.0,  1.0, 0.0));
    BOOST_CHECK(fabs(magnitude(s7.rate_of_change_of_normal_of_impact(s6, point_t<>(0.0, 0.0,  1.0)) - point_t<>( 21.0, 1.0, 0.0))) < result_tolerance);

    /* Translating in the normal and in sync */
    p6->set_velocity(point_t<>(0.0, 1.0,  5.0));
    p7->set_velocity(point_t<>(0.0, 1.0, -8.0));
    p6->set_angular_velocity(point_t<>(0.0, -1.0, 0.0));
    p7->set_angular_velocity(point_t<>(0.0, 1.0, 0.0));
    BOOST_CHECK(fabs(magnitude(s6.rate_of_change_of_normal_of_impact(s7, point_t<>(0.0, 0.0, -1.0)) - point_t<>(-21.0, 0.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s7.rate_of_change_of_normal_of_impact(s6, point_t<>(0.0, 0.0,  1.0)) - point_t<>( 21.0, 0.0, 0.0))) < result_tolerance);

    /* Translating */
    p6->set_velocity(point_t<>(0.0, 1.0, 0.0));
    p7->set_velocity(point_t<>(0.0, 0.0, 0.0));
    p6->set_angular_velocity(point_t<>(0.0, 1.0, 0.0));
    p7->set_angular_velocity(point_t<>(0.0, -1.0, 0.0));
    BOOST_CHECK(fabs(magnitude(s6.rate_of_change_of_normal_of_impact(s7, point_t<>(0.0, 0.0, -1.0)) - point_t<>( 21.0,  10.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s7.rate_of_change_of_normal_of_impact(s6, point_t<>(0.0, 0.0,  1.0)) - point_t<>(-21.0, -10.0, 0.0))) < result_tolerance);

    p6->set_velocity(point_t<>( 0.0,  1.0, 0.0));
    p7->set_velocity(point_t<>(-1.0, -1.0, 0.0));
    p6->set_angular_velocity(point_t<>(0.0,  1.0, 0.0));
    p7->set_angular_velocity(point_t<>(0.0, -1.0, 0.0));
    BOOST_CHECK(fabs(magnitude(s6.rate_of_change_of_normal_of_impact(s7, point_t<>(0.0, 0.0, -1.0)) - point_t<>( 31.0,  20.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s7.rate_of_change_of_normal_of_impact(s6, point_t<>(0.0, 0.0,  1.0)) - point_t<>(-31.0, -20.0, 0.0))) < result_tolerance);
}

BOOST_AUTO_TEST_CASE ( line_rate_of_change_of_normal_of_impact_tests )
{
    const point_t<> zero(0.0f, 0.f, 0.0f);

    /* Parallel line */
    // s0.add(1);  /* 0, 1 */
    // BOOST_CHECK(fabs(magnitude(s0.rate_of_change_of_normal_of_impact(s2, point_t<>(0.0, 0.0, -1.0), zero, point_t<>(1.0, 0.0, 0.0), com0, com2) - zero)) < result_tolerance);
    // BOOST_CHECK(fabs(magnitude(s2.rate_of_change_of_normal_of_impact(s0, point_t<>(0.0, 0.0,  1.0), point_t<>(0.0, 0.0, 1.0), zero, com2, com0) - zero)) < result_tolerance);

    // BOOST_CHECK(fabs(magnitude(s0.rate_of_change_of_normal_of_impact(s2, point_t<>(0.0, 0.0, -1.0), zero, point_t<>(0.0, 1.0, 0.0), com0, com2) - point_t<>(-10.0, 0.0, 0.0))) < result_tolerance);
    // BOOST_CHECK(fabs(magnitude(s2.rate_of_change_of_normal_of_impact(s0, point_t<>(0.0, 0.0,  1.0), point_t<>(0.0, 1.0, 0.0), zero, com2, com0) - point_t<>( 10.0, 0.0, 0.0))) < result_tolerance);

    // /* Anti-parallel line */
    // const int retain0[] = { 1, 0 };
    // s2.retain_vertices(retain0, 2); /* 1, 0 */
    // BOOST_CHECK(fabs(magnitude(s0.rate_of_change_of_normal_of_impact(s2, point_t<>(0.0, 0.0, -1.0), zero, point_t<>(1.0, 0.0, 0.0), com0, com2) - zero)) < result_tolerance);
    // BOOST_CHECK(fabs(magnitude(s2.rate_of_change_of_normal_of_impact(s0, point_t<>(0.0, 0.0,  1.0), point_t<>(0.0, 0.0, 1.0), zero, com2, com0) - zero)) < result_tolerance);
//shouldnt matter that com isnt above point[0]
    
    // BOOST_CHECK(fabs(magnitude(s0.rate_of_change_of_normal_of_impact(s2, point_t<>(0.0, 0.0, -1.0), zero, point_t<>(0.0, 1.0, 0.0), com0, com2) - point_t<>(-10.0, 0.0, 0.0))) < result_tolerance);
    // BOOST_CHECK(fabs(magnitude(s2.rate_of_change_of_normal_of_impact(s0, point_t<>(0.0, 0.0,  1.0), point_t<>(0.0, 1.0, 0.0), zero, com2, com0) - point_t<>( 10.0, 0.0, 0.0))) < result_tolerance);

    /* Skew lines */
    p9->set_angular_velocity(point_t<>(1.0, 1.0, 0.0));
    p10->set_angular_velocity(point_t<>(0.0, 0.0, 0.0));
    BOOST_CHECK(fabs(magnitude(s9.rate_of_change_of_normal_of_impact(s10, point_t<>(0.0, 0.0, -1.0)) - zero)) < result_tolerance);

    p9->set_angular_velocity(point_t<>(1.0,  1.0, 0.0));
    p10->set_angular_velocity(point_t<>(1.0, -1.0, 0.0));
    BOOST_CHECK(fabs(magnitude(s10.rate_of_change_of_normal_of_impact(s9, point_t<>(0.0, 0.0,  1.0)) - zero)) < result_tolerance);

    p9->set_angular_velocity(point_t<>(0.0, 0.0, 1.0));
    p10->set_angular_velocity(point_t<>(0.0, 0.0, 1.0));
    BOOST_CHECK(fabs(magnitude(s9.rate_of_change_of_normal_of_impact(s10, point_t<>(0.0, 0.0, -1.0)) - zero)) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s10.rate_of_change_of_normal_of_impact(s9, point_t<>(0.0, 0.0,  1.0)) - zero)) < result_tolerance);

    p9->set_angular_velocity(point_t<>(2.0, 0.0, 0.0));
    p10->set_angular_velocity(point_t<>(0.0, 3.0, 0.0));
    BOOST_CHECK(fabs(magnitude(s9.rate_of_change_of_normal_of_impact(s10, point_t<>(0.0, 0.0, -1.0)) - point_t<>(-0.5, -2.5, 0.0))) < result_tolerance);

    p9->set_angular_velocity(point_t<>(0.0, 3.0, 0.0));
    p10->set_angular_velocity(point_t<>(2.0, 0.0, 0.0));
    BOOST_CHECK(fabs(magnitude(s10.rate_of_change_of_normal_of_impact(s9, point_t<>(0.0, 0.0,  1.0)) - point_t<>( 2.5, -0.5, 0.0))) < result_tolerance);

    /* Anti-skew line */
    p9->set_angular_velocity(point_t<>(1.0, 1.0, 0.0));
    p11->set_angular_velocity(point_t<>(0.0, 0.0, 0.0));
    BOOST_CHECK(fabs(magnitude(s9.rate_of_change_of_normal_of_impact(s11, point_t<>(0.0, 0.0, -1.0)) - zero)) < result_tolerance);

    p9->set_angular_velocity(point_t<>(1.0,  1.0, 0.0));
    p11->set_angular_velocity(point_t<>(1.0, -1.0, 0.0));
    BOOST_CHECK(fabs(magnitude(s11.rate_of_change_of_normal_of_impact(s9, point_t<>(0.0, 0.0,  1.0)) - zero)) < result_tolerance);

    p9->set_angular_velocity(point_t<>(0.0, 0.0, 1.0));
    p11->set_angular_velocity(point_t<>(0.0, 0.0, 1.0));
    BOOST_CHECK(fabs(magnitude(s9.rate_of_change_of_normal_of_impact(s11, point_t<>(0.0, 0.0, -1.0)) - zero)) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s11.rate_of_change_of_normal_of_impact(s9, point_t<>(0.0, 0.0,  1.0)) - zero)) < result_tolerance);

    p9->set_angular_velocity(point_t<>(2.0, 0.0, 0.0));
    p11->set_angular_velocity(point_t<>(0.0, 3.0, 0.0));
    BOOST_CHECK(fabs(magnitude(s9.rate_of_change_of_normal_of_impact(s11, point_t<>(0.0, 0.0, -1.0)) - point_t<>(-0.5, -2.5, 0.0))) < result_tolerance);

    p9->set_angular_velocity(point_t<>(0.0, 3.0, 0.0));
    p11->set_angular_velocity(point_t<>(2.0, 0.0, 0.0));
    BOOST_CHECK(fabs(magnitude(s11.rate_of_change_of_normal_of_impact(s9, point_t<>(0.0, 0.0,  1.0)) - point_t<>( 2.5, -0.5, 0.0))) < result_tolerance);

    /* Line versus point */
    p7->set_angular_velocity(point_t<>(1.0, 0.0, 0.0));
    p12->set_angular_velocity(point_t<>(0.0, 0.0, 0.0));
    BOOST_CHECK(fabs(magnitude(s12.rate_of_change_of_normal_of_impact(s7, point_t<>(0.0, 0.0, -1.0)) - zero)) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s7.rate_of_change_of_normal_of_impact(s12, point_t<>(0.0, 0.0,  1.0)) - zero)) < result_tolerance);

    p7->set_angular_velocity(point_t<>(0.0, 0.0, 0.0));
    p12->set_angular_velocity(point_t<>(1.0, 0.0, 0.0));
    BOOST_CHECK(fabs(magnitude(s12.rate_of_change_of_normal_of_impact(s7, point_t<>(0.0, 0.0, -1.0)) - point_t<>(0.0, -1.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s7.rate_of_change_of_normal_of_impact(s12, point_t<>(0.0, 0.0,  1.0)) - point_t<>(0.0,  1.0, 0.0))) < result_tolerance);

    p7->set_angular_velocity(point_t<>(0.0, 1.0, 0.0));
    p12->set_angular_velocity(point_t<>(0.0, 0.0, 0.0));
    BOOST_CHECK(fabs(magnitude(s12.rate_of_change_of_normal_of_impact(s7, point_t<>(0.0, 0.0, -1.0)) - point_t<>( 11.0, 0.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s7.rate_of_change_of_normal_of_impact(s12, point_t<>(0.0, 0.0,  1.0)) - point_t<>(-11.0, 0.0, 0.0))) < result_tolerance);

    /* Point at the other end of the line */
    p8->set_angular_velocity(point_t<>(1.0, 0.0, 0.0));
    p12->set_angular_velocity(point_t<>(0.0, 0.0, 0.0));
    BOOST_CHECK(fabs(magnitude(s12.rate_of_change_of_normal_of_impact(s8, point_t<>(0.0, 0.0, -1.0)) - zero)) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s8.rate_of_change_of_normal_of_impact(s12, point_t<>(0.0, 0.0,  1.0)) - zero)) < result_tolerance);

    p8->set_angular_velocity(point_t<>(0.0, 0.0, 0.0));
    p12->set_angular_velocity(point_t<>(1.0, 0.0, 0.0));
    BOOST_CHECK(fabs(magnitude(s12.rate_of_change_of_normal_of_impact(s8, point_t<>(0.0, 0.0, -1.0)) - point_t<>(0.0, -1.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s8.rate_of_change_of_normal_of_impact(s12, point_t<>(0.0, 0.0,  1.0)) - point_t<>(0.0,  1.0, 0.0))) < result_tolerance);

    p8->set_angular_velocity(point_t<>(0.0, 0.0, 1.0));
    p12->set_angular_velocity(point_t<>(0.0, 0.0, 0.0));
    BOOST_CHECK(fabs(magnitude(s12.rate_of_change_of_normal_of_impact(s8, point_t<>(0.0, 0.0, -1.0)) - point_t<>( 5.0, 0.0, 0.0))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s8.rate_of_change_of_normal_of_impact(s12, point_t<>(0.0, 0.0,  1.0)) - point_t<>(-5.0, 0.0, 0.0))) < result_tolerance);
}

BOOST_AUTO_TEST_CASE ( face_rate_of_change_of_normal_of_impact_tests )
{
    /* Face versus anything */
    /* Static and moving about normal */
    p0->set_angular_velocity(point_t<>(0.0f, 0.0f, 1.0f));
    p3->set_angular_velocity(point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(fabs(magnitude(s0.rate_of_change_of_normal_of_impact(s3, point_t<>(0.0f, 0.0f, -1.0f)))) < result_tolerance);

    p0->set_angular_velocity(point_t<>(0.0f, 0.0f, 0.0f));
    p3->set_angular_velocity(point_t<>(0.0f, 0.0f, 1.0f));
    BOOST_CHECK(fabs(magnitude(s3.rate_of_change_of_normal_of_impact(s0, point_t<>(0.0f, 0.0f,  1.0f)))) < result_tolerance);

    /* Moving */
    p0->set_angular_velocity(point_t<>(0.0f, 1.0f, 0.0f));
    p3->set_angular_velocity(point_t<>(1.0f, 0.0f, 0.0f));
    BOOST_CHECK(fabs(magnitude(s0.rate_of_change_of_normal_of_impact(s3, point_t<>(0.0f, 0.0f, -1.0f)) - point_t<>(-1.0f,  0.0f, 0.0f))) < result_tolerance);

    p0->set_angular_velocity(point_t<>(1.0f, 0.0f, 0.0f));
    p3->set_angular_velocity(point_t<>(0.0f, 1.0f, 0.0f));
    BOOST_CHECK(fabs(magnitude(s3.rate_of_change_of_normal_of_impact(s0, point_t<>(0.0f, 0.0f, 1.0f)) - point_t<>(0.0f, 1.0f, 0.0f))) < result_tolerance);
}

BOOST_AUTO_TEST_CASE ( vertex_center_of_impact_tests )
{
    s0.distance_to_impact(point_t<>(0.0f, 0.0f, -0.1f));
    s3.distance_to_impact(point_t<>(0.0f, 0.0f,  0.1f));
    BOOST_CHECK(fabs(magnitude(s0.center_of_impact(s3, point_t<>(0.0f, 0.0f, -1.0f)) - point_t<>( 1.5f,  1.5f, 1.05f))) < result_tolerance);
    BOOST_REQUIRE(s0.contact_manifold_size() == 1);
    BOOST_CHECK(fabs(magnitude(s0.contact_manifold_point(0) - point_t<>( 1.5f,  1.5f, 1.05f))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(s3.center_of_impact(s0, point_t<>(0.0f, 0.0f, 1.0f)) - point_t<>( 1.5f,  1.5f, 1.05f))) < result_tolerance);
    BOOST_REQUIRE(s3.contact_manifold_size() == 1);
    BOOST_CHECK(fabs(magnitude(s3.contact_manifold_point(0) - point_t<>( 1.5f,  1.5f, 1.05f))) < result_tolerance);
}

BOOST_AUTO_TEST_CASE ( line_center_of_impact_tests )
{
    s0.distance_to_impact(point_t<>(0.0f, 0.0f, -0.1f));
    s4.distance_to_impact(point_t<>(0.0f, 0.0f,  0.1f));
    BOOST_CHECK(fabs(magnitude(s0.center_of_impact(s4, point_t<>(0.0f, 0.0f, -1.0f)) - point_t<>( 1.5f,  0.5f, 1.05f))) < result_tolerance);
    BOOST_REQUIRE(s0.contact_manifold_size() == 2);
    BOOST_CHECK(fabs(magnitude(s0.contact_manifold_point(0) - point_t<>(1.5f,  1.5f, 1.05f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s0.contact_manifold_point(1) - point_t<>(1.5f, -0.5f, 1.05f))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(s4.center_of_impact(s0, point_t<>(0.0f, 0.0f,  1.0f)) - point_t<>( 1.5f,  0.5f, 1.05f))) < result_tolerance);
    BOOST_REQUIRE(s4.contact_manifold_size() == 2);
    BOOST_CHECK(fabs(magnitude(s4.contact_manifold_point(0) - point_t<>(1.5f,  1.5f, 1.05f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s4.contact_manifold_point(1) - point_t<>(1.5f, -0.5f, 1.05f))) < result_tolerance);
}

BOOST_AUTO_TEST_CASE ( face_center_of_impact_tests )
{
    s0.distance_to_impact(point_t<>(0.0f, 0.0f, -0.1f));
    s2.distance_to_impact(point_t<>(0.0f, 0.0f,  0.1f));
    BOOST_CHECK(fabs(magnitude(s0.center_of_impact(s2, point_t<>(0.0f, 0.0f, -1.0f)) - point_t<>( 0.5f,  0.5f, 1.05f))) < result_tolerance);
    BOOST_REQUIRE(s0.contact_manifold_size() == 4);
    BOOST_CHECK(fabs(magnitude(s0.contact_manifold_point(0) - point_t<>( 1.5f,  1.5f, 1.05f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s0.contact_manifold_point(1) - point_t<>(-0.5f,  1.5f, 1.05f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s0.contact_manifold_point(2) - point_t<>(-0.5f, -0.5f, 1.05f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s0.contact_manifold_point(3) - point_t<>( 1.5f, -0.5f, 1.05f))) < result_tolerance);
    
    BOOST_CHECK(fabs(magnitude(s2.center_of_impact(s0, point_t<>(0.0f, 0.0f, 1.0f)) - point_t<>( 0.5f,  0.5f, 1.05f))) < result_tolerance);
    BOOST_REQUIRE(s2.contact_manifold_size() == 4);
    BOOST_CHECK(fabs(magnitude(s2.contact_manifold_point(0) - point_t<>( 1.5f,  1.5f, 1.05f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s2.contact_manifold_point(1) - point_t<>(-0.5f,  1.5f, 1.05f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s2.contact_manifold_point(2) - point_t<>(-0.5f, -0.5f, 1.05f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s2.contact_manifold_point(3) - point_t<>( 1.5f, -0.5f, 1.05f))) < result_tolerance);
}

BOOST_AUTO_TEST_CASE ( off_center_face_center_of_impact_tests )
{
    s0.distance_to_impact(point_t<>(0.0f, 0.0f, -0.1f));
    s5.distance_to_impact(point_t<>(0.0f, 0.0f,  0.1f));
    BOOST_CHECK(fabs(magnitude(s0.center_of_impact(s5, point_t<>(0.0f, 0.0f, -1.0f)) - point_t<>( 0.75f,  0.75f, 1.05f))) < result_tolerance);
    BOOST_REQUIRE(s0.contact_manifold_size() == 4);
    BOOST_CHECK(fabs(magnitude(s0.contact_manifold_point(0) - point_t<>( 0.0f,  1.5f, 1.05f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s0.contact_manifold_point(1) - point_t<>( 0.0f,  0.0f, 1.05f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s0.contact_manifold_point(2) - point_t<>( 1.5f,  0.0f, 1.05f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s0.contact_manifold_point(3) - point_t<>( 1.5f,  1.5f, 1.05f))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(s5.center_of_impact(s0, point_t<>(0.0f, 0.0f, 1.0f)) - point_t<>( 0.75f,  0.75f, 1.05f))) < result_tolerance);
    BOOST_REQUIRE(s5.contact_manifold_size() == 4);
    BOOST_CHECK(fabs(magnitude(s5.contact_manifold_point(0) - point_t<>( 0.0f,  1.5f, 1.05f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s5.contact_manifold_point(1) - point_t<>( 0.0f,  0.0f, 1.05f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s5.contact_manifold_point(2) - point_t<>( 1.5f,  0.0f, 1.05f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s5.contact_manifold_point(3) - point_t<>( 1.5f,  1.5f, 1.05f))) < result_tolerance);
}

BOOST_AUTO_TEST_CASE ( large_face_center_of_impact_tests )
{
    s15.distance_to_impact(point_t<>(0.0f, 0.0f, -0.1f));
    s16.distance_to_impact(point_t<>(0.0f, 0.0f,  0.1f));
    BOOST_CHECK(fabs(magnitude(s15.center_of_impact(s16, point_t<>(0.0f, 0.0f, -1.0f)) - point_t<>( 2.0f,  2.0f, 0.05f))) < result_tolerance);
    BOOST_REQUIRE(s15.contact_manifold_size() == 4);
    BOOST_CHECK(fabs(magnitude(s15.contact_manifold_point(0) - point_t<>( 3.0f, 1.0f, 0.05f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s15.contact_manifold_point(1) - point_t<>( 1.0f, 2.0f, 0.05f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s15.contact_manifold_point(2) - point_t<>( 2.0f, 3.0f, 0.05f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s15.contact_manifold_point(3) - point_t<>( 2.0f, 1.0f, 0.05f))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(s16.center_of_impact(s15, point_t<>(0.0f, 0.0f, 1.0f)) - point_t<>( 2.0f,  2.0f, 0.05f))) < result_tolerance);
    BOOST_REQUIRE(s16.contact_manifold_size() == 4);
    BOOST_CHECK(fabs(magnitude(s16.contact_manifold_point(0) - point_t<>( 3.0f, 1.0f, 0.05f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s16.contact_manifold_point(1) - point_t<>( 1.0f, 2.0f, 0.05f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s16.contact_manifold_point(2) - point_t<>( 2.0f, 3.0f, 0.05f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s16.contact_manifold_point(3) - point_t<>( 2.0f, 1.0f, 0.05f))) < result_tolerance);
}

BOOST_AUTO_TEST_CASE ( vertex_normal_of_impact_tests )
{
    /* Vertex simplices */
    BOOST_CHECK(fabs(magnitude(s0.normal_of_impact(s1) - normalise(point_t<>(-1.0f,  1.0f, -0.5f)))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s1.normal_of_impact(s0) - normalise(point_t<>( 1.0f, -1.0f,  0.5f)))) < result_tolerance);
}

BOOST_AUTO_TEST_CASE ( line_normal_of_impact_tests )
{
    /* Parallel line */
    s0.add(1);  /* 0, 1 */
    s2.add(1);  /* 0, 1 */
    BOOST_CHECK(fabs(magnitude(s0.normal_of_impact(s2) - point_t<>(0.0f, 0.0f, -1.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s2.normal_of_impact(s0) - point_t<>(0.0f, 0.0f,  1.0f))) < result_tolerance);

    /* Anti-parallel line */
    const int retain0[] = { 1, 0 };
    s2.retain_vertices(retain0, 2); /* 1, 0 */
    BOOST_CHECK(fabs(magnitude(s0.normal_of_impact(s2) - point_t<>(0.0f, 0.0f, -1.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s2.normal_of_impact(s0) - point_t<>(0.0f, 0.0f,  1.0f))) < result_tolerance);

    /* Skew lines */
    s0.add(2);  /* 0, 1, 2 */
    const int retain1[] = { 0, 2 };
    s0.retain_vertices(retain1, 2);   /* 0, 2 */

    s2.add(3);  /* 1, 0, 3 */
    const int retain2[] = { 0, 2 };
    s2.retain_vertices(retain2, 2); /* 1, 3 */
    BOOST_CHECK(fabs(magnitude(s0.normal_of_impact(s2) - point_t<>(0.0f, 0.0f, -1.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s2.normal_of_impact(s0) - point_t<>(0.0f, 0.0f,  1.0f))) < result_tolerance);

    /* Anti-skew line */
    const int retain3[] = { 1, 0 };
    s2.retain_vertices(retain3, 2); /* 3, 1 */
    BOOST_CHECK(fabs(magnitude(s0.normal_of_impact(s2) - point_t<>(0.0f, 0.0f, -1.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s2.normal_of_impact(s0) - point_t<>(0.0f, 0.0f,  1.0f))) < result_tolerance);

    /* Line versus point */
    s0.add(1);  /* 0, 2, 1 */
    s0.retain_vertices(retain1, 2);   /* 0, 1 */
    s2.add(0);  /* 3, 1, 0 */
    s2.add(0);  /* 3, 1, 0, 0 */
    const int retain4[] = { 2, 3 };
    s2.retain_vertices(retain4, 2); /* 0, 0 */
    BOOST_CHECK(fabs(magnitude(s0.normal_of_impact(s2) - point_t<>(0.0f, 0.0f, -1.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s2.normal_of_impact(s0) - point_t<>(0.0f, 0.0f,  1.0f))) < result_tolerance);

    /* Point at the other end of the line */
    s2.add(1);
    const int retain5[] = { 1, 1 };
    s2.retain_vertices(retain5, 2); /* 1, 1 */
    BOOST_CHECK(fabs(magnitude(s0.normal_of_impact(s2) - point_t<>(0.0f, 0.0f, -1.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s2.normal_of_impact(s0) - point_t<>(0.0f, 0.0f,  1.0f))) < result_tolerance);
}

BOOST_AUTO_TEST_CASE ( face_normal_of_impact_tests )
{
    /* Face versus anything */
    s0.add(1);
    s0.add(2);
    s3.add(0);
    s3.add(0);
    BOOST_CHECK(fabs(magnitude(s0.normal_of_impact(s3) - point_t<>(0.0f, 0.0f, -1.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s3.normal_of_impact(s0) - point_t<>(0.0f, 0.0f,  1.0f))) < result_tolerance);

    /* Cause flip because of general plane direction */
    s13.add(1);
    s13.add(2);
    s14.add(0);
    s14.add(0);
    BOOST_CHECK(fabs(magnitude(s13.normal_of_impact(s14) - point_t<>(0.0f, 0.0f, -1.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(s14.normal_of_impact(s13) - point_t<>(0.0f, 0.0f,  1.0f))) < result_tolerance);
}

BOOST_AUTO_TEST_SUITE_END()
}; /* namespace test */
}; /* namespace raptor_physics */
