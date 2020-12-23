#ifdef STAND_ALONE
#define BOOST_TEST_MODULE voxel_set test
#endif /* #ifdef STAND_ALONE */


/* Standard headers */

/* Boost headers */
#include "boost/noncopyable.hpp"
#include "boost/test/unit_test.hpp"

/* Convex Decomposition headers */
#include "voxel_set.h"


namespace raptor_convex_decomposition
{
namespace test
{
/* Test data */
struct voxel_set_fixture : private boost::noncopyable
{
    voxel_set_fixture() :
        empty(   {  }, point_t<>( 3.5f, -2.7f, 1.6f), 1.0f),
        cube(    { voxel(point_ti<>(3, 4, 2)) }, point_t<>( 3.5f, -2.7f, 1.6f), 1.0f),
        cube2_5x({ voxel(point_ti<>(3, 4, 2)) }, point_t<>(-7.8f,  3.9f, 4.6f), 2.5f),
        cube3x(  { voxel(point_ti<>(3, 4, 2)) }, point_t<>(-7.8f,  3.9f, 4.6f), 3.0f),
        diagonal(
        {
            voxel(point_ti<>(3, 4,   2)),
            voxel(point_ti<>(4, 6,   4)),
            voxel(point_ti<>(5, 8,   8)),
            voxel(point_ti<>(6, 10, 16)),
            voxel(point_ti<>(7, 12, 32)),
            voxel(point_ti<>(8, 14, 64))
        }, point_t<>(-7.8f,  3.9f, 4.6f), 2.0f),
        row(
        {
            voxel(point_ti<>(0, 0, 0)),
            voxel(point_ti<>(1, 0, 0)),
            voxel(point_ti<>(2, 0, 0)),
            voxel(point_ti<>(3, 0, 0)),
            voxel(point_ti<>(4, 0, 0)),
            voxel(point_ti<>(5, 0, 0)),
            voxel(point_ti<>(6, 0, 0)),
            voxel(point_ti<>(7, 0, 0))
        }, point_t<>(1.0f, 2.0f, 3.0f), 1.0f),
        on_surface(
        {
            voxel(point_ti<>(0, 0, 0), voxel_value_t::primitive_on_surface),
            voxel(point_ti<>(1, 0, 0), voxel_value_t::primitive_on_surface),
            voxel(point_ti<>(2, 0, 0), voxel_value_t::primitive_undefined),
            voxel(point_ti<>(3, 0, 0), voxel_value_t::primitive_outside_surface),
            voxel(point_ti<>(4, 0, 0), voxel_value_t::primitive_inside_surface),
            voxel(point_ti<>(5, 0, 0), voxel_value_t::primitive_on_surface),
            voxel(point_ti<>(6, 0, 0), voxel_value_t::primitive_undefined),
            voxel(point_ti<>(7, 0, 0), voxel_value_t::primitive_undefined)
        }, point_t<>(1.0f, 2.0f, 3.0f), 1.0f)
    {  }

    voxel_set   empty;
    voxel_set   cube;
    voxel_set   cube2_5x;
    voxel_set   cube3x;
    voxel_set   diagonal;
    voxel_set   row;
    voxel_set   on_surface;
};

BOOST_FIXTURE_TEST_SUITE( voxel_set_tests, voxel_set_fixture )

const float result_tolerance = 0.0005f;


/* Test Ctor */
BOOST_AUTO_TEST_CASE( ctor_test )
{
    BOOST_CHECK(cube.get_min_bb()                           == point_t<>(3.5f, -2.7f, 1.6f));
    BOOST_CHECK(cube.get_barycenter()                       == point_ti<>(0, 0, 0));
    BOOST_CHECK(cube.compute_volume()                       == 1.0f);
    BOOST_CHECK(cube.max_volume_error()                     == 0.0f);
    BOOST_CHECK(cube.get_scale()                            == 1.0f);
    BOOST_CHECK(cube.get_unit_volume()                      == 1.0f);
    BOOST_CHECK(cube.number_of_primitives()                 == 1);
    BOOST_CHECK(cube.number_of_primitives_on_surface()      == 0);
    BOOST_CHECK(cube.number_of_primitives_inside()          == 0);

    BOOST_CHECK(cube3x.get_min_bb()                         == point_t<>(-7.8f, 3.9f, 4.6f));
    BOOST_CHECK(cube3x.get_barycenter()                     == point_ti<>(0, 0, 0));
    BOOST_CHECK(cube3x.compute_volume()                     == 27.0f);
    BOOST_CHECK(cube3x.max_volume_error()                   == 0.0f);
    BOOST_CHECK(cube3x.get_scale()                          == 3.0f);
    BOOST_CHECK(cube3x.get_unit_volume()                    == 27.0f);
    BOOST_CHECK(cube3x.number_of_primitives()               == 1);
    BOOST_CHECK(cube3x.number_of_primitives_on_surface()    == 0);
    BOOST_CHECK(cube3x.number_of_primitives_inside()        == 0);

    BOOST_CHECK(diagonal.get_min_bb()                       == point_t<>(-7.8f, 3.9f, 4.6f));
    BOOST_CHECK(diagonal.get_barycenter()                   == point_ti<>(0, 0, 0));
    BOOST_CHECK(diagonal.compute_volume()                   == 48.0f);
    BOOST_CHECK(diagonal.max_volume_error()                 == 0.0f);
    BOOST_CHECK(diagonal.get_scale()                        == 2.0f);
    BOOST_CHECK(diagonal.get_unit_volume()                  == 8.0f);
    BOOST_CHECK(diagonal.number_of_primitives()             == 6);
    BOOST_CHECK(diagonal.number_of_primitives_on_surface()  == 0);
    BOOST_CHECK(diagonal.number_of_primitives_inside()      == 0);
}

/* Test get voxels */
BOOST_AUTO_TEST_CASE( const_get_voxels_test )
{
    const auto *const const_diagonal = &diagonal;
    const auto &v = const_diagonal->get_voxels();
    BOOST_REQUIRE(v.size() == 6);
    BOOST_CHECK(v[0].coord == point_ti<>(3, 4,   2));
    BOOST_CHECK(v[1].coord == point_ti<>(4, 6,   4));
    BOOST_CHECK(v[2].coord == point_ti<>(5, 8,   8));
    BOOST_CHECK(v[3].coord == point_ti<>(6, 10, 16));
    BOOST_CHECK(v[4].coord == point_ti<>(7, 12, 32));
    BOOST_CHECK(v[5].coord == point_ti<>(8, 14, 64));
}

BOOST_AUTO_TEST_CASE( get_voxels_test )
{
    auto &v =  diagonal.get_voxels();
    BOOST_REQUIRE(v.size() == 6);
    BOOST_CHECK(v[0].coord == point_ti<>(3, 4,   2));
    BOOST_CHECK(v[1].coord == point_ti<>(4, 6,   4));
    BOOST_CHECK(v[2].coord == point_ti<>(5, 8,   8));
    BOOST_CHECK(v[3].coord == point_ti<>(6, 10, 16));
    BOOST_CHECK(v[4].coord == point_ti<>(7, 12, 32));
    BOOST_CHECK(v[5].coord == point_ti<>(8, 14, 64));
}

/* Test set scale */
BOOST_AUTO_TEST_CASE( set_scale_test )
{
    cube.set_scale(5.0f);
    BOOST_CHECK(cube.get_scale()        == 5.0f);

    cube3x.set_scale(5.1f);
    BOOST_CHECK(cube3x.get_scale()      == 5.1f);

    diagonal.set_scale(5.2f);
    BOOST_CHECK(diagonal.get_scale()    == 5.2f);
}

/* Test set min bb */
BOOST_AUTO_TEST_CASE( set_min_bb_test )
{
    cube.set_min_bb(point_t<>(-9.0f, 1.4f, 5.0f));
    BOOST_CHECK(cube.get_min_bb()       == point_t<>(-9.0f, 1.4f, 5.0f));

    cube3x.set_min_bb(point_t<>(-9.4f, 2.4f, 4.0f));
    BOOST_CHECK(cube3x.get_min_bb()     == point_t<>(-9.4f, 2.4f, 4.0f));

    diagonal.set_min_bb(point_t<>(-9.8f, 6.4f, 2.0f));
    BOOST_CHECK(diagonal.get_min_bb()   == point_t<>(-9.8f, 6.4f, 2.0f));
}

/* Test reserve */
BOOST_AUTO_TEST_CASE( reserve_test )
{
    cube.reserve(1);
    cube3x.reserve(2);
    diagonal.reserve(3);
}

/* Test get point */
BOOST_AUTO_TEST_CASE( get_point_point_test )
{
    BOOST_CHECK(fabs(magnitude(cube3x.get_point(point_t<>(0.0f,  0.0f, 0.0f)) - point_t<>(-7.8f,  3.9f,  4.6f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(cube3x.get_point(point_t<>(1.0f,  1.0f, 1.0f)) - point_t<>(-4.8f,  6.9f,  7.6f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(cube3x.get_point(point_t<>(1.7f, -3.6f, 8.6f)) - point_t<>(-2.7f, -6.9f, 30.4f))) < result_tolerance);
}

BOOST_AUTO_TEST_CASE( get_point_voxel_test )
{
    BOOST_CHECK(fabs(magnitude(cube3x.get_point(voxel(point_ti<>(0,  0, 0))) - point_t<>(-7.8f,  3.9f,  4.6f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(cube3x.get_point(voxel(point_ti<>(1,  1, 1))) - point_t<>(-4.8f,  6.9f,  7.6f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(cube3x.get_point(voxel(point_ti<>(2, -4, 9))) - point_t<>(-1.8f, -8.1f, 31.6f))) < result_tolerance);

    BOOST_CHECK(fabs(magnitude(cube2_5x.get_point(voxel(point_ti<>(0,  0, 0))) - point_t<>(-7.8f,  3.9f,  4.6f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(cube2_5x.get_point(voxel(point_ti<>(1,  1, 1))) - point_t<>(-5.3f,  6.4f,  7.1f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(cube2_5x.get_point(voxel(point_ti<>(2, -4, 9))) - point_t<>(-2.8f, -6.1f, 27.1f))) < result_tolerance);
}

BOOST_AUTO_TEST_CASE( get_points_test )
{
    point_t<> pts[8];
    cube.get_points(voxel(point_ti<>(0,  0, 0)), &pts[0]);
    BOOST_CHECK(pts[0] == point_t<>(3.0f, -3.2f,  1.1f));
    BOOST_CHECK(pts[1] == point_t<>(4.0f, -3.2f,  1.1f));
    BOOST_CHECK(pts[2] == point_t<>(4.0f, -2.2f,  1.1f));
    BOOST_CHECK(pts[3] == point_t<>(3.0f, -2.2f,  1.1f));
    BOOST_CHECK(pts[4] == point_t<>(3.0f, -3.2f,  2.1f));
    BOOST_CHECK(pts[5] == point_t<>(4.0f, -3.2f,  2.1f));
    BOOST_CHECK(pts[6] == point_t<>(4.0f, -2.2f,  2.1f));
    BOOST_CHECK(pts[7] == point_t<>(3.0f, -2.2f,  2.1f));

    cube.get_points(voxel(point_ti<>(1,  1, 1)), &pts[0]);
    BOOST_CHECK(pts[0] == point_t<>(4.0f, -2.2f,  2.1f));
    BOOST_CHECK(pts[1] == point_t<>(5.0f, -2.2f,  2.1f));
    BOOST_CHECK(pts[2] == point_t<>(5.0f, -1.2f,  2.1f));
    BOOST_CHECK(pts[3] == point_t<>(4.0f, -1.2f,  2.1f));
    BOOST_CHECK(pts[4] == point_t<>(4.0f, -2.2f,  3.1f));
    BOOST_CHECK(pts[5] == point_t<>(5.0f, -2.2f,  3.1f));
    BOOST_CHECK(pts[6] == point_t<>(5.0f, -1.2f,  3.1f));
    BOOST_CHECK(pts[7] == point_t<>(4.0f, -1.2f,  3.1f));

    cube.get_points(voxel(point_ti<>(2, -4, 9)), &pts[0]);
    BOOST_CHECK(pts[0] == point_t<>(5.0f, -7.2f, 10.1f));
    BOOST_CHECK(pts[1] == point_t<>(6.0f, -7.2f, 10.1f));
    BOOST_CHECK(pts[2] == point_t<>(6.0f, -6.2f, 10.1f));
    BOOST_CHECK(pts[3] == point_t<>(5.0f, -6.2f, 10.1f));
    BOOST_CHECK(pts[4] == point_t<>(5.0f, -7.2f, 11.1f));
    BOOST_CHECK(pts[5] == point_t<>(6.0f, -7.2f, 11.1f));
    BOOST_CHECK(pts[6] == point_t<>(6.0f, -6.2f, 11.1f));
    BOOST_CHECK(pts[7] == point_t<>(5.0f, -6.2f, 11.1f));

    cube2_5x.get_points(voxel(point_ti<>(0,  0, 0)), &pts[0]);
    BOOST_CHECK(pts[0] == point_t<>(-9.05f, 2.65f, 3.35f));
    BOOST_CHECK(pts[1] == point_t<>(-6.55f, 2.65f, 3.35f));
    BOOST_CHECK(pts[2] == point_t<>(-6.55f, 5.15f, 3.35f));
    BOOST_CHECK(pts[3] == point_t<>(-9.05f, 5.15f, 3.35f));
    BOOST_CHECK(pts[4] == point_t<>(-9.05f, 2.65f, 5.85f));
    BOOST_CHECK(pts[5] == point_t<>(-6.55f, 2.65f, 5.85f));
    BOOST_CHECK(pts[6] == point_t<>(-6.55f, 5.15f, 5.85f));
    BOOST_CHECK(pts[7] == point_t<>(-9.05f, 5.15f, 5.85f));

    cube2_5x.get_points(voxel(point_ti<>(1,  1, 1)), &pts[0]);
    BOOST_CHECK(pts[0] == point_t<>(-6.55f, 5.15f, 5.85f));
    BOOST_CHECK(pts[1] == point_t<>(-4.05f, 5.15f, 5.85f));
    BOOST_CHECK(pts[2] == point_t<>(-4.05f, 7.65f, 5.85f));
    BOOST_CHECK(pts[3] == point_t<>(-6.55f, 7.65f, 5.85f));
    BOOST_CHECK(pts[4] == point_t<>(-6.55f, 5.15f, 8.35f));
    BOOST_CHECK(pts[5] == point_t<>(-4.05f, 5.15f, 8.35f));
    BOOST_CHECK(pts[6] == point_t<>(-4.05f, 7.65f, 8.35f));
    BOOST_CHECK(pts[7] == point_t<>(-6.55f, 7.65f, 8.35f));

    cube2_5x.get_points(voxel(point_ti<>(2, -4, 9)), &pts[0]);
    BOOST_CHECK(fabs(magnitude(pts[0] - point_t<>(-4.05f, -7.35f, 25.85f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(pts[1] - point_t<>(-1.55f, -7.35f, 25.85f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(pts[2] - point_t<>(-1.55f, -4.85f, 25.85f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(pts[3] - point_t<>(-4.05f, -4.85f, 25.85f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(pts[4] - point_t<>(-4.05f, -7.35f, 28.35f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(pts[5] - point_t<>(-1.55f, -7.35f, 28.35f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(pts[6] - point_t<>(-1.55f, -4.85f, 28.35f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(pts[7] - point_t<>(-4.05f, -4.85f, 28.35f))) < result_tolerance);
}

/* Test compute bounding box */
BOOST_AUTO_TEST_CASE( compute_bounding_box_test )
{
    empty.compute_bounding_box();
    BOOST_CHECK(empty.get_min_bb_voxels()   == point_ti<>(0, 0, 0));
    BOOST_CHECK(empty.get_max_bb_voxels()   == point_ti<>(0, 0, 0));
    BOOST_CHECK(empty.get_barycenter()      == point_ti<>(0, 0, 0));

    cube.compute_bounding_box();
    BOOST_CHECK(cube.get_min_bb_voxels()    == point_ti<>(3, 4, 2));
    BOOST_CHECK(cube.get_max_bb_voxels()    == point_ti<>(3, 4, 2));
    BOOST_CHECK(cube.get_barycenter()       == point_ti<>(3, 4, 2));

    diagonal.compute_bounding_box();
    BOOST_CHECK(diagonal.get_min_bb_voxels()    == point_ti<>(3,  4,  2));
    BOOST_CHECK(diagonal.get_max_bb_voxels()    == point_ti<>(8, 14, 64));
    BOOST_CHECK(diagonal.get_barycenter()       == point_ti<>(6,  9, 21));
}

/* Test intersect */
BOOST_AUTO_TEST_CASE( intersect_clean_test )
{
    point_t<> pts[8];
    std::vector<point_t<>> pos_pts;
    std::vector<point_t<>> neg_pts;
    diagonal.intersect(plane(point_t<>(1.0f, 0.0f, 0.0f), 3.2f, axis_t::x_axis), &pos_pts, &neg_pts, 1);

    BOOST_REQUIRE(pos_pts.size() == 24);
    diagonal.get_points(voxel(point_ti<>(6, 10, 16)), &pts[0]);
    BOOST_CHECK(pos_pts[0] == pts[0]);
    BOOST_CHECK(pos_pts[1] == pts[1]);
    BOOST_CHECK(pos_pts[2] == pts[2]);
    BOOST_CHECK(pos_pts[3] == pts[3]);
    BOOST_CHECK(pos_pts[4] == pts[4]);
    BOOST_CHECK(pos_pts[5] == pts[5]);
    BOOST_CHECK(pos_pts[6] == pts[6]);
    BOOST_CHECK(pos_pts[7] == pts[7]);
    diagonal.get_points(voxel(point_ti<>(7, 12, 32)), &pts[0]);
    BOOST_CHECK(pos_pts[ 8] == pts[0]);
    BOOST_CHECK(pos_pts[ 9] == pts[1]);
    BOOST_CHECK(pos_pts[10] == pts[2]);
    BOOST_CHECK(pos_pts[11] == pts[3]);
    BOOST_CHECK(pos_pts[12] == pts[4]);
    BOOST_CHECK(pos_pts[13] == pts[5]);
    BOOST_CHECK(pos_pts[14] == pts[6]);
    BOOST_CHECK(pos_pts[15] == pts[7]);
    diagonal.get_points(voxel(point_ti<>(8, 14, 64)), &pts[0]);
    BOOST_CHECK(pos_pts[16] == pts[0]);
    BOOST_CHECK(pos_pts[17] == pts[1]);
    BOOST_CHECK(pos_pts[18] == pts[2]);
    BOOST_CHECK(pos_pts[19] == pts[3]);
    BOOST_CHECK(pos_pts[20] == pts[4]);
    BOOST_CHECK(pos_pts[21] == pts[5]);
    BOOST_CHECK(pos_pts[22] == pts[6]);
    BOOST_CHECK(pos_pts[23] == pts[7]);

    BOOST_REQUIRE(neg_pts.size() == 24);
    diagonal.get_points(voxel(point_ti<>(3, 4,   2)), &pts[0]);
    BOOST_CHECK(neg_pts[0] == pts[0]);
    BOOST_CHECK(neg_pts[1] == pts[1]);
    BOOST_CHECK(neg_pts[2] == pts[2]);
    BOOST_CHECK(neg_pts[3] == pts[3]);
    BOOST_CHECK(neg_pts[4] == pts[4]);
    BOOST_CHECK(neg_pts[5] == pts[5]);
    BOOST_CHECK(neg_pts[6] == pts[6]);
    BOOST_CHECK(neg_pts[7] == pts[7]);
    diagonal.get_points(voxel(point_ti<>(4, 6,   4)), &pts[0]);
    BOOST_CHECK(neg_pts[ 8] == pts[0]);
    BOOST_CHECK(neg_pts[ 9] == pts[1]);
    BOOST_CHECK(neg_pts[10] == pts[2]);
    BOOST_CHECK(neg_pts[11] == pts[3]);
    BOOST_CHECK(neg_pts[12] == pts[4]);
    BOOST_CHECK(neg_pts[13] == pts[5]);
    BOOST_CHECK(neg_pts[14] == pts[6]);
    BOOST_CHECK(neg_pts[15] == pts[7]);
    diagonal.get_points(voxel(point_ti<>(5, 8,   8)), &pts[0]);
    BOOST_CHECK(neg_pts[16] == pts[0]);
    BOOST_CHECK(neg_pts[17] == pts[1]);
    BOOST_CHECK(neg_pts[18] == pts[2]);
    BOOST_CHECK(neg_pts[19] == pts[3]);
    BOOST_CHECK(neg_pts[20] == pts[4]);
    BOOST_CHECK(neg_pts[21] == pts[5]);
    BOOST_CHECK(neg_pts[22] == pts[6]);
    BOOST_CHECK(neg_pts[23] == pts[7]);
}

BOOST_AUTO_TEST_CASE( intersect_pos_sampling_test )
{
    point_t<> pts[8];
    std::vector<point_t<>> pos_pts;
    std::vector<point_t<>> neg_pts;
    row.intersect(plane(point_t<>(0.0f, 1.0f, 0.0f), 0.9f, axis_t::x_axis), &pos_pts, &neg_pts, 3);

    BOOST_REQUIRE(pos_pts.size() == 16);
    row.get_points(voxel(point_ti<>(2, 0, 0)), &pts[0]);
    BOOST_CHECK(pos_pts[0] == pts[0]);
    BOOST_CHECK(pos_pts[1] == pts[1]);
    BOOST_CHECK(pos_pts[2] == pts[2]);
    BOOST_CHECK(pos_pts[3] == pts[3]);
    BOOST_CHECK(pos_pts[4] == pts[4]);
    BOOST_CHECK(pos_pts[5] == pts[5]);
    BOOST_CHECK(pos_pts[6] == pts[6]);
    BOOST_CHECK(pos_pts[7] == pts[7]);
    row.get_points(voxel(point_ti<>(5, 0, 0)), &pts[0]);
    BOOST_CHECK(pos_pts[ 8] == pts[0]);
    BOOST_CHECK(pos_pts[ 9] == pts[1]);
    BOOST_CHECK(pos_pts[10] == pts[2]);
    BOOST_CHECK(pos_pts[11] == pts[3]);
    BOOST_CHECK(pos_pts[12] == pts[4]);
    BOOST_CHECK(pos_pts[13] == pts[5]);
    BOOST_CHECK(pos_pts[14] == pts[6]);
    BOOST_CHECK(pos_pts[15] == pts[7]);

    BOOST_REQUIRE(neg_pts.size() == 0);
}

BOOST_AUTO_TEST_CASE( intersect_neg_sampling_test )
{
    point_t<> pts[8];
    std::vector<point_t<>> pos_pts;
    std::vector<point_t<>> neg_pts;
    row.intersect(plane(point_t<>(0.0f, -1.0f, 0.0f), 3.1f, axis_t::x_axis), &pos_pts, &neg_pts, 3);

    BOOST_REQUIRE(pos_pts.size() == 0);

    BOOST_REQUIRE(neg_pts.size() == 16);
    row.get_points(voxel(point_ti<>(2, 0, 0)), &pts[0]);
    BOOST_CHECK(neg_pts[0] == pts[0]);
    BOOST_CHECK(neg_pts[1] == pts[1]);
    BOOST_CHECK(neg_pts[2] == pts[2]);
    BOOST_CHECK(neg_pts[3] == pts[3]);
    BOOST_CHECK(neg_pts[4] == pts[4]);
    BOOST_CHECK(neg_pts[5] == pts[5]);
    BOOST_CHECK(neg_pts[6] == pts[6]);
    BOOST_CHECK(neg_pts[7] == pts[7]);
    row.get_points(voxel(point_ti<>(5, 0, 0)), &pts[0]);
    BOOST_CHECK(neg_pts[ 8] == pts[0]);
    BOOST_CHECK(neg_pts[ 9] == pts[1]);
    BOOST_CHECK(neg_pts[10] == pts[2]);
    BOOST_CHECK(neg_pts[11] == pts[3]);
    BOOST_CHECK(neg_pts[12] == pts[4]);
    BOOST_CHECK(neg_pts[13] == pts[5]);
    BOOST_CHECK(neg_pts[14] == pts[6]);
    BOOST_CHECK(neg_pts[15] == pts[7]);
}

/* Test compute principal axes */
BOOST_AUTO_TEST_CASE( compute_principal_axes_test )
{
    cube.compute_bounding_box();
    cube.compute_principal_axes();
    BOOST_CHECK(cube.eigen_value(axis_t::x_axis) == 0.0f);
    BOOST_CHECK(cube.eigen_value(axis_t::y_axis) == 0.0f);
    BOOST_CHECK(cube.eigen_value(axis_t::z_axis) == 0.0f);

    row.compute_bounding_box();
    row.compute_principal_axes();
    BOOST_CHECK(row.eigen_value(axis_t::x_axis) == 5.5f);
    BOOST_CHECK(row.eigen_value(axis_t::y_axis) == 0.0f);
    BOOST_CHECK(row.eigen_value(axis_t::z_axis) == 0.0f);

    diagonal.compute_bounding_box();
    diagonal.compute_principal_axes();
    BOOST_CHECK(fabs(diagonal.eigen_value(axis_t::x_axis) -   0.195948f)    < result_tolerance);
    BOOST_CHECK(fabs(diagonal.eigen_value(axis_t::y_axis) -   2.60634f)     < result_tolerance);
    BOOST_CHECK(fabs(diagonal.eigen_value(axis_t::z_axis) - 481.031f)       < result_tolerance);
}

/* Test select on surface */
BOOST_AUTO_TEST_CASE( select_on_surface_empty_test )
{
    std::unique_ptr<voxel_set> on_surf(empty.select_on_surface());
    BOOST_CHECK(on_surf == nullptr);
}

BOOST_AUTO_TEST_CASE( select_on_surface_test )
{
    std::unique_ptr<voxel_set> on_surf(on_surface.select_on_surface());
    BOOST_REQUIRE(on_surf != nullptr);

    const auto &v = on_surf->get_voxels();
    BOOST_REQUIRE(v.size() == 3);
    BOOST_CHECK(v[0].coord == point_ti<>(0, 0, 0));
    BOOST_CHECK(v[1].coord == point_ti<>(1, 0, 0));
    BOOST_CHECK(v[2].coord == point_ti<>(5, 0, 0));

    BOOST_CHECK(v[0].loc == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(v[1].loc == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(v[2].loc == voxel_value_t::primitive_on_surface);
}

/* Test cut */
BOOST_AUTO_TEST_CASE( cut_diagonal_test )
{
    primitive_set *pos_part_p = nullptr;
    primitive_set *neg_part_p = nullptr;
    diagonal.cut(plane(point_t<>(1.0f, 0.0f, 0.0f), 3.1f, axis_t::x_axis), &pos_part_p, &neg_part_p);
    voxel_set* pos_part = static_cast<voxel_set *>(pos_part_p);
    voxel_set* neg_part = static_cast<voxel_set *>(neg_part_p);

    BOOST_CHECK(pos_part->number_of_primitives_on_surface()  == 1);
    BOOST_CHECK(pos_part->number_of_primitives_inside()      == 2);
    BOOST_CHECK(pos_part->max_volume_error()                 == 8.0f);
    BOOST_REQUIRE(pos_part->number_of_primitives()           == 3);
    BOOST_CHECK(pos_part->get_voxels()[0].coord == point_ti<>(6, 10, 16));
    BOOST_CHECK(pos_part->get_voxels()[1].coord == point_ti<>(7, 12, 32));
    BOOST_CHECK(pos_part->get_voxels()[2].coord == point_ti<>(8, 14, 64));


    BOOST_CHECK(neg_part->number_of_primitives_on_surface()  == 1);
    BOOST_CHECK(neg_part->number_of_primitives_inside()      == 2);
    BOOST_CHECK(neg_part->max_volume_error()                 == 8.0f);
    BOOST_REQUIRE(neg_part->number_of_primitives()           == 3);
    BOOST_CHECK(neg_part->get_voxels()[0].coord == point_ti<>(3, 4,   2));
    BOOST_CHECK(neg_part->get_voxels()[1].coord == point_ti<>(4, 6,   4));
    BOOST_CHECK(neg_part->get_voxels()[2].coord == point_ti<>(5, 8,   8));

    delete pos_part;
    delete neg_part;
}

BOOST_AUTO_TEST_CASE( cut_on_surface_test )
{
    primitive_set *pos_part_p = nullptr;
    primitive_set *neg_part_p = nullptr;
    on_surface.cut(plane(point_t<>(1.0f, 0.0f, 0.0f), 1.25f, axis_t::x_axis), &pos_part_p, &neg_part_p);
    voxel_set* pos_part = static_cast<voxel_set *>(pos_part_p);
    voxel_set* neg_part = static_cast<voxel_set *>(neg_part_p);

    BOOST_CHECK(pos_part->number_of_primitives_on_surface()  == 2);
    BOOST_CHECK(pos_part->number_of_primitives_inside()      == 5);
    BOOST_CHECK(pos_part->max_volume_error()                 == 2.0f);
    BOOST_REQUIRE(pos_part->number_of_primitives()           == 7);
    BOOST_CHECK(pos_part->get_voxels()[0].coord == point_ti<>(1, 0, 0));
    BOOST_CHECK(pos_part->get_voxels()[1].coord == point_ti<>(2, 0, 0));
    BOOST_CHECK(pos_part->get_voxels()[2].coord == point_ti<>(3, 0, 0));
    BOOST_CHECK(pos_part->get_voxels()[3].coord == point_ti<>(4, 0, 0));
    BOOST_CHECK(pos_part->get_voxels()[4].coord == point_ti<>(5, 0, 0));
    BOOST_CHECK(pos_part->get_voxels()[5].coord == point_ti<>(6, 0, 0));
    BOOST_CHECK(pos_part->get_voxels()[6].coord == point_ti<>(7, 0, 0));


    BOOST_CHECK(neg_part->number_of_primitives_on_surface()  == 1);
    BOOST_CHECK(neg_part->number_of_primitives_inside()      == 0);
    BOOST_CHECK(neg_part->max_volume_error()                 == 1.0f);
    BOOST_REQUIRE(neg_part->number_of_primitives()           == 1);
    BOOST_CHECK(neg_part->get_voxels()[0].coord == point_ti<>(0, 0, 0));

    delete pos_part;
    delete neg_part;
}

/* Test compute cut volumes */
BOOST_AUTO_TEST_CASE( cut_volumes_test )
{
    float pos_vol;
    float neg_vol;
    diagonal.compute_cut_volumes(plane(point_t<>(1.0f, 0.0f, 0.0f), 3.1f, axis_t::x_axis), &pos_vol, &neg_vol);
    BOOST_CHECK(pos_vol == 24.0f);
    BOOST_CHECK(neg_vol == 24.0f);

    on_surface.compute_cut_volumes(plane(point_t<>(1.0f, 0.0f, 0.0f), 1.25f, axis_t::x_axis), &pos_vol, &neg_vol);
    BOOST_CHECK(pos_vol == 7.0f);
    BOOST_CHECK(neg_vol == 1.0f);

    empty.compute_cut_volumes(plane(point_t<>(1.0f, 0.0f, 0.0f), 2.25f, axis_t::x_axis), &pos_vol, &neg_vol);
    BOOST_CHECK(pos_vol == 0.0f);
    BOOST_CHECK(neg_vol == 0.0f);
}

/* Test convert */
BOOST_AUTO_TEST_CASE( convert_test )
{
    convex_mesh mesh;
    cube.convert(&mesh, voxel_value_t::primitive_undefined);
    BOOST_CHECK(mesh.number_of_points()     == 8);
    BOOST_CHECK(mesh.number_of_triangles()  == 12);
    BOOST_CHECK(mesh.volume()               == 1.0f);

    cube3x.convert(&mesh, voxel_value_t::primitive_undefined);
    BOOST_CHECK(mesh.number_of_points()     == 16);
    BOOST_CHECK(mesh.number_of_triangles()  == 24);
    BOOST_CHECK(mesh.volume()               == 28.0f);

    on_surface.convert(&mesh, voxel_value_t::primitive_on_surface);
    BOOST_CHECK(mesh.number_of_points()     == 40);
    BOOST_CHECK(mesh.number_of_triangles()  == 60);
    BOOST_CHECK(fabs(mesh.volume() - 31.0f) < result_tolerance);
}

/* Test compute axes aligned clipping planes */
BOOST_AUTO_TEST_CASE( compute_axes_aligned_clipping_planes_test )
{
    std::vector<plane> planes;
    diagonal.compute_bounding_box();
    diagonal.compute_axes_aligned_clipping_planes(&planes, 9);

    BOOST_REQUIRE(planes.size() == 10);
    BOOST_CHECK(planes[0].n             == point_t<>(1.0f, 0.0f, 0.0f));
    BOOST_CHECK_CLOSE(planes[0].p, -0.8f, result_tolerance);
    BOOST_CHECK(planes[0].major_axis    == axis_t::x_axis);
    BOOST_CHECK(planes[1].n             == point_t<>(0.0f, 1.0f, 0.0f));
    BOOST_CHECK_CLOSE(planes[1].p, 12.9f, result_tolerance);
    BOOST_CHECK(planes[1].major_axis    == axis_t::y_axis);
    BOOST_CHECK(planes[2].n             == point_t<>(0.0f, 1.0f, 0.0f));
    BOOST_CHECK_CLOSE(planes[2].p, 30.9f, result_tolerance);
    BOOST_CHECK(planes[2].major_axis    == axis_t::y_axis);
    BOOST_CHECK(planes[3].n             == point_t<>(0.0f, 0.0f, 1.0f));
    BOOST_CHECK_CLOSE(planes[3].p, 9.6f, result_tolerance);
    BOOST_CHECK(planes[3].major_axis    == axis_t::z_axis);
    BOOST_CHECK(planes[4].n             == point_t<>(0.0f, 0.0f, 1.0f));
    BOOST_CHECK_CLOSE(planes[4].p, 27.6f, result_tolerance);
    BOOST_CHECK(planes[4].major_axis    == axis_t::z_axis);
    BOOST_CHECK(planes[5].n             == point_t<>(0.0f, 0.0f, 1.0f));
    BOOST_CHECK_CLOSE(planes[5].p, 45.6f, result_tolerance);
    BOOST_CHECK(planes[5].major_axis    == axis_t::z_axis);
    BOOST_CHECK(planes[6].n             == point_t<>(0.0f, 0.0f, 1.0f));
    BOOST_CHECK_CLOSE(planes[6].p, 63.6f, result_tolerance);
    BOOST_CHECK(planes[6].major_axis    == axis_t::z_axis);
    BOOST_CHECK(planes[7].n             == point_t<>(0.0f, 0.0f, 1.0f));
    BOOST_CHECK_CLOSE(planes[7].p, 81.6f, result_tolerance);
    BOOST_CHECK(planes[7].major_axis    == axis_t::z_axis);
    BOOST_CHECK(planes[8].n             == point_t<>(0.0f, 0.0f, 1.0f));
    BOOST_CHECK_CLOSE(planes[8].p, 99.6f, result_tolerance);
    BOOST_CHECK(planes[8].major_axis    == axis_t::z_axis);
    BOOST_CHECK(planes[9].n             == point_t<>(0.0f, 0.0f, 1.0f));
    BOOST_CHECK_CLOSE(planes[9].p, 117.6f, result_tolerance);
    BOOST_CHECK(planes[9].major_axis    == axis_t::z_axis);
}

/* Test refine axes aligned clipping planes */
BOOST_AUTO_TEST_CASE( refine_axes_aligned_clipping_planes_x_test )
{
    std::vector<plane> planes;
    diagonal.compute_bounding_box();

    diagonal.refine_axes_aligned_clipping_planes(&planes, plane(point_t<>(0.0f, 0.0f, 0.0f), 0.0f, axis_t::x_axis), 9, 1);
    BOOST_REQUIRE(planes.size() == 6);
    BOOST_CHECK(planes[0].n             == point_t<>(1.0f, 0.0f, 0.0f));
    BOOST_CHECK_CLOSE(planes[0].p, -0.8f, result_tolerance);
    BOOST_CHECK(planes[0].major_axis    == axis_t::x_axis);
    BOOST_CHECK(planes[1].n             == point_t<>(1.0f, 0.0f, 0.0f));
    BOOST_CHECK_CLOSE(planes[1].p, 1.2f, result_tolerance);
    BOOST_CHECK(planes[1].major_axis    == axis_t::x_axis);
    BOOST_CHECK(planes[2].n             == point_t<>(1.0f, 0.0f, 0.0f));
    BOOST_CHECK_CLOSE(planes[2].p, 3.2f, result_tolerance);
    BOOST_CHECK(planes[2].major_axis    == axis_t::x_axis);
    BOOST_CHECK(planes[3].n             == point_t<>(1.0f, 0.0f, 0.0f));
    BOOST_CHECK_CLOSE(planes[3].p, 5.2f, result_tolerance);
    BOOST_CHECK(planes[3].major_axis    == axis_t::x_axis);
    BOOST_CHECK(planes[4].n             == point_t<>(1.0f, 0.0f, 0.0f));
    BOOST_CHECK_CLOSE(planes[4].p, 7.2f, result_tolerance);
    BOOST_CHECK(planes[4].major_axis    == axis_t::x_axis);
    BOOST_CHECK(planes[5].n             == point_t<>(1.0f, 0.0f, 0.0f));
    BOOST_CHECK_CLOSE(planes[5].p, 9.2f, result_tolerance);
    BOOST_CHECK(planes[5].major_axis    == axis_t::x_axis);
}

BOOST_AUTO_TEST_CASE( refine_axes_aligned_clipping_planes_y_test )
{
    std::vector<plane> planes;
    diagonal.compute_bounding_box();

    diagonal.refine_axes_aligned_clipping_planes(&planes, plane(point_t<>(0.0f, 0.0f, 0.0f), 0.0f, axis_t::y_axis), 9, 6);
    BOOST_REQUIRE(planes.size() == 11);
    BOOST_CHECK(planes[0].n             == point_t<>(0.0f, 1.0f, 0.0f));
    BOOST_CHECK_CLOSE(planes[0].p, 12.9f, result_tolerance);
    BOOST_CHECK(planes[0].major_axis    == axis_t::y_axis);
    BOOST_CHECK(planes[1].n             == point_t<>(0.0f, 1.0f, 0.0f));
    BOOST_CHECK_CLOSE(planes[1].p, 14.9f, result_tolerance);
    BOOST_CHECK(planes[1].major_axis    == axis_t::y_axis);
    BOOST_CHECK(planes[2].n             == point_t<>(0.0f, 1.0f, 0.0f));
    BOOST_CHECK_CLOSE(planes[2].p, 16.9f, result_tolerance);
    BOOST_CHECK(planes[2].major_axis    == axis_t::y_axis);
    BOOST_CHECK(planes[3].n             == point_t<>(0.0f, 1.0f, 0.0f));
    BOOST_CHECK_CLOSE(planes[3].p, 18.9f, result_tolerance);
    BOOST_CHECK(planes[3].major_axis    == axis_t::y_axis);
    BOOST_CHECK(planes[4].n             == point_t<>(0.0f, 1.0f, 0.0f));
    BOOST_CHECK_CLOSE(planes[4].p, 20.9f, result_tolerance);
    BOOST_CHECK(planes[4].major_axis    == axis_t::y_axis);
    BOOST_CHECK(planes[5].n             == point_t<>(0.0f, 1.0f, 0.0f));
    BOOST_CHECK_CLOSE(planes[5].p, 22.9f, result_tolerance);
    BOOST_CHECK(planes[5].major_axis    == axis_t::y_axis);
    BOOST_CHECK(planes[6].n             == point_t<>(0.0f, 1.0f, 0.0f));
    BOOST_CHECK_CLOSE(planes[6].p, 24.9f, result_tolerance);
    BOOST_CHECK(planes[6].major_axis    == axis_t::y_axis);
    BOOST_CHECK(planes[7].n             == point_t<>(0.0f, 1.0f, 0.0f));
    BOOST_CHECK_CLOSE(planes[7].p, 26.9f, result_tolerance);
    BOOST_CHECK(planes[7].major_axis    == axis_t::y_axis);
    BOOST_CHECK(planes[8].n             == point_t<>(0.0f, 1.0f, 0.0f));
    BOOST_CHECK_CLOSE(planes[8].p, 28.9f, result_tolerance);
    BOOST_CHECK(planes[8].major_axis    == axis_t::y_axis);
    BOOST_CHECK(planes[9].n             == point_t<>(0.0f, 1.0f, 0.0f));
    BOOST_CHECK_CLOSE(planes[9].p, 30.9f, result_tolerance);
    BOOST_CHECK(planes[9].major_axis    == axis_t::y_axis);
    BOOST_CHECK(planes[10].n            == point_t<>(0.0f, 1.0f, 0.0f));
    BOOST_CHECK_CLOSE(planes[10].p, 32.9f, result_tolerance);
    BOOST_CHECK(planes[10].major_axis   == axis_t::y_axis);
}

BOOST_AUTO_TEST_CASE( refine_axes_aligned_clipping_planes_z_test )
{
    std::vector<plane> planes;
    diagonal.compute_bounding_box();

    diagonal.refine_axes_aligned_clipping_planes(&planes, plane(point_t<>(0.0f, 0.0f, 0.0f), 0.0f, axis_t::z_axis), 4, 30);
    BOOST_REQUIRE(planes.size() == 9);
    BOOST_CHECK(planes[0].n             == point_t<>(0.0f, 0.0f, 1.0f));
    BOOST_CHECK_CLOSE(planes[0].p, 61.6f, result_tolerance);
    BOOST_CHECK(planes[0].major_axis    == axis_t::z_axis);
    BOOST_CHECK(planes[1].n             == point_t<>(0.0f, 0.0f, 1.0f));
    BOOST_CHECK_CLOSE(planes[1].p, 63.6f, result_tolerance);
    BOOST_CHECK(planes[1].major_axis    == axis_t::z_axis);
    BOOST_CHECK(planes[2].n             == point_t<>(0.0f, 0.0f, 1.0f));
    BOOST_CHECK_CLOSE(planes[2].p, 65.6f, result_tolerance);
    BOOST_CHECK(planes[2].major_axis    == axis_t::z_axis);
    BOOST_CHECK(planes[3].n             == point_t<>(0.0f, 0.0f, 1.0f));
    BOOST_CHECK_CLOSE(planes[3].p, 67.6f, result_tolerance);
    BOOST_CHECK(planes[3].major_axis    == axis_t::z_axis);
    BOOST_CHECK(planes[4].n             == point_t<>(0.0f, 0.0f, 1.0f));
    BOOST_CHECK_CLOSE(planes[4].p, 69.6f, result_tolerance);
    BOOST_CHECK(planes[4].major_axis    == axis_t::z_axis);
    BOOST_CHECK(planes[5].n             == point_t<>(0.0f, 0.0f, 1.0f));
    BOOST_CHECK_CLOSE(planes[5].p, 71.6f, result_tolerance);
    BOOST_CHECK(planes[5].major_axis    == axis_t::z_axis);
    BOOST_CHECK(planes[6].n             == point_t<>(0.0f, 0.0f, 1.0f));
    BOOST_CHECK_CLOSE(planes[6].p, 73.6f, result_tolerance);
    BOOST_CHECK(planes[6].major_axis    == axis_t::z_axis);
    BOOST_CHECK(planes[7].n             == point_t<>(0.0f, 0.0f, 1.0f));
    BOOST_CHECK_CLOSE(planes[7].p, 75.6f, result_tolerance);
    BOOST_CHECK(planes[7].major_axis    == axis_t::z_axis);
    BOOST_CHECK(planes[8].n             == point_t<>(0.0f, 0.0f, 1.0f));
    BOOST_CHECK_CLOSE(planes[8].p, 77.6f, result_tolerance);
    BOOST_CHECK(planes[8].major_axis    == axis_t::z_axis);
}

/* Test compute convex hull */
BOOST_AUTO_TEST_CASE( empty_compute_convex_hull_test )
{
    convex_mesh mesh;
    empty.compute_convex_hull(&mesh, 1);

    /* Checks */
    BOOST_REQUIRE(mesh.number_of_points()       == 0);
    BOOST_REQUIRE(mesh.number_of_triangles()    == 0);
}

BOOST_AUTO_TEST_CASE( cube_compute_convex_hull_test )
{
    convex_mesh mesh;
    cube.compute_convex_hull(&mesh, 1);

    /* Checks */
    BOOST_REQUIRE(mesh.number_of_points()       == 0);
    BOOST_REQUIRE(mesh.number_of_triangles()    == 0);
}

BOOST_AUTO_TEST_CASE( on_surface_compute_convex_hull_test )
{
    convex_mesh mesh;
    on_surface.compute_convex_hull(&mesh, 1);

    /* Checks */
    BOOST_REQUIRE(mesh.number_of_points() == 8);
    BOOST_CHECK(mesh.points()[0] == point_t<>(6.5f, 2.5f, 3.5f));
    BOOST_CHECK(mesh.points()[1] == point_t<>(6.5f, 2.5f, 2.5f));
    BOOST_CHECK(mesh.points()[2] == point_t<>(0.5f, 2.5f, 3.5f));
    BOOST_CHECK(mesh.points()[3] == point_t<>(6.5f, 1.5f, 3.5f));
    BOOST_CHECK(mesh.points()[4] == point_t<>(6.5f, 1.5f, 2.5f));
    BOOST_CHECK(mesh.points()[5] == point_t<>(0.5f, 2.5f, 2.5f));
    BOOST_CHECK(mesh.points()[6] == point_t<>(0.5f, 1.5f, 3.5f));
    BOOST_CHECK(mesh.points()[7] == point_t<>(0.5f, 1.5f, 2.5f));

    BOOST_REQUIRE(mesh.number_of_triangles() == 12);
    BOOST_CHECK(mesh.triangles()[ 0].x == 1);
    BOOST_CHECK(mesh.triangles()[ 0].y == 5);
    BOOST_CHECK(mesh.triangles()[ 0].z == 2);
    BOOST_CHECK(mesh.triangles()[ 1].x == 1);
    BOOST_CHECK(mesh.triangles()[ 1].y == 2);
    BOOST_CHECK(mesh.triangles()[ 1].z == 0);
    BOOST_CHECK(mesh.triangles()[ 2].x == 2);
    BOOST_CHECK(mesh.triangles()[ 2].y == 6);
    BOOST_CHECK(mesh.triangles()[ 2].z == 3);
    BOOST_CHECK(mesh.triangles()[ 3].x == 2);
    BOOST_CHECK(mesh.triangles()[ 3].y == 3);
    BOOST_CHECK(mesh.triangles()[ 3].z == 0);
    BOOST_CHECK(mesh.triangles()[ 4].x == 3);
    BOOST_CHECK(mesh.triangles()[ 4].y == 4);
    BOOST_CHECK(mesh.triangles()[ 4].z == 1);
    BOOST_CHECK(mesh.triangles()[ 5].x == 3);
    BOOST_CHECK(mesh.triangles()[ 5].y == 1);
    BOOST_CHECK(mesh.triangles()[ 5].z == 0);
    BOOST_CHECK(mesh.triangles()[ 6].x == 4);
    BOOST_CHECK(mesh.triangles()[ 6].y == 7);
    BOOST_CHECK(mesh.triangles()[ 6].z == 5);
    BOOST_CHECK(mesh.triangles()[ 7].x == 4);
    BOOST_CHECK(mesh.triangles()[ 7].y == 5);
    BOOST_CHECK(mesh.triangles()[ 7].z == 1);
    BOOST_CHECK(mesh.triangles()[ 8].x == 5);
    BOOST_CHECK(mesh.triangles()[ 8].y == 7);
    BOOST_CHECK(mesh.triangles()[ 8].z == 6);
    BOOST_CHECK(mesh.triangles()[ 9].x == 5);
    BOOST_CHECK(mesh.triangles()[ 9].y == 6);
    BOOST_CHECK(mesh.triangles()[ 9].z == 2);
    BOOST_CHECK(mesh.triangles()[10].x == 6);
    BOOST_CHECK(mesh.triangles()[10].y == 7);
    BOOST_CHECK(mesh.triangles()[10].z == 4);
    BOOST_CHECK(mesh.triangles()[11].x == 6);
    BOOST_CHECK(mesh.triangles()[11].y == 4);
    BOOST_CHECK(mesh.triangles()[11].z == 3);
}

BOOST_AUTO_TEST_CASE( on_surface_down_sampling_compute_convex_hull_test )
{
    convex_mesh mesh;
    on_surface.compute_convex_hull(&mesh, 3);

    /* Checks */
    BOOST_REQUIRE(mesh.number_of_points() == 8);
    BOOST_CHECK(mesh.points()[0] == point_t<>(6.5f, 2.5f, 3.5f));
    BOOST_CHECK(mesh.points()[1] == point_t<>(6.5f, 2.5f, 2.5f));
    BOOST_CHECK(mesh.points()[2] == point_t<>(5.5f, 2.5f, 3.5f));
    BOOST_CHECK(mesh.points()[3] == point_t<>(6.5f, 1.5f, 3.5f));
    BOOST_CHECK(mesh.points()[4] == point_t<>(6.5f, 1.5f, 2.5f));
    BOOST_CHECK(mesh.points()[5] == point_t<>(5.5f, 2.5f, 2.5f));
    BOOST_CHECK(mesh.points()[6] == point_t<>(5.5f, 1.5f, 3.5f));
    BOOST_CHECK(mesh.points()[7] == point_t<>(5.5f, 1.5f, 2.5f));

    BOOST_REQUIRE(mesh.number_of_triangles() == 12);
    BOOST_CHECK(mesh.triangles()[ 0].x == 1);
    BOOST_CHECK(mesh.triangles()[ 0].y == 5);
    BOOST_CHECK(mesh.triangles()[ 0].z == 2);
    BOOST_CHECK(mesh.triangles()[ 1].x == 1);
    BOOST_CHECK(mesh.triangles()[ 1].y == 2);
    BOOST_CHECK(mesh.triangles()[ 1].z == 0);
    BOOST_CHECK(mesh.triangles()[ 2].x == 2);
    BOOST_CHECK(mesh.triangles()[ 2].y == 6);
    BOOST_CHECK(mesh.triangles()[ 2].z == 3);
    BOOST_CHECK(mesh.triangles()[ 3].x == 2);
    BOOST_CHECK(mesh.triangles()[ 3].y == 3);
    BOOST_CHECK(mesh.triangles()[ 3].z == 0);
    BOOST_CHECK(mesh.triangles()[ 4].x == 3);
    BOOST_CHECK(mesh.triangles()[ 4].y == 4);
    BOOST_CHECK(mesh.triangles()[ 4].z == 1);
    BOOST_CHECK(mesh.triangles()[ 5].x == 3);
    BOOST_CHECK(mesh.triangles()[ 5].y == 1);
    BOOST_CHECK(mesh.triangles()[ 5].z == 0);
    BOOST_CHECK(mesh.triangles()[ 6].x == 4);
    BOOST_CHECK(mesh.triangles()[ 6].y == 7);
    BOOST_CHECK(mesh.triangles()[ 6].z == 5);
    BOOST_CHECK(mesh.triangles()[ 7].x == 4);
    BOOST_CHECK(mesh.triangles()[ 7].y == 5);
    BOOST_CHECK(mesh.triangles()[ 7].z == 1);
    BOOST_CHECK(mesh.triangles()[ 8].x == 5);
    BOOST_CHECK(mesh.triangles()[ 8].y == 7);
    BOOST_CHECK(mesh.triangles()[ 8].z == 6);
    BOOST_CHECK(mesh.triangles()[ 9].x == 5);
    BOOST_CHECK(mesh.triangles()[ 9].y == 6);
    BOOST_CHECK(mesh.triangles()[ 9].z == 2);
    BOOST_CHECK(mesh.triangles()[10].x == 6);
    BOOST_CHECK(mesh.triangles()[10].y == 7);
    BOOST_CHECK(mesh.triangles()[10].z == 4);
    BOOST_CHECK(mesh.triangles()[11].x == 6);
    BOOST_CHECK(mesh.triangles()[11].y == 4);
    BOOST_CHECK(mesh.triangles()[11].z == 3);
}

BOOST_AUTO_TEST_CASE( low_cluster_size_on_surface_compute_convex_hull_test )
{
    convex_mesh mesh;
    on_surface.compute_convex_hull(&mesh, 1, 1);

    /* Checks */
    BOOST_REQUIRE(mesh.number_of_points() == 8);
    BOOST_CHECK(mesh.points()[0] == point_t<>(6.5f, 2.5f, 3.5f));
    BOOST_CHECK(mesh.points()[1] == point_t<>(6.5f, 1.5f, 3.5f));
    BOOST_CHECK(mesh.points()[2] == point_t<>(6.5f, 2.5f, 2.5f));
    BOOST_CHECK(mesh.points()[3] == point_t<>(0.5f, 2.5f, 3.5f));
    BOOST_CHECK(mesh.points()[4] == point_t<>(6.5f, 1.5f, 2.5f));
    BOOST_CHECK(mesh.points()[5] == point_t<>(0.5f, 1.5f, 3.5f));
    BOOST_CHECK(mesh.points()[6] == point_t<>(0.5f, 2.5f, 2.5f));
    BOOST_CHECK(mesh.points()[7] == point_t<>(0.5f, 1.5f, 2.5f));

    BOOST_REQUIRE(mesh.number_of_triangles() == 12);
    BOOST_CHECK(mesh.triangles()[ 0].x == 1);
    BOOST_CHECK(mesh.triangles()[ 0].y == 4);
    BOOST_CHECK(mesh.triangles()[ 0].z == 2);
    BOOST_CHECK(mesh.triangles()[ 1].x == 1);
    BOOST_CHECK(mesh.triangles()[ 1].y == 2);
    BOOST_CHECK(mesh.triangles()[ 1].z == 0);
    BOOST_CHECK(mesh.triangles()[ 2].x == 2);
    BOOST_CHECK(mesh.triangles()[ 2].y == 6);
    BOOST_CHECK(mesh.triangles()[ 2].z == 3);
    BOOST_CHECK(mesh.triangles()[ 3].x == 2);
    BOOST_CHECK(mesh.triangles()[ 3].y == 3);
    BOOST_CHECK(mesh.triangles()[ 3].z == 0);
    BOOST_CHECK(mesh.triangles()[ 4].x == 3);
    BOOST_CHECK(mesh.triangles()[ 4].y == 5);
    BOOST_CHECK(mesh.triangles()[ 4].z == 1);
    BOOST_CHECK(mesh.triangles()[ 5].x == 3);
    BOOST_CHECK(mesh.triangles()[ 5].y == 1);
    BOOST_CHECK(mesh.triangles()[ 5].z == 0);
    BOOST_CHECK(mesh.triangles()[ 6].x == 5);
    BOOST_CHECK(mesh.triangles()[ 6].y == 7);
    BOOST_CHECK(mesh.triangles()[ 6].z == 4);
    BOOST_CHECK(mesh.triangles()[ 7].x == 5);
    BOOST_CHECK(mesh.triangles()[ 7].y == 4);
    BOOST_CHECK(mesh.triangles()[ 7].z == 1);
    BOOST_CHECK(mesh.triangles()[ 8].x == 4);
    BOOST_CHECK(mesh.triangles()[ 8].y == 7);
    BOOST_CHECK(mesh.triangles()[ 8].z == 6);
    BOOST_CHECK(mesh.triangles()[ 9].x == 4);
    BOOST_CHECK(mesh.triangles()[ 9].y == 6);
    BOOST_CHECK(mesh.triangles()[ 9].z == 2);
    BOOST_CHECK(mesh.triangles()[10].x == 6);
    BOOST_CHECK(mesh.triangles()[10].y == 7);
    BOOST_CHECK(mesh.triangles()[10].z == 5);
    BOOST_CHECK(mesh.triangles()[11].x == 6);
    BOOST_CHECK(mesh.triangles()[11].y == 5);
    BOOST_CHECK(mesh.triangles()[11].z == 3);
}

BOOST_AUTO_TEST_SUITE_END()
}; /* namespace test */
}; /* namespace raptor_convex_decomposition */