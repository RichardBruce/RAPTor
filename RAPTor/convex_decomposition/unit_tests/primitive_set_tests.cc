#ifdef STAND_ALONE
#define BOOST_TEST_MODULE primitive_set test
#endif /* #ifdef STAND_ALONE */


/* Standard headers */

/* Boost headers */
#include "boost/noncopyable.hpp"
#include "boost/test/unit_test.hpp"

/* Convex Decomposition headers */
#include "primitive_set.h"
#include "voxel_set.h"


namespace raptor_convex_decomposition
{
namespace test
{
/* Test data */
struct primitive_set_fixture : private boost::noncopyable
{
    primitive_set_fixture() :
        cube(    { voxel(point_ti(3, 4, 2)) }, point_t( 3.5f, -2.7f, 1.6f), 1.0f),
        diagonal(
        {
            voxel(point_ti(3, 4,   2)),
            voxel(point_ti(4, 6,   4)),
            voxel(point_ti(5, 8,   8)),
            voxel(point_ti(6, 10, 16)),
            voxel(point_ti(7, 12, 32)),
            voxel(point_ti(8, 14, 64))
        }, point_t(-7.8f,  3.9f, 4.6f), 2.0f),
        row(
        {
            voxel(point_ti(0, 0, 0)),
            voxel(point_ti(1, 0, 0)),
            voxel(point_ti(2, 0, 0)),
            voxel(point_ti(3, 0, 0)),
            voxel(point_ti(4, 0, 0)),
            voxel(point_ti(5, 0, 0)),
            voxel(point_ti(6, 0, 0)),
            voxel(point_ti(7, 0, 0))
        }, point_t(1.0f, 2.0f, 3.0f), 1.0f),
        stack(
        {
            voxel(point_ti(0, 0, 0)),
            voxel(point_ti(0, 1, 0)),
            voxel(point_ti(0, 2, 0)),
            voxel(point_ti(0, 3, 0)),
            voxel(point_ti(0, 4, 0)),
            voxel(point_ti(0, 5, 0)),
            voxel(point_ti(0, 6, 0)),
            voxel(point_ti(0, 7, 0))
        }, point_t(1.0f, 2.0f, 3.0f), 1.0f)
    {  }

    voxel_set   cube;
    voxel_set   diagonal;
    voxel_set   row;
    voxel_set   stack;
    point_t     dir;
};


BOOST_FIXTURE_TEST_SUITE( primitive_set_tests, primitive_set_fixture )

const float result_tolerance = 0.0005f;


/* Test compute preferred cutting direction */
BOOST_AUTO_TEST_CASE( cube_compute_preferred_cutting_direction_test )
{
    cube.compute_bounding_box();
    cube.compute_principal_axes();
    BOOST_CHECK(cube.eigen_value(axis_t::x_axis) == 0.0f);
    BOOST_CHECK(cube.eigen_value(axis_t::y_axis) == 0.0f);
    BOOST_CHECK(cube.eigen_value(axis_t::z_axis) == 0.0f);

    BOOST_CHECK(cube.compute_preferred_cutting_direction(&dir) == 0.0f);
    BOOST_CHECK(dir == point_t(0.0f, 0.0f, 1.0f));
}

BOOST_AUTO_TEST_CASE( row_compute_preferred_cutting_direction_test )
{
    row.compute_bounding_box();
    row.compute_principal_axes();
    BOOST_CHECK(row.eigen_value(axis_t::x_axis) == 5.5f);
    BOOST_CHECK(row.eigen_value(axis_t::y_axis) == 0.0f);
    BOOST_CHECK(row.eigen_value(axis_t::z_axis) == 0.0f);

    BOOST_CHECK(row.compute_preferred_cutting_direction(&dir) == 0.0f);
    BOOST_CHECK(dir == point_t(1.0f, 0.0f, 0.0f));
}

BOOST_AUTO_TEST_CASE( stack_compute_preferred_cutting_direction_test )
{
    stack.compute_bounding_box();
    stack.compute_principal_axes();
    BOOST_CHECK(stack.eigen_value(axis_t::x_axis) == 0.0f);
    BOOST_CHECK(stack.eigen_value(axis_t::y_axis) == 5.5f);
    BOOST_CHECK(stack.eigen_value(axis_t::z_axis) == 0.0f);

    BOOST_CHECK(stack.compute_preferred_cutting_direction(&dir) == 0.0f);
    BOOST_CHECK(dir == point_t(0.0f, 1.0f, 0.0f));
}

BOOST_AUTO_TEST_CASE( diagonal_compute_preferred_cutting_direction_test )
{
    diagonal.compute_bounding_box();
    diagonal.compute_principal_axes();
    BOOST_CHECK(fabs(diagonal.eigen_value(axis_t::x_axis) -   0.195948f)    < result_tolerance);
    BOOST_CHECK(fabs(diagonal.eigen_value(axis_t::y_axis) -   2.60634f)     < result_tolerance);
    BOOST_CHECK(fabs(diagonal.eigen_value(axis_t::z_axis) - 481.031f)       < result_tolerance);

    BOOST_CHECK_CLOSE(diagonal.compute_preferred_cutting_direction(&dir), 0.149518f, result_tolerance);
    BOOST_CHECK(dir == point_t(0.0f, 0.0f, 1.0f));
}

BOOST_AUTO_TEST_SUITE_END()
}; /* namespace test */
}; /* namespace raptor_convex_decomposition */
