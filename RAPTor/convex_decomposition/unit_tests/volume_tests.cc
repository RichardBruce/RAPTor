#ifdef STAND_ALONE
#define BOOST_TEST_MODULE volume test
#endif /* #ifdef STAND_ALONE */


/* Standard headers */

/* Boost headers */
#include "boost/noncopyable.hpp"
#include "boost/test/unit_test.hpp"

/* Convex Decomposition headers */
#include "volume.h"


namespace raptor_convex_decomposition
{
namespace test
{
/* Test data */
struct volume_fixture : private boost::noncopyable
{
    volume_fixture() :
        cube_points(
            {
                point_t(-0.5f, -0.5f, -0.5f),
                point_t( 0.5f, -0.5f, -0.5f),
                point_t( 0.5f,  0.5f, -0.5f),
                point_t(-0.5f,  0.5f, -0.5f),
                point_t(-0.5f, -0.5f,  0.5f),
                point_t( 0.5f, -0.5f,  0.5f),
                point_t( 0.5f,  0.5f,  0.5f),
                point_t(-0.5f,  0.5f,  0.5f)
            }),
        cubiod_x_points(
            {
                point_t(-1.0f, -0.5f, -0.5f),
                point_t( 1.0f, -0.5f, -0.5f),
                point_t( 1.0f,  0.5f, -0.5f),
                point_t(-1.0f,  0.5f, -0.5f),
                point_t(-1.0f, -0.5f,  0.5f),
                point_t( 1.0f, -0.5f,  0.5f),
                point_t( 1.0f,  0.5f,  0.5f),
                point_t(-1.0f,  0.5f,  0.5f)
            }),
        cubiod_y_points(
            {
                point_t(-0.5f, -1.0f, -0.5f),
                point_t( 0.5f, -1.0f, -0.5f),
                point_t( 0.5f,  1.0f, -0.5f),
                point_t(-0.5f,  1.0f, -0.5f),
                point_t(-0.5f, -1.0f,  0.5f),
                point_t( 0.5f, -1.0f,  0.5f),
                point_t( 0.5f,  1.0f,  0.5f),
                point_t(-0.5f,  1.0f,  0.5f)
            }),
        cubiod_z_points(
            {
                point_t(-0.5f, -0.5f, -2.0f),
                point_t( 0.5f, -0.5f, -2.0f),
                point_t( 0.5f,  0.5f, -2.0f),
                point_t(-0.5f,  0.5f, -2.0f),
                point_t(-0.5f, -0.5f,  2.0f),
                point_t( 0.5f, -0.5f,  2.0f),
                point_t( 0.5f,  0.5f,  2.0f),
                point_t(-0.5f,  0.5f,  2.0f)
            }),
        cube2x_points(
            {
                point_t(-1.0f, -1.0f, -1.0f),
                point_t( 1.0f, -1.0f, -1.0f),
                point_t( 1.0f,  1.0f, -1.0f),
                point_t(-1.0f,  1.0f, -1.0f),
                point_t(-1.0f, -1.0f,  1.0f),
                point_t( 1.0f, -1.0f,  1.0f),
                point_t( 1.0f,  1.0f,  1.0f),
                point_t(-1.0f,  1.0f,  1.0f)
            }),
        cube_triangles(
            {
                point_ti<>(0, 1, 2), point_ti<>(0, 2, 3), /* Front face     */
                point_ti<>(4, 6, 5), point_ti<>(4, 7, 6), /* Back face      */
                point_ti<>(4, 0, 7), point_ti<>(7, 0, 3), /* Left face      */
                point_ti<>(1, 6, 2), point_ti<>(1, 5, 6), /* Right face     */
                point_ti<>(3, 6, 7), point_ti<>(3, 2, 6), /* Top face       */
                point_ti<>(0, 4, 1), point_ti<>(1, 4, 5)  /* Bottom face    */
            })
    {  }

    std::vector<point_t> make_cube_pair_points() const
    {
        auto shifted_points(cube_points);
        std::transform(shifted_points.begin(), shifted_points.end(), shifted_points.begin(), [](const point_t &p){ return p + 2.0f; });
        shifted_points.insert(shifted_points.end(), cube2x_points.begin(), cube2x_points.end());
        return shifted_points;
    }

    std::vector<point_ti<>> make_cube_pair_triangles() const
    {
        auto shifted_triangles(cube_triangles);
        std::transform(shifted_triangles.begin(), shifted_triangles.end(), shifted_triangles.begin(), [](const point_ti<> &p){ return p + 8; });
        shifted_triangles.insert(shifted_triangles.end(), cube_triangles.begin(), cube_triangles.end());
        return shifted_triangles;
    }

    const std::vector<point_t>      cube_points;
    const std::vector<point_t>      cubiod_x_points;
    const std::vector<point_t>      cubiod_y_points;
    const std::vector<point_t>      cubiod_z_points;
    const std::vector<point_t>      cube2x_points;
    const std::vector<point_ti<>>   cube_triangles;
};


BOOST_FIXTURE_TEST_SUITE( volume_tests, volume_fixture )

const float result_tolerance = 0.0005f;


/* Test Ctor */
BOOST_AUTO_TEST_CASE( cube_ctor_test )
{
    volume cube(cube_points, cube_triangles, 2, point_t(0.0f, 0.0f, 0.0f), {{ 1.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, { 0.0f, 0.0f, 1.0f }});
    BOOST_CHECK(cube.number_of_primitives_on_surface()  == 8);
    BOOST_CHECK(cube.number_of_primitives_inside()      == 0);
    BOOST_CHECK(cube.number_of_primitives_outside()     == 24);
    BOOST_CHECK(cube.get_voxel(0, 0, 0) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(cube.get_voxel(1, 0, 0) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(cube.get_voxel(2, 0, 0) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cube.get_voxel(3, 0, 0) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cube.get_voxel(0, 1, 0) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(cube.get_voxel(1, 1, 0) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(cube.get_voxel(2, 1, 0) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cube.get_voxel(3, 1, 0) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cube.get_voxel(0, 2, 0) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cube.get_voxel(1, 2, 0) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cube.get_voxel(2, 2, 0) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cube.get_voxel(3, 2, 0) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cube.get_voxel(0, 3, 0) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cube.get_voxel(1, 3, 0) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cube.get_voxel(2, 3, 0) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cube.get_voxel(3, 3, 0) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cube.get_voxel(0, 0, 1) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(cube.get_voxel(1, 0, 1) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(cube.get_voxel(2, 0, 1) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cube.get_voxel(3, 0, 1) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cube.get_voxel(0, 1, 1) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(cube.get_voxel(1, 1, 1) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(cube.get_voxel(2, 1, 1) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cube.get_voxel(3, 1, 1) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cube.get_voxel(0, 2, 1) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cube.get_voxel(1, 2, 1) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cube.get_voxel(2, 2, 1) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cube.get_voxel(3, 2, 1) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cube.get_voxel(0, 3, 1) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cube.get_voxel(1, 3, 1) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cube.get_voxel(2, 3, 1) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cube.get_voxel(3, 3, 1) == voxel_value_t::primitive_outside_surface);
}

BOOST_AUTO_TEST_CASE( cube_with_inside_ctor_test )
{
    volume cube(cube_points, cube_triangles, 3, point_t(0.0f, 0.0f, 0.0f), {{ 1.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, { 0.0f, 0.0f, 1.0f }});
    BOOST_CHECK(cube.number_of_primitives_on_surface()  == 26);
    BOOST_CHECK(cube.number_of_primitives_inside()      == 1);
    BOOST_CHECK(cube.number_of_primitives_outside()     == 48);
    BOOST_CHECK(cube.get_voxel(0, 0, 0) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(cube.get_voxel(1, 0, 0) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(cube.get_voxel(2, 0, 0) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(cube.get_voxel(3, 0, 0) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cube.get_voxel(4, 0, 0) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cube.get_voxel(0, 1, 0) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(cube.get_voxel(1, 1, 0) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(cube.get_voxel(2, 1, 0) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(cube.get_voxel(3, 1, 0) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cube.get_voxel(4, 1, 0) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cube.get_voxel(0, 2, 0) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(cube.get_voxel(1, 2, 0) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(cube.get_voxel(2, 2, 0) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(cube.get_voxel(3, 2, 0) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cube.get_voxel(4, 2, 0) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cube.get_voxel(0, 3, 0) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cube.get_voxel(1, 3, 0) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cube.get_voxel(2, 3, 0) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cube.get_voxel(3, 3, 0) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cube.get_voxel(4, 3, 0) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cube.get_voxel(0, 4, 0) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cube.get_voxel(1, 4, 0) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cube.get_voxel(2, 4, 0) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cube.get_voxel(3, 4, 0) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cube.get_voxel(4, 4, 0) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cube.get_voxel(0, 0, 1) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(cube.get_voxel(1, 0, 1) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(cube.get_voxel(2, 0, 1) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(cube.get_voxel(3, 0, 1) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cube.get_voxel(4, 0, 1) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cube.get_voxel(0, 1, 1) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(cube.get_voxel(1, 1, 1) == voxel_value_t::primitive_inside_surface);
    BOOST_CHECK(cube.get_voxel(2, 1, 1) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(cube.get_voxel(3, 1, 1) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cube.get_voxel(4, 1, 1) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cube.get_voxel(0, 2, 1) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(cube.get_voxel(1, 2, 1) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(cube.get_voxel(2, 2, 1) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(cube.get_voxel(3, 2, 1) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cube.get_voxel(4, 2, 1) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cube.get_voxel(0, 3, 1) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cube.get_voxel(1, 3, 1) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cube.get_voxel(2, 3, 1) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cube.get_voxel(3, 3, 1) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cube.get_voxel(4, 3, 1) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cube.get_voxel(0, 4, 1) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cube.get_voxel(1, 4, 1) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cube.get_voxel(2, 4, 1) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cube.get_voxel(3, 4, 1) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cube.get_voxel(4, 4, 1) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cube.get_voxel(0, 0, 2) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(cube.get_voxel(1, 0, 2) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(cube.get_voxel(2, 0, 2) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(cube.get_voxel(3, 0, 2) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cube.get_voxel(4, 0, 2) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cube.get_voxel(0, 1, 2) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(cube.get_voxel(1, 1, 2) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(cube.get_voxel(2, 1, 2) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(cube.get_voxel(3, 1, 2) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cube.get_voxel(4, 1, 2) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cube.get_voxel(0, 2, 2) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(cube.get_voxel(1, 2, 2) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(cube.get_voxel(2, 2, 2) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(cube.get_voxel(3, 2, 2) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cube.get_voxel(4, 2, 2) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cube.get_voxel(0, 3, 2) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cube.get_voxel(1, 3, 2) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cube.get_voxel(2, 3, 2) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cube.get_voxel(3, 3, 2) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cube.get_voxel(4, 3, 2) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cube.get_voxel(0, 4, 2) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cube.get_voxel(1, 4, 2) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cube.get_voxel(2, 4, 2) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cube.get_voxel(3, 4, 2) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cube.get_voxel(4, 4, 2) == voxel_value_t::primitive_outside_surface);
}

BOOST_AUTO_TEST_CASE( cube_pair_ctor_test )
{
    const auto shifted_points(make_cube_pair_points());
    const auto shifted_triangles(make_cube_pair_triangles());

    volume cubes(shifted_points, shifted_triangles, 3, point_t(0.0f, 0.0f, 0.0f), {{ 1.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, { 0.0f, 0.0f, 1.0f }});
    BOOST_CHECK(cubes.number_of_primitives_on_surface() == 15);
    BOOST_CHECK(cubes.number_of_primitives_inside()     == 0);
    BOOST_CHECK(cubes.number_of_primitives_outside()    == 60);
    BOOST_CHECK(cubes.get_voxel(0, 0, 0) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(cubes.get_voxel(1, 0, 0) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(cubes.get_voxel(2, 0, 0) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubes.get_voxel(3, 0, 0) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubes.get_voxel(4, 0, 0) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubes.get_voxel(0, 1, 0) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(cubes.get_voxel(1, 1, 0) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(cubes.get_voxel(2, 1, 0) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubes.get_voxel(3, 1, 0) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubes.get_voxel(4, 1, 0) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubes.get_voxel(0, 2, 0) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubes.get_voxel(1, 2, 0) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubes.get_voxel(2, 2, 0) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubes.get_voxel(3, 2, 0) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubes.get_voxel(4, 2, 0) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubes.get_voxel(0, 3, 0) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubes.get_voxel(1, 3, 0) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubes.get_voxel(2, 3, 0) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubes.get_voxel(3, 3, 0) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubes.get_voxel(4, 3, 0) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubes.get_voxel(0, 4, 0) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubes.get_voxel(1, 4, 0) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubes.get_voxel(2, 4, 0) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubes.get_voxel(3, 4, 0) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubes.get_voxel(4, 4, 0) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubes.get_voxel(0, 0, 1) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(cubes.get_voxel(1, 0, 1) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(cubes.get_voxel(2, 0, 1) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubes.get_voxel(3, 0, 1) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubes.get_voxel(4, 0, 1) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubes.get_voxel(0, 1, 1) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(cubes.get_voxel(1, 1, 1) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(cubes.get_voxel(2, 1, 1) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(cubes.get_voxel(3, 1, 1) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubes.get_voxel(4, 1, 1) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubes.get_voxel(0, 2, 1) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubes.get_voxel(1, 2, 1) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(cubes.get_voxel(2, 2, 1) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(cubes.get_voxel(3, 2, 1) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubes.get_voxel(4, 2, 1) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubes.get_voxel(0, 3, 1) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubes.get_voxel(1, 3, 1) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubes.get_voxel(2, 3, 1) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubes.get_voxel(3, 3, 1) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubes.get_voxel(4, 3, 1) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubes.get_voxel(0, 4, 1) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubes.get_voxel(1, 4, 1) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubes.get_voxel(2, 4, 1) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubes.get_voxel(3, 4, 1) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubes.get_voxel(4, 4, 1) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubes.get_voxel(0, 0, 2) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubes.get_voxel(1, 0, 2) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubes.get_voxel(2, 0, 2) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubes.get_voxel(3, 0, 2) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubes.get_voxel(4, 0, 2) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubes.get_voxel(0, 1, 2) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubes.get_voxel(1, 1, 2) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(cubes.get_voxel(2, 1, 2) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(cubes.get_voxel(3, 1, 2) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubes.get_voxel(4, 1, 2) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubes.get_voxel(0, 2, 2) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubes.get_voxel(1, 2, 2) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(cubes.get_voxel(2, 2, 2) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(cubes.get_voxel(3, 2, 2) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubes.get_voxel(4, 2, 2) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubes.get_voxel(0, 3, 2) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubes.get_voxel(1, 3, 2) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubes.get_voxel(2, 3, 2) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubes.get_voxel(3, 3, 2) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubes.get_voxel(4, 3, 2) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubes.get_voxel(0, 4, 2) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubes.get_voxel(1, 4, 2) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubes.get_voxel(2, 4, 2) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubes.get_voxel(3, 4, 2) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubes.get_voxel(4, 4, 2) == voxel_value_t::primitive_outside_surface);
}

BOOST_AUTO_TEST_CASE( cubiod_x_ctor_test )
{
    volume cubiod(cubiod_x_points, cube_triangles, 3, point_t(0.0f, 0.0f, 0.0f), {{ 1.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, { 0.0f, 0.0f, 1.0f }});
    BOOST_CHECK(cubiod.number_of_primitives_on_surface() == 12);
    BOOST_CHECK(cubiod.number_of_primitives_inside()     == 0);
    BOOST_CHECK(cubiod.number_of_primitives_outside()    == 15);
    BOOST_CHECK(cubiod.get_voxel(0, 0, 0) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(cubiod.get_voxel(1, 0, 0) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(cubiod.get_voxel(2, 0, 0) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(cubiod.get_voxel(0, 1, 0) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(cubiod.get_voxel(1, 1, 0) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(cubiod.get_voxel(2, 1, 0) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(cubiod.get_voxel(0, 2, 0) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubiod.get_voxel(1, 2, 0) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubiod.get_voxel(2, 2, 0) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubiod.get_voxel(0, 0, 1) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(cubiod.get_voxel(1, 0, 1) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(cubiod.get_voxel(2, 0, 1) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(cubiod.get_voxel(0, 1, 1) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(cubiod.get_voxel(1, 1, 1) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(cubiod.get_voxel(2, 1, 1) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(cubiod.get_voxel(0, 2, 1) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubiod.get_voxel(1, 2, 1) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubiod.get_voxel(2, 2, 1) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubiod.get_voxel(0, 0, 2) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubiod.get_voxel(1, 0, 2) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubiod.get_voxel(2, 0, 2) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubiod.get_voxel(0, 1, 2) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubiod.get_voxel(1, 1, 2) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubiod.get_voxel(2, 1, 2) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubiod.get_voxel(0, 2, 2) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubiod.get_voxel(1, 2, 2) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubiod.get_voxel(2, 2, 2) == voxel_value_t::primitive_outside_surface);
}

BOOST_AUTO_TEST_CASE( cubiod_y_ctor_test )
{
    volume cubiod(cubiod_y_points, cube_triangles, 3, point_t(0.0f, 0.0f, 0.0f), {{ 1.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, { 0.0f, 0.0f, 1.0f }});
    BOOST_CHECK(cubiod.number_of_primitives_on_surface() == 12);
    BOOST_CHECK(cubiod.number_of_primitives_inside()     == 0);
    BOOST_CHECK(cubiod.number_of_primitives_outside()    == 15);
    BOOST_CHECK(cubiod.get_voxel(0, 0, 0) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(cubiod.get_voxel(1, 0, 0) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(cubiod.get_voxel(2, 0, 0) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubiod.get_voxel(0, 1, 0) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(cubiod.get_voxel(1, 1, 0) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(cubiod.get_voxel(2, 1, 0) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubiod.get_voxel(0, 2, 0) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(cubiod.get_voxel(1, 2, 0) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(cubiod.get_voxel(2, 2, 0) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubiod.get_voxel(0, 0, 1) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(cubiod.get_voxel(1, 0, 1) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(cubiod.get_voxel(2, 0, 1) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubiod.get_voxel(0, 1, 1) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(cubiod.get_voxel(1, 1, 1) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(cubiod.get_voxel(2, 1, 1) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubiod.get_voxel(0, 2, 1) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(cubiod.get_voxel(1, 2, 1) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(cubiod.get_voxel(2, 2, 1) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubiod.get_voxel(0, 0, 2) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubiod.get_voxel(1, 0, 2) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubiod.get_voxel(2, 0, 2) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubiod.get_voxel(0, 1, 2) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubiod.get_voxel(1, 1, 2) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubiod.get_voxel(2, 1, 2) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubiod.get_voxel(0, 2, 2) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubiod.get_voxel(1, 2, 2) == voxel_value_t::primitive_outside_surface);
    BOOST_CHECK(cubiod.get_voxel(2, 2, 2) == voxel_value_t::primitive_outside_surface);
}

BOOST_AUTO_TEST_CASE( cubiod_z_ctor_test )
{
    volume cubiod(cubiod_z_points, cube_triangles, 3, point_t(0.0f, 0.0f, 0.0f), {{ 1.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, { 0.0f, 0.0f, 1.0f }});
    BOOST_CHECK(cubiod.number_of_primitives_on_surface() == 12);
    BOOST_CHECK(cubiod.number_of_primitives_inside()     == 0);
    BOOST_CHECK(cubiod.number_of_primitives_outside()    == 0);
    BOOST_CHECK(cubiod.get_voxel(0, 0, 0) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(cubiod.get_voxel(1, 0, 0) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(cubiod.get_voxel(0, 1, 0) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(cubiod.get_voxel(1, 1, 0) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(cubiod.get_voxel(0, 0, 1) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(cubiod.get_voxel(1, 0, 1) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(cubiod.get_voxel(0, 1, 1) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(cubiod.get_voxel(1, 1, 1) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(cubiod.get_voxel(0, 0, 2) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(cubiod.get_voxel(1, 0, 2) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(cubiod.get_voxel(0, 1, 2) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(cubiod.get_voxel(1, 1, 2) == voxel_value_t::primitive_on_surface);
}

/* Test convert to voxel set */
BOOST_AUTO_TEST_CASE( convert_cube_with_inside_voxel_set_test )
{
    /* Convert */
    volume cube(cube_points, cube_triangles, 3, point_t(0.0f, 0.0f, 0.0f), {{ 1.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, { 0.0f, 0.0f, 1.0f }});
    std::unique_ptr<voxel_set> vset(static_cast<voxel_set *>(cube.convert(discretisation_type_t::voxel)));

    /* Checks */
    vset->compute_bounding_box();
    BOOST_CHECK(vset->number_of_primitives()             == 27);
    BOOST_CHECK(vset->number_of_primitives_on_surface()  == 26);
    BOOST_CHECK(vset->number_of_primitives_inside()      == 1);
    BOOST_CHECK(vset->get_min_bb()                       == point_t(-0.5f, -0.5f, -0.5f));
    BOOST_CHECK(vset->get_min_bb_voxels()                == point_ti<>(0, 0, 0));
    BOOST_CHECK(vset->get_max_bb_voxels()                == point_ti<>(2, 2, 2));
    BOOST_CHECK(vset->get_barycenter()                   == point_ti<>(1, 1, 1));
    BOOST_CHECK(vset->get_scale()                        == 0.5f);
    BOOST_CHECK(vset->get_unit_volume()                  == 0.125f);
    BOOST_CHECK_CLOSE(vset->compute_volume(), 3.375f, result_tolerance);
}

BOOST_AUTO_TEST_CASE( convert_cube_pair_voxel_set_test )
{
    const auto shifted_points(make_cube_pair_points());
    const auto shifted_triangles(make_cube_pair_triangles());
    
    /* Convert */
    volume cubes(shifted_points, shifted_triangles, 3, point_t(0.0f, 0.0f, 0.0f), {{ 1.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, { 0.0f, 0.0f, 1.0f }});
    std::unique_ptr<voxel_set> vset(static_cast<voxel_set *>(cubes.convert(discretisation_type_t::voxel)));

    /* Checks */
    vset->compute_bounding_box();
    BOOST_CHECK(vset->number_of_primitives()             == 15);
    BOOST_CHECK(vset->number_of_primitives_on_surface()  == 15);
    BOOST_CHECK(vset->number_of_primitives_inside()      == 0);
    BOOST_CHECK(vset->get_min_bb()                       == point_t(-1.0f, -1.0f, -1.0f));
    BOOST_CHECK(vset->get_min_bb_voxels()                == point_ti<>(0, 0, 0));
    BOOST_CHECK(vset->get_max_bb_voxels()                == point_ti<>(2, 2, 2));
    BOOST_CHECK(vset->get_barycenter()                   == point_ti<>(1, 1, 1));
    BOOST_CHECK(vset->get_scale()                        == 1.75f);
    BOOST_CHECK(vset->get_unit_volume()                  == 5.359375f);
    BOOST_CHECK_CLOSE(vset->compute_volume(), 80.3906f, result_tolerance);
}

/* Test convert to voxel set */
BOOST_AUTO_TEST_CASE( convert_cube_with_inside_tetrahedron_set_test )
{
    /* Convert */
    volume cube(cube_points, cube_triangles, 3, point_t(0.0f, 0.0f, 0.0f), {{ 1.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, { 0.0f, 0.0f, 1.0f }});
    std::unique_ptr<tetrahedron_set> tset(static_cast<tetrahedron_set *>(cube.convert(discretisation_type_t::tetrahedron)));

    /* Checks */
    tset->compute_bounding_box();
    BOOST_CHECK(tset->number_of_primitives()             == 135);
    BOOST_CHECK(tset->number_of_primitives_on_surface()  == 130);
    BOOST_CHECK(tset->number_of_primitives_inside()      == 5);
    BOOST_CHECK(tset->get_min_bb()                       == point_t(-0.75f, -0.75f, -0.75f));
    BOOST_CHECK(tset->get_barycenter()                   == point_t( 0.0f,   0.0f,   0.0f));
    BOOST_CHECK(tset->get_scale()                        == 0.5f);
    BOOST_CHECK_CLOSE(tset->compute_volume(), 3.375f, result_tolerance);
}

BOOST_AUTO_TEST_CASE( convert_cube_pair_tetrahedron_set_test )
{
    const auto shifted_points(make_cube_pair_points());
    const auto shifted_triangles(make_cube_pair_triangles());
    
    /* Convert */
    volume cubes(shifted_points, shifted_triangles, 3, point_t(0.0f, 0.0f, 0.0f), {{ 1.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, { 0.0f, 0.0f, 1.0f }});
    std::unique_ptr<tetrahedron_set> tset(static_cast<tetrahedron_set *>(cubes.convert(discretisation_type_t::tetrahedron)));

    /* Checks */
    tset->compute_bounding_box();
    BOOST_CHECK(tset->number_of_primitives()             == 75);
    BOOST_CHECK(tset->number_of_primitives_on_surface()  == 75);
    BOOST_CHECK(tset->number_of_primitives_inside()      == 0);
    BOOST_CHECK(tset->get_min_bb()                       == point_t(-1.875f, -1.875f, -1.875f));
    BOOST_CHECK(tset->get_barycenter()                   == point_t( 0.75f,   0.75f,   0.75f));
    BOOST_CHECK(tset->get_scale()                        == 1.75f);
    BOOST_CHECK_CLOSE(tset->compute_volume(), 80.3906f, result_tolerance);
}

/* Test convert to mesh */
BOOST_AUTO_TEST_CASE( convert_cube_with_inside_mesh_test )
{
    /* Convert */
    volume cube(cube_points, cube_triangles, 3, point_t(0.0f, 0.0f, 0.0f), {{ 1.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, { 0.0f, 0.0f, 1.0f }});
    std::unique_ptr<convex_mesh> mesh(cube.convert_to_convex_mesh(voxel_value_t::primitive_on_surface));

    /* Checks */
    BOOST_CHECK(mesh->number_of_points()     == 208);
    BOOST_CHECK(mesh->number_of_triangles()  == 312);
    BOOST_CHECK_CLOSE(mesh->volume(), 3.25f, result_tolerance);
}

BOOST_AUTO_TEST_CASE( convert_cube_pair_mesh_test )
{
    const auto shifted_points(make_cube_pair_points());
    const auto shifted_triangles(make_cube_pair_triangles());
    
    /* Convert */
    volume cubes(shifted_points, shifted_triangles, 3, point_t(0.0f, 0.0f, 0.0f), {{ 1.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, { 0.0f, 0.0f, 1.0f }});
    std::unique_ptr<convex_mesh> mesh(cubes.convert_to_convex_mesh(voxel_value_t::primitive_on_surface));

    /* Checks */
    BOOST_CHECK(mesh->number_of_points()     == 120);
    BOOST_CHECK(mesh->number_of_triangles()  == 180);
    BOOST_CHECK_CLOSE(mesh->volume(), 80.3906f, result_tolerance);
}

/* Test compute principal aces */
BOOST_AUTO_TEST_CASE( cube_compute_principal_axes_test )
{
    volume cube(cube_points, cube_triangles, 2, point_t(0.0f, 0.0f, 0.0f), {{ 1.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, { 0.0f, 0.0f, 1.0f }});

    /* Compute principal axes */
    float rot[3][3];
    cube.compute_principal_axes(rot);

    /* Checks */
    BOOST_CHECK(rot[0][0] == 1.0f);
    BOOST_CHECK(rot[0][1] == 0.0f);
    BOOST_CHECK(rot[0][2] == 0.0f);
    BOOST_CHECK(rot[1][0] == 0.0f);
    BOOST_CHECK(rot[1][1] == 1.0f);
    BOOST_CHECK(rot[1][2] == 0.0f);
    BOOST_CHECK(rot[2][0] == 0.0f);
    BOOST_CHECK(rot[2][1] == 0.0f);
    BOOST_CHECK(rot[2][2] == 1.0f);
}

BOOST_AUTO_TEST_CASE( cube2x_compute_principal_axes_test )
{
    volume cube(cube2x_points, cube_triangles, 2, point_t(1.0f, 5.0f, -7.0f), {{ 1.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, { 0.0f, 0.0f, 1.0f }});

    /* Compute principal axes */
    float rot[3][3];
    cube.compute_principal_axes(rot);

    /* Checks */
    BOOST_CHECK(rot[0][0] == 1.0f);
    BOOST_CHECK(rot[0][1] == 0.0f);
    BOOST_CHECK(rot[0][2] == 0.0f);
    BOOST_CHECK(rot[1][0] == 0.0f);
    BOOST_CHECK(rot[1][1] == 1.0f);
    BOOST_CHECK(rot[1][2] == 0.0f);
    BOOST_CHECK(rot[2][0] == 0.0f);
    BOOST_CHECK(rot[2][1] == 0.0f);
    BOOST_CHECK(rot[2][2] == 1.0f);
}

BOOST_AUTO_TEST_CASE( cubiod_y_compute_principal_axes_test )
{
    volume cube(cubiod_y_points, cube_triangles, 2, point_t(0.0f, 0.0f, 0.0f), {{ 1.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, { 0.0f, 0.0f, 1.0f }});

    /* Compute principal axes */
    float rot[3][3];
    cube.compute_principal_axes(rot);

    /* Checks */
    BOOST_CHECK(rot[0][0] == 1.0f);
    BOOST_CHECK(rot[0][1] == 0.0f);
    BOOST_CHECK(rot[0][2] == 0.0f);
    BOOST_CHECK(rot[1][0] == 0.0f);
    BOOST_CHECK(rot[1][1] == 1.0f);
    BOOST_CHECK(rot[1][2] == 0.0f);
    BOOST_CHECK(rot[2][0] == 0.0f);
    BOOST_CHECK(rot[2][1] == 0.0f);
    BOOST_CHECK(rot[2][2] == 1.0f);
}

BOOST_AUTO_TEST_CASE( cube_pair_compute_principal_axes_test )
{
    const auto shifted_points(make_cube_pair_points());
    const auto shifted_triangles(make_cube_pair_triangles());
    
    /* Compute principal axes */
    float rot[3][3];
    volume cubes(shifted_points, shifted_triangles, 3, point_t(0.0f, 0.0f, 0.0f), {{ 1.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, { 0.0f, 0.0f, 1.0f }});
    cubes.compute_principal_axes(rot);

    /* Checks */
    BOOST_CHECK_CLOSE(rot[0][0],  0.577293f, result_tolerance);
    BOOST_CHECK_CLOSE(rot[0][1], -0.738525f, result_tolerance);
    BOOST_CHECK_CLOSE(rot[0][2], -0.348303f, result_tolerance);
    BOOST_CHECK_CLOSE(rot[1][0],  0.577357f, result_tolerance);
    BOOST_CHECK_CLOSE(rot[1][1],  0.670828f, result_tolerance);
    BOOST_CHECK_CLOSE(rot[1][2], -0.465455f, result_tolerance);
    BOOST_CHECK_CLOSE(rot[2][0],  0.577402f, result_tolerance);
    BOOST_CHECK_CLOSE(rot[2][1],  0.067609f, result_tolerance);
    BOOST_CHECK_CLOSE(rot[2][2],  0.813656f, result_tolerance);
}

/* Test const get voxel */
BOOST_AUTO_TEST_CASE( const_get_voxel_test )
{
    volume cube(cube_points, cube_triangles, 2, point_t(0.0f, 0.0f, 0.0f), {{ 1.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, { 0.0f, 0.0f, 1.0f }});

    const auto *const const_cube = &cube;
    BOOST_CHECK(const_cube->get_voxel(0, 0, 0) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(const_cube->get_voxel(1, 0, 0) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(const_cube->get_voxel(0, 1, 0) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(const_cube->get_voxel(1, 1, 0) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(const_cube->get_voxel(0, 0, 1) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(const_cube->get_voxel(1, 0, 1) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(const_cube->get_voxel(0, 1, 1) == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(const_cube->get_voxel(1, 1, 1) == voxel_value_t::primitive_on_surface);
}

BOOST_AUTO_TEST_SUITE_END()
}; /* namespace test */
}; /* namespace raptor_convex_decomposition */
