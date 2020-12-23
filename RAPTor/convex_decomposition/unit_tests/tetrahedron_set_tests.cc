#ifdef STAND_ALONE
#define BOOST_TEST_MODULE tetrahedron_set test
#endif /* #ifdef STAND_ALONE */


/* Standard headers */
#include <memory>

/* Boost headers */
#include "boost/noncopyable.hpp"
#include "boost/test/unit_test.hpp"

/* Convex Decomposition headers */
#include "tetrahedron_set.h"


namespace raptor_convex_decomposition
{
namespace test
{
/* Test data */
struct tetrahedron_set_fixture : private boost::noncopyable
{
    tetrahedron_set_fixture() :
        empty(   {  }, point_t<>( 3.5f, -2.7f, 1.6f), 1.0f),
        simple(
        {
            tetrahedron(point_t<>( 0.0f,  0.0f,  0.0f), point_t<>( 0.0f,  1.0f,  0.0f), point_t<>( 1.0f,  0.0f,  0.0f), point_t<>( 0.0f,  0.0f,  1.0f))
        }, point_t<>(0.0f, 0.0f, 0.0f), 1.0f),
        cube(
        {
            tetrahedron(point_t<>( 0.5f, -0.5f, -0.5f), point_t<>(-0.5f,  0.5f, -0.5f), point_t<>( 0.5f,  0.5f,  0.5f), point_t<>(-0.5f, -0.5f,  0.5f)),
            tetrahedron(point_t<>( 0.5f, -0.5f,  0.5f), point_t<>( 0.5f, -0.5f, -0.5f), point_t<>( 0.5f,  0.5f,  0.5f), point_t<>(-0.5f, -0.5f,  0.5f)),
            tetrahedron(point_t<>( 0.5f,  0.5f, -0.5f), point_t<>(-0.5f,  0.5f, -0.5f), point_t<>( 0.5f,  0.5f,  0.5f), point_t<>( 0.5f, -0.5f, -0.5f)),
            tetrahedron(point_t<>(-0.5f, -0.5f, -0.5f), point_t<>(-0.5f,  0.5f, -0.5f), point_t<>( 0.5f, -0.5f, -0.5f), point_t<>(-0.5f, -0.5f,  0.5f)),
            tetrahedron(point_t<>(-0.5f,  0.5f,  0.5f), point_t<>(-0.5f, -0.5f,  0.5f), point_t<>( 0.5f,  0.5f,  0.5f), point_t<>(-0.5f,  0.5f, -0.5f))
        }, point_t<>( 3.5f, -2.7f, 1.6f), 1.0f),
        cube2_5x(
        {
            tetrahedron(point_t<>( 0.5f, -0.5f, -0.5f), point_t<>(-0.5f,  0.5f, -0.5f), point_t<>( 0.5f,  0.5f,  0.5f), point_t<>(-0.5f, -0.5f,  0.5f)),
            tetrahedron(point_t<>( 0.5f, -0.5f,  0.5f), point_t<>( 0.5f, -0.5f, -0.5f), point_t<>( 0.5f,  0.5f,  0.5f), point_t<>(-0.5f, -0.5f,  0.5f)),
            tetrahedron(point_t<>( 0.5f,  0.5f, -0.5f), point_t<>(-0.5f,  0.5f, -0.5f), point_t<>( 0.5f,  0.5f,  0.5f), point_t<>( 0.5f, -0.5f, -0.5f)),
            tetrahedron(point_t<>(-0.5f, -0.5f, -0.5f), point_t<>(-0.5f,  0.5f, -0.5f), point_t<>( 0.5f, -0.5f, -0.5f), point_t<>(-0.5f, -0.5f,  0.5f)),
            tetrahedron(point_t<>(-0.5f,  0.5f,  0.5f), point_t<>(-0.5f, -0.5f,  0.5f), point_t<>( 0.5f,  0.5f,  0.5f), point_t<>(-0.5f,  0.5f, -0.5f))
        }, point_t<>(-7.8f,  3.9f, 4.6f), 2.5f),
        cube3x(
        {
            tetrahedron(point_t<>( 0.5f, -0.5f, -0.5f), point_t<>(-0.5f,  0.5f, -0.5f), point_t<>( 0.5f,  0.5f,  0.5f), point_t<>(-0.5f, -0.5f,  0.5f)),
            tetrahedron(point_t<>( 0.5f, -0.5f,  0.5f), point_t<>( 0.5f, -0.5f, -0.5f), point_t<>( 0.5f,  0.5f,  0.5f), point_t<>(-0.5f, -0.5f,  0.5f)),
            tetrahedron(point_t<>( 0.5f,  0.5f, -0.5f), point_t<>(-0.5f,  0.5f, -0.5f), point_t<>( 0.5f,  0.5f,  0.5f), point_t<>( 0.5f, -0.5f, -0.5f)),
            tetrahedron(point_t<>(-0.5f, -0.5f, -0.5f), point_t<>(-0.5f,  0.5f, -0.5f), point_t<>( 0.5f, -0.5f, -0.5f), point_t<>(-0.5f, -0.5f,  0.5f)),
            tetrahedron(point_t<>(-0.5f,  0.5f,  0.5f), point_t<>(-0.5f, -0.5f,  0.5f), point_t<>( 0.5f,  0.5f,  0.5f), point_t<>(-0.5f,  0.5f, -0.5f))
        }, point_t<>(-7.8f,  3.9f, 4.6f), 3.0f),
        cubiod(
        {
            tetrahedron(point_t<>( 2.0f, -0.5f, -0.5f), point_t<>(-2.0f,  0.5f, -0.5f), point_t<>( 2.0f,  0.5f,  0.5f), point_t<>(-2.0f, -0.5f,  0.5f)),
            tetrahedron(point_t<>( 2.0f, -0.5f,  0.5f), point_t<>( 2.0f, -0.5f, -0.5f), point_t<>( 2.0f,  0.5f,  0.5f), point_t<>(-2.0f, -0.5f,  0.5f)),
            tetrahedron(point_t<>( 2.0f,  0.5f, -0.5f), point_t<>(-2.0f,  0.5f, -0.5f), point_t<>( 2.0f,  0.5f,  0.5f), point_t<>( 2.0f, -0.5f, -0.5f)),
            tetrahedron(point_t<>(-2.0f, -0.5f, -0.5f), point_t<>(-2.0f,  0.5f, -0.5f), point_t<>( 2.0f, -0.5f, -0.5f), point_t<>(-2.0f, -0.5f,  0.5f)),
            tetrahedron(point_t<>(-2.0f,  0.5f,  0.5f), point_t<>(-2.0f, -0.5f,  0.5f), point_t<>( 2.0f,  0.5f,  0.5f), point_t<>(-2.0f,  0.5f, -0.5f))
        }, point_t<>( 3.5f, -2.7f, 1.6f), 1.0f),
        cube_rot(
        {
            tetrahedron(point_t<>(-0.0857864f, 0.5f, -0.914214f), point_t<>(-0.914214f, -0.0857864f, 0.5f), point_t<>( 0.5f,  0.5f,  0.5f), point_t<>(0.5f, -0.914214f, -0.0857864f)),
            tetrahedron(point_t<>(0.914214f, 0.0857864f, -0.5f), point_t<>(-0.0857864f, 0.5f, -0.914214f), point_t<>( 0.5f,  0.5f,  0.5f), point_t<>(0.5f, -0.914214f, -0.0857864f)),
            tetrahedron(point_t<>(-0.5f, 0.914214f, 0.0857864f), point_t<>(-0.914214f, -0.0857864f, 0.5f), point_t<>( 0.5f,  0.5f,  0.5f), point_t<>(-0.0857864f, 0.5f, -0.914214f)),
            tetrahedron(point_t<>(-0.5f, -0.5f, -0.5f), point_t<>(-0.914214f, -0.0857864f, 0.5f), point_t<>(-0.0857864f, 0.5f, -0.914214f), point_t<>(0.5f, -0.914214f, -0.0857864f)),
            tetrahedron(point_t<>( 0.0857864f, -0.5f, 0.914214f), point_t<>(0.5f, -0.914214f, -0.0857864f), point_t<>( 0.5f,  0.5f,  0.5f), point_t<>(-0.914214f, -0.0857864f, 0.5f))
        }, point_t<>( 3.5f, -2.7f, 1.6f), 1.0f),
        diagonal(
        {
            tetrahedron(point_t<>(  6.5f,   7.5f,    3.5f), point_t<>(  5.5f,   8.5f,    3.5f), point_t<>(  6.5f,   8.5f,    4.5f), point_t<>(  5.5f,   7.5f,    4.5f)),
            tetrahedron(point_t<>(  8.5f,  11.5f,    7.5f), point_t<>(  7.5f,  12.5f,    7.5f), point_t<>(  8.5f,  12.5f,    8.5f), point_t<>(  7.5f,  11.5f,    8.5f)),
            tetrahedron(point_t<>( 10.5f,  15.5f,   15.5f), point_t<>(  9.5f,  16.5f,   15.5f), point_t<>( 10.5f,  16.5f,   16.5f), point_t<>(  9.5f,  15.5f,   16.5f)),
            tetrahedron(point_t<>( 12.5f,  19.5f,   31.5f), point_t<>( 11.5f,  20.5f,   31.5f), point_t<>( 12.5f,  20.5f,   32.5f), point_t<>( 11.5f,  19.5f,   32.5f)),
            tetrahedron(point_t<>( 14.5f,  23.5f,   63.5f), point_t<>( 13.5f,  24.5f,   63.5f), point_t<>( 14.5f,  24.5f,   64.5f), point_t<>( 13.5f,  23.5f,   64.5f)),
            tetrahedron(point_t<>( 16.5f,  27.5f,  127.5f), point_t<>( 15.5f,  28.5f,  127.5f), point_t<>( 16.5f,  28.5f,  128.5f), point_t<>( 15.5f,  27.5f,  128.5f))
        }, point_t<>(-7.8f,  3.9f, 4.6f), 2.0f),
        row(
        {
            tetrahedron(point_t<>( 0.5f, -0.5f, -0.5f), point_t<>(-0.5f,  0.5f, -0.5f), point_t<>( 0.5f,  0.5f,  0.5f), point_t<>(-0.5f, -0.5f,  0.5f)),
            tetrahedron(point_t<>( 1.5f, -0.5f,  0.5f), point_t<>( 1.5f, -0.5f, -0.5f), point_t<>( 1.5f,  0.5f,  0.5f), point_t<>( 0.5f, -0.5f,  0.5f)),
            tetrahedron(point_t<>( 2.5f,  0.5f, -0.5f), point_t<>( 1.5f,  0.5f, -0.5f), point_t<>( 2.5f,  0.5f,  0.5f), point_t<>( 2.5f, -0.5f, -0.5f)),
            tetrahedron(point_t<>( 2.5f, -0.5f, -0.5f), point_t<>( 2.5f,  0.5f, -0.5f), point_t<>( 3.5f, -0.5f, -0.5f), point_t<>( 2.5f, -0.5f,  0.5f)),
            tetrahedron(point_t<>( 3.5f,  0.5f,  0.5f), point_t<>( 3.5f, -0.5f,  0.5f), point_t<>( 4.5f,  0.5f,  0.5f), point_t<>( 3.5f,  0.5f, -0.5f)),
            tetrahedron(point_t<>( 5.5f,  0.5f, -0.5f), point_t<>( 4.5f,  0.5f, -0.5f), point_t<>( 5.5f,  0.5f,  0.5f), point_t<>( 5.5f, -0.5f, -0.5f)),
            tetrahedron(point_t<>( 5.5f, -0.5f, -0.5f), point_t<>( 5.5f,  0.5f, -0.5f), point_t<>( 6.5f, -0.5f, -0.5f), point_t<>( 5.5f, -0.5f,  0.5f)),
            tetrahedron(point_t<>( 6.5f,  0.5f,  0.5f), point_t<>( 6.5f, -0.5f,  0.5f), point_t<>( 7.5f,  0.5f,  0.5f), point_t<>( 6.5f,  0.5f, -0.5f))
        }, point_t<>(1.0f, 2.0f, 3.0f), 1.0f),
        on_surface(
        {
            tetrahedron(point_t<>( 0.5f, -0.5f, -0.5f), point_t<>(-0.5f,  0.5f, -0.5f), point_t<>( 0.5f,  0.5f,  0.5f), point_t<>(-0.5f, -0.5f,  0.5f), voxel_value_t::primitive_on_surface),
            tetrahedron(point_t<>( 1.5f, -0.5f,  0.5f), point_t<>( 1.5f, -0.5f, -0.5f), point_t<>( 1.5f,  0.5f,  0.5f), point_t<>( 0.5f, -0.5f,  0.5f), voxel_value_t::primitive_on_surface),
            tetrahedron(point_t<>( 2.5f,  0.5f, -0.5f), point_t<>( 1.5f,  0.5f, -0.5f), point_t<>( 2.5f,  0.5f,  0.5f), point_t<>( 2.5f, -0.5f, -0.5f), voxel_value_t::primitive_undefined),
            tetrahedron(point_t<>( 2.5f, -0.5f, -0.5f), point_t<>( 2.5f,  0.5f, -0.5f), point_t<>( 3.5f, -0.5f, -0.5f), point_t<>( 2.5f, -0.5f,  0.5f), voxel_value_t::primitive_outside_surface),
            tetrahedron(point_t<>( 3.5f,  0.5f,  0.5f), point_t<>( 3.5f, -0.5f,  0.5f), point_t<>( 4.5f,  0.5f,  0.5f), point_t<>( 3.5f,  0.5f, -0.5f), voxel_value_t::primitive_inside_surface),
            tetrahedron(point_t<>( 5.5f,  0.5f, -0.5f), point_t<>( 4.5f,  0.5f, -0.5f), point_t<>( 5.5f,  0.5f,  0.5f), point_t<>( 5.5f, -0.5f, -0.5f), voxel_value_t::primitive_on_surface),
            tetrahedron(point_t<>( 5.5f, -0.5f, -0.5f), point_t<>( 5.5f,  0.5f, -0.5f), point_t<>( 6.5f, -0.5f, -0.5f), point_t<>( 5.5f, -0.5f,  0.5f), voxel_value_t::primitive_undefined),
            tetrahedron(point_t<>( 6.5f,  0.5f,  0.5f), point_t<>( 6.5f, -0.5f,  0.5f), point_t<>( 7.5f,  0.5f,  0.5f), point_t<>( 6.5f,  0.5f, -0.5f), voxel_value_t::primitive_undefined)
        }, point_t<>(0.5f, -0.5f, -0.5f), 1.0f)
    {  }

    tetrahedron_set empty;
    tetrahedron_set simple;
    tetrahedron_set cube;
    tetrahedron_set cube2_5x;
    tetrahedron_set cube3x;
    tetrahedron_set cubiod;
    tetrahedron_set cube_rot;
    tetrahedron_set diagonal;
    tetrahedron_set row;
    tetrahedron_set on_surface;
};


BOOST_FIXTURE_TEST_SUITE( tetrahedron_set_tests, tetrahedron_set_fixture )

const float result_tolerance = 0.0005f;


/* Test Ctor */
BOOST_AUTO_TEST_CASE( ctor_test )
{
    BOOST_CHECK(cube.get_min_bb()                           == point_t<>(3.5f, -2.7f, 1.6f));
    BOOST_CHECK(cube.get_max_bb()                           == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(cube.get_barycenter()                       == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(cube.compute_volume()                       == 1.0f);
    BOOST_CHECK(cube.max_volume_error()                     == 0.0f);
    BOOST_CHECK(cube.get_scale()                            == 1.0f);
    BOOST_CHECK(cube.number_of_primitives()                 == 5);
    BOOST_CHECK(cube.number_of_primitives_on_surface()      == 0);
    BOOST_CHECK(cube.number_of_primitives_inside()          == 0);

    BOOST_CHECK(cube3x.get_min_bb()                         == point_t<>(-7.8f, 3.9f, 4.6f));
    BOOST_CHECK(cube3x.get_max_bb()                         == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(cube3x.get_barycenter()                     == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(cube3x.compute_volume()                     == 1.0f);
    BOOST_CHECK(cube3x.max_volume_error()                   == 0.0f);
    BOOST_CHECK(cube3x.get_scale()                          == 3.0f);
    BOOST_CHECK(cube3x.number_of_primitives()               == 5);
    BOOST_CHECK(cube3x.number_of_primitives_on_surface()    == 0);
    BOOST_CHECK(cube3x.number_of_primitives_inside()        == 0);

    BOOST_CHECK(diagonal.get_min_bb()                       == point_t<>(-7.8f, 3.9f, 4.6f));
    BOOST_CHECK(diagonal.get_max_bb()                       == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(diagonal.get_barycenter()                   == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(diagonal.compute_volume()                   == 2.0f);
    BOOST_CHECK(diagonal.max_volume_error()                 == 0.0f);
    BOOST_CHECK(diagonal.get_scale()                        == 2.0f);
    BOOST_CHECK(diagonal.number_of_primitives()             == 6);
    BOOST_CHECK(diagonal.number_of_primitives_on_surface()  == 0);
    BOOST_CHECK(diagonal.number_of_primitives_inside()      == 0);
}

/* Test get voxels */
BOOST_AUTO_TEST_CASE( const_get_voxels_test )
{
    const auto *const const_diagonal = &diagonal;
    const auto &v = const_diagonal->get_tetrahedra();
    BOOST_REQUIRE(v.size() == 6);
    BOOST_CHECK(v[0].pts[0] == point_t<>(  6.5f,   7.5f,    3.5f));
    BOOST_CHECK(v[0].pts[1] == point_t<>(  5.5f,   8.5f,    3.5f));
    BOOST_CHECK(v[0].pts[2] == point_t<>(  6.5f,   8.5f,    4.5f));
    BOOST_CHECK(v[0].pts[3] == point_t<>(  5.5f,   7.5f,    4.5f));
    BOOST_CHECK(v[1].pts[0] == point_t<>(  8.5f,  11.5f,    7.5f));
    BOOST_CHECK(v[1].pts[1] == point_t<>(  7.5f,  12.5f,    7.5f));
    BOOST_CHECK(v[1].pts[2] == point_t<>(  8.5f,  12.5f,    8.5f));
    BOOST_CHECK(v[1].pts[3] == point_t<>(  7.5f,  11.5f,    8.5f));
    BOOST_CHECK(v[2].pts[0] == point_t<>( 10.5f,  15.5f,   15.5f));
    BOOST_CHECK(v[2].pts[1] == point_t<>(  9.5f,  16.5f,   15.5f));
    BOOST_CHECK(v[2].pts[2] == point_t<>( 10.5f,  16.5f,   16.5f));
    BOOST_CHECK(v[2].pts[3] == point_t<>(  9.5f,  15.5f,   16.5f));
    BOOST_CHECK(v[3].pts[0] == point_t<>( 12.5f,  19.5f,   31.5f));
    BOOST_CHECK(v[3].pts[1] == point_t<>( 11.5f,  20.5f,   31.5f));
    BOOST_CHECK(v[3].pts[2] == point_t<>( 12.5f,  20.5f,   32.5f));
    BOOST_CHECK(v[3].pts[3] == point_t<>( 11.5f,  19.5f,   32.5f));
    BOOST_CHECK(v[4].pts[0] == point_t<>( 14.5f,  23.5f,   63.5f));
    BOOST_CHECK(v[4].pts[1] == point_t<>( 13.5f,  24.5f,   63.5f));
    BOOST_CHECK(v[4].pts[2] == point_t<>( 14.5f,  24.5f,   64.5f));
    BOOST_CHECK(v[4].pts[3] == point_t<>( 13.5f,  23.5f,   64.5f));
    BOOST_CHECK(v[5].pts[0] == point_t<>( 16.5f,  27.5f,  127.5f));
    BOOST_CHECK(v[5].pts[1] == point_t<>( 15.5f,  28.5f,  127.5f));
    BOOST_CHECK(v[5].pts[2] == point_t<>( 16.5f,  28.5f,  128.5f));
    BOOST_CHECK(v[5].pts[3] == point_t<>( 15.5f,  27.5f,  128.5f));
}

BOOST_AUTO_TEST_CASE( get_voxels_test )
{
    auto &v =  diagonal.get_tetrahedra();
    BOOST_REQUIRE(v.size() == 6);
    BOOST_CHECK(v[0].pts[0] == point_t<>(  6.5f,   7.5f,    3.5f));
    BOOST_CHECK(v[0].pts[1] == point_t<>(  5.5f,   8.5f,    3.5f));
    BOOST_CHECK(v[0].pts[2] == point_t<>(  6.5f,   8.5f,    4.5f));
    BOOST_CHECK(v[0].pts[3] == point_t<>(  5.5f,   7.5f,    4.5f));
    BOOST_CHECK(v[1].pts[0] == point_t<>(  8.5f,  11.5f,    7.5f));
    BOOST_CHECK(v[1].pts[1] == point_t<>(  7.5f,  12.5f,    7.5f));
    BOOST_CHECK(v[1].pts[2] == point_t<>(  8.5f,  12.5f,    8.5f));
    BOOST_CHECK(v[1].pts[3] == point_t<>(  7.5f,  11.5f,    8.5f));
    BOOST_CHECK(v[2].pts[0] == point_t<>( 10.5f,  15.5f,   15.5f));
    BOOST_CHECK(v[2].pts[1] == point_t<>(  9.5f,  16.5f,   15.5f));
    BOOST_CHECK(v[2].pts[2] == point_t<>( 10.5f,  16.5f,   16.5f));
    BOOST_CHECK(v[2].pts[3] == point_t<>(  9.5f,  15.5f,   16.5f));
    BOOST_CHECK(v[3].pts[0] == point_t<>( 12.5f,  19.5f,   31.5f));
    BOOST_CHECK(v[3].pts[1] == point_t<>( 11.5f,  20.5f,   31.5f));
    BOOST_CHECK(v[3].pts[2] == point_t<>( 12.5f,  20.5f,   32.5f));
    BOOST_CHECK(v[3].pts[3] == point_t<>( 11.5f,  19.5f,   32.5f));
    BOOST_CHECK(v[4].pts[0] == point_t<>( 14.5f,  23.5f,   63.5f));
    BOOST_CHECK(v[4].pts[1] == point_t<>( 13.5f,  24.5f,   63.5f));
    BOOST_CHECK(v[4].pts[2] == point_t<>( 14.5f,  24.5f,   64.5f));
    BOOST_CHECK(v[4].pts[3] == point_t<>( 13.5f,  23.5f,   64.5f));
    BOOST_CHECK(v[5].pts[0] == point_t<>( 16.5f,  27.5f,  127.5f));
    BOOST_CHECK(v[5].pts[1] == point_t<>( 15.5f,  28.5f,  127.5f));
    BOOST_CHECK(v[5].pts[2] == point_t<>( 16.5f,  28.5f,  128.5f));
    BOOST_CHECK(v[5].pts[3] == point_t<>( 15.5f,  27.5f,  128.5f));
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

/* Test reserve */
BOOST_AUTO_TEST_CASE( reserve_test )
{
    cube.reserve(1);
    cube3x.reserve(2);
    diagonal.reserve(3);
}

/* Test add */
BOOST_AUTO_TEST_CASE( add_too_small_test )
{
    BOOST_CHECK(!empty.add(tetrahedron(point_t<>( 0.0f,  0.0f,  0.0f), point_t<>( 0.0f,  0.0f,  0.0f), point_t<>( 0.0f,  0.0f,  0.0f), point_t<>( 0.0f,  0.0f,  0.0f))));
    BOOST_CHECK(empty.compute_volume()                  == 0.0f);
    BOOST_CHECK(empty.max_volume_error()                == 0.0f);
    BOOST_CHECK(empty.number_of_primitives()            == 0);
    BOOST_CHECK(empty.number_of_primitives_on_surface() == 0);
    BOOST_CHECK(empty.number_of_primitives_inside()     == 0);
}

BOOST_AUTO_TEST_CASE( add_test )
{
    BOOST_CHECK(empty.add(tetrahedron(point_t<>( 0.0f,  0.0f,  0.0f), point_t<>( 0.0f,  1.0f,  0.0f), point_t<>( 1.0f,  0.0f,  0.0f), point_t<>( 0.0f,  0.0f,  1.0f))));
    BOOST_CHECK_CLOSE(empty.compute_volume(), 0.166667f, result_tolerance);
    BOOST_CHECK(empty.max_volume_error()                == 0.0f);
    BOOST_CHECK(empty.number_of_primitives()            == 1);
    BOOST_CHECK(empty.number_of_primitives_on_surface() == 0);
    BOOST_CHECK(empty.number_of_primitives_inside()     == 0);

    BOOST_CHECK(empty.add(tetrahedron(point_t<>( 0.0f,  0.0f,  0.0f), point_t<>( 0.0f,  1.0f,  0.0f), point_t<>( 1.0f,  0.0f,  0.0f), point_t<>( 0.0f,  0.0f,  1.0f), voxel_value_t::primitive_on_surface)));
    BOOST_CHECK_CLOSE(empty.compute_volume(), 0.333333f, result_tolerance);
    BOOST_CHECK_CLOSE(empty.max_volume_error(), 0.166667f, result_tolerance);
    BOOST_CHECK(empty.number_of_primitives()            == 2);
    BOOST_CHECK(empty.number_of_primitives_on_surface() == 1);
    BOOST_CHECK(empty.number_of_primitives_inside()     == 0);

    BOOST_CHECK(empty.add(tetrahedron(point_t<>( 0.0f,  0.0f,  0.0f), point_t<>( 0.0f,  1.0f,  0.0f), point_t<>( 1.0f,  0.0f,  0.0f), point_t<>( 0.0f,  0.0f,  1.0f), voxel_value_t::primitive_inside_surface)));
    BOOST_CHECK_CLOSE(empty.compute_volume(), 0.5f, result_tolerance);
    BOOST_CHECK_CLOSE(empty.max_volume_error(), 0.166667f, result_tolerance);
    BOOST_CHECK(empty.number_of_primitives()            == 3);
    BOOST_CHECK(empty.number_of_primitives_on_surface() == 1);
    BOOST_CHECK(empty.number_of_primitives_inside()     == 1);
}

BOOST_AUTO_TEST_CASE( add_inside_out_test )
{
    BOOST_CHECK(empty.add(tetrahedron(point_t<>( 0.0f,  1.0f,  0.0f), point_t<>( 0.0f,  0.0f,  0.0f), point_t<>( 1.0f,  0.0f,  0.0f), point_t<>( 0.0f,  0.0f,  1.0f))));
    BOOST_CHECK_CLOSE(empty.compute_volume(), 0.166667f, result_tolerance);
    BOOST_CHECK(empty.max_volume_error()                == 0.0f);
    BOOST_CHECK(empty.number_of_primitives()            == 1);
    BOOST_CHECK(empty.number_of_primitives_on_surface() == 0);
    BOOST_CHECK(empty.number_of_primitives_inside()     == 0);
}

/* Test compute bounding box */
BOOST_AUTO_TEST_CASE( compute_bounding_box_test )
{
    empty.compute_bounding_box();
    BOOST_CHECK(empty.get_barycenter()      == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(empty.get_min_bb()          == point_t<>( 3.5f, -2.7f,  1.6f));
    BOOST_CHECK(empty.get_max_bb()          == point_t<>( 0.0f,  0.0f,  0.0f));

    cube.compute_bounding_box();
    BOOST_CHECK(cube.get_barycenter()       == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(cube.get_min_bb()           == point_t<>(-0.5f, -0.5f, -0.5f));
    BOOST_CHECK(cube.get_max_bb()           == point_t<>( 0.5f,  0.5f,  0.5f));

    diagonal.compute_bounding_box();
    BOOST_CHECK(diagonal.get_barycenter()   == point_t<>(11.0f, 18.0f, 42.0f));
    BOOST_CHECK(diagonal.get_min_bb()       == point_t<>( 5.5f,  7.5f,  3.5f));
    BOOST_CHECK(diagonal.get_max_bb()       == point_t<>(16.5f, 28.5f, 128.5f));
}

/* Test max volume error */
BOOST_AUTO_TEST_CASE( max_volume_error_test )
{
    BOOST_CHECK(empty.max_volume_error()    == 0.0f);
    BOOST_CHECK(cube.max_volume_error()     == 0.0f);
    BOOST_CHECK_CLOSE(on_surface.max_volume_error(), 0.666667f, result_tolerance);
}

/* Test compute principal axes */
BOOST_AUTO_TEST_CASE( compute_principal_axes_test )
{
    cube.compute_bounding_box();
    cube.compute_principal_axes();
    BOOST_CHECK(cube.eigen_value(axis_t::x_axis) == 0.25f);
    BOOST_CHECK(cube.eigen_value(axis_t::y_axis) == 0.25f);
    BOOST_CHECK(cube.eigen_value(axis_t::z_axis) == 0.25f);

    row.compute_bounding_box();
    row.compute_principal_axes();
    BOOST_CHECK_CLOSE(row.eigen_value(axis_t::x_axis), 4.97208f,            result_tolerance);
    BOOST_CHECK_CLOSE(row.eigen_value(axis_t::y_axis), 0.244312f,           result_tolerance);
    BOOST_CHECK_CLOSE(row.eigen_value(axis_t::z_axis), 0.249432f,           result_tolerance);

    diagonal.compute_bounding_box();
    diagonal.compute_principal_axes();
    BOOST_CHECK_CLOSE(diagonal.eigen_value(axis_t::x_axis),    0.250008f,   result_tolerance);
    BOOST_CHECK_CLOSE(diagonal.eigen_value(axis_t::y_axis),   10.4642f,     result_tolerance);
    BOOST_CHECK_CLOSE(diagonal.eigen_value(axis_t::z_axis), 1924.37f,       result_tolerance);
}

/* Test select on surface */
BOOST_AUTO_TEST_CASE( select_on_surface_empty_test )
{
    std::unique_ptr<tetrahedron_set> on_surf(empty.select_on_surface());
    BOOST_CHECK(on_surf == nullptr);
}

BOOST_AUTO_TEST_CASE( select_on_surface_test )
{
    std::unique_ptr<tetrahedron_set> on_surf(on_surface.select_on_surface());
    BOOST_REQUIRE(on_surf != nullptr);

    const auto &v = on_surf->get_tetrahedra();
    BOOST_REQUIRE(v.size() == 3);
    BOOST_CHECK(v[0].pts[0] == point_t<>( 0.5f, -0.5f, -0.5f));
    BOOST_CHECK(v[0].pts[1] == point_t<>(-0.5f,  0.5f, -0.5f));
    BOOST_CHECK(v[0].pts[2] == point_t<>( 0.5f,  0.5f,  0.5f));
    BOOST_CHECK(v[0].pts[3] == point_t<>(-0.5f, -0.5f,  0.5f));

    BOOST_CHECK(v[1].pts[0] == point_t<>( 1.5f, -0.5f,  0.5f));
    BOOST_CHECK(v[1].pts[1] == point_t<>( 1.5f, -0.5f, -0.5f));
    BOOST_CHECK(v[1].pts[2] == point_t<>( 1.5f,  0.5f,  0.5f));
    BOOST_CHECK(v[1].pts[3] == point_t<>( 0.5f, -0.5f,  0.5f));

    BOOST_CHECK(v[2].pts[0] == point_t<>( 5.5f,  0.5f, -0.5f));
    BOOST_CHECK(v[2].pts[1] == point_t<>( 4.5f,  0.5f, -0.5f));
    BOOST_CHECK(v[2].pts[2] == point_t<>( 5.5f,  0.5f,  0.5f));
    BOOST_CHECK(v[2].pts[3] == point_t<>( 5.5f, -0.5f, -0.5f));


    BOOST_CHECK(v[0].loc == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(v[1].loc == voxel_value_t::primitive_on_surface);
    BOOST_CHECK(v[2].loc == voxel_value_t::primitive_on_surface);
}

/* Test cut */
BOOST_AUTO_TEST_CASE( cut_below_test )
{
    primitive_set *pos_part_p = nullptr;
    primitive_set *neg_part_p = nullptr;
    simple.cut(plane(point_t<>(0.0f, 0.0f, 1.0f), -1.0f, axis_t::x_axis), &pos_part_p, &neg_part_p);
    tetrahedron_set* pos_part = static_cast<tetrahedron_set *>(pos_part_p);
    tetrahedron_set* neg_part = static_cast<tetrahedron_set *>(neg_part_p);

    BOOST_CHECK(pos_part->number_of_primitives_on_surface()  == 0);
    BOOST_CHECK(pos_part->number_of_primitives_inside()      == 0);
    BOOST_CHECK(pos_part->max_volume_error()                 == 0.0f);
    BOOST_REQUIRE(pos_part->number_of_primitives()           == 1);
    BOOST_CHECK(pos_part->get_tetrahedra()[0].pts[0]         == point_t<>( 0.0f,  0.0f,  0.0f));
    BOOST_CHECK(pos_part->get_tetrahedra()[0].pts[1]         == point_t<>( 0.0f,  1.0f,  0.0f));
    BOOST_CHECK(pos_part->get_tetrahedra()[0].pts[2]         == point_t<>( 1.0f,  0.0f,  0.0f));
    BOOST_CHECK(pos_part->get_tetrahedra()[0].pts[3]         == point_t<>( 0.0f,  0.0f,  1.0f));

    BOOST_CHECK(neg_part->number_of_primitives_on_surface()  == 0);
    BOOST_CHECK(neg_part->number_of_primitives_inside()      == 0);
    BOOST_CHECK(neg_part->max_volume_error()                 == 0.0f);
    BOOST_REQUIRE(neg_part->number_of_primitives()           == 0);

    delete pos_part;
    delete neg_part;
}

BOOST_AUTO_TEST_CASE( cut_above_test )
{
    primitive_set *pos_part_p = nullptr;
    primitive_set *neg_part_p = nullptr;
    simple.cut(plane(point_t<>(0.0f, 0.0f, 1.0f), 3.0f, axis_t::x_axis), &pos_part_p, &neg_part_p);
    tetrahedron_set* pos_part = static_cast<tetrahedron_set *>(pos_part_p);
    tetrahedron_set* neg_part = static_cast<tetrahedron_set *>(neg_part_p);

    BOOST_CHECK(pos_part->number_of_primitives_on_surface()  == 0);
    BOOST_CHECK(pos_part->number_of_primitives_inside()      == 0);
    BOOST_CHECK(pos_part->max_volume_error()                 == 0.0f);
    BOOST_REQUIRE(pos_part->number_of_primitives()           == 0);

    BOOST_CHECK(neg_part->number_of_primitives_on_surface()  == 0);
    BOOST_CHECK(neg_part->number_of_primitives_inside()      == 0);
    BOOST_CHECK(neg_part->max_volume_error()                 == 0.0f);
    BOOST_REQUIRE(neg_part->number_of_primitives()           == 1);
    BOOST_CHECK(neg_part->get_tetrahedra()[0].pts[0]         == point_t<>( 0.0f,  0.0f,  0.0f));
    BOOST_CHECK(neg_part->get_tetrahedra()[0].pts[1]         == point_t<>( 0.0f,  1.0f,  0.0f));
    BOOST_CHECK(neg_part->get_tetrahedra()[0].pts[2]         == point_t<>( 1.0f,  0.0f,  0.0f));
    BOOST_CHECK(neg_part->get_tetrahedra()[0].pts[3]         == point_t<>( 0.0f,  0.0f,  1.0f));

    delete pos_part;
    delete neg_part;
}

BOOST_AUTO_TEST_CASE( cut_split_4_6_test )
{
    primitive_set *pos_part_p = nullptr;
    primitive_set *neg_part_p = nullptr;
    simple.cut(plane(point_t<>(0.0f, 0.0f, 1.0f), 0.5f, axis_t::x_axis), &pos_part_p, &neg_part_p);
    tetrahedron_set* pos_part = static_cast<tetrahedron_set *>(pos_part_p);
    tetrahedron_set* neg_part = static_cast<tetrahedron_set *>(neg_part_p);

    BOOST_CHECK(pos_part->number_of_primitives_on_surface()  == 1);
    BOOST_CHECK(pos_part->number_of_primitives_inside()      == 0);
    BOOST_CHECK_CLOSE(pos_part->max_volume_error(), 0.0208333f, result_tolerance);
    BOOST_REQUIRE(pos_part->number_of_primitives()           == 1);
    BOOST_CHECK(pos_part->get_tetrahedra()[0].pts[0]         == point_t<>(0.0f, 0.0f, 0.5f));
    BOOST_CHECK(pos_part->get_tetrahedra()[0].pts[1]         == point_t<>(0.0f, 0.0f, 1.0f));
    BOOST_CHECK(pos_part->get_tetrahedra()[0].pts[2]         == point_t<>(0.0f, 0.5f, 0.5f));
    BOOST_CHECK(pos_part->get_tetrahedra()[0].pts[3]         == point_t<>(0.5f, 0.0f, 0.5f));

    
    BOOST_CHECK(neg_part->number_of_primitives_on_surface()  == 3);
    BOOST_CHECK(neg_part->number_of_primitives_inside()      == 0);
    BOOST_CHECK_CLOSE(neg_part->max_volume_error(), 0.145833f, result_tolerance);
    BOOST_REQUIRE(neg_part->number_of_primitives()           == 3);
    BOOST_CHECK(neg_part->get_tetrahedra()[0].pts[0]         == point_t<>(0.0f, 0.0f, 0.0f));
    BOOST_CHECK(neg_part->get_tetrahedra()[0].pts[1]         == point_t<>(0.0f, 1.0f, 0.0f));
    BOOST_CHECK(neg_part->get_tetrahedra()[0].pts[2]         == point_t<>(1.0f, 0.0f, 0.0f));
    BOOST_CHECK(neg_part->get_tetrahedra()[0].pts[3]         == point_t<>(0.5f, 0.0f, 0.5f));

    BOOST_CHECK(neg_part->get_tetrahedra()[1].pts[0]         == point_t<>(0.0f, 0.0f, 0.5f));
    BOOST_CHECK(neg_part->get_tetrahedra()[1].pts[1]         == point_t<>(0.5f, 0.0f, 0.5f));
    BOOST_CHECK(neg_part->get_tetrahedra()[1].pts[2]         == point_t<>(0.0f, 1.0f, 0.0f));
    BOOST_CHECK(neg_part->get_tetrahedra()[1].pts[3]         == point_t<>(0.0f, 0.0f, 0.0f));

    BOOST_CHECK(neg_part->get_tetrahedra()[2].pts[0]         == point_t<>(0.0f, 0.5f, 0.5f));
    BOOST_CHECK(neg_part->get_tetrahedra()[2].pts[1]         == point_t<>(0.0f, 0.0f, 0.5f));
    BOOST_CHECK(neg_part->get_tetrahedra()[2].pts[2]         == point_t<>(0.5f, 0.0f, 0.5f));
    BOOST_CHECK(neg_part->get_tetrahedra()[2].pts[3]         == point_t<>(0.0f, 1.0f, 0.0f));

    delete pos_part;
    delete neg_part;
}

BOOST_AUTO_TEST_CASE( cut_split_6_6_points_1_2_test )
{
    primitive_set *pos_part_p = nullptr;
    primitive_set *neg_part_p = nullptr;
    simple.cut(plane(point_t<>(std::sqrt(2.0f), std::sqrt(2.0f), 0.0f), 0.5f, axis_t::x_axis), &pos_part_p, &neg_part_p);
    tetrahedron_set* pos_part = static_cast<tetrahedron_set *>(pos_part_p);
    tetrahedron_set* neg_part = static_cast<tetrahedron_set *>(neg_part_p);
    
    BOOST_CHECK(pos_part->number_of_primitives_on_surface()  == 3);
    BOOST_CHECK(pos_part->number_of_primitives_inside()      == 0);
    BOOST_CHECK_CLOSE(pos_part->max_volume_error(), 0.118898f, result_tolerance);
    BOOST_REQUIRE(pos_part->number_of_primitives()           == 3);
    BOOST_CHECK(std::fabs(magnitude(pos_part->get_tetrahedra()[0].pts[0] - point_t<>(0.0f,      1.0f,      0.0f     ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(pos_part->get_tetrahedra()[0].pts[1] - point_t<>(1.0f,      0.0f,      0.0f     ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(pos_part->get_tetrahedra()[0].pts[2] - point_t<>(0.353553f, 0.0f,      0.0f     ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(pos_part->get_tetrahedra()[0].pts[3] - point_t<>(0.353553f, 0.0f,      0.646447f))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(pos_part->get_tetrahedra()[1].pts[0] - point_t<>(0.0f,      0.353553f, 0.0f     ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(pos_part->get_tetrahedra()[1].pts[1] - point_t<>(0.353553f, 0.0f,      0.646447f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(pos_part->get_tetrahedra()[1].pts[2] - point_t<>(0.0f,      1.0f,      0.0f     ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(pos_part->get_tetrahedra()[1].pts[3] - point_t<>(0.353553f, 0.0f,      0.0f     ))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(pos_part->get_tetrahedra()[2].pts[0] - point_t<>(0.0f,      0.353553f, 0.646447f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(pos_part->get_tetrahedra()[2].pts[1] - point_t<>(0.0f,      0.353553f, 0.0f     ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(pos_part->get_tetrahedra()[2].pts[2] - point_t<>(0.353553f, 0.0f,      0.646447f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(pos_part->get_tetrahedra()[2].pts[3] - point_t<>(0.0f,      1.0f,      0.0f     ))) < result_tolerance);


    BOOST_CHECK(neg_part->number_of_primitives_on_surface()  == 3);
    BOOST_CHECK(neg_part->number_of_primitives_inside()      == 0);
    BOOST_CHECK_CLOSE(neg_part->max_volume_error(), 0.0477686f, result_tolerance);
    BOOST_REQUIRE(neg_part->number_of_primitives()           == 3);
    BOOST_CHECK(std::fabs(magnitude(neg_part->get_tetrahedra()[0].pts[0] - point_t<>(0.0f,      0.0f,      0.0f     ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(neg_part->get_tetrahedra()[0].pts[1] - point_t<>(0.0f,      0.0f,      1.0f     ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(neg_part->get_tetrahedra()[0].pts[2] - point_t<>(0.0f,      0.353553f, 0.0f     ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(neg_part->get_tetrahedra()[0].pts[3] - point_t<>(0.353553f, 0.0f,      0.0f     ))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(neg_part->get_tetrahedra()[1].pts[0] - point_t<>(0.0f,      0.353553f, 0.646447f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(neg_part->get_tetrahedra()[1].pts[1] - point_t<>(0.0f,      0.353553f, 0.0f     ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(neg_part->get_tetrahedra()[1].pts[2] - point_t<>(0.0f,      0.0f,      1.0f     ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(neg_part->get_tetrahedra()[1].pts[3] - point_t<>(0.353553f, 0.0f,      0.0f     ))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(neg_part->get_tetrahedra()[2].pts[0] - point_t<>(0.353553f, 0.0f,      0.646447f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(neg_part->get_tetrahedra()[2].pts[1] - point_t<>(0.353553f, 0.0f,      0.0f     ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(neg_part->get_tetrahedra()[2].pts[2] - point_t<>(0.0f,      0.353553f, 0.646447f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(neg_part->get_tetrahedra()[2].pts[3] - point_t<>(0.0f,      0.0f,      1.0f     ))) < result_tolerance);

    delete pos_part;
    delete neg_part;
}

BOOST_AUTO_TEST_CASE( cut_split_6_6_points_1_3_test )
{
    primitive_set *pos_part_p = nullptr;
    primitive_set *neg_part_p = nullptr;
    simple.cut(plane(point_t<>(0.0f, std::sqrt(2.0f), std::sqrt(2.0f)), 0.5f, axis_t::x_axis), &pos_part_p, &neg_part_p);
    tetrahedron_set* pos_part = static_cast<tetrahedron_set *>(pos_part_p);
    tetrahedron_set* neg_part = static_cast<tetrahedron_set *>(neg_part_p);
    
    BOOST_CHECK(pos_part->number_of_primitives_on_surface()  == 3);
    BOOST_CHECK(pos_part->number_of_primitives_inside()      == 0);
    BOOST_CHECK_CLOSE(pos_part->max_volume_error(), 0.118898f, result_tolerance);
    BOOST_REQUIRE(pos_part->number_of_primitives()           == 3);
    BOOST_CHECK(std::fabs(magnitude(pos_part->get_tetrahedra()[0].pts[0] - point_t<>(0.0f,      0.0f,      1.0f     ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(pos_part->get_tetrahedra()[0].pts[1] - point_t<>(0.0f,      1.0f,      0.0f     ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(pos_part->get_tetrahedra()[0].pts[2] - point_t<>(0.0f,      0.0f,      0.353553f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(pos_part->get_tetrahedra()[0].pts[3] - point_t<>(0.646447f, 0.0f,      0.353553f))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(pos_part->get_tetrahedra()[1].pts[0] - point_t<>(0.0f,      0.353553f, 0.0f     ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(pos_part->get_tetrahedra()[1].pts[1] - point_t<>(0.0f,      0.0f,      0.353553f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(pos_part->get_tetrahedra()[1].pts[2] - point_t<>(0.0f,      1.0f,      0.0f     ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(pos_part->get_tetrahedra()[1].pts[3] - point_t<>(0.646447f, 0.0f,      0.353553f))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(pos_part->get_tetrahedra()[2].pts[0] - point_t<>(0.646447f, 0.353553f, 0.0f     ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(pos_part->get_tetrahedra()[2].pts[1] - point_t<>(0.646447f, 0.0f,      0.353553f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(pos_part->get_tetrahedra()[2].pts[2] - point_t<>(0.0f,      0.353553f, 0.0f     ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(pos_part->get_tetrahedra()[2].pts[3] - point_t<>(0.0f,      1.0f,      0.0f     ))) < result_tolerance);

    BOOST_CHECK(neg_part->number_of_primitives_on_surface()  == 2);
    BOOST_CHECK(neg_part->number_of_primitives_inside()      == 0);
    BOOST_CHECK_CLOSE(neg_part->max_volume_error(), 0.034301f, result_tolerance);
    BOOST_REQUIRE(neg_part->number_of_primitives()           == 2);
    BOOST_CHECK(std::fabs(magnitude(neg_part->get_tetrahedra()[0].pts[0] - point_t<>(1.0f,      0.0f,      0.0f     ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(neg_part->get_tetrahedra()[0].pts[1] - point_t<>(0.0f,      0.0f,      0.0f     ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(neg_part->get_tetrahedra()[0].pts[2] - point_t<>(0.0f,      0.353553f, 0.0f     ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(neg_part->get_tetrahedra()[0].pts[3] - point_t<>(0.646447f, 0.0f,      0.353553f))) < result_tolerance);

    BOOST_CHECK(std::fabs(magnitude(neg_part->get_tetrahedra()[1].pts[0] - point_t<>(0.0f,      0.0f,      0.353553f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(neg_part->get_tetrahedra()[1].pts[1] - point_t<>(0.0f,      0.353553f, 0.0f     ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(neg_part->get_tetrahedra()[1].pts[2] - point_t<>(0.0f,      0.0f,      0.0f     ))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(neg_part->get_tetrahedra()[1].pts[3] - point_t<>(0.646447f, 0.0f,      0.353553f))) < result_tolerance);

    delete pos_part;
    delete neg_part;
}

BOOST_AUTO_TEST_CASE( cut_on_surface_test )
{
    primitive_set *pos_part_p = nullptr;
    primitive_set *neg_part_p = nullptr;
    on_surface.compute_bounding_box();
    on_surface.cut(plane(point_t<>(1.0f, 0.0f, 0.0f), 0.5f, axis_t::x_axis), &pos_part_p, &neg_part_p);
    tetrahedron_set* pos_part = static_cast<tetrahedron_set *>(pos_part_p);
    tetrahedron_set* neg_part = static_cast<tetrahedron_set *>(neg_part_p);

    BOOST_CHECK(pos_part->number_of_primitives_on_surface()  == 2);
    BOOST_CHECK(pos_part->number_of_primitives_inside()      == 1);
    BOOST_CHECK_CLOSE(pos_part->max_volume_error(), 0.33333333f, result_tolerance);
    BOOST_REQUIRE(pos_part->number_of_primitives()           == 7);
    BOOST_CHECK(pos_part->get_tetrahedra()[0].pts[0] == point_t<>( 1.5f, -0.5f,  0.5f));
    BOOST_CHECK(pos_part->get_tetrahedra()[0].pts[1] == point_t<>( 1.5f, -0.5f, -0.5f));
    BOOST_CHECK(pos_part->get_tetrahedra()[0].pts[2] == point_t<>( 1.5f,  0.5f,  0.5f));
    BOOST_CHECK(pos_part->get_tetrahedra()[0].pts[3] == point_t<>( 0.5f, -0.5f,  0.5f));
    BOOST_CHECK(pos_part->get_tetrahedra()[1].pts[0] == point_t<>( 2.5f,  0.5f, -0.5f));
    BOOST_CHECK(pos_part->get_tetrahedra()[1].pts[1] == point_t<>( 1.5f,  0.5f, -0.5f));
    BOOST_CHECK(pos_part->get_tetrahedra()[1].pts[2] == point_t<>( 2.5f,  0.5f,  0.5f));
    BOOST_CHECK(pos_part->get_tetrahedra()[1].pts[3] == point_t<>( 2.5f, -0.5f, -0.5f));
    BOOST_CHECK(pos_part->get_tetrahedra()[2].pts[0] == point_t<>( 2.5f, -0.5f, -0.5f));
    BOOST_CHECK(pos_part->get_tetrahedra()[2].pts[1] == point_t<>( 2.5f,  0.5f, -0.5f));
    BOOST_CHECK(pos_part->get_tetrahedra()[2].pts[2] == point_t<>( 3.5f, -0.5f, -0.5f));
    BOOST_CHECK(pos_part->get_tetrahedra()[2].pts[3] == point_t<>( 2.5f, -0.5f,  0.5f));
    BOOST_CHECK(pos_part->get_tetrahedra()[3].pts[0] == point_t<>( 3.5f,  0.5f,  0.5f));
    BOOST_CHECK(pos_part->get_tetrahedra()[3].pts[1] == point_t<>( 3.5f, -0.5f,  0.5f));
    BOOST_CHECK(pos_part->get_tetrahedra()[3].pts[2] == point_t<>( 4.5f,  0.5f,  0.5f));
    BOOST_CHECK(pos_part->get_tetrahedra()[3].pts[3] == point_t<>( 3.5f,  0.5f, -0.5f));
    BOOST_CHECK(pos_part->get_tetrahedra()[4].pts[0] == point_t<>( 5.5f,  0.5f, -0.5f));
    BOOST_CHECK(pos_part->get_tetrahedra()[4].pts[1] == point_t<>( 4.5f,  0.5f, -0.5f));
    BOOST_CHECK(pos_part->get_tetrahedra()[4].pts[2] == point_t<>( 5.5f,  0.5f,  0.5f));
    BOOST_CHECK(pos_part->get_tetrahedra()[4].pts[3] == point_t<>( 5.5f, -0.5f, -0.5f));
    BOOST_CHECK(pos_part->get_tetrahedra()[5].pts[0] == point_t<>( 5.5f, -0.5f, -0.5f));
    BOOST_CHECK(pos_part->get_tetrahedra()[5].pts[1] == point_t<>( 5.5f,  0.5f, -0.5f));
    BOOST_CHECK(pos_part->get_tetrahedra()[5].pts[2] == point_t<>( 6.5f, -0.5f, -0.5f));
    BOOST_CHECK(pos_part->get_tetrahedra()[5].pts[3] == point_t<>( 5.5f, -0.5f,  0.5f));
    BOOST_CHECK(pos_part->get_tetrahedra()[6].pts[0] == point_t<>( 6.5f,  0.5f,  0.5f));
    BOOST_CHECK(pos_part->get_tetrahedra()[6].pts[1] == point_t<>( 6.5f, -0.5f,  0.5f));
    BOOST_CHECK(pos_part->get_tetrahedra()[6].pts[2] == point_t<>( 7.5f,  0.5f,  0.5f));
    BOOST_CHECK(pos_part->get_tetrahedra()[6].pts[3] == point_t<>( 6.5f,  0.5f, -0.5f));

    BOOST_CHECK(neg_part->number_of_primitives_on_surface()  == 1);
    BOOST_CHECK(neg_part->number_of_primitives_inside()      == 0);
    BOOST_CHECK_CLOSE(neg_part->max_volume_error(), 0.33333333f, result_tolerance);
    BOOST_REQUIRE(neg_part->number_of_primitives()           == 1);
    BOOST_CHECK(neg_part->get_tetrahedra()[0].pts[0] == point_t<>( 0.5f, -0.5f, -0.5f));
    BOOST_CHECK(neg_part->get_tetrahedra()[0].pts[1] == point_t<>(-0.5f,  0.5f, -0.5f));
    BOOST_CHECK(neg_part->get_tetrahedra()[0].pts[2] == point_t<>( 0.5f,  0.5f,  0.5f));
    BOOST_CHECK(neg_part->get_tetrahedra()[0].pts[3] == point_t<>(-0.5f, -0.5f,  0.5f));

    delete pos_part;
    delete neg_part;
}

/* Test convert */
BOOST_AUTO_TEST_CASE( convert_test )
{
    convex_mesh mesh;
    cube.convert(&mesh, voxel_value_t::primitive_undefined);
    BOOST_CHECK(mesh.number_of_points()     == 20);
    BOOST_CHECK(mesh.number_of_triangles()  == 20);
    BOOST_CHECK(mesh.volume()               == 1.0f);

    cube3x.convert(&mesh, voxel_value_t::primitive_undefined);
    BOOST_CHECK(mesh.number_of_points()     == 40);
    BOOST_CHECK(mesh.number_of_triangles()  == 40);
    BOOST_CHECK(mesh.volume()               == 2.0f);

    on_surface.convert(&mesh, voxel_value_t::primitive_on_surface);
    BOOST_CHECK(mesh.number_of_points()     == 52);
    BOOST_CHECK(mesh.number_of_triangles()  == 52);
    BOOST_CHECK(fabs(mesh.volume() - 2.666667f) < result_tolerance);
}

/* Test align to principal axes */
BOOST_AUTO_TEST_CASE( align_to_principal_axes_cube_test )
{
    cube.compute_bounding_box();
    cube.compute_principal_axes();
    cube.align_to_principal_axes();
    const auto &v = cube.get_tetrahedra();
    BOOST_REQUIRE(v.size() == 5);
    BOOST_CHECK(v[0].pts[0] == point_t<>( 0.5f, -0.5f, -0.5f));
    BOOST_CHECK(v[0].pts[1] == point_t<>(-0.5f,  0.5f, -0.5f));
    BOOST_CHECK(v[0].pts[2] == point_t<>( 0.5f,  0.5f,  0.5f));
    BOOST_CHECK(v[0].pts[3] == point_t<>(-0.5f, -0.5f,  0.5f));
    BOOST_CHECK(v[1].pts[0] == point_t<>( 0.5f, -0.5f,  0.5f));
    BOOST_CHECK(v[1].pts[1] == point_t<>( 0.5f, -0.5f, -0.5f));
    BOOST_CHECK(v[1].pts[2] == point_t<>( 0.5f,  0.5f,  0.5f));
    BOOST_CHECK(v[1].pts[3] == point_t<>(-0.5f, -0.5f,  0.5f));
    BOOST_CHECK(v[2].pts[0] == point_t<>( 0.5f,  0.5f, -0.5f));
    BOOST_CHECK(v[2].pts[1] == point_t<>(-0.5f,  0.5f, -0.5f));
    BOOST_CHECK(v[2].pts[2] == point_t<>( 0.5f,  0.5f,  0.5f));
    BOOST_CHECK(v[2].pts[3] == point_t<>( 0.5f, -0.5f, -0.5f));
    BOOST_CHECK(v[3].pts[0] == point_t<>(-0.5f, -0.5f, -0.5f));
    BOOST_CHECK(v[3].pts[1] == point_t<>(-0.5f,  0.5f, -0.5f));
    BOOST_CHECK(v[3].pts[2] == point_t<>( 0.5f, -0.5f, -0.5f));
    BOOST_CHECK(v[3].pts[3] == point_t<>(-0.5f, -0.5f,  0.5f));
    BOOST_CHECK(v[4].pts[0] == point_t<>(-0.5f,  0.5f,  0.5f));
    BOOST_CHECK(v[4].pts[1] == point_t<>(-0.5f, -0.5f,  0.5f));
    BOOST_CHECK(v[4].pts[2] == point_t<>( 0.5f,  0.5f,  0.5f));
    BOOST_CHECK(v[4].pts[3] == point_t<>(-0.5f,  0.5f, -0.5f));
}

BOOST_AUTO_TEST_CASE( align_to_principal_axes_cubiod_test )
{
    cubiod.compute_bounding_box();
    cubiod.compute_principal_axes();
    cubiod.align_to_principal_axes();
    const auto &v = cubiod.get_tetrahedra();
    BOOST_REQUIRE(v.size() == 5);
    BOOST_CHECK(v[0].pts[0] == point_t<>( 2.0f, -0.5f, -0.5f));
    BOOST_CHECK(v[0].pts[1] == point_t<>(-2.0f,  0.5f, -0.5f));
    BOOST_CHECK(v[0].pts[2] == point_t<>( 2.0f,  0.5f,  0.5f));
    BOOST_CHECK(v[0].pts[3] == point_t<>(-2.0f, -0.5f,  0.5f));
    BOOST_CHECK(v[1].pts[0] == point_t<>( 2.0f, -0.5f,  0.5f));
    BOOST_CHECK(v[1].pts[1] == point_t<>( 2.0f, -0.5f, -0.5f));
    BOOST_CHECK(v[1].pts[2] == point_t<>( 2.0f,  0.5f,  0.5f));
    BOOST_CHECK(v[1].pts[3] == point_t<>(-2.0f, -0.5f,  0.5f));
    BOOST_CHECK(v[2].pts[0] == point_t<>( 2.0f,  0.5f, -0.5f));
    BOOST_CHECK(v[2].pts[1] == point_t<>(-2.0f,  0.5f, -0.5f));
    BOOST_CHECK(v[2].pts[2] == point_t<>( 2.0f,  0.5f,  0.5f));
    BOOST_CHECK(v[2].pts[3] == point_t<>( 2.0f, -0.5f, -0.5f));
    BOOST_CHECK(v[3].pts[0] == point_t<>(-2.0f, -0.5f, -0.5f));
    BOOST_CHECK(v[3].pts[1] == point_t<>(-2.0f,  0.5f, -0.5f));
    BOOST_CHECK(v[3].pts[2] == point_t<>( 2.0f, -0.5f, -0.5f));
    BOOST_CHECK(v[3].pts[3] == point_t<>(-2.0f, -0.5f,  0.5f));
    BOOST_CHECK(v[4].pts[0] == point_t<>(-2.0f,  0.5f,  0.5f));
    BOOST_CHECK(v[4].pts[1] == point_t<>(-2.0f, -0.5f,  0.5f));
    BOOST_CHECK(v[4].pts[2] == point_t<>( 2.0f,  0.5f,  0.5f));
    BOOST_CHECK(v[4].pts[3] == point_t<>(-2.0f,  0.5f, -0.5f));
}

BOOST_AUTO_TEST_CASE( align_to_principal_axes_cube_rot_test )
{
    cube_rot.compute_bounding_box();
    cube_rot.compute_principal_axes();
    cube_rot.align_to_principal_axes();
    const auto &v = cube_rot.get_tetrahedra();
    BOOST_REQUIRE(v.size() == 5);
    BOOST_CHECK(std::fabs(magnitude(v[0].pts[0] - point_t<>( 0.659395f,     0.758348f,    -0.288536f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(v[0].pts[1] - point_t<>(-0.986396f,     0.191946f,    -0.288664f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(v[0].pts[2] - point_t<>(-1.77115e-05f, -0.000143769f,  0.866025f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(v[0].pts[3] - point_t<>( 0.327018f,    -0.95015f,     -0.288826f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(v[1].pts[0] - point_t<>( 0.986396f,    -0.191946f,     0.288664f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(v[1].pts[1] - point_t<>( 0.659395f,     0.758348f,    -0.288536f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(v[1].pts[2] - point_t<>(-1.77115e-05f, -0.000143769f,  0.866025f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(v[1].pts[3] - point_t<>( 0.327018f,    -0.95015f,     -0.288826f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(v[2].pts[0] - point_t<>(-0.327018f,     0.95015f,      0.288826f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(v[2].pts[1] - point_t<>(-0.986396f,     0.191946f,    -0.288664f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(v[2].pts[2] - point_t<>(-1.77115e-05f, -0.000143769f,  0.866025f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(v[2].pts[3] - point_t<>( 0.659395f,     0.758348f,    -0.288536f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(v[3].pts[0] - point_t<>( 1.75744e-05f,  0.000143674f, -0.866025f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(v[3].pts[1] - point_t<>(-0.986396f,     0.191946f,    -0.288664f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(v[3].pts[2] - point_t<>( 0.659395f,     0.758348f,    -0.288536f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(v[3].pts[3] - point_t<>( 0.327018f,    -0.95015f,     -0.288826f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(v[4].pts[0] - point_t<>(-0.659395f,    -0.758348f,     0.288536f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(v[4].pts[1] - point_t<>( 0.327018f,    -0.95015f,     -0.288826f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(v[4].pts[2] - point_t<>(-1.77115e-05f, -0.000143769f,  0.866025f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(v[4].pts[3] - point_t<>(-0.986396f,     0.191946f,    -0.288664f))) < result_tolerance);
}

BOOST_AUTO_TEST_CASE( align_to_principal_axes_simple_test )
{
    simple.compute_bounding_box();
    simple.compute_principal_axes();
    simple.align_to_principal_axes();
    const auto &v = simple.get_tetrahedra();
    BOOST_REQUIRE(v.size() == 1);
    BOOST_CHECK(fabs(magnitude(v[0].pts[0] - point_t<>(-0.183013f,  0.250022f,  0.250025f)))     < result_tolerance);
    BOOST_CHECK(fabs(magnitude(v[0].pts[1] - point_t<>( 0.394344f,  0.92085f,  -0.21543f)))      < result_tolerance);
    BOOST_CHECK(fabs(magnitude(v[0].pts[2] - point_t<>( 0.39428f,  -0.488503f, -0.0982773f)))    < result_tolerance);
    BOOST_CHECK(fabs(magnitude(v[0].pts[3] - point_t<>( 0.394389f,  0.317631f,  1.06368f)))      < result_tolerance);
}

/* Test revert align to principal axes */
BOOST_AUTO_TEST_CASE( revert_align_to_principal_axes_simple_test )
{
    simple.compute_bounding_box();
    simple.compute_principal_axes();
    simple.align_to_principal_axes();
    simple.revert_align_to_principal_axes();
    const auto &v = simple.get_tetrahedra();
    BOOST_REQUIRE(v.size() == 1);
    BOOST_CHECK(fabs(magnitude(v[0].pts[0] - point_t<>( 0.0f,  0.0f,  0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(v[0].pts[1] - point_t<>( 0.0f,  1.0f,  0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(v[0].pts[2] - point_t<>( 1.0f,  0.0f,  0.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(v[0].pts[3] - point_t<>( 0.0f,  0.0f,  1.0f))) < result_tolerance);
}

/* Test compute axes aligned clipping planes */
BOOST_AUTO_TEST_CASE( compute_axes_aligned_clipping_planes_test )
{
    std::vector<plane> planes;
    diagonal.compute_bounding_box();
    diagonal.compute_axes_aligned_clipping_planes(&planes, 9);

    BOOST_REQUIRE(planes.size() == 11);
    BOOST_CHECK(planes[0].n             == point_t<>(1.0f, 0.0f, 0.0f));
    BOOST_CHECK_CLOSE(planes[0].p, 5.5f, result_tolerance);
    BOOST_CHECK(planes[0].major_axis    == axis_t::x_axis);
    BOOST_CHECK(planes[1].n             == point_t<>(0.0f, 1.0f, 0.0f));
    BOOST_CHECK_CLOSE(planes[1].p, 7.5f, result_tolerance);
    BOOST_CHECK(planes[1].major_axis    == axis_t::y_axis);
    BOOST_CHECK(planes[2].n             == point_t<>(0.0f, 1.0f, 0.0f));
    BOOST_CHECK_CLOSE(planes[2].p, 25.5f, result_tolerance);
    BOOST_CHECK(planes[2].major_axis    == axis_t::y_axis);
    BOOST_CHECK(planes[3].n             == point_t<>(0.0f, 0.0f, 1.0f));
    BOOST_CHECK_CLOSE(planes[3].p, 3.5f, result_tolerance);
    BOOST_CHECK(planes[3].major_axis    == axis_t::z_axis);
    BOOST_CHECK(planes[4].n             == point_t<>(0.0f, 0.0f, 1.0f));
    BOOST_CHECK_CLOSE(planes[4].p, 21.5f, result_tolerance);
    BOOST_CHECK(planes[4].major_axis    == axis_t::z_axis);
    BOOST_CHECK(planes[5].n             == point_t<>(0.0f, 0.0f, 1.0f));
    BOOST_CHECK_CLOSE(planes[5].p, 39.5f, result_tolerance);
    BOOST_CHECK(planes[5].major_axis    == axis_t::z_axis);
    BOOST_CHECK(planes[6].n             == point_t<>(0.0f, 0.0f, 1.0f));
    BOOST_CHECK_CLOSE(planes[6].p, 57.5f, result_tolerance);
    BOOST_CHECK(planes[6].major_axis    == axis_t::z_axis);
    BOOST_CHECK(planes[7].n             == point_t<>(0.0f, 0.0f, 1.0f));
    BOOST_CHECK_CLOSE(planes[7].p, 75.5f, result_tolerance);
    BOOST_CHECK(planes[7].major_axis    == axis_t::z_axis);
    BOOST_CHECK(planes[8].n             == point_t<>(0.0f, 0.0f, 1.0f));
    BOOST_CHECK_CLOSE(planes[8].p, 93.5f, result_tolerance);
    BOOST_CHECK(planes[8].major_axis    == axis_t::z_axis);
    BOOST_CHECK(planes[9].n             == point_t<>(0.0f, 0.0f, 1.0f));
    BOOST_CHECK_CLOSE(planes[9].p, 111.5f, result_tolerance);
    BOOST_CHECK(planes[9].major_axis    == axis_t::z_axis);
    BOOST_CHECK(planes[10].n            == point_t<>(0.0f, 0.0f, 1.0f));
    BOOST_CHECK_CLOSE(planes[10].p, 129.5f, result_tolerance);
    BOOST_CHECK(planes[10].major_axis   == axis_t::z_axis);
}

/* Test refine axes aligned clipping planes */
BOOST_AUTO_TEST_CASE( refine_axes_aligned_clipping_planes_x_test )
{
    std::vector<plane> planes;
    diagonal.compute_bounding_box();

    diagonal.refine_axes_aligned_clipping_planes(&planes, plane(point_t<>(0.0f, 0.0f, 0.0f), 0.0f, axis_t::x_axis), 9, 4);
    BOOST_REQUIRE(planes.size() == 7);
    BOOST_CHECK(planes[0].n             == point_t<>(1.0f, 0.0f, 0.0f));
    BOOST_CHECK_CLOSE(planes[0].p, 5.5f, result_tolerance);
    BOOST_CHECK(planes[0].major_axis    == axis_t::x_axis);
    BOOST_CHECK(planes[1].n             == point_t<>(1.0f, 0.0f, 0.0f));
    BOOST_CHECK_CLOSE(planes[1].p, 7.5f, result_tolerance);
    BOOST_CHECK(planes[1].major_axis    == axis_t::x_axis);
    BOOST_CHECK(planes[2].n             == point_t<>(1.0f, 0.0f, 0.0f));
    BOOST_CHECK_CLOSE(planes[2].p, 9.5f, result_tolerance);
    BOOST_CHECK(planes[2].major_axis    == axis_t::x_axis);
    BOOST_CHECK(planes[3].n             == point_t<>(1.0f, 0.0f, 0.0f));
    BOOST_CHECK_CLOSE(planes[3].p, 11.5f, result_tolerance);
    BOOST_CHECK(planes[3].major_axis    == axis_t::x_axis);
    BOOST_CHECK(planes[4].n             == point_t<>(1.0f, 0.0f, 0.0f));
    BOOST_CHECK_CLOSE(planes[4].p, 13.5f, result_tolerance);
    BOOST_CHECK(planes[4].major_axis    == axis_t::x_axis);
    BOOST_CHECK(planes[5].n             == point_t<>(1.0f, 0.0f, 0.0f));
    BOOST_CHECK_CLOSE(planes[5].p, 15.5f, result_tolerance);
    BOOST_CHECK(planes[5].major_axis    == axis_t::x_axis);
}

BOOST_AUTO_TEST_CASE( refine_axes_aligned_clipping_planes_y_test )
{
    std::vector<plane> planes;
    diagonal.compute_bounding_box();

    diagonal.refine_axes_aligned_clipping_planes(&planes, plane(point_t<>(0.0f, 0.0f, 0.0f), 0.0f, axis_t::y_axis), 9, 10);
    BOOST_REQUIRE(planes.size() == 11);
    BOOST_CHECK(planes[0].n             == point_t<>(0.0f, 1.0f, 0.0f));
    BOOST_CHECK_CLOSE(planes[0].p, 9.5f, result_tolerance);
    BOOST_CHECK(planes[0].major_axis    == axis_t::y_axis);
    BOOST_CHECK(planes[1].n             == point_t<>(0.0f, 1.0f, 0.0f));
    BOOST_CHECK_CLOSE(planes[1].p, 11.5f, result_tolerance);
    BOOST_CHECK(planes[1].major_axis    == axis_t::y_axis);
    BOOST_CHECK(planes[2].n             == point_t<>(0.0f, 1.0f, 0.0f));
    BOOST_CHECK_CLOSE(planes[2].p, 13.5f, result_tolerance);
    BOOST_CHECK(planes[2].major_axis    == axis_t::y_axis);
    BOOST_CHECK(planes[3].n             == point_t<>(0.0f, 1.0f, 0.0f));
    BOOST_CHECK_CLOSE(planes[3].p, 15.5f, result_tolerance);
    BOOST_CHECK(planes[3].major_axis    == axis_t::y_axis);
    BOOST_CHECK(planes[4].n             == point_t<>(0.0f, 1.0f, 0.0f));
    BOOST_CHECK_CLOSE(planes[4].p, 17.5f, result_tolerance);
    BOOST_CHECK(planes[4].major_axis    == axis_t::y_axis);
    BOOST_CHECK(planes[5].n             == point_t<>(0.0f, 1.0f, 0.0f));
    BOOST_CHECK_CLOSE(planes[5].p, 19.5f, result_tolerance);
    BOOST_CHECK(planes[5].major_axis    == axis_t::y_axis);
    BOOST_CHECK(planes[6].n             == point_t<>(0.0f, 1.0f, 0.0f));
    BOOST_CHECK_CLOSE(planes[6].p, 21.5f, result_tolerance);
    BOOST_CHECK(planes[6].major_axis    == axis_t::y_axis);
    BOOST_CHECK(planes[7].n             == point_t<>(0.0f, 1.0f, 0.0f));
    BOOST_CHECK_CLOSE(planes[7].p, 23.5f, result_tolerance);
    BOOST_CHECK(planes[7].major_axis    == axis_t::y_axis);
    BOOST_CHECK(planes[8].n             == point_t<>(0.0f, 1.0f, 0.0f));
    BOOST_CHECK_CLOSE(planes[8].p, 25.5f, result_tolerance);
    BOOST_CHECK(planes[8].major_axis    == axis_t::y_axis);
    BOOST_CHECK(planes[9].n             == point_t<>(0.0f, 1.0f, 0.0f));
    BOOST_CHECK_CLOSE(planes[9].p, 27.5f, result_tolerance);
    BOOST_CHECK(planes[9].major_axis    == axis_t::y_axis);
    BOOST_CHECK(planes[10].n            == point_t<>(0.0f, 1.0f, 0.0f));
    BOOST_CHECK_CLOSE(planes[10].p, 29.5f, result_tolerance);
    BOOST_CHECK(planes[10].major_axis   == axis_t::y_axis);
}

BOOST_AUTO_TEST_CASE( refine_axes_aligned_clipping_planes_z_test )
{
    std::vector<plane> planes;
    diagonal.compute_bounding_box();

    diagonal.refine_axes_aligned_clipping_planes(&planes, plane(point_t<>(0.0f, 0.0f, 0.0f), 0.0f, axis_t::z_axis), 4, 32);
    BOOST_REQUIRE(planes.size() == 9);
    BOOST_CHECK(planes[0].n             == point_t<>(0.0f, 0.0f, 1.0f));
    BOOST_CHECK_CLOSE(planes[0].p, 59.5f, result_tolerance);
    BOOST_CHECK(planes[0].major_axis    == axis_t::z_axis);
    BOOST_CHECK(planes[1].n             == point_t<>(0.0f, 0.0f, 1.0f));
    BOOST_CHECK_CLOSE(planes[1].p, 61.5f, result_tolerance);
    BOOST_CHECK(planes[1].major_axis    == axis_t::z_axis);
    BOOST_CHECK(planes[2].n             == point_t<>(0.0f, 0.0f, 1.0f));
    BOOST_CHECK_CLOSE(planes[2].p, 63.5f, result_tolerance);
    BOOST_CHECK(planes[2].major_axis    == axis_t::z_axis);
    BOOST_CHECK(planes[3].n             == point_t<>(0.0f, 0.0f, 1.0f));
    BOOST_CHECK_CLOSE(planes[3].p, 65.5f, result_tolerance);
    BOOST_CHECK(planes[3].major_axis    == axis_t::z_axis);
    BOOST_CHECK(planes[4].n             == point_t<>(0.0f, 0.0f, 1.0f));
    BOOST_CHECK_CLOSE(planes[4].p, 67.5f, result_tolerance);
    BOOST_CHECK(planes[4].major_axis    == axis_t::z_axis);
    BOOST_CHECK(planes[5].n             == point_t<>(0.0f, 0.0f, 1.0f));
    BOOST_CHECK_CLOSE(planes[5].p, 69.5f, result_tolerance);
    BOOST_CHECK(planes[5].major_axis    == axis_t::z_axis);
    BOOST_CHECK(planes[6].n             == point_t<>(0.0f, 0.0f, 1.0f));
    BOOST_CHECK_CLOSE(planes[6].p, 71.5f, result_tolerance);
    BOOST_CHECK(planes[6].major_axis    == axis_t::z_axis);
    BOOST_CHECK(planes[7].n             == point_t<>(0.0f, 0.0f, 1.0f));
    BOOST_CHECK_CLOSE(planes[7].p, 73.5f, result_tolerance);
    BOOST_CHECK(planes[7].major_axis    == axis_t::z_axis);
    BOOST_CHECK(planes[8].n             == point_t<>(0.0f, 0.0f, 1.0f));
    BOOST_CHECK_CLOSE(planes[8].p, 75.5f, result_tolerance);
    BOOST_CHECK(planes[8].major_axis    == axis_t::z_axis);
}

/* Test compute convex hull */
BOOST_AUTO_TEST_CASE( empty_compute_convex_hull_test )
{
    convex_mesh mesh;
    empty.compute_bounding_box();
    empty.compute_convex_hull(&mesh, 1);

    /* Checks */
    BOOST_REQUIRE(mesh.number_of_points()       == 0);
    BOOST_REQUIRE(mesh.number_of_triangles()    == 0);
}

BOOST_AUTO_TEST_CASE( cube_compute_convex_hull_test )
{
    convex_mesh mesh;
    cube.compute_bounding_box();
    cube.compute_convex_hull(&mesh, 1);

    /* Checks */
    BOOST_REQUIRE(mesh.number_of_points()       == 0);
    BOOST_REQUIRE(mesh.number_of_triangles()    == 0);
}

BOOST_AUTO_TEST_CASE( on_surface_compute_convex_hull_test )
{
    convex_mesh mesh;
    on_surface.compute_bounding_box();
    on_surface.compute_convex_hull(&mesh, 1);

    /* Checks */
    BOOST_REQUIRE(mesh.number_of_points() == 8);
    BOOST_CHECK(std::fabs(magnitude(mesh.points()[0] - point_t<>( 5.5f,  0.5f,  0.5f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(mesh.points()[1] - point_t<>( 5.5f,  0.5f, -0.5f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(mesh.points()[2] - point_t<>( 0.5f,  0.5f,  0.5f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(mesh.points()[3] - point_t<>( 1.5f, -0.5f,  0.5f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(mesh.points()[4] - point_t<>( 5.5f, -0.5f, -0.5f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(mesh.points()[5] - point_t<>(-0.5f,  0.5f, -0.5f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(mesh.points()[6] - point_t<>(-0.5f, -0.5f,  0.5f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(mesh.points()[7] - point_t<>( 0.5f, -0.5f, -0.5f))) < result_tolerance);

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
    BOOST_CHECK(mesh.triangles()[ 4].z == 0);
    BOOST_CHECK(mesh.triangles()[ 5].x == 4);
    BOOST_CHECK(mesh.triangles()[ 5].y == 1);
    BOOST_CHECK(mesh.triangles()[ 5].z == 0);
    BOOST_CHECK(mesh.triangles()[ 6].x == 4);
    BOOST_CHECK(mesh.triangles()[ 6].y == 7);
    BOOST_CHECK(mesh.triangles()[ 6].z == 5);
    BOOST_CHECK(mesh.triangles()[ 7].x == 4);
    BOOST_CHECK(mesh.triangles()[ 7].y == 5);
    BOOST_CHECK(mesh.triangles()[ 7].z == 1);
    BOOST_CHECK(mesh.triangles()[ 8].x == 5);
    BOOST_CHECK(mesh.triangles()[ 8].y == 6);
    BOOST_CHECK(mesh.triangles()[ 8].z == 2);
    BOOST_CHECK(mesh.triangles()[ 9].x == 6);
    BOOST_CHECK(mesh.triangles()[ 9].y == 7);
    BOOST_CHECK(mesh.triangles()[ 9].z == 4);
    BOOST_CHECK(mesh.triangles()[10].x == 6);
    BOOST_CHECK(mesh.triangles()[10].y == 4);
    BOOST_CHECK(mesh.triangles()[10].z == 3);
    BOOST_CHECK(mesh.triangles()[11].x == 7);
    BOOST_CHECK(mesh.triangles()[11].y == 6);
    BOOST_CHECK(mesh.triangles()[11].z == 5);
}

BOOST_AUTO_TEST_CASE( on_surface_down_sampling_compute_convex_hull_test )
{
    convex_mesh mesh;
    on_surface.compute_bounding_box();
    on_surface.compute_convex_hull(&mesh, 3);

    /* Checks */
    BOOST_REQUIRE(mesh.number_of_points() == 4);
    BOOST_CHECK(mesh.points()[0] == point_t<>(5.5f,  0.5f,  0.5f));
    BOOST_CHECK(mesh.points()[1] == point_t<>(5.5f,  0.5f, -0.5f));
    BOOST_CHECK(mesh.points()[2] == point_t<>(4.5f,  0.5f, -0.5f));
    BOOST_CHECK(mesh.points()[3] == point_t<>(5.5f, -0.5f, -0.5f));

    BOOST_REQUIRE(mesh.number_of_triangles() == 4);
    BOOST_CHECK(mesh.triangles()[0].x == 1);
    BOOST_CHECK(mesh.triangles()[0].y == 2);
    BOOST_CHECK(mesh.triangles()[0].z == 0);
    BOOST_CHECK(mesh.triangles()[1].x == 2);
    BOOST_CHECK(mesh.triangles()[1].y == 3);
    BOOST_CHECK(mesh.triangles()[1].z == 0);
    BOOST_CHECK(mesh.triangles()[2].x == 3);
    BOOST_CHECK(mesh.triangles()[2].y == 1);
    BOOST_CHECK(mesh.triangles()[2].z == 0);
    BOOST_CHECK(mesh.triangles()[3].x == 3);
    BOOST_CHECK(mesh.triangles()[3].y == 2);
    BOOST_CHECK(mesh.triangles()[3].z == 1);
}

BOOST_AUTO_TEST_CASE( low_cluster_size_on_surface_compute_convex_hull_test )
{
    convex_mesh mesh;
    on_surface.compute_bounding_box();
    on_surface.compute_convex_hull(&mesh, 1);

    /* Checks */
    BOOST_REQUIRE(mesh.number_of_points() == 8);
    BOOST_CHECK(std::fabs(magnitude(mesh.points()[0] - point_t<>( 5.5f,  0.5f,  0.5f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(mesh.points()[1] - point_t<>( 5.5f,  0.5f, -0.5f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(mesh.points()[2] - point_t<>( 0.5f,  0.5f,  0.5f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(mesh.points()[3] - point_t<>( 1.5f, -0.5f,  0.5f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(mesh.points()[4] - point_t<>( 5.5f, -0.5f, -0.5f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(mesh.points()[5] - point_t<>(-0.5f,  0.5f, -0.5f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(mesh.points()[6] - point_t<>(-0.5f, -0.5f,  0.5f))) < result_tolerance);
    BOOST_CHECK(std::fabs(magnitude(mesh.points()[7] - point_t<>( 0.5f, -0.5f, -0.5f))) < result_tolerance);

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
    BOOST_CHECK(mesh.triangles()[ 4].z == 0);
    BOOST_CHECK(mesh.triangles()[ 5].x == 4);
    BOOST_CHECK(mesh.triangles()[ 5].y == 1);
    BOOST_CHECK(mesh.triangles()[ 5].z == 0);
    BOOST_CHECK(mesh.triangles()[ 6].x == 4);
    BOOST_CHECK(mesh.triangles()[ 6].y == 7);
    BOOST_CHECK(mesh.triangles()[ 6].z == 5);
    BOOST_CHECK(mesh.triangles()[ 7].x == 4);
    BOOST_CHECK(mesh.triangles()[ 7].y == 5);
    BOOST_CHECK(mesh.triangles()[ 7].z == 1);
    BOOST_CHECK(mesh.triangles()[ 8].x == 5);
    BOOST_CHECK(mesh.triangles()[ 8].y == 6);
    BOOST_CHECK(mesh.triangles()[ 8].z == 2);
    BOOST_CHECK(mesh.triangles()[ 9].x == 6);
    BOOST_CHECK(mesh.triangles()[ 9].y == 7);
    BOOST_CHECK(mesh.triangles()[ 9].z == 4);
    BOOST_CHECK(mesh.triangles()[10].x == 6);
    BOOST_CHECK(mesh.triangles()[10].y == 4);
    BOOST_CHECK(mesh.triangles()[10].z == 3);
    BOOST_CHECK(mesh.triangles()[11].x == 7);
    BOOST_CHECK(mesh.triangles()[11].y == 6);
    BOOST_CHECK(mesh.triangles()[11].z == 5);
}

/* Test compute cut volumes */
BOOST_AUTO_TEST_CASE( cut_volumes_test )
{
    float pos_vol;
    float neg_vol;
    diagonal.compute_cut_volumes(plane(point_t<>(1.0f, 0.0f, 0.0f), 7.6f, axis_t::x_axis), &pos_vol, &neg_vol);
    BOOST_CHECK_CLOSE(pos_vol, 1.666667f, result_tolerance);
    BOOST_CHECK_CLOSE(neg_vol, 0.333333f, result_tolerance);

    on_surface.compute_cut_volumes(plane(point_t<>(1.0f, 0.0f, 0.0f), 2.25f, axis_t::x_axis), &pos_vol, &neg_vol);
    BOOST_CHECK_CLOSE(pos_vol, 1.0f, result_tolerance);
    BOOST_CHECK_CLOSE(neg_vol, 0.5f, result_tolerance);

    empty.compute_cut_volumes(plane(point_t<>(1.0f, 0.0f, 0.0f), 2.25f, axis_t::x_axis), &pos_vol, &neg_vol);
    BOOST_CHECK(pos_vol == 0.0f);
    BOOST_CHECK(neg_vol == 0.0f);
}

/* Test intersect */
BOOST_AUTO_TEST_CASE( intersect_clean_test )
{
    std::vector<point_t<>> pos_pts;
    std::vector<point_t<>> neg_pts;
    diagonal.intersect(plane(point_t<>(1.0f, 0.0f, 0.0f), 11.0f, axis_t::x_axis), &pos_pts, &neg_pts, 1);

    BOOST_REQUIRE(pos_pts.size() == 12);
    const auto &t = diagonal.get_tetrahedra();
    BOOST_CHECK(neg_pts[ 0] == t[0].pts[0]);
    BOOST_CHECK(neg_pts[ 1] == t[0].pts[1]);
    BOOST_CHECK(neg_pts[ 2] == t[0].pts[2]);
    BOOST_CHECK(neg_pts[ 3] == t[0].pts[3]);
    BOOST_CHECK(neg_pts[ 4] == t[1].pts[0]);
    BOOST_CHECK(neg_pts[ 5] == t[1].pts[1]);
    BOOST_CHECK(neg_pts[ 6] == t[1].pts[2]);
    BOOST_CHECK(neg_pts[ 7] == t[1].pts[3]);
    BOOST_CHECK(neg_pts[ 8] == t[2].pts[0]);
    BOOST_CHECK(neg_pts[ 9] == t[2].pts[1]);
    BOOST_CHECK(neg_pts[10] == t[2].pts[2]);
    BOOST_CHECK(neg_pts[11] == t[2].pts[3]);

    BOOST_REQUIRE(neg_pts.size() == 12);
    BOOST_CHECK(pos_pts[ 0] == t[3].pts[0]);
    BOOST_CHECK(pos_pts[ 1] == t[3].pts[1]);
    BOOST_CHECK(pos_pts[ 2] == t[3].pts[2]);
    BOOST_CHECK(pos_pts[ 3] == t[3].pts[3]);
    BOOST_CHECK(pos_pts[ 4] == t[4].pts[0]);
    BOOST_CHECK(pos_pts[ 5] == t[4].pts[1]);
    BOOST_CHECK(pos_pts[ 6] == t[4].pts[2]);
    BOOST_CHECK(pos_pts[ 7] == t[4].pts[3]);
    BOOST_CHECK(pos_pts[ 8] == t[5].pts[0]);
    BOOST_CHECK(pos_pts[ 9] == t[5].pts[1]);
    BOOST_CHECK(pos_pts[10] == t[5].pts[2]);
    BOOST_CHECK(pos_pts[11] == t[5].pts[3]);
}

BOOST_AUTO_TEST_CASE( intersect_pos_sampling_test )
{
    std::vector<point_t<>> pos_pts;
    std::vector<point_t<>> neg_pts;
    row.intersect(plane(point_t<>(0.0f, 1.0f, 0.0f), -0.4f, axis_t::x_axis), &pos_pts, &neg_pts, 3);

    BOOST_REQUIRE(pos_pts.size() == 8);
    const auto &t = row.get_tetrahedra();
    BOOST_CHECK(pos_pts[0] == t[2].pts[0]);
    BOOST_CHECK(pos_pts[1] == t[2].pts[1]);
    BOOST_CHECK(pos_pts[2] == t[2].pts[2]);
    BOOST_CHECK(pos_pts[3] == t[2].pts[3]);
    BOOST_CHECK(pos_pts[4] == t[5].pts[0]);
    BOOST_CHECK(pos_pts[5] == t[5].pts[1]);
    BOOST_CHECK(pos_pts[6] == t[5].pts[2]);
    BOOST_CHECK(pos_pts[7] == t[5].pts[3]);

    BOOST_REQUIRE(neg_pts.size() == 0);
}

BOOST_AUTO_TEST_CASE( intersect_neg_sampling_test )
{
    std::vector<point_t<>> pos_pts;
    std::vector<point_t<>> neg_pts;
    row.intersect(plane(point_t<>(0.0f, -1.0f, 0.0f), 0.4f, axis_t::x_axis), &pos_pts, &neg_pts, 3);

    BOOST_REQUIRE(neg_pts.size() == 8);
    const auto &t = row.get_tetrahedra();
    BOOST_CHECK(neg_pts[0] == t[2].pts[0]);
    BOOST_CHECK(neg_pts[1] == t[2].pts[1]);
    BOOST_CHECK(neg_pts[2] == t[2].pts[2]);
    BOOST_CHECK(neg_pts[3] == t[2].pts[3]);
    BOOST_CHECK(neg_pts[4] == t[5].pts[0]);
    BOOST_CHECK(neg_pts[5] == t[5].pts[1]);
    BOOST_CHECK(neg_pts[6] == t[5].pts[2]);
    BOOST_CHECK(neg_pts[7] == t[5].pts[3]);

    BOOST_REQUIRE(pos_pts.size() == 0);
}

BOOST_AUTO_TEST_SUITE_END()
}; /* namespace test */
}; /* namespace raptor_convex_decomposition */
