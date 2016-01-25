#ifdef STAND_ALONE
#define BOOST_TEST_MODULE convex_mesh test
#endif /* #ifdef STAND_ALONE */


/* Standard headers */

/* Boost headers */
#include "boost/noncopyable.hpp"
#include "boost/test/unit_test.hpp"

/* Convex Decomposition headers */
#include "convex_mesh.h"


namespace raptor_convex_decomposition
{
namespace test
{
/* Test data */
struct convex_mesh_fixture : private boost::noncopyable
{
    convex_mesh_fixture() :
    pyramid({
                point_t(-0.5f, -0.5f, -0.5f), point_t( 0.5f, -0.5f, -0.5f), point_t( 0.5f,  0.5f, -0.5f), point_t(-0.5f,  0.5f, -0.5f), point_t(0.0f, 0.0f, 2.5f)
            },
            { 
                point_ti<>(0, 2, 1), point_ti<>(0, 3, 2), /* Front face */
                point_ti<>(0, 1, 4), point_ti<>(1, 2, 4), point_ti<>(2, 3, 4), point_ti<>(3, 0, 4)
            }),
    cube(   {
                point_t(-0.5f, -0.5f, -0.5f), point_t( 0.5f, -0.5f, -0.5f), point_t( 0.5f,  0.5f, -0.5f), point_t(-0.5f,  0.5f, -0.5f),
                point_t(-0.5f, -0.5f,  0.5f), point_t( 0.5f, -0.5f,  0.5f), point_t( 0.5f,  0.5f,  0.5f), point_t(-0.5f,  0.5f,  0.5f)
            }, 
            {
                point_ti<>(0, 2, 1), point_ti<>(0, 3, 2),   /* Front face   */
                point_ti<>(4, 5, 6), point_ti<>(4, 6, 7),   /* Back face    */
                point_ti<>(4, 7, 0), point_ti<>(7, 3, 0),   /* Left face    */
                point_ti<>(1, 2, 6), point_ti<>(1, 6, 5),   /* Right face   */
                point_ti<>(3, 7, 6), point_ti<>(3, 6, 2),   /* Top face     */
                point_ti<>(0, 1, 4), point_ti<>(1, 5, 4)    /* Bottom face  */
            }),
    cube2x( {
                point_t( 0.0f,  0.0f,  0.0f), point_t( 2.0f,  0.0f,  0.0f), point_t( 2.0f,  2.0f,  0.0f), point_t( 0.0f,  2.0f,  0.0f),
                point_t( 0.0f,  0.0f,  2.0f), point_t( 2.0f,  0.0f,  2.0f), point_t( 2.0f,  2.0f,  2.0f), point_t( 0.0f,  2.0f,  2.0f)
            }, 
            {
                point_ti<>(0, 2, 1), point_ti<>(0, 3, 2),   /* Front face   */
                point_ti<>(4, 5, 6), point_ti<>(4, 6, 7),   /* Back face    */
                point_ti<>(4, 7, 0), point_ti<>(7, 3, 0),   /* Left face    */
                point_ti<>(1, 2, 6), point_ti<>(1, 6, 5),   /* Right face   */
                point_ti<>(3, 7, 6), point_ti<>(3, 6, 2),   /* Top face     */
                point_ti<>(0, 1, 4), point_ti<>(1, 5, 4)    /* Bottom face  */
            }),
    stellated_cube(
            {
                point_t(-0.5f, -0.5f, -0.5f), point_t( 0.5f, -0.5f, -0.5f), point_t( 0.5f,  0.5f, -0.5f), point_t(-0.5f,  0.5f, -0.5f),
                point_t(-0.5f, -0.5f,  0.5f), point_t( 0.5f, -0.5f,  0.5f), point_t( 0.5f,  0.5f,  0.5f), point_t(-0.5f,  0.5f,  0.5f),
                point_t( 3.5f,  0.0f,  0.0f), point_t(-3.5f,  0.0f,  0.0f), point_t( 0.0f,  3.5f,  0.0f), point_t( 0.0f, -3.5f,  0.0f), point_t( 0.0f,  0.0f,  3.5f), point_t( 0.0f,  0.0f, -3.5f)
            }, 
            {
                point_ti<>(1, 0, 13), point_ti<>(2, 1, 13), point_ti<>(3, 2, 13), point_ti<>(0, 3, 13), /* Front face   */
                point_ti<>(4, 5, 12), point_ti<>(5, 6, 12), point_ti<>(6, 7, 12), point_ti<>(7, 4, 12), /* Back face    */
                point_ti<>(3, 0,  9), point_ti<>(0, 4,  9), point_ti<>(4, 7,  9), point_ti<>(7, 3,  9), /* Left face    */
                point_ti<>(1, 2,  8), point_ti<>(2, 6,  8), point_ti<>(6, 5,  8), point_ti<>(5, 1,  8), /* Right face   */
                point_ti<>(2, 3, 10), point_ti<>(3, 7, 10), point_ti<>(7, 6, 10), point_ti<>(6, 2, 10), /* Top face     */
                point_ti<>(0, 1, 11), point_ti<>(1, 5, 11), point_ti<>(5, 4, 11), point_ti<>(4, 0, 11)  /* Bottom face  */
            })
      {  };

    convex_mesh empty;
    convex_mesh pyramid;
    convex_mesh cube;
    convex_mesh cube2x;
    convex_mesh stellated_cube;
};

BOOST_FIXTURE_TEST_SUITE( convex_mesh_tests, convex_mesh_fixture )

const float result_tolerance = 0.0005f;

/* Test add points and triangles */
BOOST_AUTO_TEST_CASE( add_points_and_triangles_test )
{
    convex_mesh uut;
    BOOST_CHECK(uut.number_of_points()      == 0);
    BOOST_CHECK(uut.number_of_triangles()   == 0);

    uut.add_point(point_t(-0.5f, -0.5f, -0.5f));
    uut.add_point(point_t( 0.5f, -0.5f, -0.5f));
    uut.add_point(point_t( 0.5f,  0.5f, -0.5f));
    uut.add_point(point_t(-0.5f,  0.5f, -0.5f));
    uut.add_point(point_t( 0.0f,  0.0f,  2.5f));
    BOOST_CHECK(uut.number_of_points()      == 5);
    BOOST_CHECK(uut.number_of_triangles()   == 0);

    uut.add_triangle(0, 2, 1);
    uut.add_triangle(0, 3, 2);
    uut.add_triangle(0, 1, 4);
    uut.add_triangle(1, 2, 4);
    uut.add_triangle(2, 3, 4);
    uut.add_triangle(3, 0, 4);
    BOOST_CHECK(uut.number_of_points()      == 5);
    BOOST_CHECK(uut.number_of_triangles()   == 6);
}

/* Test clear */
BOOST_AUTO_TEST_CASE( clear_test )
{
    convex_mesh uut;
    BOOST_CHECK(uut.number_of_points()      == 0);
    BOOST_CHECK(uut.number_of_triangles()   == 0);

    uut.add_point(point_t(-0.5f, -0.5f, -0.5f));
    uut.add_point(point_t( 0.5f, -0.5f, -0.5f));
    uut.add_point(point_t( 0.5f,  0.5f, -0.5f));
    uut.add_point(point_t(-0.5f,  0.5f, -0.5f));
    uut.add_point(point_t( 0.0f,  0.0f,  2.5f));
    BOOST_CHECK(uut.number_of_points()      == 5);
    BOOST_CHECK(uut.number_of_triangles()   == 0);

    uut.add_triangle(0, 2, 1);
    uut.add_triangle(0, 3, 2);
    uut.add_triangle(0, 1, 4);
    uut.add_triangle(1, 2, 4);
    uut.add_triangle(2, 3, 4);
    uut.add_triangle(3, 0, 4);
    BOOST_CHECK(uut.number_of_points()      == 5);
    BOOST_CHECK(uut.number_of_triangles()   == 6);

    /* Clear */
    uut.clear();
    BOOST_CHECK(uut.number_of_points()      == 0);
    BOOST_CHECK(uut.number_of_triangles()   == 0);

    uut.add_point(point_t(-0.5f, -0.5f, -0.5f));
    uut.add_point(point_t( 0.5f, -0.5f, -0.5f));
    uut.add_point(point_t( 0.5f,  0.5f, -0.5f));
    uut.add_point(point_t(-0.5f,  0.5f, -0.5f));
    uut.add_point(point_t( 0.0f,  0.0f,  2.5f));
    BOOST_CHECK(uut.number_of_points()      == 5);
    BOOST_CHECK(uut.number_of_triangles()   == 0);

    uut.add_triangle(0, 2, 1);
    uut.add_triangle(0, 3, 2);
    uut.add_triangle(0, 1, 4);
    uut.add_triangle(1, 2, 4);
    uut.add_triangle(2, 3, 4);
    uut.add_triangle(3, 0, 4);
    BOOST_CHECK(uut.number_of_points()      == 5);
    BOOST_CHECK(uut.number_of_triangles()   == 6);
}

/* Test volume */
BOOST_AUTO_TEST_CASE( volume_test )
{
    BOOST_CHECK(empty.volume()          == 0.0f);
    BOOST_CHECK(pyramid.volume()        == 1.0f);
    BOOST_CHECK(cube.volume()           == 1.0f);
    BOOST_CHECK(cube2x.volume()         == 8.0f);
    BOOST_CHECK(stellated_cube.volume() == 7.0f);
}

/* Test diagonal */
BOOST_AUTO_TEST_CASE( diagonal_test )
{
    BOOST_CHECK(empty.diagonal()            == 0.0f);
    BOOST_CHECK(pyramid.diagonal()          == std::sqrt(11.0f));
    BOOST_CHECK(cube.diagonal()             == std::sqrt(3.0f));
    BOOST_CHECK(cube2x.diagonal()           == std::sqrt(12.0f));
    BOOST_CHECK(stellated_cube.diagonal()   == std::sqrt(147.0f));
}

/* Test is_inside */
BOOST_AUTO_TEST_CASE( is_inside_empty_test )
{
    BOOST_CHECK(!empty.is_inside(point_t( 0.0f,  0.0f,  0.0f)));
}

BOOST_AUTO_TEST_CASE( is_inside_pyramid_test )
{
    BOOST_CHECK( pyramid.is_inside(point_t( 0.0f,  0.0f, 0.0f)));
    BOOST_CHECK( pyramid.is_inside(point_t( 0.0f,  0.2f, 1.0f)));
    BOOST_CHECK( pyramid.is_inside(point_t( 0.0f, -0.2f, 1.0f)));
    BOOST_CHECK( pyramid.is_inside(point_t( 0.2f,  0.0f, 1.0f)));
    BOOST_CHECK( pyramid.is_inside(point_t(-0.2f,  0.0f, 1.0f)));
    BOOST_CHECK( pyramid.is_inside(point_t( 0.0f,  0.0f, 2.5f)));
    BOOST_CHECK(!pyramid.is_inside(point_t( 0.0f,  0.1f, 3.0f)));
    BOOST_CHECK(!pyramid.is_inside(point_t( 0.0f, -0.1f, 3.0f)));
    BOOST_CHECK(!pyramid.is_inside(point_t( 0.1f,  0.0f, 3.0f)));
    BOOST_CHECK(!pyramid.is_inside(point_t(-0.1f,  0.0f, 3.0f)));
}

BOOST_AUTO_TEST_CASE( is_inside_cube_test )
{
    BOOST_CHECK( cube.is_inside(point_t( 0.0f,  0.0f,  0.0f)));
    BOOST_CHECK( cube.is_inside(point_t( 0.4f,  0.0f,  0.0f)));
    BOOST_CHECK( cube.is_inside(point_t( 0.0f,  0.4f,  0.0f)));
    BOOST_CHECK( cube.is_inside(point_t( 0.0f,  0.0f,  0.4f)));
    BOOST_CHECK( cube.is_inside(point_t( 0.4f,  0.4f,  0.4f)));
    BOOST_CHECK(!cube.is_inside(point_t( 0.6f,  0.0f,  0.0f)));
    BOOST_CHECK(!cube.is_inside(point_t( 0.0f,  0.6f,  0.0f)));
    BOOST_CHECK(!cube.is_inside(point_t( 0.0f,  0.0f,  0.6f)));
    BOOST_CHECK(!cube.is_inside(point_t(-0.6f, -0.6f, -0.6f)));
}

BOOST_AUTO_TEST_CASE( is_inside_cube2x_test )
{
    BOOST_CHECK( cube2x.is_inside(point_t( 0.0f,  0.0f,  0.0f)));
    BOOST_CHECK( cube2x.is_inside(point_t( 1.9f,  0.0f,  0.0f)));
    BOOST_CHECK( cube2x.is_inside(point_t( 0.0f,  1.9f,  0.0f)));
    BOOST_CHECK( cube2x.is_inside(point_t( 0.0f,  0.0f,  1.9f)));
    BOOST_CHECK( cube2x.is_inside(point_t( 1.9f,  1.9f,  1.9f)));
    BOOST_CHECK(!cube2x.is_inside(point_t(-0.1f,  0.0f,  0.0f)));
    BOOST_CHECK(!cube2x.is_inside(point_t( 0.0f, -0.1f,  0.0f)));
    BOOST_CHECK(!cube2x.is_inside(point_t( 0.0f,  0.0f, -0.1f)));
    BOOST_CHECK(!cube2x.is_inside(point_t( 2.1f,  0.0f,  0.0f)));
    BOOST_CHECK(!cube2x.is_inside(point_t( 0.0f,  2.1f,  0.0f)));
    BOOST_CHECK(!cube2x.is_inside(point_t( 0.0f,  0.0f,  2.1f)));
    BOOST_CHECK(!cube2x.is_inside(point_t( 2.1f,  2.1f,  2.1f)));
}

/* Cant test stellate cube because it isnt convex */

/* Test cut */
BOOST_AUTO_TEST_CASE( cut_pyramid_test )
{
    /* Cut */
    std::vector<point_t> pos_cut;
    std::vector<point_t> neg_cut;
    pyramid.cut(&pos_cut, &neg_cut, point_t(0.0f, 0.0f, 1.0f), 0.0f);

    /* Checks */
    BOOST_REQUIRE(pos_cut.size()    == 1);
    BOOST_CHECK(pos_cut[0] == point_t(0.0f, 0.0f, 2.5f));

    BOOST_REQUIRE(neg_cut.size()    == 4);
    BOOST_CHECK(neg_cut[0] == point_t(-0.5f, -0.5f, -0.5f));
    BOOST_CHECK(neg_cut[1] == point_t( 0.5f, -0.5f, -0.5f));
    BOOST_CHECK(neg_cut[2] == point_t( 0.5f,  0.5f, -0.5f));
    BOOST_CHECK(neg_cut[3] == point_t(-0.5f,  0.5f, -0.5f));

    /* Invert cut */
    pos_cut.clear();
    neg_cut.clear();
    pyramid.cut(&pos_cut, &neg_cut, point_t(0.0f, 0.0f, -1.0f), 0.0f);

    /* Checks */
    BOOST_REQUIRE(neg_cut.size()    == 1);
    BOOST_CHECK(neg_cut[0] == point_t(0.0f, 0.0f, 2.5f));

    BOOST_REQUIRE(pos_cut.size()    == 4);
    BOOST_CHECK(pos_cut[0] == point_t(-0.5f, -0.5f, -0.5f));
    BOOST_CHECK(pos_cut[1] == point_t( 0.5f, -0.5f, -0.5f));
    BOOST_CHECK(pos_cut[2] == point_t( 0.5f,  0.5f, -0.5f));
    BOOST_CHECK(pos_cut[3] == point_t(-0.5f,  0.5f, -0.5f));
}

BOOST_AUTO_TEST_CASE( cut_pyramid_on_base_test )
{
    /* Cut */
    std::vector<point_t> pos_cut;
    std::vector<point_t> neg_cut;
    pyramid.cut(&pos_cut, &neg_cut, point_t(0.0f, 0.0f, 1.0f), -0.5f);

    /* Checks */
    BOOST_REQUIRE(pos_cut.size()    == 5);
    BOOST_CHECK(pos_cut[0] == point_t(-0.5f, -0.5f, -0.5f));
    BOOST_CHECK(pos_cut[1] == point_t( 0.5f, -0.5f, -0.5f));
    BOOST_CHECK(pos_cut[2] == point_t( 0.5f,  0.5f, -0.5f));
    BOOST_CHECK(pos_cut[3] == point_t(-0.5f,  0.5f, -0.5f));
    BOOST_CHECK(pos_cut[4] == point_t( 0.0f,  0.0f,  2.5f));

    BOOST_REQUIRE(neg_cut.size()    == 4);
    BOOST_CHECK(neg_cut[0] == point_t(-0.5f, -0.5f, -0.5f));
    BOOST_CHECK(neg_cut[1] == point_t( 0.5f, -0.5f, -0.5f));
    BOOST_CHECK(neg_cut[2] == point_t( 0.5f,  0.5f, -0.5f));
    BOOST_CHECK(neg_cut[3] == point_t(-0.5f,  0.5f, -0.5f));
}

BOOST_AUTO_TEST_CASE( cut_cube_test )
{
    /* Cut */
    std::vector<point_t> pos_cut;
    std::vector<point_t> neg_cut;
    cube.cut(&pos_cut, &neg_cut, point_t(0.0f, 1.0f, 0.0f), 0.0f);

    /* Checks */
    BOOST_REQUIRE(pos_cut.size()    == 4);
    BOOST_CHECK(pos_cut[0] == point_t( 0.5f,  0.5f, -0.5f));
    BOOST_CHECK(pos_cut[1] == point_t(-0.5f,  0.5f, -0.5f));
    BOOST_CHECK(pos_cut[2] == point_t( 0.5f,  0.5f,  0.5f));
    BOOST_CHECK(pos_cut[3] == point_t(-0.5f,  0.5f,  0.5f));

    BOOST_REQUIRE(neg_cut.size()    == 4);
    BOOST_CHECK(neg_cut[0] == point_t(-0.5f, -0.5f, -0.5f));
    BOOST_CHECK(neg_cut[1] == point_t( 0.5f, -0.5f, -0.5f));
    BOOST_CHECK(neg_cut[2] == point_t(-0.5f, -0.5f,  0.5f));
    BOOST_CHECK(neg_cut[3] == point_t( 0.5f, -0.5f,  0.5f));

    /* Invert cut */
    pos_cut.clear();
    neg_cut.clear();
    cube.cut(&pos_cut, &neg_cut, point_t(0.0f, -1.0f, 0.0f), 0.0f);

    /* Checks */
    BOOST_REQUIRE(neg_cut.size()    == 4);
    BOOST_CHECK(neg_cut[0] == point_t( 0.5f,  0.5f, -0.5f));
    BOOST_CHECK(neg_cut[1] == point_t(-0.5f,  0.5f, -0.5f));
    BOOST_CHECK(neg_cut[2] == point_t( 0.5f,  0.5f,  0.5f));
    BOOST_CHECK(neg_cut[3] == point_t(-0.5f,  0.5f,  0.5f));

    BOOST_REQUIRE(pos_cut.size()    == 4);
    BOOST_CHECK(pos_cut[0] == point_t(-0.5f, -0.5f, -0.5f));
    BOOST_CHECK(pos_cut[1] == point_t( 0.5f, -0.5f, -0.5f));
    BOOST_CHECK(pos_cut[2] == point_t(-0.5f, -0.5f,  0.5f));
    BOOST_CHECK(pos_cut[3] == point_t( 0.5f, -0.5f,  0.5f));
}

BOOST_AUTO_TEST_CASE( cut_cube2x_test )
{
    /* Cut */
    std::vector<point_t> pos_cut;
    std::vector<point_t> neg_cut;
    cube2x.cut(&pos_cut, &neg_cut, point_t(1.0f, 0.0f, 0.0f), 1.0f);

    /* Checks */
    BOOST_REQUIRE(pos_cut.size()    == 4);
    BOOST_CHECK(pos_cut[0] == point_t( 2.0f,  0.0f,  0.0f));
    BOOST_CHECK(pos_cut[1] == point_t( 2.0f,  2.0f,  0.0f));
    BOOST_CHECK(pos_cut[2] == point_t( 2.0f,  0.0f,  2.0f));
    BOOST_CHECK(pos_cut[3] == point_t( 2.0f,  2.0f,  2.0f));

    BOOST_REQUIRE(neg_cut.size()    == 4);
    BOOST_CHECK(neg_cut[0] == point_t( 0.0f,  0.0f,  0.0f));
    BOOST_CHECK(neg_cut[1] == point_t( 0.0f,  2.0f,  0.0f));
    BOOST_CHECK(neg_cut[2] == point_t( 0.0f,  0.0f,  2.0f));
    BOOST_CHECK(neg_cut[3] == point_t( 0.0f,  2.0f,  2.0f));

    /* Invert cut */
    pos_cut.clear();
    neg_cut.clear();
    cube2x.cut(&pos_cut, &neg_cut, point_t(-1.0f, 0.0f, 0.0f), -1.0f);

    /* Checks */
    BOOST_REQUIRE(neg_cut.size()    == 4);
    BOOST_CHECK(neg_cut[0] == point_t( 2.0f,  0.0f,  0.0f));
    BOOST_CHECK(neg_cut[1] == point_t( 2.0f,  2.0f,  0.0f));
    BOOST_CHECK(neg_cut[2] == point_t( 2.0f,  0.0f,  2.0f));
    BOOST_CHECK(neg_cut[3] == point_t( 2.0f,  2.0f,  2.0f));

    BOOST_REQUIRE(pos_cut.size()    == 4);
    BOOST_CHECK(pos_cut[0] == point_t( 0.0f,  0.0f,  0.0f));
    BOOST_CHECK(pos_cut[1] == point_t( 0.0f,  2.0f,  0.0f));
    BOOST_CHECK(pos_cut[2] == point_t( 0.0f,  0.0f,  2.0f));
    BOOST_CHECK(pos_cut[3] == point_t( 0.0f,  2.0f,  2.0f));
}

BOOST_AUTO_TEST_CASE( cut_stellated_cube_test )
{
    /* Cut positive x point */
    std::vector<point_t> pos_cut;
    std::vector<point_t> neg_cut;
    stellated_cube.cut(&pos_cut, &neg_cut, point_t(1.0f, 0.0f, 0.0f), 2.5f);

    /* Checks */
    BOOST_REQUIRE(neg_cut.size()    == 13);
    BOOST_CHECK(neg_cut[ 0] == point_t(-0.5f, -0.5f, -0.5f));
    BOOST_CHECK(neg_cut[ 1] == point_t( 0.5f, -0.5f, -0.5f));
    BOOST_CHECK(neg_cut[ 2] == point_t( 0.5f,  0.5f, -0.5f));
    BOOST_CHECK(neg_cut[ 3] == point_t(-0.5f,  0.5f, -0.5f));
    BOOST_CHECK(neg_cut[ 4] == point_t(-0.5f, -0.5f,  0.5f));
    BOOST_CHECK(neg_cut[ 5] == point_t( 0.5f, -0.5f,  0.5f));
    BOOST_CHECK(neg_cut[ 6] == point_t( 0.5f,  0.5f,  0.5f));
    BOOST_CHECK(neg_cut[ 7] == point_t(-0.5f,  0.5f,  0.5f));
    BOOST_CHECK(neg_cut[ 8] == point_t(-3.5f,  0.0f,  0.0f));
    BOOST_CHECK(neg_cut[ 9] == point_t( 0.0f,  3.5f,  0.0f));
    BOOST_CHECK(neg_cut[10] == point_t( 0.0f, -3.5f,  0.0f));
    BOOST_CHECK(neg_cut[11] == point_t( 0.0f,  0.0f,  3.5f));
    BOOST_CHECK(neg_cut[12] == point_t( 0.0f,  0.0f, -3.5f));

    BOOST_REQUIRE(pos_cut.size()    == 1);
    BOOST_CHECK(pos_cut[0] == point_t( 3.5f,  0.0f,  0.0f));

    /* Cut negative x point */
    pos_cut.clear();
    neg_cut.clear();
    stellated_cube.cut(&pos_cut, &neg_cut, point_t(-1.0f, 0.0f, 0.0f), 2.5f);

    /* Checks */
    BOOST_REQUIRE(neg_cut.size()    == 13);
    BOOST_CHECK(neg_cut[ 0] == point_t(-0.5f, -0.5f, -0.5f));
    BOOST_CHECK(neg_cut[ 1] == point_t( 0.5f, -0.5f, -0.5f));
    BOOST_CHECK(neg_cut[ 2] == point_t( 0.5f,  0.5f, -0.5f));
    BOOST_CHECK(neg_cut[ 3] == point_t(-0.5f,  0.5f, -0.5f));
    BOOST_CHECK(neg_cut[ 4] == point_t(-0.5f, -0.5f,  0.5f));
    BOOST_CHECK(neg_cut[ 5] == point_t( 0.5f, -0.5f,  0.5f));
    BOOST_CHECK(neg_cut[ 6] == point_t( 0.5f,  0.5f,  0.5f));
    BOOST_CHECK(neg_cut[ 7] == point_t(-0.5f,  0.5f,  0.5f));
    BOOST_CHECK(neg_cut[ 8] == point_t( 3.5f,  0.0f,  0.0f));
    BOOST_CHECK(neg_cut[ 9] == point_t( 0.0f,  3.5f,  0.0f));
    BOOST_CHECK(neg_cut[10] == point_t( 0.0f, -3.5f,  0.0f));
    BOOST_CHECK(neg_cut[11] == point_t( 0.0f,  0.0f,  3.5f));
    BOOST_CHECK(neg_cut[12] == point_t( 0.0f,  0.0f, -3.5f));

    BOOST_REQUIRE(pos_cut.size()    == 1);
    BOOST_CHECK(pos_cut[0] == point_t(-3.5f,  0.0f,  0.0f));

    /* Cut positive y point */
    pos_cut.clear();
    neg_cut.clear();
    stellated_cube.cut(&pos_cut, &neg_cut, point_t(0.0f, 1.0f, 0.0f), 2.5f);

    /* Checks */
    BOOST_REQUIRE(neg_cut.size()    == 13);
    BOOST_CHECK(neg_cut[ 0] == point_t(-0.5f, -0.5f, -0.5f));
    BOOST_CHECK(neg_cut[ 1] == point_t( 0.5f, -0.5f, -0.5f));
    BOOST_CHECK(neg_cut[ 2] == point_t( 0.5f,  0.5f, -0.5f));
    BOOST_CHECK(neg_cut[ 3] == point_t(-0.5f,  0.5f, -0.5f));
    BOOST_CHECK(neg_cut[ 4] == point_t(-0.5f, -0.5f,  0.5f));
    BOOST_CHECK(neg_cut[ 5] == point_t( 0.5f, -0.5f,  0.5f));
    BOOST_CHECK(neg_cut[ 6] == point_t( 0.5f,  0.5f,  0.5f));
    BOOST_CHECK(neg_cut[ 7] == point_t(-0.5f,  0.5f,  0.5f));
    BOOST_CHECK(neg_cut[ 8] == point_t( 3.5f,  0.0f,  0.0f));
    BOOST_CHECK(neg_cut[ 9] == point_t(-3.5f,  0.0f,  0.0f));
    BOOST_CHECK(neg_cut[10] == point_t( 0.0f, -3.5f,  0.0f));
    BOOST_CHECK(neg_cut[11] == point_t( 0.0f,  0.0f,  3.5f));
    BOOST_CHECK(neg_cut[12] == point_t( 0.0f,  0.0f, -3.5f));

    BOOST_REQUIRE(pos_cut.size()    == 1);
    BOOST_CHECK(pos_cut[0] == point_t( 0.0f,  3.5f,  0.0f));

    /* Cut negative y point */
    pos_cut.clear();
    neg_cut.clear();
    stellated_cube.cut(&pos_cut, &neg_cut, point_t(0.0f, -1.0f, 0.0f), 2.5f);

    /* Checks */
    BOOST_REQUIRE(neg_cut.size()    == 13);
    BOOST_CHECK(neg_cut[ 0] == point_t(-0.5f, -0.5f, -0.5f));
    BOOST_CHECK(neg_cut[ 1] == point_t( 0.5f, -0.5f, -0.5f));
    BOOST_CHECK(neg_cut[ 2] == point_t( 0.5f,  0.5f, -0.5f));
    BOOST_CHECK(neg_cut[ 3] == point_t(-0.5f,  0.5f, -0.5f));
    BOOST_CHECK(neg_cut[ 4] == point_t(-0.5f, -0.5f,  0.5f));
    BOOST_CHECK(neg_cut[ 5] == point_t( 0.5f, -0.5f,  0.5f));
    BOOST_CHECK(neg_cut[ 6] == point_t( 0.5f,  0.5f,  0.5f));
    BOOST_CHECK(neg_cut[ 7] == point_t(-0.5f,  0.5f,  0.5f));
    BOOST_CHECK(neg_cut[ 8] == point_t( 3.5f,  0.0f,  0.0f));
    BOOST_CHECK(neg_cut[ 9] == point_t(-3.5f,  0.0f,  0.0f));
    BOOST_CHECK(neg_cut[10] == point_t( 0.0f,  3.5f,  0.0f));
    BOOST_CHECK(neg_cut[11] == point_t( 0.0f,  0.0f,  3.5f));
    BOOST_CHECK(neg_cut[12] == point_t( 0.0f,  0.0f, -3.5f));

    BOOST_REQUIRE(pos_cut.size()    == 1);
    BOOST_CHECK(pos_cut[0] == point_t( 0.0f, -3.5f,  0.0f));

    /* Cut positive z point */
    pos_cut.clear();
    neg_cut.clear();
    stellated_cube.cut(&pos_cut, &neg_cut, point_t(0.0f, 0.0f, 1.0f), 2.5f);

    /* Checks */
    BOOST_REQUIRE(neg_cut.size()    == 13);
    BOOST_CHECK(neg_cut[ 0] == point_t(-0.5f, -0.5f, -0.5f));
    BOOST_CHECK(neg_cut[ 1] == point_t( 0.5f, -0.5f, -0.5f));
    BOOST_CHECK(neg_cut[ 2] == point_t( 0.5f,  0.5f, -0.5f));
    BOOST_CHECK(neg_cut[ 3] == point_t(-0.5f,  0.5f, -0.5f));
    BOOST_CHECK(neg_cut[ 4] == point_t(-0.5f, -0.5f,  0.5f));
    BOOST_CHECK(neg_cut[ 5] == point_t( 0.5f, -0.5f,  0.5f));
    BOOST_CHECK(neg_cut[ 6] == point_t( 0.5f,  0.5f,  0.5f));
    BOOST_CHECK(neg_cut[ 7] == point_t(-0.5f,  0.5f,  0.5f));
    BOOST_CHECK(neg_cut[ 8] == point_t( 3.5f,  0.0f,  0.0f));
    BOOST_CHECK(neg_cut[ 9] == point_t(-3.5f,  0.0f,  0.0f));
    BOOST_CHECK(neg_cut[10] == point_t( 0.0f,  3.5f,  0.0f));
    BOOST_CHECK(neg_cut[11] == point_t( 0.0f, -3.5f,  0.0f));
    BOOST_CHECK(neg_cut[12] == point_t( 0.0f,  0.0f, -3.5f));

    BOOST_REQUIRE(pos_cut.size()    == 1);
    BOOST_CHECK(pos_cut[0] == point_t( 0.0f,  0.0f,  3.5f));

    /* Cut negative z point */
    pos_cut.clear();
    neg_cut.clear();
    stellated_cube.cut(&pos_cut, &neg_cut, point_t(0.0f, 0.0f, -1.0f), 2.5f);

    /* Checks */
    BOOST_REQUIRE(neg_cut.size()    == 13);
    BOOST_CHECK(neg_cut[ 0] == point_t(-0.5f, -0.5f, -0.5f));
    BOOST_CHECK(neg_cut[ 1] == point_t( 0.5f, -0.5f, -0.5f));
    BOOST_CHECK(neg_cut[ 2] == point_t( 0.5f,  0.5f, -0.5f));
    BOOST_CHECK(neg_cut[ 3] == point_t(-0.5f,  0.5f, -0.5f));
    BOOST_CHECK(neg_cut[ 4] == point_t(-0.5f, -0.5f,  0.5f));
    BOOST_CHECK(neg_cut[ 5] == point_t( 0.5f, -0.5f,  0.5f));
    BOOST_CHECK(neg_cut[ 6] == point_t( 0.5f,  0.5f,  0.5f));
    BOOST_CHECK(neg_cut[ 7] == point_t(-0.5f,  0.5f,  0.5f));
    BOOST_CHECK(neg_cut[ 8] == point_t( 3.5f,  0.0f,  0.0f));
    BOOST_CHECK(neg_cut[ 9] == point_t(-3.5f,  0.0f,  0.0f));
    BOOST_CHECK(neg_cut[10] == point_t( 0.0f,  3.5f,  0.0f));
    BOOST_CHECK(neg_cut[11] == point_t( 0.0f, -3.5f,  0.0f));
    BOOST_CHECK(neg_cut[12] == point_t( 0.0f,  0.0f,  3.5f));

    BOOST_REQUIRE(pos_cut.size()    == 1);
    BOOST_CHECK(pos_cut[0] == point_t( 0.0f,  0.0f, -3.5f));
}

/* Test copute convex hull */
BOOST_AUTO_TEST_CASE( cube_compute_convex_hull_test )
{
    convex_mesh uut;
    uut.compute_convex_hull(
    {
        point_t(-0.5f, -0.5f, -0.5f), point_t( 0.5f, -0.5f, -0.5f), point_t( 0.5f,  0.5f, -0.5f), point_t(-0.5f,  0.5f, -0.5f),
        point_t(-0.5f, -0.5f,  0.5f), point_t( 0.5f, -0.5f,  0.5f), point_t( 0.5f,  0.5f,  0.5f), point_t(-0.5f,  0.5f,  0.5f)
    });

    /* Checks */
    BOOST_REQUIRE(uut.number_of_points() == 8);
    BOOST_CHECK(uut.points()[0] == point_t( 0.5f,  0.5f,  0.5f));
    BOOST_CHECK(uut.points()[1] == point_t( 0.5f,  0.5f, -0.5f));
    BOOST_CHECK(uut.points()[2] == point_t(-0.5f,  0.5f,  0.5f));
    BOOST_CHECK(uut.points()[3] == point_t( 0.5f, -0.5f,  0.5f));
    BOOST_CHECK(uut.points()[4] == point_t( 0.5f, -0.5f, -0.5f));
    BOOST_CHECK(uut.points()[5] == point_t(-0.5f,  0.5f, -0.5f));
    BOOST_CHECK(uut.points()[6] == point_t(-0.5f, -0.5f,  0.5f));
    BOOST_CHECK(uut.points()[7] == point_t(-0.5f, -0.5f, -0.5f));

    BOOST_REQUIRE(uut.number_of_triangles() == 12);
    BOOST_CHECK(uut.triangles()[ 0].x == 1);
    BOOST_CHECK(uut.triangles()[ 0].y == 5);
    BOOST_CHECK(uut.triangles()[ 0].z == 2);
    BOOST_CHECK(uut.triangles()[ 1].x == 1);
    BOOST_CHECK(uut.triangles()[ 1].y == 2);
    BOOST_CHECK(uut.triangles()[ 1].z == 0);
    BOOST_CHECK(uut.triangles()[ 2].x == 2);
    BOOST_CHECK(uut.triangles()[ 2].y == 6);
    BOOST_CHECK(uut.triangles()[ 2].z == 3);
    BOOST_CHECK(uut.triangles()[ 3].x == 2);
    BOOST_CHECK(uut.triangles()[ 3].y == 3);
    BOOST_CHECK(uut.triangles()[ 3].z == 0);
    BOOST_CHECK(uut.triangles()[ 4].x == 3);
    BOOST_CHECK(uut.triangles()[ 4].y == 4);
    BOOST_CHECK(uut.triangles()[ 4].z == 1);
    BOOST_CHECK(uut.triangles()[ 5].x == 3);
    BOOST_CHECK(uut.triangles()[ 5].y == 1);
    BOOST_CHECK(uut.triangles()[ 5].z == 0);
    BOOST_CHECK(uut.triangles()[ 6].x == 4);
    BOOST_CHECK(uut.triangles()[ 6].y == 7);
    BOOST_CHECK(uut.triangles()[ 6].z == 5);
    BOOST_CHECK(uut.triangles()[ 7].x == 4);
    BOOST_CHECK(uut.triangles()[ 7].y == 5);
    BOOST_CHECK(uut.triangles()[ 7].z == 1);
    BOOST_CHECK(uut.triangles()[ 8].x == 5);
    BOOST_CHECK(uut.triangles()[ 8].y == 7);
    BOOST_CHECK(uut.triangles()[ 8].z == 6);
    BOOST_CHECK(uut.triangles()[ 9].x == 5);
    BOOST_CHECK(uut.triangles()[ 9].y == 6);
    BOOST_CHECK(uut.triangles()[ 9].z == 2);
    BOOST_CHECK(uut.triangles()[10].x == 6);
    BOOST_CHECK(uut.triangles()[10].y == 7);
    BOOST_CHECK(uut.triangles()[10].z == 4);
    BOOST_CHECK(uut.triangles()[11].x == 6);
    BOOST_CHECK(uut.triangles()[11].y == 4);
    BOOST_CHECK(uut.triangles()[11].z == 3);
}

BOOST_AUTO_TEST_CASE( tetrahedron_compute_convex_hull_test )
{
    convex_mesh uut;
    uut.compute_convex_hull(
    {
        point_t(5.5f,  0.5f,  0.5f), point_t(5.5f,  0.5f, -0.5f), point_t(4.5f,  0.5f, -0.5f), point_t(5.5f, -0.5f, -0.5f)
    });

    /* Checks */
    BOOST_REQUIRE(uut.number_of_points() == 4);
    BOOST_CHECK(uut.points()[0] == point_t(5.5f,  0.5f,  0.5f));
    BOOST_CHECK(uut.points()[1] == point_t(5.5f,  0.5f, -0.5f));
    BOOST_CHECK(uut.points()[2] == point_t(4.5f,  0.5f, -0.5f));
    BOOST_CHECK(uut.points()[3] == point_t(5.5f, -0.5f, -0.5f));

    BOOST_REQUIRE(uut.number_of_triangles() == 4);
    BOOST_CHECK(uut.triangles()[ 0].x == 1);
    BOOST_CHECK(uut.triangles()[ 0].y == 2);
    BOOST_CHECK(uut.triangles()[ 0].z == 0);
    BOOST_CHECK(uut.triangles()[ 1].x == 2);
    BOOST_CHECK(uut.triangles()[ 1].y == 3);
    BOOST_CHECK(uut.triangles()[ 1].z == 0);
    BOOST_CHECK(uut.triangles()[ 2].x == 3);
    BOOST_CHECK(uut.triangles()[ 2].y == 1);
    BOOST_CHECK(uut.triangles()[ 2].z == 0);
    BOOST_CHECK(uut.triangles()[ 3].x == 3);
    BOOST_CHECK(uut.triangles()[ 3].y == 2);
    BOOST_CHECK(uut.triangles()[ 3].z == 1);
}

BOOST_AUTO_TEST_SUITE_END()
}; /* namespace test */
}; /* namespace raptor_convex_decomposition */