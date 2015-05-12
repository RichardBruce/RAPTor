#ifdef STAND_ALONE
#define BOOST_TEST_MODULE bvh_node test

/* Common headers */
#include "logging.h"

/* Ray tracer headers */
#include "bvh_node.h"
#include "triangle.h"

/* Initialise logger */
const raptor_physics::init_logger init_logger;

namespace raptor_raytracer
{
/* Primitives array so we dont need to include more source */
std::vector<triangle *> *   bvh_node::o     = nullptr;
}
#endif /* #ifdef STAND_ALONE */

/* Standard headers */
#include <array>

/* Boost headerrs */
#include "boost/test/unit_test.hpp"

/* Ray tracer headers */
#include "bvh_node.h"
#include "phong_shader.h"


namespace raptor_raytracer
{
namespace test
{
const float result_tolerance = 0.0001f;

struct bvh_node_fixture
{
    bvh_node_fixture() : nodes(2)
    {
        nodes[0].create_leaf_node(point_t(5.0f, 4.0f, 3.0f), point_t(0.0f, 1.0f, -1.0f), 6, 6);
        nodes[1].create_leaf_node(point_t(6.0f, 4.0f, 5.0f), point_t(2.0f, 1.0f,  0.0f), 6, 17);
    }

    bvh_node                uut;
    std::vector<bvh_node>   nodes;
};


BOOST_FIXTURE_TEST_SUITE( bvh_node_tests, bvh_node_fixture );

/* Leaf node */
BOOST_AUTO_TEST_CASE( create_empty_leaf_node_test )
{
    uut.create_leaf_node(point_t(5.0f, 4.0f, 3.0f), point_t(0.0f, 1.0f, 2.0f), 6, 6);

    BOOST_CHECK(uut.is_leaf());
    BOOST_CHECK(uut.is_empty());
    BOOST_CHECK(uut.size()          == 0);
    BOOST_CHECK(uut.high_point()    == point_t(5.0f, 4.0f, 3.0f));
    BOOST_CHECK(uut.low_point()     == point_t(0.0f, 1.0f, 2.0f));
}

BOOST_AUTO_TEST_CASE( create_leaf_node_test )
{
    uut.create_leaf_node(point_t(3.0f, 4.0f, 5.0f), point_t(2.0f, 1.0f, 0.0f), 6, 17);

    BOOST_CHECK(uut.is_leaf());
    BOOST_CHECK(!uut.is_empty());
    BOOST_CHECK(uut.size()          == 11);
    BOOST_CHECK(uut.high_point()    == point_t(3.0f, 4.0f, 5.0f));
    BOOST_CHECK(uut.low_point()     == point_t(2.0f, 1.0f, 0.0f));
}

/* Generic node*/
BOOST_AUTO_TEST_CASE( create_generic_node_test )
{
    uut.create_generic_node(nodes, 0, 1);

    BOOST_CHECK(!uut.is_leaf());
    BOOST_CHECK(uut.left_index()    == 0);
    BOOST_CHECK(uut.right_index()   == 1);
    BOOST_CHECK(uut.high_point()    == point_t(6.0f, 4.0f,  5.0f));
    BOOST_CHECK(uut.low_point()     == point_t(0.0f, 1.0f, -1.0f));

    uut.create_generic_node(nodes, 1, 0);

    BOOST_CHECK(!uut.is_leaf());
    BOOST_CHECK(uut.left_index()    == 1);
    BOOST_CHECK(uut.right_index()   == 0);
    BOOST_CHECK(uut.high_point()    == point_t(6.0f, 4.0f,  5.0f));
    BOOST_CHECK(uut.low_point()     == point_t(0.0f, 1.0f, -1.0f));
}

/* Cost function */
BOOST_AUTO_TEST_CASE( combined_surface_area_test )
{
    uut.create_leaf_node(point_t(3.0f, 4.0f, 5.0f), point_t(2.0f, 1.0f, -3.0f), 6, 17);
    BOOST_CHECK(uut.high_point()    == point_t(3.0f, 4.0f,  5.0f));
    BOOST_CHECK(uut.low_point()     == point_t(2.0f, 1.0f, -3.0f));

    BOOST_CHECK(uut.combined_surface_area(nodes[0]) == 79.0f);
    BOOST_CHECK(uut.combined_surface_area(nodes[1]) == 68.0f);
}

/* Intersection function */
BOOST_AUTO_TEST_CASE( intersection_distance_unit_node_test )
{
    uut.create_leaf_node(point_t(2.0f, 2.0f, 2.0f), point_t(1.0f, 1.0f, 1.0f), 6, 6);
    BOOST_CHECK(uut.intersection_distance(ray(point_t(1.01f, 0.0f, 0.0f),  1.0f, 1.0f, 1.0f), point_t(1.0f, 1.0f, 1.0f)) == std::numeric_limits<float>::max());
    BOOST_CHECK(uut.intersection_distance(ray(point_t(0.0f,  0.0f, 0.0f),  1.0f, 1.0f, 1.0f), point_t(1.0f, 1.0f, 1.0f)) == 1.0f);
    BOOST_CHECK(uut.intersection_distance(ray(point_t(1.0f,  0.0f, 0.0f),  1.0f, 1.0f, 1.0f), point_t(1.0f, 1.0f, 1.0f)) == 1.0f);
}

BOOST_AUTO_TEST_CASE( intersection_distance_axis_aligned_ray_test )
{
    uut.create_leaf_node(point_t(2.0f, 2.0f, 2.0f), point_t(1.0f, 1.0f, 1.0f), 6, 6);
    BOOST_CHECK(uut.intersection_distance(ray(point_t(0.0f, 1.5f, 1.5f),  1.0f, 0.0f, 0.0f), 1.0f / point_t( 1.0f,  0.0f,  0.0f)) == 1.0f);
    BOOST_CHECK(uut.intersection_distance(ray(point_t(3.0f, 1.5f, 1.5f), -1.0f, 0.0f, 0.0f), 1.0f / point_t(-1.0f,  0.0f,  0.0f)) == 1.0f);
    
    BOOST_CHECK(uut.intersection_distance(ray(point_t(1.5f, 0.0f, 1.5f), 0.0f,  1.0f, 0.0f), 1.0f / point_t( 0.0f,  1.0f,  0.0f)) == 1.0f);
    BOOST_CHECK(uut.intersection_distance(ray(point_t(1.5f, 3.0f, 1.5f), 0.0f, -1.0f, 0.0f), 1.0f / point_t( 0.0f, -1.0f,  0.0f)) == 1.0f);

    BOOST_CHECK(uut.intersection_distance(ray(point_t(1.5f, 1.5f, 0.0f), 0.0f, 0.0f,  1.0f), 1.0f / point_t( 0.0f,  0.0f,  1.0f)) == 1.0f);
    BOOST_CHECK(uut.intersection_distance(ray(point_t(1.5f, 1.5f, 3.0f), 0.0f, 0.0f, -1.0f), 1.0f / point_t( 0.0f,  0.0f, -1.0f)) == 1.0f);
}

BOOST_AUTO_TEST_CASE( intersection_distance_point_and_spread_test )
{
    uut.create_leaf_node(point_t(1.0f, 1.0f, 1.0f), point_t(-1.0f, -1.0f, -1.0f), 6, 6);
    BOOST_CHECK_CLOSE(uut.intersection_distance(ray(point_t(0.0f, 0.0f, -2.0f), 0.0f, 0.0f, 0.0f), 1.0f / normalise(point_t( 1.0f,  1.0f, 1.0f))), 1.732051f, result_tolerance);
    BOOST_CHECK_CLOSE(uut.intersection_distance(ray(point_t(0.0f, 0.0f, -2.0f), 0.0f, 0.0f, 0.0f), 1.0f / normalise(point_t(-1.0f,  1.0f, 1.0f))), 1.732051f, result_tolerance);
    BOOST_CHECK_CLOSE(uut.intersection_distance(ray(point_t(0.0f, 0.0f, -2.0f), 0.0f, 0.0f, 0.0f), 1.0f / normalise(point_t( 1.0f, -1.0f, 1.0f))), 1.732051f, result_tolerance);
    BOOST_CHECK_CLOSE(uut.intersection_distance(ray(point_t(0.0f, 0.0f, -2.0f), 0.0f, 0.0f, 0.0f), 1.0f / normalise(point_t(-1.0f, -1.0f, 1.0f))), 1.732051f, result_tolerance);

    /* Further away and test again */
    uut.create_leaf_node(point_t(11.0f, 11.0f, 11.0f), point_t(9.0f, 9.0f, 9.0f), 6, 6);
    BOOST_CHECK_CLOSE(uut.intersection_distance(ray(point_t(0.0f, 0.0f, -2.0f), 0.0f, 0.0f, 0.0f), 1.0f / normalise(point_t(11.0f, 11.0f, 11.0f))), 19.0525589f, result_tolerance);
    BOOST_CHECK_CLOSE(uut.intersection_distance(ray(point_t(0.0f, 0.0f, -2.0f), 0.0f, 0.0f, 0.0f), 1.0f / normalise(point_t( 9.0f, 11.0f, 11.0f))), 17.9722004f, result_tolerance);
    BOOST_CHECK_CLOSE(uut.intersection_distance(ray(point_t(0.0f, 0.0f, -2.0f), 0.0f, 0.0f, 0.0f), 1.0f / normalise(point_t(11.0f,  9.0f, 11.0f))), 17.9722004f, result_tolerance);
    BOOST_CHECK_CLOSE(uut.intersection_distance(ray(point_t(0.0f, 0.0f, -2.0f), 0.0f, 0.0f, 0.0f), 1.0f / normalise(point_t( 9.0f,  9.0f, 11.0f))), 16.8226051f, result_tolerance);

    /* Further away and test again */
    uut.create_leaf_node(point_t(101.0f, 101.0f, 101.0f), point_t(99.0f, 99.0f, 99.0f), 6, 6);
    BOOST_CHECK_CLOSE(uut.intersection_distance(ray(point_t(0.0f, 0.0f, -2.0f), 0.0f, 0.0f, 0.0f), 1.0f / normalise(point_t(101.0f, 101.0f, 101.0f))), 174.937149f, result_tolerance);
    BOOST_CHECK_CLOSE(uut.intersection_distance(ray(point_t(0.0f, 0.0f, -2.0f), 0.0f, 0.0f, 0.0f), 1.0f / normalise(point_t( 99.0f, 101.0f, 101.0f))), 173.7901f,   result_tolerance);
    BOOST_CHECK_CLOSE(uut.intersection_distance(ray(point_t(0.0f, 0.0f, -2.0f), 0.0f, 0.0f, 0.0f), 1.0f / normalise(point_t(101.0f,  99.0f, 101.0f))), 173.7901f,   result_tolerance);
    BOOST_CHECK_CLOSE(uut.intersection_distance(ray(point_t(0.0f, 0.0f, -2.0f), 0.0f, 0.0f, 0.0f), 1.0f / normalise(point_t( 99.0f,  99.0f, 101.0f))), 172.635468f, result_tolerance);

    /* Further away and test again */
    uut.create_leaf_node(point_t(1001.0f, 1001.0f, 1001.0f), point_t(999.0f, 999.0f, 999.0f), 6, 6);
    BOOST_CHECK_CLOSE(uut.intersection_distance(ray(point_t(0.0f, 0.0f, -2.0f), 0.0f, 0.0f, 0.0f), 1.0f / normalise(point_t(1001.0f, 1001.0f, 1001.0f))), 1733.78296f, result_tolerance);
    BOOST_CHECK_CLOSE(uut.intersection_distance(ray(point_t(0.0f, 0.0f, -2.0f), 0.0f, 0.0f, 0.0f), 1.0f / normalise(point_t( 999.0f, 1001.0f, 1001.0f))), 1732.62891f, result_tolerance);
    BOOST_CHECK_CLOSE(uut.intersection_distance(ray(point_t(0.0f, 0.0f, -2.0f), 0.0f, 0.0f, 0.0f), 1.0f / normalise(point_t(1001.0f,  999.0f, 1001.0f))), 1732.62891f, result_tolerance);
    BOOST_CHECK_CLOSE(uut.intersection_distance(ray(point_t(0.0f, 0.0f, -2.0f), 0.0f, 0.0f, 0.0f), 1.0f / normalise(point_t( 999.0f,  999.0f, 1001.0f))), 1731.47424f, result_tolerance);
}

BOOST_AUTO_TEST_CASE( vector_intersection_distance_unit_node_test )
{
    uut.create_leaf_node(point_t(2.0f, 2.0f, 2.0f), point_t(1.0f, 1.0f, 1.0f), 6, 6);

    packet_ray r;
    r.set_up(vfp_t(1.01f, 0.0f, 1.0f, 1.09f), vfp_t(0.0f, 0.0f, 0.0f, 0.0f), vfp_t(0.0f, 0.0f, 0.0f, 0.0f), vfp_t(1.0f, 1.0f, 1.0f, 1.0f), vfp_t(1.0f, 1.0f, 1.0f, 1.0f), vfp_t(1.0f, 1.0f, 1.0f, 1.0f));

    const vfp_t i_rd[3] = { vfp_t(1.0f, 1.0f, 1.0f, 1.0f), vfp_t(1.0f, 1.0f, 1.0f, 1.0f), vfp_t(1.0f, 1.0f, 1.0f, 1.0f) };

    const vfp_t res(uut.intersection_distance(r, i_rd));
    BOOST_CHECK_MESSAGE(res[0] == std::numeric_limits<float>::max(), res[0]);
    BOOST_CHECK_MESSAGE(res[1] == 1.0f, res[1]);
    BOOST_CHECK_MESSAGE(res[2] == 1.0f, res[2]);
}

BOOST_AUTO_TEST_CASE( vector_intersection_distance_axis_aligned_ray_test )
{
    uut.create_leaf_node(point_t(2.0f, 2.0f, 2.0f), point_t(1.0f, 1.0f, 1.0f), 6, 6);

    /* X and y axes */
    packet_ray r;
    r.set_up(vfp_t(0.0f, 3.0f, 1.5f, 1.5f), vfp_t(1.5f, 1.5f, 0.0f, 3.0f), vfp_t(1.5f, 1.5f, 1.5f, 1.5f), vfp_t(1.0f, -1.0f, 0.0f, 0.0f), vfp_t(0.0f, 0.0f, 1.0f, -1.0f), vfp_t(0.0f, 0.0f, 0.0f, 0.0f));

    const vfp_t i_rd0[3] = { 1.0f / vfp_t(1.0f, -1.0f, 0.0f, 0.0f), 1.0f / vfp_t(0.0f, 0.0f, 1.0f, -1.0f), 1.0f / vfp_t(0.0f, 0.0f, 0.0f, 0.0f) };

    const vfp_t res0(uut.intersection_distance(r, i_rd0));
    BOOST_CHECK(res0[0] == 1.0f);
    BOOST_CHECK(res0[1] == 1.0f);
    BOOST_CHECK(res0[2] == 1.0f);
    BOOST_CHECK(res0[3] == 1.0f);

    /* Z axis */
    r.set_up(vfp_t(1.5f, 1.5f, 1.5f, 1.5f), vfp_t(1.5f, 1.5f, 0.0f, 3.0f), vfp_t(0.0f, 3.0f, 1.5f, 1.5f), vfp_t(0.0f, 0.0f, 0.0f, 0.0f), vfp_t(0.0f, 0.0f, 1.0f, -1.0f), vfp_t( 1.0f, -1.0f, 0.0f, 0.0f));

    const vfp_t i_rd1[3] = { 1.0f / vfp_t(0.0f, 0.0f, 0.0f, 0.0f), 1.0f / vfp_t(0.0f, 0.0f, 1.0f, -1.0f), 1.0f / vfp_t( 1.0f, -1.0f, 0.0f, 0.0f) };

    const vfp_t res1(uut.intersection_distance(r, i_rd1));
    BOOST_CHECK(res1[0] == 1.0f);
    BOOST_CHECK(res1[1] == 1.0f);
}

BOOST_AUTO_TEST_CASE( vector_intersection_distance_point_and_spread_test )
{
    uut.create_leaf_node(point_t(1.0f, 1.0f, 1.0f), point_t(-1.0f, -1.0f, -1.0f), 6, 6);

    packet_ray r(point_t(0.0f, 0.0f, -2.0f), vfp_zero, vfp_zero, vfp_zero);

    const point_t dir0_0(1.0f / normalise(point_t( 1.0f,  1.0f, 1.0f)));
    const point_t dir0_1(1.0f / normalise(point_t(-1.0f,  1.0f, 1.0f)));
    const point_t dir0_2(1.0f / normalise(point_t( 1.0f, -1.0f, 1.0f)));
    const point_t dir0_3(1.0f / normalise(point_t(-1.0f, -1.0f, 1.0f)));
    const vfp_t i_rd0[3] = { vfp_t(dir0_0.x, dir0_1.x, dir0_2.x, dir0_3.x), vfp_t(dir0_0.y, dir0_1.y, dir0_2.y, dir0_3.y), vfp_t(dir0_0.z, dir0_1.z, dir0_2.z, dir0_3.z) };

    const vfp_t res0(uut.intersection_distance(r, i_rd0));

    BOOST_CHECK_CLOSE(res0[0], 1.732051f, result_tolerance);
    BOOST_CHECK_CLOSE(res0[1], 1.732051f, result_tolerance);
    BOOST_CHECK_CLOSE(res0[2], 1.732051f, result_tolerance);
    BOOST_CHECK_CLOSE(res0[3], 1.732051f, result_tolerance);

    /* Further away and test again */
    uut.create_leaf_node(point_t(11.0f, 11.0f, 11.0f), point_t(9.0f, 9.0f, 9.0f), 6, 6);

    const point_t dir1_0(1.0f / normalise(point_t(11.0f, 11.0f, 11.0f)));
    const point_t dir1_1(1.0f / normalise(point_t( 9.0f, 11.0f, 11.0f)));
    const point_t dir1_2(1.0f / normalise(point_t(11.0f,  9.0f, 11.0f)));
    const point_t dir1_3(1.0f / normalise(point_t( 9.0f,  9.0f, 11.0f)));
    const vfp_t i_rd1[3] = { vfp_t(dir1_0.x, dir1_1.x, dir1_2.x, dir1_3.x), vfp_t(dir1_0.y, dir1_1.y, dir1_2.y, dir1_3.y), vfp_t(dir1_0.z, dir1_1.z, dir1_2.z, dir1_3.z) };

    const vfp_t res1(uut.intersection_distance(r, i_rd1));

    BOOST_CHECK_CLOSE(res1[0], 19.0525589f, result_tolerance);
    BOOST_CHECK_CLOSE(res1[1], 17.9722004f, result_tolerance);
    BOOST_CHECK_CLOSE(res1[2], 17.9722004f, result_tolerance);
    BOOST_CHECK_CLOSE(res1[3], 16.8226051f, result_tolerance);

    /* Further away and test again */
    uut.create_leaf_node(point_t(101.0f, 101.0f, 101.0f), point_t(99.0f, 99.0f, 99.0f), 6, 6);

    const point_t dir2_0(1.0f / normalise(point_t(101.0f, 101.0f, 101.0f)));
    const point_t dir2_1(1.0f / normalise(point_t( 99.0f, 101.0f, 101.0f)));
    const point_t dir2_2(1.0f / normalise(point_t(101.0f,  99.0f, 101.0f)));
    const point_t dir2_3(1.0f / normalise(point_t( 99.0f,  99.0f, 101.0f)));
    const vfp_t i_rd2[3] = { vfp_t(dir2_0.x, dir2_1.x, dir2_2.x, dir2_3.x), vfp_t(dir2_0.y, dir2_1.y, dir2_2.y, dir2_3.y), vfp_t(dir2_0.z, dir2_1.z, dir2_2.z, dir2_3.z) };

    const vfp_t res2(uut.intersection_distance(r, i_rd2));

    BOOST_CHECK_CLOSE(res2[0], 174.937149f, result_tolerance);
    BOOST_CHECK_CLOSE(res2[1], 173.7901f,   result_tolerance);
    BOOST_CHECK_CLOSE(res2[2], 173.7901f,   result_tolerance);
    BOOST_CHECK_CLOSE(res2[3], 172.635468f, result_tolerance);

    /* Further away and test again */
    uut.create_leaf_node(point_t(1001.0f, 1001.0f, 1001.0f), point_t(999.0f, 999.0f, 999.0f), 6, 6);

    const point_t dir3_0(1.0f / normalise(point_t(1001.0f, 1001.0f, 1001.0f)));
    const point_t dir3_1(1.0f / normalise(point_t( 999.0f, 1001.0f, 1001.0f)));
    const point_t dir3_2(1.0f / normalise(point_t(1001.0f,  999.0f, 1001.0f)));
    const point_t dir3_3(1.0f / normalise(point_t( 999.0f,  999.0f, 1001.0f)));
    const vfp_t i_rd3[3] = { vfp_t(dir3_0.x, dir3_1.x, dir3_2.x, dir3_3.x), vfp_t(dir3_0.y, dir3_1.y, dir3_2.y, dir3_3.y), vfp_t(dir3_0.z, dir3_1.z, dir3_2.z, dir3_3.z) };

    const vfp_t res3(uut.intersection_distance(r, i_rd3));

    BOOST_CHECK_CLOSE(res3[0], 1733.78296f, result_tolerance);
    BOOST_CHECK_CLOSE(res3[1], 1732.62891f, result_tolerance);
    BOOST_CHECK_CLOSE(res3[2], 1732.62891f, result_tolerance);
    BOOST_CHECK_CLOSE(res3[3], 1731.47424f, result_tolerance);
}

BOOST_AUTO_TEST_SUITE_END()
}; /* namespace test */
}; /* namespace raptor_raytracer */
