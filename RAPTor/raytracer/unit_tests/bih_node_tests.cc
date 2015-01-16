#ifdef STAND_ALONE
#define BOOST_TEST_MODULE bih_node test

/* Common headers */
#include "logging.h"

/* Initialise logger */
const raptor_physics::init_logger init_logger;

namespace raptor_raytracer
{
/* Primitives array so we dont need to include more source */
std::vector<triangle *> *   bih_node::o     = nullptr;
}
#endif /* #ifdef STAND_ALONE */

/* Standard headers */
#include <array>

/* Boost headerrs */
#include "boost/test/unit_test.hpp"

/* Ray tracer headers */
#include "bih_node.h"
#include "phong_shader.h"


namespace raptor_raytracer
{
namespace test
{
const float result_tolerance = 0.00001f;

struct bih_node_fixture
{
    bih_node_fixture() :
    mat(new phong_shader(ext_colour_t(255.0f, 255.0f, 255.0f))), tris(6)
    {
        /* A square light */
        tris[0] = new triangle(mat.get(), point_t(0.0f, 0.0f, 0.0f), point_t(0.0f, 1.0f, 0.0f), point_t(0.0f, 1.0f, 1.0f), true);
        tris[1] = new triangle(mat.get(), point_t(0.0f, 0.0f, 0.0f), point_t(0.0f, 1.0f, 1.0f), point_t(0.0f, 0.0f, 1.0f), true);

        /* A more distant square */
        tris[2] = new triangle(mat.get(), point_t(2.0f, 0.0f, 0.0f), point_t(2.0f, 1.0f, 0.0f), point_t(2.0f, 1.0f, 1.0f), false);
        tris[3] = new triangle(mat.get(), point_t(2.0f, 0.0f, 0.0f), point_t(2.0f, 1.0f, 1.0f), point_t(2.0f, 0.0f, 1.0f), false);

        /* A square */
        tris[4] = new triangle(mat.get(), point_t(1.0f, 0.0f, 0.0f), point_t(1.0f, 1.0f, 0.0f), point_t(1.0f, 1.0f, 1.0f), false);
        tris[5] = new triangle(mat.get(), point_t(1.0f, 0.0f, 0.0f), point_t(1.0f, 1.0f, 1.0f), point_t(1.0f, 0.0f, 1.0f), false);

        /* Set for nodes to use */
        bih_node::set_primitives(&tris);
    }

    ~bih_node_fixture()
    {
        for (auto *t : tris)
        {
            delete t;
        }
    }

    bih_node                    uut;
    std::unique_ptr<material>   mat;
    std::vector<triangle *>     tris;
};

BOOST_FIXTURE_TEST_SUITE( bih_node_tests, bih_node_fixture );

/* Leaf node */
BOOST_AUTO_TEST_CASE( create_empty_leaf_node_test )
{
    uut.create_leaf_node(6, 5);

    BOOST_CHECK(uut.is_empty());
    BOOST_CHECK(uut.size() == 0);
}

BOOST_AUTO_TEST_CASE( create_leaf_node_test )
{
    uut.create_leaf_node(6, 19);

    BOOST_CHECK(!uut.is_empty());
    BOOST_CHECK(uut.size() == 14);
}

/* Generic node */
BOOST_AUTO_TEST_CASE( create_generic_node_test )
{
    uut.create_generic_node(4.2f, 7.3f);

    BOOST_CHECK(uut.get_left_split()    == 4.2f);
    BOOST_CHECK(uut.get_right_split()   == 7.3f);
}

/* Nearest tests */
BOOST_AUTO_TEST_CASE( ray_test_leaf_node_nearest )
{
    uut.create_leaf_node(2, 5);

    /* A hit */
    hit_description h0;
    ray r0(point_t(0.0f, 0.5f, 0.25f), 1.0f);
    BOOST_CHECK(uut.test_leaf_node_nearest(&r0, &h0) == tris[4]);
    BOOST_CHECK_CLOSE(h0.d, 1.0f, result_tolerance);

    /* A miss */
    hit_description h1;
    ray r1(point_t(5.0f, 0.5f, 0.25f), 1.0f);
    BOOST_CHECK(uut.test_leaf_node_nearest(&r1, &h1) == nullptr);
    BOOST_CHECK(h1.h == hit_t::miss);
}

BOOST_AUTO_TEST_CASE( packet_ray_test_leaf_node_nearest )
{
    uut.create_leaf_node(2, 5);

    /* Some hits and misses */
    packet_hit_description h0;
    const triangle *i_o0[4] = { nullptr };
    packet_ray r0(point_t(0.0f, 0.5f, 0.25f), vfp_t(1.0f), vfp_t(0.0f), vfp_t(0.0f, 0.0f, 0.0f, 1.0f));
    uut.test_leaf_node_nearest(&r0, i_o0, &h0, 1);
    BOOST_CHECK(i_o0[0] == tris[4]);
    BOOST_CHECK(i_o0[1] == tris[4]);
    BOOST_CHECK(i_o0[2] == tris[4]);
    BOOST_CHECK(i_o0[3] == nullptr);
    BOOST_CHECK_CLOSE(h0.d[0], 1.0f, result_tolerance);
    BOOST_CHECK_CLOSE(h0.d[1], 1.0f, result_tolerance);
    BOOST_CHECK_CLOSE(h0.d[2], 1.0f, result_tolerance);
}

BOOST_AUTO_TEST_CASE( frustrum_test_leaf_node_nearest )
{
    uut.create_leaf_node(2, 5);

    /* Some hits and misses */
    packet_hit_description h;
    const triangle *i_o[4] = { nullptr };
    packet_ray r(point_t(0.0f, 0.5f, 0.25f), vfp_t(1.0f), vfp_t(0.0f, 0.05f, 0.05f, 0.0f), vfp_t(0.0f, 0.0f, 0.05f, 1.0f));

    /* Build the frustrum and adapt it to the leaf node */
    /* Note - We have given some generous proportions to the leaf nodes */
    frustrum f(&r, 1);
    unsigned int packet_addr[1] = { 0 };
    f.adapt_to_leaf(&r, point_t(5.0f, 5.0f, 5.0f), point_t(-5.0f, -5.0f, -5.0f), packet_addr, 1);

    uut.test_leaf_node_nearest(f, &r, i_o, &h, packet_addr, 1);
    BOOST_CHECK(i_o[0] == tris[4]);
    BOOST_CHECK(i_o[1] == tris[4]);
    BOOST_CHECK(i_o[2] == tris[4]);
    BOOST_CHECK(i_o[3] == nullptr);
    BOOST_CHECK_CLOSE(h.d[0], 1.0f, result_tolerance);
    BOOST_CHECK_CLOSE(h.d[1], 1.0f, result_tolerance);
    BOOST_CHECK_CLOSE(h.d[2], 1.0f, result_tolerance);
}

BOOST_AUTO_TEST_CASE( tris_culled_frustrum_test_leaf_node_nearest )
{
    uut.create_leaf_node(2, 5);

    /* Some hits and misses */
    packet_hit_description h;
    const triangle *i_o[4] = { nullptr };
    packet_ray r(point_t(0.0f, 0.5f, 0.25f), vfp_t(1.0f), vfp_t(0.0f, 0.05f, 0.05f, 0.0f), vfp_t(1.0f, 1.0f, 1.05f, 1.0f));

    /* Build the frustrum and adapt it to the leaf node */
    /* Note - We have given some generous proportions to the leaf nodes */
    frustrum f(&r, 1);
    unsigned int packet_addr[1] = { 0 };
    f.adapt_to_leaf(&r, point_t(5.0f, 5.0f, 5.0f), point_t(-5.0f, -5.0f, -5.0f), packet_addr, 1);

    uut.test_leaf_node_nearest(f, &r, i_o, &h, packet_addr, 1);
    BOOST_CHECK(i_o[0] == nullptr);
    BOOST_CHECK(i_o[1] == nullptr);
    BOOST_CHECK(i_o[2] == nullptr);
    BOOST_CHECK(i_o[3] == nullptr);
}

/* Nearer tests */
BOOST_AUTO_TEST_CASE( ray_test_leaf_node_nearer )
{
    uut.create_leaf_node(0, 5);

    /* Hit */
    ray r(point_t(-1.0f, 0.5f, 0.25f), 1.0f);
    BOOST_CHECK(uut.test_leaf_node_nearer(&r, 5.0f));
}

BOOST_AUTO_TEST_CASE( ray_hit_light_test_leaf_node_nearer )
{
    uut.create_leaf_node(0, 5);

    /* Only enough range to hit a light */
    ray r(point_t(-1.0f, 0.5f, 0.25f), 1.0f);
    BOOST_CHECK(!uut.test_leaf_node_nearer(&r, 1.5f));
}

BOOST_AUTO_TEST_CASE( ray_miss_test_leaf_node_nearer )
{
    uut.create_leaf_node(0, 5);

    /* Miss */
    ray r(point_t(5.0f, 0.5f, 0.25f), 1.0f);
    BOOST_CHECK(!uut.test_leaf_node_nearer(&r, 5.0f));
}

BOOST_AUTO_TEST_CASE( packet_ray_test_leaf_node_nearer )
{
    uut.create_leaf_node(0, 5);

    /* Some hits and misses */
    vfp_t closer0(vfp_zero);
    packet_hit_description h0;
    packet_ray r0(point_t(-1.0f, 0.5f, 0.25f), vfp_t(1.0f), vfp_t(0.0f), vfp_t(0.0f, 0.0f, 0.0f, 1.0f));
    uut.test_leaf_node_nearer(&r0, &closer0, vfp_t(5.0f), &h0);
    BOOST_CHECK_CLOSE(h0.d[0], 2.0f, result_tolerance);
    BOOST_CHECK_CLOSE(h0.d[1], 2.0f, result_tolerance);
    BOOST_CHECK_CLOSE(h0.d[2], 2.0f, result_tolerance);
    BOOST_CHECK(move_mask(closer0) == 0x7);
}

BOOST_AUTO_TEST_CASE( packet_ray_hit_light_test_leaf_node_nearer )
{
    uut.create_leaf_node(0, 5);

    /* Only enough range to hit a light */
    vfp_t closer(vfp_zero);
    packet_hit_description h;
    packet_ray r(point_t(-1.0f, 0.5f, 0.25f), vfp_t(1.0f), vfp_t(0.0f), vfp_t(0.0f, 0.0f, 0.0f, 1.0f));
    uut.test_leaf_node_nearer(&r, &closer, vfp_t(1.5f), &h);
    BOOST_CHECK(move_mask(closer) == 0x0);
}

BOOST_AUTO_TEST_CASE( packet_ray_all_hit_test_leaf_node_nearer )
{
    uut.create_leaf_node(0, 5);
    
    /* All hit, early exit */
    vfp_t closer(vfp_zero);
    packet_hit_description h;
    packet_ray r(point_t(-1.0f, 0.5f, 0.25f), vfp_t(1.0f), vfp_t(0.0f), vfp_t(0.0f));
    uut.test_leaf_node_nearer(&r, &closer, vfp_t(5.0f), &h);
    BOOST_CHECK_CLOSE(h.d[0], 3.0f, result_tolerance);  /* Distance is 3 so we didnt bother testing the closer triangle */
    BOOST_CHECK_CLOSE(h.d[1], 3.0f, result_tolerance);
    BOOST_CHECK_CLOSE(h.d[2], 3.0f, result_tolerance);
    BOOST_CHECK_CLOSE(h.d[3], 3.0f, result_tolerance);
    BOOST_CHECK(move_mask(closer) == 0xf);
}

BOOST_AUTO_TEST_CASE( frustrum_test_leaf_node_nearer )
{
    uut.create_leaf_node(0, 5);

    /* Some hits and misses */
    packet_ray r(point_t(-1.0f, 0.5f, 0.25f), vfp_t(1.0f), vfp_t(0.0f, 0.05f, 0.05f, 0.0f), vfp_t(0.0f, 0.0f, 0.05f, 1.0f));

    /* Build the frustrum and adapt it to the leaf node */
    /* Note - We have given some generous proportions to the leaf nodes */
    frustrum f(&r, 1);
    unsigned int packet_addr[1] = { 0 };
    f.adapt_to_leaf(&r, point_t(5.0f, 5.0f, 5.0f), point_t(-5.0f, -5.0f, -5.0f), packet_addr, 1);

    vfp_t t(5.0f, 2.5f, 2.5f, 5.0f);
    vfp_t closer(vfp_zero);
    packet_hit_description h;
    uut.test_leaf_node_nearer(f, &r, &closer, &t, &h, packet_addr, 1);
    BOOST_CHECK_CLOSE(h.d[0], 2.0f, result_tolerance);
    BOOST_CHECK_CLOSE(h.d[1], 2.0f, result_tolerance);
    BOOST_CHECK_CLOSE(h.d[2], 2.0f, result_tolerance);
    BOOST_CHECK(move_mask(closer) == 0x7);
}

BOOST_AUTO_TEST_CASE( frustrum_hit_light_test_leaf_node_nearer )
{
    uut.create_leaf_node(0, 5);

    /* Some hits and misses */
    packet_ray r(point_t(-1.0f, 0.5f, 0.25f), vfp_t(1.0f), vfp_t(0.0f, 0.05f, 0.05f, 0.0f), vfp_t(1.0f, 1.0f, 1.05f, 1.0f));

    /* Build the frustrum and adapt it to the leaf node */
    /* Note - We have given some generous proportions to the leaf nodes */
    frustrum f(&r, 1);
    unsigned int packet_addr[1] = { 0 };
    f.adapt_to_leaf(&r, point_t(5.0f, 5.0f, 5.0f), point_t(-5.0f, -5.0f, -5.0f), packet_addr, 1);

    vfp_t t(1.5f);
    vfp_t closer(vfp_zero);
    packet_hit_description h;
    uut.test_leaf_node_nearer(f, &r, &closer, &t, &h, packet_addr, 1);
    BOOST_CHECK(move_mask(closer) == 0x0);
}

BOOST_AUTO_TEST_CASE( frustrum_all_hit_test_leaf_node_nearer )
{
    uut.create_leaf_node(0, 5);

    /* Some hits and misses */
    packet_ray r(point_t(-1.0f, 0.5f, 0.25f), vfp_t(1.0f), vfp_t(0.0f, 0.05f, 0.05f, 0.0f), vfp_t(0.0f, 0.0f, 0.05f, 0.05f));

    /* Build the frustrum and adapt it to the leaf node */
    /* Note - We have given some generous proportions to the leaf nodes */
    frustrum f(&r, 1);
    unsigned int packet_addr[1] = { 0 };
    f.adapt_to_leaf(&r, point_t(5.0f, 5.0f, 5.0f), point_t(-5.0f, -5.0f, -5.0f), packet_addr, 1);

    vfp_t t(5.0f);
    vfp_t closer(vfp_zero);
    packet_hit_description h;
    uut.test_leaf_node_nearer(f, &r, &closer, &t, &h, packet_addr, 1);
    BOOST_CHECK_CLOSE(h.d[0], 3.0f, result_tolerance);
    BOOST_CHECK_CLOSE(h.d[1], 3.0f, result_tolerance);
    BOOST_CHECK_CLOSE(h.d[2], 3.0f, result_tolerance);
    BOOST_CHECK_CLOSE(h.d[3], 3.0f, result_tolerance);
    BOOST_CHECK(move_mask(closer) == 0xf);
}

BOOST_AUTO_TEST_CASE( tris_culled_frustrum_test_leaf_node_nearer )

{
    uut.create_leaf_node(0, 5);

    /* Some hits and misses */
    packet_ray r(point_t(-1.0f, 0.5f, 0.25f), vfp_t(1.0f), vfp_t(0.0f, 0.05f, 0.05f, 0.0f), vfp_t(1.0f, 1.0f, 1.05f, 1.0f));

    /* Build the frustrum and adapt it to the leaf node */
    /* Note - We have given some generous proportions to the leaf nodes */
    frustrum f(&r, 1);
    unsigned int packet_addr[1] = { 0 };
    f.adapt_to_leaf(&r, point_t(5.0f, 5.0f, 5.0f), point_t(-5.0f, -5.0f, -5.0f), packet_addr, 1);

    vfp_t t(5.0f);
    vfp_t closer(vfp_zero);
    packet_hit_description h;
    uut.test_leaf_node_nearer(f, &r, &closer, &t, &h, packet_addr, 1);
    BOOST_CHECK(move_mask(closer) == 0x0);
}

BOOST_AUTO_TEST_SUITE_END()
}; /* namespace test */
}; /* namespace raptor_raytracer */
