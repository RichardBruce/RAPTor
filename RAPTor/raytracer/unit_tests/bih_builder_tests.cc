#ifdef STAND_ALONE
#define BOOST_TEST_MODULE simd test

/* Common headers */
#include "logging.h"

/* Initialise logger */
const raptor_physics::init_logger init_logger;
#endif /* #ifdef STAND_ALONE */

/* Standard headers */

/* Boost headerrs */
#include "boost/test/unit_test.hpp"

/* Raytracer headers */
#include "bih_builder.h"


namespace raptor_raytracer
{
namespace test
{
const float result_tolerance = 0.00001;

struct bih_builder_fixture
{
    bih_builder_fixture() {  }

    ~bih_builder_fixture()
    {
        for (auto *p : primitives)
        {
            delete p;
        }
    }

    void x_axis_primitives(const int size)
    {
        float x = 0.0f;
        for (int i = 0; i < size; ++i)
        {
            primitives.push_back(new triangle(nullptr, point_t(x, 0.0f, 0.0f), point_t(x, 0.0f, 1.0f), point_t(x, 1.0f, 1.0f)));
            x += 5.0f;
        }
    }

    primitive_list primitives;
    const float half_epsilon = 0.5f * EPSILON;
};

BOOST_FIXTURE_TEST_SUITE( bih_builder_tests, bih_builder_fixture );

// tests
BOOST_AUTO_TEST_CASE( empty_build_test )
{
    auto bih = build_bih(&primitives);

    /* Checks - One empty leaf */
    BOOST_CHECK(bih->size()                 == 1);
    BOOST_CHECK(bih->at(0).get_split_axis() == axis_t::not_set);
    BOOST_CHECK(bih->at(0).is_empty());
}

BOOST_AUTO_TEST_CASE( one_triangle_build_test )
{
    primitives.push_back(new triangle(nullptr, point_t(0.0f, 0.0f, 0.0f), point_t(0.0f, 0.0f, 1.0f), point_t(0.0f, 1.0f, 1.0f)));

    auto bih = build_bih(&primitives);
    
    /* Checks - One leaf of one primitive */
    BOOST_CHECK(bih->size()                 == (primitives.size() * 3));
    BOOST_CHECK(bih->at(0).get_split_axis() == axis_t::not_set);
    BOOST_CHECK(bih->at(0).size()           == 1);
}

BOOST_AUTO_TEST_CASE( full_node_build_test )
{
    x_axis_primitives(MAX_BIH_NODE_SIZE);

    auto bih = build_bih(&primitives);
    
    /* Checks - One leaf full of primitives */
    BOOST_CHECK(bih->size()                 == (primitives.size() * 3));
    BOOST_CHECK(bih->at(0).get_split_axis() == axis_t::not_set);
    BOOST_CHECK(bih->at(0).size()           == MAX_BIH_NODE_SIZE);
}

BOOST_AUTO_TEST_CASE( regular_tree_build_test )
{
    x_axis_primitives(4);

    auto bih = build_bih(&primitives, 1);
    
    /* Checks - Root internal node */
    BOOST_CHECK(bih->size()                     == (primitives.size() * 3));
    BOOST_REQUIRE(bih->at(0).get_split_axis()   == axis_t::x_axis);
    BOOST_CHECK_CLOSE(bih->at(0).get_left_split(),   5.0f + half_epsilon, result_tolerance);
    BOOST_CHECK_CLOSE(bih->at(0).get_right_split(), 10.0f - half_epsilon, result_tolerance);

    /* Descend left sub tree */
    auto l0 = bih->at(0).get_left_child();
    BOOST_REQUIRE(l0->get_split_axis() == axis_t::x_axis);
    BOOST_CHECK_CLOSE(l0->get_left_split(),  0.0f + half_epsilon, result_tolerance);
    BOOST_CHECK_CLOSE(l0->get_right_split(), 5.0f - half_epsilon, result_tolerance);

    /* Left tree leaf nodes*/
    auto leaf0 = l0->get_left_child();
    BOOST_CHECK(leaf0->get_split_axis() == axis_t::not_set);
    BOOST_CHECK(leaf0->size()           == 1);

    auto leaf1 = l0->get_right_child();
    BOOST_CHECK(leaf1->get_split_axis() == axis_t::not_set);
    BOOST_CHECK(leaf1->size()           == 1);
    
    /* Descend right sub tree */
    auto r0 = bih->at(0).get_right_child();
    BOOST_REQUIRE_MESSAGE(r0->get_split_axis() == axis_t::x_axis, static_cast<int>(r0->get_split_axis()));
    BOOST_CHECK_CLOSE(r0->get_left_split(),  10.0f + half_epsilon, result_tolerance);
    BOOST_CHECK_CLOSE(r0->get_right_split(), 15.0f - half_epsilon, result_tolerance);

    /* Right tree leaf nodes */
    auto leaf2 = r0->get_left_child();
    BOOST_CHECK(leaf2->get_split_axis() == axis_t::not_set);
    BOOST_CHECK(leaf2->size()           == 1);

    auto leaf3 = r0->get_right_child();
    BOOST_CHECK(leaf3->get_split_axis() == axis_t::not_set);
    BOOST_CHECK(leaf3->size()           == 1);
}

BOOST_AUTO_TEST_CASE( dislocated_primitive_build_test )
{
    x_axis_primitives(4);
    primitives.push_back(new triangle(nullptr, point_t(5.0f, 20.0f, 0.0f), point_t(5.0f, 20.0f, 1.0f), point_t(5.0f, 21.0f, 1.0f)));

    auto bih = build_bih(&primitives, 1);
    
    /* Checks - Root internal node */
    BOOST_CHECK(bih->size()                     == (primitives.size() * 3));
    BOOST_REQUIRE(bih->at(0).get_split_axis()   == axis_t::y_axis);
    BOOST_CHECK_CLOSE(bih->at(0).get_left_split(),   1.0f, result_tolerance);
    BOOST_CHECK_CLOSE(bih->at(0).get_right_split(), 20.0f, result_tolerance);

    /* Descend left sub tree */
    auto x_root = bih->at(0).get_left_child();
    BOOST_REQUIRE(x_root->get_split_axis() == axis_t::x_axis);
    BOOST_CHECK_CLOSE(x_root->get_left_split(),   5.0f + half_epsilon, result_tolerance);
    BOOST_CHECK_CLOSE(x_root->get_right_split(), 10.0f - half_epsilon, result_tolerance);

    /* Descend left sub tree */
    auto l0 = x_root->get_left_child();
    BOOST_REQUIRE(l0->get_split_axis() == axis_t::x_axis);
    BOOST_CHECK_CLOSE(l0->get_left_split(),  0.0f + half_epsilon, result_tolerance);
    BOOST_CHECK_CLOSE(l0->get_right_split(), 5.0f - half_epsilon, result_tolerance);

    /* Left tree leaf nodes*/
    auto leaf0 = l0->get_left_child();
    BOOST_CHECK(leaf0->get_split_axis() == axis_t::not_set);
    BOOST_CHECK(leaf0->size()           == 1);

    auto leaf1 = l0->get_right_child();
    BOOST_CHECK(leaf1->get_split_axis() == axis_t::not_set);
    BOOST_CHECK(leaf1->size()           == 1);
    
    /* Descend right sub tree */
    auto r0 = x_root->get_right_child();
    BOOST_REQUIRE_MESSAGE(r0->get_split_axis() == axis_t::x_axis, static_cast<int>(r0->get_split_axis()));
    BOOST_CHECK_CLOSE(r0->get_left_split(),  10.0f + half_epsilon, result_tolerance);
    BOOST_CHECK_CLOSE(r0->get_right_split(), 15.0f - half_epsilon, result_tolerance);

    /* Right tree leaf nodes */
    auto leaf2 = r0->get_left_child();
    BOOST_CHECK(leaf2->get_split_axis() == axis_t::not_set);
    BOOST_CHECK(leaf2->size()           == 1);

    auto leaf3 = r0->get_right_child();
    BOOST_CHECK(leaf3->get_split_axis() == axis_t::not_set);
    BOOST_CHECK(leaf3->size()           == 1);
}

BOOST_AUTO_TEST_CASE( xyz_tree_build_test )
{
    primitives.push_back(new triangle(nullptr, point_t( 0.0f,  0.0f,  0.0f), point_t( 0.0f,  0.0f,  1.0f), point_t( 0.0f,  1.0f,  1.0f)));
    primitives.push_back(new triangle(nullptr, point_t(10.0f,  0.0f,  0.0f), point_t(10.0f,  0.0f,  1.0f), point_t(10.0f,  1.0f,  1.0f)));
    primitives.push_back(new triangle(nullptr, point_t( 0.0f, 15.0f,  0.0f), point_t( 0.0f, 15.0f,  1.0f), point_t( 0.0f, 16.0f,  1.0f)));
    primitives.push_back(new triangle(nullptr, point_t( 0.0f,  0.0f, 20.0f), point_t( 0.0f,  0.0f, 21.0f), point_t( 0.0f,  1.0f, 21.0f)));
    auto bih = build_bih(&primitives, 1);
    
    /* Checks - Root internal node */
    BOOST_CHECK(bih->size()                     == (primitives.size() * 3));
    BOOST_REQUIRE(bih->at(0).get_split_axis()   == axis_t::z_axis);
    BOOST_CHECK_CLOSE(bih->at(0).get_left_split(),   1.0f, result_tolerance);
    BOOST_CHECK_CLOSE(bih->at(0).get_right_split(), 20.0f, result_tolerance);

    /* Descend the not z tree */
    auto not_z_tree = bih->at(0).get_left_child();
    BOOST_CHECK(not_z_tree->get_split_axis() == axis_t::y_axis);
    BOOST_CHECK_CLOSE(not_z_tree->get_left_split(),   1.0f, result_tolerance);
    BOOST_CHECK_CLOSE(not_z_tree->get_right_split(), 15.0f, result_tolerance);

    /* Descend the not y tree */
    auto not_y_tree = not_z_tree->get_left_child();
    BOOST_CHECK(not_y_tree->get_split_axis() == axis_t::x_axis);
    BOOST_CHECK_CLOSE(not_y_tree->get_left_split(),   0.0f + half_epsilon, result_tolerance);
    BOOST_CHECK_CLOSE(not_y_tree->get_right_split(), 10.0f - half_epsilon, result_tolerance);

    /* Leaf containing the origin */
    auto origin_leaf = not_y_tree->get_left_child();
    BOOST_CHECK(origin_leaf->get_split_axis() == axis_t::not_set);
    BOOST_CHECK(origin_leaf->size()           == 1);

    /* Leaf containing the x extreme */
    auto x_leaf = not_y_tree->get_right_child();
    BOOST_CHECK(x_leaf->get_split_axis() == axis_t::not_set);
    BOOST_CHECK(x_leaf->size()           == 1);

    /* Out of not y tree*/
    /* Descend the y tree, leaf containing the y extreme */
    auto y_leaf = not_z_tree->get_right_child();
    BOOST_CHECK(y_leaf->get_split_axis() == axis_t::not_set);
    BOOST_CHECK(y_leaf->size()           == 1);

    /* Out of y tree*/
    /* Out of not z tree*/

    /* Descend the z tree, leaf containing z extreme */
    auto z_leaf = bih->at(0).get_right_child();
    BOOST_CHECK(z_leaf->get_split_axis() == axis_t::not_set);
    BOOST_CHECK(z_leaf->size()           == 1);
}

BOOST_AUTO_TEST_CASE( xz_diagonal_tree_build_test )
{
    primitives.push_back(new triangle(nullptr, point_t(-10.0f, 0.0f, -20.0f), point_t(-10.0f, 0.0f, -19.0f), point_t(-20.0f, 1.0f, -19.0f)));
    primitives.push_back(new triangle(nullptr, point_t( 10.0f, 0.0f,  20.0f), point_t( 10.0f, 0.0f,  21.0f), point_t( 20.0f, 1.0f,  21.0f)));
    auto bih = build_bih(&primitives, 1);
    
    /* Checks - Root internal node */
    BOOST_CHECK(bih->size()                     == (primitives.size() * 3));
    BOOST_REQUIRE(bih->at(0).get_split_axis()   == axis_t::z_axis);
    BOOST_CHECK_CLOSE(bih->at(0).get_left_split(),  -19.0f, result_tolerance);
    BOOST_CHECK_CLOSE(bih->at(0).get_right_split(),  20.0f, result_tolerance);

    /* Leaf containing the origin */
    auto leaf0 = bih->at(0).get_left_child();
    BOOST_CHECK(leaf0->get_split_axis() == axis_t::not_set);
    BOOST_CHECK(leaf0->size()           == 1);

    auto leaf1 = bih->at(0).get_left_child();
    BOOST_CHECK(leaf1->get_split_axis() == axis_t::not_set);
    BOOST_CHECK(leaf1->size()           == 1);
}

BOOST_AUTO_TEST_SUITE_END()
}; // namespace test
}; // namespace raptor_raytracer
