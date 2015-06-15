#ifdef STAND_ALONE
#define BOOST_TEST_MODULE bih_builder test

/* Common headers */
#include "logging.h"

/* Initialise logger */
const raptor_physics::init_logger init_logger;
#endif /* #ifdef STAND_ALONE */

/* Standard headers */

/* Boost headerrs */
#include "boost/test/unit_test.hpp"

/* Ray tracer headers */
#include "bih_builder.h"


namespace raptor_raytracer
{
namespace test
{
const float result_tolerance = 0.00001f;

struct bih_builder_fixture
{
    bih_builder_fixture() :
    uut(1), half_epsilon(0.5f * EPSILON)
    {  }

    void x_axis_primitives(const int size)
    {
        float x = 0.0f;
        for (int i = 0; i < size; ++i)
        {
            primitives.emplace_back(nullptr, point_t(x, 0.0f, 0.0f), point_t(x, 0.0f, 1.0f), point_t(x, 1.0f, 1.0f));
            x += 5.0f;
        }
    }

    primitive_store         primitives;
    std::vector<bih_block>  bih;
    bih_builder             uut;
    const float             half_epsilon;
};

BOOST_FIXTURE_TEST_SUITE( bih_builder_tests, bih_builder_fixture );

/* Tests */
BOOST_AUTO_TEST_CASE( empty_build_test )
{
    uut.build(&primitives, &bih);

    /* Checks - One empty leaf */
    BOOST_CHECK(bih.size()                  == 1);
    BOOST_CHECK(bih.at(0).get_split_axis(0) == axis_t::not_set);
    BOOST_CHECK(bih.at(0).get_node(0)->is_empty());
}

BOOST_AUTO_TEST_CASE( one_triangle_build_test )
{
    primitives.emplace_back(nullptr, point_t(0.0f, 0.0f, 0.0f), point_t(0.0f, 0.0f, 1.0f), point_t(0.0f, 1.0f, 1.0f));

    uut.build(&primitives, &bih);
    
    /* Checks - One leaf of one primitive */
    BOOST_CHECK(bih.size()                      == 1);
    BOOST_CHECK(bih.at(0).get_split_axis(0)     == axis_t::not_set);
    BOOST_CHECK(bih.at(0).get_node(0)->size()   == 1);
}

// BOOST_AUTO_TEST_CASE( full_node_build_test )
// {
//     x_axis_primitives(MAX_BIH_NODE_SIZE);

//     bih_builder uut(MAX_BIH_NODE_SIZE);
//     uut.build(&primitives, &bih);
    
//      Checks - One leaf full of primitives 
//     BOOST_CHECK(bih.size()                      == 2);
//     BOOST_CHECK(bih.at(0).get_split_axis(0)     == axis_t::not_set);
//     BOOST_CHECK(bih.at(0).get_node(0)->size()   == MAX_BIH_NODE_SIZE);
// }

// BOOST_AUTO_TEST_CASE( regular_tree_build_test )
// {
//     x_axis_primitives(4);

//     uut.build(&primitives, &bih);
    
//     /* Checks - Root internal node */
//     BOOST_CHECK(bih.size()                      == 2);
//     BOOST_REQUIRE(bih.at(0).get_split_axis(0)   == axis_t::x_axis);
//     BOOST_CHECK_CLOSE(bih.at(0).get_node(0)->get_left_split(),   5.0f + half_epsilon, result_tolerance);
//     BOOST_CHECK_CLOSE(bih.at(0).get_node(0)->get_right_split(), 10.0f - half_epsilon, result_tolerance);

//     /* Descend left sub tree */
//     int r0;
//     const int l0 = bih.at(0).get_siblings(&r0, 0, 0);
//     auto node_l0 = bih.at(block_index(l0)).get_node(node_index(l0));
//     BOOST_REQUIRE(bih.at(block_index(l0)).get_split_axis(node_index(l0)) == axis_t::x_axis);
//     BOOST_CHECK_CLOSE(node_l0->get_left_split(),  0.0f + half_epsilon, result_tolerance);
//     BOOST_CHECK_CLOSE(node_l0->get_right_split(), 5.0f - half_epsilon, result_tolerance);

//     /* Left tree leaf nodes*/
//     int leaf1;
//     const int leaf0 = bih.at(0).get_siblings(&leaf1, block_index(l0), node_index(l0));
//     auto node_leaf0 = bih.at(block_index(leaf0)).get_node(node_index(leaf0));
//     BOOST_CHECK(bih.at(block_index(leaf0)).get_split_axis(node_index(leaf0))    == axis_t::not_set);
//     BOOST_CHECK(node_leaf0->size()                                              == 1);

//     auto node_leaf1 = bih.at(block_index(leaf1)).get_node(node_index(leaf1));
//     BOOST_CHECK(bih.at(block_index(leaf1)).get_split_axis(node_index(leaf1))    == axis_t::not_set);
//     BOOST_CHECK(node_leaf1->size()                                              == 1);
    
//     /* Descend right sub tree */
//     auto node_r0 = bih.at(block_index(r0)).get_node(node_index(r0));
//     BOOST_REQUIRE(bih.at(block_index(r0)).get_split_axis(node_index(r0)) == axis_t::x_axis);
//     BOOST_CHECK_CLOSE(node_r0->get_left_split(),  10.0f + half_epsilon, result_tolerance);
//     BOOST_CHECK_CLOSE(node_r0->get_right_split(), 15.0f - half_epsilon, result_tolerance);

//     /* Right tree leaf nodes */
//     int leaf3;
//     const int leaf2 = bih.at(block_index(r0)).get_siblings(&leaf3, block_index(r0), node_index(r0));
//     auto node_leaf2 = bih.at(block_index(leaf2)).get_node(node_index(leaf2));
//     BOOST_CHECK(bih.at(block_index(leaf2)).get_split_axis(node_index(leaf2))    == axis_t::not_set);
//     BOOST_CHECK(node_leaf2->size()                                              == 1);

//     auto node_leaf3 = bih.at(block_index(leaf3)).get_node(node_index(leaf3));
//     BOOST_CHECK(bih.at(block_index(leaf3)).get_split_axis(node_index(leaf3))    == axis_t::not_set);
//     BOOST_CHECK(node_leaf3->size()                                              == 1);
// }

// BOOST_AUTO_TEST_CASE( dislocated_primitive_build_test )
// {
//     x_axis_primitives(4);
//     primitives.emplace_back(nullptr, point_t(5.0f, 20.0f, 0.0f), point_t(5.0f, 20.0f, 1.0f), point_t(5.0f, 21.0f, 1.0f));

//     uut.build(&primitives, &bih);
    
//     /* Checks - Root internal node */
//     BOOST_CHECK(bih.size()                     == 17);
//     BOOST_REQUIRE(bih.at(0).get_split_axis(0)  == axis_t::y_axis);
//     BOOST_CHECK_CLOSE(bih.at(0).get_node(0)->get_left_split(),   1.0f, result_tolerance);
//     BOOST_CHECK_CLOSE(bih.at(0).get_node(0)->get_right_split(), 20.0f, result_tolerance);

//     /* Descend x sub tree */
//     const int x_root = bih.at(0).get_left_child(0, 0);
//     auto x_root_node = bih.at(block_index(x_root)).get_node(node_index(x_root));
//     BOOST_REQUIRE(bih.at(block_index(x_root)).get_split_axis(node_index(x_root)) == axis_t::x_axis);
//     BOOST_CHECK_CLOSE(x_root_node->get_left_split(),   5.0f + half_epsilon, result_tolerance);
//     BOOST_CHECK_CLOSE(x_root_node->get_right_split(), 10.0f - half_epsilon, result_tolerance);

//     /* Descend left sub tree */
//     const int l0 = bih.at(block_index(x_root)).get_left_child(block_index(x_root), node_index(x_root));
//     auto node_l0 = bih.at(block_index(l0)).get_node(node_index(l0));
//     BOOST_REQUIRE(bih.at(block_index(l0)).get_split_axis(node_index(l0)) == axis_t::x_axis);
//     BOOST_CHECK_CLOSE(node_l0->get_left_split(),  0.0f + half_epsilon, result_tolerance);
//     BOOST_CHECK_CLOSE(node_l0->get_right_split(), 5.0f - half_epsilon, result_tolerance);

//     /* Left tree leaf nodes*/
//     const int leaf0 = bih.at(block_index(l0)).get_left_child(block_index(l0), node_index(l0));
//     auto node_leaf0 = bih.at(block_index(leaf0)).get_node(node_index(leaf0));
//     BOOST_CHECK(bih.at(block_index(leaf0)).get_split_axis(node_index(leaf0))    == axis_t::not_set);
//     BOOST_CHECK(node_leaf0->size()                                              == 1);

//     const int leaf1 = bih.at(block_index(l0)).get_right_child(block_index(l0), node_index(l0));
//     auto node_leaf1 = bih.at(block_index(leaf1)).get_node(node_index(leaf1));
//     BOOST_CHECK(bih.at(block_index(leaf1)).get_split_axis(node_index(leaf1))    == axis_t::not_set);
//     BOOST_CHECK(node_leaf1->size()                                              == 1);
    
//     /* Descend right sub tree */
//     const int r0 = bih.at(block_index(x_root)).get_right_child(block_index(x_root), node_index(x_root));
//     auto node_r0 = bih.at(block_index(r0)).get_node(node_index(r0));
//     BOOST_REQUIRE(bih.at(block_index(r0)).get_split_axis(node_index(r0)) == axis_t::x_axis);
//     BOOST_CHECK_CLOSE(node_r0->get_left_split(),  10.0f + half_epsilon, result_tolerance);
//     BOOST_CHECK_CLOSE(node_r0->get_right_split(), 15.0f - half_epsilon, result_tolerance);

//     /* Right tree leaf nodes */
//     const int leaf2 = bih.at(block_index(r0)).get_left_child(block_index(r0), node_index(r0));
//     auto node_leaf2 = bih.at(block_index(leaf2)).get_node(node_index(leaf2));
//     BOOST_CHECK(bih.at(block_index(leaf2)).get_split_axis(node_index(leaf2))    == axis_t::not_set);
//     BOOST_CHECK(node_leaf2->size()                                              == 1);

//     const int leaf3 = bih.at(block_index(r0)).get_left_child(block_index(r0), node_index(r0));
//     auto node_leaf3 = bih.at(block_index(leaf3)).get_node(node_index(leaf3));
//     BOOST_CHECK(bih.at(block_index(leaf3)).get_split_axis(node_index(leaf3))    == axis_t::not_set);
//     BOOST_CHECK(node_leaf3->size()                                              == 1);

//     /* Descend y sub tree */
//     const int y_root = bih.at(0).get_right_child(0, 0);
//     auto y_root_node = bih.at(block_index(y_root)).get_node(node_index(y_root));
//     BOOST_CHECK(bih.at(block_index(y_root)).get_split_axis(node_index(y_root))  == axis_t::not_set);
//     BOOST_CHECK(y_root_node->size()                                             == 1);
// }

// BOOST_AUTO_TEST_CASE( xyz_tree_build_test )
// {
//     primitives.emplace_back(nullptr, point_t( 0.0f,  0.0f,  0.0f), point_t( 0.0f,  0.0f,  1.0f), point_t( 0.0f,  1.0f,  1.0f));
//     primitives.emplace_back(nullptr, point_t(10.0f,  0.0f,  0.0f), point_t(10.0f,  0.0f,  1.0f), point_t(10.0f,  1.0f,  1.0f));
//     primitives.emplace_back(nullptr, point_t( 0.0f, 15.0f,  0.0f), point_t( 0.0f, 15.0f,  1.0f), point_t( 0.0f, 16.0f,  1.0f));
//     primitives.emplace_back(nullptr, point_t( 0.0f,  0.0f, 20.0f), point_t( 0.0f,  0.0f, 21.0f), point_t( 0.0f,  1.0f, 21.0f));
//     uut.build(&primitives, &bih);
    
//     /* Checks - Root internal node */
//     BOOST_CHECK(bih.size()                     == 9);
//     BOOST_REQUIRE(bih.at(0).get_split_axis(0)  == axis_t::z_axis);
//     BOOST_CHECK_CLOSE(bih.at(0).get_node(0)->get_left_split(),   1.0f, result_tolerance);
//     BOOST_CHECK_CLOSE(bih.at(0).get_node(0)->get_right_split(), 20.0f, result_tolerance);

//     /* Descend the not z tree */
//     const int not_z_tree = bih.at(0).get_left_child(0, 0);
//     auto not_z_tree_node = bih.at(block_index(not_z_tree)).get_node(node_index(not_z_tree));
//     BOOST_CHECK(bih.at(block_index(not_z_tree)).get_split_axis(node_index(not_z_tree)) == axis_t::y_axis);
//     BOOST_CHECK_CLOSE(not_z_tree_node->get_left_split(),   1.0f, result_tolerance);
//     BOOST_CHECK_CLOSE(not_z_tree_node->get_right_split(), 15.0f, result_tolerance);

//     /* Descend the not y tree */
//     const int not_y_tree = bih.at(block_index(not_z_tree)).get_left_child(block_index(not_z_tree), node_index(not_z_tree));
//     auto not_y_tree_node = bih.at(block_index(not_y_tree)).get_node(node_index(not_y_tree));
//     BOOST_CHECK(bih.at(block_index(not_y_tree)).get_split_axis(node_index(not_y_tree)) == axis_t::x_axis);
//     BOOST_CHECK_CLOSE(not_y_tree_node->get_left_split(),   0.0f + half_epsilon, result_tolerance);
//     BOOST_CHECK_CLOSE(not_y_tree_node->get_right_split(), 10.0f - half_epsilon, result_tolerance);

//     /* Leaf containing the origin */
//     const int origin_leaf = bih.at(block_index(not_y_tree)).get_left_child(block_index(not_y_tree), node_index(not_y_tree));
//     auto origin_leaf_node = bih.at(block_index(origin_leaf)).get_node(node_index(origin_leaf));
//     BOOST_CHECK(bih.at(block_index(origin_leaf)).get_split_axis(node_index(origin_leaf))    == axis_t::not_set);
//     BOOST_CHECK(origin_leaf_node->size()                                                    == 1);

//     /* Leaf containing the x extreme */
//     const int x_leaf = bih.at(block_index(not_y_tree)).get_right_child(block_index(not_y_tree), node_index(not_y_tree));
//     auto x_leaf_node = bih.at(block_index(x_leaf)).get_node(node_index(x_leaf));
//     BOOST_CHECK(bih.at(block_index(x_leaf)).get_split_axis(node_index(x_leaf))  == axis_t::not_set);
//     BOOST_CHECK(x_leaf_node->size()                                             == 1);

//     /* Out of not y tree*/
//     /* Descend the y tree, leaf containing the y extreme */
//     const int y_leaf = bih.at(block_index(not_z_tree)).get_right_child(block_index(not_z_tree), node_index(not_z_tree));
//     auto y_leaf_node = bih.at(block_index(y_leaf)).get_node(node_index(y_leaf));
//     BOOST_CHECK(bih.at(block_index(y_leaf)).get_split_axis(node_index(y_leaf))  == axis_t::not_set);
//     BOOST_CHECK(y_leaf_node->size()                                             == 1);

//     /* Out of y tree*/
//     /* Out of not z tree*/

//     /* Descend the z tree, leaf containing z extreme */
//     const int z_leaf = bih.at(0).get_right_child(0, 0);
//     const auto z_leaf_node = bih.at(block_index(z_leaf)).get_node(node_index(z_leaf));
//     BOOST_CHECK(bih.at(block_index(z_leaf)).get_split_axis(node_index(z_leaf))  == axis_t::not_set);
//     BOOST_CHECK(z_leaf_node->size()                                             == 1);
// }

// BOOST_AUTO_TEST_CASE( xz_diagonal_tree_build_test )
// {
//     primitives.emplace_back(nullptr, point_t(-10.0f, 0.0f, -20.0f), point_t(-10.0f, 0.0f, -19.0f), point_t(-20.0f, 1.0f, -19.0f));
//     primitives.emplace_back(nullptr, point_t( 10.0f, 0.0f,  20.0f), point_t( 10.0f, 0.0f,  21.0f), point_t( 20.0f, 1.0f,  21.0f));
//     uut.build(&primitives, &bih);
    
//     /* Checks - Root internal node */
//     BOOST_CHECK(bih.size()                      == 1);
//     BOOST_REQUIRE(bih.at(0).get_split_axis(0)   == axis_t::z_axis);
//     BOOST_CHECK_CLOSE(bih.at(0).get_node(0)->get_left_split(),  -19.0f, result_tolerance);
//     BOOST_CHECK_CLOSE(bih.at(0).get_node(0)->get_right_split(),  20.0f, result_tolerance);

//     /* Leaf containing the origin */
//     const int leaf0 = bih.at(0).get_left_child(0, 0);
//     const auto node_leaf0 = bih.at(block_index(leaf0)).get_node(node_index(leaf0));
//     BOOST_CHECK(bih.at(block_index(leaf0)).get_split_axis(node_index(leaf0))    == axis_t::not_set);
//     BOOST_CHECK(node_leaf0->size()                                              == 1);

//     const int leaf1 = bih.at(0).get_left_child(0, 0);
//     const auto node_leaf1 = bih.at(block_index(leaf1)).get_node(node_index(leaf1));
//     BOOST_CHECK(bih.at(block_index(leaf1)).get_split_axis(node_index(leaf1))    == axis_t::not_set);
//     BOOST_CHECK(node_leaf1->size()                                              == 1);
// }

// BOOST_AUTO_TEST_CASE( leaf_at_root_of_new_block_tree_build_test )
// {
//     primitives.emplace_back(nullptr, point_t(0.0f, 0.0f, 0.0f), point_t(0.0f, 9.0f, 0.0f), point_t(0.0f, 9.0f, 9.0f), true);
//     primitives.emplace_back(nullptr, point_t(0.0f, 0.0f, 0.0f), point_t(0.0f, 9.0f, 9.0f), point_t(0.0f, 0.0f, 9.0f), true));      
//     primitives.emplace_back(nullptr, point_t(1.0f, 0.0f, 0.0f), point_t(1.0f, 2.0f, 0.0f), point_t(1.0f, 2.0f, 2.0f), true);
//     primitives.emplace_back(nullptr, point_t(1.0f, 0.0f, 0.0f), point_t(1.0f, 2.0f, 2.0f), point_t(1.0f, 0.0f, 2.0f), true);
//     primitives.emplace_back(nullptr, point_t(3.0f, 0.0f, 0.0f), point_t(3.0f, 2.0f, 0.0f), point_t(3.0f, 2.0f, 2.0f), true);
//     primitives.emplace_back(nullptr, point_t(3.0f, 0.0f, 0.0f), point_t(3.0f, 2.0f, 2.0f), point_t(3.0f, 0.0f, 2.0f), true);
 
//     bih_builder uut;
//     uut.build(&primitives, &bih);

//     /* Checks - Root internal node */
//     BOOST_CHECK(bih.size()                      == 9);
//     BOOST_REQUIRE(bih.at(0).get_split_axis(0)   == axis_t::z_axis);
//     BOOST_CHECK_CLOSE(bih.at(0).get_node(0)->get_left_split(),  9.0f, result_tolerance);
//     BOOST_CHECK(bih.at(0).get_node(0)->get_right_split() == std::numeric_limits<float>::infinity());

//     /* Follow the left chain */
//     const int left0 = bih.at(0).get_left_child(0, 0);
//     const auto node_left0 = bih.at(block_index(left0)).get_node(node_index(left0));
//     BOOST_CHECK(bih.at(block_index(left0)).get_split_axis(node_index(left0)) == axis_t::y_axis);
//     BOOST_CHECK_CLOSE(node_left0->get_left_split(),  9.0f, result_tolerance);
//     BOOST_CHECK(node_left0->get_right_split() == std::numeric_limits<float>::infinity());

//     const int left1 = bih.at(block_index(left0)).get_left_child(block_index(left0), node_index(left0));
//     const auto node_left1 = bih.at(block_index(left1)).get_node(node_index(left1));
//     BOOST_CHECK(bih.at(block_index(left1)).get_split_axis(node_index(left1)) == axis_t::z_axis);
//     BOOST_CHECK_CLOSE(node_left1->get_left_split(),  2.0f, result_tolerance);
//     BOOST_CHECK_CLOSE(node_left1->get_right_split(), 0.0f, result_tolerance);

//     /* Enter child block at index 8 */

//     /* Check leaf in left child node*/
//     const int leaf0 = bih.at(block_index(left1)).get_left_child(block_index(left1), node_index(left1));
//     const auto node_leaf0 = bih.at(block_index(leaf0)).get_node(node_index(leaf0));
//     BOOST_CHECK(bih.at(block_index(leaf0)).get_split_axis(node_index(leaf0))    == axis_t::not_set);
//     BOOST_CHECK(node_leaf0->size()                                              == 4);

//     /* Exit child block at index 8 */
//     /* Enter child block at index 16 */

//     /* Check leaf in right index */
//     const int leaf1 = bih.at(block_index(left1)).get_right_child(block_index(left1), node_index(left1));
//     const auto node_leaf1 = bih.at(block_index(leaf1)).get_node(node_index(leaf1));
//     BOOST_CHECK(bih.at(block_index(leaf1)).get_split_axis(node_index(leaf1))    == axis_t::not_set);
//     BOOST_CHECK(node_leaf1->size()                                              == 2);

//     /* Exit child block at index 16 */

//     /* Recurse back checking right leaf nodes */
//     const int right1 = bih.at(block_index(left0)).get_right_child(block_index(left0), node_index(left0));
//     const auto node_leaf2 = bih.at(block_index(right1)).get_node(node_index(right1));
//     BOOST_CHECK(bih.at(block_index(right1)).get_split_axis(node_index(right1))  == axis_t::not_set);
//     BOOST_CHECK(node_leaf2->size()                                              == 0);

//     const int right0 = bih.at(0).get_right_child(0, 0);
//     const auto node_leaf3 = bih.at(block_index(right0)).get_node(node_index(right0));
//     BOOST_CHECK(bih.at(block_index(right0)).get_split_axis(node_index(right0))  == axis_t::not_set);
//     BOOST_CHECK(node_leaf3->size()                                              == 0);

// }

BOOST_AUTO_TEST_SUITE_END()
}; /* namespace test */
}; /* namespace raptor_raytracer */
