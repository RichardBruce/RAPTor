#ifdef STAND_ALONE
#define BOOST_TEST_MODULE bih_block test

/* Common headers */
#include "logging.h"

/* Initialise logger */
const raptor_physics::init_logger init_logger;
#endif /* #ifdef STAND_ALONE */

/* Standard headers */

/* Boost headerrs */
#include "boost/test/unit_test.hpp"

/* Ray tracer headers */
#include "bih_block.h"


namespace raptor_raytracer
{
namespace test
{
const float result_tolerance = 0.00001f;

struct bih_block_fixture
{
    bih_block_fixture() :
    bih(5) { }

    std::vector<bih_block>  bih;
};

BOOST_FIXTURE_TEST_SUITE( bih_block_tests, bih_block_fixture );

/* Size test */
BOOST_AUTO_TEST_CASE( size_test )
{
    /* The whole point is to be the size of a cache line */
    BOOST_CHECK(sizeof(bih_block) == 64);
}

/* Static function tests */
BOOST_AUTO_TEST_CASE( requires_block_children_test )
{
    BOOST_CHECK(!bih.at(0).requires_block_children(0));
    BOOST_CHECK(!bih.at(0).requires_block_children(1));
    BOOST_CHECK(!bih.at(0).requires_block_children(2));
    BOOST_CHECK( bih.at(0).requires_block_children(3));
    BOOST_CHECK( bih.at(0).requires_block_children(4));
    BOOST_CHECK( bih.at(0).requires_block_children(5));
    BOOST_CHECK( bih.at(0).requires_block_children(6));
}

BOOST_AUTO_TEST_CASE( leaf_added_added_requires_block_children_test )
{
    /* Fill block 4 with generic nodes at the top and leaves at the bottom */
    bih.at(0).create_generic_node(0.0f, 0.0f, 0, axis_t::x_axis);
    bih.at(0).create_generic_node(1.0f, 0.1f, 1, axis_t::y_axis);
    bih.at(0).create_generic_node(2.0f, 0.2f, 2, axis_t::z_axis);
    bih.at(0).create_leaf_node(3, 10, 3);
    
    BOOST_CHECK(!bih.at(0).requires_block_children(0));
    BOOST_CHECK(!bih.at(0).requires_block_children(1));
    BOOST_CHECK(!bih.at(0).requires_block_children(2));
    BOOST_CHECK( bih.at(0).requires_block_children(3));
    BOOST_CHECK( bih.at(0).requires_block_children(4));
    BOOST_CHECK( bih.at(0).requires_block_children(5));
    BOOST_CHECK( bih.at(0).requires_block_children(6));
}

BOOST_AUTO_TEST_CASE( block_added_added_requires_block_children_test )
{
    /* Fill block 4 with generic nodes at the top and leaves at the bottom */
    bih.at(0).create_generic_node(0.0f, 0.0f, 0, axis_t::x_axis);
    bih.at(0).create_generic_node(1.0f, 0.1f, 1, axis_t::y_axis);
    bih.at(0).create_generic_node(2.0f, 0.2f, 2, axis_t::z_axis);
    bih.at(0).create_generic_node(2.0f, 0.2f, 3, axis_t::x_axis);
    
    BOOST_CHECK(!bih.at(0).requires_block_children(0));
    BOOST_CHECK(!bih.at(0).requires_block_children(1));
    BOOST_CHECK(!bih.at(0).requires_block_children(2));
    BOOST_CHECK(!bih.at(0).requires_block_children(3));
    BOOST_CHECK(!bih.at(0).requires_block_children(4));
    BOOST_CHECK(!bih.at(0).requires_block_children(5));
    BOOST_CHECK(!bih.at(0).requires_block_children(6));
}

BOOST_AUTO_TEST_CASE( child_blocks_required_test )
{
    BOOST_CHECK(bih_block::child_blocks_required() == 8);
}

/* Create node tests */
BOOST_AUTO_TEST_CASE( create_generic_node_test )
{
    /* Fill block 0 with generic nodes */
    bih.at(0).create_generic_node(0.0f, 0.0f, 0, axis_t::x_axis);
    bih.at(0).create_generic_node(1.0f, 0.1f, 1, axis_t::y_axis);
    bih.at(0).create_generic_node(2.0f, 0.2f, 2, axis_t::z_axis);
    bih.at(0).create_generic_node(3.0f, 0.3f, 3, axis_t::y_axis);
    bih.at(0).create_generic_node(4.0f, 0.4f, 4, axis_t::x_axis);
    bih.at(0).create_generic_node(5.0f, 0.5f, 5, axis_t::z_axis);
    bih.at(0).create_generic_node(6.0f, 0.6f, 6, axis_t::x_axis);
    
    /* Checks */
    auto node_0 = bih.at(0).get_node(0);
    BOOST_CHECK(bih.at(0).get_split_axis(0) == axis_t::x_axis);
    BOOST_CHECK(node_0 == &reinterpret_cast<const bih_node *>(&bih.at(0))[0]);
    BOOST_CHECK(node_0->get_left_split()        == 0.0f);
    BOOST_CHECK(node_0->get_right_split()       == 0.0f);

    auto node_1 = bih.at(0).get_node(1);
    BOOST_CHECK(bih.at(0).get_split_axis(1) == axis_t::y_axis);
    BOOST_CHECK(node_1 == &reinterpret_cast<const bih_node *>(&bih.at(0))[1]);
    BOOST_CHECK(node_1->get_left_split()        == 1.0f);
    BOOST_CHECK(node_1->get_right_split()       == 0.1f);

    auto node_2 = bih.at(0).get_node(2);
    BOOST_CHECK(bih.at(0).get_split_axis(2) == axis_t::z_axis);
    BOOST_CHECK(node_2 == &reinterpret_cast<const bih_node *>(&bih.at(0))[2]);
    BOOST_CHECK(node_2->get_left_split()        == 2.0f);
    BOOST_CHECK(node_2->get_right_split()       == 0.2f);

    auto node_3 = bih.at(0).get_node(3);
    BOOST_CHECK(bih.at(0).get_split_axis(3) == axis_t::y_axis);
    BOOST_CHECK(node_3 == &reinterpret_cast<const bih_node *>(&bih.at(0))[3]);
    BOOST_CHECK(node_3->get_left_split()        == 3.0f);
    BOOST_CHECK(node_3->get_right_split()       == 0.3f);

    auto node_4 = bih.at(0).get_node(4);
    BOOST_CHECK(bih.at(0).get_split_axis(4) == axis_t::x_axis);
    BOOST_CHECK(node_4 == &reinterpret_cast<const bih_node *>(&bih.at(0))[4]);
    BOOST_CHECK(node_4->get_left_split()        == 4.0f);
    BOOST_CHECK(node_4->get_right_split()       == 0.4f);

    auto node_5 = bih.at(0).get_node(5);
    BOOST_CHECK(bih.at(0).get_split_axis(5) == axis_t::z_axis);
    BOOST_CHECK(node_5 == &reinterpret_cast<const bih_node *>(&bih.at(0))[5]);
    BOOST_CHECK(node_5->get_left_split()        == 5.0f);
    BOOST_CHECK(node_5->get_right_split()       == 0.5f);

    auto node_6 = bih.at(0).get_node(6);
    BOOST_CHECK(bih.at(0).get_split_axis(6) == axis_t::x_axis);
    BOOST_CHECK(node_6 == &reinterpret_cast<const bih_node *>(&bih.at(0))[6]);
    BOOST_CHECK(node_6->get_left_split()        == 6.0f);
    BOOST_CHECK(node_6->get_right_split()       == 0.6f);
}

BOOST_AUTO_TEST_CASE( create_leaf_node_level_0_test )
{
    /* Fill block 4 with generic nodes at the top and leaves at the bottom */
    bih.at(3).create_leaf_node(3, 10, 0);
    
    /* Checks */
    auto node_0 = bih.at(3).get_node(0);
    BOOST_CHECK(bih.at(3).get_split_axis(0) == axis_t::not_set);
    BOOST_CHECK(node_0 == &reinterpret_cast<const bih_node *>(&bih.at(3))[0]);
    BOOST_CHECK(node_0->size() == 8);
}

BOOST_AUTO_TEST_CASE( create_leaf_node_level_1_test )
{
    /* Fill block 4 with generic nodes at the top and leaves at the bottom */
    bih.at(2).create_generic_node(0.0f, 0.0f, 0, axis_t::y_axis);
    bih.at(2).create_leaf_node(3, 10, 1);
    bih.at(2).create_leaf_node(4,  9, 2);
    
    /* Checks */
    auto node_0 = bih.at(2).get_node(0);
    BOOST_CHECK(bih.at(2).get_split_axis(0) == axis_t::y_axis);
    BOOST_CHECK(node_0 == &reinterpret_cast<const bih_node *>(&bih.at(2))[0]);
    BOOST_CHECK(node_0->get_left_split()        == 0.0f);
    BOOST_CHECK(node_0->get_right_split()       == 0.0f);

    auto node_1 = bih.at(2).get_node(1);
    BOOST_CHECK(bih.at(2).get_split_axis(1) == axis_t::not_set);
    BOOST_CHECK(node_1 == &reinterpret_cast<const bih_node *>(&bih.at(2))[1]);
    BOOST_CHECK(node_1->size() == 8);

    auto node_2 = bih.at(2).get_node(2);
    BOOST_CHECK(bih.at(2).get_split_axis(2) == axis_t::not_set);
    BOOST_CHECK(node_2 == &reinterpret_cast<const bih_node *>(&bih.at(2))[2]);
    BOOST_CHECK(node_2->size() == 6);
}

BOOST_AUTO_TEST_CASE( create_leaf_node_level_2_test )
{
    /* Fill block 4 with generic nodes at the top and leaves at the bottom */
    bih.at(4).create_generic_node(0.0f, 0.0f, 0, axis_t::x_axis);
    bih.at(4).create_generic_node(1.0f, 0.1f, 1, axis_t::y_axis);
    bih.at(4).create_generic_node(2.0f, 0.2f, 2, axis_t::z_axis);
    bih.at(4).create_leaf_node(3, 10, 3);
    bih.at(4).create_leaf_node(4,  9, 4);
    bih.at(4).create_leaf_node(5,  8, 5);
    bih.at(4).create_leaf_node(6,  7, 6);
    
    /* Checks */
    auto node_0 = bih.at(4).get_node(0);
    BOOST_CHECK(bih.at(4).get_split_axis(0) == axis_t::x_axis);
    BOOST_CHECK(node_0 == &reinterpret_cast<const bih_node *>(&bih.at(4))[0]);
    BOOST_CHECK(node_0->get_left_split()        == 0.0f);
    BOOST_CHECK(node_0->get_right_split()       == 0.0f);

    auto node_1 = bih.at(4).get_node(1);
    BOOST_CHECK(bih.at(4).get_split_axis(1) == axis_t::y_axis);
    BOOST_CHECK(node_1 == &reinterpret_cast<const bih_node *>(&bih.at(4))[1]);
    BOOST_CHECK(node_1->get_left_split()        == 1.0f);
    BOOST_CHECK(node_1->get_right_split()       == 0.1f);

    auto node_2 = bih.at(4).get_node(2);
    BOOST_CHECK(bih.at(4).get_split_axis(2) == axis_t::z_axis);
    BOOST_CHECK(node_2 == &reinterpret_cast<const bih_node *>(&bih.at(4))[2]);
    BOOST_CHECK(node_2->get_left_split()        == 2.0f);
    BOOST_CHECK(node_2->get_right_split()       == 0.2f);

    auto node_3 = bih.at(4).get_node(3);
    BOOST_CHECK(bih.at(4).get_split_axis(3) == axis_t::not_set);
    BOOST_CHECK(node_3 == &reinterpret_cast<const bih_node *>(&bih.at(4))[3]);
    BOOST_CHECK(node_3->size()  == 8);

    auto node_4 = bih.at(4).get_node(4);
    BOOST_CHECK(bih.at(4).get_split_axis(4) == axis_t::not_set);
    BOOST_CHECK(node_4 == &reinterpret_cast<const bih_node *>(&bih.at(4))[4]);
    BOOST_CHECK(node_4->size()  == 6);

    auto node_5 = bih.at(4).get_node(5);
    BOOST_CHECK(bih.at(4).get_split_axis(5) == axis_t::not_set);
    BOOST_CHECK(node_5 == &reinterpret_cast<const bih_node *>(&bih.at(4))[5]);
    BOOST_CHECK(node_5->size()  == 4);

    auto node_6 = bih.at(4).get_node(6);
    BOOST_CHECK(bih.at(4).get_split_axis(6) == axis_t::not_set);
    BOOST_CHECK(node_6 == &reinterpret_cast<const bih_node *>(&bih.at(4))[6]);
    BOOST_CHECK(node_6->size()  == 2);
}

/* Traversal tests */
BOOST_AUTO_TEST_CASE ( traversal_test )
{
    bih.resize(16);

    /* Create generic nodes at the lower level of this block */
    bih.at(0).create_generic_node(3.0f, 0.3f, 3, axis_t::z_axis);
    bih.at(0).create_generic_node(4.0f, 0.4f, 4, axis_t::x_axis);
    bih.at(0).create_generic_node(5.0f, 0.5f, 5, axis_t::x_axis);
    bih.at(0).create_generic_node(6.0f, 0.6f, 6, axis_t::z_axis);

    /* Create leaves in all the leaf blocks */
    bih.at(8).create_generic_node(5.0f, 0.5f, 1, axis_t::x_axis);
    bih.at(8).create_generic_node(6.0f, 0.6f, 2, axis_t::z_axis);
    bih.at(8).create_leaf_node(3, 110, 3);
    bih.at(8).create_leaf_node(4,  19, 4);
    bih.at(8).create_leaf_node(5,  18, 5);
    bih.at(8).create_leaf_node(6,  17, 6);

    bih.at(9).create_generic_node(5.0f, 0.5f, 1, axis_t::x_axis);
    bih.at(9).create_generic_node(6.0f, 0.6f, 2, axis_t::z_axis);
    bih.at(9).create_leaf_node(3, 210, 3);
    bih.at(9).create_leaf_node(4,  29, 4);
    bih.at(9).create_leaf_node(5,  28, 5);
    bih.at(9).create_leaf_node(6,  27, 6);
    
    bih.at(10).create_generic_node(5.0f, 0.5f, 1, axis_t::x_axis);
    bih.at(10).create_generic_node(6.0f, 0.6f, 2, axis_t::z_axis);
    bih.at(10).create_leaf_node(3, 310, 3);
    bih.at(10).create_leaf_node(4,  39, 4);
    bih.at(10).create_leaf_node(5,  38, 5);
    bih.at(10).create_leaf_node(6,  37, 6);
    
    bih.at(11).create_generic_node(5.0f, 0.5f, 1, axis_t::x_axis);
    bih.at(11).create_generic_node(6.0f, 0.6f, 2, axis_t::z_axis);
    bih.at(11).create_leaf_node(3, 410, 3);
    bih.at(11).create_leaf_node(4,  49, 4);
    bih.at(11).create_leaf_node(5,  48, 5);
    bih.at(11).create_leaf_node(6,  47, 6);
    
    bih.at(12).create_generic_node(5.0f, 0.5f, 1, axis_t::x_axis);
    bih.at(12).create_generic_node(6.0f, 0.6f, 2, axis_t::z_axis);
    bih.at(12).create_leaf_node(3, 510, 3);
    bih.at(12).create_leaf_node(4,  59, 4);
    bih.at(12).create_leaf_node(5,  58, 5);
    bih.at(12).create_leaf_node(6,  57, 6);
    
    bih.at(13).create_generic_node(5.0f, 0.5f, 1, axis_t::x_axis);
    bih.at(13).create_generic_node(6.0f, 0.6f, 2, axis_t::z_axis);
    bih.at(13).create_leaf_node(3, 610, 3);
    bih.at(13).create_leaf_node(4,  69, 4);
    bih.at(13).create_leaf_node(5,  68, 5);
    bih.at(13).create_leaf_node(6,  67, 6);
    
    bih.at(14).create_generic_node(5.0f, 0.5f, 1, axis_t::x_axis);
    bih.at(14).create_generic_node(6.0f, 0.6f, 2, axis_t::z_axis);
    bih.at(14).create_leaf_node(3, 710, 3);
    bih.at(14).create_leaf_node(4,  79, 4);
    bih.at(14).create_leaf_node(5,  78, 5);
    bih.at(14).create_leaf_node(6,  77, 6);
    
    bih.at(15).create_generic_node(5.0f, 0.5f, 1, axis_t::x_axis);
    bih.at(15).create_generic_node(6.0f, 0.6f, 2, axis_t::z_axis);
    bih.at(15).create_leaf_node(3, 10, 3);
    bih.at(15).create_leaf_node(4,  9, 4);
    bih.at(15).create_leaf_node(5,  8, 5);
    bih.at(15).create_leaf_node(6,  7, 6);

    /* Set the child block */
    bih.at(0).set_child_block(8 << 3);

    /* Find the leaves */
    /* Level 0 */
    BOOST_CHECK(bih.at(0).get_node(0) == &reinterpret_cast<const bih_node *>(&bih.at(0))[0]);

    /* Level 1 */
    const int l_0 = bih.at(0).get_left_child(0, 0);
    const int r_0 = bih.at(0).get_right_child(0, 0);
    BOOST_CHECK(bih.at(block_index(l_0)).get_node(node_index(l_0)) == &reinterpret_cast<const bih_node *>(&bih.at(0))[1]);
    BOOST_CHECK(bih.at(block_index(r_0)).get_node(node_index(r_0)) == &reinterpret_cast<const bih_node *>(&bih.at(0))[2]);

    /* Level 2 */
    const int l_1_0 = bih.at(block_index(l_0)).get_left_child( block_index(l_0), node_index(l_0));
    const int r_1_0 = bih.at(block_index(l_0)).get_right_child(block_index(l_0), node_index(l_0));
    const int l_1_1 = bih.at(block_index(r_0)).get_left_child( block_index(r_0), node_index(r_0));
    const int r_1_1 = bih.at(block_index(r_0)).get_right_child(block_index(r_0), node_index(r_0));
    BOOST_CHECK(bih.at(block_index(l_1_0)).get_node(node_index(l_1_0)) == &reinterpret_cast<const bih_node *>(&bih.at(0))[3]);
    BOOST_CHECK(bih.at(block_index(r_1_0)).get_node(node_index(r_1_0)) == &reinterpret_cast<const bih_node *>(&bih.at(0))[4]);
    BOOST_CHECK(bih.at(block_index(l_1_1)).get_node(node_index(l_1_1)) == &reinterpret_cast<const bih_node *>(&bih.at(0))[5]);
    BOOST_CHECK(bih.at(block_index(r_1_1)).get_node(node_index(r_1_1)) == &reinterpret_cast<const bih_node *>(&bih.at(0))[6]);

    /* Level 3, 1 block down */
    const int l_2_0 = bih.at(block_index(l_1_0)).get_left_child( block_index(l_1_0), node_index(l_1_0));
    const int r_2_0 = bih.at(block_index(l_1_0)).get_right_child(block_index(l_1_0), node_index(l_1_0));
    const int l_2_1 = bih.at(block_index(r_1_0)).get_left_child( block_index(r_1_0), node_index(r_1_0));
    const int r_2_1 = bih.at(block_index(r_1_0)).get_right_child(block_index(r_1_0), node_index(r_1_0));
    const int l_2_2 = bih.at(block_index(l_1_1)).get_left_child( block_index(l_1_1), node_index(l_1_1));
    const int r_2_2 = bih.at(block_index(l_1_1)).get_right_child(block_index(l_1_1), node_index(l_1_1));
    const int l_2_3 = bih.at(block_index(r_1_1)).get_left_child( block_index(r_1_1), node_index(r_1_1));
    const int r_2_3 = bih.at(block_index(r_1_1)).get_right_child(block_index(r_1_1), node_index(r_1_1));
    BOOST_CHECK(bih.at(block_index(l_2_0)).get_node(node_index(l_2_0)) == &reinterpret_cast<const bih_node *>(&bih.at(8))[0]);
    BOOST_CHECK(bih.at(block_index(r_2_0)).get_node(node_index(r_2_0)) == &reinterpret_cast<const bih_node *>(&bih.at(9))[0]);
    BOOST_CHECK(bih.at(block_index(l_2_1)).get_node(node_index(l_2_1)) == &reinterpret_cast<const bih_node *>(&bih.at(10))[0]);
    BOOST_CHECK(bih.at(block_index(r_2_1)).get_node(node_index(r_2_1)) == &reinterpret_cast<const bih_node *>(&bih.at(11))[0]);
    BOOST_CHECK(bih.at(block_index(l_2_2)).get_node(node_index(l_2_2)) == &reinterpret_cast<const bih_node *>(&bih.at(12))[0]);
    BOOST_CHECK(bih.at(block_index(r_2_2)).get_node(node_index(r_2_2)) == &reinterpret_cast<const bih_node *>(&bih.at(13))[0]);
    BOOST_CHECK(bih.at(block_index(l_2_3)).get_node(node_index(l_2_3)) == &reinterpret_cast<const bih_node *>(&bih.at(14))[0]);
    BOOST_CHECK(bih.at(block_index(r_2_3)).get_node(node_index(r_2_3)) == &reinterpret_cast<const bih_node *>(&bih.at(15))[0]);

    /* Level 4 */
    const int l_3_0 = bih.at(block_index(l_2_0)).get_left_child( block_index(l_2_0), node_index(l_2_0));
    const int r_3_0 = bih.at(block_index(l_2_0)).get_right_child(block_index(l_2_0), node_index(l_2_0));
    const int l_3_1 = bih.at(block_index(r_2_0)).get_left_child( block_index(r_2_0), node_index(r_2_0));
    const int r_3_1 = bih.at(block_index(r_2_0)).get_right_child(block_index(r_2_0), node_index(r_2_0));
    const int l_3_2 = bih.at(block_index(l_2_1)).get_left_child( block_index(l_2_1), node_index(l_2_1));
    const int r_3_2 = bih.at(block_index(l_2_1)).get_right_child(block_index(l_2_1), node_index(l_2_1));
    const int l_3_3 = bih.at(block_index(r_2_1)).get_left_child( block_index(r_2_1), node_index(r_2_1));
    const int r_3_3 = bih.at(block_index(r_2_1)).get_right_child(block_index(r_2_1), node_index(r_2_1));
    const int l_3_4 = bih.at(block_index(l_2_2)).get_left_child( block_index(l_2_2), node_index(l_2_2));
    const int r_3_4 = bih.at(block_index(l_2_2)).get_right_child(block_index(l_2_2), node_index(l_2_2));
    const int l_3_5 = bih.at(block_index(r_2_2)).get_left_child( block_index(r_2_2), node_index(r_2_2));
    const int r_3_5 = bih.at(block_index(r_2_2)).get_right_child(block_index(r_2_2), node_index(r_2_2));
    const int l_3_6 = bih.at(block_index(l_2_3)).get_left_child( block_index(l_2_3), node_index(l_2_3));
    const int r_3_6 = bih.at(block_index(l_2_3)).get_right_child(block_index(l_2_3), node_index(l_2_3));
    const int l_3_7 = bih.at(block_index(r_2_3)).get_left_child( block_index(r_2_3), node_index(r_2_3));
    const int r_3_7 = bih.at(block_index(r_2_3)).get_right_child(block_index(r_2_3), node_index(r_2_3));
    BOOST_CHECK(bih.at(block_index(l_3_0)).get_node(node_index(l_3_0)) == &reinterpret_cast<const bih_node *>(&bih.at(8))[1]);
    BOOST_CHECK(bih.at(block_index(r_3_0)).get_node(node_index(r_3_0)) == &reinterpret_cast<const bih_node *>(&bih.at(8))[2]);
    BOOST_CHECK(bih.at(block_index(l_3_1)).get_node(node_index(l_3_1)) == &reinterpret_cast<const bih_node *>(&bih.at(9))[1]);
    BOOST_CHECK(bih.at(block_index(r_3_1)).get_node(node_index(r_3_1)) == &reinterpret_cast<const bih_node *>(&bih.at(9))[2]);
    BOOST_CHECK(bih.at(block_index(l_3_2)).get_node(node_index(l_3_2)) == &reinterpret_cast<const bih_node *>(&bih.at(10))[1]);
    BOOST_CHECK(bih.at(block_index(r_3_2)).get_node(node_index(r_3_2)) == &reinterpret_cast<const bih_node *>(&bih.at(10))[2]);
    BOOST_CHECK(bih.at(block_index(l_3_3)).get_node(node_index(l_3_3)) == &reinterpret_cast<const bih_node *>(&bih.at(11))[1]);
    BOOST_CHECK(bih.at(block_index(r_3_3)).get_node(node_index(r_3_3)) == &reinterpret_cast<const bih_node *>(&bih.at(11))[2]);
    BOOST_CHECK(bih.at(block_index(l_3_4)).get_node(node_index(l_3_4)) == &reinterpret_cast<const bih_node *>(&bih.at(12))[1]);
    BOOST_CHECK(bih.at(block_index(r_3_4)).get_node(node_index(r_3_4)) == &reinterpret_cast<const bih_node *>(&bih.at(12))[2]);
    BOOST_CHECK(bih.at(block_index(l_3_5)).get_node(node_index(l_3_5)) == &reinterpret_cast<const bih_node *>(&bih.at(13))[1]);
    BOOST_CHECK(bih.at(block_index(r_3_5)).get_node(node_index(r_3_5)) == &reinterpret_cast<const bih_node *>(&bih.at(13))[2]);
    BOOST_CHECK(bih.at(block_index(l_3_6)).get_node(node_index(l_3_6)) == &reinterpret_cast<const bih_node *>(&bih.at(14))[1]);
    BOOST_CHECK(bih.at(block_index(r_3_6)).get_node(node_index(r_3_6)) == &reinterpret_cast<const bih_node *>(&bih.at(14))[2]);
    BOOST_CHECK(bih.at(block_index(l_3_7)).get_node(node_index(l_3_7)) == &reinterpret_cast<const bih_node *>(&bih.at(15))[1]);
    BOOST_CHECK(bih.at(block_index(r_3_7)).get_node(node_index(r_3_7)) == &reinterpret_cast<const bih_node *>(&bih.at(15))[2]);

    /* Level 5 leaves */
    const int l_4_0  = bih.at(block_index(l_3_0)).get_left_child( block_index(l_3_0), node_index(l_3_0));
    const int r_4_0  = bih.at(block_index(l_3_0)).get_right_child(block_index(l_3_0), node_index(l_3_0));
    const int l_4_1  = bih.at(block_index(r_3_0)).get_left_child( block_index(r_3_0), node_index(r_3_0));
    const int r_4_1  = bih.at(block_index(r_3_0)).get_right_child(block_index(r_3_0), node_index(r_3_0));
    const int l_4_2  = bih.at(block_index(l_3_1)).get_left_child( block_index(l_3_1), node_index(l_3_1));
    const int r_4_2  = bih.at(block_index(l_3_1)).get_right_child(block_index(l_3_1), node_index(l_3_1));
    const int l_4_3  = bih.at(block_index(r_3_1)).get_left_child( block_index(r_3_1), node_index(r_3_1));
    const int r_4_3  = bih.at(block_index(r_3_1)).get_right_child(block_index(r_3_1), node_index(r_3_1));
    const int l_4_4  = bih.at(block_index(l_3_2)).get_left_child( block_index(l_3_2), node_index(l_3_2));
    const int r_4_4  = bih.at(block_index(l_3_2)).get_right_child(block_index(l_3_2), node_index(l_3_2));
    const int l_4_5  = bih.at(block_index(r_3_2)).get_left_child( block_index(r_3_2), node_index(r_3_2));
    const int r_4_5  = bih.at(block_index(r_3_2)).get_right_child(block_index(r_3_2), node_index(r_3_2));
    const int l_4_6  = bih.at(block_index(l_3_3)).get_left_child( block_index(l_3_3), node_index(l_3_3));
    const int r_4_6  = bih.at(block_index(l_3_3)).get_right_child(block_index(l_3_3), node_index(l_3_3));
    const int l_4_7  = bih.at(block_index(r_3_3)).get_left_child( block_index(r_3_3), node_index(r_3_3));
    const int r_4_7  = bih.at(block_index(r_3_3)).get_right_child(block_index(r_3_3), node_index(r_3_3));
    const int l_4_8  = bih.at(block_index(l_3_4)).get_left_child( block_index(l_3_4), node_index(l_3_4));
    const int r_4_8  = bih.at(block_index(l_3_4)).get_right_child(block_index(l_3_4), node_index(l_3_4));
    const int l_4_9  = bih.at(block_index(r_3_4)).get_left_child( block_index(r_3_4), node_index(r_3_4));
    const int r_4_9  = bih.at(block_index(r_3_4)).get_right_child(block_index(r_3_4), node_index(r_3_4));
    const int l_4_10 = bih.at(block_index(l_3_5)).get_left_child( block_index(l_3_5), node_index(l_3_5));
    const int r_4_10 = bih.at(block_index(l_3_5)).get_right_child(block_index(l_3_5), node_index(l_3_5));
    const int l_4_11 = bih.at(block_index(r_3_5)).get_left_child( block_index(r_3_5), node_index(r_3_5));
    const int r_4_11 = bih.at(block_index(r_3_5)).get_right_child(block_index(r_3_5), node_index(r_3_5));
    const int l_4_12 = bih.at(block_index(l_3_6)).get_left_child( block_index(l_3_6), node_index(l_3_6));
    const int r_4_12 = bih.at(block_index(l_3_6)).get_right_child(block_index(l_3_6), node_index(l_3_6));
    const int l_4_13 = bih.at(block_index(r_3_6)).get_left_child( block_index(r_3_6), node_index(r_3_6));
    const int r_4_13 = bih.at(block_index(r_3_6)).get_right_child(block_index(r_3_6), node_index(r_3_6));
    const int l_4_14 = bih.at(block_index(l_3_7)).get_left_child( block_index(l_3_7), node_index(l_3_7));
    const int r_4_14 = bih.at(block_index(l_3_7)).get_right_child(block_index(l_3_7), node_index(l_3_7));
    const int l_4_15 = bih.at(block_index(r_3_7)).get_left_child( block_index(r_3_7), node_index(r_3_7));
    const int r_4_15 = bih.at(block_index(r_3_7)).get_right_child(block_index(r_3_7), node_index(r_3_7));
    BOOST_CHECK(bih.at(block_index(l_4_0 )).get_node(node_index(l_4_0 )) == &reinterpret_cast<const bih_node *>(&bih.at(8))[3]);
    BOOST_CHECK(bih.at(block_index(r_4_0 )).get_node(node_index(r_4_0 )) == &reinterpret_cast<const bih_node *>(&bih.at(8))[4]);
    BOOST_CHECK(bih.at(block_index(l_4_1 )).get_node(node_index(l_4_1 )) == &reinterpret_cast<const bih_node *>(&bih.at(8))[5]);
    BOOST_CHECK(bih.at(block_index(r_4_1 )).get_node(node_index(r_4_1 )) == &reinterpret_cast<const bih_node *>(&bih.at(8))[6]);
    BOOST_CHECK(bih.at(block_index(l_4_2 )).get_node(node_index(l_4_2 )) == &reinterpret_cast<const bih_node *>(&bih.at(9))[3]);
    BOOST_CHECK(bih.at(block_index(r_4_2 )).get_node(node_index(r_4_2 )) == &reinterpret_cast<const bih_node *>(&bih.at(9))[4]);
    BOOST_CHECK(bih.at(block_index(l_4_3 )).get_node(node_index(l_4_3 )) == &reinterpret_cast<const bih_node *>(&bih.at(9))[5]);
    BOOST_CHECK(bih.at(block_index(r_4_3 )).get_node(node_index(r_4_3 )) == &reinterpret_cast<const bih_node *>(&bih.at(9))[6]);
    BOOST_CHECK(bih.at(block_index(l_4_4 )).get_node(node_index(l_4_4 )) == &reinterpret_cast<const bih_node *>(&bih.at(10))[3]);
    BOOST_CHECK(bih.at(block_index(r_4_4 )).get_node(node_index(r_4_4 )) == &reinterpret_cast<const bih_node *>(&bih.at(10))[4]);
    BOOST_CHECK(bih.at(block_index(l_4_5 )).get_node(node_index(l_4_5 )) == &reinterpret_cast<const bih_node *>(&bih.at(10))[5]);
    BOOST_CHECK(bih.at(block_index(r_4_5 )).get_node(node_index(r_4_5 )) == &reinterpret_cast<const bih_node *>(&bih.at(10))[6]);
    BOOST_CHECK(bih.at(block_index(l_4_6 )).get_node(node_index(l_4_6 )) == &reinterpret_cast<const bih_node *>(&bih.at(11))[3]);
    BOOST_CHECK(bih.at(block_index(r_4_6 )).get_node(node_index(r_4_6 )) == &reinterpret_cast<const bih_node *>(&bih.at(11))[4]);
    BOOST_CHECK(bih.at(block_index(l_4_7 )).get_node(node_index(l_4_7 )) == &reinterpret_cast<const bih_node *>(&bih.at(11))[5]);
    BOOST_CHECK(bih.at(block_index(r_4_7 )).get_node(node_index(r_4_7 )) == &reinterpret_cast<const bih_node *>(&bih.at(11))[6]);
    BOOST_CHECK(bih.at(block_index(l_4_8 )).get_node(node_index(l_4_8 )) == &reinterpret_cast<const bih_node *>(&bih.at(12))[3]);
    BOOST_CHECK(bih.at(block_index(r_4_8 )).get_node(node_index(r_4_8 )) == &reinterpret_cast<const bih_node *>(&bih.at(12))[4]);
    BOOST_CHECK(bih.at(block_index(l_4_9 )).get_node(node_index(l_4_9 )) == &reinterpret_cast<const bih_node *>(&bih.at(12))[5]);
    BOOST_CHECK(bih.at(block_index(r_4_9 )).get_node(node_index(r_4_9 )) == &reinterpret_cast<const bih_node *>(&bih.at(12))[6]);
    BOOST_CHECK(bih.at(block_index(l_4_10)).get_node(node_index(l_4_10)) == &reinterpret_cast<const bih_node *>(&bih.at(13))[3]);
    BOOST_CHECK(bih.at(block_index(r_4_10)).get_node(node_index(r_4_10)) == &reinterpret_cast<const bih_node *>(&bih.at(13))[4]);
    BOOST_CHECK(bih.at(block_index(l_4_11)).get_node(node_index(l_4_11)) == &reinterpret_cast<const bih_node *>(&bih.at(13))[5]);
    BOOST_CHECK(bih.at(block_index(r_4_11)).get_node(node_index(r_4_11)) == &reinterpret_cast<const bih_node *>(&bih.at(13))[6]);
    BOOST_CHECK(bih.at(block_index(l_4_12)).get_node(node_index(l_4_12)) == &reinterpret_cast<const bih_node *>(&bih.at(14))[3]);
    BOOST_CHECK(bih.at(block_index(r_4_12)).get_node(node_index(r_4_12)) == &reinterpret_cast<const bih_node *>(&bih.at(14))[4]);
    BOOST_CHECK(bih.at(block_index(l_4_13)).get_node(node_index(l_4_13)) == &reinterpret_cast<const bih_node *>(&bih.at(14))[5]);
    BOOST_CHECK(bih.at(block_index(r_4_13)).get_node(node_index(r_4_13)) == &reinterpret_cast<const bih_node *>(&bih.at(14))[6]);
    BOOST_CHECK(bih.at(block_index(l_4_14)).get_node(node_index(l_4_14)) == &reinterpret_cast<const bih_node *>(&bih.at(15))[3]);
    BOOST_CHECK(bih.at(block_index(r_4_14)).get_node(node_index(r_4_14)) == &reinterpret_cast<const bih_node *>(&bih.at(15))[4]);
    BOOST_CHECK(bih.at(block_index(l_4_15)).get_node(node_index(l_4_15)) == &reinterpret_cast<const bih_node *>(&bih.at(15))[5]);
    BOOST_CHECK(bih.at(block_index(r_4_15)).get_node(node_index(r_4_15)) == &reinterpret_cast<const bih_node *>(&bih.at(15))[6]);
}

BOOST_AUTO_TEST_SUITE_END()
}; /* namespace test */
}; /* namespace raptor_raytracer */
