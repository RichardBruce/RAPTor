#ifdef STAND_ALONE
#define BOOST_TEST_MODULE primitive_store test

/* Common headers */
#include "logging.h"

/* Initialise logger */
const raptor_physics::init_logger init_logger;
#endif /* #ifdef STAND_ALONE */

/* Standard headers */

/* Boost headerrs */
#include "boost/test/unit_test.hpp"

/* Ray tracer headers */
#include "primitive_store.h"
#include "triangle.h"


namespace raptor_raytracer
{
namespace test
{
const float result_tolerance = 0.0001f;

struct primitive_store_fixture
{
    primitive_store_fixture() {  }

    void add_triangles()
    {
        uut.emplace_back(nullptr, point_t(1.0f, 0.0f, 0.0f), point_t(0.0f, 1.0f, 0.0f), point_t(0.0f, 0.0f, 1.0f));
        uut.emplace_back(nullptr, point_t(2.0f, 0.0f, 0.0f), point_t(0.0f, 2.0f, 0.0f), point_t(0.0f, 0.0f, 2.0f));
        uut.emplace_back(nullptr, point_t(3.0f, 0.0f, 0.0f), point_t(0.0f, 3.0f, 0.0f), point_t(0.0f, 0.0f, 3.0f));
        uut.emplace_back(nullptr, point_t(4.0f, 0.0f, 0.0f), point_t(0.0f, 4.0f, 0.0f), point_t(0.0f, 0.0f, 4.0f));
        uut.emplace_back(nullptr, point_t(5.0f, 0.0f, 0.0f), point_t(0.0f, 5.0f, 0.0f), point_t(0.0f, 0.0f, 5.0f));
        uut.emplace_back(nullptr, point_t(6.0f, 0.0f, 0.0f), point_t(0.0f, 6.0f, 0.0f), point_t(0.0f, 0.0f, 6.0f));
        uut.emplace_back(nullptr, point_t(7.0f, 0.0f, 0.0f), point_t(0.0f, 7.0f, 0.0f), point_t(0.0f, 0.0f, 7.0f));
        uut.emplace_back(nullptr, point_t(8.0f, 0.0f, 0.0f), point_t(0.0f, 8.0f, 0.0f), point_t(0.0f, 0.0f, 8.0f));
        uut.emplace_back(nullptr, point_t(9.0f, 0.0f, 0.0f), point_t(0.0f, 9.0f, 0.0f), point_t(0.0f, 0.0f, 9.0f));
    }

    primitive_store uut;
};


BOOST_FIXTURE_TEST_SUITE( primitive_store_tests, primitive_store_fixture );

BOOST_AUTO_TEST_CASE( ctor_test )
{
    BOOST_CHECK(uut.empty());
    BOOST_CHECK(uut.size() == 0);
}

BOOST_AUTO_TEST_CASE( reserve_test )
{
    uut.reserve(10);
    BOOST_CHECK(uut.empty());
    BOOST_CHECK(uut.size()      == 0);
    BOOST_CHECK(uut.capacity()  == 10);
}

BOOST_AUTO_TEST_CASE( emplace_back_test )
{
    BOOST_CHECK(uut.emplace_back(nullptr, point_t(1.0f, 0.0f, 0.0f), point_t(0.0f, 1.0f, 0.0f), point_t(0.0f, 0.0f, 1.0f)) == 0);
    BOOST_CHECK(uut.emplace_back(nullptr, point_t(2.0f, 0.0f, 0.0f), point_t(0.0f, 2.0f, 0.0f), point_t(0.0f, 0.0f, 2.0f)) == 1);
    BOOST_CHECK(uut.emplace_back(nullptr, point_t(3.0f, 0.0f, 0.0f), point_t(0.0f, 3.0f, 0.0f), point_t(0.0f, 0.0f, 3.0f)) == 2);
    BOOST_CHECK(!uut.empty());
    BOOST_CHECK(uut.size()      == 3);
    BOOST_CHECK(uut.capacity()  >= 3);

    BOOST_CHECK(uut.primitive(0)->get_vertex_a() == point_t(1.0f, 0.0f, 0.0f));
    BOOST_CHECK(uut.primitive(1)->get_vertex_a() == point_t(2.0f, 0.0f, 0.0f));
    BOOST_CHECK(uut.primitive(2)->get_vertex_a() == point_t(3.0f, 0.0f, 0.0f));

    BOOST_CHECK(uut.primitive(0)->get_vertex_b() == point_t(0.0f, 1.0f, 0.0f));
    BOOST_CHECK(uut.primitive(1)->get_vertex_b() == point_t(0.0f, 2.0f, 0.0f));
    BOOST_CHECK(uut.primitive(2)->get_vertex_b() == point_t(0.0f, 3.0f, 0.0f));

    BOOST_CHECK(uut.primitive(0)->get_vertex_c() == point_t(0.0f, 0.0f, 1.0f));
    BOOST_CHECK(uut.primitive(1)->get_vertex_c() == point_t(0.0f, 0.0f, 2.0f));
    BOOST_CHECK(uut.primitive(2)->get_vertex_c() == point_t(0.0f, 0.0f, 3.0f));

    const auto *const uut_ptr = &uut;
    BOOST_CHECK(uut_ptr->primitive(0)->get_vertex_a() == point_t(1.0f, 0.0f, 0.0f));
    BOOST_CHECK(uut_ptr->primitive(1)->get_vertex_a() == point_t(2.0f, 0.0f, 0.0f));
    BOOST_CHECK(uut_ptr->primitive(2)->get_vertex_a() == point_t(3.0f, 0.0f, 0.0f));

    BOOST_CHECK(uut_ptr->primitive(0)->get_vertex_b() == point_t(0.0f, 1.0f, 0.0f));
    BOOST_CHECK(uut_ptr->primitive(1)->get_vertex_b() == point_t(0.0f, 2.0f, 0.0f));
    BOOST_CHECK(uut_ptr->primitive(2)->get_vertex_b() == point_t(0.0f, 3.0f, 0.0f));

    BOOST_CHECK(uut_ptr->primitive(0)->get_vertex_c() == point_t(0.0f, 0.0f, 1.0f));
    BOOST_CHECK(uut_ptr->primitive(1)->get_vertex_c() == point_t(0.0f, 0.0f, 2.0f));
    BOOST_CHECK(uut_ptr->primitive(2)->get_vertex_c() == point_t(0.0f, 0.0f, 3.0f));
}

BOOST_AUTO_TEST_CASE( indirection_test )
{
    add_triangles();
    uut.indirection(0) = uut.indirection(1);
    std::swap(uut.indirection(2), uut.indirection(3));
    uut.indirection(5) = uut.indirection(4);
    
    BOOST_CHECK(uut.size() == 9);
    BOOST_CHECK(uut.indirection(0) == 1);
    BOOST_CHECK(uut.indirection(1) == 1);
    BOOST_CHECK(uut.indirection(2) == 3);
    BOOST_CHECK(uut.indirection(3) == 2);
    BOOST_CHECK(uut.indirection(4) == 4);
    BOOST_CHECK(uut.indirection(5) == 4);
    BOOST_CHECK(uut.indirection(6) == 6);
    BOOST_CHECK(uut.indirection(7) == 7);
    BOOST_CHECK(uut.indirection(8) == 8);

    BOOST_CHECK(uut.indirect_primitive(0)->get_vertex_a() == point_t(2.0f, 0.0f, 0.0f));
    BOOST_CHECK(uut.indirect_primitive(1)->get_vertex_a() == point_t(2.0f, 0.0f, 0.0f));
    BOOST_CHECK(uut.indirect_primitive(2)->get_vertex_a() == point_t(4.0f, 0.0f, 0.0f));
    BOOST_CHECK(uut.indirect_primitive(3)->get_vertex_a() == point_t(3.0f, 0.0f, 0.0f));
    BOOST_CHECK(uut.indirect_primitive(4)->get_vertex_a() == point_t(5.0f, 0.0f, 0.0f));
    BOOST_CHECK(uut.indirect_primitive(5)->get_vertex_a() == point_t(5.0f, 0.0f, 0.0f));
    BOOST_CHECK(uut.indirect_primitive(6)->get_vertex_a() == point_t(7.0f, 0.0f, 0.0f));
    BOOST_CHECK(uut.indirect_primitive(7)->get_vertex_a() == point_t(8.0f, 0.0f, 0.0f));
    BOOST_CHECK(uut.indirect_primitive(8)->get_vertex_a() == point_t(9.0f, 0.0f, 0.0f));

    BOOST_CHECK(uut.indirect_primitive(0)->get_vertex_b() == point_t(0.0f, 2.0f, 0.0f));
    BOOST_CHECK(uut.indirect_primitive(1)->get_vertex_b() == point_t(0.0f, 2.0f, 0.0f));
    BOOST_CHECK(uut.indirect_primitive(2)->get_vertex_b() == point_t(0.0f, 4.0f, 0.0f));
    BOOST_CHECK(uut.indirect_primitive(3)->get_vertex_b() == point_t(0.0f, 3.0f, 0.0f));
    BOOST_CHECK(uut.indirect_primitive(4)->get_vertex_b() == point_t(0.0f, 5.0f, 0.0f));
    BOOST_CHECK(uut.indirect_primitive(5)->get_vertex_b() == point_t(0.0f, 5.0f, 0.0f));
    BOOST_CHECK(uut.indirect_primitive(6)->get_vertex_b() == point_t(0.0f, 7.0f, 0.0f));
    BOOST_CHECK(uut.indirect_primitive(7)->get_vertex_b() == point_t(0.0f, 8.0f, 0.0f));
    BOOST_CHECK(uut.indirect_primitive(8)->get_vertex_b() == point_t(0.0f, 9.0f, 0.0f));

    BOOST_CHECK(uut.indirect_primitive(0)->get_vertex_c() == point_t(0.0f, 0.0f, 2.0f));
    BOOST_CHECK(uut.indirect_primitive(1)->get_vertex_c() == point_t(0.0f, 0.0f, 2.0f));
    BOOST_CHECK(uut.indirect_primitive(2)->get_vertex_c() == point_t(0.0f, 0.0f, 4.0f));
    BOOST_CHECK(uut.indirect_primitive(3)->get_vertex_c() == point_t(0.0f, 0.0f, 3.0f));
    BOOST_CHECK(uut.indirect_primitive(4)->get_vertex_c() == point_t(0.0f, 0.0f, 5.0f));
    BOOST_CHECK(uut.indirect_primitive(5)->get_vertex_c() == point_t(0.0f, 0.0f, 5.0f));
    BOOST_CHECK(uut.indirect_primitive(6)->get_vertex_c() == point_t(0.0f, 0.0f, 7.0f));
    BOOST_CHECK(uut.indirect_primitive(7)->get_vertex_c() == point_t(0.0f, 0.0f, 8.0f));
    BOOST_CHECK(uut.indirect_primitive(8)->get_vertex_c() == point_t(0.0f, 0.0f, 9.0f));

    const auto *const uut_ptr = &uut;
    BOOST_CHECK(uut_ptr->indirect_primitive(0)->get_vertex_a() == point_t(2.0f, 0.0f, 0.0f));
    BOOST_CHECK(uut_ptr->indirect_primitive(1)->get_vertex_a() == point_t(2.0f, 0.0f, 0.0f));
    BOOST_CHECK(uut_ptr->indirect_primitive(2)->get_vertex_a() == point_t(4.0f, 0.0f, 0.0f));
    BOOST_CHECK(uut_ptr->indirect_primitive(3)->get_vertex_a() == point_t(3.0f, 0.0f, 0.0f));
    BOOST_CHECK(uut_ptr->indirect_primitive(4)->get_vertex_a() == point_t(5.0f, 0.0f, 0.0f));
    BOOST_CHECK(uut_ptr->indirect_primitive(5)->get_vertex_a() == point_t(5.0f, 0.0f, 0.0f));
    BOOST_CHECK(uut_ptr->indirect_primitive(6)->get_vertex_a() == point_t(7.0f, 0.0f, 0.0f));
    BOOST_CHECK(uut_ptr->indirect_primitive(7)->get_vertex_a() == point_t(8.0f, 0.0f, 0.0f));
    BOOST_CHECK(uut_ptr->indirect_primitive(8)->get_vertex_a() == point_t(9.0f, 0.0f, 0.0f));

    BOOST_CHECK(uut_ptr->indirect_primitive(0)->get_vertex_b() == point_t(0.0f, 2.0f, 0.0f));
    BOOST_CHECK(uut_ptr->indirect_primitive(1)->get_vertex_b() == point_t(0.0f, 2.0f, 0.0f));
    BOOST_CHECK(uut_ptr->indirect_primitive(2)->get_vertex_b() == point_t(0.0f, 4.0f, 0.0f));
    BOOST_CHECK(uut_ptr->indirect_primitive(3)->get_vertex_b() == point_t(0.0f, 3.0f, 0.0f));
    BOOST_CHECK(uut_ptr->indirect_primitive(4)->get_vertex_b() == point_t(0.0f, 5.0f, 0.0f));
    BOOST_CHECK(uut_ptr->indirect_primitive(5)->get_vertex_b() == point_t(0.0f, 5.0f, 0.0f));
    BOOST_CHECK(uut_ptr->indirect_primitive(6)->get_vertex_b() == point_t(0.0f, 7.0f, 0.0f));
    BOOST_CHECK(uut_ptr->indirect_primitive(7)->get_vertex_b() == point_t(0.0f, 8.0f, 0.0f));
    BOOST_CHECK(uut_ptr->indirect_primitive(8)->get_vertex_b() == point_t(0.0f, 9.0f, 0.0f));

    BOOST_CHECK(uut_ptr->indirect_primitive(0)->get_vertex_c() == point_t(0.0f, 0.0f, 2.0f));
    BOOST_CHECK(uut_ptr->indirect_primitive(1)->get_vertex_c() == point_t(0.0f, 0.0f, 2.0f));
    BOOST_CHECK(uut_ptr->indirect_primitive(2)->get_vertex_c() == point_t(0.0f, 0.0f, 4.0f));
    BOOST_CHECK(uut_ptr->indirect_primitive(3)->get_vertex_c() == point_t(0.0f, 0.0f, 3.0f));
    BOOST_CHECK(uut_ptr->indirect_primitive(4)->get_vertex_c() == point_t(0.0f, 0.0f, 5.0f));
    BOOST_CHECK(uut_ptr->indirect_primitive(5)->get_vertex_c() == point_t(0.0f, 0.0f, 5.0f));
    BOOST_CHECK(uut_ptr->indirect_primitive(6)->get_vertex_c() == point_t(0.0f, 0.0f, 7.0f));
    BOOST_CHECK(uut_ptr->indirect_primitive(7)->get_vertex_c() == point_t(0.0f, 0.0f, 8.0f));
    BOOST_CHECK(uut_ptr->indirect_primitive(8)->get_vertex_c() == point_t(0.0f, 0.0f, 9.0f));
}

BOOST_AUTO_TEST_CASE( reset_indirection_test )
{
    add_triangles();
    uut.indirection(0) = uut.indirection(1);
    std::swap(uut.indirection(2), uut.indirection(3));
    uut.indirection(5) = uut.indirection(4);
    
    BOOST_CHECK(uut.size() == 9);
    BOOST_CHECK(uut.indirection(0) == 1);
    BOOST_CHECK(uut.indirection(1) == 1);
    BOOST_CHECK(uut.indirection(2) == 3);
    BOOST_CHECK(uut.indirection(3) == 2);
    BOOST_CHECK(uut.indirection(4) == 4);
    BOOST_CHECK(uut.indirection(5) == 4);
    BOOST_CHECK(uut.indirection(6) == 6);
    BOOST_CHECK(uut.indirection(7) == 7);
    BOOST_CHECK(uut.indirection(8) == 8);

    uut.reset_indirection();
    BOOST_CHECK(uut.indirection(0) == 0);
    BOOST_CHECK(uut.indirection(1) == 1);
    BOOST_CHECK(uut.indirection(2) == 2);
    BOOST_CHECK(uut.indirection(3) == 3);
    BOOST_CHECK(uut.indirection(4) == 4);
    BOOST_CHECK(uut.indirection(5) == 5);
    BOOST_CHECK(uut.indirection(6) == 6);
    BOOST_CHECK(uut.indirection(7) == 7);
    BOOST_CHECK(uut.indirection(8) == 8);
}

BOOST_AUTO_TEST_CASE( swap_indirection_test )
{
    add_triangles();
    uut.indirection(0) = uut.indirection(1);
    std::swap(uut.indirection(2), uut.indirection(3));
    uut.indirection(5) = uut.indirection(4);
    
    BOOST_CHECK(uut.size() == 9);
    BOOST_CHECK(uut.indirection(0) == 1);
    BOOST_CHECK(uut.indirection(1) == 1);
    BOOST_CHECK(uut.indirection(2) == 3);
    BOOST_CHECK(uut.indirection(3) == 2);
    BOOST_CHECK(uut.indirection(4) == 4);
    BOOST_CHECK(uut.indirection(5) == 4);
    BOOST_CHECK(uut.indirection(6) == 6);
    BOOST_CHECK(uut.indirection(7) == 7);
    BOOST_CHECK(uut.indirection(8) == 8);

    std::vector<int> indirect{8, 7, 6, 5, 4, 3, 2, 1, 0};
    uut.swap(indirect);
    
    BOOST_CHECK(uut.indirection(0) == 8);
    BOOST_CHECK(uut.indirection(1) == 7);
    BOOST_CHECK(uut.indirection(2) == 6);
    BOOST_CHECK(uut.indirection(3) == 5);
    BOOST_CHECK(uut.indirection(4) == 4);
    BOOST_CHECK(uut.indirection(5) == 3);
    BOOST_CHECK(uut.indirection(6) == 2);
    BOOST_CHECK(uut.indirection(7) == 1);
    BOOST_CHECK(uut.indirection(8) == 0);

    BOOST_CHECK(indirect[0] == 1);
    BOOST_CHECK(indirect[1] == 1);
    BOOST_CHECK(indirect[2] == 3);
    BOOST_CHECK(indirect[3] == 2);
    BOOST_CHECK(indirect[4] == 4);
    BOOST_CHECK(indirect[5] == 4);
    BOOST_CHECK(indirect[6] == 6);
    BOOST_CHECK(indirect[7] == 7);
    BOOST_CHECK(indirect[8] == 8);
}

BOOST_AUTO_TEST_CASE( move_to_indirect_test )
{
    add_triangles();
    uut.indirection(0) = uut.indirection(1);
    std::swap(uut.indirection(2), uut.indirection(3));
    uut.indirection(5) = uut.indirection(4);
    
    uut.move_to_indirect();
    BOOST_CHECK(uut.size() == 9);
    BOOST_CHECK(uut.primitive(0)->get_vertex_a() == point_t(2.0f, 0.0f, 0.0f));
    BOOST_CHECK(uut.primitive(1)->get_vertex_a() == point_t(2.0f, 0.0f, 0.0f));
    BOOST_CHECK(uut.primitive(2)->get_vertex_a() == point_t(4.0f, 0.0f, 0.0f));
    BOOST_CHECK(uut.primitive(3)->get_vertex_a() == point_t(3.0f, 0.0f, 0.0f));
    BOOST_CHECK(uut.primitive(4)->get_vertex_a() == point_t(5.0f, 0.0f, 0.0f));
    BOOST_CHECK(uut.primitive(5)->get_vertex_a() == point_t(5.0f, 0.0f, 0.0f));
    BOOST_CHECK(uut.primitive(6)->get_vertex_a() == point_t(7.0f, 0.0f, 0.0f));
    BOOST_CHECK(uut.primitive(7)->get_vertex_a() == point_t(8.0f, 0.0f, 0.0f));
    BOOST_CHECK(uut.primitive(8)->get_vertex_a() == point_t(9.0f, 0.0f, 0.0f));

    BOOST_CHECK(uut.primitive(0)->get_vertex_b() == point_t(0.0f, 2.0f, 0.0f));
    BOOST_CHECK(uut.primitive(1)->get_vertex_b() == point_t(0.0f, 2.0f, 0.0f));
    BOOST_CHECK(uut.primitive(2)->get_vertex_b() == point_t(0.0f, 4.0f, 0.0f));
    BOOST_CHECK(uut.primitive(3)->get_vertex_b() == point_t(0.0f, 3.0f, 0.0f));
    BOOST_CHECK(uut.primitive(4)->get_vertex_b() == point_t(0.0f, 5.0f, 0.0f));
    BOOST_CHECK(uut.primitive(5)->get_vertex_b() == point_t(0.0f, 5.0f, 0.0f));
    BOOST_CHECK(uut.primitive(6)->get_vertex_b() == point_t(0.0f, 7.0f, 0.0f));
    BOOST_CHECK(uut.primitive(7)->get_vertex_b() == point_t(0.0f, 8.0f, 0.0f));
    BOOST_CHECK(uut.primitive(8)->get_vertex_b() == point_t(0.0f, 9.0f, 0.0f));

    BOOST_CHECK(uut.primitive(0)->get_vertex_c() == point_t(0.0f, 0.0f, 2.0f));
    BOOST_CHECK(uut.primitive(1)->get_vertex_c() == point_t(0.0f, 0.0f, 2.0f));
    BOOST_CHECK(uut.primitive(2)->get_vertex_c() == point_t(0.0f, 0.0f, 4.0f));
    BOOST_CHECK(uut.primitive(3)->get_vertex_c() == point_t(0.0f, 0.0f, 3.0f));
    BOOST_CHECK(uut.primitive(4)->get_vertex_c() == point_t(0.0f, 0.0f, 5.0f));
    BOOST_CHECK(uut.primitive(5)->get_vertex_c() == point_t(0.0f, 0.0f, 5.0f));
    BOOST_CHECK(uut.primitive(6)->get_vertex_c() == point_t(0.0f, 0.0f, 7.0f));
    BOOST_CHECK(uut.primitive(7)->get_vertex_c() == point_t(0.0f, 0.0f, 8.0f));
    BOOST_CHECK(uut.primitive(8)->get_vertex_c() == point_t(0.0f, 0.0f, 9.0f));

    BOOST_CHECK(uut.indirect_primitive(0)->get_vertex_a() == point_t(2.0f, 0.0f, 0.0f));
    BOOST_CHECK(uut.indirect_primitive(1)->get_vertex_a() == point_t(2.0f, 0.0f, 0.0f));
    BOOST_CHECK(uut.indirect_primitive(2)->get_vertex_a() == point_t(4.0f, 0.0f, 0.0f));
    BOOST_CHECK(uut.indirect_primitive(3)->get_vertex_a() == point_t(3.0f, 0.0f, 0.0f));
    BOOST_CHECK(uut.indirect_primitive(4)->get_vertex_a() == point_t(5.0f, 0.0f, 0.0f));
    BOOST_CHECK(uut.indirect_primitive(5)->get_vertex_a() == point_t(5.0f, 0.0f, 0.0f));
    BOOST_CHECK(uut.indirect_primitive(6)->get_vertex_a() == point_t(7.0f, 0.0f, 0.0f));
    BOOST_CHECK(uut.indirect_primitive(7)->get_vertex_a() == point_t(8.0f, 0.0f, 0.0f));
    BOOST_CHECK(uut.indirect_primitive(8)->get_vertex_a() == point_t(9.0f, 0.0f, 0.0f));

    BOOST_CHECK(uut.indirect_primitive(0)->get_vertex_b() == point_t(0.0f, 2.0f, 0.0f));
    BOOST_CHECK(uut.indirect_primitive(1)->get_vertex_b() == point_t(0.0f, 2.0f, 0.0f));
    BOOST_CHECK(uut.indirect_primitive(2)->get_vertex_b() == point_t(0.0f, 4.0f, 0.0f));
    BOOST_CHECK(uut.indirect_primitive(3)->get_vertex_b() == point_t(0.0f, 3.0f, 0.0f));
    BOOST_CHECK(uut.indirect_primitive(4)->get_vertex_b() == point_t(0.0f, 5.0f, 0.0f));
    BOOST_CHECK(uut.indirect_primitive(5)->get_vertex_b() == point_t(0.0f, 5.0f, 0.0f));
    BOOST_CHECK(uut.indirect_primitive(6)->get_vertex_b() == point_t(0.0f, 7.0f, 0.0f));
    BOOST_CHECK(uut.indirect_primitive(7)->get_vertex_b() == point_t(0.0f, 8.0f, 0.0f));
    BOOST_CHECK(uut.indirect_primitive(8)->get_vertex_b() == point_t(0.0f, 9.0f, 0.0f));

    BOOST_CHECK(uut.indirect_primitive(0)->get_vertex_c() == point_t(0.0f, 0.0f, 2.0f));
    BOOST_CHECK(uut.indirect_primitive(1)->get_vertex_c() == point_t(0.0f, 0.0f, 2.0f));
    BOOST_CHECK(uut.indirect_primitive(2)->get_vertex_c() == point_t(0.0f, 0.0f, 4.0f));
    BOOST_CHECK(uut.indirect_primitive(3)->get_vertex_c() == point_t(0.0f, 0.0f, 3.0f));
    BOOST_CHECK(uut.indirect_primitive(4)->get_vertex_c() == point_t(0.0f, 0.0f, 5.0f));
    BOOST_CHECK(uut.indirect_primitive(5)->get_vertex_c() == point_t(0.0f, 0.0f, 5.0f));
    BOOST_CHECK(uut.indirect_primitive(6)->get_vertex_c() == point_t(0.0f, 0.0f, 7.0f));
    BOOST_CHECK(uut.indirect_primitive(7)->get_vertex_c() == point_t(0.0f, 0.0f, 8.0f));
    BOOST_CHECK(uut.indirect_primitive(8)->get_vertex_c() == point_t(0.0f, 0.0f, 9.0f));
}

BOOST_AUTO_TEST_CASE( iterate_test )
{
    add_triangles();
    uut.indirection(0) = uut.indirection(1);
    std::swap(uut.indirection(2), uut.indirection(3));
    uut.indirection(5) = uut.indirection(4);
    
    auto iter = uut.begin();
    BOOST_CHECK(iter->get_vertex_a() == point_t(1.0f, 0.0f, 0.0f));
    BOOST_CHECK(iter->get_vertex_b() == point_t(0.0f, 1.0f, 0.0f));
    BOOST_CHECK(iter->get_vertex_c() == point_t(0.0f, 0.0f, 1.0f));
    
    ++iter;
    BOOST_CHECK(iter->get_vertex_a() == point_t(2.0f, 0.0f, 0.0f));
    BOOST_CHECK(iter->get_vertex_b() == point_t(0.0f, 2.0f, 0.0f));
    BOOST_CHECK(iter->get_vertex_c() == point_t(0.0f, 0.0f, 2.0f));
    
    ++iter;
    BOOST_CHECK(iter->get_vertex_a() == point_t(3.0f, 0.0f, 0.0f));
    BOOST_CHECK(iter->get_vertex_b() == point_t(0.0f, 3.0f, 0.0f));
    BOOST_CHECK(iter->get_vertex_c() == point_t(0.0f, 0.0f, 3.0f));
    
    ++iter;
    BOOST_CHECK(iter->get_vertex_a() == point_t(4.0f, 0.0f, 0.0f));
    BOOST_CHECK(iter->get_vertex_b() == point_t(0.0f, 4.0f, 0.0f));
    BOOST_CHECK(iter->get_vertex_c() == point_t(0.0f, 0.0f, 4.0f));
    
    ++iter;
    BOOST_CHECK(iter->get_vertex_a() == point_t(5.0f, 0.0f, 0.0f));
    BOOST_CHECK(iter->get_vertex_b() == point_t(0.0f, 5.0f, 0.0f));
    BOOST_CHECK(iter->get_vertex_c() == point_t(0.0f, 0.0f, 5.0f));
    
    ++iter;
    BOOST_CHECK(iter->get_vertex_a() == point_t(6.0f, 0.0f, 0.0f));
    BOOST_CHECK(iter->get_vertex_b() == point_t(0.0f, 6.0f, 0.0f));
    BOOST_CHECK(iter->get_vertex_c() == point_t(0.0f, 0.0f, 6.0f));
    
    ++iter;
    BOOST_CHECK(iter->get_vertex_a() == point_t(7.0f, 0.0f, 0.0f));
    BOOST_CHECK(iter->get_vertex_b() == point_t(0.0f, 7.0f, 0.0f));
    BOOST_CHECK(iter->get_vertex_c() == point_t(0.0f, 0.0f, 7.0f));
    
    ++iter;
    BOOST_CHECK(iter->get_vertex_a() == point_t(8.0f, 0.0f, 0.0f));
    BOOST_CHECK(iter->get_vertex_b() == point_t(0.0f, 8.0f, 0.0f));
    BOOST_CHECK(iter->get_vertex_c() == point_t(0.0f, 0.0f, 8.0f));
    
    ++iter;
    BOOST_CHECK(iter->get_vertex_a() == point_t(9.0f, 0.0f, 0.0f));
    BOOST_CHECK(iter->get_vertex_b() == point_t(0.0f, 9.0f, 0.0f));
    BOOST_CHECK(iter->get_vertex_c() == point_t(0.0f, 0.0f, 9.0f));

    ++iter;
    BOOST_CHECK(iter == uut.end());
    
    const auto *const uut_ptr = &uut;
    auto const_iter = uut_ptr->begin();
    BOOST_CHECK(const_iter->get_vertex_a() == point_t(1.0f, 0.0f, 0.0f));
    BOOST_CHECK(const_iter->get_vertex_b() == point_t(0.0f, 1.0f, 0.0f));
    BOOST_CHECK(const_iter->get_vertex_c() == point_t(0.0f, 0.0f, 1.0f));
    
    ++const_iter;
    BOOST_CHECK(const_iter->get_vertex_a() == point_t(2.0f, 0.0f, 0.0f));
    BOOST_CHECK(const_iter->get_vertex_b() == point_t(0.0f, 2.0f, 0.0f));
    BOOST_CHECK(const_iter->get_vertex_c() == point_t(0.0f, 0.0f, 2.0f));
    
    ++const_iter;
    BOOST_CHECK(const_iter->get_vertex_a() == point_t(3.0f, 0.0f, 0.0f));
    BOOST_CHECK(const_iter->get_vertex_b() == point_t(0.0f, 3.0f, 0.0f));
    BOOST_CHECK(const_iter->get_vertex_c() == point_t(0.0f, 0.0f, 3.0f));
    
    ++const_iter;
    BOOST_CHECK(const_iter->get_vertex_a() == point_t(4.0f, 0.0f, 0.0f));
    BOOST_CHECK(const_iter->get_vertex_b() == point_t(0.0f, 4.0f, 0.0f));
    BOOST_CHECK(const_iter->get_vertex_c() == point_t(0.0f, 0.0f, 4.0f));
    
    ++const_iter;
    BOOST_CHECK(const_iter->get_vertex_a() == point_t(5.0f, 0.0f, 0.0f));
    BOOST_CHECK(const_iter->get_vertex_b() == point_t(0.0f, 5.0f, 0.0f));
    BOOST_CHECK(const_iter->get_vertex_c() == point_t(0.0f, 0.0f, 5.0f));
    
    ++const_iter;
    BOOST_CHECK(const_iter->get_vertex_a() == point_t(6.0f, 0.0f, 0.0f));
    BOOST_CHECK(const_iter->get_vertex_b() == point_t(0.0f, 6.0f, 0.0f));
    BOOST_CHECK(const_iter->get_vertex_c() == point_t(0.0f, 0.0f, 6.0f));
    
    ++const_iter;
    BOOST_CHECK(const_iter->get_vertex_a() == point_t(7.0f, 0.0f, 0.0f));
    BOOST_CHECK(const_iter->get_vertex_b() == point_t(0.0f, 7.0f, 0.0f));
    BOOST_CHECK(const_iter->get_vertex_c() == point_t(0.0f, 0.0f, 7.0f));
    
    ++const_iter;
    BOOST_CHECK(const_iter->get_vertex_a() == point_t(8.0f, 0.0f, 0.0f));
    BOOST_CHECK(const_iter->get_vertex_b() == point_t(0.0f, 8.0f, 0.0f));
    BOOST_CHECK(const_iter->get_vertex_c() == point_t(0.0f, 0.0f, 8.0f));
    
    ++const_iter;
    BOOST_CHECK(const_iter->get_vertex_a() == point_t(9.0f, 0.0f, 0.0f));
    BOOST_CHECK(const_iter->get_vertex_b() == point_t(0.0f, 9.0f, 0.0f));
    BOOST_CHECK(const_iter->get_vertex_c() == point_t(0.0f, 0.0f, 9.0f));

    ++const_iter;
    BOOST_CHECK(const_iter == uut_ptr->end());
}

BOOST_AUTO_TEST_SUITE_END()
}; /* namespace test */
}; /* namespace raptor_raytracer */
