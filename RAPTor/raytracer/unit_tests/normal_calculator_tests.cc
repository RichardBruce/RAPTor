#ifdef STAND_ALONE
#define BOOST_TEST_MODULE normal_calculator test

/* Common headers */
#include "logging.h"

/* Initialise logger */
const raptor_physics::init_logger init_logger;
#endif /* #ifdef STAND_ALONE */

/* Standard headers */

/* Common headers */
#include "common.h"

#include "normal_calculator.h"

#include "boost/test/unit_test.hpp"


namespace raptor_raytracer
{
namespace test
{
const float result_tolerance = 0.00001;

BOOST_AUTO_TEST_SUITE( normal_calculator_tests );


/* Support class shared_normal tests */
struct shared_normal_fixture
{
    std::vector<shared_normal *>    users;
};

BOOST_FIXTURE_TEST_CASE( shared_normal_ctor_test, shared_normal_fixture )
{
    /* Test data */
    users.resize(2, nullptr);

    /* Construct */
    std::unique_ptr<shared_normal> uut(new shared_normal(&users[0], &users[1], point_t<>(1.0f, 0.0f, 0.0f), point_t<>(0.0f, 1.0f, 0.0f)));

    /* Checks */
    BOOST_CHECK(uut->normal() == point_t<>(0.5f, 0.5f, 0.0f));
    BOOST_CHECK(users[0] == uut.get());
    BOOST_CHECK(users[1] == uut.get());
}

BOOST_FIXTURE_TEST_CASE( shared_normal_add_test, shared_normal_fixture )
{
    /* Test data */
    users.resize(4, nullptr);
    std::unique_ptr<shared_normal> uut(new shared_normal(&users[0], &users[1], point_t<>(1.0f, 0.0f, 0.0f), point_t<>(0.0f, 1.0f, 0.0f)));

    /* Checks */
    BOOST_CHECK(uut->normal() == point_t<>(0.5f, 0.5f, 0.0f));
    BOOST_CHECK(users[0] == uut.get());
    BOOST_CHECK(users[1] == uut.get());

    /* Add a normal */
    uut->add(&users[2], point_t<>(0.0f, 0.0f, 1.0f));
    uut->add(&users[3], point_t<>(0.0f, 0.0f, 1.0f));

    /* Checks */
    BOOST_CHECK(uut->normal() == point_t<>(0.25f, 0.25f, 0.5f));
    BOOST_CHECK(users[0] == uut.get());
    BOOST_CHECK(users[1] == uut.get());
    BOOST_CHECK(users[2] == uut.get());
    BOOST_CHECK(users[3] == uut.get());
}

BOOST_FIXTURE_TEST_CASE( shared_normal_merge_test, shared_normal_fixture )
{
    /* Test data */
    users.resize(4, nullptr);

    /* Construct */
    std::unique_ptr<shared_normal> uut(new shared_normal(&users[0], &users[1], point_t<>(1.0f, 0.0f, 0.0f), point_t<>(0.0f, 1.0f, 0.0f)));

    /* Checks */
    BOOST_CHECK(uut->normal() == point_t<>(0.5f, 0.5f, 0.0f));
    BOOST_CHECK(users[0] == uut.get());
    BOOST_CHECK(users[1] == uut.get());

    /* Construct */
    std::unique_ptr<shared_normal> merge(new shared_normal(&users[2], &users[3], point_t<>(0.0f, 0.0f, 1.0f), point_t<>(0.0f, 0.0f, 1.0f)));

    /* Checks */
    BOOST_CHECK(merge->normal() == point_t<>(0.0f, 0.0f, 1.0f));
    BOOST_CHECK(users[2] == merge.get());
    BOOST_CHECK(users[3] == merge.get());

    /* Merge */
    uut->merge(merge.release());

    /* Checks */
    BOOST_CHECK(uut->normal() == point_t<>(0.25f, 0.25f, 0.5f));
    BOOST_CHECK(users[0] == uut.get());
    BOOST_CHECK(users[1] == uut.get());
    BOOST_CHECK(users[2] == uut.get());
    BOOST_CHECK(users[3] == uut.get());
}

BOOST_FIXTURE_TEST_CASE( shared_normal_add_and_merge_test, shared_normal_fixture )
{
    /* Test data */
    users.resize(10, nullptr);

    /* Construct */
    std::unique_ptr<shared_normal> uut(new shared_normal(&users[0], &users[1], point_t<>(1.0f, 0.0f, 0.0f), point_t<>(0.0f, 1.0f, 0.0f)));

    /* Checks */
    BOOST_CHECK(uut->normal() == point_t<>(0.5f, 0.5f, 0.0f));
    BOOST_CHECK(users[0] == uut.get());
    BOOST_CHECK(users[1] == uut.get());

    /* Add a normal */
    uut->add(&users[2], point_t<>(0.0f, 0.0f, 1.0f));
    uut->add(&users[3], point_t<>(0.0f, 0.0f, 1.0f));

    /* Checks */
    BOOST_CHECK(uut->normal() == point_t<>(0.25f, 0.25f, 0.5f));
    BOOST_CHECK(users[0] == uut.get());
    BOOST_CHECK(users[1] == uut.get());
    BOOST_CHECK(users[2] == uut.get());
    BOOST_CHECK(users[3] == uut.get());

    /* Construct */
    std::unique_ptr<shared_normal> merge(new shared_normal(&users[4], &users[5], point_t<>(0.0f, 0.0f, 1.0f), point_t<>(0.0f, 0.0f, 1.0f)));

    /* Checks */
    BOOST_CHECK(merge->normal() == point_t<>(0.0f, 0.0f, 1.0f));
    BOOST_CHECK(users[4] == merge.get());
    BOOST_CHECK(users[5] == merge.get());

    /* Add a normal */
    merge->add(&users[6], point_t<>(0.0f, 1.0f, 0.0f));
    merge->add(&users[7], point_t<>(1.0f, 0.0f, 0.0f));

    /* Checks */
    BOOST_CHECK(merge->normal() == point_t<>(0.25f, 0.25f, 0.5f));
    BOOST_CHECK(users[4] == merge.get());
    BOOST_CHECK(users[5] == merge.get());
    BOOST_CHECK(users[6] == merge.get());
    BOOST_CHECK(users[7] == merge.get());

    /* Merge */
    uut->merge(merge.release());

    /* Checks */
    BOOST_CHECK(uut->normal() == point_t<>(0.25f, 0.25f, 0.5f));
    BOOST_CHECK(users[0] == uut.get());
    BOOST_CHECK(users[1] == uut.get());
    BOOST_CHECK(users[2] == uut.get());
    BOOST_CHECK(users[3] == uut.get());
    BOOST_CHECK(users[4] == uut.get());
    BOOST_CHECK(users[5] == uut.get());
    BOOST_CHECK(users[6] == uut.get());
    BOOST_CHECK(users[7] == uut.get());

    /* Add again */
    uut->add(&users[8], point_t<>(0.0f, 1.0f, 0.0f));
    uut->add(&users[9], point_t<>(1.0f, 0.0f, 0.0f));

    /* Checks */
    BOOST_CHECK(uut->normal() == point_t<>(0.3f, 0.3f, 0.4f));
    BOOST_CHECK(users[0] == uut.get());
    BOOST_CHECK(users[1] == uut.get());
    BOOST_CHECK(users[2] == uut.get());
    BOOST_CHECK(users[3] == uut.get());
    BOOST_CHECK(users[4] == uut.get());
    BOOST_CHECK(users[5] == uut.get());
    BOOST_CHECK(users[6] == uut.get());
    BOOST_CHECK(users[7] == uut.get());
    BOOST_CHECK(users[8] == uut.get());
    BOOST_CHECK(users[9] == uut.get());
}

BOOST_FIXTURE_TEST_CASE( shared_normal_merge_and_merge_test, shared_normal_fixture )
{
    /* Test data */
    users.resize(8, nullptr);

    /* Construct */
    std::unique_ptr<shared_normal> uut(new shared_normal(&users[0], &users[1], point_t<>(1.0f, 0.0f, 0.0f), point_t<>(0.0f, 1.0f, 0.0f)));

    /* Checks */
    BOOST_CHECK(uut->normal() == point_t<>(0.5f, 0.5f, 0.0f));
    BOOST_CHECK(users[0] == uut.get());
    BOOST_CHECK(users[1] == uut.get());

    /* Construct */
    std::unique_ptr<shared_normal> merge_0(new shared_normal(&users[2], &users[3], point_t<>(0.0f, 0.0f, 1.0f), point_t<>(0.0f, 0.0f, 1.0f)));

    /* Checks */
    BOOST_CHECK(merge_0->normal() == point_t<>(0.0f, 0.0f, 1.0f));
    BOOST_CHECK(users[2] == merge_0.get());
    BOOST_CHECK(users[3] == merge_0.get());

    /* Construct */
    std::unique_ptr<shared_normal> merge_1(new shared_normal(&users[4], &users[5], point_t<>(0.0f, 0.0f, 1.0f), point_t<>(0.0f, 0.0f, 1.0f)));

    /* Checks */
    BOOST_CHECK(merge_1->normal() == point_t<>(0.0f, 0.0f, 1.0f));
    BOOST_CHECK(users[4] == merge_1.get());
    BOOST_CHECK(users[5] == merge_1.get());

    /* Add a normal */
    merge_1->add(&users[6], point_t<>(0.0f, 0.0f, 1.0f));
    merge_1->add(&users[7], point_t<>(0.0f, 0.0f, 1.0f));

    /* Checks */
    BOOST_CHECK(merge_1->normal() == point_t<>(0.0f, 0.0f, 1.0f));
    BOOST_CHECK(users[4] == merge_1.get());
    BOOST_CHECK(users[5] == merge_1.get());
    BOOST_CHECK(users[6] == merge_1.get());
    BOOST_CHECK(users[7] == merge_1.get());

    /* Merge */
    uut->merge(merge_0.release());

    /* Checks */
    BOOST_CHECK(uut->normal() == point_t<>(0.25f, 0.25f, 0.5f));
    BOOST_CHECK(users[0] == uut.get());
    BOOST_CHECK(users[1] == uut.get());
    BOOST_CHECK(users[2] == uut.get());
    BOOST_CHECK(users[3] == uut.get());

    /* Merge */
    uut->merge(merge_1.release());

    /* Checks */
    BOOST_CHECK(uut->normal() == point_t<>(0.125f, 0.125f, 0.75f));
    BOOST_CHECK(users[0] == uut.get());
    BOOST_CHECK(users[1] == uut.get());
    BOOST_CHECK(users[2] == uut.get());
    BOOST_CHECK(users[3] == uut.get());
    BOOST_CHECK(users[4] == uut.get());
    BOOST_CHECK(users[5] == uut.get());
    BOOST_CHECK(users[6] == uut.get());
    BOOST_CHECK(users[7] == uut.get());
}

/* normal_calculator Tests */
struct normal_calculator_fixture
{
    normal_calculator_fixture() :
        pnts_0(
        {
            point_t<>( 0.0f,  0.0f, 0.0f), point_t<>( 1.0f, 1.0f, 0.0f), point_t<>( 2.0f, 0.0f, 0.0f), point_t<>(1.0f, -1.0f, 0.0f), 
            point_t<>(-1.0f, -1.0f, 0.0f), point_t<>(-2.0f, 0.0f, 0.0f), point_t<>(-1.0f, 1.0f, 0.0f)
        }),
        pnts_1(
        {
            point_t<>( 0.0f,  0.0f, 1.0f), point_t<>( 1.0f, 1.0f, 0.0f), point_t<>( 2.0f, 0.0f, 0.0f), point_t<>(1.0f, -1.0f, 0.0f), 
            point_t<>(-1.0f, -1.0f, 0.0f), point_t<>(-2.0f, 0.0f, 0.0f), point_t<>(-1.0f, 1.0f, 0.0f)
        }),
        pnts_2(
        {
            point_t<>( 0.0f,  0.0f, 0.0f), point_t<>( 2.415f, 1.0f, 1.0f), point_t<>( 2.415f, -1.0f, 1.0f), point_t<>(4.83f, 0.0f, 0.0f)
        }),
        pnts_3(
        {
            point_t<>(0.0f,  0.0f,  0.0f), point_t<>(-1.0f, -1.0f, 0.0f), point_t<>(0.0f, -1.0f, 0.0f), point_t<>(0.5f, -0.5f, -0.5f), 
            point_t<>(0.5f, -0.25f, -1.0f), point_t<>(0.5f, -0.25f, -2.0f)
        }) { }

    std::vector<point_t<>> pnts_0;
    std::vector<point_t<>> pnts_1;
    std::vector<point_t<>> pnts_2;
    std::vector<point_t<>> pnts_3;
};

BOOST_FIXTURE_TEST_CASE( normal_calculator_add_test, normal_calculator_fixture )
{
    /* Construct */
    std::unique_ptr<normal_calculator> uut(new normal_calculator(pnts_0));
    BOOST_CHECK(uut->number_of_polygons() == 0);

    /* Add polygons and check */
    uut->add_point_usage(std::vector<int>{ 0, 1, 2 }, 0.5f, 0, 0);
    BOOST_CHECK(uut->number_of_polygons() == 1);

    uut->add_point_usage(std::vector<int>{ 0, 2, 3 }, 1.0f, 0, 1);
    BOOST_CHECK(uut->number_of_polygons() == 2);

    uut->add_point_usage(std::vector<int>{ 0, 3, 4 }, 2.0f, 0, 2);
    BOOST_CHECK(uut->number_of_polygons() == 3);

    uut->add_point_usage(std::vector<int>{ 0, 4, 5 }, 3.0f, 0, 3);
    BOOST_CHECK(uut->number_of_polygons() == 4);

    uut->add_point_usage(std::vector<int>{ 0, 5, 6 }, 4.0f, 0, 4);
    BOOST_CHECK(uut->number_of_polygons() == 5);

    uut->add_point_usage(std::vector<int>{ 0, 5 }, 5.0f, 0, 5); /* A line */
    BOOST_CHECK(uut->number_of_polygons() == 6);

    uut->add_point_usage(std::vector<int>{ 0 }, 6.0f, 0, 6); /* A point */
    BOOST_CHECK(uut->number_of_polygons() == 7);

    BOOST_CHECK(uut->number_of_points(0) == 3);
    BOOST_CHECK(uut->number_of_points(1) == 3);
    BOOST_CHECK(uut->number_of_points(2) == 3);
    BOOST_CHECK(uut->number_of_points(3) == 3);
    BOOST_CHECK(uut->number_of_points(4) == 3);
    BOOST_CHECK(uut->number_of_points(5) == 2);
    BOOST_CHECK(uut->number_of_points(6) == 1);

}

BOOST_FIXTURE_TEST_CASE( normal_calculator_global_point_test, normal_calculator_fixture )
{
    /* Construct */
    std::unique_ptr<normal_calculator> uut(new normal_calculator(pnts_0));

    /* Add polygons */
    uut->add_point_usage(std::vector<int>{ 0, 1, 2 }, 0.5f, 0, 0);
    uut->add_point_usage(std::vector<int>{ 0, 2, 3 }, 1.0f, 0, 1);
    uut->add_point_usage(std::vector<int>{ 0, 3, 4 }, 2.0f, 0, 2);
    uut->add_point_usage(std::vector<int>{ 0, 4, 5 }, 3.0f, 0, 3);
    uut->add_point_usage(std::vector<int>{ 0, 5, 6 }, 4.0f, 0, 4);

    BOOST_CHECK(uut->global_point(0, 0) == 0);
    BOOST_CHECK(uut->global_point(0, 1) == 1);
    BOOST_CHECK(uut->global_point(0, 2) == 2);

    BOOST_CHECK(uut->global_point(1, 0) == 0);
    BOOST_CHECK(uut->global_point(1, 1) == 2);
    BOOST_CHECK(uut->global_point(1, 2) == 3);

    BOOST_CHECK(uut->global_point(2, 0) == 0);
    BOOST_CHECK(uut->global_point(2, 1) == 3);
    BOOST_CHECK(uut->global_point(2, 2) == 4);

    BOOST_CHECK(uut->global_point(3, 0) == 0);
    BOOST_CHECK(uut->global_point(3, 1) == 4);
    BOOST_CHECK(uut->global_point(3, 2) == 5);

    BOOST_CHECK(uut->global_point(4, 0) == 0);
    BOOST_CHECK(uut->global_point(4, 1) == 5);
    BOOST_CHECK(uut->global_point(4, 2) == 6);
}

BOOST_FIXTURE_TEST_CASE( normal_calculator_groups_test, normal_calculator_fixture )
{
    /* Construct */
    std::unique_ptr<normal_calculator> uut(new normal_calculator(pnts_0));

    /* Add polygons */
    uut->add_point_usage(std::vector<int>{ 0, 1, 2 }, 0.5f, 0, 0);
    uut->add_point_usage(std::vector<int>{ 0, 2, 3 }, 1.0f, 4, 1);
    uut->add_point_usage(std::vector<int>{ 0, 3, 4 }, 2.0f, 2, 2);
    uut->add_point_usage(std::vector<int>{ 0, 4, 5 }, 3.0f, 3, 3);
    uut->add_point_usage(std::vector<int>{ 0, 5, 6 }, 4.0f, 1, 4);

    BOOST_CHECK(uut->group(0) == 0);
    BOOST_CHECK(uut->group(1) == 4);
    BOOST_CHECK(uut->group(2) == 2);
    BOOST_CHECK(uut->group(3) == 3);
    BOOST_CHECK(uut->group(4) == 1);
}

BOOST_FIXTURE_TEST_CASE( normal_calculator_polygons_on_point_test, normal_calculator_fixture )
{
    /* Construct */
    std::unique_ptr<normal_calculator> uut(new normal_calculator(pnts_0));

    /* Add polygons */
    uut->add_point_usage(std::vector<int>{ 0, 1, 2 }, 0.5f, 0, 0);
    uut->add_point_usage(std::vector<int>{ 0, 2, 3 }, 1.0f, 0, 1);
    uut->add_point_usage(std::vector<int>{ 0, 3, 4 }, 2.0f, 0, 2);
    uut->add_point_usage(std::vector<int>{ 0, 4, 5 }, 3.0f, 0, 3);
    uut->add_point_usage(std::vector<int>{ 0, 5, 6 }, 4.0f, 0, 4);

    BOOST_CHECK(uut->polygons_on_point(0).size() == 5);
    BOOST_CHECK(uut->polygons_on_point(1).size() == 1);
    BOOST_CHECK(uut->polygons_on_point(2).size() == 2);
    BOOST_CHECK(uut->polygons_on_point(3).size() == 2);
    BOOST_CHECK(uut->polygons_on_point(4).size() == 2);
    BOOST_CHECK(uut->polygons_on_point(5).size() == 2);
    BOOST_CHECK(uut->polygons_on_point(6).size() == 1);

    BOOST_CHECK(uut->polygons_on_point(0)[0] == 0);
    BOOST_CHECK(uut->polygons_on_point(0)[1] == 1);
    BOOST_CHECK(uut->polygons_on_point(0)[2] == 2);
    BOOST_CHECK(uut->polygons_on_point(0)[3] == 3);
    BOOST_CHECK(uut->polygons_on_point(0)[4] == 4);

    BOOST_CHECK(uut->polygons_on_point(1)[0] == 0);

    BOOST_CHECK(uut->polygons_on_point(2)[0] == 0);
    BOOST_CHECK(uut->polygons_on_point(2)[1] == 1);

    BOOST_CHECK(uut->polygons_on_point(3)[0] == 1);
    BOOST_CHECK(uut->polygons_on_point(3)[1] == 2);

    BOOST_CHECK(uut->polygons_on_point(4)[0] == 2);
    BOOST_CHECK(uut->polygons_on_point(4)[1] == 3);

    BOOST_CHECK(uut->polygons_on_point(5)[0] == 3);
    BOOST_CHECK(uut->polygons_on_point(5)[1] == 4);

    BOOST_CHECK(uut->polygons_on_point(6)[0] == 4);
}

BOOST_FIXTURE_TEST_CASE( normal_calculator_point_indexes_on_polygon_test, normal_calculator_fixture )
{
    /* Construct */
    std::unique_ptr<normal_calculator> uut(new normal_calculator(pnts_0));

    /* Add polygons */
    uut->add_point_usage(std::vector<int>{ 0, 1, 2 }, 0.5f, 0, 0);
    uut->add_point_usage(std::vector<int>{ 0, 2, 3 }, 1.0f, 0, 1);
    uut->add_point_usage(std::vector<int>{ 0, 3, 4 }, 2.0f, 0, 2);
    uut->add_point_usage(std::vector<int>{ 0, 4, 5 }, 3.0f, 0, 3);
    uut->add_point_usage(std::vector<int>{ 0, 5, 6 }, 4.0f, 0, 4);

    BOOST_CHECK(uut->points_on_polygon(0).size() == 3);
    BOOST_CHECK(uut->points_on_polygon(1).size() == 3);
    BOOST_CHECK(uut->points_on_polygon(2).size() == 3);
    BOOST_CHECK(uut->points_on_polygon(3).size() == 3);
    BOOST_CHECK(uut->points_on_polygon(4).size() == 3);

    BOOST_CHECK(uut->points_on_polygon(0)[0] == 0);
    BOOST_CHECK(uut->points_on_polygon(0)[1] == 1);
    BOOST_CHECK(uut->points_on_polygon(0)[2] == 2);

    BOOST_CHECK(uut->points_on_polygon(1)[0] == 0);
    BOOST_CHECK(uut->points_on_polygon(1)[1] == 2);
    BOOST_CHECK(uut->points_on_polygon(1)[2] == 3);

    BOOST_CHECK(uut->points_on_polygon(2)[0] == 0);
    BOOST_CHECK(uut->points_on_polygon(2)[1] == 3);
    BOOST_CHECK(uut->points_on_polygon(2)[2] == 4);

    BOOST_CHECK(uut->points_on_polygon(3)[0] == 0);
    BOOST_CHECK(uut->points_on_polygon(3)[1] == 4);
    BOOST_CHECK(uut->points_on_polygon(3)[2] == 5);

    BOOST_CHECK(uut->points_on_polygon(4)[0] == 0);
    BOOST_CHECK(uut->points_on_polygon(4)[1] == 5);
    BOOST_CHECK(uut->points_on_polygon(4)[2] == 6);
}

BOOST_FIXTURE_TEST_CASE( normal_calculator_points_on_polygon_test, normal_calculator_fixture )
{
    /* Construct */
    std::unique_ptr<normal_calculator> uut(new normal_calculator(pnts_0));

    /* Add polygons */
    uut->add_point_usage(std::vector<int>{ 0, 1, 2 }, 0.5f, 0, 0);
    uut->add_point_usage(std::vector<int>{ 0, 2, 3 }, 1.0f, 0, 1);
    uut->add_point_usage(std::vector<int>{ 0, 3, 4 }, 2.0f, 0, 2);
    uut->add_point_usage(std::vector<int>{ 0, 4, 5 }, 3.0f, 0, 3);
    uut->add_point_usage(std::vector<int>{ 0, 5, 6 }, 4.0f, 0, 4);

    std::vector<point_t<>> pnts;
    uut->points_on_polygon(&pnts, 0);
    BOOST_CHECK(pnts[0] == pnts_0[0]);
    BOOST_CHECK(pnts[1] == pnts_0[1]);
    BOOST_CHECK(pnts[2] == pnts_0[2]);

    pnts.clear();
    uut->points_on_polygon(&pnts, 1);
    BOOST_CHECK(pnts[0] == pnts_0[0]);
    BOOST_CHECK(pnts[1] == pnts_0[2]);
    BOOST_CHECK(pnts[2] == pnts_0[3]);

    pnts.clear();
    uut->points_on_polygon(&pnts, 2);
    BOOST_CHECK(pnts[0] == pnts_0[0]);
    BOOST_CHECK(pnts[1] == pnts_0[3]);
    BOOST_CHECK(pnts[2] == pnts_0[4]);

    pnts.clear();
    uut->points_on_polygon(&pnts, 3);
    BOOST_CHECK(pnts[0] == pnts_0[0]);
    BOOST_CHECK(pnts[1] == pnts_0[4]);
    BOOST_CHECK(pnts[2] == pnts_0[5]);

    pnts.clear();
    uut->points_on_polygon(&pnts, 4);
    BOOST_CHECK(pnts[0] == pnts_0[0]);
    BOOST_CHECK(pnts[1] == pnts_0[5]);
    BOOST_CHECK(pnts[2] == pnts_0[6]);
}

BOOST_FIXTURE_TEST_CASE( edge_merge_normal_calculator_calculate_test, normal_calculator_fixture )
{
    /* Construct */
    std::unique_ptr<normal_calculator> uut(new normal_calculator(pnts_2));

    /* Add polygons */
    uut->add_point_usage(std::vector<int>{ 0, 1, 2 }, PI * 0.25f, 0, 0);
    uut->add_point_usage(std::vector<int>{ 1, 3, 2 }, PI * 0.25f, 0, 1);

    /* Calculate */
    uut->calculate();

    /* Checks */
    std::vector<point_t<>> norms;
    BOOST_REQUIRE(uut->normals(&norms, 0) != nullptr);
    BOOST_CHECK(norms.size() == 3);
    BOOST_CHECK(fabs(magnitude(norms[0] - point_t<>(0.382577f, -0.0f, -0.923924f))) < result_tolerance);
    BOOST_CHECK(norms[1] == point_t<>(0.0f, 0.0f, -1.0f));
    BOOST_CHECK(norms[2] == point_t<>(0.0f, 0.0f, -1.0f));

    norms.clear();
    BOOST_REQUIRE(uut->normals(&norms, 1) != nullptr);
    BOOST_CHECK(norms.size() == 3);
    BOOST_CHECK(norms[0] == point_t<>(0.0f, 0.0f, -1.0f));
    BOOST_CHECK(fabs(magnitude(norms[1] - point_t<>(-0.382577f, 0.0f, -0.923924f))) < result_tolerance);
    BOOST_CHECK(norms[2] == point_t<>(0.0f, 0.0f, -1.0f));
}

BOOST_FIXTURE_TEST_CASE( flat_normal_calculator_calculate_test, normal_calculator_fixture )
{
    /* Construct */
    std::unique_ptr<normal_calculator> uut(new normal_calculator(pnts_0));

    /* Add polygons */
    uut->add_point_usage(std::vector<int>{ 0, 1, 2 }, PI * 0.25f, 0, 0);
    uut->add_point_usage(std::vector<int>{ 0, 2, 3 }, PI * 0.25f, 0, 1);
    uut->add_point_usage(std::vector<int>{ 0, 3, 4 }, PI * 0.25f, 0, 2);
    uut->add_point_usage(std::vector<int>{ 0, 4, 5 }, PI * 0.25f, 0, 3);
    uut->add_point_usage(std::vector<int>{ 0, 6, 5 }, PI * 0.25f, 0, 4);   /* Upside down */

    /* Calculate */
    uut->calculate();

    /* Checks */
    std::vector<point_t<>> norms;
    BOOST_REQUIRE(uut->normals(&norms, 0) != nullptr);
    BOOST_CHECK(norms.size() == 3);
    BOOST_CHECK(fabs(magnitude(norms[0] - point_t<>(0.0f, 0.0f, -1.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(norms[1] - point_t<>(0.0f, 0.0f, -1.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(norms[2] - point_t<>(0.0f, 0.0f, -1.0f))) < result_tolerance);

    norms.clear();
    BOOST_REQUIRE(uut->normals(&norms, 1) != nullptr);
    BOOST_CHECK(norms.size() == 3);
    BOOST_CHECK(fabs(magnitude(norms[0] - point_t<>(0.0f, 0.0f, -1.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(norms[1] - point_t<>(0.0f, 0.0f, -1.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(norms[2] - point_t<>(0.0f, 0.0f, -1.0f))) < result_tolerance);

    norms.clear();
    BOOST_REQUIRE(uut->normals(&norms, 2) != nullptr);
    BOOST_CHECK(norms.size() == 3);
    BOOST_CHECK(fabs(magnitude(norms[0] - point_t<>(0.0f, 0.0f, -1.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(norms[1] - point_t<>(0.0f, 0.0f, -1.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(norms[2] - point_t<>(0.0f, 0.0f, -1.0f))) < result_tolerance);

    norms.clear();
    BOOST_REQUIRE(uut->normals(&norms, 3) != nullptr);
    BOOST_CHECK(norms.size() == 3);
    BOOST_CHECK(fabs(magnitude(norms[0] - point_t<>(0.0f, 0.0f, -1.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(norms[1] - point_t<>(0.0f, 0.0f, -1.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(norms[2] - point_t<>(0.0f, 0.0f, -1.0f))) < result_tolerance);

    norms.clear();
    BOOST_REQUIRE(uut->normals(&norms, 4) == nullptr);
}

BOOST_FIXTURE_TEST_CASE( normal_calculator_calculate_test, normal_calculator_fixture )
{
    /* Construct */
    std::unique_ptr<normal_calculator> uut(new normal_calculator(pnts_1));

    /* Add polygons */
    uut->add_point_usage(std::vector<int>{ 0, 1, 2 }, PI * 0.25f, 0, 0);
    uut->add_point_usage(std::vector<int>{ 0, 2, 3 }, PI * 0.25f, 0, 1);
    uut->add_point_usage(std::vector<int>{ 0, 3, 4 }, PI * 0.25f, 0, 2);
    uut->add_point_usage(std::vector<int>{ 0, 4, 5 }, PI * 0.25f, 0, 3);
    uut->add_point_usage(std::vector<int>{ 0, 6, 5 }, PI * 0.25f, 0, 4);   /* Upside down */

    /* Calculate */
    uut->calculate();

    /* Checks */
    std::vector<point_t<>> norms;
    BOOST_REQUIRE(uut->normals(&norms, 0) == nullptr);

    norms.clear();
    BOOST_REQUIRE(uut->normals(&norms, 1) != nullptr);
    BOOST_CHECK(norms.size() == 3);
    BOOST_CHECK(fabs(magnitude(norms[0] - point_t<>(-0.0f,      0.545627f, -0.838028f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(norms[1] - point_t<>(-0.408248f, 0.408248f, -0.816497f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(norms[2] - point_t<>(-0.211325f, 0.57735f,  -0.788675f))) < result_tolerance);

    norms.clear();
    BOOST_REQUIRE(uut->normals(&norms, 2) != nullptr);
    BOOST_CHECK(norms.size() == 3);
    BOOST_CHECK(fabs(magnitude(norms[0] - point_t<>(-0.0f,      0.545627f, -0.838028f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(norms[1] - point_t<>(-0.211325f, 0.57735f,  -0.788675f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(norms[2] - point_t<>( 0.211325f, 0.57735f,  -0.788675f))) < result_tolerance);

    norms.clear();
    BOOST_REQUIRE(uut->normals(&norms, 3) != nullptr);
    BOOST_CHECK(norms.size() == 3);
    BOOST_CHECK(fabs(magnitude(norms[0] - point_t<>(-0.0f,      0.545627f, -0.838028f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(norms[1] - point_t<>( 0.211325f, 0.57735f,  -0.788675f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(norms[2] - point_t<>( 0.408248f, 0.408248f, -0.816497f))) < result_tolerance);

    norms.clear();
    BOOST_REQUIRE(uut->normals(&norms, 4) == nullptr);
}

BOOST_FIXTURE_TEST_CASE( normal_calculator_calculate_mixed_groups_test, normal_calculator_fixture )
{
    /* Construct */
    std::unique_ptr<normal_calculator> uut(new normal_calculator(pnts_0));

    /* Add polygons */
    uut->add_point_usage(std::vector<int>{ 0, 1, 2 }, PI * 0.25f, 1, 0);
    uut->add_point_usage(std::vector<int>{ 0, 2, 3 }, PI * 0.25f, 0, 1);
    uut->add_point_usage(std::vector<int>{ 0, 3, 4 }, PI * 0.25f, 0, 2);
    uut->add_point_usage(std::vector<int>{ 0, 4, 5 }, PI * 0.25f, 0, 3);
    uut->add_point_usage(std::vector<int>{ 0, 6, 5 }, PI * 0.25f, 2, 4);   /* Upside down */

    /* Calculate */
    uut->calculate();

    /* Checks */
    std::vector<point_t<>> norms;
    BOOST_REQUIRE(uut->normals(&norms, 0) == nullptr);

    norms.clear();
    BOOST_REQUIRE(uut->normals(&norms, 1) != nullptr);
    BOOST_CHECK(norms.size() == 3);

    BOOST_CHECK(fabs(magnitude(norms[0] - point_t<>(0.0f, 0.0f, -1.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(norms[1] - point_t<>(0.0f, 0.0f, -1.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(norms[2] - point_t<>(0.0f, 0.0f, -1.0f))) < result_tolerance);

    norms.clear();
    BOOST_REQUIRE(uut->normals(&norms, 2) != nullptr);
    BOOST_CHECK(norms.size() == 3);
    BOOST_CHECK(fabs(magnitude(norms[0] - point_t<>(0.0f, 0.0f, -1.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(norms[1] - point_t<>(0.0f, 0.0f, -1.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(norms[2] - point_t<>(0.0f, 0.0f, -1.0f))) < result_tolerance);

    norms.clear();
    BOOST_REQUIRE(uut->normals(&norms, 3) != nullptr);
    BOOST_CHECK(norms.size() == 3);
    BOOST_CHECK(fabs(magnitude(norms[0] - point_t<>(0.0f, 0.0f, -1.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(norms[1] - point_t<>(0.0f, 0.0f, -1.0f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(norms[2] - point_t<>(0.0f, 0.0f, -1.0f))) < result_tolerance);

    norms.clear();
    BOOST_REQUIRE(uut->normals(&norms, 4) == nullptr);
}

BOOST_FIXTURE_TEST_CASE( normal_calculator_calculate_with_reverse_add_test, normal_calculator_fixture )
{
    /* Construct */
    std::unique_ptr<normal_calculator> uut(new normal_calculator(pnts_3));

    /* Add polygons */
    uut->add_point_usage(std::vector<int>{ 0, 1, 2 }, PI * 0.3f, 0, 0);
    uut->add_point_usage(std::vector<int>{ 0, 3, 4 }, PI * 0.3f, 0, 1);
    uut->add_point_usage(std::vector<int>{ 0, 2, 3 }, PI * 0.3f, 0, 2);

    /* Calculate */
    uut->calculate();

    /* Checks */
    std::vector<point_t<>> norms;
    BOOST_REQUIRE(uut->normals(&norms, 0) != nullptr);
    BOOST_CHECK(norms.size() == 3);
    BOOST_CHECK(fabs(magnitude(norms[0] - point_t<>(0.593638f, 0.210295f, 0.776769f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(norms[1] - point_t<>(0.0f,      0.0f,      1.0f     ))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(norms[2] - point_t<>(0.382683f, 0.0f,      0.92388f ))) < result_tolerance);

    norms.clear();
    BOOST_REQUIRE(uut->normals(&norms, 1) != nullptr);
    BOOST_CHECK(norms.size() == 3);
    BOOST_CHECK(fabs(magnitude(norms[0] - point_t<>(0.593638f, 0.210295f, 0.776769f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(norms[1] - point_t<>(0.805173f, 0.285232f, 0.519942f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(norms[2] - point_t<>(0.801784f, 0.534522f, 0.267261f))) < result_tolerance);

    norms.clear();
    BOOST_REQUIRE(uut->normals(&norms, 2) != nullptr);
    BOOST_CHECK(norms.size() == 3);
    BOOST_CHECK(fabs(magnitude(norms[0] - point_t<>(0.593638f, 0.210295f, 0.776769f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(norms[1] - point_t<>(0.382683f, 0.0f,      0.92388f ))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(norms[2] - point_t<>(0.805173f, 0.285232f, 0.519942f))) < result_tolerance);
}

BOOST_FIXTURE_TEST_CASE( normal_calculator_calculate_with_merge_test, normal_calculator_fixture )
{
    /* Construct */
    std::unique_ptr<normal_calculator> uut(new normal_calculator(pnts_3));

    /* Add polygons */
    uut->add_point_usage(std::vector<int>{ 0, 1, 2 }, PI * 0.3f, 0, 0);
    uut->add_point_usage(std::vector<int>{ 0, 4, 5 }, PI * 0.3f, 0, 1);
    uut->add_point_usage(std::vector<int>{ 0, 2, 3 }, PI * 0.3f, 0, 2);
    uut->add_point_usage(std::vector<int>{ 0, 3, 4 }, PI * 0.3f, 0, 3);

    /* Calculate */
    uut->calculate();

    /* Checks */
    std::vector<point_t<>> norms;
    BOOST_REQUIRE(uut->normals(&norms, 0) != nullptr);
    BOOST_CHECK(norms.size() == 3);
    BOOST_CHECK(fabs(magnitude(norms[0] - point_t<>(0.625929f, 0.457246f, 0.631774f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(norms[1] - point_t<>(0.0f,      0.0f,      1.0f     ))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(norms[2] - point_t<>(0.382683f, 0.0f,      0.92388f ))) < result_tolerance);

    norms.clear();
    BOOST_REQUIRE(uut->normals(&norms, 1) != nullptr);
    BOOST_CHECK(norms.size() == 3);
    BOOST_CHECK(fabs(magnitude(norms[0] - point_t<>(0.625929f, 0.457246f, 0.631774f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(norms[1] - point_t<>(0.651677f, 0.745569f, 0.139446f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(norms[2] - point_t<>(0.447214f, 0.894427f, 0.0f     ))) < result_tolerance);

    norms.clear();
    BOOST_REQUIRE(uut->normals(&norms, 2) != nullptr);
    BOOST_CHECK(norms.size() == 3);
    BOOST_CHECK(fabs(magnitude(norms[0] - point_t<>(0.625929f, 0.457246f, 0.631774f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(norms[1] - point_t<>(0.382683f, 0.0f,      0.92388f ))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(norms[2] - point_t<>(0.805173f, 0.285232f, 0.519942f))) < result_tolerance);

    norms.clear();
    BOOST_REQUIRE(uut->normals(&norms, 2) != nullptr);
    BOOST_CHECK(norms.size() == 3);
    BOOST_CHECK(fabs(magnitude(norms[0] - point_t<>(0.625929f, 0.457246f, 0.631774f))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(norms[1] - point_t<>(0.382683f, 0.0f,      0.92388f ))) < result_tolerance);
    BOOST_CHECK(fabs(magnitude(norms[2] - point_t<>(0.805173f, 0.285232f, 0.519942f))) < result_tolerance);
}

BOOST_AUTO_TEST_SUITE_END()
}; /* namespace test */
}; /* namespace raptor_raytracer */
