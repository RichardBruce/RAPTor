#ifdef STAND_ALONE
#define BOOST_TEST_MODULE dac_convex_hull_2d test
#endif /* #ifdef STAND_ALONE */


/* Standard headers */
#include <chrono>
#include <random>

/* Common headers */
#include "logging.h"

/* Boost headers */
#include "boost/test/unit_test.hpp"
#include "boost/noncopyable.hpp"

/* Convex decomposition headers */
#include "dac_convex_hull_2d.h"


namespace test
{
/* Test data */
struct dac_convex_hull_2d_fixture : private boost::noncopyable
{
    dac_convex_hull_2d_fixture() :
        scratch(std::make_shared<std::vector<point2d<long>>>(100, point2d<long>(0, 0))),
        data_point0({{ 0, 0 }}),
        data_point1({{ 9, 0 }}),
        data_line_h0({{ 1, 0 }, { 3, 0 }}),
        data_line_h1({{ 5, 0 }, { 7, 0 }}),
        data_line_v0({{ 0, 2 }, { 0, -2 }}),
        data_line_v1({{ 1, -2 }, { 1, 2 }}),
        data_triange({{ 0, 0 }, { 2, 0 }, { 1, 1 }}),
        data_square({{ 0, 0 }, { 2, 0 }, { 2, 2 }, { 0, 2 }}),
        data_curve_left({{ 0, 3 }, { 1, 3 }, { 2, 2 }, { 2, 1 }, { 1, 0 }, { 0, 0 }}),
        data_curve_right({{ 5, 0 }, { 4, 0 }, { 3, 1 }, { 3, 2 }, { 4, 3 }, { 5, 3 }}),
        data_curve_left_rev({{ 2, 0 }, { 1, 0 }, { 0, 1 }, { 0, 2 }, { 1, 3 }, { 2, 3 }}),
        data_curve_right_rev({{ 3, 3 }, { 4, 3 }, { 5, 2 }, { 5, 1 }, { 4, 0 }, { 3, 0 }}),
        data_curve_left_rot({{ 1, 0 }, { 0, 0 }, { 0, 3 }, { 1, 3 }, { 2, 2 }, { 2, 1 }}),
        data_curve_right_rot({{ 4, 3 }, { 5, 3 }, { 5, 0 }, { 4, 0 }, { 3, 1 }, { 3, 2 }}),
        data_curve_left_rot_rev({{ 1, 3 }, { 2, 3 }, { 2, 0 }, { 1, 0 }, { 0, 1 }, { 0, 2 }}),
        data_curve_right_rot_rev({{ 4, 0 }, { 3, 0 }, { 3, 3 }, { 4, 3 }, { 5, 2 }, { 5, 1 }}),
        point0(std::make_unique<dac_convex_hull_2d>(scratch, &data_point0[0], &data_point0[data_point0.size()])),
        point1(std::make_unique<dac_convex_hull_2d>(scratch, &data_point1[0], &data_point1[data_point1.size()])),
        line_h0(std::make_unique<dac_convex_hull_2d>(scratch, &data_line_h0[0], &data_line_h0[data_line_h0.size()])),
        line_h1(std::make_unique<dac_convex_hull_2d>(scratch, &data_line_h1[0], &data_line_h1[data_line_h1.size()])),
        line_v0(std::make_unique<dac_convex_hull_2d>(scratch, &data_line_v0[0], &data_line_v0[data_line_v0.size()])),
        line_v1(std::make_unique<dac_convex_hull_2d>(scratch, &data_line_v1[0], &data_line_v1[data_line_v1.size()])),
        triange(std::make_unique<dac_convex_hull_2d>(scratch, &data_triange[0], &data_triange[data_triange.size()])),
        square(std::make_unique<dac_convex_hull_2d>(scratch, &data_square[0], &data_square[data_square.size()])),
        curve_left(std::make_unique<dac_convex_hull_2d>(scratch, &data_curve_left[0], &data_curve_left[data_curve_left.size()])),
        curve_right(std::make_unique<dac_convex_hull_2d>(scratch, &data_curve_right[0], &data_curve_right[data_curve_right.size()]))
    {  }

    std::pair<dac_convex_hull_2d, dac_convex_hull_2d> prepare_polygons(const std::vector<point2d<long>> &l, const std::vector<point2d<long>> &r)
    {
        data.insert(data.end(), l.begin(), l.end());
        data.insert(data.end(), r.begin(), r.end());
        return std::pair(dac_convex_hull_2d(scratch, &data[0], &data[l.size()]), dac_convex_hull_2d(scratch, &data[l.size()], &data[data.size()]));
    }

    std::default_random_engine                  random;
    std::shared_ptr<std::vector<point2d<long>>> scratch;
    std::vector<point2d<long>>                  data;
    std::vector<point2d<long>>                  data_point0;
    std::vector<point2d<long>>                  data_point1;
    std::vector<point2d<long>>                  data_line_h0;
    std::vector<point2d<long>>                  data_line_h1;
    std::vector<point2d<long>>                  data_line_v0;
    std::vector<point2d<long>>                  data_line_v1;
    std::vector<point2d<long>>                  data_triange;
    std::vector<point2d<long>>                  data_square;
    std::vector<point2d<long>>                  data_curve_left;
    std::vector<point2d<long>>                  data_curve_right;
    std::vector<point2d<long>>                  data_curve_left_rev;
    std::vector<point2d<long>>                  data_curve_right_rev;
    std::vector<point2d<long>>                  data_curve_left_rot;
    std::vector<point2d<long>>                  data_curve_right_rot;
    std::vector<point2d<long>>                  data_curve_left_rot_rev;
    std::vector<point2d<long>>                  data_curve_right_rot_rev;
    std::unique_ptr<dac_convex_hull_2d>         point0;
    std::unique_ptr<dac_convex_hull_2d>         point1;
    std::unique_ptr<dac_convex_hull_2d>         line_h0;
    std::unique_ptr<dac_convex_hull_2d>         line_h1;
    std::unique_ptr<dac_convex_hull_2d>         line_v0;
    std::unique_ptr<dac_convex_hull_2d>         line_v1;
    std::unique_ptr<dac_convex_hull_2d>         triange;
    std::unique_ptr<dac_convex_hull_2d>         square;
    std::unique_ptr<dac_convex_hull_2d>         curve_left;
    std::unique_ptr<dac_convex_hull_2d>         curve_right;
};

BOOST_FIXTURE_TEST_SUITE( dac_convex_hull_2d_tests, dac_convex_hull_2d_fixture );


/* Test find tangent*/
BOOST_AUTO_TEST_CASE( point_tangent_test )
{
    const auto [a0, b0] = point0->find_tangent(*point0, 0, 0, 1);
    BOOST_CHECK(a0 == 0);
    BOOST_CHECK(b0 == 0);

    const auto [a1, b1] = point0->find_tangent(*point0, 0, 0, -1);
    BOOST_CHECK(a1 == 0);
    BOOST_CHECK(b1 == 0);
}

BOOST_AUTO_TEST_CASE( point_line_h_tangent_test )
{
    const auto [a0, b0] = point0->find_tangent(*line_h0, 0, 0, 1);
    BOOST_CHECK(a0 == 0);
    BOOST_CHECK(b0 == 1);

    const auto [a1, b1] = point0->find_tangent(*line_h0, 0, 0, -1);
    BOOST_CHECK(a1 == 0);
    BOOST_CHECK(b1 == 1);

    const auto [a2, b2] = point0->find_tangent(*line_h0, 0, 1, 1);
    BOOST_CHECK(a2 == 0);
    BOOST_CHECK(b2 == 1);

    const auto [a3, b3] = point0->find_tangent(*line_h0, 0, 1, -1);
    BOOST_CHECK(a3 == 0);
    BOOST_CHECK(b3 == 1);
}

BOOST_AUTO_TEST_CASE( line_h_point_tangent_test )
{
    const auto [a0, b0] = line_h0->find_tangent(*point1, 0, 0, 1);
    BOOST_CHECK(a0 == 0);
    BOOST_CHECK(b0 == 0);

    const auto [a1, b1] = line_h0->find_tangent(*point1, 0, 0, -1);
    BOOST_CHECK(a1 == 0);
    BOOST_CHECK(b1 == 0);

    const auto [a2, b2] = line_h0->find_tangent(*point1, 1, 0, 1);
    BOOST_CHECK(a2 == 0);
    BOOST_CHECK(b2 == 0);

    const auto [a3, b3] = line_h0->find_tangent(*point1, 1, 0, -1);
    BOOST_CHECK(a3 == 0);
    BOOST_CHECK(b3 == 0);
}

BOOST_AUTO_TEST_CASE( line_h_0_line_h_1_tangent_test )
{
    const auto [a0, b0] = line_h0->find_tangent(*line_h1, 0, 0, 1);
    BOOST_CHECK(a0 == 0);
    BOOST_CHECK(b0 == 1);

    const auto [a1, b1] = line_h0->find_tangent(*line_h1, 0, 0, -1);
    BOOST_CHECK(a1 == 0);
    BOOST_CHECK(b1 == 1);

    const auto [a2, b2] = line_h0->find_tangent(*line_h1, 1, 0, 1);
    BOOST_CHECK(a2 == 0);
    BOOST_CHECK(b2 == 1);

    const auto [a3, b3] = line_h0->find_tangent(*line_h1, 1, 0, -1);
    BOOST_CHECK(a3 == 0);
    BOOST_CHECK(b3 == 1);

    const auto [a4, b4] = line_h0->find_tangent(*line_h1, 0, 1, 1);
    BOOST_CHECK(a4 == 0);
    BOOST_CHECK(b4 == 1);

    const auto [a5, b5] = line_h0->find_tangent(*line_h1, 0, 1, -1);
    BOOST_CHECK(a5 == 0);
    BOOST_CHECK(b5 == 1);

    const auto [a6, b6] = line_h0->find_tangent(*line_h1, 1, 1, 1);
    BOOST_CHECK(a6 == 0);
    BOOST_CHECK(b6 == 1);

    const auto [a7, b7] = line_h0->find_tangent(*line_h1, 1, 1, -1);
    BOOST_CHECK(a7 == 0);
    BOOST_CHECK(b7 == 1);
}

BOOST_AUTO_TEST_CASE( point_line_v_tangent_test )
{
    const auto [a0, b0] = point0->find_tangent(*line_v1, 0, 0, 1);
    BOOST_CHECK(a0 == 0);
    BOOST_CHECK(b0 == 1);

    const auto [a1, b1] = point0->find_tangent(*line_v1, 0, 0, -1);
    BOOST_CHECK(a1 == 0);
    BOOST_CHECK(b1 == 0);
}

BOOST_AUTO_TEST_CASE( line_v_line_v_0_0_tangent_test )
{
    const auto [a0, b0] = line_v0->find_tangent(*line_v1, 0, 0, 1);
    BOOST_CHECK(a0 == 0);
    BOOST_CHECK(b0 == 1);

    const auto [a1, b1] = line_v0->find_tangent(*line_v1, 0, 0, -1);
    BOOST_CHECK(a1 == 1);
    BOOST_CHECK(b1 == 0);
}

BOOST_AUTO_TEST_CASE( line_v_line_v_0_1_tangent_test )
{
    const auto [a0, b0] = line_v0->find_tangent(*line_v1, 0, 1, 1);
    BOOST_CHECK(a0 == 0);
    BOOST_CHECK(b0 == 1);

    const auto [a1, b1] = line_v0->find_tangent(*line_v1, 0, 1, -1);
    BOOST_CHECK(a1 == 1);
    BOOST_CHECK(b1 == 0);
}

BOOST_AUTO_TEST_CASE( line_v_line_v_1_0_tangent_test )
{
    const auto [a0, b0] = line_v0->find_tangent(*line_v1, 1, 0, 1);
    BOOST_CHECK(a0 == 0);
    BOOST_CHECK(b0 == 1);

    const auto [a1, b1] = line_v0->find_tangent(*line_v1, 1, 0, -1);
    BOOST_CHECK(a1 == 1);
    BOOST_CHECK(b1 == 0);
}

BOOST_AUTO_TEST_CASE( line_v_line_v_1_1_tangent_test )
{
    const auto [a0, b0] = line_v0->find_tangent(*line_v1, 1, 1, 1);
    BOOST_CHECK(a0 == 0);
    BOOST_CHECK(b0 == 1);

    const auto [a1, b1] = line_v0->find_tangent(*line_v1, 1, 1, -1);
    BOOST_CHECK(a1 == 1);
    BOOST_CHECK(b1 == 0);
}

BOOST_AUTO_TEST_CASE( curve_1_1_tangent_test )
{
    const auto [a0, b0] = curve_left->find_tangent(*curve_right, 5, 0, 1);
    BOOST_CHECK(a0 == 5); /* Cant move past the flat start*/
    BOOST_CHECK(b0 == 0);

    const auto [a1, b1] = curve_left->find_tangent(*curve_right, 4, 1, 1);
    BOOST_CHECK(a1 == 0);
    BOOST_CHECK(b1 == 5);

    const auto [a2, b2] = curve_left->find_tangent(*curve_right, 3, 2, 1);
    BOOST_CHECK(a2 == 0);
    BOOST_CHECK(b2 == 5);

    const auto [a3, b3] = curve_left->find_tangent(*curve_right, 2, 3, 1);
    BOOST_CHECK(a3 == 0);
    BOOST_CHECK(b3 == 5);

    const auto [a4, b4] = curve_left->find_tangent(*curve_right, 1, 4, 1);
    BOOST_CHECK(a4 == 0);
    BOOST_CHECK(b4 == 5);

    const auto [a5, b5] = curve_left->find_tangent(*curve_right, 0, 5, 1);
    BOOST_CHECK(a5 == 0);
    BOOST_CHECK(b5 == 5);
}

BOOST_AUTO_TEST_CASE( curve_m1_m1_tangent_test )
{
    const auto [a0, b0] = curve_left->find_tangent(*curve_right, 5, 0, -1);
    BOOST_CHECK(a0 == 5);
    BOOST_CHECK(b0 == 0);

    const auto [a1, b1] = curve_left->find_tangent(*curve_right, 4, 1, -1);
    BOOST_CHECK(a1 == 5);
    BOOST_CHECK(b1 == 0);

    const auto [a2, b2] = curve_left->find_tangent(*curve_right, 3, 2, -1);
    BOOST_CHECK(a2 == 5);
    BOOST_CHECK(b2 == 0);

    const auto [a3, b3] = curve_left->find_tangent(*curve_right, 2, 3, -1);
    BOOST_CHECK(a3 == 5);
    BOOST_CHECK(b3 == 0);

    const auto [a4, b4] = curve_left->find_tangent(*curve_right, 1, 4, -1);
    BOOST_CHECK(a4 == 5);
    BOOST_CHECK(b4 == 0);

    const auto [a5, b5] = curve_left->find_tangent(*curve_right, 0, 5, -1);
    BOOST_CHECK(a5 == 0);   /* Cant move past the flat start*/
    BOOST_CHECK(b5 == 5);
}

/* Merge test */
BOOST_AUTO_TEST_CASE( point_merge_test )
{
    auto [l, r] = prepare_polygons(data_point0, data_point1);
    l.merge(r);

    BOOST_CHECK(l.size() == 2);
    BOOST_CHECK(l[0] == point2d<long>(9, 0));
    BOOST_CHECK(l[1] == point2d<long>(0, 0));
}

BOOST_AUTO_TEST_CASE( point_line_h_merge_test )
{
    auto [l, r] = prepare_polygons(data_point0, data_line_h0);
    l.merge(r);

    BOOST_CHECK(l.size() == 2);
    BOOST_CHECK(l[0] == point2d<long>(3, 0));
    BOOST_CHECK(l[1] == point2d<long>(0, 0));
}

BOOST_AUTO_TEST_CASE( line_h_point_merge_test )
{
    auto [l, r] = prepare_polygons(data_line_h0, data_point1);
    l.merge(r);

    BOOST_CHECK(l.size() == 2);
    BOOST_CHECK(l[0] == point2d<long>(1, 0));
    BOOST_CHECK(l[1] == point2d<long>(9, 0));
}

BOOST_AUTO_TEST_CASE( line_h_0_line_h_1_merge_test )
{
    auto [l, r] = prepare_polygons(data_line_h0, data_line_h1);
    l.merge(r);

    BOOST_CHECK(l.size() == 2);
    BOOST_CHECK(l[0] == point2d<long>(1, 0));
    BOOST_CHECK(l[1] == point2d<long>(7, 0));
}

BOOST_AUTO_TEST_CASE( point0_line_v1_merge_test )
{
    auto [l, r] = prepare_polygons(data_point0, data_line_v1);
    l.merge(r);

    BOOST_CHECK(l.size() == 3);
    BOOST_CHECK(l[0] == point2d<long>(1,  2));
    BOOST_CHECK(l[1] == point2d<long>(1, -2));
    BOOST_CHECK(l[2] == point2d<long>(0,  0));
}

BOOST_AUTO_TEST_CASE( point1_line_v0_merge_test )
{
    auto [l, r] = prepare_polygons(data_line_v0, data_point1);
    l.merge(r);

    BOOST_CHECK(l.size() == 3);
    BOOST_CHECK(l[0] == point2d<long>(0,  2));
    BOOST_CHECK(l[1] == point2d<long>(9,  0));
    BOOST_CHECK(l[2] == point2d<long>(0, -2));
}

BOOST_AUTO_TEST_CASE( line_v0_line_v1_merge_test )
{
    auto [l, r] = prepare_polygons(data_line_v0, data_line_v1);
    l.merge(r);

    BOOST_CHECK(l.size() == 4);
    BOOST_CHECK(l[0] == point2d<long>(0,  2));
    BOOST_CHECK(l[1] == point2d<long>(1,  2));
    BOOST_CHECK(l[2] == point2d<long>(1, -2));
    BOOST_CHECK(l[3] == point2d<long>(0, -2));
}

BOOST_AUTO_TEST_CASE( curve_merge_test )
{
    auto [l, r] = prepare_polygons(data_curve_left, data_curve_right);
    l.merge(r);

    BOOST_CHECK(l.size() == 4);
    BOOST_CHECK(l[0] == point2d<long>(0, 3));
    BOOST_CHECK(l[1] == point2d<long>(5, 3));
    BOOST_CHECK(l[2] == point2d<long>(5, 0));
    BOOST_CHECK(l[3] == point2d<long>(0, 0));
}

BOOST_AUTO_TEST_CASE( curve_merge_rev_test )
{
    auto [l, r] = prepare_polygons(data_curve_left_rev, data_curve_right_rev);
    l.merge(r);

    BOOST_CHECK(l.size() == 8);
    BOOST_CHECK(l[0] == point2d<long>(5, 2));
    BOOST_CHECK(l[1] == point2d<long>(5, 1));
    BOOST_CHECK(l[2] == point2d<long>(4, 0));
    BOOST_CHECK(l[3] == point2d<long>(1, 0));
    BOOST_CHECK(l[4] == point2d<long>(0, 1));
    BOOST_CHECK(l[5] == point2d<long>(0, 2));
    BOOST_CHECK(l[6] == point2d<long>(1, 3));
    BOOST_CHECK(l[7] == point2d<long>(4, 3));
}

BOOST_AUTO_TEST_CASE( curve_merge_rot_test )
{
    auto [l, r] = prepare_polygons(data_curve_left_rot, data_curve_right_rot);
    l.merge(r);

    BOOST_CHECK(l.size() == 4);
    BOOST_CHECK(l[0] == point2d<long>(0, 0));
    BOOST_CHECK(l[1] == point2d<long>(0, 3));
    BOOST_CHECK(l[2] == point2d<long>(5, 3));
    BOOST_CHECK(l[3] == point2d<long>(5, 0));
}

BOOST_AUTO_TEST_CASE( curve_merge_rot_rev_test )
{
    auto [l, r] = prepare_polygons(data_curve_left_rot_rev, data_curve_right_rot_rev);
    l.merge(r);

    BOOST_CHECK(l.size() == 8);
    BOOST_CHECK(l[0] == point2d<long>(1, 3));
    BOOST_CHECK(l[1] == point2d<long>(4, 3));
    BOOST_CHECK(l[2] == point2d<long>(5, 2));
    BOOST_CHECK(l[3] == point2d<long>(5, 1));
    BOOST_CHECK(l[4] == point2d<long>(4, 0));
    BOOST_CHECK(l[5] == point2d<long>(1, 0));
    BOOST_CHECK(l[6] == point2d<long>(0, 1));
    BOOST_CHECK(l[7] == point2d<long>(0, 2));
}

/* Clean points */
BOOST_AUTO_TEST_CASE( clean_points_co_incident_test )
{
    data = {{0, 0}, {0, 0}, {0, 0}, {0, 0}};
    clean_points(data);

    BOOST_CHECK(data.size() == 1);
    BOOST_CHECK(data[0] == point2d<long>(0, 0));
}

BOOST_AUTO_TEST_CASE( clean_points_range_test )
{
    data = {{0, -1}, {0, 1}};
    clean_points(data);

    BOOST_CHECK(data.size() == 2);
    BOOST_CHECK(data[0] == point2d<long>(0, -1));
    BOOST_CHECK(data[1] == point2d<long>(0,  1));
}

BOOST_AUTO_TEST_CASE( clean_points_co_linear_test )
{
    data = {{0, 3}, {0, -1}, {0, 1}, {0, -2}};
    clean_points(data);

    BOOST_CHECK(data.size() == 2);
    BOOST_CHECK(data[0] == point2d<long>(0, -2));
    BOOST_CHECK(data[1] == point2d<long>(0,  3));
}

BOOST_AUTO_TEST_CASE( clean_points_co_linear_co_incident_test )
{
    data = {{0, 3}, {0, -2}, {0, -1}, {0, 1}, {0, -2}};
    clean_points(data);

    BOOST_CHECK(data.size() == 2);
    BOOST_CHECK(data[0] == point2d<long>(0, -2));
    BOOST_CHECK(data[1] == point2d<long>(0,  3));
}

BOOST_AUTO_TEST_CASE( clean_points_x_co_linear_co_incident_test )
{
    data =  {
                { 0, 3}, { 0, -2}, { 0, -1}, { 0, 1}, { 0, -2},
                { 4, 3}, { 4, -2}, { 4, -1}, { 4, 1}, { 4, -2},
                { 1, 3}, { 1, -2}, { 1, -1}, { 1, 1}, { 1, -2},
                {-1, 3}, {-1, -2}, {-1, -1}, {-1, 1}, {-1, -2}
            };
    clean_points(data);

    BOOST_CHECK(data.size() == 8);
    BOOST_CHECK(data[0] == point2d<long>(-1, -2));
    BOOST_CHECK(data[1] == point2d<long>(-1,  3));
    BOOST_CHECK(data[2] == point2d<long>( 0, -2));
    BOOST_CHECK(data[3] == point2d<long>( 0,  3));
    BOOST_CHECK(data[4] == point2d<long>( 1, -2));
    BOOST_CHECK(data[5] == point2d<long>( 1,  3));
    BOOST_CHECK(data[6] == point2d<long>( 4, -2));
    BOOST_CHECK(data[7] == point2d<long>( 4,  3));
}

/* Special test */
BOOST_AUTO_TEST_CASE( special_0_test )
{
    data =  {
                {-2233,  998}, {-2228,  967}, {-2233, -969}, {-2234, -997}, {-2245, -1000}, {-2264, 999},
                {-2200, -989}, {-2212, -995}, {-2227,  995}
            };
    dac_convex_hull_2d l(scratch, &data[0], &data[6]);
    dac_convex_hull_2d r(scratch, &data[6], &data[9]);
    l.merge(r);

    BOOST_CHECK(l.size() == 6);
    BOOST_CHECK(data[0] == point2d<long>(-2233,   998));
    BOOST_CHECK(data[1] == point2d<long>(-2227,   995));
    BOOST_CHECK(data[2] == point2d<long>(-2200,  -989));
    BOOST_CHECK(data[3] == point2d<long>(-2212,  -995));
    BOOST_CHECK(data[4] == point2d<long>(-2245, -1000));
    BOOST_CHECK(data[5] == point2d<long>(-2264,   999));
}

BOOST_AUTO_TEST_CASE( special_1_test )
{
    data =  {
                {-9996,  97}, {-9995, -98},
                {-9994, -91}, {-9995, 100}, {-9994, 73}
            };
    dac_convex_hull_2d l(scratch, &data[0], &data[2]);
    dac_convex_hull_2d r(scratch, &data[2], &data[5]);
    l.merge(r);

    BOOST_CHECK(l.size() == 5);
    BOOST_CHECK(data[0] == point2d<long>(-9996,  97));
    BOOST_CHECK(data[1] == point2d<long>(-9995, 100));
    BOOST_CHECK(data[2] == point2d<long>(-9994,  73));
    BOOST_CHECK(data[3] == point2d<long>(-9994, -91));
    BOOST_CHECK(data[4] == point2d<long>(-9995, -98));
}

/* Performance tests */
BOOST_AUTO_TEST_CASE( square_point_cloud_performance_test )
{
    const long size_x = 1000000000;
    const long size_y = 1000000000;
    const int number_points = 1000000;

    std::uniform_int_distribution<long> dist_x(-size_x, size_x);
    std::uniform_int_distribution<long> dist_y(-size_y, size_y);
    for (int i = 0; i < number_points; ++i)
    {
        const long x = dist_x(random);
        const long y = dist_y(random);
        data.emplace_back(x, y);
    }

    std::cout << "building" << std::endl;
    auto t0(std::chrono::system_clock::now());
    const auto hull(build(data));
    auto t1(std::chrono::system_clock::now());
    BOOST_LOG_TRIVIAL(fatal) << "PERF 0 - Test: square_point_cloud_performance_test " << data.size() << " hull size " << hull.size();
    BOOST_LOG_TRIVIAL(fatal) << "PERF 1 - Runtime us: " << std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();

    for (size_t i = 0; i < hull.size() - 2; ++i)
    {
        BOOST_CHECK_MESSAGE(winding(hull[i], hull[i + 1], hull[i + 2]) < 0, (i + 1) << ", " << hull[i + 1]);
    }
}

BOOST_AUTO_TEST_CASE( circle_point_cloud_performance_test )
{
    const float size_r = 1000000000.0;
    const int number_points = 1000000;

    std::uniform_real_distribution<float> dist_r(0, size_r);
    std::uniform_real_distribution<float> dist_pi(0, 2.0 * M_PI);
    for (int i = 0; i < number_points; ++i)
    {
        const float r = dist_r(random);
        const float pi = dist_pi(random);
        data.emplace_back(r * cos(pi), r * sin(pi));
    }

    std::cout << "building" << std::endl;
    auto t0(std::chrono::system_clock::now());
    const auto hull(build(data));
    auto t1(std::chrono::system_clock::now());
    BOOST_LOG_TRIVIAL(fatal) << "PERF 0 - Test: circle_point_cloud_performance_test " << data.size() << " hull size " << hull.size();
    BOOST_LOG_TRIVIAL(fatal) << "PERF 1 - Runtime us: " << std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();

    for (size_t i = 0; i < hull.size() - 2; ++i)
    {
        BOOST_CHECK_MESSAGE(winding(hull[i], hull[i + 1], hull[i + 2]) < 0, (i + 1) << ", " << hull[i + 1]);
    }
}

BOOST_AUTO_TEST_SUITE_END()
}; /* namespace test */
