#pragma once
/* Standard headers */
#include <chrono>

/* Boost headers */
#include "boost/noncopyable.hpp"

/* Convex decomposition headers */
#include "parser.h"


namespace raptor_convex_decomposition
{
namespace test
{
const float result_tolerance = 0.0005f;

#define LOAD_TEST_DATA(FILE)   const std::string test_name(boost::unit_test::framework::current_test_case().p_name); \
    BOOST_LOG_TRIVIAL(fatal) << "PERF 0 - Test: " << test_name; \
    const std::string data_dir(getenv("RAPTOR_DATA")); \
    BOOST_REQUIRE(load_off(data_dir + "/" + (FILE), &points, &triangles));


struct regression_fixture : private boost::noncopyable
{
    regression_fixture(const convex_decomposition_options &options) : options(options), uut(nullptr) {  }

    void run()
    {
        const auto t0(std::chrono::system_clock::now());
        uut.reset(new convex_decomposition(points, triangles, options));
        const auto t1(std::chrono::system_clock::now());
        BOOST_LOG_TRIVIAL(fatal) << "PERF 5 - Runtime ms: " << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
    }

    void check(const std::string &file)
    {
        const float original_volume = convex_mesh(points, triangles).volume();
        BOOST_LOG_TRIVIAL(fatal) << "PERF 1 - Original Volume: " << original_volume;
        BOOST_LOG_TRIVIAL(fatal) << "PERF 2 - Voxelised Relative Volume: " << (((uut->get_primitive_set()->compute_volume() / original_volume) - 1.0f) * 100.0f);

        BOOST_LOG_TRIVIAL(error) << "PERF 3 - Decomposed Number of hulls: " << uut->number_of_convex_hulls();
        float total_volume = 0.0f;
        for (int i = 0; i <  uut->number_of_convex_hulls(); ++i)
        {
            total_volume += uut->get_convex_hull(i)->volume();
        }
        BOOST_LOG_TRIVIAL(error) << "PERF 4 - Decomposed Relative Volume: " << (((total_volume / original_volume) - 1.0f) * 100.0f);

        /* Sum number of points and triangles */
        int nr_pts = 0;
        int nr_tri = 0;
        for (int i = 0; i < uut->number_of_convex_hulls(); ++i)
        {
            const auto *const mesh = uut->get_convex_hull(i);
            nr_pts += mesh->number_of_points();
            nr_tri += mesh->number_of_triangles();
        }

        /* Save actual results */
        save_convex_decomposition(*uut, "test_data/" + file + ".wrl");
        save_off(*uut, "test_data/" + file + "_act.off", nr_pts, nr_tri);

        /* Load expected data */
        std::vector<point_t>    points_exp;
        std::vector<point_ti<>> triangles_exp;
        BOOST_REQUIRE(load_off("test_data/" + file + "_exp.off", &points_exp, &triangles_exp));

        /* Check points */
        BOOST_REQUIRE(nr_pts == static_cast<int>(points_exp.size()));

        int pt_idx = 0;
        for (int i = 0; i < uut->number_of_convex_hulls(); ++i)
        {
            const auto *const mesh = uut->get_convex_hull(i);
            for (const auto &pt : mesh->points())
            {
                if (std::fabs(magnitude(pt - points_exp[pt_idx])) > result_tolerance)
                {
                    BOOST_CHECK(std::fabs(magnitude(pt - points_exp[pt_idx])) < result_tolerance);
                }
                pt_idx++;
            }
        }

        /* Check triangles */
        BOOST_REQUIRE(nr_tri == static_cast<int>(triangles_exp.size()));

        int tri_idx = 0;
        for (int i = 0; i < uut->number_of_convex_hulls(); ++i)
        {
            const auto *const mesh = uut->get_convex_hull(i);
            for (const auto &t : mesh->triangles())
            {
                BOOST_CHECK(t == triangles_exp[tri_idx++]);
            }
        }
    }

    convex_decomposition_options            options;
    std::unique_ptr<convex_decomposition>   uut;
    std::vector<point_t>                    points;
    std::vector<point_ti<>>                 triangles;
};
}; /* namespace test */
}; /* namespace raptor_convex_decomposition */
