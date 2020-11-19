#pragma once
/* Standard headers */
#include <chrono>

/* Boost headers */
#include "boost/noncopyable.hpp"

/* Common headers */
#include "logging.h"

/* Parser headers */
#include "parser.h"

/* Mesh decimation headers */
#include "mesh_decimation.h"
#include "mesh_decimation_options.h"

namespace raptor_mesh_decimation
{
namespace test
{
#define LOAD_TEST_DATA(FILE)   const std::string test_name(boost::unit_test::framework::current_test_case().p_name); \
    BOOST_LOG_TRIVIAL(fatal) << "PERF 0 - Test: " << test_name; \
    const std::string data_dir(getenv("RAPTOR_DATA")); \
    BOOST_REQUIRE(raptor_parsers::load_off(data_dir + "/" + (FILE), &points, &triangles)); \
    raptor_parsers::save_vrml2(std::string("test_data/") + (FILE) + "_before.wrl", points, triangles, 1.0f, 1.0f, 1.0f);


struct regression_fixture : private boost::noncopyable
{
    regression_fixture(const mesh_decimation_options &options) : options(options), uut(nullptr) {  }

    void run()
    {
        BOOST_LOG_TRIVIAL(fatal) << "PERF 1 - # Vertices: " << points.size();
        BOOST_LOG_TRIVIAL(fatal) << "PERF 2 - # Triangles: " << triangles.size();
        const auto t0(std::chrono::system_clock::now());
        uut.reset(new mesh_decimation(points, triangles, options));
        const auto t1(std::chrono::system_clock::now());
        BOOST_LOG_TRIVIAL(fatal) << "PERF 3 - Runtime ms: " << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
        BOOST_LOG_TRIVIAL(fatal) << "PERF 4 - Iterations: " << uut->iterations();
        BOOST_LOG_TRIVIAL(fatal) << "PERF 5 - Cost: " << uut->cost();
    }

    void check(const std::string &file)
    {
        /* Save actual results */
        std::vector<point_t> vertices;
        std::vector<point_ti<>> tris;
        uut->compact_output(vertices, tris);
        raptor_parsers::save_off(vertices, tris, "test_data/" + file + "_act.off");
        raptor_parsers::save_vrml2("test_data/" + file + "_act.wrl", vertices, tris, 1.0f, 1.0f, 1.0f);

        // raptor_parsers::save_off(uut->vertices(), uut->triangles(), "test_data/" + file + "_act.off");
        // raptor_parsers::save_vrml2("test_data/" + file + "_act.wrl", uut->vertices(), uut->triangles(), 1.0f, 1.0f, 1.0f);

        // /* Load expected data */
        // std::vector<point_t>    points_exp;
        // std::vector<point_ti<>> triangles_exp;
        // BOOST_REQUIRE(load_off("test_data/" + file + "_exp.off", &points_exp, &triangles_exp));

        // /* Check points */
        // BOOST_REQUIRE(nr_pts == static_cast<int>(points_exp.size()));

        // int pt_idx = 0;
        // for (int i = 0; i < uut->number_of_convex_hulls(); ++i)
        // {
        //     const auto *const mesh = uut->get_convex_hull(i);
        //     for (const auto &pt : mesh->points())
        //     {
        //         if (std::fabs(magnitude(pt - points_exp[pt_idx])) > result_tolerance)
        //         {
        //             BOOST_CHECK(std::fabs(magnitude(pt - points_exp[pt_idx])) < result_tolerance);
        //         }
        //         pt_idx++;
        //     }
        // }

        // /* Check triangles */
        // BOOST_REQUIRE(nr_tri == static_cast<int>(triangles_exp.size()));

        // int tri_idx = 0;
        // for (int i = 0; i < uut->number_of_convex_hulls(); ++i)
        // {
        //     const auto *const mesh = uut->get_convex_hull(i);
        //     for (const auto &t : mesh->triangles())
        //     {
        //         BOOST_CHECK(t == triangles_exp[tri_idx++]);
        //     }
        // }
    }

    mesh_decimation_options             options;
    std::unique_ptr<mesh_decimation>    uut;
    std::vector<point_t>                points;
    std::vector<point_ti<>>             triangles;
};
} /* namespace test */
} /* namespace raptor_mesh_decimation */
