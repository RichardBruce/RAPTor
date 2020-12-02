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
const float result_tolerance = 0.000001f;

#define LOAD_TEST_DATA(FILE)   const std::string test_name(boost::unit_test::framework::current_test_case().p_name); \
    BOOST_LOG_TRIVIAL(fatal) << "PERF 0 - Test: " << test_name; \
    const std::string data_dir(getenv("RAPTOR_DATA")); \
    BOOST_REQUIRE(raptor_parsers::load_off(data_dir + "/" + (FILE), &points, &triangles));


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
        BOOST_LOG_TRIVIAL(fatal) << "PERF 3 - Runtime us: " << std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
        BOOST_LOG_TRIVIAL(fatal) << "PERF 4 - Iterations: " << uut->iterations();
        BOOST_LOG_TRIVIAL(fatal) << "PERF 5 - Cost: " << uut->cost();
    }

    void check(const std::string &file)
    {
        /* Save actual results */
        std::vector<point_t<>>  points_act;
        std::vector<point_ti<>> triangles_act;
        uut->compact_output(points_act, triangles_act);
        raptor_parsers::save_off(points_act, triangles_act, "test_data/" + file + "_act.off");
        raptor_parsers::save_vrml2("test_data/" + file + "_act.wrl", points_act, triangles_act, 1.0f, 1.0f, 1.0f);

        /* Load expected data */
        std::vector<point_t<>>  points_exp;
        std::vector<point_ti<>> triangles_exp;
        BOOST_REQUIRE(raptor_parsers::load_off("test_data/" + file + "_exp.off", &points_exp, &triangles_exp));

        /* Check points */
        BOOST_REQUIRE(points_act.size() == points_exp.size());
        for (size_t i = 0; i < points_exp.size(); ++i)
        {
            if (std::fabs(magnitude(points_act[i] - points_exp[i])) > result_tolerance)
            {
                BOOST_CHECK(std::fabs(magnitude(points_act[i] - points_exp[i])) < result_tolerance);
            }
        }

        /* Check triangles */
        BOOST_REQUIRE(triangles_act.size() == triangles_exp.size());
        for (size_t i = 0; i < triangles_exp.size(); ++i)
        {
            BOOST_CHECK(triangles_act[i] == triangles_exp[i]);
        }
    }

    mesh_decimation_options             options;
    std::unique_ptr<mesh_decimation>    uut;
    std::vector<point_t<>>              points;
    std::vector<point_ti<>>             triangles;
};
} /* namespace test */
} /* namespace raptor_mesh_decimation */
