/* Standard headers */
#include <iostream>
#include <string>

/* Boost headers */
#include "boost/program_options.hpp"

/* Common headers */
#include "logging.h"

/* Convex decomposition headers */
#include "convex_decomposition.h"
#include "convex_decomposition_options.h"
#include "discretisation_type.h"
#include "hull_parser.h"


namespace po = boost::program_options;
using raptor_convex_decomposition::discretisation_type_t;

int main(int argc, char const *argv[])
{
    std::string             input;
    std::string             output;
    float                   concavity;
    float                   alpha;
    float                   beta;
    float                   gamma;
    float                   min_volume_per_hull;
    int                     resolution;
    int                     max_vertices_per_hull;
    int                     depth;
    int                     plane_down_sampling;
    int                     hull_down_sampling;
    discretisation_type_t   mode;
    bool                    pca;
    bool                    approx_hulls;
    po::options_description desc("Allowed options");
    desc.add_options()
        ( "help",                                                                                                           "produce help message")
        ( "input",                  po::value<std::string>(&input)->required(),                                             "Input file") 
        ( "output",                 po::value<std::string>(&output)->required(),                                            "Output file") 
        ( "concavity",              po::value<float>(&concavity)->default_value(0),                                         "concavity") 
        ( "alpha",                  po::value<float>(&alpha)->default_value(0),                                             "alpha") 
        ( "beta",                   po::value<float>(&beta)->default_value(0),                                              "beta") 
        ( "gamma",                  po::value<float>(&gamma)->default_value(0),                                             "gamma") 
        ( "min_volume_per_hull",    po::value<float>(&min_volume_per_hull)->default_value(0),                               "min_volume_per_hull") 
        ( "resolution",             po::value<int>(&resolution)->default_value(100000),                                     "resolution") 
        ( "max_vertices_per_hull",  po::value<int>(&max_vertices_per_hull)->default_value(64),                              "max_vertices_per_hull") 
        ( "depth",                  po::value<int>(&depth)->default_value(20),                                              "depth") 
        ( "plane_down_sampling",    po::value<int>(&plane_down_sampling)->default_value(4),                                 "plane_down_sampling") 
        ( "hull_down_sampling",     po::value<int>(&hull_down_sampling)->default_value(4),                                  "hull_down_sampling") 
        ( "mode",                   po::value<discretisation_type_t>(&mode)->default_value(discretisation_type_t::voxel),   "mode") 
        ( "pca",                    po::value<bool>(&pca)->default_value(false),                                            "pca") 
        ( "approx_hulls",           po::value<bool>(&approx_hulls)->default_value(true),                                    "approx_hulls")         
    ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);    

    if (vm.count("help"))
    {
        BOOST_LOG_TRIVIAL(error) << desc << std::endl;
        return 1;
    }

    /* Load input */
    std::vector<point_t<>>  points;
    std::vector<point_ti<>> triangles;
    if (!raptor_parsers::load_off(input, &points, &triangles))
    {
        return 2;
    }

    /* Decompose */
    const raptor_convex_decomposition::convex_decomposition_options params(concavity, alpha, beta, gamma, min_volume_per_hull, resolution, max_vertices_per_hull, depth, plane_down_sampling, hull_down_sampling, mode, pca, approx_hulls);
    raptor_convex_decomposition::convex_decomposition cd(points, triangles, params);

    /* Save output */
    raptor_convex_decomposition::save_convex_decomposition(cd, output);

    return 0;
}