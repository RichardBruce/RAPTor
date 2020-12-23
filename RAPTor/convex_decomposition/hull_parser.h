#pragma once

/* Standard headers */
#include <iostream>
#include <fstream>
#include <random>

/* Boost headers */

/* Common headers */
#include "logging.h"
#include "parser_common.h"

/* Parsers */
#include "parser.h"

/* Convex decomposition headers */
#include "convex_decomposition.h"


namespace raptor_convex_decomposition
{
inline bool save_off(const convex_decomposition &cd, const std::string &out_file, const int nr_pts, const int nr_tri) 
{
    /* Open file */
    std::ofstream fout(out_file.c_str());
    if (!fout.is_open())
    {
        BOOST_LOG_TRIVIAL(error) << "Can't open file: " << out_file;
        return false;
    }

    /* Write header */
    fout << "OFF" << std::endl;
    fout << nr_pts << " " << nr_tri << " " << 0 << std::endl;

    /* For each mesh write points */
    fout.setf(std::ios::fixed, std::ios::floatfield);
    fout.setf(std::ios::showpoint);
    fout.precision(6);
    for (int i = 0; i < cd.number_of_convex_hulls(); ++i)
    {
        const auto *const mesh = cd.get_convex_hull(i);
        for (const auto &pt : mesh->points())
        {
            fout << pt.x << " " << pt.y << " " << pt.z << std::endl;
        }
    }

    /* For each mesh write triangles */
    for (int i = 0; i < cd.number_of_convex_hulls(); ++i)
    {
        const auto *const mesh = cd.get_convex_hull(i);
        for (const auto &t : mesh->triangles())
        {
            fout << "3 " << t.x << " " << t.y << " " << t.z << std::endl;
        }
    }

    fout.close();
    return true;
}

inline void save_convex_decomposition(const convex_decomposition &cd, const std::string &out_file)
{
    /* Open output file */
    std::ofstream fout(out_file.c_str());
    if (!fout.is_open())
    {
        BOOST_LOG_TRIVIAL(error) << "Can't open file: " << out_file;
        return;
    }

    /* Write output */    
    std::default_random_engine gen;
    std::uniform_real_distribution<float> dist(0.0f, 1.0f);
    for (int i = 0; i < cd.number_of_convex_hulls(); ++i)
    {
        const auto *const mesh = cd.get_convex_hull(i);
        raptor_parsers::save_vrml2(fout, mesh->points(), mesh->triangles(), dist(gen), dist(gen), dist(gen));
    }

    /* Clean up */
    fout.close();
}
}; /* namespace raptor_convex_decomposition */
