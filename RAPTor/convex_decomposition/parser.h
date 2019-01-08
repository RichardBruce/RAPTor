#pragma once

/* Standard headers */
#include <iostream>
#include <fstream>
#include <random>

/* Boost headers */

/* Common headers */
#include "logging.h"
#include "parser_common.h"

/* Convex decomposition headers */
#include "convex_decomposition.h"


namespace raptor_convex_decomposition
{
inline bool load_off(const std::string &in_file, std::vector<point_t> *const points, std::vector<point_ti<>> *const triangles)
{
    /* Open file */
    std::ifstream off_file(in_file.c_str());
    if (!off_file.is_open())
    {
        BOOST_LOG_TRIVIAL(error) << "File: " << in_file << " not found";
        return false;
    }

    /* Find the size of the file */
    off_file.seekg(0, std::ios::end);
    const size_t len = off_file.tellg();
    off_file.seekg(0, std::ios::beg);
    
    /* Read the whole file into a buffer */
    BOOST_LOG_TRIVIAL(info) << "Loading: " << in_file;
    std::unique_ptr<char []> buffer(new char [len]);
    off_file.read(buffer.get(), len);
    const char *at = &buffer[0];

    /* Check header */
    const std::string header(raptor_raytracer::get_this_string(&at));
    if ((header != "OFF") && (header != "OFF\r"))
    {
        BOOST_LOG_TRIVIAL(error) << "Format not recognized";
        off_file.close();
        return false;
    }

    /* Get vertex and face counts */
    // find_next_line(&at);
    const unsigned int nr_v = raptor_raytracer::get_this_unsigned(&at);
    const unsigned int nr_f = raptor_raytracer::get_next_unsigned(&at);
    const unsigned int nr_e = raptor_raytracer::get_next_unsigned(&at);
    BOOST_LOG_TRIVIAL(trace) << "Scene has vertices: " << nr_v << ", faces: " << nr_f << ", edges: " << nr_e;
    if (nr_e != 0)
    {
        BOOST_LOG_TRIVIAL(warning) << "Expected number of edges to be 0";
    }
    raptor_raytracer::find_next_line(&at);

    /* Get vertices */
    point_t vert;
    points->reserve(nr_v);
    for (unsigned int i = 0; i < nr_v; ++i)
    {
        vert.x = raptor_raytracer::get_this_float(&at);
        vert.y = raptor_raytracer::get_next_float(&at);
        vert.z = raptor_raytracer::get_next_float(&at);
        points->push_back(vert);

        raptor_raytracer::find_next_line(&at);
    }

    /* Get faces */
    for (unsigned int i = 0; i < nr_f; ++i)
    {
        const unsigned int nr_verts = raptor_raytracer::get_this_unsigned(&at);
        assert(nr_verts == 3);

        const unsigned int vert_x = raptor_raytracer::get_next_unsigned(&at);
        const unsigned int vert_y = raptor_raytracer::get_next_unsigned(&at);
        const unsigned int vert_z = raptor_raytracer::get_next_unsigned(&at);
        triangles->push_back(point_ti<>(vert_x, vert_y, vert_z));

        raptor_raytracer::find_next_line(&at);
    }

    return true;
}


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

inline bool save_vrml2(std::ofstream &fout, const std::vector<point_t> &points, const std::vector<point_ti<>> &triangles, const float r, const float g, const float b)
{
    if (!fout.is_open())
    {
        BOOST_LOG_TRIVIAL(error) << "Can't open file";
        return false;
    }

    /* Set up stream */
    fout.setf(std::ios::fixed, std::ios::floatfield);
    fout.setf(std::ios::showpoint);
    fout.precision(6);

    /* Write header */
    fout << "#VRML V2.0 utf8" << std::endl;
    fout << "" << std::endl;
    fout << "# Vertices: " << points.size() << std::endl;
    fout << "# Triangles: " << triangles.size() << std::endl;
    fout << "" << std::endl;
    fout << "Group {" << std::endl;
    fout << "    children [" << std::endl;
    fout << "        Shape {" << std::endl;
    fout << "            appearance Appearance {" << std::endl;
    fout << "                material Material {" << std::endl;
    fout << "                    diffuseColor " << r << " "
                                                << g << " "
                                                << b << std::endl;
    fout << "                    ambientIntensity " << 0.5f << std::endl;
    fout << "                }" << std::endl;
    fout << "            }" << std::endl;
    fout << "            geometry IndexedFaceSet {" << std::endl;
    fout << "                ccw TRUE" << std::endl;
    fout << "                solid TRUE" << std::endl;
    fout << "                convex TRUE" << std::endl;

    /* Write points */
    if (!points.empty())
    {
        fout << "                coord DEF co Coordinate {" << std::endl;
        fout << "                    point [" << std::endl;
        for (const auto &p : points)
        {
            fout << "                        " << p.x << " " << p.y << " " << p.z << "," << std::endl;
        }
        fout << "                    ]" << std::endl;
        fout << "                }" << std::endl;
    }

    /* Write triangles */
    if (!triangles.empty())
    {
        fout << "                coordIndex [ " << std::endl;
        for (const auto &t : triangles)
        {
            fout << "                        " << t.x << ", " << t.y << ", " << t.z << ", -1," << std::endl;
        }
        fout << "                ]" << std::endl;
    }
    fout << "            }" << std::endl;
    fout << "        }" << std::endl;
    fout << "    ]" << std::endl;
    fout << "}" << std::endl;

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
        save_vrml2(fout, mesh->points(), mesh->triangles(), dist(gen), dist(gen), dist(gen));
    }

    /* Clean up */
    fout.close();
}
}; /* namespace raptor_convex_decomposition */