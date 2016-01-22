#pragma once
/* Standard headers */
#include <chrono>
#include <iostream>
#include <fstream>
#include <random>

/* Boost headers */
#include "boost/noncopyable.hpp"

namespace raptor_convex_decomposition
{
namespace test
{
const float result_tolerance = 0.0005f;

struct regression_fixture : private boost::noncopyable
{
    regression_fixture(const convex_decomposition_options &options) : options(options) {  }

    bool load_off(const std::string &file_name, std::vector<point_t> *const points, std::vector<point_ti> *const triangles)
    {
        FILE * fid = fopen(file_name.c_str(), "r");
        if (!fid)
        {
            std::cout << "Error: file " << file_name << " not found" << std::endl;
            return false;
        }

        std::cout << "Loading: " << file_name << std::endl;
        const std::string strOFF("OFF");
        char temp[1024];
        fscanf(fid, "%s", temp);
        if (std::string(temp) != strOFF)
        {
            std::cout << "Loading error: format not recognized" << std::endl;
            fclose(fid);
            return false;
        }
        else
        {
            int nv = 0;
            int nf = 0;
            int ne = 0;
            fscanf(fid, "%i", &nv);
            fscanf(fid, "%i", &nf);
            fscanf(fid, "%i", &ne);
            points->resize(nv);
            for (int p = 0; p < nv; p++)
            {
                fscanf(fid, "%f", &points->data()[p].x);
                fscanf(fid, "%f", &points->data()[p].y);
                fscanf(fid, "%f", &points->data()[p].z);
            }

            int s;
            triangles->resize(nf);
            for (int t = 0, r = 0; t < nf; ++t) {
                fscanf(fid, "%i", &s);
                if (s == 3)
                {
                    fscanf(fid, "%i", &(triangles->data()[r].x));
                    fscanf(fid, "%i", &(triangles->data()[r].y));
                    fscanf(fid, "%i", &(triangles->data()[r++].z));
                }
                else            // Fix me: support only triangular meshes
                {
                    for (int h = 0; h < s; ++h) fscanf(fid, "%i", &s);
                }
            }
            fclose(fid);
        }

        return true;
    }


    bool save_off(const convex_decomposition &uut, const std::string &out_file, const int nr_pts, const int nr_tri) 
    {
        /* Open file */
        std::ofstream fout(out_file.c_str());
        if (!fout.is_open())
        {
            return false;
        }

        /* Write header */
        fout << "OFF" << std::endl;
        fout << nr_pts << " " << nr_tri << " " << 0 << std::endl;

        /* For each mesh write points */
        fout.setf(std::ios::fixed, std::ios::floatfield);
        fout.setf(std::ios::showpoint);
        fout.precision(6);
        for (int i = 0; i < uut.number_of_convex_hulls(); ++i)
        {
            const auto *const mesh = uut.get_convex_hull(i);
            for (const auto &pt : mesh->points())
            {
                fout << pt.x << " " << pt.y << " " << pt.z << std::endl;
            }
        }

        /* For each mesh write triangles */
        for (int i = 0; i < uut.number_of_convex_hulls(); ++i)
        {
            const auto *const mesh = uut.get_convex_hull(i);
            for (const auto &t : mesh->triangles())
            {
                fout << "3 " << t.x << " " << t.y << " " << t.z << std::endl;
            }
        }

        fout.close();
        return true;
    }

    bool save_vrml2(std::ofstream &fout, const std::vector<point_t> &points, const std::vector<point_ti> &triangles, const float r, const float g, const float b)
    {
        if (!fout.is_open())
        {
            std::cout << "Can't open file" << std::endl;
            return false;
        }

        fout.setf(std::ios::fixed, std::ios::floatfield);
        fout.setf(std::ios::showpoint);
        fout.precision(6);
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
        // fout << "                    specularColor " << material.m_specularColor[0] << " "
        //                                              << material.m_specularColor[1] << " "
        //                                              << material.m_specularColor[2] << std::endl;
        // fout << "                    emissiveColor " << material.m_emissiveColor[0] << " "
        //                                              << material.m_emissiveColor[1] << " "
        //                                              << material.m_emissiveColor[2] << std::endl;
        // fout << "                    shininess " << material.m_shininess << std::endl;
        // fout << "                    transparency " << material.m_transparency << std::endl;
        fout << "                }" << std::endl;
        fout << "            }" << std::endl;
        fout << "            geometry IndexedFaceSet {" << std::endl;
        fout << "                ccw TRUE" << std::endl;
        fout << "                solid TRUE" << std::endl;
        fout << "                convex TRUE" << std::endl;
        if (!points.empty())
        {
            fout << "                coord DEF co Coordinate {" << std::endl;
            fout << "                    point [" << std::endl;
            for (const auto &p : points)
            {
                fout << "                        " << p.x << " "
                                                   << p.y << " "
                                                   << p.z << "," << std::endl;
            }
            fout << "                    ]" << std::endl;
            fout << "                }" << std::endl;
        }

        if (!triangles.empty())
        {
            fout << "                coordIndex [ " << std::endl;
            for (const auto &t : triangles)
            {
                fout << "                        " << t.x << ", "
                                                   << t.y << ", "
                                                   << t.z << ", -1," << std::endl;
            }
            fout << "                ]" << std::endl;
        }
        fout << "            }" << std::endl;
        fout << "        }" << std::endl;
        fout << "    ]" << std::endl;
        fout << "}" << std::endl;
        return true;
    }

    void save_results(const convex_decomposition &uut, const std::string &out_file)
    {
        std::default_random_engine gen;
        std::uniform_real_distribution<float> dist(0.0f, 1.0f);
        std::ofstream fout(out_file.c_str());
        if (fout.is_open())
        {
            for (int i = 0; i < uut.number_of_convex_hulls(); ++i)
            {
                const auto *const mesh = uut.get_convex_hull(i);
                save_vrml2(fout, mesh->points(), mesh->triangles(), dist(gen), dist(gen), dist(gen));
            }
            fout.close();
        }
    }

    void check(const convex_decomposition &uut, const std::string &file)
    {
        /* Sum number of points and triangles */
        int nr_pts = 0;
        int nr_tri = 0;
        for (int i = 0; i < uut.number_of_convex_hulls(); ++i)
        {
            const auto *const mesh = uut.get_convex_hull(i);
            nr_pts += mesh->number_of_points();
            nr_tri += mesh->number_of_triangles();
        }

        /* Save actual results */
        save_results(uut, "test_data/" + file + ".wrl");
        save_off(uut, "test_data/" + file + "_act.off", nr_pts, nr_tri);

        /* Load expected data */
        std::vector<point_t>    points_exp;
        std::vector<point_ti>   triangles_exp;
        BOOST_REQUIRE(load_off("test_data/" + file + "_exp.off", &points_exp, &triangles_exp));

        /* Check points */
        BOOST_REQUIRE(nr_pts == static_cast<int>(points_exp.size()));

        int pt_idx = 0;
        for (int i = 0; i < uut.number_of_convex_hulls(); ++i)
        {
            const auto *const mesh = uut.get_convex_hull(i);
            for (const auto &pt : mesh->points())
            {
                BOOST_CHECK_MESSAGE(std::fabs(magnitude(pt - points_exp[pt_idx])) < result_tolerance, pt << " versus: " << points_exp[pt_idx]);
                pt_idx++;
            }
        }

        /* Check triangles */
        BOOST_REQUIRE(nr_tri == static_cast<int>(triangles_exp.size()));

        int tri_idx = 0;
        for (int i = 0; i < uut.number_of_convex_hulls(); ++i)
        {
            const auto *const mesh = uut.get_convex_hull(i);
            for (const auto &t : mesh->triangles())
            {
                BOOST_CHECK(t == triangles_exp[tri_idx++]);
            }
        }
    }

    convex_decomposition_options    options;
    std::vector<point_t>            points;
    std::vector<point_ti>           triangles;
};
}; /* namespace test */
}; /* namespace raptor_convex_decomposition */
