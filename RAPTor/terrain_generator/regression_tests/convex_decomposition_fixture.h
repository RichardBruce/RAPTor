#ifndef __CONVEX_DECOMPOSITION_FIXTURE_H__
#define __CONVEX_DECOMPOSITION_FIXTURE_H__

/* Boost headers */
#include "boost/test/unit_test.hpp"

/* Common headers */
#include "point_t.h"
#include "logging.h"

/* Raytracer headers */
#include "parser_common.h"

/* Terrain generator headers */
#include "convex_decomposition.h"


using namespace raptor_terrain;

const fp_t result_tolerance = 0.005;
const std::string test_data_location = "test_data/convex_decomposition/";

void CallBack(const char *const msg, double progress, double concavity, size_t nVertices)
{
    std::cout << msg;
}


bool save_wrl(std::ofstream &fout, const std::vector<Vec3<Real>> & points, const std::vector<Vec3<long>> & triangles,
    const Material & material, const Vec3<Real> *colours = nullptr)
{
    if (fout.is_open()) 
    {
        size_t nV = points.size();
        size_t nT = triangles.size();            
        fout <<"#VRML V2.0 utf8" << std::endl;            
        fout <<"" << std::endl;
        fout <<"# Vertices: " << nV << std::endl;        
        fout <<"# Triangles: " << nT << std::endl;        
        fout <<"" << std::endl;
        fout <<"Group {" << std::endl;
        fout <<"    children [" << std::endl;
        fout <<"        Shape {" << std::endl;
        fout <<"            appearance Appearance {" << std::endl;
        fout <<"                material Material {" << std::endl;
        fout <<"                    diffuseColor "      << material.m_diffuseColor.X()      << " " 
                                                        << material.m_diffuseColor.Y()      << " "
                                                        << material.m_diffuseColor.Z()      << std::endl;  
        fout <<"                    ambientIntensity "  << material.m_ambientIntensity      << std::endl;
        fout <<"                    specularColor "     << material.m_specularColor.X()     << " " 
                                                        << material.m_specularColor.Y()     << " "
                                                        << material.m_specularColor.Z()     << std::endl; 
        fout <<"                    emissiveColor "     << material.m_emissiveColor.X()     << " " 
                                                        << material.m_emissiveColor.Y()     << " "
                                                        << material.m_emissiveColor.Z()     << std::endl; 
        fout <<"                    shininess "         << material.m_shininess             << std::endl;
        fout <<"                    transparency "      << material.m_transparency          << std::endl;
        fout <<"                }" << std::endl;
        fout <<"            }" << std::endl;
        fout <<"            geometry IndexedFaceSet {" << std::endl;
        fout <<"                ccw TRUE" << std::endl;
        fout <<"                solid TRUE" << std::endl;
        fout <<"                convex TRUE" << std::endl;
        if (colours && nT>0)
        {
            fout <<"                colorPerVertex FALSE" << std::endl;
            fout <<"                color Color {" << std::endl;
            fout <<"                    color [" << std::endl;
            for(size_t c = 0; c < nT; c++)
            {
                fout <<"                        " << colours[c].X() << " " 
                                                  << colours[c].Y() << " " 
                                                  << colours[c].Z() << "," << std::endl;
            }
            fout <<"                    ]" << std::endl;
            fout <<"                }" << std::endl;
                    }
        if (nV > 0) 
        {
            fout <<"                coord DEF co Coordinate {" << std::endl;
            fout <<"                    point [" << std::endl;
            for(size_t v = 0; v < nV; v++)
            {
                fout <<"                        " << points[v].X() << " " 
                                                  << points[v].Y() << " " 
                                                  << points[v].Z() << "," << std::endl;
            }
            fout <<"                    ]" << std::endl;
            fout <<"                }" << std::endl;
        }
        if (nT > 0) 
        {
            fout <<"                coordIndex [ " << std::endl;
            for(size_t f = 0; f < nT; f++)
            {
                fout <<"                        " << triangles[f].X() << ", " 
                                                  << triangles[f].Y() << ", "                                                  
                                                  << triangles[f].Z() << ", -1," << std::endl;
            }
            fout <<"                ]" << std::endl;
        }
        fout <<"            }" << std::endl;
        fout <<"        }" << std::endl;
        fout <<"    ]" << std::endl;
        fout <<"}" << std::endl;    
        return true;
    }
    return false;
}


bool save_wrl(const std::string &file_name, const std::vector<Vec3<Real>> & points, const std::vector<Vec3<long>> & triangles, const Vec3<Real> * colours = nullptr)
{
    std::ofstream fout(file_name.c_str());
    if (fout.is_open()) 
    {
        const Material material;
        
        if (save_wrl(fout, points, triangles, material, colours))
        {
            fout.close();
            return true;
        }
        return false;
    }
    return false;
}


bool save_off(const std::string &file_name, size_t nV, size_t nT, const Vec3<Real> *const points, const Vec3<long> *const triangles)
{
    std::ofstream fout(file_name.c_str());
    if (fout.is_open()) 
    {           
        fout <<"OFF" << std::endl;            
        fout << nV << " " << nT << " " << 0<< std::endl;        
        for(size_t v = 0; v < nV; v++)
        {
            fout << points[v].X() << " " 
                 << points[v].Y() << " " 
                 << points[v].Z() << std::endl;
        }
        for(size_t f = 0; f < nT; f++)
        {
            fout <<"3 " << triangles[f].X() << " " 
                        << triangles[f].Y() << " "                                                  
                        << triangles[f].Z() << std::endl;
        }
        fout.close();
        return true;
    }
    return false;
}


bool save_off(const std::string &file_name, const std::vector<Vec3<Real>> & points, const std::vector<Vec3<long>> & triangles)
{
    return save_off(file_name, points.size(), triangles.size(), &points[0], &triangles[0]);
}


bool load_off(const std::string &file_name, std::vector<Vec3<Real>> & points, std::vector<Vec3<long>> & triangles, bool invert) 
{    
    FILE * fid = fopen(file_name.c_str(), "r");
    if (fid) 
    {
        const std::string strOFF("OFF");
        char temp[1024];
        fscanf(fid, "%s", temp);
        if (std::string(temp) != strOFF)
        {
            printf( "Loading error: format not recognized \n");
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
            points.resize(nv);
            triangles.resize(nf);
            Vec3<Real> coord;
            float x = 0;
            float y = 0;
            float z = 0;
            for (long p = 0; p < nv ; p++) 
            {
                fscanf(fid, "%f", &x);
                fscanf(fid, "%f", &y);
                fscanf(fid, "%f", &z);
                points[p].X() = x;
                points[p].Y() = y;
                points[p].Z() = z;
            }        
            int i = 0;
            int j = 0;
            int k = 0;
            int s = 0;
            for (long t = 0; t < nf ; ++t)
            {
                fscanf(fid, "%i", &s);
                if (s == 3)
                {
                    fscanf(fid, "%i", &i);
                    fscanf(fid, "%i", &j);
                    fscanf(fid, "%i", &k);
                    triangles[t].X() = i;
                    if (invert)
                    {
                        triangles[t].Y() = k;
                        triangles[t].Z() = j;
                    }
                    else
                    {
                        triangles[t].Y() = j;
                        triangles[t].Z() = k;
                    }
                }
                else            // Fix me: support only triangular meshes
                {
                    for(long h = 0; h < s; ++h) fscanf(fid, "%i", &s);
                }
            }
            fclose(fid);
        }
    }
    else 
    {
        printf( "Loading error: file not found \n");
        return false;
    }
    return true;
}


void load_wrl(const std::string &input_file, std::vector<std::pair<unsigned int, unsigned int>> *const sizes, std::vector<Vec3<Real>> *const points, std::vector<Vec3<long>> *const triangles)
{    
    std::ifstream vrml_file(input_file.c_str());
    assert(vrml_file.is_open());

    /* Find the size of the file */
    vrml_file.seekg(0, ios::end);
    size_t len = vrml_file.tellg();
    vrml_file.seekg(0, ios::beg);
    
    /* Read the whole file into a buffer */
    char *buffer = new char [len];
    vrml_file.read(buffer, len);
    const char *at = &buffer[0];

    /* Until we find the end of the file */
    while (true)
    {
        /* Find some points */
        do
        {
            find_next_line(&at);
            skip_white_space(&at);
        } while ((at < &buffer[len - 8]) && (strncmp(at, "point [", 7) != 0));

        /* Check if we're done */
        if (at >= &buffer[len - 8])
        {
            break;
        }

        /* Read the points */
        unsigned int nr_pts = 0;
        find_next_line(&at);
        while (at[0] != ']')
        {
            --at;
            const fp_t x = get_next_float(&at);
            const fp_t y = get_next_float(&at);
            const fp_t z = get_next_float(&at);
            points->emplace_back(x, y, z);

            ++nr_pts;

            find_next_line(&at);
            skip_white_space(&at);
        }

        /* Find triangles */
        do
        {
            find_next_line(&at);
            skip_white_space(&at);
        } while (strncmp(at, "coordIndex [", 12) != 0);

        /* Read the triangles */
        unsigned int nr_tris = 0;
        find_next_line(&at);
        while (at[0] != ']')
        {
            --at;
            unsigned int va = get_next_unsigned(&at);
            unsigned int vb = get_next_unsigned(&at);
            unsigned int vc = get_next_unsigned(&at);
            assert(get_next_unsigned(&at) == (unsigned int)-1);
            triangles->emplace_back(va, vb, vc);

            ++nr_tris;

            find_next_line(&at);
            skip_white_space(&at);
        }

        /* Save the size of the groups */
        sizes->emplace_back(nr_pts, nr_tris);
    }

    /* Clean up */
    delete [] buffer;
    vrml_file.close();
}

struct convex_decomposition_fixture
{
    convex_decomposition_fixture()
    :  hm(createHeapManager(65536*(1000))),
       cd(CreateHACD(hm))
    { }

    ~convex_decomposition_fixture()
    {
        DestroyHACD(cd);
        releaseHeapManager(hm);
    }

    convex_decomposition_fixture& load_data(const std::string &file, const double concavity, const double ccConnectDist, 
        const size_t targetNTrianglesDecimatedMesh, const long nClusters, const bool invert, const bool addExtraDistPoints, const bool addFacesPoints)
    {
        /* Load from off */
        std::string off_file = test_data_location + file + ".off";
        std::vector< Vec3<Real> > points;
        std::vector< Vec3<long> > triangles;
        load_off(off_file, points, triangles, invert);

        Vec3<Real> *pts = new Vec3<Real>[points.size()];
        memcpy(pts, points.data(), points.size() * sizeof(Vec3<Real>));

        Vec3<long> *tri = new Vec3<long>[triangles.size()];
        memcpy(tri, triangles.data(), triangles.size() * sizeof(Vec3<long>));
        
        /* Save wrl equivalant */
        std::string wrl_file = test_data_location + file + ".wrl";
        save_wrl(wrl_file, points, triangles);

        /* Set convex decomposition parameters */
        cd->SetPoints(pts);
        cd->SetNPoints(points.size());
        cd->SetTriangles(tri);
        cd->SetNTriangles(triangles.size());
        cd->SetCompacityWeight(0.0001);
        cd->SetVolumeWeight(0.0);
        cd->SetConnectDist(ccConnectDist);               // if two connected components are seperated by distance < ccConnectDist
                                                            // then create a virtual edge between them so the can be merged during 
                                                            // the simplification process
              
        cd->SetNClusters(nClusters);                     // minimum number of clusters
        cd->SetNVerticesPerCH(100);                      // max of 100 vertices per convex-hull
        cd->SetConcavity(concavity);                     // maximum concavity
        cd->SetSmallClusterThreshold(0.25);                 // threshold to detect small clusters
        cd->SetNTargetTrianglesDecimatedMesh(targetNTrianglesDecimatedMesh); // # triangles in the decimated mesh
        cd->SetCallBack(&CallBack);
        cd->SetAddExtraDistPoints(addExtraDistPoints);   
        cd->SetAddFacesPoints(addFacesPoints);

        return *this;
    }


    convex_decomposition_fixture& check(const std::string &file)
    {
        /* Save results */
        std::string outfile_name = test_data_location + file + "_act.wrl";
        cd->Save(outfile_name.c_str(), false);

        const Vec3<Real> * const decimatedPoints = cd->GetDecimatedPoints();
        const Vec3<long> * const decimatedTriangles    = cd->GetDecimatedTriangles();
        std::string outOFFfile_nameDecimated = test_data_location + file + "_dec_act.off";
        save_off(outOFFfile_nameDecimated, cd->GetNDecimatedPoints(), cd->GetNDecimatedTriangles(), decimatedPoints, decimatedTriangles);

        /* Load expected results */
        std::vector<Vec3<long>> exp_tris;
        std::vector<Vec3<Real>> exp_points;
        std::vector<std::pair<unsigned int, unsigned int>> hull_sizes;
        load_wrl(test_data_location + file + "_exp.wrl", &hull_sizes, &exp_points, &exp_tris);

        /* Check results */
        int pts_offset = 0;
        int tri_offset = 0;
        BOOST_CHECK(cd->GetNClusters() == hull_sizes.size());
        for(size_t i = 0; i < cd->GetNClusters(); ++i)
        {
            /* Get data for hull i */
            std::unique_ptr<Vec3<Real> []> pointsCH(new Vec3<Real>[cd->GetNPointsCH(i)]);
            std::unique_ptr<Vec3<long> []> trianglesCH(new Vec3<long>[cd->GetNTrianglesCH(i)]);
            cd->GetCH(i, pointsCH.get(), trianglesCH.get());
            
            /* Check size */
            BOOST_CHECK(cd->GetNPointsCH(i) == hull_sizes[i].first);
            BOOST_CHECK(cd->GetNTrianglesCH(i) == hull_sizes[i].second);

            /* Check points */
            for(size_t j = 0; j < hull_sizes[i].first; ++j)
            {
                const point_t act(pointsCH[j].X(), pointsCH[j].Y(), pointsCH[j].Z());
                const point_t exp(exp_points[pts_offset + j].X(), exp_points[pts_offset + j].Y(), exp_points[pts_offset + j].Z());
                BOOST_ASSERT(fabs(magnitude(act - exp)) < result_tolerance);
            }

            /* Check triangles */
            for(size_t j = 0; j < hull_sizes[i].second; ++j)
            {
                BOOST_ASSERT(trianglesCH[j].X() == exp_tris[tri_offset + j].X());
                BOOST_ASSERT(trianglesCH[j].Y() == exp_tris[tri_offset + j].Y());
                BOOST_ASSERT(trianglesCH[j].Z() == exp_tris[tri_offset + j].Z());
            }

            /* Increase offsets to next hull */
            pts_offset += hull_sizes[i].first;
            tri_offset += hull_sizes[i].second;
        }
        
        /* Check decimated results */
        std::vector< Vec3<long> > dec_exp_tris;
        std::vector< Vec3<Real> > dec_exp_points;
        load_off(test_data_location + file + "_dec_exp.off", dec_exp_points, dec_exp_tris, false);

        /* Check size */
        BOOST_CHECK(cd->GetNDecimatedPoints() == dec_exp_points.size());
        BOOST_CHECK(cd->GetNDecimatedTriangles() == dec_exp_tris.size());

        /* Check points */
        for(size_t i = 0; i < dec_exp_points.size(); ++i)
        {
            BOOST_CHECK(fabs(decimatedPoints[i].X() - dec_exp_points[i].X()) < result_tolerance);
            BOOST_CHECK(fabs(decimatedPoints[i].Y() - dec_exp_points[i].Y()) < result_tolerance);
            BOOST_CHECK(fabs(decimatedPoints[i].Z() - dec_exp_points[i].Z()) < result_tolerance);
        }

        /* Check triangles */
        for(size_t i = 0; i < dec_exp_tris.size(); ++i)
        {
            BOOST_CHECK(decimatedTriangles[i].X() == dec_exp_tris[i].X());
            BOOST_CHECK(decimatedTriangles[i].Y() == dec_exp_tris[i].Y());
            BOOST_CHECK(decimatedTriangles[i].Z() == dec_exp_tris[i].Z());
        }

        return *this;
    }

    HeapManager * hm;
    HACD *        cd;
};
#endif /* #ifndef __CONVEX_DECOMPOSITION_FIXTURE_H__ */
