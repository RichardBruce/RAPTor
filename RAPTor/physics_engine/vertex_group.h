#pragma once

/* Standard headers */
#include <vector>

/* Boost headers */
#include "boost/noncopyable.hpp"

/* Common headers */
#include "common.h"
#include "physics_common.h"
#include "logging.h"
#include "point_t.h"
#include "quaternion_t.h"

/* Ray tracer headers */
#include "primitive_store.h"
#include "triangle.h"

/* Physics headers */
#include "inertia_tensor.h"
#include "polygon.h"


/* Forward declaration */
namespace raptor_raytracer
{
class material;
}; /* namespace raptor_raytracer */

namespace raptor_physics
{
class vertex_group : private boost::noncopyable
{
    public :
        /* CTORs and DTORs */
        vertex_group(const std::vector<point_t> *const verts, const std::vector<polygon> &polys, raptor_raytracer::material *const mat) : 
            _verts(verts),
            _polys(polys),
            _tris(build_tris()),
            _edges(build_edges_from_tris()),
            _mat(mat)
        {  };
              
        ~vertex_group()
        {
            delete _verts;
            delete [] _edges;
        }

        inertia_tensor *const build_inertia_tensor(const float density) const
        {
            return new inertia_tensor(*_verts, _tris, density);
        }

        /* Vertex access */
        int     get_number_of_vertices()    const { return _verts->size();  }
        point_t get_vertex(const int i)     const { return (*_verts)[i];    }
        
        point_t get_vertex(const quaternion_t &r, const point_t &t, const int i) const
        {
            return r.rotate((*_verts)[i]) + t;
        }

        /* Get the groups bounding box */
        const vertex_group& get_bounds(point_t *const hi, point_t *const lo) const
        {
            /* Initialise to the first data point */
            *hi = (*_verts)[0];
            *lo = (*_verts)[0];
            
            /* Loop finding extremities */
            for (int i = 1; i < static_cast<int>(_verts->size()); ++i)
            {
                *hi = max(*hi, (*_verts)[i]);
                *lo = min(*lo, (*_verts)[i]);
            }

            return *this;
        }

        /* TODO -- These could all potentially be sped up with some acceleration structure */
        /* Find the most extreme vertex in direction d */
        int find_support_vertex(const point_t &d) const
        {
            /* Find a support vertex as normal */
            return raptor_physics::find_support_vertex(*_verts, d);
        }

        /* Find the most extreme vertex in direction d taking the movement of the objects into account */
        int find_support_vertex(const point_t &d, const point_t &rel_disp) const
        {
            /* Find a support vertex as normal */
            int max_support_vertex = find_support_vertex(d);
            
            /* Adjust its position for the relative displacement */
            const float disp_gain = dot_product(d, rel_disp);
            if (disp_gain > 0.0f)
            {
                max_support_vertex |= 0x80000000;
            }
            
            return max_support_vertex;
        }

        /* Find support vertex including movement and rotation and penalising for distance in n */
        int find_support_vertex(const point_t &w, const point_t &c, const point_t &p, const point_t &n, float *const val) const
        {
            return raptor_physics::find_support_vertex(*_verts, w, c, p, n, val);
        }

        float find_intersection_time(const vertex_group &vg, const point_t &nb, const point_t &x0, const point_t &x1, const point_t &q0, const point_t &q1, const float r0, const float r1, const float full_t)
        {
            BOOST_LOG_TRIVIAL(trace) << "Verts: " << _verts->size();
            BOOST_LOG_TRIVIAL(trace) << "Tris: " << vg._tris.size();

            /* Check each vertex of this object */
            float frac_t = 1.001f;
            for (int i = 0; i < static_cast<int>(_verts->size()); ++i)
            {
                /* See when it is inside all face of vg */
                const point_t &pa = (*_verts)[i];
                for (int j = 0; j < static_cast<int>(vg._tris.size()); j += 3)
                {
                    /* Build face normal */
                    const point_t &pb_0 = (*vg._verts)[vg._tris[j]];
                    // const point_t nb(cross_product(pb_1 - pb_0, pb_2 - pb_0));

                    /* Find intersection time */
                    const float inter = find_exact_collision_time(pa, pb_0, nb, x0, x1, q0, q1, r0, r1);
                    frac_t = std::min(frac_t, inter);
                    BOOST_LOG_TRIVIAL(trace) << "Determining exact collision time up to: " << (full_t * frac_t);
                }
            }

            return frac_t;
        }

        /* Find the polygon most matched to the points and normal */
        const polygon * find_polygon(const int *const pt_begin, const int *const pt_end, const point_t &n) const
        {
            float best_dir = -0.5f; /* Back face culling */
            const polygon *best_poly = nullptr;
            for (const auto &p : _polys)
            {
                /* Find a polygon that contains all the points */
                /* TODO - maybe we need point to polygon look up */
                if (p.contains_vertices(pt_begin, pt_end))
                {
                    /* And has a normal most matching the one give */
                    const float dir = dot_product(p.normal(), n);
                    // BOOST_LOG_TRIVIAL(trace) << "Found contain with normal: " << p.normal() << " versus normal: " << n;
                    if (dir > best_dir)
                    {
                        best_dir    = dir;
                        best_poly   = &p;
                    }
                }
            }

            return best_poly;
        }

        /* Build triangles for rendering */
        const vertex_group& triangles(raptor_raytracer::primitive_store *p, const quaternion_t &r, const point_t &t) const
        {
            /* Position vertices */
            std::vector<point_t> pos_verts;
            pos_verts.reserve(_verts->size());
            for (int i = 0; i < static_cast<int>(_verts->size()); ++i)
            {
                pos_verts.push_back(get_vertex(r, t, i));
            }

            /* For each triple of edges build a triangle */
            for (unsigned int i = 0; i < _tris.size(); i += 3)
            {
                p->emplace_back(_mat, pos_verts[_tris[i]], pos_verts[_tris[i + 1]], pos_verts[_tris[i + 2]]);
            }
            return *this;
        }

        /* Debug */
        const vertex_group& dump() const
        {
            for (int i = 0; i < static_cast<int>(_verts->size()); ++i)
            {
                std::cout << "Point: " << i << " " << (*_verts)[i] << std::endl;
            }
            return *this;
        }

    private :
        std::vector<int> build_tris() const
        {
            std::vector<int> ret;
            for (const auto &p : _polys)
            {
                p.to_triangles(&ret);
            }

            return ret;
        }

        std::vector<int>* build_edges_from_tris() const
        {
            auto edges = new std::vector<int>[_verts->size()];
            for (unsigned int i = 0; i < _tris.size(); i += 3)
            {
                edges[_tris[i    ]].push_back(_tris[i + 1]);
                edges[_tris[i + 1]].push_back(_tris[i + 2]);
                edges[_tris[i + 2]].push_back(_tris[i    ]);
            }

            return edges;
        }        

        const std::vector<point_t> *const   _verts; /* Vertices making up the group     */
        const std::vector<polygon>          _polys; /* Face polygons                    */
        const std::vector<int>              _tris;  /* Triples of triangle edges        */
        const std::vector<int>      *const  _edges; /* Per vertex connectivity          */
        raptor_raytracer::material  *const  _mat;   /* Shader for the triangles         */
};


/* Routine to make plane out of triangles */
inline vertex_group* make_plane(raptor_raytracer::material *m, const point_t &bl, const point_t &br, const point_t &tl, const point_t &tr)
{
    /* Vertices of the plane */
    auto verts = new std::vector<point_t>(
    {
        bl, tl, tr, br
    });
    
    /* Polygon making up the plane */
    std::vector<polygon> polys(
    {
        polygon(verts, { 0, 1, 2, 3 })
    });

    /* Find the center of the plane and build it */
    return new vertex_group(verts, polys, m);
}


/* Routine to make cubes out of triangles */
inline vertex_group* make_cube(raptor_raytracer::material *m, const point_t &bl, const point_t &tr)
{
    /* Vertices of the cube */
    auto verts = new std::vector<point_t>(
    {
        point_t(bl.x, bl.y, bl.z),
        point_t(tr.x, bl.y, bl.z),
        point_t(tr.x, tr.y, bl.z),
        point_t(bl.x, tr.y, bl.z),
        point_t(bl.x, bl.y, tr.z),
        point_t(tr.x, bl.y, tr.z),
        point_t(tr.x, tr.y, tr.z),
        point_t(bl.x, tr.y, tr.z)
    });

    /* Polygons making up the cube */
    std::vector<polygon> polys(
    {
        polygon(verts, { 3, 2, 1, 0 }), /* Front face  */
        polygon(verts, { 4, 5, 6, 7 }), /* Back face   */
        polygon(verts, { 0, 4, 7, 3 }), /* Left face   */
        polygon(verts, { 1, 2, 6, 5 }), /* Right face  */
        polygon(verts, { 2, 3, 7, 6 }), /* Top face    */
        polygon(verts, { 0, 1, 5, 4 })  /* Bottom face */
    });

    /* Build the cube */
    return new vertex_group(verts, polys, m);
}
}; /* namespace raptor_physics */