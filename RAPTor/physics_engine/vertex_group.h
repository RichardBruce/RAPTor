#ifndef __VERTEX_GROUP_H__
#define __VERTEX_GROUP_H__

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
#include "triangle.h"

/* Physics headers */
#include "matrix_3d.h"
#include "inertia_tensor.h"


/* Forward declaration */
class material;

namespace raptor_physics
{
class vertex_group : private boost::noncopyable
{
    public :
        /* CTORs and DTORs */
        vertex_group(const std::vector<point_t> &verts, std::vector<int> *const tris, material *const mat)
            : _verts(new matrix_3d(verts)),
              _tris(tris),
              _edges(build_edges_from_tris()),
              _mat(mat){  };
              
        ~vertex_group()
        {
            delete _verts;
            delete _tris;
            delete [] _edges;
        }

        inertia_tensor *const build_inertia_tensor(const fp_t density) const
        {
            return new inertia_tensor(*_verts, *_tris, density);
        }

        /* Vertex access */
        int     get_number_of_vertices()    const { return _verts->size();      }
        point_t get_vertex(const int i)     const { return _verts->get_row(i);  }

        /* Get the groups bounding box */
        const vertex_group& get_bounds(point_t *const hi, point_t *const lo) const
        {
            _verts->get_extremities(hi, lo);
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
            const fp_t disp_gain = dot_product(d, rel_disp);
            if (disp_gain > 0.0)
            {
                max_support_vertex |= 0x80000000;
            }
            
            return max_support_vertex;
        }

        /* Find support vertex including movement and rotation and penalising for distance in n */
        int find_support_vertex(const point_t &w, const point_t &c, const point_t &p, const point_t &n, fp_t *const val) const
        {
            return raptor_physics::find_support_vertex(*_verts, w, c, p, n, val);
        }

        fp_t find_intersection_time(const vertex_group &vg, const point_t &nb, const point_t &x0, const point_t &x1, const point_t &q0, const point_t &q1, const fp_t r0, const fp_t r1, const fp_t full_t)
        {
            BOOST_LOG_TRIVIAL(trace) << "Verts: " << _verts->size();
            BOOST_LOG_TRIVIAL(trace) << "Tris: " << vg._tris->size();

            /* Check each vertex of this object */
            fp_t frac_t = 1.001;
            for (int i = 0; i < _verts->size(); ++i)
            {
                /* See when it is inside all face of vg */
                const point_t &pa = _verts->get_row(i);
                for (int j = 0; j < static_cast<int>(vg._tris->size()); j += 3)
                {
                    /* Build face normal */
                    const point_t &pb_0 = vg._verts->get_row((*vg._tris)[j]);
                    // const point_t pb_1(vg._verts->get_row((*vg._tris)[j + 1]));
                    // const point_t pb_2(vg._verts->get_row((*vg._tris)[j + 2]));
                    // const point_t nb(cross_product(pb_1 - pb_0, pb_2 - pb_0));

                    /* Find intersection time */
                    const fp_t inter = find_exact_collision_time(pa, pb_0, nb, x0, x1, q0, q1, r0, r1);
                    frac_t = std::min(frac_t, inter);
                    BOOST_LOG_TRIVIAL(trace) << "Determining exact collision time up to: " << (full_t * frac_t);
                }
            }

            return frac_t;
        }

        /* Build triangles for rendering */
        const vertex_group& triangles(primitive_list *p, const quaternion_t &r, const point_t &t) const
        {
            /* Position vertices */
            std::vector<point_t> pos_verts;
            pos_verts.reserve(_verts->size());
            for (int i = 0; i < _verts->size(); ++i)
            {
                pos_verts.push_back(r.rotate(_verts->get_row(i)));
                pos_verts[i] += t;
            }

            /* For each triple of edges build a triangle */
            for (unsigned int i = 0; i < _tris->size(); i += 3)
            {
                p->push_back(new triangle(_mat, 
                    pos_verts[_tris->at(i    )], 
                    pos_verts[_tris->at(i + 1)], 
                    pos_verts[_tris->at(i + 2)]));
            }
            return *this;
        }

        /* Debug */
        const vertex_group& dump() const
        {
            _verts->dump();
            return *this;
        }

    private :
        std::vector<int>* build_edges_from_tris() const
        {
            std::vector<int> *edges = new std::vector<int>[_verts->size()];

            for (unsigned int i = 0; i < _tris->size(); i += 3)
            {
                edges[(*_tris)[i    ]].push_back((*_tris)[i + 1]);
                edges[(*_tris)[i + 1]].push_back((*_tris)[i + 2]);
                edges[(*_tris)[i + 2]].push_back((*_tris)[i    ]);
            }

            return edges;
        }        

        matrix_3d           *const  _verts; /* Vertices making up the group     */
        std::vector<int>    *const  _tris;  /* Triples of triangle edges        */
        std::vector<int>    *const  _edges; /* Per vertex connectivity          */
        material            *const  _mat;   /* Shader for the triangles         */
};


/* Routine to make plane out of triangles */
inline vertex_group* make_plane(material *m, const point_t &bl, const point_t &br, const point_t &tl, const point_t &tr)
{
    /* Vertices of the plane */
    std::vector<point_t> verts(
    {
        point_t(bl.x, bl.y, bl.z),
        point_t(tl.x, tl.y, tl.z),
        point_t(tr.x, tr.y, tr.z),
        point_t(br.x, br.y, br.z)
    });
    
    /* Triples of triangles making up the plane */
    std::vector<int> *tris = new std::vector<int>(
    {
        0, 2, 1, 0, 3, 2
    });

    /* Find the center of the plane and build it */
    return new vertex_group(verts, tris, m);
}


/* Routine to make cubes out of triangles */
inline vertex_group* make_cube(material *m, const point_t &bl, const point_t &tr)
{
    /* Vertices of the cube */
    std::vector<point_t> verts(
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

    /* Triples of triangle vertices making up the cube */
    std::vector<int> *tris = new std::vector<int>(
    {
        0, 2, 1, 0, 3, 2, /* Front face */
        4, 5, 6, 4, 6, 7, /* Back face */
        4, 7, 0, 7, 3, 0, /* Left face */
        1, 2, 6, 1, 6, 5, /* Right face */
        3, 7, 6, 3, 6, 2, /* Top face */
        0, 1, 4, 1, 5, 4 /* Bottom face */
    });

    /* Build the cube */
    return new vertex_group(verts, tris, m);
}
}; /* namespace raptor_physics */

#endif /* #ifndef __VERTEX_GROUP_H__ */
