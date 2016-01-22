#pragma once

/* Standard headers */
#include <cstring>
#include <limits>
#include <vector>

/* Common headers */
#include "point_t.h"

/* Convex decomposition headers */
#include "convex_mesh.h"
#include "discretisation_type.h"
#include "primitive_set.h"
#include "tetrahedron_set.h"
#include "voxel_set.h"
#include "voxel_value.h"


namespace raptor_convex_decomposition
{
bool triangle_box_overlap(const point_t &boxcenter, const point_t &boxhalfsize, const point_t &triver0, const point_t &triver1, const point_t &triver2);

inline point_t compute_aligned_point(const point_t &pt, const point_t &barycenter, const float (& rot)[3][3])
{
    point_t ret;
    point_t diff(pt - barycenter);
    ret.x = (rot[0][0] * diff.x) + (rot[1][0] * diff.y) + (rot[2][0] * diff.z);
    ret.y = (rot[0][1] * diff.x) + (rot[1][1] * diff.y) + (rot[2][1] * diff.z);
    ret.z = (rot[0][2] * diff.x) + (rot[1][2] * diff.y) + (rot[2][2] * diff.z);

    return ret;
}

class volume
{            
    public :
        volume(const std::vector<point_t> &points, const std::vector<point_ti> &triangles, const int dim, const point_t &barycenter, const float (& rot)[3][3]) :
            _scale(1.0f),
            _dim{ 0, 0, 0 },
            _prim_on_surface(0),
            _prim_inside_surface(0),
            _prim_outside_surface(0)
        {
            if (points.empty())
            {
                return;
            }

            /* Compute bounds */
            compute_bounding_box(points, barycenter, rot);

            /* Divide into regions proportional to the dim */
            float r;
            const point_t d(_max_bb - _min_bb);
            if ((d.x > d.y) && (d.x > d.z))
            {
                r = d.x;
                _dim[0] = dim;
                _dim[1] = static_cast<int>(dim * (d.y / d.x)) + 2;
                _dim[2] = static_cast<int>(dim * (d.z / d.x)) + 2;
            }
            else if ((d.y > d.x) && (d.y > d.z))
            {
                r = d.y;
                _dim[0] = static_cast<int>(dim * (d.x / d.y)) + 2;
                _dim[1] = dim;
                _dim[2] = static_cast<int>(dim * (d.z / d.y)) + 2;
            }
            else
            {
                r = d.z;
                _dim[0] = static_cast<int>(dim * (d.x / d.z)) + 2;
                _dim[1] = static_cast<int>(dim * (d.y / d.z)) + 2;
                _dim[2] = dim;
            }

            /* Set scale and number of voxels based on dim */
            _scale = r / (dim - 1);
            const float scale_inv = (dim - 1) / r;
            _loc.resize((_dim[0] * _dim[1] * _dim[2]), voxel_value_t::primitive_undefined);

            point_t p[3];
            int i0 = -1, j0 = -1, k0 = -1;
            int i1 = -1, j1 = -1, k1 = -1;
            const point_t boxhalfsize(0.5f, 0.5f, 0.5f);
            for (int ti = 0; ti < static_cast<int>(triangles.size()); ++ti)
            {
                for (int c = 0; c < 3; ++c)
                {
                    const point_t pt(compute_aligned_point(points[triangles[ti][c]], barycenter, rot));
                    p[c] = (pt - _min_bb) * scale_inv;
                    const int i = static_cast<int>(p[c].x + 0.5f);
                    const int j = static_cast<int>(p[c].y + 0.5f);
                    const int k = static_cast<int>(p[c].z + 0.5f);
                    assert((i < _dim[0]) && (i >= 0) && (j < _dim[1]) && (j >= 0) && (k < _dim[2]) && (k >= 0));

                    if (c == 0)
                    {
                        i0 = i1 = i;
                        j0 = j1 = j;
                        k0 = k1 = k;
                    }
                    else
                    {
                        i0 = std::min(i0, i);
                        j0 = std::min(j0, j);
                        k0 = std::min(k0, k);
                        i1 = std::max(i1, i);
                        j1 = std::max(j1, j);
                        k1 = std::max(k1, k);
                    }
                }

                if (i0 > 0) --i0;
                if (j0 > 0) --j0;
                if (k0 > 0) --k0;
                if (i1 < _dim[0]) ++i1;
                if (j1 < _dim[1]) ++j1;
                if (k1 < _dim[2]) ++k1;

                point_t boxcenter;
                for (int i = i0; i < i1; ++i)
                {
                    boxcenter.x = static_cast<float>(i);
                    for (int j = j0; j < j1; ++j)
                    {
                        boxcenter.y = static_cast<float>(j);
                        for (int k = k0; k < k1; ++k)
                        {
                            boxcenter.z = static_cast<float>(k);
                            voxel_value_t &value = get_voxel(i, j, k);
                            if ((value == voxel_value_t::primitive_undefined) && triangle_box_overlap(boxcenter, boxhalfsize, p[0], p[1], p[2]))
                            {
                                value = voxel_value_t::primitive_on_surface;
                                ++_prim_on_surface;
                            }
                        }
                    }
                }
            }

            /* Start on the faces and fill in voxels as being outside if not already set */
            fill_outside_surface(0,           0,           0,           _dim[0], _dim[1], 1);
            fill_outside_surface(0,           0,           _dim[2] - 1, _dim[0], _dim[1], _dim[2]);
            fill_outside_surface(0,           0,           0,           _dim[0], 1,       _dim[2]);
            fill_outside_surface(0,           _dim[1] - 1, 0,           _dim[0], _dim[1], _dim[2]);
            fill_outside_surface(0,           0,           0,           1,       _dim[1], _dim[2]);
            fill_outside_surface(_dim[0] - 1, 0,           0,           _dim[0], _dim[1], _dim[2]);

            /* Fill remaining points as inside */
            fill_inside_surface();
        }

        voxel_value_t& get_voxel(const int i, const int j, const int k)
        {
            assert((i < _dim[0]) || (i >= 0));
            assert((j < _dim[0]) || (j >= 0));
            assert((k < _dim[0]) || (k >= 0));
            return _loc[i + (j * _dim[0]) + (k * _dim[0] * _dim[1])];
        }

        const voxel_value_t& get_voxel(const int i, const int j, const int k) const
        {
            assert( i < _dim[0] || i >= 0);
            assert( j < _dim[0] || j >= 0);
            assert( k < _dim[0] || k >= 0);
            return _loc[i + (j * _dim[0]) + (k * _dim[0] * _dim[1])];
        }

        primitive_set * convert(const discretisation_type_t mode) const
        {
            if (mode == discretisation_type_t::voxel)
            {
                return convert_to_voxel_set();
            }
            else
            {
                return convert_to_tetrahedron_set();
            }
        }

        convex_mesh *   convert_to_convex_mesh(const voxel_value_t value)   const;
        void            compute_principal_axes(float (&rot)[3][3])          const;

        int     number_of_primitives_on_surface()   const { return _prim_on_surface;        }
        int     number_of_primitives_inside()       const { return _prim_inside_surface;    }
        int     number_of_primitives_outside()      const { return _prim_outside_surface;   }

    private :
        voxel_set *         convert_to_voxel_set()          const;
        tetrahedron_set *   convert_to_tetrahedron_set()    const;

        void fill_inside_surface();
        void fill_outside_surface(const int i0, const int j0, const int k0, const int i1, const int j1, const int k1);

        void compute_bounding_box(const std::vector<point_t> &points, const point_t &barycenter, const float (& rot)[3][3])
        {
            point_t pt(compute_aligned_point(points[0], barycenter, rot));
            _max_bb = pt;
            _min_bb = pt;
            for (int v = 1; v < static_cast<int>(points.size()); ++v)
            {
                pt = compute_aligned_point(points[v], barycenter, rot);
                _min_bb = min(_min_bb, pt);
                _max_bb = max(_max_bb, pt);
            }
        }

        std::vector<voxel_value_t>  _loc;
        point_t                     _min_bb;
        point_t                     _max_bb;
        float                       _scale;
        int                         _dim[3];
        int                         _prim_on_surface;
        int                         _prim_inside_surface;
        int                         _prim_outside_surface;
};
} /* namespace raptor_convex_decomposition */
