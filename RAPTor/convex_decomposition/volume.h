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


namespace raptor_convex_decomposition
{
enum class axis_t { x_axis = 0, y_axis = 1, z_axis = 2 };
enum class voxel_value_t { primitive_undefined = 0, primitive_outside_surface = 1, primitive_inside_surface = 2, primitive_on_surface = 3 };


struct plane
{
    /* Default Ctor */
    plane() = default;

    /* Ctor */
    plane(const point_t &n, const float p, const axis_t major_axis) : n(n), p(p), major_axis(major_axis) {  }

    float distance(const point_t &pt) const
    {
        return dot_product(pt, n) - p;
    }

    point_t n;
    float   p;
    axis_t  major_axis;
};

class primitive_set
{
    public :
        /* Ctor */
        primitive_set()
        {
            memset(_d, 0, sizeof(float) * 9);
        }

        /* Virtual Dtor for derived types */
        virtual ~primitive_set() { };

        const convex_mesh & convex_hull()                   const   { return _convex_hull; };
        convex_mesh &       convex_hull()                           { return _convex_hull; };
        float               eigen_value(const axis_t axis)  const   { return _d[static_cast<int>(axis)][static_cast<int>(axis)];  }
        float               compute_preferred_cutting_direction(point_t *const dir);
        
        virtual primitive_set * select_on_surface()                                                                                                                 const = 0;
        virtual void            cut(const plane &p, primitive_set **const pos_part, primitive_set **const neg_part)                                                 const = 0;
        virtual void            intersect(const plane &p, std::vector<point_t> *const pos_pts, std::vector<point_t> *const neg_pts, const int sampling)             const = 0;
        virtual void            compute_cut_volumes(const plane &p, float *const pos_volume, float *const neg_volume)                                               const = 0;
        virtual void            compute_convex_hull(convex_mesh *const mesh, const int sampling = 1, const int cluster_size = 65536)                                const = 0;
        virtual void            compute_bounding_box()                                                                                                              = 0;
        virtual void            compute_principal_axes()                                                                                                            = 0;
        virtual void            align_to_principal_axes()                                                                                                           = 0;
        virtual void            revert_align_to_principal_axes()                                                                                                    = 0;
        virtual void            convert(convex_mesh *const mesh, const voxel_value_t value)                                                                         const = 0;
        virtual float           max_volume_error()                                                                                                                  const = 0;
        virtual float           compute_volume()                                                                                                                    const = 0;
        virtual int             number_of_primitives()                                                                                                              const = 0;
        virtual int             number_of_primitives_on_surface()                                                                                                   const = 0;
        virtual int             number_of_primitives_inside()                                                                                                       const = 0;
        virtual void            compute_axes_aligned_clipping_planes(std::vector<plane> *const planes, const int downsampling)                                      const = 0;
        virtual void            refine_axes_aligned_clipping_planes(std::vector<plane> *const planes, const plane &best, const int downsampling, const int index)   const = 0;


    protected :
        float       _d[3][3];

    private :
        convex_mesh _convex_hull;
};

struct voxel
{
    public :
        voxel(const point_ti &coord, const voxel_value_t vv = voxel_value_t::primitive_undefined) : coord(coord), loc(vv) {  };

        point_ti        coord;
        voxel_value_t   loc;
};

class voxel_set : public primitive_set
{            
    public :
        voxel_set(const std::vector<voxel> &voxels, const point_t &min_bb, const float scale) : 
            _voxels(voxels),
            _min_bb(min_bb),
            _scale(scale),
            _unit_volume(scale * scale * scale),
            _prim_on_surface(0),
            _prim_inside_surface(0)
        {
            std::memset(_q, 0, 9 * sizeof(float));
        }

        voxel_set() : voxel_set(std::vector<voxel>(), point_t(0.0f, 0.0f, 0.0f), 1.0f) {  };

        const point_t &     get_min_bb()                        const { return _min_bb;                         }
        const point_ti &    get_min_bb_voxels()                 const { return _min_bb_voxels;                  }
        const point_ti &    get_max_bb_voxels()                 const { return _max_bb_voxels;                  }
        const point_ti &    get_barycenter()                    const { return _barycenter;                     }
        float               get_scale()                         const { return _scale;                          }
        float               get_unit_volume()                   const { return _unit_volume;                    }
        float               compute_volume()                    const { return _unit_volume * _voxels.size();   }
        float               max_volume_error()                  const { return _unit_volume * _prim_on_surface; }
        int                 number_of_primitives()              const { return _voxels.size();                  }
        int                 number_of_primitives_on_surface()   const { return _prim_on_surface;                }
        int                 number_of_primitives_inside()       const { return _prim_inside_surface;            }
    
        point_t get_point(const voxel &v) const
        {
            return point_t(static_cast<point_t>(v.coord) * _scale) + _min_bb;
        }

        point_t get_point(const point_t &v) const
        {
            return point_t(v * _scale) +  _min_bb;
        }

        voxel_set& set_scale(const float scale)
        {
            _scale          = scale;
            _unit_volume    = _scale * _scale * _scale;
            return *this;
        }

        voxel_set& set_min_bb(const point_t &min_bb)
        {
            _min_bb = min_bb;
            return *this;
        }

        voxel_set& reserve(const int size)
        {
            _voxels.reserve(size);
            return *this;
        }


        std::vector<voxel> &        get_voxels()       { return _voxels; }
        const std::vector<voxel> &  get_voxels() const { return _voxels; }

        voxel_set * select_on_surface()                                                                                                                 const;
        void        get_points(const voxel &v, point_t *const pts)                                                                                      const;
        void        compute_convex_hull(convex_mesh *const mesh, const int sampling = 1, const int cluster_size = 65536)                                const;
        void        cut(const plane &p, primitive_set **const pos_part, primitive_set **const neg_part)                                                 const;
        void        intersect(const plane &p, std::vector<point_t> *const pos_pts, std::vector<point_t> *const neg_pts, const int sampling)             const;
        void        compute_cut_volumes(const plane &p, float *const pos_volume, float *const neg_volume)                                               const;
        void        compute_bounding_box();
        void        convert(convex_mesh *const mesh, const voxel_value_t value)                                                                         const;
        void        compute_principal_axes();
        void        align_to_principal_axes() { };
        void        revert_align_to_principal_axes() { };
        void        compute_axes_aligned_clipping_planes(std::vector<plane> *const planes, const int downsampling)                                      const;
        void        refine_axes_aligned_clipping_planes(std::vector<plane> *const planes, const plane &best, const int downsampling, const int index)   const;
        bool        add(const voxel &v);

    private :
        std::vector<voxel>  _voxels;
        point_t             _min_bb;
        point_ti            _min_bb_voxels;
        point_ti            _max_bb_voxels;
        point_ti            _barycenter;
        float               _scale;
        float               _unit_volume;
        float               _q[3][3];
        int                 _prim_on_surface;
        int                 _prim_inside_surface;
};


struct tetrahedron
{
    public :
        tetrahedron() = default;

        tetrahedron(const point_t &a, const point_t &b, const point_t &c, const point_t &d, const voxel_value_t vv = voxel_value_t::primitive_undefined) : 
            pts{ a, b, c, d},
            loc(vv)
        {  };

        point_t         pts[4];
        voxel_value_t   loc;
};

class tetrahedron_set : public primitive_set
{            
    public :
        tetrahedron_set(const std::vector<tetrahedron> &tetrahedra, const point_t &min_bb, const float scale) : 
            _tetrahedra(tetrahedra),
            _min_bb(min_bb),
            _scale(scale),
            _prim_on_surface(0),
            _prim_inside_surface(0)
        {
            memset(_q, 0, sizeof(float) * 9);
        }

        tetrahedron_set() : tetrahedron_set(std::vector<tetrahedron>(), point_t(0.0f, 0.0f, 0.0f), 1.0f) {  }

        const point_t & get_min_bb()                        const { return _min_bb;                 }
        const point_t & get_max_bb()                        const { return _max_bb;                 }
        const point_t & get_barycenter()                    const { return _barycenter;             }
        float           get_scale()                         const { return _scale;                  }
        int             number_of_primitives()              const { return _tetrahedra.size();      }
        int             number_of_primitives_on_surface()   const { return _prim_on_surface;        }
        int             number_of_primitives_inside()       const { return _prim_inside_surface;    }

        tetrahedron_set& set_scale(const float scale)
        {
            _scale = scale;
            return *this;
        }

        tetrahedron_set& reserve(const int size)
        {
            _tetrahedra.reserve(size);
            return *this;
        }

        std::vector<tetrahedron> &          get_tetrahedra()       { return _tetrahedra; }
        const std::vector<tetrahedron> &    get_tetrahedra() const { return _tetrahedra; }


        tetrahedron_set *   select_on_surface()                                                                                                                 const;
        float               compute_volume()                                                                                                                    const;
        float               max_volume_error()                                                                                                                  const;
        void                compute_convex_hull(convex_mesh *const mesh, const int sampling = 1, const int cluster_size = 65536)                                const;
        void                compute_principal_axes();
        void                align_to_principal_axes();
        void                revert_align_to_principal_axes();
        void                cut(const plane &p, primitive_set **const pos_part, primitive_set **const neg_part)                                                 const;
        void                intersect(const plane &p, std::vector<point_t> *const pos_pts, std::vector<point_t> *const neg_pts, const int sampling)             const;
        void                compute_cut_volumes(const plane &p, float *const pos_volume, float *const neg_volume)                                               const;
        void                compute_bounding_box();
        void                convert(convex_mesh *const mesh, const voxel_value_t value)                                                                         const;
        void                compute_axes_aligned_clipping_planes(std::vector<plane> *const planes, const int downsampling)                                      const;
        void                refine_axes_aligned_clipping_planes(std::vector<plane> *const planes, const plane &best, const int downsampling, const int index)   const;
        bool                add(tetrahedron t);

    private :
        void    add_clipped_tetrahedra(const point_t (&pts) [10], const int size);

        std::vector<tetrahedron>    _tetrahedra;
        point_t                     _min_bb;
        point_t                     _max_bb;
        point_t                     _barycenter;
        float                       _scale;
        float                       _q[3][3];
        int                         _prim_on_surface;
        int                         _prim_inside_surface;
};


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
