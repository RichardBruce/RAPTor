#pragma once

/* Standard headers */
#include <cstring>
#include <limits>
#include <vector>

/* Common headers */
#include "point_t.h"

/* Convex decomposition headers */
#include "convex_mesh.h"
#include "primitive_set.h"
#include "plane.h"
#include "voxel_value.h"


namespace raptor_convex_decomposition
{
struct voxel
{
    public :
        voxel(const point_ti<> &coord, const voxel_value_t vv = voxel_value_t::primitive_undefined) : coord(coord), loc(vv) {  };

        point_ti<>      coord;
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
        const point_ti<> &  get_min_bb_voxels()                 const { return _min_bb_voxels;                  }
        const point_ti<> &  get_max_bb_voxels()                 const { return _max_bb_voxels;                  }
        const point_ti<> &  get_barycenter()                    const { return _barycenter;                     }
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
        point_ti<>          _min_bb_voxels;
        point_ti<>          _max_bb_voxels;
        point_ti<>          _barycenter;
        float               _scale;
        float               _unit_volume;
        float               _q[3][3];
        int                 _prim_on_surface;
        int                 _prim_inside_surface;
};
}; /* namespace raptor_convex_decomposition */
