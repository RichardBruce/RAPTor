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
struct tetrahedron
{
    public :
        tetrahedron() = default;

        tetrahedron(const point_t<> &a, const point_t<> &b, const point_t<> &c, const point_t<> &d, const voxel_value_t vv = voxel_value_t::primitive_undefined) : 
            pts{ a, b, c, d},
            loc(vv)

        {  };

        point_t<>       pts[4];
        voxel_value_t   loc;
};

class tetrahedron_set : public primitive_set
{            
    public :
        tetrahedron_set(const std::vector<tetrahedron> &tetrahedra, const point_t<> &min_bb, const float scale) : 
            _tetrahedra(tetrahedra),
            _min_bb(min_bb),
            _scale(scale),
            _prim_on_surface(0),
            _prim_inside_surface(0)
        {
            memset(_q, 0, sizeof(float) * 9);
        }

        tetrahedron_set() : tetrahedron_set(std::vector<tetrahedron>(), point_t<>(0.0f, 0.0f, 0.0f), 1.0f) {  }

        const point_t<> & get_min_bb()                        const { return _min_bb;                 }
        const point_t<> & get_max_bb()                        const { return _max_bb;                 }
        const point_t<> & get_barycenter()                    const { return _barycenter;             }
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
        void                intersect(const plane &p, std::vector<point_t<>> *const pos_pts, std::vector<point_t<>> *const neg_pts, const int sampling)             const;
        void                compute_cut_volumes(const plane &p, float *const pos_volume, float *const neg_volume)                                               const;
        void                compute_bounding_box();
        void                convert(convex_mesh *const mesh, const voxel_value_t value)                                                                         const;
        void                compute_axes_aligned_clipping_planes(std::vector<plane> *const planes, const int downsampling)                                      const;
        void                refine_axes_aligned_clipping_planes(std::vector<plane> *const planes, const plane &best, const int downsampling, const int index)   const;
        bool                add(tetrahedron t);

    private :
        void    add_clipped_tetrahedra(const point_t<> (&pts) [10], const int size);

        std::vector<tetrahedron>    _tetrahedra;
        point_t<>                     _min_bb;
        point_t<>                     _max_bb;
        point_t<>                     _barycenter;
        float                       _scale;
        float                       _q[3][3];
        int                         _prim_on_surface;
        int                         _prim_inside_surface;
};
}; /* namespace raptor_convex_decomposition */
