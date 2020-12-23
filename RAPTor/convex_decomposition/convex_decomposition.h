#pragma once

/* Standard headers */

/* Boost headers */

/* Common headers */

/* Convex decomposition headers */
#include "convex_mesh.h"
#include "convex_decomposition_options.h"
#include "volume.h"


namespace raptor_convex_decomposition
{
class convex_decomposition
{
    public :
        convex_decomposition(const std::vector<point_t<>> &points, const std::vector<point_ti<>> &triangles, const convex_decomposition_options &params) :
            _params(params),
            _rot{{ 1.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, { 0.0f, 0.0f, 1.0f }},
            _volume_hull(0.0f),
            _dim(64)
        {
            align_mesh(points, triangles);
            voxelise_mesh(points, triangles);
            _pset.reset(_volume->convert(_params.mode));

            compute_acd();
            merge_convex_hulls();
            simplify_convex_hulls();
        }

        std::unique_ptr<primitive_set> get_primitive_set() const { return std::unique_ptr<primitive_set>(_volume->convert(_params.mode)); }

        int number_of_convex_hulls() const
        {
            return _convex_hulls.size();
        }

        convex_mesh* get_convex_hull(const int index) const
        {
            return _convex_hulls[index].get();
        }

    private :
        void compute_acd();
        void merge_convex_hulls();
        void simplify_convex_hulls();
        int compute_best_clipping_plane(const std::unique_ptr<primitive_set> &pset, const float volume, const std::vector<plane> &planes, const point_t<> &cut_dir, const float alpha, const float beta, const int dn_samp, plane *const best_plane);

        void align_mesh(const std::vector<point_t<>> &points, const std::vector<point_ti<>> &triangles)
        {
            if (!_params.pca)
            {
                return;
            }

            _dim = static_cast<int>(pow(static_cast<float>(_params.resolution), 1.0f / 3.0f) + 0.5f);
            const volume v(points, triangles, _dim, _barycenter, _rot);
            v.compute_principal_axes(_rot);
        }

        void voxelise_mesh(const std::vector<point_t<>> &points, const std::vector<point_ti<>> &triangles)
        {
            int iter = 0;
            const int max_iter = 5;
            while (iter++ < max_iter)
            {
                _volume.reset(new volume(points, triangles, _dim, _barycenter, _rot));
                const int n = _volume->number_of_primitives_on_surface() + _volume->number_of_primitives_inside();

                const float a = pow(static_cast<float>(_params.resolution) / n, 0.33f);
                const int di_next = static_cast<int>(_dim * a + 0.5f);
                if ((n < _params.resolution) && (iter < max_iter) && (_volume->number_of_primitives_on_surface() < _params.resolution / 8) && (_dim != di_next))
                {
                    _dim = di_next;
                }
                else
                {
                    break;
                }
            }
        }

        const convex_decomposition_options &        _params;
        std::vector<std::unique_ptr<convex_mesh>>   _convex_hulls;
        std::unique_ptr<volume>                     _volume;
        std::unique_ptr<primitive_set>              _pset;
        point_t<>                                     _barycenter;
        float                                       _rot[3][3];
        float                                       _volume_hull;
        int                                         _dim;
};
}
