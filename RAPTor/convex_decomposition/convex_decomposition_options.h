#pragma once

/* Convex decomposition headers */
#include "discretisation_type.h"


namespace raptor_convex_decomposition
{
class convex_decomposition_options
{
    public :
        convex_decomposition_options(
            const float concavity               = 0.001f,
            const float alpha                   = 0.05f,
            const float beta                    = 0.05f,
            const float gamma                   = 0.0005f,
            const float min_volume_per_hull     = 0.0001f,
            const int resolution                = 100000,
            const int max_vertices_per_hull     = 64,
            const int depth                     = 20,
            const int plane_down_sampling       = 4,
            const int hull_down_sampling        = 4,
            const discretisation_type_t mode    = discretisation_type_t::voxel,
            const bool pca                      = false,
            const bool approx_hulls             = true) :
        concavity(concavity),
        alpha(alpha),
        beta(beta),
        gamma(gamma),
        min_volume_per_hull(min_volume_per_hull),
        resolution(resolution),
        max_vertices_per_hull(max_vertices_per_hull),
        depth(depth),
        plane_down_sampling(plane_down_sampling),
        hull_down_sampling(hull_down_sampling),
        mode(mode),
        pca(pca),
        approx_hulls(approx_hulls)
        {  }

        const float                 concavity;
        const float                 alpha;
        const float                 beta;
        const float                 gamma;
        const float                 min_volume_per_hull;
        const int                   resolution;
        const int                   max_vertices_per_hull;
        const int                   depth;
        const int                   plane_down_sampling;
        const int                   hull_down_sampling;
        const discretisation_type_t mode;
        const bool                  pca;
        const bool                  approx_hulls;
};
}; /* namespace raptor_convex_decomposition */
