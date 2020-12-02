#pragma once

/* Standard headers */

/* Boost headers */

/* Common headers */


namespace raptor_mesh_decimation
{
class mesh_decimation_options
{
    public:
        /* Convenient for use on different meshes */
        /* The default parameters give a reasonable quality performance trade off, but tweak if you want something special */
        /* face_clean_perc is a trade off for performance between time spent cleaning and time spent iterating over removed faces */
        /* Higher percentile_pass_max is faster and tends to get more iterations for cost limits, but at high cost on vertex limits */
        /* Higher percentile_pass_stride is faster, but tends to reduce iterations and increase cost. Try making it an integer divisor of percentile_pass_max. Values over ~20.0 dont get much performance improvement */
        mesh_decimation_options(const float max_merge_cost, const float max_cumulative_cost, const float max_perc_vertices, const double face_clean_perc = 0.30, const double percentile_pass_max = 70.0, const double percentile_pass_stride = 10.0) : 
            max_merge_cost(max_merge_cost), 
            max_cumulative_cost(max_cumulative_cost),
            max_perc_vertices(max_perc_vertices),
            face_clean_perc(face_clean_perc),
            percentile_pass_max(percentile_pass_max),
            percentile_pass_stride(percentile_pass_stride),
            max_iterations(0)
        { }

        /* Convenient for test to stop on specific iteration */
        mesh_decimation_options(const float max_merge_cost, const float max_cumulative_cost, const int max_iterations, const double face_clean_perc = 0.30, const double percentile_pass_max = 70.0, const double percentile_pass_stride = 10.0) : 
            max_merge_cost(max_merge_cost), 
            max_cumulative_cost(max_cumulative_cost),
            max_perc_vertices(0.0),
            face_clean_perc(face_clean_perc),
            percentile_pass_max(percentile_pass_max),
            percentile_pass_stride(percentile_pass_stride),
            max_iterations(max_iterations)
        { }

        void absolute_costs(const int vertices)
        {
            if (max_iterations == 0)
            {
                max_iterations = vertices * max_perc_vertices;
            }
        }

    const double max_merge_cost;
    const double max_cumulative_cost;
    const double max_perc_vertices;
    const double face_clean_perc;
    const double percentile_pass_max;
    const double percentile_pass_stride;
    int max_iterations;
};
} /* namespace raptor_mesh_decimation */
