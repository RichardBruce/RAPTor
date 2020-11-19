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
        mesh_decimation_options(const float max_perc_cost, const float max_perc_vertices) : 
            max_perc_cost(max_perc_cost), max_perc_vertices(max_perc_vertices), max_iterations(0)
        { }

        /* Convenient for test to stop on specific iteration */
        mesh_decimation_options(const float max_cost, const int max_iterations) : 
            max_perc_cost(0.0), max_perc_vertices(0.0), max_cost(max_cost), max_iterations(max_iterations)
        { }

        void absolute_costs(const float com_cost, const int vertices)
        {
        	if (max_iterations == 0)
        	{
	            max_cost = com_cost * max_perc_cost;
	            max_iterations = vertices * max_perc_vertices;
	        }
        }

    const float max_perc_cost;
    const float max_perc_vertices;
    float max_cost;
    int max_iterations;
};
} /* namespace raptor_mesh_decimation */
