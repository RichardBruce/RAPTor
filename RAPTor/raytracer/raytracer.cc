#include "common.h"
#include "raytracer.h"

#include "scene.h"

#include "line.h"
#include "triangle.h"

#include "ray.h"
#include "packet_ray.h"
#include "frustrum.h"

#include "kdt_node.h"
#include "kd_tree_builder.h"

#include "bih_node.h"
#include "bih_builder.h"

namespace raptor_raytracer
{
vector<triangle *>   * bih_node::o      = nullptr;
bih_node             * bih_node::bih    = nullptr;


/* Globals for drawing the KD-tree */
#ifdef SHOW_KD_TREE
fp_t x_split_node_crossed;
fp_t y_split_node_crossed;
fp_t z_split_node_crossed;
#endif

/* Global performance counters for the spatial sub division */
#ifdef SPATIAL_SUBDIVISION_STATISTICS
/* Dynamic properties */
unsigned nr         = 0;    /* Number of rays shot */
unsigned nit        = 0;    /* Number of intersection tests */
unsigned ritm       = 0;    /* Ratio of intersection tests performed to minimum required tests */
unsigned nts        = 0;    /* Average number of nodes accessed per ray */
unsigned nets       = 0;    /* Average number of elementary nodes accessed per ray */
unsigned neets      = 0;    /* Average number of empty elementary nodes accessed per ray */

/* Static properties */
unsigned ng         = 0;    /* Number of generic nodes */
unsigned ne         = 0;    /* Number of elementary nodes */
unsigned nee        = 0;    /* Number of empty elementary nodes */
unsigned ner        = 0;    /* Maximum size of an elementary node */
unsigned ave_ob     = 0;    /* Averafe size of an elementary node */
unsigned max_depth  = 0;    /* Maximum depth of the tree */
#endif


#ifdef SPATIAL_SUBDIVISION_BIH
#ifdef SIMD_PACKET_TRACING
/**********************************************************
 
**********************************************************/
inline int ray_trace_engine::find_bih_leaf_node(const frustrum &r, bih_stack_element *const entry_point, bih_stack_element **const out, unsigned size) const
{
    bih_stack_element *exit_point = *out;
    const bih_node *current_node = entry_point->n;
    point_t u  = entry_point->u;
    point_t l  = entry_point->l;
    fp_t t_max = entry_point->t_max;
    fp_t t_min = entry_point->t_min;
    while (true)
    {
#ifdef SPATIAL_SUBDIVISION_STATISTICS
        /* Count nodes accessed */
        ++nts;
#endif
        fp_t     near_split         = current_node->get_left_split();
        fp_t     far_split          = current_node->get_right_split();
        const bih_node *near_node   = current_node->get_left_child();
        const bih_node *far_node    = current_node->get_right_child();

        switch (current_node->get_split_axis())
        {
            /* This node is not split in any plane, ie/ it is a leaf */
            case axis_t::not_set : 
            {
#ifdef SPATIAL_SUBDIVISION_STATISTICS
                /* Count elementary nodes accessed */
                ++nets;
#endif
                /* update the state and return 1 for intersection testing */
                *out = exit_point;
                entry_point->n = current_node;
                entry_point->u = u;
                entry_point->l = l;
                return 1;
            }
    
            case axis_t::x_axis :
            {
                /* Note -- All rays in the frustrum must have the same direction */
                fp_t min_near_plane;
                fp_t min_far_plane;
                fp_t max_near_plane;
                fp_t max_far_plane;
                if (r.get_max_x_grad() < 0.0)
                {
                    min_near_plane = (far_split  - r.get_min_x0()) * r.get_min_x_igrad();
                    min_far_plane  = (near_split - r.get_min_x0()) * r.get_min_x_igrad();
                    max_near_plane = (far_split  - r.get_max_x0()) * r.get_max_x_igrad();
                    max_far_plane  = (near_split - r.get_max_x0()) * r.get_max_x_igrad();

                    const fp_t tmp_split = near_split;
                    near_split = far_split;
                    far_split  = tmp_split;
    
                    const bih_node *tmp_node = near_node;
                    near_node = far_node;
                    far_node  = tmp_node;
                }
                else
                {
                    max_near_plane = (near_split - r.get_min_x0()) * r.get_min_x_igrad();
                    max_far_plane  = (far_split  - r.get_min_x0()) * r.get_min_x_igrad();
                    min_near_plane = (near_split - r.get_max_x0()) * r.get_max_x_igrad();
                    min_far_plane  = (far_split  - r.get_max_x0()) * r.get_max_x_igrad();
                }
                
                /* Empty space is traversed, return 0 to pop the stack */
                if ((t_min > max_near_plane) && (t_max < min_far_plane))
                {   
#ifdef SPATIAL_SUBDIVISION_STATISTICS
                    /* Count empty elementary nodes accessed */
                    ++neets;
#endif
                    *out = exit_point;
                    return 0;         
                }

                /* If the corners make different decisions, split the frustrum */
                if ((((t_min > min_near_plane) && (t_min < max_near_plane)) || 
                     ((t_max <  max_far_plane) && (t_max >  min_far_plane))) && (size > MINIMUM_PACKET_SIZE))
                {
                    *out = exit_point;
                    entry_point->n      = current_node;
                    entry_point->t_max  = t_max;
                    entry_point->t_min  = t_min;
                    entry_point->u      = u;
                    entry_point->l      = l;
                    return -1;
                }

                /* Only the far node is traversed */
                if (t_min > max_near_plane)
                {
                    l.x             = far_split;
                    t_min           = max(t_min, min_far_plane);
                    current_node    = far_node;
                }
                /* Only the near node is traversed */
                else if (t_max < min_far_plane)
                {
                    u.x             = near_split;
                    t_max           = min(t_max, max_near_plane);
                    current_node    = near_node;
                }
                /* Both nodes are traversed, near node first */
                else
                {
                    /* Stack the far node */
                    ++exit_point;
                    exit_point->n       = far_node;
                    exit_point->l       = point_t(far_split, l.y, l.z);
                    exit_point->u       = u;
                    exit_point->t_max   = t_max;
                    exit_point->t_min   = max(t_min, min_far_plane);
        
                    u.x             = near_split;
                    t_max           = min(t_max, max_near_plane);
                    current_node    = near_node;
                }
                continue;
            }
    
            case axis_t::y_axis :
            {
                /* Note -- All rays in the frustrum must have the same direction */
                fp_t min_near_plane;
                fp_t min_far_plane;
                fp_t max_near_plane;
                fp_t max_far_plane;
                if (r.get_max_y_grad() < 0.0)
                {
                    min_near_plane = (far_split  - r.get_min_y0()) * r.get_min_y_igrad();
                    min_far_plane  = (near_split - r.get_min_y0()) * r.get_min_y_igrad();
                    max_near_plane = (far_split  - r.get_max_y0()) * r.get_max_y_igrad();
                    max_far_plane  = (near_split - r.get_max_y0()) * r.get_max_y_igrad();

                    const fp_t tmp_split = near_split;
                    near_split = far_split;
                    far_split  = tmp_split;
    
                    const bih_node *tmp_node = near_node;
                    near_node = far_node;
                    far_node  = tmp_node;
                }
                else
                {
                    max_near_plane = (near_split - r.get_min_y0()) * r.get_min_y_igrad();
                    max_far_plane  = (far_split  - r.get_min_y0()) * r.get_min_y_igrad();
                    min_near_plane = (near_split - r.get_max_y0()) * r.get_max_y_igrad();
                    min_far_plane  = (far_split  - r.get_max_y0()) * r.get_max_y_igrad();
                }

                /* Empty space is traversed, return 0 to pop the stack */
                if ((t_min > max_near_plane) && (t_max < min_far_plane))
                {   
#ifdef SPATIAL_SUBDIVISION_STATISTICS
                    /* Count empty elementary nodes accessed */
                    ++neets;
#endif
                    *out = exit_point;
                    return 0;         
                }

                /* If the corners make different decisions, split the frustrum */
                if ((((t_min > min_near_plane) && (t_min < max_near_plane)) || 
                     ((t_max <  max_far_plane) && (t_max >  min_far_plane))) && (size > MINIMUM_PACKET_SIZE))
                {
                    *out = exit_point;
                    entry_point->n      = current_node;
                    entry_point->t_max  = t_max;
                    entry_point->t_min  = t_min;
                    entry_point->u      = u;
                    entry_point->l      = l;
                    return -1;
                }

                /* Only the far node is traversed */
                if (t_min > max_near_plane)
                {
                    l.y             = far_split;
                    t_min           = max(t_min, min_far_plane);
                    current_node    = far_node;
                }
                /* Only the near node is traversed */
                else if (t_max < min_far_plane)
                {
                    u.y             = near_split;
                    t_max           = min(t_max, max_near_plane);
                    current_node    = near_node;
                }
                /* Both nodes are traversed, near node first */
                else
                {
                    /* Stack the far node */
                    ++exit_point;
                    exit_point->n       = far_node;
                    exit_point->l       = point_t(l.x, far_split, l.z);
                    exit_point->u       = u;
                    exit_point->t_max   = t_max;
                    exit_point->t_min   = max(t_min, min_far_plane);

                    u.y             = near_split;
                    t_max           = min(t_max, max_near_plane);
                    current_node    = near_node;
                }
                continue;
            }

            case axis_t::z_axis :
            {
                /* Note -- All rays in the frustrum must have the same direction */
                fp_t min_near_plane;
                fp_t min_far_plane;
                fp_t max_near_plane;
                fp_t max_far_plane;
                if (r.get_max_z_grad() < 0.0)
                {
                    min_near_plane = (far_split  - r.get_min_z0()) * r.get_min_z_igrad();
                    min_far_plane  = (near_split - r.get_min_z0()) * r.get_min_z_igrad();
                    max_near_plane = (far_split  - r.get_max_z0()) * r.get_max_z_igrad();
                    max_far_plane  = (near_split - r.get_max_z0()) * r.get_max_z_igrad();

                    const fp_t tmp_split = near_split;
                    near_split = far_split;
                    far_split  = tmp_split;
    
                    const bih_node *tmp_node = near_node;
                    near_node = far_node;
                    far_node  = tmp_node;
                }
                else
                {
                    max_near_plane = (near_split - r.get_min_z0()) * r.get_min_z_igrad();
                    max_far_plane  = (far_split  - r.get_min_z0()) * r.get_min_z_igrad();
                    min_near_plane = (near_split - r.get_max_z0()) * r.get_max_z_igrad();
                    min_far_plane  = (far_split  - r.get_max_z0()) * r.get_max_z_igrad();
                }
                
                /* Empty space is traversed, return 0 to pop the stack */
                if ((t_min > max_near_plane) && (t_max < min_far_plane))
                {   
#ifdef SPATIAL_SUBDIVISION_STATISTICS
                    /* Count empty elementary nodes accessed */
                    ++neets;
#endif
                    *out = exit_point;
                    return 0;         
                }
                
                /* If the corners make different decisions, split the frustrum */
                if ((((t_min > min_near_plane) && (t_min < max_near_plane)) || 
                     ((t_max <  max_far_plane) && (t_max >  min_far_plane))) && (size > MINIMUM_PACKET_SIZE))
                {
                    *out = exit_point;
                    entry_point->n      = current_node;
                    entry_point->t_max  = t_max;
                    entry_point->t_min  = t_min;
                    entry_point->u      = u;
                    entry_point->l      = l;
                    return -1;
                }

                /* Only the far node is traversed */
                if (t_min > max_near_plane)
                {
                    l.z             = far_split;
                    t_min           = max(t_min, min_far_plane);
                    current_node    = far_node;
                }
                /* Only the near node is traversed */
                else if (t_max < min_far_plane)
                {
                    u.z             = near_split;
                    t_max           = min(t_max, max_near_plane);
                    current_node    = near_node;
                }
                /* Both nodes are traversed, near node first */
                else
                {
                    /* Stack the far node */
                    ++exit_point;
                    exit_point->n       = far_node;
                    exit_point->l       = point_t(l.x, l.y, far_split);
                    exit_point->u       = u;
                    exit_point->t_max   = t_max;
                    exit_point->t_min   = max(t_min, min_far_plane);

                    u.z             = near_split;
                    t_max           = min(t_max, max_near_plane);
                    current_node    = near_node;
                }
                continue;
            }
        }
    }
}
/**********************************************************
 
**********************************************************/
void ray_trace_engine::bih_frustrum_find_nearest_object(const packet_ray *const r, const triangle **const i_o, packet_hit_description *const h, 
                                                        bih_stack_element entry_point, bih_stack_element *exit_point, int size) const
{
    assert(false);
    const bih_stack_element *top_of_stack = exit_point;

    /* Build a frustrum to traverse */
    frustrum f(r, size);

#ifdef FRUSTRUM_CULLING
    unsigned clipped_r[MAXIMUM_PACKET_SIZE];
#endif

    /* Traverse the whole tree */
    while (true)
    {
        /* Find a leaf node */
        const int cmd = this->find_bih_leaf_node(f, &entry_point, &exit_point, size);

        /* If the leaf contains objects find the closest intersecting object */
        if ((cmd == 1) && !entry_point.n->is_empty())
        {
#ifdef FRUSTRUM_CULLING
            unsigned clipped_size = 0;
#endif
            for (int i = 0; i < size; ++i)
            {
                /* Clip packet to node */
                const vfp_t i_x_grad = inverse(r[i].get_x_grad());
                const vfp_t i_y_grad = inverse(r[i].get_y_grad());
                const vfp_t i_z_grad = inverse(r[i].get_z_grad());
                const vfp_t x_exit   = (entry_point.u.x - r[i].get_x0()) * i_x_grad;
                const vfp_t x_entry  = (entry_point.l.x - r[i].get_x0()) * i_x_grad;
                const vfp_t y_exit   = (entry_point.u.y - r[i].get_y0()) * i_y_grad;
                const vfp_t y_entry  = (entry_point.l.y - r[i].get_y0()) * i_y_grad;
                const vfp_t z_exit   = (entry_point.u.z - r[i].get_z0()) * i_z_grad;
                const vfp_t z_entry  = (entry_point.l.z - r[i].get_z0()) * i_z_grad;
            
                const vfp_t mask = ((y_entry > x_exit) | (x_entry > y_exit)) | 
                                   ((z_entry > x_exit) | (x_entry > z_exit)) | 
                                   ((z_entry > y_exit) | (y_entry > z_exit));

                /* If packet enters leaf then test it */
                if (move_mask(mask) != 0xf)
                {
#ifdef FRUSTRUM_CULLING
                    clipped_r[clipped_size++] = i;
#else
                    entry_point.n->test_leaf_node_nearest(&r[i], &i_o[i << LOG2_SIMD_WIDTH], &h[i], 1);
#endif
                }
            }

#ifdef FRUSTRUM_CULLING
            if (clipped_size > 0)
            {
                f.adapt_to_leaf(r, entry_point.u, entry_point.l, clipped_r, clipped_size);
                entry_point.n->test_leaf_node_nearest(f, r, i_o, h, clipped_r, clipped_size);
            }
#endif
        }
        else if (cmd == -1)
        {
            for (int i = 0; i < size; i += (size/SPLIT_PACKET_DIVISOR))
            {
                this->bih_frustrum_find_nearest_object(&r[i], &i_o[i << LOG2_SIMD_WIDTH], &h[i], entry_point, exit_point, size/SPLIT_PACKET_DIVISOR);
            }
        }
        

        /* Unwind the stack */
        vfp_t vmax_d = h[0].d;
        for (int i = 1; i < size; ++i)
        {
            vmax_d = max(vmax_d, h[i].d);
        }
        const fp_t max_d = max(max(vmax_d[0], vmax_d[1]), max(vmax_d[2], vmax_d[3]));

        do
        {
            /* If the whole tree has been traversed, return */
            if (exit_point == top_of_stack)
            {
                return;
            }
            
            entry_point.n       = exit_point->n;
            entry_point.t_max   = exit_point->t_max;
            entry_point.t_min   = exit_point->t_min;
            entry_point.u       = exit_point->u;
            entry_point.l       = exit_point->l;
            exit_point--;
            
        } while (entry_point.t_min >= max_d);
        
        entry_point.t_max = min(entry_point.t_max, max_d);
    }
}


/**********************************************************
 
**********************************************************/
void ray_trace_engine::bih_frustrum_find_nearest_object(const packet_ray *const r, const triangle **const i_o, packet_hit_description *const h, int size) const
{
#ifdef SPATIAL_SUBDIVISION_STATISTICS
    /* Count number of rays shot */
    ++nr;
#endif
    /* state of stack */
    bih_stack_element *exit_point = &(this->bih_stack[0]);
    exit_point->n       = nullptr;
    exit_point->t_max   = MAX_DIST;
    exit_point->t_min   = 0.0;

    bih_stack_element  entry_point;
    entry_point.n      = this->bih_base;
    entry_point.t_max  = MAX_DIST;
    entry_point.t_min  = 0.0;
    
    /* Build a frustrum to traverse */
    frustrum f(r, size);
    
#ifdef FRUSTRUM_CULLING
    unsigned clipped_r[MAXIMUM_PACKET_SIZE];
#endif

    /* Set the scene bounding box based on ray direction */
    if (f.get_min_x_grad() < 0)
    {
        entry_point.u.x    = triangle::get_scene_lower_bounds().x;
        entry_point.l.x    = triangle::get_scene_upper_bounds().x;
    }
    else
    {
        entry_point.u.x    = triangle::get_scene_upper_bounds().x;
        entry_point.l.x    = triangle::get_scene_lower_bounds().x;
    }

    if (f.get_min_y_grad() < 0)
    {
        entry_point.u.y    = triangle::get_scene_lower_bounds().y;
        entry_point.l.y    = triangle::get_scene_upper_bounds().y;
    }
    else
    {
        entry_point.u.y    = triangle::get_scene_upper_bounds().y;
        entry_point.l.y    = triangle::get_scene_lower_bounds().y;
    }

    if (f.get_min_z_grad() < 0)
    {
        entry_point.u.z    = triangle::get_scene_lower_bounds().z;
        entry_point.l.z    = triangle::get_scene_upper_bounds().z;
    }
    else
    {
        entry_point.u.z    = triangle::get_scene_upper_bounds().z;
        entry_point.l.z    = triangle::get_scene_lower_bounds().z;
    }

    /* Clip packet to the world */
    const vfp_t x_bounds    = vfp_t(entry_point.u.x, entry_point.l.x, entry_point.u.x, entry_point.l.x);
    const vfp_t y_bounds    = vfp_t(entry_point.u.y, entry_point.l.y, entry_point.u.y, entry_point.l.y);
    const vfp_t z_bounds    = vfp_t(entry_point.u.z, entry_point.l.z, entry_point.u.z, entry_point.l.z);
    const vfp_t x0          = (x_bounds - f.get_mm_ogn(0)) * f.get_mm_idir(0);
    const vfp_t y0          = (y_bounds - f.get_mm_ogn(1)) * f.get_mm_idir(1);
    const vfp_t z0          = (z_bounds - f.get_mm_ogn(2)) * f.get_mm_idir(2);

    const vfp_t x_zip       = shuffle<2, 3, 0, 1>(x0, x0); 
    const vfp_t y_zip       = shuffle<2, 3, 0, 1>(y0, y0); 
    const vfp_t z_zip       = shuffle<2, 3, 0, 1>(z0, z0); 

    const vfp_t x_entry     = min(x0, x_zip);
    const vfp_t x_exit      = max(x0, x_zip);
    const vfp_t y_entry     = min(y0, y_zip);
    const vfp_t y_exit      = max(y0, y_zip);
    const vfp_t z_entry     = min(z0, z_zip);
    const vfp_t z_exit      = max(z0, z_zip);

    const vfp_t mask        = (y_entry > x_exit) | (x_entry > y_exit) | 
                              (z_entry > x_exit) | (x_entry > z_exit) | 
                              (z_entry > y_exit) | (y_entry > z_exit);

    const int int_mask = move_mask(mask);
    if ((int_mask & 0x3) && (int_mask & 0xc))
    {
        return;
    }
    
    const fp_t near     = min(min(x_entry[0], y_entry[0]), z_entry[0]);
    const fp_t far      = max(max(x_exit[2],  y_exit[2] ), y_exit[2] );
    entry_point.t_max   = far;
    entry_point.t_min   = max((fp_t)0.0, near);
    if (entry_point.t_min > entry_point.t_max)
    {
        return;
    }

    /* Traverse the whole tree */
    fp_t max_d = MAX_DIST;
    while (true)
    {
        /* Find a leaf node */
        const int cmd = this->find_bih_leaf_node(f, &entry_point, &exit_point, size);

        /* If the leaf contains objects find the closest intersecting object */
        if ((cmd == 1) && !entry_point.n->is_empty())
        {
#ifdef FRUSTRUM_CULLING
            unsigned clipped_size = 0;
#endif
            for (int i = 0; i < size; ++i)
            {
                /* Clip packet to node */
                const vfp_t i_x_grad = inverse(r[i].get_x_grad());
                const vfp_t i_y_grad = inverse(r[i].get_y_grad());
                const vfp_t i_z_grad = inverse(r[i].get_z_grad());
                const vfp_t x_exit   = (entry_point.u.x - r[i].get_x0()) * i_x_grad;
                const vfp_t x_entry  = (entry_point.l.x - r[i].get_x0()) * i_x_grad;
                const vfp_t y_exit   = (entry_point.u.y - r[i].get_y0()) * i_y_grad;
                const vfp_t y_entry  = (entry_point.l.y - r[i].get_y0()) * i_y_grad;
                const vfp_t z_exit   = (entry_point.u.z - r[i].get_z0()) * i_z_grad;
                const vfp_t z_entry  = (entry_point.l.z - r[i].get_z0()) * i_z_grad;
            
                const vfp_t mask = ((y_entry > x_exit) | (x_entry > y_exit)) | 
                                   ((z_entry > x_exit) | (x_entry > z_exit)) | 
                                   ((z_entry > y_exit) | (y_entry > z_exit));
          
                /* If packet enters leaf then test it */
                if (move_mask(mask) != 0xf)
                {
#ifdef FRUSTRUM_CULLING
                    clipped_r[clipped_size++] = i;
#else
                    entry_point.n->test_leaf_node_nearest(&r[i], &i_o[i << LOG2_SIMD_WIDTH], &h[i], 1);
#endif
                }
            }
#ifdef FRUSTRUM_CULLING
            if (clipped_size > 0)
            {
                f.adapt_to_leaf(r, entry_point.u, entry_point.l, clipped_r, clipped_size);
                entry_point.n->test_leaf_node_nearest(f, r, i_o, h, clipped_r, clipped_size);
            }
#endif
            /* Update furthest intersection */
            vfp_t vmax_d = h[0].d;
            for (int i = 1; i < size; ++i)
            {
                vmax_d = max(vmax_d, h[i].d);
            }
            max_d = max(max(vmax_d[0], vmax_d[1]), max(vmax_d[2], vmax_d[3]));

        }
        else if (cmd == -1)
        {
//            for (unsigned i = 0; i < size; i += (size/SPLIT_PACKET_DIVISOR))
//            {
//                this->bih_frustrum_find_nearest_object(&r[i], &i_o[i << LOG2_SIMD_WIDTH], &h[i], entry_point, exit_point, size/SPLIT_PACKET_DIVISOR);
//            }
            for (int i = 0; i < size; ++i)
            {
                this->bih_find_nearest_object(&r[i], &i_o[i << LOG2_SIMD_WIDTH], &h[i], entry_point, exit_point);
            }
        }

        /* Unwind the stack */
        do
        {
            /* If the whole tree has been traversed, return */
            if (exit_point == &(this->bih_stack[0]))
            {
                return;
            }
            
            entry_point.n       = exit_point->n;
            entry_point.t_max   = exit_point->t_max;
            entry_point.t_min   = exit_point->t_min;
            entry_point.u       = exit_point->u;
            entry_point.l       = exit_point->l;
            exit_point--;
            
        } while (entry_point.t_min >= max_d);
        
        entry_point.t_max = min(entry_point.t_max, max_d);
    }
}


/**********************************************************
 
**********************************************************/
void ray_trace_engine::bih_frustrum_found_nearer_object(const packet_ray *const r, const vfp_t *t, vfp_t *closer, unsigned size) const
{
#ifdef SPATIAL_SUBDIVISION_STATISTICS
    /* Count number of rays shot */
    ++nr;
#endif
    vfp_t vmax_d = t[0];
    vfp_t vmin_d = t[0];
    packet_hit_description h[MAXIMUM_PACKET_SIZE];
    for (unsigned i = 1; i < size; ++i)
    {
        vmax_d = max(vmax_d, t[i]);
        vmin_d = min(vmin_d, t[i]);
        h[i].d = t[i];
    }
    fp_t max_d = max(max(vmax_d[0], vmax_d[1]), max(vmax_d[2], vmax_d[3]));
    fp_t min_d = min(min(vmin_d[0], vmin_d[1]), min(vmin_d[2], vmin_d[3]));

    /* state of stack */
    bih_stack_element *exit_point = &(this->bih_stack[0]);
    exit_point->n       = nullptr;
    exit_point->t_max   = max_d;
    exit_point->t_min   = 0.0;

    bih_stack_element  entry_point;
    entry_point.n      = this->bih_base;
    entry_point.t_max  = max_d;
    entry_point.t_min  = 0.0;
    
    /* Build a reverse frustrum to traverse */
    /* The rays are more coherant at the light */
#ifdef SOFT_SHADOW
    frustrum f(r, size);
#else
    frustrum f(r, point_t(r[0].get_dst(0)[0], r[0].get_dst(1)[0], r[0].get_dst(2)[0]), size);
#endif
    
#ifdef FRUSTRUM_CULLING
    unsigned clipped_r[MAXIMUM_PACKET_SIZE];
#endif

    /* Set the scene bounding box based on ray direction */
    if (f.get_min_x_grad() < 0)
    {
        entry_point.u.x    = triangle::get_scene_lower_bounds().x;
        entry_point.l.x    = triangle::get_scene_upper_bounds().x;
    }
    else
    {
        entry_point.u.x    = triangle::get_scene_upper_bounds().x;
        entry_point.l.x    = triangle::get_scene_lower_bounds().x;
    }

    if (f.get_min_y_grad() < 0)
    {
        entry_point.u.y    = triangle::get_scene_lower_bounds().y;
        entry_point.l.y    = triangle::get_scene_upper_bounds().y;
    }
    else
    {
        entry_point.u.y    = triangle::get_scene_upper_bounds().y;
        entry_point.l.y    = triangle::get_scene_lower_bounds().y;
    }

    if (f.get_min_z_grad() < 0)
    {
        entry_point.u.z    = triangle::get_scene_lower_bounds().z;
        entry_point.l.z    = triangle::get_scene_upper_bounds().z;
    }
    else
    {
        entry_point.u.z    = triangle::get_scene_upper_bounds().z;
        entry_point.l.z    = triangle::get_scene_lower_bounds().z;
    }

    /* Traverse the whole tree */
    while (true)
    {
        /* Find a leaf node */
        const int cmd = this->find_bih_leaf_node(f, &entry_point, &exit_point, size);

        /* If the leaf contains objects find the closest intersecting object */
        if ((cmd == 1) && !entry_point.n->is_empty())
        {
#ifdef FRUSTRUM_CULLING
            unsigned clipped_size = 0;
#endif
            for (unsigned int i = 0; i < size; ++i)
            {
                if (move_mask(closer[i]) == ((1 << SIMD_WIDTH) - 1))
                {
                    continue;
                }

                /* Clip packet to node */
                const vfp_t i_x_grad = inverse(r[i].get_x_grad());
                const vfp_t i_y_grad = inverse(r[i].get_y_grad());
                const vfp_t i_z_grad = inverse(r[i].get_z_grad());
#ifdef SOFT_SHADOW
                const vfp_t x_exit   = (entry_point.u.x - r[i].get_x0()) * i_x_grad;
                const vfp_t x_entry  = (entry_point.l.x - r[i].get_x0()) * i_x_grad;
                const vfp_t y_exit   = (entry_point.u.y - r[i].get_y0()) * i_y_grad;
                const vfp_t y_entry  = (entry_point.l.y - r[i].get_y0()) * i_y_grad;
                const vfp_t z_exit   = (entry_point.u.z - r[i].get_z0()) * i_z_grad;
                const vfp_t z_entry  = (entry_point.l.z - r[i].get_z0()) * i_z_grad;

#else
                const vfp_t x_entry  = (entry_point.u.x - r[i].get_x0()) * i_x_grad;
                const vfp_t x_exit   = (entry_point.l.x - r[i].get_x0()) * i_x_grad;
                const vfp_t y_entry  = (entry_point.u.y - r[i].get_y0()) * i_y_grad;
                const vfp_t y_exit   = (entry_point.l.y - r[i].get_y0()) * i_y_grad;
                const vfp_t z_entry  = (entry_point.u.z - r[i].get_z0()) * i_z_grad;
                const vfp_t z_exit   = (entry_point.l.z - r[i].get_z0()) * i_z_grad;
#endif /* #ifdef SOFT_SHADOW */

                const vfp_t mask = ((y_entry > x_exit) | (x_entry > y_exit)) | 
                                   ((z_entry > x_exit) | (x_entry > z_exit)) | 
                                   ((z_entry > y_exit) | (y_entry > z_exit));
                                   
                /* If packet enters leaf then test it */
                if (move_mask(mask) != ((1 << SIMD_WIDTH) - 1))
                {
#ifdef FRUSTRUM_CULLING
                    clipped_r[clipped_size++] = i;
#else
                    entry_point.n->test_leaf_node_nearer(&r[i], &closer[i], t[i], &h[i]);
#endif
                }
            }
#ifdef FRUSTRUM_CULLING
            if (clipped_size > 0)
            {
                f.adapt_to_leaf(r, entry_point.u, entry_point.l, clipped_r, clipped_size);
                entry_point.n->test_leaf_node_nearer(f, r, closer, t, h, clipped_r, clipped_size);
            }
#endif

            /* Update furthest intersection */
            vmax_d = h[0].d;
            for (unsigned i = 1; i < size; ++i)
            {
                vmax_d = max(vmax_d, h[i].d);
            }
            max_d = max(max(vmax_d[0], vmax_d[1]), max(vmax_d[2], vmax_d[3]));
            
            /* Early exit for all rays occluded */
            if (max_d < min_d)
            {
                return;
            }

        }
        else if (cmd == -1)
        {
            for (unsigned i = 0; i < size; ++i)
            {
                entry_point.vt_max  = vfp_t(t[i] - entry_point.t_min);
                entry_point.vt_min  = vfp_t(t[i] - entry_point.t_max);
                closer[i] |= this->bih_found_nearer_object(&r[i], t[i], entry_point, exit_point);
            }
        }

        /* Unwind the stack */
        do
        {
            /* If the whole tree has been traversed, return */
            if (exit_point == &(this->bih_stack[0]))
            {
                return;
            }
            
            entry_point.n       = exit_point->n;
            entry_point.t_max   = exit_point->t_max;
            entry_point.t_min   = exit_point->t_min;
            entry_point.u       = exit_point->u;
            entry_point.l       = exit_point->l;
            exit_point--;
            
        } while (entry_point.t_min >= max_d);
        
        entry_point.t_max = min(entry_point.t_max, max_d);
    }
}


/**********************************************************
 
**********************************************************/
inline bool ray_trace_engine::find_bih_leaf_node(const packet_ray &r, bih_stack_element *const entry_point, bih_stack_element **const out, const vfp_t *const i_rd) const
{
    bih_stack_element *exit_point = *out;
    const bih_node *current_node = entry_point->n;
    vfp_t t_max = entry_point->vt_max;
    vfp_t t_min = entry_point->vt_min;
    while (true)
    {
#ifdef SPATIAL_SUBDIVISION_STATISTICS
        /* Count nodes accessed */
        ++nts;
#endif
        const vfp_t     near_split(current_node->get_left_split());
        const vfp_t     far_split (current_node->get_right_split());
        const bih_node *near_node   = current_node->get_left_child();
        const bih_node *far_node    = current_node->get_right_child();

        switch (current_node->get_split_axis())
        {
            /* This node is not split in any plane, ie/ it is a leaf */
            case axis_t::not_set : 
            {
#ifdef SPATIAL_SUBDIVISION_STATISTICS
                /* Count elementary nodes accessed */
                ++nets;
#endif
                /* update the state and return 1 for intersection testing */
                entry_point->n  = current_node;
                *out = exit_point;
                return true;
            }
    
            case axis_t::x_axis :
            {
                vfp_t near_plane = (near_split - r.get_x0()) * i_rd[0];
                vfp_t far_plane  = (far_split  - r.get_x0()) * i_rd[0];
                
                int neg_dir = move_mask(r.get_x_grad());
                if (neg_dir != 0)
                {
                    const vfp_t tmp_plane = near_plane;
                    near_plane = far_plane;
                    far_plane  = tmp_plane;
    
                    const bih_node *tmp_node = near_node;
                    near_node = far_node;
                    far_node  = tmp_node;
                }

                /* Empty space is traversed, return 0 to pop the stack */
                int a_node = move_mask((t_min < near_plane) | (t_max > far_plane));
                if (!a_node)
                {   
#ifdef SPATIAL_SUBDIVISION_STATISTICS
                    /* Count empty elementary nodes accessed */
                    ++neets;
#endif
                   *out = exit_point;
                    return false;         
                }

                /* Only the far node is traversed */
                int o_far   = move_mask(t_min < near_plane);
                int o_near  = move_mask(t_max > far_plane);
                if (!o_far)
                {
                    t_min           = max(t_min, far_plane);
                    current_node    = far_node;
                }
                /* Only the near node is traversed */
                else if (!o_near)
                {
                    t_max           = min(t_max, near_plane);
                    current_node    = near_node;
                }
                /* Both nodes are traversed, near node first */
                else
                {
                    /* Stack the far node */
                    ++exit_point;
                    exit_point->n       = far_node;
                    exit_point->vt_max  = t_max;
                    exit_point->vt_min  = max(t_min, far_plane);

                    t_max           = min(t_max, near_plane);
                    current_node    = near_node;
                }
                continue;
            }
    
            case axis_t::y_axis :
            {
                vfp_t near_plane = (near_split - r.get_y0()) * i_rd[1];
                vfp_t far_plane  = (far_split  - r.get_y0()) * i_rd[1];
                
                int neg_dir = move_mask(r.get_y_grad());
                if (neg_dir != 0)
                {
                    const vfp_t tmp_plane = near_plane;
                    near_plane = far_plane;
                    far_plane  = tmp_plane;
    
                    const bih_node *tmp_node = near_node;
                    near_node = far_node;
                    far_node  = tmp_node;
                }
 
                /* Empty space is traversed, return 0 to pop the stack */
                int a_node = move_mask((t_min < near_plane) | (t_max > far_plane));
                if (!a_node)
                {   
#ifdef SPATIAL_SUBDIVISION_STATISTICS
                    /* Count empty elementary nodes accessed */
                    ++neets;
#endif
                   *out = exit_point;
                    return false;         
                }

                /* Only the far node is traversed */
                int o_far   = move_mask(t_min < near_plane);
                int o_near  = move_mask(t_max > far_plane);
                if (!o_far)
                {
                    t_min           = max(t_min, far_plane);
                    current_node    = far_node;
                }
                /* Only the near node is traversed */
                else if (!o_near)
                {
                    t_max           = min(t_max, near_plane);
                    current_node    = near_node;
                }
                /* Both nodes are traversed, near node first */
                else
                {
                    /* Stack the far node */
                    ++exit_point;
                    exit_point->n       = far_node;
                    exit_point->vt_max  = t_max;
                    exit_point->vt_min  = max(t_min, far_plane);

                    t_max           = min(t_max, near_plane);
                    current_node    = near_node;
                } 
                continue;
            }

            case axis_t::z_axis :
            {
                vfp_t near_plane = (near_split - r.get_z0()) * i_rd[2];
                vfp_t far_plane  = (far_split  - r.get_z0()) * i_rd[2];
                
                int neg_dir = move_mask(r.get_z_grad());
                if (neg_dir != 0)
                {
                    const vfp_t tmp_plane = near_plane;
                    near_plane = far_plane;
                    far_plane  = tmp_plane;
    
                    const bih_node *tmp_node = near_node;
                    near_node = far_node;
                    far_node  = tmp_node;
                }

                /* Empty space is traversed, return 0 to pop the stack */
                int a_node = move_mask((t_min < near_plane) | (t_max > far_plane));
                if (!a_node)
                {   
#ifdef SPATIAL_SUBDIVISION_STATISTICS
                    /* Count empty elementary nodes accessed */
                    ++neets;
#endif
                    *out = exit_point;
                    return false;         
                }

                /* Only the far node is traversed */
                int o_far   = move_mask(t_min < near_plane);
                int o_near  = move_mask(t_max > far_plane);
                if (!o_far)
                {
                    t_min           = max(t_min, far_plane);
                    current_node    = far_node;
                }
                /* Only the near node is traversed */
                else if (!o_near)
                {
                    t_max           = min(t_max, near_plane);
                    current_node    = near_node;
                }
                /* Both nodes are traversed, near node first */
                else
                {
                    /* Stack the far node */
                    ++exit_point;
                    exit_point->n       = far_node;
                    exit_point->vt_max  = t_max;
                    exit_point->vt_min  = max(t_min, far_plane);

                    t_max           = min(t_max, near_plane);
                    current_node    = near_node;
                } 
                continue;
            }
        }
    }
}


/**********************************************************
 
**********************************************************/
void ray_trace_engine::bih_find_nearest_object(const packet_ray *const r, const triangle **const i_o, packet_hit_description *const h) const
{
#ifdef SPATIAL_SUBDIVISION_STATISTICS
    /* Count number of rays shot */
    ++nr;
#endif
    /* state of stack */
    bih_stack_element *exit_point = &(this->bih_stack[0]);

    bih_stack_element   entry_point;
    entry_point.n       = this->bih_base;
    entry_point.vt_max  = vfp_t(MAX_DIST);
    entry_point.vt_min  = vfp_t(0.0);
    
    /* Take the inverse direction of the ray for faster traversal */
    vfp_t i_rd[3] = { inverse(r->get_x_grad()), inverse(r->get_y_grad()), inverse(r->get_z_grad()) };

    /* Traverse the whole tree */
    while (true)
    {
        /* Find a leaf node */
        const bool cmd = this->find_bih_leaf_node(*r, &entry_point, &exit_point, &i_rd[0]);

        /* If the leaf contains objects find the closest intersecting object */
        if (cmd)
        {
            entry_point.n->test_leaf_node_nearest(r, i_o, h, 1);
        }

        /* Unwind the stack */
        do
        {
            /* If the whole tree has been traversed, return */
            if (exit_point == &(this->bih_stack[0]))
            {
                return;
            }
            
            entry_point.n      = exit_point->n;
            entry_point.vt_max = exit_point->vt_max;
            entry_point.vt_min = exit_point->vt_min;
            exit_point--;
            
        } while (!move_mask(entry_point.vt_min < h->d));
                
        entry_point.vt_max = min(entry_point.vt_max, h->d);
    }
}


/**********************************************************
 
**********************************************************/
void ray_trace_engine::bih_find_nearest_object(const packet_ray *const r, const triangle **const i_o, packet_hit_description *const h,
                                                bih_stack_element entry_point, bih_stack_element *exit_point) const
{
    const bih_stack_element *const top_of_stack = exit_point;
    entry_point.vt_max  = vfp_t(entry_point.t_max);
    entry_point.vt_min  = vfp_t(entry_point.t_min);

    /* Take the inverse direction of the ray for faster traversal */
    vfp_t i_rd[3] = { inverse(r->get_x_grad()), inverse(r->get_y_grad()), inverse(r->get_z_grad()) };

    /* Traverse the whole tree */
    while (true)
    {
        /* Find a leaf node */
        const bool cmd = this->find_bih_leaf_node(*r, &entry_point, &exit_point, &i_rd[0]);

        /* If the leaf contains objects find the closest intersecting object */
        if (cmd)
        {
            entry_point.n->test_leaf_node_nearest(r, i_o, h, 1);
        }

        /* Unwind the stack */
        do
        {
            /* If the whole tree has been traversed, return */
            if (exit_point == top_of_stack)
            {
                return;
            }
            
            entry_point.n      = exit_point->n;
            entry_point.vt_max = exit_point->vt_max;
            entry_point.vt_min = exit_point->vt_min;
            exit_point--;
            
        } while (!move_mask(entry_point.vt_min < h->d));
                
        entry_point.vt_max = min(entry_point.vt_max, h->d);
    }
}


/**********************************************************
 
**********************************************************/
vfp_t ray_trace_engine::bih_found_nearer_object(const packet_ray *const r, const vfp_t &t, bih_stack_element entry_point, bih_stack_element *exit_point) const
{
    vfp_t closer(vfp_zero);
    packet_hit_description h(t);
    
    bih_stack_element *const top_of_stack = exit_point;

    /* Take the inverse direction of the ray for faster traversal */
    vfp_t i_rd[3] = { inverse(r->get_x_grad()), inverse(r->get_y_grad()), inverse(r->get_z_grad()) };

    /* Traverse the whole tree */
    while (true)
    {
        /* Find a leaf node */
        const bool cmd = this->find_bih_leaf_node(*r, &entry_point, &exit_point, &i_rd[0]);

        /* If the leaf contains objects find the closest intersecting object */
        if (cmd)
        {
            entry_point.n->test_leaf_node_nearer(r, &closer, t, &h);
            
            /* If all rays have been occluded, return */
            if (move_mask(closer) == ((1 << SIMD_WIDTH) - 1))
            {
                return closer;
            }
        }

        /* Unwind the stack */
        do
        {
            /* If the whole tree has been traversed, return */
            if (exit_point == top_of_stack)
            {
                return closer;
            }
            
            entry_point.n      = exit_point->n;
            entry_point.vt_max = exit_point->vt_max;
            entry_point.vt_min = exit_point->vt_min;
            exit_point--;
            
        } while (!move_mask(entry_point.vt_min < h.d));
                
        entry_point.vt_max = min(entry_point.vt_max, h.d);
    }
}
#endif /* #ifdef SIMD_PACKET_TRACING */


/**********************************************************

**********************************************************/
vfp_t ray_trace_engine::bih_found_nearer_object(const packet_ray *const r, const vfp_t &t) const
{
#ifdef SPATIAL_SUBDIVISION_STATISTICS
    /* Count number of rays shot */
    ++nr;
#endif
    /* state of stack */
    bih_stack_element *exit_point = &(this->bih_stack[0]);

    bih_stack_element   entry_point;
    entry_point.n       = this->bih_base;
    entry_point.vt_max  = t;
    entry_point.vt_min  = vfp_zero;
    
    vfp_t closer(vfp_zero);
    packet_hit_description h(t);

    /* Take the inverse direction of the ray for faster traversal */
    vfp_t i_rd[3] = { inverse(r->get_x_grad()), inverse(r->get_y_grad()), inverse(r->get_z_grad()) };

    /* Traverse the whole tree */
    while (true)
    {
        /* Find a leaf node */
        const bool cmd = this->find_bih_leaf_node(*r, &entry_point, &exit_point, &i_rd[0]);

        /* If the leaf contains objects find the closest intersecting object */
        if (cmd)
        {
            entry_point.n->test_leaf_node_nearer(r, &closer, t, &h);
            
            /* If all rays have been occluded, return */
            if (move_mask(closer) == ((1 << SIMD_WIDTH) - 1))
            {
                return closer;
            }
        }

        /* Unwind the stack */
        do
        {
            /* If the whole tree has been traversed, return */
            if (exit_point == &(this->bih_stack[0]))
            {
                return closer;
            }
            
            entry_point.n      = exit_point->n;
            entry_point.vt_max = exit_point->vt_max;
            entry_point.vt_min = exit_point->vt_min;
            --exit_point;
            
        } while (!move_mask(entry_point.vt_min < h.d));
                
        entry_point.vt_max = min(entry_point.vt_max, h.d);
    }
}


/**********************************************************
 
**********************************************************/
inline bool ray_trace_engine::find_bih_leaf_node(const ray &r, bih_stack_element *const entry_point, bih_stack_element **const out, const point_t &i_rd) const
{
    bih_stack_element *exit_point = *out;
    const bih_node *current_node = entry_point->n;
    fp_t t_max      = entry_point->t_max;
    fp_t t_min      = entry_point->t_min;
    while (true)
    {
#ifdef SPATIAL_SUBDIVISION_STATISTICS
        /* Count nodes accessed */
        ++nts;
#endif
        fp_t near_split       = current_node->get_left_split();
        fp_t far_split        = current_node->get_right_split();
        const bih_node *near_node   = current_node->get_left_child();
        const bih_node *far_node    = current_node->get_right_child();

        switch (current_node->get_split_axis())
        {
            /* This node is not split in any plane, ie/ it is a leaf */
            case axis_t::not_set : 
            {
#ifdef SPATIAL_SUBDIVISION_STATISTICS
                /* Count elementary nodes accessed */
                ++nets;
#endif
                /* update the state and return */
                entry_point->n  = current_node;
                *out = exit_point;
                return true;
            }
    
            case axis_t::x_axis :
            {
                fp_t near_plane = (near_split - r.get_x0()) * i_rd.x;
                fp_t far_plane  = (far_split  - r.get_x0()) * i_rd.x;
                if (r.get_x_grad() < 0.0)
                {
                   const fp_t tmp_plane = near_plane;
                   near_plane = far_plane;
                   far_plane  = tmp_plane;
    
                   const bih_node *tmp_node = near_node;
                   near_node = far_node;
                   far_node  = tmp_node;
                }

                /* Empty space is traversed, return false to pop the stack */
                if ((t_min > near_plane) && (t_max < far_plane))
                {   
#ifdef SPATIAL_SUBDIVISION_STATISTICS
                    /* Count empty elementary nodes accessed */
                    ++neets;
#endif
                   *out = exit_point;
                    return false;         
                }

                /* Only the far node is traversed */
                if (t_min > near_plane)
                {
                    t_min           = max(t_min, far_plane);
                    current_node    = far_node;
                }
                /* Only the near node is traversed */
                else if (t_max < far_plane)
                {
                    t_max           = min(t_max, near_plane);
                    current_node    = near_node;
                }
                /* Both nodes are traversed, near node first */
                else
                {
                    /* Stack the far node */
                    ++exit_point;
                    exit_point->n       = far_node;
                    exit_point->t_max   = t_max;
                    exit_point->t_min   = max(t_min, far_plane);

                    t_max           = min(t_max, near_plane);
                    current_node    = near_node;
                }
                continue;
            }
    
            case axis_t::y_axis :
            {
                fp_t near_plane = (near_split - r.get_y0()) * i_rd.y;
                fp_t far_plane  = (far_split  - r.get_y0()) * i_rd.y;
                if (r.get_y_grad() < 0.0)
                {
                   const fp_t tmp_plane = near_plane;
                   near_plane = far_plane;
                   far_plane  = tmp_plane;
    
                   const bih_node *tmp_node = near_node;
                   near_node = far_node;
                   far_node  = tmp_node;
                }
 
                /* Empty space is traversed, return false to pop the stack */
                if ((t_min > near_plane) && (t_max < far_plane))
                {   
#ifdef SPATIAL_SUBDIVISION_STATISTICS
                    /* Count empty elementary nodes accessed */
                    ++neets;
#endif
                   *out = exit_point;
                    return false;         
                }

                /* Only the far node is traversed */
                if (t_min > near_plane)
                {
                    t_min           = max(t_min, far_plane);
                    current_node    = far_node;
                }
                /* Only the near node is traversed */
                else if (t_max < far_plane)
                {
                    t_max           = min(t_max, near_plane);
                    current_node    = near_node;
                }
                /* Both nodes are traversed, near node first */
                else
                {
                    /* Stack the far node */
                    ++exit_point;
                    exit_point->n       = far_node;
                    exit_point->t_max   = t_max;
                    exit_point->t_min   = max(t_min, far_plane);

                    t_max           = min(t_max, near_plane);
                    current_node    = near_node;
                } 
                continue;
            }

            case axis_t::z_axis :
            {
                fp_t near_plane = (near_split - r.get_z0()) * i_rd.z;
                fp_t far_plane  = (far_split  - r.get_z0()) * i_rd.z;
                if (r.get_z_grad() < 0.0)
                {
                   const fp_t tmp_plane = near_plane;
                   near_plane = far_plane;
                   far_plane  = tmp_plane;
    
                   const bih_node *tmp_node = near_node;
                   near_node = far_node;
                   far_node  = tmp_node;
                }

                /* Empty space is traversed, return false to pop the stack */
                if ((t_min > near_plane) && (t_max < far_plane))
                {   
#ifdef SPATIAL_SUBDIVISION_STATISTICS
                    /* Count empty elementary nodes accessed */
                    ++neets;
#endif
                    *out = exit_point;
                    return false;         
                }

                /* Only the far node is traversed */
                if (t_min > near_plane)
                {
                    t_min           = max(t_min, far_plane);
                    current_node    = far_node;
                }
                /* Only the near node is traversed */
                else if (t_max < far_plane)
                {
                    t_max           = min(t_max, near_plane);
                    current_node    = near_node;
                }
                /* Both nodes are traversed, near node first */
                else
                {
                    /* Stack the far node */
                    ++exit_point;
                    exit_point->n       = far_node;
                    exit_point->t_max   = t_max;
                    exit_point->t_min   = max(t_min, far_plane);

                    t_max           = min(t_max, near_plane);
                    current_node    = near_node;
                } 
                continue;
            }
        }
    }
}


/**********************************************************
 
**********************************************************/
triangle* ray_trace_engine::bih_find_nearest_object(const ray *const r, hit_description *const h) const
{
#ifdef SPATIAL_SUBDIVISION_STATISTICS
    /* Count number of rays shot */
    ++nr;
#endif
    
    /* state of stack */
    bih_stack_element *exit_point = &(this->bih_stack[0]);
    exit_point->n       = nullptr;
    exit_point->t_max   = MAX_DIST;
    exit_point->t_min   = 0.0;

    bih_stack_element   entry_point;
    entry_point.n       = this->bih_base;
    entry_point.t_max   = MAX_DIST;
    entry_point.t_min   = 0.0;

    /* Set the scene bounding box based on ray direction */
    if (r->get_x_grad() >= (fp_t)0.0)
    {
        entry_point.u.x    = triangle::get_scene_lower_bounds().x;
        entry_point.l.x    = triangle::get_scene_upper_bounds().x;
    }
    else
    {
        entry_point.u.x    = triangle::get_scene_upper_bounds().x;
        entry_point.l.x    = triangle::get_scene_lower_bounds().x;
    }

    if (r->get_y_grad() >= (fp_t)0.0)
    {
        entry_point.u.y    = triangle::get_scene_lower_bounds().y;
        entry_point.l.y    = triangle::get_scene_upper_bounds().y;
    }
    else
    {
        entry_point.u.y    = triangle::get_scene_upper_bounds().y;
        entry_point.l.y    = triangle::get_scene_lower_bounds().y;
    }

    if (r->get_z_grad() >= (fp_t)0.0)
    {
        entry_point.u.z    = triangle::get_scene_lower_bounds().z;
        entry_point.l.z    = triangle::get_scene_upper_bounds().z;
    }
    else
    {
        entry_point.u.z    = triangle::get_scene_upper_bounds().z;
        entry_point.l.z    = triangle::get_scene_lower_bounds().z;
    }

    /* Clip packet to the world */
    const fp_t i_x_grad = (fp_t)1.0/r->get_x_grad();
    const fp_t i_y_grad = (fp_t)1.0/r->get_y_grad();
    const fp_t i_z_grad = (fp_t)1.0/r->get_z_grad();
    const fp_t x_entry  = (entry_point.u.x - r->get_x0()) * i_x_grad;
    const fp_t x_exit   = (entry_point.l.x - r->get_x0()) * i_x_grad;
    const fp_t y_entry  = (entry_point.u.y - r->get_y0()) * i_y_grad;
    const fp_t y_exit   = (entry_point.l.y - r->get_y0()) * i_y_grad;
    const fp_t z_entry  = (entry_point.u.z - r->get_z0()) * i_z_grad;
    const fp_t z_exit   = (entry_point.l.z - r->get_z0()) * i_z_grad;

    if ((y_entry > x_exit) || (x_entry > y_exit) || 
        (z_entry > x_exit) || (x_entry > z_exit) || 
        (z_entry > y_exit) || (y_entry > z_exit))
    {
        return nullptr;
    }
    
    const fp_t near     = min(min(x_entry, y_entry), z_entry);
    const fp_t far      = max(max(x_exit,  y_exit ), z_exit );
    entry_point.t_max   = far;
    entry_point.t_min   = max((fp_t)0.0, near);
    
    if (entry_point.t_min > entry_point.t_max)
    {
        return nullptr;
    }
    
    point_t i_ray_dir(i_x_grad, i_y_grad, i_z_grad);

    /* Traverse the whole tree */
    triangle        *hit_object  = nullptr;
    hit_description nearest_hit;
    while (true)
    {
        /* Find a leaf node */
        const bool leaf = this->find_bih_leaf_node(*r, &entry_point, &exit_point, i_ray_dir);

        /* If the leaf contains objects find the closest intersecting object */
        if (leaf)
        {
            triangle *intersecting_object = entry_point.n->test_leaf_node_nearest(r, &nearest_hit);

            /* If an intersecting object is found it is the closest so return */
            if (intersecting_object != nullptr)
            {
                hit_object = intersecting_object;
            }
        }

        /* Unwind the stack */
        do
        {
            /* If the whole tree has been traversed, return */
            if (exit_point == &(this->bih_stack[0]))
            {
                *h = nearest_hit;
                return hit_object;
            }
      
            entry_point.n      = exit_point->n;
            entry_point.t_max  = exit_point->t_max;
            entry_point.t_min  = exit_point->t_min;
            exit_point--;

        } while (entry_point.t_min >= nearest_hit.d);
        
        entry_point.t_max = min(entry_point.t_max, nearest_hit.d);
    }
}


/**********************************************************
 
**********************************************************/
bool ray_trace_engine::bih_found_nearer_object(const ray *const r, const fp_t t) const
{
#ifdef SPATIAL_SUBDIVISION_STATISTICS
    /* Count number of rays shot */
    ++nr;
#endif

    /* state of stack */
    bih_stack_element *exit_point = &(this->bih_stack[0]);
    exit_point->n       = nullptr;
    exit_point->t_max   = t;
    exit_point->t_min   = 0.0;

    bih_stack_element   entry_point;
    entry_point.n       = this->bih_base;
    entry_point.t_max   = t;
    entry_point.t_min   = 0.0;

    /* Set the scene bounding box based on ray direction */
    if (r->get_x_grad() >= (fp_t)0.0)
    {
        entry_point.u.x    = triangle::get_scene_lower_bounds().x;
        entry_point.l.x    = triangle::get_scene_upper_bounds().x;
    }
    else
    {
        entry_point.u.x    = triangle::get_scene_upper_bounds().x;
        entry_point.l.x    = triangle::get_scene_lower_bounds().x;
    }

    if (r->get_y_grad() >= (fp_t)0.0)
    {
        entry_point.u.y    = triangle::get_scene_lower_bounds().y;
        entry_point.l.y    = triangle::get_scene_upper_bounds().y;
    }
    else
    {
        entry_point.u.y    = triangle::get_scene_upper_bounds().y;
        entry_point.l.y    = triangle::get_scene_lower_bounds().y;
    }

    if (r->get_z_grad() >= (fp_t)0.0)
    {
        entry_point.u.z    = triangle::get_scene_lower_bounds().z;
        entry_point.l.z    = triangle::get_scene_upper_bounds().z;
    }
    else
    {
        entry_point.u.z    = triangle::get_scene_upper_bounds().z;
        entry_point.l.z    = triangle::get_scene_lower_bounds().z;
    }

    /* Clip packet to the world */
    const fp_t i_x_grad = (fp_t)1.0/r->get_x_grad();
    const fp_t i_y_grad = (fp_t)1.0/r->get_y_grad();
    const fp_t i_z_grad = (fp_t)1.0/r->get_z_grad();
    const fp_t x_entry  = (entry_point.u.x - r->get_x0()) * i_x_grad;
    const fp_t x_exit   = (entry_point.l.x - r->get_x0()) * i_x_grad;
    const fp_t y_entry  = (entry_point.u.y - r->get_y0()) * i_y_grad;
    const fp_t y_exit   = (entry_point.l.y - r->get_y0()) * i_y_grad;
    const fp_t z_entry  = (entry_point.u.z - r->get_z0()) * i_z_grad;
    const fp_t z_exit   = (entry_point.l.z - r->get_z0()) * i_z_grad;

    if ((y_entry > x_exit) || (x_entry > y_exit) || 
        (z_entry > x_exit) || (x_entry > z_exit) || 
        (z_entry > y_exit) || (y_entry > z_exit))
    {
        return false;
    }
    
    const fp_t near     = min(min(x_entry, y_entry), z_entry);
    const fp_t far      = max(max(x_exit,  y_exit ), z_exit );
    entry_point.t_max   = far;
    entry_point.t_min   = max((fp_t)0.0, near);
    
    if (entry_point.t_min > entry_point.t_max)
    {
        return false;
    }
    
    point_t i_ray_dir(i_x_grad, i_y_grad, i_z_grad);

    /* Traverse the whole tree */
    while (true)
    {
        /* Find a leaf node */
        const bool leaf = this->find_bih_leaf_node(*r, &entry_point, &exit_point, i_ray_dir);
        
        /* If the leaf contains objects find the closest intersecting object */
        if (leaf)
        {
            const bool closer = entry_point.n->test_leaf_node_nearer(r, t);
            /* If an intersecting object is found it is the closest so return */
            if (closer) 
            {
               return true;
            }
        }
  
        /* If the whole tree is traversed without finding an intersection return false */
        if (exit_point == &(this->bih_stack[0]))
        {
            return false;
        }
  
        /* Unwind the stack */
        entry_point.n      = exit_point->n;
        entry_point.t_max  = exit_point->t_max;
        entry_point.t_min  = exit_point->t_min;
      
        exit_point--;
    }
}
#else /* #ifdef SPATIAL_SUBDIVISION_BIH */

#ifdef SIMD_PACKET_TRACING
/**********************************************************
 
**********************************************************/
inline void ray_trace_engine::find_kdt_leaf_node(const packet_ray *const r, kdt_stack_element **const out, kdt_stack_element *const entry_point, const vfp_t *const i_rd, const int *const near_offset) const
{
    kdt_node *furthest;
    kdt_stack_element *exit_point   = *out;
    const kdt_node *current_node    = entry_point->n;
    vfp_t mask                      = entry_point->m;
    vfp_t t_max                     = entry_point->vt_max;
    vfp_t t_min                     = entry_point->vt_min;
    vfp_t dist;
    
    while (current_node->get_normal() != axis_t::not_set)
    {
        int axis = (int)current_node->get_normal() - 1;
        vfp_t split_pos(current_node->get_split_position());
#ifdef SPATIAL_SUBDIVISION_STATISTICS
        /* Count nodes accessed */
        ++nts;
#endif

        dist        = (split_pos - r->get_ogn(axis)) * i_rd[axis];
        furthest    = current_node->get_left() +  near_offset[axis];
            
        if (move_mask((dist > t_min) & mask) == 0)
        {
            current_node = furthest;
            continue;
        } 
     
        current_node    = current_node->get_left() + (near_offset[axis] ^ 0x1);
        if (move_mask((dist < t_max) & mask) != 0)
        {
            ++exit_point;
            exit_point->n       = furthest;
            exit_point->vt_min  = max(dist, t_min);
            exit_point->vt_max  = t_max;
            exit_point->m       = mask;

            t_max   = min(dist, t_max);
            mask    = (t_min < t_max); 
        }
    }

    entry_point->n  = current_node;
    *out            = exit_point;
#ifdef SPATIAL_SUBDIVISION_STATISTICS
     /* Count elementary nodes accessed */
     ++nets;
#endif
    return;
}


/**********************************************************
 
**********************************************************/
void ray_trace_engine::kdt_find_nearest_object(const packet_ray *const r, const triangle **const i_o, packet_hit_description *const h) const
{
#ifdef SHOW_KD_TREE
    x_split_node_crossed = 0.0;
    y_split_node_crossed = 0.0;
    z_split_node_crossed = 0.0;
#endif

#ifdef SPATIAL_SUBDIVISION_STATISTICS
    /* Count number of rays shot */
    ++nr;
#endif
    
    /* exit point setting */
    kdt_stack_element *exit_point = &(this->kdt_stack[0]);
    exit_point->vt_max  = vfp_t(MAX_DIST);
    exit_point->vt_min  = vfp_zero;
    exit_point->n       = nullptr;
    exit_point->m       = vfp_true;

    /* entry point setting */
    kdt_stack_element entry_point;
    entry_point.vt_max  = vfp_t(MAX_DIST);
    entry_point.vt_min  = vfp_zero;
    entry_point.n       = this->kdt_base;
    entry_point.m       = vfp_true;

    /* Take the inverse direction of the ray for faster traversal */
    vfp_t i_rd[3] = { inverse(r->get_x_grad()), inverse(r->get_y_grad()), inverse(r->get_z_grad()) };

    /* Ray direction LUT */
    int near_offset[3] = { (r->get_x_grad()[0] >= 0)?1:0, (r->get_y_grad()[0] >= 0)?1:0, (r->get_z_grad()[0] >= 0)?1:0 }; 

    /* Traverse the whole tree */
    while (true)
    {
        /* Find a leaf node */
        this->find_kdt_leaf_node(r, &exit_point, &entry_point, &i_rd[0], &near_offset[0]);

        /* If the leaf contains objects find the closest intersecting object */
        if (!entry_point.n->is_empty())
        {
            entry_point.n->test_leaf_node_nearest(r, i_o, h);
        } 
#ifdef SPATIAL_SUBDIVISION_STATISTICS
        /* Count empty elementary nodes accessed */
        else
        {
            ++neets;
        }
#endif
  
        /* Work back up the tree by poping form the stack */
        do 
        {
            /* If the whole tree is traversed without finding an intersection 
               return a null pointer */
            if (exit_point == &(this->kdt_stack[0]))
            {
                return;
            }
      
            entry_point.n       = exit_point->n;
            entry_point.vt_max  = exit_point->vt_max;
            entry_point.vt_min  = exit_point->vt_min;
            entry_point.m       = exit_point->m;
            exit_point --;

        } while (move_mask((entry_point.vt_min >= h->d) & entry_point.m) == ((1 << SIMD_WIDTH) - 1));
    
        entry_point.vt_max = min(entry_point.vt_max, h->d);
    }
}


/**********************************************************
 
**********************************************************/
void ray_trace_engine::kdt_find_nearest_object(const packet_ray *const r, const triangle **const i_o, packet_hit_description *const h,
                                                kdt_stack_element entry_point, kdt_stack_element *exit_point) const
{
    kdt_stack_element *const top_of_stack = exit_point;
    entry_point.m       = vfp_true;
    
    /* Take the inverse direction of the ray for faster traversal */
    vfp_t i_rd[3] = { inverse(r->get_x_grad()), inverse(r->get_y_grad()), inverse(r->get_z_grad()) };

    /* Ray direction LUT */
    int near_offset[3] = { (r->get_x_grad()[0] >= 0)?1:0, (r->get_y_grad()[0] >= 0)?1:0, (r->get_z_grad()[0] >= 0)?1:0 }; 

    /* Traverse the whole tree */
    while (true)
    {
        /* Find a leaf node */
        this->find_kdt_leaf_node(r, &exit_point, &entry_point, &i_rd[0], &near_offset[0]);

        /* If the leaf contains objects find the closest intersecting object */
        if (!entry_point.n->is_empty())
        {
            entry_point.n->test_leaf_node_nearest(r, i_o, h);
        } 
#ifdef SPATIAL_SUBDIVISION_STATISTICS
        /* Count empty elementary nodes accessed */
        else
        {
            ++neets;
        }
#endif
  
        /* Work back up the tree by poping form the stack */
        do 
        {
            /* If the whole tree is traversed without finding an intersection 
               return a null pointer */
            if (exit_point == top_of_stack)
            {
                return;
            }
      
            entry_point.n       = exit_point->n;
            entry_point.vt_max  = exit_point->vt_max;
            entry_point.vt_min  = exit_point->vt_min;
            entry_point.m       = exit_point->m;
            exit_point --;

        } while (move_mask((entry_point.vt_min >= h->d) & entry_point.m) == ((1 << SIMD_WIDTH) - 1));
    
        entry_point.vt_max = min(entry_point.vt_max, h->d);
    }
}


/**********************************************************
 
**********************************************************/
vfp_t ray_trace_engine::kdt_found_nearer_object(const packet_ray *const r, const vfp_t &t) const
{
#ifdef SPATIAL_SUBDIVISION_STATISTICS
    /* Count number of rays shot */
    ++nr;
#endif

    /* exit point setting */
    kdt_stack_element *exit_point = &(this->kdt_stack[0]);
    exit_point->vt_max  = t;
    exit_point->vt_min  = vfp_zero;
    exit_point->n       = nullptr;
    exit_point->m       = vfp_true;

    /* entry point setting */
    kdt_stack_element entry_point;
    entry_point.vt_max  = t;
    entry_point.vt_min  = vfp_zero;
    entry_point.n       = this->kdt_base;
    entry_point.m       = vfp_true;

    /* Traverse the whole tree */
    vfp_t closer(vfp_zero);
    packet_hit_description h(t);

    /* Take the inverse direction of the ray for faster traversal */
    vfp_t i_rd[3] = { inverse(r->get_x_grad()), inverse(r->get_y_grad()), inverse(r->get_z_grad()) };

    /* Ray direction LUT */
    int near_offset[3] = { (r->get_x_grad()[0] >= 0)?1:0, (r->get_y_grad()[0] >= 0)?1:0, (r->get_z_grad()[0] >= 0)?1:0 }; 

    while (true)
    {
        /* Find a leaf node */
        this->find_kdt_leaf_node(r, &exit_point, &entry_point, &i_rd[0], &near_offset[0]);
  
        /* If the leaf contains objects find the closest intersecting object */
        if (!entry_point.n->is_empty())
        {
            entry_point.n->test_leaf_node_nearer(r, &closer, t, &h);

            /* If an intersecting object is found it is the closest so return */
            if (move_mask(closer) == ((1 << SIMD_WIDTH) - 1))
            {
               return closer;
            }
        }
#ifdef SPATIAL_SUBDIVISION_STATISTICS
        /* Count empty elementary nodes accessed */
        else
        {
            ++neets;
        }
#endif
  
        /* Work back up the tree by poping form the stack */
        do
        {
            /* If the whole tree is traversed without finding an intersection 
               return a null pointer */
            if (exit_point == &(this->kdt_stack[0]))
            {
                return closer;
            }
  
            entry_point.n       = exit_point->n;
            entry_point.vt_max  = exit_point->vt_max;
            entry_point.vt_min  = exit_point->vt_min;
            entry_point.m       = exit_point->m;
            exit_point--;

        } while (move_mask((entry_point.vt_min >= h.d) & entry_point.m) == ((1 << SIMD_WIDTH) - 1));
    
        entry_point.vt_max = min(entry_point.vt_max, h.d);
    }
}


/**********************************************************
 
**********************************************************/
vfp_t ray_trace_engine::kdt_found_nearer_object(const packet_ray *const r, const vfp_t &t, kdt_stack_element entry_point, kdt_stack_element *exit_point) const
{
    kdt_stack_element *const top_of_stack = exit_point;
    entry_point.m       = vfp_true;

    /* Traverse the whole tree */
    vfp_t closer(vfp_zero);
    packet_hit_description h(t);

    /* Take the inverse direction of the ray for faster traversal */
    vfp_t i_rd[3] = { inverse(r->get_x_grad()), inverse(r->get_y_grad()), inverse(r->get_z_grad()) };

    /* Ray direction LUT */
    int near_offset[3] = { (r->get_x_grad()[0] >= 0)?1:0, (r->get_y_grad()[0] >= 0)?1:0, (r->get_z_grad()[0] >= 0)?1:0 }; 

    while (true)
    {
        /* Find a leaf node */
        this->find_kdt_leaf_node(r, &exit_point, &entry_point, &i_rd[0], &near_offset[0]);
  
        /* If the leaf contains objects find the closest intersecting object */
        if (!entry_point.n->is_empty())
        {
            entry_point.n->test_leaf_node_nearer(r, &closer, t, &h);

            /* If an intersecting object is found it is the closest so return */
            if (move_mask(closer) == ((1 << SIMD_WIDTH) - 1))
            {
               return closer;
            }
        }
#ifdef SPATIAL_SUBDIVISION_STATISTICS
        /* Count empty elementary nodes accessed */
        else
        {
            ++neets;
        }
#endif
  
        /* Work back up the tree by poping form the stack */
        do
        {
            /* If the whole tree is traversed without finding an intersection 
               return a null pointer */
            if (exit_point == top_of_stack)
            {
                return closer;
            }
  
            entry_point.n       = exit_point->n;
            entry_point.vt_max  = exit_point->vt_max;
            entry_point.vt_min  = exit_point->vt_min;
            entry_point.m       = exit_point->m;
            exit_point--;

        } while (move_mask((entry_point.vt_min >= h.d) & entry_point.m) == ((1 << SIMD_WIDTH) - 1));
    
        entry_point.vt_max = min(entry_point.vt_max, h.d);
    }
}


/**********************************************************
 
**********************************************************/
inline bool ray_trace_engine::find_kdt_leaf_node(const frustrum &r, kdt_stack_element *const entry_point, kdt_stack_element **const out, const int *const near_offset, unsigned size) const
{
    kdt_stack_element *exit_point = *out;
    const kdt_node *current_node = entry_point->n;
    point_t u  = entry_point->u;
    point_t l  = entry_point->l;
    fp_t t_max = entry_point->t_max;
    fp_t t_min = entry_point->t_min;
    
    kdt_node *furthest;
    fp_t min_dist, max_dist;
    while (true)
    {
#ifdef SPATIAL_SUBDIVISION_STATISTICS
        /* Count nodes accessed */
        ++nts;
#endif
        fp_t split_pos = current_node->get_split_position();

        switch (current_node->get_normal())
        {
            /* This node is not split in any plane, ie/ it is a leaf */
            case axis_t::not_set : 
            {
#ifdef SPATIAL_SUBDIVISION_STATISTICS
                /* Count elementary nodes accessed */
                ++nets;
#endif
                /* update the state and return 1 for intersection testing */
                *out = exit_point;
                entry_point->n = current_node;
                entry_point->u = u;
                entry_point->l = l;
                return true;
            }
    
            case axis_t::x_axis :
            {
                if (!near_offset[0])
                {
                    min_dist    = (split_pos - r.get_min_x0()) * r.get_min_x_igrad();
                    max_dist    = (split_pos - r.get_max_x0()) * r.get_max_x_igrad();
                }
                else
                {
                    min_dist    = (split_pos - r.get_max_x0()) * r.get_max_x_igrad();
                    max_dist    = (split_pos - r.get_min_x0()) * r.get_min_x_igrad();
                }
                furthest    = current_node->get_left() + near_offset[0];
                
                if ((((min_dist < t_min) && (max_dist > t_min)) || ((min_dist < t_max) && (max_dist > t_max))) && (size > MINIMUM_PACKET_SIZE))
                {
                    *out = exit_point;
                    entry_point->n      = current_node;
                    entry_point->vt_max = vfp_t(t_max);
                    entry_point->vt_min = vfp_t(t_min);
                    entry_point->u      = u;
                    entry_point->l      = l;
                    return false;
                }
    
                if (max_dist < t_min)
                {
                    current_node = furthest;
                    l.x          = split_pos;
                    continue;
                } 
    
                current_node    = current_node->get_left() + (near_offset[0] ^ 0x1);
                if (min_dist < t_max)
                {
                    ++exit_point;
                    exit_point->n       = furthest;
                    exit_point->t_min   = max(min_dist, t_min);
                    exit_point->t_max   = t_max;
                    exit_point->l       = point_t(split_pos, l.y, l.z);
                    exit_point->u       = u;

                    t_max   = min(max_dist, t_max);
                }

                u.x = split_pos;
                continue;
            }
    
            case axis_t::y_axis :
            {
                if (!near_offset[1])
                {
                    min_dist    = (split_pos - r.get_min_y0()) * r.get_min_y_igrad();
                    max_dist    = (split_pos - r.get_max_y0()) * r.get_max_y_igrad();
                }
                else
                {
                    min_dist    = (split_pos - r.get_max_y0()) * r.get_max_y_igrad();
                    max_dist    = (split_pos - r.get_min_y0()) * r.get_min_y_igrad();
                }
                furthest    = current_node->get_left() +  near_offset[1];
    
                if ((((min_dist < t_min) && (max_dist > t_min)) || ((min_dist < t_max) && (max_dist > t_max))) && (size > MINIMUM_PACKET_SIZE))
                {
                    *out = exit_point;
                    entry_point->n      = current_node;
                    entry_point->vt_max = vfp_t(t_max);
                    entry_point->vt_min = vfp_t(t_min);
                    entry_point->u      = u;
                    entry_point->l      = l;
                    return false;
                }

                if (max_dist < t_min)
                {
                    current_node = furthest;
                    l.y          = split_pos;
                    continue;
                } 
    
                current_node    = current_node->get_left() + (near_offset[1] ^ 0x1);
                if (min_dist < t_max)
                {
                    ++exit_point;
                    exit_point->n       = furthest;
                    exit_point->t_min   = max(min_dist, t_min);
                    exit_point->t_max   = t_max;
                    exit_point->l       = point_t(l.x, split_pos, l.z);
                    exit_point->u       = u;

                    t_max   = min(max_dist, t_max);
                }

                u.y = split_pos;
                continue;
            }

            case axis_t::z_axis :
            {
                if (!near_offset[2])
                {
                    min_dist    = (split_pos - r.get_min_z0()) * r.get_min_z_igrad();
                    max_dist    = (split_pos - r.get_max_z0()) * r.get_max_z_igrad();
                }
                else
                {
                    min_dist    = (split_pos - r.get_max_z0()) * r.get_max_z_igrad();
                    max_dist    = (split_pos - r.get_min_z0()) * r.get_min_z_igrad();
                }
                furthest    = current_node->get_left() +  near_offset[2];

                if ((((min_dist < t_min) && (max_dist > t_min)) || ((min_dist < t_max) && (max_dist > t_max))) && (size > MINIMUM_PACKET_SIZE))
                {
                    *out = exit_point;
                    entry_point->n      = current_node;
                    entry_point->vt_max = vfp_t(t_max);
                    entry_point->vt_min = vfp_t(t_min);
                    entry_point->u      = u;
                    entry_point->l      = l;
                    return false;
                }

                if (max_dist < t_min)
                {
                    current_node = furthest;
                    l.z          = split_pos;
                    continue;
                } 
    
                current_node    = current_node->get_left() + (near_offset[2] ^ 0x1);
                if (min_dist < t_max)
                {
                    ++exit_point;
                    exit_point->n       = furthest;
                    exit_point->t_min   = max(min_dist, t_min);
                    exit_point->t_max   = t_max;
                    exit_point->l       = point_t(l.x, l.y, split_pos);
                    exit_point->u       = u;

                    t_max   = min(max_dist, t_max);
                }

                u.z = split_pos;
                continue;
            }
        }
    }
}


/**********************************************************
 
**********************************************************/
void ray_trace_engine::kdt_frustrum_find_nearest_object(const packet_ray *const r, const triangle **const i_o, packet_hit_description *const h, int size) const
{
#ifdef SPATIAL_SUBDIVISION_STATISTICS
    /* Count number of rays shot */
    ++nr;
#endif
    /* state of stack */
    kdt_stack_element *exit_point = &(this->kdt_stack[0]);
    exit_point->n       = nullptr;
    exit_point->t_max   = MAX_DIST;
    exit_point->t_min   = 0.0;

    kdt_stack_element  entry_point;
    entry_point.n      = this->kdt_base;
    entry_point.t_max  = MAX_DIST;
    entry_point.t_min  = 0.0;
    entry_point.s      = nullptr;
    entry_point.d      = 0.0;
    entry_point.m      = vfp_true;
    
    /* Build a frustrum to traverse */
    frustrum f(r, size);
    
#ifdef FRUSTRUM_CULLING
    unsigned clipped_r[MAXIMUM_PACKET_SIZE];
#endif

    /* Set the scene bounding box based on frustrum direction */
    if (f.get_min_x_grad() < 0)
    {
        entry_point.u.x    = triangle::get_scene_lower_bounds().x;
        entry_point.l.x    = triangle::get_scene_upper_bounds().x;
    }
    else
    {
        entry_point.u.x    = triangle::get_scene_upper_bounds().x;
        entry_point.l.x    = triangle::get_scene_lower_bounds().x;
    }

    if (f.get_min_y_grad() < 0)
    {
        entry_point.u.y    = triangle::get_scene_lower_bounds().y;
        entry_point.l.y    = triangle::get_scene_upper_bounds().y;
    }
    else
    {
        entry_point.u.y    = triangle::get_scene_upper_bounds().y;
        entry_point.l.y    = triangle::get_scene_lower_bounds().y;
    }

    if (f.get_min_z_grad() < 0)
    {
        entry_point.u.z    = triangle::get_scene_lower_bounds().z;
        entry_point.l.z    = triangle::get_scene_upper_bounds().z;
    }
    else
    {
        entry_point.u.z    = triangle::get_scene_upper_bounds().z;
        entry_point.l.z    = triangle::get_scene_lower_bounds().z;
    }

    /* Clip packet to the world */
    const vfp_t x_bounds    = vfp_t(entry_point.u.x, entry_point.l.x, entry_point.u.x, entry_point.l.x);
    const vfp_t y_bounds    = vfp_t(entry_point.u.y, entry_point.l.y, entry_point.u.y, entry_point.l.y);
    const vfp_t z_bounds    = vfp_t(entry_point.u.z, entry_point.l.z, entry_point.u.z, entry_point.l.z);
    const vfp_t x0          = (x_bounds - f.get_mm_ogn(0)) * f.get_mm_idir(0);
    const vfp_t y0          = (y_bounds - f.get_mm_ogn(1)) * f.get_mm_idir(1);
    const vfp_t z0          = (z_bounds - f.get_mm_ogn(2)) * f.get_mm_idir(2);

    const vfp_t x_zip       = shuffle<2, 3, 0, 1>(x0, x0); 
    const vfp_t y_zip       = shuffle<2, 3, 0, 1>(y0, y0); 
    const vfp_t z_zip       = shuffle<2, 3, 0, 1>(z0, z0); 

    const vfp_t x_entry     = min(x0, x_zip);
    const vfp_t x_exit      = max(x0, x_zip);
    const vfp_t y_entry     = min(y0, y_zip);
    const vfp_t y_exit      = max(y0, y_zip);
    const vfp_t z_entry     = min(z0, z_zip);
    const vfp_t z_exit      = max(z0, z_zip);

    const vfp_t mask        = (y_entry > x_exit) | (x_entry > y_exit) | 
                              (z_entry > x_exit) | (x_entry > z_exit) | 
                              (z_entry > y_exit) | (y_entry > z_exit);

    const int int_mask = move_mask(mask);
    if ((int_mask & 0x3) && (int_mask & 0xc))
    {
        return;
    }
    
    const fp_t near     = min(min(x_entry[0], y_entry[0]), z_entry[0]);
    const fp_t far      = max(max(x_exit[2],  y_exit[2] ), y_exit[2] );
    entry_point.t_max   = far;
    entry_point.t_min   = max((fp_t)0.0, near);
    if (entry_point.t_min > entry_point.t_max)
    {
        return;
    }

    /* Frustrum direction LUT */
    int near_offset[3] = { (f.get_min_x_grad() >= 0)?1:0, (f.get_min_y_grad() >= 0)?1:0, (f.get_min_z_grad() >= 0)?1:0 }; 

    /* Traverse the whole tree */
    fp_t max_d = MAX_DIST;
    while (true)
    {
        /* Find a leaf node */
        bool cmd = this->find_kdt_leaf_node(f, &entry_point, &exit_point, &near_offset[0], size);

        /* If the leaf contains objects find the closest intersecting object */
        if (cmd && !entry_point.n->is_empty())
        {
#ifdef FRUSTRUM_CULLING
            unsigned clipped_size = 0;
#endif
            for (int i = 0; i < size; ++i)
            {
                /* Clip packet to node */
                const vfp_t i_x_grad = inverse(r[i].get_x_grad());
                const vfp_t i_y_grad = inverse(r[i].get_y_grad());
                const vfp_t i_z_grad = inverse(r[i].get_z_grad());
                const vfp_t x_exit   = (entry_point.u.x - r[i].get_x0()) * i_x_grad;
                const vfp_t x_entry  = (entry_point.l.x - r[i].get_x0()) * i_x_grad;
                const vfp_t y_exit   = (entry_point.u.y - r[i].get_y0()) * i_y_grad;
                const vfp_t y_entry  = (entry_point.l.y - r[i].get_y0()) * i_y_grad;
                const vfp_t z_exit   = (entry_point.u.z - r[i].get_z0()) * i_z_grad;
                const vfp_t z_entry  = (entry_point.l.z - r[i].get_z0()) * i_z_grad;

                const vfp_t mask = ((y_entry > x_exit) | (x_entry > y_exit)) | 
                                   ((z_entry > x_exit) | (x_entry > z_exit)) | 
                                   ((z_entry > y_exit) | (y_entry > z_exit));
          
                /* If packet enters leaf then test it */
                if (move_mask(mask) != ((1 << SIMD_WIDTH) - 1))
                {
#ifdef FRUSTRUM_CULLING
                    clipped_r[clipped_size++] = i;
#else
                    entry_point.n->test_leaf_node_nearest(&r[i], &i_o[i << LOG2_SIMD_WIDTH], &h[i]);
#endif
                }
            }
#ifdef FRUSTRUM_CULLING
            if (clipped_size > 0)
            {
                f.adapt_to_leaf(r, entry_point.u, entry_point.l, clipped_r, clipped_size);
                entry_point.n->test_leaf_node_nearest(f, r, i_o, h, clipped_r, clipped_size);
            }
//            else
//            {
//                for (int i = 0; i < clipped_size; ++i)
//                {
//                    entry_point.n->test_leaf_node_nearest(&r[clipped_r[i]], &i_o[clipped_r[i] << LOG2_SIMD_WIDTH], &h[clipped_r[i]]);
//                }
//            }
#endif

            /* Update furthest intersection */
            vfp_t vmax_d = h[0].d;
            for (int i = 1; i < size; ++i)
            {
                vmax_d = max(vmax_d, h[i].d);
            }
            max_d = max(max(vmax_d[0], vmax_d[1]), max(vmax_d[2], vmax_d[3]));

        }
        else
        {
            for (int i = 0; i < size; ++i)
            {
                this->kdt_find_nearest_object(&r[i], &i_o[i << LOG2_SIMD_WIDTH], &h[i], entry_point, exit_point);
            }
        }

        /* Unwind the stack */
        do
        {
            /* If the whole tree has been traversed, return */
            if (exit_point == &(this->kdt_stack[0]))
            {
                return;
            }
            
            entry_point.n       = exit_point->n;
            entry_point.t_max   = exit_point->t_max;
            entry_point.t_min   = exit_point->t_min;
            entry_point.u       = exit_point->u;
            entry_point.l       = exit_point->l;
            exit_point--;
            
        } while (entry_point.t_min >= max_d);
        
        entry_point.t_max = min(entry_point.t_max, max_d);
    }
}


/**********************************************************
 
**********************************************************/
void ray_trace_engine::kdt_frustrum_found_nearer_object(const packet_ray *const r, const vfp_t *t, vfp_t *closer, unsigned size) const
{
#ifdef SPATIAL_SUBDIVISION_STATISTICS
    /* Count number of rays shot */
    ++nr;
#endif
    vfp_t vmax_d = t[0];
    vfp_t vmin_d = t[0];
    packet_hit_description h[MAXIMUM_PACKET_SIZE];
    for (unsigned i = 1; i < size; ++i)
    {
        vmax_d = max(vmax_d, t[i]);
        vmin_d = min(vmin_d, t[i]);
        h[i].d = t[i];
    }
    fp_t max_d = max(max(vmax_d[0], vmax_d[1]), max(vmax_d[2], vmax_d[3]));
    fp_t min_d = min(min(vmin_d[0], vmin_d[1]), min(vmin_d[2], vmin_d[3]));

    /* state of stack */
    kdt_stack_element *exit_point = &(this->kdt_stack[0]);
    exit_point->n       = nullptr;
    exit_point->t_max   = max_d;
    exit_point->t_min   = (fp_t)0.0;

    kdt_stack_element entry_point;
    entry_point.n       = this->kdt_base;
    entry_point.t_max   = max_d;
    entry_point.t_min   = (fp_t)0.0;
    entry_point.s       = nullptr;
    entry_point.d       = 0.0;
    entry_point.m       = vfp_true;
    
//#ifdef SOFT_SHADOW
    frustrum f(r, size);
//#else
//    /* Build a reverse frustrum to traverse */
//    /* The rays are more coherant at the light */
//    frustrum f(r, point_t(r[0].get_dst(0)[0], r[0].get_dst(1)[0], r[0].get_dst(2)[0]), size);
//#endif
    
#ifdef FRUSTRUM_CULLING
    unsigned clipped_r[MAXIMUM_PACKET_SIZE];
#endif

    /* Set the scene bounding box based on frustrum direction */
    if (f.get_min_x_grad() < 0)
    {
        entry_point.u.x    = triangle::get_scene_lower_bounds().x;
        entry_point.l.x    = triangle::get_scene_upper_bounds().x;
    }
    else
    {
        entry_point.u.x    = triangle::get_scene_upper_bounds().x;
        entry_point.l.x    = triangle::get_scene_lower_bounds().x;
    }

    if (f.get_min_y_grad() < 0)
    {
        entry_point.u.y    = triangle::get_scene_lower_bounds().y;
        entry_point.l.y    = triangle::get_scene_upper_bounds().y;
    }
    else
    {
        entry_point.u.y    = triangle::get_scene_upper_bounds().y;
        entry_point.l.y    = triangle::get_scene_lower_bounds().y;
    }

    if (f.get_min_z_grad() < 0)
    {
        entry_point.u.z    = triangle::get_scene_lower_bounds().z;
        entry_point.l.z    = triangle::get_scene_upper_bounds().z;
    }
    else
    {
        entry_point.u.z    = triangle::get_scene_upper_bounds().z;
        entry_point.l.z    = triangle::get_scene_lower_bounds().z;
    }

    /* Frustrum direction LUT */
    int near_offset[3] = { (f.get_min_x_grad() >= 0)?1:0, (f.get_min_y_grad() >= 0)?1:0, (f.get_min_z_grad() >= 0)?1:0 }; 

    /* Traverse the whole tree */
    while (true)
    {
        /* Find a leaf node */
        bool cmd = this->find_kdt_leaf_node(f, &entry_point, &exit_point, &near_offset[0], size);

        /* If the leaf contains objects find the closest intersecting object */
        if (cmd && !entry_point.n->is_empty())
        {
#ifdef FRUSTRUM_CULLING
            unsigned clipped_size = 0;
#endif
            for (unsigned int i = 0; i < size; ++i)
            {
                if (move_mask(closer[i]) == ((1 << SIMD_WIDTH) - 1))
                {
                    continue;
                }

                /* Clip packet to node */
                const vfp_t i_x_grad = inverse(r[i].get_x_grad());
                const vfp_t i_y_grad = inverse(r[i].get_y_grad());
                const vfp_t i_z_grad = inverse(r[i].get_z_grad());
//#ifdef SOFT_SHADOW
                const vfp_t x_exit   = (entry_point.u.x - r[i].get_x0()) * i_x_grad;
                const vfp_t x_entry  = (entry_point.l.x - r[i].get_x0()) * i_x_grad;
                const vfp_t y_exit   = (entry_point.u.y - r[i].get_y0()) * i_y_grad;
                const vfp_t y_entry  = (entry_point.l.y - r[i].get_y0()) * i_y_grad;
                const vfp_t z_exit   = (entry_point.u.z - r[i].get_z0()) * i_z_grad;
                const vfp_t z_entry  = (entry_point.l.z - r[i].get_z0()) * i_z_grad;

//#else
//                const vfp_t x_entry  = (entry_point.u.x - r[i].get_x0()) * i_x_grad;
//                const vfp_t x_exit   = (entry_point.l.x - r[i].get_x0()) * i_x_grad;
//                const vfp_t y_entry  = (entry_point.u.y - r[i].get_y0()) * i_y_grad;
//                const vfp_t y_exit   = (entry_point.l.y - r[i].get_y0()) * i_y_grad;
//                const vfp_t z_entry  = (entry_point.u.z - r[i].get_z0()) * i_z_grad;
//                const vfp_t z_exit   = (entry_point.l.z - r[i].get_z0()) * i_z_grad;
//#endif /* #ifdef SOFT_SHADOW */

                const vfp_t mask = ((y_entry > x_exit) | (x_entry > y_exit)) | 
                                   ((z_entry > x_exit) | (x_entry > z_exit)) | 
                                   ((z_entry > y_exit) | (y_entry > z_exit));
          
                /* If packet enters leaf then test it */
                if (move_mask(mask) != ((1 << SIMD_WIDTH) - 1))
                {
#ifdef FRUSTRUM_CULLING
                    clipped_r[clipped_size++] = i;
#else
                    entry_point.n->test_leaf_node_nearer(&r[i], &closer[i], t[i], &h[i]);
#endif
                }
            }
#ifdef FRUSTRUM_CULLING
            if (clipped_size > 0)
            {
                f.adapt_to_leaf(r, entry_point.u, entry_point.l, clipped_r, clipped_size);
                entry_point.n->test_leaf_node_nearer(f, r, closer, t, h, clipped_r, clipped_size);
            }
//            else
//            {
//                for (int i = 0; i < clipped_size; ++i)
//                {
//                    entry_point.n->test_leaf_node_nearer(&r[clipped_r[i]], &closer[clipped_r[i]], t[clipped_r[i]], &h[clipped_r[i]]);
//                }
//            }
#endif

            /* Update furthest intersection */
            vmax_d = h[0].d;
            for (unsigned i = 1; i < size; ++i)
            {
                vmax_d = max(vmax_d, h[i].d);
            }
            max_d = max(max(vmax_d[0], vmax_d[1]), max(vmax_d[2], vmax_d[3]));
            
            /* Early exit for all rays occluded */
            if (max_d < min_d)
            {
                return;
            }

        }
        else
        {
            for (unsigned i = 0; i < size; ++i)
            {
                entry_point.vt_max  = vfp_t(t[i] - entry_point.t_min);
                entry_point.vt_min  = vfp_t(t[i] - entry_point.t_max);
                closer[i] |= this->kdt_found_nearer_object(&r[i], t[i], entry_point, exit_point);
            }
        }

        /* Unwind the stack */
        do
        {
            /* If the whole tree has been traversed, return */
            if (exit_point == &(this->kdt_stack[0]))
            {
                return;
            }
            
            entry_point.n       = exit_point->n;
            entry_point.t_max   = exit_point->t_max;
            entry_point.t_min   = exit_point->t_min;
            entry_point.u       = exit_point->u;
            entry_point.l       = exit_point->l;
            exit_point--;
            
        } while (entry_point.t_min >= max_d);
        
        entry_point.t_max = min(entry_point.t_max, max_d);
    }
}
#endif /* #ifdef SIMD_PACKET_TRACING */


/**********************************************************
 
**********************************************************/
inline void ray_trace_engine::find_kdt_leaf_node(const ray *const r, const kdt_node **const n, kdt_stack_element **const out, const kdt_stack_element *const entry_point) const
{
    kdt_node *furthest;
    const kdt_node *current_node = *n;
    kdt_stack_element *exit_point  = *out;
    fp_t dist;
    while (true)
    {
        fp_t split_pos = current_node->get_split_position();
#ifdef SPATIAL_SUBDIVISION_STATISTICS
        /* Count nodes accessed */
        ++nts;
#endif


        /* Based on the normal of the plane and which side of the plane
           the ray start search for leaves */
        switch (current_node->get_normal())
        {
            /* This node is not split in any plane, ie/ it is a leaf */
            case axis_t::not_set: 
            {
                *n   = current_node;
                *out = exit_point;
#ifdef SPATIAL_SUBDIVISION_STATISTICS
                /* Count elementary nodes accessed */
                ++nets;
#endif
                return;
            }

            case axis_t::x_axis:
            {
#ifdef SHOW_KD_TREE
                ++x_split_node_crossed;
#endif
                if (entry_point->p.x <= split_pos)
                {
                    if (exit_point->p.x <= split_pos)
                    {
                        current_node = current_node->get_left(); /* cases N1,N2,N3,P5,Z2,Z3 */
                        continue;
                    }
                    /* case N4 */
                    furthest = current_node->get_right();
                    current_node = current_node->get_left();
                }
                else 
                {
                    if (split_pos <= exit_point->p.x)
                    {
                        current_node = current_node->get_right(); /* cases P1,P2,P3,N5,Z1 */
                        continue; 
                    }
                    furthest = current_node->get_left(); /* case P4 */
                    current_node = current_node->get_right();
                }
                /* case N4 or P4 */
                /* Added to the stack what needs checking later */
                dist = (split_pos - r->get_x0()) / r->get_x_grad();
  
                kdt_stack_element *tmp = exit_point;
                if (++exit_point == entry_point)
                {
                    ++exit_point;
                }
  
                exit_point->s   = tmp;
                exit_point->n   = furthest;
                exit_point->d   = dist;
                exit_point->p.x = split_pos;
                exit_point->p.y = r->get_y0() + (dist * r->get_y_grad());
                exit_point->p.z = r->get_z0() + (dist * r->get_z_grad());
                continue;
            }
      
            case axis_t::y_axis:
            {
#ifdef SHOW_KD_TREE
                ++y_split_node_crossed;
#endif
                if (entry_point->p.y <= split_pos)
                {
                    if (exit_point->p.y <= split_pos)
                    {
                        current_node = current_node->get_left(); /* case N1,N2,N3,P5,Z2,Z3 */
                        continue;
                    }
                    /* case N4 */
                    furthest = current_node->get_right();
                    current_node = current_node->get_left();
                }
                else
                {
                    if (split_pos <= exit_point->p.y)
                    {
                        current_node = current_node->get_right(); /* case P1,P2,P3,N5 */
                        continue; 
                    }
                    furthest = current_node->get_left(); /* case P4 */
                    current_node = current_node->get_right();
                }
                /* case N4 or P4 */
                /* Added to the stack what needs checking later */
                dist = (split_pos - r->get_y0()) / r->get_y_grad();
  
                kdt_stack_element *tmp = exit_point;
                if (++exit_point == entry_point)
                {
                    ++exit_point;
                }
                exit_point->s   = tmp;
                exit_point->n   = furthest;
                exit_point->d   = dist;
                exit_point->p.x = r->get_x0() + (dist * r->get_x_grad());
                exit_point->p.y = split_pos;
                exit_point->p.z = r->get_z0() + (dist * r->get_z_grad());
                continue;
            }
      
            case axis_t::z_axis:
            {
#ifdef SHOW_KD_TREE
                ++z_split_node_crossed;
#endif
                if (entry_point->p.z <= split_pos)
                {
                    if (exit_point->p.z <= split_pos)
                    {
                        current_node = current_node->get_left(); /* case N1,N2,N3,P5,Z2,Z3 */
                        continue;
                    }
                    /* case N4 */
                    furthest = current_node->get_right();
                    current_node = current_node->get_left();
                }
                else
                {
                    if (split_pos <= exit_point->p.z)
                    {
                        current_node = current_node->get_right(); /* case P1,P2,P3,N5 */
                        continue;
                    }
                    furthest = current_node->get_left(); /* case P4 */
                    current_node = current_node->get_right();
                }
                /* case N4 or P4 */
                /* Added to the stack what needs checking later */
                dist = (split_pos - r->get_z0()) / r->get_z_grad();
  
                kdt_stack_element *tmp = exit_point;
                if (++exit_point == entry_point)
                {
                    ++exit_point;
                }
                exit_point->s   = tmp;
                exit_point->n   = furthest;
                exit_point->d   = dist;
                exit_point->p.x = r->get_x0() + (dist * r->get_x_grad());
                exit_point->p.y = r->get_y0() + (dist * r->get_y_grad());
                exit_point->p.z = split_pos;
                continue;
            }
        }
    }
}


/**********************************************************
 
**********************************************************/
triangle* ray_trace_engine::kdt_find_nearest_object(const ray *const r, hit_description *const h) const
{
#ifdef SHOW_KD_TREE
    x_split_node_crossed = 0.0;
    y_split_node_crossed = 0.0;
    z_split_node_crossed = 0.0;
#endif

#ifdef SPATIAL_SUBDIVISION_STATISTICS
    /* Count number of rays shot */
    ++nr;
#endif
    
    hit_description nearest_hit;
  
    /* test if the whole BSP tree is missed by the input ray or not */
//    if (!GetMinMaxT(&bbox, r, &tmin, &tmax))
//      return nullptr; /* no object can be intersected */

    const kdt_node *current_node = this->kdt_base; /* start from the root node */

    /* exit point setting */
    kdt_stack_element *exit_point = &(this->kdt_stack[1]);
    exit_point->p.x = r->get_x0() + h->d * r->get_x_grad();
    exit_point->p.y = r->get_y0() + h->d * r->get_y_grad();
    exit_point->p.z = r->get_z0() + h->d * r->get_z_grad();
    exit_point->n = nullptr;
    exit_point->s = nullptr;
    exit_point->d = MAX_DIST;
//    point *extp = &(this->kdt_stack[1]);
//    extp->x = r->get_x0() + r->get_x_grad() * tmax;
//    extp->y = r->get_y0() + r->get_y_grad() * tmax;
//    extp->z = r->get_z0() + r->get_z_grad() * tmax;
//    extp->nodep = nullptr;
//    extp->prev = nullptr;
//    extp->t = tmax;

    /* entry point setting */
    kdt_stack_element *entry_point = &(this->kdt_stack[0]);
    entry_point->p = r->get_ogn();
    entry_point->n = nullptr;
    entry_point->s = nullptr;
    entry_point->d = 0.0;

//    point *entp = &(this->kdt_stack[0]);
//    entp->nodep = nullptr;
//    entp->prev = nullptr;
    /* entry point setting, tmin > 0.0 */
//    if (tmin > 0.0)
//    { /* a ray with external origin */
//      entp->x = r->get_x0() + r->get_x_grad() * tmin;
//      entp->y = r->get_y0() + r->get_y_grad() * tmin;
//      entp->z = r->get_z0() + r->get_z_grad() * tmin;
//      entp->t = tmin;
//    }
//    else
//    { /* a ray with internal origin */
//      entp->x = r->get_x0();
//      entp->y = r->get_y0();
//      entp->z = r->get_z0();
//      entp->t = 0.0;
//    }

    /* Traverse the whole tree */
    while (true)
    {
        /* Find a leaf node */
        this->find_kdt_leaf_node(r, &current_node, &exit_point, entry_point);
//        find_every_leaf_node(r, &current_node, &entry_point, &exit_point);

        /* If the leaf contains objects find the closest intersecting object */
        if (!current_node->is_empty())
        {
            triangle *intersecting_object;
            nearest_hit.d = exit_point->d;
            
            intersecting_object = current_node->test_leaf_node_nearest(r, &nearest_hit, entry_point->d);
            /* If an intersecting object is found it is the closest so return */
            if (intersecting_object != nullptr) 
            {
                *h = nearest_hit;
                return intersecting_object;
            }
        } 
#ifdef SPATIAL_SUBDIVISION_STATISTICS
        /* Count empty elementary nodes accessed */
        else
        {
            ++neets;
        }
#endif
  
        /* Work back up the tree by poping form the stack */
        entry_point  = exit_point;
        current_node = entry_point->n;
      
        /* If the whole tree is traversed without finding an intersection 
           return a nullptr */
        if (current_node == nullptr)
        {
            return nullptr;
        }
  
        exit_point = exit_point->s;
    }
}


/**********************************************************
 
**********************************************************/
bool ray_trace_engine::kdt_found_nearer_object(const ray *const r, const fp_t t) const
{
#ifdef SPATIAL_SUBDIVISION_STATISTICS
    /* Count number of rays shot */
    ++nr;
#endif

    /* signed distances */
    const kdt_node *current_node = this->kdt_base; /* start from the root node */

    /* exit point setting */
    kdt_stack_element *exit_point = &(this->kdt_stack[1]);
    exit_point->p.x = r->get_x0() + t * r->get_x_grad();
    exit_point->p.y = r->get_y0() + t * r->get_y_grad();
    exit_point->p.z = r->get_z0() + t * r->get_z_grad();
    exit_point->n = nullptr;
    exit_point->s = nullptr;
    exit_point->d = MAX_DIST;

    /* entry point setting */
    kdt_stack_element *entry_point = &(this->kdt_stack[0]);
    entry_point->p = r->get_ogn();
    entry_point->n = nullptr;
    entry_point->s = nullptr;
    entry_point->d = 0.0;

    /* Traverse the whole tree */
    while (true)
    {
        /* Find a leaf node */
        this->find_kdt_leaf_node(r, &current_node, &exit_point, entry_point);
  
        /* If the leaf contains objects find the closest intersecting object */
        if (!current_node->is_empty())
        {
            bool closer = current_node->test_leaf_node_nearer(r, t);
            /* If an intersecting object is found it is the closest so return */
            if (closer) 
            {
               return true;
            }
        }
#ifdef SPATIAL_SUBDIVISION_STATISTICS
        /* Count empty elementary nodes accessed */
        else
        {
            ++neets;
        }
#endif
  
        /* Work back up the tree by poping form the stack */
        entry_point  = exit_point;
        current_node = entry_point->n;
      
        /* If the whole tree is traversed without finding an intersection 
           return a null pointer */
        if (current_node == nullptr)
        {
            return false;
        }
  
        exit_point = exit_point->s;
    }
}
#endif /* #ifdef SPATIAL_SUBDIVISION_BIH */


/**********************************************************
 
**********************************************************/
void ray_trace_engine::ray_trace(ray &r, ext_colour_t *const c) const
{
    /* Does the ray intersect any objects */
    /* If the ray intersects multiple objects find the closest */
    const triangle *intersecting_object;
    hit_description hit_type;
    
#ifndef SPATIAL_SUBDIVISION_BIH
    intersecting_object = this->kdt_find_nearest_object(&r, &hit_type);
#else
    intersecting_object = this->bih_find_nearest_object(&r, &hit_type);
#endif /* #ifndef SPATIAL_SUBDIVISION_BIH */

#ifdef SHOW_KD_TREE
    c->r = (fp_t)10.0 * x_split_node_crossed;
    c->g = (fp_t)10.0 * y_split_node_crossed;
    c->b = (fp_t)10.0 * z_split_node_crossed;
    return;        
#endif
 
    /* If there was an intersection set the rays endpoint and call that objects shader */
    if (hit_type.d < MAX_DIST)
    {
        r.calculate_destination(hit_type.d);
        
        this->shader_nr = 0;
        fp_t            nr_reflections = 0.0;
        fp_t            nr_refractions = 0.0;
        ray             reflection_rays[REFLECTION_ARRAY_SIZE];
        ray             refraction_rays[REFLECTION_ARRAY_SIZE];
        ext_colour_t    reflection_colour[REFLECTION_ARRAY_SIZE];
        ext_colour_t    refraction_colour[REFLECTION_ARRAY_SIZE];

        const line n = intersecting_object->generate_rays(*this, r, &hit_type, &reflection_rays[0], &refraction_rays[0], &nr_reflections, &nr_refractions);
        
        /* Trace shadow rays */
        for (unsigned int i = 0; i < this->lights.size(); ++i)
        {
            /* Clear the count of un-occluded rays */
            fp_t made_it = (fp_t)0.0;

            const int ray_addr = (i * (SHADOW_ARRAY_SIZE * MAXIMUM_PACKET_SIZE * SIMD_WIDTH));
            const int shader   = (i * (MAXIMUM_PACKET_SIZE * SIMD_WIDTH));
            for (int l = 0; l < this->nr_pending_shadows[shader]; ++l)
            {
#ifndef SPATIAL_SUBDIVISION_BIH
                if (!this->kdt_found_nearer_object(&this->pending_shadows[ray_addr + l], this->pending_shadows[ray_addr + l].get_length()))
#else
                if (!this->bih_found_nearer_object(&this->pending_shadows[ray_addr + l], this->pending_shadows[ray_addr + l].get_length()))
#endif /* #ifndef SPATIAL_SUBDIVISION_BIH */
                {
                    ++made_it;
                }
            }

            /* Collect the illumintation data */
            if (this->nr_pending_shadows[shader] > 0.0)
            {
                this->pending_shadows[ray_addr].set_magnitude(made_it / this->nr_pending_shadows[shader]);
            }
        }

        /* Call the shader */
        intersecting_object->shade(*this, r, n, hit_type, c);
        
        /* Process secondary rays */
        for (int i = 0; i < nr_reflections; ++i)
        {
            this->ray_trace(reflection_rays[i], &reflection_colour[i]);
        }
        
        for (int i = 0; i < nr_refractions; ++i)
        {
            this->ray_trace(refraction_rays[i], &refraction_colour[i]);
        }

        intersecting_object->combind_secondary_rays(*this, (*c), &reflection_rays[0], &refraction_rays[0], &reflection_colour[0], &refraction_colour[0], &nr_reflections, &nr_refractions);
    }
    /* Otherwise colour with the background colour */
    else
    {
        *c = this->c.shade(r);
    }
}


#ifdef SIMD_PACKET_TRACING
/**********************************************************
 
**********************************************************/
void ray_trace_engine::shoot_shadow_packet(packet_ray *const r, ray *const *const sr, vfp_t *const t, unsigned int *r_to_s, int *m, const int s, const int l) const
{
    /* Coherency check */
    bool    coherant = (s > 4);
    int     size[MAXIMUM_PACKET_SIZE];

    const int pkt_x_dir = move_mask(r[0].get_x_grad());
    const int pkt_y_dir = move_mask(r[0].get_y_grad());
    const int pkt_z_dir = move_mask(r[0].get_z_grad());
    for (int j = 0; j < s; ++j)
    {
        const int x_dir = move_mask(r[j].get_x_grad());
        const int y_dir = move_mask(r[j].get_y_grad());
        const int z_dir = move_mask(r[j].get_z_grad());

        if (((x_dir != 0) && (x_dir != ((1 << SIMD_WIDTH) - 1))) || 
            ((y_dir != 0) && (y_dir != ((1 << SIMD_WIDTH) - 1))) || 
            ((z_dir != 0) && (z_dir != ((1 << SIMD_WIDTH) - 1))))
        {
            coherant = false;
            size[j] = 0;
        }
        else if ((x_dir != pkt_x_dir) || (y_dir != pkt_y_dir) || (z_dir != pkt_z_dir))
        {
            coherant = false;
            size[j] = 1;
        }
        else
        {
            size[j] = 1;
        }
    }

    /* Shoot the most co-herant packet */
    if (!coherant)
    {
        /* Trace single rays */
        for (int j = 0; j < s; ++j)
        {
            if (size[j] == 0)
            {
                for (int k = 0; k < SIMD_WIDTH; ++k)
                {
                    int addr = (j << LOG2_SIMD_WIDTH) + k;
#ifndef SPATIAL_SUBDIVISION_BIH
                    if(!this->kdt_found_nearer_object(sr[addr], sr[addr]->get_length()))
#else
                    if(!this->bih_found_nearer_object(sr[addr], sr[addr]->get_length()))
#endif
                    {
                        m[r_to_s[addr]]++;
                    }  
                }
            }
            else
            {
#ifndef SPATIAL_SUBDIVISION_BIH
                vfp_t closer = this->kdt_found_nearer_object(&r[j], t[j]);
#else
                vfp_t closer = this->bih_found_nearer_object(&r[j], t[j]);
#endif
                for (int k = 0; k < SIMD_WIDTH; ++k)
                {
                    int addr = (j << LOG2_SIMD_WIDTH) + k;
                    m[r_to_s[addr]] += (int)(closer[k] == 0.0);
                }
            }
        }
    }
    else
    {
        vfp_t closer[MAXIMUM_PACKET_SIZE];
        memset(closer, 0, MAXIMUM_PACKET_SIZE * sizeof(vfp_t));

#ifndef SPATIAL_SUBDIVISION_BIH
        this->kdt_frustrum_found_nearer_object(&r[0], &t[0], &closer[0], s);
#else
        this->bih_frustrum_found_nearer_object(&r[0], &t[0], &closer[0], s);
#endif
        for (int k = 0; k < (s << LOG2_SIMD_WIDTH); ++k)
        {
            m[r_to_s[k]] += (int)(closer[k >> LOG2_SIMD_WIDTH][k & 0x3] == 0.0);
        }
    }
    
    return;
}



/**********************************************************
 
**********************************************************/
void ray_trace_engine::ray_trace(packet_ray *const r, ext_colour_t *const c, const unsigned int *const ray_to_colour_lut, const int s) const
{
    const triangle *intersecting_object[MAXIMUM_PACKET_SIZE * SIMD_WIDTH];
    packet_hit_description h[MAXIMUM_PACKET_SIZE];

    /* Check co-herency */
    bool coherant = (s > 4);
    int size[MAXIMUM_PACKET_SIZE];
    const int pkt_x_dir = move_mask(r[0].get_x_grad());
    const int pkt_y_dir = move_mask(r[0].get_y_grad());
    const int pkt_z_dir = move_mask(r[0].get_z_grad());
    
    for (int i = 0; i < s; ++i)
    {
        const int x_dir = move_mask(r[i].get_x_grad());
        const int y_dir = move_mask(r[i].get_y_grad());
        const int z_dir = move_mask(r[i].get_z_grad());
    
        if (((x_dir != 0) && (x_dir != ((1 << SIMD_WIDTH) - 1))) || 
            ((y_dir != 0) && (y_dir != ((1 << SIMD_WIDTH) - 1))) || 
            ((z_dir != 0) && (z_dir != ((1 << SIMD_WIDTH) - 1))))
        {
            coherant = false;
            size[i] = 0;
        }
        else if ((x_dir != pkt_x_dir) || (y_dir != pkt_y_dir) || (z_dir != pkt_z_dir))
        {
            coherant = false;
            size[i] = 1;
        }
        else
        {
            size[i] = 1;
        }
    }
    
    /* Shoot the most co-herant packet */
    if (!coherant)
    {
        /* Trace single rays */
        for (int i = 0; i < s; ++i)
        {
            if (size[i] == 0)
            {
                for (unsigned int j = 0; j < SIMD_WIDTH; ++j)
                {
                    ray             ray_0 = r[i].extract(j);
                    hit_description hit_0 = h[i][j];
#ifndef SPATIAL_SUBDIVISION_BIH
                    intersecting_object[(i * SIMD_WIDTH) + j]  = this->kdt_find_nearest_object(&ray_0, &hit_0);
#else
                    intersecting_object[(i * SIMD_WIDTH) + j]  = this->bih_find_nearest_object(&ray_0, &hit_0);
#endif
                    h[i].d[j] = hit_0.d;
                    h[i].u[j] = hit_0.u;
                    h[i].v[j] = hit_0.v;
                }
            }
            else
            {
#ifndef SPATIAL_SUBDIVISION_BIH
                this->kdt_find_nearest_object(&r[i], &intersecting_object[i << LOG2_SIMD_WIDTH], &h[i]);
#else
                this->bih_find_nearest_object(&r[i], &intersecting_object[i << LOG2_SIMD_WIDTH], &h[i]);
#endif
            }
        }
    }
    else
    {
#ifndef SPATIAL_SUBDIVISION_BIH
        this->kdt_frustrum_find_nearest_object(r, &intersecting_object[0], &h[0], s);
#else
        this->bih_frustrum_find_nearest_object(r, &intersecting_object[0], &h[0], s);
#endif
    }

    /* Generate secondary rays, currently limited to shadow rays */
    line normals[MAXIMUM_PACKET_SIZE << LOG2_SIMD_WIDTH];
    ray ray_p[MAXIMUM_PACKET_SIZE << LOG2_SIMD_WIDTH];
    hit_description hit_p[MAXIMUM_PACKET_SIZE << LOG2_SIMD_WIDTH];

    fp_t            nr_reflections[MAXIMUM_PACKET_SIZE * SIMD_WIDTH];
    fp_t            nr_refractions[MAXIMUM_PACKET_SIZE * SIMD_WIDTH];
    ray             reflection_rays[REFLECTION_ARRAY_SIZE * MAXIMUM_PACKET_SIZE * SIMD_WIDTH];
    ray             refraction_rays[REFLECTION_ARRAY_SIZE * MAXIMUM_PACKET_SIZE * SIMD_WIDTH];
    ext_colour_t    reflection_colour[REFLECTION_ARRAY_SIZE * MAXIMUM_PACKET_SIZE * SIMD_WIDTH];
    ext_colour_t    refraction_colour[REFLECTION_ARRAY_SIZE * MAXIMUM_PACKET_SIZE * SIMD_WIDTH];
    
    memset(nr_reflections, 0, (MAXIMUM_PACKET_SIZE * SIMD_WIDTH * sizeof(fp_t)));
    memset(nr_refractions, 0, (MAXIMUM_PACKET_SIZE * SIMD_WIDTH * sizeof(fp_t)));
    
//    memset(this->nr_pending_shadows, 0, this->lights.size() * (MAXIMUM_PACKET_SIZE * SIMD_WIDTH * sizeof(fp_t)));
    int addr = 0;
    for (int i = 0; i < s; ++i)
    {
        r[i].calculate_destination(h[i].d);
        r[i].extract(&ray_p[i << LOG2_SIMD_WIDTH]);
        h[i].extract(&hit_p[i << LOG2_SIMD_WIDTH]);

        for (int j = 0; j < SIMD_WIDTH; ++j)
        {
            if (hit_p[addr].d < MAX_DIST)
            {
                this->shader_nr = addr;
                int ray_addr = addr * REFLECTION_ARRAY_SIZE;
                normals[addr] = intersecting_object[addr]->generate_rays(*this, ray_p[addr], &hit_p[addr], &reflection_rays[ray_addr], &refraction_rays[ray_addr], &nr_reflections[addr], &nr_refractions[addr]);
            }
            else
            {
                for (unsigned int l = 0; l < this->lights.size(); ++l)
                {
                    const unsigned int nr_addr          = (l * (MAXIMUM_PACKET_SIZE * SIMD_WIDTH)) + addr;
                    this->nr_pending_shadows[nr_addr]   = (fp_t)0.0;
                }
                nr_reflections[addr] = (fp_t)0.0;
                nr_refractions[addr] = (fp_t)0.0;
            }
            
            ++addr;
        }
    }

    
    /* Trace the shadow rays to each light */
    packet_ray      next_packet[MAXIMUM_PACKET_SIZE];
    int             made_it[MAXIMUM_PACKET_SIZE * SIMD_WIDTH];
    ray *           rays_this_packet[MAXIMUM_PACKET_SIZE * SIMD_WIDTH];
    unsigned int    ray_to_shader[MAXIMUM_PACKET_SIZE * SIMD_WIDTH];
    vfp_t           t[MAXIMUM_PACKET_SIZE];
    for (unsigned int i = 0; i < this->lights.size(); ++i)
    {
        memset(made_it, 0, (MAXIMUM_PACKET_SIZE * SIMD_WIDTH) * sizeof(int));

        /* Pack the rays into a packet */
        int packed          = 0;
        int nr_of_packets   = 0;
        for (int k = 0; k < (s << LOG2_SIMD_WIDTH); ++k)
        {
            int ray_addr = (i * (SHADOW_ARRAY_SIZE * MAXIMUM_PACKET_SIZE * SIMD_WIDTH)) + (k * SHADOW_ARRAY_SIZE);
            
            for (int l = 0; l < this->nr_pending_shadows[(i * (MAXIMUM_PACKET_SIZE * SIMD_WIDTH)) + k]; ++l)
            {
                rays_this_packet[(nr_of_packets << LOG2_SIMD_WIDTH) + packed] = &this->pending_shadows[ray_addr];
                ray_to_shader[(nr_of_packets << LOG2_SIMD_WIDTH) + packed] = k;
                ++packed;
                ++ray_addr;
                
                if (packed == SIMD_WIDTH)
                {
                    packed = 0;
                    t[nr_of_packets] = next_packet[nr_of_packets].pack(&rays_this_packet[(nr_of_packets << LOG2_SIMD_WIDTH)]);
                    ++nr_of_packets;
                    
                    if (nr_of_packets == MAXIMUM_PACKET_SIZE)
                    {
                        this->shoot_shadow_packet(next_packet, rays_this_packet, t, ray_to_shader, made_it, nr_of_packets, i);
                        nr_of_packets = 0;
                    }
                }
            }
        }
        
        this->shoot_shadow_packet(next_packet, rays_this_packet, t, ray_to_shader, made_it, nr_of_packets, i);
    
        /* Shoot rays mod SIMD_WIDTH alone */
        for (int k = 0; k < packed; ++k)
        {
            int addr = (nr_of_packets << LOG2_SIMD_WIDTH) + k;
#ifndef SPATIAL_SUBDIVISION_BIH
            if(!this->kdt_found_nearer_object(rays_this_packet[addr], rays_this_packet[addr]->get_length()))
#else
            if(!this->bih_found_nearer_object(rays_this_packet[addr], rays_this_packet[addr]->get_length()))
#endif
            {
                made_it[ray_to_shader[addr]]++;
            }
        }

        
        /* Collect the illumintation data */
        for (int k = 0; k < (s << LOG2_SIMD_WIDTH); ++k)
        {
            const int addr   = (i * (SHADOW_ARRAY_SIZE * MAXIMUM_PACKET_SIZE * SIMD_WIDTH)) + (k * SHADOW_ARRAY_SIZE);
            const int shader = (i * (MAXIMUM_PACKET_SIZE * SIMD_WIDTH)) + k;
            
            if (this->nr_pending_shadows[shader] > 0.0)
            {
                assert(made_it[k] >= 0);
                assert(made_it[k] <= 256);
                this->pending_shadows[addr].set_magnitude(made_it[k] / this->nr_pending_shadows[shader]);
            }
        }
    }
    

    /* If there was an intersection set the rays endpoint and call that objects shader */
    for (int i = 0; i < (s << LOG2_SIMD_WIDTH); ++i)
    {
        const unsigned int addr = ray_to_colour_lut[i];
        this->shader_nr = i;

        if (hit_p[i].d < MAX_DIST)
        {
            intersecting_object[i]->shade(*this, ray_p[i], normals[i], hit_p[i], &c[addr]);
        }
        /* Otherwise colour with the background colour */
        else
        {
            c[addr] = this->c.shade(ray_p[i]);
        }
    }
    

    /* Recurse for reflections */
    /* Pack the rays into a packet */
    int packed          = 0;
    int nr_of_packets   = 0;
    for (int i = 0; i < (s << LOG2_SIMD_WIDTH); ++i)
    {
        int ray_addr = (i * REFLECTION_ARRAY_SIZE);
        for (int j = 0; j < (int)nr_reflections[i]; ++j)
        {
            rays_this_packet[(nr_of_packets << LOG2_SIMD_WIDTH) + packed] = &reflection_rays[ray_addr];
            ray_to_shader[(nr_of_packets << LOG2_SIMD_WIDTH) + packed] = ray_addr;
            ++packed;
            ++ray_addr;
            
            if (packed == SIMD_WIDTH)
            {
                packed = 0;
                next_packet[nr_of_packets].pack(&rays_this_packet[(nr_of_packets << LOG2_SIMD_WIDTH)]);
                ++nr_of_packets;
                
                if (nr_of_packets == MAXIMUM_PACKET_SIZE)
                {
                    this->ray_trace(&next_packet[0], &reflection_colour[0], &ray_to_shader[0], nr_of_packets);
                    nr_of_packets = 0;
                }
            }
        }
    }

    if (nr_of_packets > 0)
    {
        this->ray_trace(&next_packet[0], &reflection_colour[0], &ray_to_shader[0], nr_of_packets);
    }
    
    /* Shoot rays mod SIMD_WIDTH alone */
    for (int i = 0; i < packed; ++i)
    {
        int addr = (nr_of_packets << LOG2_SIMD_WIDTH) + i;
        this->ray_trace(*(rays_this_packet[addr]), &reflection_colour[ray_to_shader[addr]]);        
    }
    
    
    /* Recurse for refractions */
    /* Pack the rays into a packet */
    packed          = 0;
    nr_of_packets   = 0;
    for (int i = 0; i < (s << LOG2_SIMD_WIDTH); ++i)
    {
        int ray_addr = (i * REFLECTION_ARRAY_SIZE);
        for (int j = 0; j < (int)nr_refractions[i]; ++j)
        {
            rays_this_packet[(nr_of_packets << LOG2_SIMD_WIDTH) + packed] = &refraction_rays[ray_addr];
            ray_to_shader[(nr_of_packets << LOG2_SIMD_WIDTH) + packed] = ray_addr;
            ++packed;
            ++ray_addr;
            
            if (packed == SIMD_WIDTH)
            {
                packed = 0;
                next_packet[nr_of_packets].pack(&rays_this_packet[(nr_of_packets << LOG2_SIMD_WIDTH)]);
                ++nr_of_packets;
                
                if (nr_of_packets == MAXIMUM_PACKET_SIZE)
                {
                    this->ray_trace(&next_packet[0], &reflection_colour[0], &ray_to_shader[0], nr_of_packets);
                    nr_of_packets = 0;
                }
            }
        }
    }

    if (nr_of_packets > 0)
    {
        this->ray_trace(&next_packet[0], &refraction_colour[0], &ray_to_shader[0], nr_of_packets);
    }
    
    /* Shoot rays mod SIMD_WIDTH alone */
    for (int i = 0; i < packed; ++i)
    {
        int addr = (nr_of_packets << LOG2_SIMD_WIDTH) + i;
        this->ray_trace(*(rays_this_packet[addr]), &refraction_colour[ray_to_shader[addr]]);        
    }

    /* Combine secondary rays in the shaders */
    for (int i = 0; i < (s << LOG2_SIMD_WIDTH); ++i)
    {
        this->shader_nr = i;
        const unsigned int addr = ray_to_colour_lut[i];

        if (hit_p[i].d < MAX_DIST)
        {
            int ray_addr = i * REFLECTION_ARRAY_SIZE;
            intersecting_object[i]->combind_secondary_rays(*this, c[addr], &reflection_rays[ray_addr], &refraction_rays[ray_addr], &reflection_colour[ray_addr], &refraction_colour[ray_addr], &nr_reflections[i], &nr_refractions[i]);
        }
    }


    /* Done */
    return;
}


/**********************************************************
 
**********************************************************/
void ray_trace_engine::ray_trace_one_packet(const int x, const int y) const
{
    /* Create a packet of rays through the screen */
    packet_ray r[MAXIMUM_PACKET_SIZE];
    this->c.pixel_to_co_ordinate(r, x, y);
    
    /* Ray trace the packet */
    ext_colour_t pixel_colour[MAXIMUM_PACKET_SIZE * SIMD_WIDTH];
    this->ray_trace(r, &pixel_colour[0], packet_ray_to_pixel_lut, MAXIMUM_PACKET_SIZE);
    
    /* Saturate the colour and output */
    for (int i = 0; i < (sqrt(MAXIMUM_PACKET_SIZE * SIMD_WIDTH)); ++i)
    {
        for (int j = 0; j < (sqrt(MAXIMUM_PACKET_SIZE * SIMD_WIDTH)); ++j)
        {
            /* Save output */
            this->c.set_pixel(pixel_colour[j + (i * (unsigned)sqrt(MAXIMUM_PACKET_SIZE * SIMD_WIDTH))], (x + j), (y + i));
        }
    }
}
#endif /* #ifdef SIMD_PACKET_TRACING */


/**********************************************************
 
**********************************************************/
inline void ray_trace_engine::ray_trace_one_pixel(const int x, const int y) const
{
    /* Convert to co-ordinate system */
    point_t ray_dir = this->c.pixel_to_co_ordinate(x, y);
    normalise(&ray_dir);

    /* Create a ray with this information */
    ray ray_0(this->c.camera_position(), ray_dir.x, ray_dir.y, ray_dir.z);

    /* Work on the pixel as a fp_t and then saturate back to an unsigned char */
    ext_colour_t pixel_colour;
    this->ray_trace(ray_0, &pixel_colour);

    /* Saturate colours and save output */
     this->c.set_pixel(pixel_colour, x, y);
}


#ifdef THREADED_RAY_TRACE
/**********************************************************
 
**********************************************************/
void ray_trace_engine::operator() (const blocked_range2d<unsigned>& r) const
{
#ifdef SIMD_PACKET_TRACING
    for (unsigned y = r.rows().begin(); y != r.rows().end(); ++y)
    {     
        for(unsigned x = r.cols().begin(); x != r.cols().end(); ++x)
        {
            this->ray_trace_one_packet((x * PACKET_WIDTH), (y * PACKET_WIDTH));
        }                     
    }

#else
    /* Trace the square block specified in r */
    for (unsigned y = r.rows().begin(); y != r.rows().end(); ++y)
    {     
        for(unsigned x = r.cols().begin(); x != r.cols().end(); ++x)
        {
            this->ray_trace_one_pixel(x, y);
        }                     
    }
#endif /* #ifdef SIMD_PACKET_TRACING */
}
#endif /* #ifdef THREADED_RAY_TRACE */


/**********************************************************
 ray_tracer is the main ray tracing function. 
 
 The whole screen is ray traced based on the X/Y resolution, 
 minimum X/Y values and the X/Y increments. The scene is 
 passed in 'everything' and the light sources passrd in 
 'lights'. 'eye' specifies the launch point of the rays.
 'x_vec', 'y_vec' and 'z_vec'  specify the axis for ray 
 launch. The generated picture is put in camera.
**********************************************************/
void ray_tracer(const light_list &lights, const primitive_list &everything, camera &c)
{
#ifdef THREADED_RAY_TRACE
    /* Start the thread scheduler */
    task_scheduler_init init(task_scheduler_init::automatic);
#endif

    kdt_node kdt_base;
    bih_node *bih_base = nullptr;
#ifndef SPATIAL_SUBDIVISION_BIH
    /* Build a KD-tree to speed up ray tracing */    
    /* The base of the tree will hold everything for now, but adding it will 
      just be a waste of time for now */
    build_kd_tree(&everything, &kdt_base, axis_t::x_axis);
#else
    
    /* Grab an array to hold the BIH */
    /* Maximum theoretical size is everything.size() * 6, but this is very unlikely */
    bih_base = new bih_node [everything.size() * 3];
    const vector<triangle *> *bih_prim = build_bih(&everything, bih_base);
#endif /* #ifndef SPATIAL_SUBDIVISION_BIH */

#ifdef SPATIAL_SUBDIVISION_STATISTICS
    cout << "Static properties of the tree :" << endl;
    cout << "Scene primitves                                             (nsp) : " << everything.size()         << endl;
    cout << "Tree primitves                                              (ntp) : " << ave_ob                    << endl;
    cout << "Number of generic nodes                                     (ng ) : " << ng                        << endl;     
    cout << "Number of elementary nodes                                  (ne ) : " << ne                        << endl;     
    cout << "Number of empty elementary nodes                            (nee) : " << nee                       << endl;     
    cout << "Maximum size of an elementary node                          (ner) : " << ner                       << endl;    
    cout << "Average size of an elementary node                          (nea) : " << ((fp_t)ave_ob/(fp_t)ne)   << endl;  
    cout << "Maximum depth of the tree                                   (d  ) : " << max_depth                 << endl << endl;
    
    ng          = 0;
    ne          = 0;
    nee         = 0;
    ner         = 0;
    ave_ob      = 0;
    max_depth   = 0;
#endif /* #ifdef SPATIAL_SUBDIVISION_STATISTICS */
    
    /* Make the screen 20 wide and 20 high ie/ -10 to 10 */
#ifdef THREADED_RAY_TRACE
#ifdef SIMD_PACKET_TRACING
    parallel_for(blocked_range2d<unsigned>(0, (unsigned)c.y_number_of_rays()/PACKET_WIDTH, 4, 0, (unsigned)c.x_number_of_rays()/PACKET_WIDTH, 4), ray_trace_engine(lights, c, &kdt_base, bih_base));

#else
    /* Thread using blocked_range2d to specify the size and triangle of block to trace */
    parallel_for(blocked_range2d<unsigned>(0, c.y_number_of_rays(), 32, 0, c.x_number_of_rays(), 32), ray_trace_engine(lights, c, &kdt_base, bih_base));

#endif /* #ifdef SIMD_PACKET_TRACING */
#else
    /* Instantiate the ray trace engine */
    ray_trace_engine engine(lights, c, &kdt_base, bih_base);
    
    /* Trace a ray through each pixel of the screen working bottom left to top right */
#ifdef SIMD_PACKET_TRACING
    for(unsigned y = 0; y < c.y_number_of_rays(); y += PACKET_WIDTH)
    {
        for(unsigned x = 0; x < c.x_number_of_rays(); x += PACKET_WIDTH)
        {
            engine.ray_trace_one_packet(x, y);
        }
    }
#else
    for(unsigned y = 0; y < c.y_number_of_rays(); ++y)
    {
        for(unsigned x = 0; x < c.x_number_of_rays(); ++x)
        {
            engine.ray_trace_one_pixel(x, y);
        }
    }
#endif /* #ifdef SIMD_PACKET_TRACING */
#endif /* #ifdef THREADED_RAY_TRACE */


    /* Output dynamc statistics about the scene */
#ifdef SPATIAL_SUBDIVISION_STATISTICS
    cout << "Dynamic properties of the KD-tree :" << endl;
    cout << "Number of rays shot                                       (nr   ) : " << nr                        << endl;
    cout << "Intersection tests performed                              (nit  ) : " << nit                       << endl;
    cout << "Intersection tests performed : to minimum required tests  (ritm ) : " << ((fp_t)nit/(fp_t)ritm)    << endl;
    cout << "Average number of nodes accessed per ray                  (nts  ) : " << ((fp_t)nts/(fp_t)nr)      << endl;
    cout << "Average number of elementary nodes accessed per ray       (nets ) : " << ((fp_t)nets/(fp_t)nr)     << endl;
    cout << "Average number of empty elementary nodes accessed per ray (neets) : " << ((fp_t)neets/(fp_t)nr)    << endl;
#endif

#ifdef SPATIAL_SUBDIVISION_BIH
    /* Clean up */
    delete [] bih_base;
    delete    bih_prim;
#endif /* #ifdef SPATIAL_SUBDIVISION_BIH */
}






///**********************************************************
// 
//**********************************************************/
//void find_every_leaf_node(const ray *const r, const kdt_node **const n, kdt_stack_element **const in, kdt_stack_element **const out)
//{
//    kdt_node *furthest;
//    const kdt_node *current_node = *n;
//    kdt_stack_element *entry_point = *in;
//    kdt_stack_element *exit_point  = *out;
//    while (true)
//    {
//        fp_t split_pos = current_node->get_split_position();
//
//        /* Based on the normal of the plane and which side of the plane
//           the ray start search for leaves */
//        switch (current_node->get_normal())
//        {
//            case axis_t::x_axis :
//            {
//                if (split_pos >= entry_point->p.x)
//                {
//                    furthest = current_node->get_right();
//                    current_node = current_node->get_left();
//                }
//                else
//                {
//                    furthest = current_node->get_left();
//                    current_node = current_node->get_right();
//                }
//
//                fp_t dist = (split_pos - r->get_x0()) / r->get_x_grad();
//                
//                /* Added to the stack what needs checking later */
//                kdt_stack_element *tmp = exit_point;
//                if (++exit_point == entry_point)
//                {
//                    ++exit_point;
//                }
//                
//                exit_point->p.x = split_pos;
//                exit_point->p.y = r->get_y0() + (dist * r->get_y_grad());
//                exit_point->p.z = r->get_z0() + (dist * r->get_z_grad());
//                exit_point->s   = tmp;
//                exit_point->n   = furthest;
//                exit_point->d   = dist;
//                continue;
//            }
//            
//            case axis_t::y_axis :
//            {
//                if (split_pos >= entry_point->p.y)
//                {
//                    furthest = current_node->get_right();
//                    current_node = current_node->get_left();
//                }
//                else
//                {
//                    furthest = current_node->get_left();
//                    current_node = current_node->get_right();
//                }
//
//                fp_t dist = (split_pos - r->get_y0()) / r->get_y_grad();
//                
//                /* Added to the stack what needs checking later */
//                kdt_stack_element *tmp = exit_point;
//                if (++exit_point == entry_point)
//                {
//                    ++exit_point;
//                }
//                
//                exit_point->p.x = r->get_x0() + (dist * r->get_x_grad());
//                exit_point->p.y = split_pos;
//                exit_point->p.z = r->get_z0() + (dist * r->get_z_grad());
//                exit_point->s   = tmp;
//                exit_point->n   = furthest;
//                exit_point->d   = dist;
//                continue;
//            }
//            
//            case axis_t::z_axis :
//            {
//                if (split_pos >= entry_point->p.z)
//                {
//                    furthest = current_node->get_right();
//                    current_node = current_node->get_left();
//                }
//                else
//                {
//                    furthest = current_node->get_left();
//                    current_node = current_node->get_right();
//                }
//
//                fp_t dist = (split_pos - r->get_z0()) / r->get_z_grad();
//                
//                /* Added to the stack what needs checking later */
//                kdt_stack_element *tmp = exit_point;
//                if (++exit_point == entry_point)
//                {
//                    ++exit_point;
//                }
//                
//                exit_point->p.x = r->get_x0() + (dist * r->get_x_grad());
//                exit_point->p.y = r->get_y0() + (dist * r->get_y_grad());
//                exit_point->p.z = split_pos;
//                exit_point->s   = tmp;
//                exit_point->n   = furthest;
//                exit_point->d   = dist;
//                continue;
//            }
//            /* This node is not split in any plane, ie/ it is a leaf */
//            case axis_t::not_set : 
//            {
//                *n   = current_node;
//                *out = exit_point;
//                return;
//            }
//        }
//    }
//}
//
//
//
//
//inline bool ray_trace_engine::find_bih_leaf_node(const ray &r, const point_t ext, const bih_node **const n, 
//                                                    bih_stack_element **const in, bih_stack_element **const out,
//                                                    fp_t *const tmin, fp_t *const tmax) const
//{
//    const point_t ent = r.get_ogn();
//    const bih_node *furthest;
//    const bih_node *current_node = *n;
//    bih_stack_element *entry_point = *in;
//    bih_stack_element *exit_point  = *out;
//    while (true)
//    {
//        const fp_t left_split  = current_node->get_left_split();
//        const fp_t right_split = current_node->get_right_split();
//
//        switch (current_node->get_split_axis())
//        {
//            case axis_t::x_axis :
//            {
//                const fp_t left_dist  = (left_split  - r.get_x0()) / r.get_x_grad();
//                const fp_t right_dist = (right_split - r.get_x0()) / r.get_x_grad();
//
//                /* Must traverse left */
//                if (ent.x < left_split)
//                {
//                    /* No need to traverse right */
//                    if (((ext.x < right_split) || (right_dist > *tmax)) && (ent.x < right_split))
//                    {
//                        current_node = current_node->get_left_child();
//                        continue;
//                    }
//                    /* This is not a strict ordering because left and right can over lap */
//                    furthest = current_node->get_right_child();
//                    current_node = current_node->get_left_child();
//                }
//                /* Must traverse right */
//                else if (ent.x > right_split)
//                {
//                    /* No need to traverse left */
//                    if ((ext.x > left_split) || (left_dist > *tmax))
//                    {
//                        current_node = current_node->get_right_child();
//                        continue;
//                    }
//                    /* This is not a strict ordering because left and right can over lap */
//                    furthest = current_node->get_left_child();
//                    current_node = current_node->get_right_child();
//                }
//                /* Start in empty space */
//                else
//                {
//                    /* Traverse left only */
//                    if ((ext.x < left_split) && (left_dist < *tmax))
//                    {
//                        current_node = current_node->get_left_child();
//                        continue;                   
//                    }
//                    /* Traverse right only */
//                    else if ((ext.x > right_split) && (right_dist < *tmax))
//                    {
//                        current_node = current_node->get_right_child();
//                        continue;                   
//                    }
//                    /* Traverse empty space only */
//                    else
//                    {
//                        return false;
//                    }
//                }
//
//                /* Added to the stack what needs checking later */
//                bih_stack_element *tmp = exit_point;
//                if (++exit_point == entry_point)
//                {
//                    ++exit_point;
//                }
//
//                exit_point->s   = tmp;
//                exit_point->n   = furthest;
//                continue;
//            }
//            
//            case axis_t::y_axis :
//            {
//                const fp_t left_dist  = (left_split  - r.get_y0()) / r.get_y_grad();
//                const fp_t right_dist = (right_split - r.get_y0()) / r.get_y_grad();
//
//                /* Must traverse left */
//                if (ent.y < left_split)
//                {
//                    /* No need to traverse right */
//                    if (((ext.y < right_split) || (right_dist > *tmax)) && (ent.y < right_split))
//                    {
//                        current_node = current_node->get_left_child();
//                        continue;
//                    }
//                    /* This is not a strict ordering because left and right can over lap */
//                    furthest = current_node->get_right_child();
//                    current_node = current_node->get_left_child();
//                }
//                /* Must traverse right */
//                else if (ent.y > right_split)
//                {
//                    /* No need to traverse left */
//                    if ((ext.y > left_split) || (left_dist > *tmax))
//                    {
//                        current_node = current_node->get_right_child();
//                        continue;
//                    }
//                    /* This is not a strict ordering because left and right can over lap */
//                    furthest = current_node->get_left_child();
//                    current_node = current_node->get_right_child();
//                }
//                /* Start in empty space */
//                else
//                {
//                    /* Traverse left only */
//                    if ((ext.y < left_split) && (left_dist < *tmax))
//                    {
//                        current_node = current_node->get_left_child();
//                        continue;                   
//                    }
//                    /* Traverse right only */
//                    else if ((ext.y > right_split) && (right_dist < *tmax))
//                    {
//                        current_node = current_node->get_right_child();
//                        continue;                   
//                    }
//                    /* Traverse empty space only */
//                    else
//                    {
//                        return false;
//                    }
//                }
//
//
//                /* Added to the stack what needs checking later */
//                bih_stack_element *tmp = exit_point;
//                if (++exit_point == entry_point)
//                {
//                    ++exit_point;
//                }
//
//                exit_point->s   = tmp;
//                exit_point->n   = furthest;
//                continue;
//            }
//            
//            case axis_t::z_axis :
//            {
//                const fp_t left_dist  = (left_split  - r.get_z0()) / r.get_z_grad();
//                const fp_t right_dist = (right_split - r.get_z0()) / r.get_z_grad();
//
//                /* Must traverse left */
//                if (ent.z < left_split)
//                {
//                    /* No need to traverse right */
//                    if (((ext.z < right_split) || (right_dist > *tmax)) && (ent.z < right_split))
//                    {
//                        current_node = current_node->get_left_child();
//                        continue;
//                    }
//                    /* This is not a strict ordering because left and right can over lap */
//                    furthest = current_node->get_right_child();
//                    current_node = current_node->get_left_child();
//                }
//                /* Must traverse right */
//                else if (ent.z > right_split)
//                {
//                    /* No need to traverse left */
//                    if ((ext.z > left_split) || (left_dist > *tmax))
//                    {
//                        current_node = current_node->get_right_child();
//                        continue;
//                    }
//                    /* This is not a strict ordering because left and right can over lap */
//                    furthest = current_node->get_left_child();
//                    current_node = current_node->get_right_child();
//                }
//                /* Start in empty space */
//                else
//                {
//                    /* Traverse left only */
//                    if ((ext.z < left_split) && (left_dist < *tmax))
//                    {
//                        current_node = current_node->get_left_child();
//                        continue;                   
//                    }
//                    /* Traverse right only */
//                    else if ((ext.z > right_split) && (right_dist < *tmax))
//                    {
//                        current_node = current_node->get_right_child();
//                        continue;                   
//                    }
//                    /* Traverse empty space only */
//                    else
//                    {
//                        return false;
//                    }
//                }
//
//
//                /* Added to the stack what needs checking later */
//                bih_stack_element *tmp = exit_point;
//                if (++exit_point == entry_point)
//                {
//                    ++exit_point;
//                }
//
//                exit_point->s   = tmp;
//                exit_point->n   = furthest;
//                continue;
//            }
//
//            /* This node is not split in any plane, ie/ it is a leaf */
//            case axis_t::not_set : 
//            {
//                *n   = current_node;
//                *out = exit_point;
//                return true;
//            }
//            
//            /* Traverse every node */
//            default :
//            {
//                furthest = current_node->get_right_child();
//                current_node = current_node->get_left_child();
//
//                /* Added to the stack what needs checking later */
//                bih_stack_element *tmp = exit_point;
//                if (++exit_point == entry_point)
//                {
//                    ++exit_point;
//                }
//
//                exit_point->s   = tmp;
//                exit_point->n   = furthest;
//                continue;            
//            }
//        }
//    }
//}
//
}; /* namespace raptor_raytracer */
