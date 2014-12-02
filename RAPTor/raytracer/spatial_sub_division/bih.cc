/* Standard headers */

/* Boost headers */

/* Comonn heders */

/* Ray tracer headers */
#include "bih.h"


namespace raptor_raytracer
{
std::vector<triangle *> *   bih_node::o     = nullptr;
std::vector<bih_node> *     bih_node::bih   = nullptr;

#ifdef SIMD_PACKET_TRACING
/* Find the next leaf node intersected by the frustrum */
inline int bih::find_leaf_node(const frustrum &r, bih_stack_element *const entry_point, bih_stack_element **const out, unsigned int size) const
{
    bih_stack_element *exit_point = *out;
    const bih_node *current_node = entry_point->n;
    point_t u   = entry_point->u;
    point_t l   = entry_point->l;
    float t_max = entry_point->t_max;
    float t_min = entry_point->t_min;
    while (true)
    {
#ifdef SPATIAL_SUBDIVISION_STATISTICS
        /* Count nodes accessed */
        ++nts;
#endif
        float    near_split         = current_node->get_left_split();
        float    far_split          = current_node->get_right_split();
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
                float min_near_plane;
                float min_far_plane;
                float max_near_plane;
                float max_far_plane;
                if (r.get_max_x_grad() < 0.0f)
                {
                    min_near_plane = (far_split  - r.get_min_x0()) * r.get_min_x_igrad();
                    min_far_plane  = (near_split - r.get_min_x0()) * r.get_min_x_igrad();
                    max_near_plane = (far_split  - r.get_max_x0()) * r.get_max_x_igrad();
                    max_far_plane  = (near_split - r.get_max_x0()) * r.get_max_x_igrad();

                    const float tmp_split = near_split;
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
                    t_min           = std::max(t_min, min_far_plane);
                    current_node    = far_node;
                }
                /* Only the near node is traversed */
                else if (t_max < min_far_plane)
                {
                    u.x             = near_split;
                    t_max           = std::min(t_max, max_near_plane);
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
                    exit_point->t_min   = std::max(t_min, min_far_plane);
        
                    u.x             = near_split;
                    t_max           = std::min(t_max, max_near_plane);
                    current_node    = near_node;
                }
                continue;
            }
    
            case axis_t::y_axis :
            {
                /* Note -- All rays in the frustrum must have the same direction */
                float min_near_plane;
                float min_far_plane;
                float max_near_plane;
                float max_far_plane;
                if (r.get_max_y_grad() < 0.0f)
                {
                    min_near_plane = (far_split  - r.get_min_y0()) * r.get_min_y_igrad();
                    min_far_plane  = (near_split - r.get_min_y0()) * r.get_min_y_igrad();
                    max_near_plane = (far_split  - r.get_max_y0()) * r.get_max_y_igrad();
                    max_far_plane  = (near_split - r.get_max_y0()) * r.get_max_y_igrad();

                    const float tmp_split = near_split;
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
                    t_min           = std::max(t_min, min_far_plane);
                    current_node    = far_node;
                }
                /* Only the near node is traversed */
                else if (t_max < min_far_plane)
                {
                    u.y             = near_split;
                    t_max           = std::min(t_max, max_near_plane);
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
                    exit_point->t_min   = std::max(t_min, min_far_plane);

                    u.y             = near_split;
                    t_max           = std::min(t_max, max_near_plane);
                    current_node    = near_node;
                }
                continue;
            }

            case axis_t::z_axis :
            {
                /* Note -- All rays in the frustrum must have the same direction */
                float min_near_plane;
                float min_far_plane;
                float max_near_plane;
                float max_far_plane;
                if (r.get_max_z_grad() < 0.0f)
                {
                    min_near_plane = (far_split  - r.get_min_z0()) * r.get_min_z_igrad();
                    min_far_plane  = (near_split - r.get_min_z0()) * r.get_min_z_igrad();
                    max_near_plane = (far_split  - r.get_max_z0()) * r.get_max_z_igrad();
                    max_far_plane  = (near_split - r.get_max_z0()) * r.get_max_z_igrad();

                    const float tmp_split = near_split;
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
                    t_min           = std::max(t_min, min_far_plane);
                    current_node    = far_node;
                }
                /* Only the near node is traversed */
                else if (t_max < min_far_plane)
                {
                    u.z             = near_split;
                    t_max           = std::min(t_max, max_near_plane);
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
                    exit_point->t_min   = std::max(t_min, min_far_plane);

                    u.z             = near_split;
                    t_max           = std::min(t_max, max_near_plane);
                    current_node    = near_node;
                }
                continue;
            }
        }
    }
}

/**********************************************************
 
**********************************************************/
void bih::frustrum_find_nearest_object(const packet_ray *const r, const triangle **const i_o, packet_hit_description *const h, int size) const
{
#ifdef SPATIAL_SUBDIVISION_STATISTICS
    /* Count number of rays shot */
    ++nr;
#endif
    /* state of stack */
    bih_stack_element *exit_point = &(this->bih_stack[0]);
    exit_point->n       = nullptr;
    exit_point->t_max   = MAX_DIST;
    exit_point->t_min   = 0.0f;

    bih_stack_element  entry_point;
    entry_point.n      = &this->bih_base[0];
    entry_point.t_max  = MAX_DIST;
    entry_point.t_min  = 0.0f;
    
    /* Build a frustrum to traverse */
    frustrum f(r, size);
    
#ifdef FRUSTRUM_CULLING
    unsigned int clipped_r[MAXIMUM_PACKET_SIZE];
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

    const vfp_t x_entry     = std::min(x0, x_zip);
    const vfp_t x_exit      = std::max(x0, x_zip);
    const vfp_t y_entry     = std::min(y0, y_zip);
    const vfp_t y_exit      = std::max(y0, y_zip);
    const vfp_t z_entry     = std::min(z0, z_zip);
    const vfp_t z_exit      = std::max(z0, z_zip);

    const vfp_t mask        = (y_entry > x_exit) | (x_entry > y_exit) | 
                              (z_entry > x_exit) | (x_entry > z_exit) | 
                              (z_entry > y_exit) | (y_entry > z_exit);

    const int int_mask = move_mask(mask);
    if ((int_mask & 0x3) && (int_mask & 0xc))
    {
        return;
    }
    
    const float near    = std::min(std::min(x_entry[0], y_entry[0]), z_entry[0]);
    const float far     = std::max(std::max(x_exit[2],  y_exit[2] ), y_exit[2] );
    entry_point.t_max   = far;
    entry_point.t_min   = std::max(0.0f, near);
    if (entry_point.t_min > entry_point.t_max)
    {
        return;
    }

    /* Traverse the whole tree */
    float max_d = MAX_DIST;
    while (true)
    {
        /* Find a leaf node */
        const int cmd = this->find_leaf_node(f, &entry_point, &exit_point, size);

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
                vmax_d = std::max(vmax_d, h[i].d);
            }
            max_d = std::max(std::max(vmax_d[0], vmax_d[1]), std::max(vmax_d[2], vmax_d[3]));

        }
        else if (cmd == -1)
        {
//            for (unsigned i = 0; i < size; i += (size/SPLIT_PACKET_DIVISOR))
//            {
//                this->bih_frustrum_find_nearest_object(&r[i], &i_o[i << LOG2_SIMD_WIDTH], &h[i], entry_point, exit_point, size/SPLIT_PACKET_DIVISOR);
//            }
            for (int i = 0; i < size; ++i)
            {
                this->find_nearest_object(&r[i], &i_o[i << LOG2_SIMD_WIDTH], &h[i], entry_point, exit_point);
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
        
        entry_point.t_max = std::min(entry_point.t_max, max_d);
    }
}


/**********************************************************
 
**********************************************************/
void bih::frustrum_found_nearer_object(const packet_ray *const r, const vfp_t *t, vfp_t *closer, unsigned size) const
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
        vmax_d = std::max(vmax_d, t[i]);
        vmin_d = std::min(vmin_d, t[i]);
        h[i].d = t[i];
    }
    float max_d = std::max(std::max(vmax_d[0], vmax_d[1]), std::max(vmax_d[2], vmax_d[3]));
    float min_d = std::min(std::min(vmin_d[0], vmin_d[1]), std::min(vmin_d[2], vmin_d[3]));

    /* state of stack */
    bih_stack_element *exit_point = &(this->bih_stack[0]);
    exit_point->n       = nullptr;
    exit_point->t_max   = max_d;
    exit_point->t_min   = 0.0f;

    bih_stack_element  entry_point;
    entry_point.n      = &this->bih_base[0];
    entry_point.t_max  = max_d;
    entry_point.t_min  = 0.0f;
    
    /* Build a reverse frustrum to traverse */
    /* The rays are more coherant at the light */
#ifdef SOFT_SHADOW
    frustrum f(r, size);
#else
    frustrum f(r, point_t(r[0].get_dst(0)[0], r[0].get_dst(1)[0], r[0].get_dst(2)[0]), size);
#endif
    
#ifdef FRUSTRUM_CULLING
    unsigned int clipped_r[MAXIMUM_PACKET_SIZE];
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
        const int cmd = this->find_leaf_node(f, &entry_point, &exit_point, size);

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
                vmax_d = std::max(vmax_d, h[i].d);
            }
            max_d = std::max(std::max(vmax_d[0], vmax_d[1]), std::max(vmax_d[2], vmax_d[3]));
            
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
                closer[i] |= this->found_nearer_object(&r[i], t[i], entry_point, exit_point);
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
        
        entry_point.t_max = std::min(entry_point.t_max, max_d);
    }
}


/**********************************************************
 
**********************************************************/
inline bool bih::find_leaf_node(const packet_ray &r, bih_stack_element *const entry_point, bih_stack_element **const out, const vfp_t *const i_rd) const
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
                    t_min           = std::max(t_min, far_plane);
                    current_node    = far_node;
                }
                /* Only the near node is traversed */
                else if (!o_near)
                {
                    t_max           = std::min(t_max, near_plane);
                    current_node    = near_node;
                }
                /* Both nodes are traversed, near node first */
                else
                {
                    /* Stack the far node */
                    ++exit_point;
                    exit_point->n       = far_node;
                    exit_point->vt_max  = t_max;
                    exit_point->vt_min  = std::max(t_min, far_plane);

                    t_max           = std::min(t_max, near_plane);
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
                    t_min           = std::max(t_min, far_plane);
                    current_node    = far_node;
                }
                /* Only the near node is traversed */
                else if (!o_near)
                {
                    t_max           = std::min(t_max, near_plane);
                    current_node    = near_node;
                }
                /* Both nodes are traversed, near node first */
                else
                {
                    /* Stack the far node */
                    ++exit_point;
                    exit_point->n       = far_node;
                    exit_point->vt_max  = t_max;
                    exit_point->vt_min  = std::max(t_min, far_plane);

                    t_max           = std::min(t_max, near_plane);
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
                    t_min           = std::max(t_min, far_plane);
                    current_node    = far_node;
                }
                /* Only the near node is traversed */
                else if (!o_near)
                {
                    t_max           = std::min(t_max, near_plane);
                    current_node    = near_node;
                }
                /* Both nodes are traversed, near node first */
                else
                {
                    /* Stack the far node */
                    ++exit_point;
                    exit_point->n       = far_node;
                    exit_point->vt_max  = t_max;
                    exit_point->vt_min  = std::max(t_min, far_plane);

                    t_max           = std::min(t_max, near_plane);
                    current_node    = near_node;
                } 
                continue;
            }
        }
    }
}


/**********************************************************
 
**********************************************************/
void bih::find_nearest_object(const packet_ray *const r, const triangle **const i_o, packet_hit_description *const h) const
{
#ifdef SPATIAL_SUBDIVISION_STATISTICS
    /* Count number of rays shot */
    ++nr;
#endif
    /* state of stack */
    bih_stack_element *exit_point = &(this->bih_stack[0]);

    bih_stack_element   entry_point;
    entry_point.n       = &this->bih_base[0];
    entry_point.vt_max  = vfp_t(MAX_DIST);
    entry_point.vt_min  = vfp_t(0.0f);
    
    /* Take the inverse direction of the ray for faster traversal */
    vfp_t i_rd[3] = { inverse(r->get_x_grad()), inverse(r->get_y_grad()), inverse(r->get_z_grad()) };

    /* Traverse the whole tree */
    while (true)
    {
        /* Find a leaf node */
        const bool cmd = this->find_leaf_node(*r, &entry_point, &exit_point, &i_rd[0]);

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
                
        entry_point.vt_max = std::min(entry_point.vt_max, h->d);
    }
}


/**********************************************************
 
**********************************************************/
void bih::find_nearest_object(const packet_ray *const r, const triangle **const i_o, packet_hit_description *const h,
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
        const bool cmd = this->find_leaf_node(*r, &entry_point, &exit_point, &i_rd[0]);

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
                
        entry_point.vt_max = std::min(entry_point.vt_max, h->d);
    }
}


/**********************************************************
 
**********************************************************/
vfp_t bih::found_nearer_object(const packet_ray *const r, const vfp_t &t, bih_stack_element entry_point, bih_stack_element *exit_point) const
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
        const bool cmd = this->find_leaf_node(*r, &entry_point, &exit_point, &i_rd[0]);

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
                
        entry_point.vt_max = std::min(entry_point.vt_max, h.d);
    }
}
#endif /* #ifdef SIMD_PACKET_TRACING */


/**********************************************************

**********************************************************/
vfp_t bih::found_nearer_object(const packet_ray *const r, const vfp_t &t) const
{
#ifdef SPATIAL_SUBDIVISION_STATISTICS
    /* Count number of rays shot */
    ++nr;
#endif
    /* state of stack */
    bih_stack_element *exit_point = &(this->bih_stack[0]);

    bih_stack_element   entry_point;
    entry_point.n       = &this->bih_base[0];
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
        const bool cmd = this->find_leaf_node(*r, &entry_point, &exit_point, &i_rd[0]);

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
                
        entry_point.vt_max = std::min(entry_point.vt_max, h.d);
    }
}


/**********************************************************
 
**********************************************************/
inline bool bih::find_leaf_node(const ray &r, bih_stack_element *const entry_point, bih_stack_element **const out, const point_t &i_rd) const
{
    bih_stack_element *exit_point = *out;
    const bih_node *current_node = entry_point->n;
    float t_max = entry_point->t_max;
    float t_min = entry_point->t_min;
    while (true)
    {
#ifdef SPATIAL_SUBDIVISION_STATISTICS
        /* Count nodes accessed */
        ++nts;
#endif
        float near_split            = current_node->get_left_split();
        float far_split             = current_node->get_right_split();
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
                float near_plane = (near_split - r.get_x0()) * i_rd.x;
                float far_plane  = (far_split  - r.get_x0()) * i_rd.x;
                if (r.get_x_grad() < 0.0f)
                {
                   const float tmp_plane = near_plane;
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
                    t_min           = std::max(t_min, far_plane);
                    current_node    = far_node;
                }
                /* Only the near node is traversed */
                else if (t_max < far_plane)
                {
                    t_max           = std::min(t_max, near_plane);
                    current_node    = near_node;
                }
                /* Both nodes are traversed, near node first */
                else
                {
                    /* Stack the far node */
                    ++exit_point;
                    exit_point->n       = far_node;
                    exit_point->t_max   = t_max;
                    exit_point->t_min   = std::max(t_min, far_plane);

                    t_max           = std::min(t_max, near_plane);
                    current_node    = near_node;
                }
                continue;
            }
    
            case axis_t::y_axis :
            {
                float near_plane = (near_split - r.get_y0()) * i_rd.y;
                float far_plane  = (far_split  - r.get_y0()) * i_rd.y;
                if (r.get_y_grad() < 0.0f)
                {
                   const float tmp_plane = near_plane;
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
                    t_min           = std::max(t_min, far_plane);
                    current_node    = far_node;
                }
                /* Only the near node is traversed */
                else if (t_max < far_plane)
                {
                    t_max           = std::min(t_max, near_plane);
                    current_node    = near_node;
                }
                /* Both nodes are traversed, near node first */
                else
                {
                    /* Stack the far node */
                    ++exit_point;
                    exit_point->n       = far_node;
                    exit_point->t_max   = t_max;
                    exit_point->t_min   = std::max(t_min, far_plane);

                    t_max           = std::min(t_max, near_plane);
                    current_node    = near_node;
                } 
                continue;
            }

            case axis_t::z_axis :
            {
                float near_plane = (near_split - r.get_z0()) * i_rd.z;
                float far_plane  = (far_split  - r.get_z0()) * i_rd.z;
                if (r.get_z_grad() < 0.0f)
                {
                   const float tmp_plane = near_plane;
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
                    t_min           = std::max(t_min, far_plane);
                    current_node    = far_node;
                }
                /* Only the near node is traversed */
                else if (t_max < far_plane)
                {
                    t_max           = std::min(t_max, near_plane);
                    current_node    = near_node;
                }
                /* Both nodes are traversed, near node first */
                else
                {
                    /* Stack the far node */
                    ++exit_point;
                    exit_point->n       = far_node;
                    exit_point->t_max   = t_max;
                    exit_point->t_min   = std::max(t_min, far_plane);

                    t_max           = std::min(t_max, near_plane);
                    current_node    = near_node;
                } 
                continue;
            }
        }
    }
}


/**********************************************************
 
**********************************************************/
triangle* bih::find_nearest_object(const ray *const r, hit_description *const h) const
{
#ifdef SPATIAL_SUBDIVISION_STATISTICS
    /* Count number of rays shot */
    ++nr;
#endif
    
    /* state of stack */
    bih_stack_element *exit_point = &(this->bih_stack[0]);
    exit_point->n       = nullptr;
    exit_point->t_max   = MAX_DIST;
    exit_point->t_min   = 0.0f;

    bih_stack_element   entry_point;
    entry_point.n       = &this->bih_base[0];
    entry_point.t_max   = MAX_DIST;
    entry_point.t_min   = 0.0;

    /* Set the scene bounding box based on ray direction */
    if (r->get_x_grad() >= 0.0f)
    {
        entry_point.u.x    = triangle::get_scene_lower_bounds().x;
        entry_point.l.x    = triangle::get_scene_upper_bounds().x;
    }
    else
    {
        entry_point.u.x    = triangle::get_scene_upper_bounds().x;
        entry_point.l.x    = triangle::get_scene_lower_bounds().x;
    }

    if (r->get_y_grad() >= 0.0f)
    {
        entry_point.u.y    = triangle::get_scene_lower_bounds().y;
        entry_point.l.y    = triangle::get_scene_upper_bounds().y;
    }
    else
    {
        entry_point.u.y    = triangle::get_scene_upper_bounds().y;
        entry_point.l.y    = triangle::get_scene_lower_bounds().y;
    }

    if (r->get_z_grad() >= 0.0f)
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
    const float i_x_grad = 1.0f / r->get_x_grad();
    const float i_y_grad = 1.0f / r->get_y_grad();
    const float i_z_grad = 1.0f / r->get_z_grad();
    const float x_entry  = (entry_point.u.x - r->get_x0()) * i_x_grad;
    const float x_exit   = (entry_point.l.x - r->get_x0()) * i_x_grad;
    const float y_entry  = (entry_point.u.y - r->get_y0()) * i_y_grad;
    const float y_exit   = (entry_point.l.y - r->get_y0()) * i_y_grad;
    const float z_entry  = (entry_point.u.z - r->get_z0()) * i_z_grad;
    const float z_exit   = (entry_point.l.z - r->get_z0()) * i_z_grad;

    if ((y_entry > x_exit) || (x_entry > y_exit) || (z_entry > x_exit) || (x_entry > z_exit) || (z_entry > y_exit) || (y_entry > z_exit))
    {
        return nullptr;
    }
    
    const float near    = std::min(std::min(x_entry, y_entry), z_entry);
    const float far     = std::max(std::max(x_exit,  y_exit ), z_exit );
    entry_point.t_max   = far;
    entry_point.t_min   = std::max(0.0f, near);
    
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
        const bool leaf = this->find_leaf_node(*r, &entry_point, &exit_point, i_ray_dir);

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
        
        entry_point.t_max = std::min(entry_point.t_max, nearest_hit.d);
    }
}


/**********************************************************
 
**********************************************************/
bool bih::found_nearer_object(const ray *const r, const float t) const
{
#ifdef SPATIAL_SUBDIVISION_STATISTICS
    /* Count number of rays shot */
    ++nr;
#endif

    /* state of stack */
    bih_stack_element *exit_point = &(this->bih_stack[0]);
    exit_point->n       = nullptr;
    exit_point->t_max   = t;
    exit_point->t_min   = 0.0f;

    bih_stack_element   entry_point;
    entry_point.n       = &this->bih_base[0];
    entry_point.t_max   = t;
    entry_point.t_min   = 0.0f;

    /* Set the scene bounding box based on ray direction */
    if (r->get_x_grad() >= 0.0f)
    {
        entry_point.u.x    = triangle::get_scene_lower_bounds().x;
        entry_point.l.x    = triangle::get_scene_upper_bounds().x;
    }
    else
    {
        entry_point.u.x    = triangle::get_scene_upper_bounds().x;
        entry_point.l.x    = triangle::get_scene_lower_bounds().x;
    }

    if (r->get_y_grad() >= 0.0f)
    {
        entry_point.u.y    = triangle::get_scene_lower_bounds().y;
        entry_point.l.y    = triangle::get_scene_upper_bounds().y;
    }
    else
    {
        entry_point.u.y    = triangle::get_scene_upper_bounds().y;
        entry_point.l.y    = triangle::get_scene_lower_bounds().y;
    }

    if (r->get_z_grad() >= 0.0f)
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
    const float i_x_grad = 1.0f / r->get_x_grad();
    const float i_y_grad = 1.0f / r->get_y_grad();
    const float i_z_grad = 1.0f / r->get_z_grad();
    const float x_entry  = (entry_point.u.x - r->get_x0()) * i_x_grad;
    const float x_exit   = (entry_point.l.x - r->get_x0()) * i_x_grad;
    const float y_entry  = (entry_point.u.y - r->get_y0()) * i_y_grad;
    const float y_exit   = (entry_point.l.y - r->get_y0()) * i_y_grad;
    const float z_entry  = (entry_point.u.z - r->get_z0()) * i_z_grad;
    const float z_exit   = (entry_point.l.z - r->get_z0()) * i_z_grad;

    if ((y_entry > x_exit) || (x_entry > y_exit) || 
        (z_entry > x_exit) || (x_entry > z_exit) || 
        (z_entry > y_exit) || (y_entry > z_exit))
    {
        return false;
    }
    
    const float near    = std::min(std::min(x_entry, y_entry), z_entry);
    const float far     = std::max(std::max(x_exit,  y_exit ), z_exit );
    entry_point.t_max   = far;
    entry_point.t_min   = std::max(0.0f, near);
    
    if (entry_point.t_min > entry_point.t_max)
    {
        return false;
    }
    
    point_t i_ray_dir(i_x_grad, i_y_grad, i_z_grad);

    /* Traverse the whole tree */
    while (true)
    {
        /* Find a leaf node */
        const bool leaf = this->find_leaf_node(*r, &entry_point, &exit_point, i_ray_dir);
        
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
}; /* namespace raptor_raytracer*/
