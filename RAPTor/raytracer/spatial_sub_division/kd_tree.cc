/* Standard headers */

/* Boost headers */

/* Comonn heders */

/* Ray tracer headers */
#include "kd_tree.h"


namespace raptor_raytracer
{
    
#ifdef SIMD_PACKET_TRACING
/**********************************************************
 
**********************************************************/
inline void kd_tree::find_leaf_node(const packet_ray *const r, kdt_stack_element **const out, kdt_stack_element *const entry_point, const vfp_t *const i_rd, const int *const near_offset) const
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
    return;
}


/**********************************************************
 
**********************************************************/
void kd_tree::find_nearest_object(const packet_ray *const r, const triangle **const i_o, packet_hit_description *const h) const
{
    /* exit point setting */
    kdt_stack_element *exit_point = &(_kdt_stack[0]);
    exit_point->vt_max  = vfp_t(MAX_DIST);
    exit_point->vt_min  = vfp_zero;
    exit_point->n       = nullptr;
    exit_point->m       = vfp_true;

    /* entry point setting */
    kdt_stack_element entry_point;
    entry_point.vt_max  = vfp_t(MAX_DIST);
    entry_point.vt_min  = vfp_zero;
    entry_point.n       = &(*_kdt_base)[0];
    entry_point.m       = vfp_true;

    /* Take the inverse direction of the ray for faster traversal */
    vfp_t i_rd[3] = { inverse(r->get_x_grad()), inverse(r->get_y_grad()), inverse(r->get_z_grad()) };

    /* Ray direction LUT */
    int near_offset[3] = { (r->get_x_grad()[0] >= 0)?1:0, (r->get_y_grad()[0] >= 0)?1:0, (r->get_z_grad()[0] >= 0)?1:0 }; 

    /* Traverse the whole tree */
    while (true)
    {
        /* Find a leaf node */
        this->find_leaf_node(r, &exit_point, &entry_point, &i_rd[0], &near_offset[0]);

        /* If the leaf contains objects find the closest intersecting object */
        if (!entry_point.n->is_empty())
        {
            entry_point.n->test_leaf_node_nearest(r, i_o, h);
        } 
  
        /* Work back up the tree by poping form the stack */
        do 
        {
            /* If the whole tree is traversed without finding an intersection 
               return a null pointer */
            if (exit_point == &(_kdt_stack[0]))
            {
                return;
            }
      
            entry_point.n       = exit_point->n;
            entry_point.vt_max  = exit_point->vt_max;
            entry_point.vt_min  = exit_point->vt_min;
            entry_point.m       = exit_point->m;
            --exit_point;
        } while (move_mask((entry_point.vt_min >= h->d) & entry_point.m) == ((1 << SIMD_WIDTH) - 1));
    
        entry_point.vt_max = min(entry_point.vt_max, h->d);
    }
}


/**********************************************************
 
**********************************************************/
void kd_tree::find_nearest_object(const packet_ray *const r, const triangle **const i_o, packet_hit_description *const h,
                                                kdt_stack_element entry_point, kdt_stack_element *exit_point) const
{
    kdt_stack_element *const top_of_stack = exit_point;
    entry_point.m = vfp_true;
    
    /* Take the inverse direction of the ray for faster traversal */
    vfp_t i_rd[3] = { inverse(r->get_x_grad()), inverse(r->get_y_grad()), inverse(r->get_z_grad()) };

    /* Ray direction LUT */
    int near_offset[3] = { (r->get_x_grad()[0] >= 0) ? 1 : 0, (r->get_y_grad()[0] >= 0) ? 1 : 0, (r->get_z_grad()[0] >= 0) ? 1 : 0 }; 

    /* Traverse the whole tree */
    while (true)
    {
        /* Find a leaf node */
        this->find_leaf_node(r, &exit_point, &entry_point, &i_rd[0], &near_offset[0]);

        /* If the leaf contains objects find the closest intersecting object */
        if (!entry_point.n->is_empty())
        {
            entry_point.n->test_leaf_node_nearest(r, i_o, h);
        } 
  
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
            --exit_point;
        } while (move_mask((entry_point.vt_min >= h->d) & entry_point.m) == ((1 << SIMD_WIDTH) - 1));
    
        entry_point.vt_max = min(entry_point.vt_max, h->d);
    }
}


/**********************************************************
 
**********************************************************/
vfp_t kd_tree::found_nearer_object(const packet_ray *const r, const vfp_t &t) const
{
    /* exit point setting */
    kdt_stack_element *exit_point = &(_kdt_stack[0]);
    exit_point->vt_max  = t;
    exit_point->vt_min  = vfp_zero;
    exit_point->n       = nullptr;
    exit_point->m       = vfp_true;

    /* entry point setting */
    kdt_stack_element entry_point;
    entry_point.vt_max  = t;
    entry_point.vt_min  = vfp_zero;
    entry_point.n       = &(*_kdt_base)[0];
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
        this->find_leaf_node(r, &exit_point, &entry_point, &i_rd[0], &near_offset[0]);
  
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
  
        /* Work back up the tree by poping form the stack */
        do
        {
            /* If the whole tree is traversed without finding an intersection 
               return a null pointer */
            if (exit_point == &(_kdt_stack[0]))
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
vfp_t kd_tree::found_nearer_object(const packet_ray *const r, const vfp_t &t, kdt_stack_element entry_point, kdt_stack_element *exit_point) const
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
        this->find_leaf_node(r, &exit_point, &entry_point, &i_rd[0], &near_offset[0]);
  
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
inline bool kd_tree::find_leaf_node(const frustrum &r, kdt_stack_element *const entry_point, kdt_stack_element **const out, const int *const near_offset, unsigned size) const
{
    kdt_stack_element *exit_point = *out;
    const kdt_node *current_node = entry_point->n;
    point_t u   = entry_point->u;
    point_t l   = entry_point->l;
    float t_max = entry_point->t_max;
    float t_min = entry_point->t_min;
    
    kdt_node *furthest;
    float min_dist, max_dist;
    while (true)
    {
        float split_pos = current_node->get_split_position();
        switch (current_node->get_normal())
        {
            /* This node is not split in any plane, ie/ it is a leaf */
            case axis_t::not_set : 
            {
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
                    exit_point->t_min   = std::max(min_dist, t_min);
                    exit_point->t_max   = t_max;
                    exit_point->l       = point_t(split_pos, l.y, l.z);
                    exit_point->u       = u;

                    t_max   = std::min(max_dist, t_max);
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
                    exit_point->t_min   = std::max(min_dist, t_min);
                    exit_point->t_max   = t_max;
                    exit_point->l       = point_t(l.x, split_pos, l.z);
                    exit_point->u       = u;

                    t_max   = std::min(max_dist, t_max);
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
                    exit_point->t_min   = std::max(min_dist, t_min);
                    exit_point->t_max   = t_max;
                    exit_point->l       = point_t(l.x, l.y, split_pos);
                    exit_point->u       = u;

                    t_max   = std::min(max_dist, t_max);
                }

                u.z = split_pos;
                continue;
            }
        }
    }
}


/**********************************************************
 
**********************************************************/
void kd_tree::frustrum_find_nearest_object(const packet_ray *const r, const triangle **const i_o, packet_hit_description *const h, int size) const
{
    /* state of stack */
    kdt_stack_element *exit_point = &(_kdt_stack[0]);
    exit_point->n       = nullptr;
    exit_point->t_max   = MAX_DIST;
    exit_point->t_min   = 0.0f;

    kdt_stack_element  entry_point;
    entry_point.n      = &(*_kdt_base)[0];
    entry_point.t_max  = MAX_DIST;
    entry_point.t_min  = 0.0f;
    entry_point.s      = nullptr;
    entry_point.d      = 0.0f;
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
    const vfp_t low_x((vfp_t(entry_point.l.x) - f.get_mm_ogn(0)) * f.get_mm_idir(0));
    const vfp_t high_x((vfp_t(entry_point.u.x) - f.get_mm_ogn(0)) * f.get_mm_idir(0));

    const vfp_t low_y((vfp_t(entry_point.l.y) - f.get_mm_ogn(1)) * f.get_mm_idir(1));
    const vfp_t high_y((vfp_t(entry_point.u.y) - f.get_mm_ogn(1)) * f.get_mm_idir(1));

    const vfp_t low_z((vfp_t(entry_point.l.z) - f.get_mm_ogn(2)) * f.get_mm_idir(2));
    const vfp_t high_z((vfp_t(entry_point.u.z) - f.get_mm_ogn(2)) * f.get_mm_idir(2));

    /* Check intersection for the best corner ray */
    const vfp_t enter_t(max(max(low_x, low_y), max(low_z, vfp_zero)));
    const vfp_t exit_t(min(min(high_x, high_y), high_z));
    entry_point.t_min   = horizontal_min(enter_t);
    entry_point.t_max   = horizontal_max(exit_t);
    if (entry_point.t_min > entry_point.t_max)
    {
        return;
    }

    /* Frustrum direction LUT */
    int near_offset[3] = { (f.get_min_x_grad() >= 0)?1:0, (f.get_min_y_grad() >= 0)?1:0, (f.get_min_z_grad() >= 0)?1:0 }; 

    /* Traverse the whole tree */
    float max_d = MAX_DIST;
    while (true)
    {
        /* Find a leaf node */
        bool cmd = this->find_leaf_node(f, &entry_point, &exit_point, &near_offset[0], size);

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
            
                const vfp_t entry_t = max(max(x_entry, y_entry), max(z_entry, vfp_zero));
                const vfp_t exit_t = min(x_exit, min(x_exit, z_exit));
                const vfp_t mask = entry_t < exit_t;
          
                /* If packet enters leaf then test it */
                if (move_mask(mask) != 0)
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
            max_d = std::max(std::max(vmax_d[0], vmax_d[1]), std::max(vmax_d[2], vmax_d[3]));

        }
        else
        {
            for (int i = 0; i < size; ++i)
            {
                this->find_nearest_object(&r[i], &i_o[i << LOG2_SIMD_WIDTH], &h[i], entry_point, exit_point);
            }
        }

        /* Unwind the stack */
        do
        {
            /* If the whole tree has been traversed, return */
            if (exit_point == &(_kdt_stack[0]))
            {
                return;
            }
            
            entry_point.n       = exit_point->n;
            entry_point.t_max   = exit_point->t_max;
            entry_point.t_min   = exit_point->t_min;
            entry_point.u       = exit_point->u;
            entry_point.l       = exit_point->l;
            --exit_point;
        } while (entry_point.t_min >= max_d);
        
        entry_point.t_max = std::min(entry_point.t_max, max_d);
    }
}


/**********************************************************
 
**********************************************************/
void kd_tree::frustrum_found_nearer_object(const packet_ray *const r, const vfp_t *t, vfp_t *closer, unsigned size) const
{
    vfp_t vmax_d = t[0];
    vfp_t vmin_d = t[0];
    packet_hit_description h[MAXIMUM_PACKET_SIZE];
    for (unsigned i = 1; i < size; ++i)
    {
        vmax_d = max(vmax_d, t[i]);
        vmin_d = min(vmin_d, t[i]);
        h[i].d = t[i];
    }
    float max_d = std::max(std::max(vmax_d[0], vmax_d[1]), std::max(vmax_d[2], vmax_d[3]));
    float min_d = std::min(std::min(vmin_d[0], vmin_d[1]), std::min(vmin_d[2], vmin_d[3]));

    /* state of stack */
    kdt_stack_element *exit_point = &(_kdt_stack[0]);
    exit_point->n       = nullptr;
    exit_point->t_max   = max_d;
    exit_point->t_min   = 0.0f;

    kdt_stack_element entry_point;
    entry_point.n       = &(*_kdt_base)[0];
    entry_point.t_max   = max_d;
    entry_point.t_min   = 0.0f;
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
    int near_offset[3] = { (f.get_min_x_grad() >= 0) ? 1 : 0, (f.get_min_y_grad() >= 0) ? 1 : 0, (f.get_min_z_grad() >= 0) ? 1 : 0 }; 

    /* Traverse the whole tree */
    while (true)
    {
        /* Find a leaf node */
        bool cmd = this->find_leaf_node(f, &entry_point, &exit_point, &near_offset[0], size);

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
            
                const vfp_t entry_t = max(max(x_entry, y_entry), max(z_entry, vfp_zero));
                const vfp_t exit_t = min(x_exit, min(x_exit, z_exit));
                const vfp_t mask = entry_t < exit_t;
          
                /* If packet enters leaf then test it */
                if (move_mask(mask) != 0)
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
            max_d = std::max(std::max(vmax_d[0], vmax_d[1]), std::max(vmax_d[2], vmax_d[3]));
            
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
                closer[i] |= this->found_nearer_object(&r[i], t[i], entry_point, exit_point);
            }
        }

        /* Unwind the stack */
        do
        {
            /* If the whole tree has been traversed, return */
            if (exit_point == &(_kdt_stack[0]))
            {
                return;
            }
            
            entry_point.n       = exit_point->n;
            entry_point.t_max   = exit_point->t_max;
            entry_point.t_min   = exit_point->t_min;
            entry_point.u       = exit_point->u;
            entry_point.l       = exit_point->l;
            --exit_point;
        } while (entry_point.t_min >= max_d);
        
        entry_point.t_max = std::min(entry_point.t_max, max_d);
    }
}
#endif /* #ifdef SIMD_PACKET_TRACING */


/**********************************************************
 
**********************************************************/
inline void kd_tree::find_leaf_node(const ray *const r, const kdt_node **const n, kdt_stack_element **const out, const kdt_stack_element *const entry_point) const
{
    kdt_node *furthest;
    const kdt_node *current_node = *n;
    kdt_stack_element *exit_point  = *out;
    float dist;
    while (true)
    {
        float split_pos = current_node->get_split_position();

        /* Based on the normal of the plane and which side of the plane
           the ray start search for leaves */
        switch (current_node->get_normal())
        {
            /* This node is not split in any plane, ie/ it is a leaf */
            case axis_t::not_set: 
            {
                *n   = current_node;
                *out = exit_point;
                return;
            }

            case axis_t::x_axis:
            {
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
triangle* kd_tree::find_nearest_object(const ray *const r, hit_description *const h) const
{
    hit_description nearest_hit;
  
    /* test if the whole BSP tree is missed by the input ray or not */
//    if (!GetMinMaxT(&bbox, r, &tmin, &tmax))
//      return nullptr; /* no object can be intersected */

    const kdt_node *current_node = &(*_kdt_base)[0]; /* start from the root node */

    /* exit point setting */
    kdt_stack_element *exit_point = &(_kdt_stack[1]);
    exit_point->p.x = r->get_x0() + h->d * r->get_x_grad();
    exit_point->p.y = r->get_y0() + h->d * r->get_y_grad();
    exit_point->p.z = r->get_z0() + h->d * r->get_z_grad();
    exit_point->n = nullptr;
    exit_point->s = nullptr;
    exit_point->d = MAX_DIST;
//    point *extp = &(_kdt_stack[1]);
//    extp->x = r->get_x0() + r->get_x_grad() * tmax;
//    extp->y = r->get_y0() + r->get_y_grad() * tmax;
//    extp->z = r->get_z0() + r->get_z_grad() * tmax;
//    extp->nodep = nullptr;
//    extp->prev = nullptr;
//    extp->t = tmax;

    /* entry point setting */
    kdt_stack_element *entry_point = &(_kdt_stack[0]);
    entry_point->p = r->get_ogn();
    entry_point->n = nullptr;
    entry_point->s = nullptr;
    entry_point->d = 0.0f;

//    point *entp = &(_kdt_stack[0]);
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
        this->find_leaf_node(r, &current_node, &exit_point, entry_point);
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
bool kd_tree::found_nearer_object(const ray *const r, const float t) const
{
    /* Signed distances */
    const kdt_node *current_node = &(*_kdt_base)[0]; /* Start from the root node */

    /* exit point setting */
    kdt_stack_element *exit_point = &(_kdt_stack[1]);
    exit_point->p.x = r->get_x0() + t * r->get_x_grad();
    exit_point->p.y = r->get_y0() + t * r->get_y_grad();
    exit_point->p.z = r->get_z0() + t * r->get_z_grad();
    exit_point->n = nullptr;
    exit_point->s = nullptr;
    exit_point->d = MAX_DIST;

    /* entry point setting */
    kdt_stack_element *entry_point = &(_kdt_stack[0]);
    entry_point->p = r->get_ogn();
    entry_point->n = nullptr;
    entry_point->s = nullptr;
    entry_point->d = 0.0f;

    /* Traverse the whole tree */
    while (true)
    {
        /* Find a leaf node */
        this->find_leaf_node(r, &current_node, &exit_point, entry_point);
  
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
}; /* namespace raptor_raytracer */
