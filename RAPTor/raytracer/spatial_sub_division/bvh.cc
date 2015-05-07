/* Standard headers */
#include <fstream>

/* Boost headers */

/* Comonn heders */
#include "logging.h"

/* Ray tracer headers */
#include "bvh.h"


namespace raptor_raytracer
{
std::vector<triangle *> *bvh_node::o = nullptr;

#ifdef SIMD_PACKET_TRACING
/* Find the next leaf node intersected by the frustrum */
inline int bvh::find_leaf_node(const frustrum &r, bvh_stack_element *const entry_point, bvh_stack_element **const out) const
{
    /* Get current state */
    bvh_stack_element *exit_point = *out;
    int cur_idx = entry_point->idx;

    while (true)
    {
        if ((*_bvh_base)[cur_idx].is_leaf())
        {
            /* This node is a leaf, update the state and return */
            entry_point->idx = cur_idx;
            *out = exit_point;
            // BOOST_LOG_TRIVIAL(trace) << "Found leaf at index: " << entry_point->idx;
            return 1;
        }
        else
        {
            /* Find distance to the children */
            const int idx_0 = (*_bvh_base)[cur_idx].left_index();
            const int idx_1 = (*_bvh_base)[cur_idx].right_index();
            const bool cull_0 = r.cull((*_bvh_base)[idx_0].low_point(), (*_bvh_base)[idx_0].high_point());
            const bool cull_1 = r.cull((*_bvh_base)[idx_1].low_point(), (*_bvh_base)[idx_1].high_point());

            /* We never make it to these node */
            if (cull_0 && cull_1)
            {   
               *out = exit_point;
                return 0;
            }

            /* Only node 1 to traverse */
            if (cull_0)
            {
                cur_idx = idx_1;
                // BOOST_LOG_TRIVIAL(trace) << "Traversing near node only";
            }
            /* Only node 0 to traverse */
            else if (cull_1)
            {
                cur_idx = idx_0;
            }
            /* Both nodes are traversed */
            else
            {
                /* Stack the far node */
                assert((exit_point - &_bvh_stack[0]) < MAX_BVH_STACK_HEIGHT);
                ++exit_point;
                exit_point->idx = idx_1;
                cur_idx = idx_0;
                // BOOST_LOG_TRIVIAL(trace) << "Traversing both nodes : " << idx_0 << " then: " << idx_1;
            }
        }
    }
}

void bvh::frustrum_find_nearest_object(const packet_ray *const r, const triangle **const i_o, packet_hit_description *const h, int size) const
{
#if 1
    for (int i = 0; i < size; ++i)
    {
        find_nearest_object(&r[i], &i_o[i << LOG2_SIMD_WIDTH], &h[i]);
    }

    return;
#endif

    unsigned int clipped_r[MAXIMUM_PACKET_SIZE];
    for (int i = 0; i < size; ++i)
    {
        clipped_r[i] = i;
    }

    for (int i = size; i < MAXIMUM_PACKET_SIZE; ++i)
    {
        h[i].d = vfp_t(MAX_DIST);
    }

    /* state of stack */
    bvh_stack_element *exit_point = &(_bvh_stack[0]);
    exit_point->idx     = _root_node;
    exit_point->t_max   = MAX_DIST;
    exit_point->t_min   = 0.0f;

    bvh_stack_element  entry_point;
    entry_point.idx    = _root_node;
    entry_point.t_max  = MAX_DIST;
    entry_point.t_min  = 0.0f;
    
    /* Build a frustrum to traverse */
    frustrum f(r, size);
    f.adapt_to_leaf(r, triangle::get_scene_upper_bounds(), triangle::get_scene_lower_bounds(), clipped_r, size);
    
    /* Traverse the whole tree */
    while (true)
    {
        /* Find a leaf node */
        const int cmd = this->find_leaf_node(f, &entry_point, &exit_point);

        /* If the leaf contains objects find the closest intersecting object */
        if (cmd == 1)
        {
            unsigned clipped_size = 0;
            const auto &leaf = (*_bvh_base)[entry_point.idx];
            for (int i = 0; i < size; ++i)
            {
                /* Clip packet to node */
                vfp_t i_rd[3] = { 1.0f / r[i].get_x_grad(), 1.0f / r[i].get_y_grad(), 1.0f / r[i].get_z_grad() };
                const vfp_t mask(leaf.intersection_distance(r[i], i_rd) < vfp_t(MAX_DIST));
          
                /* If packet enters leaf then test it */
                if (move_mask(mask) != 0x0)
                {
                    clipped_r[clipped_size++] = i;
                }
            }

            if (clipped_size > 0)
            {
                // f.adapt_to_leaf(r, leaf.high_point(), leaf.low_point(), clipped_r, clipped_size);
                leaf.test_leaf_node_nearest(f, r, i_o, h, clipped_r, clipped_size);
            }
        }

        /* If the whole tree has been traversed, return */
        if (exit_point == &(_bvh_stack[0]))
        {
            return;
        }
        
        /* Unwind the stack without distance culling */
        entry_point.idx = exit_point->idx;
        --exit_point;
    }
}


void bvh::frustrum_found_nearer_object(const packet_ray *const r, const vfp_t *t, vfp_t *closer, const unsigned int size) const
{
#if 1
    for (int i = 0; i < static_cast<int>(size); ++i)
    {
        closer[i] = found_nearer_object(&r[i], t[i]);
    }

    return;
#endif

    unsigned int clipped_r[MAXIMUM_PACKET_SIZE];
    clipped_r[0] = 0;
 
    vfp_t vmin_d(t[0]);
    packet_hit_description h[MAXIMUM_PACKET_SIZE];
    for (unsigned i = 1; i < size; ++i)
    {
        clipped_r[i] = i;
        vmin_d = min(vmin_d, t[i]);
        h[i].d = t[i];
    }
    float min_d = horizontal_min(vmin_d);

    for (int i = size; i < MAXIMUM_PACKET_SIZE; ++i)
    {
        h[i].d = vfp_zero;
    }

    /* state of stack */
    bvh_stack_element *exit_point = &(_bvh_stack[0]);
    exit_point->idx     = _root_node;
    exit_point->t_min   = MAX_DIST;

    bvh_stack_element  entry_point;
    entry_point.idx    = _root_node;
    entry_point.t_min  = MAX_DIST;
    
    /* Build a reverse frustrum to traverse */
    /* The rays are more coherant at the light */
#ifdef SOFT_SHADOW
    frustrum f(r, size);
#else
    frustrum f(r, point_t(r[0].get_dst(0)[0], r[0].get_dst(1)[0], r[0].get_dst(2)[0]), size);
#endif
    f.adapt_to_leaf(r, triangle::get_scene_upper_bounds(), triangle::get_scene_lower_bounds(), clipped_r, size);

    /* Traverse the whole tree */
    while (true)
    {
        /* Find a leaf node */
        const int cmd = this->find_leaf_node(f, &entry_point, &exit_point);

        /* If the leaf contains objects find the closest intersecting object */
        if (cmd == 1)
        {
            unsigned clipped_size = 0;
            const auto &leaf = (*_bvh_base)[entry_point.idx];
            for (unsigned int i = 0; i < size; ++i)
            {
                if (move_mask(closer[i]) == ((1 << SIMD_WIDTH) - 1))
                {
                    continue;
                }

                /* Clip packet to node */
                vfp_t i_rd[3] = { 1.0f / r[i].get_x_grad(), 1.0f / r[i].get_y_grad(), 1.0f / r[i].get_z_grad() };
                const vfp_t mask(leaf.intersection_distance(r[i], i_rd) < vfp_t(MAX_DIST));

                /* If packet enters leaf then test it */
                if (move_mask(mask) != 0x0)
                {
                    clipped_r[clipped_size++] = i;
                }
            }

            if (clipped_size > 0)
            {
                // f.adapt_to_leaf(r, leaf.high_point(), leaf.low_point(), clipped_r, clipped_size);
                leaf.test_leaf_node_nearer(f, r, closer, t, h, clipped_r, clipped_size);
            }

            /* Update furthest intersection */
            static_assert(MAXIMUM_PACKET_SIZE == 16, "Error: Max expects MAXIMUM_PACKET_SIZE to be 16");
            const vfp_t vmax_d(max(max(max(max(h[0].d, h[1].d), max(h[ 2].d, h[ 3].d)), max(max(h[ 4].d, h[ 5].d), max(h[ 6].d, h[ 7].d))), 
                              max(max(max(h[8].d, h[9].d), max(h[10].d, h[11].d)), max(max(h[12].d, h[13].d), max(h[14].d, h[15].d)))));
            const float max_d = horizontal_max(vmax_d);
            
            /* Early exit for all rays occluded */
            if (max_d < min_d)
            {
                return;
            }
        }

        /* If the whole tree has been traversed, return */
        if (exit_point == &(_bvh_stack[0]))
        {
            return;
        }
        
        /* Unwind the stack without distance culling */
        entry_point.idx     = exit_point->idx;
        --exit_point;
    }
}

inline bool bvh::find_leaf_node(const packet_ray &r, bvh_stack_element *const entry_point, bvh_stack_element **const out, const vfp_t *const i_rd, const vfp_t &t_max) const
{
    /* Get current state */
    bvh_stack_element *exit_point = *out;
    int cur_idx = entry_point->idx;

    while (true)
    {
        if ((*_bvh_base)[cur_idx].is_leaf())
        {
            /* This node is a leaf, update the state and return */
            entry_point->idx = cur_idx;
            *out = exit_point;
            // BOOST_LOG_TRIVIAL(trace) << "Found leaf at index: " << entry_point->idx;
            return true;
        }
        else
        {
            /* Find distance to the children */
            int near_idx    = (*_bvh_base)[cur_idx].left_index();
            int far_idx     = (*_bvh_base)[cur_idx].right_index();
            const vfp_t dist_0((*_bvh_base)[near_idx].intersection_distance(r, i_rd));
            const vfp_t dist_1((*_bvh_base)[far_idx].intersection_distance(r, i_rd));

            /* Order the children */
            const vfp_t far_dist(max(dist_0, dist_1));
            const vfp_t near_dist(min(dist_0, dist_1));

            /* We never make it to these node */
            const int o_near = move_mask(near_dist < t_max);
            if (!o_near)
            {   
               *out = exit_point;
                return false;
            }

            /* The far node is too far away to traverse */
            const int o_same_order = move_mask(dist_0 < dist_1);
            const int o_far = move_mask(far_dist < t_max);
            if (!o_far && (o_same_order == 0xf))
            {
                cur_idx = near_idx;
                // BOOST_LOG_TRIVIAL(trace) << "Traversing near node only";
            }
            else if (!o_far && (o_same_order == 0x0))
            {
                cur_idx = far_idx;
            }
            /* Both nodes are traversed, near node first */
            else
            {
                /* Stack the far node */
                assert((exit_point - &_bvh_stack[0]) < MAX_BVH_STACK_HEIGHT);
                ++exit_point;
                exit_point->idx     = far_idx;
                exit_point->vt_min  = dist_1;//far_dist;
                cur_idx = near_idx;
                // BOOST_LOG_TRIVIAL(trace) << "Traversing both nodes : " << near_idx << " then: " << far_idx;
            }
        }
    }

    return false;
}

void bvh::find_nearest_object(const packet_ray *const r, const triangle **const i_o, packet_hit_description *const h) const
{
    /* State of stack */
    bvh_stack_element *exit_point = &(_bvh_stack[0]);
    exit_point->idx     = _root_node;
    exit_point->vt_min  = vfp_t(MAX_DIST);

    bvh_stack_element   entry_point;
    entry_point.idx     = _root_node;
    entry_point.vt_min  = vfp_t(MAX_DIST);
    
    /* Take the inverse direction of the ray for faster traversal */
    vfp_t i_rd[3] = { 1.0f / r->get_x_grad(), 1.0f / r->get_y_grad(), 1.0f / r->get_z_grad() };

    /* Traverse the whole tree */
    h->d = vfp_t(MAX_DIST);
    while (true)
    {
        /* Find a leaf node */
        const bool cmd = this->find_leaf_node(*r, &entry_point, &exit_point, &i_rd[0], h->d);

        /* If the leaf contains objects find the closest intersecting object */
        if (cmd)
        {
            (*_bvh_base)[entry_point.idx].test_leaf_node_nearest(r, i_o, h, 1);
        }

        /* Unwind the stack */
        do
        {
            /* If the whole tree has been traversed, return */
            if (exit_point == &(_bvh_stack[0]))
            {
                return;
            }
            
            entry_point.idx    = exit_point->idx;
            entry_point.vt_min = exit_point->vt_min;
            --exit_point;
        } while (!move_mask(entry_point.vt_min < h->d));
    }
}

vfp_t bvh::found_nearer_object(const packet_ray *const r, const vfp_t &t) const
{
    /* State of stack */
    bvh_stack_element *exit_point = &(_bvh_stack[0]);
    exit_point->idx     = _root_node;
    exit_point->vt_min  = vfp_t(MAX_DIST);

    bvh_stack_element   entry_point;
    entry_point.idx     = _root_node;
    entry_point.vt_min  = vfp_t(MAX_DIST);
    
    vfp_t closer(vfp_zero);
    packet_hit_description h(t);

    /* Take the inverse direction of the ray for faster traversal */
    vfp_t i_rd[3] = { 1.0f / r->get_x_grad(), 1.0f / r->get_y_grad(), 1.0f / r->get_z_grad() };

    /* Traverse the whole tree */
    while (true)
    {
        /* Find a leaf node */
        const bool cmd = this->find_leaf_node(*r, &entry_point, &exit_point, &i_rd[0], t);

        /* If the leaf contains objects find the closest intersecting object */
        if (cmd)
        {
            (*_bvh_base)[entry_point.idx].test_leaf_node_nearer(r, &closer, t, &h);
            
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
            if (exit_point == &(_bvh_stack[0]))
            {
                return closer;
            }
            
            entry_point.idx    = exit_point->idx;
            entry_point.vt_min = exit_point->vt_min;
            --exit_point;            
        } while (!move_mask(entry_point.vt_min < h.d));
    }

    return vfp_zero;
}

void bvh::find_nearest_object(const packet_ray *const r, const triangle **const i_o, packet_hit_description *const h,
                                                bvh_stack_element entry_point, bvh_stack_element *exit_point) const
{
    const bvh_stack_element *const top_of_stack = exit_point;
    entry_point.vt_min  = vfp_t(entry_point.t_min);

    /* Take the inverse direction of the ray for faster traversal */
    vfp_t i_rd[3] = { 1.0f / r->get_x_grad(), 1.0f / r->get_y_grad(), 1.0f / r->get_z_grad() };

    /* Traverse the whole tree */
    while (true)
    {
        /* Find a leaf node */
        const bool cmd = this->find_leaf_node(*r, &entry_point, &exit_point, &i_rd[0], h->d);

        /* If the leaf contains objects find the closest intersecting object */
        if (cmd)
        {
            (*_bvh_base)[entry_point.idx].test_leaf_node_nearest(r, i_o, h, 1);
        }

        /* Unwind the stack */
        do
        {
            /* If the whole tree has been traversed, return */
            if (exit_point == top_of_stack)
            {
                return;
            }
            
            entry_point.idx    = exit_point->idx;
            entry_point.vt_min = exit_point->vt_min;
            --exit_point;
            
        } while (!move_mask(entry_point.vt_min < h->d));
                
        entry_point.vt_max = min(entry_point.vt_max, h->d);
    }
}

vfp_t bvh::found_nearer_object(const packet_ray *const r, const vfp_t &t, bvh_stack_element entry_point, bvh_stack_element *exit_point) const
{
    vfp_t closer(vfp_zero);
    packet_hit_description h(t);
    
    bvh_stack_element *const top_of_stack = exit_point;

    /* Take the inverse direction of the ray for faster traversal */
    vfp_t i_rd[3] = { 1.0f / r->get_x_grad(), 1.0f / r->get_y_grad(), 1.0f / r->get_z_grad() };

    /* Traverse the whole tree */
    while (true)
    {
        /* Find a leaf node */
        const bool cmd = this->find_leaf_node(*r, &entry_point, &exit_point, &i_rd[0], t);

        /* If the leaf contains objects find the closest intersecting object */
        if (cmd)
        {
            (*_bvh_base)[entry_point.idx].test_leaf_node_nearer(r, &closer, t, &h);
            
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
            
            entry_point.idx    = exit_point->idx;
            entry_point.vt_min = exit_point->vt_min;
            --exit_point;
        } while (!move_mask(entry_point.vt_min < h.d));
    }

    return vfp_zero;
}
#endif /* #ifdef SIMD_PACKET_TRACING */

inline bool bvh::find_leaf_node(const ray &r, bvh_stack_element *const entry_point, bvh_stack_element **const out, const point_t &i_rd, const float t_max) const
{
    /* Get current state */
    bvh_stack_element *exit_point = *out;
    int cur_idx = entry_point->idx;
    // float t_min = entry_point->t_min;

    while (true)
    {
        if ((*_bvh_base)[cur_idx].is_leaf())
        {
            /* This node is a leaf, update the state and return */
            entry_point->idx = cur_idx;
            *out = exit_point;
            // BOOST_LOG_TRIVIAL(trace) << "Found leaf at index: " << entry_point->idx;
            return true;
        }
        else
        {
            /* Find distance to the children */
            int near_idx    = (*_bvh_base)[cur_idx].left_index();
            int far_idx     = (*_bvh_base)[cur_idx].right_index();
            float near_dist = (*_bvh_base)[near_idx].intersection_distance(r, i_rd);
            float far_dist  = (*_bvh_base)[far_idx].intersection_distance(r, i_rd);

            /* Order the children */
            if (near_dist > far_dist)
            {
                std::swap(near_idx, far_idx);
                std::swap(near_dist, far_dist);
            }
            // BOOST_LOG_TRIVIAL(trace) << "Distance to children: " << near_dist << ", " << far_dist;

            /* We never make it to these node */
            if (near_dist >= (t_max + (100.0f * EPSILON)))
            {   
               *out = exit_point;
                return false;
            }

            /* The far node is too far away to traverse */
            if (far_dist >= (t_max + (100.0f * EPSILON)))
            {
                cur_idx = near_idx;
                // BOOST_LOG_TRIVIAL(trace) << "Traversing near node only";
            }
            /* Both nodes are traversed, near node first */
            else
            {
                /* Stack the far node */
                assert((exit_point - &_bvh_stack[0]) < MAX_BVH_STACK_HEIGHT);
                ++exit_point;
                exit_point->idx     = far_idx;
                exit_point->t_min   = far_dist;
                cur_idx = near_idx;
                // BOOST_LOG_TRIVIAL(trace) << "Traversing both nodes : " << near_idx << " then: " << far_idx;
            }
        }
    }

    return false;
}

triangle* bvh::find_nearest_object(const ray *const r, hit_description *const h) const
{
    /* Check we hit the scene at all */
    const point_t ray_dir_inv(1.0f / r->get_dir());
    if ((*_bvh_base)[_root_node].intersection_distance(*r, ray_dir_inv) == MAX_DIST)
    {
        return nullptr;
    }
    
    /* State of stack */
    bvh_stack_element *exit_point = &(_bvh_stack[0]);
    exit_point->idx     = _root_node;
    exit_point->t_min   = MAX_DIST;

    bvh_stack_element   entry_point;
    entry_point.idx     = _root_node;
    entry_point.t_min   = MAX_DIST;

    /* Traverse the whole tree */
    hit_description nearest_hit;
    triangle *hit_object = nullptr;
    // BOOST_LOG_TRIVIAL(trace) << "Beginning search for nearest object";
    while (true)
    {
        /* Find a leaf node */
        const bool leaf = this->find_leaf_node(*r, &entry_point, &exit_point, ray_dir_inv, nearest_hit.d);

        /* If the leaf contains objects find the closest intersecting object */
        if (leaf)
        {
            // BOOST_LOG_TRIVIAL(trace) << "Found leaf node";
            triangle *intersecting_object = (*_bvh_base)[entry_point.idx].test_leaf_node_nearest(r, &nearest_hit);

            /* If an intersecting object is found it is the closest so return */
            if (intersecting_object != nullptr)
            {
                hit_object = intersecting_object;
                // BOOST_LOG_TRIVIAL(trace) << "Found intersecting object";
            }
        }

        /* Unwind the stack */
        do
        {
            /* If the whole tree has been traversed, return */
            if (exit_point == &(_bvh_stack[0]))
            {
                // BOOST_LOG_TRIVIAL(trace) << "Search complete";
                *h = nearest_hit;
                return hit_object;
            }
      
            entry_point.idx    = exit_point->idx;
            entry_point.t_min  = exit_point->t_min;
            --exit_point;
            // BOOST_LOG_TRIVIAL(trace) << "Unwound one stack level: " << entry_point.t_min;
        } while (entry_point.t_min > (nearest_hit.d + (100.0f * EPSILON)));
    }

    return nullptr;
}

bool bvh::found_nearer_object(const ray *const r, const float t) const
{
    /* state of stack */
    bvh_stack_element *exit_point = &(_bvh_stack[0]);
    exit_point->idx     = _root_node;
    exit_point->t_min   = MAX_DIST;

    bvh_stack_element   entry_point;
    entry_point.idx     = _root_node;
    entry_point.t_min   = MAX_DIST;
    
    /* Traverse the whole tree */
    const point_t ray_dir_inv(1.0f / r->get_dir());
    // BOOST_LOG_TRIVIAL(trace) << "Beginning search for nearer object";
    while (true)
    {
        /* Find a leaf node */
        const bool leaf = this->find_leaf_node(*r, &entry_point, &exit_point, ray_dir_inv, t);
        
        /* If the leaf contains objects find the closest intersecting object */
        if (leaf)
        {
            // BOOST_LOG_TRIVIAL(trace) << "Testing leaf node: " << entry_point.idx << ", " << t;
            const bool closer = (*_bvh_base)[entry_point.idx].test_leaf_node_nearer(r, t);

            /* If an intersecting object is found it is the closest so return */
            if (closer) 
            {
                // BOOST_LOG_TRIVIAL(trace) << "Found closer object, search complete";
               return true;
            }
        }
  
        /* If the whole tree is traversed without finding an intersection return false */
        if (exit_point == &(_bvh_stack[0]))
        {
            // BOOST_LOG_TRIVIAL(trace) << "Stack unwound, no object hit";
            return false;
        }
  
        /* Unwind the stack */
        entry_point.idx    = exit_point->idx;
        entry_point.t_min  = exit_point->t_min;
        --exit_point;
        // BOOST_LOG_TRIVIAL(trace) << "Unwound one stack level";
    }

    return false;
}
}; /* namespace raptor_raytracer*/
