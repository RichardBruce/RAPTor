#include "voxel.h"


namespace raptor_raytracer
{
/**********************************************************
 
**********************************************************/
voxel voxel::divide(kdt_node *const k)
{
    /* Calculate the cost of this node */
    float nr_of_primitives = this->p->size();
    float x_dist = this->t.x - this->b.x;
    float y_dist = this->t.y - this->b.y;
    float z_dist = this->t.z - this->b.z;
    float area   = (x_dist * y_dist) + (x_dist * z_dist) + (z_dist * y_dist);
    float lowest_cost = COST_OF_INTERSECTION * nr_of_primitives * area;
    float cost_before = lowest_cost;

#if 1   /* Approximate builder */
    axis_t normal;
    float best_split;
    
#ifdef SIMD_PACKET_TRACING
    /* If using SIMD allow early exit for nodes under a given size */
    if (this->p->size() <= MAX_KDT_NODE_SIZE)
    {
        k->set_primitives(this->p);
        return *this;
    } 
    else
#endif /* #ifdef SIMD_PACKET_TRACING */
    /* Invoke the exact builder for small node */
    if (this->p->size() <= MIN_APPROX_KDT_BUILDER_NODE_SIZE)
    {
        best_split = this->brute_force_split_all_axis(&lowest_cost, &normal);
    }
    else
    {
#if 0   /* Split approximately in all axis */
        best_split = this->approximate_split_all_axis(&lowest_cost, &normal);
#else   /* Split approximately in the first lower cost axis */
        const float dx = t.x - b.x;
        const float dy = t.y - b.y;
        const float dz = t.z - b.z;
        if (dx > dy)
        {
            if (dx > dz)
            {
                normal = axis_t::x_axis;
            }
            else
            {
                normal = axis_t::z_axis;
            }
        }
        else
        {
            if (dy > dz)
            {
                normal = axis_t::y_axis;
            }
            else
            {
                normal = axis_t::z_axis;
            }
        }
        best_split = approximate_split_one_axis(&lowest_cost, normal);
#endif
    }
#else   /* Exact builder */
#if 0   /* Signal axis aplit */
    float best_split = this->split_this_axis(&lowest_cost);
#else   /* All axis aplit */
    axis_t normal;
    float best_split = this->split_all_axis(&lowest_cost, &normal);
#endif
#endif

    /* Stop splitting if the cost metric cannot be reduced */
    assert(lowest_cost >= 0.0f);
    if (lowest_cost >= cost_before)
    {
        k->set_primitives(this->p);
        return *this;
    }
    
    /* Adjust the size of the voxel */
    point_t upper_limit = this->t;
    point_t lower_limit = this->b;
    axis_t  nxt_n;
    switch (normal)
    {
        case axis_t::x_axis:
            /* Assert the the voxel is really split */
            assert(best_split > this->b.x);
            assert(best_split < this->t.x);
            lower_limit.x   = best_split;
            this->t.x       = best_split;
            nxt_n           = axis_t::y_axis;
            break;
        case axis_t::y_axis:
            assert(best_split > this->b.y);
            assert(best_split < this->t.y);
            lower_limit.y   = best_split;
            this->t.y       = best_split;
            nxt_n           = axis_t::z_axis;
            break;
        case axis_t::z_axis:
            assert(best_split > this->b.z);
            assert(best_split < this->t.z);
            lower_limit.z   = best_split;
            this->t.z       = best_split;
            nxt_n           = axis_t::x_axis;
            break;
        default:
            assert(false);
            break;
    }
    
    /* Divide the primitives */
    float nearest_clip = best_split;
#if 0   /* Set to enable clip of split plane to nearest primitive */
    float dist_to_clip = MAX_DIST;
#endif
    primitive_list *child_p = new primitive_list;
    
    primitive_list::iterator p_dst = this->p->begin();
    switch (normal)
    {
        case axis_t::x_axis:
            for (primitive_list::iterator i = this->p->begin(); i != this->p->end(); ++i)
            {
#if 0   /* Set to enable clip of split plane to nearest primitive */
                if (abs((*i)->highest_x() - best_split) < dist_to_clip)
                {
                    dist_to_clip = abs((*i)->highest_x() - best_split);
                    nearest_clip = (*i)->highest_x();
                }
                
                if (abs((*i)->lowest_x() - best_split) < dist_to_clip)
                {
                    dist_to_clip = abs((*i)->lowest_x() - best_split);
                    nearest_clip = (*i)->lowest_x();
                }
#endif
                
                if ((*i)->is_intersecting_x(best_split))
                {
                    child_p->push_back(*i);
                    (*p_dst++) = (*i);
                }
                else if ((*i)->highest_x() > best_split)
                {
                    child_p->push_back(*i);
                }
                else
                {
                    (*p_dst++) = (*i);
                }
            }
            break;
        case axis_t::y_axis:
            for (primitive_list::iterator i = this->p->begin(); i != this->p->end(); ++i)
            {
#if 0  /* Set to enable clip of split plane to nearest primitive */
                if (abs((*i)->highest_y() - best_split) < dist_to_clip)
                {
                    dist_to_clip = abs((*i)->highest_y() - best_split);
                    nearest_clip = (*i)->highest_y();
                }
                
                if (abs((*i)->lowest_y() - best_split) < dist_to_clip)
                {
                    dist_to_clip = abs((*i)->lowest_y() - best_split);
                    nearest_clip = (*i)->lowest_y();
                }
#endif

                if ((*i)->is_intersecting_y(best_split))
                {
                    child_p->push_back(*i);
                    (*p_dst++) = (*i);
                }
                else if ((*i)->highest_y() > best_split)
                {
                    child_p->push_back(*i);
                }
                else
                {
                    (*p_dst++) = (*i);
                }
            }
            break;
        case axis_t::z_axis:
            for (primitive_list::iterator i = this->p->begin(); i != this->p->end(); ++i)
            {
#if 0  /* Set to enable clip of split plane to nearest primitive */
                if (abs((*i)->highest_z() - best_split) < dist_to_clip)
                {
                    dist_to_clip = abs((*i)->highest_z() - best_split);
                    nearest_clip = (*i)->highest_z();
                }
                
                if (abs((*i)->lowest_z() - best_split) < dist_to_clip)
                {
                    dist_to_clip = abs((*i)->lowest_z() - best_split);
                    nearest_clip = (*i)->lowest_z();
                }
#endif

                if ((*i)->is_intersecting_z(best_split))
                {
                    child_p->push_back(*i);
                    (*p_dst++) = (*i);
                }
                else if ((*i)->highest_z() > best_split)
                {
                    child_p->push_back(*i);
                }
                else
                {
                    (*p_dst++) = (*i);
                }
            }
            break;
        default :
            assert(false);
            break;
    }

    /* Remove unused primitives */
    this->p->erase(p_dst, this->p->end());

    /* Set the split kdt node */
    k->split_node(nearest_clip, normal);

    assert(nr_of_primitives <= (this->p->size() + child_p->size()));
    this->n = nxt_n;
    return voxel(child_p, upper_limit, lower_limit, nxt_n);
}


/**********************************************************
 
**********************************************************/
// float voxel::split_all_axis(float *s, axis_t *normal)
// {
//     /* Find the best split position of the primitives */
//     axis_t   best_axis   = axis_t::not_set;
//     float     best_split  = axis_t::not_set;
//     float     lowest_cost = *s;
    
//      /* Initial primitive split */
//     unsigned size_of_prim = this->p->size();
//     float     right = size_of_prim;
//     float     left  = 0;
//     float     guess;
//     float     last_guess = MAX_DIST;
    
//     /* Elements for evaluation */
//     float lo_x, lo_y, lo_z, hi_x, hi_y, hi_z;
    
//     /* Allocated arrays to hold the bounding box points */    
//     if (low_points == NULL)
//     {
//         low_points  = new float [size_of_prim];
//         high_points = new float [size_of_prim];
//     }
    
//     unsigned index=0;
//     for (primitive_list::iterator i=this->p->begin(); i!= this->p->end(); ++i)
//     {
//         low_points [index  ] = (*i)->lowest_x();
//         high_points[index++] = (*i)->highest_x();
//     }
    
//     quick_sort(low_points,  0, (size_of_prim-1));
//     quick_sort(high_points, 0, (size_of_prim-1));
    
//     /* Move past any out of range triangles */
//     unsigned low_index  =  0;
//     unsigned high_index =  0;
//     lo_x = -MAX_DIST;
//     while ((lo_x <= this->b.x) && (low_index < size_of_prim))
//     {
//         lo_x = low_points[low_index];
//         low_index++;
//         /* This will cause primitives to be split */
//         left++;
//     }
    
//     /* Evaluate the SAH from left to right */
//     hi_x = -MAX_DIST;
//     while (true)
//     {
//         /* Get point to evaluate or MAX_DIST if non remain */
//         if (low_index == size_of_prim)
//         {
//             lo_x = MAX_DIST;
//         }
//         else
//         {
//             lo_x = low_points[low_index];
//         }
        
//         if (high_index == size_of_prim)
//         {
//             hi_x = MAX_DIST;
//         }
//         else
//         {
//             hi_x = high_points[high_index];
//         }

//         /* Pick the point furthest left and adjust primitive counts */
//         if (lo_x < hi_x)
//         {
//             guess = lo_x;
//         }
//         else
//         {
//             guess = hi_x;
//             right--;
//         }

//         /* Break when any further points will be out of bounds */
//         if (guess >= this->t.x)
//         {
//             break;
//         }
        
//         /* Evaluate unique points */
//         if (last_guess != guess)
//         {
//             float this_cost = calculate_sah_cost(left, right, guess, axis_t::x_axis);
//             if (this_cost < lowest_cost)
//             {
//                 lowest_cost = this_cost;
//                 best_split  = guess;
//                 best_axis   = axis_t::x_axis;
//             }
//             last_guess = guess;
//         }

//         /* Advance iterators and primitive counts */
//         if (lo_x < hi_x)
//         {
//             left++;
//             low_index++;
//         }
//         else
//         {
//             high_index++;
//         }
//     }

//     right = size_of_prim;
//     left  = 0;
//     last_guess = MAX_DIST;
//     index=0;
//     for (primitive_list::iterator i=this->p->begin(); i!= this->p->end(); ++i)
//     {
//         low_points [index  ] = (*i)->lowest_y();
//         high_points[index++] = (*i)->highest_y();
//     }
    
//     quick_sort(low_points,  0, (size_of_prim-1));
//     quick_sort(high_points, 0, (size_of_prim-1));
    
//     /* Move past any out of range triangles */
//     low_index  = 0;
//     high_index = 0;
//     lo_y = -MAX_DIST;
//     while ((lo_y <= this->b.y) && (low_index < size_of_prim))
//     {
//         lo_y = low_points[low_index];
//         low_index++;
//         /* This will cause primitives to be split */
//         left++;
//     }
    
//     /* Evaluate the SAH from left to right */
//     hi_y = -MAX_DIST;
//     while (true)
//     {
//         /* Get point to evaluate or MAX_DIST if non remain */
//         if (low_index == size_of_prim)
//         {
//             lo_y = MAX_DIST;
//         }
//         else
//         {
//             lo_y = low_points[low_index];
//         }
        
//         if (high_index == size_of_prim)
//         {
//             hi_y = MAX_DIST;
//         }
//         else
//         {
//             hi_y = high_points[high_index];
//         }

//         /* Pick the point furthest left and adjust primitive counts */
//         if (lo_y < hi_y)
//         {
//             guess = lo_y;
//         }
//         else
//         {
//             guess = hi_y;
//             right--;
//         }

//         /* Break when any further points will be out of bounds */
//         if (guess >= this->t.y)
//         {
//             break;
//         }
        
//         /* Evaluate unique points */
//         if (last_guess != guess)
//         {
//             float this_cost = calculate_sah_cost(left, right, guess, axis_t::y_axis);
//             if (this_cost < lowest_cost)
//             {
//                 lowest_cost = this_cost;
//                 best_split  = guess;
//                 best_axis   = axis_t::y_axis;
//             }
//             last_guess = guess;
//         }

//         /* Advance iterators and primitive counts */
//         if (lo_y < hi_y)
//         {
//             left++;
//             low_index++;
//         }
//         else
//         {
//             high_index++;
//         }
//     }

//     right = size_of_prim;
//     left  = 0;
//     last_guess = MAX_DIST;
//     index=0;
//     for (primitive_list::iterator i=this->p->begin(); i!= this->p->end(); ++i)
//     {
//         low_points [index  ] = (*i)->lowest_z();
//         high_points[index++] = (*i)->highest_z();
//     }
    
//     quick_sort(low_points,  0, (size_of_prim-1));
//     quick_sort(high_points, 0, (size_of_prim-1));
    
//     /* Move past any out of range triangles */
//     low_index  = 0;
//     high_index = 0;
//     lo_z = -MAX_DIST;
//     while ((lo_z <= this->b.z) && (low_index < size_of_prim))
//     {
//         lo_z = low_points[low_index];
//         low_index++;
//         /* This will cause primitives to be split */
//         left++;
//     }
    
//     /* Evaluate the SAH from left to right */
//     hi_z = -MAX_DIST;
//     while (true)
//     {
//         /* Get point to evaluate or MAX_DIST if non remain */
//         if (low_index == size_of_prim)
//         {
//             lo_z = MAX_DIST;
//         }
//         else
//         {
//             lo_z = low_points[low_index];
//         }
        
//         if (high_index == size_of_prim)
//         {
//             hi_z = MAX_DIST;
//         }
//         else
//         {
//             hi_z = high_points[high_index];
//         }

//         /* Pick the point furthest left and adjust primitive counts */
//         if (lo_z < hi_z)
//         {
//             guess = lo_z;
//         }
//         else
//         {
//             guess = hi_z;
//             right--;
//         }

//         /* Break when any further points will be out of bounds */
//         if (guess >= this->t.z)
//         {
//             break;
//         }
        
//         /* Evaluate unique points */
//         if (last_guess != guess)
//         {
//             float this_cost = calculate_sah_cost(left, right, guess, axis_t::z_axis);
//             if (this_cost < lowest_cost)
//             {
//                 lowest_cost = this_cost;
//                 best_split  = guess;
//                 best_axis   = axis_t::z_axis;
//             }
//             last_guess = guess;
//         }

//         /* Advance iterators and primitive counts */
//         if (lo_z < hi_z)
//         {
//             left++;
//             low_index++;
//         }
//         else
//         {
//             high_index++;
//         }
//     }
    
//     *normal = best_axis;
//     *s = lowest_cost;
//     return best_split;
// }


// float voxel::approximate_split_all_axis(float *s, axis_t * normal) const
// {
//     axis_t   best_axis      = axis_t::not_set;
//     float     best_split     = MAX_DIST;
//     float     cost           = *s;
//     float     lowest_cost    = *s;

//     /* Split in x and compare the result */
//     float split =  approximate_split_one_axis(&cost, axis_t::x_axis);
//     if (cost < lowest_cost)
//     {
//         lowest_cost = cost;
//         best_axis   = axis_t::x_axis;
//         best_split  = split;
//     }

//     /* Split in y and compare the result */
//     split =  approximate_split_one_axis(&cost, axis_t::y_axis);
//     if (cost < lowest_cost)
//     {
//         lowest_cost = cost;
//         best_axis   = axis_t::y_axis;
//         best_split  = split;
//     }

//     /* Split in z and compare the result */
//     split =  approximate_split_one_axis(&cost, axis_t::z_axis);
//     if (cost < lowest_cost)
//     {
//         lowest_cost = cost;
//         best_axis   = axis_t::z_axis;
//         best_split  = split;
//     }
    
//     *s = lowest_cost;
//     *normal = best_axis;
    
//     return best_split;
// }


float voxel::approximate_split_one_axis(float *const s, const axis_t normal) const
{
    /* Find the best split position of the primitives */
    float best_split     = MAX_DIST;
    float lowest_cost    = *s;
    
    /* Adaptive sampling bin sizes */
    const float adapt_bin_size   = this->p->size() * (2.0/9.0);
    
    /* Voxel dimensions */
    const float xw       = this->t.x - this->b.x;
    const float yw       = this->t.y - this->b.y;
    const float zw       = this->t.z - this->b.z;
    const float xb_yw    = this->b.x * yw;
    const float xb_zw    = this->b.x * zw;
    const float xt_yw    = this->t.x * yw;
    const float xt_zw    = this->t.x * zw;
    const float yb_xw    = this->b.y * xw;
    const float yb_zw    = this->b.y * zw;
    const float yt_xw    = this->t.y * xw;
    const float yt_zw    = this->t.y * zw;
    const float zb_xw    = this->b.z * xw;
    const float zb_yw    = this->b.z * yw;
    const float zt_xw    = this->t.z * xw;
    const float zt_yw    = this->t.z * yw;
    const float xw_yw    = xw * yw;
    const float xw_zw    = xw * zw;
    const float yw_zw    = yw * zw;
    const float xw_p_yw  = xw + yw;
    const float xw_p_zw  = xw + zw;
    const float yw_p_zw  = yw + zw;
//    const float sa       = 1.0 / (xw_yw + xw_zw + yw_zw);
    
    /* Sampling variable */
    float dw;
    float evaluate;
    float sample[8] ALIGN(16);
    float bins_per_sample[9];
    float adaptive_width[9];
    float cl[9];
    float cr[9];
    float cla[9];
    float cra[9];
    float last_cl;
    float last_cr;
    float sp;
    int  adaptive_offset;

    switch (normal)
    {
        case axis_t::x_axis :   
            /* Pick fixed points to evaluate */
            dw          = xw * (1.0 / 9.0);
            evaluate    = this->t.x;
            
            /* Fixed points samples and adaptive bin boundaries */
            cl[8]               = this->p->size();
            cr[8]               = 0.0;
            bins_per_sample[8]  = 8.0;
            
            for (int i = 0; i < 8; i++)
            {
                sample[i] = this->b.x + (dw * static_cast<float>(i + 1));
            }
            this->count_primitives(cl, cr, sample, 8, axis_t::x_axis);
            
            /* Fix adaptive sample width per bin */
            for (int i = 7; i >= 0; i--)
            {
                bins_per_sample[i] = int((cl[i] - cr[i]) / adapt_bin_size) + 4;
                bins_per_sample[i+1]   -= bins_per_sample[i];
                adaptive_width[i+1]     = dw / (bins_per_sample[i+1] + 1.0);
            }
            adaptive_width[0]   = dw / (bins_per_sample[0] + 1.0);
            
            /* Take adaptive samples */
            adaptive_offset = 0;
            for (int i = 0; i < 9; i++)
            {
                evaluate = this->b.x + (i * dw);
                for (int j = adaptive_offset; j < (bins_per_sample[i] + adaptive_offset); j++)
                {
                    evaluate   += adaptive_width[i];
                    sample[j] = evaluate;
                }
                adaptive_offset += (int)bins_per_sample[i];
            }
            this->count_primitives(cla, cra, sample, 8, axis_t::x_axis);
            
            /* Evaluate SAH minima for each sample as -b/2a */
            adaptive_offset = 0;
            last_cl         = 0.0;
            last_cr         = this->p->size();
            sp              = this->b.x;
            for (int i = 0; i < 9; ++i)
            {
                /* Process any adaptive samples in this bin first because they will be encountered first */
                for (int j = adaptive_offset; j < (bins_per_sample[i] + adaptive_offset); j++)
                {
                    /* Bin primitive gradients */
                    const float clg = (cla[j] - last_cl) / adaptive_width[i];
                    const float crg = (cra[j] - last_cr) / adaptive_width[i];
                    
                    /* Sample location */
                    const float sp_yw    = sp * yw;
                    const float sp_zw    = sp * zw;
                    
                    /* Minima point bound to this bin */
                    const float a    = 2.0f * ((clg * yw_p_zw) - (crg * yw_p_zw));
                    const float b    =   ( last_cl * yw_p_zw) - (clg * (xb_yw + xb_zw - yw_zw + sp_yw + sp_zw))
                                      + (-last_cr * yw_p_zw) + (crg * (xt_yw + xt_zw + yw_zw + sp_yw + sp_zw));
                    
                    const float delta    = adaptive_width[i] * 0.05f;
                    const float xm       = std::max((sp + delta), std::min((sp + adaptive_width[i] - delta), (b / a)));
        
                    /* Cost at minima */
                    const float xmb  = xm - this->b.x;
                    const float xmt  = this->t.x - xm;
        
                    const float cost = COST_OF_TRAVERSAL + (COST_OF_INTERSECTION * 
                                      (((last_cl + (clg * (xm - sp))) * ((xmb * yw) + (xmb * zw) + yw_zw))  + 
                                       ((last_cr + (crg * (xm - sp))) * ((xmt * yw) + (xmt * zw) + yw_zw))));
                    
                    if (cost < lowest_cost)
                    {
                        lowest_cost = cost;
                        best_split  = xm;
                    }
                    
                    last_cl = cla[j];
                    last_cr = cra[j];
                    sp     += adaptive_width[i];
                }
                adaptive_offset += static_cast<int>(bins_per_sample[i]);
        
                /* Bin primitive gradients */
                const float clg = (cl[i] - last_cl) / adaptive_width[i];
                const float crg = (cr[i] - last_cr) / adaptive_width[i];
              
                /* Sample location */
                const float sp_yw    = sp * yw;
                const float sp_zw    = sp * zw;
              
                /* Minima point bound to this bin */
                const float a    = 2.0f * ((clg * yw_p_zw) - (crg * yw_p_zw));
                const float b    =   ( last_cl * yw_p_zw) - (clg * (xb_yw + xb_zw - yw_zw + sp_yw + sp_zw))
                                  + (-last_cr * yw_p_zw) + (crg * (xt_yw + xt_zw + yw_zw + sp_yw + sp_zw));
              
                const float delta    = adaptive_width[i] * 0.05f;
                const float xm       = std::max((sp + delta), std::min((sp + adaptive_width[i] - delta), (b / a)));
              
                /* Cost at minima */
                const float xmb  = xm - this->b.x;
                const float xmt  = this->t.x - xm;
        
                const float cost = COST_OF_TRAVERSAL + (COST_OF_INTERSECTION * 
                                  (((last_cl + (clg * (xm - sp))) * ((xmb * yw) + (xmb * zw) + yw_zw))  + 
                                   ((last_cr + (crg * (xm - sp))) * ((xmt * yw) + (xmt * zw) + yw_zw))));
              
                if (cost < lowest_cost)
                {
                    lowest_cost = cost;
                    best_split  = xm;
                }
              
                last_cl = cl[i];
                last_cr = cr[i];
                sp     += adaptive_width[i];
            }
            break;
            
        case axis_t::y_axis :
            /* Pick fixed points to evaluate */
            dw          = yw * (1.0f / 9.0f);
            evaluate    = this->t.y;
            
            /* Fixed points samples and adaptive bin boundaries */
            cl[8]               = this->p->size();
            cr[8]               = 0.0f;
            bins_per_sample[8]  = 8.0f;
        
            for (int i = 0; i < 8; ++i)
            {
                sample[i] = this->b.y + (dw * static_cast<float>(i + 1));
            }
            this->count_primitives(cl, cr, sample, 8, axis_t::y_axis);
        
            /* Fix adaptive sample width per bin */
            for (int i = 7; i >= 0; i--)
            {
                bins_per_sample[i]      = int((cl[i] - cr[i]) / adapt_bin_size) + 4;
                bins_per_sample[i+1]   -= bins_per_sample[i];
                adaptive_width[i+1]     = dw / (bins_per_sample[i+1] + 1.0);
            }
            adaptive_width[0]   = dw / (bins_per_sample[0] + 1.0);
            
            /* Take adaptive samples */
            adaptive_offset = 0;
            for (int i = 0; i < 9; i++)
            {
                evaluate = this->b.y + (i * dw);
                for (int j = adaptive_offset; j < (bins_per_sample[i] + adaptive_offset); j++)
                {
                    evaluate   += adaptive_width[i];
                    sample[j] = evaluate;
                }
                adaptive_offset += (int)bins_per_sample[i];
            }
            this->count_primitives(cla, cra, sample, 8, axis_t::y_axis);
            
            /* Evaluate SAH minima for each sample as -b/2a */
            adaptive_offset = 0;
            last_cl         = 0.0f;
            last_cr         = this->p->size();
            sp              = this->b.y;
            for (int i = 0; i < 9; ++i)
            {
                /* Process any adaptive samples in this bin first because they will be encountered first */
                for (int j = adaptive_offset; j < (bins_per_sample[i] + adaptive_offset); j++)
                {
                    /* Bin primitive gradients */
                    const float clg = (cla[j] - last_cl) / adaptive_width[i];
                    const float crg = (cra[j] - last_cr) / adaptive_width[i];
                    
                    /* Sample location */
                    const float sp_xw    = sp * xw;
                    const float sp_zw    = sp * zw;
                    
                    /* Minima point bound to this bin */
                    const float a    = 2.0f * ((clg * xw_p_zw) - (crg * xw_p_zw));
                    const float b    =   ( last_cl * xw_p_zw) - (clg * (yb_xw + yb_zw - xw_zw + sp_xw + sp_zw))
                                      + (-last_cr * xw_p_zw) + (crg * (yt_xw + yt_zw + xw_zw + sp_xw + sp_zw));
                    
                    const float delta    = adaptive_width[i] * 0.05f;
                    const float ym       = std::max((sp + delta), std::min((sp + adaptive_width[i] - delta), (b / a)));
                    
                    /* Cost at minima */
                    const float ymb  = ym - this->b.y;
                    const float ymt  = this->t.y - ym;
        
                    const float cost = COST_OF_TRAVERSAL + (COST_OF_INTERSECTION * 
                                      (((last_cl + (clg * (ym - sp))) * ((ymb * xw) + (ymb * zw) + xw_zw))  + 
                                       ((last_cr + (crg * (ym - sp))) * ((ymt * xw) + (ymt * zw) + xw_zw))));
                    
                    if (cost < lowest_cost)
                    {
                        lowest_cost = cost;
                        best_split  = ym;
                    }
                    
                    last_cl = cla[j];
                    last_cr = cra[j];
                    sp     += adaptive_width[i];
                }
                adaptive_offset += static_cast<int>(bins_per_sample[i]);
        
                /* Bin primitive gradients */
                const float clg = (cl[i] - last_cl) / adaptive_width[i];
                const float crg = (cr[i] - last_cr) / adaptive_width[i];
              
                /* Sample location */
                const float sp_xw    = sp * xw;
                const float sp_zw    = sp * zw;
              
                /* Minima point bound to this bin */
                const float a    = 2.0f * ((clg * xw_p_zw) - (crg * xw_p_zw));
                const float b    =   ( last_cl * xw_p_zw) - (clg * (yb_xw + yb_zw - xw_zw + sp_xw + sp_zw))
                                  + (-last_cr * xw_p_zw) + (crg * (yt_xw + yt_zw + xw_zw + sp_xw + sp_zw));
              
                const float delta    = adaptive_width[i] * 0.05f;
                const float ym       = std::max((sp + delta), std::min((sp + adaptive_width[i] - delta), (b / a)));
              
                /* Cost at minima */
                const float ymb  = ym - this->b.y;
                const float ymt  = this->t.y - ym;
        
                const float cost = COST_OF_TRAVERSAL + (COST_OF_INTERSECTION * 
                                  (((last_cl + (clg * (ym - sp))) * ((ymb * xw) + (ymb * zw) + xw_zw))  + 
                                   ((last_cr + (crg * (ym - sp))) * ((ymt * xw) + (ymt * zw) + xw_zw))));
              
                if (cost < lowest_cost)
                {
                    lowest_cost = cost;
                    best_split  = ym;
                }
              
                last_cl = cl[i];
                last_cr = cr[i];
                sp     += adaptive_width[i];
            }
            break;

        case axis_t::z_axis :
            /* Pick fixed points to evaluate */
            dw          = zw * (1.0f / 9.0f);
            evaluate    = this->t.z;
            
            /* Fixed points samples and adaptive bin boundaries */
            cl[8]               = this->p->size();
            cr[8]               = 0.0;
            bins_per_sample[8]  = 8.0;
        
            for (int i = 0; i < 8; i++)
            {
                sample[i] = this->b.z + (dw * static_cast<float>(i + 1));
            }
            this->count_primitives(cl, cr, sample, 8, axis_t::z_axis);
        
            /* Fix adaptive sample width per bin */
            for (int i = 7; i >= 0; --i)
            {
                bins_per_sample[i] = int((cl[i] - cr[i]) / adapt_bin_size) + 4;
                bins_per_sample[i+1]   -= bins_per_sample[i];
                adaptive_width[i+1]     = dw / (bins_per_sample[i + 1] + 1.0);
            }
            adaptive_width[0]   = dw / (bins_per_sample[0] + 1.0);
            
            /* Take adaptive samples */
            adaptive_offset = 0;
            for (int i = 0; i < 9; i++)
            {
                evaluate = this->b.z + (i * dw);
                for (int j = adaptive_offset; j < (bins_per_sample[i] + adaptive_offset); j++)
                {
                    evaluate   += adaptive_width[i];
                    sample[j] = evaluate;
                }
                adaptive_offset += (int)bins_per_sample[i];
            }
            this->count_primitives(cla, cra, sample, 8, axis_t::z_axis);
            
            /* Evaluate SAH minima for each sample as -b/2a */
            adaptive_offset = 0;
            last_cl         = 0.0f;
            last_cr         = this->p->size();
            sp              = this->b.z;
            for (int i = 0; i < 9; ++i)
            {
                /* Process any adaptive samples in this bin first because they will be encountered first */
                for (int j = adaptive_offset; j < (bins_per_sample[i] + adaptive_offset); j++)
                {
                    /* Bin primitive gradients */
                    const float clg = (cla[j] - last_cl) / adaptive_width[i];
                    const float crg = (cra[j] - last_cr) / adaptive_width[i];
                    
                    /* Sample location */
                    const float sp_xw    = sp * xw;
                    const float sp_yw    = sp * yw;
                    
                    /* Minima point bound to this bin */
                    const float a    = 2.0f * ((clg * xw_p_yw) - (crg * xw_p_zw));
                    const float b    =   ( last_cl * xw_p_yw) - (clg * (zb_xw + zb_yw - xw_yw + sp_xw + sp_yw))
                                      + (-last_cr * xw_p_yw) + (crg * (zt_xw + zt_yw + xw_yw + sp_xw + sp_yw));
                    
                    const float delta    = adaptive_width[i] * 0.05f;
                    const float zm       = std::max((sp + delta), std::min((sp + adaptive_width[i] - delta), (b / a)));
                    
                    /* Cost at minima */
                    const float zmb  = zm - this->b.z;
                    const float zmt  = this->t.z - zm;
        
                    const float cost = COST_OF_TRAVERSAL + (COST_OF_INTERSECTION * 
                                      (((last_cl + (clg * (zm - sp))) * ((zmb * xw) + (zmb * yw) + xw_yw))  + 
                                       ((last_cr + (crg * (zm - sp))) * ((zmt * xw) + (zmt * yw) + xw_yw))));
                    
                    if (cost < lowest_cost)
                    {
                        lowest_cost = cost;
                        best_split  = zm;
                    }
                    
                    last_cl = cla[j];
                    last_cr = cra[j];
                    sp     += adaptive_width[i];
                }
                adaptive_offset += static_cast<int>(bins_per_sample[i]);
        
                /* Bin primitive gradients */
                const float clg = (cl[i] - last_cl) / adaptive_width[i];
                const float crg = (cr[i] - last_cr) / adaptive_width[i];
              
                /* Sample location */
                const float sp_xw    = sp * xw;
                const float sp_yw    = sp * yw;
             
                /* Minima point bound to this bin */
                const float a    = 2.0f * ((clg * xw_p_yw) - (crg * xw_p_zw));
                const float b    =   ( last_cl * xw_p_yw) - (clg * (zb_xw + zb_yw - xw_yw + sp_xw + sp_yw))
                                  + (-last_cr * xw_p_yw) + (crg * (zt_xw + zt_yw + xw_yw + sp_xw + sp_yw));
              
                const float delta    = adaptive_width[i] * 0.05f;
                const float zm       = std::max((sp + delta), std::min((sp + adaptive_width[i] - delta), (b / a)));
              
                /* Cost at minima */
                const float zmb  = zm - this->b.z;
                const float zmt  = this->t.z - zm;
        
                const float cost = COST_OF_TRAVERSAL + (COST_OF_INTERSECTION * 
                                  (((last_cl + (clg * (zm - sp))) * ((zmb * xw) + (zmb * yw) + xw_yw))  + 
                                   ((last_cr + (crg * (zm - sp))) * ((zmt * xw) + (zmt * yw) + xw_yw))));
              
                if (cost < lowest_cost)
                {
                    lowest_cost = cost;
                    best_split  = zm;
                }
              
                last_cl = cl[i];
                last_cr = cr[i];
                sp     += adaptive_width[i];
            }
            break;

        default :
            assert(false);
            break;
    }
    
    *s = lowest_cost;
    return best_split;
}



/**********************************************************
 
// **********************************************************/
// inline float voxel::count_primitives(float *const r, const float s, const axis_t normal) const
// {
//     /* Sub divide the objects */
//     float left_objects  = 0.0f;
//     float right_objects = 0.0f;
//     switch (normal)
//     {
//         case axis_t::x_axis:
//             for (primitive_list::const_iterator i=this->p->begin(); i!=this->p->end(); ++i)
//             {
//                 left_objects  += (int)((*i)->lowest_x()  < s);
//                 right_objects += (int)((*i)->highest_x() > s);
//             }
//             break;
//         case axis_t::y_axis:
//             for (primitive_list::const_iterator i=this->p->begin(); i!=this->p->end(); ++i)
//             {
//                 left_objects  += (int)((*i)->lowest_y()  < s);
//                 right_objects += (int)((*i)->highest_y() > s);
//             }
//             break;
//         case axis_t::z_axis:
//             for (primitive_list::const_iterator i=this->p->begin(); i!=this->p->end(); ++i)
//             {
//                 left_objects  += (int)((*i)->lowest_z()  < s);
//                 right_objects += (int)((*i)->highest_z() > s);
//             }
//             break;
//         default :
//             assert(false);
//             break;
//     }

//     *r = right_objects;
//     return left_objects;
// }

/**********************************************************
 
**********************************************************/
float voxel::brute_force_split_all_axis(float *s, axis_t * normal) const
{
    /* Find the best split position of the primitives */
    float sample[9] ALIGN(16);
    vfp_t valid_mask[2];

    vfp_t l_vec[2], r_vec[2], s_vec[2];
    vfp_t lc_vec(*s);
    vfp_t ba_vec(static_cast<float>(axis_t::not_set));
    vfp_t bs_vec(MAX_DIST);

    /* Evaluate the X axis */
    int bucketted = 0;
    for (primitive_list::const_iterator i = this->p->begin(); i != this->p->end(); ++i)
    {
        /* Add samples to their bucket */
        if (((*i)->lowest_x() > this->b.x) && ((*i)->lowest_x() < this->t.x))
        {
            sample[bucketted++] = (*i)->lowest_x();
        }

        if (((*i)->highest_x() > this->b.x) && ((*i)->highest_x() < this->t.x))
        {
            sample[bucketted++] = (*i)->highest_x();
        }
        
        /* Check for bucket full */
        if (bucketted > 7)
        {
            s_vec[0] = &sample[0];
            s_vec[1] = &sample[SIMD_WIDTH];
            this->count_primitives(l_vec, r_vec, s_vec, axis_t::x_axis);
            for (int j = 0; j < 2; j++)
            {
                vfp_t cost = this->calculate_sah_cost(l_vec[j], r_vec[j], s_vec[j], axis_t::x_axis);
                vfp_t mask = (cost < lc_vec);

                lc_vec = min(lc_vec, cost);
                bs_vec = mov_p(mask, s_vec[j], bs_vec);
                ba_vec = mov_p(mask, vfp_t(static_cast<float>(axis_t::x_axis)), ba_vec);
            }
            
            /* Move down any overflow */
            if (bucketted == 9)
            {
                sample[0] = sample[8];
                bucketted = 1;
            }
            else
            {
                bucketted = 0;
            }
        }
    }
    
    /*  Process left over samples */
    s_vec[0] = &sample[0];
    s_vec[1] = &sample[SIMD_WIDTH];
    valid_mask[0] = (bucketted & SIMD_WIDTH) ? vfp_true : index_to_mask_lut[bucketted];
    valid_mask[1] = (bucketted & SIMD_WIDTH) ? index_to_mask_lut[bucketted - SIMD_WIDTH] : vfp_zero;
    
    this->count_primitives(l_vec, r_vec, s_vec, axis_t::x_axis);
    for (int j = 0; j < ((bucketted >> LOG2_SIMD_WIDTH) + (int)(bucketted > 0)); j++)
    {
        vfp_t cost = this->calculate_sah_cost(l_vec[j], r_vec[j], s_vec[j], axis_t::x_axis);
        vfp_t mask = (cost < lc_vec) & valid_mask[j];

        lc_vec = mov_p(mask, cost,                lc_vec);
        bs_vec = mov_p(mask, s_vec[j],            bs_vec);
        ba_vec = mov_p(mask, vfp_t(static_cast<float>(axis_t::x_axis)), ba_vec);
    }

    /* Evaluate the Y axis */
    bucketted = 0;
    for (primitive_list::const_iterator i = this->p->begin(); i != this->p->end(); ++i)
    {
        /* Add samples to their bucket */
        if (((*i)->lowest_y() > this->b.y) && ((*i)->lowest_y() < this->t.y))
        {
            sample[bucketted++] = (*i)->lowest_y();
        }

        if (((*i)->highest_y() > this->b.y) && ((*i)->highest_y() < this->t.y))
        {
            sample[bucketted++] = (*i)->highest_y();
        }
        
        /* Check for bucket full */
        if (bucketted > 7)
        {
            s_vec[0] = &sample[0];
            s_vec[1] = &sample[SIMD_WIDTH];
            this->count_primitives(l_vec, r_vec, s_vec, axis_t::y_axis);
            for (int j = 0; j < 2; j++)
            {
                vfp_t cost = this->calculate_sah_cost(l_vec[j], r_vec[j], s_vec[j], axis_t::y_axis);
                vfp_t mask = (cost < lc_vec);

                lc_vec = min(lc_vec, cost);
                bs_vec = mov_p(mask, s_vec[j], bs_vec);
                ba_vec = mov_p(mask, vfp_t(static_cast<float>(axis_t::y_axis)), ba_vec);
            }
            
            /* Move down any overflow */
            if (bucketted == 9)
            {
                sample[0] = sample[8];
                bucketted = 1;
            }
            else
            {
                bucketted = 0;
            }
        }
    }
    
    /*  Process left over samples */
    s_vec[0] = &sample[0];
    s_vec[1] = &sample[SIMD_WIDTH];
    valid_mask[0] = (bucketted & SIMD_WIDTH) ? vfp_true : index_to_mask_lut[bucketted];
    valid_mask[1] = (bucketted & SIMD_WIDTH) ? index_to_mask_lut[bucketted - SIMD_WIDTH] : vfp_zero;
    
    this->count_primitives(l_vec, r_vec, s_vec, axis_t::y_axis);
    for (int j = 0; j < ((bucketted >> LOG2_SIMD_WIDTH) + (int)(bucketted > 0)); j++)
    {
        vfp_t cost = this->calculate_sah_cost(l_vec[j], r_vec[j], s_vec[j], axis_t::y_axis);
        vfp_t mask = (cost < lc_vec) & valid_mask[j];

        lc_vec = mov_p(mask, cost,                lc_vec);
        bs_vec = mov_p(mask, s_vec[j],            bs_vec);
        ba_vec = mov_p(mask, vfp_t(static_cast<float>(axis_t::y_axis)), ba_vec);
    }

    /* Evaluate the Z axis */
    bucketted = 0;
    for (primitive_list::const_iterator i = this->p->begin(); i != this->p->end(); ++i)
    {
        /* Add samples to their bucket */
        if (((*i)->lowest_z() > this->b.z) && ((*i)->lowest_z() < this->t.z))
        {
            sample[bucketted++] = (*i)->lowest_z();
        }

        if (((*i)->highest_z() > this->b.z) && ((*i)->highest_z() < this->t.z))
        {
            sample[bucketted++] = (*i)->highest_z();
        }
        
        /* Check for bucket full */
        if (bucketted > 7)
        {
            s_vec[0] = &sample[0];
            s_vec[1] = &sample[SIMD_WIDTH];
            this->count_primitives(l_vec, r_vec, s_vec, axis_t::z_axis);
            for (int j = 0; j < 2; j++)
            {
                vfp_t cost = this->calculate_sah_cost(l_vec[j], r_vec[j], s_vec[j], axis_t::z_axis);
                vfp_t mask = (cost < lc_vec);

                lc_vec = min(lc_vec, cost);
                bs_vec = mov_p(mask, s_vec[j], bs_vec);
                ba_vec = mov_p(mask, vfp_t(static_cast<float>(axis_t::z_axis)), ba_vec);
            }
            
            /* Move down any overflow */
            if (bucketted == 9)
            {
                sample[0] = sample[8];
                bucketted = 1;
            }
            else
            {
                bucketted = 0;
            }
        }
    }
    
    /*  Process left over samples */
    s_vec[0] = &sample[0];
    s_vec[1] = &sample[SIMD_WIDTH];
    valid_mask[0] = (bucketted & SIMD_WIDTH) ? vfp_true : index_to_mask_lut[bucketted];
    valid_mask[1] = (bucketted & SIMD_WIDTH) ? index_to_mask_lut[bucketted - SIMD_WIDTH] : vfp_zero;
    
    this->count_primitives(l_vec, r_vec, s_vec, axis_t::z_axis);
    for (int j = 0; j < ((bucketted >> LOG2_SIMD_WIDTH) + static_cast<int>(bucketted > 0)); ++j)
    {
        vfp_t cost = this->calculate_sah_cost(l_vec[j], r_vec[j], s_vec[j], axis_t::z_axis);
        vfp_t mask = (cost < lc_vec) & valid_mask[j];

        lc_vec = mov_p(mask, cost,                lc_vec);
        bs_vec = mov_p(mask, s_vec[j],            bs_vec);
        ba_vec = mov_p(mask, vfp_t(static_cast<float>(axis_t::z_axis)), ba_vec);
    }
    
    /* Collect results */
    (*s) = std::min(std::min(lc_vec[0], lc_vec[1]), std::min(lc_vec[2], lc_vec[3]));

    const int index = mask_to_index_lut[move_mask(vfp_t(*s) == lc_vec)];
    (*normal) = static_cast<axis_t>(ba_vec[index]);
    return bs_vec[index];
}

/**********************************************************
 
**********************************************************/
inline void voxel::count_primitives(float *const l, float *const r, const float *const s, const int len, const axis_t n) const
{
    assert(len == 8);
    vfp_t l_vec[2] = { vfp_zero, vfp_zero };
    vfp_t r_vec[2] = { vfp_zero, vfp_zero };
    vfp_t s_vec[2] = { &s[0], &s[SIMD_WIDTH] };


    /* Count in the given axis */
    switch (n)
    {
        case axis_t::x_axis:
            for (primitive_list::const_iterator i = this->p->begin(); i != this->p->end(); ++i)
            {
                const vfp_t hi((*i)->highest_x());
                const vfp_t lo((*i)->lowest_x());
                
                l_vec[0] += (lo < s_vec[0]) & vfp_one;
                r_vec[0] += (hi > s_vec[0]) & vfp_one;
                l_vec[1] += (lo < s_vec[1]) & vfp_one;
                r_vec[1] += (hi > s_vec[1]) & vfp_one;
            }
            break;

        case axis_t::y_axis:
            for (primitive_list::const_iterator i = this->p->begin(); i != this->p->end(); ++i)
            {
                const vfp_t hi((*i)->highest_y());
                const vfp_t lo((*i)->lowest_y());
                
                l_vec[0] += (lo < s_vec[0]) & vfp_one;
                r_vec[0] += (hi > s_vec[0]) & vfp_one;
                l_vec[1] += (lo < s_vec[1]) & vfp_one;
                r_vec[1] += (hi > s_vec[1]) & vfp_one;
            }
            break;

        case axis_t::z_axis:
            for (primitive_list::const_iterator i = this->p->begin(); i != this->p->end(); ++i)
            {
                const vfp_t hi((*i)->highest_z());
                const vfp_t lo((*i)->lowest_z());
                
                l_vec[0] += (lo < s_vec[0]) & vfp_one;
                r_vec[0] += (hi > s_vec[0]) & vfp_one;
                l_vec[1] += (lo < s_vec[1]) & vfp_one;
                r_vec[1] += (hi > s_vec[1]) & vfp_one;
            }
            break;
        default :
            assert(false);
            break;
    }

    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < SIMD_WIDTH; j++)
        {
            l[(i << LOG2_SIMD_WIDTH) + j] = l_vec[i][j];
            r[(i << LOG2_SIMD_WIDTH) + j] = r_vec[i][j];
        }
    }
}


/**********************************************************
 
**********************************************************/
inline void voxel::count_primitives(vfp_t *const l, vfp_t *const r, const vfp_t *const s, const axis_t n) const
{
    l[0] = vfp_zero;
    l[1] = vfp_zero;
    r[0] = vfp_zero;
    r[1] = vfp_zero;

    /* Count in the given axis */
    switch (n)
    {
        case axis_t::x_axis:
            for (primitive_list::const_iterator i = this->p->begin(); i != this->p->end(); ++i)
            {
                const vfp_t hi((*i)->highest_x());
                const vfp_t lo((*i)->lowest_x());
                
                l[0] += (lo < s[0]) & vfp_one;
                r[0] += (hi > s[0]) & vfp_one;
                l[1] += (lo < s[1]) & vfp_one;
                r[1] += (hi > s[1]) & vfp_one;
            }
            break;

        case axis_t::y_axis:
            for (primitive_list::const_iterator i = this->p->begin(); i != this->p->end(); ++i)
            {
                const vfp_t hi((*i)->highest_y());
                const vfp_t lo((*i)->lowest_y());
                
                l[0] += (lo < s[0]) & vfp_one;
                r[0] += (hi > s[0]) & vfp_one;
                l[1] += (lo < s[1]) & vfp_one;
                r[1] += (hi > s[1]) & vfp_one;
            }
            break;

        case axis_t::z_axis:
            for (primitive_list::const_iterator i = this->p->begin(); i != this->p->end(); ++i)
            {
                const vfp_t hi((*i)->highest_z());
                const vfp_t lo((*i)->lowest_z());
                
                l[0] += (lo < s[0]) & vfp_one;
                r[0] += (hi > s[0]) & vfp_one;
                l[1] += (lo < s[1]) & vfp_one;
                r[1] += (hi > s[1]) & vfp_one;
            }
            break;
        default :
            assert(false);
            break;
    }
}


/**********************************************************
 
**********************************************************/
inline vfp_t voxel::calculate_sah_cost(const vfp_t &l, const vfp_t &r, const vfp_t &s, const axis_t normal) const
{
    /* Surface area of sub-divides */
    vfp_t left_area, right_area;
    vfp_t x_dist, y_dist, z_dist;
//    const float sa   = 1.0 / ((x_dist * y_dist) + (x_dist * z_dist) + (y_dist * z_dist));
    switch (normal)
    {
        case axis_t::x_axis:
//            assert(move_mask(s < vfp_t(this->b.x)) == 0x0);
//            assert(move_mask(s > vfp_t(this->t.x)) == 0x0);

            x_dist = s - vfp_t(this->b.x);
            y_dist = vfp_t(this->t.y - this->b.y);
            z_dist = vfp_t(this->t.z - this->b.z);
            left_area  = (x_dist * y_dist) + (x_dist * z_dist) + (z_dist * y_dist);

            x_dist = vfp_t(this->t.x) - s;
            right_area = (x_dist * y_dist) + (x_dist * z_dist) + (z_dist * y_dist);
            break;
        case axis_t::y_axis:
//            assert(move_mask(s < vfp_t(this->b.y)) == 0x0);
//            assert(move_mask(s > vfp_t(this->t.y)) == 0x0);

            x_dist = vfp_t(this->t.x - this->b.x);
            y_dist = s - vfp_t(this->b.y);
            z_dist = vfp_t(this->t.z - this->b.z);
            left_area  = (x_dist * y_dist) + (x_dist * z_dist) + (z_dist * y_dist);

            y_dist = vfp_t(this->t.y) - s;
            right_area = (x_dist * y_dist) + (x_dist * z_dist) + (z_dist * y_dist);
            break;
        case axis_t::z_axis:
//            assert(move_mask(s < vfp_t(this->b.z)) == 0x0);
//            assert(move_mask(s > vfp_t(this->t.z)) == 0x0);

            x_dist = vfp_t((this->t.x - this->b.x));
            y_dist = vfp_t((this->t.y - this->b.y));
            z_dist = s - vfp_t(this->b.z);
            left_area  = (x_dist * y_dist) + (x_dist * z_dist) + (z_dist * y_dist);

            z_dist = vfp_t(this->t.z) - s;
            right_area = (x_dist * y_dist) + (x_dist * z_dist) + (z_dist * y_dist);
            break;
        default:
            assert(false);
            break;
    }

    /* Calculate the surface area heuristic metric */
    /* cost of traversal + cost of intersection * (left cell count * left area + right cell count * right area) */
    return vfp_t(COST_OF_TRAVERSAL) + (vfp_t(COST_OF_INTERSECTION) * ((l * left_area) + (r * right_area)));
}


/**********************************************************
 
**********************************************************/
// inline float voxel::calculate_sah_cost(const float l, const float r, const float s, const axis_t normal) const
// {
//     /* Surface area of sub-divides */
//     float left_area, right_area;
//     float x_dist     = this->t.x - this->b.x;
//     float y_dist     = this->t.y - this->b.y;
//     float z_dist     = this->t.z - this->b.z;
// //    const float sa   = 1.0 / ((x_dist * y_dist) + (x_dist * z_dist) + (y_dist * z_dist));
//     switch (normal)
//     {
//         case axis_t::x_axis:
//             assert(s > this->b.x);
//             assert(s < this->t.x);
//             x_dist = s - this->b.x;
//             left_area  = (x_dist * y_dist) + (x_dist * z_dist) + (z_dist * y_dist);
//             x_dist = this->t.x - s;
//             right_area = (x_dist * y_dist) + (x_dist * z_dist) + (z_dist * y_dist);
//             break;
//         case axis_t::y_axis:
//             assert(s > this->b.y);
//             assert(s < this->t.y);
//             y_dist = s - this->b.y;
//             left_area  = (x_dist * y_dist) + (x_dist * z_dist) + (z_dist * y_dist);
//             y_dist = this->t.y - s;
//             right_area = (x_dist * y_dist) + (x_dist * z_dist) + (z_dist * y_dist);
//             break;
//         case axis_t::z_axis:
//             assert(s > this->b.z);
//             assert(s < this->t.z);
//             z_dist = s - this->b.z;
//             left_area  = (x_dist * y_dist) + (x_dist * z_dist) + (z_dist * y_dist);
//             z_dist = this->t.z - s;
//             right_area = (x_dist * y_dist) + (x_dist * z_dist) + (z_dist * y_dist);
//             break;
//         default:
//             assert(false);
//             break;
//     }

//     /* Calculate the surface area heuristic metric */
//     /* cost of traversal + cost of intersection * (left cell count * left area + right cell count * right area) */
//     return COST_OF_TRAVERSAL + (COST_OF_INTERSECTION * ((l * left_area) + (r * right_area)));
// }


///* Sequential SAH cost calculation */
//
///**********************************************************
// 
//**********************************************************/
//float voxel::calculate_sah_cost(const float s, const axis_t normal) const
//{
//    /* Surface area of sub-divides */
//    float left_area, right_area;
//    float x_dist = this->t.x - this->b.x;
//    float y_dist = this->t.y - this->b.y;
//    float z_dist = this->t.z - this->b.z;
//    switch (normal)
//    {
//        case axis_t::x_axis:
//            assert(s > this->b.x);
//            assert(s < this->t.x);
//            x_dist = s - this->b.x;
//            left_area  = (x_dist * y_dist) + (x_dist * z_dist) + (z_dist * y_dist);
//            x_dist = this->t.x - s;
//            right_area = (x_dist * y_dist) + (x_dist * z_dist) + (z_dist * y_dist);
//            break;
//        case axis_t::y_axis:
//            assert(s > this->b.y);
//            assert(s < this->t.y);
//            y_dist = s - this->b.y;
//            left_area  = (x_dist * y_dist) + (x_dist * z_dist) + (z_dist * y_dist);
//            y_dist = this->t.y - s;
//            right_area = (x_dist * y_dist) + (x_dist * z_dist) + (z_dist * y_dist);
//            break;
//        case axis_t::z_axis:
//            assert(s > this->b.z);
//            assert(s < this->t.z);
//            z_dist = s - this->b.z;
//            left_area  = (x_dist * y_dist) + (x_dist * z_dist) + (z_dist * y_dist);
//            z_dist = this->t.z - s;
//            right_area = (x_dist * y_dist) + (x_dist * z_dist) + (z_dist * y_dist);
//            break;
//        default:
//            assert(false);
//            break;
//    }
//  
//    /* Sub divide the objects */
//    float left_objects  = 0.0;
//    float right_objects = 0.0;
//    switch (normal)
//    {
//        case axis_t::x_axis:
//            for (primitive_list::const_iterator i=this->p->begin(); i!=this->p->end(); i++)
//            {
//                triangle *current_p = *i;
//                if (current_p->is_intersecting_x(s))
//                {
//                    left_objects++;
//                    right_objects++;
//                }
//                else if (current_p->get_x0() < s)
//                {
//                    left_objects++;
//                }
//                else
//                {
//                    right_objects++;
//                }
//            }
//            break;
//        case axis_t::y_axis:
//            for (primitive_list::const_iterator i=this->p->begin(); i!=this->p->end(); i++)
//            {
//                triangle *current_p = *i;
//                if (current_p->is_intersecting_y(s))
//                {
//                    left_objects++;
//                    right_objects++;
//                }
//                else if (current_p->get_y0() < s)
//                {
//                    left_objects++;
//                }
//                else
//                {
//                    right_objects++;
//                }
//            }
//            break;
//        case axis_t::z_axis:
//            for (primitive_list::const_iterator i=this->p->begin(); i!=this->p->end(); i++)
//            {
//                triangle *current_p = *i;
//                if (current_p->is_intersecting_z(s))
//                {
//                    left_objects++;
//                    right_objects++;
//                }
//                else if (current_p->get_z0() < s)
//                {
//                    left_objects++;
//                }
//                else
//                {
//                    right_objects++;
//                }
//            }
//            break;
//        default :
//            assert(false);
//            break;
//    }
//
//    float sah_cost = COST_OF_TRAVERSAL + (COST_OF_INTERSECTION * ((left_objects * left_area) + (right_objects * right_area)));
//
//    /* Calculate the surface area heuristic metric */
//    /* cost of traversal + cost of intersection * (left cell count * left area + right cell count * right area) */
//    return sah_cost;
//}


/* Sequential */

//float voxel::approximate_split_all_axis(float *s, axis_t * normal) const
//{
//    /* Find the best split position of the primitives */
//    float      best_split = MAX_DIST;
//    float      lowest_cost = *s;
//    axis_t    best_axis = axis_t::not_set;
//
//    for (primitive_list::const_iterator i=p.begin(); i!=p.end(); i++)
//    {
//        const triangle *current_p = *i;
//        if ((current_p->lowest_x() > this->b.x) && (current_p->lowest_x() < this->t.x))
//        {
//            float this_cost = this->calculate_sah_cost(current_p->lowest_x(), axis_t::x_axis);
//            if (this_cost < lowest_cost)
//            {
//                lowest_cost = this_cost;
//                best_split  = current_p->lowest_x();
//                best_axis   = axis_t::x_axis;
//            }
//        }
//        if ((current_p->highest_x() > this->b.x) && (current_p->highest_x() < this->t.x))
//        {
//            float this_cost = this->calculate_sah_cost(current_p->highest_x(), axis_t::x_axis);
//            if (this_cost < lowest_cost)
//            {
//                lowest_cost = this_cost;
//                best_split  = current_p->highest_x();
//                best_axis   = axis_t::x_axis;
//            }
//        }
//    }
//
//    for (primitive_list::const_iterator i=p.begin(); i!=p.end(); i++)
//    {
//        const triangle *current_p = *i;
//        if ((current_p->lowest_y() > this->b.y) && (current_p->lowest_y() < this->t.y))
//        {
//            float this_cost = this->calculate_sah_cost(current_p->lowest_y(), axis_t::y_axis);
//            if (this_cost < lowest_cost)
//            {
//                lowest_cost = this_cost;
//                best_split  = current_p->lowest_y();
//                best_axis   = axis_t::y_axis;
//            }
//        }
//        if ((current_p->highest_y() > this->b.y) && (current_p->highest_y() < this->t.y))
//        {
//            float this_cost = this->calculate_sah_cost(current_p->highest_y(), axis_t::y_axis);
//            if (this_cost < lowest_cost)
//            {
//                lowest_cost = this_cost;
//                best_split  = current_p->highest_y();
//                best_axis   = axis_t::y_axis;
//            }
//        }
//    }
//
//    for (primitive_list::const_iterator i=p.begin(); i!=p.end(); i++)
//    {
//        const triangle *current_p = *i;
//        if ((current_p->lowest_z() > this->b.z) && (current_p->lowest_z() < this->t.z))
//        {
//            float this_cost = this->calculate_sah_cost(current_p->lowest_z(), axis_t::z_axis);
//            if (this_cost < lowest_cost)
//            {
//                lowest_cost = this_cost;
//                best_split  = current_p->lowest_z();
//                best_axis   = axis_t::z_axis;
//            }
//        }
//        if ((current_p->highest_z() > this->b.z) && (current_p->highest_z() < this->t.z))
//        {
//            float this_cost = this->calculate_sah_cost(current_p->highest_z(), axis_t::z_axis);
//            if (this_cost < lowest_cost)
//            {
//                lowest_cost = this_cost;
//                best_split  = current_p->highest_z();
//                best_axis   = axis_t::z_axis;
//            }
//        }
//    }
//    
//    *s = lowest_cost;
//    *normal = best_axis;
//    
//    return best_split;
//}


/* Sequencial */

//float voxel::split_this_axis(float *s) const
//{
//    /* Find the best split position of the primitives */
//    float best_split = MAX_DIST;
//    float lowest_cost = *s;
//    switch (this->n)
//    {
//        case axis_t::x_axis:
//            for (primitive_list::const_iterator i=p.begin(); i!=p.end(); i++)
//            {
//                const triangle *current_p = *i;
//                if ((current_p->lowest_x() > this->b.x) && (current_p->lowest_x() < this->t.x))
//                {
//                    float this_cost = this->calculate_sah_cost(current_p->lowest_x());
//                    if (this_cost < lowest_cost)
//                    {
//                        lowest_cost = this_cost;
//                        best_split  = current_p->lowest_x();
//                    }
//                }
//                if ((current_p->highest_x() > this->b.x) && (current_p->highest_x() < this->t.x))
//                {
//                    float this_cost = this->calculate_sah_cost(current_p->highest_x());
//                    if (this_cost < lowest_cost)
//                    {
//                        lowest_cost = this_cost;
//                        best_split  = current_p->highest_x();
//                    }
//                }
//            }
//            break;
//        case axis_t::y_axis:
//            for (primitive_list::const_iterator i=p.begin(); i!=p.end(); i++)
//            {
//                const triangle *current_p = *i;
//                if ((current_p->lowest_y() > this->b.y) && (current_p->lowest_y() < this->t.y))
//                {
//                    float this_cost = this->calculate_sah_cost(current_p->lowest_y());
//                    if (this_cost < lowest_cost)
//                    {
//                        lowest_cost = this_cost;
//                        best_split  = current_p->lowest_y();
//                    }
//                }
//                if ((current_p->highest_y() > this->b.y) && (current_p->highest_y() < this->t.y))
//                {
//                    float this_cost = this->calculate_sah_cost(current_p->highest_y());
//                    if (this_cost < lowest_cost)
//                    {
//                        lowest_cost = this_cost;
//                        best_split  = current_p->highest_y();
//                    }
//                }
//            }
//            break;
//        case axis_t::z_axis:
//            for (primitive_list::const_iterator i=p.begin(); i!=p.end(); i++)
//            {
//                const triangle *current_p = *i;
//                if ((current_p->lowest_z() > this->b.z) && (current_p->lowest_z() < this->t.z))
//                {
//                    float this_cost = this->calculate_sah_cost(current_p->lowest_z());
//                    if (this_cost < lowest_cost)
//                    {
//                        lowest_cost = this_cost;
//                        best_split  = current_p->lowest_z();
//                    }
//                }
//                if ((current_p->highest_z() > this->b.z) && (current_p->highest_z() < this->t.z))
//                {
//                    float this_cost = this->calculate_sah_cost(current_p->highest_z());
//                    if (this_cost < lowest_cost)
//                    {
//                        lowest_cost = this_cost;
//                        best_split  = current_p->highest_z();
//                    }
//                }
//            }
//            break;
//        default :
//            assert(false);
//            break;
//    }
//    
//    *s = lowest_cost;
//    
//    return best_split;
//}


///**********************************************************
// 
//**********************************************************/
//float voxel::calculate_sah_cost(const float s) const
//{
//    /* Surface area of sub-divides */
//    float left_area, right_area;
//    float x_dist = this->t.x - this->b.x;
//    float y_dist = this->t.y - this->b.y;
//    float z_dist = this->t.z - this->b.z;
//    switch (this->n)
//    {
//        case axis_t::x_axis:
//            assert(s > this->b.x);
//            assert(s < this->t.x);
//            x_dist = s - this->b.x;
//            left_area  = (x_dist * y_dist) + (x_dist * z_dist) + (z_dist * y_dist);
//            x_dist = this->t.x - s;
//            right_area = (x_dist * y_dist) + (x_dist * z_dist) + (z_dist * y_dist);
//            break;
//        case axis_t::y_axis:
//            assert(s > this->b.y);
//            assert(s < this->t.y);
//            y_dist = s - this->b.y;
//            left_area  = (x_dist * y_dist) + (x_dist * z_dist) + (z_dist * y_dist);
//            y_dist = this->t.y - s;
//            right_area = (x_dist * y_dist) + (x_dist * z_dist) + (z_dist * y_dist);
//            break;
//        case axis_t::z_axis:
//            assert(s > this->b.z);
//            assert(s < this->t.z);
//            z_dist = s - this->b.z;
//            left_area  = (x_dist * y_dist) + (x_dist * z_dist) + (z_dist * y_dist);
//            z_dist = this->t.z - s;
//            right_area = (x_dist * y_dist) + (x_dist * z_dist) + (z_dist * y_dist);
//            break;
//        default:
//            assert(false);
//            break;
//    }
//  
//    /* Sub divide the objects */
//    float left_objects  = 0.0;
//    float right_objects = 0.0;
//    switch (this->n)
//    {
//        case axis_t::x_axis:
//            for (primitive_list::const_iterator i=this->p->begin(); i!=this->p->end(); i++)
//            {
//                triangle *current_p = *i;
//                if (current_p->is_intersecting_x(s))
//                {
//                    left_objects++;
//                    right_objects++;
//                }
//                else if (current_p->get_x0() < s)
//                {
//                    left_objects++;
//                }
//                else
//                {
//                    right_objects++;
//                }
//            }
//            break;
//        case axis_t::y_axis:
//            for (primitive_list::const_iterator i=this->p->begin(); i!=this->p->end(); i++)
//            {
//                triangle *current_p = *i;
//                if (current_p->is_intersecting_y(s))
//                {
//                    left_objects++;
//                    right_objects++;
//                }
//                else if (current_p->get_y0() < s)
//                {
//                    left_objects++;
//                }
//                else
//                {
//                    right_objects++;
//                }
//            }
//            break;
//        case axis_t::z_axis:
//            for (primitive_list::const_iterator i=this->p->begin(); i!=this->p->end(); i++)
//            {
//                triangle *current_p = *i;
//                if (current_p->is_intersecting_z(s))
//                {
//                    left_objects++;
//                    right_objects++;
//                }
//                else if (current_p->get_z0() < s)
//                {
//                    left_objects++;
//                }
//                else
//                {
//                    right_objects++;
//                }
//            }
//            break;
//        default :
//            assert(false);
//            break;
//    }
//
//    float sah_cost = COST_OF_TRAVERSAL + (COST_OF_INTERSECTION * ((left_objects * left_area) + (right_objects * right_area)));
//
//    /* Calculate the surface area heuristic metric */
//    /* cost of traversal + cost of intersection * (left cell count * left area + right cell count * right area) */
//    return sah_cost;
//}
//
//
///**********************************************************
// 
//**********************************************************/
//float voxel::calculate_sah_cost(const float s, const float l, const float r) const
//{
//    /* Surface area of sub-divides */
//    float left_area, right_area;
//    float x_dist = this->t.x - this->b.x;
//    float y_dist = this->t.y - this->b.y;
//    float z_dist = this->t.z - this->b.z;
//    switch (this->n)
//    {
//        case axis_t::x_axis:
//            assert(s > this->b.x);
//            assert(s < this->t.x);
//            x_dist = s - this->b.x;
//            left_area  = (x_dist * y_dist) + (x_dist * z_dist) + (z_dist * y_dist);
//            x_dist = this->t.x - s;
//            right_area = (x_dist * y_dist) + (x_dist * z_dist) + (z_dist * y_dist);
//            break;
//        case axis_t::y_axis:
//            assert(s > this->b.y);
//            assert(s < this->t.y);
//            y_dist = s - this->b.y;
//            left_area  = (x_dist * y_dist) + (x_dist * z_dist) + (z_dist * y_dist);
//            y_dist = this->t.y - s;
//            right_area = (x_dist * y_dist) + (x_dist * z_dist) + (z_dist * y_dist);
//            break;
//        case axis_t::z_axis:
//            assert(s > this->b.z);
//            assert(s < this->t.z);
//            z_dist = s - this->b.z;
//            left_area  = (x_dist * y_dist) + (x_dist * z_dist) + (z_dist * y_dist);
//            z_dist = this->t.z - s;
//            right_area = (x_dist * y_dist) + (x_dist * z_dist) + (z_dist * y_dist);
//            break;
//        default:
//            assert(false);
//            break;
//    }
//  
//    float sah_cost = COST_OF_TRAVERSAL + (COST_OF_INTERSECTION * ((l * left_area) + (r * right_area)));
//
//    /* Calculate the surface area heuristic metric */
//    /* cost of traversal + cost of intersection * (left cell count * left area + right cell count * right area) */
//    return sah_cost;
//}
//
//


///**********************************************************
// 
//**********************************************************/
//float voxel::split_all_axis(float *s, axis_t *normal)
//{
//    /* Find the best split position of the primitives */
//    axis_t    best_axis   = axis_t::not_set;
//    float  best_split  = axis_t::not_set;
//    float  lowest_cost = *s;
//    
//     /* Initial primitive split */
//    unsigned size_of_prim = this->p->size();
//    float     right = size_of_prim;
//    float     left  = 0;
//    float     guess;
//    float     last_guess = MAX_DIST;
//    
//    float x_dist = this->t.x - this->b.x;
//    float y_dist = this->t.y - this->b.y;
//    float z_dist = this->t.z - this->b.z;
//    
//    /* Elements for evaluation */
//    float lo_x, lo_y, lo_z, hi_x, hi_y, hi_z;
//    
//    bb_point_t *bb_points = new bb_point_t [(size_of_prim << 1)];
//    
//    unsigned index=0;
//    for (primitive_list::iterator i=this->p->begin(); i!= this->p->end(); i++)
//    {
//        bb_points[index++] = bb_point_t((*i)->lowest_x(), false);
//        bb_points[index++] = bb_point_t((*i)->highest_x(), true);
//    }
//    
//    quick_sort(bb_points,  0, ((size_of_prim << 1) - 1));
//    
//    /* Move past any out of range triangles */
//    unsigned low_index  =  0;
//    unsigned high_index =  0;
//    lo_x = -MAX_DIST;
//    while ((lo_x <= this->b.x) && (low_index < (size_of_prim << 1)))
//    {
//        lo_x = bb_points[low_index].p;
//        /* This will cause primitives to be split */
//        if (bb_points[low_index].hi)
//        {
//            right--;
//        }
//        else
//        {
//            left++;
//        }
//        low_index++;
//    }
//    
//    /* Evaluate the SAH from left to right */
//    hi_x = -MAX_DIST;
//    while (true)
//    {
//        /* Get point to evaluate or MAX_DIST if non remain */
//        if ((low_index == (size_of_prim << 1)) || (bb_points[low_index].p >= this->t.x))
//        {
//            break;
//        }
//        else
//        {
//            guess = bb_points[low_index].p;
//        }
//        
//        if (bb_points[low_index].hi)
//        {
//            right--;
//        }
//
//        /* Evaluate unique points */
//        if (last_guess != guess)
//        {
//            assert(guess > this->b.x);
//            assert(guess < this->t.x);
//            x_dist = guess - this->b.x;
//            float left_area  = (x_dist * y_dist) + (x_dist * z_dist) + (z_dist * y_dist);
//            x_dist = this->t.x - guess;
//            float right_area = (x_dist * y_dist) + (x_dist * z_dist) + (z_dist * y_dist);
//
//            float this_cost = COST_OF_TRAVERSAL + (COST_OF_INTERSECTION * ((left * left_area) + (right * right_area)));
//            if (this_cost < lowest_cost)
//            {
//                lowest_cost = this_cost;
//                best_split  = guess;
//                best_axis   = axis_t::x_axis;
//            }
//            last_guess = guess;
//        }
//
//        /* Advance iterators and primitive counts */
//        if (!bb_points[low_index].hi)
//        {
//            left++;
//        }
//        low_index++;
//    }
//
//    right = size_of_prim;
//    left  = 0;
//    last_guess = MAX_DIST;
//    x_dist = this->t.x - this->b.x;
//    index=0;
//    for (primitive_list::iterator i=this->p->begin(); i!= this->p->end(); i++)
//    {
//        bb_points[index++] = bb_point_t((*i)->lowest_y(), false);
//        bb_points[index++] = bb_point_t((*i)->highest_y(), true);
//    }
//    
//    quick_sort(bb_points,  0, ((size_of_prim << 1) - 1));
//    
//    /* Move past any out of range triangles */
//    low_index  =  0;
//    high_index =  0;
//    lo_y = -MAX_DIST;
//    while ((lo_y <= this->b.y) && (low_index < (size_of_prim << 1)))
//    {
//        lo_y = bb_points[low_index].p;
//        /* This will cause primitives to be split */
//        if (bb_points[low_index].hi)
//        {
//            right--;
//        }
//        else
//        {
//            left++;
//        }
//        low_index++;
//    }
//    
//    /* Evaluate the SAH from left to right */
//    hi_y = -MAX_DIST;
//    while (true)
//    {
//        /* Get point to evaluate or MAX_DIST if non remain */
//        if ((low_index == (size_of_prim << 1)) || (bb_points[low_index].p >= this->t.y))
//        {
//            break;
//        }
//        else
//        {
//            guess = bb_points[low_index].p;
//        }
//        
//        if (bb_points[low_index].hi)
//        {
//            right--;
//        }
//
//        /* Evaluate unique points */
//        if (last_guess != guess)
//        {
//            assert(guess > this->b.y);
//            assert(guess < this->t.y);
//            y_dist = guess - this->b.y;
//            float left_area  = (x_dist * y_dist) + (x_dist * z_dist) + (z_dist * y_dist);
//            y_dist = this->t.y - guess;
//            float right_area = (x_dist * y_dist) + (x_dist * z_dist) + (z_dist * y_dist);
//
//            float this_cost = COST_OF_TRAVERSAL + (COST_OF_INTERSECTION * ((left * left_area) + (right * right_area)));
//            if (this_cost < lowest_cost)
//            {
//                lowest_cost = this_cost;
//                best_split  = guess;
//                best_axis   = axis_t::y_axis;
//            }
//            last_guess = guess;
//        }
//
//        /* Advance iterators and primitive counts */
//        if (!bb_points[low_index].hi)
//        {
//            left++;
//        }
//        low_index++;
//    }
//
//    right = size_of_prim;
//    left  = 0;
//    last_guess = MAX_DIST;
//    y_dist = this->t.y - this->b.y;
//    index=0;
//
//    for (primitive_list::iterator i=this->p->begin(); i!= this->p->end(); i++)
//    {
//        bb_points[index++] = bb_point_t((*i)->lowest_z(), false);
//        bb_points[index++] = bb_point_t((*i)->highest_z(), true);
//    }
//    
//    quick_sort(bb_points,  0, ((size_of_prim << 1) - 1));
//    
//    /* Move past any out of range triangles */
//    low_index  =  0;
//    high_index =  0;
//    lo_z = -MAX_DIST;
//    while ((lo_z <= this->b.z) && (low_index < (size_of_prim << 1)))
//    {
//        lo_z = bb_points[low_index].p;
//        /* This will cause primitives to be split */
//        if (bb_points[low_index].hi)
//        {
//            right--;
//        }
//        else
//        {
//            left++;
//        }
//        low_index++;
//    }
//    
//    /* Evaluate the SAH from left to right */
//    hi_z = -MAX_DIST;
//    while (true)
//    {
//        /* Get point to evaluate or MAX_DIST if non remain */
//        if ((low_index == (size_of_prim << 1)) || (bb_points[low_index].p >= this->t.z))
//        {
//            break;
//        }
//        else
//        {
//            guess = bb_points[low_index].p;
//        }
//        
//        if (bb_points[low_index].hi)
//        {
//            right--;
//        }
//
//        /* Evaluate unique points */
//        if (last_guess != guess)
//        {
//            assert(guess > this->b.z);
//            assert(guess < this->t.z);
//            z_dist = guess - this->b.z;
//            float left_area  = (x_dist * y_dist) + (x_dist * z_dist) + (z_dist * y_dist);
//            z_dist = this->t.z - guess;
//            float right_area = (x_dist * y_dist) + (x_dist * z_dist) + (z_dist * y_dist);
//
//            float this_cost = COST_OF_TRAVERSAL + (COST_OF_INTERSECTION * ((left * left_area) + (right * right_area)));
//            if (this_cost < lowest_cost)
//            {
//                lowest_cost = this_cost;
//                best_split  = guess;
//                best_axis   = axis_t::z_axis;
//            }
//            last_guess = guess;
//        }
//
//        /* Advance iterators and primitive counts */
//        if (!bb_points[low_index].hi)
//        {
//            left++;
//        }
//        low_index++;
//    }
//    
//    delete [] bb_points;
//    *normal = best_axis;
//    *s = lowest_cost;
//    return best_split;
//}


/* Bounding box and quciksort based */

/**********************************************************
 
**********************************************************/
//float voxel::split_all_axis(float *s, axis_t *normal)
//{
//    /* Find the best split position of the primitives */
//    axis_t    best_axis   = axis_t::not_set;
//    float      best_split  = axis_t::not_set;
//    float      lowest_cost = *s;
//    
//     /* Initial primitive split */
//    unsigned size_of_prim = this->p->size();
//    float     right = size_of_prim;
//    float     left  = 0;
//    float     guess;
//    float     last_guess = MAX_DIST;
//    
//    float x_dist = this->t.x - this->b.x;
//    float y_dist = this->t.y - this->b.y;
//    float z_dist = this->t.z - this->b.z;
//    
//    /* Elements for evaluation */
//    float lo_x, lo_y, lo_z, hi_x, hi_y, hi_z;
//    
//    bb_point_t *low_points, *high_points;
//    low_points = new bb_point_t [size_of_prim];
//    high_points = new bb_point_t [size_of_prim];
//    
//    unsigned index=0;
//    for (primitive_list::iterator i=this->p->begin(); i!= this->p->end(); i++)
//    {
//        low_points [index  ] = bb_point_t((*i)->lowest_x(), false);
//        high_points[index++] = bb_point_t((*i)->highest_x(), true);
//    }
//    
//    quick_sort(low_points,  0, (size_of_prim-1));
//    quick_sort(high_points, 0, (size_of_prim-1));
//    
//    /* Move past any out of range triangles */
//    unsigned low_index  =  0;
//    unsigned high_index =  0;
//    lo_x = -MAX_DIST;
//    while ((lo_x <= this->b.x) && (low_index < size_of_prim))
//    {
//        lo_x = low_points[low_index].p;
//        low_index++;
//        /* This will cause primitives to be split */
//        left++;
//    }
//    
//    /* Evaluate the SAH from left to right */
//    hi_x = -MAX_DIST;
//    while (true)
//    {
//        /* Get point to evaluate or MAX_DIST if non remain */
//        if (low_index == size_of_prim)
//        {
//            lo_x = MAX_DIST;
//        }
//        else
//        {
//            lo_x = low_points[low_index].p;
//        }
//        
//        if (high_index == size_of_prim)
//        {
//            hi_x = MAX_DIST;
//        }
//        else
//        {
//            hi_x = high_points[high_index].p;
//        }
//
//        /* Pick the point furthest left and adjust primitive counts */
//        if (lo_x < hi_x)
//        {
//            guess = lo_x;
//        }
//        else
//        {
//            guess = hi_x;
//            right--;
//        }
//
//        /* Break when any further points will be out of bounds */
//        if (guess >= this->t.x)
//        {
//            break;
//        }
//        
//        /* Evaluate unique points */
//        if (last_guess != guess)
//        {
//            assert(guess > this->b.x);
//            assert(guess < this->t.x);
//            x_dist = guess - this->b.x;
//            float left_area  = (x_dist * y_dist) + (x_dist * z_dist) + (z_dist * y_dist);
//            x_dist = this->t.x - guess;
//            float right_area = (x_dist * y_dist) + (x_dist * z_dist) + (z_dist * y_dist);
//
//            float this_cost = COST_OF_TRAVERSAL + (COST_OF_INTERSECTION * ((left * left_area) + (right * right_area)));
//            if (this_cost < lowest_cost)
//            {
//                lowest_cost = this_cost;
//                best_split  = guess;
//                best_axis   = axis_t::x_axis;
//            }
//            last_guess = guess;
//        }
//
//        /* Advance iterators and primitive counts */
//        if (lo_x < hi_x)
//        {
//            left++;
//            low_index++;
//        }
//        else
//        {
//            high_index++;
//        }
//    }
//
//    right = size_of_prim;
//    left  = 0;
//    last_guess = MAX_DIST;
//    x_dist = this->t.x - this->b.x;
//    index=0;
//    for (primitive_list::iterator i=this->p->begin(); i!= this->p->end(); i++)
//    {
//        low_points [index  ] = bb_point_t((*i)->lowest_y(), false);
//        high_points[index++] = bb_point_t((*i)->highest_y(), true);
//    }
//    
//    quick_sort(low_points,  0, (size_of_prim-1));
//    quick_sort(high_points, 0, (size_of_prim-1));
//    
//    /* Move past any out of range triangles */
//    low_index  = 0;
//    high_index = 0;
//    lo_y = -MAX_DIST;
//    while ((lo_y <= this->b.y) && (low_index < size_of_prim))
//    {
//        lo_y = low_points[low_index].p;
//        low_index++;
//        /* This will cause primitives to be split */
//        left++;
//    }
//    
//    /* Evaluate the SAH from left to right */
//    hi_y = -MAX_DIST;
//    while (true)
//    {
//        /* Get point to evaluate or MAX_DIST if non remain */
//        if (low_index == size_of_prim)
//        {
//            lo_y = MAX_DIST;
//        }
//        else
//        {
//            lo_y = low_points[low_index].p;
//        }
//        
//        if (high_index == size_of_prim)
//        {
//            hi_y = MAX_DIST;
//        }
//        else
//        {
//            hi_y = high_points[high_index].p;
//        }
//
//        /* Pick the point furthest left and adjust primitive counts */
//        if (lo_y < hi_y)
//        {
//            guess = lo_y;
//        }
//        else
//        {
//            guess = hi_y;
//            right--;
//        }
//
//        /* Break when any further points will be out of bounds */
//        if (guess >= this->t.y)
//        {
//            break;
//        }
//        
//        /* Evaluate unique points */
//        if (last_guess != guess)
//        {
//            assert(guess > this->b.y);
//            assert(guess < this->t.y);
//            y_dist = guess - this->b.y;
//            float left_area  = (x_dist * y_dist) + (x_dist * z_dist) + (z_dist * y_dist);
//            y_dist = this->t.y - guess;
//            float right_area = (x_dist * y_dist) + (x_dist * z_dist) + (z_dist * y_dist);
//
//            float this_cost = COST_OF_TRAVERSAL + (COST_OF_INTERSECTION * ((left * left_area) + (right * right_area)));
//            if (this_cost < lowest_cost)
//            {
//                lowest_cost = this_cost;
//                best_split  = guess;
//                best_axis   = axis_t::y_axis;
//            }
//            last_guess = guess;
//        }
//
//        /* Advance iterators and primitive counts */
//        if (lo_y < hi_y)
//        {
//            left++;
//            low_index++;
//        }
//        else
//        {
//            high_index++;
//        }
//    }
//
//    right = size_of_prim;
//    left  = 0;
//    last_guess = MAX_DIST;
//    y_dist = this->t.y - this->b.y;
//    index=0;
//    for (primitive_list::iterator i=this->p->begin(); i!= this->p->end(); i++)
//    {
//        low_points [index  ] = bb_point_t((*i)->lowest_z(), false);
//        high_points[index++] = bb_point_t((*i)->highest_z(), true);
//    }
//    
//    quick_sort(low_points,  0, (size_of_prim-1));
//    quick_sort(high_points, 0, (size_of_prim-1));
//    
//    /* Move past any out of range triangles */
//    low_index  = 0;
//    high_index = 0;
//    lo_z = -MAX_DIST;
//    while ((lo_z <= this->b.z) && (low_index < size_of_prim))
//    {
//        lo_z = low_points[low_index].p;
//        low_index++;
//        /* This will cause primitives to be split */
//        left++;
//    }
//    
//    /* Evaluate the SAH from left to right */
//    hi_z = -MAX_DIST;
//    while (true)
//    {
//        /* Get point to evaluate or MAX_DIST if non remain */
//        if (low_index == size_of_prim)
//        {
//            lo_z = MAX_DIST;
//        }
//        else
//        {
//            lo_z = low_points[low_index].p;
//        }
//        
//        if (high_index == size_of_prim)
//        {
//            hi_z = MAX_DIST;
//        }
//        else
//        {
//            hi_z = high_points[high_index].p;
//        }
//
//        /* Pick the point furthest left and adjust primitive counts */
//        if (lo_z < hi_z)
//        {
//            guess = lo_z;
//        }
//        else
//        {
//            guess = hi_z;
//            right--;
//        }
//
//        /* Break when any further points will be out of bounds */
//        if (guess >= this->t.z)
//        {
//            break;
//        }
//        
//        /* Evaluate unique points */
//        if (last_guess != guess)
//        {
//            assert(guess > this->b.z);
//            assert(guess < this->t.z);
//            z_dist = guess - this->b.z;
//            float left_area  = (x_dist * y_dist) + (x_dist * z_dist) + (z_dist * y_dist);
//            z_dist = this->t.z - guess;
//            float right_area = (x_dist * y_dist) + (x_dist * z_dist) + (z_dist * y_dist);
//
//            float this_cost = COST_OF_TRAVERSAL + (COST_OF_INTERSECTION * ((left * left_area) + (right * right_area)));
//            if (this_cost < lowest_cost)
//            {
//                lowest_cost = this_cost;
//                best_split  = guess;
//                best_axis   = axis_t::z_axis;
//            }
//            last_guess = guess;
//        }
//
//        /* Advance iterators and primitive counts */
//        if (lo_z < hi_z)
//        {
//            left++;
//            low_index++;
//        }
//        else
//        {
//            high_index++;
//        }
//    }
//    
//    delete [] low_points;
//    delete [] high_points;
//    *normal = best_axis;
//    *s = lowest_cost;
//    return best_split;
//}


//float voxel::approximate_split_all_axis(float *s, axis_t * normal) const
//{
//    /* Find the best split position of the primitives */
//    float      best_split     = MAX_DIST;
//    float      lowest_cost    = *s;
//    axis_t    best_axis      = axis_t::not_set;
//    
//    /* Adaptive sampling bin sizes */
//    const float adapt_bin_size   = this->p->size() * (2.0/9.0);
//    
//    /* Voxel dimensions */
//    const float xw       = this->t.x - this->b.x;
//    const float yw       = this->t.y - this->b.y;
//    const float zw       = this->t.z - this->b.z;
//    const float xb_yw    = this->b.x * yw;
//    const float xb_zw    = this->b.x * zw;
//    const float xt_yw    = this->t.x * yw;
//    const float xt_zw    = this->t.x * zw;
//    const float yb_xw    = this->b.y * xw;
//    const float yb_zw    = this->b.y * zw;
//    const float yt_xw    = this->t.y * xw;
//    const float yt_zw    = this->t.y * zw;
//    const float zb_xw    = this->b.z * xw;
//    const float zb_yw    = this->b.z * yw;
//    const float zt_xw    = this->t.z * xw;
//    const float zt_yw    = this->t.z * yw;
//    const float xw_yw    = xw * yw;
//    const float xw_zw    = xw * zw;
//    const float yw_zw    = yw * zw;
//    const float xw_p_yw  = xw + yw;
//    const float xw_p_zw  = xw + zw;
//    const float yw_p_zw  = yw + zw;
//   
//    /* Pick fixed points to evaluate */
//    float divide_width   = xw * (1.0 / 9.0);
//    float evaluate       = this->t.x;
//    
//    /* Fixed points samples and adaptive bin boundaries */
//    float cl_x[9];
//    float cr_x[9];
//    float bins_per_sample[9];
//    float adaptive_width[9];
//    cl_x[8]             = this->p->size();
//    cr_x[8]             = 0.0;
//    bins_per_sample[8]  = 8.0;
//    
//    float sample[8];
//    for (int i = 0; i < 8; i++)
//    {
//        sample[i] = this->b.x + (divide_width * static_cast<float>(i + 1));
//    }
//    this->count_primitives(cl_x, cr_x, sample, 8, axis_t::x_axis);
//    
//    for (int i = 7; i >= 0; i--)
//    {
////        evaluate -= divide_width;
////        cl_x[i] = count_primitives(&cr_x[i], evaluate, axis_t::x_axis);
////        cout << "At: " << evaluate << " found " << cl_x[i] << ", " << cr_x[i] << endl;
//        bins_per_sample[i] = int((cl_x[i] - cr_x[i]) / adapt_bin_size) + 4;
//
//
//        /* Fix adaptive sample width per bin */
//        bins_per_sample[i+1]   -= bins_per_sample[i];
//        adaptive_width[i+1]     = divide_width / (bins_per_sample[i+1] + 1.0);
//    }
//    adaptive_width[0]   = divide_width / (bins_per_sample[0] + 1.0);
//    
//    /* Take adaptive samples */
//    float cl_ax[9];
//    float cr_ax[9];
//    int adaptive_offset = 0;
//    for (int i = 0; i < 9; i++)
//    {
//        evaluate = this->b.x + (i * divide_width);
//        for (int j = adaptive_offset; j < (bins_per_sample[i] + adaptive_offset); j++)
//        {
//            evaluate   += adaptive_width[i];
////            cl_ax[j]    = count_primitives(&cr_ax[j], evaluate, axis_t::x_axis);
//            sample[j] = evaluate;
//        }
//        adaptive_offset += bins_per_sample[i];
//    }
//    this->count_primitives(cl_ax, cr_ax, sample, 8, axis_t::x_axis);
//    
//    /* Evaluate SAH minima for each sample as -b/2a */
//    adaptive_offset = 0;
//    float last_cl    = 0.0;
//    float last_cr    = this->p->size();
//    float xs         = this->b.x;
//    for (int i = 0; i < 9; i++)
//    {
//        /* Process any adaptive samples in this bin first because they will be encountered first */
//        for (int j = adaptive_offset; j < (bins_per_sample[i] + adaptive_offset); j++)
//        {
//            /* Bin primitive gradients */
//            const float clg = (cl_ax[j] - last_cl) / adaptive_width[i];
//            const float crg = (cr_ax[j] - last_cr) / adaptive_width[i];
//            
//            /* Sample location */
//            const float xs_yw    = xs * yw;
//            const float xs_zw    = xs * zw;
//            
//            /* Minima point bound to this bin */
//            const float a    = 2.0 * ((clg * yw_p_zw) - (crg * yw_p_zw));
//            const float b    =   ( last_cl * yw_p_zw) - (clg * (xb_yw + xb_zw - yw_zw + xs_yw + xs_zw))
//                              + (-last_cr * yw_p_zw) + (crg * (xt_yw + xt_zw + yw_zw + xs_yw + xs_zw));
//            
//            const float xm   = max(xs, min((xs + adaptive_width[i]), (b / a)));
//
//            /* Cost at minima */
//            const float xmb  = xm - this->b.x;
//            const float xmt  = this->t.x - xm;
//
//            const float cost = COST_OF_TRAVERSAL + (COST_OF_INTERSECTION * 
//                              (((last_cl + (clg * (xm - xs))) * ((xmb * yw) + (xmb * zw) + yw_zw))  + 
//                               ((last_cr + (crg * (xm - xs))) * ((xmt * yw) + (xmt * zw) + yw_zw))));
//            
//            if (cost < lowest_cost)
//            {
//                lowest_cost = cost;
//                best_axis   = axis_t::x_axis;
//                best_split  = xm;
//            }
//            
//            last_cl = cl_ax[j];
//            last_cr = cr_ax[j];
//            xs     += adaptive_width[i];
//        }
//        adaptive_offset += bins_per_sample[i];
//
//        /* Bin primitive gradients */
//        const float clg = (cl_x[i] - last_cl) / adaptive_width[i];
//        const float crg = (cr_x[i] - last_cr) / adaptive_width[i];
//      
//        /* Sample location */
//        const float xs_yw    = xs * yw;
//        const float xs_zw    = xs * zw;
//      
//        /* Minima point bound to this bin */
//        const float a    = 2.0 * ((clg * yw_p_zw) - (crg * yw_p_zw));
//        const float b    =   ( last_cl * yw_p_zw) - (clg * (xb_yw + xb_zw - yw_zw + xs_yw + xs_zw))
//                          + (-last_cr * yw_p_zw) + (crg * (xt_yw + xt_zw + yw_zw + xs_yw + xs_zw));
//      
//        const float xm   = max(xs, min((xs + adaptive_width[i]), (b / a)));
//      
//        /* Cost at minima */
//        const float xmb  = xm - this->b.x;
//        const float xmt  = this->t.x - xm;
//
//        const float cost = COST_OF_TRAVERSAL + (COST_OF_INTERSECTION * 
//                          (((last_cl + (clg * (xm - xs))) * ((xmb * yw) + (xmb * zw) + yw_zw))  + 
//                           ((last_cr + (crg * (xm - xs))) * ((xmt * yw) + (xmt * zw) + yw_zw))));
//      
//        if (cost < lowest_cost)
//        {
//            lowest_cost = cost;
//            best_axis   = axis_t::x_axis;
//            best_split  = xm;
//        }
//      
//        last_cl = cl_x[i];
//        last_cr = cr_x[i];
//        xs     += adaptive_width[i];
//    }
//    
//    /* Pick fixed points to evaluate */
//    divide_width   = yw * (1.0 / 9.0);
//    evaluate       = this->t.y;
//    
//    /* Fixed points samples and adaptive bin boundaries */
//    float cl_y[9];
//    float cr_y[9];
//    cl_y[8]             = this->p->size();
//    cr_y[8]             = 0.0;
//    bins_per_sample[8]  = 8.0;
//
//    for (int i = 0; i < 8; i++)
//    {
//        sample[i] = this->b.y + (divide_width * static_cast<float>(i + 1));
//    }
//    this->count_primitives(cl_y, cr_y, sample, 8, axis_t::y_axis);
//
//    for (int i = 7; i >= 0; i--)
//    {
////        evaluate -= divide_width;
////        cl_y[i] = count_primitives(&cr_y[i], evaluate, axis_t::y_axis);
//        bins_per_sample[i] = int((cl_y[i] - cr_y[i]) / adapt_bin_size) + 4;
//
//
//        /* Fix adaptive sample width per bin */
//        bins_per_sample[i+1]   -= bins_per_sample[i];
//        adaptive_width[i+1]     = divide_width / (bins_per_sample[i+1] + 1.0);
//    }
//    
//    adaptive_width[0]   = divide_width / (bins_per_sample[0] + 1.0);
//    
//    /* Take adaptive samples */
//    float cl_ay[9];
//    float cr_ay[9];
//    adaptive_offset = 0;
//    for (int i = 0; i < 9; i++)
//    {
//        evaluate = this->b.y + (i * divide_width);
//        for (int j = adaptive_offset; j < (bins_per_sample[i] + adaptive_offset); j++)
//        {
//            evaluate   += adaptive_width[i];
////            cl_ay[j]    = count_primitives(&cr_ay[j], evaluate, axis_t::y_axis);
//            sample[j] = evaluate;
//        }
//        adaptive_offset += bins_per_sample[i];
//    }
//    this->count_primitives(cl_ay, cr_ay, sample, 8, axis_t::y_axis);
//    
//    /* Evaluate SAH minima for each sample as -b/2a */
//    adaptive_offset = 0;
//         last_cl    = 0.0;
//         last_cr    = this->p->size();
//    float ys         = this->b.y;
//    for (int i = 0; i < 9; i++)
//    {
//        /* Process any adaptive samples in this bin first because they will be encountered first */
//        for (int j = adaptive_offset; j < (bins_per_sample[i] + adaptive_offset); j++)
//        {
//            /* Bin primitive gradients */
//            const float clg = (cl_ay[j] - last_cl) / adaptive_width[i];
//            const float crg = (cr_ay[j] - last_cr) / adaptive_width[i];
//            
//            /* Sample location */
//            const float ys_xw    = ys * xw;
//            const float ys_zw    = ys * zw;
//            
//            /* Minima point bound to this bin */
//            const float a    = 2.0 * ((clg * xw_p_zw) - (crg * xw_p_zw));
//            const float b    =   ( last_cl * xw_p_zw) - (clg * (yb_xw + yb_zw - xw_zw + ys_xw + ys_zw))
//                              + (-last_cr * xw_p_zw) + (crg * (yt_xw + yt_zw + xw_zw + ys_xw + ys_zw));
//            
//            const float ym   = max(ys, min((ys + adaptive_width[i]), (b / a)));
//            
//            /* Cost at minima */
//            const float ymb  = ym - this->b.y;
//            const float ymt  = this->t.y - ym;
//
//            const float cost = COST_OF_TRAVERSAL + (COST_OF_INTERSECTION * 
//                              (((last_cl + (clg * (ym - ys))) * ((ymb * xw) + (ymb * zw) + xw_zw))  + 
//                               ((last_cr + (crg * (ym - ys))) * ((ymt * xw) + (ymt * zw) + xw_zw))));
//            
//            if (cost < lowest_cost)
//            {
//                lowest_cost = cost;
//                best_axis   = axis_t::y_axis;
//                best_split  = ym;
//            }
//            
//            last_cl = cl_ay[j];
//            last_cr = cr_ay[j];
//            ys     += adaptive_width[i];
//        }
//        adaptive_offset += bins_per_sample[i];
//
//        /* Bin primitive gradients */
//        const float clg = (cl_y[i] - last_cl) / adaptive_width[i];
//        const float crg = (cr_y[i] - last_cr) / adaptive_width[i];
//      
//        /* Sample location */
//        const float ys_xw    = ys * xw;
//        const float ys_zw    = ys * zw;
//      
//        /* Minima point bound to this bin */
//        const float a    = 2.0 * ((clg * xw_p_zw) - (crg * xw_p_zw));
//        const float b    =   ( last_cl * xw_p_zw) - (clg * (yb_xw + yb_zw - xw_zw + ys_xw + ys_zw))
//                          + (-last_cr * xw_p_zw) + (crg * (yt_xw + yt_zw + xw_zw + ys_xw + ys_zw));
//      
//        const float ym   = max(ys, min((ys + adaptive_width[i]), (b / a)));
//      
//        /* Cost at minima */
//        const float ymb  = ym - this->b.y;
//        const float ymt  = this->t.y - ym;
//
//        const float cost = COST_OF_TRAVERSAL + (COST_OF_INTERSECTION * 
//                          (((last_cl + (clg * (ym - ys))) * ((ymb * xw) + (ymb * zw) + xw_zw))  + 
//                           ((last_cr + (crg * (ym - ys))) * ((ymt * xw) + (ymt * zw) + xw_zw))));
//      
//        if (cost < lowest_cost)
//        {
//            lowest_cost = cost;
//            best_axis   = axis_t::y_axis;
//            best_split  = ym;
//        }
//      
//        last_cl = cl_y[i];
//        last_cr = cr_y[i];
//        ys     += adaptive_width[i];
//    }
//    
//    /* Pick fixed points to evaluate */
//    divide_width   = zw * (1.0 / 9.0);
//    evaluate       = this->t.z;
//    
//    /* Fixed points samples and adaptive bin boundaries */
//    float cl_z[9];
//    float cr_z[9];
//    cl_z[8]             = this->p->size();
//    cr_z[8]             = 0.0;
//    bins_per_sample[8]  = 8.0;
//
//    for (int i = 0; i < 8; i++)
//    {
//        sample[i] = this->b.z + (divide_width * static_cast<float>(i + 1));
//    }
//    this->count_primitives(cl_z, cr_z, sample, 8, axis_t::z_axis);
//
//    for (int i = 7; i >= 0; i--)
//    {
////        evaluate -= divide_width;
////        cl_z[i] = count_primitives(&cr_z[i], evaluate, axis_t::z_axis);
//        bins_per_sample[i] = int((cl_z[i] - cr_z[i]) / adapt_bin_size) + 4;
//
//
//        /* Fix adaptive sample width per bin */
//        bins_per_sample[i+1]   -= bins_per_sample[i];
//        adaptive_width[i+1]     = divide_width / (bins_per_sample[i+1] + 1.0);
//    }
//    
//    adaptive_width[0]   = divide_width / (bins_per_sample[0] + 1.0);
//    
//    /* Take adaptive samples */
//    float cl_az[9];
//    float cr_az[9];
//    adaptive_offset = 0;
//    for (int i = 0; i < 9; i++)
//    {
//        evaluate = this->b.z + (i * divide_width);
//        for (int j = adaptive_offset; j < (bins_per_sample[i] + adaptive_offset); j++)
//        {
//            evaluate   += adaptive_width[i];
//            //cl_az[j]    = count_primitives(&cr_az[j], evaluate, axis_t::z_axis);
//            sample[j] = evaluate;
//        }
//        adaptive_offset += bins_per_sample[i];
//    }
//    this->count_primitives(cl_az, cr_az, sample, 8, axis_t::z_axis);
//
//    
//    /* Evaluate SAH minima for each sample as -b/2a */
//    adaptive_offset = 0;
//         last_cl    = 0.0;
//         last_cr    = this->p->size();
//    float zs         = this->b.z;
//    for (int i = 0; i < 9; i++)
//    {
//        /* Process any adaptive samples in this bin first because they will be encountered first */
//        for (int j = adaptive_offset; j < (bins_per_sample[i] + adaptive_offset); j++)
//        {
//            /* Bin primitive gradients */
//            const float clg = (cl_az[j] - last_cl) / adaptive_width[i];
//            const float crg = (cr_az[j] - last_cr) / adaptive_width[i];
//            
//            /* Sample location */
//            const float zs_xw    = zs * xw;
//            const float zs_yw    = zs * yw;
//            
//            /* Minima point bound to this bin */
//            const float a    = 2.0 * ((clg * xw_p_yw) - (crg * xw_p_zw));
//            const float b    =   ( last_cl * xw_p_yw) - (clg * (zb_xw + zb_yw - xw_yw + zs_xw + zs_yw))
//                              + (-last_cr * xw_p_yw) + (crg * (zt_xw + zt_yw + xw_yw + zs_xw + zs_yw));
//            
//            const float zm   = max(zs, min((zs + adaptive_width[i]), (b / a)));
//            
//            /* Cost at minima */
//            const float zmb  = zm - this->b.z;
//            const float zmt  = this->t.z - zm;
//
//            const float cost = COST_OF_TRAVERSAL + (COST_OF_INTERSECTION * 
//                              (((last_cl + (clg * (zm - zs))) * ((zmb * xw) + (zmb * yw) + xw_yw))  + 
//                               ((last_cr + (crg * (zm - zs))) * ((zmt * xw) + (zmt * yw) + xw_yw))));
//            
//            if (cost < lowest_cost)
//            {
//                lowest_cost = cost;
//                best_axis   = axis_t::z_axis;
//                best_split  = zm;
//            }
//            
//            last_cl = cl_az[j];
//            last_cr = cr_az[j];
//            zs     += adaptive_width[i];
//        }
//        adaptive_offset += bins_per_sample[i];
//
//        /* Bin primitive gradients */
//        const float clg = (cl_z[i] - last_cl) / adaptive_width[i];
//        const float crg = (cr_z[i] - last_cr) / adaptive_width[i];
//      
//        /* Sample location */
//        const float zs_xw    = zs * xw;
//        const float zs_yw    = zs * yw;
//     
//        /* Minima point bound to this bin */
//        const float a    = 2.0 * ((clg * xw_p_zw) - (crg * xw_p_zw));
//        const float b    =   ( last_cl * xw_p_zw) - (clg * (zb_xw + zb_yw - xw_yw + zs_xw + zs_yw))
//                          + (-last_cr * xw_p_zw) + (crg * (zt_xw + zt_yw + xw_yw + zs_xw + zs_yw));
//      
//        const float zm   = max(zs, min((zs + adaptive_width[i]), (b / a)));
//      
//        /* Cost at minima */
//        const float zmb  = zm - this->b.z;
//        const float zmt  = this->t.z - zm;
//
//        const float cost = COST_OF_TRAVERSAL + (COST_OF_INTERSECTION * 
//                          (((last_cl + (clg * (zm - zs))) * ((zmb * xw) + (zmb * yw) + xw_yw))  + 
//                           ((last_cr + (crg * (zm - zs))) * ((zmt * xw) + (zmt * yw) + xw_yw))));
//      
//        if (cost < lowest_cost)
//        {
//            lowest_cost = cost;
//            best_axis   = axis_t::z_axis;
//            best_split  = zm;
//        }
//      
//        last_cl = cl_z[i];
//        last_cr = cr_z[i];
//        zs     += adaptive_width[i];
//    }
//    
//    *s = lowest_cost;
//    *normal = best_axis;
//    
//    return best_split;
//}




//float voxel::stl_split_all_axis(float *s, axis_t *normal)
//{
//    axis_t best_axis;
//    /* Find the best split position of the primitives */
//    float best_split;
//    float lowest_cost = *s;
//    
//     /* Initial primitive split */
//    unsigned size_of_prim = this->p->size();
//    float right = size_of_prim;
//    float left  = 0;
//    float guess;
//    float last_guess = MAX_DIST;
//    
//    float x_dist = this->t.x - this->b.x;
//    float y_dist = this->t.y - this->b.y;
//    float z_dist = this->t.z - this->b.z;
//    
//    primitive_list high = this->p;
//    primitive_list::iterator l;
//    primitive_list::iterator h;
//    triangle *hi_triangle;
//    triangle *lo_triangle;
//
//    right = size_of_prim;
//    left  = 0;
//    last_guess = MAX_DIST;
//    x_dist = this->t.x - this->b.x;
//    
//    /* Sort primitives */
//    this->p->sort(sort_triangle_by_lowest_x());
//    high.sort(sort_triangle_by_highest_x());
//
//    l = this->p->begin();
//    h = high.begin();
//
//    /* Move past any out of range triangles */
//    float lo_x = -MAX_DIST;
//    while ((lo_x <= this->b.x) && (l != this->p->end()))
//    {
//        triangle *cur = *l;
//        lo_x = cur->lowest_x();
//        l++;
//        /* This will cause primitives to be split */
//        left++;
//    }
//
//    /* Evaluate the SAH from left to right */
//    float hi_x = -MAX_DIST;
//    hi_triangle = *h;
//    lo_triangle = *l;
//    while (true)
//    {
//        /* Get point to evaluate or MAX_DIST if non remain */
//        if (l == this->p->end())
//        {
//            lo_x = MAX_DIST;
//        }
//        else
//        {
//            lo_x = lo_triangle->lowest_x();
//        }
//        
//        if (h == high.end())
//        {
//            hi_x = MAX_DIST;
//        }
//        else
//        {
//            hi_x = hi_triangle->highest_x();
//        }
//
//        /* Pick the point furthest left and adjust primitive counts */
//        if (lo_x < hi_x)
//        {
//            guess = lo_x;
//        }
//        else
//        {
//            guess = hi_x;
//            right--;
//        }
//
//        /* Break when any further points will be out of bounds */
//        if (guess >= this->t.x)
//        {
//            break;
//        }
//        
//        /* Evaluate unique points */
//        if (last_guess != guess)
//        {
//            float this_cost = calculate_sah_cost(left, right, guess, axis_t::x_axis);
//            if (this_cost < lowest_cost)
//            {
//                lowest_cost = this_cost;
//                best_split  = guess;
//                best_axis   = axis_t::x_axis;
//            }
//            last_guess = guess;
//        }
//
//        /* Advance iterators and primitive counts */
//        if (lo_x < hi_x)
//        {
//            left++;
//            l++;
//            lo_triangle = *l;
//        }
//        else
//        {
//            h++;
//            hi_triangle = *h;
//        }
//    }
//    
//    right = size_of_prim;
//    left  = 0;
//    last_guess = MAX_DIST;
//    x_dist = this->t.x - this->b.x;
//    
//    /* Sort primitives */
//    this->p->sort(sort_triangle_by_lowest_y());
//    high.sort(sort_triangle_by_highest_y());
//
//    l = this->p->begin();
//    h = high.begin();
//
//    /* Move past any out of range triangles */
//    float lo_y = -MAX_DIST;
//    while ((lo_y <= this->b.y) && (l != this->p->end()))
//    {
//        triangle *cur = *l;
//        lo_y = cur->lowest_y();
//        l++;
//        /* This will cause primitives to be split */
//        left++;
//    }
//
//    /* Evaluate the SAH from left to right */
//    float hi_y = -MAX_DIST;
//    hi_triangle = *h;
//    lo_triangle = *l;
//    while (true)
//    {
//        /* Get point to evaluate or MAX_DIST if non remain */
//        if (l == this->p->end())
//        {
//            lo_y = MAX_DIST;
//        }
//        else
//        {
//            lo_y = lo_triangle->lowest_y();
//        }
//        
//        if (h == high.end())
//        {
//            hi_y = MAX_DIST;
//        }
//        else
//        {
//            hi_y = hi_triangle->highest_y();
//        }
//
//        /* Pick the point furthest left and adjust primitive counts */
//        if (lo_y < hi_y)
//        {
//            guess = lo_y;
//        }
//        else
//        {
//            guess = hi_y;
//            right--;
//        }
//
//        /* Break when any further points will be out of bounds */
//        if (guess >= this->t.y)
//        {
//            break;
//        }
//        
//        /* Evaluate unique points */
//        if (last_guess != guess)
//        {
//            float this_cost = calculate_sah_cost(left, right, guess, axis_t::y_axis);
//            if (this_cost < lowest_cost)
//            {
//                lowest_cost = this_cost;
//                best_split  = guess;
//                best_axis   = axis_t::y_axis;
//            }
//            last_guess = guess;
//        }
//
//        /* Advance iterators and primitive counts */
//        if (lo_y < hi_y)
//        {
//            left++;
//            l++;
//            lo_triangle = *l;
//        }
//        else
//        {
//            h++;
//            hi_triangle = *h;
//        }
//    }
//
//    right = size_of_prim;
//    left  = 0;
//    last_guess = MAX_DIST;
//    y_dist = this->t.y - this->b.y;
//    
//    /* Sort primitives */
//    this->p->sort(sort_triangle_by_lowest_z());
//    high.sort(sort_triangle_by_highest_z());
//
//    l = this->p->begin();
//    h = high.begin();
//
//    /* Move past any out of range triangles */
//    float lo_z = -MAX_DIST;
//    while ((lo_z <= this->b.z) && (l != this->p->end()))
//    {
//        triangle *cur = *l;
//        lo_z = cur->lowest_z();
//        l++;
//        /* This will cause primitives to be split */
//        left++;
//    }
//
//    /* Evaluate the SAH from left to right */
//    float hi_z = -MAX_DIST;
//    hi_triangle = *h;
//    lo_triangle = *l;
//    while (true)
//    {
//        /* Get point to evaluate or MAX_DIST if non remain */
//        if (l == this->p->end())
//        {
//            lo_z = MAX_DIST;
//        }
//        else
//        {
//            lo_z = lo_triangle->lowest_z();
//        }
//        
//        if (h == high.end())
//        {
//            hi_z = MAX_DIST;
//        }
//        else
//        {
//            hi_z = hi_triangle->highest_z();
//        }
//
//        /* Pick the point furthest left and adjust primitive counts */
//        if (lo_z < hi_z)
//        {
//            guess = lo_z;
//        }
//        else
//        {
//            guess = hi_z;
//            right--;
//        }
//
//        /* Break when any further points will be out of bounds */
//        if (guess >= this->t.z)
//        {
//            break;
//        }
//        
//        /* Evaluate unique points */
//        if (last_guess != guess)
//        {
//            float this_cost = calculate_sah_cost(left, right, guess, axis_t::z_axis);
//            if (this_cost < lowest_cost)
//            {
//                lowest_cost = this_cost;
//                best_split  = guess;
//                best_axis   = axis_t::z_axis;
//            }
//            last_guess = guess;
//        }
//
//        /* Advance iterators and primitive counts */
//        if (lo_z < hi_z)
//        {
//            left++;
//            l++;
//            lo_triangle = *l;
//        }
//        else
//        {
//            h++;
//            hi_triangle = *h;
//        }
//    }
//    
//    *normal = best_axis;
//    *s = lowest_cost;
//    return best_split;
//}

//    child.x.reserve(child.size());
//    child.y.reserve(child.size());
//    child.z.reserve(child.size());
//    unsigned this_index  = 0;
//    switch (normal)
//    {
//        case axis_t::x_axis:
//            for (unsigned i = 0; i < this->x.size(); i++)
//            {
//                if (this->x[i] > nearest_clip)
//                {
//                    child.x.push_back(this->x[i]);
//                    child.y.push_back(this->y[i]);
//                    child.z.push_back(this->z[i]);
//                }
//                else
//                {
//                    this->x[this_index] = this->x[i];
//                    this->y[this_index] = this->y[i];
//                    this->z[this_index] = this->z[i];
//                    this_index++;                
//                }
//            }
//            break;
//        case axis_t::y_axis:
//            for (unsigned i = 0; i < this->y.size(); i++)
//            {
//                if (this->y[i] > nearest_clip)
//                {
//                    child.x.push_back(this->x[i]);
//                    child.y.push_back(this->y[i]);
//                    child.z.push_back(this->z[i]);
//                }
//                else
//                {
//                    this->x[this_index] = this->x[i];
//                    this->y[this_index] = this->y[i];
//                    this->z[this_index] = this->z[i];
//                    this_index++;                
//                }
//            }
//            break;
//        case axis_t::z_axis:
//            for (unsigned i = 0; i < this->z.size(); i++)
//            {
//                if (this->z[i] > nearest_clip)
//                {
//                    child.x.push_back(this->x[i]);
//                    child.y.push_back(this->y[i]);
//                    child.z.push_back(this->z[i]);
//                }
//                else
//                {
//                    this->x[this_index] = this->x[i];
//                    this->y[this_index] = this->y[i];
//                    this->z[this_index] = this->z[i];
//                    this_index++;                
//                }
//            }
//            break;
//        default :
//            assert(false);
//            break;
//    }    
//    this->x.erase(this->x.begin() + this_index, this->x.end());
//    this->y.erase(this->y.begin() + this_index, this->y.end());
//    this->z.erase(this->z.begin() + this_index, this->z.end());
}; /* namespace raptor_raytracer */
