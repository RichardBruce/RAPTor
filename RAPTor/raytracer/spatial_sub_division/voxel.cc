#include "voxel.h"


namespace raptor_raytracer
{
/**********************************************************
 
**********************************************************/
voxel voxel::divide(kdt_node *const k)
{
    /* Calculate the cost of this node */
    fp_t nr_of_primitives = this->p->size();
    fp_t x_dist = this->t.x - this->b.x;
    fp_t y_dist = this->t.y - this->b.y;
    fp_t z_dist = this->t.z - this->b.z;
    fp_t area   = (x_dist * y_dist) + (x_dist * z_dist) + (z_dist * y_dist);
    fp_t lowest_cost = COST_OF_INTERSECTION * nr_of_primitives * area;
    fp_t cost_before = lowest_cost;

#if 1   /* Approximate builder */
    axis normal;
    fp_t best_split;
    
#ifdef SIMD_PACKET_TRACING
    /* If using SIMD allow early exit for nodes under a given size */
    if (this->p->size() <= MIN_KDT_NODE_SIZE)
    {
        k->set_primitives(this->p);
        return *this;
    } 
    else
#endif /* #ifdef SIMD_PACKET_TRACING */
    /* Invoke the exact builder for small node */
    if (this->p->size() <= MIN_APPROX_KDT_BUILDER_NODE_SIZE)
    {
#ifdef SIMD_PACKET_TRACING
        best_split = this->brute_force_split_all_axis(&lowest_cost, &normal);
#else
        best_split = this->split_all_axis(&lowest_cost, &normal);
#endif
    }
    else
    {
#if 0   /* Split approximately in all axis */
        best_split = this->approximate_split_all_axis(&lowest_cost, &normal);
#else   /* Split approximately in the first lower cost axis */
        normal = this->n;
        best_split = approximate_split_one_axis(&lowest_cost, normal);
        if (lowest_cost >= cost_before)
        {
            switch (normal)
            {
                case x_axis:
                    normal  = y_axis;
                    break;
                case y_axis:
                    normal  = z_axis;
                    break;
                case z_axis:
                    normal  = x_axis;
                    break;
                default:
                    assert(false);
                    break;
        }
        best_split = approximate_split_one_axis(&lowest_cost, this->n);
            
            if (lowest_cost >= cost_before)
            {
                switch (normal)
                {
                    case x_axis:
                        normal  = y_axis;
                        break;
                    case y_axis:
                        normal  = z_axis;
                        break;
                    case z_axis:
                        normal  = x_axis;
                        break;
                    default:
                        assert(false);
                        break;
                }
                best_split = approximate_split_one_axis(&lowest_cost, this->n);
            }
        }
#endif
    }
#else   /* Exact builder */
#if 0   /* Signal axis aplit */
    fp_t best_split = this->split_this_axis(&lowest_cost);
#else   /* All axis aplit */
    axis normal;
    fp_t best_split = this->split_all_axis(&lowest_cost, &normal);
#endif
#endif

    /* Stop splitting if the cost metric cannot be reduced */
    assert(lowest_cost >= 0.0);
    if (lowest_cost >= cost_before)
    {
        k->set_primitives(this->p);
        return *this;
    }
    
    /* Adjust the size of the voxel */
    point_t upper_limit = this->t;
    point_t lower_limit = this->b;
    axis    nxt_n;
    switch (normal)
    {
        case x_axis:
            /* Assert the the voxel is really split */
            assert(best_split > this->b.x);
            assert(best_split < this->t.x);
            lower_limit.x   = best_split;
            this->t.x       = best_split;
            nxt_n           = y_axis;
            break;
        case y_axis:
            assert(best_split > this->b.y);
            assert(best_split < this->t.y);
            lower_limit.y   = best_split;
            this->t.y       = best_split;
            nxt_n           = z_axis;
            break;
        case z_axis:
            assert(best_split > this->b.z);
            assert(best_split < this->t.z);
            lower_limit.z   = best_split;
            this->t.z       = best_split;
            nxt_n           = x_axis;
            break;
        default:
            assert(false);
            break;
    }
    
    /* Divide the primitives */
    fp_t nearest_clip = best_split;
#if 0   /* Set to enable clip of split plane to nearest primitive */
    fp_t dist_to_clip = MAX_DIST;
#endif
    primitive_list *child_p = new primitive_list;
    
    primitive_list::iterator p_dst = this->p->begin();
    switch (normal)
    {
        case x_axis:
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
        case y_axis:
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
        case z_axis:
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
// fp_t voxel::split_all_axis(fp_t *s, axis *normal)
// {
//     /* Find the best split position of the primitives */
//     axis best_axis   = not_set;
//     fp_t best_split  = not_set;
//     fp_t lowest_cost = *s;
    
//      /* Initial primitive split */
//     unsigned size_of_prim = this->p->size();
//     fp_t     right = size_of_prim;
//     fp_t     left  = 0;
//     fp_t     guess;
//     fp_t     last_guess = MAX_DIST;
    
//     /* Elements for evaluation */
//     fp_t lo_x, lo_y, lo_z, hi_x, hi_y, hi_z;
    
//     /* Allocated arrays to hold the bounding box points */    
//     if (low_points == NULL)
//     {
//         low_points  = new fp_t [size_of_prim];
//         high_points = new fp_t [size_of_prim];
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
//             fp_t this_cost = calculate_sah_cost(left, right, guess, x_axis);
//             if (this_cost < lowest_cost)
//             {
//                 lowest_cost = this_cost;
//                 best_split  = guess;
//                 best_axis   = x_axis;
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
//             fp_t this_cost = calculate_sah_cost(left, right, guess, y_axis);
//             if (this_cost < lowest_cost)
//             {
//                 lowest_cost = this_cost;
//                 best_split  = guess;
//                 best_axis   = y_axis;
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
//             fp_t this_cost = calculate_sah_cost(left, right, guess, z_axis);
//             if (this_cost < lowest_cost)
//             {
//                 lowest_cost = this_cost;
//                 best_split  = guess;
//                 best_axis   = z_axis;
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


// fp_t voxel::approximate_split_all_axis(fp_t *s, axis * normal) const
// {
//     axis best_axis      = not_set;
//     fp_t best_split     = MAX_DIST;
//     fp_t cost           = *s;
//     fp_t lowest_cost    = *s;

//     /* Split in x and compare the result */
//     fp_t split =  approximate_split_one_axis(&cost, x_axis);
//     if (cost < lowest_cost)
//     {
//         lowest_cost = cost;
//         best_axis   = x_axis;
//         best_split  = split;
//     }

//     /* Split in y and compare the result */
//     split =  approximate_split_one_axis(&cost, y_axis);
//     if (cost < lowest_cost)
//     {
//         lowest_cost = cost;
//         best_axis   = y_axis;
//         best_split  = split;
//     }

//     /* Split in z and compare the result */
//     split =  approximate_split_one_axis(&cost, z_axis);
//     if (cost < lowest_cost)
//     {
//         lowest_cost = cost;
//         best_axis   = z_axis;
//         best_split  = split;
//     }
    
//     *s = lowest_cost;
//     *normal = best_axis;
    
//     return best_split;
// }


fp_t voxel::approximate_split_one_axis(fp_t *const s, const axis normal) const
{
    /* Find the best split position of the primitives */
    fp_t best_split     = MAX_DIST;
    fp_t lowest_cost    = *s;
    
    /* Adaptive sampling bin sizes */
    const fp_t adapt_bin_size   = this->p->size() * (2.0/9.0);
    
    /* Voxel dimensions */
    const fp_t xw       = this->t.x - this->b.x;
    const fp_t yw       = this->t.y - this->b.y;
    const fp_t zw       = this->t.z - this->b.z;
    const fp_t xb_yw    = this->b.x * yw;
    const fp_t xb_zw    = this->b.x * zw;
    const fp_t xt_yw    = this->t.x * yw;
    const fp_t xt_zw    = this->t.x * zw;
    const fp_t yb_xw    = this->b.y * xw;
    const fp_t yb_zw    = this->b.y * zw;
    const fp_t yt_xw    = this->t.y * xw;
    const fp_t yt_zw    = this->t.y * zw;
    const fp_t zb_xw    = this->b.z * xw;
    const fp_t zb_yw    = this->b.z * yw;
    const fp_t zt_xw    = this->t.z * xw;
    const fp_t zt_yw    = this->t.z * yw;
    const fp_t xw_yw    = xw * yw;
    const fp_t xw_zw    = xw * zw;
    const fp_t yw_zw    = yw * zw;
    const fp_t xw_p_yw  = xw + yw;
    const fp_t xw_p_zw  = xw + zw;
    const fp_t yw_p_zw  = yw + zw;
//    const fp_t sa       = 1.0 / (xw_yw + xw_zw + yw_zw);
    
    /* Sampling variable */
    fp_t dw;
    fp_t evaluate;
    fp_t sample[8] ALIGN(16);
    fp_t bins_per_sample[9];
    fp_t adaptive_width[9];
    fp_t cl[9];
    fp_t cr[9];
    fp_t cla[9];
    fp_t cra[9];
    fp_t last_cl;
    fp_t last_cr;
    fp_t sp;
    int  adaptive_offset;

    switch (normal)
    {
        case x_axis :   
            /* Pick fixed points to evaluate */
            dw          = xw * (1.0 / 9.0);
            evaluate    = this->t.x;
            
            /* Fixed points samples and adaptive bin boundaries */
            cl[8]               = this->p->size();
            cr[8]               = 0.0;
            bins_per_sample[8]  = 8.0;
            
            for (int i = 0; i < 8; i++)
            {
                sample[i] = this->b.x + (dw * (fp_t)(i+1));
            }
            this->count_primitives(cl, cr, sample, 8, x_axis);
            
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
            this->count_primitives(cla, cra, sample, 8, x_axis);
            
            /* Evaluate SAH minima for each sample as -b/2a */
            adaptive_offset = 0;
            last_cl         = 0.0;
            last_cr         = this->p->size();
            sp              = this->b.x;
            for (int i = 0; i < 9; i++)
            {
                /* Process any adaptive samples in this bin first because they will be encountered first */
                for (int j = adaptive_offset; j < (bins_per_sample[i] + adaptive_offset); j++)
                {
                    /* Bin primitive gradients */
                    const fp_t clg = (cla[j] - last_cl) / adaptive_width[i];
                    const fp_t crg = (cra[j] - last_cr) / adaptive_width[i];
                    
                    /* Sample location */
                    const fp_t sp_yw    = sp * yw;
                    const fp_t sp_zw    = sp * zw;
                    
                    /* Minima point bound to this bin */
                    const fp_t a    = 2.0 * ((clg * yw_p_zw) - (crg * yw_p_zw));
                    const fp_t b    =   ( last_cl * yw_p_zw) - (clg * (xb_yw + xb_zw - yw_zw + sp_yw + sp_zw))
                                      + (-last_cr * yw_p_zw) + (crg * (xt_yw + xt_zw + yw_zw + sp_yw + sp_zw));
                    
                    const fp_t delta    = adaptive_width[i] * 0.05;
                    const fp_t xm       = max((sp + delta), min((sp + adaptive_width[i] - delta), (b / a)));
        
                    /* Cost at minima */
                    const fp_t xmb  = xm - this->b.x;
                    const fp_t xmt  = this->t.x - xm;
        
                    const fp_t cost = COST_OF_TRAVERSAL + (COST_OF_INTERSECTION * 
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
                adaptive_offset += (int)bins_per_sample[i];
        
                /* Bin primitive gradients */
                const fp_t clg = (cl[i] - last_cl) / adaptive_width[i];
                const fp_t crg = (cr[i] - last_cr) / adaptive_width[i];
              
                /* Sample location */
                const fp_t sp_yw    = sp * yw;
                const fp_t sp_zw    = sp * zw;
              
                /* Minima point bound to this bin */
                const fp_t a    = 2.0 * ((clg * yw_p_zw) - (crg * yw_p_zw));
                const fp_t b    =   ( last_cl * yw_p_zw) - (clg * (xb_yw + xb_zw - yw_zw + sp_yw + sp_zw))
                                  + (-last_cr * yw_p_zw) + (crg * (xt_yw + xt_zw + yw_zw + sp_yw + sp_zw));
              
                const fp_t delta    = adaptive_width[i] * 0.05;
                const fp_t xm       = max((sp + delta), min((sp + adaptive_width[i] - delta), (b / a)));
              
                /* Cost at minima */
                const fp_t xmb  = xm - this->b.x;
                const fp_t xmt  = this->t.x - xm;
        
                const fp_t cost = COST_OF_TRAVERSAL + (COST_OF_INTERSECTION * 
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
            
        case y_axis :
            /* Pick fixed points to evaluate */
            dw          = yw * (1.0 / 9.0);
            evaluate    = this->t.y;
            
            /* Fixed points samples and adaptive bin boundaries */
            cl[8]               = this->p->size();
            cr[8]               = 0.0;
            bins_per_sample[8]  = 8.0;
        
            for (int i = 0; i < 8; i++)
            {
                sample[i] = this->b.y + (dw * (fp_t)(i+1));
            }
            this->count_primitives(cl, cr, sample, 8, y_axis);
        
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
            this->count_primitives(cla, cra, sample, 8, y_axis);
            
            /* Evaluate SAH minima for each sample as -b/2a */
            adaptive_offset = 0;
            last_cl         = 0.0;
            last_cr         = this->p->size();
            sp              = this->b.y;
            for (int i = 0; i < 9; i++)
            {
                /* Process any adaptive samples in this bin first because they will be encountered first */
                for (int j = adaptive_offset; j < (bins_per_sample[i] + adaptive_offset); j++)
                {
                    /* Bin primitive gradients */
                    const fp_t clg = (cla[j] - last_cl) / adaptive_width[i];
                    const fp_t crg = (cra[j] - last_cr) / adaptive_width[i];
                    
                    /* Sample location */
                    const fp_t sp_xw    = sp * xw;
                    const fp_t sp_zw    = sp * zw;
                    
                    /* Minima point bound to this bin */
                    const fp_t a    = 2.0 * ((clg * xw_p_zw) - (crg * xw_p_zw));
                    const fp_t b    =   ( last_cl * xw_p_zw) - (clg * (yb_xw + yb_zw - xw_zw + sp_xw + sp_zw))
                                      + (-last_cr * xw_p_zw) + (crg * (yt_xw + yt_zw + xw_zw + sp_xw + sp_zw));
                    
                    const fp_t delta    = adaptive_width[i] * 0.05;
                    const fp_t ym       = max((sp + delta), min((sp + adaptive_width[i] - delta), (b / a)));
                    
                    /* Cost at minima */
                    const fp_t ymb  = ym - this->b.y;
                    const fp_t ymt  = this->t.y - ym;
        
                    const fp_t cost = COST_OF_TRAVERSAL + (COST_OF_INTERSECTION * 
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
                adaptive_offset += (int)bins_per_sample[i];
        
                /* Bin primitive gradients */
                const fp_t clg = (cl[i] - last_cl) / adaptive_width[i];
                const fp_t crg = (cr[i] - last_cr) / adaptive_width[i];
              
                /* Sample location */
                const fp_t sp_xw    = sp * xw;
                const fp_t sp_zw    = sp * zw;
              
                /* Minima point bound to this bin */
                const fp_t a    = 2.0 * ((clg * xw_p_zw) - (crg * xw_p_zw));
                const fp_t b    =   ( last_cl * xw_p_zw) - (clg * (yb_xw + yb_zw - xw_zw + sp_xw + sp_zw))
                                  + (-last_cr * xw_p_zw) + (crg * (yt_xw + yt_zw + xw_zw + sp_xw + sp_zw));
              
                const fp_t delta    = adaptive_width[i] * 0.05;
                const fp_t ym       = max((sp + delta), min((sp + adaptive_width[i] - delta), (b / a)));
              
                /* Cost at minima */
                const fp_t ymb  = ym - this->b.y;
                const fp_t ymt  = this->t.y - ym;
        
                const fp_t cost = COST_OF_TRAVERSAL + (COST_OF_INTERSECTION * 
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

        case z_axis :
            /* Pick fixed points to evaluate */
            dw          = zw * (1.0 / 9.0);
            evaluate    = this->t.z;
            
            /* Fixed points samples and adaptive bin boundaries */
            cl[8]               = this->p->size();
            cr[8]               = 0.0;
            bins_per_sample[8]  = 8.0;
        
            for (int i = 0; i < 8; i++)
            {
                sample[i] = this->b.z + (dw * (fp_t)(i+1));
            }
            this->count_primitives(cl, cr, sample, 8, z_axis);
        
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
                evaluate = this->b.z + (i * dw);
                for (int j = adaptive_offset; j < (bins_per_sample[i] + adaptive_offset); j++)
                {
                    evaluate   += adaptive_width[i];
                    sample[j] = evaluate;
                }
                adaptive_offset += (int)bins_per_sample[i];
            }
            this->count_primitives(cla, cra, sample, 8, z_axis);
            
            /* Evaluate SAH minima for each sample as -b/2a */
            adaptive_offset = 0;
            last_cl         = 0.0;
            last_cr         = this->p->size();
            sp              = this->b.z;
            for (int i = 0; i < 9; i++)
            {
                /* Process any adaptive samples in this bin first because they will be encountered first */
                for (int j = adaptive_offset; j < (bins_per_sample[i] + adaptive_offset); j++)
                {
                    /* Bin primitive gradients */
                    const fp_t clg = (cla[j] - last_cl) / adaptive_width[i];
                    const fp_t crg = (cra[j] - last_cr) / adaptive_width[i];
                    
                    /* Sample location */
                    const fp_t sp_xw    = sp * xw;
                    const fp_t sp_yw    = sp * yw;
                    
                    /* Minima point bound to this bin */
                    const fp_t a    = 2.0 * ((clg * xw_p_yw) - (crg * xw_p_zw));
                    const fp_t b    =   ( last_cl * xw_p_yw) - (clg * (zb_xw + zb_yw - xw_yw + sp_xw + sp_yw))
                                      + (-last_cr * xw_p_yw) + (crg * (zt_xw + zt_yw + xw_yw + sp_xw + sp_yw));
                    
                    const fp_t delta    = adaptive_width[i] * 0.05;
                    const fp_t zm       = max((sp + delta), min((sp + adaptive_width[i] - delta), (b / a)));
                    
                    /* Cost at minima */
                    const fp_t zmb  = zm - this->b.z;
                    const fp_t zmt  = this->t.z - zm;
        
                    const fp_t cost = COST_OF_TRAVERSAL + (COST_OF_INTERSECTION * 
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
                adaptive_offset += (int)bins_per_sample[i];
        
                /* Bin primitive gradients */
                const fp_t clg = (cl[i] - last_cl) / adaptive_width[i];
                const fp_t crg = (cr[i] - last_cr) / adaptive_width[i];
              
                /* Sample location */
                const fp_t sp_xw    = sp * xw;
                const fp_t sp_yw    = sp * yw;
             
                /* Minima point bound to this bin */
                const fp_t a    = 2.0 * ((clg * xw_p_yw) - (crg * xw_p_zw));
                const fp_t b    =   ( last_cl * xw_p_yw) - (clg * (zb_xw + zb_yw - xw_yw + sp_xw + sp_yw))
                                  + (-last_cr * xw_p_yw) + (crg * (zt_xw + zt_yw + xw_yw + sp_xw + sp_yw));
              
                const fp_t delta    = adaptive_width[i] * 0.05;
                const fp_t zm       = max((sp + delta), min((sp + adaptive_width[i] - delta), (b / a)));
              
                /* Cost at minima */
                const fp_t zmb  = zm - this->b.z;
                const fp_t zmt  = this->t.z - zm;
        
                const fp_t cost = COST_OF_TRAVERSAL + (COST_OF_INTERSECTION * 
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
 
**********************************************************/
inline fp_t voxel::count_primitives(fp_t *const r, const fp_t s, const axis normal) const
{
    /* Sub divide the objects */
    fp_t left_objects  = (fp_t)0.0;
    fp_t right_objects = (fp_t)0.0;
    switch (normal)
    {
        case x_axis:
            for (primitive_list::const_iterator i=this->p->begin(); i!=this->p->end(); ++i)
            {
                left_objects  += (int)((*i)->lowest_x()  < s);
                right_objects += (int)((*i)->highest_x() > s);
            }
            break;
        case y_axis:
            for (primitive_list::const_iterator i=this->p->begin(); i!=this->p->end(); ++i)
            {
                left_objects  += (int)((*i)->lowest_y()  < s);
                right_objects += (int)((*i)->highest_y() > s);
            }
            break;
        case z_axis:
            for (primitive_list::const_iterator i=this->p->begin(); i!=this->p->end(); ++i)
            {
                left_objects  += (int)((*i)->lowest_z()  < s);
                right_objects += (int)((*i)->highest_z() > s);
            }
            break;
        default :
            assert(false);
            break;
    }

    *r = right_objects;
    return left_objects;
}


#ifndef SIMD_PACKET_TRACING
/**********************************************************
 
**********************************************************/
inline void voxel::count_primitives(fp_t *const l, fp_t *const r, const fp_t *const s, const int len, const axis n) const
{
//    if (this->p->size() > 1000000)
//    {
//        primitive_count pc(this->p, s, n);
//        parallel_reduce(blocked_range<size_t>(0,this->p->size()), pc, auto_partitioner());
//    
//        for (int i = 0; i < 8; i++)
//        {
//            l[i] = pc.sum_l[i];
//            r[i] = pc.sum_r[i];
//        }
//    }
//    else
//    {
    /* Clear the counters */
    for (int i = 0; i < len; i++)
    {
        l[i] = (fp_t)0.0;
        r[i] = (fp_t)0.0;
    }

    /* Count in the given axis */
    switch (n)
    {
        case x_axis:
            for (primitive_list::const_iterator i = this->p->begin(); i != this->p->end(); ++i)
            {
                for (int j = 0; j < len; j++)
                {
                    l[j] += (int)((*i)->lowest_x()  < s[j]);
                    r[j] += (int)((*i)->highest_x() > s[j]);
                }
            }
            break;

        case y_axis:
            for (primitive_list::const_iterator i = this->p->begin(); i != this->p->end(); ++i)
            {
                for (int j = 0; j < len; j++)
                {
                    l[j] += (int)((*i)->lowest_y()  < s[j]);
                    r[j] += (int)((*i)->highest_y() > s[j]);
                }
            }
            break;

    case z_axis:
            for (primitive_list::const_iterator i = this->p->begin(); i != this->p->end(); ++i)
            {
                for (int j = 0; j < len; j++)
                {
                    l[j] += (int)((*i)->lowest_z()  < s[j]);
                    r[j] += (int)((*i)->highest_z() > s[j]);
                }
            }
            break;
        default :
            assert(false);
            break;
    }
//    }
}


#else
/**********************************************************
 
**********************************************************/
inline void voxel::count_primitives(fp_t *const l, fp_t *const r, const fp_t *const s, const int len, const axis n) const
{
    assert(len == 8);
    vfp_t l_vec[2] = { vfp_zero, vfp_zero };
    vfp_t r_vec[2] = { vfp_zero, vfp_zero };
    vfp_t s_vec[2] = { &s[0], &s[SIMD_WIDTH] };


    /* Count in the given axis */
    switch (n)
    {
        case x_axis:
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

        case y_axis:
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

        case z_axis:
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

//Static properties of the tree :
//Scene primitves                                             (nsp) : 76152
//Tree primitves                                              (ntp) : 226924
//Number of generic nodes                                     (ng ) : 19642
//Number of elementary nodes                                  (ne ) : 19643
//Number of empty elementary nodes                            (nee) : 2223
//Maximum size of an elementary node                          (ner) : 38
//Average size of an elementary node                          (nea) : 11.5524
//Maximum depth of the tree                                   (d  ) : 26
//
//Dynamic properties of the KD-tree :
//Number of rays shot                                       (nr   ) : 38077
//Intersection tests performed                              (nit  ) : 374006
//Intersection tests performed : to minimum required tests  (ritm ) : 80.9362
//Average number of nodes accessed per ray                  (nts  ) : 54.3604
//Average number of elementary nodes accessed per ray       (nets ) : 15.2664
//Average number of empty elementary nodes accessed per ray (neets) : 1.64764

/**********************************************************
 
**********************************************************/
fp_t voxel::brute_force_split_all_axis(fp_t *s, axis * normal) const
{
    /* Find the best split position of the primitives */
    fp_t sample[9] ALIGN(16);
    vfp_t valid_mask[2];

    vfp_t l_vec[2], r_vec[2], s_vec[2];
    vfp_t lc_vec(*s);
    vfp_t ba_vec((fp_t)not_set);
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
            this->count_primitives(l_vec, r_vec, s_vec, x_axis);
            for (int j = 0; j < 2; j++)
            {
                vfp_t cost = this->calculate_sah_cost(l_vec[j], r_vec[j], s_vec[j], x_axis);
                vfp_t mask = (cost < lc_vec);

                lc_vec = min(lc_vec, cost);
                bs_vec = mov_p(mask, s_vec[j], bs_vec);
                ba_vec = mov_p(mask, vfp_t((fp_t)x_axis), ba_vec);
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
    
    this->count_primitives(l_vec, r_vec, s_vec, x_axis);
    for (int j = 0; j < ((bucketted >> LOG2_SIMD_WIDTH) + (int)(bucketted > 0)); j++)
    {
        vfp_t cost = this->calculate_sah_cost(l_vec[j], r_vec[j], s_vec[j], x_axis);
        vfp_t mask = (cost < lc_vec) & valid_mask[j];

        lc_vec = mov_p(mask, cost,                lc_vec);
        bs_vec = mov_p(mask, s_vec[j],            bs_vec);
        ba_vec = mov_p(mask, vfp_t((fp_t)x_axis), ba_vec);
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
            this->count_primitives(l_vec, r_vec, s_vec, y_axis);
            for (int j = 0; j < 2; j++)
            {
                vfp_t cost = this->calculate_sah_cost(l_vec[j], r_vec[j], s_vec[j], y_axis);
                vfp_t mask = (cost < lc_vec);

                lc_vec = min(lc_vec, cost);
                bs_vec = mov_p(mask, s_vec[j], bs_vec);
                ba_vec = mov_p(mask, vfp_t((fp_t)y_axis), ba_vec);
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
    
    this->count_primitives(l_vec, r_vec, s_vec, y_axis);
    for (int j = 0; j < ((bucketted >> LOG2_SIMD_WIDTH) + (int)(bucketted > 0)); j++)
    {
        vfp_t cost = this->calculate_sah_cost(l_vec[j], r_vec[j], s_vec[j], y_axis);
        vfp_t mask = (cost < lc_vec) & valid_mask[j];

        lc_vec = mov_p(mask, cost,                lc_vec);
        bs_vec = mov_p(mask, s_vec[j],            bs_vec);
        ba_vec = mov_p(mask, vfp_t((fp_t)y_axis), ba_vec);
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
            this->count_primitives(l_vec, r_vec, s_vec, z_axis);
            for (int j = 0; j < 2; j++)
            {
                vfp_t cost = this->calculate_sah_cost(l_vec[j], r_vec[j], s_vec[j], z_axis);
                vfp_t mask = (cost < lc_vec);

                lc_vec = min(lc_vec, cost);
                bs_vec = mov_p(mask, s_vec[j], bs_vec);
                ba_vec = mov_p(mask, vfp_t((fp_t)z_axis), ba_vec);
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
    
    this->count_primitives(l_vec, r_vec, s_vec, z_axis);
    for (int j = 0; j < ((bucketted >> LOG2_SIMD_WIDTH) + (int)(bucketted > 0)); ++j)
    {
        vfp_t cost = this->calculate_sah_cost(l_vec[j], r_vec[j], s_vec[j], z_axis);
        vfp_t mask = (cost < lc_vec) & valid_mask[j];

        lc_vec = mov_p(mask, cost,                lc_vec);
        bs_vec = mov_p(mask, s_vec[j],            bs_vec);
        ba_vec = mov_p(mask, vfp_t((fp_t)z_axis), ba_vec);
    }
    
    /* Collect results */
    (*s) = min(min(lc_vec[0], lc_vec[1]), min(lc_vec[2], lc_vec[3]));

    const int index = mask_to_index_lut[move_mask(vfp_t(*s) == lc_vec)];
    (*normal) = (axis)ba_vec[index];
    return bs_vec[index];
}


/**********************************************************
 
**********************************************************/
inline void voxel::count_primitives(vfp_t *const l, vfp_t *const r, const vfp_t *const s, const axis n) const
{
    l[0] = vfp_zero;
    l[1] = vfp_zero;
    r[0] = vfp_zero;
    r[1] = vfp_zero;

    /* Count in the given axis */
    switch (n)
    {
        case x_axis:
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

        case y_axis:
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

        case z_axis:
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
inline vfp_t voxel::calculate_sah_cost(const vfp_t &l, const vfp_t &r, const vfp_t &s, const axis normal) const
{
    /* Surface area of sub-divides */
    vfp_t left_area, right_area;
    vfp_t x_dist, y_dist, z_dist;
//    const fp_t sa   = 1.0 / ((x_dist * y_dist) + (x_dist * z_dist) + (y_dist * z_dist));
    switch (normal)
    {
        case x_axis:
//            assert(move_mask(s < vfp_t(this->b.x)) == 0x0);
//            assert(move_mask(s > vfp_t(this->t.x)) == 0x0);

            x_dist = s - vfp_t(this->b.x);
            y_dist = vfp_t(this->t.y - this->b.y);
            z_dist = vfp_t(this->t.z - this->b.z);
            left_area  = (x_dist * y_dist) + (x_dist * z_dist) + (z_dist * y_dist);

            x_dist = vfp_t(this->t.x) - s;
            right_area = (x_dist * y_dist) + (x_dist * z_dist) + (z_dist * y_dist);
            break;
        case y_axis:
//            assert(move_mask(s < vfp_t(this->b.y)) == 0x0);
//            assert(move_mask(s > vfp_t(this->t.y)) == 0x0);

            x_dist = vfp_t(this->t.x - this->b.x);
            y_dist = s - vfp_t(this->b.y);
            z_dist = vfp_t(this->t.z - this->b.z);
            left_area  = (x_dist * y_dist) + (x_dist * z_dist) + (z_dist * y_dist);

            y_dist = vfp_t(this->t.y) - s;
            right_area = (x_dist * y_dist) + (x_dist * z_dist) + (z_dist * y_dist);
            break;
        case z_axis:
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
#endif /* #ifndef SIMD_PACKET_TRACING */


/**********************************************************
 
**********************************************************/
// inline fp_t voxel::calculate_sah_cost(const fp_t l, const fp_t r, const fp_t s, const axis normal) const
// {
//     /* Surface area of sub-divides */
//     fp_t left_area, right_area;
//     fp_t x_dist     = this->t.x - this->b.x;
//     fp_t y_dist     = this->t.y - this->b.y;
//     fp_t z_dist     = this->t.z - this->b.z;
// //    const fp_t sa   = 1.0 / ((x_dist * y_dist) + (x_dist * z_dist) + (y_dist * z_dist));
//     switch (normal)
//     {
//         case x_axis:
//             assert(s > this->b.x);
//             assert(s < this->t.x);
//             x_dist = s - this->b.x;
//             left_area  = (x_dist * y_dist) + (x_dist * z_dist) + (z_dist * y_dist);
//             x_dist = this->t.x - s;
//             right_area = (x_dist * y_dist) + (x_dist * z_dist) + (z_dist * y_dist);
//             break;
//         case y_axis:
//             assert(s > this->b.y);
//             assert(s < this->t.y);
//             y_dist = s - this->b.y;
//             left_area  = (x_dist * y_dist) + (x_dist * z_dist) + (z_dist * y_dist);
//             y_dist = this->t.y - s;
//             right_area = (x_dist * y_dist) + (x_dist * z_dist) + (z_dist * y_dist);
//             break;
//         case z_axis:
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
//fp_t voxel::calculate_sah_cost(const fp_t s, const axis normal) const
//{
//    /* Surface area of sub-divides */
//    fp_t left_area, right_area;
//    fp_t x_dist = this->t.x - this->b.x;
//    fp_t y_dist = this->t.y - this->b.y;
//    fp_t z_dist = this->t.z - this->b.z;
//    switch (normal)
//    {
//        case x_axis:
//            assert(s > this->b.x);
//            assert(s < this->t.x);
//            x_dist = s - this->b.x;
//            left_area  = (x_dist * y_dist) + (x_dist * z_dist) + (z_dist * y_dist);
//            x_dist = this->t.x - s;
//            right_area = (x_dist * y_dist) + (x_dist * z_dist) + (z_dist * y_dist);
//            break;
//        case y_axis:
//            assert(s > this->b.y);
//            assert(s < this->t.y);
//            y_dist = s - this->b.y;
//            left_area  = (x_dist * y_dist) + (x_dist * z_dist) + (z_dist * y_dist);
//            y_dist = this->t.y - s;
//            right_area = (x_dist * y_dist) + (x_dist * z_dist) + (z_dist * y_dist);
//            break;
//        case z_axis:
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
//    fp_t left_objects  = 0.0;
//    fp_t right_objects = 0.0;
//    switch (normal)
//    {
//        case x_axis:
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
//        case y_axis:
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
//        case z_axis:
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
//    fp_t sah_cost = COST_OF_TRAVERSAL + (COST_OF_INTERSECTION * ((left_objects * left_area) + (right_objects * right_area)));
//
//    /* Calculate the surface area heuristic metric */
//    /* cost of traversal + cost of intersection * (left cell count * left area + right cell count * right area) */
//    return sah_cost;
//}


/* Sequential */

//fp_t voxel::approximate_split_all_axis(fp_t *s, axis * normal) const
//{
//    /* Find the best split position of the primitives */
//    fp_t best_split = MAX_DIST;
//    fp_t lowest_cost = *s;
//    axis best_axis = not_set;
//
//    for (primitive_list::const_iterator i=p.begin(); i!=p.end(); i++)
//    {
//        const triangle *current_p = *i;
//        if ((current_p->lowest_x() > this->b.x) && (current_p->lowest_x() < this->t.x))
//        {
//            fp_t this_cost = this->calculate_sah_cost(current_p->lowest_x(), x_axis);
//            if (this_cost < lowest_cost)
//            {
//                lowest_cost = this_cost;
//                best_split  = current_p->lowest_x();
//                best_axis   = x_axis;
//            }
//        }
//        if ((current_p->highest_x() > this->b.x) && (current_p->highest_x() < this->t.x))
//        {
//            fp_t this_cost = this->calculate_sah_cost(current_p->highest_x(), x_axis);
//            if (this_cost < lowest_cost)
//            {
//                lowest_cost = this_cost;
//                best_split  = current_p->highest_x();
//                best_axis   = x_axis;
//            }
//        }
//    }
//
//    for (primitive_list::const_iterator i=p.begin(); i!=p.end(); i++)
//    {
//        const triangle *current_p = *i;
//        if ((current_p->lowest_y() > this->b.y) && (current_p->lowest_y() < this->t.y))
//        {
//            fp_t this_cost = this->calculate_sah_cost(current_p->lowest_y(), y_axis);
//            if (this_cost < lowest_cost)
//            {
//                lowest_cost = this_cost;
//                best_split  = current_p->lowest_y();
//                best_axis   = y_axis;
//            }
//        }
//        if ((current_p->highest_y() > this->b.y) && (current_p->highest_y() < this->t.y))
//        {
//            fp_t this_cost = this->calculate_sah_cost(current_p->highest_y(), y_axis);
//            if (this_cost < lowest_cost)
//            {
//                lowest_cost = this_cost;
//                best_split  = current_p->highest_y();
//                best_axis   = y_axis;
//            }
//        }
//    }
//
//    for (primitive_list::const_iterator i=p.begin(); i!=p.end(); i++)
//    {
//        const triangle *current_p = *i;
//        if ((current_p->lowest_z() > this->b.z) && (current_p->lowest_z() < this->t.z))
//        {
//            fp_t this_cost = this->calculate_sah_cost(current_p->lowest_z(), z_axis);
//            if (this_cost < lowest_cost)
//            {
//                lowest_cost = this_cost;
//                best_split  = current_p->lowest_z();
//                best_axis   = z_axis;
//            }
//        }
//        if ((current_p->highest_z() > this->b.z) && (current_p->highest_z() < this->t.z))
//        {
//            fp_t this_cost = this->calculate_sah_cost(current_p->highest_z(), z_axis);
//            if (this_cost < lowest_cost)
//            {
//                lowest_cost = this_cost;
//                best_split  = current_p->highest_z();
//                best_axis   = z_axis;
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

//fp_t voxel::split_this_axis(fp_t *s) const
//{
//    /* Find the best split position of the primitives */
//    fp_t best_split = MAX_DIST;
//    fp_t lowest_cost = *s;
//    switch (this->n)
//    {
//        case x_axis:
//            for (primitive_list::const_iterator i=p.begin(); i!=p.end(); i++)
//            {
//                const triangle *current_p = *i;
//                if ((current_p->lowest_x() > this->b.x) && (current_p->lowest_x() < this->t.x))
//                {
//                    fp_t this_cost = this->calculate_sah_cost(current_p->lowest_x());
//                    if (this_cost < lowest_cost)
//                    {
//                        lowest_cost = this_cost;
//                        best_split  = current_p->lowest_x();
//                    }
//                }
//                if ((current_p->highest_x() > this->b.x) && (current_p->highest_x() < this->t.x))
//                {
//                    fp_t this_cost = this->calculate_sah_cost(current_p->highest_x());
//                    if (this_cost < lowest_cost)
//                    {
//                        lowest_cost = this_cost;
//                        best_split  = current_p->highest_x();
//                    }
//                }
//            }
//            break;
//        case y_axis:
//            for (primitive_list::const_iterator i=p.begin(); i!=p.end(); i++)
//            {
//                const triangle *current_p = *i;
//                if ((current_p->lowest_y() > this->b.y) && (current_p->lowest_y() < this->t.y))
//                {
//                    fp_t this_cost = this->calculate_sah_cost(current_p->lowest_y());
//                    if (this_cost < lowest_cost)
//                    {
//                        lowest_cost = this_cost;
//                        best_split  = current_p->lowest_y();
//                    }
//                }
//                if ((current_p->highest_y() > this->b.y) && (current_p->highest_y() < this->t.y))
//                {
//                    fp_t this_cost = this->calculate_sah_cost(current_p->highest_y());
//                    if (this_cost < lowest_cost)
//                    {
//                        lowest_cost = this_cost;
//                        best_split  = current_p->highest_y();
//                    }
//                }
//            }
//            break;
//        case z_axis:
//            for (primitive_list::const_iterator i=p.begin(); i!=p.end(); i++)
//            {
//                const triangle *current_p = *i;
//                if ((current_p->lowest_z() > this->b.z) && (current_p->lowest_z() < this->t.z))
//                {
//                    fp_t this_cost = this->calculate_sah_cost(current_p->lowest_z());
//                    if (this_cost < lowest_cost)
//                    {
//                        lowest_cost = this_cost;
//                        best_split  = current_p->lowest_z();
//                    }
//                }
//                if ((current_p->highest_z() > this->b.z) && (current_p->highest_z() < this->t.z))
//                {
//                    fp_t this_cost = this->calculate_sah_cost(current_p->highest_z());
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
//fp_t voxel::calculate_sah_cost(const fp_t s) const
//{
//    /* Surface area of sub-divides */
//    fp_t left_area, right_area;
//    fp_t x_dist = this->t.x - this->b.x;
//    fp_t y_dist = this->t.y - this->b.y;
//    fp_t z_dist = this->t.z - this->b.z;
//    switch (this->n)
//    {
//        case x_axis:
//            assert(s > this->b.x);
//            assert(s < this->t.x);
//            x_dist = s - this->b.x;
//            left_area  = (x_dist * y_dist) + (x_dist * z_dist) + (z_dist * y_dist);
//            x_dist = this->t.x - s;
//            right_area = (x_dist * y_dist) + (x_dist * z_dist) + (z_dist * y_dist);
//            break;
//        case y_axis:
//            assert(s > this->b.y);
//            assert(s < this->t.y);
//            y_dist = s - this->b.y;
//            left_area  = (x_dist * y_dist) + (x_dist * z_dist) + (z_dist * y_dist);
//            y_dist = this->t.y - s;
//            right_area = (x_dist * y_dist) + (x_dist * z_dist) + (z_dist * y_dist);
//            break;
//        case z_axis:
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
//    fp_t left_objects  = 0.0;
//    fp_t right_objects = 0.0;
//    switch (this->n)
//    {
//        case x_axis:
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
//        case y_axis:
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
//        case z_axis:
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
//    fp_t sah_cost = COST_OF_TRAVERSAL + (COST_OF_INTERSECTION * ((left_objects * left_area) + (right_objects * right_area)));
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
//fp_t voxel::calculate_sah_cost(const fp_t s, const fp_t l, const fp_t r) const
//{
//    /* Surface area of sub-divides */
//    fp_t left_area, right_area;
//    fp_t x_dist = this->t.x - this->b.x;
//    fp_t y_dist = this->t.y - this->b.y;
//    fp_t z_dist = this->t.z - this->b.z;
//    switch (this->n)
//    {
//        case x_axis:
//            assert(s > this->b.x);
//            assert(s < this->t.x);
//            x_dist = s - this->b.x;
//            left_area  = (x_dist * y_dist) + (x_dist * z_dist) + (z_dist * y_dist);
//            x_dist = this->t.x - s;
//            right_area = (x_dist * y_dist) + (x_dist * z_dist) + (z_dist * y_dist);
//            break;
//        case y_axis:
//            assert(s > this->b.y);
//            assert(s < this->t.y);
//            y_dist = s - this->b.y;
//            left_area  = (x_dist * y_dist) + (x_dist * z_dist) + (z_dist * y_dist);
//            y_dist = this->t.y - s;
//            right_area = (x_dist * y_dist) + (x_dist * z_dist) + (z_dist * y_dist);
//            break;
//        case z_axis:
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
//    fp_t sah_cost = COST_OF_TRAVERSAL + (COST_OF_INTERSECTION * ((l * left_area) + (r * right_area)));
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
//fp_t voxel::split_all_axis(fp_t *s, axis *normal)
//{
//    /* Find the best split position of the primitives */
//    axis best_axis   = not_set;
//    fp_t best_split  = not_set;
//    fp_t lowest_cost = *s;
//    
//     /* Initial primitive split */
//    unsigned size_of_prim = this->p->size();
//    fp_t     right = size_of_prim;
//    fp_t     left  = 0;
//    fp_t     guess;
//    fp_t     last_guess = MAX_DIST;
//    
//    fp_t x_dist = this->t.x - this->b.x;
//    fp_t y_dist = this->t.y - this->b.y;
//    fp_t z_dist = this->t.z - this->b.z;
//    
//    /* Elements for evaluation */
//    fp_t lo_x, lo_y, lo_z, hi_x, hi_y, hi_z;
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
//            fp_t left_area  = (x_dist * y_dist) + (x_dist * z_dist) + (z_dist * y_dist);
//            x_dist = this->t.x - guess;
//            fp_t right_area = (x_dist * y_dist) + (x_dist * z_dist) + (z_dist * y_dist);
//
//            fp_t this_cost = COST_OF_TRAVERSAL + (COST_OF_INTERSECTION * ((left * left_area) + (right * right_area)));
//            if (this_cost < lowest_cost)
//            {
//                lowest_cost = this_cost;
//                best_split  = guess;
//                best_axis   = x_axis;
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
//            fp_t left_area  = (x_dist * y_dist) + (x_dist * z_dist) + (z_dist * y_dist);
//            y_dist = this->t.y - guess;
//            fp_t right_area = (x_dist * y_dist) + (x_dist * z_dist) + (z_dist * y_dist);
//
//            fp_t this_cost = COST_OF_TRAVERSAL + (COST_OF_INTERSECTION * ((left * left_area) + (right * right_area)));
//            if (this_cost < lowest_cost)
//            {
//                lowest_cost = this_cost;
//                best_split  = guess;
//                best_axis   = y_axis;
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
//            fp_t left_area  = (x_dist * y_dist) + (x_dist * z_dist) + (z_dist * y_dist);
//            z_dist = this->t.z - guess;
//            fp_t right_area = (x_dist * y_dist) + (x_dist * z_dist) + (z_dist * y_dist);
//
//            fp_t this_cost = COST_OF_TRAVERSAL + (COST_OF_INTERSECTION * ((left * left_area) + (right * right_area)));
//            if (this_cost < lowest_cost)
//            {
//                lowest_cost = this_cost;
//                best_split  = guess;
//                best_axis   = z_axis;
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
//fp_t voxel::split_all_axis(fp_t *s, axis *normal)
//{
//    /* Find the best split position of the primitives */
//    axis best_axis   = not_set;
//    fp_t best_split  = not_set;
//    fp_t lowest_cost = *s;
//    
//     /* Initial primitive split */
//    unsigned size_of_prim = this->p->size();
//    fp_t     right = size_of_prim;
//    fp_t     left  = 0;
//    fp_t     guess;
//    fp_t     last_guess = MAX_DIST;
//    
//    fp_t x_dist = this->t.x - this->b.x;
//    fp_t y_dist = this->t.y - this->b.y;
//    fp_t z_dist = this->t.z - this->b.z;
//    
//    /* Elements for evaluation */
//    fp_t lo_x, lo_y, lo_z, hi_x, hi_y, hi_z;
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
//            fp_t left_area  = (x_dist * y_dist) + (x_dist * z_dist) + (z_dist * y_dist);
//            x_dist = this->t.x - guess;
//            fp_t right_area = (x_dist * y_dist) + (x_dist * z_dist) + (z_dist * y_dist);
//
//            fp_t this_cost = COST_OF_TRAVERSAL + (COST_OF_INTERSECTION * ((left * left_area) + (right * right_area)));
//            if (this_cost < lowest_cost)
//            {
//                lowest_cost = this_cost;
//                best_split  = guess;
//                best_axis   = x_axis;
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
//            fp_t left_area  = (x_dist * y_dist) + (x_dist * z_dist) + (z_dist * y_dist);
//            y_dist = this->t.y - guess;
//            fp_t right_area = (x_dist * y_dist) + (x_dist * z_dist) + (z_dist * y_dist);
//
//            fp_t this_cost = COST_OF_TRAVERSAL + (COST_OF_INTERSECTION * ((left * left_area) + (right * right_area)));
//            if (this_cost < lowest_cost)
//            {
//                lowest_cost = this_cost;
//                best_split  = guess;
//                best_axis   = y_axis;
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
//            fp_t left_area  = (x_dist * y_dist) + (x_dist * z_dist) + (z_dist * y_dist);
//            z_dist = this->t.z - guess;
//            fp_t right_area = (x_dist * y_dist) + (x_dist * z_dist) + (z_dist * y_dist);
//
//            fp_t this_cost = COST_OF_TRAVERSAL + (COST_OF_INTERSECTION * ((left * left_area) + (right * right_area)));
//            if (this_cost < lowest_cost)
//            {
//                lowest_cost = this_cost;
//                best_split  = guess;
//                best_axis   = z_axis;
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


//fp_t voxel::approximate_split_all_axis(fp_t *s, axis * normal) const
//{
//    /* Find the best split position of the primitives */
//    fp_t best_split     = MAX_DIST;
//    fp_t lowest_cost    = *s;
//    axis best_axis      = not_set;
//    
//    /* Adaptive sampling bin sizes */
//    const fp_t adapt_bin_size   = this->p->size() * (2.0/9.0);
//    
//    /* Voxel dimensions */
//    const fp_t xw       = this->t.x - this->b.x;
//    const fp_t yw       = this->t.y - this->b.y;
//    const fp_t zw       = this->t.z - this->b.z;
//    const fp_t xb_yw    = this->b.x * yw;
//    const fp_t xb_zw    = this->b.x * zw;
//    const fp_t xt_yw    = this->t.x * yw;
//    const fp_t xt_zw    = this->t.x * zw;
//    const fp_t yb_xw    = this->b.y * xw;
//    const fp_t yb_zw    = this->b.y * zw;
//    const fp_t yt_xw    = this->t.y * xw;
//    const fp_t yt_zw    = this->t.y * zw;
//    const fp_t zb_xw    = this->b.z * xw;
//    const fp_t zb_yw    = this->b.z * yw;
//    const fp_t zt_xw    = this->t.z * xw;
//    const fp_t zt_yw    = this->t.z * yw;
//    const fp_t xw_yw    = xw * yw;
//    const fp_t xw_zw    = xw * zw;
//    const fp_t yw_zw    = yw * zw;
//    const fp_t xw_p_yw  = xw + yw;
//    const fp_t xw_p_zw  = xw + zw;
//    const fp_t yw_p_zw  = yw + zw;
//   
//    /* Pick fixed points to evaluate */
//    fp_t divide_width   = xw * (1.0 / 9.0);
//    fp_t evaluate       = this->t.x;
//    
//    /* Fixed points samples and adaptive bin boundaries */
//    fp_t cl_x[9];
//    fp_t cr_x[9];
//    fp_t bins_per_sample[9];
//    fp_t adaptive_width[9];
//    cl_x[8]             = this->p->size();
//    cr_x[8]             = 0.0;
//    bins_per_sample[8]  = 8.0;
//    
//    fp_t sample[8];
//    for (int i = 0; i < 8; i++)
//    {
//        sample[i] = this->b.x + (divide_width * (fp_t)(i+1));
//    }
//    this->count_primitives(cl_x, cr_x, sample, 8, x_axis);
//    
//    for (int i = 7; i >= 0; i--)
//    {
////        evaluate -= divide_width;
////        cl_x[i] = count_primitives(&cr_x[i], evaluate, x_axis);
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
//    fp_t cl_ax[9];
//    fp_t cr_ax[9];
//    int adaptive_offset = 0;
//    for (int i = 0; i < 9; i++)
//    {
//        evaluate = this->b.x + (i * divide_width);
//        for (int j = adaptive_offset; j < (bins_per_sample[i] + adaptive_offset); j++)
//        {
//            evaluate   += adaptive_width[i];
////            cl_ax[j]    = count_primitives(&cr_ax[j], evaluate, x_axis);
//            sample[j] = evaluate;
//        }
//        adaptive_offset += bins_per_sample[i];
//    }
//    this->count_primitives(cl_ax, cr_ax, sample, 8, x_axis);
//    
//    /* Evaluate SAH minima for each sample as -b/2a */
//    adaptive_offset = 0;
//    fp_t last_cl    = 0.0;
//    fp_t last_cr    = this->p->size();
//    fp_t xs         = this->b.x;
//    for (int i = 0; i < 9; i++)
//    {
//        /* Process any adaptive samples in this bin first because they will be encountered first */
//        for (int j = adaptive_offset; j < (bins_per_sample[i] + adaptive_offset); j++)
//        {
//            /* Bin primitive gradients */
//            const fp_t clg = (cl_ax[j] - last_cl) / adaptive_width[i];
//            const fp_t crg = (cr_ax[j] - last_cr) / adaptive_width[i];
//            
//            /* Sample location */
//            const fp_t xs_yw    = xs * yw;
//            const fp_t xs_zw    = xs * zw;
//            
//            /* Minima point bound to this bin */
//            const fp_t a    = 2.0 * ((clg * yw_p_zw) - (crg * yw_p_zw));
//            const fp_t b    =   ( last_cl * yw_p_zw) - (clg * (xb_yw + xb_zw - yw_zw + xs_yw + xs_zw))
//                              + (-last_cr * yw_p_zw) + (crg * (xt_yw + xt_zw + yw_zw + xs_yw + xs_zw));
//            
//            const fp_t xm   = max(xs, min((xs + adaptive_width[i]), (b / a)));
//
//            /* Cost at minima */
//            const fp_t xmb  = xm - this->b.x;
//            const fp_t xmt  = this->t.x - xm;
//
//            const fp_t cost = COST_OF_TRAVERSAL + (COST_OF_INTERSECTION * 
//                              (((last_cl + (clg * (xm - xs))) * ((xmb * yw) + (xmb * zw) + yw_zw))  + 
//                               ((last_cr + (crg * (xm - xs))) * ((xmt * yw) + (xmt * zw) + yw_zw))));
//            
//            if (cost < lowest_cost)
//            {
//                lowest_cost = cost;
//                best_axis   = x_axis;
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
//        const fp_t clg = (cl_x[i] - last_cl) / adaptive_width[i];
//        const fp_t crg = (cr_x[i] - last_cr) / adaptive_width[i];
//      
//        /* Sample location */
//        const fp_t xs_yw    = xs * yw;
//        const fp_t xs_zw    = xs * zw;
//      
//        /* Minima point bound to this bin */
//        const fp_t a    = 2.0 * ((clg * yw_p_zw) - (crg * yw_p_zw));
//        const fp_t b    =   ( last_cl * yw_p_zw) - (clg * (xb_yw + xb_zw - yw_zw + xs_yw + xs_zw))
//                          + (-last_cr * yw_p_zw) + (crg * (xt_yw + xt_zw + yw_zw + xs_yw + xs_zw));
//      
//        const fp_t xm   = max(xs, min((xs + adaptive_width[i]), (b / a)));
//      
//        /* Cost at minima */
//        const fp_t xmb  = xm - this->b.x;
//        const fp_t xmt  = this->t.x - xm;
//
//        const fp_t cost = COST_OF_TRAVERSAL + (COST_OF_INTERSECTION * 
//                          (((last_cl + (clg * (xm - xs))) * ((xmb * yw) + (xmb * zw) + yw_zw))  + 
//                           ((last_cr + (crg * (xm - xs))) * ((xmt * yw) + (xmt * zw) + yw_zw))));
//      
//        if (cost < lowest_cost)
//        {
//            lowest_cost = cost;
//            best_axis   = x_axis;
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
//    fp_t cl_y[9];
//    fp_t cr_y[9];
//    cl_y[8]             = this->p->size();
//    cr_y[8]             = 0.0;
//    bins_per_sample[8]  = 8.0;
//
//    for (int i = 0; i < 8; i++)
//    {
//        sample[i] = this->b.y + (divide_width * (fp_t)(i+1));
//    }
//    this->count_primitives(cl_y, cr_y, sample, 8, y_axis);
//
//    for (int i = 7; i >= 0; i--)
//    {
////        evaluate -= divide_width;
////        cl_y[i] = count_primitives(&cr_y[i], evaluate, y_axis);
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
//    fp_t cl_ay[9];
//    fp_t cr_ay[9];
//    adaptive_offset = 0;
//    for (int i = 0; i < 9; i++)
//    {
//        evaluate = this->b.y + (i * divide_width);
//        for (int j = adaptive_offset; j < (bins_per_sample[i] + adaptive_offset); j++)
//        {
//            evaluate   += adaptive_width[i];
////            cl_ay[j]    = count_primitives(&cr_ay[j], evaluate, y_axis);
//            sample[j] = evaluate;
//        }
//        adaptive_offset += bins_per_sample[i];
//    }
//    this->count_primitives(cl_ay, cr_ay, sample, 8, y_axis);
//    
//    /* Evaluate SAH minima for each sample as -b/2a */
//    adaptive_offset = 0;
//         last_cl    = 0.0;
//         last_cr    = this->p->size();
//    fp_t ys         = this->b.y;
//    for (int i = 0; i < 9; i++)
//    {
//        /* Process any adaptive samples in this bin first because they will be encountered first */
//        for (int j = adaptive_offset; j < (bins_per_sample[i] + adaptive_offset); j++)
//        {
//            /* Bin primitive gradients */
//            const fp_t clg = (cl_ay[j] - last_cl) / adaptive_width[i];
//            const fp_t crg = (cr_ay[j] - last_cr) / adaptive_width[i];
//            
//            /* Sample location */
//            const fp_t ys_xw    = ys * xw;
//            const fp_t ys_zw    = ys * zw;
//            
//            /* Minima point bound to this bin */
//            const fp_t a    = 2.0 * ((clg * xw_p_zw) - (crg * xw_p_zw));
//            const fp_t b    =   ( last_cl * xw_p_zw) - (clg * (yb_xw + yb_zw - xw_zw + ys_xw + ys_zw))
//                              + (-last_cr * xw_p_zw) + (crg * (yt_xw + yt_zw + xw_zw + ys_xw + ys_zw));
//            
//            const fp_t ym   = max(ys, min((ys + adaptive_width[i]), (b / a)));
//            
//            /* Cost at minima */
//            const fp_t ymb  = ym - this->b.y;
//            const fp_t ymt  = this->t.y - ym;
//
//            const fp_t cost = COST_OF_TRAVERSAL + (COST_OF_INTERSECTION * 
//                              (((last_cl + (clg * (ym - ys))) * ((ymb * xw) + (ymb * zw) + xw_zw))  + 
//                               ((last_cr + (crg * (ym - ys))) * ((ymt * xw) + (ymt * zw) + xw_zw))));
//            
//            if (cost < lowest_cost)
//            {
//                lowest_cost = cost;
//                best_axis   = y_axis;
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
//        const fp_t clg = (cl_y[i] - last_cl) / adaptive_width[i];
//        const fp_t crg = (cr_y[i] - last_cr) / adaptive_width[i];
//      
//        /* Sample location */
//        const fp_t ys_xw    = ys * xw;
//        const fp_t ys_zw    = ys * zw;
//      
//        /* Minima point bound to this bin */
//        const fp_t a    = 2.0 * ((clg * xw_p_zw) - (crg * xw_p_zw));
//        const fp_t b    =   ( last_cl * xw_p_zw) - (clg * (yb_xw + yb_zw - xw_zw + ys_xw + ys_zw))
//                          + (-last_cr * xw_p_zw) + (crg * (yt_xw + yt_zw + xw_zw + ys_xw + ys_zw));
//      
//        const fp_t ym   = max(ys, min((ys + adaptive_width[i]), (b / a)));
//      
//        /* Cost at minima */
//        const fp_t ymb  = ym - this->b.y;
//        const fp_t ymt  = this->t.y - ym;
//
//        const fp_t cost = COST_OF_TRAVERSAL + (COST_OF_INTERSECTION * 
//                          (((last_cl + (clg * (ym - ys))) * ((ymb * xw) + (ymb * zw) + xw_zw))  + 
//                           ((last_cr + (crg * (ym - ys))) * ((ymt * xw) + (ymt * zw) + xw_zw))));
//      
//        if (cost < lowest_cost)
//        {
//            lowest_cost = cost;
//            best_axis   = y_axis;
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
//    fp_t cl_z[9];
//    fp_t cr_z[9];
//    cl_z[8]             = this->p->size();
//    cr_z[8]             = 0.0;
//    bins_per_sample[8]  = 8.0;
//
//    for (int i = 0; i < 8; i++)
//    {
//        sample[i] = this->b.z + (divide_width * (fp_t)(i+1));
//    }
//    this->count_primitives(cl_z, cr_z, sample, 8, z_axis);
//
//    for (int i = 7; i >= 0; i--)
//    {
////        evaluate -= divide_width;
////        cl_z[i] = count_primitives(&cr_z[i], evaluate, z_axis);
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
//    fp_t cl_az[9];
//    fp_t cr_az[9];
//    adaptive_offset = 0;
//    for (int i = 0; i < 9; i++)
//    {
//        evaluate = this->b.z + (i * divide_width);
//        for (int j = adaptive_offset; j < (bins_per_sample[i] + adaptive_offset); j++)
//        {
//            evaluate   += adaptive_width[i];
//            //cl_az[j]    = count_primitives(&cr_az[j], evaluate, z_axis);
//            sample[j] = evaluate;
//        }
//        adaptive_offset += bins_per_sample[i];
//    }
//    this->count_primitives(cl_az, cr_az, sample, 8, z_axis);
//
//    
//    /* Evaluate SAH minima for each sample as -b/2a */
//    adaptive_offset = 0;
//         last_cl    = 0.0;
//         last_cr    = this->p->size();
//    fp_t zs         = this->b.z;
//    for (int i = 0; i < 9; i++)
//    {
//        /* Process any adaptive samples in this bin first because they will be encountered first */
//        for (int j = adaptive_offset; j < (bins_per_sample[i] + adaptive_offset); j++)
//        {
//            /* Bin primitive gradients */
//            const fp_t clg = (cl_az[j] - last_cl) / adaptive_width[i];
//            const fp_t crg = (cr_az[j] - last_cr) / adaptive_width[i];
//            
//            /* Sample location */
//            const fp_t zs_xw    = zs * xw;
//            const fp_t zs_yw    = zs * yw;
//            
//            /* Minima point bound to this bin */
//            const fp_t a    = 2.0 * ((clg * xw_p_yw) - (crg * xw_p_zw));
//            const fp_t b    =   ( last_cl * xw_p_yw) - (clg * (zb_xw + zb_yw - xw_yw + zs_xw + zs_yw))
//                              + (-last_cr * xw_p_yw) + (crg * (zt_xw + zt_yw + xw_yw + zs_xw + zs_yw));
//            
//            const fp_t zm   = max(zs, min((zs + adaptive_width[i]), (b / a)));
//            
//            /* Cost at minima */
//            const fp_t zmb  = zm - this->b.z;
//            const fp_t zmt  = this->t.z - zm;
//
//            const fp_t cost = COST_OF_TRAVERSAL + (COST_OF_INTERSECTION * 
//                              (((last_cl + (clg * (zm - zs))) * ((zmb * xw) + (zmb * yw) + xw_yw))  + 
//                               ((last_cr + (crg * (zm - zs))) * ((zmt * xw) + (zmt * yw) + xw_yw))));
//            
//            if (cost < lowest_cost)
//            {
//                lowest_cost = cost;
//                best_axis   = z_axis;
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
//        const fp_t clg = (cl_z[i] - last_cl) / adaptive_width[i];
//        const fp_t crg = (cr_z[i] - last_cr) / adaptive_width[i];
//      
//        /* Sample location */
//        const fp_t zs_xw    = zs * xw;
//        const fp_t zs_yw    = zs * yw;
//     
//        /* Minima point bound to this bin */
//        const fp_t a    = 2.0 * ((clg * xw_p_zw) - (crg * xw_p_zw));
//        const fp_t b    =   ( last_cl * xw_p_zw) - (clg * (zb_xw + zb_yw - xw_yw + zs_xw + zs_yw))
//                          + (-last_cr * xw_p_zw) + (crg * (zt_xw + zt_yw + xw_yw + zs_xw + zs_yw));
//      
//        const fp_t zm   = max(zs, min((zs + adaptive_width[i]), (b / a)));
//      
//        /* Cost at minima */
//        const fp_t zmb  = zm - this->b.z;
//        const fp_t zmt  = this->t.z - zm;
//
//        const fp_t cost = COST_OF_TRAVERSAL + (COST_OF_INTERSECTION * 
//                          (((last_cl + (clg * (zm - zs))) * ((zmb * xw) + (zmb * yw) + xw_yw))  + 
//                           ((last_cr + (crg * (zm - zs))) * ((zmt * xw) + (zmt * yw) + xw_yw))));
//      
//        if (cost < lowest_cost)
//        {
//            lowest_cost = cost;
//            best_axis   = z_axis;
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




//fp_t voxel::stl_split_all_axis(fp_t *s, axis *normal)
//{
//    axis best_axis;
//    /* Find the best split position of the primitives */
//    fp_t best_split;
//    fp_t lowest_cost = *s;
//    
//     /* Initial primitive split */
//    unsigned size_of_prim = this->p->size();
//    fp_t right = size_of_prim;
//    fp_t left  = 0;
//    fp_t guess;
//    fp_t last_guess = MAX_DIST;
//    
//    fp_t x_dist = this->t.x - this->b.x;
//    fp_t y_dist = this->t.y - this->b.y;
//    fp_t z_dist = this->t.z - this->b.z;
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
//    fp_t lo_x = -MAX_DIST;
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
//    fp_t hi_x = -MAX_DIST;
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
//            fp_t this_cost = calculate_sah_cost(left, right, guess, x_axis);
//            if (this_cost < lowest_cost)
//            {
//                lowest_cost = this_cost;
//                best_split  = guess;
//                best_axis   = x_axis;
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
//    fp_t lo_y = -MAX_DIST;
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
//    fp_t hi_y = -MAX_DIST;
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
//            fp_t this_cost = calculate_sah_cost(left, right, guess, y_axis);
//            if (this_cost < lowest_cost)
//            {
//                lowest_cost = this_cost;
//                best_split  = guess;
//                best_axis   = y_axis;
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
//    fp_t lo_z = -MAX_DIST;
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
//    fp_t hi_z = -MAX_DIST;
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
//            fp_t this_cost = calculate_sah_cost(left, right, guess, z_axis);
//            if (this_cost < lowest_cost)
//            {
//                lowest_cost = this_cost;
//                best_split  = guess;
//                best_axis   = z_axis;
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
//        case x_axis:
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
//        case y_axis:
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
//        case z_axis:
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
