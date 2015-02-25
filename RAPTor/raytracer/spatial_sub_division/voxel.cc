/* Standard header */

/* Boost headers */

/* Common headers */
#include "logging.h"

/* Raytracer headers */
#include "voxel.h"


namespace raptor_raytracer
{
float approximate_sah_minima(const float cl0, const float cl1, const float cr0, const float cr1, const float x0, const float d, const float xw, const float yw, const float zw)
{
    /* Check there is a gradient to the counts, minimise area with the most primitives */
    const float c_grad = (cl0 - cl1 - cr0 + cr1);
    if (c_grad == 0.0f)
    {
        return (cl0 > cr0) ? -MAX_DIST : MAX_DIST;
    }

    const float a = 2.0f * c_grad * (yw + zw);
    const float b = ((cl0 * (d - x0)) + (cl1 * x0) - (cr1 * x0) + (cr1 * xw) - (cr0 * (d - x0 + xw))) * yw + 
        ((cl0 * (d - x0 - yw)) + (cl1 * (x0 + yw)) + (cr1 * (xw + yw - x0)) - (cr0 * (d - x0 + xw + yw))) * zw;
    return (b / a);
}

template<typename CostFn>
float find_sah_minima(const CostFn &cost_fn, float *const lc, const float *const cl, const float *const cr, const float *const cla, const float *const cra, 
    const float *const adaptive_width, const float *const bins_per_sample, const float xw, const float yw, const float zw, const float b, const int nr_prims)
{
    int adaptive_offset = 0;
    float best_split    = MAX_DIST;
    float lowest_cost   = *lc;
    float last_cl       = 0.0f;
    float last_cr       = nr_prims;
    float sp            = b;
    for (int i = 0; i < 9; ++i)
    {
        /* Process any adaptive samples in this bin first because they will be encountered first */
        for (int j = adaptive_offset; j < (bins_per_sample[i] + adaptive_offset); ++j)
        {
            /* Sample location */
            const float sp_delta = approximate_sah_minima(last_cl, cla[j], last_cr, cra[j], sp, adaptive_width[i], xw, yw, zw);
            const float xm       = std::max(sp, std::min(sp + adaptive_width[i], sp + sp_delta));

            /* Cost at minima */
            const float cost = cost_fn(xm, cla[j], cra[j], last_cl, last_cr, sp, adaptive_width[i]);
            // BOOST_LOG_TRIVIAL(trace) << "sp: " << sp << ", cla: " << cla[j] << ", cra: " << cra[j] << ", split " << xm << ", cost: " << cost;
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

        /* Sample location */
        const float sp_delta = approximate_sah_minima(last_cl, cl[i], last_cr, cr[i], sp, adaptive_width[i], xw, yw, zw);
        const float xm       = std::max(sp, std::min(sp + adaptive_width[i], sp + sp_delta));
      
        /* Cost at minima */
        const float cost = cost_fn(xm, cl[i], cr[i], last_cl, last_cr, sp, adaptive_width[i]);
        // BOOST_LOG_TRIVIAL(trace) << "sp: " << sp << ", cl: " << cl[i] << ", cr: " << cr[i] << ", split: " << xm << ", cost: " << cost;
        if (cost < lowest_cost)
        {
            lowest_cost = cost;
            best_split  = xm;
        }
      
        last_cl = cl[i];
        last_cr = cr[i];
        sp     += adaptive_width[i];
    }

    *lc = lowest_cost;
    // assert(false);
    return best_split;
}

void fix_adaptive_samples(float *const nr_samples, float *const widths, float *const samples, const float *const cl, const float *const cr, const float d0, const float dw, const float prims)
{
    /* Fix adaptive sample width per fixed bin */
    /* Calculate cl - cr */
    for (int i = 0; i < 8; ++i)
    {
        widths[i] = cl[i] - cr[i];
    }
    widths[8] = prims;

    /* Work out which bin each cl - cr sample falls into */
    int cl_m_cr_sample_idx = 0;
    float cl_m_cr_sample = -prims;
    memset(&nr_samples[0], 0, 9 * sizeof(int));
    const float bin_size = (2.0f * prims) / 9.0f;
    for (int i = 0; i < 8; ++i)
    {
        cl_m_cr_sample += bin_size;
        while (widths[cl_m_cr_sample_idx] < cl_m_cr_sample)
        {
            ++cl_m_cr_sample_idx;
        }

        nr_samples[cl_m_cr_sample_idx]++;
    }

    /* Work out widths from number of samples and position adaptive samples */
    int sample_idx = 0;
    for (int i = 0; i < 9; ++i)
    {
        widths[i] = dw / (nr_samples[i] + 1.0f);

        float evaluate = d0 + (static_cast<float>(i) * dw);
        for (int j = 0; j < nr_samples[i]; ++j)
        {
            evaluate               += widths[i];
            samples[sample_idx++]   = evaluate;
        }
    }
}


voxel voxel::divide(kdt_node *const k, kdt_node *const children, const int depth)
{
    /* If we hit the depth limit we're done */
    assert(depth < MAX_KDT_STACK_HEIGHT);
    if ((depth + 1) == MAX_KDT_STACK_HEIGHT)
    {
        auto leaf_prims = new primitive_list(_nr_prims);
        for (int i = 0; i < _nr_prims; ++i)
        {
            (*leaf_prims)[i] = (*_ping)[_ping_idx + i].prim;
        }
        k->set_primitives(leaf_prims);
        return *this;
    }

    /* Calculate the cost of this node */
    float lowest_cost = COST_OF_INTERSECTION * _nr_prims;
    float cost_before = lowest_cost;

#if 1   /* Approximate builder */
    axis_t normal;
    float best_split;
    
#ifdef SIMD_PACKET_TRACING
    /* If using SIMD allow early exit for nodes under a given size */
    if (_nr_prims <= MIN_KDT_NODE_SIZE)
    {
        auto leaf_prims = new primitive_list(_nr_prims);
        for (int i = 0; i < _nr_prims; ++i)
        {
            (*leaf_prims)[i] = (*_ping)[_ping_idx + i].prim;
        }
        k->set_primitives(leaf_prims);
        return *this;
    } 
    else
#endif /* #ifdef SIMD_PACKET_TRACING */
    /* Invoke the exact builder for small node */
    if (_nr_prims <= MIN_APPROX_KDT_BUILDER_NODE_SIZE)
    {
        // BOOST_LOG_TRIVIAL(trace) << "Calling brute force builder";
        best_split = brute_force_split_all_axis(&lowest_cost, &normal);
    }
    else
    {
        // BOOST_LOG_TRIVIAL(trace) << "Calling approximate builder";
#if 0   /* Split approximately in all axis */
        best_split = approximate_split_all_axis(&lowest_cost, &normal);
#else   /* Split approximately in the first lower cost axis */

        const float dx = _t.x - _b.x;
        const float dy = _t.y - _b.y;
        const float dz = _t.z - _b.z;
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
    float best_split = split_this_axis(&lowest_cost);
#else   /* All axis aplit */
    axis_t normal;
    float best_split = split_all_axis(&lowest_cost, &normal);
#endif
#endif

    /* Stop splitting if the cost metric cannot be reduced */
    assert(lowest_cost >= 0.0f);
    if (lowest_cost >= cost_before)
    {
        auto leaf_prims = new primitive_list(_nr_prims);
        for (int i = 0; i < _nr_prims; ++i)
        {
            (*leaf_prims)[i] = (*_ping)[_ping_idx + i].prim;
        }
        k->set_primitives(leaf_prims);
        return *this;
    }
    // BOOST_LOG_TRIVIAL(trace) << "Lowest cost: " << lowest_cost << ", cost before: " << cost_before;
    
    /* Adjust the size of the voxel */
    point_t upper_limit = _t;
    point_t lower_limit = _b;
    axis_t  nxt_n;
    switch (normal)
    {
        case axis_t::x_axis:
            /* Assert the the voxel is really split */
            // BOOST_LOG_TRIVIAL(trace) << std::setprecision(10) << "Splitting: " << _b.x << " - " << best_split << " - " << _t.x << std::setprecision(6);
            assert(best_split > _b.x);
            assert(best_split < _t.x);
            lower_limit.x   = best_split;
            _t.x            = best_split;
            nxt_n           = axis_t::y_axis;
            break;
        case axis_t::y_axis:
            // BOOST_LOG_TRIVIAL(trace) << std::setprecision(10) << "Splitting: " << _b.y << " - " << best_split << " - " << _t.y << std::setprecision(6);
            assert(best_split > _b.y);
            assert(best_split < _t.y);
            lower_limit.y   = best_split;
            _t.y            = best_split;
            nxt_n           = axis_t::z_axis;
            break;
        case axis_t::z_axis:
            // BOOST_LOG_TRIVIAL(trace) << std::setprecision(10) << "Splitting: " << _b.z << " - " << best_split << " - " << _t.z << std::setprecision(6);
            assert(best_split > _b.z);
            assert(best_split < _t.z);
            lower_limit.z   = best_split;
            _t.z            = best_split;
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
    if ((_pong_idx + _nr_prims) > static_cast<int>(_pong->size()))
    {
        _pong->resize(_pong->size() << 1);
    }
    
    int left_idx = _pong_idx;
    int right_idx = _ping_idx;
    switch (normal)
    {
        case axis_t::x_axis:
            for (voxel_aab_data *i = &(*_ping)[_ping_idx]; i < &(*_ping)[(_ping_idx + _nr_prims)]; ++i)
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
                if (i->low.x <= best_split)
                {
                    (*_pong)[left_idx++]  = *i;
                }

                if (i->high.x >= best_split)
                {
                    (*_ping)[right_idx++] = *i;
                }
            }
            break;
        case axis_t::y_axis:
            for (voxel_aab_data *i = &(*_ping)[_ping_idx]; i < &(*_ping)[(_ping_idx + _nr_prims)]; ++i)
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
                if (i->low.y <= best_split)
                {
                    (*_pong)[left_idx++]  = *i;
                }

                if (i->high.y >= best_split)
                {
                    (*_ping)[right_idx++] = *i;
                }
            }
            break;
        case axis_t::z_axis:
            for (voxel_aab_data *i = &(*_ping)[_ping_idx]; i < &(*_ping)[(_ping_idx + _nr_prims)]; ++i)
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
                if (i->low.z <= best_split)
                {
                    (*_pong)[left_idx++]  = *i;
                }

                if (i->high.z >= best_split)
                {
                    (*_ping)[right_idx++] = *i;
                }
            }
            break;
        default :
            assert(false);
            break;
    }

    /* Set the split kdt node */
    k->split_node(children, nearest_clip, normal);

    assert(_nr_prims <= (left_idx + right_idx));
    _n = nxt_n;

    const int ping_idx = _ping_idx;
    const int pong_idx = _pong_idx;

    _nr_prims = left_idx - pong_idx;
    _ping_idx = right_idx;
    std::swap(_ping, _pong);
    std::swap(_ping_idx, _pong_idx);
    return voxel(_pong, _ping, ping_idx, left_idx, right_idx - ping_idx, upper_limit, lower_limit, nxt_n);
}


float voxel::approximate_split_one_axis(float *const s, const axis_t normal) const
{
    /* Voxel dimensions */
    const float xw      = _t.x - _b.x;
    const float yw      = _t.y - _b.y;
    const float zw      = _t.z - _b.z;
    const float sa_inv  = 1.0f / ((xw * yw) + (xw * zw) + (yw * zw));
    const float xw_yw   = xw * yw;
    const float xw_zw   = xw * zw;
    const float yw_zw   = yw * zw;
    // BOOST_LOG_TRIVIAL(trace) << "Widths: " << xw << ", " << yw << ", " << zw;
    // BOOST_LOG_TRIVIAL(trace) << "Bottoms: " << _b;
    
    /* Sampling variable */
    float cl[9];
    float cr[9];
    float cla[9];
    float cra[9];
    float best_split = MAX_DIST;
    float adaptive_width[9];
    float bins_per_sample[9];
    float sample[8] ALIGN(16);
    switch (normal)
    {
        case axis_t::x_axis :
        {
            // BOOST_LOG_TRIVIAL(trace) << "Splitting x"; 
            /* Pick fixed points to evaluate */
            const float dw = xw * (1.0f / 9.0f);

            cr[8] = 0.0f;
            cl[8] = _nr_prims;
            sample[0] = _b.x + dw;
            for (int i = 1; i < 8; ++i)
            {
                sample[i] = sample[i - 1] + dw;
            }
            count_primitives(cl, cr, sample, 8, axis_t::x_axis);
            
            /* Take adaptive samples */
            fix_adaptive_samples(bins_per_sample, adaptive_width, sample, cl, cr, _b.x, dw, _nr_prims);
            count_primitives(cla, cra, sample, 8, axis_t::x_axis);
            
            /* Evaluate SAH minima for each sample */
            best_split = find_sah_minima([this, xw, yw, zw, yw_zw, sa_inv](const float split, const float cl, const float cr, const float last_cl, const float last_cr, const float sp, const float width)
                {
                    const float xmb  = split - _b.x;
                    const float xmt  = _t.x - split;
                    const float clg  = (cl - last_cl) / width;
                    const float crg  = (cr - last_cr) / width;
                    const float cost = COST_OF_TRAVERSAL + (COST_OF_INTERSECTION * 
                          (((last_cl + (clg * (split - sp))) * ((xmb * yw) + (xmb * zw) + yw_zw) * sa_inv)  + 
                           ((last_cr + (crg * (split - sp))) * ((xmt * yw) + (xmt * zw) + yw_zw) * sa_inv)));

                    return cost;
                }, s, cl, cr, cla, cra, adaptive_width, bins_per_sample, xw, yw, zw, _b.x, _nr_prims);
            break;
        }            
        case axis_t::y_axis :
        {
            // BOOST_LOG_TRIVIAL(trace) << "Splitting y";
            /* Pick fixed points to evaluate */
            const float dw = yw * (1.0f / 9.0f);
            
            cr[8] = 0.0f;
            cl[8] = _nr_prims;
            sample[0] = _b.y + dw;
            for (int i = 1; i < 8; ++i)
            {
                sample[i] = sample[i - 1] + dw;
            }
            count_primitives(cl, cr, sample, 8, axis_t::y_axis);
        
            /* Take adaptive samples */
            fix_adaptive_samples(bins_per_sample, adaptive_width, sample, cl, cr, _b.y, dw, _nr_prims);
            count_primitives(cla, cra, sample, 8, axis_t::y_axis);
            
            /* Evaluate SAH minima for each sample */
            best_split = find_sah_minima([this, xw, yw, zw, xw_zw, sa_inv](const float split, const float cl, const float cr, const float last_cl, const float last_cr, const float sp, const float width)
                {
                    const float ymb  = split - _b.y;
                    const float ymt  = _t.y - split;
                    const float clg  = (cl - last_cl) / width;
                    const float crg  = (cr - last_cr) / width;
                    const float cost = COST_OF_TRAVERSAL + (COST_OF_INTERSECTION * 
                                      (((last_cl + (clg * (split - sp))) * ((ymb * xw) + (ymb * zw) + xw_zw) * sa_inv)  + 
                                       ((last_cr + (crg * (split - sp))) * ((ymt * xw) + (ymt * zw) + xw_zw) * sa_inv)));

                    return cost;
                }, s, cl, cr, cla, cra, adaptive_width, bins_per_sample, yw, xw, zw, _b.y, _nr_prims);
            break;
        }
        case axis_t::z_axis :
        {
            // BOOST_LOG_TRIVIAL(trace) << "Splitting z"
            /* Pick fixed points to evaluate */
            const float dw = zw * (1.0f / 9.0f);
            
            cr[8] = 0.0f;
            cl[8] = _nr_prims;
            sample[0] = _b.z + dw;
            for (int i = 1; i < 8; ++i)
            {
                sample[i] = sample[i - 1] + dw;
            }
            count_primitives(cl, cr, sample, 8, axis_t::z_axis);
        
            /* Take adaptive samples */
            fix_adaptive_samples(bins_per_sample, adaptive_width, sample, cl, cr, _b.z, dw, _nr_prims);
            count_primitives(cla, cra, sample, 8, axis_t::z_axis);
            
            /* Evaluate SAH minima for each sample */
            best_split = find_sah_minima([this, xw, yw, zw, xw_yw, sa_inv](const float split, const float cl, const float cr, const float last_cl, const float last_cr, const float sp, const float width)
                {
                    const float zmb  = split - _b.z;
                    const float zmt  = _t.z - split;
                    const float clg  = (cl - last_cl) / width;
                    const float crg  = (cr - last_cr) / width;
                    const float cost = COST_OF_TRAVERSAL + (COST_OF_INTERSECTION * 
                                      (((last_cl + (clg * (split - sp))) * ((zmb * xw) + (zmb * yw) + xw_yw) * sa_inv)  + 
                                       ((last_cr + (crg * (split - sp))) * ((zmt * xw) + (zmt * yw) + xw_yw) * sa_inv)));

                    return cost;
                }, s, cl, cr, cla, cra, adaptive_width, bins_per_sample, zw, yw, xw, _b.z, _nr_prims);
            break;
        }
        default :
            assert(false);
            break;
    }

    return best_split;
}

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
    for (int i = _ping_idx; i < static_cast<int>(_ping_idx + _nr_prims); ++i)
    {
        /* Add samples to their bucket */
        if (((*_ping)[i].low.x > _b.x) && ((*_ping)[i].low.x < _t.x))
        {
            sample[bucketted++] = (*_ping)[i].low.x;
        }

        if (((*_ping)[i].high.x > _b.x) && ((*_ping)[i].high.x < _t.x))
        {
            sample[bucketted++] = (*_ping)[i].high.x;
        }
        
        /* Check for bucket full */
        if (bucketted > 7)
        {
            s_vec[0] = &sample[0];
            s_vec[1] = &sample[SIMD_WIDTH];
            count_primitives(l_vec, r_vec, s_vec, axis_t::x_axis);
            for (int j = 0; j < 2; j++)
            {
                vfp_t cost(calculate_sah_cost(l_vec[j], r_vec[j], s_vec[j], axis_t::x_axis));
                vfp_t mask(cost < lc_vec);

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
    
    count_primitives(l_vec, r_vec, s_vec, axis_t::x_axis);
    for (int j = 0; j < ((bucketted >> LOG2_SIMD_WIDTH) + (int)(bucketted > 0)); j++)
    {
        vfp_t cost(calculate_sah_cost(l_vec[j], r_vec[j], s_vec[j], axis_t::x_axis));
        vfp_t mask((cost < lc_vec) & valid_mask[j]);

        lc_vec = mov_p(mask, cost,                lc_vec);
        bs_vec = mov_p(mask, s_vec[j],            bs_vec);
        ba_vec = mov_p(mask, vfp_t(static_cast<float>(axis_t::x_axis)), ba_vec);
    }

    /* Evaluate the Y axis */
    bucketted = 0;
    for (int i = _ping_idx; i < static_cast<int>(_ping_idx + _nr_prims); ++i)
    {
        /* Add samples to their bucket */
        if (((*_ping)[i].low.y > _b.y) && ((*_ping)[i].low.x < _t.y))
        {
            sample[bucketted++] = (*_ping)[i].low.y;
        }

        if (((*_ping)[i].high.y > _b.y) && ((*_ping)[i].high.y < _t.y))
        {
            sample[bucketted++] = (*_ping)[i].high.y;
        }
        
        /* Check for bucket full */
        if (bucketted > 7)
        {
            s_vec[0] = &sample[0];
            s_vec[1] = &sample[SIMD_WIDTH];
            count_primitives(l_vec, r_vec, s_vec, axis_t::y_axis);
            for (int j = 0; j < 2; j++)
            {
                vfp_t cost(calculate_sah_cost(l_vec[j], r_vec[j], s_vec[j], axis_t::y_axis));
                vfp_t mask(cost < lc_vec);

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
    
    count_primitives(l_vec, r_vec, s_vec, axis_t::y_axis);
    for (int j = 0; j < ((bucketted >> LOG2_SIMD_WIDTH) + (int)(bucketted > 0)); j++)
    {
        vfp_t cost(calculate_sah_cost(l_vec[j], r_vec[j], s_vec[j], axis_t::y_axis));
        vfp_t mask((cost < lc_vec) & valid_mask[j]);

        lc_vec = mov_p(mask, cost,                lc_vec);
        bs_vec = mov_p(mask, s_vec[j],            bs_vec);
        ba_vec = mov_p(mask, vfp_t(static_cast<float>(axis_t::y_axis)), ba_vec);
    }

    /* Evaluate the Z axis */
    bucketted = 0;
    for (int i = _ping_idx; i < static_cast<int>(_ping_idx + _nr_prims); ++i)
    {
        /* Add samples to their bucket */
        if (((*_ping)[i].low.z > _b.z) && ((*_ping)[i].low.x < _t.z))
        {
            sample[bucketted++] = (*_ping)[i].low.z;
        }

        if (((*_ping)[i].high.z > _b.z) && ((*_ping)[i].high.z < _t.z))
        {
            sample[bucketted++] = (*_ping)[i].high.z;
        }
        
        /* Check for bucket full */
        if (bucketted > 7)
        {
            s_vec[0] = &sample[0];
            s_vec[1] = &sample[SIMD_WIDTH];
            count_primitives(l_vec, r_vec, s_vec, axis_t::z_axis);
            for (int j = 0; j < 2; j++)
            {
                vfp_t cost(calculate_sah_cost(l_vec[j], r_vec[j], s_vec[j], axis_t::z_axis));
                vfp_t mask(cost < lc_vec);

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
    
    count_primitives(l_vec, r_vec, s_vec, axis_t::z_axis);
    for (int j = 0; j < ((bucketted >> LOG2_SIMD_WIDTH) + static_cast<int>(bucketted > 0)); ++j)
    {
        vfp_t cost(calculate_sah_cost(l_vec[j], r_vec[j], s_vec[j], axis_t::z_axis));
        vfp_t mask((cost < lc_vec) & valid_mask[j]);

        lc_vec = mov_p(mask, cost,                lc_vec);
        bs_vec = mov_p(mask, s_vec[j],            bs_vec);
        ba_vec = mov_p(mask, vfp_t(static_cast<float>(axis_t::z_axis)), ba_vec);
    }
    
    /* Collect results */
    (*s) = horizontal_min(lc_vec);

    const int index = mask_to_index_lut[move_mask(vfp_t(*s) == lc_vec)];
    (*normal) = static_cast<axis_t>(ba_vec[index]);
    return bs_vec[index];
}

// __attribute__((optimize("unroll-loops")))
void voxel::count_primitives(float *const l, float *const r, const float *const s, const int len, const axis_t n) const
{
    assert(len == 8);
    vfp_t l_vec[2] = { vfp_zero, vfp_zero };
    vfp_t r_vec[2] = { vfp_zero, vfp_zero };
    vfp_t s_vec[2] = { &s[0], &s[SIMD_WIDTH] };


    /* Count in the given axis */
    switch (n)
    {
        case axis_t::x_axis:
            for (int i = _ping_idx; i < static_cast<int>(_ping_idx + _nr_prims); ++i)
            {
                const vfp_t lo((*_ping)[i].low.x);
                const vfp_t hi((*_ping)[i].high.x);
                l_vec[0] += (lo <= s_vec[0]) & vfp_one;
                r_vec[0] += (hi >  s_vec[0]) & vfp_one;
                l_vec[1] += (lo <= s_vec[1]) & vfp_one;
                r_vec[1] += (hi >  s_vec[1]) & vfp_one;
            }
            break;

        case axis_t::y_axis:
            for (int i = _ping_idx; i < static_cast<int>(_ping_idx + _nr_prims); ++i)
            {
                const vfp_t lo((*_ping)[i].low.y);
                const vfp_t hi((*_ping)[i].high.y);
                l_vec[0] += (lo <= s_vec[0]) & vfp_one;
                r_vec[0] += (hi >  s_vec[0]) & vfp_one;
                l_vec[1] += (lo <= s_vec[1]) & vfp_one;
                r_vec[1] += (hi >  s_vec[1]) & vfp_one;
            }
            break;

        case axis_t::z_axis:
            for (int i = _ping_idx; i < static_cast<int>(_ping_idx + _nr_prims); ++i)
            {
                const vfp_t lo((*_ping)[i].low.z);
                const vfp_t hi((*_ping)[i].high.z);
                l_vec[0] += (lo <= s_vec[0]) & vfp_one;
                r_vec[0] += (hi >  s_vec[0]) & vfp_one;
                l_vec[1] += (lo <= s_vec[1]) & vfp_one;
                r_vec[1] += (hi >  s_vec[1]) & vfp_one;
            }
            break;
        default :
            assert(false);
            break;
    }

    /* Save results */
    l_vec[0].store(&l[0]);
    l_vec[1].store(&l[SIMD_WIDTH]);

    r_vec[0].store(&r[0]);
    r_vec[1].store(&r[SIMD_WIDTH]);
}

void voxel::count_primitives(vfp_t *const l, vfp_t *const r, const vfp_t *const s, const axis_t n) const
{
    l[0] = vfp_zero;
    l[1] = vfp_zero;
    r[0] = vfp_zero;
    r[1] = vfp_zero;

    /* Count in the given axis */
    switch (n)
    {
        case axis_t::x_axis:
            for (int i = _ping_idx; i < static_cast<int>(_ping_idx + _nr_prims); ++i)
            {
                const vfp_t lo((*_ping)[i].low.x);
                const vfp_t hi((*_ping)[i].high.x);
                l[0] += (lo <= s[0]) & vfp_one;
                r[0] += (hi >  s[0]) & vfp_one;
                l[1] += (lo <= s[1]) & vfp_one;
                r[1] += (hi >  s[1]) & vfp_one;
            }
            break;

        case axis_t::y_axis:
            for (int i = _ping_idx; i < static_cast<int>(_ping_idx + _nr_prims); ++i)
            {
                const vfp_t lo((*_ping)[i].low.y);
                const vfp_t hi((*_ping)[i].high.y);
                l[0] += (lo <= s[0]) & vfp_one;
                r[0] += (hi >  s[0]) & vfp_one;
                l[1] += (lo <= s[1]) & vfp_one;
                r[1] += (hi >  s[1]) & vfp_one;
            }
            break;

        case axis_t::z_axis:
            for (int i = _ping_idx; i < static_cast<int>(_ping_idx + _nr_prims); ++i)
            {
                const vfp_t lo((*_ping)[i].low.z);
                const vfp_t hi((*_ping)[i].high.z);
                l[0] += (lo <= s[0]) & vfp_one;
                r[0] += (hi >  s[0]) & vfp_one;
                l[1] += (lo <= s[1]) & vfp_one;
                r[1] += (hi >  s[1]) & vfp_one;
            }
            break;
        default :
            assert(false);
            break;
    }
}

vfp_t voxel::calculate_sah_cost(const vfp_t &l, const vfp_t &r, const vfp_t &s, const axis_t normal) const
{
    /* Surface area of sub-divides */
    vfp_t left_area, right_area;
    const vfp_t x_dist(_t.x - _b.x);
    const vfp_t y_dist(_t.y - _b.y);
    const vfp_t z_dist(_t.z - _b.z);
    const vfp_t sa((x_dist * y_dist) + (x_dist * z_dist) + (y_dist * z_dist));
    switch (normal)
    {
        case axis_t::x_axis:
        {
            const vfp_t xl_dist(s - vfp_t(_b.x));
            left_area  = (xl_dist * y_dist) + (xl_dist * z_dist) + (z_dist * y_dist);

            const vfp_t xr_dist(vfp_t(_t.x) - s);
            right_area = (xr_dist * y_dist) + (xr_dist * z_dist) + (z_dist * y_dist);
            break;
        }
        case axis_t::y_axis:
        {
            const vfp_t yl_dist(s - vfp_t(_b.y));
            left_area  = (x_dist * yl_dist) + (x_dist * z_dist) + (z_dist * yl_dist);

            const vfp_t yr_dist(vfp_t(_t.y) - s);
            right_area = (x_dist * yr_dist) + (x_dist * z_dist) + (z_dist * yr_dist);
            break;
        }
        case axis_t::z_axis:
        {
            const vfp_t zl_dist(s - vfp_t(_b.z));
            left_area  = (x_dist * y_dist) + (x_dist * zl_dist) + (zl_dist * y_dist);

            const vfp_t zr_dist(vfp_t(_t.z) - s);
            right_area = (x_dist * y_dist) + (x_dist * zr_dist) + (zr_dist * y_dist);
            break;
        }
        default:
            assert(false);
            break;
    }

    /* Calculate the surface area heuristic metric */
    /* cost of traversal + cost of intersection * (left cell count * left area + right cell count * right area) */
    return vfp_t(COST_OF_TRAVERSAL) + (vfp_t(COST_OF_INTERSECTION) * (((l * left_area) + (r * right_area)) / sa));
}
}; /* namespace raptor_raytracer */

// float voxel::split_all_axis(float *s, axis_t *normal)
// {
//     /* Find the best split position of the primitives */
//     axis_t   best_axis   = axis_t::not_set;
//     float     best_split  = axis_t::not_set;
//     float     lowest_cost = *s;
//  
//      /* Initial primitive split */
//     unsigned size_of_prim = this->_nr_prims;
//     float     right = size_of_prim;
//     float     left  = 0;
//     float     guess;
//     float     last_guess = MAX_DIST;
//  
//     /* Elements for evaluation */
//     float lo_x, lo_y, lo_z, hi_x, hi_y, hi_z;
//  
//     /* Allocated arrays to hold the bounding box points */    
//     if (low_points == NULL)
//     {
//         low_points  = new float [size_of_prim];
//         high_points = new float [size_of_prim];
//     }
//  
//     unsigned index=0;
//     for (primitive_list::iterator i=this->p->begin(); i!= this->p->end(); ++i)
//     {
//         low_points [index  ] = (*i)->lowest_x();
//         high_points[index++] = (*i)->highest_x();
//     }
//  
//     quick_sort(low_points,  0, (size_of_prim-1));
//     quick_sort(high_points, 0, (size_of_prim-1));
//  
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
//  
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
//      
//         if (high_index == size_of_prim)
//         {
//             hi_x = MAX_DIST;
//         }
//         else
//         {
//             hi_x = high_points[high_index];
//         }
// 
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
// 
//         /* Break when any further points will be out of bounds */
//         if (guess >= this->t.x)
//         {
//             break;
//         }
//      
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
// 
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
// 
//     right = size_of_prim;
//     left  = 0;
//     last_guess = MAX_DIST;
//     index=0;
//     for (primitive_list::iterator i=this->p->begin(); i!= this->p->end(); ++i)
//     {
//         low_points [index  ] = (*i)->lowest_y();
//         high_points[index++] = (*i)->highest_y();
//     }
//  
//     quick_sort(low_points,  0, (size_of_prim-1));
//     quick_sort(high_points, 0, (size_of_prim-1));
//  
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
//  
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
//      
//         if (high_index == size_of_prim)
//         {
//             hi_y = MAX_DIST;
//         }
//         else
//         {
//             hi_y = high_points[high_index];
//         }
// 
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
// 
//         /* Break when any further points will be out of bounds */
//         if (guess >= this->t.y)
//         {
//             break;
//         }
//      
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
// 
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
// 
//     right = size_of_prim;
//     left  = 0;
//     last_guess = MAX_DIST;
//     index=0;
//     for (primitive_list::iterator i=this->p->begin(); i!= this->p->end(); ++i)
//     {
//         low_points [index  ] = (*i)->lowest_z();
//         high_points[index++] = (*i)->highest_z();
//     }
//  
//     quick_sort(low_points,  0, (size_of_prim-1));
//     quick_sort(high_points, 0, (size_of_prim-1));
//  
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
//  
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
//      
//         if (high_index == size_of_prim)
//         {
//             hi_z = MAX_DIST;
//         }
//         else
//         {
//             hi_z = high_points[high_index];
//         }
// 
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
//      
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
// 
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
//  
//     *normal = best_axis;
//     *s = lowest_cost;
//     return best_split;
// }
