#include "bih_builder.h"
#include <chrono>


namespace raptor_raytracer
{
/* Tree depth */
static unsigned depth = 0;


/**********************************************************
  swap swaps the elements a and b of the vector o. 
**********************************************************/
inline void swap(std::vector<triangle *> *const o, const int a, const int b)
{
    triangle *s = (*o)[a];
    (*o)[a]     = (*o)[b];
    (*o)[b]     = s;
}


/**********************************************************
 
**********************************************************/
void divide_bih_node(std::vector<triangle *> *const o, unsigned cur, unsigned *child, point_t bl, point_t tr, const point_t &node_bl, const point_t &node_tr, int b, int e)
{
    /* Depth check */
    ++depth;
    assert(depth < MAX_BIH_STACK_HEIGHT);
    assert((unsigned)e < o->size());
    assert(((unsigned)b < o->size()) || (b > e));
    
    /* Pick longest side to divide in */
    point_t bm;
    point_t tm;
    float split_pnt;
    axis_t split_axis   = axis_t::not_set;
    const float dx      = tr.x - bl.x;
    const float dy      = tr.y - bl.y;
    const float dz      = tr.z - bl.z;
    if (dx > dy)
    {
        if (dx > dz)
        {
            split_axis = axis_t::x_axis;
            split_pnt  = bl.x + (dx * 0.5f);
            bm         = point_t(split_pnt, bl.y, bl.z);
            tm         = point_t(split_pnt, tr.y, tr.z);
        }
        else
        {
            split_axis = axis_t::z_axis;
            split_pnt  = bl.z + (dz * 0.5f);
            bm         = point_t(bl.x, bl.y, split_pnt);
            tm         = point_t(tr.x, tr.y, split_pnt);
        }
    }
    else
    {
        if (dy > dz)
        {
            split_axis = axis_t::y_axis;
            split_pnt  = bl.y + (dy * 0.5f);
            bm         = point_t(bl.x, split_pnt, bl.z);
            tm         = point_t(tr.x, split_pnt, tr.z);
        }
        else
        {
            split_axis = axis_t::z_axis;
            split_pnt  = bl.z + (dz * 0.5f);
            bm         = point_t(bl.x, bl.y, split_pnt);
            tm         = point_t(tr.x, tr.y, split_pnt);
        }
    }
    
    /* Partition primitives */
    int left_size;
    int right_size;
    point_t node_tm(node_tr);
    point_t node_bm(node_bl);
    float max_left  = -MAX_DIST;
    float min_right =  MAX_DIST;
    int top         = e;
    int bottom      = b;
    switch (split_axis)
    {
        case axis_t::x_axis :
        {
            while(bottom < top)
            {
                while (split_pnt < (((*o)[top]->highest_x() + (*o)[top]->lowest_x()) * 0.5f) && top > bottom)
                {
                    min_right = min(min_right, (*o)[top--]->lowest_x());
                }
                swap(o, bottom, top);
                
                while(split_pnt >= (((*o)[bottom]->highest_x() + (*o)[bottom]->lowest_x()) * 0.5f) && bottom < top)
                {
                    max_left = max(max_left, (*o)[bottom++]->highest_x());
                }
                swap(o, bottom, top);
            }             

            /* Parition the last primitive */
            if (split_pnt >= (((*o)[bottom]->highest_x() + (*o)[bottom]->lowest_x()) * 0.5f))
            {
                max_left = max(max_left, (*o)[bottom++]->highest_x());
            }
            else
            {
                min_right = min(min_right, (*o)[top]->lowest_x());
            }

            /* Reduce the bouding boxes to just fit the new split */
            float pos = (tm.x - bl.x) * 0.5f;
            while (((bl.x + pos) > max_left) && (max_left > -MAX_DIST))
            {
                tm.x = bl.x + pos;
                pos *= 0.5f;
            }

            pos = (tr.x - bm.x) * 0.5f;
            while (((tr.x - pos) < min_right) && (min_right < MAX_DIST))
            {
                bm.x = tr.x - pos;
                pos *= 0.5f;
            }

            right_size = e - bottom; /* 1 less than the actual node size */
            if ((right_size == -1) && (max_left == node_tm.x))
            {
                // BOOST_LOG_TRIVIAL(trace) << "Blank node recursing left: " << max_left << ", " << node_tm.x;
                const point_t width(tm - bl);
                if ((depth + 1) == MAX_BIH_STACK_HEIGHT)
                {
                    bih_node::get_node_array(cur).create_leaf_node(b, max(0, bottom - 1));
                }
                else
                {
                    divide_bih_node(o, cur, child, bl, tm, node_bl, node_tr, b, e);
                }
                --depth;
                return;
            }

            left_size = bottom - b;
            if ((left_size == 0) && (min_right == node_bm.x))
            {
                // BOOST_LOG_TRIVIAL(trace) << "Blank node recursing right: " << min_right << ", " << node_bm.x;
                const point_t width(tr - bm);
                if ((depth + 1) == MAX_BIH_STACK_HEIGHT)
                {
                    bih_node::get_node_array(cur).create_leaf_node(bottom, e);
                }
                else
                {
                    divide_bih_node(o, cur, child, bm, tr, node_bl, node_tr, b, e);
                }
                --depth;
                return;
            }

            node_tm.x = max_left;
            node_bm.x = min_right;
            break;
        }
        case axis_t::y_axis :
        {
            while(bottom < top)
            {
                while(split_pnt < (((*o)[top]->highest_y() + (*o)[top]->lowest_y()) * 0.5f) && top > bottom)
                {
                    min_right = min(min_right, (*o)[top--]->lowest_y());
                }
                swap(o, bottom, top);
         
                while(split_pnt >= (((*o)[bottom]->highest_y() + (*o)[bottom]->lowest_y()) * 0.5f) && bottom < top)
                {
                    max_left = max(max_left, (*o)[bottom++]->highest_y());
                }
                swap(o, bottom, top);
            }

            /* Parition the last primitive */
            if (split_pnt >= (((*o)[bottom]->highest_y() + (*o)[bottom]->lowest_y()) * 0.5f))
            {
                max_left = max(max_left, (*o)[bottom++]->highest_y());
            }
            else
            {
                min_right = min(min_right, (*o)[top]->lowest_y());
            }

            /* Reduce the bouding boxes to just fit the new split */
            float pos = (tm.y - bl.y) * 0.5f;
            while (((bl.y + pos) > max_left) && (max_left > -MAX_DIST))
            {
                tm.y = bl.y + pos;
                pos *= 0.5f;
            }

            pos = (tr.y - bm.y) * 0.5f;
            while (((tr.y - pos) < min_right) && (min_right < MAX_DIST))
            {
                bm.y = tr.y - pos;
                pos *= 0.5f;
            }

            right_size = e - bottom; /* 1 less than the actual node size */
            if ((right_size == -1) && (max_left == node_tm.y))
            {
                // BOOST_LOG_TRIVIAL(trace) << "Blank node recursing left: " << max_left << ", " << node_tm.y;
                const point_t width(tm - bl);
                if ((depth + 1) == MAX_BIH_STACK_HEIGHT)
                {
                    bih_node::get_node_array(cur).create_leaf_node(b, max(0, bottom - 1));
                }
                else
                {
                    divide_bih_node(o, cur, child, bl, tm, node_bl, node_tr, b, e);
                }
                --depth;
                return;
            }

            left_size = bottom - b;
            if ((left_size == 0) && (min_right == node_bm.y))
            {
                // BOOST_LOG_TRIVIAL(trace) << "Blank node recursing right: " << min_right << ", " << node_bm.y;
                const point_t width(tr - bm);
                if ((depth + 1) == MAX_BIH_STACK_HEIGHT)
                {
                    bih_node::get_node_array(cur).create_leaf_node(bottom, e);
                }
                else
                {
                    divide_bih_node(o, cur, child, bm, tr, node_bl, node_tr, b, e);
                }
                --depth;
                return;
            }

            node_tm.y = max_left;
            node_bm.y = min_right;
            break;
        }
        case axis_t::z_axis :
        {
            while(bottom < top)
            {
                while(split_pnt < (((*o)[top]->highest_z() + (*o)[top]->lowest_z()) * 0.5f) && top > bottom)
                {
                    min_right = min(min_right, (*o)[top--]->lowest_z());
                }
                swap(o, bottom, top);
         
                while(split_pnt >= (((*o)[bottom]->highest_z() + (*o)[bottom]->lowest_z()) * 0.5f) && bottom < top)
                {
                    max_left = max(max_left, (*o)[bottom++]->highest_z());
                }
                swap(o, bottom, top);
            }

            /* Parition the last primitive */
            if (split_pnt >= (((*o)[bottom]->highest_z() + (*o)[bottom]->lowest_z()) * 0.5f))
            {
                max_left = max(max_left, (*o)[bottom++]->highest_z());
            }
            else
            {
                min_right = min(min_right, (*o)[top]->lowest_z());
            }
        
            /* Reduce the bouding boxes to just fit the new split */
            float pos = (tm.z - bl.z) * 0.5f;
            while (((bl.z + pos) > max_left) && (max_left > -MAX_DIST))
            {
                tm.z = bl.z + pos;
                pos *= 0.5f;
            }

            pos = (tr.z - bm.z) * 0.5f;
            while (((tr.z - pos) < min_right) && (min_right < MAX_DIST))
            {
                bm.z = tr.z - pos;
                pos *= 0.5f;
            }

            right_size = e - bottom; /* 1 less than the actual node size */
            if ((right_size == -1) && (max_left == node_tm.z))
            {
                // BOOST_LOG_TRIVIAL(trace) << "Blank node recursing left: " << max_left << ", " << node_tm.z;
                const point_t width(tm - bl);
                if ((depth + 1) == MAX_BIH_STACK_HEIGHT)
                {
                    bih_node::get_node_array(cur).create_leaf_node(b, max(0, bottom - 1));
                }
                else
                {
                    divide_bih_node(o, cur, child, bl, tm, node_bl, node_tr, b, e);
                }
                --depth;
                return;
            }

            left_size = bottom - b;
            if ((left_size == 0) && (min_right == node_bm.z))
            {
                // BOOST_LOG_TRIVIAL(trace) << "Blank node recursing right: " << min_right << ", " << node_bm.z;
                const point_t width(tr - bm);
                if ((depth + 1) == MAX_BIH_STACK_HEIGHT)
                {
                    bih_node::get_node_array(cur).create_leaf_node(bottom, e);
                }
                else
                {
                    divide_bih_node(o, cur, child, bm, tr, node_bl, node_tr, b, e);
                }
                --depth;
                return;
            }

            node_tm.z = max_left;
            node_bm.z = min_right;
            break;
        }
        default :
            assert(false);
    }

    /* Construct the BIH nodes */
    bih_node::get_node_array(cur).create_generic_node(*child, max_left, min_right, split_axis);
#ifdef SPATIAL_SUBDIVISION_STATISTICS
    ++ng;
#endif

    /* Recurse */
    unsigned left_child  = *child;
    unsigned right_child = *child + 1;

    /* Recurse left */
    int node_size = bottom - b;
    if ((node_size < (MAX_BIH_NODE_SIZE + 1)) || ((depth + 1) == MAX_BIH_STACK_HEIGHT))
    {
        bih_node::get_node_array(left_child).create_leaf_node(b, max(0, bottom - 1));
#ifdef SPATIAL_SUBDIVISION_STATISTICS
        ++ne;
        nee       += (node_size < 2);
        ave_ob    +=  node_size;
        max_depth  = max(max_depth, depth + 1);
        ner        = max(ner, (unsigned)node_size);
#endif /* #ifdef SPATIAL_SUBDIVISION_STATISTICS */
    }
    else
    {
        (*child) += 2;
        divide_bih_node(o, left_child, child, bl, tm, node_bl, node_tm, b, bottom - 1);
    }

    /* Recurse right */
    node_size = e - bottom;
    if ((node_size < MAX_BIH_NODE_SIZE) || ((depth + 1) == MAX_BIH_STACK_HEIGHT))
    {
        bih_node::get_node_array(right_child).create_leaf_node(bottom, e);
#ifdef SPATIAL_SUBDIVISION_STATISTICS
        ++ne;
        nee       += (node_size < 1);
        ave_ob    +=  node_size + 1;
        max_depth  = max(max_depth, depth + 1);
        ner        = max(ner, (unsigned)node_size + 1);
#endif /* #ifdef SPATIAL_SUBDIVISION_STATISTICS */
    }
    else
    {
        (*child) += 2;
        divide_bih_node(o, right_child, child, bm, tr, node_bm, node_tr, bottom, e);
    }
    
    depth--;
    return;
}


/**********************************************************
 
**********************************************************/
const std::vector<triangle *> * build_bih(const primitive_list *const o, std::vector<bih_node> *const bih)
{
//    cout << "BIH construction has begun" << endl;
    auto t0(std::chrono::system_clock::now());

    /* Bound all primitives in the scene and copy them */
    std::vector<triangle *> *object_copy = new std::vector<triangle *>;
    *object_copy = *o;

    /* Divide the nodes */
    bih_node::set_primitives(object_copy);
    bih_node::set_node_array(bih);
    unsigned grand_child = 1;

    depth = 0;
    divide_bih_node(object_copy, 0, &grand_child, triangle::get_scene_lower_bounds(), triangle::get_scene_upper_bounds(), triangle::get_scene_lower_bounds(), triangle::get_scene_upper_bounds(), 0, object_copy->size() - 1);

//    approximate_sort(o, object_copy);
    
    assert(depth == 0);
    auto t1(std::chrono::system_clock::now());
    std::cout << "builder took: " << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count() << std::endl;
    return object_copy;
}
}; /* namespace raptor_raytracer */
