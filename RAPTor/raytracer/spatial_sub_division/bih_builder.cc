/* Standard headers */

/* Boost headers */

/* Common headers */
#include "logging.h"

/* Ray tracer headers */
#include "bih_builder.h"


namespace raptor_raytracer
{
/* Tree depth */
static int depth = 0;

/* Find a split position and divide the bih node in 2 */
void divide_bih_node(std::vector<triangle *> *const o, unsigned cur, unsigned *child, point_t bl, point_t tr, int b, int e, const int max_node_size)
{
    /* Depth check */
    ++depth;
    assert(depth < MAX_BIH_STACK_HEIGHT);
    assert(static_cast<unsigned int>(e) < o->size());
    assert((static_cast<unsigned int>(b) < o->size()) || (b > e));
    
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
    float max_left  = -MAX_DIST;
    float min_right =  MAX_DIST;
    int top         = e;
    int bottom      = b;
    switch (split_axis)
    {
        case axis_t::x_axis :
            while (bottom < top)
            {
                while (split_pnt < (((*o)[top]->highest_x() + (*o)[top]->lowest_x()) * 0.5f) && top > bottom)
                {
                    min_right = std::min(min_right, (*o)[top--]->lowest_x());
                }
                std::swap((*o)[bottom], (*o)[top]);
                
                while(split_pnt >= (((*o)[bottom]->highest_x() + (*o)[bottom]->lowest_x()) * 0.5f) && bottom < top)
                {
                    max_left = std::max(max_left, (*o)[bottom++]->highest_x());
                }
                std::swap((*o)[bottom], (*o)[top]);
            }             

            /* Parition the last primitive */
            if (split_pnt >= (((*o)[bottom]->highest_x() + (*o)[bottom]->lowest_x()) * 0.5f))
            {
                max_left = std::max(max_left, (*o)[bottom++]->highest_x());
            }
            else
            {
                min_right = std::min(min_right, (*o)[top]->lowest_x());
            }
            break;
    
        case axis_t::y_axis :
            while(bottom < top)
            {
                while(split_pnt < (((*o)[top]->highest_y() + (*o)[top]->lowest_y()) * 0.5f) && top > bottom)
                {
                    min_right = std::min(min_right, (*o)[top--]->lowest_y());
                }
                std::swap((*o)[bottom], (*o)[top]);
         
                while(split_pnt >= (((*o)[bottom]->highest_y() + (*o)[bottom]->lowest_y()) * 0.5f) && bottom < top)
                {
                    max_left = std::max(max_left, (*o)[bottom++]->highest_y());
                }
                std::swap((*o)[bottom], (*o)[top]);
            }

            /* Parition the last primitive */
            if (split_pnt >= (((*o)[bottom]->highest_y() + (*o)[bottom]->lowest_y()) * 0.5f))
            {
                max_left = std::max(max_left, (*o)[bottom++]->highest_y());
            }
            else
            {
                min_right = std::min(min_right, (*o)[top]->lowest_y());
            }
            break;
    
         case axis_t::z_axis :
            while(bottom < top)
            {
                while(split_pnt < (((*o)[top]->highest_z() + (*o)[top]->lowest_z()) * 0.5f) && top > bottom)
                {
                    min_right = std::min(min_right, (*o)[top--]->lowest_z());
                }
                std::swap((*o)[bottom], (*o)[top]);
         
                while(split_pnt >= (((*o)[bottom]->highest_z() + (*o)[bottom]->lowest_z()) * 0.5f) && bottom < top)
                {
                    max_left = std::max(max_left, (*o)[bottom++]->highest_z());
                }
                std::swap((*o)[bottom], (*o)[top]);
            }

            /* Parition the last primitive */
            if (split_pnt >= (((*o)[bottom]->highest_z() + (*o)[bottom]->lowest_z()) * 0.5f))
            {
                max_left = std::max(max_left, (*o)[bottom++]->highest_z());
            }
            else
            {
                min_right = std::min(min_right, (*o)[top]->lowest_z());
            }
            break;

        default :
            assert(false);
    }

    /* Construct the BIH nodes */
    // BOOST_LOG_TRIVIAL(trace) << "Creating internal node at index: " << cur << " with splits at: " << max_left << ", " << min_right << " in: " << static_cast<int>(split_axis);
    bih_node::get_node_array(cur).create_generic_node(*child, max_left, min_right, split_axis);

    /* Reduce the bouding boxes to just fit the new split */
    float pos;
    switch (split_axis)
    {
        case axis_t::x_axis :
            pos = (tm.x - bl.x) * 0.5f;
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
            break;

        case axis_t::y_axis :
            pos = (tm.y - bl.y) * 0.5f;
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
            break;

        case axis_t::z_axis :
            pos = (tm.z - bl.z) * 0.5f;
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
            break;

        default :
            assert(false);
   }
  
    /* Recurse */
    unsigned left_child  = *child;
    unsigned right_child = *child + 1;

    /* Recurse left */
    int node_size = bottom - b;
    if ((node_size <= max_node_size) || ((depth + 1) == MAX_BIH_STACK_HEIGHT))
    {
        // BOOST_LOG_TRIVIAL(trace) << "Creating leaf for left child at index: " << left_child;
        bih_node::get_node_array(left_child).create_leaf_node(b, max(0, bottom - 1));
    }
    else
    {
        (*child) += 2;
        // BOOST_LOG_TRIVIAL(trace) << "Recursing for left child";
        divide_bih_node(o, left_child, child, bl, tm, b, bottom - 1, max_node_size);
    }

    /* Recurse right */
    node_size = e - bottom; /* 1 less than the actual node size */
    if ((node_size < max_node_size) || ((depth + 1) == MAX_BIH_STACK_HEIGHT))
    {
        // BOOST_LOG_TRIVIAL(trace) << "Creating leaf for right child at index: " << right_child;
        bih_node::get_node_array(right_child).create_leaf_node(bottom, e);
    }
    else
    {
        (*child) += 2;
        // BOOST_LOG_TRIVIAL(trace) << "Recursing for right child";
        divide_bih_node(o, right_child, child, bm, tr, bottom, e, max_node_size);
    }
    
    --depth;
    return;
}

/* Build a Bounding Interval Heirarchy for the primitives in o */
const std::vector<bih_node> * build_bih(primitive_list *const o, const int max_node_size)
{
    // BOOST_LOG_TRIVIAL(trace) << "BIH construction has begun";

    /* Maximum theoretical size is everything.size() * 6, but this is very unlikely */
    bih_node::set_primitives(o);
    auto ret = bih_node::resize_node_array(o->size() * 3);

    /* Check if we have anything to do */
    if (o->size() <= static_cast<unsigned int>(max_node_size))
    {
        /* Create one leaf node that hold the whole scene */
        bih_node::get_node_array(0).create_leaf_node(0, o->size() - 1);
    }
    else
    {
        /* Divide the nodes */
        depth = 0;
        unsigned grand_child = 1;
        divide_bih_node(o, 0, &grand_child, triangle::get_scene_lower_bounds(), triangle::get_scene_upper_bounds(), 0, o->size() - 1, max_node_size);
        assert(depth == 0);
    }
    
    return ret;
}
}; /* namespace raptor_raytracer */
