#include "bih_builder.h"


namespace raptor_raytracer
{
/* Tree depth */
static unsigned depth = 0;


/**********************************************************
  swap swaps the elements a and b of the vector o. 
**********************************************************/
inline void swap(vector<triangle *> *const o, const int a, const int b)
{
    triangle *s = (*o)[a];
    (*o)[a]     = (*o)[b];
    (*o)[b]     = s;
}


inline void swap(bih_bucket *const o, const int a, const int b)
{
     bih_bucket s   = o[a];
    o[a]            = o[b];
    o[b]            = s;
}


/**********************************************************
 
**********************************************************/
void divide_bih_node(vector<triangle *> *const o, unsigned cur, unsigned *child, point_t bl, point_t tr, int b, int e)
{
    /* Depth check */
    ++depth;
    assert(depth < MAX_BIH_STACK_HEIGHT);
    assert((unsigned)e < o->size());
    assert(((unsigned)b < o->size()) || (b > e));
    
    /* Pick longest side to divide in */
    point_t bm;
    point_t tm;
    fp_t split_pnt;
    axis_t split_axis   = axis_t::not_set;
    const fp_t dx       = tr.x - bl.x;
    const fp_t dy       = tr.y - bl.y;
    const fp_t dz       = tr.z - bl.z;
    if (dx > dy)
    {
        if (dx > dz)
        {
            split_axis = axis_t::x_axis;
            split_pnt  = bl.x + (dx * (fp_t)0.5);
            bm         = point_t(split_pnt, bl.y, bl.z);
            tm         = point_t(split_pnt, tr.y, tr.z);
        }
        else
        {
            split_axis = axis_t::z_axis;
            split_pnt  = bl.z + (dz * (fp_t)0.5);
            bm         = point_t(bl.x, bl.y, split_pnt);
            tm         = point_t(tr.x, tr.y, split_pnt);
        }
    }
    else
    {
        if (dy > dz)
        {
            split_axis = axis_t::y_axis;
            split_pnt  = bl.y + (dy * (fp_t)0.5);
            bm         = point_t(bl.x, split_pnt, bl.z);
            tm         = point_t(tr.x, split_pnt, tr.z);
        }
        else
        {
            split_axis = axis_t::z_axis;
            split_pnt  = bl.z + (dz * (fp_t)0.5);
            bm         = point_t(bl.x, bl.y, split_pnt);
            tm         = point_t(tr.x, tr.y, split_pnt);
        }
    }
    
    /* Partition primitives */
    fp_t max_left   = -MAX_DIST;
    fp_t min_right  =  MAX_DIST;
    int top         = e;
    int bottom      = b;
    switch (split_axis)
    {
        case axis_t::x_axis :
            while(bottom < top)
            {
                while (split_pnt < (((*o)[top]->highest_x() + (*o)[top]->lowest_x()) * (fp_t)0.5) && top > bottom)
                {
                    min_right = min(min_right, (*o)[top--]->lowest_x());
                }
                swap(o, bottom, top);
                
                while(split_pnt >= (((*o)[bottom]->highest_x() + (*o)[bottom]->lowest_x()) * (fp_t)0.5) && bottom < top)
                {
                    max_left = max(max_left, (*o)[bottom++]->highest_x());
                }
                swap(o, bottom, top);
            }             

            /* Parition the last primitive */
            if (split_pnt >= (((*o)[bottom]->highest_x() + (*o)[bottom]->lowest_x()) * (fp_t)0.5))
            {
                max_left = max(max_left, (*o)[bottom++]->highest_x());
            }
            else
            {
                min_right = min(min_right, (*o)[top]->lowest_x());
            }
            break;
    
        case axis_t::y_axis :
            while(bottom < top)
            {
                while(split_pnt < (((*o)[top]->highest_y() + (*o)[top]->lowest_y()) * (fp_t)0.5) && top > bottom)
                {
                    min_right = min(min_right, (*o)[top--]->lowest_y());
                }
                swap(o, bottom, top);
         
                while(split_pnt >= (((*o)[bottom]->highest_y() + (*o)[bottom]->lowest_y()) * (fp_t)0.5) && bottom < top)
                {
                    max_left = max(max_left, (*o)[bottom++]->highest_y());
                }
                swap(o, bottom, top);
            }

            /* Parition the last primitive */
            if (split_pnt >= (((*o)[bottom]->highest_y() + (*o)[bottom]->lowest_y()) * (fp_t)0.5))
            {
                max_left = max(max_left, (*o)[bottom++]->highest_y());
            }
            else
            {
                min_right = min(min_right, (*o)[top]->lowest_y());
            }
            break;
    
         case axis_t::z_axis :
            while(bottom < top)
            {
                while(split_pnt < (((*o)[top]->highest_z() + (*o)[top]->lowest_z()) * (fp_t)0.5) && top > bottom)
                {
                    min_right = min(min_right, (*o)[top--]->lowest_z());
                }
                swap(o, bottom, top);
         
                while(split_pnt >= (((*o)[bottom]->highest_z() + (*o)[bottom]->lowest_z()) * (fp_t)0.5) && bottom < top)
                {
                    max_left = max(max_left, (*o)[bottom++]->highest_z());
                }
                swap(o, bottom, top);
            }

            /* Parition the last primitive */
            if (split_pnt >= (((*o)[bottom]->highest_z() + (*o)[bottom]->lowest_z()) * (fp_t)0.5))
            {
                max_left = max(max_left, (*o)[bottom++]->highest_z());
            }
            else
            {
                min_right = min(min_right, (*o)[top]->lowest_z());
            }
            break;
    
        default :
            assert(false);
    }

    /* Construct the BIH nodes */
    bih_node::get_node_array(cur).create_generic_node(*child, max_left, min_right, split_axis);
#ifdef SPATIAL_SUBDIVISION_STATISTICS
    ++ng;
#endif

    /* Reduce the bouding boxes to just fit the new split */
    fp_t pos;
    switch (split_axis)
    {
        case axis_t::x_axis :
            pos = (tm.x - bl.x) * (fp_t)0.5;
            while (((bl.x + pos) > max_left) && (max_left > -MAX_DIST))
            {
                tm.x = bl.x + pos;
                pos *= (fp_t)0.5;
            }

            pos = (tr.x - bm.x) * (fp_t)0.5;
            while (((tr.x - pos) < min_right) && (min_right < MAX_DIST))
            {
                bm.x = tr.x - pos;
                pos *= (fp_t)0.5;
            }
            break;

        case axis_t::y_axis :
            pos = (tm.y - bl.y) * (fp_t)0.5;
            while (((bl.y + pos) > max_left) && (max_left > -MAX_DIST))
            {
                tm.y = bl.y + pos;
                pos *= (fp_t)0.5;
            }

            pos = (tr.y - bm.y) * (fp_t)0.5;
            while (((tr.y - pos) < min_right) && (min_right < MAX_DIST))
            {
                bm.y = tr.y - pos;
                pos *= (fp_t)0.5;
            }
            break;

        case axis_t::z_axis :
            pos = (tm.z - bl.z) * (fp_t)0.5;
            while (((bl.z + pos) > max_left) && (max_left > -MAX_DIST))
            {
                tm.z = bl.z + pos;
                pos *= (fp_t)0.5;
            }

            pos = (tr.z - bm.z) * (fp_t)0.5;
            while (((tr.z - pos) < min_right) && (min_right < MAX_DIST))
            {
                bm.z = tr.z - pos;
                pos *= (fp_t)0.5;
            }
            break;

        default :
            assert(false);
            break;
   }
  
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
        divide_bih_node(o, left_child, child, bl, tm, b, bottom - 1);
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
        divide_bih_node(o, right_child, child, bm, tr, bottom, e);
    }
    
    depth--;
    return;
}


/**********************************************************
 
**********************************************************/
void divide_bih_node(bih_bucket *const o, vector<triangle *> *const p, unsigned cur, unsigned *child, point_t bl, point_t tr, int b, int e)
{
    /* Depth check */
    ++depth;
    assert(depth < MAX_BIH_STACK_HEIGHT);
    
    /* Pick longest side to divide in */
    point_t bm;
    point_t tm;
    fp_t split_pnt;
    axis_t split_axis   = axis_t::not_set;
    const fp_t dx       = tr.x - bl.x;
    const fp_t dy       = tr.y - bl.y;
    const fp_t dz       = tr.z - bl.z;
    if (dx > dy)
    {
        if (dx > dz)
        {
            split_axis = axis_t::x_axis;
            split_pnt  = bl.x + (dx * (fp_t)0.5);
            bm         = point_t(split_pnt, bl.y, bl.z);
            tm         = point_t(split_pnt, tr.y, tr.z);
        }
        else
        {
            split_axis = axis_t::z_axis;
            split_pnt  = bl.z + (dz * (fp_t)0.5);
            bm         = point_t(bl.x, bl.y, split_pnt);
            tm         = point_t(tr.x, tr.y, split_pnt);
        }
    }
    else
    {
        if (dy > dz)
        {
            split_axis = axis_t::y_axis;
            split_pnt  = bl.y + (dy * (fp_t)0.5);
            bm         = point_t(bl.x, split_pnt, bl.z);
            tm         = point_t(tr.x, split_pnt, tr.z);
        }
        else
        {
            split_axis = axis_t::z_axis;
            split_pnt  = bl.z + (dz * (fp_t)0.5);
            bm         = point_t(bl.x, bl.y, split_pnt);
            tm         = point_t(tr.x, tr.y, split_pnt);
        }
    }
    
    /* Partition primitives */
    fp_t max_left   = -MAX_DIST;
    fp_t min_right  =  MAX_DIST;
    int top         = e;
    int bottom      = b;
    switch (split_axis)
    {
        case axis_t::x_axis :
            while(bottom < top)
            {
                while (split_pnt < ((o[top].highest_x() + o[top].lowest_x()) * (fp_t)0.5) && top > bottom)
                {
                    min_right = min(min_right, o[top--].lowest_x());
                }
                swap(o, bottom, top);
                
                while(split_pnt >= ((o[bottom].highest_x() + o[bottom].lowest_x()) * (fp_t)0.5) && bottom < top)
                {
                    max_left = max(max_left, o[bottom++].highest_x());
                }
                swap(o, bottom, top);
            }             

            /* Parition the last primitive */
            if (split_pnt >= ((o[bottom].highest_x() + o[bottom].lowest_x()) * (fp_t)0.5))
            {
                max_left = max(max_left, o[bottom++].highest_x());
            }
            else
            {
                min_right = min(min_right, o[top].lowest_x());
            }
            break;
    
        case axis_t::y_axis :
            while(bottom < top)
            {
                while(split_pnt < ((o[top].highest_y() + o[top].lowest_y()) * (fp_t)0.5) && top > bottom)
                {
                    min_right = min(min_right, o[top--].lowest_y());
                }
                swap(o, bottom, top);
         
                while(split_pnt >= ((o[bottom].highest_y() + o[bottom].lowest_y()) * (fp_t)0.5) && bottom < top)
                {
                    max_left = max(max_left, o[bottom++].highest_y());
                }
                swap(o, bottom, top);
            }

            /* Parition the last primitive */
            if (split_pnt >= ((o[bottom].highest_y() + o[bottom].lowest_y()) * (fp_t)0.5))
            {
                max_left = max(max_left, o[bottom++].highest_y());
            }
            else
            {
                min_right = min(min_right, o[top].lowest_y());
            }
            break;
    
         case axis_t::z_axis :
            while(bottom < top)
            {
                while(split_pnt < ((o[top].highest_z() + o[top].lowest_z()) * (fp_t)0.5) && top > bottom)
                {
                    min_right = min(min_right, o[top--].lowest_z());
                }
                swap(o, bottom, top);
         
                while(split_pnt >= ((o[bottom].highest_z() + o[bottom].lowest_z()) * (fp_t)0.5) && bottom < top)
                {
                    max_left = max(max_left, o[bottom++].highest_z());
                }
                swap(o, bottom, top);
            }

            /* Parition the last primitive */
            if (split_pnt >= ((o[bottom].highest_z() + o[bottom].lowest_z()) * (fp_t)0.5))
            {
                max_left = max(max_left, o[bottom++].highest_z());
            }
            else
            {
                min_right = min(min_right, o[top].lowest_z());
            }
            break;
    
        default :
            assert(false);
    }

    /* Construct the BIH nodes */
    bih_node::get_node_array(cur).create_generic_node(*child, max_left, min_right, split_axis);
#ifdef SPATIAL_SUBDIVISION_STATISTICS
    ++ng;
#endif

    /* Reduce the bouding boxes to just fit the new split */
    fp_t pos;
    switch (split_axis)
    {
        case axis_t::x_axis :
            pos = (tm.x - bl.x) * (fp_t)0.5;
            while (((bl.x + pos) > max_left) && (max_left > -MAX_DIST))
            {
                tm.x = bl.x + pos;
                pos *= (fp_t)0.5;
            }

            pos = (tr.x - bm.x) * (fp_t)0.5;
            while (((tr.x - pos) < min_right) && (min_right < MAX_DIST))
            {
                bm.x = tr.x - pos;
                pos *= (fp_t)0.5;
            }
            break;

        case axis_t::y_axis :
            pos = (tm.y - bl.y) * (fp_t)0.5;
            while (((bl.y + pos) > max_left) && (max_left > -MAX_DIST))
            {
                tm.y = bl.y + pos;
                pos *= (fp_t)0.5;
            }

            pos = (tr.y - bm.y) * (fp_t)0.5;
            while (((tr.y - pos) < min_right) && (min_right < MAX_DIST))
            {
                bm.y = tr.y - pos;
                pos *= (fp_t)0.5;
            }
            break;

        case axis_t::z_axis :
            pos = (tm.z - bl.z) * (fp_t)0.5;
            while (((bl.z + pos) > max_left) && (max_left > -MAX_DIST))
            {
                tm.z = bl.z + pos;
                pos *= (fp_t)0.5;
            }

            pos = (tr.z - bm.z) * (fp_t)0.5;
            while (((tr.z - pos) < min_right) && (min_right < MAX_DIST))
            {
                bm.z = tr.z - pos;
                pos *= (fp_t)0.5;
            }
            break;

        default :
            assert(false);
            break;
   }
  
    /* Recurse */
    unsigned left_child  = *child;
    unsigned right_child = *child + 1;
    
    /* Recurse left */
    int node_size = (bottom - b);
    cout << "node size 1: " << node_size << endl;
    if (node_size == 2)
    {
        cout << o[b    ].lowest_x()  << ", " << o[b    ].lowest_y()  << ", " << o[b    ].lowest_z() << endl;
        cout << o[b + 1].lowest_x()  << ", " << o[b + 1].lowest_y()  << ", " << o[b + 1].lowest_z() << endl;
//        cout << o[b + 2].lowest_x()  << ", " << o[b + 2].lowest_y()  << ", " << o[b + 2].lowest_z() << endl;
//        cout << o[b + 3].lowest_x()  << ", " << o[b + 3].lowest_y()  << ", " << o[b + 3].lowest_z() << endl;
//        cout << o[b + 4].lowest_x()  << ", " << o[b + 4].lowest_y()  << ", " << o[b + 4].lowest_z() << endl;
//        cout << o[b + 5].lowest_x()  << ", " << o[b + 5].lowest_y()  << ", " << o[b + 5].lowest_z() << endl;
//        cout << o[b + 6].lowest_x()  << ", " << o[b + 6].lowest_y()  << ", " << o[b + 6].lowest_z() << endl;
//        cout << o[b + 7].lowest_x()  << ", " << o[b + 7].lowest_y()  << ", " << o[b + 7].lowest_z() << endl;
        cout << o[b    ].highest_x() << ", " << o[b    ].highest_y() << ", " << o[b    ].highest_z() << endl;
        cout << o[b + 1].highest_x() << ", " << o[b + 1].highest_y() << ", " << o[b + 1].highest_z() << endl;
//        cout << o[b + 2].highest_x() << ", " << o[b + 2].highest_y() << ", " << o[b + 2].highest_z() << endl;
//        cout << o[b + 3].highest_x() << ", " << o[b + 3].highest_y() << ", " << o[b + 3].highest_z() << endl;
//        cout << o[b + 4].highest_x() << ", " << o[b + 4].highest_y() << ", " << o[b + 4].highest_z() << endl;
//        cout << o[b + 5].highest_x() << ", " << o[b + 5].highest_y() << ", " << o[b + 5].highest_z() << endl;
//        cout << o[b + 6].highest_x() << ", " << o[b + 6].highest_y() << ", " << o[b + 6].highest_z() << endl;
//        cout << o[b + 7].highest_x() << ", " << o[b + 7].highest_y() << ", " << o[b + 7].highest_z() << endl;
    }

    if (node_size <= 1)
    {
        if ((o[b].size() < MAX_BIH_NODE_SIZE) && (node_size == 1))
        {
            bih_node::get_node_array(left_child).create_leaf_node(o[b].get_begin(), max(0, o[b].get_end() - 1));
        }
        else if (node_size <= 0)
        {
            bih_node::get_node_array(left_child).create_leaf_node(0, 0);
        }
        else
        {
            cout << "primitive builder" << endl;
            (*child) += 2;
            divide_bih_node(p, left_child, child, bl, tm, o[bottom].get_begin(), o[bottom].get_end());
            cout << "returned" << endl;
        }
    }
    else
    {
        (*child) += 2;
        divide_bih_node(o, p, left_child, child, bl, tm, b, bottom - 1);
    }

    /* Recurse right */
    node_size = (e - bottom) + 1;
    cout << "node size 2: " << node_size << endl;
    if (node_size <= 1)
    {
        if ((o[bottom].size() < MAX_BIH_NODE_SIZE) && (node_size == 1))
        {
            bih_node::get_node_array(right_child).create_leaf_node(o[bottom].get_begin(), max(0, o[bottom].get_end() - 1));
        }
        else if (node_size == 0)
        {
            bih_node::get_node_array(right_child).create_leaf_node(0, 0);
        }
        else
        {
            cout << "primitive builder" << endl;
            (*child) += 2;
            divide_bih_node(p, right_child, child, bm, tr, o[bottom].get_begin(), o[bottom].get_end());
            cout << "returned" << endl;
        }
    }
    else
    {
        (*child) += 2;
        divide_bih_node(o, p, right_child, child, bm, tr, bottom, e);
    }
    
    depth--;
    return;
}


/**********************************************************
 
**********************************************************/
// void approximate_sort(const primitive_list *const o, vector<triangle *> *const copy)
// {
//     /* Calculate the total average size primitive */
//     point_t average(0.0, 0.0, 0.0);
//     point_t scale(((fp_t)o->size() * (fp_t)0.16), ((fp_t)o->size() * (fp_t)0.16), ((fp_t)o->size() * (fp_t)0.16));
//     for (unsigned int i = 0; i < o->size(); i++)
//     {
//         average += ((*o)[i]->highest_point() - (*o)[i]->lowest_point()) / scale;
//     }
//     cout << "average: " << average.x << ", " << average.y << ", " << average.z << endl;
    
//     /* Calculate the number of cells */
//     point_t scene_size  = triangle::get_scene_upper_bounds() - triangle::get_scene_lower_bounds();
//     point_t grid_res    = scene_size / average;

//     cout << "scene_size: " << scene_size.x << ", " << scene_size.y << ", " << scene_size.z << endl;
//     cout << "grid_res: " << grid_res.x << ", " << grid_res.y << ", " << grid_res.z << endl << endl;
    
//     /* Calculate the number of primitives in each cell */
//     unsigned int x_width        = (unsigned int)ceil(grid_res.x) + 1;
//     unsigned int y_width        = (unsigned int)ceil(grid_res.y) + 1;
//     unsigned int z_width        = (unsigned int)ceil(grid_res.z) + 1;
//     unsigned int nr_of_cells    = x_width * y_width * z_width;
//     unsigned int *prim_per_cell = new unsigned int [nr_of_cells + 1];
//     memset(prim_per_cell, 0, ((nr_of_cells + 1) * sizeof(unsigned int)));
//     cout << "dimensions: " << x_width  << ", " << y_width << ", " << z_width << endl;
//     cout << "number of cells: " << nr_of_cells << endl;
    
//     /* Adjust average for the number of cells */
//     average.x = scene_size.x / (unsigned int)ceil(grid_res.x);
//     average.y = scene_size.y / (unsigned int)ceil(grid_res.y);
//     average.z = scene_size.z / (unsigned int)ceil(grid_res.z);
//     cout << "average: " << average.x << ", " << average.y << ", " << average.z << endl;

//     for (unsigned int i = 0; i < o->size(); i++)
//     {
//         unsigned int cell_x = (unsigned int)ceil((((*o)[i]->lowest_x()) - triangle::get_scene_lower_bounds().x) / average.x);
//         unsigned int cell_y = (unsigned int)ceil((((*o)[i]->lowest_y()) - triangle::get_scene_lower_bounds().y) / average.y);
//         unsigned int cell_z = (unsigned int)ceil((((*o)[i]->lowest_z()) - triangle::get_scene_lower_bounds().z) / average.z);
        
//         assert((cell_x + (cell_y * x_width) + (cell_z * x_width * y_width)) <= nr_of_cells);
//         prim_per_cell[cell_x + (cell_y * x_width) + (cell_z * x_width * y_width)]++;
//     }
    
//     /* Convert count to offset */
//     bih_bucket   *bucket_list   = new bih_bucket [nr_of_cells + 1];
//     bucket_list[0].add_indices(0, (prim_per_cell[0] - 1));
//     for (unsigned int i = 1; i <= nr_of_cells; i++)
//     {
//         prim_per_cell[i] = prim_per_cell[i] + prim_per_cell[i - 1];
//         bucket_list[i].add_indices(prim_per_cell[i - 1], (prim_per_cell[i] - 1));
//     }
    
//     /* Approximate sort */
//     for (unsigned int i = 0; i < o->size(); i++)
//     {
//         unsigned int cell_x = (unsigned int)ceil((((*o)[i]->lowest_x()) - triangle::get_scene_lower_bounds().x) / average.x);
//         unsigned int cell_y = (unsigned int)ceil((((*o)[i]->lowest_y()) - triangle::get_scene_lower_bounds().y) / average.y);
//         unsigned int cell_z = (unsigned int)ceil((((*o)[i]->lowest_z()) - triangle::get_scene_lower_bounds().z) / average.z);
        
//         unsigned int cell_addr = cell_x + (cell_y * x_width) + (cell_z * x_width * y_width);
//         bucket_list[cell_addr].expand((*o)[i]->highest_point(), (*o)[i]->lowest_point());
//         (*copy)[prim_per_cell[cell_addr]] = (*o)[i];
//     }
    
//     unsigned int addr = 0;
//     for(unsigned int i = 1; i <= nr_of_cells; i++)
//     {
//         if (bucket_list[i].size())
//         {
//             bucket_list[addr++] = bucket_list[i];
//         }
//     }
//     cout << "addr: " << addr << endl;
    
//     cout << "bucketted" << endl;
// //    assert(false);
//     delete [] prim_per_cell;
    
//     /* Sort the buckets like they were primitives */
//     unsigned grand_child = 1;
//     divide_bih_node(bucket_list, copy, 0, &grand_child, triangle::get_scene_lower_bounds(), triangle::get_scene_upper_bounds(), 0, addr - 1);
//     assert(depth == 0);
// }


/**********************************************************
 
**********************************************************/
const vector<triangle *> * build_bih(const primitive_list *const o, bih_node *bih)
{
//    cout << "BIH construction has begun" << endl;

    /* Bound all primitives in the scene and copy them */
    vector<triangle *> *object_copy = new vector<triangle *>;
    *object_copy = *o;

    /* Divide the nodes */
    bih_node::set_primitives(object_copy);
    bih_node::set_node_array(bih);
    unsigned grand_child = 1;

    depth = 0;
    divide_bih_node(object_copy, 0, &grand_child, triangle::get_scene_lower_bounds(), triangle::get_scene_upper_bounds(), 0, object_copy->size() - 1);

//    approximate_sort(o, object_copy);
    
    assert(depth == 0);
    
    return object_copy;
}
}; /* namespace raptor_raytracer */
