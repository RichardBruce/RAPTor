#include "kdt_node.h"


/**********************************************************
  test_leaf_node_nearest tests all primitives of the 
  KD-tree leaf node to find the nearest.
  
  'r' is a ray traversing this leaf node. 'h' is the direction 
  of the hit or miss if no intersection was found. 'm' is 
  the maximum distance and 'min' is the minimum distance to
  accept intersections within. This range check is 
  important. If it is not met there may be a closer 
  intersecting object in another leaf node.
**********************************************************/
triangle* kdt_node::test_leaf_node_nearest(const ray *const r, hit_description *const h, const fp_t min) const
{
    triangle *intersecting_object = NULL;

    for (primitive_list::const_iterator i = this->p->begin(); i != this->p->end(); ++i)
    {
        hit_description hit_type;
        (*i)->is_intersecting(r, &hit_type);
#ifdef SPATIAL_SUBDIVISION_STATISTICS
        nit++;
#endif
        if ((hit_type.d < ((h->d) + (1.0 * EPSILON))) && (hit_type.d > (min - (1.0 * EPSILON))))
        {
            *h = hit_type;
            intersecting_object = *i;
        }
    }
    
#ifdef SPATIAL_SUBDIVISION_STATISTICS
    if (h->h != miss)
    {
        ritm++;
    }
#endif
    
    return intersecting_object;
}


/**********************************************************
  test_leaf_node_nearer tests the primitives of the 
  KD-tree leaf node until one closer than 'max' is found.
  If such an intersection is found true is returned. 
  Otherwise false is returned.
  
  r' is a ray traversing this leaf node. 'max' is the 
  maximum distance to find intersections before. If an 
  intersection closer than 'max' is found then true may be
  returned immediately.
**********************************************************/
bool kdt_node::test_leaf_node_nearer(const ray *const r, const fp_t max) const
{
    for (primitive_list::const_iterator i = this->p->begin(); i != this->p->end(); ++i)
    {
        if ((*i)->get_light() || (*i)->is_transparent())
        {
            continue;
        }
        
		hit_description hit_type;
        (*i)->is_intersecting(r, &hit_type);
#ifdef SPATIAL_SUBDIVISION_STATISTICS
        nit++;
#endif
        if ((hit_type.d < max) && (hit_type.d > (1.0 * EPSILON)))
        {
#ifdef SPATIAL_SUBDIVISION_STATISTICS
            ritm++;
#endif
            return true;
        }
    }
    
    return false;
}
