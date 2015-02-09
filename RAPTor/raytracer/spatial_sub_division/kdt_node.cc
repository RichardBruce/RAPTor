#include "kdt_node.h"


namespace raptor_raytracer
{
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
triangle* kdt_node::test_leaf_node_nearest(const ray *const r, hit_description *const h, const float min) const
{
    triangle *intersecting_object = nullptr;

    for (primitive_list::const_iterator i = this->p->begin(); i != this->p->end(); ++i)
    {
        hit_description hit_type;
        (*i)->is_intersecting(r, &hit_type);
        if ((hit_type.d < ((h->d) + (1.0f * EPSILON))) && (hit_type.d > (min - (1.0f * EPSILON))))
        {
            *h = hit_type;
            intersecting_object = *i;
        }
    }
    
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
bool kdt_node::test_leaf_node_nearer(const ray *const r, const float max) const
{
    for (primitive_list::const_iterator i = this->p->begin(); i != this->p->end(); ++i)
    {
        if ((*i)->get_light() || (*i)->is_transparent())
        {
            continue;
        }
        
		hit_description hit_type;
        (*i)->is_intersecting(r, &hit_type);
        if ((hit_type.d < max) && (hit_type.d > (1.0f * EPSILON)))
        {
            return true;
        }
    }
    
    return false;
}
}; /* namespace raptor_raytracer */
