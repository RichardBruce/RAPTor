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

}; /* namespace raptor_raytracer */
