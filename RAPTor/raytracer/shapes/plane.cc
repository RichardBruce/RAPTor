/***********************************************************
 Implementation of the shape virtual functions for a plane
************************************************************/

#include "plane.h"


/**********************************************************
 is_intersecting returns the distance along the line l that 
 the plane and the line intersect. If the plane and the line 
 do not intersect 'DOUBLE_MAX' is be returned.
 
 The distance to intersection is calculated by substituting 
 the equation for a line (x0 + grad*d) in 3-D into the 
 equation of a plane N dot (P-p3) where N is the normal to
 the plane at point P and p3 is a point on the plane.
**********************************************************/
fp_t plane::is_intersecting(const ray *const r, hit_t *const h) const
{
    /* Check if the line will intersect the plane */
    fp_t denom = (this->x_n * r->get_x_grad()) +
                 (this->y_n * r->get_y_grad()) +
                 (this->z_n * r->get_z_grad());
    
    /* If line is perphendicular to the plane */
    if (denom == 0.0)
    {
        return DOUBLE_MAX;
    }
    
    /* Calculate after what distance the line will intersect the plane */
    fp_t num = (this->x_n * (this->get_x0() - r->get_x0())) +
               (this->y_n * (this->get_y0() - r->get_y0())) +
               (this->z_n * (this->get_z0() - r->get_z0()));

	if (denom < 0.0)
	{
		*h = out_in;
	}
	else
	{
		*h = in_out;
	}
    return (num/denom);
}


/**********************************************************
 normal_at_point returns a line normal to the plane at the 
 point p.
 
 The normal is calculated as the normal to the plane that 
 the plane is describred with. The line create starts from 
 the point p.
**********************************************************/
line plane::normal_at_point(ray *const r, const hit_t h) const
{
    fp_t x_dir = this->x_n;
    fp_t y_dir = this->y_n;
    fp_t z_dir = this->z_n;

    /* If line dot normal is > 0, cos(theta) < 90 degrees and the normal is in 
       the opposite direction */
    if (h == in_out)
    {
        x_dir = -x_dir;
        y_dir = -y_dir;
        z_dir = -z_dir;
    }
    
    return line(r->get_dst(), x_dir, y_dir, z_dir);
}
