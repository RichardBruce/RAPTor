/***********************************************************
 Implementation of the shape virtual functions for a sphere
************************************************************/

#include "sphere.h"


namespace raptor_raytracer
{
/***********************************************************
 is_intersecting returns the distance along the line l that 
 the sphere and the line intersect. If the shere and the line 
 do not intersect 'DOUBLE_MAX' is be returned.
 
 The distance to intersection is calculated by substituting 
 the equation for a line (x0 + grad*d) in 3-D into the 
 equation of a circle (x^2 +y^2 + z^2 = r^2) and solving as
 a quadratic equation.
************************************************************/
fp_t sphere::is_intersecting(const ray *const r, hit_t *const h) const
{
    fp_t x_dist = r->get_x0() - this->get_x0();
    fp_t y_dist = r->get_y0() - this->get_y0();
    fp_t z_dist = r->get_z0() - this->get_z0();
    
    fp_t a = (r->get_x_grad() * r->get_x_grad()) + 
             (r->get_y_grad() * r->get_y_grad()) + 
             (r->get_z_grad() * r->get_z_grad());

    fp_t b = -((r->get_x_grad() * x_dist)  + 
               (r->get_y_grad() * y_dist)  + 
               (r->get_z_grad() * z_dist));

    fp_t c = (x_dist  * x_dist) +
             (y_dist  * y_dist) +
             (z_dist  * z_dist) - 
             (this->r * this->r); 

    fp_t intersection = (b * b) - (a * c);

    /* Return if there is no intersection 
            -ve = no intersection
              0 =  1 intersection
            +ve =  2 intersection */
    if (intersection < 0.0)
    {
        return DOUBLE_MAX;
    }
    
    /* Return shortest absolute distance to intersection */
    intersection = sqrt(intersection);
    fp_t dist_to_inter = (b - intersection) / a;
    if (dist_to_inter < DOUBLE_ERR)
    {
        dist_to_inter = (b + intersection) / a;
        /* If distance is positive we are inside the sphere */
		*h = in_out;
		return dist_to_inter;
    }
	else
	{
		*h = out_in;
	    return dist_to_inter;
	}
}


/***********************************************************
 normal_at_point returns a line normal to the sphere at the 
 point p.
 
 The normal is calculated as a line from the centre of the 
 sphere through the point p. The line create starts from 
 the point p.
************************************************************/
line sphere::normal_at_point(ray *const r, const hit_t h) const
{
    /* Start point of line is given by point p */ 
    /* Direction is given by the line from the centre of the sphere to point p */
    fp_t x_dir = (r->get_x1() - this->get_x0())/this->r;
    fp_t y_dir = (r->get_y1() - this->get_y0())/this->r;
    fp_t z_dir = (r->get_z1() - this->get_z0())/this->r;
    
    /* If the ray is leaving the sphere change the 
	   direction of the line */
    if (h == in_out)
    {
        x_dir = -x_dir;
        y_dir = -y_dir;
        z_dir = -z_dir;
    }
    
    return line(r->get_dst(), x_dir, y_dir, z_dir);
}


/***********************************************************
************************************************************/
point_t sphere::get_random_point(const int i) const
{
    fp_t x_h8 = this->r/4.0;
    fp_t y_h8 = this->r/4.0;
    fp_t z_h8 = this->r/4.0;
    
    fp_t x = this->get_x0() + ((fp_t) (i     & 0x3) * x_h8) + (gen_random_mersenne_twister() * x_h8);
    fp_t y = this->get_y0() + ((fp_t)((i>>2) & 0x3) * y_h8) + (gen_random_mersenne_twister() * y_h8);
    fp_t z = this->get_z0() + ((fp_t)((i>>2) & 0x3) * z_h8) + (gen_random_mersenne_twister() * z_h8);
    
    return point_t(x,y,z);
}
}; /* namespace raptor_raytracer */
