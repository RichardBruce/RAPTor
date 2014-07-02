#include "aab.h"


/***********************************************************
  For a aab (x0,y0,z0) defines the botttom left corner and 
  the normal at this point defines which way the top of the 
  aab faces
************************************************************/
fp_t aab::is_intersecting(const ray *const r, hit_t *const h) const
{
    /* If the line is no gradient is a direction and didnt start within
       the width of that face it will not intersect */
    if (( (r->get_x_grad() == 0)   && 
         ((r->get_x0() < this->get_x0()) || (r->get_x0() > this->highest_x()))) ||
         
        ( (r->get_y_grad() == 0)   && 
         ((r->get_y0() < this->get_y0()) || (r->get_y0() > this->highest_y()))) ||
         
        ( (r->get_z_grad() == 0)   && 
         ((r->get_z0() < this->get_z0()) || (r->get_z0() > this->highest_z()))))
    {
        return DOUBLE_MAX;
    }
    
    /* Calculate the distance for the line to intersect witrh each face */
    fp_t x_inter_lo = (this->get_x0()    - r->get_x0())/r->get_x_grad();
    fp_t x_inter_hi = (this->highest_x() - r->get_x0())/r->get_x_grad();
    fp_t y_inter_lo = (this->get_y0()    - r->get_y0())/r->get_y_grad();
    fp_t y_inter_hi = (this->highest_y() - r->get_y0())/r->get_y_grad();
    fp_t z_inter_lo = (this->get_z0()    - r->get_z0())/r->get_z_grad();
    fp_t z_inter_hi = (this->highest_z() - r->get_z0())/r->get_z_grad();
    
    fp_t near = -DOUBLE_MAX;
    fp_t far  =  DOUBLE_MAX;
    fp_t tmp;
    if (x_inter_lo > x_inter_hi)
    {
        tmp        = x_inter_lo;
        x_inter_lo = x_inter_hi;
        x_inter_hi = tmp;
    }
    if (y_inter_lo > y_inter_hi)
    {
        tmp        = y_inter_lo;
        y_inter_lo = y_inter_hi;
        y_inter_hi = tmp;
    }
    if (z_inter_lo > z_inter_hi)
    {
        tmp        = z_inter_lo;
        z_inter_lo = z_inter_hi;
        z_inter_hi = tmp;
    }

     near = max(x_inter_lo, near);
     far  = min(x_inter_hi, far);    
     near = max(y_inter_lo, near);
     far  = min(y_inter_hi, far);
     near = max(z_inter_lo, near);
     far  = min(z_inter_hi, far);
    
    if (near - far > DOUBLE_ERR)
    {
        return DOUBLE_MAX;
    }
    
    if (near < DOUBLE_ERR)
    {
        *h = in_out;
        return far;
    }
    else
    {
        *h = out_in;
        return near;
    }
}


/***********************************************************
 normal_at_point returns a line normal to the aab at the 
 point p.
 
 The normal is calculated as a line from the centre of the 
 aab through the point p. The line create starts from 
 the point p.
************************************************************/
line aab::normal_at_point(ray *const r, const hit_t h) const
{
    /* Start point of line is given by point p */ 
    /* Direction is given by the line from the centre of the aab to point p */
    fp_t x_dir = 0.0;
    fp_t y_dir = 0.0;
    fp_t z_dir = 0.0;
  
    if      (abs(r->get_x1() - this->get_x0())    < DOUBLE_ERR)  x_dir = -1.0;
    else if (abs(r->get_y1() - this->get_y0())    < DOUBLE_ERR)  y_dir = -1.0;
    else if (abs(r->get_z1() - this->get_z0())    < DOUBLE_ERR)  z_dir = -1.0;
    else if (abs(r->get_x1() - this->highest_x()) < DOUBLE_ERR)  x_dir =  1.0;
    else if (abs(r->get_y1() - this->highest_y()) < DOUBLE_ERR)  y_dir =  1.0;
    else if (abs(r->get_z1() - this->highest_z()) < DOUBLE_ERR)  z_dir =  1.0;

    assert(!((x_dir != 0.0) & (y_dir != 0.0)));
    assert(!((x_dir != 0.0) & (z_dir != 0.0)));
    assert(!((z_dir != 0.0) & (y_dir != 0.0)));

    assert((x_dir != 0.0) | (y_dir != 0.0) | (z_dir != 0.0));

    /* If the ray is leaving the aab change the 
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
// cppcheck-suppress unusedFunction
point_t aab::get_random_point(const int i) const
{
    fp_t x_h8 = this->highest_x()/4.0;
    fp_t y_h8 = this->highest_y()/4.0;
    fp_t z_h8 = this->highest_z()/4.0;
    
    fp_t x = this->get_x0() + ((fp_t) (i     & 0x3) * x_h8) + (gen_random_mersenne_twister() * x_h8);
    fp_t y = this->get_y0() + ((fp_t)((i>>2) & 0x3) * y_h8) + (gen_random_mersenne_twister() * y_h8);
    fp_t z = this->get_z0() + ((fp_t)((i>>2) & 0x3) * z_h8) + (gen_random_mersenne_twister() * z_h8);
    
    return point_t(x,y,z);
}
