/***********************************************************
 Implementation of the shape virtual functions for a ring
************************************************************/

#include "ring.h"


ring::ring(material *const m, const point_t &c, const vector_t &n, const fp_t r, const fp_t i, const bool l) : 
                   shape(m,c,l), r_sq(r*r), i_sq(i*i), n(n)
{
    assert(i < r);
    assert(i >= 0.0);
    
    /* Normalise the normal */
    normalise(&this->n);

    /* Rotate the extremities of the ring */
    /* This is r - (r * cos(angle between axis and normal)) */
    fp_t x_width = abs(r * this->n.y) + abs(r * this->n.z);
    fp_t y_width = abs(r * this->n.x) + abs(r * this->n.z);
    fp_t z_width = abs(r * this->n.x) + abs(r * this->n.y);
    this->set_bounding_box(point_t((c.x + x_width), (c.y + y_width), (c.z + z_width)), point_t((c.x - x_width), (c.y - y_width), (c.z - z_width)));
}


/***********************************************************
 is_intersecting returns the distance along the line l that 
 the ring and the line intersect. If the ring and the line 
 do not intersect 'DOUBLE_MAX' is be returned.
 
 The distance to intersection is calculated by checking if 
 line would intersect an infinite plane embedding the ring.
 If the line intersects this plane the point of intersection
 is checked to see if it lyes within the ring. If the point
 of intersection is within the ring the distance to 
 intersection is returned otherwise 'DOUBLE_MAX' is returned
************************************************************/
fp_t ring::is_intersecting(const ray *const r, hit_t *const h) const
{
    /* Check if the line will intersect the plane of the ring */
    fp_t denom = (this->n.x * r->get_x_grad()) +
                   (this->n.y * r->get_y_grad()) +
                   (this->n.z * r->get_z_grad());
    
    /* If line is perphendicular to the plane of the ring */
    if (denom == 0.0)
    {
        return DOUBLE_MAX;
    }
    
    /* Calculate after what distance the line will intersect the plane of the ring */
    fp_t num = (this->n.x * (this->get_x0() - r->get_x0())) +
                 (this->n.y * (this->get_y0() - r->get_y0())) +
                 (this->n.z * (this->get_z0() - r->get_z0()));
    
    fp_t dist = num/denom;
    if (dist < DOUBLE_ERR)
    {
        return DOUBLE_MAX;
    }
    
    /* Find the point on the plane where intersecion occurs */
    /* REVISIT -- Would the 2-D optimisation be faster? */
    fp_t h_x = (r->get_x0() - this->get_x0()) + (dist * r->get_x_grad());
    fp_t h_y = (r->get_y0() - this->get_y0()) + (dist * r->get_y_grad());
    fp_t h_z = (r->get_z0() - this->get_z0()) + (dist * r->get_z_grad());
    
    /* Check it falls within the bounds of the ring */
    fp_t point = (h_x * h_x) + (h_y * h_y) + (h_z * h_z);
    if ((point > this->r_sq) | (point < this->i_sq))
    {
        return DOUBLE_MAX;
    }

    /* Use the dot product to decide if the ring is being entered or exitted */
	if (denom < 0.0)
	{
		*h = out_in;
	}
	else
	{
		*h = in_out;
	}
    return dist;
}


/***********************************************************
 normal_at_point returns a line normal to the ring at the 
 point p.
 
 The normal of the ring is returned.
************************************************************/
line ring::normal_at_point(ray *const r, const hit_t h) const
{
    line l(r->get_dst(), this->n.x, this->n.y, this->n.z);
    
    if (h == in_out)
    {
        l.opposite_dir();
    }
    return l;
}


/***********************************************************
************************************************************/
point_t ring::get_random_point(const int i) const
{
    fp_t x_h8 = this->r_sq/4.0;
    fp_t y_h8 = this->r_sq/4.0;
    fp_t z_h8 = this->r_sq/4.0;
    
    fp_t x = this->get_x0() + ((fp_t) (i     & 0x3) * x_h8) + (gen_random_mersenne_twister() * x_h8);
    fp_t y = this->get_y0() + ((fp_t)((i>>2) & 0x3) * y_h8) + (gen_random_mersenne_twister() * y_h8);
    fp_t z = this->get_z0() + ((fp_t)((i>>2) & 0x3) * z_h8) + (gen_random_mersenne_twister() * z_h8);
    
    return point_t(x,y,z);
}
