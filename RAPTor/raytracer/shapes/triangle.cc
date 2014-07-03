/***********************************************************
 Implementation of the shape virtual functions for a triangle
************************************************************/

#include "triangle.h"


namespace raptor_raytracer
{
/***********************************************************
 Constructor for the triangle.
 
 The constructor takes 3 points defining the vetices of the 
 triangle
 
 The major axis and normal of the triangle are pre-calculated.
 The scaling factors for conversion to barycentric 
 co-ordinates and into the plane of the triangle are also 
 pre calculated.
************************************************************/
triangle::triangle(material *const m, const point_t &a, const point_t &, const point_t &c, bool l) : 
           shape(m,c,l), a(a)
{
    /* Pick the bounds of the triangle */
    fp_t hi_x, lo_x, hi_y, lo_y, hi_z, lo_z;
    hi_x = max(a.x, max(b.x, c.x));
    lo_x = min(a.x, min(b.x, c.x));
    hi_y = max(a.y, max(b.y, c.y));
    lo_y = min(a.y, min(b.y, c.y));
    hi_z = max(a.z, max(b.z, c.z));
    lo_z = min(a.z, min(b.z, c.z));
    
    /* If the triangle falls into a plane add some width to it */
    if (lo_x == hi_x)
    {
        hi_x += DOUBLE_ERR;
    }
    if (lo_y == hi_y)
    {
        hi_y += DOUBLE_ERR;
    }
    if (lo_z == hi_z)
    {
        hi_z += DOUBLE_ERR;
    }
    this->set_bounding_box(point_t(hi_x, hi_y, hi_z), point_t(lo_x, lo_y, lo_z));

    
    // Calculate the normal
    vector_t dir_b, dir_c;
    
    dir_b.x = b.x - a.x;
    dir_b.y = b.y - a.y;
    dir_b.z = b.z - a.z;

    dir_c.x = c.x - a.x;
    dir_c.y = c.y - a.y;
    dir_c.z = c.z - a.z;

    n.x = (dir_b.y * dir_c.z) - (dir_b.z * dir_c.y);
    n.y = (dir_b.z * dir_c.x) - (dir_b.x * dir_c.z);
    n.z = (dir_b.x * dir_c.y) - (dir_b.y * dir_c.x);
    
    // Pick the major axis
    fp_t k_value, u_value, v_value, b_u_value, b_v_value, c_u_value, c_v_value;
    if (fabs(n.x) > fabs(n.y))
    {
        if (fabs(n.x) > fabs(n.z))
        {
            this->k   = 0;
            k_value   = n.x;
            u_value   = n.y;
            v_value   = n.z;
            b_u_value = dir_b.y;
            b_v_value = dir_b.z;
            c_u_value = dir_c.y;
            c_v_value = dir_c.z;
        }
        else
        {
            this->k   = 2;
            this->a.x = a.z;
            this->a.y = a.x;
            this->a.z = a.y;
            k_value   = n.z;
            u_value   = n.x;
            v_value   = n.y;
            b_u_value = dir_b.x;
            b_v_value = dir_b.y;
            c_u_value = dir_c.x;
            c_v_value = dir_c.y;
        }
    }
    else
    {
        if (fabs(n.y) > fabs(n.z))
        {
            this->k   = 1;
            this->a.x = a.y;
            this->a.y = a.z;
            this->a.z = a.x;
            k_value   = n.y;
            u_value   = n.z;
            v_value   = n.x;
            b_u_value = dir_b.z;
            b_v_value = dir_b.x;
            c_u_value = dir_c.z;
            c_v_value = dir_c.x;
        }
        else
        {
            this->k   = 2;
            this->a.x = a.z;
            this->a.y = a.x;
            this->a.z = a.y;
            k_value   = n.z;
            u_value   = n.x;
            v_value   = n.y;
            b_u_value = dir_b.x;
            b_v_value = dir_b.y;
            c_u_value = dir_c.x;
            c_v_value = dir_c.y;
        }
    }
    
    // Precompute for intersection
    fp_t k_normal = 1.0/k_value;
    this->n_u = u_value * k_normal;
    this->n_v = v_value * k_normal;
    this->n_dot = ((n.x * a.x) + (n.y * a.y) + (n.z * a.z)) * k_normal;
    
    fp_t i_normal = 1.0/((b_u_value * c_v_value) - (b_v_value * c_u_value));
    this->b_n_u =  b_u_value * i_normal;
    this->b_n_v = -b_v_value * i_normal;
    this->c_n_u =  c_v_value * i_normal;
    this->c_n_v = -c_u_value * i_normal;
    
    // Normalise the normal
    fp_t dist = sqrt((n.x * n.x) + (n.y * n.y) + (n.z * n.z));
    assert(dist > 0.0);
    n.x /= dist;
    n.y /= dist;
    n.z /= dist;
}


/***********************************************************
 is_intersecting returns the distance along the ray r that 
 the triangle and the ray intersect. If the triangle and the 
 ray do not intersect 'DOUBLE_MAX' is be returned.
 
 h should indicate how the ray is interacting with the primitive
 
 The ray is intersected with an infinite plane embedding the 
 triangle. The point of intersection is check to be within 
 the triangle or not using barycentric co-ordinates
************************************************************/
fp_t triangle::is_intersecting(const ray *const r, hit_t *const h) const 
{
    /* Pick componants based on the major axis of the triangle */
    fp_t r_k, r_u, r_v, dir_k, dir_u, dir_v;
    switch (this->k)
    {
        case  0 : r_k   = r->get_x0();
                  r_u   = r->get_y0();
                  r_v   = r->get_z0();
                  dir_k = r->get_x_grad();
                  dir_u = r->get_y_grad();
                  dir_v = r->get_z_grad();
                  break;
        case  1 : r_k   = r->get_y0();
                  r_u   = r->get_z0();
                  r_v   = r->get_x0();
                  dir_k = r->get_y_grad();
                  dir_u = r->get_z_grad();
                  dir_v = r->get_x_grad();
                  break;
        case  2 : r_k   = r->get_z0();
                  r_u   = r->get_x0();
                  r_v   = r->get_y0();
                  dir_k = r->get_z_grad();
                  dir_u = r->get_x_grad();
                  dir_v = r->get_y_grad();
                  break;
        default : assert(false);
                  break;
    }
    
    /* Calculate distance to intersection with the plane of the triangle */
    fp_t ray_dot_normal = dir_k + (this->n_u * dir_u) + (this->n_v * dir_v);
    if (ray_dot_normal == 0.0)
    {
        return DOUBLE_MAX;
    }

    fp_t dist = (this->n_dot - r_k - (this->n_u * r_u) - (this->n_v * r_v)) / ray_dot_normal;
    if (dist < 0.0)
    {
        return DOUBLE_MAX;
    }

    /* Check intersection is within the bounds of the triangle */
    /* NOTE -- this->a has already been translated into the major axis */
    fp_t h_u  = (r_u - this->a.y) + (dist * dir_u);
    fp_t h_v  = (r_v - this->a.z) + (dist * dir_v);
    fp_t beta = (h_v * this->b_n_u) + (h_u  * this->b_n_v);
    if (beta < 0.0)
    {
        return DOUBLE_MAX;
    }
    
    fp_t gamma = (h_u * this->c_n_u) + (h_v * this->c_n_v);
    if ((gamma < 0.0) || ((beta + gamma) > 1.0))
    {
        return DOUBLE_MAX;
    }
    
    /* Calculate weather the ray is entering or leaving the triangle */
    ray_dot_normal = ((r->get_x_grad() * n.x) + (r->get_y_grad() * n.y) + (r->get_z_grad() * n.z));
    if (ray_dot_normal > 0.0)
    {
        *h = in_out;
    }
    else
    {
        *h = out_in;
    }

    return dist;
}


/***********************************************************
 normal_at_point returns a line normal to the triangle at the 
 point p. h is used to determine from which side the triangle 
 is hit
 
 Return the precalculated normal
************************************************************/
line triangle::normal_at_point(ray *const r, const hit_t h) const
{
#ifdef SINGLE_PRECISION
    /* Re-calculate the intesection of the ray with the plane of the triangle */
    /* This gives a more accurate hit point to adjust the ray to */
    fp_t denom  = dot_product(this->n, r->get_dir());
    fp_t num    = dot_product(this->n, (this->get_centre() - r->get_dst()));
    r->change_length(num/denom);
#endif /* #ifdef SINGLE_PRECISION */

    line l(r->get_dst(), this->n.x, this->n.y, this->n.z);
    
    /* If the line is leaving the volume enclosed by 
       the triangle use the opposite normal */
    if (h == in_out)
    {
        l.opposite_dir();
    }
    
	return l;
}


/***********************************************************
  get_random point returns a periodically random, evenly 
  distribute point on the triangle

  This is incorrect and for a cube
************************************************************/
point_t triangle::get_random_point(const int i) const
{
    fp_t x_h8 = (this->highest_x() - this->lowest_x())/8.0;
    fp_t y_h8 = (this->highest_y() - this->lowest_y())/8.0;
    fp_t z_h8 = (this->highest_z() - this->lowest_z())/8.0;
    
    fp_t x = this->get_x0() + ((fp_t) (i     & 0x3) * x_h8) + (gen_random_mersenne_twister() * x_h8);
    fp_t y = this->get_y0() + ((fp_t)((i>>2) & 0x3) * y_h8) + (gen_random_mersenne_twister() * y_h8);
    fp_t z = this->get_z0() + ((fp_t)((i>>2) & 0x3) * z_h8) + (gen_random_mersenne_twister() * z_h8);
    
    return point_t(x,y,z);
}
}; /* namespace raptor_raytracer */
