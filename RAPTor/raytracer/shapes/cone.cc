/***********************************************************
 Implementation of the shape virtual functions for a cone
************************************************************/

#include "cone.h"


cone::cone(material *const m, const point_t &c, const fp_t r, const fp_t h, const fp_t top, 
             const vector_t &n, const fp_t theta, const bool l) : 
           shape(m,c,l), r(r), h(h), top(top ? top : h), theta(theta), rot_axis(n)
{
    if ((theta != 0.0) && (rot_axis.x || rot_axis.y || rot_axis.z))
    {
        rotate = true;

        /* Normalise rot_axis */
        normalise(&this->rot_axis);
        
        /* Rotate the opposite direction to the rays */
        fp_t costheta = cos(((2.0 * PI) - this->theta));
        fp_t sintheta = sin(((2.0 * PI) - this->theta));
    
        /* Rotate a unit height from the origin about the rotate vector */
        point_t q;
        q.x = (1.0 - costheta) * this->rot_axis.x * this->rot_axis.z + this->rot_axis.y * sintheta;
        q.y = (1.0 - costheta) * this->rot_axis.y * this->rot_axis.z - this->rot_axis.x * sintheta;
        q.z = costheta + (1.0 - costheta) * this->rot_axis.z * this->rot_axis.z;

        /* Calculate contributions from height and radius */
        const point_t rad(fabs(q.x * this->r),fabs(q.y * this->r),fabs(q.z * this->r));
        const point_t hei((q.x * top),(q.y * top),(q.z * top));

        /* The highest bounding box points are:
            The centre plus a contribution from the height if the cyclinder points upwards 
            Plus contributions from the radius in the perphendiculars to the new up */
        point_t max_bb = c;
        max_bb.x += max((fp_t)0.0, hei.x);
        max_bb.y += max((fp_t)0.0, hei.y);
        max_bb.z += max((fp_t)0.0, hei.z);
        max_bb.x += rad.y;
        max_bb.y += rad.z;
        max_bb.z += rad.x;
        max_bb.x += rad.z;
        max_bb.y += rad.x;
        max_bb.z += rad.y;

        /* The lowest bounding box points are:
            The centre plus a contribution from the height if the cyclinder points downwards 
            Minus contributions from the radius in the perphendiculars to the new up */
        point_t min_bb = c;
        min_bb.x += min((fp_t)0.0, hei.x);
        min_bb.y += min((fp_t)0.0, hei.y);
        min_bb.z += min((fp_t)0.0, hei.z);
        min_bb.x -= rad.y;
        min_bb.y -= rad.z;
        min_bb.z -= rad.x;
        min_bb.x -= rad.z;
        min_bb.y -= rad.x;
        min_bb.z -= rad.y;

        this->set_bounding_box(max_bb, min_bb);
    }
    else
    {
        rotate = false;
        fp_t width = max(this->r, ((top - this->h)*(this->r/this->h)));
        this->set_bounding_box(point_t((c.x + width), (c.y + width), (c.z + top)), point_t((c.x - width), (c.y - width), (c.z)));
    }
}

cone::cone(material *const m, const point_t &top, const point_t &c, const fp_t r0, const fp_t r1, const bool l) : 
           shape(m,c,l), r(r1)
{
    /* Find the height and vector through the cylinder */
    vector_t cur_dir = top - c;
    top          = magnitude(cur_dir);
    cur_dir         /= top;

    /* The original cone had a vector through it in 'z'                             */
    /* Take the cross product of the vector of the original and the rotated cone    */
    /* This give the normal to the plane the cone was rotated in                    */
    /* This is the axis to rotate around                                            */
    this->rot_axis = point_t(cur_dir.y, -cur_dir.x, (fp_t)0.0);

    /* This is for the special case of the cone pointing in the opposite direction  */
    if ((cur_dir.y == 0.0) && (cur_dir.x == 0.0))
    {
        this->rot_axis.x = 1.0;
    }

    /* acos of the dot product of the two centre vectors gives the angle of rotation */
    this->theta = acos(cur_dir.z);

    /* Calculte the height of the point of the cone */
    this->h = (r1 * top)/(r1 - r0);

    if (theta != 0.0)
    {
        rotate = true;

        /* Normalise rot_axis */
        normalise(&this->rot_axis);
        
        /* Rotate the opposite direction to the rays */
        fp_t costheta = cos(((2.0 * PI) - this->theta));
        fp_t sintheta = sin(((2.0 * PI) - this->theta));
    
        /* Rotate a unit height from the origin about the rotate vector */
        point_t q;
        q.x = (1.0 - costheta) * this->rot_axis.x * this->rot_axis.z + this->rot_axis.y * sintheta;
        q.y = (1.0 - costheta) * this->rot_axis.y * this->rot_axis.z - this->rot_axis.x * sintheta;
        q.z = costheta + (1.0 - costheta) * this->rot_axis.z * this->rot_axis.z;

        /* Calculate contributions from height and radius */
        const point_t rad(fabs(q.x * this->r),fabs(q.y * this->r),fabs(q.z * this->r));
        const point_t hei((q.x * top),(q.y * top),(q.z * top));

        /* The highest bounding box points are:
            The centre plus a contribution from the height if the cyclinder points upwards 
            Plus contributions from the radius in the perphendiculars to the new up */
        point_t max_bb = c;
        max_bb.x += max((fp_t)0.0, hei.x);
        max_bb.y += max((fp_t)0.0, hei.y);
        max_bb.z += max((fp_t)0.0, hei.z);
        max_bb.x += rad.y;
        max_bb.y += rad.z;
        max_bb.z += rad.x;
        max_bb.x += rad.z;
        max_bb.y += rad.x;
        max_bb.z += rad.y;

        /* The lowest bounding box points are:
            The centre plus a contribution from the height if the cyclinder points downwards 
            Minus contributions from the radius in the perphendiculars to the new up */
        point_t min_bb = c;
        min_bb.x += min((fp_t)0.0, hei.x);
        min_bb.y += min((fp_t)0.0, hei.y);
        min_bb.z += min((fp_t)0.0, hei.z);
        min_bb.x -= rad.y;
        min_bb.y -= rad.z;
        min_bb.z -= rad.x;
        min_bb.x -= rad.z;
        min_bb.y -= rad.x;
        min_bb.z -= rad.y;

        this->set_bounding_box(max_bb, min_bb);
    }
    else
    {
        rotate = false;
        fp_t width = max(this->r, ((top - this->h)*(this->r/this->h)));
        this->set_bounding_box(point_t((c.x + width), (c.y + width), (c.z + top)), point_t((c.x - width), (c.y - width), (c.z)));
    }
}



/***********************************************************
 is_intersecting returns the distance along the line l that 
 the cone and the line intersect. If the shere and the line 
 do not intersect 'DOUBLE_MAX' is be returned.
 
 The distance to intersection is calculated by substituting 
 the equation for a line (x0 + grad*d) in 3-D into the 
 equation of a cone and solving as a quadratic equation.
 Addition extra calculations may be performed for the end 
 caps of the cone.
************************************************************/
fp_t cone::is_intersecting(const ray *const r, hit_t *const h) const
{
    /* Rotate the ray into the cones co-ordinate system */
    ray rotated;
    if (this->rotate)
    {
        rotated = r->rotate(this->rot_axis, this->get_centre(), this->theta);
    }
    else
    {
        rotated = *r;
    }
    
    fp_t x_dist = rotated.get_x0() - this->get_x0();
    fp_t y_dist = rotated.get_y0() - this->get_y0();
    fp_t z_dist = rotated.get_z0() - this->get_z0();
    fp_t r_sq   = this->r * this->r;
    fp_t h_sq   = this->h * this->h;
    fp_t top_of_con = this->get_z0() + top;

    fp_t a = (rotated.get_x_grad() * rotated.get_x_grad()) + 
             (rotated.get_y_grad() * rotated.get_y_grad()) - 
            ((rotated.get_z_grad() * rotated.get_z_grad() * r_sq)/ h_sq);

    fp_t b = -((rotated.get_x_grad() * x_dist)  + 
               (rotated.get_y_grad() * y_dist)  - 
             ((((rotated.get_z_grad() * z_dist)/h_sq) - (rotated.get_z_grad()/this->h)) * r_sq));

    fp_t c = (x_dist * x_dist) +
             (y_dist * y_dist) -
            ((((z_dist * z_dist)/h_sq) + 1.0 - ((2.0 * z_dist)/this->h)) * r_sq); 

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
    
    /* Range check against the top of the cone so it can be capped */
    fp_t z_at_inter = rotated.get_z0() + (dist_to_inter * rotated.get_z_grad());
    bool z_in_range = ((z_at_inter > this->get_z0()) && (z_at_inter < top_of_con));
    
    /* Try the furthest wall if the nearest was missed */
    hit_t wall_hit = out_in;
    if ((dist_to_inter < DOUBLE_ERR) | (!z_in_range))
    {
        dist_to_inter = (b + intersection) / a;
        /* If distance is positive we are inside the cylinder */
        
        z_at_inter = rotated.get_z0() + (dist_to_inter * rotated.get_z_grad());
        z_in_range = ((z_at_inter > this->get_z0()) && (z_at_inter < top_of_con));
        
        /* If we miss both walls */
        if ((dist_to_inter < DOUBLE_ERR) | (!z_in_range))
        {
            dist_to_inter = DOUBLE_MAX;
        }
		else
		{
			wall_hit = in_out;
		}
    }

#if 0
    /* Check for intersections with the end cap */
    fp_t dist_to_cap = -z_dist/rotated.get_z_grad();
    /* Dont look backward down the ray */
	hit_t cap_hit = out_in;
    if (dist_to_cap < DOUBLE_ERR)
	{
		cap_hit     = in_out;
		dist_to_cap = DOUBLE_MAX;
	}
    fp_t radius_sq = r_sq;
    
    if (top != this->h)
    {
        fp_t dist_to_top = -(rotated.get_z0() - top_of_con)/rotated.get_z_grad();
        if (dist_to_top < DOUBLE_ERR)
		{
			cap_hit     = in_out;
     	    dist_to_top = DOUBLE_MAX;
		}
        
        if (dist_to_top < dist_to_cap)
        {
            dist_to_cap = dist_to_top;
        
            z_at_inter = rotated.get_z0() + (dist_to_top * rotated.get_z_grad());
            fp_t radius = this->r - ((this->r/this->h) * (z_at_inter - this->get_z0()));
            radius_sq = radius * radius;
        }
    }
    
    fp_t x_at = x_dist + (dist_to_cap * rotated.get_x_grad());
    fp_t y_at = y_dist + (dist_to_cap * rotated.get_y_grad());
    fp_t cir_at = ((x_at * x_at) + (y_at * y_at));
    
    if (cir_at > radius_sq) /* shouldnt be r_sq should be radius at cap squared */
    {
        dist_to_cap = DOUBLE_MAX;
    }
    
    /* Return the closest intersection */
	if (dist_to_inter < dist_to_cap)
	{
		*h = wall_hit;
	    return dist_to_inter;
	}
	else
	{
		*h = cap_hit;
	    return dist_to_cap;
	}
#else
    *h = wall_hit;
    return dist_to_inter;
#endif
}


/***********************************************************
 normal_at_point returns a line normal to the cone at the 
 point p.
 
 The normal is calculated as a line from the centre of the 
 cone through the point p. The line create starts from 
 the point p. Addtionally normals for the end caps of the cone
 may be returned.
************************************************************/
line cone::normal_at_point(ray *const r, const hit_t h) const
{
    fp_t x_dir = 0.0;
    fp_t y_dir = 0.0;
    fp_t z_dir = 0.0;
    point_t dst;
    
    /* Move the end point of the ray into the cones co-ordinate system */
    if(this->rotate)
    {
        dst = r->rotate_dst(this->rot_axis, this->get_centre(), this->theta);
    }
    else
    {
        dst = r->get_dst();
    }
    
    fp_t end = dst.z - this->get_z0();
#if 0
    if (abs(end) < DOUBLE_ERR)
    {
        z_dir = -1.0;
    }
	else if (abs(end - top) < DOUBLE_ERR)
	{
	    z_dir = 1.0;
	}
    else
    {
        /* Start point of line is given by point p */ 
        /* Direction is given by the line from the centre of the cone to point p */
        /* Allow radius to become negative because really the z_dir would become
           negative and the dot product check can correct this */
		z_dir = this->r/this->h;
        fp_t radius = this->r - (z_dir * end);
        x_dir = (dst.x - this->get_x0())/radius;
        y_dir = (dst.y - this->get_y0())/radius;
		
		if (end > this->h)
		{
			x_dir = -x_dir;
			y_dir = -y_dir;
			z_dir = -z_dir;
		}
    }
#else
    z_dir = this->r/this->h;
    fp_t radius = this->r - (z_dir * end);
    x_dir = (dst.x - this->get_x0())/radius;
    y_dir = (dst.y - this->get_y0())/radius;

    if (end > this->h)
    {
    	x_dir = -x_dir;
    	y_dir = -y_dir;
    	z_dir = -z_dir;
    }
#endif
    
    /* Create the normal and move into the rays co-ordinate system */
    line l(dst, x_dir, y_dir, z_dir);
    if(this->rotate)
    {
        l.rotate(this->rot_axis, this->get_centre(), -this->theta);
    }

    /* If the ray is leaving the cone change the 
	   direction of the line */
    if (h == in_out)
    {
        l.opposite_dir();
    }
    
    return l;
}


/***********************************************************
************************************************************/
point_t cone::get_random_point(const int i) const
{
    fp_t x_h8 = this->r/4.0;
    fp_t y_h8 = this->r/4.0;
    fp_t z_h8 = this->r/4.0;
    
    fp_t x = this->get_x0() + ((fp_t) (i     & 0x3) * x_h8) + (gen_random_mersenne_twister() * x_h8);
    fp_t y = this->get_y0() + ((fp_t)((i>>2) & 0x3) * y_h8) + (gen_random_mersenne_twister() * y_h8);
    fp_t z = this->get_z0() + ((fp_t)((i>>2) & 0x3) * z_h8) + (gen_random_mersenne_twister() * z_h8);
    
    return point_t(x,y,z);
}
