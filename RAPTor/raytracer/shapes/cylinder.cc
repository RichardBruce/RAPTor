/***********************************************************
 Implementation of the shape virtual functions for a cylinder
************************************************************/

#include "cylinder.h"


namespace raptor_raytracer
{
cylinder::cylinder(material *const m, const point_t &c, const fp_t h, const fp_t r, const fp_t i, const vector_t &n, const fp_t theta, const bool l) : 
                   shape(m,c,l), rot_axis(n), r(r), h(h), i_sq(i*i), theta(theta)
{
    assert(this->i_sq < (this->r * this->r));
    assert(this->i_sq >= 0.0);
    
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
        const point_t hei((q.x * this->h),(q.y * this->h),(q.z * this->h));

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
        this->set_bounding_box(point_t((c.x + this->r), (c.y + this->r), (c.z + this->h)), point_t((c.x - this->r), (c.y - this->r), (c.z)));
    }
}


cylinder::cylinder(material *const m, const point_t &c, const point_t &b, const fp_t r, const fp_t i, const bool l) : 
                   shape(m,c,l), r(r), i_sq(i*i)
{
    assert(this->i_sq < (this->r * this->r));
    assert(this->i_sq >= 0.0);
    
    /* Find the height and vector through the cylinder */
    point_t cur_dir  = b - c;
    this->h          = magnitude(cur_dir);
    cur_dir         /= this->h;

    /* The original cylinder had a vector through it in 'z'                             */
    /* Take the cross product of the vector of the original and the rotated cylinder    */
    /* This give the normal to the plane the cylinder was rotated in                    */
    /* This is the axis to rotate around                                                */
    this->rot_axis = point_t(cur_dir.y, -cur_dir.x, 0.0);

    /* This is for the special case of the cylinder pointing in the opposite direction  */
    if ((cur_dir.y == 0.0) && (cur_dir.x == 0.0))
    {
        rot_axis.x = 1.0;
    }

    /* acos of the dot product of the two centre vectors gives the angle of rotation    */
    this->theta = acos(cur_dir.z);
    
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
        const point_t hei((q.x * this->h),(q.y * this->h),(q.z * this->h));

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
        this->set_bounding_box(point_t((c.x + this->r), (c.y + this->r), (c.z + this->h)), point_t((c.x - this->r), (c.y - this->r), (c.z)));
    }
}


/***********************************************************
 is_intersecting returns the distance along the line l that 
 the cylinder and the line intersect. If the shere and the line 
 do not intersect 'DOUBLE_MAX' is be returned.
 
 The distance to intersection is calculated by substituting 
 the equation for a line (x0 + grad*d) in 3-D into the 
 equation of a circle (x^2 +y^2 + z^2 = r^2) and solving as
 a quadratic equation.
************************************************************/
fp_t cylinder::is_intersecting(const ray *const r, hit_t *const h) const
{
    /* Rotate the ray into the cycliners co-ordinate system */
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
    fp_t r_sq   = (this->r * this->r);
    fp_t top_of_cyl = this->get_z0() + this->h;
        
    fp_t a = (rotated.get_x_grad() * rotated.get_x_grad()) + 
             (rotated.get_y_grad() * rotated.get_y_grad());

    fp_t b = -((rotated.get_x_grad() * x_dist)  + 
               (rotated.get_y_grad() * y_dist));

    fp_t c = (x_dist  * x_dist) +
             (y_dist  * y_dist); 

    fp_t intersection = (b * b) - (a * (c-r_sq));

    /* Return if there is no intersection of an infinite cylinder
            -ve = no intersection
              0 =  1 intersection
            +ve =  2 intersection */
    if (intersection < 0.0)
    {
        return DOUBLE_MAX;
    }

    /* Check for intersetion with the closest wall */
    intersection = sqrt(intersection);
    fp_t dist_to_wall = (b - intersection) / a;
    
    /* Did we hit the cyclinder within its length */
    fp_t z_at_inter = rotated.get_z0() + (dist_to_wall * rotated.get_z_grad());
    bool z_in_range = ((z_at_inter > this->get_z0()) && (z_at_inter < top_of_cyl));
    
    /* Try the furthest wall if the nearest was missed */
    hit_t wall_hit = out_in;
    if ((dist_to_wall < DOUBLE_ERR) | (!z_in_range))
    {
        dist_to_wall = (b + intersection) / a;
        z_at_inter = rotated.get_z0() + (dist_to_wall * rotated.get_z_grad());
        z_in_range = ((z_at_inter > this->get_z0()) && (z_at_inter < top_of_cyl));
        
        /* If we miss both outer walls */
        if ((dist_to_wall < DOUBLE_ERR) | (!z_in_range))
        {
            dist_to_wall = DOUBLE_MAX;
        }
		else
		{
			wall_hit = in_out;
		}
    }

#if 0
    /* If there is an internal wall test it for intersections */
    if(this->i_sq > 0.0)
    {        
        intersection = (b * b) - (a * (c-this->i_sq));
    
        intersection = sqrt(intersection);
        fp_t dist_to_inner = (b - intersection) / a;
    
        /* Did we hit the cyclinder within its length */
        z_at_inter = rotated.get_z0() + (dist_to_inner * rotated.get_z_grad());
        z_in_range = ((z_at_inter > this->get_z0()) && (z_at_inter < top_of_cyl));
    
        /* Try the furthest wall if the nearest was missed */
		hit_t inner_hit = in_cav;
        if ((dist_to_inner < DOUBLE_ERR) | (!z_in_range))
        {
            dist_to_inner = (b + intersection) / a;
            z_at_inter = rotated.get_z0() + (dist_to_inner * rotated.get_z_grad());
            z_in_range = ((z_at_inter > this->get_z0()) && (z_at_inter < top_of_cyl));
        
            /* If we miss both inner walls */
            if ((dist_to_inner < DOUBLE_ERR) | (!z_in_range))
            {
                dist_to_inner = DOUBLE_MAX;
            }
			else
			{
				inner_hit = cav_in;
			}
        }
		
		if (dist_to_inner < dist_to_wall)
		{
	        dist_to_wall = dist_to_inner;
			wall_hit     = inner_hit;
		}
    }

    /* Check for intersections with the end cap */
    fp_t dist_to_low  = -z_dist/rotated.get_z_grad();
    fp_t dist_to_high = -(rotated.get_z0() - top_of_cyl)/rotated.get_z_grad();
    
    /* Dont look backward down the ray */
	hit_t cap_hit = out_in;
    if (dist_to_low  < DOUBLE_ERR)
	{
		cap_hit     = in_out;
		dist_to_low = DOUBLE_MAX;
	}

    if (dist_to_high < DOUBLE_ERR)
	{
		cap_hit      = in_out;
		dist_to_high = DOUBLE_MAX;
	}

	/* See if the cap is intersected */    
    fp_t dist_to_cap = min(dist_to_high, dist_to_low);
    fp_t x_at = x_dist + (dist_to_cap * rotated.get_x_grad());
    fp_t y_at = y_dist + (dist_to_cap * rotated.get_y_grad());
    fp_t cir_at = ((x_at * x_at) + (y_at * y_at));
    
    if ((cir_at > r_sq) | (cir_at < this->i_sq))
    {
        dist_to_cap = DOUBLE_MAX;
    }

    /* Return the closest intersection */
	if (dist_to_wall < dist_to_cap)
	{
		*h = wall_hit;
	    return dist_to_wall;
	}
	else
	{
		*h = cap_hit;
	    return dist_to_cap;
	}
#else
    *h = wall_hit;
    return dist_to_wall;
#endif
}


/***********************************************************
 normal_at_point returns a line normal to the cylinder at the 
 point p.
 
 The normal is calculated as a line from the centre of the 
 cylinder through the point p. The line create starts from 
 the point p.
************************************************************/
line cylinder::normal_at_point(ray *const r, const hit_t h) const
{
    fp_t x_dir = 0.0;
    fp_t y_dir = 0.0;
    fp_t z_dir = 0.0;
    point_t dst;
    
    /* Move the end point of the ray into the cyclinders co-ordinate system */
    if(this->rotate)
    {
        dst = r->rotate_dst(this->rot_axis, this->get_centre(), this->theta);
    }
    else
    {
        dst = r->get_dst();
    }

#if 0
    fp_t end = dst.z - this->get_z0();
    if (abs(end) < DOUBLE_ERR)
    {
        z_dir = -1.0;
    }
	else if (abs(end - this->h) < DOUBLE_ERR)
	{
        z_dir = 1.0;
	}
    else
    {
        /* Start point of line is given by point p */ 
        /* Direction is given by the line from the centre of the cylinder to point p */
        x_dir = (dst.x - this->get_x0())/this->r;
        y_dir = (dst.y - this->get_y0())/this->r;
    }
#else
    x_dir = (dst.x - this->get_x0())/this->r;
    y_dir = (dst.y - this->get_y0())/this->r;
#endif
    
    /* Create the normal and move into the rays co-ordinate system */
    line l(dst, x_dir, y_dir, z_dir);
    if(this->rotate)
    {
        l.rotate(this->rot_axis, this->get_centre(), -this->theta);
    }

    /* If the ray is leaving the cylinder change the 
	   direction of the line */
    if ((h == in_out) || (h == cav_in))
    {
        l.opposite_dir();
    }
    
    return l;
}


/***********************************************************
************************************************************/
point_t cylinder::get_random_point(const int i) const
{
    fp_t x_h8 = this->r/4.0;
    fp_t y_h8 = this->r/4.0;
    fp_t z_h8 = this->h/4.0;
    
    fp_t x = this->get_x0() + ((fp_t) (i     & 0x3) * x_h8) + (gen_random_mersenne_twister() * x_h8);
    fp_t y = this->get_y0() + ((fp_t)((i>>2) & 0x3) * y_h8) + (gen_random_mersenne_twister() * y_h8);
    fp_t z = this->get_z0() + ((fp_t)((i>>2) & 0x3) * z_h8) + (gen_random_mersenne_twister() * z_h8);
    
    return point_t(x,y,z);
}
}; /* namespace raptor_raytracer */
