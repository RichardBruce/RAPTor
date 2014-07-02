/***********************************************************
 Implementation of the shape virtual functions for a ellipse
************************************************************/

#include "ellipse.h"


ellipse::ellipse(material *const m, point_t c, fp_t h, fp_t r, fp_t a, fp_t i, vector_t n, fp_t theta, bool l) : 
                 shape(m,c,l), rot_axis(n), rot_point(c), r(r), h(h), a(a*r), c(EXP * a), i_sq(i*i), a_sq(a*a), theta(theta)
{ 
    assert(a>0.0);
    
    if ((rot_axis.x || rot_axis.y || rot_axis.z) & (theta != 0.0))
    {
        rotate = true;
        /* Normalise rot_axis */
        fp_t size = sqrt((rot_axis.x * rot_axis.x) + (rot_axis.y * rot_axis.y) + (rot_axis.z * rot_axis.z));
        assert(size > 0);
        rot_axis.x /= size;
        rot_axis.y /= size;
        rot_axis.z /= size;
        
        rot_point.z += this->h/2.0;

        /* Rotate the extremities of the ellipse */
        point_t p(this->a, this->r, this->h/2.0);
        point_t q(0.0,0.0,0.0);

        fp_t costheta = cos(this->theta);
        fp_t sintheta = sin(this->theta);

        /* Rotate about r */
        q.x += fabs((costheta + (1.0 - costheta) * this->rot_axis.x * this->rot_axis.x) * p.x);
        q.x += fabs(((1.0 - costheta) * this->rot_axis.x * this->rot_axis.y - this->rot_axis.z * sintheta) * p.y);
        q.x += fabs(((1.0 - costheta) * this->rot_axis.x * this->rot_axis.z + this->rot_axis.y * sintheta) * p.z);

        q.y += fabs(((1.0 - costheta) * this->rot_axis.x * this->rot_axis.y + this->rot_axis.z * sintheta) * p.x);
        q.y += fabs((costheta + (1.0 - costheta) * this->rot_axis.y * this->rot_axis.y) * p.y);
        q.y += fabs(((1.0 - costheta) * this->rot_axis.y * this->rot_axis.z - this->rot_axis.x * sintheta) * p.z);

        q.z += fabs(((1.0 - costheta) * this->rot_axis.x * this->rot_axis.z - this->rot_axis.y * sintheta) * p.x);
        q.z += fabs(((1.0 - costheta) * this->rot_axis.y * this->rot_axis.z + this->rot_axis.x * sintheta) * p.y);
        q.z += fabs((costheta + (1.0 - costheta) * this->rot_axis.z * this->rot_axis.z) * p.z);

        this->set_bounding_box(point_t((c.x + q.x), (c.y + q.y), (c.z + this->h/2.0 + q.z)), point_t((c.x - q.x), (c.y - q.y), (c.z + this->h/2.0 - q.z)));
    }
    else
    {
        rotate = false;
        this->set_bounding_box(point_t((c.x + this->a), (c.y + this->r), (c.z + this->h)), point_t((c.x - this->a), (c.y - this->r), (c.z)));
    }
}


/***********************************************************
 is_intersecting returns the distance along the line l that 
 the ellipse and the line intersect. If the shere and the line 
 do not intersect 'DOUBLE_MAX' is be returned.
 
 The distance to intersection is calculated by substituting 
 the equation for a line (x0 + grad*d) in 3-D into the 
 equation of a circle (x^2 +y^2 + z^2 = r^2) and solving as
 a quadratic equation.
************************************************************/
fp_t ellipse::is_intersecting(const ray *const r, hit_t *const h) const
{
    /* Rotate the ray into the cycliners co-ordinate system */
    ray rotated;
    if (this->rotate)
    {
        rotated = r->rotate(this->rot_axis, this->rot_point, this->theta);
    }
    else
    {
        rotated = *r;
    }

    fp_t x_dist = rotated.get_x0() - this->get_x0(); 
    fp_t y_dist = rotated.get_y0() - this->get_y0(); 
    fp_t z_dist = rotated.get_z0() - this->get_z0(); 
    fp_t r_sq   = this->r * this->r;
    fp_t top_of_ell = this->get_z0() + this->h;
        
    fp_t a = ((rotated.get_x_grad() * rotated.get_x_grad())/this->a_sq) + 
              (rotated.get_y_grad() * rotated.get_y_grad());

    fp_t b = -(((rotated.get_x_grad() * x_dist)/this->a_sq)  + 
                (rotated.get_y_grad() * y_dist));

    fp_t c = ((x_dist  * x_dist)/this->a_sq) +
              (y_dist  * y_dist); 

    fp_t intersection = (b * b) - (a * (c-r_sq));

    /* Return if there is no intersection of an infinite ellipse
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
    bool z_in_range = ((z_at_inter > this->get_z0()) && (z_at_inter < top_of_ell));
    
    /* Try the furthest wall if the nearest was missed */
    hit_t wall_hit = out_in;
    if ((dist_to_wall < DOUBLE_ERR) | (!z_in_range))
    {
        dist_to_wall = (b + intersection) / a;
        z_at_inter = rotated.get_z0() + (dist_to_wall * rotated.get_z_grad());
        z_in_range = ((z_at_inter > this->get_z0()) && (z_at_inter < top_of_ell));
        
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

    /* If there is an internal wall test it for intersections */
    if(this->i_sq > 0.0)
    {        
        intersection = (b * b) - (a * (c-this->i_sq));
    
        intersection = sqrt(intersection);
        fp_t dist_to_inner = (b - intersection) / a;
    
        /* Did we hit the cyclinder within its length */
        z_at_inter = rotated.get_z0() + (dist_to_inner * rotated.get_z_grad());
        z_in_range = ((z_at_inter > this->get_z0()) && (z_at_inter < top_of_ell));
    
        /* Try the furthest wall if the nearest was missed */
		hit_t inner_hit = in_cav;
        if ((dist_to_inner < DOUBLE_ERR) | (!z_in_range))
        {
            dist_to_inner = (b + intersection) / a;
            z_at_inter = rotated.get_z0() + (dist_to_inner * rotated.get_z_grad());
            z_in_range = ((z_at_inter > this->get_z0()) && (z_at_inter < top_of_ell));
        
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
			wall_hit = inner_hit;
	        dist_to_wall = dist_to_inner;
		}
    }

    /* Check for intersections with the end cap */
    fp_t dist_to_low  =   (-z_dist/rotated.get_z_grad());
    fp_t dist_to_high = (-(rotated.get_z0() - top_of_ell)/rotated.get_z_grad());
    
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
    fp_t cir_at = ((x_at * x_at)/this->a_sq + (y_at * y_at));
    
    if ((cir_at >= r_sq) | (cir_at <= this->i_sq))
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
}


/***********************************************************
 normal_at_point returns a line normal to the ellipse at the 
 point p.
 
 The normal is calculated as a line from the centre of the 
 ellipse through the point p. The line create starts from 
 the point p.
************************************************************/
line ellipse::normal_at_point(ray *const r, const hit_t h) const
{
    fp_t x_dir = 0.0;
    fp_t y_dir = 0.0;
    fp_t z_dir = 0.0;
    point_t dst;
    
    /* Move the end point of the ray into the cyclinders co-ordinate system */
    if(this->rotate)
    {
        dst = r->rotate_dst(this->rot_axis, this->rot_point, this->theta);
    }
    else
    {
        dst = r->get_dst();
    }

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
        /* Direction is given by the line from the centre of the ellipse to point p */
        fp_t y_dist  = dst.y -  this->get_y0();
        fp_t x_dist0 = dst.x - (this->get_x0() - this->c);
        fp_t x_dist1 = dst.x - (this->get_x0() + this->c);
        
        fp_t dist0 = sqrt((x_dist0 * x_dist0) + (y_dist * y_dist));
        fp_t dist1 = sqrt((x_dist1 * x_dist1) + (y_dist * y_dist));
        
        fp_t x_dir0 = x_dist0 / dist0;
        fp_t y_dir0 = y_dist  / dist0;
        fp_t x_dir1 = x_dist1 / dist1;
        fp_t y_dir1 = y_dist  / dist1;
        
        x_dir = (x_dir0 + x_dir1)/2.0;
        y_dir = (y_dir0 + y_dir1)/2.0;
    }

    /* Create the normal and move into the rays co-ordinate system */
    line l(dst, x_dir, y_dir, z_dir);
    if(this->rotate)
    {
        l.rotate(this->rot_axis, this->rot_point, -this->theta);
    }


    /* If the ray is leaving the ellipse change the 
	   direction of the line */
    if ((h == in_out) || (h == cav_in))
    {
        l.opposite_dir();
    }
    
    return l;
}


/***********************************************************
************************************************************/
point_t ellipse::get_random_point(const int i) const
{
    fp_t x_h8 = this->r/4.0;
    fp_t y_h8 = this->r/4.0;
    fp_t z_h8 = this->h/4.0;
    
    fp_t x = this->get_x0() + ((fp_t) (i     & 0x3) * x_h8) + (gen_random_mersenne_twister() * x_h8);
    fp_t y = this->get_y0() + ((fp_t)((i>>2) & 0x3) * y_h8) + (gen_random_mersenne_twister() * y_h8);
    fp_t z = this->get_z0() + ((fp_t)((i>>2) & 0x3) * z_h8) + (gen_random_mersenne_twister() * z_h8);
    
    return point_t(x,y,z);
}
