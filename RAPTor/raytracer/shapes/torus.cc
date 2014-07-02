/***********************************************************
 Implementation of the shape virtual functions for a torus
************************************************************/

#include "torus.h"


torus::torus(material *const m, point_t c, fp_t r, fp_t a, vector_t n, fp_t theta, bool l) : 
    shape(m,c,l), rot_axis(n), rot_point(c), r(r), a(a), sr(r + a), theta(theta)
{
    if ((rot_axis.x || rot_axis.y || rot_axis.z) & (theta != 0.0))
    {
        this->rotate = true;

        /* Normalise rot_axis */
        fp_t size = sqrt((rot_axis.x * rot_axis.x) + (rot_axis.y * rot_axis.y) + (rot_axis.z * rot_axis.z));
        assert(size > 0.0);
        rot_axis.x /= size;
        rot_axis.y /= size;
        rot_axis.z /= size;
        
        /* Rotate the extremities of the torus */
        point_t p(2.0*this->sr, 2.0*this->sr, 2.0*this->a);
        point_t q(0.0,0.0,0.0);

        fp_t costheta = cos(this->theta);
        fp_t sintheta = sin(this->theta);

        /* Rotate about rot_axis */
        q.x += fabs((costheta + (1.0 - costheta) * this->rot_axis.x * this->rot_axis.x) * p.x);
        q.x += fabs(((1.0 - costheta) * this->rot_axis.x * this->rot_axis.y - this->rot_axis.z * sintheta) * p.y);
        q.x += fabs(((1.0 - costheta) * this->rot_axis.x * this->rot_axis.z + this->rot_axis.y * sintheta) * p.z);

        q.y += fabs(((1 - costheta) * this->rot_axis.x * this->rot_axis.y + this->rot_axis.z * sintheta) * p.x);
        q.y += fabs((costheta + (1.0 - costheta) * this->rot_axis.y * this->rot_axis.y) * p.y);
        q.y += fabs(((1.0 - costheta) * this->rot_axis.y * this->rot_axis.z - this->rot_axis.x * sintheta) * p.z);

        q.z += fabs(((1.0 - costheta) * this->rot_axis.x * this->rot_axis.z - this->rot_axis.y * sintheta) * p.x);
        q.z += fabs(((1.0 - costheta) * this->rot_axis.y * this->rot_axis.z + this->rot_axis.x * sintheta) * p.y);
        q.z += fabs((costheta + (1.0 - costheta) * this->rot_axis.z * this->rot_axis.z) * p.z);

        this->set_bounding_box(point_t((c.x + q.x), (c.y + q.y), (c.z + q.z)), point_t((c.x - q.x), (c.y - q.y), (c.z - q.z)));
    }
    else
    {
        this->rotate = false;
        this->set_bounding_box(point_t((c.x + this->sr), (c.y + this->sr), (c.z + this->a)), point_t((c.x - this->sr), (c.y - this->sr), (c.z - this->a)));
    }
}


/***********************************************************
 sturmian_roots uses sturm's theorium to return the number
 of roots between two points. sturmian_roots can be used
 to find a single root, but with only linear convergence and
 lower speed them a secant root finder
 
 REVISIT -- Currently has significant error leading to 
            failure in the later root finders
************************************************************/
int torus::sturmian_roots(const ray *const r, fp_t bottom_bracket, fp_t top_bracket) const
{
    fp_t x_dist = r->get_x0() - this->get_x0();
    fp_t y_dist = r->get_y0() - this->get_y0();
    fp_t z_dist = r->get_z0() - this->get_z0();
	fp_t r_sq = this->r * this->r;
	fp_t a_sq = this->a * this->a;
	
    fp_t x = x_dist + (bottom_bracket * r->get_x_grad());
    fp_t y = y_dist + (bottom_bracket * r->get_y_grad());
    fp_t z = z_dist + (bottom_bracket * r->get_z_grad());
    fp_t x_sq = x * x;
    fp_t y_sq = y * y;
    fp_t z_sq = z * z;
  
    /* Evaluate f(x) */
//    fp_t part0 = this->r - sqrt(x_sq + y_sq);//(x_sq + y_sq + z_sq + r_sq - a_sq);
//    fp_t f0 = (part0 * part0) + z_sq - a_sq;// - (4.0 * r_sq * (x_sq + y_sq));
    fp_t part0 = (x_sq + y_sq + z_sq - (a_sq - r_sq));
    fp_t f0 = (part0 * part0) + (4.0 * r_sq * (z_sq - a_sq));

    /* Evaluate f`(x) */
    fp_t dx = r->get_x_grad() * x;
    fp_t dy = r->get_y_grad() * y;
    fp_t dz = r->get_z_grad() * z;
//    fp_t f1 = (4.0 * (dx + dy + dz) * (x_sq + y_sq + z_sq + r_sq - a_sq)) - (8.0 * r_sq * (dx + dy));
    fp_t f1 = (4.0 * (dx + dy + dz) * (x_sq + y_sq + z_sq - (a_sq - r_sq))) - (8.0 * r_sq * dz);
    
    fp_t f2 = -fmod(f1, f0);
    fp_t f3 = -fmod(f2, f1);
    fp_t f4 = -fmod(f3, f2);

//    cout << "Sturm bottom evaluation: " << f0 << ", " << f1 << ", " << f2 << ", " << f3 << ", " << f4 << endl;
    int sign_changes_bottom = (f0 * f1) < 0.0;
    sign_changes_bottom += (f1 * f2) < 0.0;
    sign_changes_bottom += (f2 * f3) < 0.0;
    sign_changes_bottom += (f3 * f4) < 0.0;
    
//    cout << "Bottom roots: " << sign_changes_bottom << endl;
    
    x = x_dist + (top_bracket * r->get_x_grad());
    y = y_dist + (top_bracket * r->get_y_grad());
    z = z_dist + (top_bracket * r->get_z_grad());
    x_sq = x * x;
    y_sq = y * y;
    z_sq = z * z;
  
    /* Evaluate f(x) */
//    part0 = this->r - sqrt(x_sq + y_sq);//(x_sq + y_sq + z_sq + r_sq - a_sq);
//    f0 = (part0 * part0) + z_sq - a_sq;// - (4.0 * r_sq * (x_sq + y_sq));
    part0 = (x_sq + y_sq + z_sq - (a_sq - r_sq));
    f0 = (part0 * part0) + (4.0 * r_sq * (z_sq - a_sq));

    /* Evaluate f`(x) */
    dx = r->get_x_grad() * x;
    dy = r->get_y_grad() * y;
    dz = r->get_z_grad() * z;
//    f1 = (4.0 * (dx + dy + dz) * (x_sq + y_sq + z_sq + r_sq - a_sq)) - (8.0 * r_sq * (dx + dy));
    f1 = (4.0 * (dx + dy + dz) * (x_sq + y_sq + z_sq - (a_sq - r_sq))) - (8.0 * r_sq * dz);
    
    f2 = -fmod(f1, f0);
    f3 = -fmod(f2, f1);
    f4 = -fmod(f3, f2);

//    cout << "Sturm top evaluation: " << f0 << ", " << f1 << ", " << f2 << ", " << f3 << ", " << f4 << endl;
    
    int sign_changes_top = (f0 * f1) < 0.0;
    sign_changes_top += (f1 * f2) < 0.0;
    sign_changes_top += (f2 * f3) < 0.0;
    sign_changes_top += (f3 * f4) < 0.0;
    
//    cout << "Top roots: " << sign_changes_top << endl;
    int roots = sign_changes_bottom - sign_changes_top;
//    assert(roots >= 0);
    assert(roots <= 4);

    return roots;
}

/***********************************************************
 secant_roots uses the secant method to find a single 
 bracketed. This method has linear convergence. The position
 of a single root will be returned. If the algorithm cannot 
 converge within 'SECANT_ITERATIONS' iterations then 
 'DOUBLE_MAX' is returned
************************************************************/
fp_t torus::secant_roots(const ray *const r, fp_t bottom_bracket, fp_t top_bracket) const
{
    int iter = 0;
    fp_t d = bottom_bracket + (top_bracket - bottom_bracket)/2.0;
    
    fp_t x_dist = r->get_x0() - this->get_x0();
    fp_t y_dist = r->get_y0() - this->get_y0();
    fp_t z_dist = r->get_z0() - this->get_z0();
	fp_t r_sq = this->r * this->r;
	fp_t a_sq = this->a * this->a;
    
    while (iter < SECANT_ITERATIONS)
    {
        fp_t x = x_dist + (d * r->get_x_grad());
        fp_t y = y_dist + (d * r->get_y_grad());
        fp_t z = z_dist + (d * r->get_z_grad());
        fp_t x_sq = x * x;
        fp_t y_sq = y * y;
        fp_t z_sq = z * z;
  
        fp_t part0 = (x_sq + y_sq + z_sq + r_sq - a_sq);
        fp_t fn = (part0 * part0) - (4.0 * r_sq * (x_sq + y_sq));
//        fp_t part0 = this->r - sqrt(x_sq + y_sq);   // REVISIT -- These equations allow fewer iterations
//        fp_t fn = (part0 * part0) + z_sq - a_sq;    //            but may be slower

        if (abs(fn) < SECANT_ERROR_LIMIT)
        {
            return d;
        }

        if (fn > 0.0)
        {
            bottom_bracket = d;
        }
        else
        {
            top_bracket = d;
        }
        d = bottom_bracket + (top_bracket - bottom_bracket)/2.0;
        iter++;
    }
    
    return DOUBLE_MAX;
}


/***********************************************************
 is_intersecting returns the distance along the line l that 
 the torus and the line intersect. If the shere and the line 
 do not intersect 'DOUBLE_MAX' is be returned.
 
 The distance to intersection is calculated by substituting 
 the equation for a line (x0 + grad*d) in 3-D into the 
 equation of a circle (x^2 +y^2 + z^2 = r^2) and solving as
 a quadratic equation.
 
 If the sphere is intersected the intersections with the 
 sphere are used as a start point for sturmian root 
 bracketting. When a single roots is brackets Newton-Raphson
 or the secant method can be used to find the root.
 
    Equation of a torus -
	    (x^2 + y^2 + z^2 + c^2 - a^2)^2 - 4c^2(x^2 + y^2) = 0;
	   
    Derivative of a torus -
	    4(x + y + z)(x^2 + y^2 + z^2 + c^2 - a^2) - 8c^2(x + y) = 0;
************************************************************/
fp_t torus::is_intersecting(const ray *const r, hit_t *const h) const
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

    fp_t c = (x_dist   * x_dist) +
               (y_dist   * y_dist) +
               (z_dist   * z_dist) - 
               (this->sr * this->sr); 

    fp_t intersection = (b * b) - (a * c);

    /* Return if there is no intersection 
            -ve = no intersection
              0 =  1 intersection
            +ve =  2 intersection */
    if (intersection < 0.0)
    {
        return DOUBLE_MAX;
    }
    
    /* Initial guess for Strumian root bracketing */
    intersection = sqrt(intersection);
    fp_t dist_to_inter = (b - intersection) / a;
    fp_t bottom_bracket;
    if (dist_to_inter < 0.0)
    {
        /* If there is a negative intersection us 0 as the lower bracket */
        bottom_bracket = DOUBLE_ERR;
    }
	else
	{
        /* If the nearest intersection is positive use it as the lower bracket */
        bottom_bracket = 1.0;
	}
    /* Use the furthest intersection as the upper bracket */
    fp_t top_bracket = (b + intersection) / a;
    if (top_bracket < 0.0)
    {
        return DOUBLE_MAX;
    }
    
    /* Rotate the ray into the toruses co-ordinate system */
    ray rotated;
    if (this->rotate)
    {
        rotated = r->rotate(this->rot_axis, this->rot_point, this->theta);
    }
    else
    {
        rotated = *r;
    }
    x_dist = rotated.get_x0() - this->get_x0();
    y_dist = rotated.get_y0() - this->get_y0();
    z_dist = rotated.get_z0() - this->get_z0();
    
    /* Sturmian root bracketing */
    /* Check the torus and not just the sphere are intersected */
    int roots = this->sturmian_roots(&rotated, bottom_bracket, top_bracket);
    if (roots <= 0)
    {
        return DOUBLE_MAX;
    }
    
    // Pick weather the ray is entering or leaving
    if (roots & 0x1)
    {
        *h = out_in;
    }
    else
    {
        *h = in_out;
    }
    
    fp_t d;
    fp_t top_limit    = top_bracket;
    while (roots != 1)
    {
        /* Halve the interval */
        top_bracket = bottom_bracket + (top_limit - bottom_bracket)/2.0;
        roots = this->sturmian_roots(&rotated, bottom_bracket, top_bracket);
        
        /* If no roots exsist move into the upper half */
        if (roots == 0)
        {
            bottom_bracket = top_bracket;
        }
        /* Else set the limits to the lower half */
        else
        {
            top_limit = top_bracket;
        }

        assert((top_limit - bottom_bracket) > DOUBLE_ERR);
    }
    d = bottom_bracket + (top_limit - bottom_bracket)/2.0;
    
//    return this->secant_roots(&rotated, bottom_bracket, top_bracket);

    /* Solve using Newton Raphson - d+1 = d - (f(d) / f`(d)) */
	int iter = 0;
    fp_t r_sq = this->r * this->r;
    fp_t a_sq = this->a * this->a;
	while (iter < NEWTON_RAPHSON_ITERATIONS)
	{
		fp_t x = x_dist + (d * rotated.get_x_grad());
		fp_t y = y_dist + (d * rotated.get_y_grad());
		fp_t z = z_dist + (d * rotated.get_z_grad());
		fp_t x_sq = x * x;
		fp_t y_sq = y * y;
		fp_t z_sq = z * z;
	
        fp_t part0 = (x_sq + y_sq + z_sq + r_sq - a_sq);
        fp_t fn = (part0 * part0) - (4.0 * r_sq * (x_sq + y_sq));

		if (fabs(fn) < NEWTON_RAPHSON_ERROR_LIMIT)
		{
            if (d < bottom_bracket) 
            {
                d = this->secant_roots(&rotated, bottom_bracket, top_bracket);
            }
			return d;
		}

        fp_t dx = rotated.get_x_grad() * x;
        fp_t dy = rotated.get_y_grad() * y;
        fp_t dz = rotated.get_z_grad() * z;
        fp_t dfn = (4.0 * (dx + dy + dz) * part0) - (8.0 * r_sq * (dx + dy));

		d -= (fn / dfn);
		iter++;
	}
    
	return DOUBLE_MAX;
}


/***********************************************************
 normal_at_point returns a line normal to the torus at the 
 point p.
 
 The normal is calculated as a line from the centre of the 
 ring of the torus through the point p. The line create 
 starts from the point p.
************************************************************/
line torus::normal_at_point(ray *const r, const hit_t h) const
{
    point_t dst;
    
    /* Move the end point of the ray into the toruses co-ordinate system */
    if(this->rotate)
    {
        dst = r->rotate_dst(this->rot_axis, this->rot_point, this->theta);
    }
    else
    {
        dst = r->get_dst();
    }

    /* Start point of line is given by point p */ 
    /* Direction is given by the line from the centre of the torus to point p */
    fp_t x_dist = dst.x - this->get_x0();
    fp_t y_dist = dst.y - this->get_y0();
    
    fp_t dist = sqrt((x_dist * x_dist) + (y_dist * y_dist));
    
    fp_t dist_ratio = this->r/dist;
    
    fp_t ring_x = x_dist * dist_ratio;
    fp_t ring_y = y_dist * dist_ratio;
    
    fp_t x_dir = (x_dist - ring_x)        /this->a;
    fp_t y_dir = (y_dist - ring_y)        /this->a;
    fp_t z_dir = (dst.z  - this->get_z0())/this->a;
    
    /* Create the normal and move into the rays co-ordinate system */
    line l(dst, x_dir, y_dir, z_dir);
    if(this->rotate)
    {
        l.rotate(this->rot_axis, this->rot_point, -this->theta);
    }

    /* If the ray is leaving the torus change the 
	   direction of the line */
    if (h == in_out)
    {
        l.opposite_dir();
    }
    
    return l;
}


/***********************************************************
************************************************************/
point_t torus::get_random_point(const int i) const
{
    fp_t x_h8 = this->r/4.0;
    fp_t y_h8 = this->r/4.0;
    fp_t z_h8 = this->r/4.0;
    
    fp_t x = this->get_x0() + ((fp_t) (i     & 0x3) * x_h8) + (gen_random_mersenne_twister() * x_h8);
    fp_t y = this->get_y0() + ((fp_t)((i>>2) & 0x3) * y_h8) + (gen_random_mersenne_twister() * y_h8);
    fp_t z = this->get_z0() + ((fp_t)((i>>2) & 0x3) * z_h8) + (gen_random_mersenne_twister() * z_h8);
    
    return point_t(x,y,z);
}
