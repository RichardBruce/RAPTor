#include "ray.h"
#include "triangle.h"
#include "light.h"


namespace raptor_raytracer
{
/**********************************************************
 calculate_destination returns the 3-D co-ordinate of the 
 ray after it has travelled a distance d.
 
 The 3-D co-ordinate is calculated by substituting the 
 distance d into the equation of a line.
**********************************************************/
point_t ray::calculate_destination(const fp_t d)
{
    this->dst = this->ogn + (this->dir * d);
    this->length = d;
    return this->dst;
}


/**********************************************************
 rotate returns a ray rotated theta radians about vector r,
 through point c.
 
 A general rotation algorithm is used. The cos_lut is used
 for the sin and cos function calls.
**********************************************************/
ray ray::rotate(const vector_t &r, const point_t &c, const fp_t theta) const
{
    /* Rotate the origin into the new coordinate system */
    point_t p = this->ogn;
    point_t q(0.0,0.0,0.0);
   
    fp_t costheta = cos_lut.get_cos(theta);
    fp_t sintheta = cos_lut.get_sin(theta);
    
    /* Transform r into the origin */
    p -= c;

    /* Rotate about r */
    q.x += (costheta + ((fp_t)1.0 - costheta) * r.x * r.x) * p.x;
    q.x += (((fp_t)1.0 - costheta) * r.x * r.y - r.z * sintheta) * p.y;
    q.x += (((fp_t)1.0 - costheta) * r.x * r.z + r.y * sintheta) * p.z;

    q.y += (((fp_t)1.0 - costheta) * r.x * r.y + r.z * sintheta) * p.x;
    q.y += (costheta + ((fp_t)1.0 - costheta) * r.y * r.y) * p.y;
    q.y += (((fp_t)1.0 - costheta) * r.y * r.z - r.x * sintheta) * p.z;

    q.z += (((fp_t)1.0 - costheta) * r.x * r.z - r.y * sintheta) * p.x;
    q.z += (((fp_t)1.0 - costheta) * r.y * r.z + r.x * sintheta) * p.y;
    q.z += (costheta + ((fp_t)1.0 - costheta) * r.z * r.z) * p.z;
    
    
    /* Move the point up a unit distance */
    point_t q2(0.0,0.0,0.0);
    p += this->dir;
    
    /* Rotate about r */
    q2.x += (costheta + ((fp_t)1.0 - costheta) * r.x * r.x) * p.x;
    q2.x += (((fp_t)1.0 - costheta) * r.x * r.y - r.z * sintheta) * p.y;
    q2.x += (((fp_t)1.0 - costheta) * r.x * r.z + r.y * sintheta) * p.z;

    q2.y += (((fp_t)1.0 - costheta) * r.x * r.y + r.z * sintheta) * p.x;
    q2.y += (costheta + ((fp_t)1.0 - costheta) * r.y * r.y) * p.y;
    q2.y += (((fp_t)1.0 - costheta) * r.y * r.z - r.x * sintheta) * p.z;

    q2.z += (((fp_t)1.0 - costheta) * r.x * r.z - r.y * sintheta) * p.x;
    q2.z += (((fp_t)1.0 - costheta) * r.y * r.z + r.x * sintheta) * p.y;
    q2.z += (costheta + ((fp_t)1.0 - costheta) * r.z * r.z) * p.z;
    
    /* Convert back to gradiant */
    q2 -= q;
    
    /* Move points back to where they came from */
    q  += c;
    
    return ray(q,q2.x,q2.y,q2.z);
}


/**********************************************************
 rotate_dst returns the rays destination rotated theta 
 radians about vector r, through point c.
 
 A general rotation algorithm is used. The cos_lut is used
 for the sin and cos function calls.
**********************************************************/
point_t ray::rotate_dst(const vector_t &r, const point_t &c, const fp_t theta) const
{
    /* Rotate the origin into the new coordinate system */
    point_t p = this->dst;
    point_t q(0.0,0.0,0.0);
   
    fp_t costheta = cos_lut.get_cos(theta);
    fp_t sintheta = cos_lut.get_sin(theta);
    
    /* Transform r into the origin */
    p -= c;

    /* Rotate about r */
    q.x += (costheta + ((fp_t)1.0 - costheta) * r.x * r.x) * p.x;
    q.x += (((fp_t)1.0 - costheta) * r.x * r.y - r.z * sintheta) * p.y;
    q.x += (((fp_t)1.0 - costheta) * r.x * r.z + r.y * sintheta) * p.z;

    q.y += (((fp_t)1.0 - costheta) * r.x * r.y + r.z * sintheta) * p.x;
    q.y += (costheta + ((fp_t)1.0 - costheta) * r.y * r.y) * p.y;
    q.y += (((fp_t)1.0 - costheta) * r.y * r.z - r.x * sintheta) * p.z;

    q.z += (((fp_t)1.0 - costheta) * r.x * r.z - r.y * sintheta) * p.x;
    q.z += (((fp_t)1.0 - costheta) * r.y * r.z + r.x * sintheta) * p.y;
    q.z += (costheta + ((fp_t)1.0 - costheta) * r.z * r.z) * p.z;
    
    
    /* Move points back to where they came from */
    q  += c;
    
    return q;
}


/**********************************************************
 find_rays returns the number of shadow rays generate and 
 a list of rays from the rays destination to the object l.
 
 The rays to the object l are constructed using the rays
 destination and the centre of the object. Multiple rays 
 may be created if soft shadows are enabled.
**********************************************************/
fp_t ray::find_rays(ray rays[], const light &l, const line &n, const hit_t h) const
{
#ifdef SINGLE_PRECISION
    /* Adjust the start point of the shadow ray a small distance a long the surface normal */
    point_t start = this->offset_start_point(n, 1);
#else
    point_t start = this->dst;
#endif /* #ifdef SINGLE_PRECISION */
    
#ifdef SOFT_SHADOW
    int nr_rays = max(((int)(SOFT_SHADOW/this->componant)), 1);
#else
    int nr_rays = 1;
#endif
    nr_rays = l.find_rays(rays, start, nr_rays);
    
    /* Return the number of shadow rays */
    return static_cast<fp_t>(nr_rays);
}
        

/**********************************************************
 reflect returns the number of reflection rays and a list
 of reflected rays.
 
 The direction of the rays is calculated with the surface 
 normal n. The intensity of the rays is calculated by the
 reflection co-efficient r. Multiple rays may be generated
 if diffuse reflections are enable and the diffuse 
 reflection co-efficient dr is greater than 0. dr gives the 
 spread of the rays.
**********************************************************/
fp_t ray::reflect(ray rays[], const line &n, const fp_t r, const fp_t dr) const
{
    /* Check the ray will be strong enough to continue */
    
    assert(this->magn <  1.1);
    assert(this->magn > -0.1);
    fp_t refl_power = this->magn * r;
    if (refl_power <= MIN_REFLECTIVE_POWER)
    {
        return 0.0;
    }
    
    /* Cos(angle between normal and the ray) */
    fp_t    ray_dot_normal  = (fp_t)2.0 * dot_product(this->dir, n.get_dir());
    point_t ref             = this->dir - n.get_dir() * ray_dot_normal;

    /* Move a little way along the ray */
#ifndef SINGLE_PRECISION
    point_t start_point(this->dst);
    start_point += ref * (fp_t)10.0 * EPSILON;
#else
    point_t start_point = this->offset_start_point(n, 1);
#endif

#ifdef DIFFUSE_REFLECTIONS
    /* Check to see if the object has a diffuse reflection */
    if (dr == (fp_t)0.0)
    {
        rays[0].set_up(start_point, ref.x, ref.y, ref.z, refl_power, this->componant);
        return 1.0;
    }
    
    /* Adjust importance sampling factor */
    int new_comp = this->componant << 2;

    /* Orthogonal vector to the reflection */    
    point_t perpen(perpendicular(ref));
    
    /* Orthogonal vector to the above and the reflection */
    point_t cross;
    cross_product(ref, perpen, &cross);
    
    /* Number of rays to create */
    int nr_rays = max(((int)DIFFUSE_REFLECTIONS/this->componant), 1);
    for (int i=0; i<nr_rays; i++)
    {
        /* Pick random offsets */
        fp_t r_off = gen_random_mersenne_twister() * dr;
        fp_t a_off = gen_random_mersenne_twister() * ((fp_t)2.0 * PI);
        fp_t x_off = r_off * cos_lut.get_cos(a_off);
        fp_t y_off = r_off * cos_lut.get_sin(a_off);

        /* Scale perpendicular vectors */
        point_t perpen_scaled = perpen * x_off;
        point_t cross_scaled  = cross  * y_off;

        /* Offset direction */
        point_t diff = ref + perpen_scaled + cross_scaled;
        
        /* Create a new ray */
        rays[i].set_up(start_point, diff.x, diff.y, diff.z, refl_power, new_comp);
    }
    
    /* Return the number of rays created */
    return fp_t(nr_rays);
#else

    rays[0].set_up(start_point, ref.x, ref.y, ref.z, refl_power);
    return 1.0;
#endif /* #ifdef DIFFUSE_REFLECTIONS */
#else

    return 0.0;
}


/**********************************************************
 refract returns the number of refraction rays and a list
 of refracted rays.
 
 The direction of the rays is calculated with the surface 
 normal n and the refractive index ri. h gives the direction
 of the incident ray relative to the material and is used to 
 invert the refractive index for exitent rays. Multiple rays 
 may be generated if diffuse refractions are enable and the 
 diffuse refraction co-efficient dr is greater than 0. dr 
 gives the spread of the rays.
**********************************************************/
fp_t ray::refract(ray rays[], const line &n, const fp_t t, fp_t ri, const hit_t h, const fp_t dr) const 
{
    /* Check the ray will be strong enough to continue */
    fp_t refr_power = this->magn * t;
    if (refr_power <= MIN_REFLECTIVE_POWER)
    {
        return 0.0;
    }
    
    /* Cos(angle between normal and the ray) */
    fp_t ray_dot_normal = -dot_product(this->dir, n.get_dir());
                            
    /* sin^2(x) = 1 - cos^2(x) */
    /* cos(refratced angle) */
	if (h == out_in)
	{
        ri = (fp_t)1.0/ri;
    }

    fp_t cosT2 = (fp_t)1.0 - ri * ri * ((fp_t)1.0 - ray_dot_normal * ray_dot_normal);
    
    /* Check critical angle */
    /* Beware of total internal reflection!!!!!! */
    if (cosT2 < (fp_t)0.0)
    {
        cosT2 = -cosT2;
        return 0.0;
    }

    /* Convert back to gradients */
	fp_t offset = (ri * ray_dot_normal - sqrt(cosT2));
    point_t ref = (ri * this->dir) + offset * n.get_dir();

    /* Move a little way along the ray */
#ifndef SINGLE_PRECISION
    point_t start_point(this->dst);
    start_point += this->dir * (fp_t)10.0 * EPSILON;
#else
    point_t start_point = this->offset_start_point(n, -1);
#endif


#ifdef DIFFUSE_REFLECTIONS
    /* Check to see if the object has a diffuse refraction */
    if (dr == (fp_t)0.0)
    {
        rays[0].set_up(start_point, ref.x, ref.y, ref.z, refr_power, this->componant);
        return 1.0;
    }
    
    /* Adjust importance sampling factor */
    int new_comp = this->componant << 2;

    /* Orthogonal vector to the refraction */    
    point_t perpen = perpendicular(ref);
    
    /* Orthogonal vector to the above and the refraction */
    point_t cross;
    cross_product(ref, perpen, &cross);

    /* Create a number of rays with a little bit of noise added to their direction */
    int nr_rays = max(((int)DIFFUSE_REFLECTIONS/this->componant), 1);
    for (int i=0; i<nr_rays; i++)
    {
        /* Pick random offsets */
        fp_t r_off = gen_random_mersenne_twister() * dr;
        fp_t a_off = gen_random_mersenne_twister() * ((fp_t)2.0 * PI);
        fp_t x_off = r_off * cos_lut.get_cos(a_off);
        fp_t y_off = r_off * cos_lut.get_sin(a_off);

        /* Scale perpendicular vectors */
        point_t perpen_scaled = perpen * x_off;
        point_t cross_scaled  = cross  * y_off;

        /* Offset direction */
        point_t diff = ref + perpen_scaled + cross_scaled;
        
        /* Create a new ray */
        rays[i].set_up(start_point, diff.x, diff.y, diff.z, refr_power, new_comp);
    }

    return fp_t(nr_rays);
#else
    
    rays[0].set_up(start_point, ref.x, ref.y, ref.z, refr_power);
    return 1.0;
#endif /* #ifdef DIFFUSE_REFLECTIONS */
#else

    return 0.0;
}


///**********************************************************
// 
//**********************************************************/
//ray ray::refract(const line *const n, const hit_t h, fp_t ri, const fp_t a) const 
//{
//    /* Cos(angle between normal and the ray) */
//    fp_t ray_dot_normal = -((this->x_grad * n->get_x_grad()) + 
//                            (this->y_grad * n->get_y_grad()) + 
//                            (this->z_grad * n->get_z_grad()));
//                            
//    /* sin^2(x) = 1 - cos^2(x) */
//    /* cos(refratced angle) */
//	if (h == out_in)
//	{
//        ri = (fp_t)1.0/ri;
//    }
//
//    fp_t cosT2 = (fp_t)1.0 - ri * ri * ((fp_t)1.0 - ray_dot_normal * ray_dot_normal);
//    
//    /* Check critical angle */
//    /* Beware of total internal reflection!!!!!! */
////    if (cosT2 <= 0.0)
////    {
////        return ray(this->dst, 0.0, 0.0, 0.0, 0.0);
////    }
//    /* Check critical angle */
//    if (cosT2 < (fp_t)0.0)
//    {
//        cosT2 = -cosT2;
//    }
//
//    /* Convert back to gradients */
//	fp_t offset = (ri * ray_dot_normal - sqrt(cosT2));
//    fp_t x_ref  = (ri * this->x_grad) + offset * n->get_x_grad();
//    fp_t y_ref  = (ri * this->y_grad) + offset * n->get_y_grad();
//    fp_t z_ref  = (ri * this->z_grad) + offset * n->get_z_grad();
//
//    /* Move a little way along the ray */
//    point_t start_point(this->dst);
//    start_point.x += (fp_t)10.0 * this->x_grad * EPSILON;
//    start_point.y += (fp_t)10.0 * this->y_grad * EPSILON;
//    start_point.z += (fp_t)10.0 * this->z_grad * EPSILON;
//    
//    ray r(start_point, x_ref, y_ref, z_ref, this->magn);
//    r.set_refractive_index(ri);
//    r.set_absorb(a);
//    return r;
//}
}; /* namespace raptor_raytracer */
