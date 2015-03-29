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
point_t ray::calculate_destination(const float d)
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
ray ray::rotate(const vector_t &r, const point_t &c, const float theta) const
{
    /* Rotate the origin into the new coordinate system */
    point_t p = this->ogn;
    point_t q(0.0f, 0.0f, 0.0f);
   
    float costheta = cos_lut.get_cos(theta);
    float sintheta = cos_lut.get_sin(theta);
    
    /* Transform r into the origin */
    p -= c;

    /* Rotate about r */
    q.x += (costheta + (1.0f - costheta) * r.x * r.x) * p.x;
    q.x += ((1.0f - costheta) * r.x * r.y - r.z * sintheta) * p.y;
    q.x += ((1.0f - costheta) * r.x * r.z + r.y * sintheta) * p.z;

    q.y += ((1.0f - costheta) * r.x * r.y + r.z * sintheta) * p.x;
    q.y += (costheta + (1.0f - costheta) * r.y * r.y) * p.y;
    q.y += ((1.0f - costheta) * r.y * r.z - r.x * sintheta) * p.z;

    q.z += ((1.0f - costheta) * r.x * r.z - r.y * sintheta) * p.x;
    q.z += ((1.0f - costheta) * r.y * r.z + r.x * sintheta) * p.y;
    q.z += (costheta + (1.0f - costheta) * r.z * r.z) * p.z;
    
    
    /* Move the point up a unit distance */
    point_t q2(0.0f, 0.0f, 0.0);
    p += this->dir;
    
    /* Rotate about r */
    q2.x += (costheta + (1.0f - costheta) * r.x * r.x) * p.x;
    q2.x += ((1.0f - costheta) * r.x * r.y - r.z * sintheta) * p.y;
    q2.x += ((1.0f - costheta) * r.x * r.z + r.y * sintheta) * p.z;

    q2.y += ((1.0f - costheta) * r.x * r.y + r.z * sintheta) * p.x;
    q2.y += (costheta + (1.0f - costheta) * r.y * r.y) * p.y;
    q2.y += ((1.0f - costheta) * r.y * r.z - r.x * sintheta) * p.z;

    q2.z += ((1.0f - costheta) * r.x * r.z - r.y * sintheta) * p.x;
    q2.z += ((1.0f - costheta) * r.y * r.z + r.x * sintheta) * p.y;
    q2.z += (costheta + (1.0f - costheta) * r.z * r.z) * p.z;
    
    /* Convert back to gradiant */
    q2 -= q;
    
    /* Move points back to where they came from */
    q  += c;
    
    return ray(q,q2.x,q2.y,q2.z);
}


/**********************************************************
 find_rays returns the number of shadow rays generate and 
 a list of rays from the rays destination to the object l.
 
 The rays to the object l are constructed using the rays
 destination and the centre of the object. Multiple rays 
 may be created if soft shadows are enabled.
**********************************************************/
float ray::find_rays(ray rays[], const light &l, const hit_t h) const
{
    /* Adjust the start point of the shadow ray a small distance a long the surface normal */
    const point_t start(offset_start_point(1));
#ifdef SOFT_SHADOW
    int nr_rays = std::max((static_cast<int>(SOFT_SHADOW / this->componant)), 1);
#else
    int nr_rays = 1;
#endif
    nr_rays = l.find_rays(rays, start, nr_rays);
    
    /* Return the number of shadow rays */
    return static_cast<float>(nr_rays);
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
float ray::reflect(ray rays[], const point_t &n, const float r, const float dr) const
{
    /* Check the ray will be strong enough to continue */
    
    const float refl_power = this->magn * r;
    assert(refl_power <  1.1f);
    assert(refl_power > -0.1f);
    if (refl_power <= MIN_REFLECTIVE_POWER)
    {
        return 0.0;
    }
    
    /* Cos(angle between normal and the ray) */
    float   ray_dot_normal  = 2.0f * dot_product(this->dir, n);
    point_t ref             = this->dir - n * ray_dot_normal;

    /* Move a little way along the ray */
    point_t start_point = offset_start_point(1);

#ifdef DIFFUSE_REFLECTIONS
    /* Check to see if the object has a diffuse reflection */
    if (dr == 0.0f)
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
    for (int i = 0; i < nr_rays; ++i)
    {
        /* Pick random offsets */
        float r_off = gen_random_mersenne_twister() * dr;
        float a_off = gen_random_mersenne_twister() * (2.0f * PI);
        float x_off = r_off * cos_lut.get_cos(a_off);
        float y_off = r_off * cos_lut.get_sin(a_off);

        /* Scale perpendicular vectors */
        point_t perpen_scaled = perpen * x_off;
        point_t cross_scaled  = cross  * y_off;

        /* Offset direction */
        point_t diff = ref + perpen_scaled + cross_scaled;
        
        /* Create a new ray */
        rays[i].set_up(start_point, diff.x, diff.y, diff.z, refl_power, new_comp);
    }
    
    /* Return the number of rays created */
    return static_cast<float>(nr_rays);
#else

    rays[0].set_up(start_point, ref.x, ref.y, ref.z, refl_power);
    return 1.0f;
#endif /* #ifdef DIFFUSE_REFLECTIONS */
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
float ray::refract(ray rays[], const point_t &n, const float t, float ri, const hit_t h, const float dr) const 
{
    /* Check the ray will be strong enough to continue */
    const float refr_power = this->magn * t;
    assert(refr_power <  1.1f);
    assert(refr_power > -0.1f);
    if (refr_power <= MIN_REFLECTIVE_POWER)
    {
        return 0.0f;
    }
    
    /* Cos(angle between normal and the ray) */
    float ray_dot_normal = -dot_product(this->dir, n);
                            
    /* sin^2(x) = 1 - cos^2(x) */
    /* cos(refratced angle) */
	if (h == hit_t::out_in)
	{
        ri = 1.0f / ri;
    }

    float cosT2 = 1.0f - ri * ri * (1.0f - ray_dot_normal * ray_dot_normal);
    
    /* Check critical angle */
    /* Beware of total internal reflection!!!!!! */
    if (cosT2 < 0.0f)
    {
        cosT2 = -cosT2;
        return 0.0f;
    }

    /* Convert back to gradients */
	float offset = (ri * ray_dot_normal - std::sqrt(cosT2));
    point_t ref = (ri * this->dir) + offset * n;

    /* Move a little way along the ray */
    point_t start_point = offset_start_point(-1);

#ifdef DIFFUSE_REFLECTIONS
    /* Check to see if the object has a diffuse refraction */
    if (dr == 0.0f)
    {
        rays[0].set_up(start_point, ref.x, ref.y, ref.z, refr_power, this->componant);
        return 1.0f;
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
    for (int i = 0; i < nr_rays; i++)
    {
        /* Pick random offsets */
        float r_off = gen_random_mersenne_twister() * dr;
        float a_off = gen_random_mersenne_twister() * (2.0f * PI);
        float x_off = r_off * cos_lut.get_cos(a_off);
        float y_off = r_off * cos_lut.get_sin(a_off);

        /* Scale perpendicular vectors */
        point_t perpen_scaled = perpen * x_off;
        point_t cross_scaled  = cross  * y_off;

        /* Offset direction */
        point_t diff = ref + perpen_scaled + cross_scaled;
        
        /* Create a new ray */
        rays[i].set_up(start_point, diff.x, diff.y, diff.z, refr_power, new_comp);
    }

    return static_cast<float>(nr_rays);
#else
    
    rays[0].set_up(start_point, ref.x, ref.y, ref.z, refr_power);
    return 1.0f;
#endif /* #ifdef DIFFUSE_REFLECTIONS */
}


///**********************************************************
// 
//**********************************************************/
//ray ray::refract(const line *const n, const hit_t h, float ri, const float a) const 
//{
//    /* Cos(angle between normal and the ray) */
//    float ray_dot_normal = -((this->x_grad * n->get_x_grad()) + 
//                            (this->y_grad * n->get_y_grad()) + 
//                            (this->z_grad * n->get_z_grad()));
//                            
//    /* sin^2(x) = 1 - cos^2(x) */
//    /* cos(refratced angle) */
//	if (h == out_in)
//	{
//        ri = 1.0f / ri;
//    }
//
//    float cosT2 = 1.0f - ri * ri * (1.0f - ray_dot_normal * ray_dot_normal);
//    
//    /* Check critical angle */
//    /* Beware of total internal reflection!!!!!! */
////    if (cosT2 <= 0.0f)
////    {
////        return ray(this->dst, 0.0f, 0.0f, 0.0f, 0.0f);
////    }
//    /* Check critical angle */
//    if (cosT2 < 0.0f)
//    {
//        cosT2 = -cosT2;
//    }
//
//    /* Convert back to gradients */
//	float offset = (ri * ray_dot_normal - sqrt(cosT2));
//    float x_ref  = (ri * this->x_grad) + offset * n->get_x_grad();
//    float y_ref  = (ri * this->y_grad) + offset * n->get_y_grad();
//    float z_ref  = (ri * this->z_grad) + offset * n->get_z_grad();
//
//    /* Move a little way along the ray */
//    point_t start_point(this->dst);
//    start_point.x += 10.0f * this->x_grad * EPSILON;
//    start_point.y += 10.0f * this->y_grad * EPSILON;
//    start_point.z += 10.0f * this->z_grad * EPSILON;
//    
//    ray r(start_point, x_ref, y_ref, z_ref, this->magn);
//    r.set_refractive_index(ri);
//    r.set_absorb(a);
//    return r;
//}
}; /* namespace raptor_raytracer */
