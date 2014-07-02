#include "phong_shader.h"
#include "raytracer.h"
#include "triangle.h"
#include "light.h"


void phong_shader::generate_rays(const ray_trace_engine &r, ray &i, const line &n, const hit_t h, ray *const rl, ray *const rf, fp_t *const n_rl, fp_t *const n_rf) const
{
    /* For each light request rays */
    for (unsigned int l = 0; l < r.get_scene_lights().size(); ++l)
    {
        r.generate_rays_to_light(i, n, h, l);
    }
    
    return;
}


void phong_shader::shade(const ray_trace_engine &r, ray &i, const line &n, const hit_t h, ext_colour_t *const c, const point_t &vt) const
{
    /* For each light shade the object */
    unsigned int l = 0;
    for (ray_trace_engine::const_light_iterator iter = r.get_scene_lights().begin(); iter != r.get_scene_lights().end(); ++iter)
    {
        /* Query the scene to see if this light is visiable */
        ray illum = r.get_illumination(l++);

        /* Cos(angle between normal and the ray) */
        fp_t shade = dot_product(illum.get_dir(), n.get_dir());
        
        /* Ignore if the surface is facing away from the ray */
        if (shade < (fp_t)0.0)
        {
            continue;
        }
        
        /* Find the reflection of the light ray */
        point_t ref_dir;
        fp_t shade_x2 = (fp_t)2.0 * shade;
        ref_dir.x = illum.get_x_grad() - n.get_x_grad() * shade_x2;
        ref_dir.y = illum.get_y_grad() - n.get_y_grad() * shade_x2;
        ref_dir.z = illum.get_z_grad() - n.get_z_grad() * shade_x2;
    
        /* Cos(angle between refelction and the ray) */
        fp_t ray_dot_reflection = dot_product(i.get_dir(), ref_dir);

        /* Scale the diffuse componant */
        ext_colour_t shade_colour = shade * this->kd;

        /* Calculate and scale the specular componant */
        if (ray_dot_reflection > (fp_t)0.0)
        {
            shade_colour += pow(ray_dot_reflection, this->s) * this->ks;
        }

        /* Add to the overall shading */
        (*c) += (*iter).get_light_intensity(illum.get_dir(), illum.get_length()) * (shade_colour * illum.get_magnitude());
    }
    
    /* Add the ambient colour */
    /* Note Ka is stored in light_intensity in the format ( r, g, b ) */
    (*c) += this->ka;
    
    /* Reflect */
    if (this->rf > (fp_t)0.0)
    {
        ext_colour_t    average;
        ray             rl[REFLECTION_ARRAY_SIZE];
        
        /* Ray reflection member */
        fp_t nr_rays = i.reflect(rl, n, this->rf, this->rfd);
        
        for (int i=0; i<(int)nr_rays; ++i)
        {
            ext_colour_t pixel;
            r.ray_trace(rl[i], &pixel);
            average += pixel;
        }

        /* Average the colours */
        if (nr_rays > (fp_t)0.0)
        {
            (*c) += average * (this->rf / nr_rays);
        }
    }

    /* Refract */
    if (tran > (fp_t)0.0)
    {
        ext_colour_t    average;
        ray             rl[REFLECTION_ARRAY_SIZE];
        
        /* Ray refraction member */
        fp_t nr_rays = i.refract(rl, n, tran, this->ri, h, this->td);
        
        for (int i=0; i<(int)nr_rays; ++i)
        {
            ext_colour_t pixel;
            r.ray_trace(rl[i], &pixel);
            average += pixel;
        }

        /* Average the colours */
        if (nr_rays > (fp_t)0.0)
        {
            (*c) += average * (tran / nr_rays);
        }
    }
}


void phong_shader::combind_secondary_rays(const ray_trace_engine &r, ext_colour_t &c, const ray *const rl, const ray *const rf, const ext_colour_t *const c_rl, const ext_colour_t *const c_rf, const fp_t *const n_rl, const fp_t *const n_rf) const
{
    return;
}
