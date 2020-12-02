#include "phong_shader.h"
#include "raytracer.h"
#include "triangle.h"
#include "light.h"


namespace raptor_raytracer
{
void phong_shader::generate_rays(const ray_trace_engine &r, ray &i, point_t<> *const n, const point_t<> &vt, const hit_t h, secondary_ray_data *const rl, secondary_ray_data *const rf) const
{
    /* For each light request rays */
    for (unsigned int l = 0; l < r.get_scene_lights().size(); ++l)
    {
        r.generate_rays_to_light(i, h, l);
    }
    
    return;
}


void phong_shader::shade(const ray_trace_engine &r, ray &i, const point_t<> &n, const hit_t h, ext_colour_t *const c, const point_t<> &vt) const
{
    /* For each light shade the object */
    unsigned int l = 0;
    for (ray_trace_engine::const_light_iterator iter = r.get_scene_lights().begin(); iter != r.get_scene_lights().end(); ++iter)
    {
        /* Query the scene to see if this light is visiable */
        ray illum = r.get_illumination(l++);

        /* Cos(angle between normal and the ray) */
        float shade = dot_product(illum.get_dir(), n);
        
        /* Ignore if the surface is facing away from the ray */
        if (shade < 0.0f)
        {
            continue;
        }
        
        /* Find the reflection of the light ray */
        point_t<> ref_dir;
        float shade_x2 = 2.0f * shade;
        ref_dir.x = illum.get_x_grad() - n.x * shade_x2;
        ref_dir.y = illum.get_y_grad() - n.y * shade_x2;
        ref_dir.z = illum.get_z_grad() - n.z * shade_x2;
    
        /* Cos(angle between refelction and the ray) */
        float ray_dot_reflection = dot_product(i.get_dir(), ref_dir);

        /* Scale the diffuse componant */
        ext_colour_t shade_colour = shade * this->kd;

        /* Calculate and scale the specular componant */
        if (ray_dot_reflection > 0.0f)
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
    if (this->rf > 0.0f)
    {
        ext_colour_t    average;
        ray             rl[MAX_SECONDARY_RAYS];
        
        /* Ray reflection member */
        float nr_rays = i.reflect(rl, n, this->rf, this->rfd);
        
        for (int i = 0; i< static_cast<int>(nr_rays); ++i)
        {
            ext_colour_t pixel;
            r.ray_trace(rl[i], &pixel);
            average += pixel;
        }

        /* Average the colours */
        if (nr_rays > 0.0f)
        {
            (*c) += average * (this->rf / nr_rays);
        }
    }

    /* Refract */
    if (this->tran > 0.0f)
    {
        ext_colour_t    average;
        ray             rl[MAX_SECONDARY_RAYS];
        
        /* Ray refraction member */
        float nr_rays = i.refract(rl, n, this->tran, this->ri, h, this->td);
        
        for (int i = 0; i< static_cast<int>(nr_rays); ++i)
        {
            ext_colour_t pixel;
            r.ray_trace(rl[i], &pixel);
            average += pixel;
        }

        /* Average the colours */
        if (nr_rays > 0.0f)
        {
            (*c) += average * (this->tran / nr_rays);
        }
    }
}


void phong_shader::combind_secondary_rays(const ray_trace_engine &r, ext_colour_t *const c, const secondary_ray_data &rl, const secondary_ray_data &rf) const
{
    return;
}
}; /* namespace raptor_raytracer */
