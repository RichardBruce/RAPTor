#include "mandelbrot_shader.h"
#include "raytracer.h"
#include "light.h"


namespace raptor_raytracer
{
void mandelbrot_shader::generate_rays(const ray_trace_engine &r, ray &i, point_t *const n, const point_t &vt, const hit_t h, secondary_ray_data *const rl, secondary_ray_data *const rf) const
{
    /* For each light request rays */
    for (unsigned int l = 0; l < r.get_scene_lights().size(); ++l)
    {
        r.generate_rays_to_light(i, h, l);
    }
    
    return;
}


void mandelbrot_shader::shade(const ray_trace_engine &r, ray &i, const point_t &n, const hit_t h, ext_colour_t *const c, const point_t &vt) const
{
    /* For each light shade the object, but only diffusely */
    float total_shade   = 0.0f;
    unsigned int l      = 0;
    for (ray_trace_engine::const_light_iterator iter = r.get_scene_lights().begin(); iter != r.get_scene_lights().end(); ++iter)
    {
        /* Query the scene to see if this light is visiable */
        ray illum = r.get_illumination(l++);

        /* Cos(angle between normal and the ray) */
        float shade = dot_product(illum.get_dir(), n);
        
        /* Ignore if the surface is facing away from the ray */
        if (shade < 0.0)
        {
            continue;
        }
        
        /* Add to the overall shading */
        total_shade += shade * illum.get_magnitude();
    }
    
    float x0;
    float y0;
    
    if ((n.x == 1.0f) || (n.x == -1.0f))
    {
        x0 = fmod(fabs(i.get_z1()),       2.4f) - 1.5f;
        y0 = fmod(fabs(i.get_y1())- 1.5f, 2.4f) - 1.0f;
    }
    else if ((n.z == 1.0f) || (n.z == -1.0f))
    {
        x0 = fmod(fabs(i.get_x1()),       2.4f) - 1.5f;
        y0 = fmod(fabs(i.get_y1())- 1.5f, 2.4f) - 1.0f;
    }
    else
    {
        x0 = fmod(fabs(i.get_x1())- 1.5f, 2.4f) - 1.0f;
        y0 = fmod(fabs(i.get_z1()),       2.4f) - 1.5f;
    }
    
    float x = x0;
    float y = y0;

    bool doomed = false;

WHILE :
    int iteration = 0;
    while ((x*x + y*y <= (10.0f * 10.0f)) && (iteration < this->i))
    {
        float xtemp = x * x - y * y + x0;
        y = 2.0f * x * y + y0;

        x = xtemp;

        ++iteration;
    }

    if ((doomed) && (iteration == this->i))
    {
        *c = ext_colour_t(0.0f, 0.0f, 0.0f);
    }
    else if (iteration == this->i)
    {
        doomed = true;
        if ((n.x == 1.0f) || (n.x == -1.0f))
        {
            x0 = fmod(fabs(i.get_z1())- 1.0f, 2.4f) * 1.5f - 0.4f;
            y0 = fmod(fabs(i.get_y1())- 2.0f, 2.4f) * 1.5f - 0.7f;
        }
        else if ((n.z == 1.0f) || (n.z == -1.0f))
        {
            x0 = fmod(fabs(i.get_x1())- 1.0f, 2.4f) * 1.5f - 0.4f;
            y0 = fmod(fabs(i.get_y1())- 2.0f, 2.4f) * 1.5f - 0.7f;
        }
        else
        {
            x0 = fmod(fabs(i.get_x1())- 2.0f, 2.4f) * 1.5f - 0.7f;
            y0 = fmod(fabs(i.get_z1())- 1.0f, 2.4f) * 1.5f - 0.4f;
        }
        x = x0;
        y = y0;
        goto WHILE;
    }
    else
    {
        float colour = 767.0f - ((iteration * 767.0f) / this->i);
        if (colour < 256.0f) 
        {
            *c = ext_colour_t(25.0f - colour, 255.0f, 255.0f);
        }
        else if (colour < 512.0f) 
        {
            *c = ext_colour_t(0.0f, 256.0f - (colour - 256.0f), 255.0f);
        }
        else 
        {
            *c = ext_colour_t(0.0f, 0.0f, 256.0f - (colour - 767.0f));
        }
    }
    
    (*c) *= ext_colour_t(total_shade, total_shade, total_shade);
}


void mandelbrot_shader::combind_secondary_rays(const ray_trace_engine &r, ext_colour_t *const c, const secondary_ray_data &rl, const secondary_ray_data &rf) const
{
    return;
}
}; /* namespace raptor_raytracer */
