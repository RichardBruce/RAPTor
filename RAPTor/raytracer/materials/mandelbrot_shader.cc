#include "mandelbrot_shader.h"
#include "raytracer.h"
#include "light.h"


namespace raptor_raytracer
{
void mandelbrot_shader::generate_rays(const ray_trace_engine &r, ray &i, const line &n, const hit_t h, ray *const rl, ray *const rf, fp_t *const n_rl, fp_t *const n_rf) const
{
    /* For each light request rays */
    for (unsigned int l = 0; l < r.get_scene_lights().size(); ++l)
    {
        r.generate_rays_to_light(i, n, h, l);
    }
    
    return;
}


void mandelbrot_shader::shade(const ray_trace_engine &r, ray &i, const line &n, const hit_t h, ext_colour_t *const c, const point_t &vt) const
{
    /* For each light shade the object, but only diffusely */
    fp_t total_shade    = 0.0;
    unsigned int l      = 0;
    for (ray_trace_engine::const_light_iterator iter = r.get_scene_lights().begin(); iter != r.get_scene_lights().end(); ++iter)
    {
        /* Query the scene to see if this light is visiable */
        ray illum = r.get_illumination(l++);

        /* Cos(angle between normal and the ray) */
        fp_t shade = dot_product(illum.get_dir(), n.get_dir());
        
        /* Ignore if the surface is facing away from the ray */
        if (shade < 0.0)
        {
            continue;
        }
        
        /* Add to the overall shading */
        total_shade += shade * illum.get_magnitude();
    }
    
    fp_t x0;
    fp_t y0;
    
    if ((n.get_x_grad() == 1.0) || (n.get_x_grad() == -1.0))
    {
        x0 = fmod(fabs(i.get_z1()),            (fp_t)2.4) - (fp_t)1.5;
        y0 = fmod(fabs(i.get_y1())- (fp_t)1.5, (fp_t)2.4) - (fp_t)1.0;
    }
    else if ((n.get_z_grad() == 1.0) || (n.get_z_grad() == -1.0))
    {
        x0 = fmod(fabs(i.get_x1()),            (fp_t)2.4) - (fp_t)1.5;
        y0 = fmod(fabs(i.get_y1())- (fp_t)1.5, (fp_t)2.4) - (fp_t)1.0;
    }
    else
    {
        x0 = fmod(fabs(i.get_x1())- (fp_t)1.5, (fp_t)2.4) - (fp_t)1.0;
        y0 = fmod(fabs(i.get_z1()),            (fp_t)2.4) - (fp_t)1.5;
    }
    
    fp_t x = x0;
    fp_t y = y0;

    bool doomed = false;

WHILE :
    int iteration = 0;
    while ((x*x + y*y <= (10.0 * 10.0)) && (iteration < this->i))
    {
        fp_t xtemp = x * x - y * y + x0;
        y = 2.0 * x * y + y0;

        x = xtemp;

        ++iteration;
    }

    if ((doomed) && (iteration == this->i))
    {
        *c = ext_colour_t(0.0,0.0,0.0);
    }
    else if (iteration == this->i)
    {
        doomed = true;
        if ((n.get_x_grad() == 1.0) || (n.get_x_grad() == -1.0))
        {
            x0 = fmod(fabs(i.get_z1())- 1.0, 2.4) * 1.5 - 0.4;
            y0 = fmod(fabs(i.get_y1())- 2.0, 2.4) * 1.5 - 0.7;
        }
        else if ((n.get_z_grad() == 1.0) || (n.get_z_grad() == -1.0))
        {
            x0 = fmod(fabs(i.get_x1())- 1.0, 2.4) * 1.5 - 0.4;
            y0 = fmod(fabs(i.get_y1())- 2.0, 2.4) * 1.5 - 0.7;
        }
        else
        {
            x0 = fmod(fabs(i.get_x1())- 2.0, 2.4) * 1.5 - 0.7;
            y0 = fmod(fabs(i.get_z1())- 1.0, 2.4) * 1.5 - 0.4;
        }
        x = x0;
        y = y0;
        goto WHILE;
    }
    else
    {
        fp_t colour = 767 - ((iteration * 767) / this->i);
        if (colour < 256) 
        {
            *c = ext_colour_t(256 - colour, 255, 255);
    	} 
        else if (colour < 512) 
        {
        	*c = ext_colour_t(0, 256 - (colour - 256), 255);
        } 
        else 
        {
        	*c = ext_colour_t(0, 0, 256 - (colour - 767));
    	}
    }
    
    (*c) *= ext_colour_t(total_shade, total_shade, total_shade);
}


void mandelbrot_shader::combind_secondary_rays(const ray_trace_engine &r, ext_colour_t &c, const ray *const rl, const ray *const rf, const ext_colour_t *const c_rl, const ext_colour_t *const c_rf, const fp_t *const n_rl, const fp_t *const n_rf) const
{
    return;
}
}; /* namespace raptor_raytracer */
