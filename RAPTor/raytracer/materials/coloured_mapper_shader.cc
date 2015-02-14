#include "coloured_mapper_shader.h"
#include "raytracer.h"
#include "triangle.h"
#include "light.h"


namespace raptor_raytracer
{
void coloured_mapper_shader::generate_rays(const ray_trace_engine &r, ray &i, const line &n, const hit_t h, ray *const rl, ray *const rf, float *const n_rl, float *const n_rf) const
{
    /* For each light request rays */
    for (unsigned int l = 0; l < r.get_scene_lights().size(); ++l)
    {
        r.generate_rays_to_light(i, n, h, l);
    }
    
    /* Request reflections */
    (*n_rl) = i.reflect(rl, n, this->rf, this->rfd);
    
    /* Request refractions */
    (*n_rf) = i.refract(rf, n, tran, this->ri, h, this->td);

    return;
}


void coloured_mapper_shader::shade(const ray_trace_engine &r, ray &i, const line &n, const hit_t h, ext_colour_t *const c, const point_t &vt) const
{
    /* Get the materials ns */
    ext_colour_t cur_ka;
    float cur_ns = this->ns;
    if(this->t_ns != nullptr)
    {
        cur_ns = this->t_ns->texture_map(&cur_ka, i.get_dst(), n.get_dir(), vt);
    }

    /* Get the materials ka */
    cur_ka = this->ka;
    if(this->t_ka != nullptr)
    {
        this->t_ka->texture_map(&cur_ka, i.get_dst(), n.get_dir(), vt);
        cur_ka *= (this->ka * (1.0f / 255.0f));
    }

    /* Get the materials kd */
    ext_colour_t cur_kd = this->kd;
    if(this->t_kd != nullptr)
    {
        this->t_kd->texture_map(&cur_kd, i.get_dst(), n.get_dir(), vt);
        cur_kd *= (this->kd* (1.0f / 255.0f));
    }
    
    /* Get the materials ks */
    ext_colour_t cur_ks = this->ks;
    if(this->t_ks != nullptr)
    {
        this->t_ks->texture_map(&cur_ks, i.get_dst(), n.get_dir(), vt);
        cur_ks *= (this->ks * (1.0f / 255.0f));
    }

    /* For each light shade the object */
    unsigned int l = 0;
    for (ray_trace_engine::const_light_iterator iter = r.get_scene_lights().begin(); iter != r.get_scene_lights().end(); ++iter)
    {
        /* Query the scene to see if this light is visiable */
        ray illum = r.get_illumination(l++);

        /* Cos(angle between normal and the ray) */
        float shade = dot_product(illum.get_dir(), n.get_dir());
        
        /* Ignore if the surface is facing away from the ray */
        if (shade < 0.0f)
        {
            continue;
        }
        
        /* Find the reflection of the light ray */
        point_t ref_dir = illum.get_dir() - n.get_dir() * (2.0f * shade);
    
        /* Cos(angle between refelction and the ray) */
        float ray_dot_reflection = dot_product(i.get_dir(), ref_dir);

        /* Scale the diffuse componant */
        ext_colour_t shade_colour = shade * cur_kd;

        /* Calculate and scale the specular componant */
        if (ray_dot_reflection > 0.0f)
        {
            shade_colour += pow(ray_dot_reflection, cur_ns) * cur_ks;
        }

        /* Add to the overall shading */
        (*c) += (*iter).get_light_intensity(illum.get_dir(), illum.get_length()) * (shade_colour * illum.get_magnitude());
    }
    
    /* Add the ambient colour */
    (*c) += cur_ka;
    
    return;
}


void coloured_mapper_shader::combind_secondary_rays(const ray_trace_engine &r, ext_colour_t &c, const ray *const rl, const ray *const rf, const ext_colour_t *const c_rl, const ext_colour_t *const c_rf, const float *const n_rl, const float *const n_rf) const
{
    /* Process reflection data */
    if ((this->rf > 0.0f) && ((*n_rl) > 0.0))
    {
        ext_colour_t average;
        for (int i = 0; i < static_cast<int>(*n_rl); ++i)
        {
            average += c_rl[i];
        }

        /* Average the colours */
        c += average * (this->rf / (*n_rl));
    }
    
    /* Process refraction data */
    if ((tran > 0.0f) && ((*n_rf) > 0.0))
    {
        ext_colour_t average;
        for (int i = 0; i < static_cast<int>(*n_rf); ++i)
        {
            average += c_rf[i];
        }

        /* Average the colours */
        c += average * (tran / (*n_rf));
    }

    return;
}
}; /* namespace raptor_raytracer */
