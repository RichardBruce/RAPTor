#include "mapper_shader.h"
#include "raytracer.h"
#include "triangle.h"
#include "light.h"


namespace raptor_raytracer
{
void mapper_shader::generate_rays(const ray_trace_engine &r, ray &i, const line &n, const point_t &vt, const hit_t h, secondary_ray_data *const rl, secondary_ray_data *const rf) const
{
    /* For each light request rays */
    for (unsigned int l = 0; l < r.get_scene_lights().size(); ++l)
    {
        r.generate_rays_to_light(i, n, h, l);
    }

    /* Request reflections */
    ext_colour_t rgb;
    float refl  = this->rf;
    float alpha = 1.0f;
    point_t normal(n.get_dir());
    for (auto j = this->rtex.begin(); j != this->rtex.end() ; ++j)
    {
        alpha *= (*j)->texture_map(i, &rgb, normal, vt);
        refl = rgb.r / 255.0f;
        
        if (alpha == 0.0f)
        {
            break;
        }
    }
    
    rl->number(i.reflect(rl->rays(), n, refl, this->rfd));
    rl->value(refl);
    
    /* Request refractions */
    float trans = this->tran;
    alpha       = 1.0f;
    for (auto j = this->ttex.begin(); j != this->ttex.end() ; ++j)
    {
        alpha *= (*j)->texture_map(i, &rgb, normal, vt);
        trans = rgb.r / 255.0f;
        
        if (alpha == 0.0f)
        {
            break;
        }
    }

    rf->number(i.refract(rf->rays(), n, trans, this->ri, h, this->td));
    rf->value(trans);
    
    return;
}


void mapper_shader::shade(const ray_trace_engine &r, ray &i, const line &n, const hit_t h, ext_colour_t *const c, const point_t &vt) const
{
    /* Normal mapping */
    point_t normal(n.get_dir());
    // for (auto j = this->btex.begin(); j != this->btex.end() ; ++j)
    // {
    //     /* Sample the surrounding texture */
    //     point_t x_0;
    //     point_t x_1;
    //     point_t y_0;
    //     point_t y_1;
    //     (*j)->texture_map(i, &x_0, n.get_dir(), vt);
    //     (*j)->texture_map(i, &x_1, n.get_dir(), vt);
    //     (*j)->texture_map(i, &y_0, n.get_dir(), vt);
    //     (*j)->texture_map(i, &y_1, n.get_dir(), vt);

    //     /* Bump map */
    //     normal = bump_map(n.get_dir(), x_0, x_1, y_0, y_1, 0.02f);
    // }

    /* Get the colour of the material */
    float alpha  = 1.0f;
    ext_colour_t rgb(this->rgb);
    for (auto j = this->ctex.begin(); j != this->ctex.end() ; ++j)
    {
        alpha *= (*j)->texture_map(i, &rgb, normal, vt);
        
        if (alpha == 0.0f)
        {
            break;
        }
    }
    
    /* Get the materials kd */
    alpha = 1.0f;
    ext_colour_t param;
    float cur_kd = this->kd;
    for (auto j = this->dtex.begin(); j != this->dtex.end() ; ++j)
    {
        alpha *= (*j)->texture_map(i, &param, normal, vt);
        cur_kd = magnitude(param) * (1.0f / 255.0f);
        
        if (alpha == 0.0f)
        {
            break;
        }
    }

    /* Get the materials s */
    alpha = 1.0f;
    float cur_s = this->s;
    for (auto j = this->stex.begin(); j != this->stex.end() ; ++j)
    {
        alpha *= (*j)->texture_map(i, &param, normal, vt);
        cur_s = magnitude(param) * (1.0f / 255.0f);
        
        if (alpha == 0.0f)
        {
            break;
        }
    }
    
    /* For each light shade the object */
    unsigned int l = 0;
    for (ray_trace_engine::const_light_iterator iter = r.get_scene_lights().begin(); iter != r.get_scene_lights().end(); ++iter)
    {
        /* Query the scene to see if this light is visiable */
        ray illum = r.get_illumination(l++);

        /* Cos(angle between normal and the ray) */
        const float diff_shade = dot_product(illum.get_dir(), normal) * cur_kd;
        
        /* Ignore if the surface is facing away from the ray */
        if (diff_shade < 0.0f)
        {
            continue;
        }

        /* Find the reflection of the light ray */
        point_t ref_dir;
        float shade_x2 = 2.0f * diff_shade;
        ref_dir.x = illum.get_x_grad() - normal.x * shade_x2;
        ref_dir.y = illum.get_y_grad() - normal.y * shade_x2;
        ref_dir.z = illum.get_z_grad() - normal.z * shade_x2;
    
        /* Cos(angle between refelction and the ray) */
        float ray_dot_reflection = dot_product(i.get_dir(), ref_dir);

        /* Calculate and scale the specular componant */
        float spec_shade = 0.0f;
        if (ray_dot_reflection > 0.0f)
        {
            spec_shade = pow(ray_dot_reflection, cur_s) * this->ks;
        }

        /* Get the lights colour */
        const ext_colour_t light_rgb((*iter).get_light_intensity(illum.get_dir(), illum.get_length()));
        
        /* Add to the overall shading */
        /* Note rgb is stored in light_intensity in the format ( r, g, b ) */
        (*c) += (rgb * diff_shade * light_rgb * illum.get_magnitude());
        (*c) += ((clrh * rgb) + ((1.0f - clrh) * light_rgb)) * spec_shade * illum.get_magnitude();
    }
}

void mapper_shader::combind_secondary_rays(const ray_trace_engine &r, ext_colour_t *const c, const secondary_ray_data &rl, const secondary_ray_data &rf) const
{
    /* Process reflection data */
    if ((rl.value() > 0.0f) && (rl.number() > 0.0f))
    {
        (*c) += rl.average_colour() * rl.value();
    }
    
    /* Process refraction data */
    if ((rf.value() > 0.0f) && (rf.number() > 0.0f))
    {
        (*c) += (clrf * (*c)) + ((1.0f - clrf) * rf.average_colour() * rf.value());
    }
}
}; /* namespace raptor_raytracer */
