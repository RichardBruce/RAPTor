#include "mapper_shader.h"
#include "raytracer.h"
#include "triangle.h"
#include "light.h"


namespace raptor_raytracer
{
void mapper_shader::generate_rays(const ray_trace_engine &r, ray &i, point_t *const n, const point_t &vt, const hit_t h, secondary_ray_data *const rl, secondary_ray_data *const rf) const
{
    /* Normal mapping */
    point_t normal(*n);
    // point_t bump_size(0.0f, 0.0f, 0.0f);
    // for (auto j : _btex)
    // {
    //     /* Sample the surrounding texture */
    //     point_t x_0;
    //     point_t x_1;
    //     point_t y_0;
    //     point_t y_1;
    //     j->texture_map_monochrome(i, &bump_size, *n, vt, 0, 0);
    //     j->texture_map_monochrome(i, &x_0, *n, vt, -1,  0);
    //     j->texture_map_monochrome(i, &x_1, *n, vt,  1,  0);
    //     j->texture_map_monochrome(i, &y_0, *n, vt,  0, -1);
    //     j->texture_map_monochrome(i, &y_1, *n, vt,  0,  1);

    //     bump_size.z *= _bump;
    //     x_0.z *= _bump;
    //     x_1.z *= _bump;
    //     y_0.z *= _bump;
    //     y_1.z *= _bump;

    //     /* Bump map */
    //     normal = bump_map(*n, x_0, x_1, y_0, y_1);
    // }
    // normalise(&normal);
    // *n = normal;

    /* For each light request rays */
    for (unsigned int l = 0; l < r.get_scene_lights().size(); ++l)
    {
        r.generate_rays_to_light(i, h, l);
    }

    /* Request reflections */
    ext_colour_t rgb;
    float refl  = _rf;
    float alpha = 1.0f;
    for (auto j : _rtex)
    {
        alpha *= j->texture_map(i, &rgb, normal, vt);
        refl = rgb.r * (1.0f / 255.0f);
        
        if (alpha == 0.0f)
        {
            break;
        }
    }
    
    rl->number(i.reflect(rl->rays(), *n, refl, _rfd));
    rl->value(refl);
    
    /* Request refractions */
    float trans = _tran;
    alpha       = 1.0f;
    for (auto j : _ttex)
    {
        alpha *= j->texture_map(i, &rgb, normal, vt);
        trans = rgb.r * (1.0f / 255.0f);
        
        if (alpha == 0.0f)
        {
            break;
        }
    }

    rf->number(i.refract(rf->rays(), *n, trans, _ri, h, _td));
    rf->value(trans);
    
    return;
}


void mapper_shader::shade(const ray_trace_engine &r, ray &i, const point_t &n, const hit_t h, ext_colour_t *const c, const point_t &vt) const
{
    /* Get the colour of the material */
    float alpha  = 1.0f;
    ext_colour_t rgb(_rgb);
    for (auto j : _ctex)
    {
        alpha *= j->texture_map(i, &rgb, n, vt);
        
        if (alpha == 0.0f)
        {
            break;
        }
    }
    
    /* Get the materials kd */
    alpha = 1.0f;
    ext_colour_t param;
    float cur_kd = _kd;
    for (auto j : _dtex)
    {
        alpha *= j->texture_map(i, &param, n, vt);
        cur_kd = magnitude(param) * (1.0f / 255.0f);
        
        if (alpha == 0.0f)
        {
            break;
        }
    }

    /* Get the materials s */
    alpha = 1.0f;
    float cur_s = _s;
    for (auto j : _stex)
    {
        alpha *= j->texture_map(i, &param, n, vt);
        cur_s = magnitude(param) * (1.0f / 255.0f);
        
        if (alpha == 0.0f)
        {
            break;
        }
    }
    
    /* For each light shade the object */
    unsigned int l = 0;
    for (auto &iter : r.get_scene_lights())
    {
        /* Query the scene to see if this light is visiable */
        const ray &illum = r.get_illumination(l++);

        /* Cos(angle between normal and the ray) */
        const float diff_shade = dot_product(illum.get_dir(), n) * cur_kd;
        
        /* Ignore if the surface is facing away from the ray */
        if (diff_shade < 0.0f)
        {
            continue;
        }

        /* Find the reflection of the light ray */
        point_t ref_dir;
        float shade_x2 = 2.0f * diff_shade;
        ref_dir.x = illum.get_x_grad() - n.x * shade_x2;
        ref_dir.y = illum.get_y_grad() - n.y * shade_x2;
        ref_dir.z = illum.get_z_grad() - n.z * shade_x2;
    
        /* Cos(angle between refelction and the ray) */
        float ray_dot_reflection = dot_product(i.get_dir(), ref_dir);

        /* Calculate and scale the specular componant */
        float spec_shade = 0.0f;
        if (ray_dot_reflection > 0.0f)
        {
            spec_shade = std::pow(ray_dot_reflection, cur_s) * _ks;
        }

        /* Get the lights colour */
        const ext_colour_t light_rgb(iter.get_light_intensity(illum.get_dir(), illum.get_length()));
        
        /* Add to the overall shading */
        /* Note rgb is stored in light_intensity in the format ( r, g, b ) */
        (*c) += (rgb * diff_shade * light_rgb * illum.get_magnitude());
        (*c) += ((_clrh * rgb) + ((1.0f - _clrh) * light_rgb)) * spec_shade * illum.get_magnitude();
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
        (*c) += (_clrf * (*c)) + ((1.0f - _clrf) * rf.average_colour() * rf.value());
    }
}
}; /* namespace raptor_raytracer */
