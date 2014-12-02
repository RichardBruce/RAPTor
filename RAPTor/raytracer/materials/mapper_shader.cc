#include "mapper_shader.h"
#include "raytracer.h"
#include "triangle.h"
#include "light.h"


namespace raptor_raytracer
{
void mapper_shader::generate_rays(const ray_trace_engine &r, ray &i, const line &n, const hit_t h, ray *const rl, ray *const rf, float *const n_rl, float *const n_rf) const
{
    /* For each light request rays */
    for (unsigned int l = 0; l < r.get_scene_lights().size(); ++l)
    {
        r.generate_rays_to_light(i, n, h, l);
    }
    
    return;
}


void mapper_shader::shade(const ray_trace_engine &r, ray &i, const line &n, const hit_t h, ext_colour_t *const c, const point_t &vt) const
{
    /* Get the colour of the material */
    ext_colour_t rgb = this->rgb;
    float alpha  = 1.0f;
    for (list<texture_mapper *>::const_iterator j = this->ctex.begin(); j != this->ctex.end() ; ++j)
    {
        alpha *= (*j)->texture_map(&rgb, i.get_dst(), n.get_dir(), vt);
        
        if (alpha == 0.0f)
        {
            break;
        }
    }
    
    /* Get the materials kd */
    float cur_kd    = this->kd;
    alpha           = 1.0f;
    ext_colour_t param;
    for (list<texture_mapper *>::const_iterator j = this->dtex.begin(); j != this->dtex.end() ; ++j)
    {
        alpha *= (*j)->texture_map(&param, i.get_dst(), n.get_dir(), vt);
        cur_kd = magnitude(param) / 255.0f;
        
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
        float shade = dot_product(illum.get_dir(), n.get_dir());
        
        /* Ignore if the surface is facing away from the ray */
        if (shade < 0.0f)
        {
            continue;
        }
        
        /* Find the reflection of the light ray */
        point_t ref_dir;
        float shade_x2 = 2.0f * shade;
        ref_dir.x = illum.get_x_grad() - n.get_x_grad() * shade_x2;
        ref_dir.y = illum.get_y_grad() - n.get_y_grad() * shade_x2;
        ref_dir.z = illum.get_z_grad() - n.get_z_grad() * shade_x2;
    
        /* Cos(angle between refelction and the ray) */
        float ray_dot_reflection = dot_product(i.get_dir(), ref_dir);

        /* Scale the diffuse componant */
        shade *= cur_kd;

        /* Calculate and scale the specular componant */
        if (ray_dot_reflection > 0.0f)
        {
            shade += pow(ray_dot_reflection, this->s) * this->ks;
        }

        /* Add to the overall shading */
        /* Note rgb is stored in light_intensity in the format ( r, g, b ) */
        (*c) += rgb * shade * illum.get_magnitude() * (*iter).get_light_intensity(illum.get_dir(), illum.get_length());
    }

    /* Get the transparency of the material */
    float refl  = this->rf;
    alpha       = 1.0f;
    for (list<texture_mapper *>::const_iterator j = this->rtex.begin(); j != this->rtex.end() ; ++j)
    {
        alpha *= (*j)->texture_map(&rgb, i.get_dst(), n.get_dir(), vt);
        refl = rgb.r / 255.0f;
        
        if (alpha == 0.0f)
        {
            break;
        }
    }
    
    /* Reflect */
    if (refl > 0.0f)
    {
        ext_colour_t    average;
        ray             rl[REFLECTION_ARRAY_SIZE];
        
        /* Ray reflection member */
        float nr_rays = i.reflect(rl, n, refl, this->rfd);
        
        for (int i = 0; i < static_cast<int>(nr_rays); ++i)
        {
            ext_colour_t pixel;
            r.ray_trace(rl[i], &pixel);
            average += pixel;
        }

        /* Average the colours */
        if (nr_rays > 0.0f)
        {
            (*c) += average * (refl / nr_rays);
        }
    }

    /* Get the transparency of the material */
    float trans = tran;
    alpha       = 1.0f;
    for (list<texture_mapper *>::const_iterator j = this->ttex.begin(); j != this->ttex.end() ; ++j)
    {
        alpha *= (*j)->texture_map(&rgb, i.get_dst(), n.get_dir(), vt);
        trans = rgb.r / 255.0f;
        
        if (alpha == 0.0f)
        {
            break;
        }
    }
    if (trans >= 0.9f)
    {
        (*c) = ext_colour_t(0.0f, 0.0f, 0.0f);
    }
    
    /* Refract */
    if (trans > 0.0f)
    {
        ext_colour_t    average;
        ray             rl[REFLECTION_ARRAY_SIZE];
        
        /* Ray refraction member */
        float nr_rays = i.refract(rl, n, trans, this->ri, h, this->td);
        
        for (int i = 0; i< static_cast<int>(nr_rays); ++i)
        {
            ext_colour_t pixel;
            r.ray_trace(rl[i], &pixel);
            average += pixel;
        }

        /* Average the colours */
        if (nr_rays > 0.0f)
        {
            (*c) += average * (trans / nr_rays);
        }
    }
}


void mapper_shader::combind_secondary_rays(const ray_trace_engine &r, ext_colour_t &c, const ray *const rl, const ray *const rf, const ext_colour_t *const c_rl, const ext_colour_t *const c_rf, const float *const n_rl, const float *const n_rf) const
{
    return;
}
}; /* namespace raptor_raytracer */
