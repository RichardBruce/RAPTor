#include "mapper_shader.h"
#include "raytracer.h"
#include "triangle.h"
#include "light.h"


namespace raptor_raytracer
{
void mapper_shader::generate_rays(const ray_trace_engine &r, ray &i, const line &n, const hit_t h, ray *const rl, ray *const rf, fp_t *const n_rl, fp_t *const n_rf) const
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
    fp_t alpha  = 1.0;
    for (list<texture_mapper *>::const_iterator j = this->ctex.begin(); j != this->ctex.end() ; ++j)
    {
        alpha *= (*j)->texture_map(i.get_dst(), i.get_dir(), &rgb, vt);
        
        if (alpha == 0.0)
        {
            break;
        }
    }
    
    /* Get the materials kd */
    fp_t cur_kd = this->kd;
    alpha       = 1.0;
    ext_colour_t param;
    for (list<texture_mapper *>::const_iterator j = this->dtex.begin(); j != this->dtex.end() ; ++j)
    {
        alpha *= (*j)->texture_map(i.get_dst(), i.get_dir(), &param, vt);
        cur_kd = magnitude(param) / 255.0;
        
        if (alpha == 0.0)
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
        shade *= cur_kd;

        /* Calculate and scale the specular componant */
        if (ray_dot_reflection > (fp_t)0.0)
        {
            shade += pow(ray_dot_reflection, this->s) * this->ks;
        }

        /* Add to the overall shading */
        /* Note rgb is stored in light_intensity in the format ( r, g, b ) */
        (*c) += rgb * shade * illum.get_magnitude() * (*iter).get_light_intensity(illum.get_dir(), illum.get_length());
    }

    /* Get the transparency of the material */
    fp_t refl   = this->rf;
    alpha       = 1.0;
    for (list<texture_mapper *>::const_iterator j = this->rtex.begin(); j != this->rtex.end() ; ++j)
    {
        alpha *= (*j)->texture_map(i.get_dst(), i.get_dir(), &rgb, vt);
        refl = magnitude(rgb);
        
        if (alpha == 0.0)
        {
            break;
        }
    }
    
    /* Reflect */
    if (refl > (fp_t)0.0)
    {
        ext_colour_t    average;
        ray             rl[REFLECTION_ARRAY_SIZE];
        
        /* Ray reflection member */
        fp_t nr_rays = i.reflect(rl, n, refl, this->rfd);
        
        for (int i=0; i<(int)nr_rays; ++i)
        {
            ext_colour_t pixel;
            r.ray_trace(rl[i], &pixel);
            average += pixel;
        }

        /* Average the colours */
        if (nr_rays > (fp_t)0.0)
        {
            (*c) += average * (refl / nr_rays);
        }
    }

    /* Get the transparency of the material */
    fp_t trans  = tran;
    alpha       = 1.0;
    for (list<texture_mapper *>::const_iterator j = this->ttex.begin(); j != this->ttex.end() ; ++j)
    {
        alpha *= (*j)->texture_map(i.get_dst(), i.get_dir(), &rgb, vt);
        trans = rgb.r;
        
        if (alpha == 0.0)
        {
            break;
        }
    }
    if (trans >= 0.9)
    {
        (*c) = ext_colour_t(0.0, 0.0, 0.0);
    }
    
    /* Refract */
    if (trans > (fp_t)0.0)
    {
        ext_colour_t    average;
        ray             rl[REFLECTION_ARRAY_SIZE];
        
        /* Ray refraction member */
        fp_t nr_rays = i.refract(rl, n, trans, this->ri, h, this->td);
        
        for (int i=0; i<(int)nr_rays; ++i)
        {
            ext_colour_t pixel;
            r.ray_trace(rl[i], &pixel);
            average += pixel;
        }

        /* Average the colours */
        if (nr_rays > (fp_t)0.0)
        {
            (*c) += average * (trans / nr_rays);
        }
    }
}


void mapper_shader::combind_secondary_rays(const ray_trace_engine &r, ext_colour_t &c, const ray *const rl, const ray *const rf, const ext_colour_t *const c_rl, const ext_colour_t *const c_rf, const fp_t *const n_rl, const fp_t *const n_rf) const
{
    return;
}
}; /* namespace raptor_raytracer */
