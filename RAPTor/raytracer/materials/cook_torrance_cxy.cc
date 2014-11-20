#include "cook_torrance_cxy.h"
#include "raytracer.h"
#include "triangle.h"
#include "light.h"


namespace raptor_raytracer
{
void cook_torrance_cxy::generate_rays(const ray_trace_engine &r, ray &i, const line &n, const hit_t h, ray *const rl, ray *const rf, float *const n_rl, float *const n_rf) const
{
    /* For each light request rays */
    for (unsigned int l = 0; l < r.get_scene_lights().size(); ++l)
    {
        r.generate_rays_to_light(i, n, h, l);
    }
    
    /* Request reflections */
    (*n_rl) = i.reflect(rl, n, this->rs, 0.0f);
    
    /* Request refractions */
    (*n_rf) = i.refract(rf, n, this->ts, this->ri_r, h, 0.0f);

    return;
}


void cook_torrance_cxy::shade(const ray_trace_engine &r, ray &i, const line &n, const hit_t h, ext_colour_t *const c, const point_t &vt) const
{
    /* A common dot product to all light sources */
    const float nv = dot_product(n.get_dir(), i.get_dir());
        
    /* For each light shade the object */
    unsigned int l = 0;
    for (ray_trace_engine::const_light_iterator iter = r.get_scene_lights().begin(); iter != r.get_scene_lights().end(); ++iter)
    {
        /* Query the scene to see if this light is visiable */
        ray illum = r.get_illumination(l++);

        const /* Cos(angle between normal and the ray) */
        float shade = dot_product(illum.get_dir(), n.get_dir());
        
        /* Ignore if the surface is facing away from the ray */
        if (shade < 0.0f)
        {
            continue;
        }
        
        /* Take the half vector */
        point_t half;
        half.x = (illum.get_x_grad() + i.get_x_grad()) * 0.5f;
        half.y = (illum.get_y_grad() + i.get_y_grad()) * 0.5f;
        half.z = (illum.get_z_grad() + i.get_z_grad()) * 0.5f;

        /* Take some common dot products */
        const float nh = dot_product(n.get_dir(), half);
        const float vh = dot_product(half, i.get_dir());
        const float nl = shade;

        const float f  = schlick_fresnell(this->sr, nv, this->ri_i);
        
        const float g  = geometric_attenuation_factor(nh, nv, nl, vh);
        
        const float ro = gaussian_facet_distribution(nh, this->sr, 5.0f);
        
        const float s  = this->rs * ((g * f * ro) / (nl * nv));

        /* Add to the overall shading */
        ext_colour_t rgb;
        cxy_to_rgb(this->x, y, (shade * (this->rd + s) * illum.get_magnitude()), &rgb);
        (*c) += rgb * (*iter).get_light_intensity(illum.get_dir(), illum.get_length());
    }
}


void cook_torrance_cxy::combind_secondary_rays(const ray_trace_engine &r, ext_colour_t &c, const ray *const rl, const ray *const rf, const ext_colour_t *const c_rl, const ext_colour_t *const c_rf, const float *const n_rl, const float *const n_rf) const
{
    /* Process reflection data */
    const float l_rl = *n_rl;
    if ((this->rs > 0.0f) & (l_rl > 0.0f))
    {
        ext_colour_t average;
        for (int i = 0; i < static_cast<int>(l_rl); ++i)
        {
            average += c_rl[i];
        }

        /* Average the colours */
        c += average * (this->rs / l_rl);
    }

    /* Process refraction data */
    const float l_rf = *n_rf;
    if ((this->ts > 0.0f) & (l_rf > 0.0f))
    {
        ext_colour_t average;
        for (int i = 0; i < static_cast<int>(l_rf); ++i)
        {
            average += c_rf[i];
        }

        /* Average the colours */
        c += average * (this->ts / l_rf);
    }

    return;
}
}; /* namespace raptor_raytracer */
