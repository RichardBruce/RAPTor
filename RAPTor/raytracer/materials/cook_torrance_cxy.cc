#include "cook_torrance_cxy.h"
#include "raytracer.h"
#include "triangle.h"
#include "light.h"


namespace raptor_raytracer
{
void cook_torrance_cxy::generate_rays(const ray_trace_engine &r, ray &i, const line &n, const hit_t h, ray *const rl, ray *const rf, fp_t *const n_rl, fp_t *const n_rf) const
{
    /* For each light request rays */
    for (unsigned int l = 0; l < r.get_scene_lights().size(); ++l)
    {
        r.generate_rays_to_light(i, n, h, l);
    }
    
    /* Request reflections */
    (*n_rl) = i.reflect(rl, n, this->rs, (fp_t)0.0);
    
    /* Request refractions */
    (*n_rf) = i.refract(rf, n, this->ts, this->ri_r, h, (fp_t)0.0);

    return;
}


void cook_torrance_cxy::shade(const ray_trace_engine &r, ray &i, const line &n, const hit_t h, ext_colour_t *const c, const point_t &vt) const
{
    /* A common dot product to all light sources */
    fp_t nv = dot_product(n.get_dir(), i.get_dir());
        
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
        
        /* Take the half vector */
        point_t half;
        half.x = (illum.get_x_grad() + i.get_x_grad()) * (fp_t)0.5;
        half.y = (illum.get_y_grad() + i.get_y_grad()) * (fp_t)0.5;
        half.z = (illum.get_z_grad() + i.get_z_grad()) * (fp_t)0.5;

        /* Take some common dot products */
        fp_t nh = dot_product(n.get_dir(), half);
        fp_t vh = dot_product(half, i.get_dir());
        fp_t nl = shade;

        fp_t f  = schlick_fresnell(this->sr, nv, this->ri_i);
        
        fp_t g  = geometric_attenuation_factor(nh, nv, nl, vh);
        
        fp_t ro = gaussian_facet_distribution(nh, this->sr, (fp_t)5.0);
        
        fp_t s  = this->rs * ((g * f * ro) / (nl * nv));

        /* Add to the overall shading */
        ext_colour_t rgb;
        cxy_to_rgb(this->x, y, (shade * (this->rd + s) * illum.get_magnitude()), &rgb);
        (*c) += rgb * (*iter).get_light_intensity(illum.get_dir(), illum.get_length());
    }
}


void cook_torrance_cxy::combind_secondary_rays(const ray_trace_engine &r, ext_colour_t &c, const ray *const rl, const ray *const rf, const ext_colour_t *const c_rl, const ext_colour_t *const c_rf, const fp_t *const n_rl, const fp_t *const n_rf) const
{
    /* Process reflection data */
    if ((this->rs > (fp_t)0.0) && ((*n_rl) > 0.0))
    {
        ext_colour_t average;
        for (int i=0; i<(int)(*n_rl); ++i)
        {
             average += c_rl[i];
        }

        /* Average the colours */
        c += average * (this->rs / (*n_rl));
    }

    /* Process refraction data */
    if ((this->ts > (fp_t)0.0) && ((*n_rf) > 0.0))
    {
        ext_colour_t average;
        for (int i=0; i<(int)(*n_rf); ++i)
        {
            average += c_rf[i];
        }

        /* Average the colours */
        c += average * (this->ts / (*n_rf));
    }

    return;
}

//float4 psCookTorrance( in VS_LIGHTING_OUTPUT v ) : COLOR
//{  
//  // Sample the textures
//  float3  Normal        = normalize( ( 2.0f * tex2D( sampNormMap, v.TexCoord ).xyz ) - 1.0f );
//  float3  Specular      = tex2D( sampSpecular, v.TexCoord ).rgb;
//  float3  Diffuse       = tex2D( sampDiffuse, v.TexCoord ).rgb;
//  float2  Roughness     = tex2D( sampRoughness, v.TexCoord ).rg;
//  
//  Roughness.r           *= 3.0f;
//  
//  // Correct the input and compute aliases
//  float3  ViewDir         = normalize( v.ViewDir );
//  float3  LightDir        = normalize( v.LightDir );
//  float3  vHalf           = normalize( LightDir + ViewDir );
//  float  NormalDotHalf    = dot( Normal, vHalf );
//  float  ViewDotHalf      = dot( vHalf,  ViewDir );
//  float  NormalDotView    = dot( Normal, ViewDir );
//  float  NormalDotLight   = dot( Normal, LightDir );
//  
//  // Compute the geometric term
//  float  G1          = ( 2.0f * NormalDotHalf * NormalDotView ) / ViewDotHalf;
//  float  G2          = ( 2.0f * NormalDotHalf * NormalDotLight ) / ViewDotHalf;
//  float  G           = min( 1.0f, max( 0.0f, min( G1, G2 ) ) );
//  
//  // Compute the fresnel term
//  float  F          = Roughness.g + ( 1.0f - Roughness.g ) * pow( 1.0f - NormalDotView, 5.0f );
//  
//  // Compute the roughness term
//  float  R_2        = Roughness.r * Roughness.r;
//  float  NDotH_2    = NormalDotHalf * NormalDotHalf;
//  float  A          = 1.0f / ( 4.0f * R_2 * NDotH_2 * NDotH_2 );
//  float  B          = exp( -( 1.0f - NDotH_2 ) / ( R_2 * NDotH_2 ) );
//  float  R          = A * B;
//  
//  // Compute the final term
//  float3  S          = Specular * ( ( G * F * R ) / ( NormalDotLight * NormalDotView ) );
//  float3  Final      = cLightColour.rgb * max( 0.0f, NormalDotLight ) * ( Diffuse + S );
//  
//  return float4( Final, 1.0f );
//}
}; /* namespace raptor_raytracer */
