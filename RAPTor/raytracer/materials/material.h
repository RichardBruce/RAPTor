#pragma once

#include "common.h"
#include "ext_colour_t.h"
#include "point_t.h"


namespace raptor_raytracer
{
/* Forward declarations */
class secondary_ray_data;
class ray_trace_engine;
class ray;

/* Change the following to suit your standard */
/* These are constants for colour space conversion */
const float CIE_x_r = 0.640f;      /* nominal CRT primaries */
const float CIE_y_r = 0.330f;
const float CIE_x_g = 0.290f;
const float CIE_y_g = 0.600f;
const float CIE_x_b = 0.150f;
const float CIE_y_b = 0.060f;
const float CIE_x_w = 0.3333f;     /* use true white */
const float CIE_y_w = 0.3333f;

const float CIE_C_rD = ((1.0f / CIE_y_w) * (CIE_x_w * (CIE_y_g - CIE_y_b) - CIE_y_w * (CIE_x_g - CIE_x_b) + CIE_x_g * CIE_y_b - CIE_x_b * CIE_y_g));
const float CIE_C_gD = ((1.0f / CIE_y_w) * (CIE_x_w * (CIE_y_b - CIE_y_r) - CIE_y_w * (CIE_x_b - CIE_x_r) - CIE_x_r * CIE_y_b + CIE_x_b * CIE_y_r));
const float CIE_C_bD = ((1.0f / CIE_y_w) * (CIE_x_w * (CIE_y_r - CIE_y_g) - CIE_y_w * (CIE_x_r - CIE_x_g) + CIE_x_r * CIE_y_g - CIE_x_g * CIE_y_r));

/* XYZ to RGB conversion matrix */
const float xyz2rgbmat[3][3] = {
    { (CIE_y_g - CIE_y_b - CIE_x_b * CIE_y_g + CIE_y_b * CIE_x_g) / CIE_C_rD,
      (CIE_x_b - CIE_x_g - CIE_x_b * CIE_y_g + CIE_x_g * CIE_y_b) / CIE_C_rD,
      (                    CIE_x_g * CIE_y_b - CIE_x_b * CIE_y_g) / CIE_C_rD },
     
    { (CIE_y_b - CIE_y_r - CIE_y_b * CIE_x_r + CIE_y_r * CIE_x_b) / CIE_C_gD,
      (CIE_x_r - CIE_x_b - CIE_x_r * CIE_y_b + CIE_x_b * CIE_y_r) / CIE_C_gD,
                          (CIE_x_b * CIE_y_r - CIE_x_r * CIE_y_b) / CIE_C_gD },
     
    { (CIE_y_r - CIE_y_g - CIE_y_r * CIE_x_g + CIE_y_g * CIE_x_r) / CIE_C_bD,
      (CIE_x_g - CIE_x_r - CIE_x_g * CIE_y_r + CIE_x_r * CIE_y_g) / CIE_C_bD,
                          (CIE_x_r * CIE_y_g - CIE_x_g * CIE_y_r) / CIE_C_bD }
};


/* Pure virtual class for material data and shading */
class material
{
    public :
        material(const bool t = false) : t(t) { };
        virtual ~material() { };

        /* Pure virtual function to the allow the shader a chance to generate SIMD packets */
        virtual void generate_rays(const ray_trace_engine &r, ray &i, point_t<> *const n, const point_t<> &vt, const hit_t h, secondary_ray_data *const rl, secondary_ray_data *const rf) const = 0;

        /* Pure virtual shading function. To allow the shader to shade the current object */
        virtual void shade(const ray_trace_engine &r, ray &i, const point_t<> &n, const hit_t h, ext_colour_t *const c, const point_t<> &vt) const = 0;

        /* Pure virtual function to the allow the shader a combined SIMD packets traced secondary rays into the image */
        virtual void combind_secondary_rays(const ray_trace_engine &r, ext_colour_t *const c, const secondary_ray_data &rl, const secondary_ray_data &rf) const = 0;

        /* Allow read transparency */
        const bool is_transparent() const { return t; }

    private :
        const bool  t;  /* Is the material transparent */
};

/* Function for calculating the Fresnell componant */
/**********************************************************
  direct_fresnell is a direct evaluation of the Fresnel term. This is
  very slow, but will give the most accurate results for conductors.
  Using the direct Fresnel evaluation doesnt necassarily give a benefit
  for non condicting materials with a 0 extinction co-efficient.
  
  The inputs to this function are :
    r is the index of refraction.
    k is the extinction co-efficient (complex part of the index of refraction).
    t is the angle of incidence.
  
  The return value is the Fresnel componant.
**********************************************************/
inline float direct_fresnell(const float r, const float t, const float k)
{
    /* Some repeatedly used variables */
    const float r_sq = r * r;
    const float k_sq = k * k;
    const float t_sq = t * t;
    
    /* g^2 = n^2 - 1 + t^2; where n is the complex index of refraction */
    const float re_g_sq = r_sq - k_sq - 1.0f + t_sq;
    const float im_g_sq = 2.0f * r * k;
    
    /* Take the absolute of g^2; Where g^2 = g_re^2 + i * g_im^2 */
    const float abs_re_g_sq = std::fabs(re_g_sq);
    const float abs_im_g_sq = std::fabs(im_g_sq);

    float abs_g_sq;
    if ((re_g_sq == 0.0f) && (im_g_sq == 0.0f))
    {
        abs_g_sq = 0.0f;
    }
    else if (abs_re_g_sq >= abs_im_g_sq)
    {
        float d   = abs_im_g_sq / abs_re_g_sq;
        abs_g_sq = abs_re_g_sq * std::sqrt(1.0f + (d * d));
    }
    else
    {
        float d   = abs_re_g_sq / abs_im_g_sq;
        abs_g_sq = abs_im_g_sq * std::sqrt(1.0f + (d * d));
    }
    
    /* s = (1 - t^2) / t */
    const float s = (1.0f - t_sq) / t;

    /* a = sqrt((abs(g^2) + re(g^2)) / 2) */
    /* b = sqrt((abs(g^2) - re(g^2)) / 2) */
    const float a     = std::sqrt((abs_g_sq + re_g_sq) * 0.5f);
    const float b     = std::sqrt((abs_g_sq - re_g_sq) * 0.5f);
    const float b_sq  = b * b;
    
    /* Some repeatedly used Fresnel terms */
    const float a_m_t = a - t;
    const float a_p_t = a + t;
    const float a_m_s = a - s;
    const float a_p_s = a + s;
    
    const float f0 = 0.5f * (((a_m_t * a_m_t) + b_sq)/((a_p_t * a_p_t) + b_sq));
    const float f1 = 1.0f + (((a_m_s * a_m_s) + b_sq)/((a_p_s * a_p_s) + b_sq));
    return f0 * f1;
}


/**********************************************************
  schlick_fresnell uses Schlick's approximation to evaluate the Fresnel 
  term. This is much faster than direct evaluation, but will leave 
  conductors looking plastic because the extinction co-efficient is ignored.
  
  The inputs to this function are :
    r is the index opf refraction.
    t is the angle of incidence.

  The return value is the Fresnel componant.
**********************************************************/
inline float schlick_fresnell(const float r, const float t, const float k)
{
    return (r + (1.0f - r) * std::pow(1.0f - t, 5.0f));
}


/**********************************************************
  rescaled_schlick_fresnell uses Schlick's approximation to evaluate the Fresnel 
  term, but the equation is rescaled to cope with a complex index of refraction.
  This allows better shading for conductor, but is still faster than direct
  evaluation. This function is slightly slower than Schlick's approximation.
  
  The inputs to this function are :
    r is the index of refraction.
    k is the extinction co-efficient (complex part of the index of refraction).
    t is the angle of incidence.

  The return value is the Fresnel componant.
**********************************************************/
inline float rescaled_schlick_fresnell(const float r, const float t, const float k)
{
    const float r_m1 = r - 1.0f;
    const float r_p1 = r + 1.0f;
    const float k_sq = k * k;
    
    return ((r_m1 * r_m1) + (4.0f * r * std::pow((1.0f - t), 5.0f)) + k_sq) / ((r_p1 * r_p1) + k_sq);
}


/* Function for calculating the Geometric attenuation factor */
/**********************************************************
  geometric_attenuation_factor calculates the geometric attenuation
  factor needed by some shaders.
  
  The inputs to this function are :
    nh is the angle between the normal and half vector.
    nv is the angle between the normal and view vector.
    nl is the angle between the normal and light vector.
    vh is angle between the view and half vector

  The return value is the geometric attenuation factor.
**********************************************************/
inline float geometric_attenuation_factor(const float nh, const float nv, const float nl, const float vh)
{
    const float g0 = (2.0f * nh * nv ) / vh;
    const float g1 = (2.0f * nh * nl ) / vh;
    return std::min(1.0f, std::max(0.0f, std::min(g0, g1)));
}


/* Function for calculating the Facet slope distributon function */
/**********************************************************
  gaussian_facet_distribution uses a guassian distribution of facets to
  approximate the amount of occulsion caused by micro-facets. This curve
  can be fixed to a real curve using 'c'. This function will run faster
  than the Beckmann distribution, but requires fitting of the curve.
  
  The inputs to this function are :
    a is the viewing angle.
    m is the rms facet distribution.
    c is an arbitary constant.

  The return value is the facet distribution.
**********************************************************/
inline float gaussian_facet_distribution(const float a, const float m, const float c)
{
    /* ce^-((alpha/m)^2) */
    const float a_div_m = a / m;
    return c * std::exp(-(a_div_m * a_div_m));
}


/**********************************************************
  beckmann_facet_distribution uses a distribution model proposed by 
  Beckmann to model the amount of occulsion caused by micro-facets.
  This function will run slower than the Guassian distribution function,
  but doesnt require fitting.
  
  The inputs to this function are :
    a is the viewing angle.
    m is the rms facet distribution.

  The return value is the facet distribution.
**********************************************************/
inline float beckmann_facet_distribution(const float a, const float m, const float c)
{
    const float cos_4_a  = pow(cos(a), 4.0f);
    const float tan_a    = tan(a);
    const float tan_a_sq = tan_a * tan_a;
    
    const float m_sq = m * m;
    
    /* 1/(m^2 * cos^4(alpha)) * e^-(tan^2(alpha)/m^2) */
    return ((1.0f / (m_sq * cos_4_a)) * std::exp(-(tan_a_sq / m_sq)));
}


/* Function for calculating colour shift */
/**********************************************************
  linear_colour_shift uses a linear interpolation model to calculate
  the attenuation of a wavelength at the current viewing angle. As the
  viewing angle approaches 90 degrees to the normal the viewed colour 
  approaches that of the light source. As the viewing angle approaches 
  0 degrees to the normal the viewed colour approaches that of the material.
  
  The inputs to this function are :
    c0  is the colour at 0 degrees viewing angle.
    c90 is the colour at 90 degrees viewing angle.
    fa  is the Fresnel term at the current viewing angle.
    f0  is the Fresnel term at 0 degrees viewing angle.
    f90 is the Fresnel term at 90 degrees viewing angle.
    
  The return value is the attenuated colour.
**********************************************************/
inline float linear_colour_shift(const float c0, const float c90, const float fa, const float f0, const float f90)
{
    /* calpha = c0 + ((c90 - c0) * (max(0,Falpha - F0)/F90- F0)) */
    return c0 + ((c90 - c0) * (std::max(0.0f, (fa - f0)) / (f90- f0)));
}


/* Function for black body colour conversion */
/**********************************************************
  black_body_temperature_to_cxy evaluates the equations below
  to give a fast approximation of the colour of an incandescant
  light emitter.
  x_c= -0.2661239 {10^9}/{T^3} - 0.2343580 {10^6}/{T^2} + 0.8776956 {10^3}/{T} + 0.179910 for 1667 <= T <=  4000
       -3.0258469 {10^9}/{T^3} + 2.1070379 {10^6}/{T^2} + 0.2226347 {10^3}/{T} + 0.24039  for 4000 <= T <= 25000

  y_c= -1.1063814 x_c^3 - 1.34811020 x_c^2 + 2.18555832 x_c - 0.20219683 for 1667 <= T <=  2222
       -0.9549476 x_c^3 - 1.37418593 x_c^2 + 2.09137015 x_c - 0.16748867 for 2222 <= T <=  4000
       +3.0817580 x_c^3 - 5.87338670 x_c^2 + 3.75112997 x_c - 0.37001483 for 4000 <= T <= 25000

  The inputs to this function are :
    t is the black body temperature in Kelvin
    
  The return value is the colour of the light emitter.
**********************************************************/
inline void black_body_temperature_to_cxy(const float t, float *const x, float *const y)
{
    /* Convert temperature to CXY X value */
    float c_x;
    const float t_sq = t    * t;
    const float t_3  = t_sq * t;
    if ((1667.0f <= t) && (t <= 4000.0f))
    {
        c_x = -0.2661239f * (10e9f / t_3) - 0.2343580f * (10e6f / t_sq) + 0.8776956f * (10e3f / t) + 0.179910f;
    }
    else if ((4000.0f <= t) && (t <= 25000.0f))
    {
        c_x = -3.0258469f * (10e9f / t_3) + 2.1070379f * (10e6f / t_sq) + 0.2226347f * (10e3f / t) + 0.24039f;
    }
    else
    {
        assert(false);
    }
    *x = c_x;
    
    /* Convert temperature and CXY X value to CXY Y value */
    const float c_x_sq = c_x    * c_x;
    const float c_x_3  = c_x_sq * c_x;
    if ((1667.0f <= t) && (t <= 2222.0f))
    {
        *y = -1.1063814f * c_x_3 - 1.34811020f * c_x_sq + 2.18555832f * c_x - 0.20219683f;
    }
    else if ((2222.0f <= t) && (t <= 4000.0f))
    {
        *y = -0.9549476f * c_x_3 - 1.37418593f * c_x_sq + 2.09137015f * c_x - 0.16748867f;
    }
    else if ((4000.0f <= t) && (t <= 25000.0f))
    {
        *y =  3.0817580f * c_x_3 - 5.87338670f * c_x_sq + 3.75112997f * c_x - 0.37001483f;
    }
    else
    {
        assert(false);
    }
}

/* Function for colour space conversion */
inline ext_colour_t& cxy_to_rgb(const float c_x, const float c_y, const float y, ext_colour_t *const rgb)
{
    float cie[3], cout[3];
    
    cie[0] = y * c_x/c_y;
    cie[1] = y;
    cie[2] = y * (1.0f/c_y - 1.0f) - cie[0];

    /* convert to RGB */
    cout[0] = (xyz2rgbmat[0][0] * cie[0]) + (xyz2rgbmat[0][1] * cie[1]) + (xyz2rgbmat[0][2] * cie[2]);
    cout[1] = (xyz2rgbmat[1][0] * cie[0]) + (xyz2rgbmat[1][1] * cie[1]) + (xyz2rgbmat[1][2] * cie[2]);
    cout[2] = (xyz2rgbmat[2][0] * cie[0]) + (xyz2rgbmat[2][1] * cie[1]) + (xyz2rgbmat[2][2] * cie[2]);

    /* Bound the rgb value */
    rgb->r = std::max(0.0f, std::min(1.0f, cout[0])) * 255.0f;
    rgb->g = std::max(0.0f, std::min(1.0f, cout[1])) * 255.0f;
    rgb->b = std::max(0.0f, std::min(1.0f, cout[2])) * 255.0f;
    return *rgb;
}
}; /* namespace raptor_raytracer */
