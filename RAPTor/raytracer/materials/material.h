#ifndef __MATERIAL_H__
#define __MATERIAL_H__

#include "common.h"
#include "ext_colour_t.h"
#include "point_t.h"


namespace raptor_raytracer
{
/* Forward declarations */
class ray_trace_engine;
class ray;
class line;

/* Change the following to suit your standard */
/* These are constants for colour space conversion */
const fp_t CIE_x_r  = 0.640;      /* nominal CRT primaries */
const fp_t CIE_y_r  = 0.330;
const fp_t CIE_x_g  = 0.290;
const fp_t CIE_y_g  = 0.600;
const fp_t CIE_x_b  = 0.150;
const fp_t CIE_y_b  = 0.060;
const fp_t CIE_x_w  = 0.3333;     /* use true white */
const fp_t CIE_y_w  = 0.3333;

const fp_t CIE_C_rD = (((fp_t)1.0 / CIE_y_w) * (CIE_x_w * (CIE_y_g - CIE_y_b) - CIE_y_w * (CIE_x_g - CIE_x_b) + CIE_x_g * CIE_y_b - CIE_x_b * CIE_y_g));
const fp_t CIE_C_gD	= (((fp_t)1.0 / CIE_y_w) * (CIE_x_w * (CIE_y_b - CIE_y_r) - CIE_y_w * (CIE_x_b - CIE_x_r) - CIE_x_r * CIE_y_b + CIE_x_b * CIE_y_r));
const fp_t CIE_C_bD	= (((fp_t)1.0 / CIE_y_w) * (CIE_x_w * (CIE_y_r - CIE_y_g) - CIE_y_w * (CIE_x_r - CIE_x_g) + CIE_x_r * CIE_y_g - CIE_x_g * CIE_y_r));

/* XYZ to RGB conversion matrix */
const fp_t xyz2rgbmat[3][3] = {
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
        virtual void generate_rays(const ray_trace_engine &r, ray &i, const line &n, const hit_t h, ray *const rl, ray *const rf, fp_t *const n_rl, fp_t *const n_rf) const = 0;

        /* Pure virtual shading function. To allow the shader to shade the current object */
        virtual void shade(const ray_trace_engine &r, ray &i, const line &n, const hit_t h, ext_colour_t *const c, const point_t &vt) const = 0;

        /* Pure virtual function to the allow the shader a combined SIMD packets traced secondary rays into the image */
        virtual void combind_secondary_rays(const ray_trace_engine &r, ext_colour_t &c, const ray *const rl, const ray *const rf, const ext_colour_t *const c_rl, const ext_colour_t *const c_rf, const fp_t *const n_rl, const fp_t *const n_rf) const = 0;

        /* Allow read transparency */
        const bool      is_transparent()    const   { return t; }

    private :
        const bool      t;  /* Is the material transparent */
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
inline fp_t direct_fresnell(const fp_t r, const fp_t t, const fp_t k)
{
    /* Some repeatedly used variables */
    fp_t r_sq = r * r;
    fp_t k_sq = k * k;
    fp_t t_sq = t * t;
    
    /* g^2 = n^2 - 1 + t^2; where n is the complex index of refraction */
    fp_t  re_g_sq = r_sq - k_sq - (fp_t)1.0 + t_sq;
    fp_t  im_g_sq = (fp_t)2.0 * r * k;
    
    /* Take the absolute of g^2; Where g^2 = g_re^2 + i * g_im^2 */
    fp_t abs_re_g_sq = fabs(re_g_sq);
    fp_t abs_im_g_sq = fabs(im_g_sq);

    fp_t abs_g_sq;
    if ((re_g_sq == (fp_t)0.0) && (im_g_sq == (fp_t)0.0))
    {
        abs_g_sq = (fp_t)0.0;
    }
    else if (abs_re_g_sq >= abs_im_g_sq)
    {
        fp_t d   = abs_im_g_sq / abs_re_g_sq;
        abs_g_sq = abs_re_g_sq * sqrt(1.0 + (d * d));
    }
    else
    {
        fp_t d   = abs_re_g_sq / abs_im_g_sq;
        abs_g_sq = abs_im_g_sq * sqrt(1.0 + (d * d));
    }
    
    /* s = (1 - t^2) / t */
    fp_t s = ((fp_t)1.0 - t_sq) / t;

    /* a = sqrt((abs(g^2) + re(g^2)) / 2) */
    /* b = sqrt((abs(g^2) - re(g^2)) / 2) */
    fp_t a    = sqrt((abs_g_sq + re_g_sq) * (fp_t)0.5);
    fp_t b    = sqrt((abs_g_sq - re_g_sq) * (fp_t)0.5);
    fp_t b_sq = b * b;
    
    /* Some repeatedly used Fresnel terms */
    fp_t a_m_t = a - t;
    fp_t a_p_t = a + t;
    fp_t a_m_s = a - s;
    fp_t a_p_s = a + s;
    
    fp_t f0 = (fp_t)0.5 * (((a_m_t * a_m_t) + b_sq)/((a_p_t * a_p_t) + b_sq));
    fp_t f1 = (fp_t)1.0 + (((a_m_s * a_m_s) + b_sq)/((a_p_s * a_p_s) + b_sq));
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
inline fp_t schlick_fresnell(const fp_t r, const fp_t t, const fp_t k)
{
    return (r + ((fp_t)1.0 - r) * pow((fp_t)1.0 - t, (fp_t)5.0));
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
inline fp_t rescaled_schlick_fresnell(const fp_t r, const fp_t t, const fp_t k)
{
    fp_t r_m1 = r - (fp_t)1.0;
    fp_t r_p1 = r + (fp_t)1.0;
    fp_t k_sq = k * k;
    
    return ((r_m1 * r_m1) + ((fp_t)4.0 * r * pow(((fp_t)1.0 - t), (fp_t)5.0)) + k_sq) / ((r_p1 * r_p1) + k_sq);
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
inline fp_t geometric_attenuation_factor(const fp_t nh, const fp_t nv, const fp_t nl, const fp_t vh)
{
    fp_t g0 = ((fp_t)2.0 * nh * nv ) / vh;
    fp_t g1 = ((fp_t)2.0 * nh * nl ) / vh;
    return min((fp_t)1.0, max((fp_t)0.0, min(g0, g1)));
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
inline fp_t gaussian_facet_distribution(const fp_t a, const fp_t m, const fp_t c)
{
    /* ce^-((alpha/m)^2) */
    fp_t a_div_m = a / m;
    return c * exp(-(a_div_m * a_div_m));
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
inline fp_t beckmann_facet_distribution(const fp_t a, const fp_t m, const fp_t c)
{
    fp_t cos_4_a  = pow(cos(a), (fp_t)4.0);
    fp_t tan_a    = tan(a);
    fp_t tan_a_sq = tan_a * tan_a;
    
    fp_t m_sq = m * m;
    
    /* 1/(m^2 * cos^4(alpha)) * e^-(tan^2(alpha)/m^2) */
    return (((fp_t)1.0 / (m_sq * cos_4_a)) * exp(-(tan_a_sq / m_sq)));
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
inline fp_t linear_colour_shift(const fp_t c0, const fp_t c90, const fp_t fa, const fp_t f0, const fp_t f90)
{
    /* calpha = c0 + ((c90 - c0) * (max(0,Falpha - F0)/F90- F0)) */
    return c0 + ((c90 - c0) * (max((fp_t)0.0, (fa - f0)) / (f90- f0)));
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
inline void black_body_temperature_to_cxy(const fp_t t, fp_t *const x, fp_t *const y)
{
    /* Convert temperature to CXY X value */
    fp_t c_x;
    fp_t t_sq = t    * t;
    fp_t t_3  = t_sq * t;
    if (((fp_t)1667.0 <= t) && (t <= (fp_t)4000.0))
    {
        c_x = (fp_t)-0.2661239 * ((fp_t)10e9 / t_3) - (fp_t)0.2343580 * ((fp_t)10e6 / t_sq) + (fp_t)0.8776956 * ((fp_t)10e3 / t) + (fp_t)0.179910;
    }
    else if (((fp_t)4000.0 <= t) && (t <= (fp_t)25000.0))
    {
        c_x = (fp_t)-3.0258469 * ((fp_t)10e9 / t_3) + (fp_t)2.1070379 * ((fp_t)10e6 / t_sq) + (fp_t)0.2226347 * ((fp_t)10e3 / t) + (fp_t)0.24039;
    }
    else
    {
        assert(false);
    }
    *x = c_x;
    
    /* Convert temperature and CXY X value to CXY Y value */
    fp_t c_x_sq = c_x    * c_x;
    fp_t c_x_3  = c_x_sq * c_x;
    if (((fp_t)1667.0 <= t) && (t <= (fp_t)2222.0))
    {
        *y = -(fp_t)1.1063814 * c_x_3 - (fp_t)1.34811020 * c_x_sq + (fp_t)2.18555832 * c_x - (fp_t)0.20219683;
    }
    else if (((fp_t)2222.0 <= t) && (t <= (fp_t)4000.0))
    {
        *y = (fp_t)-0.9549476 * c_x_3 - (fp_t)1.37418593 * c_x_sq + (fp_t)2.09137015 * c_x - (fp_t)0.16748867;
    }
    else if (((fp_t)4000.0 <= t) && (t <= (fp_t)25000.0))
    {
        *y =  (fp_t)3.0817580 * c_x_3 - (fp_t)5.87338670 * c_x_sq + (fp_t)3.75112997 * c_x - (fp_t)0.37001483;
    }
    else
    {
        assert(false);
    }
}

/* Function for colour space conversion */
inline ext_colour_t& cxy_to_rgb(const fp_t c_x, const fp_t c_y, const fp_t y, ext_colour_t *const rgb)
{
    fp_t cie[3], cout[3];
    
    cie[0] = y * c_x/c_y;
    cie[1] = y;
    cie[2] = y * ((fp_t)1.0/c_y - (fp_t)1.0) - cie[0];

    /* convert to RGB */
    cout[0] = (xyz2rgbmat[0][0] * cie[0]) + (xyz2rgbmat[0][1] * cie[1]) + (xyz2rgbmat[0][2] * cie[2]);
    cout[1] = (xyz2rgbmat[1][0] * cie[0]) + (xyz2rgbmat[1][1] * cie[1]) + (xyz2rgbmat[1][2] * cie[2]);
    cout[2] = (xyz2rgbmat[2][0] * cie[0]) + (xyz2rgbmat[2][1] * cie[1]) + (xyz2rgbmat[2][2] * cie[2]);

    /* Bound the rgb value */
    rgb->r = max((fp_t)0.0, min((fp_t)1.0, cout[0])) * (fp_t)255.0;
    rgb->g = max((fp_t)0.0, min((fp_t)1.0, cout[1])) * (fp_t)255.0;
    rgb->b = max((fp_t)0.0, min((fp_t)1.0, cout[2])) * (fp_t)255.0;
    return *rgb;
}
}; /* namespace raptor_raytracer */

#endif /* #ifndef __MATERIAL_H__ */
