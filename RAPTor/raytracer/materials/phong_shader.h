#ifndef __PHONG_SHADER_H__
#define __PHONG_SHADER_H__

#include "common.h"
#include "point_t.h"
#include "material.h"
#include "line.h"
#include "ray.h"


/* Pure virtual class for material data and shading */
class phong_shader : public material
{
    public :
        /* Constructor with white co-efficient */
        phong_shader(const ext_colour_t &rgb, const fp_t kd=0.0, const fp_t ks=0.0, const fp_t s=0.0, const fp_t tran=0.0, const fp_t ri=1.0, const fp_t rf=0.0, const fp_t td=0.0, const fp_t rfd=0.0) : 
            material(tran > (fp_t)0.0), ka(ext_colour_t(0.0, 0.0, 0.0)), kd(rgb * kd), ks(rgb * ks), s(s), tran(tran), ri(ri), rf(rf), td(td), rfd(rfd) 
            {
                /* A whole bunch of sanity checking asserts */
                /* Note rgb is stored in light_intensity in the format ( r, g, b ) */
                assert((rgb.r    >= (fp_t)0.0) &&  (rgb.r    <= (fp_t)255.0 ));                                /* Best the rgb values are in the correct range */
                assert((rgb.g    >= (fp_t)0.0) &&  (rgb.g    <= (fp_t)255.0 ));
                assert((rgb.b    >= (fp_t)0.0) &&  (rgb.b    <= (fp_t)255.0 ));
                assert((kd       >= (fp_t)0.0) &&  (kd       <= (fp_t)  1.0 ));
                assert((ks       >= (fp_t)0.0) &&  (ks       <= (fp_t)  1.0 ));
                assert((tran     >= (fp_t)0.0) &&  (tran     <= (fp_t)  0.95));
                assert((tran     == (fp_t)0.0) || ((this->ri >= (fp_t)  1.0 ) && (this->ri <= (fp_t)  5.0)));  /* 2.49 is the highest real refractive index */
                assert((this->rf >= (fp_t)0.0) &&  (this->rf <= (fp_t)  0.95));                                /* This limits the number of iterative refractions (there is no such thing as a perfect mirror) */
            };

        /* Constructor with coloured co-efficient */
        phong_shader(const ext_colour_t &ka, const ext_colour_t &kd, const ext_colour_t &ks, const fp_t s=0.0, const fp_t tran=0.0, const fp_t ri=1.0, const fp_t rf=0.0, const fp_t td=0.0, const fp_t rfd=0.0) : 
            material(tran > (fp_t)0.0), ka(ka), kd(kd), ks(ks), s(s), tran(tran), ri(ri), rf(rf), td(td), rfd(rfd) 
            {
                /* A whole bunch of sanity checking asserts */
                /* Note rgb is stored in light_intensity in the format ( r, g, b ) */
                assert((ka.r     >= (fp_t)0.0) &&  (ka.r     <= (fp_t)255.0 ));                                /* Best the rgb values are in the correct range */
                assert((ka.g     >= (fp_t)0.0) &&  (ka.g     <= (fp_t)255.0 ));
                assert((ka.b     >= (fp_t)0.0) &&  (ka.b     <= (fp_t)255.0 ));
                assert((kd.r     >= (fp_t)0.0) &&  (kd.r     <= (fp_t)255.0 )); 
                assert((kd.g     >= (fp_t)0.0) &&  (kd.g     <= (fp_t)255.0 ));
                assert((kd.b     >= (fp_t)0.0) &&  (kd.b     <= (fp_t)255.0 ));
                assert((ks.r     >= (fp_t)0.0) &&  (ks.r     <= (fp_t)255.0 ));
                assert((ks.g     >= (fp_t)0.0) &&  (ks.g     <= (fp_t)255.0 ));
                assert((ks.b     >= (fp_t)0.0) &&  (ks.b     <= (fp_t)255.0 ));
                assert((tran     >= (fp_t)0.0) &&  (tran     <= (fp_t)  0.95));
                assert((tran     == (fp_t)0.0) || ((this->ri >= (fp_t)  1.0 ) && (this->ri <= (fp_t)  5.0)));  /* 2.49 is the highest real refractive index */
                assert((this->rf >= (fp_t)0.0) &&  (this->rf <= (fp_t)  0.95));                                /* This limits the number of iterative refractions (there is no such thing as a perfect mirror) */
            };

        /* Destructor */
        virtual ~phong_shader() { };

        /* Function the allow the shader a chance to generate SIMD packets */
        void generate_rays(const ray_trace_engine &r, ray &i, const line &n, const hit_t h, ray *const rl, ray *const rf, fp_t *const n_rl, fp_t *const n_rf) const;

        /* Shading function. To allow the shader to shade the current object */
        void shade(const ray_trace_engine &r, ray &i, const line &n, const hit_t h, ext_colour_t *const c, const point_t &vt) const;

        /* Function to the allow the shader a combined SIMD packets traced secondary rays into the image */
        void combind_secondary_rays(const ray_trace_engine &r, ext_colour_t &c, const ray *const rl, const ray *const rf, const ext_colour_t *const c_rl, const ext_colour_t *const c_rf, const fp_t *const n_rl, const fp_t *const n_rf) const;
        
    private :
        const ext_colour_t  ka;     /* Ambient co-efficient         */
        const ext_colour_t  kd;     /* Diffuse co-efficient         */
        const ext_colour_t  ks;     /* Specular co-efficient        */
        const fp_t          s;      /* Phong Shine (cosine power)   */
        const fp_t          tran;   /* Transmittance                */
        const fp_t          ri;     /* Refractive index             */
        const fp_t          rf;     /* Reflectance                  */
        const fp_t          td;     /* Transmitted diffuseness      */
        const fp_t          rfd;    /* Reflected diffuseness        */
};

#endif /* #ifndef __PHONG_SHADER_H__ */
