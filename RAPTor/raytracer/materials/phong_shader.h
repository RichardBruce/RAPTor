#ifndef __PHONG_SHADER_H__
#define __PHONG_SHADER_H__

#include "common.h"
#include "point_t.h"
#include "material.h"
#include "line.h"
#include "ray.h"


namespace raptor_raytracer
{
/* Pure virtual class for material data and shading */
class phong_shader : public material
{
    public :
        /* Constructor with white co-efficient */
        phong_shader(const ext_colour_t &rgb, const float kd = 0.0f, const float ks = 0.0f, const float s = 0.0f, const float tran = 0.0f, const float ri = 1.0f, const float rf = 0.0f, 
            const float td = 0.0f, const float rfd = 0.0f) : 
            material(tran > 0.0f), ka(ext_colour_t(0.0f, 0.0f, 0.0f)), kd(rgb * kd), ks(rgb * ks), s(s), tran(tran), ri(ri), rf(rf), td(td), rfd(rfd) 
            {
                /* A whole bunch of sanity checking asserts */
                /* Note rgb is stored in light_intensity in the format ( r, g, b ) */
                assert((rgb.r    >= 0.0f) &&  (rgb.r    <= 255.0f ));                                /* Best the rgb values are in the correct range */
                assert((rgb.g    >= 0.0f) &&  (rgb.g    <= 255.0f ));
                assert((rgb.b    >= 0.0f) &&  (rgb.b    <= 255.0f ));
                assert((kd       >= 0.0f) &&  (kd       <=   1.0f ));
                assert((ks       >= 0.0f) &&  (ks       <=   1.0f ));
                assert((tran     >= 0.0f) &&  (tran     <=   0.95f));
                assert((tran     == 0.0f) || ((this->ri >=   1.0f ) && (this->ri <=   5.0f)));  /* 2.49 is the highest real refractive index */
                assert((this->rf >= 0.0f) &&  (this->rf <=   0.95f));                                /* This limits the number of iterative refractions (there is no such thing as a perfect mirror) */
            };

        /* Constructor with coloured co-efficient */
        phong_shader(const ext_colour_t &ka, const ext_colour_t &kd, const ext_colour_t &ks, const float s = 0.0f, const float tran = 0.0f, const float ri = 1.0f, const float rf = 0.0f, 
            const float td = 0.0f, const float rfd = 0.0f) : 
            material(tran > 0.0f), ka(ka), kd(kd), ks(ks), s(s), tran(tran), ri(ri), rf(rf), td(td), rfd(rfd) 
            {
                /* A whole bunch of sanity checking asserts */
                /* Note rgb is stored in light_intensity in the format ( r, g, b ) */
                assert((ka.r     >= 0.0f) &&  (ka.r     <= 255.0f ));                                /* Best the rgb values are in the correct range */
                assert((ka.g     >= 0.0f) &&  (ka.g     <= 255.0f ));
                assert((ka.b     >= 0.0f) &&  (ka.b     <= 255.0f ));
                assert((kd.r     >= 0.0f) &&  (kd.r     <= 255.0f )); 
                assert((kd.g     >= 0.0f) &&  (kd.g     <= 255.0f ));
                assert((kd.b     >= 0.0f) &&  (kd.b     <= 255.0f ));
                assert((ks.r     >= 0.0f) &&  (ks.r     <= 255.0f ));
                assert((ks.g     >= 0.0f) &&  (ks.g     <= 255.0f ));
                assert((ks.b     >= 0.0f) &&  (ks.b     <= 255.0f ));
                assert((tran     >= 0.0f) &&  (tran     <=   0.95f));
                assert((tran     == 0.0f) || ((this->ri >=   1.0f ) && (this->ri <=   5.0f)));  /* 2.49 is the highest real refractive index */
                assert((this->rf >= 0.0f) &&  (this->rf <=   0.95f));                                /* This limits the number of iterative refractions (there is no such thing as a perfect mirror) */
            };

        /* Destructor */
        virtual ~phong_shader() { };

        /* Function the allow the shader a chance to generate SIMD packets */
        void generate_rays(const ray_trace_engine &r, ray &i, const line &n, const point_t &vt, const hit_t h, secondary_ray_data *const rl, secondary_ray_data *const rf) const;

        /* Shading function. To allow the shader to shade the current object */
        void shade(const ray_trace_engine &r, ray &i, const line &n, const hit_t h, ext_colour_t *const c, const point_t &vt) const;

        /* Function to the allow the shader a combined SIMD packets traced secondary rays into the image */
        void combind_secondary_rays(const ray_trace_engine &r, ext_colour_t *const c, const secondary_ray_data &rl, const secondary_ray_data &rf) const;
        
    private :
        const ext_colour_t  ka;     /* Ambient co-efficient         */
        const ext_colour_t  kd;     /* Diffuse co-efficient         */
        const ext_colour_t  ks;     /* Specular co-efficient        */
        const float         s;      /* Phong Shine (cosine power)   */
        const float         tran;   /* Transmittance                */
        const float         ri;     /* Refractive index             */
        const float         rf;     /* Reflectance                  */
        const float         td;     /* Transmitted diffuseness      */
        const float         rfd;    /* Reflected diffuseness        */
};
}; /* namespace raptor_raytracer */

#endif /* #ifndef __PHONG_SHADER_H__ */
