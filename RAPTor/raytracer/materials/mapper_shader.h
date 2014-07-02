#ifndef __MAPPER_SHADER_H__
#define __MAPPER_SHADER_H__

#include "common.h"
#include "point_t.h"
#include "material.h"
#include "texture_mapper.h"
#include "line.h"
#include "ray.h"


/* Child of material used to attach multiple texture mapper */
/* This class with calculate illumination and pass this value to the texture mapper for shading */
class mapper_shader : public material
{
    public :
        mapper_shader(const list<texture_mapper *> &ctex, const list<texture_mapper *> &dtex, const list<texture_mapper *> &rtex, const list<texture_mapper *> &ttex, 
                      const ext_colour_t &rgb, const fp_t kd=0.0, const fp_t ks=0.0, const fp_t s=0.0, const fp_t tran=0.0, const fp_t ri=1.0, const fp_t rf=0.0, const fp_t td=0.0, const fp_t rfd=0.0) : 
            material(tran > (fp_t)0.0), ctex(ctex), dtex(dtex), rtex(rtex), ttex(ttex), rgb(rgb), kd(kd), ks(ks), s(s), tran(tran), ri(ri), rf(rf), td(td), rfd(rfd) 
            {
                /* A whole bunch of sanity checking asserts */
                /* Note rgb is stored in light_intensity in the format ( r, g, b ) */
                assert((rgb.r    >= (fp_t)0.0) &&  (rgb.r    <= (fp_t)255.0));                                /* Best the rgb values are in the correct range */
                assert((rgb.g    >= (fp_t)0.0) &&  (rgb.g    <= (fp_t)255.0));
                assert((rgb.b    >= (fp_t)0.0) &&  (rgb.b    <= (fp_t)255.0));
                assert((this->kd >= (fp_t)0.0) &&  (this->kd <= (fp_t)  1.0));
                assert((this->ks >= (fp_t)0.0) &&  (this->ks <= (fp_t)  1.0));
                assert((tran     >= (fp_t)0.0) &&  (tran     <= (fp_t)  1.0));
                assert((tran     == (fp_t)0.0) || ((this->ri >= (fp_t)  1.0) && (this->ri <= (fp_t)  5.0)));  /* 2.49 is the highest real refractive index */
                assert((this->rf >= (fp_t)0.0) &&  (this->rf <= (fp_t)  1.0));                                /* This limits the number of iterative refractions (there is no such thing as a perfect mirror) */
            };
            
        /* Tidy up */
        virtual ~mapper_shader() 
        { 
            while (!this->ctex.empty())
            {
                delete this->ctex.front();
                this->ctex.pop_front();
            }
            
            while (!this->dtex.empty())
            {
                delete this->dtex.front();
                this->dtex.pop_front();
            }
            
            while (!this->rtex.empty())
            {
                delete this->rtex.front();
                this->rtex.pop_front();
            }
            
            while (!this->ttex.empty())
            {
                delete this->ttex.front();
                this->ttex.pop_front();
            }
        };

        /* Function the allow the shader a chance to generate SIMD packets */
        void generate_rays(const ray_trace_engine &r, ray &i, const line &n, const hit_t h, ray *const rl, ray *const rf, fp_t *const n_rl, fp_t *const n_rf) const;

        /* Shading function. To allow the shader to shade the current object */
        void shade(const ray_trace_engine &r, ray &i, const line &n, const hit_t h, ext_colour_t *const c, const point_t &vt) const;

        /* Function to the allow the shader a combined SIMD packets traced secondary rays into the image */
        void combind_secondary_rays(const ray_trace_engine &r, ext_colour_t &c, const ray *const rl, const ray *const rf, const ext_colour_t *const c_rl, const ext_colour_t *const c_rf, const fp_t *const n_rl, const fp_t *const n_rf) const;

    private :
        list<texture_mapper *>          ctex;   /* A list of colour texture mappers         */
        list<texture_mapper *>          dtex;   /* A list of Kd texture mappers             */
        list<texture_mapper *>          rtex;   /* A list of reflectivity texture mappers   */
        list<texture_mapper *>          ttex;   /* A list of transparency texture mappers   */
        const ext_colour_t              rgb;    /* Base colour                              */
        const fp_t                      kd;     /* Diffuse co-efficient                     */
        const fp_t                      ks;     /* Specular co-efficient                    */
        const fp_t                      s;      /* Phong Shine (cosine power)               */
        const fp_t                      tran;   /* Transmittance                            */
        const fp_t                      ri;     /* Refractive index                         */
        const fp_t                      rf;     /* Reflectance                              */
        const fp_t                      td;     /* Transmitted diffuseness                  */
        const fp_t                      rfd;    /* Reflected diffuseness                    */
};

#endif /* #ifndef __MAPPER_SHADER_H__ */
