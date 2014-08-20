#ifndef __COLOURED_MAPPER_SHADER_H__
#define __COLOURED_MAPPER_SHADER_H__

#include "common.h"
#include "point_t.h"
#include "material.h"
#include "texture_mapper.h"
#include "line.h"
#include "ray.h"


namespace raptor_raytracer
{
/* Child of material used to attach multiple texture mapper */
/* This class with calculate illumination and pass this value to the texture mapper for shading */
class coloured_mapper_shader : public material
{
    public :
        coloured_mapper_shader(const ext_colour_t& ka, const ext_colour_t &kd, const ext_colour_t &ks=0.0, const fp_t ns=0.0, const fp_t tran=0.0, const fp_t ri=1.0, const fp_t rf=0.0, const fp_t td=0.0, const fp_t rfd=0.0,
                      const texture_mapper * const t_ka=NULL, const texture_mapper * const t_kd=NULL, const texture_mapper * const t_ks=NULL, const texture_mapper * const t_ns=NULL) : 
            material(tran > (fp_t)0.0), t_ka(t_ka), t_kd(t_kd), t_ks(t_ks), t_ns(t_ns), ka(ka), kd(kd), ks(ks), ns(ns), tran(tran), ri(ri), rf(rf), td(td), rfd(rfd) 
            {
                /* A whole bunch of sanity checking asserts */
                /* Note rgb is stored in light_intensity in the format ( r, g, b ) */
                assert((this->ka.r >= (fp_t)0.0) &&  (this->ka.r <= (fp_t)255.0));                                /* Best the rgb values are in the correct range */
                assert((this->ka.g >= (fp_t)0.0) &&  (this->ka.g <= (fp_t)255.0));
                assert((this->ka.b >= (fp_t)0.0) &&  (this->ka.b <= (fp_t)255.0));
                assert((this->kd.r >= (fp_t)0.0) &&  (this->kd.r <= (fp_t)255.0));
                assert((this->kd.g >= (fp_t)0.0) &&  (this->kd.g <= (fp_t)255.0));
                assert((this->kd.b >= (fp_t)0.0) &&  (this->kd.b <= (fp_t)255.0));
                assert((this->ks.r >= (fp_t)0.0) &&  (this->ks.r <= (fp_t)255.0));
                assert((this->ks.g >= (fp_t)0.0) &&  (this->ks.g <= (fp_t)255.0));
                assert((this->ks.b >= (fp_t)0.0) &&  (this->ks.b <= (fp_t)255.0));
                assert((tran       >= (fp_t)0.0) &&  (tran       <= (fp_t)  1.0));
                assert((tran       == (fp_t)0.0) || ((this->ri   >= (fp_t)  1.0) && (this->ri <= (fp_t)5.0))); /* 2.49 is the highest real refractive index */
                assert((this->rf   >= (fp_t)0.0) &&  (this->rf   <= (fp_t)  1.0));                             /* This limits the number of iterative refractions (there is no such thing as a perfect mirror) */
            };
            
        /* Tidy up */
        virtual ~coloured_mapper_shader() 
        {
	    if (this->t_ka != NULL)
            {
                delete this->t_ka;
                this->t_ka = NULL;
            }

	    if (this->t_kd != NULL)
            {
                delete this->t_kd;
                this->t_kd = NULL;
            }

	    if (this->t_ks != NULL)
            {
                delete this->t_ks;
                this->t_ks = NULL;
            }

	    if (this->t_ns != NULL)
            {
                delete this->t_ns;
                this->t_ns = NULL;
            }
        };

        /* Function the allow the shader a chance to generate SIMD packets */
        void generate_rays(const ray_trace_engine &r, ray &i, const line &n, const hit_t h, ray *const rl, ray *const rf, fp_t *const n_rl, fp_t *const n_rf) const;

        /* Shading function. To allow the shader to shade the current object */
        void shade(const ray_trace_engine &r, ray &i, const line &n, const hit_t h, ext_colour_t *const c, const point_t &vt) const;

        /* Function to the allow the shader a combined SIMD packets traced secondary rays into the image */
        void combind_secondary_rays(const ray_trace_engine &r, ext_colour_t &c, const ray *const rl, const ray *const rf, const ext_colour_t *const c_rl, const ext_colour_t *const c_rf, const fp_t *const n_rl, const fp_t *const n_rf) const;

    private :
        const texture_mapper    *   t_ka;   /* Texture mapper to map Ka     */
        const texture_mapper    *   t_kd;   /* Texture mapper to map Kd     */
        const texture_mapper    *   t_ks;   /* Texture mapper to map Ks     */
        const texture_mapper    *   t_ns;   /* Texture mapper to map Ns     */
        const ext_colour_t          ka;     /* Ambient co-efficient         */
        const ext_colour_t          kd;     /* Diffuse co-efficient         */
        const ext_colour_t          ks;     /* Specular co-efficient        */
        const fp_t                  ns;     /* Phong Shine (cosine power)   */
        const fp_t                  tran;   /* Transmittance                */
        const fp_t                  ri;     /* Refractive index             */
        const fp_t                  rf;     /* Reflectance                  */
        const fp_t                  td;     /* Transmitted diffuseness      */
        const fp_t                  rfd;    /* Reflected diffuseness        */
};
}; /* namespace raptor_raytracer */

#endif /* #ifndef __COLOURED_MAPPER_SHADER_H__ */
