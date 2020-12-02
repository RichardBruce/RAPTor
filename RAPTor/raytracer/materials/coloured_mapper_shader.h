#pragma once

#include "common.h"
#include "point_t.h"
#include "material.h"
#include "texture_mapper.h"
#include "line.h"
#include "ray.h"


namespace raptor_raytracer
{
/* Forward delcarations */
class secondary_ray_data;

/* Child of material used to attach multiple texture mapper */
/* This class with calculate illumination and pass this value to the texture mapper for shading */
class coloured_mapper_shader : public material
{
    public :
        coloured_mapper_shader(const ext_colour_t& ka, const ext_colour_t &kd, const ext_colour_t &ks = 0.0f, const float ns = 0.0f, const float tran = 0.0f, const float ri = 1.0f, 
            const float rf = 0.0f, const float td = 0.0f, const float rfd = 0.0f, const texture_mapper * const t_ka = nullptr, const texture_mapper * const t_kd = nullptr, 
            const texture_mapper * const t_ks = nullptr, const texture_mapper * const t_ns = nullptr, const texture_mapper * const t_refl = nullptr, const texture_mapper * const t_d = nullptr) : 
            material(tran > 0.0f), t_ka(t_ka), t_kd(t_kd), t_ks(t_ks), t_ns(t_ns), t_refl(t_refl), t_d(t_d), ka(ka), kd(kd), ks(ks), ns(ns), tran(tran), ri(ri), rf(rf), td(td), rfd(rfd)
            {
                /* A whole bunch of sanity checking asserts */
                /* Note rgb is stored in light_intensity in the format ( r, g, b ) */
                assert((this->ka.r >= 0.0f) &&  (this->ka.r <= 255.0f));                                /* Best the rgb values are in the correct range */
                assert((this->ka.g >= 0.0f) &&  (this->ka.g <= 255.0f));
                assert((this->ka.b >= 0.0f) &&  (this->ka.b <= 255.0f));
                assert((this->kd.r >= 0.0f) &&  (this->kd.r <= 255.0f));
                assert((this->kd.g >= 0.0f) &&  (this->kd.g <= 255.0f));
                assert((this->kd.b >= 0.0f) &&  (this->kd.b <= 255.0f));
                assert((this->ks.r >= 0.0f) &&  (this->ks.r <= 255.0f));
                assert((this->ks.g >= 0.0f) &&  (this->ks.g <= 255.0f));
                assert((this->ks.b >= 0.0f) &&  (this->ks.b <= 255.0f));
                assert((this->tran >= 0.0f) &&  (this->tran <=   1.0f));
                assert((this->tran == 0.0f) || ((this->ri   >=   1.0f) && (this->ri <= 5.0f))); /* 2.49 is the highest real refractive index */
                assert((this->rf   >= 0.0f) &&  (this->rf   <=   1.0f));                             /* This limits the number of iterative refractions (there is no such thing as a perfect mirror) */
            };
            
        /* Tidy up */
        virtual ~coloured_mapper_shader() 
        {
            if (this->t_ka != nullptr)
            {
                delete this->t_ka;
                this->t_ka = nullptr;
            }

            if (this->t_kd != nullptr)
            {
                delete this->t_kd;
                this->t_kd = nullptr;
            }

            if (this->t_ks != nullptr)
            {
                delete this->t_ks;
                this->t_ks = nullptr;
            }

            if (this->t_ns != nullptr)
            {
                delete this->t_ns;
                this->t_ns = nullptr;
            }
            
            if (this->t_refl != nullptr)
            {
                delete this->t_refl;
                this->t_refl = nullptr;
            }
            
            if (this->t_d != nullptr)
            {
                delete this->t_d;
                this->t_d = nullptr;
            }
        };

        /* Function the allow the shader a chance to generate SIMD packets */
        void generate_rays(const ray_trace_engine &r, ray &i, point_t<> *const n, const point_t<> &vt, const hit_t h, secondary_ray_data *const rl, secondary_ray_data *const rf) const override;

        /* Shading function. To allow the shader to shade the current object */
        void shade(const ray_trace_engine &r, ray &i, const point_t<> &n, const hit_t h, ext_colour_t *const c, const point_t<> &vt) const override;

        /* Function to the allow the shader a combined SIMD packets traced secondary rays into the image */
        void combind_secondary_rays(const ray_trace_engine &r, ext_colour_t *const c, const secondary_ray_data &rl, const secondary_ray_data &rf) const override;

    private :
        const texture_mapper    *   t_ka;   /* Texture mapper to map Ka     */
        const texture_mapper    *   t_kd;   /* Texture mapper to map Kd     */
        const texture_mapper    *   t_ks;   /* Texture mapper to map Ks     */
        const texture_mapper    *   t_ns;   /* Texture mapper to map Ns     */
        const texture_mapper    *   t_refl; /* Texture mapper to map refl   */
        const texture_mapper    *   t_d;    /* Texture mapper to map d      */
        const ext_colour_t          ka;     /* Ambient co-efficient         */
        const ext_colour_t          kd;     /* Diffuse co-efficient         */
        const ext_colour_t          ks;     /* Specular co-efficient        */
        const float                 ns;     /* Phong Shine (cosine power)   */
        const float                 tran;   /* Transmittance                */
        const float                 ri;     /* Refractive index             */
        const float                 rf;     /* Reflectance                  */
        const float                 td;     /* Transmitted diffuseness      */
        const float                 rfd;    /* Reflected diffuseness        */
};
}; /* namespace raptor_raytracer */
