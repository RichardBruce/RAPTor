#ifndef __MAPPER_SHADER_H__
#define __MAPPER_SHADER_H__

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
class mapper_shader : public material
{
    public :
        mapper_shader(const std::vector<texture_mapper *> &btex, const std::vector<texture_mapper *> &ctex, const std::vector<texture_mapper *> &dtex, const std::vector<texture_mapper *> &stex, 
                      const std::vector<texture_mapper *> &rtex, const std::vector<texture_mapper *> &ttex, 
                      const ext_colour_t &rgb, const float kd = 0.0f, const float ks = 0.0f, const float s = 0.0f, const float tran = 0.0f, const float ri = 1.0f, const float rf = 0.0f, 
                      const float clrh = 0.0f, const float clrf = 0.0f, const float bump = 1.0f, const float td = 0.0f, const float rfd = 0.0f) : 
            material(tran > 0.0f), _btex(btex), _ctex(ctex), _dtex(dtex), _stex(stex), _rtex(rtex), _ttex(ttex), _rgb(rgb), _kd(kd), _ks(ks), _s(s), _tran(tran), _ri(ri), _rf(rf), _clrh(clrh), _clrf(clrf), _bump(bump), _td(td), _rfd(rfd) 
            {
                /* A whole bunch of sanity checking asserts */
                /* Note rgb is stored in light_intensity in the format ( r, g, b ) */
                assert(_kd      >= 0.0f);
                assert((rgb.r   >= 0.0f) &&  (rgb.r <= 255.0f));                                /* Best the rgb values are in the correct range */
                assert((rgb.g   >= 0.0f) &&  (rgb.g <= 255.0f));
                assert((rgb.b   >= 0.0f) &&  (rgb.b <= 255.0f));
                assert((_ks     >= 0.0f) &&  (_ks   <=   1.0f));
                assert((tran    >= 0.0f) &&  (tran  <=   1.0f));
                assert((tran    == 0.0f) || ((_ri   >=   1.0f) && (_ri <= 5.0f)));              /* 2.49 is the highest real refractive index */
                assert((_rf     >= 0.0f) &&  (_rf   <=   1.0f));                                /* This limits the number of iterative refractions (there is no such thing as a perfect mirror) */
            };
            
        /* Tidy up */
        virtual ~mapper_shader() 
        {
            for (auto m : _btex)
            {
                delete m;
            }

            for (auto m : _ctex)
            {
                delete m;
            }
            
            for (auto m : _dtex)
            {
                delete m;
            }
            
            for (auto m : _stex)
            {
                delete m;
            }
            
            for (auto m : _rtex)
            {
                delete m;
            }
            
            for (auto m : _ttex)
            {
                delete m;
            }
        };

        /* Function the allow the shader a chance to generate SIMD packets */
        void generate_rays(const ray_trace_engine &r, ray &i, point_t *const n, const point_t &vt, const hit_t h, secondary_ray_data *const rl, secondary_ray_data *const rf) const override;

        /* Shading function. To allow the shader to shade the current object */
        void shade(const ray_trace_engine &r, ray &i, const point_t &n, const hit_t h, ext_colour_t *const c, const point_t &vt) const override;

        /* Function to the allow the shader a combined SIMD packets traced secondary rays into the image */
        void combind_secondary_rays(const ray_trace_engine &r, ext_colour_t *const c, const secondary_ray_data &rl, const secondary_ray_data &rf) const override;

    private :
        std::vector<texture_mapper *>   _btex;  /* A list of bump texture mappers           */
        std::vector<texture_mapper *>   _ctex;  /* A list of colour texture mappers         */
        std::vector<texture_mapper *>   _dtex;  /* A list of Kd texture mappers             */
        std::vector<texture_mapper *>   _stex;  /* A list of s texture mappers              */
        std::vector<texture_mapper *>   _rtex;  /* A list of reflectivity texture mappers   */
        std::vector<texture_mapper *>   _ttex;  /* A list of transparency texture mappers   */
        const ext_colour_t              _rgb;   /* Base colour                              */
        const float                     _kd;    /* Diffuse co-efficient                     */
        const float                     _ks;    /* Specular co-efficient                    */
        const float                     _s;     /* Phong Shine (cosine power)               */
        const float                     _tran;  /* Transmittance                            */
        const float                     _ri;    /* Refractive index                         */
        const float                     _rf;    /* Reflectance                              */
        const float                     _clrh;  /* Colour blending of specular highlights   */
        const float                     _clrf;  /* Colour blending for tranmittance         */
        const float                     _bump;  /* Height of bumps for dump mapping         */
        const float                     _td;    /* Transmitted diffuseness                  */
        const float                     _rfd;   /* Reflected diffuseness                    */
};
}; /* namespace raptor_raytracer */

#endif /* #ifndef __MAPPER_SHADER_H__ */
