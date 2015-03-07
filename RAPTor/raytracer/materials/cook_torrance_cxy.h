#ifndef __COOK_TORRANCE_CXY_H__
#define __COOK_TORRANCE_CXY_H__

#include "common.h"
#include "point_t.h"
#include "material.h"
#include "line.h"
#include "ray.h"


namespace raptor_raytracer
{
/* Pure virtual class for material data and shading */
class cook_torrance_cxy : public material
{
    public :
        /* Constructor for non light emmitting object */
        cook_torrance_cxy(const float x = 0.33f, const float y = 0.33f, const float rd = 0.5f, const float td = 0.0f, const float rs = 0.0f, const float ts = 0.0f, const float sr = 0.0f, const float tr = 0.0f, const float ri_r = 1.0f, const float ri_i = 0.0f) : 
            material(ts > 0.0f), x(x), y(y), rd(rd), td(td), rs(rs), ts(ts), sr(sr), tr(tr), ri_r(ri_r), ri_i(ri_i)
            {
                /* A whole bunch of sanity checking asserts */
                /* Note x, y and rd are stored in light_intensity in the format ( x, rd, y ) */
                assert((      x    >= 0.0f) && (      x    <= 1.0f));
                assert((      y    >= 0.0f) && (      y    <= 1.0f));
                assert((      rd   >= 0.0f) && (      rd   <= 1.0f));
                assert((this->td   >= 0.0f) && (this->td   <= 1.0f));
                assert((this->rs   >= 0.0f) && (this->rs   <= 1.0f));
                assert((this->ts   >= 0.0f) && (this->ts   <= 1.0f));
                assert((this->sr   >= 0.0f) && (this->sr   <= 0.5f));   /* In reality numbers greater than 0.2 are rare */
                assert((this->tr   >= 0.0f) && (this->tr   <= 1.0f));
                assert((this->ri_r >= 0.0f) && (this->ri_r <= 5.0f));
                assert((this->ri_i >= 0.0f) && (this->ri_i <= 1.0f));
            };
        virtual ~cook_torrance_cxy() { };

        /* Function the allow the shader a chance to generate SIMD packets */
        void generate_rays(const ray_trace_engine &r, ray &i, point_t *const n, const point_t &vt, const hit_t h, secondary_ray_data *const rl, secondary_ray_data *const rf) const override;

        /* Shading function. To allow the shader to shade the current object */
        void shade(const ray_trace_engine &r, ray &i, const point_t &n, const hit_t h, ext_colour_t *const c, const point_t &vt) const override;

        /* Function to the allow the shader a combined SIMD packets traced secondary rays into the image */
        void combind_secondary_rays(const ray_trace_engine &r, ext_colour_t *const c, const secondary_ray_data &rl, const secondary_ray_data &rf) const override;

    private :
        const float x;      /* X chroma                                 */
        const float y;      /* Y chroma                                 */
        const float rd;     /* Diffuse reflectance                      */
        const float td;     /* Diffuse transmittance                    */
        const float rs;     /* Specular reflectance                     */
        const float ts;     /* Specular transmittance                   */
        const float sr;     /* Surface roughness                        */
        const float tr;     /* Effective transmitted surface roughness  */
        const float ri_r;   /* Real refractive index                    */
        const float ri_i;   /* Imaginery refractive index               */
};
}; /* namespace raptor_raytracer */

#endif /* #ifndef __COOK_TORRANCE_CXY_H__ */
