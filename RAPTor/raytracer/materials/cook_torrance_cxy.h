#ifndef __COOK_TORRANCE_CXY_H__
#define __COOK_TORRANCE_CXY_H__

#include "common.h"
#include "point_t.h"
#include "material.h"
#include "line.h"
#include "ray.h"


/* Pure virtual class for material data and shading */
class cook_torrance_cxy : public material
{
    public :
        /* Constructor for non light emmitting object */
        cook_torrance_cxy(const fp_t x=0.33, const fp_t y=0.33, const fp_t rd=0.5, const fp_t td=0.0, const fp_t rs=0.0, const fp_t ts=0.0, const fp_t sr=0.0, const fp_t tr=0.0, const fp_t ri_r=1.0, const fp_t ri_i=0.0) : 
            material(ts > (fp_t)0.0), x(x), y(y), rd(rd), td(td), rs(rs), ts(ts), sr(sr), tr(tr), ri_r(ri_r), ri_i(ri_i)
            {
                /* A whole bunch of sanity checking asserts */
                /* Note x, y and rd are stored in light_intensity in the format ( x, rd, y ) */
                assert((      x    >= (fp_t)0.0) && (      x    <= (fp_t)1.0));
                assert((      y    >= (fp_t)0.0) && (      y    <= (fp_t)1.0));
                assert((      rd   >= (fp_t)0.0) && (      rd   <= (fp_t)1.0));
                assert((this->td   >= (fp_t)0.0) && (this->td   <= (fp_t)1.0));
                assert((this->rs   >= (fp_t)0.0) && (this->rs   <= (fp_t)1.0));
                assert((this->ts   >= (fp_t)0.0) && (this->ts   <= (fp_t)1.0));
                assert((this->sr   >= (fp_t)0.0) && (this->sr   <= (fp_t)0.5));   /* In reality numbers greater than 0.2 are rare */
                assert((this->tr   >= (fp_t)0.0) && (this->tr   <= (fp_t)1.0));
                assert((this->ri_r >= (fp_t)0.0) && (this->ri_r <= (fp_t)5.0));
                assert((this->ri_i >= (fp_t)0.0) && (this->ri_i <= (fp_t)1.0));
            };
        virtual ~cook_torrance_cxy() { };

        /* Function the allow the shader a chance to generate SIMD packets */
        void generate_rays(const ray_trace_engine &r, ray &i, const line &n, const hit_t h, ray *const rl, ray *const rf, fp_t *const n_rl, fp_t *const n_rf) const;

        /* Shading function. To allow the shader to shade the current object */
        void shade(const ray_trace_engine &r, ray &i, const line &n, const hit_t h, ext_colour_t *const c, const point_t &vt) const;

        /* Function to the allow the shader a combined SIMD packets traced secondary rays into the image */
        void combind_secondary_rays(const ray_trace_engine &r, ext_colour_t &c, const ray *const rl, const ray *const rf, const ext_colour_t *const c_rl, const ext_colour_t *const c_rf, const fp_t *const n_rl, const fp_t *const n_rf) const;

    private :
        const fp_t  x;      /* X chroma                                 */
        const fp_t  y;      /* Y chroma                                 */
        const fp_t  rd;     /* Diffuse reflectance                      */
        const fp_t  td;     /* Diffuse transmittance                    */
        const fp_t  rs;     /* Specular reflectance                     */
        const fp_t  ts;     /* Specular transmittance                   */
        const fp_t  sr;     /* Surface roughness                        */
        const fp_t  tr;     /* Effective transmitted surface roughness  */
        const fp_t  ri_r;   /* Real refractive index                    */
        const fp_t  ri_i;   /* Imaginery refractive index               */
};

#endif /* #ifndef __COOK_TORRANCE_CXY_H__ */
