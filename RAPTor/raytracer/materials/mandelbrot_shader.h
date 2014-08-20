#ifndef __MANDELBROT_SHADER_H__
#define __MANDELBROT_SHADER_H__

#include "common.h"
#include "material.h"
#include "line.h"
#include "ray.h"


namespace raptor_raytracer
{
/* Pure virtual class for material data and shading */
class mandelbrot_shader : public material
{
    public :
        /* Constructor for non light emmitting object */
        mandelbrot_shader(const fp_t x, const fp_t y, const int i) : 
            material(), x(x), y(y), i(i)
            {

            };
        virtual ~mandelbrot_shader() { };

        /* Function the allow the shader a chance to generate SIMD packets */
        void generate_rays(const ray_trace_engine &r, ray &i, const line &n, const hit_t h, ray *const rl, ray *const rf, fp_t *const n_rl, fp_t *const n_rf) const;

        /* Shading function. To allow the shader to shade the current object */
        void shade(const ray_trace_engine &r, ray &i, const line &n, const hit_t h, ext_colour_t *const c, const point_t &vt) const;

        /* Function to the allow the shader a combined SIMD packets traced secondary rays into the image */
        void combind_secondary_rays(const ray_trace_engine &r, ext_colour_t &c, const ray *const rl, const ray *const rf, const ext_colour_t *const c_rl, const ext_colour_t *const c_rf, const fp_t *const n_rl, const fp_t *const n_rf) const;

    private :
        fp_t    x;      /* X width */
        fp_t    y;      /* Y width */
        int     i;      /* Iteration limit */
};
}; /* namespace raptor_raytracer */

#endif /* #ifndef __MANDELBROT_SHADER_H__ */
