#pragma once

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
        mandelbrot_shader(const float x, const float y, const int i) : 
            material(), x(x), y(y), i(i) {  };
        virtual ~mandelbrot_shader() { };

        /* Function the allow the shader a chance to generate SIMD packets */
        void generate_rays(const ray_trace_engine &r, ray &i, point_t<> *const n, const point_t<> &vt, const hit_t h, secondary_ray_data *const rl, secondary_ray_data *const rf) const override;

        /* Shading function. To allow the shader to shade the current object */
        void shade(const ray_trace_engine &r, ray &i, const point_t<> &n, const hit_t h, ext_colour_t *const c, const point_t<> &vt) const override;

        /* Function to the allow the shader a combined SIMD packets traced secondary rays into the image */
        void combind_secondary_rays(const ray_trace_engine &r, ext_colour_t *const c, const secondary_ray_data &rl, const secondary_ray_data &rf) const override;

    private :
        float   x;      /* X width */
        float   y;      /* Y width */
        int     i;      /* Iteration limit */
};
}; /* namespace raptor_raytracer */
