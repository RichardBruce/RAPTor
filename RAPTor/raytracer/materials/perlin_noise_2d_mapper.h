#pragma once

/* Common headers */
#include "perlin_noise_2d.h"

/* Ray tracer headers */
#include "common.h"
#include "ext_colour_t.h"
#include "point_t.h"
#include "material.h"
#include "line.h"
#include "ray.h"


namespace raptor_raytracer
{
/* Pure virtual class for material data and shading */
class perlin_noise_2d_mapper : public material
{
    public :
        perlin_noise_2d_mapper(const ext_colour_t &rgb, const float p, const float z, const unsigned int w, const unsigned int h, const int o, const int s) : 
            material(), perlin(s), rgb(rgb), p(p), z(z), w(w), h(h), o(o)
            {
                /* Normalize colour multipliers */
                float max_color_multiplier = this->rgb.r > this->rgb.g ? this->rgb.r : this->rgb.g;
                max_color_multiplier = this->rgb.b > max_color_multiplier ? this->rgb.b : max_color_multiplier;
                this->rgb.r /= max_color_multiplier;
                this->rgb.g /= max_color_multiplier;
                this->rgb.b /= max_color_multiplier;
            };
        virtual ~perlin_noise_2d_mapper() { };

        /* Shading function. Takes the incident ray, the light 
           ray and the normal of the intersect object */
        void shade(const ray_trace_engine &r, ray &i, const line &n, const hit_t h, ext_colour_t *const c) const;

    private :
        const perlin_noise_2d   perlin;
        ext_colour_t            rgb;
        const float             p;
        const float             z;
        const unsigned int      w;
        const unsigned int      h;
        const int               o;
};
}; /* namespace raptor_raytracer */
