#ifndef __PERLIN_NOISE_2D_MAPPER_H__
#define __PERLIN_NOISE_2D_MAPPER_H__

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
        perlin_noise_2d_mapper(ext_colour_t &rgb, fp_t p, fp_t z, unsigned int w, unsigned int h, int o, int s) : 
            material(), perlin(s), rgb(rgb), p(p), z(z), w(w), h(h), o(o)
            {
                /* Normalize colour multipliers */
                fp_t max_color_multiplier = this->rgb.r > this->rgb.g ? this->rgb.r : this->rgb.g;
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
        const perlin_noise_2d & perlin;
        ext_colour_t            rgb;
        fp_t                    p;
        fp_t                    z;
        unsigned int            w;
        unsigned int            h;
        int                     o;
};
}; /* namespace raptor_raytracer */

#endif /* #ifndef __PERLIN_NOISE_2D_MAPPER_H__ */
