#ifndef __RESCALED_SCHLICK_SHADER_H__
#define __RESCALED_SCHLICK_SHADER_H__

#include "common.h"
#include "material.h"
#include "line.h"
#include "ray.h"


namespace raptor_raytracer
{
/* Pure virtual class for material data and shading */
class rescaled_schlick_shader : public material
{
    public :
        rescaled_schlick_shader(const ext_colour_t rgb, const fp_t kd=0.0, const fp_t ks=0.0, const fp_t s=0.0, const fp_t t=0.0, const fp_t ri=0.0, const fp_t rl=0.0) :  { };
        ~rescaled_schlick_shader() { };

        /* Shading function. Takes the incident ray, the light 
           ray and the normal of the intersect object */
        void shade(const ray &i, const list<ray> &l, const line &n, ext_colour_t *const c, ray *const rl, ray *const rr) const;

    private :
};
}; /* namespace raptor_raytracer */

#endif /* #ifndef __RESCALED_SCHLICK_SHADER_H__ */
