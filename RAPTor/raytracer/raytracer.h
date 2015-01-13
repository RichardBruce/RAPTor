#ifndef __RAYTRACER_H__
#define __RAYTRACER_H__

/* Common headers */
#include "point_t.h"

/* Ray tracer headers */
#include "camera.h"
#include "light.h"

#include "scalable_allocator.h"

#ifdef THREADED_RAY_TRACE
#include "task_scheduler_init.h"
#include "parallel_for.h"
#include "blocked_range2d.h"
#endif /* #ifdef THREADED_RAY_TRACE */

namespace raptor_raytracer
{
/* Global performance counters for the spatial sub division */
#ifdef SPATIAL_SUBDIVISION_STATISTICS
extern unsigned nit;            /* Number of intersection tests */
extern unsigned ritm;           /* Ratio of intersection tests performed to minimum required tests */
extern unsigned ng;             /* Number of generic nodes */
extern unsigned ne;             /* Number of elementary nodes */
extern unsigned nee;            /* Number of empty elementary nodes */
extern unsigned ner;            /* Maximum size of an elementary node */
extern unsigned ave_ob;         /* Average size of an elementary node */
extern unsigned max_depth;      /* Maximum depth of the tree */
#endif

/* Forward declarations */
class ssd;

/* Class for threaded ray tracing */
class ray_trace_engine 
{
    public :
        using light_iterator        = light_list::iterator;
        using const_light_iterator  = light_list::const_iterator;

        ray_trace_engine(const light_list &l, camera &c, const ssd *const sub_division)
        :   _ssd(sub_division), c(c), lights(l) 
            {
                this->pending_shadows      = static_cast<ray *>(scalable_malloc(  this->lights.size() * (SHADOW_ARRAY_SIZE * MAXIMUM_PACKET_SIZE * SIMD_WIDTH * sizeof(ray))));
                this->nr_pending_shadows   = static_cast<fp_t *>(scalable_malloc( this->lights.size() * (MAXIMUM_PACKET_SIZE * SIMD_WIDTH * sizeof(fp_t))));
            }

        ray_trace_engine(const ray_trace_engine &r)
        :   _ssd(r._ssd), c(r.c), lights(r.lights) 
            {
                this->pending_shadows       = static_cast<ray *>(scalable_malloc( this->lights.size() * (SHADOW_ARRAY_SIZE * MAXIMUM_PACKET_SIZE * SIMD_WIDTH * sizeof(ray))));
                this->nr_pending_shadows    = static_cast<fp_t *>(scalable_malloc(this->lights.size() * (MAXIMUM_PACKET_SIZE * SIMD_WIDTH * sizeof(fp_t))));
            }

        ~ray_trace_engine() 
        {
            scalable_free(this->pending_shadows);
            scalable_free(this->nr_pending_shadows);
        }

        /* Access functions */
        const light_list & get_scene_lights() const { return this->lights; }
        
#ifdef THREADED_RAY_TRACE
        /* Operators for TBB to ray trace a 2-D blocks */
        void operator() (const tbb::blocked_range2d<unsigned> &r) const;
#endif /* #ifdef THREADED_RAY_TRACE */
        
        /* Member to find nearest intersector and call the shader */
        void ray_trace(ray &r, ext_colour_t *const c) const;
#ifdef SIMD_PACKET_TRACING
        inline void ray_trace_one_packet(const int x, const int y) const;
               void ray_trace(packet_ray *const r, ext_colour_t *const c, const unsigned int *const ray_to_colour_lut, const int s) const;
#endif /* #ifdef SIMD_PACKET_TRACING */

        /* Secondary ray buffer write access */
        inline void generate_rays_to_light(const ray &r, const line &n, const hit_t h, const unsigned int l) const
        {
            const int addr      = (l * (SHADOW_ARRAY_SIZE * MAXIMUM_PACKET_SIZE * SIMD_WIDTH)) + (this->shader_nr * SHADOW_ARRAY_SIZE);
            const int nr_addr   = (l * (MAXIMUM_PACKET_SIZE * SIMD_WIDTH)) + this->shader_nr;

            this->nr_pending_shadows[nr_addr]  = r.find_rays(&this->pending_shadows[addr], this->lights[l], n, h);
            return;
        }
        
        /* Secondary ray buffer read access */
        inline const ray& get_illumination(const unsigned int l) const
        {
            const int addr  = (l * (SHADOW_ARRAY_SIZE * MAXIMUM_PACKET_SIZE * SIMD_WIDTH)) + (this->shader_nr * SHADOW_ARRAY_SIZE);
            return this->pending_shadows[addr];
        }
        
        /* Member to ray trace a single pixel */
        inline void ray_trace_one_pixel(const int x, const int y) const;

    private :
        /* Prevent copying of this large class */
        ray_trace_engine& operator=(const ray_trace_engine &);

#ifdef SIMD_PACKET_TRACING
        inline void shoot_shadow_packet(packet_ray *const r, ray *const *const sr, vfp_t *const t, unsigned int *r_to_s, int *m, const int s, const int l) const;
#endif
        
        const ssd *const    _ssd;
        camera              &c;    
        const light_list    &lights;
        
        mutable ray         *pending_shadows;
        mutable fp_t        *nr_pending_shadows;
        mutable int         shader_nr;
};

/* Main ray tracer function */
void ray_tracer(const ssd *const sub_division, const light_list &lights, const primitive_list &everything, camera &c);
}; /* namespace raptor_raytracer */

#endif /* #ifndef __RAYTRACER_H__ */
