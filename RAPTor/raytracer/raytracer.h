#ifndef __RAYTRACER_H__
#define __RAYTRACER_H__


#include "point_t.h"
#include "camera.h"
#include "light.h"

#include "scalable_allocator.h"

#ifdef THREADED_RAY_TRACE
#include "task_scheduler_init.h"
#include "parallel_for.h"
#include "blocked_range2d.h"

using namespace tbb;
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

class ray;
class packet_ray;
class frustrum;
class line;
class triangle;
class kdt_node;
class bih_node;

#ifndef SPATIAL_SUBDIVISION_BIH
/* Stack element for tracing through the kd tree */
struct kdt_stack_element
{
#ifdef SIMD_PACKET_TRACING
    vfp_t               vt_max;
    vfp_t               vt_min;
    vfp_t               m;
    point_t             u;
    point_t             l;
    fp_t                t_max;
    fp_t                t_min;
#endif
    const kdt_node     *n;
    kdt_stack_element  *s;
    point_t             p;
    fp_t                d;
};
#else

/* Stack element for tracing through the bih */
struct bih_stack_element
{
#ifdef SIMD_PACKET_TRACING
    vfp_t               vt_max;
    vfp_t               vt_min;
#endif
    point_t             u;
    point_t             l;
    const bih_node     *n;
    fp_t                t_max;
    fp_t                t_min;
};
#endif /* #ifndef SPATIAL_SUBDIVISION_BIH */


/* Class for threaded ray tracing */
class ray_trace_engine 
{
    public :
        typedef light_list::iterator       light_iterator;
        typedef light_list::const_iterator const_light_iterator;

        ray_trace_engine(const light_list &l, camera &c, const kdt_node *const k, const std::vector<bih_node> &b) :    
            kdt_base(k), bih_base(b), c(c), lights(l) 
            {
                this->pending_shadows      = static_cast<ray *>(scalable_malloc(  this->lights.size() * (SHADOW_ARRAY_SIZE * MAXIMUM_PACKET_SIZE * SIMD_WIDTH * sizeof(ray))));
                this->nr_pending_shadows   = static_cast<fp_t *>(scalable_malloc( this->lights.size() * (MAXIMUM_PACKET_SIZE * SIMD_WIDTH * sizeof(fp_t))));
            };

        ray_trace_engine(const ray_trace_engine &r) :    
            kdt_base(r.kdt_base), bih_base(r.bih_base), c(r.c), lights(r.lights) 
            {
                this->pending_shadows       = static_cast<ray *>(scalable_malloc( this->lights.size() * (SHADOW_ARRAY_SIZE * MAXIMUM_PACKET_SIZE * SIMD_WIDTH * sizeof(ray))));
                this->nr_pending_shadows    = static_cast<fp_t *>(scalable_malloc(this->lights.size() * (MAXIMUM_PACKET_SIZE * SIMD_WIDTH * sizeof(fp_t))));
            };

        ~ray_trace_engine() 
        {
            scalable_free(this->pending_shadows);
            scalable_free(this->nr_pending_shadows);
        };

        /* Access functions */
        const light_list & get_scene_lights() const { return this->lights; }
        
#ifdef THREADED_RAY_TRACE
        /* Operators for TBB to ray trace a 2-D blocks */
        void operator() (const blocked_range2d<unsigned> &r) const;
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

//            const ext_colour_t intensity = this->lights[l].get_light_intensity(magnitude(r.get_dst() - this->lights[l].get_centre()));
//            if ((intensity.r > 0.01) || (intensity.g > 0.01) || (intensity.b > 0.01))
//            {
                this->nr_pending_shadows[nr_addr]  = r.find_rays(&this->pending_shadows[addr], this->lights[l], n, h);
//            }
//            else
//            {
//                this->nr_pending_shadows[nr_addr]  = (fp_t)0.0;
//            }
            
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
        
#ifndef SPATIAL_SUBDIVISION_BIH
#ifdef SIMD_PACKET_TRACING
        /* SIMD KD-tree traversal */
        inline void find_kdt_leaf_node(const packet_ray *const r, kdt_stack_element **const out, kdt_stack_element *const entry_point, const vfp_t *const i_rd, const int *const near_offset) const;
        inline bool find_kdt_leaf_node(const frustrum &r, kdt_stack_element *const entry_point, kdt_stack_element **const out, const int *const near_offset, unsigned size) const;

        void        kdt_find_nearest_object(const packet_ray *const r, const triangle **const i_o, packet_hit_description *const h) const;
        vfp_t       kdt_found_nearer_object(const packet_ray *const r, const vfp_t &t) const;

        void        kdt_frustrum_found_nearer_object(const packet_ray *const r, const vfp_t *t, vfp_t *closer, unsigned size) const;
        void        kdt_frustrum_find_nearest_object(const packet_ray *const r, const triangle **const i_o, packet_hit_description *const h, int size) const;

        void        kdt_find_nearest_object(const packet_ray *const r, const triangle **const i_o, packet_hit_description *const h,
                                                kdt_stack_element entry_point, kdt_stack_element *exit_point) const;
        vfp_t       kdt_found_nearer_object(const packet_ray *const r, const vfp_t &t, kdt_stack_element entry_point, kdt_stack_element *exit_point) const;

#endif /* #ifdef SIMD_PACKET_TRACING */
        /* kdt traversal */
        inline void find_kdt_leaf_node(const ray *const r, const kdt_node **const n, kdt_stack_element **const out, const kdt_stack_element *const entry_point) const;
        triangle*   kdt_find_nearest_object(const ray *const r, hit_description *const h) const;
        // cppcheck-suppress unusedPrivateFunction
        bool        kdt_found_nearer_object(const ray *const r, const fp_t t) const;
        
        /* The stack is mutable because it will never be known to a user of this class */
        mutable kdt_stack_element   kdt_stack[MAX_KDT_STACK_HEIGHT];
#else
#ifdef SIMD_PACKET_TRACING
        /* SIMD BIH traversal */
        inline int  find_bih_leaf_node(const frustrum &r, bih_stack_element *const entry_point, bih_stack_element **const out, unsigned size) const;
        inline bool find_bih_leaf_node(const packet_ray &r, bih_stack_element *const entry_point, bih_stack_element **const out, const vfp_t *const i_rd) const;

        void        bih_frustrum_find_nearest_object(const packet_ray *const r, const triangle **const i_o, packet_hit_description *const h, 
                                                        bih_stack_element entry_point, bih_stack_element *exit_point, int size) const;
        void        bih_frustrum_find_nearest_object(const packet_ray *const r, const triangle **const i_o, packet_hit_description *const h, int size) const;
        void        bih_frustrum_found_nearer_object(const packet_ray *const r, const vfp_t *t, vfp_t *closer, unsigned size) const;

        void        bih_find_nearest_object(const packet_ray *const r, const triangle **const i_o, packet_hit_description *const h) const;
        vfp_t       bih_found_nearer_object(const packet_ray *const r, const vfp_t &t) const;

        void        bih_find_nearest_object(const packet_ray *const r, const triangle **const i_o, packet_hit_description *const h,
                                                bih_stack_element entry_point, bih_stack_element *exit_point) const;
        vfp_t       bih_found_nearer_object(const packet_ray *const r, const vfp_t &t, bih_stack_element entry_point, bih_stack_element *exit_point) const;

#endif /* #ifdef SIMD_PACKET_TRACING */

        /* bih traversal */
        inline bool find_bih_leaf_node(const ray &r, bih_stack_element *const entry_point, bih_stack_element **const out, const point_t &i_rd) const;
        triangle*   bih_find_nearest_object(const ray *const r, hit_description *const h) const;
        bool        bih_found_nearer_object(const ray *const r, const fp_t t) const;

        /* The stack is mutable because it will never be known to a user of this class */
        mutable bih_stack_element   bih_stack[MAX_BIH_STACK_HEIGHT];
#endif /* #ifndef SPATIAL_SUBDIVISION_BIH */

        const kdt_node          *const  kdt_base;
        const std::vector<bih_node>     &bih_base;
        camera                          &c;    
        const light_list                &lights;
        
        mutable ray                     *pending_shadows;
        mutable fp_t                    *nr_pending_shadows;
        mutable int                     shader_nr;
};

/* Ray tracer main function */
void ray_tracer(const light_list &lights, const primitive_list &everything, camera &c);
}; /* namespace raptor_raytracer */

#endif /* #ifndef __RAYTRACER_H__ */
