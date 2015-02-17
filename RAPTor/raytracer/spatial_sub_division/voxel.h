#ifndef __VOXEL_H__
#define __VOXEL_H__

#ifdef THREADED_RAY_TRACE
#include "parallel_reduce.h"
//#include "primitive_count.h"
#endif

/* Common headers */
#include "common.h"

/* Ray tracer headers */
#include "simd.h"
#include "sort.h"
#include "simd.h"
#include "triangle.h"
#include "kdt_node.h"


namespace raptor_raytracer
{
/* Forward declarations */
class kdt_node;

class voxel
{
    public :
        /* Constructors */
        voxel(primitive_list *p, const point_t &t, const point_t &b, axis_t n) : p(p), low_points(nullptr), high_points(nullptr), t(t), b(b), n(n) { };
        voxel(const voxel &v) : p(v.p), low_points(nullptr), high_points(nullptr), t(v.t), b(v.b), n(v.n) {  };

        /* Destructor */
        /* Dont delete this->p, is it used by the KD-tree */
        ~voxel()
        {
            if (low_points != nullptr)
            {
                delete [] low_points;
                delete [] high_points;
            }
        };
        
        /* Access functions */
        int             size()  const   { return this->p->size();   }
        
        /* Functions for creating the kd-tree */
        voxel divide(kdt_node *const k);
        
    private :
        voxel operator=(const voxel &v);

        /* SAH evaluation functions */
        // float           split_all_axis(float *s, axis_t * normal);
        // float           approximate_split_all_axis(float *s, axis_t * normal) const;
        float           approximate_split_one_axis(float *const s, const axis_t normal) const;

        /* SIMD only SAH evaluation function */
        float           brute_force_split_all_axis(float *s, axis_t * normal) const;
        inline  void    count_primitives(vfp_t *const l, vfp_t *const r, const vfp_t *const s, const axis_t n) const;
        inline  vfp_t   calculate_sah_cost(const vfp_t &l, const vfp_t &r, const vfp_t &s, const axis_t normal) const;

        /* SAH cost equation */
        // inline float    calculate_sah_cost(const float l, const float r, const float s, const axis_t normal) const;

        /* Primitive counting functions */
    	inline  float   count_primitives(float *const r, const float s, const axis_t normal) const;
    	inline  void    count_primitives(float *const l, float *const r, const float *const s, const int len, const axis_t n) const;


        primitive_list  *p;
        float           *low_points;
        float           *high_points;
        point_t          t;
        point_t          b;
        axis_t           n;
};
}; /* namespace raptor_raytracer */

#endif /* #ifndef __VOXEL_H__ */
