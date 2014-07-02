#ifndef __VOXEL_H__
#define __VOXEL_H__

#ifdef THREADED_RAY_TRACE
#include "parallel_reduce.h"
//#include "primitive_count.h"
#endif

#include "common.h"
#include "sort.h"
#include "triangle.h"
#include "kdt_node.h"

class kdt_node;

class voxel
{
    public :
        /* Constructors */
        voxel(primitive_list *p, point_t t, point_t b, axis n) : p(p), low_points(NULL), high_points(NULL), t(t), b(b), n(n) { };
        voxel(const voxel &v) : p(v.p), low_points(NULL), high_points(NULL), t(v.t), b(v.b), n(v.n) {  };

        /* Destructor */
        /* Dont delete this->p, is it used by the KD-tree */
        ~voxel()
        {
            if (low_points != NULL)
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
        // fp_t            split_all_axis(fp_t *s, axis * normal);
        // fp_t            approximate_split_all_axis(fp_t *s, axis * normal) const;
        fp_t            approximate_split_one_axis(fp_t *const s, const axis normal) const;

        /* SIMD only SAH evaluation function */
#ifdef SIMD_PACKET_TRACING
        fp_t            brute_force_split_all_axis(fp_t *s, axis * normal) const;
        inline  void    count_primitives(vfp_t *const l, vfp_t *const r, const vfp_t *const s, const axis n) const;
        inline  vfp_t   calculate_sah_cost(const vfp_t &l, const vfp_t &r, const vfp_t &s, const axis normal) const;
#endif /* #ifdef SIMD_PACKET_TRACING */

        /* SAH cost equation */
        // inline fp_t     calculate_sah_cost(const fp_t l, const fp_t r, const fp_t s, const axis normal) const;

        /* Primitive counting functions */
    	inline  fp_t    count_primitives(fp_t *const r, const fp_t s, const axis normal) const;
    	inline  void    count_primitives(fp_t *const l, fp_t *const r, const fp_t *const s, const int len, const axis n) const;


        primitive_list  *p;
        fp_t            *low_points;
        fp_t            *high_points;
        point_t          t;
        point_t          b;
        axis             n;
};

#endif /* #ifndef __VOXEL_H__ */
