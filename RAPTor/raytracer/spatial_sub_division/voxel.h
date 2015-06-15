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

float approximate_sah_minima(const float cl0, const float cl1, const float cr0, const float cr1, const float x0, const float d, const float xw, const float yw, const float zw);
void fix_adaptive_samples(float *const nr_samples, float *const widths, float *const samples, const float *const cl, const float *const cr, const float d0, const float dw, const float prims);

struct voxel_aab_data
{
    point_t     low;
    point_t     high;
    int         prim;
} ALIGN(32);

class voxel
{
    public :
        /* Constructors */
        voxel(std::vector<voxel_aab_data> *const ping, std::vector<voxel_aab_data> *const pong, const int ping_idx, const int pong_idx, const int nr_prims, const point_t &t, const point_t &b, axis_t n) :
            _ping(ping), _pong(pong), _t(t), _b(b), _sa_inv(0.0f), _ping_idx(ping_idx), _pong_idx(pong_idx), _nr_prims(nr_prims), _n(n)
        {
            const point_t scene_width(t - b);
            _sa_inv = 1.0f / ((scene_width.x * scene_width.y) + (scene_width.x * scene_width.z) + (scene_width.y * scene_width.z));
        };

        voxel(std::vector<voxel_aab_data> *const ping, std::vector<voxel_aab_data> *const pong, const float sa_inv, const int ping_idx, const int pong_idx, const int nr_prims, const point_t &t, const point_t &b, axis_t n) :
            _ping(ping), _pong(pong), _t(t), _b(b), _sa_inv(sa_inv), _ping_idx(ping_idx), _pong_idx(pong_idx), _nr_prims(nr_prims), _n(n)
        {  };

        voxel(const voxel &v) :
            _ping(v._ping), _pong(v._pong), _t(v._t), _b(v._b), _sa_inv(v._sa_inv), _ping_idx(v._ping_idx), _pong_idx(v._pong_idx), _nr_prims(v._nr_prims), _n(v._n) {  };

        /* Allow default DTOR */
        
        /* Access functions */
        int size() const { return _nr_prims; }
        
        /* Functions for creating the kd-tree */
        voxel divide(kdt_node *const k, kdt_node *const children, const int depth);
        
    private :
        voxel operator=(const voxel &v);

        /* SAH evaluation functions */
        float           approximate_split_one_axis(float *const s, const axis_t normal) const;

        /* SIMD only SAH evaluation function */
        float           brute_force_split_all_axis(float *s, axis_t * normal) const;
        inline  void    count_primitives(vfp_t *const l, vfp_t *const r, const vfp_t *const s, const axis_t n) const;
        inline  vfp_t   calculate_sah_cost(const vfp_t &l, const vfp_t &r, const vfp_t &s, const axis_t normal) const;

        /* Primitive counting functions */
    	inline  void    count_primitives(float *const l, float *const r, const float *const s, const int len, const axis_t n) const;


        std::vector<voxel_aab_data> *   _ping;
        std::vector<voxel_aab_data> *   _pong;
        point_t                         _t;
        point_t                         _b;
        float                           _sa_inv;
        int                             _ping_idx;
        int                             _pong_idx;
        int                             _nr_prims;
        axis_t                          _n;
};
}; /* namespace raptor_raytracer */

#endif /* #ifndef __VOXEL_H__ */
