#ifndef __COMMON_H__
#define __COMMON_H__

// C headers for strlen() and rand()
#include <stdlib.h>
#include <string.h>

// Standard headers
#include <iostream>
#include <fstream>
#include <cassert>
#include <climits>
#include <cmath>
#include <list>
#include <vector>
#include <map>
#include <sstream>

/* Size of a floating point number */
#define SINGLE_PRECISION
#ifdef SINGLE_PRECISION
typedef float fp_t;
#else
typedef double fp_t;
#endif /* #ifdef SINGLE_PRECISION */

/* Threading headers */
#ifdef THREADED_RAY_TRACE
#include "scalable_allocator.h"

using namespace tbb;
#endif


using namespace std;

/* Template for casting bits from one type to another */
template<class T, class S>
inline S bit_cast(const T t_in)
{
    assert(sizeof(T) == sizeof(S));

    /* Create union of the types */
    union
    {
        T t;
        S s;
    } cast_union;
    
    /* Input one type */
    cast_union.t = t_in;
    
    /* And return the other */
    return cast_union.s;
}

extern const fp_t PI;

namespace raptor_raytracer
{
/* Alignment */
#define ALIGN(x) __attribute__ ((aligned (x)))

/* Minimum magnitude of a reflected ray, effectily limits the number
   of reflections to < 22 while 'assert(this->reflectivity <= 0.9);' */
#ifndef MIN_REFLECTIVE_POWER
#define MIN_REFLECTIVE_POWER 0.1
#endif /* #ifndef MIN_REFLECTIVE_POWER */

#ifdef DIFFUSE_REFLECTIONS
#define REFLECTION_ARRAY_SIZE (int)DIFFUSE_REFLECTIONS 
#else
#define REFLECTION_ARRAY_SIZE 1
#endif /* #ifdef DIFFUSE_REFLECTIONS */

#ifdef SOFT_SHADOW
#define SHADOW_ARRAY_SIZE (int)SOFT_SHADOW 
#else
#define SHADOW_ARRAY_SIZE 1
#endif /* #ifdef SOFT_SHADOW */

/* Define the size of the bih trace stack */
/* A bih may not grow to be bigger than this */
#ifndef MAX_BIH_STACK_HEIGHT
#define MAX_BIH_STACK_HEIGHT 100
#endif

/* Define the maximum size of a BIH node */
#ifndef MAX_BIH_NODE_SIZE
#ifdef SIMD_PACKET_TRACING
#define MAX_BIH_NODE_SIZE 5
#else
#define MAX_BIH_NODE_SIZE 5
#endif /* #ifdef SIMD_PACKET_TRACING */
#endif /* #ifndef MAX_BIH_NODE_SIZE */


/* Define the size of the kd tree trace stack */
/* A kd tree may not grow to be bigger than this */
#ifndef MAX_KDT_STACK_HEIGHT
#define MAX_KDT_STACK_HEIGHT 100
#endif

#ifdef SIMD_PACKET_TRACING
#ifndef MIN_KDT_NODE_SIZE
#define MIN_KDT_NODE_SIZE 20
#endif /* #ifndef MIN_KDT_NODE_SIZE */
#endif /* #ifdef SIMD_PACKET_TRACING */

#ifdef SIMD_PACKET_TRACING
#ifndef MIN_APPROX_KDT_BUILDER_NODE_SIZE
#define MIN_APPROX_KDT_BUILDER_NODE_SIZE 36
#endif /* #ifndef MIN_KDT_NODE_SIZE */
#else
#ifndef MIN_APPROX_KDT_BUILDER_NODE_SIZE
#define MIN_APPROX_KDT_BUILDER_NODE_SIZE 256
#endif /* #ifndef MIN_KDT_NODE_SIZE */
#endif /* #ifdef SIMD_PACKET_TRACING */

/* Define the kd tree completion criteria */
#ifndef COST_OF_TRAVERSAL
#define COST_OF_TRAVERSAL 1.0
#endif

#ifndef COST_OF_INTERSECTION
#define COST_OF_INTERSECTION (80.0*COST_OF_TRAVERSAL)
#endif

#ifndef SECANT_ITERATIONS
#define SECANT_ITERATIONS 50
#endif

#ifndef SECANT_ERROR_LIMIT
#define SECANT_ERROR_LIMIT (1000.0*DOUBLE_ERR)
#endif

#ifndef NEWTON_RAPHSON_ITERATIONS
#define NEWTON_RAPHSON_ITERATIONS 25
#endif

#ifndef NEWTON_RAPHSON_ERROR_LIMIT
#define NEWTON_RAPHSON_ERROR_LIMIT (250.0*DOUBLE_ERR)
#endif

#ifndef COS_LUT_SIZE
#define COS_LUT_SIZE (4096*4)
#endif

/* SIMD numbers */
#ifdef SIMD_PACKET_TRACING
/* The maximum allowed number of SIMD vectors in a packet */
/* Must be a power of 4, including 0 */
#ifndef MAXIMUM_PACKET_SIZE
#define MAXIMUM_PACKET_SIZE     16
#endif

/* The minimum allowed number of SIMD vectors in a packet */
/* Must be a power of 4, including 0 */
#ifndef MINIMUM_PACKET_SIZE
#define MINIMUM_PACKET_SIZE     16
#endif

/* The factor to reduce the packet size by everytime it is split */
/* Must be a power of 4, including 0 */
#ifndef SPLIT_PACKET_DIVISOR
#define SPLIT_PACKET_DIVISOR    4
#endif

/* The number of elements in the SIMD vetor */
#ifndef SIMD_WIDTH
#define SIMD_WIDTH              4
#endif

#define PACKET_WIDTH            (unsigned)sqrt(MAXIMUM_PACKET_SIZE * SIMD_WIDTH)

/* Log base 2 of the SIMD_WIDTH */
#ifndef LOG2_SIMD_WIDTH
#define LOG2_SIMD_WIDTH         2
#endif

#else   /* #ifdef SIMD_PACKET_TRACING */

#ifndef MAXIMUM_PACKET_SIZE
#define MAXIMUM_PACKET_SIZE     1
#endif

/* The minimum allowed number of SIMD vectors in a packet */
/* Must be a power of 4, including 0 */
#ifndef MINIMUM_PACKET_SIZE
#define MINIMUM_PACKET_SIZE     1
#endif

/* The factor to reduce the packet size by everytime it is split */
/* Must be a power of 4, including 0 */
#ifndef SPLIT_PACKET_DIVISOR
#define SPLIT_PACKET_DIVISOR    1
#endif

/* The number of elements in the SIMD vetor */
#ifndef SIMD_WIDTH
#define SIMD_WIDTH              1
#endif

#define PACKET_WIDTH            (unsigned)sqrt(MAXIMUM_PACKET_SIZE * SIMD_WIDTH)

#ifndef LOG2_SIMD_WIDTH
#define LOG2_SIMD_WIDTH         0
#endif
#endif  /* #ifdef SIMD_PACKET_TRACING */

/* Primitive list to hold primitives */
class triangle;
class light;
#ifdef SPATIAL_SUBDIVISION_BIH
typedef vector<triangle *>                                  primitive_list;
#else
#ifdef THREADED_RAY_TRACE
typedef vector<triangle *, scalable_allocator<triangle *> > primitive_list;
#else
typedef vector<triangle *>                                  primitive_list;
#endif /* #ifdef THREADED_RAY_TRACE */
#endif /* #ifdef SPATIAL_SUBDIVISION_BIH */

typedef vector<light>                                       light_list;

/* Common numbers */
extern const fp_t       FP_DELTA;
extern const fp_t       FP_DELTA_SMALL;
extern const fp_t       MAX_DIST;
extern const fp_t       EPSILON;
extern const fp_t       EXP;

/* Modulus 3 look up table for values less than 6 */
extern const unsigned   mod_3_lut[6];

/* Class to hold a look up table of cos */
template <const unsigned int size> class static_cos
{
    public :
        /* Constructor to fill the LUT with cos values */
        static_cos()
        {
            assert(size == ((~size & (size-1)) + 1));
            for (unsigned i=0; i<size; i++)
            {
                this->lut[i] = cos(((fp_t)i*2.0*PI)/((fp_t)size));
            }
        };
        ~static_cos() {  };
        
        /* Function to access the lut */
        fp_t get_cos(fp_t a) const
        {
            unsigned entry = (unsigned)((1.0/(2.0*PI)) * size * a) & (size-1);
            return this->lut[entry];
        };
        
        /* Function to extract sin values from the LUT */
        fp_t get_sin(fp_t a) const
        {
            return this->get_cos(a - (PI/2.0));
        };
        
        /* Function to extract sin and cos values from the LUT */
        fp_t get_cos_and_sin(fp_t *cos, fp_t a) const
        {
            unsigned entry = (unsigned)((1.0/(2.0*PI)) * size * a) & (size-1);
            *cos = this->lut[entry];
            entry = (entry - (size/4)) & (size-1);
            return this->lut[entry];
        };
    
    private :
        fp_t lut[size];
};

/* LUT for sine and cosine value look up */
extern const static_cos<COS_LUT_SIZE> cos_lut;

/* Display related constants */
extern const int BPP;
extern const int DEPTH;

/* Enumerate the axes */
enum axis { x_axis = 1, y_axis = 2, z_axis = 3, not_set = 0};

/* Enumerate the ways a ray may intersect a primitive */
/*
    miss   - primitive isnt hit
    out_in - ray is entering the primitive from the outside world
    in_out - ray is leaving the primitive to the outside world
*/
enum hit_t { miss = 0, out_in = 1, in_out = -1 };

/* A struct to hold the hit information */
struct hit_description
{
    hit_description(const fp_t d = MAX_DIST, const hit_t h = miss, const fp_t u = 0.0, const fp_t v = 0.0) :
                    u(u), v(v), d(d), h(h) {  };

    fp_t    u;      /* Barycentric u co-ordinate    */
    fp_t    v;      /* Barycentric u co-ordinate    */
    fp_t    d;      /* Distance                     */
    hit_t   h;      /* Hit direction                */
};

/* Enumerate the input file formats */
enum model_format_t { cfg = 0, code = 1, mgf = 2, nff = 3, lwo = 4, obj = 5, ply = 6, vrml = 7 };

/* Enumerate the output file formats */
enum image_format_t { tga = 0, jpg = 1, png = 2 };

/* Function to generate a random number between -1 and 1 */
fp_t gen_random_mersenne_twister();
}; /* namespace raptor_raytracer */

#endif /* #ifndef __COMMON_H__ */
