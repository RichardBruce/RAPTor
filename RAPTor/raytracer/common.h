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

/* Threading headers */
#ifdef THREADED_RAY_TRACE
#include "scalable_allocator.h"
#endif


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

extern const float PI;

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
#define MAX_SECONDARY_RAYS (int)DIFFUSE_REFLECTIONS 
#else
#define MAX_SECONDARY_RAYS 1
#endif /* #ifdef DIFFUSE_REFLECTIONS */

#ifdef SOFT_SHADOW
#define SHADOW_ARRAY_SIZE (int)SOFT_SHADOW 
#else
#define SHADOW_ARRAY_SIZE 1
#endif /* #ifdef SOFT_SHADOW */

/* Define the size of the bih trace stack */
/* A bih may not grow to be bigger than this */
#ifndef MAX_BIH_STACK_HEIGHT
#define MAX_BIH_STACK_HEIGHT 30
#endif

/* Define the maximum size of a BIH node */
#ifndef MAX_BIH_NODE_SIZE
#ifdef SIMD_PACKET_TRACING
#define MAX_BIH_NODE_SIZE 5
#else
#define MAX_BIH_NODE_SIZE 5
#endif /* #ifdef SIMD_PACKET_TRACING */
#endif /* #ifndef MAX_BIH_NODE_SIZE */

/* Define the size of the bih trace stack */
/* A bih may not grow to be bigger than this */
#ifndef MAX_BVH_STACK_HEIGHT
#define MAX_BVH_STACK_HEIGHT 100
#endif

/* Define the maximum size of a BIH node */
#ifndef MAX_BVH_NODE_SIZE
#ifdef SIMD_PACKET_TRACING
#define MAX_BVH_NODE_SIZE 5
#else
#define MAX_BVH_NODE_SIZE 5
#endif /* #ifdef SIMD_PACKET_TRACING */
#endif /* #ifndef MAX_BVH_NODE_SIZE */


/* Define the size of the kd tree trace stack */
/* A kd tree may not grow to be bigger than this */
#ifndef MAX_KDT_STACK_HEIGHT
#define MAX_KDT_STACK_HEIGHT 20
#endif

#ifdef SIMD_PACKET_TRACING
#ifndef MAX_KDT_NODE_SIZE
#define MAX_KDT_NODE_SIZE 20
#endif /* #ifndef MAX_KDT_NODE_SIZE */
#endif /* #ifdef SIMD_PACKET_TRACING */

#ifndef MIN_APPROX_KDT_BUILDER_NODE_SIZE
#define MIN_APPROX_KDT_BUILDER_NODE_SIZE 36
#endif /* #ifndef MIN_APPROX_KDT_BUILDER_NODE_SIZE */

/* Define the kd tree completion criteria */
#ifndef COST_OF_TRAVERSAL
#define COST_OF_TRAVERSAL 50.0f
#endif

#ifndef COST_OF_INTERSECTION
#define COST_OF_INTERSECTION 1.0f
#endif

#ifndef SECANT_ITERATIONS
#define SECANT_ITERATIONS 50
#endif

#ifndef SECANT_ERROR_LIMIT
#define SECANT_ERROR_LIMIT (1000.0f * DOUBLE_ERR)
#endif

#ifndef NEWTON_RAPHSON_ITERATIONS
#define NEWTON_RAPHSON_ITERATIONS 25
#endif

#ifndef NEWTON_RAPHSON_ERROR_LIMIT
#define NEWTON_RAPHSON_ERROR_LIMIT (250.0f * DOUBLE_ERR)
#endif

#ifndef COS_LUT_SIZE
#define COS_LUT_SIZE (4096*4)
#endif

/* SIMD numbers */
/* The number of elements in the SIMD vetor */
#ifndef SIMD_WIDTH
#define SIMD_WIDTH              4
#endif

/* Log base 2 of the SIMD_WIDTH */
#ifndef LOG2_SIMD_WIDTH
#define LOG2_SIMD_WIDTH         2
#endif

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

#define PACKET_WIDTH            (unsigned)sqrt(MAXIMUM_PACKET_SIZE * SIMD_WIDTH)

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
#ifdef THREADED_RAY_TRACE
typedef std::vector<triangle *, scalable_allocator<triangle *> >    primitive_list;
#else
typedef std::vector<triangle *>                                     primitive_list;
#endif /* #ifdef THREADED_RAY_TRACE */

typedef std::vector<light> light_list;

/* Common numbers */
extern const float FP_DELTA;
extern const float FP_DELTA_SMALL;
extern const float MAX_DIST;
extern const float EPSILON;
extern const float EXP;

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
                this->lut[i] = cos((static_cast<float>(i) * 2.0f * PI) / (static_cast<float>(size)));
            }
        };
        ~static_cos() {  };
        
        /* Function to access the lut */
        float get_cos(float a) const
        {
            unsigned int entry = static_cast<unsigned int>((1.0f / (2.0f * PI)) * size * a) & (size - 1);
            return this->lut[entry];
        };
        
        /* Function to extract sin values from the LUT */
        float get_sin(float a) const
        {
            return this->get_cos(a - (PI / 2.0f));
        };
        
        /* Function to extract sin and cos values from the LUT */
        float get_cos_and_sin(float *cos, float a) const
        {
            unsigned int entry = static_cast<unsigned int>((1.0f / (2.0f * PI)) * size * a) & (size - 1);
            *cos = this->lut[entry];
            entry = (entry - (size / 4)) & (size - 1);
            return this->lut[entry];
        };
    
    private :
        float lut[size];
};

/* LUT for sine and cosine value look up */
extern const static_cos<COS_LUT_SIZE> cos_lut;

/* Display related constants */
extern const int BPP;
extern const int DEPTH;

/* Enumerate the axes */
enum class axis_t : char { x_axis = 1, y_axis = 2, z_axis = 3, not_set = 0};

/* Enumerate the ways a ray may intersect a primitive */
/*
    miss   - primitive isnt hit
    out_in - ray is entering the primitive from the outside world
    in_out - ray is leaving the primitive to the outside world
*/
enum class hit_t : char { miss = 0, out_in = 1, in_out = -1 };

/* A struct to hold the hit information */
struct hit_description
{
    hit_description(const float d = MAX_DIST, const hit_t h = hit_t::miss, const float u = 0.0f, const float v = 0.0f) :
                    u(u), v(v), d(d), h(h) {  };

    float   u;      /* Barycentric u co-ordinate    */
    float   v;      /* Barycentric u co-ordinate    */
    float   d;      /* Distance                     */
    hit_t   h;      /* Hit direction                */
};

/* Enumerate the input file formats */
enum class model_format_t : char { cfg = 0, code = 1, mgf = 2, nff = 3, lwo = 4, obj = 5, off = 6, ply = 7, vrml = 8 };

/* Enumerate the output file formats */
enum class image_format_t : char { tga = 0, jpg = 1, png = 2 };

/* Function to generate a random number between -1 and 1 */
float gen_random_mersenne_twister();
}; /* namespace raptor_raytracer */

#endif /* #ifndef __COMMON_H__ */
