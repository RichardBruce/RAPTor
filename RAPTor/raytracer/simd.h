#ifndef __SIMD_H__
#define __SIMD_H__

#ifdef SIMD_PACKET_TRACING

#include <xmmintrin.h>
#include <iostream>
#include <cmath>

#include "common.h"

class vfp_t;
extern const vfp_t  vfp_zero;
extern const vfp_t  vfp_one;
extern const vfp_t  vfp_true;

extern const vfp_t  index_to_mask_lut[SIMD_WIDTH];

extern const int    mask_to_index_lut[1 << SIMD_WIDTH];


typedef __m128 simd128_t;


class vfp_t
{
    public :
        /* Constructors */
        inline vfp_t()                                                                                                      { }
        inline vfp_t(const simd128_t &v)                                            : m(v)                                  { }
        inline vfp_t(const vfp_t &rhs)                                              : m(rhs.m)                              { }
        inline vfp_t(const float *a)                                                : m(_mm_set_ps(a[3], a[2], a[1], a[0])) { }
        inline vfp_t(const float a, const float b, const float c, const float d)    : m(_mm_set_ps(d, c, b, a))             { }
        inline vfp_t(const float a, const float b, const float c)                   : m(_mm_set_ps(c, c, b, a))             { }
        inline vfp_t(const float a, const float b)                                  : m(_mm_set_ps(b, b, b, a))             { }
        inline vfp_t(const float a)                                                 : m(_mm_set1_ps(a))                     { }
        
        /* Destructor */
        inline ~vfp_t() {  }

        /* Operators */
        /* Unary operators */
        inline vfp_t& operator=(const vfp_t &rhs) 
        {
            this->m = rhs.m;
            return *this;
        }
        
        inline const vfp_t& operator+=(const vfp_t &rhs)
        {
            this->m = _mm_add_ps(this->m, rhs.m);
            return *this;
        }

        inline const vfp_t& operator-=(const vfp_t &rhs)
        {
            this->m = _mm_sub_ps(this->m, rhs.m);
            return *this;
        }

        inline const vfp_t& operator*=(const vfp_t &rhs)
        {
            this->m = _mm_mul_ps(this->m, rhs.m);
            return *this;
        }

        inline const vfp_t& operator/=(const vfp_t &rhs)
        {
            this->m = _mm_div_ps(this->m, rhs.m);
            return *this;
        }

        inline const vfp_t& operator&=(const vfp_t &rhs)
        {
            this->m = _mm_and_ps(this->m, rhs.m);
            return *this;
        }

        inline const vfp_t& operator|=(const vfp_t &rhs)
        {
            this->m = _mm_or_ps(this->m, rhs.m);
            return *this;
        }

        inline const vfp_t& operator^=(const vfp_t &rhs)
        {
            this->m = _mm_xor_ps(this->m, rhs.m);
            return *this;
        }

        inline vfp_t operator-() const
        {
            return _mm_sub_ps(_mm_setzero_ps(), this->m);
        }

        /* Member access operators */
        inline float& operator[](int i)
        {
            union { simd128_t *v; float *f[4]; } a;
            a.v = &this->m;
            return (*a.f)[i];
        }

        inline operator float*()
        {
            union { simd128_t *v; float *f[4]; } a;
            a.v = &this->m;
            return *a.f;
        }

        inline operator const float*() const
        {
            union a { const simd128_t *v; float *f[4]; a(const simd128_t *v) : v(v) {}; };
            a ab(&this->m);
            return *ab.f;
        }

    private :
        /* Data */
//        union
//        {
            simd128_t m;
//            float     f[4];
//        };

        /* Friendly operators */
        friend const vfp_t operator+            (const vfp_t &lhs, const vfp_t &rhs);
        friend const vfp_t operator-            (const vfp_t &lhs, const vfp_t &rhs);
        friend const vfp_t operator*            (const vfp_t &lhs, const vfp_t &rhs);
        friend const vfp_t operator/            (const vfp_t &lhs, const vfp_t &rhs);
        friend const vfp_t operator&            (const vfp_t &lhs, const vfp_t &rhs);
        friend const vfp_t operator|            (const vfp_t &lhs, const vfp_t &rhs);
        friend const vfp_t operator^            (const vfp_t &lhs, const vfp_t &rhs);


        /* Comparisons */
        friend const vfp_t operator==           (const vfp_t &lhs, const vfp_t &rhs);
        friend const vfp_t operator!=           (const vfp_t &lhs, const vfp_t &rhs);
        friend const vfp_t operator>            (const vfp_t &lhs, const vfp_t &rhs);
        friend const vfp_t operator>=           (const vfp_t &lhs, const vfp_t &rhs);
        friend const vfp_t operator<            (const vfp_t &lhs, const vfp_t &rhs);
        friend const vfp_t operator<=           (const vfp_t &lhs, const vfp_t &rhs);


        /* Friendly functions */
        friend       vfp_t sqrt                 (const vfp_t &rhs);
        friend       vfp_t inverse              (const vfp_t &rhs);
        friend       vfp_t inverse_sqrt         (const vfp_t &rhs);
        friend       vfp_t approx_inverse       (const vfp_t &rhs);
        friend       vfp_t approx_inverse_sqrt  (const vfp_t &rhs);
        friend       vfp_t abs                  (const vfp_t &rhs);
        friend       vfp_t max                  (const vfp_t &lhs, const vfp_t &rhs);
        friend       vfp_t min                  (const vfp_t &lhs, const vfp_t &rhs);
        
        /* Non standard friendly functions */
        friend int   move_mask                  (const vfp_t &rhs);
        friend vfp_t andnot                     (const vfp_t &lhs, const vfp_t &rhs);
        template<unsigned int m0, unsigned int m1, unsigned int m2, unsigned int m3>
        friend vfp_t shuffle                    (const vfp_t &lhs, const vfp_t &rhs);
        friend void  transpose                  (vfp_t &a, vfp_t &b, vfp_t &c, vfp_t &d);

} __attribute__ ((aligned(16)));//ALIGN(16);


/* Friendly operators */
/* Binary operators */
inline const vfp_t operator+(const vfp_t &lhs, const vfp_t &rhs)
{
   return _mm_add_ps(lhs.m, rhs.m);
}

inline const vfp_t operator-(const vfp_t &lhs, const vfp_t &rhs)
{
   return _mm_sub_ps(lhs.m, rhs.m);
}

inline const vfp_t operator*(const vfp_t &lhs, const vfp_t &rhs)
{
   return _mm_mul_ps(lhs.m, rhs.m);
}

inline const vfp_t operator/(const vfp_t &lhs, const vfp_t &rhs)
{
   return _mm_div_ps(lhs.m, rhs.m);
}

inline const vfp_t operator&(const vfp_t &lhs, const vfp_t &rhs)
{
   return _mm_and_ps(lhs.m, rhs.m);
}

inline const vfp_t operator|(const vfp_t &lhs, const vfp_t &rhs)
{
   return _mm_or_ps(lhs.m, rhs.m);
}

inline const vfp_t operator^(const vfp_t &lhs, const vfp_t &rhs)
{
   return _mm_xor_ps(lhs.m, rhs.m);
}

/* Comparisons */
inline const vfp_t operator==(const vfp_t &lhs, const vfp_t &rhs)
{
   return _mm_cmpeq_ps(lhs.m, rhs.m);
}

inline const vfp_t operator!=(const vfp_t &lhs, const vfp_t &rhs)
{
   return _mm_cmpneq_ps(lhs.m, rhs.m);
}

inline const vfp_t operator>(const vfp_t &lhs, const vfp_t &rhs)
{
   return _mm_cmpgt_ps(lhs.m, rhs.m);
}

inline const vfp_t operator>=(const vfp_t &lhs, const vfp_t &rhs)
{
   return _mm_cmpge_ps(lhs.m, rhs.m);
}

inline const vfp_t operator<(const vfp_t &lhs, const vfp_t &rhs)
{
   return _mm_cmplt_ps(lhs.m, rhs.m);
}

inline const vfp_t operator<=(const vfp_t &lhs, const vfp_t &rhs)
{
   return _mm_cmple_ps(lhs.m, rhs.m);
}


/* Friendly functions */
inline vfp_t sqrt(const vfp_t &rhs)
{
   return _mm_sqrt_ps(rhs.m);
}

/* Use approximate inversion and added an interation of newton raphson */
inline vfp_t inverse(const vfp_t &rhs)
{
   simd128_t tmp0 = _mm_rcp_ps(rhs.m);
   simd128_t tmp1 = _mm_sub_ps(_mm_add_ps(tmp0, tmp0), _mm_mul_ps(_mm_mul_ps(rhs.m, tmp0), tmp0));
   return tmp1;
}

/* Use approximate inverse square root and added an interation of newton raphson */
inline vfp_t inverse_sqrt(const vfp_t &rhs)
{
   simd128_t tmp0 = _mm_rsqrt_ps(rhs.m);
   simd128_t tmp1 = _mm_mul_ps(_mm_mul_ps(_mm_set1_ps(0.5f), tmp0), _mm_sub_ps(_mm_set1_ps(3.0f), _mm_mul_ps(_mm_mul_ps(rhs.m, tmp0), tmp0)));
   return tmp1;
}

/* Approximate inversion */
inline vfp_t approx_inverse(const vfp_t &rhs)
{
   return _mm_rcp_ps(rhs.m);
}

/* Approximate square root */
inline vfp_t approx_inverse_sqrt(const vfp_t &rhs)
{
   return _mm_rsqrt_ps(rhs.m);
}

inline vfp_t abs(const vfp_t &rhs)
{
   return _mm_and_ps(_mm_set1_ps(bit_cast<unsigned int, float>(0x7fffffff)), rhs.m);
}

inline vfp_t max(const vfp_t &lhs, const vfp_t &rhs)
{
   return _mm_max_ps(lhs.m, rhs.m);
}

inline vfp_t min(const vfp_t &lhs, const vfp_t &rhs)
{
   return _mm_min_ps(lhs.m, rhs.m);
}


/* Non standard friendly functions */
inline int move_mask(const vfp_t &rhs)
{
   return _mm_movemask_ps(rhs.m);
}

inline vfp_t andnot(const vfp_t &lhs, const vfp_t &rhs)
{
   return _mm_andnot_ps(lhs.m, rhs.m);
}

/* Predicated move */
/* Returns a if pred is set else returns b */
inline vfp_t mov_p(const vfp_t &p, const vfp_t &a, const vfp_t &b)
{
   return (andnot(p, b) | (p & a));
}

/* Shuffle */
template<unsigned int m0, unsigned int m1, unsigned int m2, unsigned int m3>
inline vfp_t shuffle(const vfp_t &lhs, const vfp_t &rhs)
{
    return _mm_shuffle_ps(lhs.m, rhs.m, _MM_SHUFFLE(m0, m1, m2, m3));
}

/* 4x4 matrix transpose */
inline void transpose(vfp_t &a, vfp_t &b, vfp_t &c, vfp_t &d)
{
    simd128_t t0 = _mm_shuffle_ps(a.m, b.m, 0x44);
    simd128_t t2 = _mm_shuffle_ps(a.m, b.m, 0xEE);
    simd128_t t1 = _mm_shuffle_ps(c.m, d.m, 0x44);
    simd128_t t3 = _mm_shuffle_ps(c.m, d.m, 0xEE);
    a = _mm_shuffle_ps(t0, t1, 0x88);
    b = _mm_shuffle_ps(t0, t1, 0xDD);
    c = _mm_shuffle_ps(t2, t3, 0x88);
    d = _mm_shuffle_ps(t2, t3, 0xDD);

    return;
}

#endif /* #ifdef SIMD_PACKET_TRACING */
#endif /* #ifndef __SIMD_H__ */
