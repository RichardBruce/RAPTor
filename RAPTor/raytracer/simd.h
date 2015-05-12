#ifndef __SIMD_H__
#define __SIMD_H__

#include <cmath>
#include <immintrin.h>
#include <iostream>
#include <limits>

#include "common.h"

class vfp_t;
class vint_t;

extern const vfp_t  vfp_zero;
extern const vfp_t  vfp_one;
extern const vfp_t  vfp_true;

extern const vfp_t  index_to_mask_lut[SIMD_WIDTH];

extern const int    mask_to_index_lut[1 << SIMD_WIDTH];


/* Scalar functions */
// inline float sqrt(const float rhs)
// {
//     return _mm_cvtss_f32(_mm_sqrt_ps(_mm_set_ss(rhs.m)));
// }

/* Use approximate inversion and added an interation of newton raphson */
inline float inverse(const float rhs)
{
    const float tmp0 = _mm_cvtss_f32(_mm_rcp_ss(_mm_set_ss(rhs)));
    const float tmp1 = (tmp0 + tmp0) - ((rhs * tmp0) * tmp0);
    return tmp1;
}

/* Use approximate inverse square root and added an interation of newton raphson */
inline float inverse_sqrt(const float rhs)
{
    const float tmp0 = _mm_cvtss_f32(_mm_rsqrt_ss(_mm_set_ss(rhs)));
    const float tmp1 = (0.5f * tmp0) * (3.0f - ((rhs * tmp0) * tmp0));
    return tmp1;
}

/* Approximate inversion */
inline float approx_inverse(const float rhs)
{
   return _mm_cvtss_f32(_mm_rcp_ss(_mm_set_ss(rhs)));
}

/* Approximate square root */
inline float approx_inverse_sqrt(const float rhs)
{
   return _mm_cvtss_f32(_mm_rsqrt_ss(_mm_set_ss(rhs)));
}

class vfp_t
{
    public :
        /* Constructors */
        vfp_t() = default;
        vfp_t(const vfp_t &rhs)                                             : m(rhs.m)                  { }
        vfp_t(const float a, const float b, const float c, const float d)   : m(_mm_set_ps(d, c, b, a)) { }
        vfp_t(const float *a)                                               : m(_mm_load_ps(a))         { }
        vfp_t(const float a, const float b, const float c)                  : vfp_t(a, b, c, c)         { }
        vfp_t(const float a, const float b)                                 : vfp_t(a, b, b, b)         { }
        vfp_t(const float a)                                                : m(_mm_set1_ps(a))         { }
        
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

        /* Lane access */
        /* Note -- the c++ compile will prefer the slower non const members */
        inline float& operator[](int i)
        {
            union { __m128 *v; float *f[4]; } a;
            a.v = &this->m;
            return (*a.f)[i];
        }

        inline float operator[](int i) const
        {
            float f[4];
            store(f);
            return f[i];
        }

        inline operator float*()
        {
            union { __m128 *v; float *f[4]; } a;
            a.v = &this->m;
            return *a.f;
        }

        inline operator const float*() const
        {
            union a { const __m128 *v; float *f[4]; a(const __m128 *v) : v(v) {}; };
            a ab(&this->m);
            return *ab.f;
        }

        inline vfp_t store(float *const to) const
        {
            _mm_store_ps(to, this->m);
            return *this;
        }

        inline vfp_t stream(float *const to) const
        {
            _mm_stream_ps(to, this->m);
            return *this;
        }

        inline float extract(int i) const
        {
            float f[4];
            store(f);
            return f[i];
        }

    private :
        vfp_t(const __m128 &rhs) : m(rhs) { }

        __m128 m;

        /* Friend class */
        friend class vint_t;

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
        friend       float horizontal_max       (const vfp_t &rhs);
        friend       float horizontal_min       (const vfp_t &rhs);
        
        /* Non standard friendly functions */
        friend int   move_mask                  (const vfp_t &rhs);
        friend vfp_t andnot                     (const vfp_t &lhs, const vfp_t &rhs);
        template<unsigned int m0, unsigned int m1, unsigned int m2, unsigned int m3>
        friend vfp_t shuffle                    (const vfp_t &lhs, const vfp_t &rhs);
        friend void  transpose                  (vfp_t &a, vfp_t &b, vfp_t &c, vfp_t &d);
        friend void  merge                      (vfp_t &a, vfp_t &b);
        friend void  merge                      (vfp_t &a, vfp_t &b, vfp_t &c, vfp_t &d);
        friend void  merge                      (vfp_t &a0, vfp_t &a1, vfp_t &a2, vfp_t &a3, vfp_t &b0, vfp_t &b1, vfp_t &b3, vfp_t &b4);

        friend int   min_element                (const float *const d, float *const m, const int n);

} __attribute__ ((aligned(16)));//ALIGN(16);

/* Degugging */
inline std::ostream& operator<<(std::ostream &os, const vfp_t &v)
{
    return os << v[0] << ", " << v[1] << ", " << v[2] << ", " << v[3];
}

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
   __m128 tmp0 = _mm_rcp_ps(rhs.m);
   __m128 tmp1 = _mm_sub_ps(_mm_add_ps(tmp0, tmp0), _mm_mul_ps(_mm_mul_ps(rhs.m, tmp0), tmp0));
   return tmp1;
}

/* Use approximate inverse square root and added an interation of newton raphson */
inline vfp_t inverse_sqrt(const vfp_t &rhs)
{
   __m128 tmp0 = _mm_rsqrt_ps(rhs.m);
   __m128 tmp1 = _mm_mul_ps(_mm_mul_ps(_mm_set1_ps(0.5f), tmp0), _mm_sub_ps(_mm_set1_ps(3.0f), _mm_mul_ps(_mm_mul_ps(rhs.m, tmp0), tmp0)));
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

inline float horizontal_max(const vfp_t &rhs)
{
    __m128 max1 = _mm_shuffle_ps(rhs.m, rhs.m, _MM_SHUFFLE(0,0,3,2));
    __m128 max2 = _mm_max_ps(rhs.m, max1);
    __m128 max3 = _mm_shuffle_ps(max2, max2, _MM_SHUFFLE(0,0,0,1));
    __m128 max4 = _mm_max_ps(max2, max3);
    return _mm_cvtss_f32(max4);
}

inline float horizontal_min(const vfp_t &rhs)
{
    __m128 max1 = _mm_shuffle_ps(rhs.m, rhs.m, _MM_SHUFFLE(0,0,3,2));
    __m128 max2 = _mm_min_ps(rhs.m, max1);
    __m128 max3 = _mm_shuffle_ps(max2, max2, _MM_SHUFFLE(0,0,0,1));
    __m128 max4 = _mm_min_ps(max2, max3);
    return _mm_cvtss_f32(max4);
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
    const __m128 t0 = _mm_shuffle_ps(a.m, b.m, 0x44);
    const __m128 t2 = _mm_shuffle_ps(a.m, b.m, 0xEE);
    const __m128 t1 = _mm_shuffle_ps(c.m, d.m, 0x44);
    const __m128 t3 = _mm_shuffle_ps(c.m, d.m, 0xEE);
    a = _mm_shuffle_ps(t0, t1, 0x88);
    b = _mm_shuffle_ps(t0, t1, 0xDD);
    c = _mm_shuffle_ps(t2, t3, 0x88);
    d = _mm_shuffle_ps(t2, t3, 0xDD);

    return;
}

inline void merge_4x4_internal(__m128 &a, __m128 &b)
{
    /* Level 1 high and low */
    const __m128 l1 = _mm_min_ps(a, b);
    const __m128 h1 = _mm_max_ps(a, b);

    /* Level 1 shuffle */
    const __m128 l1_a = _mm_shuffle_ps(l1, h1, _MM_SHUFFLE(1, 0, 1, 0));
    const __m128 l1_b = _mm_shuffle_ps(l1, h1, _MM_SHUFFLE(3, 2, 3, 2));

    /* Level 2 high and low */
    const __m128 l2 = _mm_min_ps(l1_a, l1_b);
    const __m128 h2 = _mm_max_ps(l1_a, l1_b);

    /* Level 2 shuffle */
    const __m128 l2_a = _mm_shuffle_ps(l2, h2, _MM_SHUFFLE(2, 0, 2, 0));
    const __m128 l2_b = _mm_shuffle_ps(l2, h2, _MM_SHUFFLE(3, 1, 3, 1));

    /* Level 3 high and low */
    const __m128 l3 = _mm_min_ps(l2_a, l2_b);
    const __m128 h3 = _mm_max_ps(l2_a, l2_b);

    /* Level 3 shuffle */
    const __m128 l3_a = _mm_shuffle_ps(l3, h3, _MM_SHUFFLE(2, 0, 2, 0));
    const __m128 l3_b = _mm_shuffle_ps(l3, h3, _MM_SHUFFLE(3, 1, 3, 1));

    a = _mm_shuffle_ps(l3_a, l3_a, _MM_SHUFFLE(3, 1, 2, 0));
    b = _mm_shuffle_ps(l3_b, l3_b, _MM_SHUFFLE(3, 1, 2, 0));
    return;
}

inline void merge_8x8_internal(__m128 &a0, __m128 &a1, __m128 &b0, __m128 &b1)
{
    /* 8x8 merge layer */
    const __m128 min_0 = _mm_min_ps(b0, a0);
    const __m128 min_1 = _mm_min_ps(b1, a1);
    const __m128 max_0 = _mm_max_ps(b0, a0);
    const __m128 max_1 = _mm_max_ps(b1, a1);
    a0 = min_0;
    a1 = min_1;
    b0 = max_0;
    b1 = max_1;

    /* 4x4 merges */
    merge_4x4_internal(a0, a1);
    merge_4x4_internal(b0, b1);
    return;
}

inline void merge(vfp_t &a, vfp_t &b)
{
    /* Reverse b */
    b.m = _mm_shuffle_ps(b.m, b.m, _MM_SHUFFLE(0, 1, 2, 3));

    /* Merge */
    merge_4x4_internal(a.m, b.m);

    return;
}

inline void merge(vfp_t &a0, vfp_t &a1, vfp_t &b0, vfp_t &b1)
{
    /* Reverse b */
    const __m128 rev0 = _mm_shuffle_ps(b0.m, b0.m, _MM_SHUFFLE(0, 1, 2, 3));
    const __m128 rev1 = _mm_shuffle_ps(b1.m, b1.m, _MM_SHUFFLE(0, 1, 2, 3));

    b0.m = rev1;
    b1.m = rev0;

    /* Merge */
    merge_8x8_internal(a0.m, a1.m, b0.m, b1.m);
    return;
}

inline void merge(vfp_t &a0, vfp_t &a1, vfp_t &a2, vfp_t &a3, vfp_t &b0, vfp_t &b1, vfp_t &b2, vfp_t &b3)
{
    /* Reverse b */
    b0.m = _mm_shuffle_ps(b0.m, b0.m, _MM_SHUFFLE(0, 1, 2, 3));
    b1.m = _mm_shuffle_ps(b1.m, b1.m, _MM_SHUFFLE(0, 1, 2, 3));
    b2.m = _mm_shuffle_ps(b2.m, b2.m, _MM_SHUFFLE(0, 1, 2, 3));
    b3.m = _mm_shuffle_ps(b3.m, b3.m, _MM_SHUFFLE(0, 1, 2, 3));

    /* 16x16 merge layer */
    const __m128 min_0 = _mm_min_ps(b3.m, a0.m);
    const __m128 min_1 = _mm_min_ps(b2.m, a1.m);
    const __m128 min_2 = _mm_min_ps(b1.m, a2.m);
    const __m128 min_3 = _mm_min_ps(b0.m, a3.m);
    const __m128 max_0 = _mm_max_ps(b3.m, a0.m);
    const __m128 max_1 = _mm_max_ps(b2.m, a1.m);
    const __m128 max_2 = _mm_max_ps(b1.m, a2.m);
    const __m128 max_3 = _mm_max_ps(b0.m, a3.m);
    a0.m = min_3;
    a1.m = min_2;
    a2.m = min_1;
    a3.m = min_0;
    b0.m = max_3;
    b1.m = max_2;
    b2.m = max_1;
    b3.m = max_0;

    /* Merge */
    merge_8x8_internal(a0.m, a1.m, a2.m, a3.m);
    merge_8x8_internal(b0.m, b1.m, b2.m, b3.m);
    return;
}

class vint_t
{
    public :
        /* Constructors */
        vint_t() = default;
        vint_t(const vint_t &rhs)                                   : m(rhs.m)                                                  { }
        vint_t(const vfp_t &rhs)                                    : m(_mm_cvttps_epi32(rhs.m))                                { }
        vint_t(const int a, const int b, const int c, const int d)  : m(_mm_set_epi32 (d, c, b, a))                             { }
        vint_t(const int *a)                                        : m(_mm_load_si128(reinterpret_cast<const __m128i *>(a)))   { }
        vint_t(const int a, const int b, const int c)               : vint_t(a, b, c, c)                                        { }
        vint_t(const int a, const int b)                            : vint_t(a, b, b, b)                                        { }
        vint_t(const int a)                                         : m(_mm_set1_epi32(a))                                      { }
        
        /* Destructor */
        inline ~vint_t() {  }

        /* Operators */
        /* Unary operators */
        inline vint_t& operator=(const vint_t &rhs) 
        {
            this->m = rhs.m;
            return *this;
        }
        
        inline const vint_t& operator+=(const vint_t &rhs)
        {
            this->m = _mm_add_epi32(this->m, rhs.m);
            return *this;
        }

        inline const vint_t& operator-=(const vint_t &rhs)
        {
            this->m = _mm_sub_epi32(this->m, rhs.m);
            return *this;
        }
        
        inline const vint_t& operator<<=(const vint_t &rhs)
        {
            this->m = _mm_sll_epi32(this->m, rhs.m);
            return *this;
        }

        inline const vint_t& operator>>=(const vint_t &rhs)
        {
            this->m = _mm_sra_epi32(this->m, rhs.m);
            return *this;
        }
        
        inline const vint_t& operator<<=(const int rhs)
        {
            this->m = _mm_slli_epi32(this->m, rhs);
            return *this;
        }

        inline const vint_t& operator>>=(const int rhs)
        {
            this->m = _mm_srai_epi32(this->m, rhs);
            return *this;
        }

        inline const vint_t& operator&=(const vint_t &rhs)
        {
            this->m = _mm_and_si128(this->m, rhs.m);
            return *this;
        }

        inline const vint_t& operator|=(const vint_t &rhs)
        {
            this->m = _mm_or_si128(this->m, rhs.m);
            return *this;
        }

        inline const vint_t& operator^=(const vint_t &rhs)
        {
            this->m = _mm_xor_si128(this->m, rhs.m);
            return *this;
        }

        inline vint_t operator-() const
        {
            return _mm_sub_epi32(_mm_setzero_si128(), this->m);
        }

        /* Lane access */
        /* Note -- the c++ compile will prefer the slower non const members */
        inline int& operator[](int i)
        {
            union { __m128i *v; int *f[4]; } a;
            a.v = &this->m;
            return (*a.f)[i];
        }

        inline int operator[](int i) const
        {
            int f[4];
            store(f);
            return f[i];
        }

        inline vint_t store(int *const to) const
        {
            _mm_store_si128(reinterpret_cast<__m128i *const>(to), this->m);
            return *this;
        }

        inline int extract(int i) const
        {
            int f[4];
            store(f);
            return f[i];
        }

    private :
        vint_t(const __m128i &rhs) : m(rhs) { }

        __m128i m;

        /* Friendly operators */
        friend const vint_t operator+   (const vint_t &lhs, const vint_t &rhs);
        friend const vint_t operator-   (const vint_t &lhs, const vint_t &rhs);
        friend const vint_t operator<<  (const vint_t &lhs, const vint_t &rhs);
        friend const vint_t operator>>  (const vint_t &lhs, const vint_t &rhs);
        friend const vint_t operator<<  (const vint_t &lhs, const int rhs);
        friend const vint_t operator>>  (const vint_t &lhs, const int rhs);
        friend const vint_t operator&   (const vint_t &lhs, const vint_t &rhs);
        friend const vint_t operator|   (const vint_t &lhs, const vint_t &rhs);
        friend const vint_t operator^   (const vint_t &lhs, const vint_t &rhs);


        /* Comparisons */
        friend const vint_t operator==  (const vint_t &lhs, const vint_t &rhs);
        friend const vint_t operator!=  (const vint_t &lhs, const vint_t &rhs);
        friend const vint_t operator>   (const vint_t &lhs, const vint_t &rhs);
        friend const vint_t operator>=  (const vint_t &lhs, const vint_t &rhs);
        friend const vint_t operator<   (const vint_t &lhs, const vint_t &rhs);
        friend const vint_t operator<=  (const vint_t &lhs, const vint_t &rhs);


        /* Friendly functions */
        friend       vint_t max         (const vint_t &lhs, const vint_t &rhs);
        friend       vint_t min         (const vint_t &lhs, const vint_t &rhs);
        
        /* Non standard friendly functions */
        friend int   move_mask          (const vint_t &rhs);
        friend vint_t andnot            (const vint_t &lhs, const vint_t &rhs);
        friend vint_t mov_p             (const vint_t &p, const vint_t &a, const vint_t &b);
        template<unsigned int m0, unsigned int m1, unsigned int m2, unsigned int m3>
        friend vint_t shuffle           (const vint_t &lhs);
        friend void  transpose          (vint_t &a, vint_t &b, vint_t &c, vint_t &d);

        friend int   min_element        (const float *const d, float *const m, const int n);

} __attribute__ ((aligned(16)));//ALIGN(16);

/* Degugging */
inline std::ostream& operator<<(std::ostream &os, const vint_t &v)
{
    return os << v[0] << ", " << v[1] << ", " << v[2] << ", " << v[3];
}

/* Friendly operators */
/* Binary operators */
inline const vint_t operator+(const vint_t &lhs, const vint_t &rhs)
{
   return _mm_add_epi32(lhs.m, rhs.m);
}

inline const vint_t operator-(const vint_t &lhs, const vint_t &rhs)
{
   return _mm_sub_epi32(lhs.m, rhs.m);
}       
        
inline const vint_t operator<<(const vint_t &lhs, const vint_t &rhs)
{
    return _mm_sll_epi32(lhs.m, rhs.m);
}

inline const vint_t operator>>(const vint_t &lhs, const vint_t &rhs)
{
    return _mm_sra_epi32(lhs.m, rhs.m);
}
        
inline const vint_t operator<<(const vint_t &lhs, const int rhs)
{
    return _mm_slli_epi32(lhs.m, rhs);
}

inline const vint_t operator>>(const vint_t &lhs, const int rhs)
{
    return _mm_srai_epi32(lhs.m, rhs);
}

inline const vint_t operator&(const vint_t &lhs, const vint_t &rhs)
{
   return _mm_and_si128(lhs.m, rhs.m);
}

inline const vint_t operator|(const vint_t &lhs, const vint_t &rhs)
{
   return _mm_or_si128(lhs.m, rhs.m);
}

inline const vint_t operator^(const vint_t &lhs, const vint_t &rhs)
{
   return _mm_xor_si128(lhs.m, rhs.m);
}

/* Comparisons */
inline const vint_t operator==(const vint_t &lhs, const vint_t &rhs)
{
   return _mm_cmpeq_epi32(lhs.m, rhs.m);
}

inline const vint_t operator>(const vint_t &lhs, const vint_t &rhs)
{
   return _mm_cmpgt_epi32(lhs.m, rhs.m);
}

inline const vint_t operator<(const vint_t &lhs, const vint_t &rhs)
{
   return _mm_cmplt_epi32(lhs.m, rhs.m);
}


/* Friendly functions */

/* Non standard friendly functions */
inline int move_mask(const vint_t &rhs)
{
   return _mm_movemask_epi8(rhs.m);
}

inline vint_t andnot(const vint_t &lhs, const vint_t &rhs)
{
   return _mm_andnot_si128(lhs.m, rhs.m);
}

/* Predicated move */
/* Returns a if pred is set else returns b */
inline vint_t mov_p(const vint_t &p, const vint_t &a, const vint_t &b)
{
    // return _mm_blendv_epi8(b.m, a.m, p.m);
    return (andnot(p, b) | (p & a));
}

/* Shuffle */
template<unsigned int m0, unsigned int m1, unsigned int m2, unsigned int m3>
inline vint_t shuffle(const vint_t &lhs)
{
    return _mm_shuffle_epi32(lhs.m, _MM_SHUFFLE(m0, m1, m2, m3));
}

/* 4x4 matrix transpose */
inline void transpose(vint_t &a, vint_t &b, vint_t &c, vint_t &d)
{
    __m128i t0 = _mm_unpacklo_epi32(a.m, b.m);
    __m128i t1 = _mm_unpacklo_epi32(c.m, d.m);
    __m128i t2 = _mm_unpackhi_epi32(a.m, b.m);
    __m128i t3 = _mm_unpackhi_epi32(c.m, d.m);

    a = _mm_unpacklo_epi64(t0, t1);
    b = _mm_unpackhi_epi64(t0, t1);
    c = _mm_unpacklo_epi64(t2, t3);
    d = _mm_unpackhi_epi64(t2, t3);

    return;
}

/* 3D Morton code */
inline void split_by_three(int &a)
{
    a |= (a << 16);
    a &= 0x030000FF;

    a |= (a <<  8);
    a &= 0x0300F00F;

    a |= (a <<  4);
    a &= 0x030C30C3;

    a |= (a <<  2);
    a &= 0x09249249;
}

inline int morton_code(const float x, const float y, const float z, const float x_mul, const float y_mul, const float z_mul)
{
    /* Bin to nearest grid cell */
    int x_int = x * x_mul;
    int y_int = y * y_mul;
    int z_int = z * z_mul;

    /* Split by three and or together */
    split_by_three(x_int);
    split_by_three(y_int);
    split_by_three(z_int);
    return x_int | (y_int << 1) | (z_int << 2);
}

inline void split_by_three(vint_t &a)
{
    a |= (a << 16);
    a &= 0x030000FF;

    a |= (a <<  8);
    a &= 0x0300F00F;

    a |= (a <<  4);
    a &= 0x030C30C3;

    a |= (a <<  2);
    a &= 0x09249249;
}

inline vint_t morton_code(const vfp_t &x, const vfp_t &y, const vfp_t &z, const vfp_t &x_mul, const vfp_t &y_mul, const vfp_t &z_mul)
{
    /* Bin to nearest grid cell */
    vint_t x_int(x * x_mul);
    vint_t y_int(y * y_mul);
    vint_t z_int(z * z_mul);

    /* Split by three and or together */
    split_by_three(x_int);
    split_by_three(y_int);
    split_by_three(z_int);
    return x_int | (y_int << 1) | (z_int << 2);
}

/* Min index */
inline int min_element(const float *const d, float *const m, const int n)
{
    /* Min within vectors */
    vint_t min_i(-1);
    vfp_t min_v(std::numeric_limits<float>::infinity());
    for (int i = 0; i < n; i += SIMD_WIDTH)
    {
        const vfp_t data(&d[i]);
        const vfp_t pred(data < min_v);

        min_i = mov_p(_mm_castps_si128(pred.m), vint_t(i), min_i);
        min_v = min(data, min_v);
    }

    /* Horizontal min */
    *m = horizontal_min(min_v);
    for (int i = 0; i < SIMD_WIDTH - 1; ++i)
    {
        if ((*m) == min_v[i])
        {
            return i + min_i[i];
        }
    }

    return min_i[SIMD_WIDTH - 1] + SIMD_WIDTH - 1;
}
#endif /* #ifndef __SIMD_H__ */
