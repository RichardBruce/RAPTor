#ifndef __VEC_H__
#define __VEC_H__

/* Standard headers */
#include <math.h>
#include <iostream>

namespace raptor_terrain
{
typedef double Real;
//!	Vector dim 3.
template < typename T > class Vec3
{
public:
	T &					operator[](size_t i) { return m_data[i];}
	const T	&			operator[](size_t i) const { return m_data[i];}
	T &					X();
	T &					Y();
	T &					Z();
	const T	&			X() const;
	const T	&			Y() const;
	const T	&			Z() const;
	void				Normalize();
	T					GetNorm() const;
	void				operator= (const Vec3 & rhs);
	void				operator+=(const Vec3 & rhs);
	void				operator-=(const Vec3 & rhs);
	void				operator-=(T a);
	void				operator+=(T a);
	void				operator/=(T a);
	void				operator*=(T a);
	Vec3				operator^ (const Vec3 & rhs) const;
	T			    	operator* (const Vec3 & rhs) const;
	Vec3				operator+ (const Vec3 & rhs) const;
	Vec3				operator- (const Vec3 & rhs) const;
	Vec3				operator- () const;
	Vec3				operator* (T rhs) const;
	Vec3				operator/ (T rhs) const;
						Vec3();
						Vec3(T a);
						Vec3(T x, T y, T z);
						Vec3(const Vec3 & rhs);
	/*virtual*/			~Vec3(void);

private:
	T					m_data[3];
};
template<typename T>
const bool Colinear(const Vec3<T> & a, const Vec3<T> & b, const Vec3<T> & c);
template<typename T>
const T Volume(const Vec3<T> & a, const Vec3<T> & b, const Vec3<T> & c, const Vec3<T> & d);
}; /* namespace raptor_terrain */
#include "vec.inl"	// template implementation
#endif /* #ifndef __VEC_H__ */
