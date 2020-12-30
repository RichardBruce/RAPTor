#pragma once

/* Boost headers */
#include "boost/serialization/access.hpp"
#include "boost/archive/xml_oarchive.hpp"

/* Common headers */
#include "simd.h"


/* Forward declarations */
template<class T> class point_ti;
template<class T> class point_t;

template<class T>
const point_t<T> operator-(const point_t<T> &lhs, const point_t<T> &rhs);

template<class T>
T magnitude(const point_t<T> &a);

/* Class to hold a 3-D co-ordinate */
template<class T = float>
class point_t
{
    public :
        T x;
        T y;
        T z;

        point_t(const T x = 0.0f, const T y = 0.0f, const T z = 0.0f) : x(x), y(y), z(z) {  };

        int min_axis() const
        {
            return x < y ? (x < z ? 0 : 2) : (y < z ? 1 : 2);
        }

        int max_axis() const
        {
            return x < y ? (y < z ? 2 : 1) : (x < z ? 2 : 0);
        }

        T max() const 
        {
            return std::max(x, std::max(y, z));
        }

        T min() const 
        {
            return std::min(x, std::min(y, z));
        }

        bool close(const point_t &rhs, const T tolerance) const
        {
            return fabs(magnitude(*this - rhs)) < tolerance;
        }

        /* Element access */
              T &     operator[](const int i)        {       T* a = &this->x; return a[i];   }
        const T &     operator[](const int i) const  { const T* a = &this->x; return a[i];   }

        /* Unary operators */
        const point_t<T>    operator-() const { return point_t(-this->x, -this->y, -this->z); }

        template<class S>
        explicit operator point_ti<S>() const { return { static_cast<S>(x), static_cast<S>(y), static_cast<S>(z) }; }
        template<class S>
        explicit operator point_t<S>() const { return { static_cast<S>(x), static_cast<S>(y), static_cast<S>(z) }; }

        /* With Point_t */
        const bool       operator==(const point_t &rhs)  const   { return ((this->x == rhs.x) && (this->y == rhs.y) && (this->z == rhs.z));  }
        const bool       operator!=(const point_t &rhs)  const   { return ((this->x != rhs.x) || (this->y != rhs.y) || (this->z != rhs.z));  }
        const bool       operator<(const point_t &rhs)   const   { return ((this->x <  rhs.x) && (this->y <  rhs.y) && (this->z <  rhs.z));  }
        const bool       operator>(const point_t &rhs)   const   { return ((this->x >  rhs.x) && (this->y >  rhs.y) && (this->z >  rhs.z));  }

        const point_t<T>&   operator+=(const point_t &rhs)  { this->x += rhs.x; this->y += rhs.y; this->z += rhs.z; return *this;   }
        const point_t<T>&   operator-=(const point_t &rhs)  { this->x -= rhs.x; this->y -= rhs.y; this->z -= rhs.z; return *this;   }
        const point_t<T>&   operator*=(const point_t &rhs)  { this->x *= rhs.x; this->y *= rhs.y; this->z *= rhs.z; return *this;   }
        const point_t<T>&   operator/=(const point_t &rhs)  { this->x /= rhs.x; this->y /= rhs.y; this->z /= rhs.z; return *this;   }

        /* With scalar */
        const bool       operator==(const T rhs)             { return ((this->x == rhs) && (this->y == rhs) && (this->z == rhs));        }
        const bool       operator!=(const T rhs)             { return ((this->x != rhs) || (this->y != rhs) || (this->z != rhs));        }

        const point_t<T>&   operator+=(const T rhs)             { this->x += rhs;   this->y += rhs;   this->z += rhs;   return *this;       }
        const point_t<T>&   operator-=(const T rhs)             { this->x -= rhs;   this->y -= rhs;   this->z -= rhs;   return *this;       }
        const point_t<T>&   operator*=(const T rhs)             { this->x *= rhs;   this->y *= rhs;   this->z *= rhs;   return *this;       }
        const point_t<T>&   operator/=(const T rhs)             { this->x /= rhs;   this->y /= rhs;   this->z /= rhs;   return *this;       }

    private :
        friend class boost::serialization::access;

        template<class Archive>
        void serialize(Archive & ar, const unsigned int version)
        {
            ar & BOOST_SERIALIZATION_NVP(x);
            ar & BOOST_SERIALIZATION_NVP(y);
            ar & BOOST_SERIALIZATION_NVP(z);
        }
};

/* Degugging */
template<class T>
inline std::ostream& operator<<(std::ostream &os, const point_t<T> &p)
{
    return os << p.x << ", " << p.y << ", " << p.z;
}

/* Binary operators */
template<class T>
inline const point_t<T> operator+(const point_t<T> &lhs, const T rhs)
{
   return point_t(lhs.x + rhs, lhs.y + rhs, lhs.z + rhs);
}

template<class T>
inline const point_t<T> operator-(const point_t<T> &lhs, const T rhs)
{
   return point_t(lhs.x - rhs, lhs.y - rhs, lhs.z - rhs);
}

template<class T>
inline const point_t<T> operator*(const point_t<T> &lhs, const T rhs)
{
   return point_t(lhs.x * rhs, lhs.y * rhs, lhs.z * rhs);
}

template<class T>
inline const point_t<T> operator/(const point_t<T> &lhs, const T rhs)
{
   return point_t(lhs.x / rhs, lhs.y / rhs, lhs.z / rhs);
}

template<class T>
inline const point_t<T> operator+(const point_t<T> &lhs, const point_t<T> &rhs)
{
   return point_t<T>(lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z);
}

template<class T>
inline const point_t<T> operator-(const point_t<T> &lhs, const point_t<T> &rhs)
{
   return point_t<T>(lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z);
}

template<class T>
inline const point_t<T> operator*(const point_t<T> &lhs, const point_t<T> &rhs)
{
   return point_t<T>(lhs.x * rhs.x, lhs.y * rhs.y, lhs.z * rhs.z);
}

template<class T>
inline const point_t<T> operator/(const point_t<T> &lhs, const point_t<T> &rhs)
{
   return point_t<T>(lhs.x / rhs.x, lhs.y / rhs.y, lhs.z / rhs.z);
}

template<class T>
inline const point_t<T> operator+(const T lhs, const point_t<T> &rhs)
{
   return point_t<T>(lhs + rhs.x, lhs + rhs.y, lhs + rhs.z);
}

template<class T>
inline const point_t<T> operator-(const T lhs, const point_t<T> &rhs)
{
   return point_t<T>(lhs - rhs.x, lhs - rhs.y, lhs - rhs.z);
}

template<class T>
inline const point_t<T> operator*(const T lhs, const point_t<T> &rhs)
{
   return point_t<T>(lhs * rhs.x, lhs * rhs.y, lhs * rhs.z);
}

template<class T>
inline const point_t<T> operator/(const T lhs, const point_t<T> &rhs)
{
   return point_t<T>(lhs / rhs.x, lhs / rhs.y, lhs / rhs.z);
}


/* Struct to hold a 3-D vector */
typedef point_t<float> vector_t;

/* Other useful functions */
template<class T>
inline point_t<T> max(const point_t<T> &a, const point_t<T> &b)
{
    point_t<T> ret;
    ret.x = std::max(a.x, b.x);
    ret.y = std::max(a.y, b.y);
    ret.z = std::max(a.z, b.z);
    return ret;
}


template<class T>
inline point_t<T> min(const point_t<T> &a, const point_t<T> &b)
{
    point_t<T> ret;
    ret.x = std::min(a.x, b.x);
    ret.y = std::min(a.y, b.y);
    ret.z = std::min(a.z, b.z);
    return ret;
}

template<class T>
inline point_t<T> max(const point_t<T> &a, const T b)
{
    point_t<T> ret;
    ret.x = std::max(a.x, b);
    ret.y = std::max(a.y, b);
    ret.z = std::max(a.z, b);

    return ret;
}

template<class T>
inline point_t<T> min(const point_t<T> &a, const T b)
{
    point_t<T> ret;
    ret.x = std::min(a.x, b);
    ret.y = std::min(a.y, b);
    ret.z = std::min(a.z, b);

    return ret;
}


template<class T>
inline point_t<T> max_magn(const point_t<T> &a, const point_t<T> &b)
{
    point_t<T> ret(a);
    if (std::fabs(b.x) > std::fabs(a.x))
    {
        ret.x = b.x;
    }

    if (std::fabs(b.y) > std::fabs(a.y))
    {
        ret.y = b.y;
    }

    if (std::fabs(b.z) > std::fabs(a.z))
    {
        ret.z = b.z;
    }

    return ret;
}


template<class T>
inline point_t<T> min_magn(const point_t<T> &a, const point_t<T> &b)
{
    point_t<T> ret(a);
    if (std::fabs(b.x) < std::fabs(a.x))
    {
        ret.x = b.x;
    }

    if (std::fabs(b.y) < std::fabs(a.y))
    {
        ret.y = b.y;
    }

    if (std::fabs(b.z) < std::fabs(a.z))
    {
        ret.z = b.z;
    }

    return ret;
}

template<class T>
inline point_t<T> max_magn(const point_t<T> &a, const T b)
{
    point_t<T> ret(a);
    const T abs_b = std::fabs(b);
    if (abs_b > std::fabs(a.x))
    {
        ret.x = abs_b;
    }

    if (abs_b > std::fabs(a.y))
    {
        ret.y = abs_b;
    }

    if (abs_b > std::fabs(a.z))
    {
        ret.z = abs_b;
    }

    return ret;
}

template<class T>
inline point_t<T> min_magn(const point_t<T> &a, const T b)
{
    point_t<T> ret(a);
    const T abs_b = std::fabs(b);
    if (abs_b < std::fabs(a.x))
    {
        ret.x = abs_b;
    }

    if (abs_b <std:: fabs(a.y))
    {
        ret.y = abs_b;
    }

    if (abs_b < std::fabs(a.z))
    {
        ret.z = abs_b;
    }

    return ret;
}


template<class T>
inline point_t<T> fabs(const point_t<T> &a)
{
    return point_t<T>(std::fabs(a.x), std::fabs(a.y), std::fabs(a.z));
}

template<class T>
inline point_t<T> floor_to_zero(point_t<T> a, const T f)
{
    if (std::fabs(a.x) < f)
    {
        a.x = 0.0f;
    }

    if (std::fabs(a.y) < f)
    {
        a.y = 0.0f;
    }

    if (std::fabs(a.z) < f)
    {
        a.z = 0.0f;
    }

    return a;
}


/*****************************************************
 Function to get the magnitude of a vector.
 
 a is the vector to find the magnitude of and its 
 magnitude is returned.
*****************************************************/
template<class T>
inline T magnitude(const point_t<T> &a)
{
    return std::sqrt((a.x * a.x) + (a.y * a.y) + (a.z * a.z));
}


/*****************************************************
 Function to write to rotate a point about the origin.
 
 a is the point to be rotated. r is an arbitary vector
 to rotate about. theta is the angle to rotate by.
 The rotated point is returned through a.
*****************************************************/
template<class T>
inline void rotate_about_origin(point_t<T> *const a, const point_t<T> *const r, const T theta)
{
    /* Rotate the origin into the new coordinate system */
    point_t<T> p = *a;
    point_t<T> q(0.0f ,0.0f ,0.0f);
   
    const T costheta = std::cos(theta);
    const T sintheta = std::sin(theta);
    
    /* Rotate about r */
    q.x += (costheta + (1 - costheta) * r->x * r->x) * p.x;
    q.x += ((1 - costheta) * r->x * r->y - r->z * sintheta) * p.y;
    q.x += ((1 - costheta) * r->x * r->z + r->y * sintheta) * p.z;

    q.y += ((1 - costheta) * r->x * r->y + r->z * sintheta) * p.x;
    q.y += (costheta + (1 - costheta) * r->y * r->y) * p.y;
    q.y += ((1 - costheta) * r->y * r->z - r->x * sintheta) * p.z;

    q.z += ((1 - costheta) * r->x * r->z - r->y * sintheta) * p.x;
    q.z += ((1 - costheta) * r->y * r->z + r->x * sintheta) * p.y;
    q.z += (costheta + (1 - costheta) * r->z * r->z) * p.z;
    
    *a = q;
}


/*****************************************************
 Function to write to rotate a point about the x axis
 through the origin.
 
 a is the point to be rotated. theta is the angle to 
 rotate by. The rotated point is returned through a.
*****************************************************/
template<class T>
inline void rotate_about_x_axis(point_t<T> *const a, const T theta)
{
    const point_t<T> p = *a;
    point_t<T> q(p.x, 0.0, 0.0);
   
    const T costheta = cos(theta);
    const T sintheta = sin(theta);
    
    q.y += costheta * p.y;
    q.y -= sintheta * p.z;

    q.z += sintheta * p.y;
    q.z += costheta * p.z;    
    
    *a = q;
}


/*****************************************************
 Function to write to rotate a point about the y axis
 through the origin.
 
 a is the point to be rotated. theta is the angle to 
 rotate by. The rotated point is returned through a.
*****************************************************/
template<class T>
inline void rotate_about_y_axis(point_t<T> *const a, const T theta)
{
    const point_t<T> p = *a;
    point_t<T> q(0.0, p.y, 0.0);
   
    const T costheta = cos(theta);
    const T sintheta = sin(theta);
    
    /* Rotate about y */
    q.x += costheta * p.x;
    q.x += sintheta * p.z;

    q.z -= sintheta * p.x;
    q.z += costheta * p.z;

    *a = q;
}


/*****************************************************
 Function to write to rotate a point about the y axis
 through the origin.
 
 a is the point to be rotated. theta is the angle to 
 rotate by. The rotated point is returned through a.
*****************************************************/
template<class T>
inline void rotate_about_z_axis(point_t<T> *const a, const T theta)
{
    const point_t<T> p = *a;
    point_t<T> q(0.0, 0.0, p.z);
   
    const T costheta = cos(theta);
    const T sintheta = sin(theta);
    
    /* Rotate about z */

    q.x += costheta * p.x;
    q.x -= sintheta * p.y;

    q.y += sintheta * p.x;
    q.y += costheta * p.y;
    
    *a = q;
}


/*****************************************************
 Function to compute the perpendicular of 1 vectors.
 
 computes b = perp(a). There are of course an 
 infinite number of solutions so be careful this is
 what is required.
*****************************************************/
template<class T>
inline point_t<T> perpendicular(const point_t<T> &a)
{
    return point_t(a.z, a.y, -a.x);
}


/*****************************************************
 Function to compute the cross product of 2 vectors.
 
 computes c = a X b.
*****************************************************/
template<class T>
inline void cross_product(const point_t<T> &a, const point_t<T> &b, point_t<T> *const c)
{
    c->x = (a.y * b.z) - (a.z * b.y);
    c->y = (a.z * b.x) - (a.x * b.z);
    c->z = (a.x * b.y) - (a.y * b.x);
}

/*****************************************************
 Function to compute the cross product of 2 vectors.
 
 computes ret = a X b.
*****************************************************/
template<class T>
inline point_t<T> cross_product(const point_t<T> &a, const point_t<T> &b)
{
    return point_t<T>((a.y * b.z) - (a.z * b.y), (a.z * b.x) - (a.x * b.z), (a.x * b.y) - (a.y * b.x));
}

/*****************************************************
 Function to compute the dot product of 2 vectors.
 
 return a . b.
*****************************************************/
template<class T>
inline T dot_product(const point_t<T> &a, const point_t<T> &b)
{
    return ((a.x * b.x) + (a.y * b.y)+ (a.z * b.z));
}

/*****************************************************
 Function to normalise vectors.
 
 leaves a normalised.
*****************************************************/
template<class T>
inline void normalise(point_t<T> *const a)
{
    const T dist_inv = 1.0 / std::sqrt((a->x * a->x) + (a->y * a->y) + (a->z * a->z));
    a->x *= dist_inv;
    a->y *= dist_inv;
    a->z *= dist_inv;
    return;
}

template<class T>
inline point_t<T> normalise(point_t<T> a)
{
    const T dist_inv = 1.0 / std::sqrt((a.x * a.x) + (a.y * a.y) + (a.z * a.z));
    a.x *= dist_inv;
    a.y *= dist_inv;
    a.z *= dist_inv;
    return a;
}

template<>
inline void normalise<float>(point_t<float> *const a)
{
#ifdef EXACT_NORMALISE
    const float dist = std::sqrt((a->x * a->x) + (a->y * a->y) + (a->z * a->z));
    a->x /= dist;
    a->y /= dist;
    a->z /= dist;
#else
    const float dist = inverse_sqrt((a->x * a->x) + (a->y * a->y) + (a->z * a->z));
    a->x *= dist;
    a->y *= dist;
    a->z *= dist;
#endif

    return;
}

template<>
inline point_t<float> normalise<float>(point_t<float> a)
{
#ifdef EXACT_NORMALISE
    const float dist = std::sqrt((a.x * a.x) + (a.y * a.y) + (a.z * a.z));
    a.x /= dist;
    a.y /= dist;
    a.z /= dist;
#else
    const float dist = inverse_sqrt((a.x * a.x) + (a.y * a.y) + (a.z * a.z));
    a.x *= dist;
    a.y *= dist;
    a.z *= dist;
#endif

    return a;
}

template<class T>
inline T tetrahedron_volume(const point_t<T> &a, const point_t<T> &b, const point_t<T> &c, const point_t<T> &d)
{
    return dot_product((a - d), cross_product((b - d), (c - d)));
}

template<class T>
inline const bool co_linear(const point_t<T> &a, const point_t<T> &b, const point_t<T> &c)
{
    return ((c.z - a.z) * (b.y - a.y) - (b.z - a.z) * (c.y - a.y) == 0.0) &&
           ((b.z - a.z) * (c.x - a.x) - (b.x - a.x) * (c.z - a.z) == 0.0) &&
           ((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x) == 0.0);
}

/* Class to hold a 3-D integer co-ordinate */
template<class T = int>
class point_ti
{
    public :
        T x;
        T y;
        T z;
        point_ti(const T x = 0, const T y = 0, const T z = 0) : x(x), y(y), z(z) { };
 
        /* Element access */
              T &            operator[](const T i)                      {       T* a = &this->x; return a[i];                                       }
        const T &            operator[](const T i) const                { const T* a = &this->x; return a[i];                                       }

        /* Unary operators */
        const point_ti   operator-()                            const   { return point_ti<T>(-this->x, -this->y, -this->z);                         }
        template<class S> 
        explicit operator point_t<S>()                          const   { return { static_cast<S>(x), static_cast<S>(y), static_cast<S>(z) };       }

        /* With Point_t */
        const bool           operator==(const point_ti &rhs)    const   { return ((this->x == rhs.x) && (this->y == rhs.y) && (this->z == rhs.z));  }
        const bool           operator!=(const point_ti &rhs)    const   { return ((this->x != rhs.x) || (this->y != rhs.y) || (this->z != rhs.z));  }
        const bool           operator<(const point_ti &rhs)     const   { return ((this->x <  rhs.x) && (this->y <  rhs.y) && (this->z <  rhs.z));  }
        const bool           operator>(const point_ti &rhs)     const   { return ((this->x >  rhs.x) && (this->y >  rhs.y) && (this->z >  rhs.z));  }
        const bool           operator<=(const point_ti &rhs)    const   { return ((this->x <= rhs.x) && (this->y <= rhs.y) && (this->z <= rhs.z));  }
        const bool           operator>=(const point_ti &rhs)    const   { return ((this->x >= rhs.x) && (this->y >= rhs.y) && (this->z >= rhs.z));  }

        const point_ti<T>&   operator+=(const point_ti &rhs)            { this->x += rhs.x; this->y += rhs.y; this->z += rhs.z; return *this;       }
        const point_ti<T>&   operator-=(const point_ti &rhs)            { this->x -= rhs.x; this->y -= rhs.y; this->z -= rhs.z; return *this;       }
        const point_ti<T>&   operator*=(const point_ti &rhs)            { this->x *= rhs.x; this->y *= rhs.y; this->z *= rhs.z; return *this;       }
        const point_ti<T>&   operator/=(const point_ti &rhs)            { this->x /= rhs.x; this->y /= rhs.y; this->z /= rhs.z; return *this;       }

        /* With int */
        const bool           operator==(const T rhs)                    { return ((this->x == rhs) && (this->y == rhs) && (this->z == rhs));        }
        const bool           operator!=(const T rhs)                    { return ((this->x != rhs) || (this->y != rhs) || (this->z != rhs));        }

        const point_ti<T>&   operator+=(const T rhs)                    { this->x += rhs;   this->y += rhs;   this->z += rhs;   return *this;       }
        const point_ti<T>&   operator-=(const T rhs)                    { this->x -= rhs;   this->y -= rhs;   this->z -= rhs;   return *this;       }
        const point_ti<T>&   operator*=(const T rhs)                    { this->x *= rhs;   this->y *= rhs;   this->z *= rhs;   return *this;       }
        const point_ti<T>&   operator/=(const T rhs)                    { this->x /= rhs;   this->y /= rhs;   this->z /= rhs;   return *this;       }

    private :
        friend class boost::serialization::access;

        template<class Archive>
        void serialize(Archive & ar, const unsigned int version)
        {
            ar & BOOST_SERIALIZATION_NVP(x);
            ar & BOOST_SERIALIZATION_NVP(y);
            ar & BOOST_SERIALIZATION_NVP(z);
        }
};

/* Degugging */
template<class T>
inline std::ostream& operator<<(std::ostream &os, const point_ti<T> &p)
{
    return os << p.x << ", " << p.y << ", " << p.z;
}

/* Binary operators */
template<class T>
inline const point_ti<T> operator+(const point_ti<T> &lhs, const int rhs)
{
   return point_ti<T>(lhs.x + rhs, lhs.y + rhs, lhs.z + rhs);
}

template<class T>
inline const point_ti<T> operator-(const point_ti<T> &lhs, const int rhs)
{
   return point_ti<T>(lhs.x - rhs, lhs.y - rhs, lhs.z - rhs);
}

template<class T>
inline const point_ti<T> operator*(const point_ti<T> &lhs, const int rhs)
{
   return point_ti<T>(lhs.x * rhs, lhs.y * rhs, lhs.z * rhs);
}

template<class T>
inline const point_ti<T> operator/(const point_ti<T> &lhs, const int rhs)
{
   return point_ti<T>(lhs.x / rhs, lhs.y / rhs, lhs.z / rhs);
}

template<class T>
inline const point_ti<T> operator+(const point_ti<T> &lhs, const point_ti<T> &rhs)
{
   return point_ti<T>(lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z);
}

template<class T>
inline const point_ti<T> operator-(const point_ti<T> &lhs, const point_ti<T> &rhs)
{
   return point_ti<T>(lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z);
}

template<class T>
inline const point_ti<T> operator*(const point_ti<T> &lhs, const point_ti<T> &rhs)
{
   return point_ti<T>(lhs.x * rhs.x, lhs.y * rhs.y, lhs.z * rhs.z);
}

template<class T>
inline const point_ti<T> operator/(const point_ti<T> &lhs, const point_ti<T> &rhs)
{
   return point_ti<T>(lhs.x / rhs.x, lhs.y / rhs.y, lhs.z / rhs.z);
}

template<class T>
inline const point_ti<T> operator+(const int lhs, const point_ti<T> &rhs)
{
   return point_ti<T>(lhs + rhs.x, lhs + rhs.y, lhs + rhs.z);
}

template<class T>
inline const point_ti<T> operator-(const int lhs, const point_ti<T> &rhs)
{
   return point_ti<T>(lhs - rhs.x, lhs - rhs.y, lhs - rhs.z);
}

template<class T>
inline const point_ti<T> operator*(const int lhs, const point_ti<T> &rhs)
{
   return point_ti<T>(lhs * rhs.x, lhs * rhs.y, lhs * rhs.z);
}

template<class T>
inline const point_ti<T> operator/(const int lhs, const point_ti<T> &rhs)
{
   return point_ti<T>(lhs / rhs.x, lhs / rhs.y, lhs / rhs.z);
}


/* Other useful functions */
template<class T>
inline point_ti<T> max(const point_ti<T>& a, const point_ti<T> &b)
{
    point_ti<T> ret;
    ret.x = std::max(a.x, b.x);
    ret.y = std::max(a.y, b.y);
    ret.z = std::max(a.z, b.z);
    
    return ret;
}

template<class T>
inline point_ti<T> min(const point_ti<T>& a, const point_ti<T> &b)
{
    point_ti<T> ret;
    ret.x = std::min(a.x, b.x);
    ret.y = std::min(a.y, b.y);
    ret.z = std::min(a.z, b.z);
    
    return ret;
}

template<class T>
inline point_ti<T> max(const point_ti<T>& a, const int b)
{
    point_ti<T> ret;
    ret.x = std::max(a.x, b);
    ret.y = std::max(a.y, b);
    ret.z = std::max(a.z, b);
    
    return ret;
}

template<class T>
inline point_ti<T> min(const point_ti<T>& a, const int b)
{
    point_ti<T> ret;
    ret.x = std::min(a.x, b);
    ret.y = std::min(a.y, b);
    ret.z = std::min(a.z, b);
    
    return ret;
}

template<class T>
point_ti<T> normalise(point_ti<T> a, const T scale)
{
    const T dist = scale / std::sqrt((a.x * a.x) + (a.y * a.y) + (a.z * a.z));
    a.x *= dist;
    a.y *= dist;
    a.z *= dist;

    return a;
}

template<class T>
inline point_ti<T> cross_product(const point_ti<T> &a, const point_ti<T> &b)
{
    return point_ti<T>((a.y * b.z) - (a.z * b.y), (a.z * b.x) - (a.x * b.z), (a.x * b.y) - (a.y * b.x));
}

template<class T>
inline T dot_product(const point_ti<T> &a, const point_ti<T> &b)
{
    return ((a.x * b.x) + (a.y * b.y) + (a.z * b.z));
}

template<class T>
inline T magnitude_sq(const point_ti<T> &a)
{
    return (a.x * a.x) + (a.y * a.y) + (a.z * a.z);
}

template<class T>
inline T triangle_area22(const point_ti<T> &a, const point_ti<T> &b, const point_ti<T> &c)
{
    return magnitude_sq(cross_product((b - a), (c - a)));
}

template<class T>
inline T tetrahedron_volume(const point_ti<T> &a, const point_ti<T> &b, const point_ti<T> &c, const point_ti<T> &d)
{
    return dot_product((a - d), cross_product((b - d), (c - d)));
}

template<class T>
inline const bool co_linear(const point_ti<T> &a, const point_ti<T> &b, const point_ti<T> &c)
{
    return ((c.z - a.z) * (b.y - a.y) - (b.z - a.z) * (c.y - a.y) == 0) &&
           ((b.z - a.z) * (c.x - a.x) - (b.x - a.x) * (c.z - a.z) == 0) &&
           ((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x) == 0);
}
