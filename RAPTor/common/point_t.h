#ifndef __POINT_T_H__
#define __POINT_T_H__

/* Boost headers */
#include "boost/serialization/access.hpp"
#include "boost/archive/xml_oarchive.hpp"

/* Common headers */
#include "simd.h"
#include "common.h"


/* Class to hold a 3-D co-ordinate */
class point_t
{
    public :
        float x;
        float y;
        float z;
        point_t(const float x = 0.0, const float y = 0.0, const float z = 0.0) : x(x), y(y), z(z) { };
        
        /* Element access */
        inline       float &     operator[](const int i)                 {       float* a = &this->x; return a[i];                                  }
        inline const float &     operator[](const int i) const           { const float* a = &this->x; return a[i];                                  }

        /* Unary operators */
        inline const point_t    operator-()   const                     { return point_t(-this->x, -this->y, -this->z);                             }

        /* With Point_t */
        inline const bool       operator==(const point_t &rhs)  const   { return ((this->x == rhs.x) && (this->y == rhs.y) && (this->z == rhs.z));  }
        inline const bool       operator!=(const point_t &rhs)  const   { return ((this->x != rhs.x) || (this->y != rhs.y) || (this->z != rhs.z));  }
        inline const bool       operator<(const point_t &rhs)   const   { return ((this->x <  rhs.x) && (this->y <  rhs.y) && (this->z <  rhs.z));  }
        inline const bool       operator>(const point_t &rhs)   const   { return ((this->x >  rhs.x) && (this->y >  rhs.y) && (this->z >  rhs.z));  }

        inline const point_t&   operator+=(const point_t &rhs)          { this->x += rhs.x; this->y += rhs.y; this->z += rhs.z; return *this;       }
        inline const point_t&   operator-=(const point_t &rhs)          { this->x -= rhs.x; this->y -= rhs.y; this->z -= rhs.z; return *this;       }
        inline const point_t&   operator*=(const point_t &rhs)          { this->x *= rhs.x; this->y *= rhs.y; this->z *= rhs.z; return *this;       }
        inline const point_t&   operator/=(const point_t &rhs)          { this->x /= rhs.x; this->y /= rhs.y; this->z /= rhs.z; return *this;       }

        /* With float */
        inline const bool       operator==(const float rhs)              { return ((this->x == rhs) && (this->y == rhs) && (this->z == rhs));        }
        inline const bool       operator!=(const float rhs)              { return ((this->x != rhs) || (this->y != rhs) || (this->z != rhs));        }

        inline const point_t&   operator+=(const float rhs)              { this->x += rhs;   this->y += rhs;   this->z += rhs;   return *this;       }
        inline const point_t&   operator-=(const float rhs)              { this->x -= rhs;   this->y -= rhs;   this->z -= rhs;   return *this;       }
        inline const point_t&   operator*=(const float rhs)              { this->x *= rhs;   this->y *= rhs;   this->z *= rhs;   return *this;       }
        inline const point_t&   operator/=(const float rhs)              { this->x /= rhs;   this->y /= rhs;   this->z /= rhs;   return *this;       }

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
inline std::ostream& operator<<(std::ostream &os, const point_t &p)
{
    return os << p.x << ", " << p.y << ", " << p.z;
}

/* Binary operators */
inline const point_t operator+(const point_t &lhs, const float rhs)
{
   return point_t(lhs.x + rhs, lhs.y + rhs, lhs.z + rhs);
}

inline const point_t operator-(const point_t &lhs, const float rhs)
{
   return point_t(lhs.x - rhs, lhs.y - rhs, lhs.z - rhs);
}

inline const point_t operator*(const point_t &lhs, const float rhs)
{
   return point_t(lhs.x * rhs, lhs.y * rhs, lhs.z * rhs);
}

inline const point_t operator/(const point_t &lhs, const float rhs)
{
   return point_t(lhs.x / rhs, lhs.y / rhs, lhs.z / rhs);
}

inline const point_t operator+(const point_t &lhs, const point_t &rhs)
{
   return point_t(lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z);
}

inline const point_t operator-(const point_t &lhs, const point_t &rhs)
{
   return point_t(lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z);
}

inline const point_t operator*(const point_t &lhs, const point_t &rhs)
{
   return point_t(lhs.x * rhs.x, lhs.y * rhs.y, lhs.z * rhs.z);
}

inline const point_t operator/(const point_t &lhs, const point_t &rhs)
{
   return point_t(lhs.x / rhs.x, lhs.y / rhs.y, lhs.z / rhs.z);
}

inline const point_t operator+(const float lhs, const point_t &rhs)
{
   return point_t(lhs + rhs.x, lhs + rhs.y, lhs + rhs.z);
}

inline const point_t operator-(const float lhs, const point_t &rhs)
{
   return point_t(lhs - rhs.x, lhs - rhs.y, lhs - rhs.z);
}

inline const point_t operator*(const float lhs, const point_t &rhs)
{
   return point_t(lhs * rhs.x, lhs * rhs.y, lhs * rhs.z);
}

inline const point_t operator/(const float lhs, const point_t &rhs)
{
   return point_t(lhs / rhs.x, lhs / rhs.y, lhs / rhs.z);
}


/* Struct to hold a 3-D vector */
typedef point_t vector_t;

/* Other useful functions */
inline point_t max(const point_t& a, const point_t &b)
{
    point_t ret;
    ret.x = std::max(a.x, b.x);
    ret.y = std::max(a.y, b.y);
    ret.z = std::max(a.z, b.z);
    
    return ret;
}


inline point_t min(const point_t& a, const point_t &b)
{
    point_t ret;
    ret.x = std::min(a.x, b.x);
    ret.y = std::min(a.y, b.y);
    ret.z = std::min(a.z, b.z);
    
    return ret;
}


inline point_t max(const point_t& a, const float b)
{
    point_t ret;
    ret.x = std::max(a.x, b);
    ret.y = std::max(a.y, b);
    ret.z = std::max(a.z, b);
    
    return ret;
}


inline point_t min(const point_t& a, const float b)
{
    point_t ret;
    ret.x = std::min(a.x, b);
    ret.y = std::min(a.y, b);
    ret.z = std::min(a.z, b);
    
    return ret;
}


inline point_t max_magn(const point_t& a, const point_t &b)
{
    point_t ret(a);
    if (fabs(b.x) > fabs(a.x))
    {
        ret.x = b.x;
    }
    
    if (fabs(b.y) > fabs(a.y))
    {
        ret.y = b.y;
    }
    
    if (fabs(b.z) > fabs(a.z))
    {
        ret.z = b.z;
    }
    
    return ret;
}


inline point_t min_magn(const point_t& a, const point_t &b)
{
    point_t ret(a);
    if (fabs(b.x) < fabs(a.x))
    {
        ret.x = b.x;
    }
    
    if (fabs(b.y) < fabs(a.y))
    {
        ret.y = b.y;
    }
    
    if (fabs(b.z) < fabs(a.z))
    {
        ret.z = b.z;
    }
    
    return ret;
}


inline point_t max_magn(const point_t& a, const float b)
{
    point_t ret(a);
    const float abs_b = fabs(b);
    if (abs_b > fabs(a.x))
    {
        ret.x = abs_b;
    }
    
    if (abs_b > fabs(a.y))
    {
        ret.y = abs_b;
    }
    
    if (abs_b > fabs(a.z))
    {
        ret.z = abs_b;
    }
    
    return ret;
}


inline point_t min_magn(const point_t& a, const float b)
{
    point_t ret(a);
    const float abs_b = fabs(b);
    if (abs_b < fabs(a.x))
    {
        ret.x = abs_b;
    }
    
    if (abs_b < fabs(a.y))
    {
        ret.y = abs_b;
    }
    
    if (abs_b < fabs(a.z))
    {
        ret.z = abs_b;
    }
    
    return ret;
}


inline point_t fabs(const point_t &a)
{
    return point_t(fabs(a.x), fabs(a.y), fabs(a.z));
}


inline point_t floor_to_zero(point_t a, const float f)
{
    if (fabs(a.x) < f)
    {
        a.x = 0.0;
    }

    if (fabs(a.y) < f)
    {
        a.y = 0.0;
    }

    if (fabs(a.z) < f)
    {
        a.z = 0.0;
    }

    return a;
}


/*****************************************************
 Function to get the magnitude of a vector.
 
 a is the vector to find the magnitude of and its 
 magnitude is returned.
*****************************************************/
inline float magnitude(const point_t &a)
{
    return sqrt((a.x * a.x) + (a.y * a.y) + (a.z * a.z));
}


/*****************************************************
 Function to write to rotate a point about the origin.
 
 a is the point to be rotated. r is an arbitary vector
 to rotate about. theta is the angle to rotate by.
 The rotated point is returned through a.
*****************************************************/
inline void rotate_about_origin(point_t *const a, const point_t *const r, const float theta)
{
    /* Rotate the origin into the new coordinate system */
    point_t p = *a;
    point_t q(0.0,0.0,0.0);
   
    const float costheta = cos(theta);
    const float sintheta = sin(theta);
    
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
inline void rotate_about_x_axis(point_t *const a, const float theta)
{
    const point_t p = *a;
    point_t q(p.x,0.0,0.0);
   
    const float costheta = cos(theta);
    const float sintheta = sin(theta);
    
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
inline void rotate_about_y_axis(point_t *const a, const float theta)
{
    const point_t p = *a;
    point_t q(0.0,p.y,0.0);
   
    const float costheta = cos(theta);
    const float sintheta = sin(theta);
    
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
inline void rotate_about_z_axis(point_t *const a, const float theta)
{
    const point_t p = *a;
    point_t q(0.0,0.0,p.z);
   
    const float costheta = cos(theta);
    const float sintheta = sin(theta);
    
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
inline point_t perpendicular(const point_t &a)
{
    return point_t(a.z, a.y, -a.x);
}


/*****************************************************
 Function to compute the cross product of 2 vectors.
 
 computes c = a X b.
*****************************************************/
inline void cross_product(const point_t &a, const point_t &b, point_t *const c)
{
    c->x = (a.y * b.z) - (a.z * b.y);
    c->y = (a.z * b.x) - (a.x * b.z);
    c->z = (a.x * b.y) - (a.y * b.x);
}


/*****************************************************
 Function to compute the cross product of 2 vectors.
 
 computes ret = a X b.
*****************************************************/
inline point_t cross_product(const point_t &a, const point_t &b)
{
    return point_t((a.y * b.z) - (a.z * b.y), (a.z * b.x) - (a.x * b.z), (a.x * b.y) - (a.y * b.x));
}


/*****************************************************
 Function to compute the dot product of 2 vectors.
 
 return a . b.
*****************************************************/
inline float dot_product(const point_t &a, const point_t &b)
{
    return ((a.x * b.x) + (a.y * b.y)+ (a.z * b.z));
}


/*****************************************************
 Function to normalise vectors.
 
 leaves a normalised.
*****************************************************/
inline void normalise(point_t *const a)
{
    const float dist = inverse_sqrt((a->x * a->x) + (a->y * a->y) + (a->z * a->z));
    a->x *= dist;
    a->y *= dist;
    a->z *= dist;
    return;
}


inline point_t normalise(point_t a)
{
    const float dist = inverse_sqrt((a.x * a.x) + (a.y * a.y) + (a.z * a.z));
    a.x *= dist;
    a.y *= dist;
    a.z *= dist;
    return a;
}

#endif /* #ifndef __POINT_T_H__ */
