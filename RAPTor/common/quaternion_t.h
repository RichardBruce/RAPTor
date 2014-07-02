#ifndef __QUATERNION_T_H__
#define __QUATERNION_T_H__

/* Boost headers */
#include "boost/serialization/access.hpp"
#include "boost/archive/xml_oarchive.hpp"

/* Common headers */
#include "point_t.h"
#include "common.h"
#include "logging.h"


/* Forward declarations */
class quaternion_t;
inline point_t cross_product(const quaternion_t &a, const point_t &b);

/* Class representing a quaternion_t */
class quaternion_t
{
    public :
        /* CTOR */
        /* From components */
        quaternion_t(const fp_t w = 1.0, const fp_t x = 0.0, const fp_t y = 0.0, const fp_t z = 0.0)
            : w(w), x(x), y(y), z(z) { };

        /* From axis and angle */
        quaternion_t(const point_t &axis, const fp_t theta)
            : w(), x(axis.x), y(axis.y), z(axis.z)
            {
                const fp_t half_theta = theta * 0.5;
                const fp_t cos_theta = cos(half_theta);
                const fp_t sin_theta = sin(half_theta);
                w = cos_theta;
                x *= sin_theta;
                y *= sin_theta;
                z *= sin_theta;
            };

        /* From Euler angles */
        quaternion_t(const point_t &a)
        {
            const fp_t cos_x = cos(a.x * 0.5);
            const fp_t cos_y = cos(a.y * 0.5);
            const fp_t cos_z = cos(a.z * 0.5);

            const fp_t sin_x = sin(a.x * 0.5);
            const fp_t sin_y = sin(a.y * 0.5);
            const fp_t sin_z = sin(a.z * 0.5);

            w = (cos_x * cos_y * cos_z) + (sin_x * sin_y * sin_z);
            x = (sin_x * cos_y * cos_z) - (cos_x * sin_y * sin_z);
            y = (cos_x * sin_y * cos_z) + (sin_x * cos_y * sin_z);
            z = (cos_x * cos_y * sin_z) - (sin_x * sin_y * cos_z);
        }

        /* Allow default DTOR, copy and assignment */

        /* Operators */
        quaternion_t operator-() const
        {
            return quaternion_t(w, -x, -y, -z);
        }

        quaternion_t& operator+=(const quaternion_t &rhs)
        {
            w += rhs.w;
            x += rhs.x;
            y += rhs.y;
            z += rhs.z;
            return *this;
        }

        quaternion_t& operator-=(const quaternion_t &rhs)
        {
            w -= rhs.w;
            x -= rhs.x;
            y -= rhs.y;
            z -= rhs.z;
            return *this;
        }

        quaternion_t& operator*=(const quaternion_t &rhs)
        {
            const fp_t tmp_w = (w * rhs.w) - (x * rhs.x) - (y * rhs.y) - (z * rhs.z);
            const fp_t tmp_x = (w * rhs.x) + (x * rhs.w) + (y * rhs.z) - (z * rhs.y);
            const fp_t tmp_y = (w * rhs.y) - (x * rhs.z) + (y * rhs.w) + (z * rhs.x);
            const fp_t tmp_z = (w * rhs.z) + (x * rhs.y) - (y * rhs.x) + (z * rhs.w);

            w = tmp_w;
            x = tmp_x;
            y = tmp_y;
            z = tmp_z;
            return *this;
        }

        quaternion_t& operator+=(const point_t &rhs)
        {
            x += rhs.x;
            y += rhs.y;
            z += rhs.z;
            return *this;
        }

        quaternion_t& operator-=(const point_t &rhs)
        {
            x -= rhs.x;
            y -= rhs.y;
            z -= rhs.z;
            return *this;
        }

        quaternion_t& operator+=(const fp_t &rhs)
        {
            w += rhs;
            return *this;
        }

        quaternion_t& operator-=(const fp_t &rhs)
        {
            w -= rhs;
            return *this;
        }

        bool operator==(const quaternion_t &rhs) const
        {
            return (w == rhs.w) && (x == rhs.x) && (y == rhs.y) && (z == rhs.z);
        }

        /* Functions */
        quaternion_t& conjugate()
        {
            x = -x;
            y = -y;
            z = -z;
            return *this;
        }

        point_t rotate(const point_t p) const
        {
            return p + (2.0 * cross_product(*this, cross_product(*this, p))) + (2.0 * w * cross_product(*this, p));
        }

        quaternion_t rotate(point_t *const p) const
        {
            (*p) = (*p) + (2.0 * cross_product(*this, cross_product(*this, (*p)))) + (2.0 * w * cross_product(*this, (*p)));
            return *this;
        }

        const quaternion_t& rotation_matrix(fp_t *const r) const
        {
            const fp_t xx = x * x;
            const fp_t xy = x * y;
            const fp_t xz = x * z;
            const fp_t xw = x * w;

            const fp_t yy = y * y;
            const fp_t yz = y * z;
            const fp_t yw = y * w;

            const fp_t zz = z * z;
            const fp_t zw = z * w;

            r[0] = 1.0 - 2.0 * (yy + zz);
            r[1] =       2.0 * (xy - zw);
            r[2] =       2.0 * (xz + yw);

            r[3] =       2.0 * (xy + zw);
            r[4] = 1.0 - 2.0 * (xx + zz);
            r[5] =       2.0 * (yz - xw);

            r[6] =       2.0 * (xz - yw);
            r[7] =       2.0 * (yz + xw);
            r[8] = 1.0 - 2.0 * (xx + yy);

            return *this;
        }

        /* Data members */
        fp_t w;
        fp_t x;
        fp_t y;
        fp_t z;

    private :
        friend class boost::serialization::access;

        template<class Archive>
        void serialize(Archive & ar, const unsigned int version)
        {
            ar & BOOST_SERIALIZATION_NVP(w);
            ar & BOOST_SERIALIZATION_NVP(x);
            ar & BOOST_SERIALIZATION_NVP(y);
            ar & BOOST_SERIALIZATION_NVP(z);
        }
};


/* Degugging */
inline ostream& operator<<(ostream &os, const quaternion_t &p)
{
    return os << p.w << ", " << p.x << ", " << p.y << ", " << p.z;
}

/* Binary operators */
inline const quaternion_t operator+(const quaternion_t &lhs, const fp_t rhs)
{
   return quaternion_t(lhs.w + rhs, lhs.x + rhs, lhs.y + rhs, lhs.z + rhs);
}

inline const quaternion_t operator-(const quaternion_t &lhs, const fp_t rhs)
{
   return quaternion_t(lhs.w - rhs, lhs.x - rhs, lhs.y - rhs, lhs.z - rhs);
}

inline const quaternion_t operator*(const quaternion_t &lhs, const fp_t rhs)
{
   return quaternion_t(lhs.w * rhs, lhs.x * rhs, lhs.y * rhs, lhs.z * rhs);
}

inline const quaternion_t operator+(const quaternion_t &lhs, const quaternion_t &rhs)
{
   return quaternion_t(lhs.w + rhs.w, lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z);
}

inline const quaternion_t operator-(const quaternion_t &lhs, const quaternion_t &rhs)
{
   return quaternion_t(lhs.w - rhs.w, lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z);
}

inline const quaternion_t operator*(const quaternion_t &lhs, const quaternion_t &rhs)
{
    return quaternion_t((lhs.w * rhs.w) - (lhs.x * rhs.x) - (lhs.y * rhs.y) - (lhs.z * rhs.z),
                      (lhs.w * rhs.x) + (lhs.x * rhs.w) + (lhs.y * rhs.z) - (lhs.z * rhs.y),
                      (lhs.w * rhs.y) - (lhs.x * rhs.z) + (lhs.y * rhs.w) + (lhs.z * rhs.x),
                      (lhs.w * rhs.z) + (lhs.x * rhs.y) - (lhs.y * rhs.x) + (lhs.z * rhs.w));
}

inline const quaternion_t operator+(const fp_t lhs, const quaternion_t &rhs)
{
   return quaternion_t(lhs + rhs.w, lhs + rhs.x, lhs + rhs.y, lhs + rhs.z);
}

inline const quaternion_t operator-(const fp_t lhs, const quaternion_t &rhs)
{
   return quaternion_t(lhs - rhs.w, lhs - rhs.x, lhs - rhs.y, lhs - rhs.z);
}

inline const quaternion_t operator*(const fp_t lhs, const quaternion_t &rhs)
{
   return quaternion_t(lhs * rhs.w, lhs * rhs.x, lhs * rhs.y, lhs * rhs.z);
}


/* Special functions */
inline fp_t magnitude(const quaternion_t &a)
{
    return sqrt((a.w * a.w) + (a.x * a.x) + (a.y * a.y) + (a.z * a.z));
}


inline void normalise(quaternion_t *const a)
{
    const fp_t dist = sqrt((a->w * a->w) + (a->x * a->x) + (a->y * a->y) + (a->z * a->z));
    a->w /= dist;
    a->x /= dist;
    a->y /= dist;
    a->z /= dist;
    return;
}


inline quaternion_t normalise(quaternion_t a)
{
    const fp_t dist = sqrt((a.w * a.w) + (a.x * a.x) + (a.y * a.y) + (a.z * a.z));
    a.w /= dist;
    a.x /= dist;
    a.y /= dist;
    a.z /= dist;
    return a;
}


inline point_t cross_product(const quaternion_t &a, const point_t &b)
{
    return point_t((a.y * b.z) - (a.z * b.y), (a.z * b.x) - (a.x * b.z), (a.x * b.y) - (a.y * b.x));
}


inline point_t cross_product(const point_t &a, const quaternion_t &b)
{
    return point_t((a.y * b.z) - (a.z * b.y), (a.z * b.x) - (a.x * b.z), (a.x * b.y) - (a.y * b.x));
}

#endif /* #ifndef __QUATERNION_T_H__ */
