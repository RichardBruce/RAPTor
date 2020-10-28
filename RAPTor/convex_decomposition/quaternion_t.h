#ifndef __QUATERNION_T_H__
#define __QUATERNION_T_H__

/* Boost headers */
#include "boost/serialization/access.hpp"
#include "boost/archive/xml_oarchive.hpp"

/* Common headers */
#include "point_t.h"


/* Forward declarations */
class quaternion_t;
inline point_t cross_product(const quaternion_t &a, const point_t &b);

/* Class representing a quaternion_t */
class quaternion_t
{
    public :
        /* CTOR */
        /* From components */
        quaternion_t(const float w = 1.0f, const float x = 0.0f, const float y = 0.0f, const float z = 0.0f)
            : w(w), x(x), y(y), z(z) { };

        /* From axis and angle */
        quaternion_t(const point_t &axis, const float theta)
            : w(), x(axis.x), y(axis.y), z(axis.z)
            {
                const float half_theta = theta * 0.5f;
                const float cos_theta = cos(half_theta);
                const float sin_theta = sin(half_theta);
                w = cos_theta;
                x *= sin_theta;
                y *= sin_theta;
                z *= sin_theta;
            };

        /* From Euler angles */
        quaternion_t(const point_t &a)
        {
            const float cos_x = cos(a.x * 0.5f);
            const float cos_y = cos(a.y * 0.5f);
            const float cos_z = cos(a.z * 0.5f);

            const float sin_x = sin(a.x * 0.5f);
            const float sin_y = sin(a.y * 0.5f);
            const float sin_z = sin(a.z * 0.5f);

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
            const float tmp_w = (w * rhs.w) - (x * rhs.x) - (y * rhs.y) - (z * rhs.z);
            const float tmp_x = (w * rhs.x) + (x * rhs.w) + (y * rhs.z) - (z * rhs.y);
            const float tmp_y = (w * rhs.y) - (x * rhs.z) + (y * rhs.w) + (z * rhs.x);
            const float tmp_z = (w * rhs.z) + (x * rhs.y) - (y * rhs.x) + (z * rhs.w);

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

        quaternion_t& operator+=(const float &rhs)
        {
            w += rhs;
            return *this;
        }

        quaternion_t& operator-=(const float &rhs)
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

        point_t rotate(const point_t &p) const
        {
            return p + (2.0f * cross_product(*this, cross_product(*this, p))) + (2.0f * w * cross_product(*this, p));
        }

        quaternion_t rotate(point_t *const p) const
        {
            (*p) = (*p) + (2.0f * cross_product(*this, cross_product(*this, (*p)))) + (2.0f * w * cross_product(*this, (*p)));
            return *this;
        }

        const quaternion_t& rotation_matrix(float *const r) const
        {
            const float xx = x * x;
            const float xy = x * y;
            const float xz = x * z;
            const float xw = x * w;

            const float yy = y * y;
            const float yz = y * z;
            const float yw = y * w;

            const float zz = z * z;
            const float zw = z * w;

            r[0] = 1.0f - 2.0f * (yy + zz);
            r[1] =        2.0f * (xy - zw);
            r[2] =        2.0f * (xz + yw);

            r[3] =        2.0f * (xy + zw);
            r[4] = 1.0f - 2.0f * (xx + zz);
            r[5] =        2.0f * (yz - xw);

            r[6] =        2.0f * (xz - yw);
            r[7] =        2.0f * (yz + xw);
            r[8] = 1.0f - 2.0f * (xx + yy);

            return *this;
        }

        /* Data members */
        float w;
        float x;
        float y;
        float z;

    private :
        // friend class boost::serialization::access;

        // template<class Archive>
        // void serialize(Archive & ar, const unsigned int version)
        // {
        //     ar & BOOST_SERIALIZATION_NVP(w);
        //     ar & BOOST_SERIALIZATION_NVP(x);
        //     ar & BOOST_SERIALIZATION_NVP(y);
        //     ar & BOOST_SERIALIZATION_NVP(z);
        // }
};


/* Degugging */
inline std::ostream& operator<<(std::ostream &os, const quaternion_t &p)
{
    return os << p.w << ", " << p.x << ", " << p.y << ", " << p.z;
}

/* Binary operators */
inline const quaternion_t operator+(const quaternion_t &lhs, const float rhs)
{
   return quaternion_t(lhs.w + rhs, lhs.x + rhs, lhs.y + rhs, lhs.z + rhs);
}

inline const quaternion_t operator-(const quaternion_t &lhs, const float rhs)
{
   return quaternion_t(lhs.w - rhs, lhs.x - rhs, lhs.y - rhs, lhs.z - rhs);
}

inline const quaternion_t operator*(const quaternion_t &lhs, const float rhs)
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

inline const quaternion_t operator+(const float lhs, const quaternion_t &rhs)
{
   return quaternion_t(lhs + rhs.w, lhs + rhs.x, lhs + rhs.y, lhs + rhs.z);
}

inline const quaternion_t operator-(const float lhs, const quaternion_t &rhs)
{
   return quaternion_t(lhs - rhs.w, lhs - rhs.x, lhs - rhs.y, lhs - rhs.z);
}

inline const quaternion_t operator*(const float lhs, const quaternion_t &rhs)
{
   return quaternion_t(lhs * rhs.w, lhs * rhs.x, lhs * rhs.y, lhs * rhs.z);
}


/* Special functions */
inline float magnitude(const quaternion_t &a)
{
    return sqrt((a.w * a.w) + (a.x * a.x) + (a.y * a.y) + (a.z * a.z));
}


inline void normalise(quaternion_t *const a)
{
    const float dist = sqrt((a->w * a->w) + (a->x * a->x) + (a->y * a->y) + (a->z * a->z));
    a->w /= dist;
    a->x /= dist;
    a->y /= dist;
    a->z /= dist;
    return;
}


inline quaternion_t normalise(quaternion_t a)
{
    const float dist = sqrt((a.w * a.w) + (a.x * a.x) + (a.y * a.y) + (a.z * a.z));
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