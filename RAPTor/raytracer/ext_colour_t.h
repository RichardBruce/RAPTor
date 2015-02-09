#ifndef __EXT_COLOUR_T_H__
#define __EXT_COLOUR_T_H__

/* Boost headers */
#include "boost/serialization/access.hpp"

/* Common headers */
#include "common.h"

namespace raptor_raytracer
{
/* Class to hold a rgb colour as floating point numbers, to be rounded and saturated later */
class ext_colour_t
{
    public :
        float r, g, b;
        ext_colour_t(const float r, const float g, const float b) : r(r), g(g), b(b) { };
        ext_colour_t(const float r = 0.0f) : r(r), g(r), b(r) { };

        inline const bool          operator==(const ext_colour_t &rhs)  const   { return ((this->r == rhs.r) && (this->g == rhs.g) && (this->b == rhs.b));  }
        inline const bool          operator!=(const ext_colour_t &rhs)  const   { return ((this->r != rhs.r) || (this->g != rhs.g) || (this->b != rhs.b));  }

        inline const ext_colour_t& operator+=(const float rhs)                  { this->r += rhs;   this->g += rhs;   this->b += rhs;   return *this;       }
        inline const ext_colour_t& operator-=(const float rhs)                  { this->r -= rhs;   this->g -= rhs;   this->b -= rhs;   return *this;       }
        inline const ext_colour_t& operator*=(const float rhs)                  { this->r *= rhs;   this->g *= rhs;   this->b *= rhs;   return *this;       }
        inline const ext_colour_t& operator/=(const float rhs)                  { this->r /= rhs;   this->g /= rhs;   this->b /= rhs;   return *this;       }
        
        inline const ext_colour_t& operator+=(const ext_colour_t &rhs)          { this->r += rhs.r; this->g += rhs.g; this->b += rhs.b; return *this;       }
        inline const ext_colour_t& operator-=(const ext_colour_t &rhs)          { this->r -= rhs.r; this->g -= rhs.g; this->b -= rhs.b; return *this;       }
        inline const ext_colour_t& operator*=(const ext_colour_t &rhs)          { this->r *= rhs.r; this->g *= rhs.g; this->b *= rhs.b; return *this;       }
        inline const ext_colour_t& operator/=(const ext_colour_t &rhs)          { this->r /= rhs.r; this->g /= rhs.g; this->b /= rhs.b; return *this;       }

    private :
        friend class boost::serialization::access;

        template<class Archive>
        void serialize(Archive & ar, const unsigned int version)
        {
            ar & r;
            ar & g;
            ar & b;
        }
};

inline const ext_colour_t operator+(const ext_colour_t &lhs, const float rhs)
{
   return ext_colour_t(lhs.r + rhs, lhs.g + rhs, lhs.b + rhs);
}

inline const ext_colour_t operator-(const ext_colour_t &lhs, const float rhs)
{
   return ext_colour_t(lhs.r - rhs, lhs.g - rhs, lhs.b - rhs);
}

inline const ext_colour_t operator*(const ext_colour_t &lhs, const float rhs)
{
   return ext_colour_t(lhs.r * rhs, lhs.g * rhs, lhs.b * rhs);
}

inline const ext_colour_t operator/(const ext_colour_t &lhs, const float rhs)
{
   return ext_colour_t(lhs.r / rhs, lhs.g / rhs, lhs.b / rhs);
}

inline const ext_colour_t operator+(const ext_colour_t &lhs, const ext_colour_t &rhs)
{
   return ext_colour_t(lhs.r + rhs.r, lhs.g + rhs.g, lhs.b + rhs.b);
}

inline const ext_colour_t operator-(const ext_colour_t &lhs, const ext_colour_t &rhs)
{
   return ext_colour_t(lhs.r - rhs.r, lhs.g - rhs.g, lhs.b - rhs.b);
}

inline const ext_colour_t operator*(const ext_colour_t &lhs, const ext_colour_t &rhs)
{
   return ext_colour_t(lhs.r * rhs.r, lhs.g * rhs.g, lhs.b * rhs.b);
}

inline const ext_colour_t operator/(const ext_colour_t &lhs, const ext_colour_t &rhs)
{
   return ext_colour_t(lhs.r / rhs.r, lhs.g / rhs.g, lhs.b / rhs.b);
}

/*****************************************************
 Function to get the magnitude of a colour.
 
 a is the colour to find the magnitude of and its 
 magnitude is returned.
*****************************************************/
inline float magnitude(const ext_colour_t &a)
{
    return sqrt((a.r * a.r) + (a.g * a.g) + (a.b * a.b));
}
}; /* namespace raptor_raytracer */

#endif /* #ifndef __EXT_COLOUR_T_H__ */
