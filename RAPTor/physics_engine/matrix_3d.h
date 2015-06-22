#pragma once

/* Standard headers */
#include <vector>

/* Boost headers */
#include "boost/noncopyable.hpp"

/* Common headers */
#include "common.h"
#include "point_t.h"
#include "quaternion_t.h"


class matrix_3d : private boost::noncopyable
{
    public :
        /* Ownership is taken of data and it will be deleted */
        matrix_3d(const std::vector<point_t> &data) : _data(data) {  };
            
        /* Allow default DTOR */
            
        /* Standard operators. */
        matrix_3d& operator+=(const point_t &p)
        {
            for (auto &d : _data)
            {
                d += p;
            }
            
            return *this;
        }

        matrix_3d& operator-=(const point_t &p)
        {
            for (auto &d : _data)
            {
                d -= p;
            }
            
            return *this;
        }

        point_t operator*(const point_t& rhs) const
        {
            /* Calculate */
            return point_t (((rhs.x * _data[0].x) + (rhs.y * _data[0].y) + (rhs.z * _data[0].z)),
                            ((rhs.x * _data[1].x) + (rhs.y * _data[1].y) + (rhs.z * _data[1].z)),
                            ((rhs.x * _data[2].x) + (rhs.y * _data[2].y) + (rhs.z * _data[2].z)));
        }

        const point_t& operator[](const int i) const
        {
            return _data[i];
        }

        bool operator==(const matrix_3d &m)
        {
            for (unsigned int i = 0; i < _data.size(); ++i)
            {
                if (_data[i] != m._data[i])
                {
                    return false;
                }
            }

            return true;
        }

        /* Find the maximum and minimum values in each column. */
        const matrix_3d& get_extremities(point_t *const hi, point_t *const lo) const
        {
            /* Initialise to the first data point */
            *hi = _data[0];
            *lo = _data[0];
            
            /* Loop finding extremities */
            for (unsigned int i = 1; i < _data.size(); i++)
            {
                *hi = max(*hi, _data[i]);
                *lo = min(*lo, _data[i]);
            }
            
            return *this;
        }
        
        const point_t& get_row(const int i) const
        {
            return _data[i];
        }
        
        int size() const
        {
            return _data.size();
        }

        const matrix_3d& dump() const
        {
            for (unsigned int i = 0; i < _data.size(); i++)
            {
                std::cout << "Point: " << i << " " << _data[i] << std::endl;
            }
            
            return *this;
        }
        
    private :
        /* Private CTOR for return temporaries */
        matrix_3d() : _data() {  };

        /* Calculate a 2x2 determinant */
        inline float determinant_2x2(const float a00, const float a01, const float a10, const float a11) const
        {
            return (a00 * a11) - (a01 * a10);
        }

        /* Note the data goes accoss the colums and then down the rows */
        std::vector<point_t>    _data;  /* Contents of the matrix   */
        
};
