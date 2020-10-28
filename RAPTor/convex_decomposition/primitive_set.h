#pragma once

/* Standard headers */
#include <cstring>
#include <limits>
#include <vector>

/* Boost headers */
#include "boost/noncopyable.hpp"

/* Common headers */
#include "point_t.h"

/* Convex decomposition headers */
#include "convex_mesh.h"
#include "plane.h"
#include "voxel_value.h"


namespace raptor_convex_decomposition
{
class primitive_set : private boost::noncopyable
{
    public :
        /* Ctor */
        primitive_set()
        {
            memset(_d, 0, sizeof(float) * 9);
        }

        /* Virtual Dtor for derived types */
        virtual ~primitive_set() { };

        const convex_mesh & convex_hull()                   const   { return _convex_hull; };
        convex_mesh &       convex_hull()                           { return _convex_hull; };
        float               eigen_value(const axis_t axis)  const   { return _d[static_cast<int>(axis)][static_cast<int>(axis)];  }
        
        float compute_preferred_cutting_direction(point_t *const dir)
        {
            const float ex = eigen_value(axis_t::x_axis);
            const float ey = eigen_value(axis_t::y_axis);
            const float ez = eigen_value(axis_t::z_axis);
            const float vx = (ey - ez) * (ey - ez);
            const float vy = (ex - ez) * (ex - ez);
            const float vz = (ex - ey) * (ex - ey);
            if ((vx < vy) && (vx < vz))
            {
                const float e = (ey * ey) + (ez * ez);
                (*dir) = point_t(1.0f, 0.0f, 0.0f);
                return (e == 0.0f) ? 0.0f : (1.0f - vx / e);
            }
            else if ((vy < vx) && (vy < vz))
            {
                const float e = (ex * ex) + (ez * ez);
                (*dir) = point_t(0.0f, 1.0f, 0.0f);
                return (e == 0.0f) ? 0.0f : (1.0f - vy / e);
            }
            else
            {
                const float e = (ex * ex) + (ey * ey);
                (*dir) = point_t(0.0f, 0.0f, 1.0f);
                return (e == 0.0f) ? 0.0f : (1.0f - vz / e);
            }
        }
        
        virtual primitive_set * select_on_surface()                                                                                                                 const = 0;
        virtual void            cut(const plane &p, primitive_set **const pos_part, primitive_set **const neg_part)                                                 const = 0;
        virtual void            intersect(const plane &p, std::vector<point_t> *const pos_pts, std::vector<point_t> *const neg_pts, const int sampling)             const = 0;
        virtual void            compute_cut_volumes(const plane &p, float *const pos_volume, float *const neg_volume)                                               const = 0;
        virtual void            compute_convex_hull(convex_mesh *const mesh, const int sampling = 1, const int cluster_size = 65536)                                const = 0;
        virtual void            compute_bounding_box()                                                                                                              = 0;
        virtual void            compute_principal_axes()                                                                                                            = 0;
        virtual void            align_to_principal_axes()                                                                                                           = 0;
        virtual void            revert_align_to_principal_axes()                                                                                                    = 0;
        virtual void            convert(convex_mesh *const mesh, const voxel_value_t value)                                                                         const = 0;
        virtual float           max_volume_error()                                                                                                                  const = 0;
        virtual float           compute_volume()                                                                                                                    const = 0;
        virtual int             number_of_primitives()                                                                                                              const = 0;
        virtual int             number_of_primitives_on_surface()                                                                                                   const = 0;
        virtual int             number_of_primitives_inside()                                                                                                       const = 0;
        virtual void            compute_axes_aligned_clipping_planes(std::vector<plane> *const planes, const int downsampling)                                      const = 0;
        virtual void            refine_axes_aligned_clipping_planes(std::vector<plane> *const planes, const plane &best, const int downsampling, const int index)   const = 0;


    protected :
        float       _d[3][3];

    private :
        convex_mesh _convex_hull;
};


inline void diagonalise(const float (&a)[3][3], float (&q)[3][3], float (&d)[3][3])
{
    // a must be a symmetric matrix.
    // returns q and d such that 
    // Diagonal matrix d = qt * a * q;  and  a = q*d*qt
    const int maxsteps = 24;
    float o[3], m[3];
    float qu [4] = { 0.0f, 0.0f, 0.0f, 1.0f };
    float jr[4];
    float aq[3][3];
    for (int i = 0; i < maxsteps; ++i)
    {
        // quat to matrix
        const float sqx = qu[0] * qu[0];
        const float sqy = qu[1] * qu[1];
        const float sqz = qu[2] * qu[2];
        const float sqw = qu[3] * qu[3];
        q[0][0]  = ( sqx - sqy - sqz + sqw);
        q[1][1]  = (-sqx + sqy - sqz + sqw);
        q[2][2]  = (-sqx - sqy + sqz + sqw);
        float tmp1     = qu[0] * qu[1];
        float tmp2     = qu[2] * qu[3];
        q[1][0]  = 2.0f * (tmp1 + tmp2);
        q[0][1]  = 2.0f * (tmp1 - tmp2);
        tmp1     = qu[0] * qu[2];
        tmp2     = qu[1] * qu[3];
        q[2][0]  = 2.0f * (tmp1 - tmp2);
        q[0][2]  = 2.0f * (tmp1 + tmp2);
        tmp1     = qu[1] * qu[2];
        tmp2     = qu[0] * qu[3];
        q[2][1]  = 2.0f * (tmp1 + tmp2);
        q[1][2]  = 2.0f * (tmp1 - tmp2);

        // aq = a * q
        aq[0][0] = (q[0][0] * a[0][0]) + (q[1][0] * a[0][1]) + (q[2][0] * a[0][2]);
        aq[0][1] = (q[0][1] * a[0][0]) + (q[1][1] * a[0][1]) + (q[2][1] * a[0][2]);
        aq[0][2] = (q[0][2] * a[0][0]) + (q[1][2] * a[0][1]) + (q[2][2] * a[0][2]);
        aq[1][0] = (q[0][0] * a[0][1]) + (q[1][0] * a[1][1]) + (q[2][0] * a[1][2]);
        aq[1][1] = (q[0][1] * a[0][1]) + (q[1][1] * a[1][1]) + (q[2][1] * a[1][2]);
        aq[1][2] = (q[0][2] * a[0][1]) + (q[1][2] * a[1][1]) + (q[2][2] * a[1][2]);
        aq[2][0] = (q[0][0] * a[0][2]) + (q[1][0] * a[1][2]) + (q[2][0] * a[2][2]);
        aq[2][1] = (q[0][1] * a[0][2]) + (q[1][1] * a[1][2]) + (q[2][1] * a[2][2]);
        aq[2][2] = (q[0][2] * a[0][2]) + (q[1][2] * a[1][2]) + (q[2][2] * a[2][2]);
        // d = Qt * aq
        d[0][0] = (aq[0][0] * q[0][0]) + (aq[1][0] * q[1][0]) + (aq[2][0] * q[2][0]); 
        d[0][1] = (aq[0][0] * q[0][1]) + (aq[1][0] * q[1][1]) + (aq[2][0] * q[2][1]); 
        d[0][2] = (aq[0][0] * q[0][2]) + (aq[1][0] * q[1][2]) + (aq[2][0] * q[2][2]); 
        d[1][0] = (aq[0][1] * q[0][0]) + (aq[1][1] * q[1][0]) + (aq[2][1] * q[2][0]); 
        d[1][1] = (aq[0][1] * q[0][1]) + (aq[1][1] * q[1][1]) + (aq[2][1] * q[2][1]); 
        d[1][2] = (aq[0][1] * q[0][2]) + (aq[1][1] * q[1][2]) + (aq[2][1] * q[2][2]); 
        d[2][0] = (aq[0][2] * q[0][0]) + (aq[1][2] * q[1][0]) + (aq[2][2] * q[2][0]); 
        d[2][1] = (aq[0][2] * q[0][1]) + (aq[1][2] * q[1][1]) + (aq[2][2] * q[2][1]); 
        d[2][2] = (aq[0][2] * q[0][2]) + (aq[1][2] * q[1][2]) + (aq[2][2] * q[2][2]);
        o[0]    = d[1][2];
        o[1]    = d[0][2];
        o[2]    = d[0][1];
        m[0]    = std::fabs(o[0]);
        m[1]    = std::fabs(o[1]);
        m[2]    = std::fabs(o[2]);

        const int k0    = (m[0] > m[1] && m[0] > m[2]) ? 0 : (m[1] > m[2]) ? 1 : 2;                 // index of largest element of offdiag
        const int k1    = (k0 + 1) % 3;
        const int k2    = (k0 + 2) % 3;
        if (o[k0] == 0.0f)
        {
            break;                                                                                  // diagonal already
        }

        float thet      = (d[k2][k2] - d[k1][k1]) / (2.0f * o[k0]);
        const float sgn = (thet > 0.0f) ? 1.0f : -1.0f;
        thet           *= sgn;                                                                              // make it positive
        const float t   = sgn / (thet + ((thet < 1.E6f) ? std::sqrt(thet * thet + 1.0f) : thet)) ;  // sign(t)/(|t|+sqrt(t^2+1))
        const float c   = 1.0f / std::sqrt(t * t + 1.0f);                                           //  c= 1/(t^2+1) , t=s/c 
        if (c == 1.0f)
        {
            break;                                                                                  // no room for improvement - reached machine precision.
        }
        jr[0 ]  = jr[1] = jr[2] = jr[3] = 0.0f;
        jr[k0]  = sgn * std::sqrt((1.0f - c) / 2.0f);                                               // using 1/2 angle identity sin(a/2) = sqrt((1-cos(a))/2)  
        jr[k0] *= -1.0f;                                                                            // since our quat-to-matrix convention was for v*m instead of m*v
        jr[3 ]  = std::sqrt(1.0f - jr[k0] * jr[k0]);
        if(jr[3] == 1.0f)
        {
            break;                                                                                  // reached limits of floating point precision
        }
        qu[0]    = ((qu[3] * jr[0]) + (qu[0] * jr[3]) + (qu[1] * jr[2]) - (qu[2] * jr[1]));
        qu[1]    = ((qu[3] * jr[1]) - (qu[0] * jr[2]) + (qu[1] * jr[3]) + (qu[2] * jr[0]));
        qu[2]    = ((qu[3] * jr[2]) + (qu[0] * jr[1]) - (qu[1] * jr[0]) + (qu[2] * jr[3]));
        qu[3]    = ((qu[3] * jr[3]) - (qu[0] * jr[0]) - (qu[1] * jr[1]) - (qu[2] * jr[2]));
        const float mq  = std::sqrt((qu[0] * qu[0]) + (qu[1] * qu[1]) + (qu[2] * qu[2]) + (qu[3] * qu[3]));
        qu[0]   /= mq;
        qu[1]   /= mq;
        qu[2]   /= mq;
        qu[3]   /= mq;
    }
}
}; /* namespace raptor_convex_decomposition */
