#pragma once


/* Standard headers */
#include <vector>

/* Common headers */
#include "logging.h"
#include "quaternion_t.h"

/* Physics headers */
#include "physics_common.h"


namespace raptor_physics
{
inline void invert_inertia_tensor(float *const out, const float *const in)
{
    /* Calculate the co factors */
    const float cof_11 = (in[1] * in[2]) - (in[4] * in[4]);
    const float cof_12 = (in[4] * in[5]) - (in[2] * in[3]);
    const float cof_13 = (in[3] * in[4]) - (in[1] * in[5]);

    const float cof_22 = (in[0] * in[2]) - (in[5] * in[5]);
    const float cof_23 = (in[3] * in[5]) - (in[0] * in[4]);

    const float cof_33 = (in[0] * in[1]) - (in[3] * in[3]);

    /* Calculate the determinate and check it isnt 0 */
    const float det = (in[0] * cof_11) + (in[3] * cof_12) + (in[5] * cof_13);
    if (det < raptor_physics::EPSILON)
    {
        out[0] = 0.0;
        out[1] = 0.0;
        out[2] = 0.0;
        out[3] = 0.0;
        out[4] = 0.0;
        out[5] = 0.0;
        return;
    }

    /* Calculate the inverse */
    const float det_inv = 1.0f / det;
    out[0] = cof_11 * det_inv;
    out[1] = cof_22 * det_inv;
    out[2] = cof_33 * det_inv;
    out[3] = cof_12 * det_inv;
    out[4] = cof_23 * det_inv;
    out[5] = cof_13 * det_inv;
}

/* Class representing the interia tensor of an arbitary polyhedron */
class inertia_tensor
{
    public :
        /* CTOR for objects with known inertia tensors. Takes ownership of it */
        inertia_tensor(float *const it, const point_t<> &com, const float m) : _it(it), _inv_it(new float [6]), _com(com), _m(m)
        {
            METHOD_LOG;
            if (m == std::numeric_limits<float>::infinity())
            {
                _inv_it[0] = 0.0f;
                _inv_it[1] = 0.0f;
                _inv_it[2] = 0.0f;
                _inv_it[3] = 0.0f;
                _inv_it[4] = 0.0f;
                _inv_it[5] = 0.0f;
            }
            else
            {
                invert_inertia_tensor(_inv_it, _it);
            }
        };

        /* CTOR for objects of arbitary shape with no known inertia tensor */
        inertia_tensor(const std::vector<point_t<>> &p, const std::vector<int> &e, const float r = 1.0f)
            : _it(new float [6]), _inv_it(new float [6])
        {
            METHOD_LOG;
            
            /* Set fields for infinite mass object */
            if (r == std::numeric_limits<float>::infinity())
            {
                BOOST_LOG_TRIVIAL(trace) << "Infinite mass object";

                _m = std::numeric_limits<float>::infinity();
                BOOST_LOG_TRIVIAL(trace) << "Mass: " << _m;

                _com = p[0];
                for (int i = 1; i < static_cast<int>(p.size()); ++i)
                {
                    _com += p[i];
                }
                _com /= p.size();
                BOOST_LOG_TRIVIAL(trace) << "Center of Mass: " << _com;

                _it[0] = std::numeric_limits<float>::infinity();
                _it[1] = std::numeric_limits<float>::infinity();
                _it[2] = std::numeric_limits<float>::infinity();
                _it[3] = std::numeric_limits<float>::infinity();
                _it[4] = std::numeric_limits<float>::infinity();
                _it[5] = std::numeric_limits<float>::infinity();
                BOOST_LOG_TRIVIAL(trace) << "Inertia Tensor: ";
                BOOST_LOG_TRIVIAL(trace) << "    " << _it[0] << " " << _it[3] << " " << _it[5];
                BOOST_LOG_TRIVIAL(trace) << "    " << _it[3] << " " << _it[1] << " " << _it[4];
                BOOST_LOG_TRIVIAL(trace) << "    " << _it[5] << " " << _it[4] << " " << _it[2];

                _inv_it[0] = 0.0f;
                _inv_it[1] = 0.0f;
                _inv_it[2] = 0.0f;
                _inv_it[3] = 0.0f;
                _inv_it[4] = 0.0f;
                _inv_it[5] = 0.0f;
                BOOST_LOG_TRIVIAL(trace) << "Inverse Inertia Tensor: ";
                BOOST_LOG_TRIVIAL(trace) << "    " << _inv_it[0] << " " << _inv_it[3] << " " << _inv_it[5];
                BOOST_LOG_TRIVIAL(trace) << "    " << _inv_it[3] << " " << _inv_it[1] << " " << _inv_it[4];
                BOOST_LOG_TRIVIAL(trace) << "    " << _inv_it[5] << " " << _inv_it[4] << " " << _inv_it[2];
                
                return;
            }

            /* Actual calculations for finite mass object */
            point_t<> g0;
            point_t<> g1;
            point_t<> g2;
            point_t<> f1;
            point_t<> f2;
            point_t<> f3;
            float integral[10] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
            for (int i = 0; i < static_cast<int>(e.size()); i += 3)
            {
                const point_t<> &v0 = p[e[i    ]];
                const point_t<> &v1 = p[e[i + 1]];
                const point_t<> &v2 = p[e[i + 2]];

                surface_integral_subexpressions(&g0.x, &g1.x, &g2.x, &f1.x, &f2.x, &f3.x, v0.x, v1.x, v2.x);   
                surface_integral_subexpressions(&g0.y, &g1.y, &g2.y, &f1.y, &f2.y, &f3.y, v0.y, v1.y, v2.y);   
                surface_integral_subexpressions(&g0.z, &g1.z, &g2.z, &f1.z, &f2.z, &f3.z, v0.z, v1.z, v2.z);

                const point_t<> n(cross_product(v1 - v0, v2 - v0));

                integral[0] += n.x * f1.x;
                integral[1] += n.x * f2.x;
                integral[2] += n.y * f2.y;
                integral[3] += n.z * f2.z;
                integral[4] += n.x * f3.x;
                integral[5] += n.y * f3.y;
                integral[6] += n.z * f3.z;
                integral[7] += n.x * ((v0.y * g0.x) + (v1.y * g1.x) + (v2.y * g2.x));
                integral[8] += n.y * ((v0.z * g0.y) + (v1.z * g1.y) + (v2.z * g2.y));
                integral[9] += n.z * ((v0.x * g0.z) + (v1.x * g1.z) + (v2.x * g2.z));
            }
            integral[0] *= (1.0f /   6.0f);
            integral[1] *= (1.0f /  24.0f);
            integral[2] *= (1.0f /  24.0f);
            integral[3] *= (1.0f /  24.0f);
            integral[4] *= (1.0f /  60.0f);
            integral[5] *= (1.0f /  60.0f);
            integral[6] *= (1.0f /  60.0f);
            integral[7] *= (1.0f / 120.0f);
            integral[8] *= (1.0f / 120.0f);
            integral[9] *= (1.0f / 120.0f);

            /* Mass */
            _m = integral[0] * r;
            BOOST_LOG_TRIVIAL(trace) << "Mass: " << _m;
            if (_m == 0.0)
            {
                BOOST_LOG_TRIVIAL(warning) << "Warning: Massless object";
                _com = point_t<>(0.0f, 0.0f, 0.0f);
            }
            else
            {
                /* Center of mass */
                _com.x = integral[1] / integral[0];
                _com.y = integral[2] / integral[0];
                _com.z = integral[3] / integral[0];
                BOOST_LOG_TRIVIAL(trace) << "Center of Mass: " << _com;
            }

            /* Inertia tensor relative to the origin */
            _it[0] = integral[5] + integral[6];
            _it[1] = integral[4] + integral[6];
            _it[2] = integral[4] + integral[5];
            _it[3] = -integral[7];
            _it[4] = -integral[8];
            _it[5] = -integral[9];

            /* Inertia tensor relative to center of mass */
            _it[0] -= integral[0] * ((_com.y * _com.y) + (_com.z * _com.z));
            _it[1] -= integral[0] * ((_com.z * _com.z) + (_com.x * _com.x));
            _it[2] -= integral[0] * ((_com.x * _com.x) + (_com.y * _com.y));
            _it[3] += integral[0] * _com.x * _com.y;
            _it[4] += integral[0] * _com.y * _com.z;
            _it[5] += integral[0] * _com.x * _com.z;
            _it[0] *= r;
            _it[1] *= r;
            _it[2] *= r;
            _it[3] *= r;
            _it[4] *= r;
            _it[5] *= r;
            BOOST_LOG_TRIVIAL(trace) << "Inertia Tensor: ";
            BOOST_LOG_TRIVIAL(trace) << "    " << _it[0] << " " << _it[3] << " " << _it[5];
            BOOST_LOG_TRIVIAL(trace) << "    " << _it[3] << " " << _it[1] << " " << _it[4];
            BOOST_LOG_TRIVIAL(trace) << "    " << _it[5] << " " << _it[4] << " " << _it[2];

            /* Invert the inertia tensor */
            invert_inertia_tensor(_inv_it, _it);
        }

        /* Copy CTOR */
        inertia_tensor(const inertia_tensor &i) : _it(new float [6]), _inv_it(new float [6]), _com(i._com), _m(i._m)
        {
            memcpy(_it, i._it, 6 * sizeof(float));
            memcpy(_inv_it, i._inv_it, 6 * sizeof(float));
        }

        /* Move CTOR */
        inertia_tensor(inertia_tensor &&i) : _it(i._it), _inv_it(i._inv_it), _com(i._com), _m(i._m)
        {
            i._it = nullptr;
            i._inv_it = nullptr;
        }

        /* DTOR */
        ~inertia_tensor()
        {
            METHOD_LOG;
            if (_it != nullptr)
            {
                delete [] _it;
            }

            if (_inv_it != nullptr)
            {
                delete [] _inv_it;
            }
        }

        /* Access functions */
        float               mass()              const { return _m;      }
        const point_t<>&    center_of_mass()    const { return _com;    }
        const float *const  tensor()            const { return _it;     }
        const float *const  inverse_tensor()    const { return _inv_it; }

        inertia_tensor& move_center_of_mass(const point_t<> &com)
        {
            _com += com;
            return *this;
        }

    private :
        void surface_integral_subexpressions(float *const g0, float *const g1, float *const g2, float *const f1, float *const f2, float *const f3,
            const float w0, const float w1, const float w2)
        {
            /* Common expressions */
            const float w0_p_w1 = w0 + w1;
            const float w0_sq   = w0 * w0;
            const float w0_w1   = w0_sq + (w1 * w0_p_w1);

            (*f1) = w0_p_w1 + w2;
            (*f2) = w0_w1 + w2 * (*f1);
            (*f3) = w0 * w0_sq + w1 * w0_w1 + w2 * (*f2);

            (*g0) = (*f2) + w0 * ((*f1) + w0);
            (*g1) = (*f2) + w1 * ((*f1) + w1);
            (*g2) = (*f2) + w2 * ((*f1) + w2);
        }

        float *     _it;        /* The inertia tensor           */
        float *     _inv_it;    /* The inverse inertia tensor   */
        point_t<>   _com;       /* The center of mass           */
        float       _m;         /* The mass                     */
};


class inertia_tensor_view
{
    public :
        inertia_tensor_view(const inertia_tensor &it, const quaternion_t &q)
        {
            /* Infinite mass, nothing really to do */
            if (it.mass() == std::numeric_limits<float>::infinity())
            {
                memcpy(_it,     it.tensor(),         6 * sizeof(float));
                memcpy(_inv_it, it.inverse_tensor(), 6 * sizeof(float));
                return;
            }

            /* Rotation matrix to orientate to */
            float rot[9];
            q.rotation_matrix(rot);

            /* R . I . Transpose(R) */
            rotate(_it, it.tensor(), rot);

            /* Invert (faster than rotate) */
            invert_inertia_tensor(_inv_it, _it);
       }

       /* Access functions */
       const float *const inverse_tensor()  const { return _inv_it; }
       const float *const tensor()          const { return _it;     }

    private :
        static void rotate(float *const out, const float *const in, const float *const rot)
        {
            const float it_tmp[9] = 
            {
                (rot[0] * in[0]) + (rot[1] * in[3]) + (rot[2] * in[5]),
                (rot[0] * in[3]) + (rot[1] * in[1]) + (rot[2] * in[4]),
                (rot[0] * in[5]) + (rot[1] * in[4]) + (rot[2] * in[2]),
                (rot[3] * in[0]) + (rot[4] * in[3]) + (rot[5] * in[5]),
                (rot[3] * in[3]) + (rot[4] * in[1]) + (rot[5] * in[4]),
                (rot[3] * in[5]) + (rot[4] * in[4]) + (rot[5] * in[2]),
                (rot[6] * in[0]) + (rot[7] * in[3]) + (rot[8] * in[5]),
                (rot[6] * in[3]) + (rot[7] * in[1]) + (rot[8] * in[4]),
                (rot[6] * in[5]) + (rot[7] * in[4]) + (rot[8] * in[2])
            };

            out[0] = (it_tmp[0] * rot[0]) + (it_tmp[1] * rot[1]) + (it_tmp[2] * rot[2]);
            out[3] = (it_tmp[0] * rot[3]) + (it_tmp[1] * rot[4]) + (it_tmp[2] * rot[5]);
            out[5] = (it_tmp[0] * rot[6]) + (it_tmp[1] * rot[7]) + (it_tmp[2] * rot[8]);
            out[1] = (it_tmp[3] * rot[3]) + (it_tmp[4] * rot[4]) + (it_tmp[5] * rot[5]);
            out[4] = (it_tmp[3] * rot[6]) + (it_tmp[4] * rot[7]) + (it_tmp[5] * rot[8]);
            out[2] = (it_tmp[6] * rot[6]) + (it_tmp[7] * rot[7]) + (it_tmp[8] * rot[8]);
        }

        float   _inv_it[6];
        float   _it[6];
};


/* operators */
inline point_t<> tensor_multiple(const float *const t, const point_t<> &p)
{
    const float x = (p.x * t[0]) + (p.y * t[3]) + (p.z * t[5]);
    const float y = (p.x * t[3]) + (p.y * t[1]) + (p.z * t[4]);
    const float z = (p.x * t[5]) + (p.y * t[4]) + (p.z * t[2]);
    return point_t<>(x, y, z);
}

inline point_t<> operator/(const point_t<> &lhs, const inertia_tensor &rhs)
{
    return tensor_multiple(rhs.inverse_tensor(), lhs);
}

inline point_t<> operator*(const inertia_tensor &lhs, const point_t<> &rhs)
{
    return tensor_multiple(lhs.tensor(), rhs);
}

inline point_t<> operator/(const point_t<> &lhs, const inertia_tensor_view &rhs)
{
    return tensor_multiple(rhs.inverse_tensor(), lhs);
}

inline point_t<> operator*(const inertia_tensor_view &lhs, const point_t<> &rhs)
{
    return tensor_multiple(lhs.tensor(), rhs);
}
}; /* namespace raptor_physics */
