#pragma once

/* Standard headers */
#include <algorithm>
#include <iomanip>
#include <limits>
#include <numeric>
#include <sstream>
#include <vector>

/* Common header */
#include "logging.h"

/* Forward declarations */
class matrix_3d;

namespace raptor_physics
{

/* Class to define a solver of the linear complementarity problem, LCP */
/* Problem should be of the form w = Mz + q */
/* Uses the Lemke algorithm to solve the LCP */
class lcp_solver
{
    public :
        /* CTOR, lcp_solver does assume ownership of any data */
        lcp_solver(const int m_size, const int n_size)
            : _m(new float[m_size * n_size]), _q(new float[m_size]), _e(new float[m_size]), _w_def(new int [m_size]), _z_def(new int [n_size + 1]),
              _m_size(m_size), _n_size(n_size)
        {
            // METHOD_LOG;

            /* Fill in perturbation */
            _e[0] = 0.999999;
            for (int i = 1; i < _m_size; ++i)
            {
                _e[i] = _e[i - 1] * _e[0];
            }

            /* Fill auxillary vectors of which z or w are in z and w */
            /* Note that entry 0 is needed for z0 to be introduced */
            int def = 0;
            for (int i = 0; i < _n_size + 1; ++i)
            {
                _z_def[i] = def++;
            }

            /* Note this is 1 based */
            for (int i = 0; i < _m_size; ++i)
            {
                _w_def[i] = def++;
            }

            /* Fill in z0 */
            for (int i = 0; i < _m_size; ++i)
            {
                _m[i] = 1.0;
            }
        }

        /* Copy CTOR */
        lcp_solver(const lcp_solver &rhs)
            : _m(new float[rhs._m_size * rhs._n_size]), _q(new float[rhs._m_size]), _e(new float[rhs._m_size]), _w_def(new int [rhs._m_size]), 
              _z_def(new int [rhs._n_size + 1]), _m_size(rhs._m_size), _n_size(rhs._n_size)
        {
            // METHOD_LOG;

            memcpy(_m, rhs._m, _m_size * _n_size * sizeof(float));
            memcpy(_q, rhs._q, _m_size * sizeof(float));
            memcpy(_e, rhs._e, _m_size * sizeof(float));
            memcpy(_z_def, rhs._z_def, (_n_size + 1) * sizeof(int));
            memcpy(_w_def, rhs._w_def, _m_size * sizeof(int));
        }

        /* Move CTOR */
        lcp_solver(lcp_solver &&rhs)
            : _m(rhs._m), _q(rhs._q), _e(rhs._e), _w_def(rhs._w_def), _z_def(rhs._z_def), _m_size(rhs._m_size), _n_size(rhs._n_size)
        {
            // METHOD_LOG;

            rhs._m = nullptr;
            rhs._q = nullptr;
            rhs._e = nullptr;
            rhs._z_def = nullptr;
            rhs._w_def = nullptr;
        }


        /* DTOR */
        ~lcp_solver()
        {
            // METHOD_LOG;

            if (_m != nullptr)
            {
                delete [] _m;
            }

            if (_q != nullptr)
            {
                delete [] _q;
            }
            
            if (_e != nullptr)
            {
                delete [] _e;
            }
            
            if (_w_def != nullptr)
            {
                delete [] _w_def;
            }
            
            if (_z_def != nullptr)
            {
                delete [] _z_def;
            }
        }

        float * initialise_m() { return &_m[_m_size];   }
        float * initialise_q() { return _q;             }

        /* Solve the LCP using the Lemke algorithm */
        /* Returns true to show that a solution was found else false (in which case a solution is not feasible) */
        bool solve(float *const z)
        {
            // METHOD_LOG;
            dump();

            /* Find a pivot for Z0 */
            int pivot_row = find_pivot();
            // BOOST_LOG_TRIVIAL(trace) << "Found initial pivot row: " << pivot_row;

            /* Check if there is anything to do */
            if (pivot_row == -1)
            {
                // BOOST_LOG_TRIVIAL(trace) << "Nothing to be done";
                return false;
            }

            /* While it is possible to pivot and Z0 remains basic */ 
            int to_enter = 0;
            while (_w_def[pivot_row] != 0)
            {
                /* Eliminate the pivot element */
                const int left = gaussian_elimination(pivot_row, to_enter);
                // BOOST_LOG_TRIVIAL(trace) << "Variable leaving the dictionary: " << ((left > _n_size) ? "w" : "z") << ((left > _n_size) ? (left - _n_size) : left);
                dump();

                /* Find complementary variable to the new non-basic variable */
                const int complementary = (left > _n_size) ? (left - _n_size) : (left + _n_size);
                to_enter = std::distance(&_z_def[0], std::find(&_z_def[0], &_z_def[_n_size], complementary));
                // BOOST_LOG_TRIVIAL(trace) << "Variable to enter the dictionary: " << ((_z_def[to_enter] > _n_size) ? "w" : "z") << ((_z_def[to_enter] > _n_size) ? (_z_def[to_enter] - _n_size) : _z_def[to_enter]);

                /* Find new pivot row */
                pivot_row = find_pivot(to_enter);
                BOOST_LOG_TRIVIAL(trace) << "Pivot row: " << pivot_row;

                /* Check for no solution */
                if (pivot_row < 0)
                {
                    BOOST_LOG_TRIVIAL(trace) << "No Solution, done";
                    return false;
                }
            }
            gaussian_elimination_q(pivot_row, to_enter);
            dump();

            /* Reorder and copy out results */
            /* z is 0 unless an element in w represents it */
            memset(z, 0, _m_size * sizeof(float));
            for (int i = 0; i < _m_size; ++i)
            {
                if (_w_def[i] <= _n_size)
                {
                    z[_w_def[i] - 1] = _q[i];
                }
            }

            BOOST_LOG_TRIVIAL(trace) << "Terminal dictionary, z: (" << z[0] << array_to_stream(&z[1], &z[_m_size], [this](std::stringstream *s, const float z)
            {
                (*s) << ", " << z;
                return s;
            }, ")");

            /* Return if a solution is possible */
            return true;
        }

    private :
        /* Find the pivot row */
        /* Specifically the row containing a negative w with the limiting (smallest) ratio to negative q */
        /* Returns the row of the pivot or -1 to show that nothing can leave the dictionary */
        int find_pivot(const int w) const
        {
            /* Find the row with the minimum of the negative ratios of the constant to coefficient w */
            /* This is reposed as find the maximum of the positive ratio */
            int row = -1;
            float q_limit = std::numeric_limits<float>::lowest();
            float e_limit = 1.0;
            for (int i = 0; i < _m_size; ++i)
            {
                if (_m[(w * _m_size) + i] < 0.0) 
                {
                    /* q must be positive so r must be negative */
                    const float r = _q[i] / _m[(w * _m_size) + i];
                    if ((r > q_limit) || ((r == q_limit) && (_e[i] < e_limit)))
                    {
                        row = i;
                        q_limit = r;
                        e_limit = _e[i];
                    }
                }
            }

            return row;
        }


        /* Overload of find pivot for the first phase pivot search */
        /* Returns the row of the pivot or -1 to show that the system is already a terminal dictionary */
        int find_pivot() const
        {
            /* Find the row with the most negative constant for the first phase */
            int row = -1;
            float q_limit = 0.0;
            float e_limit = 1.0;
            for (int i = 0; i < _m_size; ++i)
            {
                if ((_q[i] < q_limit) || ((_q[i] == q_limit) && (_e[i] < e_limit)))
                {
                    row = i;
                    q_limit = _q[i];
                    e_limit = _e[i];
                }
            }

            return row;
        }


        /* Eliminate row based on the pivot */
        /* w_idx is the index of the row being used as the pivot */
        /* z_idx is the index of the variable being entered into the dictionary */
        /* Returns the variable id that left the dictionary */
        /* TODO - Consider moving the pivot to temporary storage so it can be updated like other rows */
        int gaussian_elimination(const int w_idx, const int z_idx)
        {
            const float divisor = 1.0 / _m[(z_idx * _m_size) + w_idx];
            
            /* Eliminate each row except the pivot row */
            for (int i = 0; i < _m_size; ++i)
            {
                if (i != w_idx)
                {
                    const float fac = _m[(z_idx * _m_size) + i] * divisor;
                    //std::cout << "fac: " << fac  << std::endl;
                    _q[i] -= _q[w_idx] * fac;
                    _e[i] -= _e[w_idx] * fac;
                    
                    for (int j = 0; j < _n_size; ++j)
                    {
                        if (j != z_idx)
                        {
                            _m[(j * _m_size) + i] -= _m[(j * _m_size) + w_idx] * fac;
                        }
                    }
                    _m[(z_idx * _m_size) + i] = fac;
                }
            }

            /* Update the pivot row */
            _q[w_idx] *= -divisor;
            _e[w_idx] *= -divisor;
            for (int i = 0; i < _n_size; ++i)
            {
                if (i != z_idx)
                {
                    _m[(i * _m_size) + w_idx] = -_m[(i * _m_size) + w_idx] * divisor;
                }
            }
            _m[(z_idx * _m_size) + w_idx] = divisor;

            /* Update the auxillary information */
            const int left = _w_def[w_idx];
            _w_def[w_idx] = _z_def[z_idx];
            _z_def[z_idx] = left;

            return left;
        }


        /* Gaussian elimination for the last iteration. This time on q and the w_def need updating */
        void gaussian_elimination_q(const int w_idx, const int z_idx)
        {
            const float divisor = 1.0 / _m[(z_idx * _m_size) + w_idx];
            
            /* Update all q except the pivot row */
            for (int i = 0; i < _m_size; ++i)
            {
                if (i != w_idx)
                {
                    const float fac = _m[(z_idx * _m_size) + i] * divisor;
                    _q[i] -= _q[w_idx] * fac;
                }
            }

            /* Update the pivot row */
            _q[w_idx] *= -divisor;
            _w_def[w_idx] = _z_def[z_idx];
        }


        void dump() const
        {
            BOOST_LOG_TRIVIAL(trace) << array_to_stream(&_z_def[0], &_z_def[_n_size], [this](std::stringstream *s, const int z)
            {
                (*s) << std::setw(4) << ((z > _n_size) ? "w" : "z") << ((z > _n_size) ? (z - _n_size) : z) << "\t";
                return s;
            }, "   q\t   e");

            for (int i = 0; i < _m_size; ++i)
            {
                std::stringstream post;
                post << std::setw(4) << std::setprecision(2) << _q[i] << "\t" << std::setw(4) << std::setprecision(2) << _e[i];
                BOOST_LOG_TRIVIAL(trace) << array_to_stream(&_m[i], [this](std::stringstream *s, const float m)
                {
                    (*s) << std::setw(4) << std::setprecision(2) << m << "\t";
                    return s;
                }, _m_size * _n_size, _m_size, post.str());
            }
        }


        float *     _m;
        float *     _q;
        float *     _e;
        int *       _w_def;
        int *       _z_def;
        const int   _m_size;
        const int   _n_size;

};


lcp_solver squared_distance_solver(const matrix_3d &p0, const matrix_3d &p1, const std::vector<int> &e0, const std::vector<int> &e1);

}; /* namespace raptor_physics */
