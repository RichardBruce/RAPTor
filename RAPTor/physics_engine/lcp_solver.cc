/* Physics headers */
#include "matrix_3d.h"
#include "lcp_solver.h"


namespace raptor_physics
{              
lcp_solver squared_distance_solver(const matrix_3d &p0, const matrix_3d &p1, const std::vector<int> &e0, const std::vector<int> &e1)
{
    // METHOD_LOG;

    /* Calculate dimenstion and build the solver */
    const int total_pts = (e0.size() + e1.size()) / 3;
    const int m_dim = total_pts + 6;
    const int m_size = m_dim * m_dim;
    lcp_solver sol(m_dim, m_dim + 1);
    
    /* Fill in the objective function */
    float *m = sol.initialise_m();
    memset(m, 0, m_size * sizeof(float));
    for (int i = 0; i < 6; ++i)
    {
        m[(i * m_dim) + i] = 2.0;
    }

    for (int i = 0; i < 3; ++i)
    {
        m[(i * m_dim) + i + 3] = -2.0;
        m[((i + 3) * m_dim) + i] = -2.0;
    }

    /* Fill in the constraints */
    int q_idx = 6;
    int m_idx = 6 * m_dim;

    float *q = sol.initialise_q();
    memset(q, 0, m_dim * sizeof(float));
    for (int i = 0; i < static_cast<int>(e0.size()); i += 3)
    {
        /* Calculate face normal */
        const point_t ab(p0.get_row(e0[i]) - p0.get_row(e0[i + 1]));
        const point_t bc(p0.get_row(e0[i]) - p0.get_row(e0[i + 2]));
        const point_t norm(normalise(cross_product(bc, ab)));
        m[m_idx    ] = norm.x;
        m[m_idx + 1] = norm.y;
        m[m_idx + 2] = norm.z;

        /* Calculate dot product to a vertex */
        assert(q_idx < m_dim);
        q[q_idx++] = dot_product(p0.get_row(e0[i]), norm);
        m_idx += m_dim;
    }

    for (int i = 0; i < static_cast<int>(e1.size()); i += 3)
    {
        /* Calculate face normal */
        const point_t ab(p1.get_row(e1[i]) - p1.get_row(e1[i + 1]));
        const point_t bc(p1.get_row(e1[i]) - p1.get_row(e1[i + 2]));
        const point_t norm(normalise(cross_product(bc, ab)));
        m[m_idx + 3] = norm.x;
        m[m_idx + 4] = norm.y;
        m[m_idx + 5] = norm.z;

        /* Calculate dot product to a vertex */
        assert(q_idx < m_dim);
        q[q_idx++] = dot_product(p1.get_row(e1[i]), norm);
        m_idx += m_dim;
    }

    /* Build transpose of a in m */
    for (int i = 6; i < m_dim; ++i)
    {
        m[              i] = -m[(i * m_dim)    ];
        m[     m_dim  + i] = -m[(i * m_dim) + 1];
        m[(2 * m_dim) + i] = -m[(i * m_dim) + 2];
        m[(3 * m_dim) + i] = -m[(i * m_dim) + 3];
        m[(4 * m_dim) + i] = -m[(i * m_dim) + 4];
        m[(5 * m_dim) + i] = -m[(i * m_dim) + 5];
        assert(((5 * m_dim) + i) < m_size);
        assert(((i * m_dim) + 5) < m_size);
    }

    return sol;
}
}; /* namespace raptor_physics */
