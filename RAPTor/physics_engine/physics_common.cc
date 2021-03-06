/* Standard headers */
#include <cmath>
#include <limits>

/* Common headers */
#include "common.h"
#include "logging.h"
#include "physics_common.h"
#include "point_t.h"


namespace raptor_physics
{
/* Find the component of v that projects on to n */
point_t<> project_vector(const point_t<> &v, const point_t<> &n)
{
    /* Check for 0 v */
    const float vel_magn = magnitude(v);
    if (vel_magn < raptor_physics::EPSILON)
    {
        return point_t<>(0.0f, 0.0f, 0.0f);
    }

    /* Project */
    const point_t<> vel_norm = v / vel_magn;
    return n * vel_magn * dot_product(n, vel_norm);
}


/* Find the most extreme vertex in direction d */
int find_support_vertex(const std::vector<point_t<>> &m, const point_t<> &d)
{
    float dummy;
    return find_support_vertex(m, d, &dummy);
}


/* Find the most extreme vertex in direction d and its projection */
int find_support_vertex(const std::vector<point_t<>> &m, const point_t<> &d, float *const val)
{
    /* Search all vertices for the most extreme in the direction d */
    int max_support_vertex = 0;
    (*val) = dot_product(m[0], d);
    for (int i = 1; i < static_cast<int>(m.size()); i++)
    {
        const float v = dot_product(m[i], d);
        if (v > (*val))
        {
            (*val) = v;
            max_support_vertex = i;
        }
    }
    
    return max_support_vertex;
}


int find_support_vertex(const std::vector<point_t<>> &m, const point_t<> &w, const point_t<> &c, const point_t<> &p, const point_t<> &n, float *const val)
{
    /* Check there is rotation */
    const float magn_w = magnitude(w);
    if (fabs(magn_w) < raptor_physics::EPSILON)
    {
        (*val) = 0.0f;
        return 0;
    }

    /* Loop constants */
    // const point_t<> n_cross_w(cross_product(n, w));

    /* Search all vertices for the most extreme in the direction d */
    int max_support_vertex = 0;
    (*val) = magnitude(cross_product(w, m[0]));
    // (*val) = (magnitude(cross_product(m[0], w_cross_n))) - dot_product((c + m[0]) - p, n);
    for (int i = 1; i < static_cast<int>(m.size()); ++i)
    {
        const float v = magnitude(cross_product(w, m[i]));
        // const float v = (magnitude(cross_product(m[i], w_cross_n))) - dot_product((c + m[i]) - p, n);
        if (v > (*val))
        {
            (*val) = v;
            max_support_vertex = i;
        }
    }
    
    return max_support_vertex;
}

int find_first_positive_real_root(float *const roots, const float a_coeff, const float b_coeff, const float c_coeff, const float d_coeff)
{
    int nr_roots;
    if (fabs(a_coeff) > 0.0f)
    {
        BOOST_LOG_TRIVIAL(trace) << "Solving as cubic";

        /* Normalise to a == 1 */
        const float b = b_coeff / a_coeff;
        const float c = c_coeff / a_coeff;
        const float d = d_coeff / a_coeff;

        /* Substitute x = y - A / 3 to make a depressed cubic (x^3 + px + q = 0) */
        const float b_sq = b * b;
        const float p = (1.0f / 3.0f) * (c - ((1.0f / 3.0f) * b_sq));
        const float q = (1.0f / 2.0f) * (((1.0f / 3.0f) * b * c) - ((2.0f / 27.0f) * b * b_sq) - d);

        /* Solve using Cardano's formula */
        const float p_cb = p * p * p;
        const float disc = q * q + p_cb;

        /* One real root */
        const float disc_sqrt = sqrt(disc);
        const float third_b = b * (1.0f / 3.0f);
        if (disc > 0.0f)
        {
            const float s = q + disc_sqrt;
            const float t = q - disc_sqrt;
            const float s_cbrt = ((s < 0.0f) ? -pow(-s, (1.0f / 3.0f)) : pow(s, (1.0f / 3.0f)));
            const float t_cbrt = ((t < 0.0f) ? -pow(-t, (1.0f / 3.0f)) : pow(t, (1.0f / 3.0f)));
            const float root = -third_b + s_cbrt + t_cbrt;

            nr_roots = 1;
            roots[0] = (root >= 0.0f) ? root : std::numeric_limits<float>::infinity();
        }        
        /* All real roots, at least 2 are equal */
        else if (disc == 0.0f)
        {
            const float q_cbrt = ((q < 0.0f) ? -pow(-q, (1.0f / 3.0f)) : pow(q, (1.0f / 3.0f)));
            const float root0 = -third_b + (2.0f * q_cbrt);
            const float root1 = -q_cbrt - third_b;
            
            nr_roots = 2;
            roots[0] = (root0 >= 0.0f) ? root0 : std::numeric_limits<float>::infinity();
            roots[1] = (root1 >= 0.0f) ? root1 : std::numeric_limits<float>::infinity();
        }
        /* Three real roots */
        else
        {
            const float neg_p = -p;
            const float neg_p_cb = -p_cb;
            const float u = acos(q / sqrt(neg_p_cb));
            const float v = 2.0f * sqrt(neg_p);
            const float root0 = -third_b + v * cos(u * (1.0f / 3.0f));
            const float root1 = -third_b + v * cos((u + 2.0f * PI) * (1.0f / 3.0f));
            const float root2 = -third_b + v * cos((u + 4.0f * PI) * (1.0f / 3.0f));

            nr_roots = 3;
            roots[0] = (root0 >= 0.0f) ? root0 : std::numeric_limits<float>::infinity();
            roots[1] = (root1 >= 0.0f) ? root1 : std::numeric_limits<float>::infinity();
            roots[2] = (root2 >= 0.0f) ? root2 : std::numeric_limits<float>::infinity();
        }
    }
    else if (fabs(b_coeff) > 0.0f)
    {
        /* Check for no root */
        BOOST_LOG_TRIVIAL(trace) << "Solving as quadratic";
        const float d = (c_coeff * c_coeff) - (4.0f * b_coeff * d_coeff);
        if (d < 0.0f)
        {
            BOOST_LOG_TRIVIAL(trace) << "No roots found";
            roots[0] = std::numeric_limits<float>::infinity();
            return 0;
        }

        /* Return first positive root */
        const float sqrt_d = sqrt(d);
        const float b_dot_2 =  2.0f * b_coeff;
        const float root0 = (-c_coeff - sqrt_d) / b_dot_2;
        const float root1 = (-c_coeff + sqrt_d) / b_dot_2;

        nr_roots = 2;
        roots[0] = (root0 >= 0.0f) ? root0 : std::numeric_limits<float>::infinity();
        roots[1] = (root1 >= 0.0f) ? root1 : std::numeric_limits<float>::infinity();
    }
    else if (fabs(c_coeff) > 0.0f)
    {
        BOOST_LOG_TRIVIAL(trace) << "Solving as linear";
        const float root = -d_coeff / c_coeff;

        nr_roots = 1;
        roots[0] = (root >= 0.0f) ? root : std::numeric_limits<float>::infinity();
    }
    else
    {
        BOOST_LOG_TRIVIAL(trace) << "Returning constant";

        nr_roots = 1;
        roots[0] = (fabs(d_coeff) < raptor_physics::EPSILON) ? 0.0f : std::numeric_limits<float>::infinity();
    }

    std::sort(&roots[0], &roots[nr_roots]);
    BOOST_LOG_TRIVIAL(trace) << array_to_stream(&roots[0], &roots[nr_roots], [](std::stringstream *s, const float r)
        {
            (*s) << r << ", ";
            return s;
        });

    return nr_roots;
}


/* Find the time that a translating point passes through a plane                */
/* pa is the point                                                              */
/* pb is a point on the plane                                                   */
/* nb is the normal of the plane                                                */
/* x0 is the position of the point during the frame                             */
/* q0 is the orientation of the point at the start of the frame                 */
/* q1 is the orientation of the point at the end of the frame                   */
float find_exact_none_translating_collision_time(const point_t<> &pa, const point_t<> &pb, const point_t<> &nb, const point_t<> &x0, const point_t<> &q0, 
    const point_t<> &q1, const float r0, const float r1)
{
    METHOD_LOG;
    BOOST_LOG_TRIVIAL(trace) << "pa: " << pa;
    BOOST_LOG_TRIVIAL(trace) << "pb: " << pb;
    BOOST_LOG_TRIVIAL(trace) << "nb: " << nb;
    BOOST_LOG_TRIVIAL(trace) << "x0: " << x0;
    BOOST_LOG_TRIVIAL(trace) << "q0: " << q0 << ", " << r0;
    BOOST_LOG_TRIVIAL(trace) << "q1: " << q1 << ", " << r1;

    /* Movements */
    const float r1_m_r0 = r1 - r0;
    const point_t<> q1_m_q0(q1 - q0);

    /* Factors of s inside dot product */
    const point_t<> q0_cross_p0(cross_product(q0, pa));
    const point_t<> q1_m_q0_cross_p0(cross_product(q1_m_q0, pa));
    const point_t<> a(2.0f * (cross_product(q1_m_q0, q1_m_q0_cross_p0) + (r1_m_r0 * q1_m_q0_cross_p0)));
    const point_t<> b(2.0f * (
                    cross_product(q0, q1_m_q0_cross_p0) + 
                    cross_product(q1_m_q0, q0_cross_p0) + 
                    (r0 * q1_m_q0_cross_p0) + 
                    (r1_m_r0 * q0_cross_p0)));
    const point_t<> c(pa + x0 - pb + (2.0f * (cross_product(q0, q0_cross_p0) + (r0 * q0_cross_p0))));

    /* Dot product with the plane normal */
    const float a_dot = dot_product(a, nb);
    const float b_dot = dot_product(b, nb);
    const float c_dot = dot_product(c, nb);
    BOOST_LOG_TRIVIAL(trace) << "Solving equation, a: " << a_dot << ", b: " << b_dot << ", c: " << c_dot;

    float roots[3];
    find_first_positive_real_root(roots, 0.0f, a_dot, b_dot, c_dot);
    return roots[0];
}


/* Find the time that a translating and rotating point passes through a plane   */
/* pa is the point                                                              */
/* pb is a point on the plane                                                   */
/* nb is the normal of the plane                                                */
/* x0 is the position of the point at the start of the frame                    */
/* x1 is the position of the point at the end of the frame                      */
/* q0 is the orientation of the point at the start of the frame                 */
/* q1 is the orientation of the point at the end of the frame                   */
float find_exact_collision_time(const point_t<> &pa, const point_t<> &pb, const point_t<> &nb, const point_t<> &x0, const point_t<> &x1,
    const point_t<> &q0, const point_t<> &q1, const float r0, const float r1)
{
    METHOD_LOG;
    BOOST_LOG_TRIVIAL(trace) << "pa: " << pa;
    BOOST_LOG_TRIVIAL(trace) << "pb: " << pb;
    BOOST_LOG_TRIVIAL(trace) << "nb: " << nb;
    BOOST_LOG_TRIVIAL(trace) << "x0: " << x0;
    BOOST_LOG_TRIVIAL(trace) << "x1: " << x1;
    BOOST_LOG_TRIVIAL(trace) << "q0: " << q0 << ", " << r0;
    BOOST_LOG_TRIVIAL(trace) << "q1: " << q1 << ", " << r1;

    /* Movements */
    const float r1_m_r0 = r1 - r0;
    const point_t<> q1_m_q0(q1 - q0);
    const point_t<> x1_m_x0(x1 - x0);

    /* Factors of s inside dot product */
    const point_t<> q0_cross_p0(cross_product(q0, pa));
    const point_t<> q1_m_q0_cross_p0(cross_product(q1_m_q0, pa));
    const point_t<> a(2.0f * (cross_product(q1_m_q0, q1_m_q0_cross_p0) + (r1_m_r0 * q1_m_q0_cross_p0)));
    const point_t<> b(x1_m_x0 + (2.0f * (
                    cross_product(q0, q1_m_q0_cross_p0) + 
                    cross_product(q1_m_q0, q0_cross_p0) + 
                    (r0 * q1_m_q0_cross_p0) + 
                    (r1_m_r0 * q0_cross_p0))));
    const point_t<> c(pa + x0 - pb + (2.0f * (cross_product(q0, q0_cross_p0) + (r0 * q0_cross_p0))));

    /* Dot product with the plane normal */
    const float a_dot = dot_product(a, nb);
    const float b_dot = dot_product(b, nb);
    const float c_dot = dot_product(c, nb);
    BOOST_LOG_TRIVIAL(trace) << "Solving equation, a: " << a_dot << ", b: " << b_dot << ", c: " << c_dot;

    float roots[3];
    find_first_positive_real_root(roots, 0.0f, a_dot, b_dot, c_dot);
    return roots[0];
}


/* Rotate a vector as done by the intersection algorithm        */
/* x is the vector to rotate                                    */
/* q0 is the orientation of the point at the start of the frame */
/* q1 is the orientation of the point at the end of the frame   */
/* r0 is the orientation of the point at the start of the frame */
/* r1 is the orientation of the point at the end of the frame   */
point_t<> interpolated_quaternion_rotate(const point_t<> &x, const point_t<> &q0, const point_t<> &q1, const float r0, const float r1, const float s)
{
    const point_t<> q_lerp(q0 + (s * (q1 - q0)));
    const point_t<> r_lerp(r0 + (s * (r1 - r0)));
    return x + (2.0f * cross_product(q_lerp, cross_product(q_lerp, x))) + (2.0f * r_lerp * cross_product(q_lerp, x));
}


/* Find the time that a translating and rotating point passes through a plane   */
/* pa is the point on the edge of object a                                      */
/* pb is the point on the edge of object b                                      */
/* ea an edge of object a                                                       */
/* eb an edge of object b                                                       */
/* x0 is the position of the point at the start of the frame                    */
/* x1 is the position of the point at the end of the frame                      */
/* q0 is the orientation of the point at the start of the frame                 */
/* q1 is the orientation of the point at the end of the frame                   */
float find_exact_collision_time(const point_t<> &pa, const point_t<> &pb, const point_t<> &ea, const point_t<> &eb, const point_t<> &x0, 
    const point_t<> &x1, const point_t<> &q0, const point_t<> &q1, const float r0, const float r1)
{
    METHOD_LOG;

    /* Movements */
    const float r1_m_r0 = r1 - r0;
    const point_t<> q1_m_q0(q1 - q0);
    const point_t<> x1_m_x0(x1 - x0);
    const point_t<> x0_m_pb(x0 - pb);

    /* Common cross products */
    const point_t<> pa_x_ea(cross_product(pa, ea));
    const point_t<> q0_x_pa_x_ea(cross_product(q0, pa_x_ea));
    const point_t<> q1_m_q0_x_pa_x_ea(cross_product(q1_m_q0, pa_x_ea));

    /* Part A polynomial */
    const point_t<> a_b(2.0f * (cross_product(q1_m_q0, q1_m_q0_x_pa_x_ea) + (r1_m_r0 * q1_m_q0_x_pa_x_ea)));
    const point_t<> a_c(2.0f * (
                    cross_product(q0, q1_m_q0_x_pa_x_ea) + 
                    cross_product(q1_m_q0, q0_x_pa_x_ea) + 
                    (r0 * q1_m_q0_x_pa_x_ea) + 
                    (r1_m_r0 * q0_x_pa_x_ea)));
    const point_t<> a_d(pa_x_ea + (2.0f * (cross_product(q0, q0_x_pa_x_ea) + (r0 * q0_x_pa_x_ea))));
    BOOST_LOG_TRIVIAL(trace) << "Part A factors of s, s^2: " << a_b << ", s:  " << a_c << ", constant: " << a_d;

    /* Dot product with the edge of object b */
    const float a_b_dot = dot_product(a_b, eb);
    const float a_c_dot = dot_product(a_c, eb);
    const float a_d_dot = dot_product(a_d, eb);
    BOOST_LOG_TRIVIAL(trace) << "Part A equation, b: " << a_b_dot << ", c: " << a_c_dot << ", d: " << a_d_dot;

    const point_t<> q0_x_ea(cross_product(q0, ea));
    const point_t<> q1_m_q0_x_ea(cross_product(q1_m_q0, ea));
    
    /* Part B polynomial */
    const point_t<> b_b(2.0f * (cross_product(q1_m_q0, q1_m_q0_x_ea) + (r1_m_r0 * q1_m_q0_x_ea)));
    const point_t<> b_c(2.0f * (
                    cross_product(q0, q1_m_q0_x_ea) + 
                    cross_product(q1_m_q0, q0_x_ea) + 
                    (r0 * q1_m_q0_x_ea) + 
                    (r1_m_r0 * q0_x_ea)));
    const point_t<> b_d(ea + (2.0f * (cross_product(q0, q0_x_ea) + (r0 * q0_x_ea))));
    BOOST_LOG_TRIVIAL(trace) << "Part B factors of s pre-cross eb, s^2: " << b_b << ", s:  " << b_c << ", constant: " << b_d;

    const point_t<> b_b_cross(cross_product(b_b, eb));
    const point_t<> b_c_cross(cross_product(b_c, eb));
    const point_t<> b_d_cross(cross_product(b_d, eb));
    BOOST_LOG_TRIVIAL(trace) << "Part B factors of s, s^2: " << b_b_cross << ", s:  " << b_c_cross << ", constant: " << b_d_cross;

    const float b_a_dot = dot_product(b_b_cross, x1_m_x0);
    const float b_b_dot = dot_product(b_b_cross, x0_m_pb) + dot_product(b_c_cross, x1_m_x0);
    const float b_c_dot = dot_product(b_c_cross, x0_m_pb) + dot_product(b_d_cross, x1_m_x0);
    const float b_d_dot = dot_product(b_d_cross, x0_m_pb);
    BOOST_LOG_TRIVIAL(trace) << "Part B equation, a: " << b_a_dot << ", b: " << b_b_dot << ", c: " << b_c_dot << ", d: " << b_d_dot;

    /* Final factors */
    const float a_dot = b_a_dot;
    const float b_dot = a_b_dot + b_b_dot;
    const float c_dot = a_c_dot + b_c_dot;
    const float d_dot = a_d_dot + b_d_dot;
    BOOST_LOG_TRIVIAL(trace) << "Solving equation, a: " << a_dot << ", b: " << b_dot << ", c: " << c_dot << ", d: " << d_dot;

    float roots[3];
    const int nr_roots = find_first_positive_real_root(roots, a_dot, b_dot, c_dot, d_dot);

    /* Check to see if roots are just parallel lines */
    for (int i = 0; i < nr_roots; ++i)
    {
        const point_t<> cfg_ea(interpolated_quaternion_rotate(ea, q0, q1, r0, r1, roots[i]));
        const point_t<> eb_cross_ea(cross_product(eb, cfg_ea));
        if (dot_product(eb_cross_ea, eb_cross_ea) > raptor_physics::EPSILON)
        {
            return roots[i];
        }
        else if (roots[i] == std::numeric_limits<float>::infinity())
        {
            return roots[i];
        }
        BOOST_LOG_TRIVIAL(trace) << "Discarding parallel root: " << roots[i];
    }

    /* All the roots are parallel */
    return std::numeric_limits<float>::infinity();
}


/* Find the time that a translating and rotating point passes through a plane   */
/* pa is the point on the edge of object a                                      */
/* pb is the point on the edge of object b                                      */
/* ea an edge of object a                                                       */
/* eb an edge of object b                                                       */
/* x0 is the position of the point at the start of the frame                    */
/* q0 is the orientation of the point at the start of the frame                 */
/* q1 is the orientation of the point at the end of the frame                   */
float find_exact_none_translating_collision_time(const point_t<> &pa, const point_t<> &pb, const point_t<> &ea, const point_t<> &eb, const point_t<> &x0, 
    const point_t<> &q0, const point_t<> &q1, const float r0, const float r1)
{
    METHOD_LOG;

    /* Movements */
    const float r1_m_r0 = r1 - r0;
    const point_t<> q1_m_q0(q1 - q0);
    const point_t<> x0_m_pb(x0 - pb);

    /* Common cross products */
    const point_t<> pa_x_ea(cross_product(pa, ea));
    const point_t<> q0_x_pa_x_ea(cross_product(q0, pa_x_ea));
    const point_t<> q1_m_q0_x_pa_x_ea(cross_product(q1_m_q0, pa_x_ea));

    /* Part A polynomial */
    const point_t<> a_b(2.0f * (cross_product(q1_m_q0, q1_m_q0_x_pa_x_ea) + (r1_m_r0 * q1_m_q0_x_pa_x_ea)));
    const point_t<> a_c(2.0f * (
                    cross_product(q0, q1_m_q0_x_pa_x_ea) + 
                    cross_product(q1_m_q0, q0_x_pa_x_ea) + 
                    (r0 * q1_m_q0_x_pa_x_ea) + 
                    (r1_m_r0 * q0_x_pa_x_ea)));
    const point_t<> a_d(pa_x_ea + (2.0f * (cross_product(q0, q0_x_pa_x_ea) + (r0 * q0_x_pa_x_ea))));
    BOOST_LOG_TRIVIAL(trace) << "Part A factors of s, s^2: " << a_b << ", s:  " << a_c << ", constant: " << a_d;

    /* Dot product with the edge of object b */
    const float a_b_dot = dot_product(a_b, eb);
    const float a_c_dot = dot_product(a_c, eb);
    const float a_d_dot = dot_product(a_d, eb);
    BOOST_LOG_TRIVIAL(trace) << "Part A equation, b: " << a_b_dot << ", c: " << a_c_dot << ", d: " << a_d_dot;

    const point_t<> q0_x_ea(cross_product(q0, ea));
    const point_t<> q1_m_q0_x_ea(cross_product(q1_m_q0, ea));
    
    /* Part B polynomial */
    const point_t<> b_b(2.0f * (cross_product(q1_m_q0, q1_m_q0_x_ea) + (r1_m_r0 * q1_m_q0_x_ea)));
    const point_t<> b_c(2.0f * (
                    cross_product(q0, q1_m_q0_x_ea) + 
                    cross_product(q1_m_q0, q0_x_ea) + 
                    (r0 * q1_m_q0_x_ea) + 
                    (r1_m_r0 * q0_x_ea)));
    const point_t<> b_d(ea + (2.0f * (cross_product(q0, q0_x_ea) + (r0 * q0_x_ea))));
    BOOST_LOG_TRIVIAL(trace) << "Part B factors of s pre-cross eb, s^2: " << b_b << ", s:  " << b_c << ", constant: " << b_d;

    const point_t<> b_b_cross(cross_product(b_b, eb));
    const point_t<> b_c_cross(cross_product(b_c, eb));
    const point_t<> b_d_cross(cross_product(b_d, eb));
    BOOST_LOG_TRIVIAL(trace) << "Part B factors of s, s^2: " << b_b_cross << ", s:  " << b_c_cross << ", constant: " << b_d_cross;

    const float b_b_dot = dot_product(b_b_cross, x0_m_pb);
    const float b_c_dot = dot_product(b_c_cross, x0_m_pb);
    const float b_d_dot = dot_product(b_d_cross, x0_m_pb);
    BOOST_LOG_TRIVIAL(trace) << "Part B equation, b: " << b_b_dot << ", c: " << b_c_dot << ", d: " << b_d_dot;

    /* Final factors */
    const float b_dot = a_b_dot + b_b_dot;
    const float c_dot = a_c_dot + b_c_dot;
    const float d_dot = a_d_dot + b_d_dot;
    BOOST_LOG_TRIVIAL(trace) << "Solving equation, b: " << b_dot << ", c: " << c_dot << ", d: " << d_dot;

    float roots[3];
    const int nr_roots = find_first_positive_real_root(roots, 0.0f, b_dot, c_dot, d_dot);

    /* Check to see if roots are just parallel lines */
    for (int i = 0; i < nr_roots; ++i)
    {
        const point_t<> cfg_ea(interpolated_quaternion_rotate(ea, q0, q1, r0, r1, roots[i]));
        const point_t<> eb_cross_ea(cross_product(eb, cfg_ea));
        if (dot_product(eb_cross_ea, eb_cross_ea) > raptor_physics::EPSILON)
        {
            return roots[i];
        }
        else if (roots[i] == std::numeric_limits<float>::infinity())
        {
            return roots[i];
        }
        BOOST_LOG_TRIVIAL(trace) << "Discarding parallel root: " << roots[i];
    }

    /* All the roots are parallel */
    return std::numeric_limits<float>::infinity();
}
} /* namespace raptor_physics */
