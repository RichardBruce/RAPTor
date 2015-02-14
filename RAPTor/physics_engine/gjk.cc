#include "gjk.h"
#include "vertex_group.h"


namespace raptor_physics
{              
bool gjk::find_minimum_distance(point_t *const dist, const point_t &fixed_rel_disp, const point_t &float_rel_disp, const quaternion_t &a_o, const quaternion_t &b_o)
{
    METHOD_LOG;
    BOOST_LOG_TRIVIAL(trace) << "Fixed relative displacement: " << fixed_rel_disp;
    BOOST_LOG_TRIVIAL(trace) << "Floating relative displacement: " << float_rel_disp;
    
    /* Until converged */
    point_t dir;
    point_t c_space[4];
    int min_verts[4];
    while (true)
    {
        /* Sample c space at the vertices of the current simplices */
        _a_simplex->compute_c_space(*_b_simplex, c_space, fixed_rel_disp, float_rel_disp);
        
        /* Search c space for the closest point to the origin */
        /* Return the closest feature and a line to it from the origin */
        const int min_simplex = find_closest_feature_to_origin(c_space, min_verts, &dir);
        BOOST_LOG_TRIVIAL(trace) << "Minimum simplex size: " << min_simplex;
        BOOST_LOG_TRIVIAL(trace) << "Direction: " << dir;
        
        /* The origin is contained within the simplices */
        if ((min_simplex == 4) || (magnitude(dir) < raptor_physics::EPSILON))
        {
            (*dist) = point_t(0.0f, 0.0f, 0.0f);
            BOOST_LOG_TRIVIAL(trace) << "Hit";
            return true;
        }
        
        /* Update the simplices */
        const int a_sup = _a.find_support_vertex(a_o.rotate(-dir), a_o.rotate(float_rel_disp));
        const int b_sup = _b.find_support_vertex(b_o.rotate( dir));
        BOOST_LOG_TRIVIAL(trace) << "Support vertex search found: " << a_sup << " " << b_sup;

        const bool improve = _a_simplex->is_new_pair(*_b_simplex, a_sup, b_sup);
        _a_simplex->retain_vertices(min_verts, min_simplex);
        _b_simplex->retain_vertices(min_verts, min_simplex);
        if (!improve)
        {
            (*dist) = dir;
            BOOST_LOG_TRIVIAL(trace) << "No hit, distance: " << (*dist);
            return false;
        }
        
        _a_simplex->add(a_sup);
        _b_simplex->add(b_sup);
    }
}

int gjk::find_closest_feature_to_origin(const point_t *const c_space, int *const verts, point_t *const dir) const
{
    METHOD_LOG;
    
    /* Find the closest point to the origin */
    unsigned int size = _a_simplex->size();
    if (size == 1)
    {
        /* Simple case, this must be the closest point */
        verts[0] = 0;
        (*dir) = c_space[0];
        return 1;
    }
        
    /* Build edges of the simplex */
    /* Output is: [ 0,       Q1 - Q2, Q1 - Q3, Q1 - Q4] */
    /*            [ Q2 - Q1, 0,       Q2 - Q3, Q2 - Q4] */
    /*            [ Q3 - Q1, Q3 - Q2, 0        Q3 - Q4] */
    /*            [ Q4 - Q1, Q4 - Q2, Q4 - Q3, 0      ] */
    point_t diffs[16];
    for (unsigned int i = 0; i < size; ++i)
    {
        for (unsigned int j = i + 1; j < size; ++j)
        {
            diffs[(i << 2) + j] = c_space[i] - c_space[j];
            diffs[(j << 2) + i] = -diffs[(i << 2) + j];
        }
    }
        
    /* Take dot product of points and edges */
    /* Output is: [0,               Q1 . (Q1 - Q2), Q1 . (Q1 - Q3), Q1 . (Q1 - Q4)] */
    /*            [ Q2 . (Q2 - Q1), 0             , Q2 . (Q2 - Q3), Q2 . (Q2 - Q4)] */
    /*            [ Q3 . (Q3 - Q1), Q3 . (Q3 - Q2), 0,              Q3 . (Q3 - Q4)] */
    /*            [ Q4 . (Q4 - Q1), Q4 . (Q4 - Q2), Q4 . (Q4 - Q3), 0             ] */
    float dot_prods[16];
    for (unsigned int i = 0; i < size; ++i)
    {
        for (unsigned int j = 0; j < size; ++j)
        {
            if (i != j)
            {
                dot_prods[(i << 2) + j] = dot_product(c_space[i], diffs[(i << 2) + j]);
            }
        }
    }
    
    /* Check vertices */
    for (unsigned int i = 0; i < size; ++i)
    {
        bool this_vert = true;
        for (unsigned int j = 0; j < size; ++j)
        {
            this_vert &= ((i == j) || (dot_prods[(i << 2) + j] <= 0.0f));
        }

        /* Return this vert as the closest */
        if (this_vert)
        {
            verts[0] = i;
            (*dir) = c_space[i];
            return 1;
        }
    }
    
    /* Its not a vertex and there's only 2 vertices so it must be an edge */
    if (size == 2)
    {
        verts[0] = 0;
        verts[1] = 1;
        const float magn_diff = dot_product(diffs[1], diffs[1]);
        if (magn_diff > raptor_physics::EPSILON)
        {
            const float dot_prod = dot_product(c_space[0], diffs[1]);
            (*dir) = c_space[0] - ((diffs[1] * dot_prod) / magn_diff);
        }
        else
        {
            (*dir) = c_space[0];
        }
                    
        return 2;
    }
        
    /* Build face normals */
    point_t norms[4];
    cross_product(diffs[ 4], diffs[8], &norms[0]); /* n123 = (Q2 - Q1) x (Q3 - Q1) */
    if (size == 3)
    {
        norms[1] = -norms[0];
        norms[2] =  norms[1];
        norms[3] =  norms[1];
    }
    else
    {
        cross_product(diffs[12], diffs[4], &norms[1]); /* n142 = (Q4 - Q1) x (Q2 - Q1) */
        cross_product(diffs[14], diffs[2], &norms[2]); /* n341 = (Q4 - Q3) x (Q1 - Q3) */
        cross_product(diffs[13], diffs[9], &norms[3]); /* n243 = (Q4 - Q2) x (Q3 - Q2) */
    }

    /* Check edges */
    /* Q1 . (Q1 - Q2) >= 0.0 */
    /* Q2 . (Q2 - Q1) >= 0.0 */
    /* Q1 . ((Q1 - Q2) x n123) >= 0.0 */
    /* Q1 . (n142 x (Q1 - Q2)) >= 0.0 */
    /* Q1 to Q3 */
    if (test_edge(norms[2], norms[0], dot_prods[2], dot_prods[ 8], diffs[2], c_space[0], 0, 2, verts, dir))
    {
        return 2;
    }

    /* Q2 to Q3 */
    if (test_edge(norms[0], norms[3], dot_prods[6], dot_prods[ 9], diffs[6], c_space[1], 1, 2, verts, dir))
    {
        return 2;
    }

    /* Q1 to Q3 and Q4 */
    if (test_edge(norms[0], norms[1], dot_prods[1], dot_prods[ 4], diffs[1], c_space[0], 0, 1, verts, dir))
    {
        return 2;
    }
    
    if ((size != 3) && test_edge(norms[1], norms[2], dot_prods[3], dot_prods[12], diffs[3], c_space[0], 0,  3, verts, dir))
    {
        return 2;
    }

    /* Q2 to Q4 */
    if ((size != 3) && test_edge(norms[3], norms[1], dot_prods[7], dot_prods[13], diffs[7], c_space[1], 1, 3, verts, dir))
    {
        return 2;
    }

    /* Q3 to Q4 */
    if ((size != 3) && test_edge(norms[2], norms[3], dot_prods[11], dot_prods[14], diffs[11], c_space[2], 2, 3, verts, dir))
    {
        return 2;
    }

    /* Its not a vertex or an edge and there's only 3 vertices so it must be a face */
    if (size == 3)
    {
        const float a = dot_product(norms[0], c_space[0]);
        verts[1] = 1;
        if (a < 0.0f)
        {
            verts[0] = 2;
            verts[2] = 0;
        }
        else
        {
            verts[0] = 0;
            verts[2] = 2;
        }

        const float b = dot_product(norms[0], norms[0]);
        (*dir) = norms[0] * (a / b);
        return 3;
    }

    /* Check faces */
    /* The first 6 checks make sure the origin is within the face */
    /* This is normally covered by the edge checks unless you have co-planar */
    /* or close to co-planar faces */
    /* Return faces in reverse order to reverse the normal and ensure integrity */
    if ((dot_product(c_space[1], norms[1]) < 0.0f) &&
        in_triangle(norms[1], diffs[1], diffs[13], c_space[1], dir, verts, 1, 3, 0))
    {
        return 3;
    }

    if ((dot_product(c_space[2], norms[2]) < 0.0) &&
        in_triangle(norms[2], diffs[14], diffs[2], c_space[2], dir, verts, 0, 3, 2))
    {
        return 3;
    }

    if ((dot_product(c_space[3], norms[3]) < 0.0f) &&
        in_triangle(norms[3], diffs[7], diffs[11], c_space[3], dir, verts, 2, 3, 1))
    {
        return 3;
    }

    if ((dot_product(c_space[0], norms[0]) < 0.0f) &&
        in_triangle(norms[0], diffs[4], diffs[8], c_space[0], dir, verts, 2, 1, 0))
    {
        return 3;
    }

    /* Size must be 4 (ie, simplex is a tetrahedron) and the origin is inside it */
    verts[0] = 0;
    verts[1] = 1;
    verts[2] = 2;
    verts[3] = 3;
    (*dir) = point_t(0.0f, 0.0f, 0.0f);
    return 4;
}


bool gjk::in_triangle(const point_t &n, const point_t &d0, const point_t &d1, const point_t &c, 
    point_t *d, int *verts, const int v0, const int v1, const int v2) const
{
    /* Get intersecting point of line and triangles plane */
    const float a = dot_product(n, c);
    const float b = dot_product(n, n);

    const float r = a / b;
    const point_t dir(n * r);
    const point_t w(dir - c);

    /* Calculate and test barycentric coordinates */
    const float uu = dot_product(d0, d0);
    const float uv = dot_product(d0, d1);
    const float vv = dot_product(d1, d1);
    
    const float wu = dot_product(w, d0);
    const float wv = dot_product(w, d1);
    const float no = uv * uv - uu * vv;

    const float s = ((uv * wv) - (vv * wu)) / no;
    if (s < -raptor_physics::EPSILON || s > 1.0f)
    {
        return false;
    }

    const float t = ((uv * wu) - (uu * wv)) / no;
    if (t < -raptor_physics::EPSILON || (s + t) > 1.0f)
    {
        return false;
    }

    /* Set output and return hit */
    verts[0] = v0;
    verts[1] = v1;
    verts[2] = v2;

    (*d) = dir;

    return true;
}
}; /* namespace raptor_physics */
