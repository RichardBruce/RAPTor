/* Common headers */
#include "logging.h"

/* Physics headers */
#include "polygon.h"


namespace raptor_physics
{
float world_polygon::intersection(const point_t &a, const point_t &ma, const point_t &b, const point_t &mb) const
{
    const point_t r(a - b);
    const float ma_dot_ma = dot_product(ma, ma);
    const float ma_dot_mb = dot_product(ma, mb);
    const float mb_dot_mb = dot_product(mb, mb);

    /* Check for parallel lines */
    const float d = (ma_dot_ma * mb_dot_mb) - (ma_dot_mb * ma_dot_mb);
    if (fabs(d) < raptor_physics::EPSILON)
    {
        // BOOST_LOG_TRIVIAL(trace) << "Parallel lines found";
        return 2.0f;
    }

    /* Check in line segment */
    const float ma_dot_r = dot_product(ma, r);
    const float mb_dot_r = dot_product(mb, r);
    const float s = ((ma_dot_mb * mb_dot_r) - (mb_dot_mb * ma_dot_r)) / d;
    if ((s < -raptor_physics::EPSILON) || (s > (1.0f + raptor_physics::EPSILON)))
    {
        // BOOST_LOG_TRIVIAL(trace) << "S out of range";
        return -1.0f;
    }

    const float t = ((ma_dot_ma * mb_dot_r) - (ma_dot_mb * ma_dot_r)) / d;
    return t;
}

void world_polygon::intersection(const world_polygon &p, const point_t &n, const point_t &dir)
{
    // BOOST_LOG_TRIVIAL(trace) << "Clipping: ";
    // BOOST_LOG_TRIVIAL(trace) << array_to_stream(_verts.begin(), _verts.end(), [](std::stringstream *ss, const point_t &p)
    // {
    //     (*ss) << "( " << p << " ) ";
    //     return ss;
    // });
    // BOOST_LOG_TRIVIAL(trace) << "Clipping to: ";
    // BOOST_LOG_TRIVIAL(trace) << array_to_stream(p._verts.begin(), p._verts.end(), [](std::stringstream *ss, const point_t &p)
    // {
    //     (*ss) << "( " << p << " ) ";
    //     return ss;
    // });

    /* Technically we should test for inclusion of this point in the other polygon      */
    /* But this isnt needed in this case because the polygon is picked by a gjk simplex */
    /* Therefore the point must be included                                             */
    if (_verts.size() == 1)
    {
        return;
    }

    /* Again this is only valid because this polygon is picked by a gjk simplex         */
    if (p._verts.size() == 1)
    {
        _verts.resize(1);
        const float norm_dist = -dot_product(p._verts[0] - _verts[0], n) / dot_product(n, n);
        _verts[0] = p._verts[0] + (norm_dist * n);
        return;
    }

    /* Special case of line versus line */
    if ((_verts.size() == 2) && (p._verts.size() == 2))
    {
        const point_t ma(p._verts[1] - p._verts[0]);
        const point_t mb(_verts[1] - _verts[0]);
        const float dist = intersection(p._verts[0], ma, _verts[0], mb);

        /* Special case parallel lines */
        if (dist == 2.0f)
        {
            point_t output[2];
            int output_idx = 0;
            const float ma_sq = dot_product(ma, ma);
            const float mb_sq = dot_product(mb, mb);
            const float ba_dist = dot_product(p._verts[0] - _verts[0], mb);
            if ((ba_dist >= 0.0f) && (ba_dist <= mb_sq))
            {
                // BOOST_LOG_TRIVIAL(trace) << "Parallel case ba_dist got: " << p._verts[0] << ", dist: " << ba_dist << ", versus: " << ma_sq;
                output[output_idx++] = p._verts[0] - dir;
            }

            const float bb_dist = dot_product(p._verts[1] - _verts[0], mb);
            if ((bb_dist >= 0.0f) && (bb_dist <= mb_sq))
            {
                // BOOST_LOG_TRIVIAL(trace) << "Parallel case bb_dist got: " << p._verts[1] << ", dist: " << bb_dist << ", versus: " << ma_sq;
                output[output_idx++] = p._verts[1] - dir;
                if (output_idx == 2)
                {
                    _verts[0] = output[0];
                    _verts[1] = output[1];
                    return;
                }
            }

            const float aa_dist = dot_product(_verts[0] - p._verts[0], ma);
            if ((aa_dist >= 0.0f) && (aa_dist <= ma_sq))
            {
                // BOOST_LOG_TRIVIAL(trace) << "Parallel case aa_dist got: " << _verts[0] << ", dist: " << aa_dist << ", versus: " << mb_sq;
                output[output_idx++] = _verts[0];
                if (output_idx == 2)
                {
                    _verts[0] = output[0];
                    _verts[1] = output[1];
                    return;
                }
            }

            const float ab_dist = dot_product(_verts[1] - p._verts[0], ma);
            if ((ab_dist >= 0.0f) && (ab_dist <= ma_sq))
            {
                // BOOST_LOG_TRIVIAL(trace) << "Parallel case ab_dist got: " << _verts[1] << ", dist: " << ab_dist << ", versus: " << mb_sq;
                output[output_idx++] = _verts[1];
                if (output_idx == 2)
                {
                    _verts[0] = output[0];
                    _verts[1] = output[1];
                    return;
                }
            }

            /* It might be possible that the lines meet at their end points */
            /* gjk should prevent this, but lets wait and see               */
            assert(!"Error: Clipped parallel lines must have 2 points inside eachother");
        }

        // BOOST_LOG_TRIVIAL(trace) << "Line distance: " << dist;
        assert(((dist > -raptor_physics::EPSILON) && (dist < (1.0f + raptor_physics::EPSILON))) || !"Error: Clipped lines must intersect");
        _verts.resize(1);
        // BOOST_LOG_TRIVIAL(trace) << "a: " << _verts[0] << ", mb: " << mb;
        _verts[0] += (mb * dist);
        return;
    }
    
    /* Special case of line versus polygon */
    const std::vector<point_t> &min_verts = (_verts.size() <= p._verts.size()) ? _verts : p._verts;
    const std::vector<point_t> &max_verts = (_verts.size() >  p._verts.size()) ? _verts : p._verts;
    if ((max_verts.size() > 2) && (min_verts.size() == 2))
    {
        /* Get the polygon normal */
        const auto fn(cross_product(max_verts[0] - max_verts[1], max_verts[2] - max_verts[1]));
        
        /* See if the vertices are inside all edges */
        bool inside_0 = true;
        bool inside_1 = true;
        point_t a(max_verts.back());
        for (auto i = max_verts.cbegin(); i != max_verts.cend(); ++i)
        {
            const point_t ab(*i - a);
            inside_0 &= same_side(min_verts[0], a, ab, fn);
            inside_1 &= same_side(min_verts[1], a, ab, fn);
            a = *i;
        }

        /* Line is entirely inside the polygon, we are done */
        const point_t min_adj((&_verts == &min_verts) ? point_t(0.0f, 0.0f, 0.0f) : dir);
        if (inside_0 && inside_1)
        {
            // BOOST_LOG_TRIVIAL(trace) << "Line inside polygon";
            _verts.resize(2);
            _verts[0] = min_verts[0] - min_adj;
            _verts[1] = min_verts[1] - min_adj;
            return;
        }

        /* Intersect line segment versus the polygon edges */
        const point_t b(min_verts[0]);
        const point_t mb(min_verts[1] - min_verts[0]);

        point_t output[2];
        int output_idx = 0;
        if (inside_0)
        {
            output[output_idx++] = min_verts[0];
        }
        else if (inside_1)
        {
            output[output_idx++] = min_verts[1];
        }

        a = max_verts.back();
        for (auto i = max_verts.cbegin(); i != max_verts.cend(); ++i)
        {
            const point_t ma(*i - a);
            const float dist = intersection(a, ma, b, mb);
            if ((dist > -raptor_physics::EPSILON) && (dist < (1.0f + raptor_physics::EPSILON)))
            {
                // BOOST_LOG_TRIVIAL(trace) << "b: " << b << ", mb: " << mb << ", dist: " << dist;
                output[output_idx++] = b + (mb * dist);
                if (output_idx == 2)
                {
                    break;
                }
            }

            a = *i;
        }

        _verts.resize(2);
        _verts[0] = output[0] - min_adj;
        _verts[1] = output[1] - min_adj;
        // BOOST_LOG_TRIVIAL(trace) << "Line clipped at: " << _verts[0] << " to: " << _verts[1];
        assert((output_idx == 2) || !"Error: Clipped line and polygon must give 2 points");
        return;
    }


    /* Resize to act as ping pong buffer */
    int ping_size = _verts.size();
    const int buf_size = _verts.size() + p._verts.size();
    _verts.resize(buf_size << 1);
    point_t *clip_ping = &_verts[0];
    point_t *clip_pong = &_verts[buf_size];
    // BOOST_LOG_TRIVIAL(trace) << "Set up ping pong buffer of size: " << buf_size;

    /* For each edge of the the polygon being clipped to */
    auto prev_i = --p._verts.crend();
    for (auto i = p._verts.crbegin(); i != p._verts.crend(); ++i)
    {
        /* Build the plane normal, points inwards for a clockwise polygon */
        // BOOST_LOG_TRIVIAL(trace) << "Testing edge: " << (*prev_i) << " to: " << (*i);
        const point_t edge((*i) - (*prev_i));
        const point_t norm(cross_product(edge, n));

        int pong_size = 0;
        point_t prev_clip(clip_ping[ping_size - 1]);
        bool prev_inside = (is_inside_edge(prev_clip, *i, norm) > 0.0f);
        for (int j = 0; j < ping_size; ++j)
        {
            const point_t cur_clip(clip_ping[j]);
            // BOOST_LOG_TRIVIAL(trace) << "Testing point: " << cur_clip;

            const float num     = is_inside_edge(cur_clip, *i, norm);
            const bool inside   = num > 0.0f;
            if (prev_inside & inside)
            {
                clip_pong[pong_size++] = cur_clip;
                // BOOST_LOG_TRIVIAL(trace) << "Remaing inside clip: " << clip_pong[pong_size - 1] << " at: " << (pong_size - 1);
            }
            else if ((!prev_inside) & inside)
            {
                const point_t clip_edge(cur_clip - prev_clip);
                const point_t intersect(cur_clip - (clip_edge * (num / dot_product(norm, clip_edge))));
                clip_pong[pong_size++] = intersect;
                // BOOST_LOG_TRIVIAL(trace) << "Outside moving in, adding: " << clip_pong[pong_size - 1] << " at: " << (pong_size - 1);
             
                clip_pong[pong_size++] = cur_clip;
                // BOOST_LOG_TRIVIAL(trace) << "Outside moving in, adding: " << clip_pong[pong_size - 1] << " at: " << (pong_size - 1);
            }
            else if (prev_inside & !inside)
            {
                const point_t clip_edge(cur_clip - prev_clip);
                const point_t intersect(cur_clip - (clip_edge * (num / dot_product(norm, clip_edge))));
                clip_pong[pong_size++] = intersect;
                // BOOST_LOG_TRIVIAL(trace) << "Inside moving out: " << clip_pong[pong_size - 1] << " at: " << (pong_size - 1);
            }
        
            prev_clip = cur_clip;
            prev_inside = inside;
        }

        /* Log the new polygon */
        // BOOST_LOG_TRIVIAL(trace) << "New clipped polygon: ";
        // BOOST_LOG_TRIVIAL(trace) << array_to_stream(clip_pong, clip_pong + pong_size, [](std::stringstream *ss, const point_t &p)
        // {
        //     (*ss) << "( " << p << " ) ";
        //     return ss;
        // });

        /* Update for next loop */
        prev_i = i;
        ping_size = pong_size;
        pong_size = 0;
        std::swap(clip_ping, clip_pong);
    }

    /* Put the final object back at the start of clip and resize */
    /* TODO -- It might be better if the co-incidence tests were done in the outer loop above or when adding point */
    /*         I would need a bigger data set to test that on that so leaving for now */
    int clip_idx = 0;
    for (int i = 1; i < ping_size; ++i)
    {
        // BOOST_LOG_TRIVIAL(trace) << "Checking for co-incidence: " << clip_ping[i - 1] << " and " << clip_ping[i];
        if (!is_coincident(clip_ping[i], clip_ping[i - 1]))
        {
            // BOOST_LOG_TRIVIAL(trace) << "Not co-incident, adding: " << clip_ping[i - 1];
            _verts[clip_idx++] = clip_ping[i - 1];
        }
    }

    /* Check the last vs. the first point */
    // BOOST_LOG_TRIVIAL(trace) << "Checking for co-incidence: " << clip_ping[0] << " and " << clip_ping[ping_size - 1];
    if (!is_coincident(clip_ping[0], clip_ping[ping_size - 1]))
    {
        // BOOST_LOG_TRIVIAL(trace) << "Not co-incident, adding: " << clip_ping[ping_size - 1];
        _verts[clip_idx++] = clip_ping[ping_size - 1];
    }
    _verts.resize(clip_idx);

    /* Log the final polygon */
    // BOOST_LOG_TRIVIAL(trace) << "Clipped polygon: ";
    // BOOST_LOG_TRIVIAL(trace) << array_to_stream(_verts.begin(), _verts.end(), [](std::stringstream *ss, const point_t &p)
    // {
    //     (*ss) << "( " << p << " ) ";
    //     return ss;
    // });
}

void polygon::to_triangles(std::vector<int> *const tris) const
{
    /* Progress tracking */
    unsigned size = _verts.size();
    std::unique_ptr<char []> invalid(new char [_verts.size()]);
    memset(invalid.get(), 0, sizeof(char) * _verts.size());

    /* Find the furthest point from the center */
    const point_t com(centre());
    unsigned max = find_furthest(com);

    /* Use the points either side of this point to find the surface normal */
    unsigned max_m1, max_p1;
    if (max == 0)
    {
        max_m1 = _verts.size() - 1;
    }
    else
    {
        max_m1 = max - 1;
    }

    if (max == _verts.size() -1)
    {
        max_p1 = 0;
    }
    else
    {
        max_p1 = max + 1;
    }
        
    point_t face_normal;
    const point_t ab((*_pts)[_verts[max_m1]] - (*_pts)[_verts[max]]);
    const point_t bc((*_pts)[_verts[max]] - (*_pts)[_verts[max_p1]]);
    
    cross_product(ab, bc, &face_normal);
    normalise(&face_normal);
    
    unsigned no_success = 0;
    while ((_verts.size() - 2) > 0)
    {
        /* Assume the guess will be a valid triangle */
        bool success = true;
        
        /* Take the first 3 points of the face to try and form a triangle */
        const int vmax_m1   = _verts[max_m1];
        const int vmax      = _verts[max   ];
        const int vmax_p1   = _verts[max_p1];
        const point_t a((*_pts)[vmax_m1]);
        const point_t b((*_pts)[vmax   ]);
        const point_t c((*_pts)[vmax_p1]);
        
        /* If the points form a straight line the triangle is invalid */ 
        if (!is_straight_line(a, b, c))
        {
            /* If the triangle has a different normal to the face it is invalid */
            const point_t ab(a - b);
            const point_t bc(b - c);
    
            point_t tri_normal;
            cross_product(ab, bc, &tri_normal);
            normalise(&tri_normal);

            /* If any of the remaining points fall in the triangle it is illegal */ 
            if ((dot_product(tri_normal, face_normal) > 0.0f) && !is_in_triangle(invalid, a, b, c, face_normal) && !intersects_polygon(invalid, a, b, c))
            {
                /* Declare if the triangle is valid */
                tris->push_back(vmax_m1);
                tris->push_back(vmax   );
                tris->push_back(vmax_p1);
            }
            else
            {
                success = false;
            }
        }

        /* If the triangle was valid or a straight line */
        /* remove the middle point and move to the next set of points */
        if (success)
        {
            --size; 
            no_success = 0;
            invalid[max] = true;
        }
        else
        {
            max_m1 = max;
            ++no_success;
        }

        max = max_p1;
        do
        {
            ++max_p1;
            if (max_p1 == _verts.size())
            {
                max_p1 = 0;
            }
        } while (invalid[max_p1] == true);
        
        if ((size == 2) || (no_success == size))
        {
            break;
        }

        assert(no_success < _verts.size());
    }
}
} /* namespace raptor_physics */
