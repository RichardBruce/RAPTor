/* Standard headers */
#include <vector>
#include <memory>

/* Common headers */
#include "logging.h"
#include "physics_common.h"

/* Physics headers */
#include "simplex.h"


namespace raptor_physics
{
point_t<> simplex::normal_of_impact(const simplex &s) const
{
    // METHOD_LOG;
    // BOOST_LOG_TRIVIAL(trace) << "Finding normal of impact with simplex size: " << _size;

    /* Simplex must alway be the same size */
    assert(_size == s._size);

    /* Get unique points defining the simplex */
    point_t<> simplex_verts[6];
    const int a_size = get_unique_points(&simplex_verts[0]);
    const int b_size = s.get_unique_points(&simplex_verts[3]);
    // BOOST_LOG_TRIVIAL(trace) << "Unique points: " << a_size << " and: " << b_size;

    /* Can only deal with planes, lines of vertices not tetrahedron */
    /* This shouldn't fail since objects are only brought close to each other */
    /* and not allowed to penetrate */
    assert(a_size < 4);
    assert(b_size < 4);

    /* This just shouldn't happen */
    assert(a_size > 0);
    assert(b_size > 0);

    /* Get the normal of the collision */
    point_t<> *l_points;
    point_t<> *s_points;
    if (a_size >= b_size)
    {
        l_points = &simplex_verts[0];
        s_points = &simplex_verts[3];
    }
    else
    {
        s_points = &simplex_verts[0];
        l_points = &simplex_verts[3];
    }

    point_t<> norm;
    switch (std::max(a_size, b_size))
    {
        /* Line between 2 points */
        case 1 :
            _po.vertex_to_global(&l_points[0]);
            s._po.vertex_to_global(&s_points[0]);
            norm = normalise(l_points[0] - s_points[0]);
            break;

        /* Find closest point on then line to a point, then the perpendicular line */
        case 2 :
        {
            if (a_size >= b_size)
            {
                _po.vertex_to_global(&l_points[0]);
                _po.vertex_to_global(&l_points[1]);
                s._po.vertex_to_global(&s_points[0]);
                s._po.vertex_to_global(&s_points[1]);
            }
            else
            {
                _po.vertex_to_global(&s_points[0]);
                _po.vertex_to_global(&s_points[1]);
                s._po.vertex_to_global(&l_points[0]);
                s._po.vertex_to_global(&l_points[1]);
            }

            const point_t<> l_dir(normalise(l_points[1] - l_points[0]));
            const point_t<> s_dir(normalise(s_points[1] - s_points[0]));

            /* If there is only a point or the lines are parallel */
            if ((std::min(a_size, b_size) == 1) || (fabs(dot_product(l_dir, s_dir)) > (1.0f - raptor_physics::EPSILON)))
            {
                // BOOST_LOG_TRIVIAL(trace) << "Finding normal for point and line";
                // BOOST_LOG_TRIVIAL(trace) << "L points: " << l_points[0] << " and " << l_points[1];
                // BOOST_LOG_TRIVIAL(trace) << "S points: " << s_points[0] << " and " << s_points[1];

                /* Find the point on l0 -> l1 most opposite s0 */
                const point_t<> perp((l_points[0] - (l_dir * dot_product(l_dir, l_points[0] - s_points[0]))) - s_points[0]);

                /* Check if the nearest point to s is s */
                const float perp_dist = magnitude(perp);
                assert((fabs(perp_dist) > raptor_physics::EPSILON) || !"Error: Distance between objects is 0");
                norm = perp / perp_dist;
            }
            /* If there are 2 non parallel lines */
            else
            {
                norm = cross_product(l_dir, s_dir);
                if (dot_product(norm, l_points[0] - s_points[0]) < 0.0f)
                {
                    norm = -norm;
                }
                normalise(&norm);
            }
        }
        break;

        /* Cross product of 2 lines in the face */
        case 3 :
            cross_product(l_points[1] - l_points[0], l_points[2] - l_points[0], &norm);
            normalise(&norm);

            if (a_size >= b_size)
            {
                // BOOST_LOG_TRIVIAL(trace) << "Rotating: " << norm << " about: " << _po.get_orientation();
                (-_po.get_orientation()).rotate(&norm);
                _po.vertex_to_global(&l_points[0]);
                s._po.vertex_to_global(&s_points[0]);
            }
            else
            {
                BOOST_LOG_TRIVIAL(trace) << "Rotating: " << norm << " about: " << s._po.get_orientation();
                (-s._po.get_orientation()).rotate(&norm);
                _po.vertex_to_global(&s_points[0]);
                s._po.vertex_to_global(&l_points[0]);
            }
            // BOOST_LOG_TRIVIAL(trace) << "Oritentated to: " << norm;

            if (dot_product(norm, l_points[0] - s_points[0]) < 0.0f)
            {
                // BOOST_LOG_TRIVIAL(trace) << "Flipped based on :" << (l_points[0] - s_points[0]) << ", dot is: " << dot_product(norm, l_points[0] - s_points[0]);
                norm = -norm;
            }
            break;
    }

    /* Maintain the relative direction of the normal */
    if (a_size < b_size)
    {
        // BOOST_LOG_TRIVIAL(trace) << "Flipped because of simplex size: " << norm;
        norm = -norm;
    }

    // BOOST_LOG_TRIVIAL(trace) << "Normal of impact: " << norm;
    return norm;
}

point_t<> simplex::center_of_impact(const simplex &s, const point_t<> &noc)
{
    // METHOD_LOG;

    /* Simplex must alway be the same size */
    assert(_size == s._size);

    const simplex *const min_s =  _forward ? this : &s;
    const simplex *const max_s = !_forward ? this : &s;
    
    const point_t<> min_noc((min_s == this) ? noc : -noc);
    const point_t<> max_noc(-min_noc);

    /* Find the polygon the simplices are on */
    int pts[4];
    for (int i = 0; i < static_cast<int>(_size); ++i)
    {
        pts[i] = min_s->_verts_rd[i] & 0x7fffffff;
    }
    const quaternion_t t_inv_o(-min_s->_po.get_orientation());
    const auto *const p0 = min_s->_po.get_vertex_group()->find_polygon(&pts[0], &pts[_size], t_inv_o.rotate(max_noc));
    assert((p0 != nullptr) || !"Error: Unable to find polygon for simplex");

    for (int i = 0; i < static_cast<int>(_size); ++i)
    {
        pts[i] = max_s->_verts_rd[i] & 0x7fffffff;
    }
    const quaternion_t s_inv_o(-max_s->_po.get_orientation());
    const auto *const p1 = max_s->_po.get_vertex_group()->find_polygon(&pts[0], &pts[_size], s_inv_o.rotate(min_noc));
    assert((p1 != nullptr) || !"Error: Unable to find polygon for simplex");

    /* Find the polygon vertices in contact */
    auto wp0(p0->to_world_polygon(min_s->_po.get_orientation(), min_s->_po.get_center_of_mass()));
    wp0.points_above_plane(min_s->_po.get_global_vertex(min_s->_verts_rd[0] & 0x7fffffff), max_noc);
    assert((wp0.number_of_vertices() > 0) || !"Error: No polygon points in plane of collision");

    auto wp1(p1->to_world_polygon(max_s->_po.get_orientation(), max_s->_po.get_center_of_mass()));
    wp1.points_above_plane(max_s->_po.get_global_vertex(max_s->_verts_rd[0] & 0x7fffffff), min_noc);
    assert((wp1.number_of_vertices() > 0) || !"Error: No polygon points in plane of collision");

    /* Clip the contact polygons to eachother */
    wp0.intersection(wp1, min_noc, max_s->_dir);

    /* Not many vertices, use the all */
    _cm_size = std::min(4, wp0.number_of_vertices());
    if (wp0.number_of_vertices() < 5)
    {
        _cm[0] = wp0.vertex(0);
        _cm[1] = wp0.vertex(1);
        _cm[2] = wp0.vertex(2);
        _cm[3] = wp0.vertex(3);
    }
    /* Find extreme vertices for the manifold */
    else
    {
        const point_t<> perp_norm(perpendicular(max_noc));
        const point_t<> norm_norm(cross_product(max_noc, perp_norm));
        _cm[0] = wp0.extreme_vertices(&_cm[1], perp_norm);
        _cm[2] = wp0.extreme_vertices(&_cm[3], norm_norm);
    }

    const point_t<> half_dir(max_s->_dir * 0.5f);
    _cm[0] += half_dir;
    _cm[1] += half_dir;
    _cm[2] += half_dir;
    _cm[3] += half_dir;
    BOOST_LOG_TRIVIAL(trace) << "dir: " << min_s->_dir << ", s._dir: " << max_s->_dir << ", contact manifold: ";
    for (int i = 0; i < _cm_size; ++i)
    {
        BOOST_LOG_TRIVIAL(trace) <<_cm[i];
    }

    /* Return the center of the polygons */
    return wp0.center() + half_dir;
}

int simplex::get_unique_points(point_t<> *const points) const
{
    /* The are only 4 verts (and the last should alway be a duplicate) */
    /* So accept O(n^2) complexity */
    int size = 0;
    for (unsigned int i = _size - 1; i < _size; --i)
    {
        /* Check if the vert is unique */
        bool unique = true;
        for (unsigned int j = i - 1; j < _size; --j)
        {
            unique &= ((_verts_rd[i] & 0x7fffffff) != (_verts_rd[j] & 0x7fffffff));
        }

        /* If so include it in the result */
        if (unique)
        {
            points[size++] = get_vertex(i);
        }
    }

    return size;
}

std::vector<point_t<>>* simplex::get_points_in_contact_plane(const point_t<> &noc) const
{
    /* Get all verts on the colliding feature */
    std::vector<point_t<>> *verts = new std::vector<point_t<>>();

    /* Find all verts perpendicular to noc in a */
    point_t<> noc_l(noc);
    const quaternion_t inv_o(-_po.get_orientation());
    inv_o.rotate(&noc_l);
    // BOOST_LOG_TRIVIAL(trace) << "Local normal of collision: " << noc_l << ", rotated: " << noc << ", by: " << inv_o;
    const point_t<> a(get_vertex(0));
    for (int i = 0; i < _po.get_vertex_group()->get_number_of_vertices(); ++i)
    {
        /* Check for vert is a */
        const point_t<> vert(_po.get_vertex_group()->get_vertex(i));
        const point_t<> diff(vert - a);
        // BOOST_LOG_TRIVIAL(trace) << "Testing: " << vert << " for inclusion against: " << a;
        if (std::fabs(dot_product(diff, noc_l)) < (5.0f * raptor_physics::EPSILON))
        {
            // BOOST_LOG_TRIVIAL(trace) << "Added";
            verts->push_back(vert);
        }
    }

    /* Check that some verts were found */
    assert(!verts->empty());

    return verts;
}
}; /* namespace raptor_physics */
