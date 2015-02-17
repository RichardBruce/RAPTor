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
point_t simplex::normal_of_impact(const simplex &s) const
{
    METHOD_LOG;
    BOOST_LOG_TRIVIAL(trace) << "Finding normal of impact with simplex size: " << _size;

    /* Simplex must alway be the same size */
    assert(_size == s._size);

    /* Get unique points defining the simplex */
    point_t simplex_verts[6];
    const int a_size = get_unique_points(&simplex_verts[0]);
    const int b_size = s.get_unique_points(&simplex_verts[3]);
    BOOST_LOG_TRIVIAL(trace) << "Unique points: " << a_size << " and: " << b_size;

    /* Can only deal with planes, lines of vertices not tetrahedron */
    /* This shouldn't fail since objects are only brought close to each other */
    /* and not allowed to penetrate */
    assert(a_size < 4);
    assert(b_size < 4);

    /* This just shouldn't happen */
    assert(a_size > 0);
    assert(b_size > 0);

    /* Get the normal of the collision */
    point_t *l_points;
    point_t *s_points;
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

    point_t norm;
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

            const point_t l_dir(normalise(l_points[1] - l_points[0]));
            const point_t s_dir(normalise(s_points[1] - s_points[0]));

            /* If there is only a point or the lines are parallel */
            if ((std::min(a_size, b_size) == 1) || (fabs(dot_product(l_dir, s_dir)) > (1.0f - raptor_physics::EPSILON)))
            {
                BOOST_LOG_TRIVIAL(trace) << "Finding normal for point and line";
                BOOST_LOG_TRIVIAL(trace) << "L points: " << l_points[0] << " and " << l_points[1];
                BOOST_LOG_TRIVIAL(trace) << "S points: " << s_points[0] << " and " << s_points[1];

                /* Find the point on l0 -> l1 most opposite s0 */
                const point_t perp((l_points[0] - (l_dir * dot_product(l_dir, l_points[0] - s_points[0]))) - s_points[0]);

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
                BOOST_LOG_TRIVIAL(trace) << "Rotating: " << norm << " about: " << _po.get_orientation();
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
            BOOST_LOG_TRIVIAL(trace) << "Oritentated to: " << norm;

            if (dot_product(norm, l_points[0] - s_points[0]) < 0.0f)
            {
                BOOST_LOG_TRIVIAL(trace) << "Flipped based on :" << (l_points[0] - s_points[0]) << ", dot is: " << dot_product(norm, l_points[0] - s_points[0]);
                norm = -norm;
            }
            break;
    }

    /* Maintain the relative direction of the normal */
    if (a_size < b_size)
    {
        BOOST_LOG_TRIVIAL(trace) << "Flipped because of simplex size: " << norm;
        norm = -norm;
    }

    BOOST_LOG_TRIVIAL(trace) << "Normal of impact: " << norm;
    return norm;
}


point_t simplex::center_of_impact(const simplex &s, const point_t &noc) const
{
    METHOD_LOG;

    /* Simplex must alway be the same size */
    assert(_size == s._size);

    /* Get all verts on the colliding feature */
    std::unique_ptr<std::vector<point_t>> verts_a(get_points_in_contact_plane(noc));
    std::unique_ptr<std::vector<point_t>> verts_b(s.get_points_in_contact_plane(noc));
    BOOST_LOG_TRIVIAL(trace) << "Found " << verts_a->size() << " a verts: " << verts_a->front() << array_to_stream(++verts_a->begin(), verts_a->end(), [this](std::stringstream *s, const point_t &p)
    {
        (*s) << " " << p;
        return s;
    });
    BOOST_LOG_TRIVIAL(trace) << "Found " << verts_b->size() << " b verts: " << verts_b->front() << array_to_stream(++verts_b->begin(), verts_b->end(), [this](std::stringstream *s, const point_t &p)
    {
        (*s) << " " << p;
        return s;
    });

    /* Check verts are hit */
    /* This is done by finding the hull of the a and b vert separately */
    /* then taking the minimum of this hull */
    _po.get_orientation().rotate(&verts_a->front());
    point_t hull_a_max(verts_a->front());
    point_t hull_a_min(verts_a->front());
    for (unsigned int i = 1; i < verts_a->size(); ++i)
    {
        _po.get_orientation().rotate(&verts_a->at(i));
        hull_a_max = max(hull_a_max, verts_a->at(i));
        hull_a_min = min(hull_a_min, verts_a->at(i));
    }

    s._po.get_orientation().rotate(&verts_b->front());
    point_t hull_b_max(verts_b->front());
    point_t hull_b_min(verts_b->front());
    for (unsigned int i = 1; i < verts_b->size(); ++i)
    {
        s._po.get_orientation().rotate(&verts_b->at(i));
        hull_b_max = max(hull_b_max, verts_b->at(i));
        hull_b_min = min(hull_b_min, verts_b->at(i));
    }

    /* Move points to global */
    hull_a_min += _po.get_center_of_mass();
    hull_a_max += _po.get_center_of_mass();
    hull_b_min += s._po.get_center_of_mass();
    hull_b_max += s._po.get_center_of_mass();

    /* Find combine hull bounds */
    const point_t hull_max(min(hull_a_max, hull_b_max));
    const point_t hull_min(max(hull_a_min, hull_b_min));

    /* Average accross verts */
    BOOST_LOG_TRIVIAL(trace) << "Point off collision: " << ((hull_min + hull_max) * 0.5f);
    return ((hull_min + hull_max) * 0.5f);
}


int simplex::get_unique_points(point_t *const points) const
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


std::vector<point_t>* simplex::get_points_in_contact_plane(const point_t &noc) const
{
    /* Get all verts on the colliding feature */
    std::vector<point_t> *verts = new std::vector<point_t>();

    /* Find all verts perpendicular to noc in a */
    point_t noc_l(noc);
    -(_po.get_orientation()).rotate(&noc_l);
    const point_t a(get_vertex(0));
    for (int i = 0; i < _po.get_vertex_group()->get_number_of_vertices(); ++i)
    {
        /* Check for vert is a */
        const point_t vert  = _po.get_vertex_group()->get_vertex(i);
        const point_t diff  = vert - a;
        if (fabs(dot_product(diff, noc_l)) < (5.0f * raptor_physics::EPSILON))/* This test needs a little extra head room after moving the vert to global co-ordinates */
        {
            verts->push_back(vert);
        }
    }

    /* Check that some verts were found */
    assert(!verts->empty());

    return verts;
}
}; /* namespace raptor_physics */
