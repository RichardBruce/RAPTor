#pragma once

/* Shared headers */
#include <vector>

/* Common headers */
#include "common.h"
#include "point_t.h"
#include "physics_common.h"
#include "physics_object.h"


namespace raptor_physics
{
class simplex
{
    public :
        /* CTOR */
        simplex(const physics_object &po)
            : _po(po), _verts{0, 0, 0, 0, 0, 0, 0, 0}, _verts_rd(&_verts[0]), _verts_wr(&_verts[4]), _size(1) {  };

        /* Copy CTOR */
        simplex(const simplex &s) : _po(s._po), _size(s._size)
        {
            memcpy(_verts, s._verts, 8 * sizeof(int));
            if (s._verts_rd > s._verts_wr)
            {
                _verts_wr = &_verts[0];
                _verts_rd = &_verts[4];
            }
            else
            {
                _verts_rd = &_verts[0];
                _verts_wr = &_verts[4];
            }
        }
            
        /* Compute the Minkowski difference for the simplex points */
        const simplex& compute_c_space(const simplex &b, point_t *const c, const point_t &fixed_rel_disp, const point_t &float_rel_disp) const
        {
            /* Simplices must always be the same size */
            assert(_size == b.size());
            
            /* a and b should only contain support vertices */
            /* Subtracting them samples the Minkowski difference */
            /* Only the vertex (not the displaced vertex) is needed from one of the simplices */
            /* Could be sped up, but a little messier */
            for (unsigned int i = 0; i < _size; i++)
            {
                c[i] = (get_displaced_vertex(float_rel_disp, i) + fixed_rel_disp) - b.get_displaced_vertex(float_rel_disp, i);
            }
            
            /* Check the integrity of the c space simplex */
            /* This is only an issue for a tetrahedron where faces can face inwards */
            /* TODO -- Could be deleted, left as a check */
#ifndef NDEBUG
            if (_size == 4)
            {
                const point_t diff_0_1 = c[1] - c[0];
                const point_t diff_0_2 = c[2] - c[0];
                const point_t diff_0_3 = normalise(c[3] - c[0]);
                const point_t diff_1_3 = normalise(c[3] - c[1]);
                const point_t diff_2_3 = normalise(c[3] - c[2]);
                const point_t norm_0_1_2 (normalise(cross_product(diff_0_1, diff_0_2)));

                assert(dot_product(norm_0_1_2, diff_0_3) < raptor_physics::EPSILON);
                assert(dot_product(norm_0_1_2, diff_1_3) < raptor_physics::EPSILON);
                assert(dot_product(norm_0_1_2, diff_2_3) < raptor_physics::EPSILON);
            }
#endif /* #ifndef NDEBUG */

            return *this;
        }
        
        /* Get the size of the simplex */
        unsigned int size() const
        {
            return _size;
        }

        bool is_new_pair(const simplex &b, const int add_a, const int add_b) const
        {
            /* Simplices should be the same size */
            assert(_size == b._size);

            /* Check each c space point for a match */
            for (unsigned int i = 0; i < _size; i++)
            {
                if ((_verts_rd[i] == add_a) && (b._verts_rd[i] == add_b))
                {
                    return false;
                }
            }

            return true;
        }
        
        /* Update the simplex keeping size number of retain_verts */
        simplex& retain_vertices(const int *retain_verts, const unsigned int size)
        {
            assert((size > 0) && (size <= _size));

            /* Copy up the vertices */
            for (unsigned int i = 0; i < size; i++)
            {
                _verts_wr[i] = _verts_rd[retain_verts[i]];
            }
            
            /* Update the buffer */
            std::swap(_verts_rd, _verts_wr);
            _size = size;            

            return *this;
        }

        /* Update the simplex by adding add_vert */
        simplex& add(const int add_vert)
        {
            _verts_rd[_size++] = add_vert;
            assert((_size > 0) && (_size < 5));

            return *this;
        }
        
        /* Get the normal of the impact */
        point_t normal_of_impact(const simplex &s) const;

        /* Get the "center" of an impact. That is center in the sense of center of mass */
        point_t center_of_impact(const simplex &s, const point_t &norm) const;
        
        /* The rate of change of the normal of impact (dN/dt) */
        point_t rate_of_change_of_normal_of_impact(const simplex &s, const point_t &noc) const
        {
            METHOD_LOG;
            BOOST_LOG_TRIVIAL(trace) << "Finding rate of change of the normal of impact with simplex size: " << _size;

            /* Simplex must alway be the same size */
            assert(_size == s._size);

            /* Can only deal with planes, lines of vertices not tetrahedron */
            /* This shouldn't fail since objects are only brought close to each other */
            /* and not allowed to penetrate */
            assert(_size   < 4);
            assert(s._size < 4);

            /* This just shouldn't happen */
            assert(_size   > 0);
            assert(s._size > 0);

            /* Get unique points defining the simplex */
            std::unique_ptr<std::vector<point_t>> verts_a(get_points_in_contact_plane(noc));
            std::unique_ptr<std::vector<point_t>> verts_b(s.get_points_in_contact_plane(noc));
            const int a_size = std::min(static_cast<int>(verts_a->size()), 3);
            const int b_size = std::min(static_cast<int>(verts_b->size()), 3);

            point_t simplex_verts[6];
            memcpy(&simplex_verts[0], verts_a->data(), a_size * sizeof(point_t));
            memcpy(&simplex_verts[3], verts_b->data(), b_size * sizeof(point_t));

            point_t *l_points;
            point_t *s_points;
            const physics_object *po_a;
            const physics_object *po_b;
            if (a_size >= b_size)
            {
                l_points = &simplex_verts[0];
                s_points = &simplex_verts[3];
                po_a = &_po;
                po_b = &s._po;
            }
            else
            {
                s_points = &simplex_verts[0];
                l_points = &simplex_verts[3];
                po_a = &s._po;
                po_b = &_po;
            }

            point_t norm;
            switch (std::max(a_size, b_size))
            {
                case 1 :
                    {
                        po_a->get_orientation().rotate(&l_points[0]);
                        po_b->get_orientation().rotate(&s_points[0]);
                        const point_t a_cross(cross_product(po_a->get_angular_velocity(), l_points[0]));
                        const point_t b_cross(cross_product(po_b->get_angular_velocity(), s_points[0]));
                        BOOST_LOG_TRIVIAL(trace) << "crosses: " << a_cross << ", " << b_cross;
                        BOOST_LOG_TRIVIAL(trace) << "magnitude: " << magnitude(l_points[0] - s_points[0]);
                        const point_t u((a_cross - b_cross) + (po_a->get_velocity() - po_b->get_velocity()));
                        norm = (u - dot_product(u, noc) * noc) / magnitude((l_points[0] + po_a->get_center_of_mass()) - (s_points[0] + po_b->get_center_of_mass()));
                    }
                    break;

                case 2 :
                    {
                        point_t ea(normalise(l_points[1] - l_points[0]));
                        point_t eb(normalise(s_points[1] - s_points[0]));
                        po_a->get_orientation().rotate(&ea);
                        po_b->get_orientation().rotate(&eb);

                        /* If there is only a point or the lines are parallel */
                        if ((std::min(a_size, b_size) == 1))// || (fabs(dot_product(ea, eb)) > (1.0 - raptor_physics::EPSILON)))
                        {
                            po_a->get_orientation().rotate(&l_points[0]);
                            po_b->get_orientation().rotate(&s_points[0]);
                            const point_t dea(cross_product(po_a->get_angular_velocity(), ea));
                            const point_t dpa(cross_product(po_a->get_angular_velocity(), l_points[0]));
                            const point_t dpb(cross_product(po_b->get_angular_velocity(), s_points[0]));
                            BOOST_LOG_TRIVIAL(trace) << "Derivatives: " << dea << " " << dpa << " " << dpb;

                            l_points[0] += po_a->get_center_of_mass();
                            s_points[0] += po_b->get_center_of_mass();
                            const point_t pb_m_pa(s_points[0] - l_points[0]);
                            const point_t dpb_m_dpa(dpb - dpa);
                            BOOST_LOG_TRIVIAL(trace) << "Diffs: " << pb_m_pa << " " << dpb_m_dpa;

                            const point_t dea_cross_p(cross_product(dea, pb_m_pa));
                            const point_t ea_cross_dp(cross_product(ea, dpb_m_dpa));
                            const point_t ea_cross_p(cross_product(ea, pb_m_pa));
                            BOOST_LOG_TRIVIAL(trace) << "Cross: " << dea_cross_p << " " << ea_cross_dp << " " << ea_cross_p;

                            const point_t u(cross_product(dea_cross_p + ea_cross_dp, ea) + cross_product(ea_cross_p, dea));
                            BOOST_LOG_TRIVIAL(trace) << "U: " << u << " magnitude: " << magnitude(cross_product(ea_cross_p, ea));
                            norm = (u - dot_product(u, noc) * noc) / magnitude(cross_product(ea_cross_p, ea));
                        }
                        else
                        {
                            assert(fabs(dot_product(ea, eb)) < (1.0 - raptor_physics::EPSILON) || !"Error: Cant find rate of change of normal for (anti-)parallel edges");

                            const point_t a_cross(cross_product(po_a->get_angular_velocity(), ea));
                            const point_t b_cross(cross_product(po_b->get_angular_velocity(), eb));
                            const point_t u(cross_product(ea, a_cross) + cross_product(b_cross, eb));

                            BOOST_LOG_TRIVIAL(trace) << "Derivatives: " << a_cross << " " << b_cross;
                            BOOST_LOG_TRIVIAL(trace) << "U: " << u << " top: " <<  (u - dot_product(u, noc) * noc) << " magnitude: " << magnitude(cross_product(ea, eb));
                            norm = (u - dot_product(u, noc) * noc) / magnitude(cross_product(ea, eb));
                        }
                    }
                    break;

                case 3 :
                    norm = cross_product(po_a->get_angular_velocity(), noc);
                    break;
            }

            /* Maintain the relative direction of the normal */
            if (a_size < b_size)
            {
                norm = -norm;
            }

            BOOST_LOG_TRIVIAL(trace) << "Rate of change of normal of impact: " << norm;
            return norm;
        }
            
        /* Get a vertex from data */
        /* If the vertex is at a negative index then adjust for the relative displacement */
        const point_t get_displaced_vertex(const point_t &rel_disp, const int i) const
        {
            const int v = _verts_rd[i];
            if (v < 0)
            {
                return _po.get_orientated_vertex(v & 0x7fffffff) + rel_disp;
            }
            else
            {
                return _po.get_orientated_vertex(v);
            }
        }

        /* Get a vertex from data ignoring any displacement */
        const point_t get_vertex(const int i) const
        {
            return _po.get_vertex_group()->get_vertex(_verts_rd[i] & 0x7fffffff);
        }

    private :
        /* Prohibit assignment */
        simplex& operator=(const simplex &s)
        {
            _size = s._size;
            
            memcpy(_verts, s._verts, 8 * sizeof(int));
            if (s._verts_rd > s._verts_wr)
            {
                _verts_wr = &_verts[0];
                _verts_rd = &_verts[4];
            }
            else
            {
                _verts_rd = &_verts[0];
                _verts_wr = &_verts[4];
            }

            return *this;
        }

        /* Get the unique points defining the simplex */        
        int get_unique_points(point_t *const points) const;

        /* Get all points on the plane of contact */
        std::vector<point_t>* get_points_in_contact_plane(const point_t &noc) const;

        const physics_object &  _po;        /* The vertices of the objects the simplex is in    */
        int                     _verts[8];  /* Index of the vertices used by the simplex        */
        int                  *  _verts_rd;  /* Pointer to write new simplex into                */
        int                  *  _verts_wr;  /* Pointer to read current simplex from             */
        unsigned int            _size;      /* The size of the simplex                          */
};
}; /* namespace raptor_physics */
