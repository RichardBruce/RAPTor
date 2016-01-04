#pragma once

/* Standard headers */
#include <algorithm>
#include <utility>
#include <vector>

/* Boost headers */
#include "boost/noncopyable.hpp"

/* Common headers */
#include "logging.h"
#include "common.h"
#include "point_t.h"

/* Physics headers */
#include "collision_info.h"
#include "tracking_info.h"
#include "vertex_group.h"
#include "lcp_solver.h"


namespace raptor_physics
{
/* Forward declarations */
class physics_object;


/* Class to hold the information associated with an edge in the contact graph */
template<class S = simplex>
class contact_graph_edge
{
    public :
        /* CTOR */
        contact_graph_edge(collision_info<S> *const info, const int to, const int edge)
        : _info(info), _to(to), _edge(edge) {  }

        /* Allow default DTOR and copy CTOR */

        /* Access functions */
        collision_info<S> *const    info()      const { return _info;   }
        int                         to()        const { return _to;     }
        int                         edge_id()   const { return _edge;   }

    private :
        collision_info<S> *const    _info;
        const int                   _to;
        const int                   _edge;
};


/* Class to represent a contact graph */
template<class PO = physics_object, class S = simplex>
class contact_graph : private boost::noncopyable
{
    public :
        typedef std::vector<PO *> vert_list;
        typedef std::vector<contact_graph_edge<S>> adj_list;

        /* CTOR */
        contact_graph() : _verts(0), _edges(0), _contacts(0) {  };

        contact_graph(std::map<PO*, tracking_info<PO, S>*> &info, PO *const r) :
            _verts(1), _edges(0), _contacts(0)
        {
            // METHOD_LOG;
            start_build_contact_graph(info, r);
        }


        /* Construct, but re-using as much dynamic memory as possible */
        /* The edges are still deleted and newed again */
        void rebuild(std::map<PO*, tracking_info<PO, S>*> &info, PO *const r)
        {
            // METHOD_LOG;

            /* Reset data structures */
            _verts      = 1;
            _edges      = 0;
            _contacts   = 0;
            _adj_map.clear();
            _inv_lookup.clear();

            /* Build */
            start_build_contact_graph(info, r);
        }

        /* Access functions */
        int                 number_of_vertices()    const { return _verts;              }
        int                 number_of_edges()       const { return _edges;              }
        int                 number_of_contacts()    const { return _contacts;           }
        const vert_list &   vertices()              const { return _inv_lookup;         }
        const adj_list &    adjacent(const int v)   const { return _adj_map[v].second;  }

        /* Resolve forces to enforce non-penetration */
        contact_graph<PO, S>& resolve_forces()
        {
            // METHOD_LOG;

            /* Contact matrix, one entry per pair of contact pairs */
            lcp_solver sol_v(_contacts, _contacts + 1);
            compute_contact_matrix(sol_v.initialise_m(), false);

            /* Fix up the velocities */
            /* Pre-impluse closing velocities, one entry per contact pair */
            compute_relative_velocity(sol_v.initialise_q());

            /* Minimise impulses */
            std::unique_ptr<float []> resting_impulses(new float [_contacts * 3]);
            const bool min_impulse = sol_v.solve(resting_impulses.get());
            // assert(min_impulse || !"Impulse minimisation failed");
            if (min_impulse)
            {
                /* Apply impluses */
                int from_idx    = 0;
                int force_idx   = 0;
                for (auto &v : _adj_map)
                {
                    /* For each edge from that vertex */
                    for (auto &e : v.second)
                    {
                        /* Only consider the "forward" contacts */
                        if (from_idx < e.to())
                        {
                            continue;
                        }

                        const auto& s = e.info()->get_simplex();
                        const point_t noc(e.info()->get_normal_of_collision());
                        for (int pt = 0; pt < s->contact_manifold_size(); ++pt)
                        {
                            const point_t poc(s->contact_manifold_point(pt));
                            const point_t impulse(noc * resting_impulses[force_idx++]);
                            
                            BOOST_LOG_TRIVIAL(trace) << "Applying impluse: " << impulse << " to contact pair: " << from_idx << ", " << e.to() << " noc: " << noc;
                            v.first->apply_impulse( impulse, poc, false);
                            _adj_map[e.to()].first->apply_impulse(-impulse, poc, false);
                            // BOOST_LOG_TRIVIAL(trace) << "New p0 v: " << v.first->get_velocity() << ", w: " << v.first->get_angular_velocity() << ", t: " << v.first->get_velocity(poc);
                            // BOOST_LOG_TRIVIAL(trace) << "New p1 v: " << _adj_map[e.to()].first->get_velocity() << ", w: " << _adj_map[e.to()].first->get_angular_velocity() << ", t: " << _adj_map[e.to()].first->get_velocity(poc);
                        }
                    }
                    ++from_idx;
                }
            }

            /* Contact matrix, one entry per pair of contact pairs */
            lcp_solver sol_f(_contacts * (SPANNING_VECTORS + 2), (_contacts * (SPANNING_VECTORS + 2)) + 1);
            compute_contact_matrix(sol_f.initialise_m(), true);

            /* One entry per contact pair */
            compute_relative_acceleration(sol_f.initialise_q());

            /* Solve */
            std::unique_ptr<float []> resting_forces(new float [_contacts * (SPANNING_VECTORS + 2)]);
            const bool solve_force = sol_f.solve(resting_forces.get());
            // assert(solve_force || !"Resting force resolution failed");
            if (solve_force)
            {
                /* Apply forces */
                int from_idx    = 0;
                int force_idx   = 0;
                int fric_idx    = _contacts + 1;
                point_t span[SPANNING_VECTORS];
                for (auto &v : _adj_map)
                {
                    /* For each edge from that vertex */
                    for (auto &e : v.second)
                    {
                        /* Only consider the "forward" contacts */
                        if (from_idx < e.to())
                        {
                            continue;
                        }

                        const auto& s = e.info()->get_simplex();
                        const point_t noc(e.info()->get_normal_of_collision());
                        build_spaning_tanjents(&span[0], noc);
                        for (int pt = 0; pt < s->contact_manifold_size(); ++pt)
                        {
                            const point_t poc(s->contact_manifold_point(pt));
                            const point_t f(noc * resting_forces[force_idx++]);
                            
                            PO *const b = _adj_map[e.to()].first;
                            BOOST_LOG_TRIVIAL(trace) << "Applying force: " << f << " to contact pair: " << from_idx << ", " << e.to() << " noc: " << noc;
                            v.first->apply_internal_force(poc - v.first->get_center_of_mass(), f);
                            b->apply_internal_force(poc - b->get_center_of_mass(), -f);

                            for (int i = 0; i < SPANNING_VECTORS; ++i)
                            {
                                const point_t f(span[i] * resting_forces[fric_idx++]);
                                BOOST_LOG_TRIVIAL(trace) << "Applying friction force: " << f << " to contact pair: " << from_idx << ", " << e.to() << " span: " << span[i];
                                v.first->apply_internal_force(poc - v.first->get_center_of_mass(), f);
                                b->apply_internal_force(poc - b->get_center_of_mass(), -f);
                            }
                        }
                    }
                    ++from_idx;
                }
            }

            /* Update object bounds */
            if (min_impulse || solve_force)
            {
                /* Apply impluses */
                int from_idx = 0;
                for (auto &v : _adj_map)
                {
                    /* For each edge from that vertex */
                    for (auto &e : v.second)
                    {
                        /* Only consider the "forward" contacts */
                        if (from_idx < e.to())
                        {
                            continue;
                        }

                        v.first->update_bounds();
                        _adj_map[e.to()].first->update_bounds();
                    }
                    ++from_idx;
                }
            }

            return *this;
        }


        /* Void collisions with everything we processed */
        const contact_graph<PO, S>& void_collisions(std::map<PO*, tracking_info<PO, S>*> *const info) const
        {
            for (auto &c : (*info))
            {
                for (auto v : _inv_lookup)  /* This is a sub set of the outter loop */
                {
                    c.second->void_collision(v);
                }
            }

            return *this;
        }

    private :
        void start_build_contact_graph(std::map<PO*, tracking_info<PO, S>*> &info, PO *const r)
        {
            /* Assign r index 0, add it to the reverse look up and the adjacent map */
            _inv_lookup.push_back(r);
            _adj_map.push_back(std::make_pair(r, adj_list()));

            /* For each object that r is in contact with */
            int from_idx = 0;
            build_contact_graph(info, r, from_idx);

            // BOOST_LOG_TRIVIAL(trace) << "Built contact graph, edges: " << _edges << ", vertices: " << _verts;
        }

        void build_contact_graph(std::map<PO*, tracking_info<PO, S>*>  &info, PO *const r, const int from_idx)
        {
            // BOOST_LOG_TRIVIAL(trace) << "Adding edges for vertex: " << from_idx;

            const tracking_info<PO, S> *const t = info[r];
            for (auto &p : (*t))
            {
                if ((p.second->get_type() == collision_t::SLIDING_COLLISION) && (p.second->get_time() < std::numeric_limits<float>::max()))
                {
                    /* If not previously encountered */
                    /* TODO -- This is a linear search for now */
                    int to_idx;
                    auto inv_iter = std::find(_inv_lookup.begin(), _inv_lookup.end(), p.first);
                    if (inv_iter == _inv_lookup.end())
                    {
                        /* Assign index, add to inverse loop up and adjacent map */
                        // BOOST_LOG_TRIVIAL(trace) << "Created new vertex: " << _verts;
                        to_idx = _verts++;
                        _inv_lookup.push_back(p.first);
                        _adj_map.push_back(std::make_pair(p.first, adj_list()));
                    }
                    else
                    {
                        /* Get index */
                        to_idx = std::distance(_inv_lookup.begin(), inv_iter);
                        // BOOST_LOG_TRIVIAL(trace) << "Found vertex: " << to_idx;
                        
                        /* Check if this edge exists */
                        auto adj_iter = this->find_edge(from_idx, to_idx);
                        if (adj_iter != _adj_map[from_idx].second.end())
                        {
                            // BOOST_LOG_TRIVIAL(trace) << "Edge forms loop, skipping";
                            continue;
                        }
                    }

                    /* Add edge to adjacent map */
                    p.second->get_point_of_collision();
                    (*info[p.first])[r]->get_point_of_collision();
                    _adj_map[from_idx].second.emplace_back(p.second, to_idx, _edges);
                    _adj_map[to_idx].second.emplace_back((*info[p.first])[r], from_idx, _edges);
                    ++_edges;
                    _contacts += p.second->get_simplex()->contact_manifold_size();
                    // BOOST_LOG_TRIVIAL(trace) << "Added edge: " << from_idx << " -> " << to_idx;
                    assert(dot_product(_adj_map[from_idx].second.back().info()->get_normal_of_collision(), r->get_center_of_mass() - p.first->get_center_of_mass()) > 0.0 || !"Error: Inconsitant normal");
                    assert(dot_product(_adj_map[to_idx].second.back().info()->get_normal_of_collision(), p.first->get_center_of_mass() - r->get_center_of_mass()) > 0.0 || !"Error: Inconsitant normal");

                    /* Recurse from this node */
                    build_contact_graph(info, p.first, to_idx);
                }
            }
        }

        const contact_graph<PO, S>& compute_contact_matrix(float *const c, const bool ext) const
        {
            /* Clear the matrix */
            const int matrix_width = ext ? (_contacts * (SPANNING_VECTORS + 2)) : _contacts;
            memset(&c[0], 0, matrix_width * matrix_width * sizeof(float));

            /* For each vertex */
            int from_idx = 0;
            const int mu_offset = _contacts;
            for (auto &v_i : _adj_map)
            {
                /* For each edge from that vertex */
                const PO *const a_i = v_i.first;
                for (auto &e_i : v_i.second)
                {
                    /* Only consider the "forward" contacts */
                    if (from_idx < e_i.to())
                    {
                        continue;
                    }

                    const PO *const b_i = _adj_map[e_i.to()].first;
                    const point_t noc_i(e_i.info()->get_normal_of_collision());
                    
                    /* Foreach point on a contact manifold */
                    const auto& simplex_i = e_i.info()->get_simplex();
                    for (int pt_i = 0; pt_i < simplex_i->contact_manifold_size(); ++pt_i)
                    {
                        const point_t poc_i(simplex_i->contact_manifold_point(pt_i));
                        
                        const point_t ra_n_i(cross_product(poc_i - a_i->get_center_of_mass(), noc_i));
                        const point_t rb_n_i(cross_product(poc_i - b_i->get_center_of_mass(), noc_i));

                        /* For everything in contact with a_i */
                        const int row_idx = (e_i.edge_id() + pt_i) * matrix_width;
                        for (auto &e_j : _adj_map[from_idx].second)
                        {
                            const point_t noc_j(e_j.info()->get_normal_of_collision());
                            const float lin = dot_product(noc_i, noc_j) / a_i->get_mass();

                            const auto& simplex_j = e_j.info()->get_simplex();
                            for (int pt_j = 0; pt_j < simplex_j->contact_manifold_size(); ++pt_j)
                            {
                                const point_t poc_j(simplex_j->contact_manifold_point(pt_j));
                                const point_t ra_n_j(cross_product(poc_j - a_i->get_center_of_mass(), noc_j));

                                const float rot = dot_product(ra_n_i, ra_n_j / a_i->get_orientated_tensor());
                                c[row_idx + e_j.edge_id() + pt_j] += (lin + rot);

                                if (ext)
                                {
                                    c[mu_offset + row_idx + e_j.edge_id() + pt_j] = 0.5f; /* Add coefficient of friction */
                                }
                            }
                        }

                        /* For everything in contact with b_i */
                        for (auto &e_j : _adj_map[e_i.to()].second)
                        {
                            const point_t noc_j(e_j.info()->get_normal_of_collision());
                            const float lin = dot_product(noc_i, noc_j) / b_i->get_mass();

                            const auto& simplex_j = e_j.info()->get_simplex();
                            for (int pt_j = 0; pt_j < simplex_j->contact_manifold_size(); ++pt_j)
                            {
                                const point_t poc_j(simplex_j->contact_manifold_point(pt_j));
                                const point_t rb_n_j(cross_product(poc_j - b_i->get_center_of_mass(), noc_j));

                                const float rot = dot_product(rb_n_i, rb_n_j / b_i->get_orientated_tensor());
                                c[row_idx + e_j.edge_id() + pt_j] -= (lin + rot);

                                if (ext)
                                {
                                    c[mu_offset + row_idx + e_j.edge_id() + pt_j] = 0.5f; /* Add coefficient of friction */
                                }
                            }
                        }
                    }
                }

                ++from_idx;
            }

            /* Populate unit vectors */
            if (ext)
            {
                int lambda_offset = lambda_offset + (_contacts * matrix_width) + _contacts + 1;
                for (int i = 0; i < SPANNING_VECTORS; ++i)
                {
                    c[lambda_offset++] = 1.0f;
                }

                int beta_offset = mu_offset + (_contacts * matrix_width) + matrix_width;
                for (int i = 0; i < _contacts; ++i)
                {
                    for (int j = 0; j < SPANNING_VECTORS; ++j)
                    {
                        c[beta_offset + (j * matrix_width)] = -1.0f;
                    }
                    ++beta_offset;
                }

            }

            return *this;
        }


        const contact_graph<PO, S>& compute_relative_velocity(float *const v_vec)
        {
            /* For each vertex */
            int idx = 0;
            int from_idx = 0;
            for (auto &v : _adj_map)
            {
                /* For each edge from that vertex */
                const PO *const a = v.first;
                for (auto &e : v.second)
                {
                    /* Only consider the "forward" contacts */
                    if (from_idx < e.to())
                    {
                        continue;
                    }

                    const PO *const b = _adj_map[e.to()].first;
                    const point_t noc(e.info()->get_normal_of_collision());

                    /* Foreach contact point calculate the relative velocity */
                    const auto& s = e.info()->get_simplex();
                    for (int pt = 0; pt < s->contact_manifold_size(); ++pt)
                    {
                        const point_t poc(s->contact_manifold_point(pt));

                        const point_t va(a->get_velocity(poc));
                        const point_t vb(b->get_velocity(poc));
                        v_vec[idx++] = dot_product(noc, va - vb) - RELATIVE_VELOCITY_EPSILON;
                    }
                }

                ++from_idx;
            }
         
            return *this;
        }


        const contact_graph<PO, S>& compute_relative_acceleration(float *const b_vec)
        {
            const int size = _contacts * (SPANNING_VECTORS + 2);
            memset(&b_vec[0], 0, size * sizeof(float));

            /* For each vertex */
            int from_idx    = 0;
            int accel_idx   = 0;
            int relvel_idx  = (_contacts << 1);
            point_t span[SPANNING_VECTORS];
            for (auto &v : _adj_map)
            {
                /* For each edge from that vertex */
                const PO *const a = v.first;
                const point_t at1(a->get_force() / a->get_mass());
                for (auto &e : v.second)
                {
                    /* Only consider the "forward" contacts */
                    if (from_idx < e.to())
                    {
                        continue;
                    }

                    const PO *const b = _adj_map[e.to()].first;
                    const point_t noc(e.info()->get_normal_of_collision());

                    build_spaning_tanjents(&span[0], noc);

                    /* Foreach contact point */
                    const auto& s = e.info()->get_simplex();
                    for (int pt = 0; pt < s->contact_manifold_size(); ++pt)
                    {
                        const point_t poc(s->contact_manifold_point(pt));

                        /* vg_a accelerations */
                        const point_t ra(poc - a->get_center_of_mass());
                        const point_t wa_x_ra(cross_product(a->get_angular_velocity(), ra));

                        const point_t at2(cross_product((a->get_torque() + cross_product(a->get_angular_momentum(), a->get_angular_velocity())) / a->get_orientated_tensor(), ra));
                        const point_t at3(cross_product(a->get_angular_velocity(), wa_x_ra));
                        const point_t at4(a->get_velocity() + wa_x_ra);
                        // BOOST_LOG_TRIVIAL(trace) << "a -> force: " << a->get_force() << ", torque: " << a->get_torque();

                        /* vg_b accelerations */
                        const point_t rb(poc - b->get_center_of_mass());
                        const point_t wb_x_rb(cross_product(b->get_angular_velocity(), rb));

                        const point_t bt1(b->get_force() / b->get_mass());
                        const point_t bt2(cross_product((b->get_torque() + cross_product(b->get_angular_momentum(), b->get_angular_velocity())) / b->get_orientated_tensor(), rb));
                        const point_t bt3(cross_product(b->get_angular_velocity(), wb_x_rb));
                        const point_t bt4(b->get_velocity() + wb_x_rb);
                        // BOOST_LOG_TRIVIAL(trace) << "b -> force: " << b->get_force() << ", torque: " << b->get_torque();

                        /* Derivative of contact normal */
                        /* TODO -- Edge lists are expected to be short so this is ok until proven otherwise */
                        auto to_info = find_edge(e.to(), from_idx);
                        assert(to_info != _adj_map[e.to()].second.end());

                        const point_t dn_dt(e.info()->get_simplex()->rate_of_change_of_normal_of_impact(*(to_info->info()->get_simplex()), noc));

                        /* b entry */
                        // BOOST_LOG_TRIVIAL(trace) << "dn_dt: " << dn_dt;
                        b_vec[accel_idx++] = dot_product(noc, at1 + at2 + at3 - bt1 - bt2 - bt3) + (2.0 * dot_product(dn_dt, at4 - bt4)) - RELATIVE_ACCELERATION_EPSILON;
                        // BOOST_LOG_TRIVIAL(trace) << "closing acceleration: " << b_vec[accel_idx - 1] << ", poc: " << poc;

                        const point_t va(a->get_velocity(poc));
                        const point_t vb(b->get_velocity(poc));
                        for (int i = 0; i < SPANNING_VECTORS; ++i)
                        {
                            b_vec[relvel_idx++] = dot_product(span[i], va - vb);
                        }
                    }
                }

                ++from_idx;
            }
         
            return *this;
        }

        void build_spaning_tanjents(point_t *span, const point_t &noc)
        {
            span[0] = perpendicular(noc);
            span[1] = cross_product(noc, span[0]);
            span[2] = -span[0];
            span[3] = -span[1];
            span[4] = normalise((span[0] + span[1]) * 0.5f);
            span[5] = normalise((span[1] + span[2]) * 0.5f);
            span[6] = normalise((span[2] + span[3]) * 0.5f);
            span[7] = normalise((span[3] + span[0]) * 0.5f);
        }

        typename adj_list::iterator find_edge(const int from, const int to)
        {
            /* TODO -- This is a linear search for now */
            auto e = std::find_if(_adj_map[from].second.begin(), _adj_map[from].second.end(), [=] (const contact_graph_edge<S> &m)
                {
                    return m.to() == to;
                });

            return e;
        }

        vert_list                               _inv_lookup;
        std::vector<std::pair<PO*, adj_list>>   _adj_map;
        int                                     _verts;
        int                                     _edges;
        int                                     _contacts;
        static constexpr float                  RELATIVE_VELOCITY_EPSILON       = 0.0f;
        static constexpr float                  RELATIVE_ACCELERATION_EPSILON   = 0.0f;
        static constexpr int                    SPANNING_VECTORS                = 8;
};
}; /* namespace raptor_physics */
