#pragma once

/* Standard headers */
#include <map>
#include <utility>
#include <vector>

/* remove */
#include <algorithm>
#include <unordered_map>

/* Common headers */
#include "common.h"
#include "point_t.h"

/* Boost headers */
#include "boost/noncopyable.hpp"

/* Physics headers */
#include "tracking_info.h"
#include "vertex_group.h"

namespace raptor_physics
{
/* Forward declarations */
class physics_object;
class physics_engine;

class contact_graph_node : private boost::noncopyable
{
    public :
        /* CTOR */
        contact_graph_node(std::unordered_map<int, physics_object*> *o, std::map<physics_object*, tracking_info<physics_object>*>  &info, physics_object *const r)
            : _r(r), _t(info[_r])
        {
            /* Track already added nodes */
            std::map<physics_object*, contact_graph_node*> added;
            added.insert({_r, this});

            /* Append all sliding contact nodes at this level */
            for (auto &p : (*_t))
            {
//    int vg_a_idx = (*std::find_if(o->begin(), o->end(), [&] (const std::pair<int, physics_object*>& n) { return n.second == _r; })).first;
//    int vg_b_idx = (*std::find_if(o->begin(), o->end(), [&] (const std::pair<int, physics_object*>& n) { return n.second == p.first; })).first;
//    cout << "Checking for contact between (top): " << vg_a_idx << " and " << vg_b_idx;

                if (p.second->get_type() == SLIDING_COLLISION)
                {
                    /* Check if this object was added to the graph already */
                    auto done = added.find(p.first);

                    /* If not add it */
                    if (done == added.end())
                    {
//                        cout << " new contact" << endl;
                        _c.push_back(new contact_graph_node(o, info, p.first, &added, _r, true));
                    }
                    /* If so copy it */
                    else
                    {
                        _c.push_back(new contact_graph_node(o, info, p.first, &added, _r, false));
//                        cout << " circular contact" << endl;
                    } 
                }
                else
                {
//                    cout << " no contact" << endl;
                }
            }
//            cout << "contact graph built" << endl;
        };

        /* DTOR */
        ~contact_graph_node()
        {
            for (auto &c : _c)
            {
                delete c;
            }
        }

        unsigned int size() const { return _c.size(); }

        /* Perform 1 iteration of resolve forces on the contact graph */
        point_t resolve_forces_iteration(std::unordered_map<int, physics_object*> *o, const float t_step);

        /* Void collisions with everything we processed */
        const contact_graph_node& void_collisions(physics_engine &p) const;

    private :
        /* Intermediate and leaf node initialisation */
        contact_graph_node(std::unordered_map<int, physics_object*> *o, std::map<physics_object*, tracking_info<physics_object>*> &info, 
            physics_object *const r, std::map<physics_object*, contact_graph_node*> *const a, const physics_object *const f, 
            const bool build)
            : _r(r), _t(info[_r])
        {
            /* No need to recurse just hold r */
            if (!build)
            {
                return;
            }

            /* Track as been added */
            a->insert({_r, this});

            /* Append all sliding contact nodes at this level that arent the from node */
            for (auto &p : (*_t))
            {
//        int vg_a_idx = (*std::find_if(o->begin(), o->end(), [&] (const std::pair<int, physics_object*>& n) { return n.second == _r; })).first;
//        int vg_b_idx = (*std::find_if(o->begin(), o->end(), [&] (const std::pair<int, physics_object*>& n) { return n.second == p.first; })).first;
//        cout << "Checking for contact between: " << vg_a_idx << " and " << vg_b_idx;

                if ((p.second->get_type() == SLIDING_COLLISION) && (p.first != f))
                {
                    /* Check if this object was added to the graph already */
                    auto done = a->find(p.first);

                    /* If not add it or added it arbitarily to the smallest pointered node */
                    if (done == a->end())
                    {
//                        cout << " new contact" << endl;
                        _c.push_back(new contact_graph_node(o, info, p.first, a, _r, true));
                    }
                    else
                    {
                        _c.push_back(new contact_graph_node(o, info, p.first, a, _r, false));
//                        cout << " circular contact" << endl;
                    }   
                }
                else
                {
//                    cout << " no contact" << endl;
                }
            }
        }

        const contact_graph_node&   componentise(std::pair<point_t, point_t> *const perp, std::pair<point_t, point_t> *const proj, const point_t &a, const point_t &b, const point_t &noc) const;
        point_t                     calculate_projected_forces(std::pair<point_t, point_t> *const proj_f, const point_t &noc, const float m_a, const float m_b) const;
        point_t                     calculate_perpendicular_force(std::pair<point_t, point_t> *const perp_f, const std::pair<point_t, point_t> &proj_f, const std::pair<point_t, point_t> &perp_v, const point_t &noc, const float m_a, const float m_b, const float mus, const float muk, const float t) const;
        const contact_graph_node&   apply_friction(std::pair<point_t, point_t> *const perp_f, const point_t &net_f_proj, const point_t &net_v_perp, const float mus, const float muk, const float m_a, const float m_b, const float t) const;
        point_t                     average_volocities(std::pair<point_t, point_t> *const v, const float m_a, const float m_b) const;

        physics_object                         *const  _r;
        tracking_info<physics_object>                   *const  _t;
        std::vector<contact_graph_node*>        _c;
};
}; /* namespace raptor_physics */
