#pragma once

/* Standard headers */

/* Common headers */
#include "point_t.h"

/* Physics headers */


namespace raptor_physics
{
namespace test
{
/* Class to act as a simplex while testing */
class mock_simplex
{
    public :
        mock_simplex(const std::vector<point_t<>> &cm, const point_t<> &noc) : _cm(cm), _noc(noc), _dn_dt(0.0f, 0.0f, 0.0f) {  }

        mock_simplex(const mock_simplex &s) : _cm(s._cm), _noc(s._noc) {  }

        /* Get the normal of the impact */
        point_t<> normal_of_impact(const mock_simplex &)  const { return _noc; }

        /* Access to the contact manifold created by center_of_impact */
        int contact_manifold_size() const
        {
            return _cm.size();
        }

        const point_t<>& contact_manifold_point(const int i) const
        {
            return _cm[i];
        }

        /* Center of impact */
        point_t<> center_of_impact(const mock_simplex &, const point_t<> &) const
        {
            return std::accumulate(_cm.begin(), _cm.end(), point_t<>(0.0f, 0.0f, 0.0f)) / static_cast<float>(_cm.size());
        }

        /* The rate of change of the normal of impact (dN/dt) */
        mock_simplex& set_rate_of_change_of_normal_of_impact(const point_t<> &dn_dt)
        {
            _dn_dt = dn_dt;
            return *this;
        }

        point_t<> rate_of_change_of_normal_of_impact(const mock_simplex &, const point_t<> &) const
        {
            return _dn_dt;
        }

    private :
        const std::vector<point_t<>>  _cm;    /* The points defining the contact manifold     */
        const point_t<>               _noc;   /* The last normal of collision                 */
        point_t<>                     _dn_dt; /* Rate of change of normal of collision        */
};

}; /* namespace test */
}; /* namespace raptor_physics */
