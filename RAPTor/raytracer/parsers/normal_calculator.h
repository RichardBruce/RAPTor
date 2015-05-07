/* Standard headers */
#include <vector>

/* Boost headers */

/* Common headers */
#include "point_t.h"

namespace raptor_raytracer
{
class shared_normal
{
    public :
        shared_normal(shared_normal **user_0, shared_normal **user_1, const point_t &norm_0, const point_t &norm_1) :
            _normal((norm_0 * 0.5f) + (norm_1 * 0.5f))
        {
            (*user_0) = this;
            (*user_1) = this;
            _users.push_back(user_0);
            _users.push_back(user_1);
        }

        ~shared_normal()
        {
            /* Stop being used */
            for (auto user : _users)
            {
                (*user) = nullptr;
            }
        }

        void add(shared_normal **user_0, const point_t &norm_0);
        void merge(shared_normal *const merge);

        const point_t & normal() const { return _normal; }

    private :
        std::vector<shared_normal **>   _users;
        point_t                         _normal;
};

struct pol_info
{
    std::vector<shared_normal *>    pnt_norms;
    std::vector<int>                pnts;
    point_t                         normal;
    float                           cos_threshold;
    int                             group;
};

class normal_calculator
{
    public :
        normal_calculator(const std::vector<point_t> &pnts) :
            _pnts(pnts), _pnt_to_pol(pnts.size()) {  }

        ~normal_calculator()
        {
            for (int i = 0; i < static_cast<int>(_pols.size()); ++i)
            {
                for (int j = 0; j < static_cast<int>(_pols[i].pnt_norms.size()); ++j)
                {
                    if (_pols[i].pnt_norms[j] != nullptr)
                    {
                        delete _pols[i].pnt_norms[j];
                        _pols[i].pnt_norms[j] = nullptr;
                    }
                }
            }
        }

        /* Just rememeber information for now */
        void add_point_usage(const std::vector<int> &pnts, const float threshold, const int group, const int pol);

        /* Runs a global smoothing accross all pols */
        void calculate();

        /* Get information out */
        int number_of_polygons() const
        {
            return _pols.size();
        }

        int number_of_points(const int pol) const
        {
            return _pols[pol].pnts.size();
        }

        int global_point(const int pol, const int pnt) const
        {
            return _pols[pol].pnts[pnt];
        }

        int group(const int pol) const
        {
            return _pols[pol].group;
        }

        const std::vector<int>& points_on_polygon(const int pol) const
        {
            return _pols[pol].pnts;
        }

        const std::vector<int>& polygons_on_point(const int pnt) const
        {
            return _pnt_to_pol[pnt];
        }

        void points_on_polygon(std::vector<point_t> *const pnts, const int pol) const
        {
            for (int pnt : _pols[pol].pnts)
            {
                pnts->push_back(_pnts[pnt]);
            }
        }

        std::vector<point_t>* normals(std::vector<point_t> *const norms, const int pol) const;

    private :
        int point_index(const pol_info *const info, const int pnt)
        {
            auto pnt_pos = std::find(info->pnts.begin(), info->pnts.end(), pnt);
            assert(pnt_pos != info->pnts.end());

            return std::distance(info->pnts.begin(), pnt_pos);
        }

        const std::vector<point_t> &    _pnts;
        std::vector<pol_info>           _pols;
        std::vector<std::vector<int>>   _pnt_to_pol;
};
}; /* namespace raptor_raytracer */
