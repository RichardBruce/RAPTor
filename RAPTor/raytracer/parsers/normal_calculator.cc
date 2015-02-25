/* Standard headers */
#include <iostream>

/* Boost headers */

/* Common headers */
#include "logging.h"

/* Raytracer headers */
#include "normal_calculator.h"


namespace raptor_raytracer
{
void shared_normal::add(shared_normal **user_0, const point_t &norm_0)
{
    /* Update normal */
    const float use_cnt_0 = static_cast<float>(_users.size());
    const float total_uses = use_cnt_0 + 1.0f;
    _normal = (_normal * (use_cnt_0 / total_uses)) + (norm_0 * (1.0f / total_uses));

    /* Add user */
    (*user_0) = this;
    _users.push_back(user_0);
}

void shared_normal::merge(shared_normal *const merge)
{
    /* Dont merge with myself */
    if (merge == this)
    {
        return;
    }

    /* Recalculate normal */
    const float use_cnt_0 = static_cast<float>(_users.size());
    const float use_cnt_1 = static_cast<float>(merge->_users.size());
    const float total_uses = use_cnt_0 + use_cnt_1;
    _normal = (_normal * (use_cnt_0 / total_uses)) + (merge->_normal * (use_cnt_1 / total_uses));

    /* Update pointers */
    for (shared_normal **moved : merge->_users)
    {
        (*moved) = this;
        _users.push_back(moved);
    }

    /* Clean up */
    merge->_users.clear();
    delete merge;
}

/* Just rememeber information for now */
void normal_calculator::add_point_usage(const std::vector<int> &pnts, const float threshold, const int group, const int pol)
{
    /* Check we have enough space for this pol*/
    if (static_cast<int>(_pols.size()) <= pol)
    {
        _pols.resize(pol + 1);
    }

    /* Look up this pols info */
    pol_info *const info = &_pols[pol];
    info->pnts = pnts;
    info->group = group;
    info->pnt_norms.resize(pnts.size(), nullptr);

    /* Calculate the polygons normal */
    if (pnts.size() > 2)
    {
        const point_t ab(_pnts[pnts[0]] - _pnts[pnts[1]]);
        const point_t bc(_pnts[pnts[2]] - _pnts[pnts[1]]);
        const point_t norm(cross_product(bc, ab));
        info->normal = normalise(norm);
    }

    /* Remember cos of threshold */
    info->cos_threshold = cos(threshold);

    /* Track all polygons a point appears on */
    for (int pnt : pnts)
    {
        _pnt_to_pol[pnt].push_back(pol);
    }
}

/* Runs a global smoothing accross all pols */
void normal_calculator::calculate()
{
    for (unsigned int pnt = 0; pnt < _pnt_to_pol.size(); ++pnt)
    {
        /* Nothing to do */
        auto &pols = _pnt_to_pol[pnt];
        if (pols.size() < 2)
        {
            continue;
        }

        /* Look if we should share a normal for this point with another pol */
        for (unsigned int i = 0; i < pols.size(); ++i)
        {
            const int pol_0 = pols[i];
            pol_info *const info_0 = &_pols[pol_0];
            for (unsigned int j = i + 1; j < pols.size(); ++j)
            {
                const int pol_1 = pols[j];

                /* Check this is the same material */
                pol_info *const info_1 = &_pols[pol_1];
                if (info_0->group != info_1->group)
                {
                    continue;
                }

                /* Check for merge */
                point_t norm_0(info_0->normal);
                point_t norm_1(info_1->normal);
                const float cos_angle = dot_product(norm_0, norm_1);
                if (cos_angle < -info_0->cos_threshold)
                {
                    norm_1 = -norm_1;
                }

                if (fabs(cos_angle) > info_0->cos_threshold)
                {
                    /* Neither surface is sharing a normal for this point */
                    shared_normal **pnt_norm_0 = &info_0->pnt_norms[point_index(info_0, pnt)];
                    shared_normal **pnt_norm_1 = &info_1->pnt_norms[point_index(info_1, pnt)];

                    if (((*pnt_norm_0) == nullptr) && ((*pnt_norm_1) == nullptr))
                    {
                        shared_normal *shared_norm = new shared_normal(pnt_norm_0, pnt_norm_1, norm_0, norm_1);
                        (*pnt_norm_0) = shared_norm;
                        (*pnt_norm_1) = shared_norm;
                    }
                    /* One surface is sharing a normal for this point */
                    else if ((*pnt_norm_1) == nullptr)
                    {
                        (*pnt_norm_0)->add(pnt_norm_1, norm_1);
                        (*pnt_norm_1) = (*pnt_norm_0);
                    }
                    else if ((*pnt_norm_0) == nullptr)
                    {
                        (*pnt_norm_1)->add(pnt_norm_0, norm_0);
                        (*pnt_norm_0) = (*pnt_norm_1);
                    }
                    /* Both surfaces are sharing normals for this point */
                    else
                    {
                        (*pnt_norm_0)->merge(*pnt_norm_1);
                    }
                }
            }
        }
    }
}

std::vector<point_t>* normal_calculator::normals(std::vector<point_t> *const norms, const int pol) const
{
    /* Check if any shared normals are used */
    const pol_info *const info = &_pols[pol];
    auto not_null_iter = std::find_if(info->pnt_norms.begin(), info->pnt_norms.end(), [](shared_normal *norm) { return norm != nullptr; });
    if (not_null_iter == info->pnt_norms.end())
    {
        return nullptr;
    }

    /* Build normals from shared or from face */
    for (auto pnt_norm : info->pnt_norms)
    {
        if (pnt_norm != nullptr)
        {
            norms->push_back(normalise(pnt_norm->normal()));
        }
        else
        {
            norms->push_back(info->normal);
        }
    }

    return norms;
}
}; /* namespace raptor_raytracer */
