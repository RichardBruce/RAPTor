/* Standard headers */

/* Boost headers */

/* Common headers */

/* Raytracer headers */

#include "gradient_mapper.h"

namespace raptor_raytracer
{
float gradient_mapper::run_procedure(ext_colour_t *const c, const point_t<> &dst, const point_t<> &dir, const point_t<> &n) const
{
    /* Calculate the key for interpolation */
    float key;
    switch (_grad_of)
    {
        case grad_of_t::previous_layer : 
            assert(false);
            break;
        case grad_of_t::bump : 
            assert(false);
            break;
        case grad_of_t::slope : 
            assert(false);
            break;
        case grad_of_t::incidence_angle : 
            key = std::acos(std::fabs(dot_product(n, dir))) * (180.0f / PI);
            break;
        case grad_of_t::light_incidence : 
            assert(false);
            break;
        case grad_of_t::distance_to_camera : 
            assert(false);
            break;
        case grad_of_t::distance_to_object : 
            assert(false);
            break;
        case grad_of_t::x_distance_to_object : 
            assert(false);
            break;
        case grad_of_t::y_distance_to_object : 
            assert(false);
            break;
        case grad_of_t::z_distance_to_object : 
            assert(false);
            break;
        case grad_of_t::weight_map : 
            assert(false);
            break;
    }

    /* Find the key we want */
    const unsigned int upper_idx = std::distance(_keys.begin(), std::upper_bound(_keys.begin(), _keys.end(), key));

    /* Edge cases */
    float alpha = 0.0f;
    if (upper_idx == 0)
    {
        (*c) = _value[0].first;
        alpha = _value[0].second;
    }
    else if (upper_idx == _keys.size())
    {
        (*c) = _value.back().first;
        alpha = _value.back().second;
    }
    else
    {
        /* Interpolate */
        switch (_grads[upper_idx])
        {
            case grad_interpolation_t::line :
            {
                const float key_weight = (key - _keys[upper_idx - 1]) / (_keys[upper_idx] - _keys[upper_idx - 1]);

                const ext_colour_t col_0(_value[upper_idx - 1].first);
                const ext_colour_t col_dist(_value[upper_idx].first - _value[upper_idx - 1].first);
                (*c) = col_0 + (col_dist * key_weight);

                const float alp_0 = _value[upper_idx - 1].second;
                const float alp_dist = alp_0 - _value[upper_idx - 1].second;
                alpha = alp_0 + (alp_dist * key_weight);
            }
            break;

            case grad_interpolation_t::spline :
                assert(false);
                break;

            case grad_interpolation_t::step :
            {
                (*c) = _value[upper_idx].first;
                alpha = _value[upper_idx].second;
            }
            break;
        }
    }

    /* Invert */
    if (inverse())
    {
        (*c) = ext_colour_t(255.0f, 255.0f, 255.0f) - (*c);
    }

    return alpha;
}
}; /* namespace raptor_raytracer */
