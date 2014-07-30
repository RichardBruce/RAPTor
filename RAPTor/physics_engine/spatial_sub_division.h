#ifndef __SPATIAL_SUB_DIVISION_H__
#define __SPATIAL_SUB_DIVISION_H__

/* Standard headers */
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

/* Boost headers */
#include "boost/noncopyable.hpp"
#include "boost/functional/hash.hpp"

/* Common headers */
#include "logging.h"
#include "point_t.h"

/* Physics headers */
#include "object_bound.h"
#include "physics_object.h"


namespace raptor_physics
{
/* Class implementing physics engine acceleration structure */
class spatial_sub_division : private boost::noncopyable
{
    public :
        typedef std::unordered_set<std::pair<const physics_object *, const physics_object *>, boost::hash<std::pair<const physics_object *, const physics_object *>>> collision_set;

        spatial_sub_division(const std::unordered_map<int, physics_object*> &objects)
        : _axis(pick_bounds_axis(objects))
        {
            /* Sweep the objects picking points in the selected axis */
            _bounds.reserve(objects.size() << 1);
            for (auto o : objects)
            {
                _bounds.push_back(o.second->lower_bound(_axis));
                _bounds.push_back(o.second->upper_bound(_axis));
            }

            /* Sort objects */
            std::sort(_bounds.begin(), _bounds.end(), [](const object_bound *const l, const object_bound *const &r)
                {
                    return (*l) < (*r);
                });

            /* Set indices */
            set_indices();

            /* Perform full collision detection */
            collision_detect();
        }

        // /* Allow default DTOR */

        spatial_sub_division& add_object(const physics_object &po)
        {
            object_bound *const lower_bound = po.lower_bound(_axis);
            _bounds.insert(std::upper_bound(_bounds.begin(), _bounds.end(), lower_bound, [](const object_bound *const l, const object_bound *const r)
                {
                    return (*l) < (*r);
                }), lower_bound);

            object_bound *const upper_bound = po.upper_bound(_axis);
            _bounds.insert(std::upper_bound(_bounds.begin(), _bounds.end(), upper_bound, [](const object_bound *const l, const object_bound *const r)
                {
                    return (*l) < (*r);
                }), upper_bound);

            /* Set indices */
            set_indices();

            /* Perform full collision detection */
            /* TODO -- Perhaps this could be an incremental collision detect */
            collision_detect();

            return *this;
        }

        spatial_sub_division& remove_object(const physics_object *const po)
        {
            /* Move both of these object references to the end */
            _bounds.erase(std::remove_if(_bounds.begin(), _bounds.end(), [po](const object_bound *const p)
                {
                    return p->object() == po;
                }), _bounds.end());

            /* Set indices */
            set_indices();

            /* Clean from possibles */
            for(auto it = _axis_possibles.begin(); it != _axis_possibles.end(); )
            {
                if ((it->first == po) | (it->second == po))
                {
                    it = _axis_possibles.erase(it);
                }
                else
                {
                    ++it;
                }
            }

            for(auto it = _possibles.begin(); it != _possibles.end(); )
            {
                if ((it->first == po) | (it->second == po))
                {
                    it = _possibles.erase(it);
                }
                else
                {
                    ++it;
                }
            };

            return *this;            
        }

        spatial_sub_division& update_object(const physics_object &po)
        {
            /* Get object bounds */
            const object_bound *const lower_bound = po.lower_bound(_axis);
            const object_bound *const upper_bound = po.upper_bound(_axis);

            /* Adjust bounds tracking changes in collision set */
            // if (*this < o)
            /* Moving lower bound down */
            while ((lower_bound->index() > 0) && (_bounds[lower_bound->index()]->compare_and_swap(_bounds[lower_bound->index() - 1])))
            {
                std::swap(_bounds[lower_bound->index()], _bounds[lower_bound->index() + 1]);
                update_possibles(_bounds[lower_bound->index()], _bounds[lower_bound->index() + 1]);
            }

            /* Moving upper bound down */
            while ((upper_bound->index() > 0) && (_bounds[upper_bound->index()]->compare_and_swap(_bounds[upper_bound->index() - 1])))
            {
                std::swap(_bounds[upper_bound->index()], _bounds[upper_bound->index() + 1]);
                update_possibles(_bounds[upper_bound->index()], _bounds[upper_bound->index() + 1]);
            }

            /* Moving upper bound up */
            while ((upper_bound->index() < static_cast<int>(_bounds.size() - 1)) && (_bounds[upper_bound->index() + 1]->compare_and_swap(_bounds[upper_bound->index()])))
            {
                std::swap(_bounds[upper_bound->index() - 1], _bounds[upper_bound->index()]);
                update_possibles(_bounds[upper_bound->index() - 1], _bounds[upper_bound->index()]);
            }

            /* Moving lower bound up */
            while ((lower_bound->index() < static_cast<int>(_bounds.size() - 1)) && (_bounds[lower_bound->index() + 1]->compare_and_swap(_bounds[lower_bound->index()])))
            {
                std::swap(_bounds[lower_bound->index() - 1], _bounds[lower_bound->index()]);
                update_possibles(_bounds[lower_bound->index() - 1], _bounds[lower_bound->index()]);
            }

            remove_from_possible(&po);

            return *this;            
        }

        const collision_set& possible_collisions() const
        {
            return _possibles;
        }

        int number_of_possible_collisions() const
        {
            return _possibles.size();
        }


    private :
        spatial_sub_division& remove_from_possible(const physics_object *const po);

        /* Set indices on the bounds */
        void set_indices()
        {
            /* Set indices */
            for (int i = 0; i < static_cast<int>(_bounds.size()); ++i)
            {
                _bounds[i]->index(i);
            }
        }

        /* Find the axis that maximises the range of the scene */
        axis_t pick_bounds_axis(const std::unordered_map<int, physics_object*> &objects) const
        {
            /* Sweep the object to find the scene bounding box */
            point_t min_pt(std::numeric_limits<fp_t>::max(), std::numeric_limits<fp_t>::max(), std::numeric_limits<fp_t>::max());
            point_t max_pt(std::numeric_limits<fp_t>::min(), std::numeric_limits<fp_t>::min(), std::numeric_limits<fp_t>::min());
            for (auto o : objects)
            {
                const point_t hi(o.second->upper_bound(X_AXIS)->bound(), o.second->upper_bound(Y_AXIS)->bound(), o.second->upper_bound(Z_AXIS)->bound());
                const point_t lo(o.second->lower_bound(X_AXIS)->bound(), o.second->lower_bound(Y_AXIS)->bound(), o.second->lower_bound(Z_AXIS)->bound());
                max_pt = max(max_pt, hi);
                min_pt = min(min_pt, lo);
            }

            /* Pick axis to maximise range (and therefore likelyness of separating objects */
            const point_t diff(max_pt - min_pt);
            if (diff.x > diff.y)
            {
                if (diff.x > diff.z)
                {
                    return X_AXIS;
                }
                else
                {
                    return Z_AXIS;
                }
            }
            else
            {
                if (diff.y > diff.z)
                {
                    return Y_AXIS;
                }
                else
                {
                    return Z_AXIS;
                }
            }
        }

        spatial_sub_division& collision_detect()
        {
            /* Clean old state */
            _possibles.clear();
            _axis_possibles.clear();

            /* For all bounds */
            for (unsigned int i = 0; i < _bounds.size(); ++i)
            {
                /* That are minimums */
                if (_bounds[i]->min())
                {
                    /* There may be collisions between this object and any found before the maximum */
                    int j = i + 1;
                    while ((_bounds[i]->object() != _bounds[j]->object()) && (j < static_cast<int>(_bounds.size())))
                    {
                        _axis_possibles.emplace(std::min(_bounds[i]->object(), _bounds[j]->object()), std::max(_bounds[i]->object(), _bounds[j]->object()));
                        ++j;
                    }
                }
            }

            /* Update possibles */
            std::copy_if(_axis_possibles.begin(), _axis_possibles.end(), std::inserter(_possibles, _possibles.begin()), [this](const std::pair<const physics_object *, const physics_object *> &p)
            {
                return is_possible(p);
            });

            return *this;
        }

        void update_possibles(const object_bound *const moving_lo, const object_bound *const moving_hi)
        {
            if (moving_lo->min() && !moving_hi->min())
            {
                const auto pair(std::make_pair(std::min(moving_lo->object(), moving_hi->object()), std::max(moving_lo->object(), moving_hi->object())));
                _axis_possibles.insert(pair);

                if (is_possible(pair))
                {
                    _possibles.insert(pair);
                }
            }

            if (!moving_lo->min() && moving_hi->min())
            {
                const auto pair(std::make_pair(std::min(moving_lo->object(), moving_hi->object()), std::max(moving_lo->object(), moving_hi->object())));
                _axis_possibles.erase(pair);
                _possibles.erase(pair);
            }
        }

        bool is_possible(const std::pair<const physics_object *, const physics_object *> &p) const
        {
            if (this->_axis != X_AXIS)
            {
                const fp_t first_lo = p.first->lower_bound(X_AXIS)->bound();
                const fp_t first_hi = p.first->upper_bound(X_AXIS)->bound();
                const fp_t second_lo = p.second->lower_bound(X_AXIS)->bound();
                const fp_t second_hi = p.second->upper_bound(X_AXIS)->bound();
                if ((first_lo > second_hi) | (first_hi < second_lo))
                {
                    return false;
                }
            }

            if (this->_axis != Y_AXIS)
            {
                const fp_t first_lo = p.first->lower_bound(Y_AXIS)->bound();
                const fp_t first_hi = p.first->upper_bound(Y_AXIS)->bound();
                const fp_t second_lo = p.second->lower_bound(Y_AXIS)->bound();
                const fp_t second_hi = p.second->upper_bound(Y_AXIS)->bound();
                if ((first_lo > second_hi) | (first_hi < second_lo))
                {
                    return false;
                }
            }

            if (this->_axis != Z_AXIS)
            {
                const fp_t first_lo = p.first->lower_bound(Z_AXIS)->bound();
                const fp_t first_hi = p.first->upper_bound(Z_AXIS)->bound();
                const fp_t second_lo = p.second->lower_bound(Z_AXIS)->bound();
                const fp_t second_hi = p.second->upper_bound(Z_AXIS)->bound();
                if ((first_lo > second_hi) | (first_hi < second_lo))
                {
                    return false;
                }
            }

            return true;
        }

        std::vector<object_bound *> _bounds;
        collision_set               _axis_possibles;
        collision_set               _possibles;
        const axis_t                _axis;
};
}; /* namespace raptor_physics */

#endif /* #ifndef __SPATIAL_SUB_DIVISION_H__ */
