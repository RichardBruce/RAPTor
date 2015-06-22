#pragma once

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
#include "pair_manager.h"
#include "physics_object.h"


namespace raptor_physics
{
/* Class implementing physics engine acceleration structure */
class spatial_sub_division : private boost::noncopyable
{
    public :
        typedef pair_manager<> collision_set;

        spatial_sub_division(const std::unordered_map<int, physics_object*> &objects)
        :   _min_sentinel(new object_bound(nullptr, -std::numeric_limits<float>::max(), true)),
            _max_sentinel(new object_bound(nullptr,  std::numeric_limits<float>::max(), false))
        {
            /* Sweep the objects picking points in the selected axis */
            _bounds[static_cast<int>(axis_t::X_AXIS)].reserve((objects.size() << 1) + 2);
            _bounds[static_cast<int>(axis_t::Y_AXIS)].reserve((objects.size() << 1) + 2);
            _bounds[static_cast<int>(axis_t::Z_AXIS)].reserve((objects.size() << 1) + 2);

            _bounds[static_cast<int>(axis_t::X_AXIS)].push_back(_min_sentinel);
            _bounds[static_cast<int>(axis_t::Y_AXIS)].push_back(_min_sentinel);
            _bounds[static_cast<int>(axis_t::Z_AXIS)].push_back(_min_sentinel);
            for (auto o : objects)
            {
                _bounds[static_cast<int>(axis_t::X_AXIS)].push_back(o.second->lower_bound(axis_t::X_AXIS));
                _bounds[static_cast<int>(axis_t::Y_AXIS)].push_back(o.second->lower_bound(axis_t::Y_AXIS));
                _bounds[static_cast<int>(axis_t::Z_AXIS)].push_back(o.second->lower_bound(axis_t::Z_AXIS));
                _bounds[static_cast<int>(axis_t::X_AXIS)].push_back(o.second->upper_bound(axis_t::X_AXIS));
                _bounds[static_cast<int>(axis_t::Y_AXIS)].push_back(o.second->upper_bound(axis_t::Y_AXIS));
                _bounds[static_cast<int>(axis_t::Z_AXIS)].push_back(o.second->upper_bound(axis_t::Z_AXIS));
            }
            _bounds[static_cast<int>(axis_t::X_AXIS)].push_back(_max_sentinel);
            _bounds[static_cast<int>(axis_t::Y_AXIS)].push_back(_max_sentinel);
            _bounds[static_cast<int>(axis_t::Z_AXIS)].push_back(_max_sentinel);

            /* Sort objects */
            for (int i = 0; i < 3; ++i)
            {
                std::sort(_bounds[i].begin(), _bounds[i].end(), [](const object_bound *const l, const object_bound *const &r)
                    {
                        return (*l) < (*r);
                    });
            }
            assert(_bounds[static_cast<int>(axis_t::X_AXIS)].front()->object()    == nullptr);
            assert(_bounds[static_cast<int>(axis_t::Y_AXIS)].front()->object()    == nullptr);
            assert(_bounds[static_cast<int>(axis_t::Z_AXIS)].front()->object()    == nullptr);
            assert(_bounds[static_cast<int>(axis_t::X_AXIS)].back()->object()     == nullptr);
            assert(_bounds[static_cast<int>(axis_t::Y_AXIS)].back()->object()     == nullptr);
            assert(_bounds[static_cast<int>(axis_t::Z_AXIS)].back()->object()     == nullptr);

            /* Set indices */
            set_indices();

            /* Perform full collision detection */
            _axis_possibles[0].reserve(_bounds[0].size());
            _axis_possibles[1].reserve(_bounds[0].size());
            _axis_possibles[2].reserve(_bounds[0].size());
            _possibles.reserve(_bounds[0].size());

            _axis_possibles[0].load_factor(0.7f);
            _axis_possibles[1].load_factor(0.7f);
            _axis_possibles[2].load_factor(0.7f);
            _possibles.load_factor(0.7f);
            collision_detect();

            _axis_possibles[0].load_factor(1.0f);
            _axis_possibles[1].load_factor(1.0f);
            _axis_possibles[2].load_factor(1.0f);
            _possibles.load_factor(1.0f);
        }

        /* DTOR */
        ~spatial_sub_division()
        {
            delete _min_sentinel;
            delete _max_sentinel;
        }

        spatial_sub_division& add_object(const physics_object &po)
        {
            /* Add bounds to the end and swap the sentinel to the end */
            _bounds[static_cast<int>(axis_t::X_AXIS)].push_back(po.upper_bound(static_cast<axis_t>(axis_t::X_AXIS)));
            _bounds[static_cast<int>(axis_t::X_AXIS)].push_back(po.lower_bound(static_cast<axis_t>(axis_t::X_AXIS)));
            std::swap(_bounds[static_cast<int>(axis_t::X_AXIS)][_bounds[static_cast<int>(axis_t::X_AXIS)].size() - 3], _bounds[static_cast<int>(axis_t::X_AXIS)][_bounds[static_cast<int>(axis_t::X_AXIS)].size() - 1]);

            _bounds[static_cast<int>(axis_t::Y_AXIS)].push_back(po.upper_bound(static_cast<axis_t>(axis_t::Y_AXIS)));
            _bounds[static_cast<int>(axis_t::Y_AXIS)].push_back(po.lower_bound(static_cast<axis_t>(axis_t::Y_AXIS)));
            std::swap(_bounds[static_cast<int>(axis_t::Y_AXIS)][_bounds[static_cast<int>(axis_t::Y_AXIS)].size() - 3], _bounds[static_cast<int>(axis_t::Y_AXIS)][_bounds[static_cast<int>(axis_t::Y_AXIS)].size() - 1]);

            _bounds[static_cast<int>(axis_t::Z_AXIS)].push_back(po.upper_bound(static_cast<axis_t>(axis_t::Z_AXIS)));
            _bounds[static_cast<int>(axis_t::Z_AXIS)].push_back(po.lower_bound(static_cast<axis_t>(axis_t::Z_AXIS)));
            std::swap(_bounds[static_cast<int>(axis_t::Z_AXIS)][_bounds[static_cast<int>(axis_t::Z_AXIS)].size() - 3], _bounds[static_cast<int>(axis_t::Z_AXIS)][_bounds[static_cast<int>(axis_t::Z_AXIS)].size() - 1]);

            /* Set indices */
            set_indices();

            /* Update the new object into place */
            update_object(po);

            return *this;
        }

        spatial_sub_division& remove_object(const physics_object *const po)
        {
            for (int i = 0; i < 3; ++i)
            {
                /* Move both of these object references to the end */
                unsigned int at = po->lower_bound(static_cast<axis_t>(i))->index();
                for (unsigned int j = at + 1; j < _bounds[2].size(); ++j)
                {
                    if (_bounds[i][j]->object() != po)
                    {
                        _bounds[i][at] = _bounds[i][j];
                        _bounds[i][at]->index(at);
                        ++at;
                    }
                }
                _bounds[i].resize(at);

                /* Clean from axis possibles */
                /* This is the major cost of remove_object */
                for (auto iter = _axis_possibles[i].begin(); iter != _axis_possibles[i].end(); )
                {
                    if ((iter->first == po) | (iter->second == po))
                    {
                        _axis_possibles[i].erase(iter);
                    }
                    else
                    {
                        ++iter;
                    }
                }
            }

            /* Clean from possibles */
            for (auto iter = _possibles.begin(); iter != _possibles.end(); )
            {
                if ((iter->first == po) | (iter->second == po))
                {
                    _possibles.erase(iter);
                }
                else
                {
                    ++iter;
                }
            };

            return *this;
        }

        spatial_sub_division& update_object(const physics_object &po)
        {
            update_axis(po, axis_t::X_AXIS);
            update_axis(po, axis_t::Y_AXIS);
            update_axis(po, axis_t::Z_AXIS);

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
        /* Set indices on the bounds */
        void set_indices()
        {
            assert(_bounds[static_cast<int>(axis_t::X_AXIS)].size() == _bounds[static_cast<int>(axis_t::Y_AXIS)].size());
            assert(_bounds[static_cast<int>(axis_t::X_AXIS)].size() == _bounds[static_cast<int>(axis_t::Z_AXIS)].size());

            /* Set indices */
            for (int i = 0; i < static_cast<int>(_bounds[static_cast<int>(axis_t::X_AXIS)].size()); ++i)
            {
                _bounds[static_cast<int>(axis_t::X_AXIS)][i]->index(i);
                _bounds[static_cast<int>(axis_t::Y_AXIS)][i]->index(i);
                _bounds[static_cast<int>(axis_t::Z_AXIS)][i]->index(i);
            }
        }

        spatial_sub_division& collision_detect()
        {
            /* Clean old state */
            _possibles.clear();
            _axis_possibles[static_cast<int>(axis_t::X_AXIS)].clear();
            _axis_possibles[static_cast<int>(axis_t::Y_AXIS)].clear();
            _axis_possibles[static_cast<int>(axis_t::Z_AXIS)].clear();

            /* For all bounds */
            collision_detect_axis(axis_t::X_AXIS);
            collision_detect_axis(axis_t::Y_AXIS);
            collision_detect_axis(axis_t::Z_AXIS);

            /* Update possibles */
            const axis_t smallest_set = (_axis_possibles[static_cast<int>(axis_t::X_AXIS)].size() < _axis_possibles[static_cast<int>(axis_t::Y_AXIS)].size()) ?
                ((_axis_possibles[static_cast<int>(axis_t::X_AXIS)].size() < _axis_possibles[static_cast<int>(axis_t::Z_AXIS)].size()) ? axis_t::X_AXIS : axis_t::Z_AXIS) :
                ((_axis_possibles[static_cast<int>(axis_t::Y_AXIS)].size() < _axis_possibles[static_cast<int>(axis_t::Z_AXIS)].size()) ? axis_t::Y_AXIS : axis_t::Z_AXIS);

            switch (smallest_set)
            {
                case axis_t::X_AXIS :
                    for (auto p : _axis_possibles[static_cast<int>(axis_t::X_AXIS)])
                    {
                        if ((_axis_possibles[static_cast<int>(axis_t::Y_AXIS)].find(p) != _axis_possibles[static_cast<int>(axis_t::Y_AXIS)].end()) && 
                            (_axis_possibles[static_cast<int>(axis_t::Z_AXIS)].find(p) != _axis_possibles[static_cast<int>(axis_t::Z_AXIS)].end()))
                        {
                            _possibles.insert(p);
                        }
                    }
                    break;
                case axis_t::Y_AXIS :
                    for (auto p : _axis_possibles[static_cast<int>(axis_t::X_AXIS)])
                    {
                        if ((_axis_possibles[static_cast<int>(axis_t::X_AXIS)].find(p) != _axis_possibles[static_cast<int>(axis_t::X_AXIS)].end()) && 
                            (_axis_possibles[static_cast<int>(axis_t::Z_AXIS)].find(p) != _axis_possibles[static_cast<int>(axis_t::Z_AXIS)].end()))
                        {
                            _possibles.insert(p);
                        }
                    }
                    break;
                case axis_t::Z_AXIS :
                    for (auto p : _axis_possibles[static_cast<int>(axis_t::X_AXIS)])
                    {
                        if ((_axis_possibles[static_cast<int>(axis_t::X_AXIS)].find(p) != _axis_possibles[static_cast<int>(axis_t::X_AXIS)].end()) && 
                            (_axis_possibles[static_cast<int>(axis_t::Y_AXIS)].find(p) != _axis_possibles[static_cast<int>(axis_t::Y_AXIS)].end()))
                        {
                            _possibles.insert(p);
                        }
                    }
                    break;
            }

            return *this;
        }

        void collision_detect_axis(const axis_t axis)
        {
            /* For all bounds */
            const int axis_int = static_cast<int>(axis);
            for (unsigned int i = 1; i < (_bounds[axis_int].size() - 1); ++i)
            {
                assert(_bounds[axis_int][i]->object() != nullptr);

                /* That are minimums */
                if (_bounds[axis_int][i]->min())
                {
                    /* There may be collisions between this object and any found before the maximum */
                    int j = i + 1;
                    while ((_bounds[axis_int][i]->object() != _bounds[axis_int][j]->object()) && (j < static_cast<int>(_bounds[axis_int].size())))
                    {
                        _axis_possibles[axis_int].insert(_bounds[axis_int][i]->object(), _bounds[axis_int][j]->object());
                        ++j;
                    }
                }
            }
        }

        void update_axis(const physics_object &po, const axis_t axis)
        {
            /* Get object bounds */
            const object_bound *const lower_bound = po.lower_bound(axis);
            const object_bound *const upper_bound = po.upper_bound(axis);

            /* Adjust bounds tracking changes in collision set */
            /* Moving lower bound down */
            const int axis_int = static_cast<int>(axis);
            while (_bounds[axis_int][lower_bound->index()]->compare_and_swap(_bounds[axis_int][lower_bound->index() - 1]))
            {
                std::swap(_bounds[axis_int][lower_bound->index()], _bounds[axis_int][lower_bound->index() + 1]);
                update_possibles(_bounds[axis_int][lower_bound->index()], _bounds[axis_int][lower_bound->index() + 1], axis);
            }

            /* Moving upper bound down */
            while (_bounds[axis_int][upper_bound->index()]->compare_and_swap(_bounds[axis_int][upper_bound->index() - 1]))
            {
                std::swap(_bounds[axis_int][upper_bound->index()], _bounds[axis_int][upper_bound->index() + 1]);
                update_possibles(_bounds[axis_int][upper_bound->index()], _bounds[axis_int][upper_bound->index() + 1], axis);
            }

            /* Moving upper bound up */
            while (_bounds[axis_int][upper_bound->index() + 1]->compare_and_swap(_bounds[axis_int][upper_bound->index()]))
            {
                std::swap(_bounds[axis_int][upper_bound->index() - 1], _bounds[axis_int][upper_bound->index()]);
                update_possibles(_bounds[axis_int][upper_bound->index() - 1], _bounds[axis_int][upper_bound->index()], axis);
            }

            /* Moving lower bound up */
            while (_bounds[axis_int][lower_bound->index() + 1]->compare_and_swap(_bounds[axis_int][lower_bound->index()]))
            {
                std::swap(_bounds[axis_int][lower_bound->index() - 1], _bounds[axis_int][lower_bound->index()]);
                update_possibles(_bounds[axis_int][lower_bound->index() - 1], _bounds[axis_int][lower_bound->index()], axis);
            }
        }

        void update_possibles(const object_bound *const moving_lo, const object_bound *const moving_hi, const axis_t axis)
        {
            const int axis_int = static_cast<int>(axis);
            if (moving_lo->min() & (!moving_hi->min()))
            {
                const std::pair<const physics_object*, const physics_object*> *pair = _axis_possibles[axis_int].insert(moving_lo->object(), moving_hi->object());

                switch (axis)
                {
                    case axis_t::X_AXIS :
                        if ((_axis_possibles[static_cast<int>(axis_t::Y_AXIS)].find(*pair) != _axis_possibles[static_cast<int>(axis_t::Y_AXIS)].end()) && 
                            (_axis_possibles[static_cast<int>(axis_t::Z_AXIS)].find(*pair) != _axis_possibles[static_cast<int>(axis_t::Z_AXIS)].end()))
                        {
                            _possibles.insert(*pair);
                        }
                        break;
                    case axis_t::Y_AXIS :
                        if ((_axis_possibles[static_cast<int>(axis_t::X_AXIS)].find(*pair) != _axis_possibles[static_cast<int>(axis_t::X_AXIS)].end()) && 
                            (_axis_possibles[static_cast<int>(axis_t::Z_AXIS)].find(*pair) != _axis_possibles[static_cast<int>(axis_t::Z_AXIS)].end()))
                        {
                            _possibles.insert(*pair);
                        }
                        break;
                    case axis_t::Z_AXIS :
                        if ((_axis_possibles[static_cast<int>(axis_t::X_AXIS)].find(*pair) != _axis_possibles[static_cast<int>(axis_t::X_AXIS)].end()) && 
                            (_axis_possibles[static_cast<int>(axis_t::Y_AXIS)].find(*pair) != _axis_possibles[static_cast<int>(axis_t::Y_AXIS)].end()))
                        {
                            _possibles.insert(*pair);
                        }
                        break;
                }
            }
            else if ((!moving_lo->min()) & moving_hi->min())
            {
                _axis_possibles[axis_int].erase(moving_lo->object(), moving_hi->object());
                _possibles.erase(moving_lo->object(), moving_hi->object());
            }
        }
        std::vector<object_bound *> _bounds[3];
        collision_set               _axis_possibles[3];
        collision_set               _possibles;
        object_bound *              _min_sentinel;
        object_bound *              _max_sentinel;
};
}; /* namespace raptor_physics */
