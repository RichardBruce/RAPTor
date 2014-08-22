#include <iostream>

#include "spatial_sub_division.h"

namespace raptor_physics
{
void spatial_sub_division::update_possibles(const object_bound *const moving_lo, const object_bound *const moving_hi, const axis_t axis)
{
    if (moving_lo->min() & !moving_hi->min())
    {
        const std::pair<const physics_object*, const physics_object*> *pair = _axis_possibles[axis].insert(moving_lo->object(), moving_hi->object());

        switch (axis)
        {
            case X_AXIS :
                if ((_axis_possibles[Y_AXIS].find(*pair) != _axis_possibles[Y_AXIS].end()) && 
                    (_axis_possibles[Z_AXIS].find(*pair) != _axis_possibles[Z_AXIS].end()))
                {
                    _possibles.insert(*pair);
                }
                break;
            case Y_AXIS :
                if ((_axis_possibles[X_AXIS].find(*pair) != _axis_possibles[X_AXIS].end()) && 
                    (_axis_possibles[Z_AXIS].find(*pair) != _axis_possibles[Z_AXIS].end()))
                {
                    _possibles.insert(*pair);
                }
                break;
            case Z_AXIS :
                if ((_axis_possibles[X_AXIS].find(*pair) != _axis_possibles[X_AXIS].end()) && 
                    (_axis_possibles[Y_AXIS].find(*pair) != _axis_possibles[Y_AXIS].end()))
                {
                    _possibles.insert(*pair);
                }
                break;
        }
    }
    else if (!moving_lo->min() & moving_hi->min())
    {
        _axis_possibles[axis].erase(moving_lo->object(), moving_hi->object());
        _possibles.erase(moving_lo->object(), moving_hi->object());
    }
}
}
