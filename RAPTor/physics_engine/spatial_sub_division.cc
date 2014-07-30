#include "spatial_sub_division.h"

namespace raptor_physics
{
spatial_sub_division& spatial_sub_division::remove_from_possible(const physics_object *const po)
{
    for(auto it = _possibles.begin(); it != _possibles.end(); )
    {
        if (((it->first == po) | (it->second == po)) && (!is_possible(*it)))
        {
            it = _possibles.erase(it);
        }
        else
        {
            ++it;
        }
    }

    return *this;
}
}
