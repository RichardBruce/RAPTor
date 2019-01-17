#pragma once


namespace raptor_physics
{
/* Enum to show the different ways objects can connect */
/* Possible is used for rotating object were only a conservative result can be given */
enum class collision_t : char { NO_COLLISION = 0, SLIDING_COLLISION = 1, COLLISION = 2, POSSIBLE_SLIDING_COLLISION = 5, POSSIBLE_COLLISION = 6 };

/* Check for a POSSIBLY_ prefix */
inline bool is_uncertain(const collision_t c)
{
    return (static_cast<int>(c) & 0x4);
}

/* Convert to the certain form */
inline collision_t to_certain(const collision_t c)
{
    return static_cast<collision_t>(static_cast<int>(c) & 0x3);
}
}; /* namespace raptor_physics */
