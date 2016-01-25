#pragma once

namespace raptor_convex_decomposition
{
enum class discretisation_type_t : char { voxel = 0, tetrahedron = 1 };

inline std::istream& operator>>(std::istream& in, discretisation_type_t &mode)
{
    std::string token;
    in >> token;
    if (token == "voxel")
    {
        mode = discretisation_type_t::voxel;
    }
    else
    {
        mode = discretisation_type_t::tetrahedron;
    }

    return in;
}


inline std::ostream& operator<<(std::ostream& out, const discretisation_type_t &mode)
{
    switch (mode)
    {
        case discretisation_type_t::voxel :
            out << "voxel";
            break;
        case discretisation_type_t::tetrahedron :
            out << "tetrahedron";
            break;
    }

    return out;
}
}; /* namespace raptor_convex_decomposition */
