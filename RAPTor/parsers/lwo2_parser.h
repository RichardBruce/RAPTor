#pragma once

namespace raptor_raytracer
{
class camera;

void lwo2_parser(
    const char *const       begin,
    const char              *at,
    std::string             &p,
    light_list              &l, 
    primitive_store         &e,
    std::list<material *>   &m,
    camera                  **c,
    const int               len);
}; /* namespace raptor_raytracer */
