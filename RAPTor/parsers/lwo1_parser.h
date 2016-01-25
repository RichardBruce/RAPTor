#pragma once

namespace raptor_raytracer
{
class camera;

const char * lwo1_parser(
    const char *const       begin,
    const char              *at,
    std::string             &p,
    light_list              &l, 
    primitive_store         &e,
    std::list<material *>   &m,
    camera                  **c);
}; /* namespace raptor_raytracer */
