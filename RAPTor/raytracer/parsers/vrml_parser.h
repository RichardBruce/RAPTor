#pragma once

namespace raptor_raytracer
{
class camera;

void vrml_parser(
    std::ifstream           &vrml_file,
    light_list              &l, 
    primitive_store         &e,
    std::list<material *>   &m,
    camera                  *c,
    const std::string       &v);
}; /* namespace raptor_raytracer */
