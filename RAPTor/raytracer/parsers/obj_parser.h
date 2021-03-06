#pragma once

namespace raptor_raytracer
{
class camera;

void obj_parser(
    std::ifstream           &obj_file,
    std::string             p,
    light_list              &l, 
    primitive_store         &e,
    std::list<material *>   &m,
    camera                  **c);
}; /* namespace raptor_raytracer */
