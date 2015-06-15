#pragma once

namespace raptor_raytracer
{
class camera;

void off_parser(
    std::ifstream           &off_file,
    light_list              &l, 
    primitive_store         &e,
    std::list<material *>   &m,
    camera                  *c);
}; /* namespace raptor_raytracer */
