#pragma once

namespace raptor_raytracer
{
class camera;

void nff_parser(
    std::ifstream           &nff_file,
    light_list              &l, 
    primitive_store         &e,
    std::list<material *>   &m,
    camera                  **c);
}; /* namespace raptor_raytracer */
