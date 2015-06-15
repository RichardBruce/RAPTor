#pragma once

namespace raptor_raytracer
{
class camera;

void cfg_parser(
    const std::string       &base_path,
    std::ifstream           &cfg_file,
    light_list              &l, 
    primitive_store         &e,
    std::list<material *>   &m,
    camera                  **c);
}; /* namespace raptor_raytracer */
