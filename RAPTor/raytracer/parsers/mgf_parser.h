#pragma once

#include "common.h"
#include "parser_common.h"
#include "cook_torrance_cxy.h"

namespace raptor_raytracer
{
void mgf_parser(
    const char              *mgf_file,
    light_list              &l, 
    primitive_store         &e,
    std::list<material *>   &m);
}; /* namespace raptor_raytracer */
