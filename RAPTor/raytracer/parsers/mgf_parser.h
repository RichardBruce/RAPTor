#ifndef __MGF_PARSER_H__
#define __MGF_PARSER_H__

#include "common.h"
#include "parser_common.h"
#include "cook_torrance_cxy.h"

namespace raptor_raytracer
{
void mgf_parser(
    const char              *mgf_file,
    light_list              &l, 
    primitive_list          &e,
    std::list<material *>   &m);
}; /* namespace raptor_raytracer */

#endif /* #ifndef __MGF_PARSER_H__ */
