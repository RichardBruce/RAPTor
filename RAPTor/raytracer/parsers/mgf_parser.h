#ifndef __MGF_PARSER_H__
#define __MGF_PARSER_H__

#include "common.h"
#include "parser_common.h"
#include "cook_torrance_cxy.h"

void mgf_parser(
    const char         *mgf_file,
    light_list          &l, 
    primitive_list      &e,
    list<material *>    &m);

#endif /* #ifndef __MGF_PARSER_H__ */
