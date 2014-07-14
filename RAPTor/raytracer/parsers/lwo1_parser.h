#ifndef __LWO1_PARSER_H__
#define __LWO1_PARSER_H__

namespace raptor_raytracer
{
class camera;

const char * lwo1_parser(
    const char *const   begin,
    const char         *at,
    string              &p,
    light_list          &l, 
    primitive_list      &e,
    list<material *>    &m,
    camera              **c);
}; /* namespace raptor_raytracer */

#endif /* #ifndef __LWO1_PARSER_H__ */
