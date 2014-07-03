#ifndef __LWO_PARSER_H__
#define __LWO_PARSER_H__

namespace raptor_raytracer
{
class camera;

void lwo_parser(
    ifstream            &lwo_file,
    string              &p,
    light_list          &l, 
    primitive_list      &e,
    list<material *>    &m,
    camera              **c);
}; /* namespace raptor_raytracer */

#endif /* #ifndef __LWO_PARSER_H__ */
