#ifndef __VRML_PARSER_H__
#define __VRML_PARSER_H__

namespace raptor_raytracer
{
class camera;

void vrml_parser(
    ifstream            &vrml_file,
    light_list          &l, 
    primitive_list      &e,
    list<material *>    &m,
    camera              *c,
    const string        &v);
}; /* namespace raptor_raytracer */

#endif /* #ifndef __VRML_PARSER_H__ */
