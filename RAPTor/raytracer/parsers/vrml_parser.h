#ifndef __VRML_PARSER_H__
#define __VRML_PARSER_H__

namespace raptor_raytracer
{
class camera;

void vrml_parser(
    std::ifstream           &vrml_file,
    light_list              &l, 
    primitive_list          &e,
    std::list<material *>   &m,
    camera                  *c,
    const std::string       &v);
}; /* namespace raptor_raytracer */

#endif /* #ifndef __VRML_PARSER_H__ */
