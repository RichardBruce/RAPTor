#ifndef __OBJ_PARSER_H__
#define __OBJ_PARSER_H__

namespace raptor_raytracer
{
class camera;

void obj_parser(
    std::ifstream           &obj_file,
    std::string             p,
    light_list              &l, 
    primitive_list          &e,
    std::list<material *>   &m,
    camera                  **c);
}; /* namespace raptor_raytracer */

#endif	/* #ifndef __OBJ_PARSER_H__ */
