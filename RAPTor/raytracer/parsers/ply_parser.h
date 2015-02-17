#ifndef __PLY_PARSER_H__
#define __PLY_PARSER_H__

namespace raptor_raytracer
{
class camera;

void ply_parser(
    std::ifstream           &ply_file,
    light_list              &l, 
    primitive_list          &e,
    std::list<material *>   &m,
    camera                  **c);
}; /* namespace raptor_raytracer */

#endif	/* #ifndef __PLY_PARSER_H__ */
