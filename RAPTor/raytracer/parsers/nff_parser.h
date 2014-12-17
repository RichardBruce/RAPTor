#ifndef __NFF_PARSER_H__
#define __NFF_PARSER_H__

namespace raptor_raytracer
{
class camera;

void nff_parser(
    std::ifstream           &nff_file,
    light_list              &l, 
    primitive_list          &e,
    std::list<material *>   &m,
    camera                  **c);
}; /* namespace raptor_raytracer */

#endif /* #ifndef __NFF_PARSER_H__ */
