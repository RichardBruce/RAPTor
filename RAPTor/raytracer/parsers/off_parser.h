#ifndef __OFF_PARSER_H__
#define __OFF_PARSER_H__

namespace raptor_raytracer
{
class camera;

void off_parser(
    std::ifstream           &off_file,
    light_list              &l, 
    primitive_list          &e,
    std::list<material *>   &m,
    camera                  *c);
}; /* namespace raptor_raytracer */

#endif /* #ifndef __OFF_PARSER_H__ */
