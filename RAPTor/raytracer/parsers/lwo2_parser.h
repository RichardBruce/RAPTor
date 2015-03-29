#ifndef __LWO2_PARSER_H__
#define __LWO2_PARSER_H__

namespace raptor_raytracer
{
class camera;

void lwo2_parser(
    const char *const       begin,
    const char              *at,
    std::string             &p,
    light_list              &l, 
    primitive_list          &e,
    std::list<material *>   &m,
    camera                  **c,
    const int               len);
}; /* namespace raptor_raytracer */

#endif /* #ifndef __LWO2_PARSER_H__ */
