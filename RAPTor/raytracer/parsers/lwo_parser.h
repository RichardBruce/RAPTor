#ifndef __LWO_PARSER_H__
#define __LWO_PARSER_H__

namespace raptor_raytracer
{
class camera;

/***********************************************************
  check_for_chunk checks that the pointer a points at the 
  same chunk name as the pointer e with length l.
  
  If the comparison is correct a is updated past the chunk
  name otherwise the program is aborted through assert()
************************************************************/
inline void check_for_chunk(const char **const a, const char *const e, const int l, const int line = 0)
{
    if (strncmp(*a, e, l) != 0)
    {
        BOOST_LOG_TRIVIAL(error) << (*a) << " found where " << e << " chunk expected at line: " << line;
        assert(false);
    }
    (*a) += l;
}


void lwo_parser(
    ifstream            &lwo_file,
    string              &p,
    light_list          &l, 
    primitive_list      &e,
    list<material *>    &m,
    camera              **c);
}; /* namespace raptor_raytracer */

#endif /* #ifndef __LWO_PARSER_H__ */
