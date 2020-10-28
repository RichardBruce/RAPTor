#pragma once

/* Parser headers */
#include "parser.h"

/* Ray tracer headers */
#include "material.h"
#include "primitive_store.h"


namespace raptor_raytracer
{
class camera;

/* Mapper enums */
enum class mapper_type_t : char { non = 0, f_noise = 1, planar = 2, cubic = 3, spherical = 4, cylindrical = 5, f_checker = 6, f_honeycomb = 7, map_grad = 8 };
enum class mapper_of_t   : char { map_btex = 0, map_ctex = 1, map_dtex = 2, map_ltex = 3, map_stex = 4, map_rtex = 5, map_ttex = 6 };

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

inline std::uint32_t parse_vx(const char **at)
{
    std::uint32_t idx = raptor_parsers::from_byte_stream<std::uint16_t>(at);
    if ((idx & 0xff00) == 0xff00)
    {
        idx &= 0x00ff;
        idx <<= 16;
        idx += raptor_parsers::from_byte_stream<std::uint16_t>(at);
    }

    return idx;
}

inline const char * parse_string(const char **at)
{
    /* Remember the string */
    const char *str = (*at);
    
    /* Update the pointer */
    int offset = strlen(str) + 1;
    offset += offset & 0x1;
    (*at) += offset;

    return str;
}

void lwo_parser(
    std::ifstream         &lwo_file,
    std::string           &p,
    light_list            &l, 
    primitive_store       &e,
    std::list<material *> &m,
    camera                **c);
}; /* namespace raptor_raytracer */
