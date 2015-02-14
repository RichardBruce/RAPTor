/* Standard headers */
#include <cstdint>

/* Common headers */
#include "common.h"
#include "logging.h"

/* Ray tracer headers */
#include "parser_common.h"
#include "lwo_parser.h"
#include "lwo1_parser.h"
#include "lwo2_parser.h"
#include "camera.h"


namespace raptor_raytracer
{
void lwo_parser(
    std::ifstream           &lwo_file,
    std::string             &p,
    light_list              &l, 
    primitive_list          &e,
    std::list<material *>   &m,
    camera                  **c)
{
    METHOD_LOG;
    
    assert(sizeof(float) == 4);
    
    /* Find the size of the file */
    lwo_file.seekg(0, std::ios::end);
    size_t len = lwo_file.tellg();
    lwo_file.seekg(0, std::ios::beg);
    
    /* Read the whole file into a buffer */
    BOOST_LOG_TRIVIAL(trace) << "Reading file of size: " << len;
    char *buffer = new char [len];
    lwo_file.read(buffer, len);
    const char *at = &buffer[0];

    /* Check that the first 4 characters are "FORM" */
    check_for_chunk(&at, "FORM", 4);
    
    /* Check the declared number of bytes matchs the actual */
    std::uint32_t file_len = from_byte_stream<std::uint32_t>(&at);
    if ((file_len + 8) != len)
    {
        BOOST_LOG_TRIVIAL(error) << "File length is " << len << " where " << (file_len + 8) << " expected";
        assert(false);
    }

    /* Check for LWO2 file */
    if (strncmp(at, "LWO2", 4) == 0)
    {
        at += 4;
        at = lwo2_parser(buffer, at, p, l, e, m, c);
    }
    else
    {
        /* Check the for LWO file */
        check_for_chunk(&at, "LWOB", 4);
        at = lwo1_parser(buffer, at, p, l, e, m, c);
    }

    /* Check we read the whole file */
    BOOST_LOG_TRIVIAL(trace) << "Now at file position: " << (at - buffer);
    assert((static_cast<size_t>(at - buffer) >= len) || !"Error: The whole file was not parsed");

    /* Clean up */
    delete [] buffer;
}
}; /* namespace raptor_raytracer */
