#include "common.h"
#include "parser_common.h"

#include "camera.h"

#include "cfg_parser.h"
#include "mgf_parser.h"
#include "nff_parser.h"
#include "lwo_parser.h"
#include "obj_parser.h"
#include "ply_parser.h"
#include "scene.h"


namespace raptor_raytracer
{
/**********************************************************
 next_statement finds the next statement that can be 
 parsed. # means comment and is ignored.

 c is a pointer to a byte stream before finding the next
 statement and a pointer to the next statement is 
 returned.
**********************************************************/
const char * cfg_next_statement(const char *c, const char *const end)
{
    c--;
    do
    {
        find_next_line(&c);
    } while ((c < end) && ((c[0] == '#') || (c[0] == '\n')));
    
    return c;
}


void cfg_parser(
    const std::string       &base_path,
    std::ifstream           &cfg_file,
    light_list              &l, 
    primitive_list          &e,
    std::list<material *>   &m,
    camera                  **c)
{
    /* Find the size of the file */
    cfg_file.seekg(0, std::ios::end);
    size_t len = cfg_file.tellg();
    cfg_file.seekg(0, std::ios::beg);

//    cout << "Parsing CFG length: " << len << endl;
       
    /* Read the whole file into a buffer */
    char *buffer = new char [len];
    cfg_file.read((char *)buffer, len);
    const char *at = &buffer[0];
    
    /* Statement hash */
    std::map<std::string, model_format_t> format;
    format["cfg"]   = model_format_t::cfg;
    format["code"]  = model_format_t::code;
    format["mgf"]   = model_format_t::mgf;
    format["nff"]   = model_format_t::nff;
    format["lwo"]   = model_format_t::lwo;
    format["obj"]   = model_format_t::obj;
    format["ply"]   = model_format_t::ply;
    
    while (true)
    {
        at = cfg_next_statement(at, &buffer[len - 1]);
        if (at >= &buffer[len - 1])
        {
            break;
        }

        const std::string statement(get_this_string(&at));
        
        auto i = format.find(statement);
//        cout << "parsing : " << statement << endl;
        if (i == format.end())
        {
            std::cout << "Error: Unknown statement: " << statement << std::endl;
            assert(false);
        }
        
        std::ifstream   input_stream;
        std::string     path;
        size_t          last_slash;
        const std::string input_file(base_path + get_this_string(&at));
//        cout << "path: " << input_file << endl;
        switch ((*i).second)
        {
            case model_format_t::cfg :
                input_stream.open(input_file.c_str());
                assert(input_stream.is_open());
                cfg_parser(base_path, input_stream, l, e, m, c);
                break;

            case model_format_t::code :
                scene_init(l, e, m, c);
                break;

            case model_format_t::mgf :
                mgf_parser(input_file.c_str(), l, e, m);
                break;

            case model_format_t::nff :
                std::cout << "opening: " << input_file << std::endl;
                input_stream.open(input_file.c_str());
                assert(input_stream.is_open());
                nff_parser(input_stream, l, e, m, c);
                break;

            case model_format_t::lwo :
                input_stream.open(input_file.c_str());
                assert(input_stream.is_open());
                last_slash  = input_file.find_last_of('/');
                path        = input_file.substr(0, last_slash + 1);
                lwo_parser(input_stream, path, l, e, m, c);
                break;
            
            case model_format_t::obj :
                input_stream.open(input_file.c_str());
                assert(input_stream.is_open());
                last_slash  = input_file.find_last_of('/');
                path        = input_file.substr(0, last_slash + 1);
                obj_parser(input_stream, path, l, e, m, c);
                break;

            case model_format_t::ply :
                input_stream.open(input_file.c_str());
                assert(input_stream.is_open());
                ply_parser(input_stream, l, e, m, c);
                break;

            default :
                assert(false);
                break;        
        }
    }

    /* Tidy up */
    delete [] buffer;
//    cout << "Parsing complete" << endl;
}
}; /* namespace raptor_raytracer */
