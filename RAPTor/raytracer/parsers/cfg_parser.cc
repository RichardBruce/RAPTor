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


/**********************************************************
 next_statement finds the next statement that can be 
 parsed. # means comment and is ignored.

 c is a pointer to a byte stream before finding the next
 statement and a pointer to the next statement is 
 returned.
**********************************************************/
const char * cfg_next_statement(const char *c)
{
    c--;
    do
    {
        find_next_line(&c);
    } while ((c[0] == '#') || (c[0] == '\n'));
    
    return c;
}


void cfg_parser(
    ifstream            &cfg_file,
    light_list          &l, 
    primitive_list      &e,
    list<material *>    &m,
    camera              **c)
{
    /* Find the size of the file */
    cfg_file.seekg(0, ios::end);
    size_t len = cfg_file.tellg();
    cfg_file.seekg(0, ios::beg);

//    cout << "Parsing CFG length: " << len << endl;
       
    /* Read the whole file into a buffer */
    char *buffer = new char [len];
    cfg_file.read((char *)buffer, len);
    const char *at = &buffer[0];
    
    /* Statement hash */
    map<string, model_format_t> format;
    format["cfg"]   = cfg;
    format["code"]  = code;
    format["mgf"]   = mgf;
    format["nff"]   = nff;
    format["lwo"]   = lwo;
    format["obj"]   = obj;
    format["ply"]   = ply;
    
    while (true)
    {
        at = cfg_next_statement(at);
        if (at >= &buffer[len])
        {
            break;
        }

        string statement = get_this_string(&at);
        
        map<string, model_format_t>::const_iterator i = format.find(statement);
//        cout << "parsing : " << statement << endl;
        if (i == format.end())
        {
            cout << "Error: Unknown statement: " << statement << endl;
            assert(false);
        }
        
        ifstream    input_stream;
        string      path;
        size_t      last_slash;
        string input_file = get_this_string(&at);
//        cout << "path: " << input_file << endl;
        switch ((*i).second)
        {
            case cfg :
                input_stream.open(input_file.c_str());
                assert(input_stream.is_open());
                cfg_parser(input_stream, l, e, m, c);
                break;

            case code :
                scene_init(l, e, m, c);
                break;

            case mgf :
                mgf_parser(input_file.c_str(), l, e, m);
                break;

            case nff :
                input_stream.open(input_file.c_str());
                assert(input_stream.is_open());
                nff_parser(input_stream, l, e, m, c);
                break;

            case lwo :
                input_stream.open(input_file.c_str());
                assert(input_stream.is_open());
                last_slash  = input_file.find_last_of('/');
                path        = input_file.substr(0, last_slash + 1);
                lwo_parser(input_stream, path, l, e, m, c);
                break;
            
            case obj :
                input_stream.open(input_file.c_str());
                assert(input_stream.is_open());
                last_slash  = input_file.find_last_of('/');
                path        = input_file.substr(0, last_slash + 1);
                obj_parser(input_stream, path, l, e, m, c);
                break;

            case ply :
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
