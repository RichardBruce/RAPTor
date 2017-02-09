#include "common.h"
#include "parser_common.h"
#include "polygon_to_triangles.h"

#include "camera.h"

#include "ply_parser.h"

/* Shaders */
#include "phong_shader.h"


namespace raptor_raytracer
{
/**********************************************************
 next_statement finds the next statement that can be 
 parsed. comment and obj_info statements are ignores.

 c is a pointer to a byte stream before finding the next
 statement and a pointer to the next statement is 
 returned.
**********************************************************/
const char * next_statement(const char *c)
{
    do
    {
        find_next_line(&c);
    } while ((strncmp(c, "comment ", 8) == 0) || (strncmp(c, "obj_info ", 9) == 0));
    
    return c;
}


/**********************************************************
 parse_element_vertex parses the vertex element of the 
 header. It mainly check the syntax is what is expected, but
 also checks the number of bytes that should be skipped 
 after each vertex. Bytes that are skipped contain 
 information that cannot be used.

 c is a pointer to a bytes stream. skip is the number of 
 bytes to be skipped after each vertex. The number of 
 vertices in the file is returned.
**********************************************************/
unsigned parse_element_vertex(const char **c, unsigned *const skip, bool *const normal, bool *const colour)
{
    /* Get the number of vertices */
    assert(strncmp(*c, "element vertex ", 15) == 0);
    (*c) += 14;
    unsigned v = get_next_unsigned(c);

    /* Check they are x, y, z vertices */
    (*c) = next_statement(*c);
    assert((strncmp(*c, "property float x", 16) == 0) || (strncmp(*c, "property float32 x", 18) == 0));

    (*c) = next_statement(*c);
    assert((strncmp(*c, "property float y", 16) == 0) || (strncmp(*c, "property float32 y", 18) == 0));

    (*c) = next_statement(*c);
    assert((strncmp(*c, "property float z", 16) == 0) || (strncmp(*c, "property float32 z", 18) == 0));
    
    /* Discard other data */
    (*c) = next_statement(*c);
    while (strncmp(*c, "property ", 9) == 0)
    {
        /* If there is a red colour componant to the vertex */
        if (strncmp(*c, "property uchar red", 18) == 0)
        {
            /* If there is a green colour componant to the vertex */
            (*c) = next_statement(*c);
            if (strncmp(*c, "property uchar green", 20) == 0)
            {
                /* If there is a blue colour componant to the vertex */
                (*c) = next_statement(*c);
                if (strncmp(*c, "property uchar blue", 19) == 0)
                {
                    (*colour) = true;
                    (*c) = next_statement(*c);
                }
                /* Increment skip for red and blue that wont be parsed */
                else
                {
                    (*skip) += 2;
                }
            }
            /* Increment skip for red that wont be parsed */
            else
            {
                ++(*skip);
            }

            /* Check the rest of the loop should be executed */
            if (strncmp(*c, "property ", 9) != 0)
            {
                break;
            }
        }
        
        /* If there is a nx componant to the vertex */
        if (strncmp(*c, "property float nx", 17) == 0)
        {
            /* If there is a ny componant to the vertex */
            (*c) = next_statement(*c);
            if (strncmp(*c, "property float ny", 17) == 0)
            {
                /* If there is a nz componant to the vertex */
                (*c) = next_statement(*c);
                if (strncmp(*c, "property float nz", 17) == 0)
                {
                    (*normal) = true;
                    (*c) = next_statement(*c);
                }
                /* Increment skip for nx and ny that wont be parsed */
                else
                {
                    (*skip) += 2;
                }
            }
            /* Increment skip for nx that wont be parsed */
            else
            {
                ++(*skip);
            }

            /* Check the rest of the loop should be executed */
            if (strncmp(*c, "property ", 9) != 0)
            {
                break;
            }
        }
            
        if ((strncmp((*c) + 9, "uchar ", 6) == 0) || (strncmp((*c) + 9, "uint8 ", 6) == 0))
        {
            ++(*skip);
        }
        else if ((strncmp((*c) + 9, "float ", 6) == 0) || (strncmp((*c) + 9, "float32 ", 8) == 0))
        {
            (*skip) += 4;
        }
        else
        {
            assert(false);
        }
        (*c) = next_statement(*c);
    }
   
    return v;
}


/**********************************************************
 parse_element_face parses the face element of the header. 
 It mainly check the syntax is what is expected, but
 also checks the number of bytes that should be skipped 
 before each face. Bytes that are skipped contain 
 information that cannot be used.

 c is a pointer to a bytes stream. skip is the number of 
 bytes to be skipped before each face. The number of 
 faces in the file is returned. 
**********************************************************/
unsigned parse_element_face(const char **c, unsigned *const pre_skip, unsigned *const post_skip, bool *const colour)
{
    /* Get the number of vertices */
    assert(strncmp(*c, "element face ", 13) == 0);
    (*c) += 12;
    unsigned f = get_next_unsigned(c);
    
    /* Check for extra data before the point list */
    (*c) = next_statement(*c);
    if (strncmp(*c, "property uchar intensity", 24) == 0)
    {
        (*pre_skip) = 1;
        (*c) = next_statement(*c);
    }
    
    /* Check they are vertex list faces */
    assert((strncmp(*c, "property list uchar int vertex_indices", 38) == 0) || (strncmp(*c, "property list uint8 int32 vertex_indices", 40) == 0));

    /* Discard other data */
    (*c) = next_statement(*c);
    while (strncmp(*c, "property ", 9) == 0)
    {
        /* If there is a red colour componant to the face */
        if (strncmp(*c, "property uchar red", 18) == 0)
        {
            /* If there is a green colour componant to the face */
            (*c) = next_statement(*c);
            if (strncmp(*c, "property uchar green", 20) == 0)
            {
                /* If there is a blue colour componant to the face */
                (*c) = next_statement(*c);
                if (strncmp(*c, "property uchar blue", 19) == 0)
                {
                    (*colour) = true;
                    (*c) = next_statement(*c);
                }
                /* Increment skip for red and blue that wont be parsed */
                else
                {
                    (*post_skip) += 2;
                }
            }
            /* Increment skip for red that wont be parsed */
            else
            {
                (*post_skip)++;
            }

            /* Check the rest of the loop should be executed */
            if (strncmp(*c, "property ", 9) != 0)
            {
                break;
            }
        }
        
        
        if ((strncmp((*c) + 9, "uchar ", 6) == 0) || (strncmp((*c) + 9, "uint8 ", 6) == 0))
        {
            (*post_skip)++;
        }
        else
        {
            assert(false);
        }
        (*c) = next_statement(*c);
    }
    
    return f;
}


/**********************************************************
 parse_binary_face parses face information in binary 
 format. The face is assumed to be describred by a char 
 giving the number of vertices in the face and then that
 number of integers referring to the vertices of the face.
 
 l is a list of lights in the scene, e is a list of all 
 primitives in the scene. vn is a list of vertex normals.
 v is a list of vertex locations. m is the current 
 material in use. c is the byte stream to be parsed.
**********************************************************/
void parse_binary_face(light_list *l, primitive_store *e, std::vector<point_t> &vn, std::vector<point_t> &v, std::map<int, material *> *const shader_map, const char **c, const bool colour)
{
    static std::vector<point_t> face;
    
    /* Parse all vextex data for the face */
    unsigned char v_this_f = from_byte_stream<unsigned char>(c);
    face.resize(v_this_f);
    for (unsigned char i = 0; i < v_this_f; ++i)
    {
        /* Parse vertex */
        int vert_num = from_byte_stream<int>(c);
        assert(vert_num < (int)v.size());
        assert(vert_num >= 0);
        face[i] = v[vert_num];
    }
    
    /* Parse optional colour information */
    material *m;
    if (colour)
    {
        /* Create the colours name from it data */
        int name = (*c)[0] + ((*c)[1] << 8) + ((*c)[2] << 16);
        
        /* If the colour doesnt exsist add it */
        if (shader_map->find(name) == shader_map->end())
        {
            ext_colour_t rgb((unsigned char)((*c)[0]), (unsigned char)((*c)[1]), (unsigned char)((*c)[2]));
            m = new phong_shader(ext_colour_t(0.0, 0.0, 0.0), rgb, ext_colour_t(0.0, 0.0, 0.0), 0.0);
            (*shader_map)[name] = m;
        }
        else
        {
            m = (*shader_map)[name];
        }

        (*c) += 3;
    }
    /* Use the default material */
    else
    {
        m = (*shader_map)[0xdefa];
    }

    /* Create the polygon */
    if (face.size() == 3)
    {
        /* Check the points show movement in atleast 2 axes  */
        point_t a = face[0];
        point_t b = face[1];
        point_t c = face[2];

        if ((a != b) && (a != c) && (b != c))
        {
            a -= b;
            b -= c;
            normalise(&a);
            normalise(&b);
        
            c  = a - b;
            a += b;
    
            if ((c != 0) && (a != 0))
            {
                new_triangle(e, NULL, m, face[0], face[1], face[2], false);
            }
        }
    }
    else
    {
        face_to_triangles(e, l, face, m, false);
    }
}


/**********************************************************
 parse_face parses face information in ascii format. 
 The face is assumed to be describred by a number giving 
 the number of vertices in the face and then that number 
 of integers referring to the vertices of the face.
 
 l is a list of lights in the scene, e is a list of all 
 primitives in the scene. vn is a list of vertex normals.
 v is a list of vertex locations. m is the current 
 material in use. c is the byte stream to be parsed.
**********************************************************/
void parse_face(light_list *l, primitive_store *e, std::vector<point_t> &vn, std::vector<point_t> &v, std::map<int, material *> *const shader_map, const char **c, const bool colour)
{
    static std::vector<point_t> face;

    /* Parse all vextex data for the face */
    unsigned v_this_f = atoi(*c);
    face.resize(v_this_f);
    for (unsigned i = 0; i < v_this_f; i++)
    {
        /* Parse vertex */
        unsigned vert_num = get_next_unsigned(c);
        assert(vert_num < v.size());
        face[i] = v[vert_num];
    }
    find_next_line(c);

    /* Parse optional colour information */
    material *m;
    if (colour)
    {
        /* let me know when this code is entered it is untested */
        assert(false);

        /* Create the colours name from it data */
        int name = (*c)[0] + ((*c)[1] << 8) + ((*c)[2] << 16);
        
        /* If the colour doesnt exsist add it */
        if (shader_map->find(name) == shader_map->end())
        {
            ext_colour_t rgb((unsigned char)((*c)[0]), (unsigned char)((*c)[1]), (unsigned char)((*c)[2]));
            (*shader_map)[name] = new phong_shader(ext_colour_t(0.0, 0.0, 0.0), rgb, ext_colour_t(0.0, 0.0, 0.0), 0.0);
        }
        
        m = (*shader_map)[name];
        (*c) += 3;
    }
    /* Use the default material */
    else
    {
        m = (*shader_map)[0xdefa];
    }

    /* Create the polygon */
    if (face.size() == 3)
    {
        /* Check the points show movement in atleast 2 axes  */
        point_t a = face[0];
        point_t b = face[1];
        point_t c = face[2];

        if ((a != b) && (a != c) && (b != c))
        {
            a -= b;
            b -= c;
            normalise(&a);
            normalise(&b);
        
            c  = a - b;
            a += b;
    
            if ((c != 0) && (a != 0))
            {
                new_triangle(e, NULL, m, face[0], face[1], face[2], false);
            }
        }
    }
    else
    {
        face_to_triangles(e, l, face, m, false);
    }
}


/**********************************************************
 parse_binary_vertex parses vertex information in binary 
 format. The vertex is assumed to be describred 3 floats.

 vs is a list of vertices to be appended too and c is a 
 pointer to the byte stream to be parsed.
**********************************************************/
void parse_binary_vertex(std::vector<point_t> *const vs, std::vector<point_t> *const vn, const char **c, const bool normal, const bool colour)
{
    point_t v;

    /* Parse the vertex position */
    v.x = from_byte_stream<float>(c);
    v.y = from_byte_stream<float>(c);
    v.z = from_byte_stream<float>(c);
    vs->push_back(v);

    /* Ignore the vertex colour */
    if (colour)
    {
        (*c) += 3;
    }

    /* Parse the vertex normal */
    if (normal)
    {
        v.x = from_byte_stream<float>(c);
        v.y = from_byte_stream<float>(c);
        v.z = from_byte_stream<float>(c);
        vn->push_back(v);
    }
}


/**********************************************************
 parse_vertex parses vertex information in ascii format. 
 The vertex is assumed to be describred 3 floats.

 vs is a list of vertices to be appended too and c is a 
 pointer to the byte stream to be parsed.
**********************************************************/
void parse_vertex(std::vector<point_t> *const vs, std::vector<point_t> *const vn, const char **c, const bool normal, const bool colour)
{
    point_t v;

    /* Parse the vertex position */
    v.x = atof(*c);
    v.y = get_next_float(c);
    v.z = get_next_float(c);
    vs->push_back(v);
    
    /* Parse the vertex normal */
    if (normal)
    {
        v.x = get_next_float(c);
        v.y = get_next_float(c);
        v.z = get_next_float(c);
        vn->push_back(v);
    }

    /* Move to the next line */
    find_next_line(c);
}


/**********************************************************
 ply_parser is the main ply parsing function. It loads the 
 ply file into a byte array and parses through it. The full
 ply standard isnt supported only enough to load the most 
 common models, namely vertex and face data.
**********************************************************/
void ply_parser(
    std::ifstream           &ply_file,
    light_list              &l, 
    primitive_store         &e,
    std::list<material *>   &m,
    camera                  **c)
{
    /* Find the size of the file */
    ply_file.seekg(0, std::ios::end);
    size_t len = ply_file.tellg();
    ply_file.seekg(0, std::ios::beg);

//    std::cout << "Parsing PLY length: " << len << std::endl;
       
    /* Read the whole file into a buffer */
    char *buffer = new char [len];
    ply_file.read((char *)buffer, len);
    const char *at = &buffer[0];
    
    /* Vectors of vertice data */
    std::vector<point_t>    vn;
    std::vector<point_t>    v;
    
    /* Map of shader names to shader */
    std::map<int, material *> shader_map;
    shader_map[0xdefa] = new phong_shader(ext_colour_t(0.0f, 0.0f, 0.0f), ext_colour_t(200.0f, 200.0f, 200.0f), ext_colour_t(0.0f, 0.0f, 0.0f), 0.0f);
    
    /* Check this is a PLY file */
    assert(strncmp(at, "ply", 3) == 0);
    at = next_statement(at);
    
    /* Check format */
    bool binary;
    if (strncmp(at, "format ascii 1.0", 16) == 0)
    {
        binary = false;
    }
    else
    {
        assert(strncmp(at, "format binary_big_endian 1.0", 28) == 0);
        binary = true;
    }
    at = next_statement(at);

    /* Get the vertices information */
    unsigned v_skip = 0;
    bool     v_colour       = false;
    bool     v_normal       = false;
    unsigned nr_v = parse_element_vertex(&at, &v_skip, &v_normal, &v_colour);
    v.reserve(nr_v);
    
    if (v_normal)
    {
        vn.reserve(nr_v);
    }

    /* Get the face information */
    unsigned pre_f_skip     = 0;
    unsigned post_f_skip    = 0;
    bool     f_colour       = false;
    unsigned nr_f = parse_element_face(&at, &pre_f_skip, &post_f_skip, &f_colour);

    assert(strncmp(at, "end_header", 10) == 0);
    at = next_statement(at);

    /* Pick the file type and parse the data */
    if (binary)
    {
        /* Parse vertices */
        for (unsigned i = 0; i < nr_v; i++)
        {
            parse_binary_vertex(&v, &vn, &at, v_normal, v_colour);
            at += v_skip;
        }
    
        /* Parse faces */
        for (unsigned i = 0; i < nr_f; i++)
        {
            at += pre_f_skip;
            parse_binary_face(&l, &e, vn, v, &shader_map, &at, f_colour);
            at += post_f_skip;
        }    
    }
    else
    {
        /* Parse vertices */
        for (unsigned i = 0; i < nr_v; i++)
        {
            parse_vertex(&v, &vn, &at, v_normal, v_colour);
        }
    
        /* Parse faces */
        for (unsigned i = 0; i < nr_f; i++)
        {
            at += pre_f_skip;
            parse_face(&l, &e, vn, v, &shader_map, &at, f_colour);
        }
    }

 
    /* Add the materials to the list of materials */
    for (auto i = shader_map.begin(); i != shader_map.end(); ++i)
    {
        m.push_back(i->second);
    }
 
    /* Tidy up */
    delete [] buffer;
//    std::cout << "Parsing complete" << std::endl;
}
}; /* namespace raptor_raytracer */
