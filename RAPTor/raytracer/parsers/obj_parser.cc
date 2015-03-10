#include "common.h"
#include "parser_common.h"

#include "camera.h"

#include "obj_parser.h"


namespace raptor_raytracer
{
const char * find_next_statement(const char *at)
{
    while (true)
    {
        /* Skip comments */
        if (*at == '#')
        {
            while (*at != '\n')
            {
                ++at;
            }
            ++at;
        }
        /* Skip white space */
        else if ((*at == ' ') || (*at == '\t') || (*at == '\n') || (*at == '\r'))
        {
            ++at;
        }
        /* Return when a char is found */
        else
        {
            return at;
        }
    }
}


void find_vertex(const char **c)
{
    while (((**c) != ' ') && ((**c) != '\n') && ((**c) != '\r'))
    {
//        std::cout << "non space increment" << std::endl;
        ++(*c);
    }
    
    while ((**c) == ' ')
    {
//        std::cout << "space increment" << std::endl;
        ++(*c);
    }
    
    return;
}


bool is_triplet_delimiter(const char **c)
{
    ++(*c);
    while ((**c != '/') && (**c != ' ') && (**c != '\n') && (**c != '\r'))
    {
        ++(*c);
    }
    
    return (**c == '/');
}


void parse_f_statement(light_list *l, primitive_list *e, std::vector<point_t> &vt, std::vector<point_t> &vn, std::vector<point_t> &v, material *const m, const char **c)
{
//    std::cout << "face: " << (*c)[0] << std::endl;
    static std::vector<point_t> face;
    static std::vector<point_t> face_t;
    static std::vector<point_t> face_n;

    /* Parse all vextex data for the face */
    find_vertex(c);
    while (((**c) != '\n') && ((**c) != '\r'))
    {
        /* Parse vertex */
        int vert_num = atoi(*c);
        if (vert_num < 0)
        {
            vert_num = v.size() + vert_num;
        }
        else
        {
            --vert_num;
        }
//        std::cout << "vertex: " << vert_num << std::endl;
        assert(static_cast<unsigned int>(vert_num) < v.size());
        face.push_back(v[vert_num]);
        
        /* Parse texture vertex */
        if (is_triplet_delimiter(c))
        {
            if ((*c)[1] != '/')
            {
                ++(*c);
                int vert_text = atoi(*c);
                if (vert_text < 0)
                {
                    vert_text = vt.size() + vert_text;
                }
                else
                {
                    --vert_text;
                }
//                std::cout << "texture: " << vert_text << std::endl;
                assert(static_cast<unsigned int>(vert_text) < vt.size());
                face_t.push_back(vt[vert_text]);
            }
        
            /* Parse vertex normal */
            if(is_triplet_delimiter(c))
            {
                ++(*c);
                int vert_norm = atoi(*c);
                if (vert_norm < 0)
                {
                    vert_norm = vn.size() + vert_norm;
                }
                else
                {
                    --vert_norm;
                }
//                std::cout << "normal: " << vert_norm << std::endl;
                assert(static_cast<unsigned int>(vert_norm) < vn.size());
                face_n.push_back(vn[vert_norm]);
            }
        }
//        std::cout << "current: " << (*c)[0] << (*c)[1] << std::endl;
        find_vertex(c);
    }

    /* Move past the new line */
    ++(*c);

    /* Create the polygon */
    if (face.size() == 3)
    {
        /* Check the points show movement in atleast 2 axes  */
        point_t a(face[0]);
        point_t b(face[1]);
        point_t c(face[2]);

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
                if (face_n.empty())
                {
                    if (face_t.empty())
                    {
                        new_triangle(e, nullptr, m, face[0], face[1], face[2], false);
                    }
                    else
                    {
                        assert(face_t.size() == 3);
                        point_t text[3] = { face_t[0], face_t[1], face_t[2] };
                        new_triangle(e, nullptr, m, face[0], face[1], face[2], false, nullptr, &text[0]);
                    }
                }
                else
                {
                    assert(face_n.size() == 3);
                    point_t normals[3] = { face_n[0], face_n[1], face_n[2] };
                    if (face_t.empty())
                    {
                        new_triangle(e, nullptr, m, face[0], face[1], face[2], false, &normals[0]);
                    }
                    else
                    {
                        assert(face_t.size() == 3);
                        point_t text[3] = { face_t[0], face_t[1], face_t[2] };
                        new_triangle(e, nullptr, m, face[0], face[1], face[2], false, &normals[0], &text[0]);
                    }
                }
            }
        }
    }
    else
    {
        assert((face_n.size() == face.size()) || (face_n.size() == 0));
        assert((face_t.size() == face.size()) || (face_t.size() == 0));
        face_to_triangles(e, l, face, m, false, &face_n, &face_t);
    }

    /* Clean up */
    face.clear();
    face_t.clear();
    face_n.clear();
}


material * parse_mtllib(std::map<std::string, material *> *const s, std::ifstream &f, const std::string &p)
{
    /* Find the size of the file */
    f.seekg(0, std::ios::end);
    size_t len = f.tellg();
    f.seekg(0, std::ios::beg);
       
    /* Read the whole file into a buffer */
    char *buffer = new char [len];
    f.read((char *)buffer, len);
    const char *at = &buffer[0];
    
    /* Colour of current material */
    std::string     mn;
    material       *m       = nullptr;
    texture_mapper *t_ka    = nullptr;  /* Ambient colour texture mapper    */
    texture_mapper *t_kd    = nullptr;  /* Diffuse colour texture mapper    */
    texture_mapper *t_ks    = nullptr;  /* Specular colour texture mapper   */
    texture_mapper *t_ns    = nullptr;  /* Specular exponant texture mapper */
    ext_colour_t    ka;                 /* Ambient colour                   */
    ext_colour_t    kd;                 /* Diffuse colour                   */
    ext_colour_t    ks;                 /* Specular colour                  */
    ext_colour_t    ke;                 /* Emitted colour                   */
    ext_colour_t    tf;                 /* Transmittaned colour             */
    float           ns      = 0.0f;     /* Specular exponant                */
    float           ri      = 1.0f;     /* Refractive index                 */
    
    /* Parse the file */
    while (at < &buffer[len - 1])
    {
        at = find_next_statement(at);
        if (at >= &buffer[len - 1])
        {
            break;
        }
        
        /* New material */
        if (strncmp(at, "newmtl", 6) == 0)
        {
            /* Create the last material parsed */
            if (!mn.empty())
            {
                m = new coloured_mapper_shader(ka, kd, ks, ns, 0.0, 1.0, 0.0, 0.0, 0.0, t_ka, t_kd, t_ks, t_ns);
                (*s)[mn] = m;
            }
            
            /* Get the name of the next material to parse */
            at += 6;
            mn = get_next_string(&at);
            
            /* Default parameters */
            t_ka    = nullptr;
            t_kd    = nullptr;
            t_ks    = nullptr;
            t_ns    = nullptr;
            ka      = ext_colour_t(0.0);
            kd      = ext_colour_t(0.0);
            ks      = ext_colour_t(0.0);
            ns      = 0.0;
        }
        /* Ambient colour */
        else if (strncmp(at, "Ka", 2) == 0)
        {
            ka.r = get_next_float(&at) * 255.0f;
            if (!end_of_line(at))
            {
                ka.g = get_next_float(&at) * 255.0f;
                ka.b = get_next_float(&at) * 255.0f;
            }
            else
            {
                ka.g = ka.r;
                ka.b = ka.r;
            }
        }
        /* Diffuse colour */
        else if (strncmp(at, "Kd", 2) == 0)
        {
            kd.r = get_next_float(&at) * 255.0f;
            if (!end_of_line(at))
            {
                kd.g = get_next_float(&at) * 255.0f;
                kd.b = get_next_float(&at) * 255.0f;
            }
            else
            {
                kd.g = kd.r;
                kd.b = kd.r;
            }
        }
        /* Specular colour */
        else if (strncmp(at, "Ks", 2) == 0)
        {
            ks.r = get_next_float(&at) * 255.0f;
            if (!end_of_line(at))
            {
                ks.g = get_next_float(&at) * 255.0f;
                ks.b = get_next_float(&at) * 255.0f;
            }
            else
            {
                ks.g = ks.r;
                ks.b = ks.r;
            }
        }
        /* Transmitted colour */
        else if (strncmp(at, "Tf", 2) == 0)
        {
            tf.r = get_next_float(&at) * 255.0f;
            if (!end_of_line(at))
            {
                tf.g = get_next_float(&at) * 255.0f;
                tf.b = get_next_float(&at) * 255.0f;
            }
            else
            {
                tf.g = tf.r;
                tf.b = tf.r;
            }
        }
        /* Emitted colour */
        else if (strncmp(at, "Ke", 2) == 0)
        {
            ke.r = get_next_float(&at) * 255.0f;
            if (!end_of_line(at))
            {
                ke.g = get_next_float(&at) * 255.0f;
                ke.b = get_next_float(&at) * 255.0f;
            }
            else
            {
                ke.g = ke.r;
                ke.b = ke.r;
            }

            // assert(ke.r == 0.0f);
            // assert(ke.g == 0.0f);
            // assert(ke.b == 0.0f);
        }//  bump
        /* Specular exponant */
        else if (strncmp(at, "Ns", 2) == 0)
        {
            ns = get_next_float(&at);
        }
        /* Refractive index */
        else if (strncmp(at, "Ni", 2) == 0)
        {
            ri = get_next_float(&at);
        }
        /* Opaque-ness */
        else if (strncmp(at, "d", 1) == 0)
        {
            const float d = get_next_float(&at);
            assert(d == 1.0f);
        }
        /* Opaque-ness by another name */
        else if (strncmp(at, "Tr", 2) == 0)
        {
            const float tr = get_next_float(&at);
            assert(tr == 0.0f);
        }
        /* Shader mode --ignored, everything is ray traced */
        else if (strncmp(at, "illum", 5) == 0)
        {

        }
        /* Ambiant texture map */
        else if (strncmp(at, "map_Ka", 6) == 0)
        {
            std::string map_file(p + get_next_string(&at));
            // float *img;
            // unsigned int img_width;
            // unsigned int img_height;
            // unsigned int cpp = read_jpeg(&img, map_file.c_str(), &img_height, &img_width);
            // t_kd = new planar_mapper(boost::shared_array<float>(img), point_t(0.0), point_t(0.0), point_t(0.0), cpp, img_width, img_height, texture_wrapping_mode_t::mirror, texture_wrapping_mode_t::mirror);
        }
        /* Diffuse texture map */
        else if (strncmp(at, "map_Kd", 6) == 0)
        {
            std::string map_file(p + get_next_string(&at));
            // float *img;
            // unsigned int img_width;
            // unsigned int img_height;
            // unsigned int cpp = read_jpeg(&img, map_file.c_str(), &img_height, &img_width);
            // t_kd = new planar_mapper(boost::shared_array<float>(img), point_t(0.0), point_t(0.0), point_t(0.0), cpp, img_width, img_height, texture_wrapping_mode_t::mirror, texture_wrapping_mode_t::mirror);
        }
        /* Bump texture map */
        else if (strncmp(at, "map_bump", 8) == 0)
        {
            std::string map_file(p + get_next_string(&at));
            // float *img;
            // unsigned int img_width;
            // unsigned int img_height;
            // unsigned int cpp = read_jpeg(&img, map_file.c_str(), &img_height, &img_width);
            // t_kd = new planar_mapper(boost::shared_array<float>(img), point_t(0.0), point_t(0.0), point_t(0.0), cpp, img_width, img_height, texture_wrapping_mode_t::mirror, texture_wrapping_mode_t::mirror);
        }

        else if (strncmp(at, "bump", 4) == 0)
        {
            std::string map_file(p + get_next_string(&at));
            // float *img;
            // unsigned int img_width;
            // unsigned int img_height;
            // unsigned int cpp = read_jpeg(&img, map_file.c_str(), &img_height, &img_width);
            // t_kd = new planar_mapper(boost::shared_array<float>(img), point_t(0.0), point_t(0.0), point_t(0.0), cpp, img_width, img_height, texture_wrapping_mode_t::mirror, texture_wrapping_mode_t::mirror);
        }
        /* Opaqueness texture map */
        else if (strncmp(at, "map_d", 5) == 0)
        {
            std::string map_file(p + get_next_string(&at));
            // float *img;
            // unsigned int img_width;
            // unsigned int img_height;
            // unsigned int cpp = read_jpeg(&img, map_file.c_str(), &img_height, &img_width);
            // t_kd = new planar_mapper(boost::shared_array<float>(img), point_t(0.0), point_t(0.0), point_t(0.0), cpp, img_width, img_height, texture_wrapping_mode_t::mirror, texture_wrapping_mode_t::mirror);
        }
        else
        {
            std::cout << "Found unknown: " << std::string(at[0], 5) << ", at: " << reinterpret_cast<long>(at) << ", of: " << reinterpret_cast<long>(&buffer[len - 1]) << std::endl;
            find_next_line(&at);
            assert(false);
        }
        
        /* Get next lone for parsing */
        find_next_line(&at);
    }

    /* Create the last material parsed */
    if (!mn.empty())
    {
        m = new coloured_mapper_shader(ka, kd, ks, ns, 0.0, ri, 0.0, 0.0, 0.0, t_ka, t_kd, t_ks, t_ns);
        (*s)[mn] = m;
    }

    /* Clean up */
    delete [] buffer;
    
    return m;
}


material * parse_mtllib_statement(std::map<std::string, material *> *const s, const std::string &p, const char **c)
{
    /* Last defined material */
    material *m = nullptr;
    
    /* Parse till the end of the line */
    while (!end_of_line(*c))
    {
        /* Get the material library to open */
        std::string mtllib = p + get_next_string(c);
//        std::cout << "Opening material file: " << mtllib << std::endl;
        std::ifstream f(mtllib.c_str());
        assert(f.is_open());

        /* Parse the file */    
        m = parse_mtllib(s, f, p);
    
        /* Close the file */
        f.close();
        (*c)--;
    }
    
    /* Advance to the next line */
    find_next_line(c);
    return m;
}


void parse_usemtl_statement(std::map<std::string, material *> *const s, material **m, const char **c)
{
    /* Get the name of the material and look it up in the hash */
    std::string usemtl = get_next_string(c);
    auto i = s->find(usemtl);
    
    /* Check if the material was found */
    if(i != s->end())
    {
        (*m) = i->second;
    }
    else
    {
        *m = new phong_shader(ext_colour_t(0.0f, 0.0f, 0.0f), ext_colour_t(200.0f, 200.0f, 200.0f), ext_colour_t(0.0f, 0.0f, 0.0f), 0.0f);
        (*s)[usemtl] = *m;
    }
        
   find_next_line(c);
}


void parse_v_statement(std::vector<point_t> *const vs, const char **c)
{
    point_t v;

    v.x = get_next_float(c);
    v.y = get_next_float(c);
    v.z = get_next_float(c);
//    std::cout << "Parsed v: " << v.x << ", " << v.y << ", " << v.z << std::endl;

    find_next_line(c);

    vs->push_back(v);
}


void parse_vt_statement(std::vector<point_t> *const vs, const char **c)
{
    point_t v;

    v.x = get_next_float(c);
    v.y = get_next_float(c);
//    std::cout << "Parsed vt: " << v.x << ", " << v.y << ", " << v.z << std::endl;

    find_next_line(c);

    vs->push_back(v);
}


void parse_vn_statement(std::vector<point_t> *const vs, const char **c)
{
    point_t v;

    v.x = get_next_float(c);
    v.y = get_next_float(c);
    v.z = get_next_float(c);
    normalise(&v);
//    std::cout << "Parsed vn: " << v.x << ", " << v.y << ", " << v.z << std::endl;

    find_next_line(c);

    vs->push_back(v);
}


void obj_parser(
    std::ifstream           &obj_file,
    std::string             p,
    light_list              &l, 
    primitive_list          &e,
    std::list<material *>   &m,
    camera                  **c)
{
    /* Find the size of the file */
    obj_file.seekg(0, std::ios::end);
    size_t len = obj_file.tellg();
    obj_file.seekg(0, std::ios::beg);

//    std::cout << "Parsing OBJ length: " << len << std::endl;
       
    /* Read the whole file into a buffer */
    char *buffer = new char [len];
    obj_file.read((char *)buffer, len);
    const char *at = &buffer[0];
    
    /* Vectors of vertice data */
    std::vector<point_t>    vt;
    std::vector<point_t>    vn;
    std::vector<point_t>    v;
    
    /* Map of shader names to shader */
    std::map<std::string, material *> shader_map;
    material *              cur_mat = new phong_shader(ext_colour_t(0.0f, 0.0f, 0.0f), ext_colour_t(200.0f, 200.0f, 200.0f), ext_colour_t(0.0f, 0.0f, 0.0f), 0.0f);
    m.push_back(cur_mat);
    
    /* Parse the file */
    while (at < &buffer[len - 1])
    {
        at = find_next_statement(at);
        if (at >= &buffer[len - 1])
        {
            break;
        }
        
        if (strncmp(at, "mtllib", 6) == 0)
        {
            cur_mat = parse_mtllib_statement(&shader_map, p, &at);
        }
        else if (strncmp(at, "usemtl", 6) == 0)
        {
            parse_usemtl_statement(&shader_map, &cur_mat, &at);
        }
        else if (strncmp(at, "vn", 2) == 0)
        {
            parse_vn_statement(&vn, &at);
        }
        else if (strncmp(at, "vt", 2) == 0)
        {
            parse_vt_statement(&vt, &at);
        }
        else if (*at == 'f')
        {
            parse_f_statement(&l, &e, vt, vn, v, cur_mat, &at);
        }
        else if (*at == 'v')
        {
            parse_v_statement(&v, &at);
        }
        /* Group */
        else if (*at == 'g')
        {
            find_next_line(&at);
        }
        /* Object */
        else if (*at == 'o')
        {
            find_next_line(&at);
        }
        /* Smoothing group */
        else if (*at == 's')
        {
            find_next_line(&at);
        }
        else
        {
            std::cout << "Found unknown statement: " << at << ", at: " << reinterpret_cast<long>(at) << ", of: " << reinterpret_cast<long>(&buffer[len - 1]) << std::endl;
            find_next_line(&at);
            assert(false);
        }
    }
    
    /* Add the materials to the list of materials */
    for (auto i = shader_map.begin(); i != shader_map.end(); ++i)
    {
        m.push_back((*i).second);
    }
 
    /* Tidy up */
    delete [] buffer;
}
}; /* namespace raptor_raytracer */
