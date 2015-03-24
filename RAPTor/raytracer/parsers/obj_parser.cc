#include "common.h"
#include "parser_common.h"

#include "camera.h"
#include "obj_parser.h"
#include "picture_functions.h"


namespace raptor_raytracer
{
const float ambiant_light = 0.5f;

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
            if (is_triplet_delimiter(c))
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
            cross_product(a, b, &c);
            if (dot_product(c, c) > 0.0f)
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


struct map_info
{
    boost::shared_array<float>  img;
    unsigned int                img_width;
    unsigned int                img_height;
    unsigned int                cpp;
};

inline texture_mapper * load_image(std::map<std::string, map_info> *const cache, const std::string &name, const bool invert = false)
{
    /* Check if loaded */
    const std::string cache_name(name + (invert ? "_inv" : ""));
    auto cache_iter = cache->find(cache_name);
    if (cache_iter == cache->end())
    {
        BOOST_LOG_TRIVIAL(error) << "Reading: " << name;
        /* Load image */
        float *img;
        map_info info;
        info.cpp = read_image_file(&img, name, &info.img_height, &info.img_width);
        info.img.reset(img);

        /* Cache */
        if (invert)
        {
            negative(img, info.cpp * info.img_height * info.img_width);
        }
        cache_iter = cache->emplace(cache_name, info).first;
    }

    const auto &info = cache_iter->second;
    return new planar_mapper(info.img, point_t(0.0f), point_t(1.0f, 0.0f, 0.0f), point_t(0.0f), info.cpp, info.img_width, info.img_height, texture_wrapping_mode_t::mirror, texture_wrapping_mode_t::mirror);
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

    /* Image cache */
    std::map<std::string, map_info> image_cache;
    
    /* Colour of current material */
    std::string     mn;
    material       *m       = nullptr;
    texture_mapper *t_ka    = nullptr;  /* Ambient colour texture mapper    */
    texture_mapper *t_kd    = nullptr;  /* Diffuse colour texture mapper    */
    texture_mapper *t_ks    = nullptr;  /* Specular colour texture mapper   */
    texture_mapper *t_ns    = nullptr;  /* Specular exponant texture mapper */
    texture_mapper *t_rf    = nullptr;  /* Reflectance texture mapper       */
    texture_mapper *t_tran  = nullptr;  /* Transmittance texture mapper     */
    ext_colour_t    ka;                 /* Ambient colour                   */
    ext_colour_t    kd;                 /* Diffuse colour                   */
    ext_colour_t    ks;                 /* Specular colour                  */
    ext_colour_t    ke;                 /* Emitted colour                   */
    ext_colour_t    tf;                 /* Transmitted colour               */
    float           ns      = 0.0f;     /* Specular exponant                */
    float           ri      = 1.0f;     /* Refractive index                 */
    float           rf      = 0.0f;     /* Reflectance                      */
    float           trans   = 0.0f;     /* Transmittance                    */
    int             illum   = 0;        /* Illumination model               */
    
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
                assert(illum != 6  || !"Error: Illumination model not handled");
                assert(illum != 8  || !"Error: Illumination model not handled");
                assert(illum != 9  || !"Error: Illumination model not handled");
                assert(illum != 10 || !"Error: Illumination model not handled");
                switch (illum)
                {
                    /* colour = Kd */
                    case 0 :
                        t_ka    = t_kd;
                        t_kd    = nullptr;
                        t_ks    = nullptr;
                        t_ns    = nullptr;
                        t_rf    = nullptr;
                        ka      = kd;
                        kd      = ext_colour_t(0.0f, 0.0f, 0.0f);
                        ks      = ext_colour_t(0.0f, 0.0f, 0.0f);
                        ke      = ext_colour_t(0.0f, 0.0f, 0.0f);
                        tf      = ext_colour_t(0.0f, 0.0f, 0.0f);
                        ns      = 0.0f;
                        ri      = 1.0f;
                        rf      = 0.0f;
                        break;
                    /* colour = KaIa + Kd { SUM j=1..ls, (N * Lj)Ij } */
                    case 1 :
                        t_ks    = nullptr;
                        t_ns    = nullptr;
                        t_rf    = nullptr;
                        ks      = ext_colour_t(0.0f, 0.0f, 0.0f);
                        ke      = ext_colour_t(0.0f, 0.0f, 0.0f);
                        tf      = ext_colour_t(0.0f, 0.0f, 0.0f);
                        ns      = 0.0f;
                        ri      = 1.0f;
                        rf      = 0.0f;
                        break;
                    /* colour = KaIa + Kd { SUM j=1..ls, (N*Lj)Ij } + Ks { SUM j=1..ls, ((H*Hj)^Ns)Ij } */
                    case 2 :
                        t_rf    = nullptr;
                        ke      = ext_colour_t(0.0f, 0.0f, 0.0f);
                        tf      = ext_colour_t(0.0f, 0.0f, 0.0f);
                        ri      = 1.0f;
                        rf      = 0.0f;
                        break;
                    /* colour = KaIa + Kd { SUM j=1..ls, (N*Lj)Ij } + Ks ({ SUM j=1..ls, ((H*Hj)^Ns)Ij } + Ir) */
                    /* Ir = (intensity of reflection map) + (ray trace)*/
                    case 3 :
                        ke      = ext_colour_t(0.0f, 0.0f, 0.0f);
                        tf      = ext_colour_t(0.0f, 0.0f, 0.0f);
                        ri      = 1.0f;
                        rf      = 0.75f;
                        break;
                    /* colour = KaIa + Kd { SUM j=1..ls, (N*Lj)Ij } + Ks ({ SUM j=1..ls, ((H*Hj)^Ns)Ij } + Ir) */
                    /* The maximum of the average intensity of highlights and reflected lights is used to adjust the dissolve factor */
                    case 4 :
                        ke      = ext_colour_t(0.0f, 0.0f, 0.0f);
                        tf      = ext_colour_t(0.0f, 0.0f, 0.0f);
                        rf      = 0.0f;
                        break;
                    /* colour = KaIa + Kd { SUM j=1..ls, (N*Lj)Ij } + Ks ({ SUM j=1..ls, ((H*Hj)^Ns)Ij Fr(Lj*Hj,Ks,Ns)Ij} + Fr(N*V,Ks,Ns)Ir}) */
                    case 5 :
                        ke      = ext_colour_t(0.0f, 0.0f, 0.0f);
                        tf      = ext_colour_t(0.0f, 0.0f, 0.0f);
                        ri      = 1.0f;
                        rf      = 0.75f;
                        break;
                    /*color = KaIa + Kd { SUM j=1..ls, (N*Lj)Ij } + Ks ({ SUM j=1..ls, ((H*Hj)^Ns)Ij Fr(Lj*Hj,Ks,Ns)Ij} + Fr(N*V,Ks,Ns)Ir}) + (1.0 - Kx)Ft (N*V,(1.0-Ks),Ns)TfIt */
                    case 7 :
                        ke      = ext_colour_t(0.0f, 0.0f, 0.0f);
                        tf      = ext_colour_t(0.0f, 0.0f, 0.0f);
                        ri      = 1.0f;
                        rf      = 0.75f;
                        break;

                    default :
                        assert(!"Error: Illumination model not handled");
                }
                trans = std::max(0.0f, std::min(1.0f, trans));
            //     coloured_mapper_shader(const ext_colour_t& ka, const ext_colour_t &kd, const ext_colour_t &ks = 0.0f, const float ns = 0.0f, const float tran = 0.0f, const float ri = 1.0f, 
            // const float rf = 0.0f, const float td = 0.0f, const float rfd = 0.0f, const texture_mapper * const t_ka = nullptr, const texture_mapper * const t_kd = nullptr, 
            // const texture_mapper * const t_ks = nullptr, const texture_mapper * const t_ns = nullptr, const texture_mapper * const t_refl = nullptr, const texture_mapper * const t_d = nullptr)
                ka *= ambiant_light;
                m = new coloured_mapper_shader(ka, kd, ks, ns, trans, ri, rf, 0.0f, 0.0f, t_ka, t_kd, t_ks, t_ns, t_rf, t_tran);
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
            t_rf    = nullptr;
            t_tran  = nullptr;
            ka      = ext_colour_t(0.0f);
            kd      = ext_colour_t(0.0f);
            ke      = ext_colour_t(0.0f);
            ks      = ext_colour_t(0.0f);
            ns      = 0.0f;
            ri      = 1.0f;
            rf      = 0.0f;
            trans   = 0.0f;
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

            BOOST_LOG_TRIVIAL(warning) << "Tf: " << tf << " (not handled)";
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

            BOOST_LOG_TRIVIAL(warning) << "Ke: " << ke << " (not handled)";
        }
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
            BOOST_LOG_TRIVIAL(trace) << "d: " << d;
            trans = 1.0f - d;
        }
        /* Opaque-ness by another name */
        else if (strncmp(at, "Tr", 2) == 0)
        {
            trans = get_next_float(&at);
            BOOST_LOG_TRIVIAL(trace) << "Tr: " << trans;
        }
        /* Shader mode --ignored, everything is ray traced */
        else if (strncmp(at, "illum", 5) == 0)
        {
            illum = get_next_unsigned(&at);
            BOOST_LOG_TRIVIAL(trace) << "illum: " << illum;
        }
        /* Ambiant texture map */
        else if (strncmp(at, "map_Ka", 6) == 0)
        {
            const std::string map_file(p + get_next_string(&at));
            BOOST_LOG_TRIVIAL(trace) << "map_Ka: " << map_file;

            t_ka = load_image(&image_cache, map_file);
        }
        /* Diffuse texture map */
        else if (strncmp(at, "map_Kd", 6) == 0)
        {
            const std::string map_file(p + get_next_string(&at));
            BOOST_LOG_TRIVIAL(trace) << "map_Kd: " << map_file;

            t_kd = load_image(&image_cache, map_file);
        }
        /* Specular texture map */
        else if (strncmp(at, "map_Ks", 6) == 0)
        {
            const std::string map_file(p + get_next_string(&at));
            BOOST_LOG_TRIVIAL(trace) << "map_Ks: " << map_file;
            
            t_ks = load_image(&image_cache, map_file);
        }
        /* Reflection texture map */
        else if (strncmp(at, "map_refl", 8) == 0)
        {
            const std::string map_file(p + get_next_string(&at));
            BOOST_LOG_TRIVIAL(trace) << "map_refl: " << map_file;

            t_rf = load_image(&image_cache, map_file);
        }
        /* Bump texture map */
        else if (strncmp(at, "map_bump", 8) == 0)
        {
            const std::string map_file(p + get_next_string(&at));
            BOOST_LOG_TRIVIAL(warning) << "map_bump: " << map_file << " (not handled)";

            // t_kd = load_image(&image_cache, map_file);
        }
        else if (strncmp(at, "bump", 4) == 0)
        {
            const std::string map_file(p + get_next_string(&at));
            BOOST_LOG_TRIVIAL(warning) << "bump: " << map_file << " (not handled)";

            // t_kd = load_image(&image_cache, map_file);
        }
        /* Opaqueness texture map */
        else if (strncmp(at, "map_d", 5) == 0)
        {
            const std::string map_file(p + get_next_string(&at));
            BOOST_LOG_TRIVIAL(trace) << "map_d: " << map_file;

            t_tran = load_image(&image_cache, map_file, true);
        }
        else
        {
            std::cout << "Found unknown: " << std::string(&at[0], 6) << ", at: " << reinterpret_cast<long>(at) << ", of: " << reinterpret_cast<long>(&buffer[len - 1]) << std::endl;
            find_next_line(&at);
            assert(false);
        }
        
        /* Get next lone for parsing */
        find_next_line(&at);
    }

    /* Create the last material parsed */
    if (!mn.empty())
    {
        BOOST_LOG_TRIVIAL(trace) << "Adding material: " << mn;
        assert(illum != 6  || !"Error: Illumination model not handled");
        assert(illum != 8  || !"Error: Illumination model not handled");
        assert(illum != 9  || !"Error: Illumination model not handled");
        assert(illum != 10 || !"Error: Illumination model not handled");
        switch (illum)
        {
            /* colour = Kd */
            case 0 :
                t_ka    = t_kd;
                t_kd    = nullptr;
                t_ks    = nullptr;
                t_ns    = nullptr;
                t_rf    = nullptr;
                ka      = kd;
                kd      = ext_colour_t(0.0f, 0.0f, 0.0f);
                ks      = ext_colour_t(0.0f, 0.0f, 0.0f);
                ke      = ext_colour_t(0.0f, 0.0f, 0.0f);
                tf      = ext_colour_t(0.0f, 0.0f, 0.0f);
                ns      = 0.0f;
                ri      = 1.0f;
                rf      = 0.0f;
                break;
            /* colour = KaIa + Kd { SUM j=1..ls, (N * Lj)Ij } */
            case 1 :
                t_ks    = nullptr;
                t_ns    = nullptr;
                t_rf    = nullptr;
                ks      = ext_colour_t(0.0f, 0.0f, 0.0f);
                ke      = ext_colour_t(0.0f, 0.0f, 0.0f);
                tf      = ext_colour_t(0.0f, 0.0f, 0.0f);
                ns      = 0.0f;
                ri      = 1.0f;
                rf      = 0.0f;
                break;
            /* colour = KaIa + Kd { SUM j=1..ls, (N*Lj)Ij } + Ks { SUM j=1..ls, ((H*Hj)^Ns)Ij } */
            case 2 :
                t_rf    = nullptr;
                ke      = ext_colour_t(0.0f, 0.0f, 0.0f);
                tf      = ext_colour_t(0.0f, 0.0f, 0.0f);
                ri      = 1.0f;
                rf      = 0.0f;
                break;
            /* colour = KaIa + Kd { SUM j=1..ls, (N*Lj)Ij } + Ks ({ SUM j=1..ls, ((H*Hj)^Ns)Ij } + Ir) */
            /* Ir = (intensity of reflection map) + (ray trace)*/
            case 3 :
                ke      = ext_colour_t(0.0f, 0.0f, 0.0f);
                tf      = ext_colour_t(0.0f, 0.0f, 0.0f);
                ri      = 1.0f;
                rf      = 0.75f;
                break;
            /* colour = KaIa + Kd { SUM j=1..ls, (N*Lj)Ij } + Ks ({ SUM j=1..ls, ((H*Hj)^Ns)Ij } + Ir) */
            /* The maximum of the average intensity of highlights and reflected lights is used to adjust the dissolve factor */
            case 4 :
                ke      = ext_colour_t(0.0f, 0.0f, 0.0f);
                tf      = ext_colour_t(0.0f, 0.0f, 0.0f);
                rf      = 0.0f;
                break;
            /* colour = KaIa + Kd { SUM j=1..ls, (N*Lj)Ij } + Ks ({ SUM j=1..ls, ((H*Hj)^Ns)Ij Fr(Lj*Hj,Ks,Ns)Ij} + Fr(N*V,Ks,Ns)Ir}) */
            case 5 :
                ke      = ext_colour_t(0.0f, 0.0f, 0.0f);
                tf      = ext_colour_t(0.0f, 0.0f, 0.0f);
                ri      = 1.0f;
                rf      = 0.75f;
                break;
            /*color = KaIa + Kd { SUM j=1..ls, (N*Lj)Ij } + Ks ({ SUM j=1..ls, ((H*Hj)^Ns)Ij Fr(Lj*Hj,Ks,Ns)Ij} + Fr(N*V,Ks,Ns)Ir}) + (1.0 - Kx)Ft (N*V,(1.0-Ks),Ns)TfIt */
            case 7 :
                ke      = ext_colour_t(0.0f, 0.0f, 0.0f);
                tf      = ext_colour_t(0.0f, 0.0f, 0.0f);
                ri      = 1.0f;
                rf      = 0.75f;
                break;

            default :
                assert(!"Error: Illumination model not handled");
        }
        trans = std::max(0.0f, std::min(1.0f, trans));
    //     coloured_mapper_shader(const ext_colour_t& ka, const ext_colour_t &kd, const ext_colour_t &ks = 0.0f, const float ns = 0.0f, const float tran = 0.0f, const float ri = 1.0f, 
    // const float rf = 0.0f, const float td = 0.0f, const float rfd = 0.0f, const texture_mapper * const t_ka = nullptr, const texture_mapper * const t_kd = nullptr, 
    // const texture_mapper * const t_ks = nullptr, const texture_mapper * const t_ns = nullptr, const texture_mapper * const t_refl = nullptr, const texture_mapper * const t_d = nullptr)
        ka *= ambiant_light;
        m = new coloured_mapper_shader(ka, kd, ks, ns, trans, ri, rf, 0.0f, 0.0f, t_ka, t_kd, t_ks, t_ns, t_rf, t_tran);
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
        BOOST_LOG_TRIVIAL(trace) << "Opening material file: " << mtllib;
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
