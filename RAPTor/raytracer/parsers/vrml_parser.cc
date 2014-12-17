#include "common.h"
#include "parser_common.h"

#include "camera.h"

#include "vrml_parser.h"


namespace raptor_raytracer
{
bool group_done(const char *a)
{
    /* Eat the string until the ']' or new line is found */
    do
    {
        ++a;
    } while (((*a) != '\n') && ((*a) != ']'));
    
    /* Check for same line ']' */
    if ((*a) == ']')
    {
        return true;
    }

    /* Check for lead ']' */
    do 
    {
        ++a;
    } while ((*a) == ' ');
 
    return ((*a) == ']');
}


void parser_bg(const char *a, const char *eob, ext_colour_t *const bg)
{
//    std::cout << "finding background" << std::endl;
    while ((a != eob) && (strncmp(a, "Background {", 12) != 0))
    {
        ++a;
    }

    if (a != eob)
    {
//        std::cout << "background found" << std::endl;
        find_next_line(&a);
        skip_white_space(&a);
        assert(strncmp(a, "skyColor [", 10) == 0);
    }
    
    return;
}


float parse_directional_light(light_list &l, const char *a, const char *eob)
{
    while ((a != eob) && (strncmp(a, "DirectionalLight {", 18) != 0))
    {
        ++a;
    }

    float ambient = 0.0f;
    if (a != eob)
    {
        find_next_line(&a);
        skip_white_space(&a);
        assert(strncmp(a, "ambientIntensity", 16) == 0);
        ambient = get_next_float(&a);


        find_next_line(&a);
        skip_white_space(&a);
        assert(strncmp(a, "color", 5) == 0);
        ext_colour_t rgb;
        rgb.r = get_next_float(&a);
        rgb.g = get_next_float(&a);
        rgb.b = get_next_float(&a);


        find_next_line(&a);
        skip_white_space(&a);
        assert(strncmp(a, "direction", 9) == 0);
        point_t d;
        d.x = get_next_float(&a);
        d.y = get_next_float(&a);
        d.z = get_next_float(&a);
        point_t c(-d * 1000.0f);


        find_next_line(&a);
        skip_white_space(&a);
        assert(strncmp(a, "intensity", 9) == 0);
        rgb *= get_next_float(&a) * 255.0f;
//        std::cout << "rgb: " << rgb.r << ", " << rgb.g << ", " << rgb.b << std::endl;
        
        new_light(&l, rgb, c, 0.0f, 0.0f);
    }
    
    return ambient;
}


void parse_viewpoint(camera *c, const std::string &v, const char *a, const char *eob)
{
    std::string desc = "";
    float angle  = 0.0f;
    point_t p, r;
    while ((a != eob) && (v != desc))
    {
//        std::cout << "finding viewpoint" << std::endl;
        while ((a != eob) && (strncmp(a, "Viewpoint {", 11) != 0))
        {
            ++a;
        }
    
        if (a != eob)
        {
//            std::cout << "viewpoint found" << std::endl;
    
            find_next_line(&a);
            skip_white_space(&a);
            assert(strncmp(a, "position", 8) == 0);
            p.x = get_next_float(&a);
            p.y = get_next_float(&a);
            p.z = get_next_float(&a);
    
            find_next_line(&a);
            skip_white_space(&a);
            assert(strncmp(a, "orientation", 11) == 0);
            r.x     = get_next_float(&a);
            r.y     = get_next_float(&a);
            r.z     = get_next_float(&a);
            angle   = get_next_float(&a);
    
            find_next_line(&a);
            skip_white_space(&a);
            assert(strncmp(a, "description", 11) == 0);
            desc = get_quoted_string(&a);

//            std::cout << "Viewpoint is: " << desc << std::endl;
        }
    }

    /* Warn if default view point is used */
    if (v != desc)
    {
        std::cout << "Warning: View point '" << v << "' not found, using default" << std::endl;
    }
    /* Set specified view point */
    else
    {
        c->move_to(p);
        c->rotate_about(r, angle);
    }
    
    return;
}


void parse_points(std::vector<point_t> *p, const char *a, const char *eob)
{
//    std::cout << "finding points" << std::endl;
    while ((a != eob) && (strncmp(a, "point [", 7) != 0))
    {
        ++a;
    }

    if (a != eob)
    {
//        std::cout << "points found" << std::endl;
        do 
        {
            point_t tp;
            find_next_line(&a);
            tp.x = get_next_float(&a);
            tp.y = get_next_float(&a);
            tp.z = get_next_float(&a);
            p->push_back(tp);
        
//            std::cout << tp.x << ", " << tp.y << ", " << tp.z << std::endl;
        } while (!group_done(a));
    }
//    std::cout << "end of points" << std::endl;
    return;
}


void parse_triangles(primitive_list &prim, std::list<material *> &m, const std::vector<point_t> &p, const std::vector<ext_colour_t> &c, const float ka, const char *a, const char *eob)
{
//    std::cout << "finding coordIndex" << std::endl;
    std::cout << "ka: " << ka << std::endl;
    while ((a != eob) && (strncmp(a, "coordIndex [", 12) != 0))
    {
        ++a;
    }

    if (a != eob)
    {
//        std::cout << "coordIndex found" << std::endl;
        do 
        {
            /* Parse the vertex data */
            find_next_line(&a);
            unsigned int va = get_next_unsigned(&a);
            unsigned int vb = get_next_unsigned(&a);
            unsigned int vc = get_next_unsigned(&a);
            assert(get_next_unsigned(&a) == (unsigned int)-1);
            
            /* Mix the current colour */
            const ext_colour_t mix((c[va] + c[vb] + c[vc]) * (1.0f / 3.0f));
            material *cur_mat = new phong_shader((mix * ka), mix, ext_colour_t(0.0f, 0.0f, 0.0f));
            m.push_back(cur_mat);

            /* Create the triangle */
            std::vector<triangle *> *t = nullptr;
            new_triangle(&prim, t, cur_mat, p[va], p[vb], p[vc], false);
        
//            std::cout << va << ", " << vb << ", " << vc << std::endl;
        } while (!group_done(a));
    }

//    std::cout << "end of coordIndex" << std::endl;
    return;
}


void parse_colours(std::vector<ext_colour_t> *c, const char *a, const char *eob)
{
    std::cout << "finding colours" << std::endl;
    while ((a != eob) && (strncmp(a, "color [", 7) != 0))
    {
        ++a;
    }

    if (a != eob)
    {
        std::cout << "colours found" << std::endl;
        do 
        {
            find_next_line(&a);
            ext_colour_t rgb;
            rgb.r = get_next_float(&a) * 255.0f;
            rgb.g = get_next_float(&a) * 255.0f;
            rgb.b = get_next_float(&a) * 255.0f;
            c->push_back(rgb);
        } while (!group_done(a));
    }

    std::cout << "end of colours" << std::endl;
    return;
}


void vrml_parser(
    std::ifstream           &vrml_file,
    light_list              &l, 
    primitive_list          &e,
    std::list<material *>   &m,
    camera                  *c,
    const std::string       &v)
{
    /* Find the size of the file */
    vrml_file.seekg(0, std::ios::end);
    size_t len = vrml_file.tellg();
    vrml_file.seekg(0, std::ios::beg);
    
    /* Read the whole file into a buffer */
    char *buffer = new char [len];
    vrml_file.read(buffer, len);
    char *at = &buffer[0];
    
    /* The background colour */
    ext_colour_t bg;

    /* Vector for face to triangle conversion */
    std::vector<point_t>        points;
    std::vector<ext_colour_t>   colours;

    /* Parse the file */
    parser_bg(at, &buffer[len - 1], &bg);
    
    const float ka = parse_directional_light(l, at, &buffer[len - 1]);
    
    parse_viewpoint(c, v, at, &buffer[len - 1]);
    
    parse_points(&points, at, &buffer[len - 1]);
    colours.reserve(points.size());
    parse_colours(&colours, at, &buffer[len - 1]);
    parse_triangles(e, m, points, colours, ka, at, &buffer[len - 1]);

    /* Clean up */
    delete [] buffer;
}
}; /* namespace raptor_raytracer */
