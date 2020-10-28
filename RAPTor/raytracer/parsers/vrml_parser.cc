#include "common.h"
#include "parser_common.h"
#include "polygon_to_triangles.h"

#include "camera.h"

#include "vrml_parser.h"

/* Shaders */
#include "phong_shader.h"


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
    while ((a != eob) && (strncmp(a, "Background {", 12) != 0))
    {
        ++a;
    }

    if (a != eob)
    {
        raptor_parsers::find_next_line(&a);
        raptor_parsers::skip_white_space(&a);
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
        raptor_parsers::find_next_line(&a);
        raptor_parsers::skip_white_space(&a);
        assert(strncmp(a, "ambientIntensity", 16) == 0);
        ambient = raptor_parsers::get_next_float(&a);


        raptor_parsers::find_next_line(&a);
        raptor_parsers::skip_white_space(&a);
        assert(strncmp(a, "color", 5) == 0);
        ext_colour_t rgb;
        rgb.r = raptor_parsers::get_next_float(&a);
        rgb.g = raptor_parsers::get_next_float(&a);
        rgb.b = raptor_parsers::get_next_float(&a);


        raptor_parsers::find_next_line(&a);
        raptor_parsers::skip_white_space(&a);
        assert(strncmp(a, "direction", 9) == 0);
        point_t d;
        d.x = raptor_parsers::get_next_float(&a);
        d.y = raptor_parsers::get_next_float(&a);
        d.z = raptor_parsers::get_next_float(&a);
        point_t c(-d * 1000.0f);


        raptor_parsers::find_next_line(&a);
        raptor_parsers::skip_white_space(&a);
        assert(strncmp(a, "intensity", 9) == 0);
        rgb *= raptor_parsers::get_next_float(&a) * 255.0f;
        
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
        while ((a != eob) && (strncmp(a, "Viewpoint {", 11) != 0))
        {
            ++a;
        }
    
        if (a != eob)
        {
    
            raptor_parsers::find_next_line(&a);
            raptor_parsers::skip_white_space(&a);
            assert(strncmp(a, "position", 8) == 0);
            p.x = raptor_parsers::get_next_float(&a);
            p.y = raptor_parsers::get_next_float(&a);
            p.z = raptor_parsers::get_next_float(&a);
    
            raptor_parsers::find_next_line(&a);
            raptor_parsers::skip_white_space(&a);
            assert(strncmp(a, "orientation", 11) == 0);
            r.x     = raptor_parsers::get_next_float(&a);
            r.y     = raptor_parsers::get_next_float(&a);
            r.z     = raptor_parsers::get_next_float(&a);
            angle   = raptor_parsers::get_next_float(&a);
    
            raptor_parsers::find_next_line(&a);
            raptor_parsers::skip_white_space(&a);
            assert(strncmp(a, "description", 11) == 0);
            desc = raptor_parsers::get_quoted_string(&a);
        }
    }

    /* Warn if default view point is used */
    if (v != desc)
    {
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
    while ((a != eob) && (strncmp(a, "point [", 7) != 0))
    {
        ++a;
    }

    if (a != eob)
    {
        do 
        {
            point_t tp;
            raptor_parsers::find_next_line(&a);
            tp.x = raptor_parsers::get_next_float(&a);
            tp.y = raptor_parsers::get_next_float(&a);
            tp.z = raptor_parsers::get_next_float(&a);
            p->push_back(tp);
        
        } while (!group_done(a));
    }
    return;
}


void parse_triangles(primitive_store &prim, std::list<material *> &m, const std::vector<point_t> &p, const std::vector<ext_colour_t> &c, const float ka, const char *a, const char *eob)
{
    while ((a != eob) && (strncmp(a, "coordIndex [", 12) != 0))
    {
        ++a;
    }

    if (a != eob)
    {
        do 
        {
            /* Parse the vertex data */
            raptor_parsers::find_next_line(&a);
            unsigned int va = raptor_parsers::get_next_unsigned(&a);
            unsigned int vb = raptor_parsers::get_next_unsigned(&a);
            unsigned int vc = raptor_parsers::get_next_unsigned(&a);
            assert(raptor_parsers::get_next_unsigned(&a) == (unsigned int)-1);
            
            /* Mix the current colour */
            const ext_colour_t mix((c[va] + c[vb] + c[vc]) * (1.0f / 3.0f));
            material *cur_mat = new phong_shader((mix * ka), mix, ext_colour_t(0.0f, 0.0f, 0.0f));
            m.push_back(cur_mat);

            /* Create the triangle */
            new_triangle(&prim, nullptr, cur_mat, p[va], p[vb], p[vc], false);
        
        } while (!group_done(a));
    }

    return;
}


void parse_colours(std::vector<ext_colour_t> *c, const char *a, const char *eob)
{
    while ((a != eob) && (strncmp(a, "color [", 7) != 0))
    {
        ++a;
    }

    if (a != eob)
    {
        do 
        {
            raptor_parsers::find_next_line(&a);
            ext_colour_t rgb;
            rgb.r = raptor_parsers::get_next_float(&a) * 255.0f;
            rgb.g = raptor_parsers::get_next_float(&a) * 255.0f;
            rgb.b = raptor_parsers::get_next_float(&a) * 255.0f;
            c->push_back(rgb);
        } while (!group_done(a));
    }

    return;
}


void vrml_parser(
    std::ifstream           &vrml_file,
    light_list              &l, 
    primitive_store         &e,
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
