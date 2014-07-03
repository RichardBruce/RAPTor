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
//    cout << "finding background" << endl;
    while ((a != eob) && (strncmp(a, "Background {", 12) != 0))
    {
        ++a;
    }

    if (a != eob)
    {
//        cout << "background found" << endl;
        find_next_line(&a);
        skip_white_space(&a);
        assert(strncmp(a, "skyColor [", 10) == 0);
    }
    
    return;
}


fp_t parse_directional_light(light_list &l, const char *a, const char *eob)
{
    while ((a != eob) && (strncmp(a, "DirectionalLight {", 18) != 0))
    {
        ++a;
    }

    fp_t ambient = (fp_t)0.0;
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
        point_t c(-d * (fp_t)1000.0);


        find_next_line(&a);
        skip_white_space(&a);
        assert(strncmp(a, "intensity", 9) == 0);
        rgb *= get_next_float(&a) * (fp_t)255.0;
//        cout << "rgb: " << rgb.r << ", " << rgb.g << ", " << rgb.b << endl;
        
        new_light(&l, rgb, c, (fp_t)0.0, (fp_t)0.0);
    }
    
    return ambient;
}


void parse_viewpoint(camera *c, const string &v, const char *a, const char *eob)
{
    string desc = "";
    fp_t angle  = (fp_t)0.0;
    point_t p, r;
    while ((a != eob) && (v != desc))
    {
//        cout << "finding viewpoint" << endl;
        while ((a != eob) && (strncmp(a, "Viewpoint {", 11) != 0))
        {
            ++a;
        }
    
        if (a != eob)
        {
//            cout << "viewpoint found" << endl;
    
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

//            cout << "Viewpoint is: " << desc << endl;
        }
    }

    /* Warn if default view point is used */
    if (v != desc)
    {
        cout << "Warning: View point '" << v << "' not found, using default" << endl;
    }
    /* Set specified view point */
    else
    {
        c->move_to(p);
        c->rotate_about(r, angle);
    }
    
    return;
}


void parse_points(vector<point_t> *p, const char *a, const char *eob)
{
//    cout << "finding points" << endl;
    while ((a != eob) && (strncmp(a, "point [", 7) != 0))
    {
        ++a;
    }

    if (a != eob)
    {
//        cout << "points found" << endl;
        do 
        {
            point_t tp;
            find_next_line(&a);
            tp.x = get_next_float(&a);
            tp.y = get_next_float(&a);
            tp.z = get_next_float(&a);
            p->push_back(tp);
        
//            cout << tp.x << ", " << tp.y << ", " << tp.z << endl;
        } while (!group_done(a));
    }
//    cout << "end of points" << endl;
    return;
}


void parse_triangles(primitive_list &prim, list<material *> &m, const vector<point_t> &p, const vector<ext_colour_t> &c, const fp_t ka, const char *a, const char *eob)
{
//    cout << "finding coordIndex" << endl;
    cout << "ka: " << ka << endl;
    while ((a != eob) && (strncmp(a, "coordIndex [", 12) != 0))
    {
        ++a;
    }

    if (a != eob)
    {
//        cout << "coordIndex found" << endl;
        do 
        {
            /* Parse the vertex data */
            find_next_line(&a);
            unsigned int va = get_next_unsigned(&a);
            unsigned int vb = get_next_unsigned(&a);
            unsigned int vc = get_next_unsigned(&a);
            assert(get_next_unsigned(&a) == (unsigned int)-1);
            
            /* Mix the current colour */
            const ext_colour_t mix((c[va] + c[vb] + c[vc]) * (fp_t)(1.0 / 3.0));
            material *cur_mat = new phong_shader((mix * ka), mix, ext_colour_t(0.0, 0.0, 0.0));
            m.push_back(cur_mat);

            /* Create the triangle */
            vector<triangle *> *t;
            new_triangle(&prim, t, cur_mat, p[va], p[vb], p[vc], false);
        
//            cout << va << ", " << vb << ", " << vc << endl;
        } while (!group_done(a));
    }

//    cout << "end of coordIndex" << endl;
    return;
}


void parse_colours(vector<ext_colour_t> *c, const char *a, const char *eob)
{
    cout << "finding colours" << endl;
    while ((a != eob) && (strncmp(a, "color [", 7) != 0))
    {
        ++a;
    }

    if (a != eob)
    {
        cout << "colours found" << endl;
        do 
        {
            find_next_line(&a);
            ext_colour_t rgb;
            rgb.r = get_next_float(&a) * (fp_t)255.0;
            rgb.g = get_next_float(&a) * (fp_t)255.0;
            rgb.b = get_next_float(&a) * (fp_t)255.0;
            c->push_back(rgb);
        } while (!group_done(a));
    }

    cout << "end of colours" << endl;
    return;
}


void vrml_parser(
    ifstream            &vrml_file,
    light_list          &l, 
    primitive_list      &e,
    list<material *>    &m,
    camera              *c,
    const string        &v)
{
    /* Find the size of the file */
    vrml_file.seekg(0, ios::end);
    size_t len = vrml_file.tellg();
    vrml_file.seekg(0, ios::beg);
    
    /* Read the whole file into a buffer */
    char *buffer = new char [len];
    vrml_file.read(buffer, len);
    char *at = &buffer[0];
    
    /* The background colour */
    ext_colour_t bg;

    /* Vector for face to triangle conversion */
    vector<point_t>         points;
    vector<ext_colour_t>    colours;

    /* Parse the file */
    parser_bg(at, &buffer[len - 1], &bg);
    
    const fp_t ka = parse_directional_light(l, at, &buffer[len - 1]);
    
    parse_viewpoint(c, v, at, &buffer[len - 1]);
    
    parse_points(&points, at, &buffer[len - 1]);
    colours.reserve(points.size());
    parse_colours(&colours, at, &buffer[len - 1]);
    parse_triangles(e, m, points, colours, ka, at, &buffer[len - 1]);

    /* Clean up */
    delete [] buffer;
}
}; /* namespace raptor_raytracer */
