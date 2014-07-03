#include "common.h"
#include "parser_common.h"

#include "camera.h"

#include "nff_parser.h"


namespace raptor_raytracer
{
void nff_parser(
    ifstream            &nff_file,
    light_list          &l, 
    primitive_list      &e,
    list<material *>    &m,
    camera              **c)
{
    /* Find the size of the file */
    nff_file.seekg(0, ios::end);
    size_t len = nff_file.tellg();
    nff_file.seekg(0, ios::beg);
    
    /* Read the whole file into a buffer */
    char *buffer = new char [len];
    nff_file.read(buffer, len);
    const char *at = &buffer[0];
    
    /* The current material properties to be used */
    phong_shader *cur_mat = NULL;
    
    /* The background colour */
    ext_colour_t bg;

    /* Vector for face to triangle conversion */
    vector<point_t> points;
    vector<point_t> normals;

    /* Parse the file */
    while (at < &buffer[len-1])
    {
        /* Parse the background colour */
        if ((*at) == 'b')
        {
            bg.r = get_next_float(&at);
            bg.g = get_next_float(&at);
            bg.b = get_next_float(&at);
            
            /* Scale the colour to rgb */
            bg *= (fp_t)255.0;
        }
        /* Parse viewing vectors and angles */
        else if ((*at) == 'v')
        {
            /* Parse the view position */
            find_next_line(&at);
            
            fp_t x, y, z, a, h, x_res, y_res;
            x  = get_next_float(&at);
            y  = get_next_float(&at);
            z  = get_next_float(&at);
            point_t from(x, y, z);
            
            /* Parse view direction */
            find_next_line(&at);
            x  = get_next_float(&at);
            y  = get_next_float(&at);
            z  = get_next_float(&at);
            point_t to(x, y, z);
            vector_t at_vec = to - from;
            
            /* Parse up direction */        
            find_next_line(&at);
            x  = get_next_float(&at);
            y  = get_next_float(&at);
            z  = get_next_float(&at);
            vector_t up_vec(x, y, z);
            
            /* Parse angle of view */        
            find_next_line(&at);
            a  = get_next_float(&at);
            fp_t angle = a * (PI/(fp_t)360.0); /* This is half of the angle in rad to make a right angle triangle */
            fp_t dist_ratio = sin(angle)/cos(angle);
            
            /* Parse the heigth of the picture */        
            find_next_line(&at);
            h  = get_next_float(&at);
            if (h == (fp_t)0.0)
            {
                h = (fp_t)1.0;
            }
            fp_t height_width = dist_ratio * h;
            
            /* Parse the resolution */        
            find_next_line(&at);
            x_res  = get_next_float(&at);
            y_res  = get_next_float(&at);

            vector_t x_vec(1.0, 0.0, 0.0);
            cross_product(up_vec, at_vec, &x_vec);
            cross_product(at_vec, x_vec, &up_vec);
            normalise(&x_vec);
            normalise(&up_vec);
            normalise(&at_vec);
            *c = new camera(from, x_vec, up_vec, at_vec, bg, height_width, height_width, h, (unsigned)x_res, (unsigned)y_res);
        }
        /* Parse a light */
        else if ((*at) == 'l')
        {
            fp_t x, y, z, r, g, b;
            r = (fp_t)255.0;
            g = (fp_t)255.0;
            b = (fp_t)255.0;
            
            x  = get_next_float(&at);
            y  = get_next_float(&at);
            z  = get_next_float(&at);
            
            /* Overload the input light colour with the colour specified in the file */
            if (!end_of_line(at))
            {
                r  = get_next_float(&at);
                g  = get_next_float(&at);
                b  = get_next_float(&at);
            }
            
            /* No fall off in intensity with distance is assumed */
            /* A radius of 0.1 is assumed */
            new_light(&l, ext_colour_t(r, g, b), point_t(x, y, z), 0.0, 0.1);
        }
        /* Parse material properties */
        else if (((*at) == 'f') && ((*(at+1)) == ' '))
        {
            ext_colour_t rgb;
            fp_t kd, ks, s, t, ri;
            
            rgb.r   = get_next_float(&at);
            rgb.g   = get_next_float(&at);
            rgb.b   = get_next_float(&at);
            kd      = get_next_float(&at);
            ks      = get_next_float(&at);
            s       = get_next_float(&at);
            t       = get_next_float(&at);
            ri      = get_next_float(&at);
            
            /* Scale the colour to rgb */
            rgb *= (fp_t)255.0;
        
            cur_mat = new phong_shader(rgb, kd, ks, s, t, ri, ks);
            m.push_back(cur_mat);
        }
        /* Parse a face */
        else if ((*at) == 'p')
        {
            /* Parse a polygn patch with vertex normals */
            if (at[1] == 'p')
            {
                /* Get the number of points on the face */
                unsigned p = (unsigned)get_next_float(&at);
                
                /* Parse each point */
                for (unsigned i=0; i<p; i++)
                {
                    /* Points */
                    fp_t x, y, z;
                    find_next_line(&at);
                    x = atof(at);
                    y = get_next_float(&at);
                    z = get_next_float(&at);
                    
                    points.emplace_back(x, y, z);
                    
                    /* Normals */
                    x = get_next_float(&at);
                    y = get_next_float(&at);
                    z = get_next_float(&at);
                    
                    normals.emplace_back(x, y, z);
                    normalise(&normals.back());
                }

                /* Convert face to triangles */
                face_to_triangles(&e, &l, points, cur_mat, false, &normals);

                /* Clean up normals */
                normals.clear();
            }
            /* Parse a face with no vertex normals */
            else
            {
                /* Get the number of points on the face */
                unsigned p = (unsigned)get_next_float(&at);
                
                /* Parse each point */
                for (unsigned int i = 0; i < p; i++)
                {
                    fp_t x, y, z;
                    find_next_line(&at);
                    x = atof(at);
                    y = get_next_float(&at);
                    z = get_next_float(&at);
                    
                    points.emplace_back(x, y, z);
                }

                /* Convert face to triangles */
                face_to_triangles(&e, &l, points, cur_mat, false);
            }
                
            /* Clean up */
            points.clear();
        }
        /* Parse a sphere */
        else if ((*at) == 's')
        {
//            fp_t x, y, z, r;
//            
//            x = get_next_float(&at);
//            y = get_next_float(&at);
//            z = get_next_float(&at);
//            r = get_next_float(&at);
            
            assert(!"Warning: Parsed un-used sphere");
//            new_sphere(&e, &l, cur_mat, point_t(x, y, z), r, false);
        }
        /* Parse a cone or cyclinder */
        else if ((*at) == 'c')
        {
            point_t t,  b;
            // fp_t    r0, r1;
            
            t.x = get_next_float(&at);
            t.y = get_next_float(&at);
            t.z = get_next_float(&at);
            // r0  = get_next_float(&at);
            b.x = get_next_float(&at);
            b.y = get_next_float(&at);
            b.z = get_next_float(&at);
            // r1  = get_next_float(&at);
            
            cout << "Warning: Parsed un-used cone" << endl;
            /* A cyclinder is a cone with the same radius at each end */
//             if (r0 == r1)
//             {
//                 /* Declare the cylinder */
// //                new_cylinder(&e, &l, cur_mat, t, b, r0, false);
//             }
//             else
//             {
//                 /* Declare the cone */
// //                new_cone(&e, &l, cur_mat, t, b, r0, r1, false);
//             }
        }
        else
        {
            cout << "Error: Unknown enitity: " << (*at) << endl;
        }

        /* Move onto the next line */
        find_next_line(&at);
    }

    /* Tidy up */
    delete [] buffer;
}
}; /* namespace raptor_raytracer */
