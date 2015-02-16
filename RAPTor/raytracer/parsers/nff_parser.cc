#include "common.h"
#include "parser_common.h"

#include "camera.h"

#include "nff_parser.h"


namespace raptor_raytracer
{
void nff_parser(
    std::ifstream           &nff_file,
    light_list              &l, 
    primitive_list          &e,
    std::list<material *>   &m,
    camera                  **c)
{
    /* Find the size of the file */
    nff_file.seekg(0, std::ios::end);
    size_t len = nff_file.tellg();
    nff_file.seekg(0, std::ios::beg);
    
    /* Read the whole file into a buffer */
    char *buffer = new char [len];
    nff_file.read(buffer, len);
    const char *at = &buffer[0];
    
    /* The current material properties to be used */
    phong_shader *cur_mat = nullptr;
    
    /* The background colour */
    ext_colour_t bg;

    /* Vector for face to triangle conversion */
    std::vector<point_t> points;
    std::vector<point_t> normals;

    /* Parse the file */
    while (at < &buffer[len - 1])
    {
        /* Parse the background colour */
        if ((*at) == 'b')
        {
            bg.r = get_next_float(&at);
            bg.g = get_next_float(&at);
            bg.b = get_next_float(&at);
            
            /* Scale the colour to rgb */
            bg *= 255.0f;
        }
        /* Parse viewing vectors and angles */
        else if ((*at) == 'v')
        {
            /* Parse the view position */
            find_next_line(&at);
            
            float x, y, z, a, h;
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
            point_t at_vec = to - from;
            
            /* Parse up direction */        
            find_next_line(&at);
            x  = get_next_float(&at);
            y  = get_next_float(&at);
            z  = get_next_float(&at);
            point_t up_vec(x, y, z);
            
            /* Parse angle of view */        
            find_next_line(&at);
            a  = get_next_float(&at);
            float angle = a * (PI / 360.0f); /* This is half of the angle in rad to make a right angle triangle */
            float dist_ratio = sin(angle)/cos(angle);
            
            /* Parse the heigth of the picture */        
            find_next_line(&at);
            h  = get_next_float(&at);
            if (h == 0.0f)
            {
                h = 1.0f;
            }
            const float height_width = dist_ratio * h;
            
            /* Parse the resolution */        
            find_next_line(&at);
            const float x_res  = get_next_float(&at);
            const float y_res  = get_next_float(&at);

            point_t x_vec(1.0f, 0.0f, 0.0f);
            cross_product(up_vec, at_vec, &x_vec);
            cross_product(at_vec, x_vec, &up_vec);
            normalise(&x_vec);
            normalise(&up_vec);
            normalise(&at_vec);
            *c = new camera(from, x_vec, up_vec, at_vec, bg, height_width, height_width, h, static_cast<unsigned int>(x_res), static_cast<unsigned int>(y_res));
        }
        /* Parse a light */
        else if ((*at) == 'l')
        {
            float r, g, b;
            r = 255.0f;
            g = 255.0f;
            b = 255.0f;
            
            const float x  = get_next_float(&at);
            const float y  = get_next_float(&at);
            const float z  = get_next_float(&at);
            
            /* Overload the input light colour with the colour specified in the file */
            if (!end_of_line(at))
            {
                r  = get_next_float(&at);
                g  = get_next_float(&at);
                b  = get_next_float(&at);
            }
            
            /* No fall off in intensity with distance is assumed */
            /* A radius of 0.1 is assumed */
            new_light(&l, ext_colour_t(r, g, b), point_t(x, y, z), 0.0f, 0.1f);
        }
        /* Parse material properties */
        else if (((*at) == 'f') && ((*(at+1)) == ' '))
        {
            ext_colour_t rgb;
            rgb.r   = get_next_float(&at);
            rgb.g   = get_next_float(&at);
            rgb.b   = get_next_float(&at);

            const float kd  = get_next_float(&at);
            const float ks  = get_next_float(&at);
            const float s   = get_next_float(&at);
            const float t   = get_next_float(&at);
            const float ri  = get_next_float(&at);
            
            /* Scale the colour to rgb */
            rgb *= 255.0f;
        
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
                const unsigned int p = static_cast<unsigned int>(get_next_float(&at));
                
                /* Parse each point */
                for (unsigned i = 0; i < p; i++)
                {
                    /* Points */
                    float x, y, z;
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
                const unsigned int p = static_cast<unsigned int>(get_next_float(&at));
                
                /* Parse each point */
                for (unsigned int i = 0; i < p; i++)
                {
                    find_next_line(&at);
                    const float x = atof(at);
                    const float y = get_next_float(&at);
                    const float z = get_next_float(&at);
                    
                    points.emplace_back(x, y, z);
                }

                /* Convert face to triangles */
                face_to_triangles(&e, &l, points, cur_mat, false);
            }
                
            /* Clean up */
            points.clear();
        }
        else
        {
            std::cout << "Error: Unknown enitity: " << (*at) << std::endl;
        }

        /* Move onto the next line */
        find_next_line(&at);
    }

    /* Tidy up */
    delete [] buffer;
}
}; /* namespace raptor_raytracer */
