/* Raytracer headers */
#include "raytracer.h"
#include "common.h"
#include "scene.h"
#include "camera.h"
#include "cfg_parser.h"
#include "mgf_parser.h"
#include "nff_parser.h"
#include "lwo_parser.h"
#include "obj_parser.h"
#include "off_parser.h"
#include "ply_parser.h"
#include "vrml_parser.h"
#include "polygon_to_triangles.h"
#include "bih.h"
#include "kd_tree.h"

/* Display headers */
#include "sdl_wrapper.h"
#include "sdl_event_handler.h"
#include "sdl_event_handler_factory.h"


/*****************************************************
 Function to write an helpful message to the screen.
*****************************************************/
void help()
{
    std::cout << "Usage: raytracer [-i] [-f file_name] [-mgf|-nff|-lwo|-obj|-vrml] [-cam x y z]"                              << std::endl;
    std::cout << "                 [-at x y z] [-rx x] [-ry x] [-rz x] [-bg r g b]  "            	                          << std::endl;
    std::cout << "                 [-light x y z ra r g b d]"                                                                 << std::endl;
    std::cout << "       -i                      	                    : enables interactive mode."                          << std::endl;
    std::cout << "       -tga        f                                   : f is tga snapshot file."                           << std::endl;
    std::cout << "       -png        f                                   : f is png snapshot file."                           << std::endl;
    std::cout << "       -jpg        f q                                 : f is jpeg snapshot file. q is image quality."      << std::endl;
    std::cout << "       -mgf        f                                   : f is mgf format scene."                            << std::endl;
    std::cout << "       -nff        f                                   : f is nff format scene."                            << std::endl;
    std::cout << "       -lwo        f                                   : f is lwo format scene."                            << std::endl;
    std::cout << "       -obj        f                                   : f is obj format scene."                            << std::endl;
    std::cout << "       -vrml       f v                                 : f is vrml format scene. v is the view point."      << std::endl;
    std::cout << "       -res        x y                                 : x y are the image resolutions."                    << std::endl;
    std::cout << "       -anti_alias x y                                 : x y are the image anti-aliasing factors."          << std::endl;
    std::cout << "       -cam        x y z                               : x y z are the camera's co-ordinate."               << std::endl;
    std::cout << "       -at         x y z                               : x y z are the co-ordinates looked at."             << std::endl;
    std::cout << "       -rx         x                                   : x is an angle to rotate about the x axis."         << std::endl;
    std::cout << "       -ry         x                                   : x is an angle to rotate about the y axis."         << std::endl;
    std::cout << "       -rz         x                                   : x is an angle to rotate about the z axis."         << std::endl;
    std::cout << "       -bg         r g b                               : r g b is the rgb background colour."               << std::endl;
    std::cout << "       -light      x y z ra r g b d                    : x y z are the lights co-ordinates."                << std::endl;
    std::cout << "                               	                      ra is the lights radius."                           << std::endl;
    std::cout << "                               	                      r g b is the lights colour."                        << std::endl;
    std::cout << "                               	                      d intensity fall off with distance scale."          << std::endl;
    std::cout << "       -spotlight  cx cy cz ra r g b d ax ay az sa sb  : cx cy cz are the lights co-ordinates."             << std::endl;
    std::cout << "                               	                      ra is the lights radius."                           << std::endl;
    std::cout << "                               	                      r g b is the lights colour."                        << std::endl;
    std::cout << "                               	                      d intensity fall off with distance scale."          << std::endl;
    std::cout << "                               	                      ax ay az are the lights target co-ordinates."       << std::endl;
    std::cout << "                               	                      sa is the angle at which the light begins to fade." << std::endl;
    std::cout << "                               	                      sb is the angle at which the light has to faded."   << std::endl;
    std::cout << "       -directionallight  cx cy cz ra g b d ax ay az   : cx cy cz are the lights co-ordinates."             << std::endl;
    std::cout << "                               	                      r g b is the lights colour."                        << std::endl;
    std::cout << "                               	                      d intensity fall off with distance scale."          << std::endl;
    std::cout << "                               	                      ax ay az are the lights target co-ordinates."       << std::endl;
    std::cout << ""                                                                              	                          << std::endl;
    std::cout << " Any number of lights may be added."                                           	                          << std::endl;
    std::cout << " All angles are in degrees."                                                                                << std::endl;
    std::cout << " The last camera parameters specified will take presidence."                   	                          << std::endl;
    std::cout << " The input file specified will take presidence."                               	                          << std::endl;
    std::cout << ""                                                                              	                          << std::endl;
    std::cout << "----------------------------------------"                                      	                          << std::endl;
    std::cout << "-      Interactive key mappings        -"                                      	                          << std::endl;
    std::cout << "----------------------------------------"                                      	                          << std::endl;
    std::cout << "   w           :   move forward."                                              	                          << std::endl;
    std::cout << "   s           :   move backward."                                             	                          << std::endl;
    std::cout << "   a           :   move left."                                                 	                          << std::endl;
    std::cout << "   d           :   move right."                                                	                          << std::endl;
    std::cout << "   right arrow :   look left."                                                 	                          << std::endl;
    std::cout << "   left arrow  :   look right."                                                	                          << std::endl;
    std::cout << "   up arrow    :   look up."                                                   	                          << std::endl;
    std::cout << "   down arrow  :   look down."                                                 	                          << std::endl;
    std::cout << "   space       :   move up."                                                   	                          << std::endl;
    std::cout << "   ctrl        :   move down."                                                 	                          << std::endl;
    std::cout << "   page up     :   roll anti-clockwise."                                       	                          << std::endl;
    std::cout << "   page down   :   roll clockwise."                                            	                          << std::endl;
    std::cout << "   escape      :   quit."                                                      	                          << std::endl;
    std::cout << "   q           :   quit."                                                      	                          << std::endl;
    std::cout << "   f           :   snapshot."                                                  	                          << std::endl;
    std::cout << "   p           :   dump position data."                                        	                          << std::endl;
}


/*****************************************************
 The main function.
*****************************************************/
int main (int argc, char **argv)
{
    using raptor_raytracer::model_format_t;
    using raptor_raytracer::image_format_t;
    using raptor_raytracer::ext_colour_t;

    /* Default parameters */
    bool            interactive     = false;
    model_format_t  input_format    = model_format_t::code;
    image_format_t  image_format    = image_format_t::tga;
    int             jpg_quality     = 50;
    ext_colour_t    bg;
    std::string     input_file;
    std::string     view_point;
    std::string     output_file     = "snapshot";
    std::string     caption         = "raytracer ";

    /* Camera parameters */
    raptor_raytracer::camera   *cam = nullptr;
    point_t  cam_p( 0.0f, 0.0f, -10.0f);    /* Position                         */
    point_t  x_vec( 1.0f, 0.0f, 0.0f);      /* Horizontal vector                */
    point_t  y_vec( 0.0f, 1.0f, 0.0f);      /* Virtical vector                  */
    point_t  z_vec( 0.0f, 0.0f, 1.0f);      /* Forward vector                   */
    float    rx = 0.0f;                     /* Rotation about horizontal vector */
    float    ry = 0.0f;                     /* Rotation about virtical vector   */
    float    rz = 0.0f;                     /* Rotation about forward vector    */
    unsigned xr = 640;                      /* X resolution                     */
    unsigned yr = 480;                      /* Y resolution                     */
    unsigned xa = 1;                        /* X anti-aliasing factor           */
    unsigned ya = 1;                        /* Y anti-aliasing factor           */

    /* Scene data */
    raptor_raytracer::light_list            lights;
    raptor_raytracer::primitive_store       everything;
    std::list<raptor_raytracer::material *> materials;

    /* Parse input arguements */
    if (argc > 1)
    {
        for (int i=1; i<argc; i++)
        {
            if ((argv[i][0] == '-') && (argv[i][1] == '-'))
            {
                argv[i]++;
            }
            
            /* Run in interactive mode */
            if (strcmp(argv[i], "-i") == 0)
            {
                interactive = true;
                caption += "-i ";
            }
            /* TGA output */
            else if (strcmp(argv[i], "-tga") == 0)
            {
                if ((argc - i) < 2)
                {
                    std::cout << "Incorrectly specified tga output file" << std::endl;
                    help();
                    return 1;
                }
                
                image_format    = image_format_t::tga;
                output_file     = argv[++i];
                caption        += "-tga ";
                caption        += argv[i];
                caption        += " ";
            }
            /* PNG output */
            else if (strcmp(argv[i], "-png") == 0)
            {
                if ((argc - i) < 2)
                {
                    std::cout << "Incorrectly specified png output file" << std::endl;
                    help();
                    return 1;
                }

                image_format    = image_format_t::png;
                output_file     = argv[++i];
                caption        += "-png ";
                caption        += argv[i];
                caption        += " ";
            }
            /* JPEG output */
            else if (strcmp(argv[i], "-jpg") == 0)
            {
                if ((argc - i) < 3)
                {
                    std::cout << "Incorrectly specified jpeg output file" << std::endl;
                    help();
                    return 1;
                }
                
                image_format    = image_format_t::jpg;
                output_file     = argv[++i];
                caption        += "-jpg ";
                caption        += argv[i];
                caption        += " ";
                jpg_quality     = atoi(argv[++i]);
                assert(jpg_quality >=   0);
                assert(jpg_quality <= 100);
            }
            /* CFG input */
            else if (strcmp(argv[i], "-cfg") == 0)
            {
                if ((argc - i) < 2)
                {
                    std::cout << "Incorrectly specified cfg file" << std::endl;
                    help();
                    return 1;
                }
                
                input_format    = model_format_t::cfg;
                input_file      = argv[++i];
            }
            /* MGF input */
            else if (strcmp(argv[i], "-mgf") == 0)
            {
                if ((argc - i) < 2)
                {
                    std::cout << "Incorrectly specified mgf file" << std::endl;
                    help();
                    return 1;
                }
                
                input_format    = model_format_t::mgf;
                input_file      = argv[++i];
            }
            /* NFF input */
            else if (strcmp(argv[i], "-nff") == 0)
            {
                if ((argc - i) < 2)
                {
                    std::cout << "Incorrectly specified nff file" << std::endl;
                    help();
                    return 1;
                }
                
                input_format    = model_format_t::nff;
                input_file      = argv[++i];
            }
            /* LWO input */
            else if (strcmp(argv[i], "-lwo") == 0)
            {
                if ((argc - i) < 2)
                {
                    std::cout << "Incorrectly specified lwo file" << std::endl;
                    help();
                    return 1;
                }
                
                input_format    = model_format_t::lwo;
                input_file      = argv[++i];
            }
            /* OBJ input */
            else if (strcmp(argv[i], "-obj") == 0)
            {
                if ((argc - i) < 2)
                {
                    std::cout << "Incorrectly specified obj file" << std::endl;
                    help();
                    return 1;
                }
                
                input_format    = model_format_t::obj;
                input_file      = argv[++i];
            }
            /* OFF input */
            else if (strcmp(argv[i], "-off") == 0)
            {
                if ((argc - i) < 2)
                {
                    std::cout << "Incorrectly specified off file" << std::endl;
                    help();
                    return 1;
                }
                
                input_format    = model_format_t::off;
                input_file      = argv[++i];
            }
            /* PLY input */
            else if (strcmp(argv[i], "-ply") == 0)
            {
                if ((argc - i) < 2)
                {
                    std::cout << "Incorrectly specified ply file" << std::endl;
                    help();
                    return 1;
                }
                
                input_format    = model_format_t::ply;
                input_file      = argv[++i];
            }
            /* VRML input */
            else if (strcmp(argv[i], "-vrml") == 0)
            {
                if ((argc - i) < 3)
                {
                    std::cout << "Incorrectly specified vrml file" << std::endl;
                    help();
                    return 1;
                }
                
                input_format    = model_format_t::vrml;
                input_file      = argv[++i];
                view_point      = argv[++i];
            }
            /* Resolution */
            else if (strcmp(argv[i], "-res") == 0)
            {
                if ((argc - i) < 3)
                {
                    std::cout << "Incorrectly specified resolution" << std::endl;
                    help();
                    return 1;
                }
                
                xr = atoi(argv[++i]);
                yr = atoi(argv[++i]);
            }
            /* Anti-aliasing factor */
            else if (strcmp(argv[i], "-anti_alias") == 0)
            {
                if ((argc - i) < 3)
                {
                    std::cout << "Incorrectly specified anti-aliasing factor" << std::endl;
                    help();
                    return 1;
                }
                
                xa = atoi(argv[++i]);
                ya = atoi(argv[++i]);
            }
            /* Camera position */
            else if (strcmp(argv[i], "-cam") == 0)
            {
                if ((argc - i) < 4)
                {
                    std::cout << "Incorrectly specified camera position" << std::endl;
                    help();
                    return 1;
                }
                
                cam_p.x = atof(argv[++i]);
                cam_p.y = atof(argv[++i]);
                cam_p.z = atof(argv[++i]);
            }
            /* At vector */
            else if (strcmp(argv[i], "-at") == 0)
            {
                if ((argc - i) < 4)
                {
                    std::cout << "Incorrectly specified model position" << std::endl;
                    help();
                    return 1;
                }
                
                z_vec.x = atof(argv[++i]);
                z_vec.y = atof(argv[++i]);
                z_vec.z = atof(argv[++i]);
                
                z_vec -= cam_p;
                normalise(&z_vec);
    
                cross_product(y_vec, z_vec, &x_vec);
                cross_product(z_vec, x_vec, &y_vec);
                normalise(&x_vec);
                normalise(&y_vec);
                normalise(&z_vec);
            }            
            /* X vector */
            else if (strcmp(argv[i], "-dx") == 0)
            {
                if ((argc - i) < 4)
                {
                    std::cout << "Incorrectly specified x vector" << std::endl;
                    help();
                    return 1;
                }
                
                x_vec.x = atof(argv[++i]);
                x_vec.y = atof(argv[++i]);
                x_vec.z = atof(argv[++i]);
            }
            /* Y vector */
            else if (strcmp(argv[i], "-dy") == 0)
            {
                if ((argc - i) < 4)
                {
                    std::cout << "Incorrectly specified y vector" << std::endl;
                    help();
                    return 1;
                }
                
                y_vec.x = atof(argv[++i]);
                y_vec.y = atof(argv[++i]);
                y_vec.z = atof(argv[++i]);
            }
            /* Z vector */
            else if (strcmp(argv[i], "-dz") == 0)
            {
                if ((argc - i) < 4)
                {
                    std::cout << "Incorrectly specified z vector" << std::endl;
                    help();
                    return 1;
                }
                
                z_vec.x = atof(argv[++i]);
                z_vec.y = atof(argv[++i]);
                z_vec.z = atof(argv[++i]);
            }
            /* X rotation */
            else if (strcmp(argv[i], "-rx") == 0)
            {
                if ((argc - i) < 2)
                {
                    std::cout << "Incorrectly specified x rotate" << std::endl;
                    help();
                    return 1;
                }
                
                rx = static_cast<float>(atof(argv[++i])) * (PI / 180.0f);
            }
            /* Y rotation */
            else if (strcmp(argv[i], "-ry") == 0)
            {
                if ((argc - i) < 2)
                {
                    std::cout << "Incorrectly specified y rotate" << std::endl;
                    help();
                    return 1;
                }
                
                ry = static_cast<float>(atof(argv[++i])) * (PI / 180.0f);
            }
            /* Z rotation */
            else if (strcmp(argv[i], "-rz") == 0)
            {
                if ((argc - i) < 2)
                {
                    std::cout << "Incorrectly specified z rotate" << std::endl;
                    help();
                    return 1;
                }
                
                rz = static_cast<float>(atof(argv[++i])) * (PI / 180.0f);
            }
            /* Background */
            else if (strcmp(argv[i], "-bg") == 0)
            {
                if ((argc - i) < 4)
                {
                    std::cout << "Incorrectly specified background colour" << std::endl;
                    help();
                    return 1;
                }
                
                bg.r = atof(argv[++i]);
                bg.g = atof(argv[++i]);
                bg.b = atof(argv[++i]);
            }
            /* Light */
            else if (strcmp(argv[i], "-light") == 0)
            {
                if ((argc - i) < 9)
                {
                    std::cout << "Incorrectly specified light" << std::endl;
                    help();
                    return 1;
                }
                ext_colour_t    rgb;
                point_t         c;
                float           r;
                float           d;
                
                c.x   = atof(argv[++i]);
                c.y   = atof(argv[++i]);
                c.z   = atof(argv[++i]);
                r     = atof(argv[++i]);
                rgb.r = atof(argv[++i]);
                rgb.g = atof(argv[++i]);
                rgb.b = atof(argv[++i]);
                d     = atof(argv[++i]);
                
//                triangle** light_area = new triangle* [12];
//                
//                point_t px0(c.x + (r/2.0), c.y + (r/2.0), c.z + (r/2.0));
//                point_t px1(c.x + (r/2.0), c.y - (r/2.0), c.z + (r/2.0));
//                point_t px2(c.x + (r/2.0), c.y - (r/2.0), c.z - (r/2.0));
//                point_t px3(c.x + (r/2.0), c.y + (r/2.0), c.z - (r/2.0));
//                point_t mx0(c.x - (r/2.0), c.y + (r/2.0), c.z + (r/2.0));
//                point_t mx1(c.x - (r/2.0), c.y - (r/2.0), c.z + (r/2.0));
//                point_t mx2(c.x - (r/2.0), c.y - (r/2.0), c.z - (r/2.0));
//                point_t mx3(c.x - (r/2.0), c.y + (r/2.0), c.z - (r/2.0));
//
//                material *light_mat = new phong_shader(ext_colour_t(255.0), 1.0);
//                materials.push_back(light_mat);
//                
//                light_area[ 0] = new triangle(light_mat, px0, px1, px2, true);
//                light_area[ 1] = new triangle(light_mat, px2, px3, px0, true);
//                light_area[ 2] = new triangle(light_mat, px0, px1, mx1, true);
//                light_area[ 3] = new triangle(light_mat, mx1, mx0, px0, true);
//                light_area[ 4] = new triangle(light_mat, px1, px2, mx2, true);
//                light_area[ 5] = new triangle(light_mat, mx2, mx1, px1, true);
//                light_area[ 6] = new triangle(light_mat, px2, px3, mx3, true);
//                light_area[ 7] = new triangle(light_mat, mx3, mx2, px2, true);
//                light_area[ 8] = new triangle(light_mat, px3, px0, mx0, true);
//                light_area[ 9] = new triangle(light_mat, mx0, mx3, px3, true);
//                light_area[10] = new triangle(light_mat, mx0, mx1, mx2, true);
//                light_area[11] = new triangle(light_mat, mx2, mx3, mx0, true);
//                everything.push_back(light_area[ 0]);
//                everything.push_back(light_area[ 1]);
//                everything.push_back(light_area[ 2]);
//                everything.push_back(light_area[ 3]);
//                everything.push_back(light_area[ 4]);
//                everything.push_back(light_area[ 5]);
//                everything.push_back(light_area[ 6]);
//                everything.push_back(light_area[ 7]);
//                everything.push_back(light_area[ 8]);
//                everything.push_back(light_area[ 9]);
//                everything.push_back(light_area[10]);
//                everything.push_back(light_area[11]);
             
                new_light(&lights, rgb, c, d, r);
            }
            /* Spotlight */
            else if (strcmp(argv[i], "-spotlight") == 0)
            {
                if ((argc - i) < 14)
                {
                    std::cout << "Incorrectly specified spotlight" << std::endl;
                    help();
                    return 1;
                }

                ext_colour_t    rgb;
                point_t         c;
                point_t         a;
                point_t         n;
                float           r;
                float           d;
                float           s_a;
                float           s_b;
                
                c.x     = atof(argv[++i]);
                c.y     = atof(argv[++i]);
                c.z     = atof(argv[++i]);
                r       = atof(argv[++i]);
                rgb.r   = atof(argv[++i]);
                rgb.g   = atof(argv[++i]);
                rgb.b   = atof(argv[++i]);
                d       = atof(argv[++i]);
                a.x     = atof(argv[++i]);
                a.y     = atof(argv[++i]);
                a.z     = atof(argv[++i]);
                s_a     = static_cast<float>(atof(argv[++i])) * (PI / 180.0f);
                s_b     = static_cast<float>(atof(argv[++i])) * (PI / 180.0f);
                
                n = c - a;
                normalise(&n);
                raptor_raytracer::new_light(&lights, rgb, c, n, d, s_a, s_b, r);
            }
            /* Directional light */
            else if (strcmp(argv[i], "-directionallight") == 0)
            {
                if ((argc - i) < 11)
                {
                    std::cout << "Incorrectly specified directional light" << std::endl;
                    help();
                    return 1;
                }

                ext_colour_t    rgb;
                point_t         c;
                point_t         a;
                point_t         n;
                float           d;
                
                c.x     = atof(argv[++i]);
                c.y     = atof(argv[++i]);
                c.z     = atof(argv[++i]);
                rgb.r   = atof(argv[++i]);
                rgb.g   = atof(argv[++i]);
                rgb.b   = atof(argv[++i]);
                d       = atof(argv[++i]);
                a.x     = atof(argv[++i]);
                a.y     = atof(argv[++i]);
                a.z     = atof(argv[++i]);
                
                n = c - a;
                normalise(&n);
                raptor_raytracer::new_light(&lights, rgb, n, d);
            }
            else
            {
                std::cout << "Unknown option: " << argv[i] << std::endl;
                help();
                return 1;
            }
        }
    }

    /* initialise the scene */
    std::ifstream   input_stream;
    std::string     path;
    size_t          last_slash;
    
    /* Get the screen aspect ratio */
    const float screen_width    = 10.0f;
    const float screen_height   = screen_width * (static_cast<float>(yr) / static_cast<float>(xr));
    switch (input_format)
    {
        case model_format_t::cfg :
        {
            input_stream.open(input_file.c_str());
            assert(input_stream.is_open());
            const unsigned int path_end = input_file.find_last_of("/\\") + 1;
            raptor_raytracer::cfg_parser(input_file.substr(0, path_end), input_stream, lights, everything, materials, &cam);

            /* If camera is not set in the scene so do it here (an nff file would have set the camera) */
            if (cam == nullptr)
            {
                cam = new raptor_raytracer::camera(cam_p, x_vec, y_vec, z_vec, bg, screen_width, screen_height, 20, xr, yr, xa, ya);
            }
            break;
        }
        case model_format_t::code :
            raptor_raytracer::scene_init(lights, everything, materials, &cam);
            break;

        case model_format_t::mgf :
            raptor_raytracer::mgf_parser(input_file.c_str(), lights, everything, materials);
            
            /* Camera is not set in the scene so do it here */
            cam = new raptor_raytracer::camera(cam_p, x_vec, y_vec, z_vec, bg, screen_width, screen_height, 10, xr, yr, xa, ya);
            break;

        case model_format_t::nff :
            input_stream.open(input_file.c_str());
            assert(input_stream.is_open());
            raptor_raytracer::nff_parser(input_stream, lights, everything, materials, &cam);
            break;

        case model_format_t::lwo :
            input_stream.open(input_file.c_str());
            assert(input_stream.is_open());
            last_slash  = input_file.find_last_of('/');
            path        = input_file.substr(0, last_slash + 1);
            raptor_raytracer::lwo_parser(input_stream, path, lights, everything, materials, &cam);

            /* Camera is not set in the scene so do it here */
            cam = new raptor_raytracer::camera(cam_p, x_vec, y_vec, z_vec, bg, screen_width, screen_height, 20, xr, yr, xa, ya);
            break;
            
        case model_format_t::obj :
            input_stream.open(input_file.c_str());
            assert(input_stream.is_open());
            last_slash  = input_file.find_last_of('/');
            path        = input_file.substr(0, last_slash + 1);
            raptor_raytracer::obj_parser(input_stream, path, lights, everything, materials, &cam);
            
            /* Camera is not set in the scene so do it here */
            cam = new raptor_raytracer::camera(cam_p, x_vec, y_vec, z_vec, bg, screen_width, screen_height, 20, xr, yr, xa, ya);
            break;
            
        case model_format_t::off :
            input_stream.open(input_file.c_str());
            assert(input_stream.is_open());
            raptor_raytracer::off_parser(input_stream, lights, everything, materials, cam);
            
            /* Camera is not set in the scene so do it here */
            cam = new raptor_raytracer::camera(cam_p, x_vec, y_vec, z_vec, bg, screen_width, screen_height, 20, xr, yr, xa, ya);
            break;

        case model_format_t::ply :
            input_stream.open(input_file.c_str());
            assert(input_stream.is_open());
            raptor_raytracer::ply_parser(input_stream, lights, everything, materials, &cam);
            
            /* Camera is not set in the scene so do it here */
            cam = new raptor_raytracer::camera(cam_p, x_vec, y_vec, z_vec, bg, screen_width, screen_height, 20, xr, yr, xa, ya);
            break;

        case model_format_t::vrml :
            /* Deafult camera set up for vrml -- NOTE negative z axis */
            cam = new raptor_raytracer::camera(cam_p, point_t(1.0f, 0.0f,  0.0f), point_t(0.0f, 1.0f,  0.0f), 
                                    point_t(0.0f, 0.0f, -1.0f), bg, screen_width, screen_height, 20, xr, yr, xa, ya);

            input_stream.open(input_file.c_str());
            assert(input_stream.is_open());
            raptor_raytracer::vrml_parser(input_stream, lights, everything, materials, cam, view_point);
            break;

        default :
            assert(false);
            break;
    }
    
    /* Assert there must be some lights to light the scene */
    assert(!lights.empty());

    /* Assert there must be some materials for the objects */
    assert(!materials.empty());

    /* Assert there is an imagine to trace */
    assert(!everything.empty());
    
    /* Pan tilt and roll the camera */
    cam->tilt(rx);
    cam->pan(ry);
    cam->roll(rz);

    /* Build spatial sub division */
    raptor_raytracer::kd_tree ssd(everything);
    
    /* Run in interactive mode */
    if (interactive)
    {
        /* Initialise and lock the screen */
        SDL_Window *window;
        SDL_Texture *texture;
        SDL_Renderer *renderer;
        if (sdl_set_up(&window, &renderer, &texture, caption.c_str(), cam->x_resolution(), cam->y_resolution()))
        {
            return 1;
        }

        /* Run until exitted */
        int do_next = 0;
        std::unique_ptr<sdl_event_handler_base> cam_event_handler(get_camera_event_handler(cam, output_file, jpg_quality, image_format));
        std::unique_ptr<unsigned char[]> screen_data(new unsigned char [cam->x_resolution() * cam->y_resolution() * 3]);
        while(do_next != 1) 
        {
            /* Poll for user input */
            do_next = cam_event_handler->process_events();
            
            /* Draw if required */
            if (do_next == 0)
            {
                /* Display output for interactive mode */
                ray_tracer(&ssd, lights, everything, *cam);

                /* Display the output */
                cam->clip_image_to_bgr(screen_data.get());

                const int draw_status = draw_screen(renderer, texture, screen_data.get());
                if (draw_status)
                {
                    return draw_status;
                }
            }
        }

        /* SDL clean up */
        sdl_clean_up(window, renderer, texture, nullptr);
    }
    /* Produce a single image */
    else
    {
        /* Ray trace the scene */
        ray_tracer(&ssd, lights, everything, *cam);
        
        /* Tone mapping */
        //cam->tone_map(local_human_histogram, 0.75, (1.0/3.0), (1.0/3.0), false, false, false);

        /* Output image */
        std::ostringstream file_name(std::ostringstream::out);
        switch (image_format)
        {
            case image_format_t::tga :
                file_name << output_file << "_0.tga";
                cam->write_tga_file(file_name.str());
                break;
            case image_format_t::jpg :
                file_name << output_file << "_0.jpg";
                cam->write_jpeg_file(file_name.str(), jpg_quality);
                break;
            case image_format_t::png :
                file_name << output_file << "_0.png";
                cam->write_png_file(file_name.str());
                break;
            default :
                assert(!"Error unknown image format");
                break;
        }
    }

    /* Clean up dynamic memory usage */
    raptor_raytracer::scene_clean(&materials, cam);
    return 0;
}
