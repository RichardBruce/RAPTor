#include "common.h"
#include "scene.h"
#include "camera.h"
#include "cfg_parser.h"
#include "mgf_parser.h"
#include "nff_parser.h"
#include "lwo_parser.h"
#include "obj_parser.h"
#include "ply_parser.h"
#include "vrml_parser.h"
#include "raytracer.h"

/* Display headers */
#include "sdl_wrapper.h"
#include "sdl_event_handler.h"
#include "sdl_event_handler_factory.h"

using namespace raptor_raytracer;


/*****************************************************
 Function to write an helpful message to the screen.
*****************************************************/
void help()
{
    cout << "Usage: raytracer [-i] [-f file_name] [-mgf|-nff|-lwo|-obj|-vrml] [-cam x y z]"                                 << endl;
    cout << "                 [-at x y z] [-rx x] [-ry x] [-rz x] [-bg r g b]  "            	                            << endl;
    cout << "                 [-light x y z ra r g b d]"                                                                    << endl;
    cout << "       -i                      	                    : enables interactive mode."                            << endl;
    cout << "       -tga        f                                   : f is tga snapshot file."                              << endl;
    cout << "       -png        f                                   : f is png snapshot file."                              << endl;
    cout << "       -jpg        f q                                 : f is jpeg snapshot file. q is image quality."         << endl;
    cout << "       -mgf        f                                   : f is mgf format scene."                               << endl;
    cout << "       -nff        f                                   : f is nff format scene."                               << endl;
    cout << "       -lwo        f                                   : f is lwo format scene."                               << endl;
    cout << "       -obj        f                                   : f is obj format scene."                               << endl;
    cout << "       -vrml       f v                                 : f is vrml format scene. v is the view point."         << endl;
    cout << "       -res        x y                                 : x y are the image resolutions."                       << endl;
    cout << "       -anti_alias x y                                 : x y are the image anti-aliasing factors."             << endl;
    cout << "       -cam        x y z                               : x y z are the camera's co-ordinate."                  << endl;
    cout << "       -at         x y z                               : x y z are the co-ordinates looked at."                << endl;
    cout << "       -rx         x                                   : x is an angle to rotate about the x axis."            << endl;
    cout << "       -ry         x                                   : x is an angle to rotate about the y axis."            << endl;
    cout << "       -rz         x                                   : x is an angle to rotate about the z axis."            << endl;
    cout << "       -bg         r g b                               : r g b is the rgb background colour."                  << endl;
    cout << "       -light      x y z ra r g b d                    : x y z are the lights co-ordinates."                   << endl;
    cout << "                               	                      ra is the lights radius."                             << endl;
    cout << "                               	                      r g b is the lights colour."                          << endl;
    cout << "                               	                      d intensity fall off with distance scale."            << endl;
    cout << "       -spotlight  cx cy cz ra r g b d ax ay az sa sb  : cx cy cz are the lights co-ordinates."                << endl;
    cout << "                               	                      ra is the lights radius."                             << endl;
    cout << "                               	                      r g b is the lights colour."                          << endl;
    cout << "                               	                      d intensity fall off with distance scale."            << endl;
    cout << "                               	                      ax ay az are the lights target co-ordinates."         << endl;
    cout << "                               	                      sa is the angle at which the light begins to fade."   << endl;
    cout << "                               	                      sb is the angle at which the light has to faded."     << endl;
    cout << "       -directionallight  cx cy cz ra g b d ax ay az   : cx cy cz are the lights co-ordinates."                << endl;
    cout << "                               	                      r g b is the lights colour."                          << endl;
    cout << "                               	                      d intensity fall off with distance scale."            << endl;
    cout << "                               	                      ax ay az are the lights target co-ordinates."         << endl;
    cout << ""                                                                              	                            << endl;
    cout << " Any number of lights may be added."                                           	                            << endl;
    cout << " All angles are in degrees."                                                                                   << endl;
    cout << " The last camera parameters specified will take presidence."                   	                            << endl;
    cout << " The input file specified will take presidence."                               	                            << endl;
    cout << ""                                                                              	                            << endl;
    cout << "----------------------------------------"                                      	                            << endl;
    cout << "-      Interactive key mappings        -"                                      	                            << endl;
    cout << "----------------------------------------"                                      	                            << endl;
    cout << "   w           :   move forward."                                              	                            << endl;
    cout << "   s           :   move backward."                                             	                            << endl;
    cout << "   a           :   move left."                                                 	                            << endl;
    cout << "   d           :   move right."                                                	                            << endl;
    cout << "   right arrow :   look left."                                                 	                            << endl;
    cout << "   left arrow  :   look right."                                                	                            << endl;
    cout << "   up arrow    :   look up."                                                   	                            << endl;
    cout << "   down arrow  :   look down."                                                 	                            << endl;
    cout << "   space       :   move up."                                                   	                            << endl;
    cout << "   ctrl        :   move down."                                                 	                            << endl;
    cout << "   page up     :   roll anti-clockwise."                                       	                            << endl;
    cout << "   page down   :   roll clockwise."                                            	                            << endl;
    cout << "   escape      :   quit."                                                      	                            << endl;
    cout << "   q           :   quit."                                                      	                            << endl;
    cout << "   f           :   snapshot."                                                  	                            << endl;
    cout << "   p           :   dump position data."                                        	                            << endl;
}


/*****************************************************
 Function to write to process user key presses. 
 
 The presses key is used to move the view point and 
 to rotate the angle of view. A snapshot may be taken,
 this is processed using a camera output function.
 
 true is returned if the program should exit. false is
 returned otherwise.
*****************************************************/
inline int key_press(camera *const c, const string &output_file, const SDL_Keycode key_pressed, const image_format_t image_format, const int jpg_quality)
{
    /* Static to unique output file naming */
    static unsigned int snapshot_nr = 0;
    ostringstream file_name(ostringstream::out);
    
#ifdef LOG_DEPTH                
    ostringstream depth_map_name(ostringstream::out);
#endif 


    /* Find which key was pressed */
    switch (key_pressed) 
    {
        /* quit */
        case SDLK_ESCAPE :
        case SDLK_KP_PLUS :
                c->speed_up();
                return -1;
        case SDLK_KP_MINUS :
                c->slow_down();
                return -1;
        case SDLK_q      :
                return 1;
        /* Rotation */
        case SDLK_UP :
                c->tilt( 0.01 * PI);
                break;
        case SDLK_DOWN :
                c->tilt(-0.01 * PI);
                break;
        case SDLK_RIGHT :
                c->pan( 0.01 * PI);
                break;
        case SDLK_LEFT :
                c->pan(-0.01 * PI);
                break;
        case SDLK_PAGEDOWN :
                c->roll( 0.01 * PI);
                break;
        case SDLK_PAGEUP :
                c->roll(-0.01 * PI);
                break;
        /* Movement */
        case SDLK_SPACE :
                c->move_up();
                break;
        case SDLK_LCTRL :
                c->move_up(-1.0);
                break;
        case SDLK_a :
                c->move_right(-1.0);
                break;
        case SDLK_d :
                c->move_right();
                break;
        case SDLK_s :
                c->move_forward(-1.0);
                break;
        case SDLK_w :
                c->move_forward();
                break;
        /* Take a snapshot */
        case SDLK_f :
                /* Tone map */
                c->tone_map(local_human_histogram);
                
                /* Output image */
                switch (image_format)
                {
                    case tga :
                        file_name << output_file << "_" << snapshot_nr++ << ".tga";
                        c->write_tga_file(file_name.str());
                        break;
                    case jpg :
                        file_name << output_file << "_" << snapshot_nr++ << ".jpg";
                        c->write_jpeg_file(file_name.str(), jpg_quality);
                        break;
                    case png :
                        file_name << output_file << "_" << snapshot_nr++ << ".png";
                        c->write_png_file(file_name.str());
                        break;
                    default :
                        assert(!"Error unknown image format");
                        break;
                }
                
        
#ifdef LOG_DEPTH                
                depth_map_name << output_file << ".dm";
                c->write_depth_map(depth_map_name.str());
#endif 

                return -1;
        /* Print the angle and position of the camera */
        case SDLK_p :
                c->print_position_data();
                return -1;
        default :
                cout << "Invalid input" << endl;
                break;
    }
    
    return 0;
}


/*****************************************************
 The main function.
*****************************************************/
int main (int argc, char **argv)
{
    /* Default parameters */
    bool            interactive     = false;
    model_format_t  input_format    = code;
    image_format_t  image_format    = tga;
    int             jpg_quality     = 50;
    ext_colour_t    bg;
    string          input_file;
    string          view_point;
    string          output_file     = "snapshot";
    string          caption         = "raytracer ";

    /* Camera parameters */
    camera   *cam;
    point_t  cam_p( 0.0, 0.0, -10);     /* Position                         */
    vector_t x_vec( 1.0, 0.0, 0.0);     /* Horizontal vector                */
    vector_t y_vec( 0.0, 1.0, 0.0);     /* Virtical vector                  */
    vector_t z_vec( 0.0, 0.0, 1.0);     /* Forward vector                   */
    fp_t     rx = 0.0;                  /* Rotation about horizontal vector */
    fp_t     ry = 0.0;                  /* Rotation about virtical vector   */
    fp_t     rz = 0.0;                  /* Rotation about forward vector    */
    unsigned xr = 640;                  /* X resolution                     */
    unsigned yr = 480;                  /* Y resolution                     */
    unsigned xa = 1;                    /* X anti-aliasing factor           */
    unsigned ya = 1;                    /* Y anti-aliasing factor           */

    /* Scene data */
    light_list          lights;
    primitive_list      everything;
    list<material *>    materials;

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
                    cout << "Incorrectly specified tga output file" << endl;
                    help();
                    return 1;
                }
                
                image_format    = tga;
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
                    cout << "Incorrectly specified png output file" << endl;
                    help();
                    return 1;
                }

                image_format    = png;
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
                    cout << "Incorrectly specified jpeg output file" << endl;
                    help();
                    return 1;
                }
                
                image_format    = jpg;
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
                    cout << "Incorrectly specified cfg file" << endl;
                    help();
                    return 1;
                }
                
                input_format    = cfg;
                input_file      = argv[++i];
            }
            /* MGF input */
            else if (strcmp(argv[i], "-mgf") == 0)
            {
                if ((argc - i) < 2)
                {
                    cout << "Incorrectly specified mgf file" << endl;
                    help();
                    return 1;
                }
                
                input_format    = mgf;
                input_file      = argv[++i];
            }
            /* NFF input */
            else if (strcmp(argv[i], "-nff") == 0)
            {
                if ((argc - i) < 2)
                {
                    cout << "Incorrectly specified nff file" << endl;
                    help();
                    return 1;
                }
                
                input_format    = nff;
                input_file      = argv[++i];
            }
            /* LWO input */
            else if (strcmp(argv[i], "-lwo") == 0)
            {
                if ((argc - i) < 2)
                {
                    cout << "Incorrectly specified lwo file" << endl;
                    help();
                    return 1;
                }
                
                input_format    = lwo;
                input_file      = argv[++i];
            }
            /* OBJ input */
            else if (strcmp(argv[i], "-obj") == 0)
            {
                if ((argc - i) < 2)
                {
                    cout << "Incorrectly specified obj file" << endl;
                    help();
                    return 1;
                }
                
                input_format    = obj;
                input_file      = argv[++i];
            }
            /* PLY input */
            else if (strcmp(argv[i], "-ply") == 0)
            {
                if ((argc - i) < 2)
                {
                    cout << "Incorrectly specified ply file" << endl;
                    help();
                    return 1;
                }
                
                input_format    = ply;
                input_file      = argv[++i];
            }
            /* VRML input */
            else if (strcmp(argv[i], "-vrml") == 0)
            {
                if ((argc - i) < 3)
                {
                    cout << "Incorrectly specified vrml file" << endl;
                    help();
                    return 1;
                }
                
                input_format    = vrml;
                input_file      = argv[++i];
                view_point      = argv[++i];
            }
            /* Resolution */
            else if (strcmp(argv[i], "-res") == 0)
            {
                if ((argc - i) < 3)
                {
                    cout << "Incorrectly specified resolution" << endl;
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
                    cout << "Incorrectly specified anti-aliasing factor" << endl;
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
                    cout << "Incorrectly specified camera position" << endl;
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
                    cout << "Incorrectly specified model position" << endl;
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
                    cout << "Incorrectly specified x vector" << endl;
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
                    cout << "Incorrectly specified y vector" << endl;
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
                    cout << "Incorrectly specified z vector" << endl;
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
                    cout << "Incorrectly specified x rotate" << endl;
                    help();
                    return 1;
                }
                
                rx = atof(argv[++i]) * (PI / 180.0);
            }
            /* Y rotation */
            else if (strcmp(argv[i], "-ry") == 0)
            {
                if ((argc - i) < 2)
                {
                    cout << "Incorrectly specified y rotate" << endl;
                    help();
                    return 1;
                }
                
                ry = atof(argv[++i]) * (PI / 180.0);
            }
            /* Z rotation */
            else if (strcmp(argv[i], "-rz") == 0)
            {
                if ((argc - i) < 2)
                {
                    cout << "Incorrectly specified z rotate" << endl;
                    help();
                    return 1;
                }
                
                rz = atof(argv[++i]) * (PI / 180.0);
            }
            /* Background */
            else if (strcmp(argv[i], "-bg") == 0)
            {
                if ((argc - i) < 4)
                {
                    cout << "Incorrectly specified background colour" << endl;
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
                    cout << "Incorrectly specified light" << endl;
                    help();
                    return 1;
                }
                ext_colour_t    rgb;
                point_t         c;
                fp_t            r;
                fp_t            d;
                
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
                    cout << "Incorrectly specified spotlight" << endl;
                    help();
                    return 1;
                }

                ext_colour_t    rgb;
                point_t         c;
                point_t         a;
                point_t         n;
                fp_t            r;
                fp_t            d;
                fp_t            s_a;
                fp_t            s_b;
                
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
                s_a     = atof(argv[++i]) * (PI / 180.0);
                s_b     = atof(argv[++i]) * (PI / 180.0);
                
                n = c - a;
                normalise(&n);
                new_light(&lights, rgb, c, n, d, s_a, s_b, r);
            }
            /* Directional light */
            else if (strcmp(argv[i], "-directionallight") == 0)
            {
                if ((argc - i) < 11)
                {
                    cout << "Incorrectly specified directional light" << endl;
                    help();
                    return 1;
                }

                ext_colour_t    rgb;
                point_t         c;
                point_t         a;
                point_t         n;
                fp_t            d;
                
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
                new_light(&lights, rgb, n, d);
            }
            else
            {
                cout << "Unknown option: " << argv[i] << endl;
                help();
                return 1;
            }
        }
    }

    /* initialise the scene */
    ifstream    input_stream;
    string      path;
    size_t      last_slash;
    
    /* Get the screen aspect ratio */
    const fp_t screen_width     = (fp_t)10.0;
    const fp_t screen_height    = screen_width * ((fp_t)yr / (fp_t)xr);
    switch (input_format)
    {
        case cfg :
            input_stream.open(input_file.c_str());
            assert(input_stream.is_open());
            cfg_parser(input_stream, lights, everything, materials, &cam);

            /* Camera is not set in the scene so do it here */
            cam = new camera(cam_p, x_vec, y_vec, z_vec, bg, screen_width, screen_height, 20, xr, yr, xa, ya);
            break;

        case code :
            scene_init(lights, everything, materials, &cam);
            break;

        case mgf :
            mgf_parser(input_file.c_str(), lights, everything, materials);
            
            /* Camera is not set in the scene so do it here */
            cam = new camera(cam_p, x_vec, y_vec, z_vec, bg, screen_width, screen_height, 10, xr, yr, xa, ya);
            break;

        case nff :
            input_stream.open(input_file.c_str());
            assert(input_stream.is_open());
            nff_parser(input_stream, lights, everything, materials, &cam);
            break;

        case lwo :
            input_stream.open(input_file.c_str());
            assert(input_stream.is_open());
            last_slash  = input_file.find_last_of('/');
            path        = input_file.substr(0, last_slash + 1);
            lwo_parser(input_stream, path, lights, everything, materials, &cam);

            /* Camera is not set in the scene so do it here */
            cam = new camera(cam_p, x_vec, y_vec, z_vec, bg, screen_width, screen_height, 20, xr, yr, xa, ya);
            break;
            
        case obj :
            input_stream.open(input_file.c_str());
            assert(input_stream.is_open());
            last_slash  = input_file.find_last_of('/');
            path        = input_file.substr(0, last_slash + 1);
            obj_parser(input_stream, path, lights, everything, materials, &cam);
            
            /* Camera is not set in the scene so do it here */
            cam = new camera(cam_p, x_vec, y_vec, z_vec, bg, screen_width, screen_height, 20, xr, yr, xa, ya);
            break;

        case ply :
            input_stream.open(input_file.c_str());
            assert(input_stream.is_open());
            ply_parser(input_stream, lights, everything, materials, &cam);
            
            /* Camera is not set in the scene so do it here */
            cam = new camera(cam_p, x_vec, y_vec, z_vec, bg, screen_width, screen_height, 20, xr, yr, xa, ya);
            break;

        case vrml :
            /* Deafult camera set up for vrml -- NOTE negative z axis */
            cam = new camera(cam_p, point_t((fp_t)1.0, (fp_t)0.0, (fp_t) 0.0), 
                                    point_t((fp_t)0.0, (fp_t)1.0, (fp_t) 0.0), 
                                    point_t((fp_t)0.0, (fp_t)0.0, (fp_t)-1.0), bg, screen_width, screen_height, 20, xr, yr, xa, ya);

            input_stream.open(input_file.c_str());
            assert(input_stream.is_open());
            vrml_parser(input_stream, lights, everything, materials, cam, view_point);
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
        std::unique_ptr<sdl_event_handler_base> cam_event_handler(get_camera_event_handler(cam));
        std::unique_ptr<unsigned char[]> screen_data(new unsigned char [cam->x_resolution() * cam->y_resolution() * 3]);
        while(do_next != 1) 
        {
            /* Poll for user input */
            do_next = cam_event_handler->process_events();
            
            /* Draw if required */
            if (do_next == 0)
            {
                /* Display output for interactive mode */
                ray_tracer(lights, everything, *cam);

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
        ray_tracer(lights, everything, *cam);
        
        /* Tone mapping */
        //cam->tone_map(local_human_histogram, 0.75, (1.0/3.0), (1.0/3.0), false, false, false);

        /* Output image */
        ostringstream file_name(ostringstream::out);
        switch (image_format)
        {
            case tga :
                file_name << output_file << "_0.tga";
                cam->write_tga_file(file_name.str());
                break;
            case jpg :
                file_name << output_file << "_0.jpg";
                cam->write_jpeg_file(file_name.str(), jpg_quality);
                break;
            case png :
                file_name << output_file << "_0.png";
                cam->write_png_file(file_name.str());
                break;
            default :
                assert(!"Error unknown image format");
                break;
        }
        
#ifdef LOG_DEPTH                
        ostringstream depth_map_name(ostringstream::out);
        depth_map_name << output_file << ".dm";
        cam->write_depth_map(depth_map_name.str());
#endif        
    }

    /* Clean up dynamic memory usage */
    scene_clean(&everything, &materials, cam);
    return 0;
}
