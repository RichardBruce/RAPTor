/* Standard headers */
#include <map>
#include <string>

/* Common headers */
#include "common.h"

/* Event handlers */
#include "SDL.h"
#include "sdl_event_handler.h"
#include "sdl_event_handler_factory.h"

/* Local headers */
#include "camera.h"
#include "common.h"

using raptor_raytracer::camera;
using raptor_raytracer::image_format_t;
using raptor_raytracer::tone_mapping_mode_t;

/* Function to create an handler for the camera mappings of SDL_Keycode */
/*  cam is the camera that will be updated. ownership is not taken of the camera */
/*  a pointer to an sdl_event_handler<camera> is returned */
sdl_event_handler<camera>* get_camera_event_handler(camera *const cam, const std::string &output_file, const int jpg_quality, const image_format_t image_format)
{
    typedef std::function<int (const SDL_Keycode, camera*)> handler_function;

    std::map<SDL_Keycode, handler_function> *handler_map = new std::map<SDL_Keycode, handler_function>({
        { SDLK_KP_PLUS, [](const SDL_Keycode key, camera* c) -> int
            {
                c->speed_up(); return -1;
            }
        },
        { SDLK_KP_MINUS, [](const SDL_Keycode key, camera* c) -> int
            {
                c->slow_down(); return -1;
            }
        },
        { SDLK_ESCAPE, [](const SDL_Keycode key, camera* c) -> int
            {
                return 1;
            }
        },
        { SDLK_q, [](const SDL_Keycode key, camera* c) -> int
            {
                return 1;
            }
        },
        /* Rotation */
        { SDLK_UP, [](const SDL_Keycode key, camera* c) -> int
            {
                c->tilt( 0.01 * PI); return 0;
            }
        },
        { SDLK_DOWN, [](const SDL_Keycode key, camera* c) -> int
            {
                c->tilt(-0.01 * PI); return 0;
            }
        },
        { SDLK_RIGHT, [](const SDL_Keycode key, camera* c) -> int
            {
                c->pan( 0.01 * PI); return 0;
            }
        },
        { SDLK_LEFT, [](const SDL_Keycode key, camera* c) -> int
            {
                c->pan(-0.01 * PI); return 0;
            }
        },
        { SDLK_PAGEDOWN, [](const SDL_Keycode key, camera* c) -> int
            {
                c->roll( 0.01 * PI); return 0;
            }
        },
        { SDLK_PAGEUP, [](const SDL_Keycode key, camera* c) -> int
            {
                c->roll(-0.01 * PI); return 0;
            }
        },
        /* Movement */
        { SDLK_SPACE, [](const SDL_Keycode key, camera* c) -> int
            {
                c->move_up(); return 0;
            }
        },
        { SDLK_LCTRL, [](const SDL_Keycode key, camera* c) -> int
            {
                c->move_up(-1.0); return 0;
            }
        },
        { SDLK_a, [](const SDL_Keycode key, camera* c) -> int
            {
                c->move_right(-1.0); return 0;
            }
        },
        { SDLK_d, [](const SDL_Keycode key, camera* c) -> int
            {
                c->move_right(); return 0;
            }
        },
        { SDLK_s, [](const SDL_Keycode key, camera* c) -> int
            {
                c->move_forward(-1.0); return 0;
            }
        },
        { SDLK_w, [](const SDL_Keycode key, camera* c) -> int
            {
                c->move_forward(); return 0;
            }
        },
        /* Print the angle and position of the camera */
        { SDLK_p, [](const SDL_Keycode key, camera* c) -> int
            {
                c->print_position_data(); return 0;
            }
        },
        { SDLK_f, [&output_file, jpg_quality, image_format](const SDL_Keycode key, camera* c) -> int
            {
                static unsigned int snapshot_nr = 0;
                ostringstream file_name(ostringstream::out);

                /* Tone map */
                c->tone_map(tone_mapping_mode_t::local_human_histogram);
                
                /* Output image */
                switch (image_format)
                {
                    case image_format_t::tga :
                        file_name << output_file << "_" << snapshot_nr++ << ".tga";
                        c->write_tga_file(file_name.str());
                        break;
                    case image_format_t::jpg :
                        file_name << output_file << "_" << snapshot_nr++ << ".jpg";
                        c->write_jpeg_file(file_name.str(), jpg_quality);
                        break;
                    case image_format_t::png :
                        file_name << output_file << "_" << snapshot_nr++ << ".png";
                        c->write_png_file(file_name.str());
                        break;
                    default :
                        assert(!"Error unknown image format");
                        break;
                }

                return 0;
            }
        }
    });

    return new sdl_event_handler<camera>(handler_map, cam);
}
