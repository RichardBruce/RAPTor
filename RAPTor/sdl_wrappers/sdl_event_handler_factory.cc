/* Standard headers */
#include <cmath>
#include <map>
#include <string>

/* Common headers */
#include "base_camera.h"

/* Event handlers */
#include "SDL.h"
#include "sdl_event_handler.h"
#include "sdl_event_handler_factory.h"


// using raptor_raytracer::camera;
// using raptor_raytracer::image_format_t;
// using raptor_raytracer::tone_mapping_mode_t;

/* Function to create an handler for the camera mappings of SDL_Keycode */
/*  cam is the camera that will be updated. ownership is not taken of the camera */
/*  a pointer to an sdl_event_handler<camera> is returned */
std::map<SDL_Keycode, sdl_event_handler::key_handler_function>* get_camera_event_handler(base_camera *const cam)
{
    using handler_function = sdl_event_handler::key_handler_function;

    std::map<SDL_Keycode, handler_function> *handler_map = new std::map<SDL_Keycode, handler_function>({
        { SDLK_KP_PLUS, [cam](const SDL_Keycode key, const float)
            {
                cam->speed_up(); return -1;
            }
        },
        { SDLK_KP_MINUS, [cam](const SDL_Keycode key, const float)
            {
                cam->slow_down(); return -1;
            }
        },
        { SDLK_ESCAPE, [cam](const SDL_Keycode key, const float)
            {
                return 1;
            }
        },
        { SDLK_q, [cam](const SDL_Keycode key, const float)
            {
                return 1;
            }
        },
        /* Rotation */
        { SDLK_UP, [cam](const SDL_Keycode key, const float delta_time)
            {
                cam->tilt( 0.01f * delta_time * M_PI); return 0;
            }
        },
        { SDLK_DOWN, [cam](const SDL_Keycode key, const float delta_time)
            {
                cam->tilt(-0.01f * delta_time * M_PI); return 0;
            }
        },
        { SDLK_RIGHT, [cam](const SDL_Keycode key, const float delta_time)
            {
                cam->pan( 0.01f * delta_time * M_PI); return 0;
            }
        },
        { SDLK_LEFT, [cam](const SDL_Keycode key, const float delta_time)
            {
                cam->pan(-0.01f * delta_time * M_PI); return 0;
            }
        },
        { SDLK_PAGEDOWN, [cam](const SDL_Keycode key, const float delta_time)
            {
                cam->roll( 0.01f * delta_time * M_PI); return 0;
            }
        },
        { SDLK_PAGEUP, [cam](const SDL_Keycode key, const float delta_time)
            {
                cam->roll(-0.01f * delta_time * M_PI); return 0;
            }
        },
        /* Movement */
        { SDLK_SPACE, [cam](const SDL_Keycode key, const float delta_time)
            {
                cam->move_up(delta_time); return 0;
            }
        },
        { SDLK_LCTRL, [cam](const SDL_Keycode key, const float delta_time)
            {
                cam->move_up(-delta_time); return 0;
            }
        },
        { SDLK_a, [cam](const SDL_Keycode key, const float delta_time)
            {
                cam->move_right(-delta_time); return 0;
            }
        },
        { SDLK_d, [cam](const SDL_Keycode key, const float delta_time)
            {
                cam->move_right(delta_time); return 0;
            }
        },
        { SDLK_s, [cam](const SDL_Keycode key, const float delta_time)
            {
                cam->move_forward(-delta_time); return 0;
            }
        },
        { SDLK_w, [cam](const SDL_Keycode key, const float delta_time)
            {
                cam->move_forward(delta_time); return 0;
            }
        },
        /* Print the angle and position of the camera */
        { SDLK_p, [cam](const SDL_Keycode key, const float)
            {
                cam->print_position_data(); return -1;
            }
        }
    });

    return handler_map;
}
