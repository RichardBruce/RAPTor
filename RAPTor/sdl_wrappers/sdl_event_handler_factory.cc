/* Standard headers */
#include <map>

/* Common headers */
#include "common.h"

/* Event handlers */
#include "SDL.h"
#include "sdl_event_handler.h"
#include "sdl_event_handler_factory.h"

/* Local headers */
#include "camera.h"

using raptor_raytracer::camera;


/* Function to create an handler for the camera mappings of SDL_Keycode */
/*  cam is the camera that will be updated. ownership is not taken of the camera */
/*  a pointer to an sdl_event_handler<camera> is returned */
sdl_event_handler<camera>* get_camera_event_handler(camera *const cam)
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
        }
    });

    return new sdl_event_handler<camera>(handler_map, cam);
}
