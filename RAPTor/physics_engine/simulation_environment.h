#ifndef __SIMULATION_ENVIRONMENT_H__
#define __SIMULATION_ENVIRONMENT_H__

/* Standard headers */
#include <ctime>

/* SDL */
#include "SDL_image.h"

/* Common headers */
#include "common.h"
#include "point_t.h"

/* Display headers */
#include "sdl_wrapper.h"
#include "sdl_event_handler.h"
#include "sdl_event_handler_factory.h"

/* Raytracer headers */
#include "camera.h"
#include "parser_common.h"

/* Physics headers */
#include "physics_options.h"
#include "physics_engine.h"
#include "rigid_body_collider.h"
#include "force.h"


/* Camera parameters */
const point_t cam_p( 0.0, -5.0, -25);                               /* Position                 */
const point_t x_vec( 1.0,  0.0, 0.0);                               /* Horizontal vector        */
const point_t y_vec( 0.0,  1.0, 0.0);                               /* Virtical vector          */
const point_t z_vec( 0.0,  0.0, 1.0);                               /* Forward vector           */
const ext_colour_t bg(0.0, 0.0, 0.0);                               /* Background colour        */
const unsigned xr = 640;                                            /* X resolution             */
const unsigned yr = 480;                                            /* Y resolution             */
const unsigned xa = 1;                                              /* X anti-aliasing factor   */
const unsigned ya = 1;                                              /* Y anti-aliasing factor   */
const fp_t screen_width     = (fp_t)10.0;                           /* Width of screen          */
const fp_t screen_height    = screen_width * ((fp_t)yr / (fp_t)xr); /* Height of screen         */

/* Clocks in a second */
const fp_t clocks_per_sec_inv = 1.0 / CLOCKS_PER_SEC;


namespace raptor_physics
{
/* Class to set up the environment and run the main loop */
class simulation_environment
{
    public :
        /* CTOR */
        simulation_environment(physics_engine *const pe, physics_options *const po)
        : _po(po), _pe(pe), _window(nullptr), _renderer(nullptr), _texture(nullptr), _load_screen(nullptr), _font(nullptr), 
          _cam(cam_p, x_vec, y_vec, z_vec, bg, screen_width, screen_height, 10, xr, yr, xa, ya), 
          _lights(), _cam_event_handler(get_camera_event_handler(&_cam)), _sim_time(clock()), _time_run(0.0), _damped_fps(0.0)
        {    
            /* Initialise and lock the screen */
            if (_po->render())
            {
                if (sdl_set_up(&_window, &_renderer, &_texture, &_font, "RAPTor Physics", _cam.x_resolution(), _cam.y_resolution()))
                {
                    return;
                }
            }
        }

        /* DTOR */
        ~simulation_environment()
        {
            /* SDL clean up */
            if (_po->render())
            {
                sdl_clean_up(_window, _renderer, _texture, _font);
            }

            if (_load_screen != nullptr)
            {
                SDL_DestroyTexture(_load_screen);
            }
        }

        simulation_environment& load_screen(const char *const file)
        {
            /* Load and draw image */
            if (_load_screen != nullptr)
            {
                SDL_DestroyTexture(_load_screen);
            }

            _load_screen = IMG_LoadTexture(_renderer, file);
            draw_screen(_renderer, _load_screen);

            return *this;
        }

        /* Run the simulation */
        int run();

        /* Access functions */
        physics_engine *const   engine()                            { return _pe;                   }
        fp_t                    time_run()                          { return _time_run;             }
        int                     number_of_lights()          const   { return _lights.size();        }
        bool                    screen_initialised()        const   { return _texture != nullptr;   }
        bool                    font_initialised()          const   { return _font != nullptr;      }

        simulation_environment& add_light(const ext_colour_t &rgb, const point_t &at)
        {
            new_light(&_lights, rgb, at, 0.0, 0.0);
            return *this;
        }

        simulation_environment& add_object(physics_object *const o)
        {
            _pe->add_object(o);
            return *this;
        }

        simulation_environment& add_moving_object(physics_object *const o)
        {
            o->register_force(new const_force(point_t(0.0, 0.0, 0.0), point_t(0.0, -raptor_physics::ACCELERATION_UNDER_GRAVITY * o->get_mass(), 0.0), numeric_limits<fp_t>::infinity()));
            _pe->add_moving_object(o);
            return *this;
        }

    private :
        physics_options *const                  _po;
        physics_engine  *const                  _pe;
        SDL_Window *                            _window;
        SDL_Renderer *                          _renderer;
        SDL_Texture *                           _texture;
        SDL_Texture *                           _load_screen;
        TTF_Font *                              _font;
        camera                                  _cam;
        light_list                              _lights;
        std::unique_ptr<sdl_event_handler_base> _cam_event_handler;
        clock_t                                 _sim_time;
        fp_t                                    _time_run;
        fp_t                                    _damped_fps;
};
}; /* namespace raptor_physics */

#endif /* #ifndef __SIMULATION_ENVIRONMENT_H__ */
