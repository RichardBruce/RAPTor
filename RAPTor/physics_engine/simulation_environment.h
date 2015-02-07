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
const point_t cam_p( 0.0f, -5.0f, -25.0f);                                                      /* Position                 */
const point_t x_vec( 1.0f,  0.0f, 0.0f);                                                        /* Horizontal vector        */
const point_t y_vec( 0.0f,  1.0f, 0.0f);                                                        /* Virtical vector          */
const point_t z_vec( 0.0f,  0.0f, 1.0f);                                                        /* Forward vector           */
const raptor_raytracer::ext_colour_t bg(0.0f, 0.0f, 0.0f);                                      /* Background colour        */
const unsigned xr = 640;                                                                        /* X resolution             */
const unsigned yr = 480;                                                                        /* Y resolution             */
const unsigned xa = 1;                                                                          /* X anti-aliasing factor   */
const unsigned ya = 1;                                                                          /* Y anti-aliasing factor   */
const float screen_width    = 10.0f;                                                            /* Width of screen          */
const float screen_height   = screen_width * (static_cast<float>(yr) / static_cast<float>(xr)); /* Height of screen         */

/* Clocks in a second */
const float clocks_per_sec_inv = 1.0f / CLOCKS_PER_SEC;


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
          _lights(), _cam_event_handler(get_camera_event_handler(&_cam, "physics_snapshot")), _sim_time(clock()), _time_run(0.0), _damped_fps(0.0)
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
            /* Clean old image */
            if (_load_screen != nullptr)
            {
                SDL_DestroyTexture(_load_screen);
            }

            if (_po->render())
            {
                /* Load image */
                _load_screen = IMG_LoadTexture(_renderer, file);
                if (_load_screen == nullptr)
                {
                    BOOST_LOG_TRIVIAL(trace) << "Failed to load image: " << IMG_GetError();
                }

                /* Render */
                draw_screen(_renderer, _load_screen);
            }

            return *this;
        }

        /* Run the simulation */
        int run();

        /* Access functions */
        physics_engine *const   engine()                            { return _pe;                       }
        float                   time_run()                          { return _time_run;                 }
        int                     number_of_lights()          const   { return _lights.size();            }
        bool                    screen_initialised()        const   { return _texture != nullptr;       }
        bool                    font_initialised()          const   { return _font != nullptr;          }
        bool                    load_screen_initialised()   const   { return _load_screen != nullptr;   }

        simulation_environment& add_light(const raptor_raytracer::ext_colour_t &rgb, const point_t &at)
        {
            new_light(&_lights, rgb, at, 0.0f, 0.0f);
            return *this;
        }

        simulation_environment& add_object(physics_object *const o)
        {
            assert((o->get_mass() == std::numeric_limits<float>::infinity()) || !"Error: Non moving object must have infinite mass, did you mean to call add_moving_object?");
            _pe->add_object(o);
            return *this;
        }

        simulation_environment& add_moving_object(physics_object *const o, const bool gravity = true)
        {
            if (gravity)
            {
                o->register_force(new const_force(point_t(0.0f, 0.0f, 0.0f), point_t(0.0f, -raptor_physics::ACCELERATION_UNDER_GRAVITY * o->get_mass(), 0.0f), std::numeric_limits<float>::infinity()));
            }
            
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
        raptor_raytracer::camera                _cam;
        raptor_raytracer::light_list            _lights;
        std::unique_ptr<sdl_event_handler_base> _cam_event_handler;
        clock_t                                 _sim_time;
        float                                   _time_run;
        float                                   _damped_fps;
};
}; /* namespace raptor_physics */

#endif /* #ifndef __SIMULATION_ENVIRONMENT_H__ */
