#pragma once

/* Forward declarations */
namespace raptor_raytracer
{
class camera;
}; /* namespace raptor_raytracer */

class base_camera;
class sdl_event_handler_base;

std::map<SDL_Keycode, sdl_event_handler::key_handler_function>* get_camera_event_handler(base_camera *const cam);
