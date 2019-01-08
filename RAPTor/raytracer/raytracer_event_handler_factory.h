#pragma once

#include "camera.h"


namespace raptor_raytracer
{
/* Forward declarations */
class sdl_event_handler_base;

void append_raytracer_camera_event_handler(std::map<SDL_Keycode, sdl_event_handler::key_handler_function> *const handler_map, raptor_raytracer::camera *const cam, const std::string &output_file, const int jpg_quality = 100, const raptor_raytracer::image_format_t image_format = raptor_raytracer::image_format_t::png);
}; /* namespace raptor_raytracer */
