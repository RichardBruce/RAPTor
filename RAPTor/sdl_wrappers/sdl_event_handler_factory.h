#ifndef __SDL_EVENT_HANDLER_FACTORY_H__
#define __SDL_EVENT_HANDLER_FACTORY_H__

/* Forward declarations */
namespace raptor_raytracer
{
class camera;
}; /* namespace raptor_raytracer */

class sdl_event_handler_base;

sdl_event_handler<raptor_raytracer::camera>* get_camera_event_handler(raptor_raytracer::camera *const cam, const std::string &output_file, const int jpg_quality = 100, const raptor_raytracer::image_format_t image_format = raptor_raytracer::image_format_t::png);

#endif /* #ifndef __SDL_EVENT_HANDLER_FACTORY_H__ */
