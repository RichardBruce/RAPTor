#ifndef __SDL_EVENT_HANDLER_FACTORY_H__
#define __SDL_EVENT_HANDLER_FACTORY_H__

/* Forward declarations */
namespace raptor_raytracer
{
class camera;
}; /* namespace raptor_raytracer */

class sdl_event_handler_base;

sdl_event_handler<raptor_raytracer::camera>* get_camera_event_handler(raptor_raytracer::camera *const cam);

#endif /* #ifndef __SDL_EVENT_HANDLER_FACTORY_H__ */
