#ifndef __SDL_EVENT_HANDLER_FACTORY_H__
#define __SDL_EVENT_HANDLER_FACTORY_H__

/* Forward declarations */
class camera;
class sdl_event_handler_base;

sdl_event_handler<camera>* get_camera_event_handler(camera *const cam);

#endif /* #ifndef __SDL_EVENT_HANDLER_FACTORY_H__ */
