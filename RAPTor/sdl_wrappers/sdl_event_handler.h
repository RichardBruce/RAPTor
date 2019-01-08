#pragma once

/* Standard headers */
#include <functional>
#include <map>
#include <set>

/* Boost */
#include "boost/noncopyable.hpp"

/* SDL */
#include "SDL.h"

/* Forward declarations */


/* Abstract base class to sdl event handlers */
/* Copying is not permitted */
class sdl_event_handler_base : private boost::noncopyable
{
    public :
        /* Virtual DTOR since this class is for sub-classing */
        virtual ~sdl_event_handler_base() {  };

        /* Abstract function that will process all outstanding sdl events */
        virtual int process_events(const bool cont = false) = 0;

        /* Abstract function that will wait to process for an sdl event */
        virtual int wait_for_event() = 0;

        /* Process all pressed keys */
        virtual void process_down_keys(const float delta_time) = 0;

    private :
        /* Private abstract function that handles a key press event */
        virtual int internal_process_event(const SDL_Keycode key, const float delta_time) = 0;

        /* Private abstract function that handles a window event */
        virtual int internal_process_event(const SDL_WindowEvent key) = 0;
};


/* Sdl event handling class. The template type is the class that will be affected by the event. */
/* Copying is not permitted */
class sdl_event_handler : public sdl_event_handler_base
{
    public :
        /* Convenience typedef */
        using key_handler_function = std::function<int (const SDL_Keycode, const float)>;
        using mouse_handler_function = std::function<int (const SDL_MouseMotionEvent)>;
        using window_handler_function = std::function<int (const SDL_WindowEvent)>;

        /* CTOR */
        sdl_event_handler(std::map<SDL_Keycode, key_handler_function> *const key_handler_map, std::map<SDL_WindowEventID, window_handler_function> *const window_handler_map, const mouse_handler_function mouse_handler) : 
            _key_handler_map(key_handler_map), _window_handler_map(window_handler_map), _mouse_handler(mouse_handler) { };

        sdl_event_handler(std::map<SDL_Keycode, key_handler_function> *const key_handler_map) : 
            sdl_event_handler(key_handler_map, nullptr, [](const SDL_MouseMotionEvent){ return 0; }) { };

        /* The _handler_map is deleted */
        virtual ~sdl_event_handler()
        {
            if (_key_handler_map != nullptr)
            {
                delete _key_handler_map;
            }

            if (_window_handler_map != nullptr)
            {
                delete _window_handler_map;
            }
        }

        /* Function to process all outstanding sdl events */
        virtual int process_events(const bool cont = false) override
        {
            /* Poll for user input */
            SDL_Event event;
            while (SDL_PollEvent(&event)) 
            {      
                const int quit = internal_process_event(event, cont);
                if (quit != 0)
                {
                    return quit;
                }
            }

            return 0;
        }

        /* Function to wait for an sdl event */
        virtual int wait_for_event() override
        {
            /* Poll for user input */
            SDL_Event event;
            SDL_WaitEvent(&event);
            return internal_process_event(event, false);
        }

        /* Process all pressed keys */
        virtual void process_down_keys(const float delta_time) override
        {
            for (const auto &k : _keys_down)
            {
                internal_process_event(k, delta_time);
            }
        }

    private :
        /* Private function that handles an sdl event */
        int internal_process_event(const SDL_Event event, const bool cont)
        {
            switch (event.type) 
            {
                case SDL_QUIT :
                        return 1;
                case SDL_KEYUP :
                        _keys_down.erase(event.key.keysym.sym);
                        return 0;
                case SDL_KEYDOWN :
                    {
                        const int ret = internal_process_event(event.key.keysym.sym, cont ? 0.0f : 1.0f);
                        if (!ret)
                        {
                            _keys_down.emplace(event.key.keysym.sym);
                        }
                        return ret;
                    }
                case SDL_WINDOWEVENT :
                        return internal_process_event(event.window);
                case SDL_MOUSEMOTION :
                        return _mouse_handler(event.motion);
                default :
                        break;
            }

            return 0;
        }

        /* Private function that handles a key press event */
        virtual int internal_process_event(const SDL_Keycode key, const float delta_time) override
        {
            if (_key_handler_map == nullptr)
            {
                return 0;
            }

            auto handler = _key_handler_map->find(key);
            if (handler != _key_handler_map->end())
            {
                return handler->second(key, delta_time);
            }
            else
            {
                return 0;
            }
        }

        /* Private function that handles a window event */
        virtual int internal_process_event(const SDL_WindowEvent window_event) override
        {
            if (_window_handler_map == nullptr)
            {
                return 0;
            }

            auto handler = _window_handler_map->find(static_cast<SDL_WindowEventID>(window_event.event));
            if (handler != _window_handler_map->end())
            {
                return handler->second(window_event);
            }
            else
            {
                return 0;
            }
        }

        std::map<SDL_Keycode, key_handler_function> *const          _key_handler_map;       /* Map between SDL key event and handling function */
        std::map<SDL_WindowEventID, window_handler_function> *const _window_handler_map;    /* Map between SDL window event and handling function */
        std::set<SDL_Keycode>                                       _keys_down;             /* Currently pressed keys */
        const mouse_handler_function                                _mouse_handler;         /* Single mouse handler function */
};
