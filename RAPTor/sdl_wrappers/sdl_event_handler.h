#ifndef __SDL_EVENT_HANDLER_H__
#define __SDL_EVENT_HANDLER_H__

/* Standard headers */
#include <map>
#include <functional>

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
        virtual int process_events() = 0;

        /* Abstract function that will wait to process for an sdl event */
        virtual int wait_for_event() = 0;
    private :
        /* Private abstract function that handles a key press event */
        virtual int internal_process_event(const SDL_Keycode key) = 0;
};


/* Sdl event handling class. The template type is the class that will be affected by the event. */
/* Copying is not permitted */
template<class T>
class sdl_event_handler : public sdl_event_handler_base
{
    public :
        /* Convenience typedef */
        typedef std::function<int (const SDL_Keycode, T*)> handler_function;

        /* CTOR */
        sdl_event_handler(std::map<SDL_Keycode, handler_function> *handler_map, T *const handler_object)
            : _handler_map(handler_map), _handler_object(handler_object) { };

        /* The _handler_map is deleted, but ownership of the _handler_object is not taken */
        virtual ~sdl_event_handler()
        {
            delete _handler_map;
        }

        /* Function to process all outstanding sdl events */
        virtual int process_events() override
        {
            /* Poll for user input */
            int quit = 0;
            SDL_Event event;
            while(SDL_PollEvent(&event)) 
            {      
                switch (event.type) 
                {
                    case SDL_QUIT :
                            return 1;
                            break;
                    case SDL_KEYDOWN :
                            quit = internal_process_event(event.key.keysym.sym);
                            break;
                    default :
                            break;
                }
            }

            return quit;
        }

        /* Function to wait for an sdl event */
        virtual int wait_for_event() override
        {
            /* Poll for user input */
            int quit = 0;
            SDL_Event event;
            SDL_WaitEvent(&event);
            switch (event.type) 
            {
                case SDL_QUIT :
                        return 1;
                        break;
                case SDL_KEYDOWN :
                        quit = internal_process_event(event.key.keysym.sym);
                        break;
                default :
                        break;
            }

            return quit;
        }

    private :
        /* Private function that handles a key press event */
        virtual int internal_process_event(const SDL_Keycode key) override
        {
            auto handler = _handler_map->find(key);
            if (handler != _handler_map->end())
            {
                return handler->second(key, _handler_object);
            }
            else
            {
                return 0;
            }
        }

        std::map<SDL_Keycode, handler_function> *const  _handler_map;       /* Map between SDL event and handling function */
        T                                       *const  _handler_object;    /* A class that may be affected by the event */
};

#endif /* #ifndef __SDL_EVENT_HANDLER_H__ */
