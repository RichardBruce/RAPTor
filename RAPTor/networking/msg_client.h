#ifndef __MSG_CLIENT_H__
#define __MSG_CLIENT_H__

/* Standard headers */
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

/* Boost headers */
#include "boost/asio/io_service.hpp"
#include "boost/lexical_cast.hpp"

/* Display headers */
#include "sdl_wrapper.h"
#include "sdl_event_handler.h"
#include "sdl_event_handler_factory.h"

/* Networking */
#include "networking_constants.h"
#include "conversion.h"
#include "group.h"
#include "protocol_stack.h"

/* Raytracer headers */
#include "camera.h"
#include "perlin_noise_3d_mapper.h"


namespace raptor_networking
{
using boost::asio::ip::address;

class msg_client
{
    public :
        msg_client(const std::string &me_addr, const std::uint16_t send_port, const std::uint16_t recv_port, const std::uint16_t port_off)
            :  _io_service(),
               _uni_cast_conn(_io_service, new uni_cast_receiver(), me_addr, send_port, recv_port, port_off, true), 
               _multi_cast_conn(nullptr),
               _group(_uni_cast_conn.stack_group()),
               _subscribed(false)
        {
            _uni_cast_conn.start(10);
        };

        ~msg_client()
        {
            /* Be nice and tell the server */
            if (_subscribed)
            {
                _group->disconnect();
            }

            /* Stop the io service */
            _io_service.stop();

            /* If it was created close the multi cast connection */
            if (_multi_cast_conn != nullptr)
            {
                delete _multi_cast_conn;
            }
        }

        bool start(const std::string &name, const std::string &addr, const unsigned int timeout_ms = 1000)
        {
            /* If already subscribed, then unsubscribe before subscribing again */
            if (_subscribed)
            {
                _group->disconnect();
            }

            /* Send subscription request and wait for a response */
            int conn_wait;
            std::future<bool> conn_future;
            do 
            {
                conn_future = _group->connect();
                conn_wait = wait_for_reponse(conn_future, timeout_ms);
            } while (conn_wait < 0);

            if ((conn_wait == 0) || !conn_future.get())
            {
                return false;
            }
            
            /* Send init request */
            int ini_wait;
            std::future<std::istream *const> ini_future;
            do 
            {
                std::shared_ptr<std::vector<char>> msg(new std::vector<char>(MSG_INIT));
                _uni_cast_conn.send(msg, boost::uuids::nil_uuid(), &ini_future);
                ini_wait = wait_for_reponse(ini_future, timeout_ms);
            } while (ini_wait < 0);

            if (ini_wait == 0)
            {
                return false;
            }

            std::unique_ptr<std::istream> ini_data(ini_future.get());
            deserialise(&_cam, *ini_data);
            std::cout << "Picture size: " << _cam->x_resolution() << "x" << _cam->y_resolution() << std::endl;

            /* Initialise and lock the screen */
            TTF_Font *      font;
            SDL_Window *    window;
            SDL_Renderer *  renderer;
            SDL_Texture *   texture;
            const int sdl_status = sdl_set_up(&window, &renderer, &texture, &font, "Video Receiver", _cam->x_resolution(), _cam->y_resolution());
            if (sdl_status)
            {
                std::cout << "Error: Couldnt initialise SDL" << std::endl;
                return false;
            }

            /* Build and start the multi cast connection */
            _multi_cast_conn = new protocol_stack(
                _io_service, 
                new multi_cast_receiver(window, renderer, texture, font, _cam), 
                _group,
                address::from_string(addr), 
                _group->group_physical_address(), 
                _group->group_port());
            _multi_cast_conn->start(0, false);

            return true;
        }

        sdl_event_handler_base* event_handler()
        {
            return get_camera_event_handler(_cam, "networking");
        }

        msg_client& send_request(std::shared_ptr<std::vector<char>> &msg)
        {
            /* Must be subscribed to call this */
            assert(_subscribed);

            /* Send request */
            _uni_cast_conn.send(msg, boost::uuids::nil_uuid());

            return *this;
        }

        template <class T>
        msg_client& send_request(const T &obj, std::shared_ptr<std::vector<char>> &msg)
        {
            /* Serialise the object after the message */
            serialise(msg, obj);

            /* Send */
            send_request(msg);

            return *this;
        }

    private :
        /* Inner class to receive uni cast messages */
        class uni_cast_receiver : public data_receiver
        {
            public :
                virtual std::vector<char>* received(std::unique_ptr<std::istream> &&data) override
                {
                    return nullptr;
                }

                virtual uni_cast_receiver* clean_clone() override
                {
                    return new uni_cast_receiver();
                }
        };

        /* Inner class to receive multi cast messages */
        class multi_cast_receiver : public data_receiver
        {
            public :
                multi_cast_receiver(SDL_Window *const window, SDL_Renderer *const renderer, SDL_Texture *const texture, TTF_Font *const font, raptor_raytracer::camera *const cam)
                    : _window(window), 
                      _renderer(renderer),
                      _texture(texture),
                      _font(font), 
                      _cam(cam), 
                      _fps(),
                      _sim_time(clock()),
                      _damped_fps(0),
                      _screen_data(new unsigned char[cam->x_resolution() * cam->y_resolution() * 3])
                    {
                        _fps << std::fixed << std::setprecision(2);
                    };

                virtual ~multi_cast_receiver()
                {
                    /* Clean up screen data */
                    delete [] _screen_data;

                    /* Clean up camera */
                    delete _cam;

                    /* SDL clean up */
                    sdl_clean_up(_window, _renderer, _texture, _font);
                }

                /* TODO -- Check this is the data we are looking for */
                std::vector<char>* received(std::unique_ptr<std::istream> &&data) override
                {
                    /* Deserialise into the camera */
                    deserialise(_cam, *data);

                    /* Get an image */
                    _cam->clip_image_to_bgr(_screen_data);

                    /* Update time */
                    const clock_t time_now  = clock();
                    const float time_step   = (time_now - _sim_time) * _clocks_per_sec_inv;
                    _sim_time               = time_now;
                    _damped_fps             = (0.5f * _damped_fps) + (0.5f / time_step);
                    
                    _fps.str(std::string());
                    _fps << _damped_fps;

                    /* Draw the screen */
                    const int draw_status = draw_screen(_window, _renderer, _texture, _font, _fps.str(), _screen_data);
                    if (draw_status)
                    {
                        std::cout << "Error: Failed to draw to screen" << std::endl;
                    }

                    return nullptr;
                }

                virtual multi_cast_receiver* clean_clone() override
                {
                    /* There can only be one screen, everything will refer back to the original class */
                    return this;
                }

            private :
                SDL_Window *const               _window;
                SDL_Renderer *const             _renderer;
                SDL_Texture *const              _texture;
                TTF_Font *const                 _font;
                raptor_raytracer::camera *const _cam;
                std::ostringstream              _fps;
                clock_t                         _sim_time;
                float                           _damped_fps;
                unsigned char *const            _screen_data;

                static constexpr float _clocks_per_sec_inv  = (1.0f / CLOCKS_PER_SEC);
        };


        template<class Future>
        int wait_for_reponse(const Future &fut, const unsigned int timeout_ms = 1000) const
        {
            /* Wait for the response */
            if (fut.wait_for(std::chrono::duration<int, std::milli>(timeout_ms)) == std::future_status::ready)
            {
                /* Return that a response was received */
                return 1;
            }

            /* If not ask weather to try again */
            std::cout << "Attempt to connect to server failed" << std::endl;
            std::cout << "Try again, y for yes, else no" << std::endl;

            /* Get response and return give up if not y or Y */
            char resp;
            std::cin >> resp;
            if ((resp != 'y') && (resp != 'Y'))
            {
                return 0;
            }

            /* Return continue trying */
            return -1;
        }

        boost::asio::io_service     _io_service;
        protocol_stack              _uni_cast_conn;
        protocol_stack       *      _multi_cast_conn;
        group  *                    _group;
        raptor_raytracer::camera *  _cam;
        std::uint32_t               _id;
        bool                        _subscribed;
};
}; /* namespace raptor_networking */

#endif /* #ifndef __MSG_CLIENT_H__ */
