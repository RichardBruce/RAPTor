#ifndef __SUBSCRIPTION_RESPONSE_H__
#define __SUBSCRIPTION_RESPONSE_H__

/* Standard headers */
#include <iostream>
#include <string>
#include <vector>

/* Networking */
#include "conversion.h"
#include "msg_server.h"


namespace raptor_networking
{
/* Class to always accept or deny a connection */
class auto_subscription_response
{
    public :
        auto_subscription_response(const bool accept = true) : _accept(accept) { };

        std::string operator()(const std::unique_ptr<std::istream> &req) const
        {
            if (_accept)
            {
                return "ok";
            }
            else
            {
                return "";
            }
        }

    private :
        const bool _accept;
};


/* Class to ask on std::out weather to accept a connection */
class console_ask_subscription_response
{
    public :
        std::string operator()(const std::unique_ptr<std::istream> &req) const
        {
            std::cout << "Permission subscription to user with the following request: " << std::endl;
            std::cout << *req;
            std::cout << std::endl;
            std::cout << "Enter Y for yes, else no" << std::endl;

            char user_resp;
            std::cin >> user_resp;
            if ((user_resp == 'y') || (user_resp == 'Y'))
            {
                return "ok";
            }
            else
            {
                return "";
            }
        }
};


template<class T>
class subscription_response
{
    public :
        subscription_response(msg_server *const server, const T &user_resp) 
            : _server(server), _user_resp(user_resp) {  };

        std::vector<char>* operator()(const std::unique_ptr<std::istream> &req) const
        {
            /* Then pass to the user handler */
            const std::string user_resp(_user_resp(req));
            if (user_resp.empty())
            {
                std::cout << "user denied" << std::endl;
                return new std::vector<char>(MSG_DENY);
            }
            else
            {
                /* User accepted, generate id and ports */
                const auto info = _server->create_user();

                /* Insert the id into the response */
                std::vector<char>* resp = new std::vector<char>(MSG_ACCEPT);
                resp->resize(MSG_ACCEPT.size() + user_resp.size() + 12);

                to_big_endian_byte_array<std::uint32_t, char>(info.first, &resp->data()[MSG_ACCEPT.size() + 0]);
                
                /* Insert the multi cast port into the response */
                const auto multi_cast_info = _server->multi_cast_info();
                to_big_endian_byte_array<std::uint16_t, char>(multi_cast_info.second, &resp->data()[MSG_ACCEPT.size() + 4]);

                /* Split the multi cast address into bytes and add to the response */
                const std::string multi_addr(multi_cast_info.first);
                const int dot0 = multi_addr.find('.');
                const int dot1 = multi_addr.find('.', dot0 + 1);
                const int dot2 = multi_addr.find('.', dot1 + 1);
                std::uint8_t multi_addr_arr[4] = 
                {
                    static_cast<std::uint8_t>(boost::lexical_cast<unsigned int>(multi_addr.substr(0, dot0))),
                    static_cast<std::uint8_t>(boost::lexical_cast<unsigned int>(multi_addr.substr(dot0 + 1, dot1 - dot0 - 1))),
                    static_cast<std::uint8_t>(boost::lexical_cast<unsigned int>(multi_addr.substr(dot1 + 1, dot2 - dot1 - 1))),
                    static_cast<std::uint8_t>(boost::lexical_cast<unsigned int>(multi_addr.substr(dot2 + 1)))
                };

                to_big_endian_byte_array<std::uint8_t, char>(multi_addr_arr[0], &resp->data()[MSG_ACCEPT.size() + 6]);
                to_big_endian_byte_array<std::uint8_t, char>(multi_addr_arr[1], &resp->data()[MSG_ACCEPT.size() + 7]);
                to_big_endian_byte_array<std::uint8_t, char>(multi_addr_arr[2], &resp->data()[MSG_ACCEPT.size() + 8]);
                to_big_endian_byte_array<std::uint8_t, char>(multi_addr_arr[3], &resp->data()[MSG_ACCEPT.size() + 9]);

                /* Add the unicast address to the response */
                to_big_endian_byte_array<std::uint16_t, char>(info.second.port(), &resp->data()[MSG_ACCEPT.size() + 10]);

                /* Append user response */
                std::copy(user_resp.begin(), user_resp.end(), &resp->data()[MSG_ACCEPT.size() + 12]);

                return resp;
            }
        }

    private :
        msg_server  *const  _server;
        const T     &       _user_resp;
};


class unsubscription_response
{
    public :
        unsubscription_response(msg_server *const server) : _server(server) {  };

        std::vector<char>* operator()(const std::unique_ptr<std::istream> &req) const
        {
            /* Remove from subscription map */
            char id_buff[sizeof(std::uint32_t)];
            req->read(id_buff, sizeof(std::uint32_t));

            const std::uint32_t id = from_big_endian_byte_array<char, std::uint32_t>(id_buff);
            std::cout << "unsubscribing: " << id << std::endl;
            _server->erase_user(id);

            return new std::vector<char>(MSG_ACKNOWLEDGE);
        }

    private :
        msg_server *const _server;
};
}; /* namespace raptor_networking */

#endif /* #ifndef __SUBSCRIPTION_RESPONSE_H__ */
