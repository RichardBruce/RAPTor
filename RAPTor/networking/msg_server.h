#pragma once

/* Standard headers */
#include <atomic>
#include <cstdint>
#include <cstdlib>
#include <string>
#include <unordered_map>
#include <vector>

/* Boost */
#include "boost/asio/io_service.hpp"
#include "boost/lexical_cast.hpp"
#include "boost/uuid/uuid.hpp"

/* Networking */
#include "user_connection_info.h"
#include "networking_constants.h"
#include "protocol_stack.h"


namespace raptor_networking
{
using boost::uuids::uuid;
using boost::asio::ip::address;

class msg_server
{
    public :
        msg_server(const std::string &me_addr, const std::string &addr, const std::string &multi_addr, const std::uint16_t send_port, 
            const std::uint16_t recv_port, const std::uint16_t multi_port, const std::uint16_t port_off)
            : _io_service(),
              _user_map(), 
              _uni_cast_conn(_io_service, new uni_cast_receiver(_handler_map), me_addr, send_port, recv_port, port_off, true), 
              _multi_cast_conn(_io_service, new multi_cast_receiver(), me_addr, address::from_string(addr), address::from_string(multi_addr), multi_port, port_off), 
              _multi_addr(multi_addr),
              _multi_port(multi_port),
              _uni_cast_port(recv_port + 2) {  };

        ~msg_server()
        {
            std::cout << "stopped listening for clients" << std::endl;
            _io_service.stop();
        }

        msg_server& start()
        {
            /* Wait for a client to subscribe */
            /* Use the resp_handler to create the response */
            _uni_cast_conn.start(2);
            _multi_cast_conn.start(2, false);

            return *this;
        }

        template<class T>
        void publish_unconfirmed(const T &msg)
        {
            /* Serialise */
            std::unique_ptr<std::vector<char>> serial_msg(new std::vector<char>());
            serialise(serial_msg.get(), msg);

            /* Send */
            _multi_cast_conn.send(serial_msg.get(), "group name here");
        }

        bool insert_request_handler(const char * req, const std::function<std::vector<char>* (const std::unique_ptr<std::istream> &)> &handler)
        {
            const auto insert_iter = _handler_map.insert({req, handler});
            return insert_iter.second;
        }

        /* Create a user */
        const std::pair<std::uint32_t, user_connection_info> create_user()
        {
            /* Generate a "random" user id and check if it is in use */
            int id;
            do
            {
                id = std::rand();
            } while (_user_map.find(id) != _user_map.end());

            /* Insert the new id */
            auto insert_iter = _user_map.insert(std::pair<std::uint32_t, user_connection_info>(id, user_connection_info(_uni_cast_port++)));

            return *(insert_iter.first);
        }

        /* Remove a user */
        void erase_user(const std::uint32_t id)
        {
            const auto id_iter = _user_map.find(id);
            if (id_iter != _user_map.end())
            {
                _user_map.erase(id_iter);
            }
        }

        /* Multi cast info */
        std::pair<std::string, std::uint16_t> multi_cast_info() const
        {
            return std::pair<std::string, std::uint16_t>(_multi_addr, _multi_port);
        }

    private :
        /* Class to compare two messages types kept as char * */
        class message_type_compare
        {
            public :
                bool operator()(const char *const lhs, const char *const rhs) const
                {
                    return strncmp(lhs, rhs, MESSAGE_TYPE_SIZE) < 0;
                }
        };

        /* Typedefs to save typing */
        typedef std::map<std::uint32_t, user_connection_info> user_info_map;
        typedef std::function<std::vector<char>*const (const std::unique_ptr<std::istream> &)> handler_function;
        typedef std::map<const char*, handler_function, message_type_compare> req_handler_map;

        boost::asio::io_service _io_service;
        req_handler_map         _handler_map;
        user_info_map           _user_map;
        protocol_stack          _uni_cast_conn;
        protocol_stack          _multi_cast_conn;
        const std::string &     _multi_addr;
        const std::uint16_t     _multi_port;
        std::uint16_t           _uni_cast_port;

        /* Inner class to receive uni cast messages */
        class uni_cast_receiver : public data_receiver
        {
            public :
                uni_cast_receiver(req_handler_map &handler_map) : _handler_map(handler_map) { };

                virtual std::vector<char>* received(const std::unique_ptr<std::istream> &data) override
                {
                    /* Check if there is a handler */
                    char msg_type[MESSAGE_TYPE_SIZE];
                    data->read(msg_type, MESSAGE_TYPE_SIZE);

                    auto handler_iter = _handler_map.find(msg_type);
                    if (handler_iter == _handler_map.end())
                    {
                        return new std::vector<char>(MSG_UNKNOWN);
                    }

                    return handler_iter->second(data);
                }

                virtual uni_cast_receiver* clean_clone() override
                {
                    return new uni_cast_receiver(_handler_map);
                }

            private :
                req_handler_map &_handler_map;
        };

        /* Inner class to receive multi cast messages */
        class multi_cast_receiver : public data_receiver
        {
            public :
                virtual std::vector<char>* received(const std::unique_ptr<std::istream> &data) override
                {
                    std::cout << "WARNING: Received multi cast message, unexpected" << std::endl;
                    return nullptr;
                }

                virtual multi_cast_receiver* clean_clone() override
                {
                    return new multi_cast_receiver();
                }
        };
};
}; /* namespace raptor_networking */
