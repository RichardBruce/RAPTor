#ifndef __PROTOCOL_STACK_H__
#define __PROTOCOL_STACK_H__

/* Standard heaaders */
#include <functional>
#include <future>
#include <vector>

/* Boost headers */
#include "boost/asio/io_service.hpp"
#include "boost/noncopyable.hpp"

/* Networking headers */
#include "stack_controller.h"
#include "connection.h"
#include "group.h"


namespace raptor_networking
{
/* Forward declarations */
class group;
class data_receiver;

/* Class repesenting the top level of a protocol stack */
class protocol_stack : private boost::noncopyable
{
    public :
        /* CTOR for unicast */
        explicit protocol_stack(boost::asio::io_service &io_service, data_receiver *const receiver, 
            group *const grp, const std::uint16_t send_port, const std::uint16_t recv_port, const bool tcp);
        
        explicit protocol_stack(boost::asio::io_service &io_service, data_receiver *const receiver, 
            const std::string &me_addr, const std::uint16_t send_port, const std::uint16_t recv_port, 
            const std::uint16_t port_offset, const bool tcp);

        /* CTOR for multi cast */
        explicit protocol_stack(boost::asio::io_service &io_service, data_receiver *const receiver, 
            group *const grp, const boost::asio::ip::address &addr, const boost::asio::ip::address &multi_addr, const std::uint16_t port);

        explicit protocol_stack(boost::asio::io_service &io_service, data_receiver *const receiver, 
            const std::string &me_addr, const boost::asio::ip::address &addr,  const boost::asio::ip::address &multi_addr, 
            const std::uint16_t port, const std::uint16_t port_offset);

        ~protocol_stack()
        {
            /* Clean up the connection */
            delete _conn;
        }

        protocol_stack& start(const std::size_t nr_threads, const bool tcp = true)
        {
            /* Start receiving data */
            if (tcp)
            {
                _conn->start_receiving();
            }
            else
            {
                _io_service.post(std::bind(&connection<stack_component>::start_receiving, _conn));
            }

            /* Start the thread pool */
            _ctrl.start(nr_threads);

            return *this;
        }

        /* Access to the groups used by this stack */
        group *const stack_group() { return _conn->stack_group(); }

        /* Send to an old physical ip address */
        bool send(std::shared_ptr<std::vector<char>> &data, const boost::asio::ip::address &to_addr, 
            std::future<std::istream *const> *resp, const std::uint32_t resp_to = 0, const bool is_resp = false)
        {
            /* Start the message down the protocol stack */
            boost::system::error_code ec;
            _ctrl.run(boost::uuids::nil_uuid(), [this, data, to_addr, resp_to, is_resp, &ec, resp](const stack_accessor &acc)
                {
                    /* Build header */
                    std::shared_ptr<msg_header> header(new msg_header(to_addr, acc.next_sequence(), data->size(), resp_to, is_resp));

                    /* Send down the stack */
                    acc.stack_top()->send(acc, data, header, resp, ec);
                });

            return !ec;
        }

        bool send(std::shared_ptr<std::vector<char>> &data, const boost::asio::ip::address &to_addr, const std::uint32_t resp_to = 0, 
            const bool is_resp = false)
        {
            /* Start the message down the protocol stack */
            boost::system::error_code ec;
            _ctrl.run(boost::uuids::nil_uuid(), [this, data, to_addr, resp_to, is_resp, &ec](const stack_accessor &acc)
                {
                    /* Build header */
                    std::shared_ptr<msg_header> header(new msg_header(to_addr, acc.next_sequence(), data->size(), resp_to, is_resp));

                    /* Send down the stack */
                    acc.stack_top()->send(acc, data, header, ec);
                });

            return !ec;
        }

        /* Send to a known individual */
        bool send(std::shared_ptr<std::vector<char>> &data, const uuid &to_addr, 
            std::future<std::istream *const> *resp, const std::uint32_t resp_to = 0, const bool is_resp = false)
        {
            /* Start the message down the protocol stack */
            boost::system::error_code ec;
            _ctrl.run(to_addr, [this, data, to_addr, resp_to, is_resp, &ec, resp](const stack_accessor &acc)
                {
                    /* Build header */
                    std::shared_ptr<msg_header> header(new msg_header(to_addr, acc.next_sequence(), data->size(), resp_to, is_resp));

                    /* Send down the stack */
                    acc.stack_top()->send(acc, data, header, resp, ec);
                });

            return !ec;
        }

        bool send(std::shared_ptr<std::vector<char>> &data, const uuid &to_addr, const std::uint32_t resp_to = 0, 
            const bool is_resp = false)
        {
            /* Start the message down the protocol stack */
            boost::system::error_code ec;
            _ctrl.run(to_addr, [this, data, to_addr, resp_to, is_resp, &ec](const stack_accessor &acc)
                {
                    /* Build header */
                    std::shared_ptr<msg_header> header(new msg_header(to_addr, acc.next_sequence(), data->size(), resp_to, is_resp));

                    /* Send down the stack */
                    acc.stack_top()->send(acc, data, header, ec);
                });

            return !ec;
        }

        /* Send to a group */
        bool send(std::shared_ptr<std::vector<char>> &data, const std::string &to_group, 
            std::future<std::istream *const> *resp, const std::uint32_t resp_to = 0, const bool is_resp = false)
        {
            /* Get group address */
            const uuid to_addr = stack_group()->group_address();

            /* Start the message down the protocol stack */
            boost::system::error_code ec;
            _ctrl.run(to_addr, [this, data, to_addr, resp_to, is_resp, &ec, resp](const stack_accessor &acc)
                {
                    /* Build header */
                    std::shared_ptr<msg_header> header(new msg_header(boost::uuids::nil_uuid(), to_addr, acc.next_sequence(), data->size(), resp_to, is_resp));

                    /* Send down the stack */
                    acc.stack_top()->send(acc, data, header, resp, ec);
                });

            return !ec;
        }

        bool send(std::shared_ptr<std::vector<char>> &data, const std::string &to_group, const std::uint32_t resp_to = 0, 
            const bool is_resp = false)
        {
            /* Get group address */
            const uuid to_addr = stack_group()->group_address();

            /* Start the message down the protocol stack */
            boost::system::error_code ec;
            _ctrl.run(to_addr, [this, data, to_addr, resp_to, is_resp, &ec](const stack_accessor &acc)
                {
                    /* Build header */
                    std::shared_ptr<msg_header> header(new msg_header(boost::uuids::nil_uuid(), to_addr, acc.next_sequence(), data->size(), resp_to, is_resp));

                    /* Send down the stack */
                    acc.stack_top()->send(acc, data, header, ec);
                });

            return !ec;
        }

    private :
        stack_controller                _ctrl;
        boost::asio::io_service &       _io_service;
        connection<stack_component> *   _conn;
};
}; /* namespace raptor_networking */

#endif /* #ifndef __PROTOCOL_STACK_H__ */
