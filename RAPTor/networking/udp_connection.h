#pragma once

/* Standard headers */
#include <array>

/* Boost headers */
#include "boost/asio/ip/udp.hpp"
#include "boost/asio/ip/multicast.hpp"

/* Common headers */
#include "common.h"

/* Networking headers */
#include "networking_constants.h"
#include "connection.h"
#include "stack_accessor.h"
#include "stack_controller.h"
#include "group.h"


namespace raptor_networking
{
using boost::asio::ip::udp;
using boost::asio::ip::address;


/* Class to represent a udp connection, which may multi cast or uni cast */
template<class UpNode>
class udp_connection : public connection<UpNode>
{
    public :
        /* Open connection for unicast */
        udp_connection(stack_controller *const ctrl, group *const grp, const short send_port, const short recv_port) 
            : connection<UpNode>(ctrl, grp),
              _recv_endpoint(udp::v4(), recv_port), 
              _send_socket(ctrl->io_service()),
              _recv_socket(ctrl->io_service(), _recv_endpoint),
              _send_port(send_port)
        {  };

        /* Open for multi cast */
        udp_connection(stack_controller *const ctrl, group *const grp, const boost::asio::ip::address &addr, 
            const boost::asio::ip::address &multi_addr, const short port) 
            : connection<UpNode>(ctrl, grp),
              _recv_endpoint(addr, port),
              _send_socket(ctrl->io_service()),
              _recv_socket(ctrl->io_service()),
              _send_port(port)
        {
            /* Create a socket that can share an address, for multicast receive */
            _recv_socket.open(_recv_endpoint.protocol());
            _recv_socket.set_option(udp::socket::reuse_address(true));
            _recv_socket.bind(_recv_endpoint);

            /* Join the multi cast group */
            _recv_socket.set_option(boost::asio::ip::multicast::join_group(multi_addr));
        };

        /* DTOR */
        virtual ~udp_connection()
        {
            _send_socket.close();
            _recv_socket.close();
        }

        /* Access functions */
        short recv_port() const { return _recv_socket. local_endpoint().port();   }
        short send_port() const { return _send_port;                              }

        /* Start waiting for data on the socket */
        virtual void start_receiving() override
        {
            /* Prepare buffers */
            this->_data_buf.reset(new char [MAX_UDP_SIZE]);
            _recv_buf = 
            {
                boost::asio::buffer(this->_head_buf),
                boost::asio::buffer(this->_data_buf.get(), MAX_UDP_SIZE)
            };

            /* Wait for data */
            _recv_socket.async_receive_from(_recv_buf, _recv_endpoint, [this](const boost::system::error_code &ec, size_t bytes)
                {
                    /* Check for error */
                    if (ec)
                    {
                        std::cout << "Receive failed: " << ec.message() << std::endl;
                        return;
                    }

                    /* Get the data from the receive buffers */
                    const auto addr(_recv_endpoint.address());
                    std::shared_ptr<msg_header> header(new msg_header(this->_head_buf.data(), addr));
                    char *data_buf = this->_data_buf.release();

                    /* Start next receive */
                    start_receiving();

                    /* Start up the stack */
                    stack_accessor acc;
                    this->_ctrl->lock_stack(acc, header->from_address());
                    received(acc, std::shared_ptr<msg_data>(new msg_data(data_buf, bytes - msg_header::size())), header);
                });
        }

        /* Virtual functions for sending and receiving messages */
        virtual void received(const stack_accessor &acc, const std::shared_ptr<msg_data> &data, const std::shared_ptr<msg_header> &header) override
        {
            /* While holding the lock on this stack, propagate the message */
            acc.stack_bottom()->received(acc, data, header);
        }

        virtual void send(const stack_accessor &acc, const std::shared_ptr<std::vector<char>> &data, const std::shared_ptr<msg_header> &header, boost::system::error_code &ec) override
        {
            /* TODO -- This might not be safe, perhaps need sync send or some locking */
            /* If a physical address is given use it, else look up from the logical address */
            const boost::asio::ip::address *to_addr = header->physical_address();
            if (to_addr == nullptr)
            {
                header->from_address(this->_grp->my_address());
                to_addr = this->_grp->physical_address(header->to_address());
            }

            /* Connect to the endpoint */
            udp::endpoint send_endpoint(*to_addr, _send_port);
            _send_socket.connect(send_endpoint, ec);
            if (ec)
            {
                std::cout << "Connect failed: " << ec.message() << std::endl;
                return;
            }

            /* Build the message */
            std::array<char, HEADER_SIZE> serial_header;
            header->serialise(serial_header.data());
            std::array<boost::asio::const_buffer, 2> send_buf =
            {
                boost::asio::buffer(serial_header), 
                boost::asio::buffer(*data, header->fragment_length())
            };

            /* Send the message to the given endpoint */
            _send_socket.send_to(send_buf, send_endpoint, 0, ec);
            if (ec)
            {
                std::cout << "Send failed: " << ec.message() << std::endl;
            }
        }

    private :
        udp::endpoint                               _recv_endpoint;
        udp::socket                                 _send_socket;
        udp::socket                                 _recv_socket;
        std::array<boost::asio::mutable_buffer, 2>  _recv_buf;
        const short                                 _send_port;
};
}; /* namespace raptor_networking */
