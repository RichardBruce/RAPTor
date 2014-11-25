#ifndef __TCP_CONNECTION_H__
#define __TCP_CONNECTION_H__

/* Standard headers */
#include <array>

/* Boost headers */
#include "boost/asio/ip/tcp.hpp"
#include "boost/asio/read.hpp"
#include "boost/asio/write.hpp"

/* Networking Headers */
#include "networking_constants.h"
#include "connection.h"
#include "stack_accessor.h"
#include "stack_controller.h"
#include "group.h"


namespace raptor_networking
{
using boost::asio::ip::tcp;


/* Class to represent a tcp connection */
template<class UpNode>
class tcp_connection : public connection<UpNode>
{
    public :
        /* Open connection */
        tcp_connection(stack_controller *const ctrl, group *const grp, const short send_port, const short recv_port) 
            : connection<UpNode>(ctrl, grp),
              _recv_endpoint(tcp::v4(), recv_port), 
              _acceptor(ctrl->io_service(), _recv_endpoint), 
              _send_socket(ctrl->io_service()),
              _recv_socket(ctrl->io_service()),
              _head_buf(new char [msg_header::size()]),
              _send_port(send_port)
        {  };

        ~tcp_connection()
        {
            delete [] _head_buf;
        }

        /* Access functions */
        short recv_port() const { return _recv_endpoint.port(); }
        short send_port() const { return _send_port;            }

        
        /* Virtual functions for sending and receiving messages */
        virtual void received(const stack_accessor &acc, const std::shared_ptr<msg_data> &data, const std::shared_ptr<msg_header> &header)
        {
            /* Pend the next read */
            _recv_socket.close();
            start_receiving();

            /* Pass the data up the stack */
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

            tcp::endpoint send_endpoint(*to_addr, _send_port);

            /* Try to connect to the given address */
            _send_socket.connect(send_endpoint, ec);
            if (ec)
            {
                std::cout << "Connect failed: " << ec.message() << std::endl;
                return;
            }

            /* Build the message */
            std::array<char, msg_header::size()> serial_header;
            header->serialise(serial_header.data());
            std::array<boost::asio::const_buffer, 2> send_buf =
            {{
                boost::asio::buffer(serial_header),
                boost::asio::buffer(*data, header->fragment_length())
            }};

            /* Send the message */
            boost::asio::write(_send_socket, send_buf, ec);
            if (ec)
            {
                std::cout << "Write failed: " << ec.message() << std::endl;
            }
            _send_socket.close();
        }

        /* Start waiting for a connection on the socket */
        virtual void start_receiving() override
        {
            /* Start async connect */
            _acceptor.async_accept(_recv_socket, _recv_endpoint, [this](const boost::system::error_code &ec)
                {
                    if (!ec)
                    {
                        receive_header();
                    }
                    else
                    {
                        std::cout << "Start receiving failed: " << ec.message() << std::endl;
                    }
                });
        }

    private :
        /* Start waiting for the header on the socket */
        void receive_header()
        {
            /* Wait for data */
            boost::asio::async_read(_recv_socket, boost::asio::buffer(_head_buf, msg_header::size()),
                [this](const boost::system::error_code &ec, size_t)
                {
                    if (!ec)
                    {
                        receive_data(_recv_endpoint.address(), std::shared_ptr<msg_header>(new msg_header(_head_buf, _recv_endpoint.address())));
                    }
                    else
                    {
                        std::cout << "Receive header failed: " << ec.message() << std::endl;
                    }
                });
        }

        /* Start waiting for the data on the socket */
        void receive_data(const boost::asio::ip::address &addr, const std::shared_ptr<msg_header> &header)
        {
            /* Prepare receive buffer */
            char *data_buf = new char [header->fragment_length()];

            /* Wait for data */
            boost::asio::async_read(_recv_socket, boost::asio::buffer(data_buf, header->fragment_length()),
                [this, data_buf, header, addr](const boost::system::error_code &, size_t)
                {
                    stack_accessor acc;
                    this->_ctrl->lock_stack(acc, header->from_address());
                    received(acc, std::shared_ptr<msg_data>(new msg_data(data_buf, header->fragment_length())), header);
                });
        }

        tcp::endpoint           _recv_endpoint;
        tcp::acceptor           _acceptor;
        tcp::socket             _send_socket;
        tcp::socket             _recv_socket;
        char *const             _head_buf;
        const short             _send_port;
};
}; /* namespace raptor_networking */

#endif /* #ifndef __TCP_CONNECTION_H__ */
