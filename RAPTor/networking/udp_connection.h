#ifndef __UDP_CONNECTION_H__
#define __UDP_CONNECTION_H__

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
              _head_buf(new char[HEADER_SIZE]),
              _send_port(send_port)
        {  };

        /* Open for multi cast */
        udp_connection(stack_controller *const ctrl, group *const grp, const boost::asio::ip::address &addr, 
            const boost::asio::ip::address &multi_addr, const short port) 
            : connection<UpNode>(ctrl, grp),
              _recv_endpoint(addr, port),
              _send_socket(ctrl->io_service()),
              _recv_socket(ctrl->io_service()),
              _head_buf(new char[HEADER_SIZE]),
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

            delete [] _head_buf;
        }

        /* Start waiting for data on the socket */
        virtual void start_receiving() override
        {
            while (true)
            {
                /* Prepare receive buffer */
                char *data_buf = new char [MAX_UDP_SIZE];
                char *head_buf = new char [HEADER_SIZE];
                std::array<boost::asio::mutable_buffer, 2> recv_buf =
                {{
                    boost::asio::buffer(head_buf, HEADER_SIZE),
                    boost::asio::buffer(data_buf, MAX_UDP_SIZE)
                }};

                /* Wait for data */
                udp::endpoint recv_endpoint;
                std::cout << "recv started" << std::endl;
                const size_t bytes = _recv_socket.receive_from(recv_buf, recv_endpoint);
                boost::asio::ip::address recv_address(recv_endpoint.address());

                std::cout << "Receiving from: " << msg_header::from_stack(head_buf) << std::endl;
                this->_ctrl->post(msg_header::from_stack(head_buf), [this, recv_address, head_buf, data_buf, bytes](const stack_accessor &acc)
                    {
                        std::shared_ptr<msg_header> header(new msg_header(head_buf, recv_address));
                        delete [] head_buf;
                        received(acc, std::shared_ptr<msg_data>(new msg_data(data_buf, bytes - HEADER_SIZE)), header);
                    });
            }
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

            udp::endpoint send_endpoint(*to_addr, _send_port);

            /* Connect to the endpoint */
            _send_socket.connect(send_endpoint, ec);
            if (ec)
            {
                return;
            }

            /* Build the message */
            std::array<char, HEADER_SIZE> serial_header;
            header->serialise(serial_header.data());
            std::array<boost::asio::const_buffer, 2> send_buf =
            {{
                boost::asio::buffer(serial_header), 
                boost::asio::buffer(*data, header->fragment_length())
            }};

            /* Send the message to the given endpoint */
            _send_socket.send_to(send_buf, send_endpoint, 0, ec);
        }

    private :
        udp::endpoint           _recv_endpoint;
        udp::socket             _send_socket;
        udp::socket             _recv_socket;
        char *const             _head_buf;
        const short             _send_port;
};
}; /* namespace raptor_networking */

#endif /* #ifndef __UDP_CONNECTION_H__ */

        // /* Start waiting for data on the socket */
        // virtual void start_receiving() override
        // {
        //     /* Prepare receive buffer */
        //     char *data_buf = new char [MAX_UDP_SIZE];
        //     std::array<boost::asio::mutable_buffer, 2> recv_buf =
        //     {{
        //         boost::asio::buffer(_head_buf, HEADER_SIZE),
        //         boost::asio::buffer(data_buf, MAX_UDP_SIZE)
        //     }};

        //     /* Wait for data */
        //     _recv_socket.async_receive_from(recv_buf, _recv_endpoint,
        //         [&,data_buf](const boost::system::error_code &, size_t bytes_transferred)
        //         {
        //             received(_recv_endpoint.address(), new msg_data(data_buf, bytes_transferred - HEADER_SIZE), new msg_header(_head_buf));
        //         });
        // }

        // /* Virtual functions for sending and receiving messages */
        // virtual void received(const boost::asio::ip::address &addr, msg_data *const data, const msg_header *const header) override
        // {
        //     /* Pend the next read */
        //     start_receiving();

        //     /* Check if there exists a parallel stack for this address, if not create it */
        //     //std::cout << "Receiving " << header->sequence() << ", " << header->fragment() << " at: " << clock() << std::endl;
        //     typename stack_map::accessor stack_acc;
        //     if (_strands.insert(stack_acc, addr.to_string()))
        //     {
        //         std::cout << "cloning stack: " << addr.to_string() << std::endl;
        //         stack_acc->second = this->_up_node->clean_clone();
        //     }

        //     /* While holding the lock on this stack, propagate the message */
        //     stack_acc->second->received(addr, data, header);

        //     /* Pass the data up the stack */
        //     //std::cout << "Got seq: " << header->sequence() << " frag: " << header->fragment() << std::endl;
        //     //this->_up_node->received(addr, data, header);
        // }