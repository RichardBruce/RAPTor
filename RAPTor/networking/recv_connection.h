#pragma once

/* Standard headers */
#include <vector>


namespace raptor_networking
{
/* Class representing a connection to a socket */
template<class Connection>
class recv_connection : public Connection
{
    public :
        recv_connection(const short port) : Connection(port) {  };

        recv_connection(const boost::asio::ip::address& listen_address, 
            const boost::asio::ip::address& multicast_address, const short port) 
            : Connection(listen_address, multicast_address, port) { };

        /* Template method to receive and deserialise without expecting to send a response */
        template<class T>
        bool listen_and_ignore(T *const recv, const int timeout_ms)
        {
            typename Connection::endpoint dummy;
            return listen_and_ignore(recv, &dummy, timeout_ms);
        }

        template<class T>
        bool listen_and_ignore(T *const recv, typename Connection::endpoint *const sender_endpoint, const int timeout_ms)
        {
            listen_and_ignore_internal(recv, sender_endpoint, timeout_ms);

            /* Close connection so it can be reused */
            this->close_connection();

            return true;
        }

        /* Template method to receive, deserialise and send a response */
        template<class T, class R>
        bool listen_and_confirm(T *const recv, R &resp, const int timeout_ms)
        {
            typename Connection::endpoint dummy;
            return listen_and_confirm(recv, resp, &dummy, timeout_ms);
        }

        template<class T, class R>
        bool listen_and_confirm(T *const recv, R &resp, typename Connection::endpoint *const sender_endpoint, 
            const int timeout_ms)
        {
            /* Receive message */
            if (!listen_and_ignore_internal(recv, sender_endpoint, timeout_ms))
            {
                return false;
            }

            /* Acknowledge */
            std::vector<char> serial_msg(serialise(resp(recv)));

            /* Add a header, chop into packets and send */
            this->packetise_and_send(serial_msg, *sender_endpoint);

            /* Close connection so it can be reused */
            this->close_connection();

            return true;
        }

    private :
        template<class T>
        bool listen_and_ignore_internal(T *const recv, typename Connection::endpoint *const sender_endpoint, const int timeout_ms)
        {
            /* Reopen the socket if it was previously closed */
            if (!this->open_socket_for_listen(timeout_ms))
            {
                return false;
            }

            /* Receive message */
            if (!this->receive_and_depacketise(recv, sender_endpoint, timeout_ms))
            {
                return false;
            }

            return true;
        }
};
}; /* namespace raptor_networking */
