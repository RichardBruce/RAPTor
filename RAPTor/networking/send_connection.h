#ifndef __SEND_CONNECTION_H__
#define __SEND_CONNECTION_H__

/* Standard headers */
#include <string>
#include <vector>


namespace raptor_networking
{
/* Class representing a connection to a socket */
template<class Connection>
class send_connection : public Connection
{
    public :
        send_connection(const boost::asio::ip::address& address, const short port) 
            : Connection(address, port) { };

        /* Template method to serialise and send without expecting a response */
        template<class T>
        bool send_and_forget(const T &msg)
        {
            /* Send message */
            if (!send_and_forget_internal(msg))
            {
                return false;
            }

            /* Close connection so it can be reused */
            this->close_connection();

            return true;
        }

        /* Template method to serialise and send expecting a response */
        template<class R, class T>
        bool send_and_confirm(const T &msg, R *const recv, const int timeout_ms = 1000)
        {
            /* Send the message */
            if (!send_and_forget_internal(msg))
            {
                return false;
            }

            /* Wait for the reply */
            typename Connection::endpoint sender_endpoint;
            if (!this->receive_and_depacketise(recv, &sender_endpoint, timeout_ms))
            {
                /* Timed out */
                return false;
            }

            /* Close connection so it can be reused */
            this->close_connection();

            return true;
        }

    private :
        template<class T>
        bool send_and_forget_internal(const T &msg)
        {
            /* Reopen the socket if it was previously closed */
            if (!this->open_socket_for_send())
            {
                return false;
            }

            /* Serialise the message */
            std::vector<char> serial_msg(serialise(msg));

            /* Add a header, chop into packets and send */
            this->packetise_and_send(serial_msg);

            return true;
        }
};
}; /* namespace raptor_networking */

#endif /* #ifndef __SEND_CONNECTION_H__ */
