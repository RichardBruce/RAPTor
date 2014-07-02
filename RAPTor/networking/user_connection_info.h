#ifndef __USER_CONNECTION_INFO_H__
#define __USER_CONNECTION_INFO_H__

/* Standard headers */
#include <cstdint>

/* Networking */
#include "user_connection_info.h"


namespace raptor_networking
{
/* Class to keep information associated with one client */
class user_connection_info
{
    public :
        user_connection_info(const std::uint16_t port) : _port(port) { };

        std::uint16_t port() const { return _port; }

    private :
        std::uint16_t _port;
};
}; /* namespace raptor_networking */

#endif /* #ifndef __USER_CONNECTION_INFO_H__ */
