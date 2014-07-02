#ifndef __NETWORKING_CONSTANTS_H__
#define __NETWORKING_CONSTANTS_H__

/* Standard headers */
#include <cstdint>
#include <string>
#include <vector>

namespace raptor_networking
{
/* Message content */
const std::vector<char> MSG_UNKNOWN({ 'u', 'n', 'k' });             /* No handler found                                                 */
const std::vector<char> MSG_SUBSCRIBE({ 's', 'u', 'b' });           /* Subscribe to a server                                            */
const std::vector<char> MSG_INIT({ 'i', 'n', 'i' });                /* Request initialisation from a server                             */
const std::vector<char> MSG_UNSUBSCRIBE({ 'u', 's', 'b' });         /* Unsubscribe from a server                                        */
const std::vector<char> MSG_ACCEPT({ 'a', 'c', 'c' });              /* Accept a join request                                            */
const std::vector<char> MSG_DENY({ 'd', 'e', 'n' });                /* Deny a join request                                              */
const std::vector<char> MSG_ACKNOWLEDGE({ 'a', 'c', 'k' });         /* Acknowledge a message                                            */
const std::vector<char> MSG_STABLE({ 's', 'b', 'l' });              /* Report messages as being seen                                    */
const std::vector<char> MSG_ERROR({ 'e', 'r', 'r' });               /* Response last message had an error                               */
const std::vector<char> MSG_RESEND({ 'r', 's', 'd' });              /* Request a resend                                                 */
const std::vector<char> MSG_BATCH({ 'b', 'a', 't' });               /* Message that is a batch of messages                              */

/* Header layout */
const int HEADER_SIZE           = 73;                               /* The header is 2 uint32_t wide                                    */
const int FRAGMENT_IDX_POSITION = 0;                                /* The fragment id goes in the first position                       */
const int SEQ_IDX_POSITION      = sizeof(std::uint32_t);            /* The connection sequence id goes in the second position           */
const int SEQ_SIZE_POSITION     = sizeof(std::uint32_t) * 2;        /* The size (in bytes) of the sequence goes in the third position   */
const int CONTEXT_SEQ_POSITION  = sizeof(std::uint32_t) * 3;        /* The per context sequence id goes in the fourth place             */
const int RESPONSE_TO_POSITION  = sizeof(std::uint32_t) * 4;        /* The sequence id being responded to goes in the fifth position    */
const int FLAGS_POSITION        = sizeof(std::uint32_t) * 5;        /* The flags go in the las position                                 */

/* Message sizes */
const int MAX_PACKET_SIZE   = 32 * 1024;                            /* The packet can be up to 64k bytes                                */
const int MESSAGE_TYPE_SIZE = 3;                                    /* The number of bytes used to define the message type              */
const int MAX_UDP_SIZE      = (1 << 16) - 1;                        /* The maximum size of a UDP datagram                               */

/* Port map */
const std::uint16_t CONTROL_SEND_PORT   = 7000;                     /* The port the server will send chat to the client on              */
const std::uint16_t CONTROL_RECV_PORT   = 7001;                     /* The port the server will receive chat from the client on         */
const std::uint16_t MULTI_CAST_PORT     = 7002;                     /* Multi cast port                                                  */
const std::string MULTI_CAST_ADDR   = "239.255.0.1";                /* Multi cast address                                               */
}; /* namespace raptor_networking */

#endif /* #ifndef __NETWORKING_CONSTANTS_H__ */
