/* Networking headers */
#include "tcp_connection.h"
#include "udp_connection.h"
#include "rate_limiter.h"
#include "retransmission.h"
#include "fragmenter.h"
#include "message_delivery.h"
#include "protocol_stack.h"


namespace raptor_networking
{
/* CTOR for unicast */
protocol_stack::protocol_stack(boost::asio::io_service &io_service, data_receiver *const receiver, 
    group *const grp, const std::uint16_t send_port, const std::uint16_t recv_port, const bool tcp)
    : _ctrl(io_service), _io_service(io_service)
{
    /* Connect the protocol stack */
    if (tcp)
    {
        std::cout << "building tcp connection" << std::endl;
        _conn = new tcp_connection<stack_component>(&_ctrl, grp, send_port, recv_port);

        grp->link_down_node(_conn);

        auto *del = new message_delivery(grp, receiver);
        del->build_up_links();
        _ctrl.clean_stack(del, grp);
    }
    else
    {
        std::cout << "building udp connection" << std::endl;
        _conn = new udp_connection<stack_component>(&_ctrl, grp, send_port, recv_port);

#ifdef __DEBUG__
        auto *limiter = new rate_limiter<stack_component, stack_component>(_conn, _io_service, MAX_PACKET_SIZE * 0.25f);
        auto *stack_bottom = limiter;
#else
        auto *limiter = _conn;
        auto *retran = new ack_retransmission<stack_component, stack_component>(limiter, _io_service, &_ctrl, 1000);
        auto *stack_bottom = retran;
#endif

        auto *frag = new fixed_size_fragmenter<stack_component, stack_component>(limiter, MAX_PACKET_SIZE);

        grp->link_down_node(frag);

        auto *del = new message_delivery(grp, receiver);
        del->build_up_links();
        _ctrl.clean_stack(del, stack_bottom);
    }
}


protocol_stack::protocol_stack(boost::asio::io_service &io_service, data_receiver *const receiver, 
    const std::string &me_addr, const std::uint16_t send_port, const std::uint16_t recv_port, 
    const std::uint16_t port_offset, const bool tcp)
    : protocol_stack(io_service, receiver, new group(&_ctrl, me_addr, port_offset), send_port, recv_port, tcp)
    {  };


/* CTOR for multi cast */
protocol_stack::protocol_stack(boost::asio::io_service &io_service, data_receiver *const receiver, 
    group *const grp, const boost::asio::ip::address &addr, const boost::asio::ip::address &multi_addr, const std::uint16_t port)
    : _ctrl(io_service), _io_service(io_service)
{
    /* Connect the protocol stack */
    std::cout << "building multi cast connection" << std::endl;
    _conn = new udp_connection<stack_component>(&_ctrl, grp, addr, multi_addr, port);

#ifdef __DEBUG__
    auto *limiter = new rate_limiter<stack_component, stack_component>(_conn, _io_service, MAX_PACKET_SIZE * 0.25f);
    auto *stack_bottom = limiter;
#else
    auto *limiter = _conn;
    auto *retran = new ack_retransmission<stack_component, stack_component>(limiter, _io_service, &_ctrl, 1000);
    auto *stack_bottom = retran;
#endif

    auto *frag = new fixed_size_fragmenter<stack_component, stack_component>(limiter, MAX_PACKET_SIZE);

    grp->link_down_node(frag);

    auto *del = new message_delivery(grp, receiver);
    del->build_up_links();
    _ctrl.clean_stack(del, stack_bottom);
}

protocol_stack::protocol_stack(boost::asio::io_service &io_service, data_receiver *const receiver, 
    const std::string &me_addr, const boost::asio::ip::address &addr, const boost::asio::ip::address &multi_addr, 
    const std::uint16_t port, const std::uint16_t port_offset)
    : protocol_stack(io_service, receiver, new group(&_ctrl, multi_addr.to_string(), me_addr, port, port_offset), addr, multi_addr, port)
    {  };

}; /* namespace raptor_networking */
