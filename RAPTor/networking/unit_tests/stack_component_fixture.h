#pragma once

/* Standard headers */

/* Boost headers */
#include "boost/noncopyable.hpp"

/* Networking headers */
#include "stack_controller.h"

/* Test headers */
#include "mock_data_receiver.h"
#include "mock_stack_component.h"

namespace raptor_networking
{
namespace test
{
/* Common base class for stack component fixtures */
template<class Uut>
struct stack_component_fixture : private boost::noncopyable
{
    stack_component_fixture()
        : ioservice(),
          ctrl(new stack_controller(ioservice)),
          data_recv(new mock_data_receiver()),
          conn_node(new mock_stack_component(true)),
          bottom_node(new mock_stack_component(conn_node)),
          mid_node(nullptr),
          top_node(new mock_stack_component(mid_node)),
          delivery_node(new message_delivery(top_node, data_recv))
    {  };

    virtual ~stack_component_fixture()
    {
    	/* Stop the io service */
    	ioservice.stop();

        /* Clean the test stack */
        delete ctrl;

        /* Clean the connection */
        delete conn_node;
    }

    stack_component_fixture& link_uut(Uut *const mid)
    {
    	/* link the node into the stack */
        mid_node = mid;
        top_node->link_down_node(mid_node);
        delivery_node->build_up_links();

        /* Set the stack into the stack controller */
        ctrl->clean_stack(delivery_node, bottom_node);

        return *this;
    }

    boost::asio::io_service     ioservice;
    stack_controller *const 	ctrl;
    mock_data_receiver *const   data_recv;
    mock_stack_component *const conn_node;
    mock_stack_component *const bottom_node;
    Uut *                       mid_node;
    mock_stack_component *const top_node;
    message_delivery *const     delivery_node;
    boost::system::error_code   ec;
};
}; /* namespace test */
}; /* namespace raptor_networking */
