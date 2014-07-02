#ifdef STAND_ALONE
#define BOOST_TEST_MODULE message_delivery test
#endif /* #ifdef STAND_ALONE */


/* Standard headers */
#include <vector>

/* Boost headers */
#include "boost/test/unit_test.hpp"

/* Networking headers */
#include "message_delivery.h"

/* Test headers */
#include "message_generator.h"
#include "mock_data_receiver.h"
#include "mock_stack_component.h"
#include "stack_component_fixture.h"


using namespace raptor_networking;

/* Test data */
struct message_delivery_fixture : public stack_component_fixture<mock_stack_component>
{
    message_delivery_fixture()
        : hellomars("Hello Mars"),
          helloworld_mg("Hello World"),
          hellomars_mg(hellomars)
        {
            link_uut(new mock_stack_component(bottom_node));
        };

    const std::string           hellomars;
    const message_generator     helloworld_mg;
    const message_generator     hellomars_mg;
    boost::system::error_code   ec;
};

BOOST_FIXTURE_TEST_SUITE( message_delivery_tests, message_delivery_fixture )

/* Test constructors */
BOOST_AUTO_TEST_CASE( ctor_test )
{
    BOOST_CHECK(delivery_node->awaited_promises() == 0);
}


/* Clone test */
BOOST_AUTO_TEST_CASE( clone_test )
{
    auto cloned = delivery_node->clean_clone();
    auto *md = dynamic_cast<message_delivery*>(cloned.last());
    BOOST_CHECK(md != nullptr);
    BOOST_CHECK(md->awaited_promises() == 0);

    md->build_up_links();
    delete cloned.bottom();
}


/* Test send */
BOOST_AUTO_TEST_CASE( send_test )
{
    stack_accessor acc;
    delivery_node->send(acc, helloworld_mg.send_data(), helloworld_mg.header(1), ec);

    /* Check for an error */
    BOOST_CHECK(!ec);

    /* Check message delivery state */
    BOOST_CHECK(delivery_node->awaited_promises() == 0);
    
    /* Check number of messages */
    BOOST_CHECK(top_node->data_sent() == 1);
    BOOST_CHECK(top_node->headers_sent() == 1);
    BOOST_CHECK(data_recv->data_received() == 0);

    /* Check message content */
    BOOST_CHECK(*helloworld_mg.header(1) == *top_node->sent_header());
    BOOST_CHECK(helloworld_mg.check_data(top_node->sent_data()));
}


/* Test send with promise */
BOOST_AUTO_TEST_CASE( send_with_promise_test )
{
    stack_accessor acc;
    std::future<std::istream *const> resp;
    delivery_node->send(acc, helloworld_mg.send_data(), helloworld_mg.header(1), &resp, ec);

    /* Check for an error */
    BOOST_CHECK(!ec);
    
    /* Check message delivery state */
    BOOST_CHECK(delivery_node->awaited_promises() == 1);

    /* Check number of messages */
    BOOST_CHECK(top_node->data_sent() == 1);
    BOOST_CHECK(top_node->headers_sent() == 1);
    BOOST_CHECK(data_recv->data_received() == 0);

    /* Check message content */
    BOOST_CHECK(*helloworld_mg.header(1) == *top_node->sent_header());
    BOOST_CHECK(helloworld_mg.check_data(top_node->sent_data()));
}


/* Test send with promise and error */
BOOST_AUTO_TEST_CASE( send_with_promise_and_error_test )
{
    stack_accessor acc;
    std::future<std::istream *const> resp;
    ec = make_error_code(boost::system::errc::errc_t::bad_message);
    delivery_node->send(acc, helloworld_mg.send_data(), helloworld_mg.header(1), &resp, ec);

    /* Check for an error */
    BOOST_CHECK(ec);
    
    /* Check message delivery state */
    BOOST_CHECK(delivery_node->awaited_promises() == 0);

    /* Check number of messages */
    BOOST_CHECK(top_node->data_sent() == 1);
    BOOST_CHECK(top_node->headers_sent() == 1);
    BOOST_CHECK(data_recv->data_received() == 0);

    /* Check message content */
    BOOST_CHECK(*helloworld_mg.header(1) == *top_node->sent_header());
    BOOST_CHECK(helloworld_mg.check_data(top_node->sent_data()));
}


/* Test receive */
BOOST_AUTO_TEST_CASE( recv_test )
{
    stack_accessor acc;
    delivery_node->received(acc, helloworld_mg.receive_data(), helloworld_mg.header(1));

    /*  Check message delivery state */
    BOOST_CHECK(delivery_node->awaited_promises() == 0);

    /* Check number of messages */
    BOOST_CHECK(top_node->data_sent() == 0);
    BOOST_CHECK(top_node->headers_sent() == 0);
    BOOST_CHECK(data_recv->data_received() == 1);

    /* Check message content */
    BOOST_CHECK(helloworld_mg.check_data(data_recv->received_data()));
}


/* Test receive with response */
BOOST_AUTO_TEST_CASE( recv_with_response_test )
{
    stack_accessor acc;
    ctrl->lock_stack(acc, boost::uuids::nil_uuid());
    data_recv->set_auto_response(new std::string(hellomars));
    delivery_node->received(acc, helloworld_mg.receive_data(), helloworld_mg.header(1));

    /*  Check message delivery state */
    BOOST_CHECK(delivery_node->awaited_promises() == 0);

    /* Check number of messages */
    BOOST_CHECK(top_node->data_sent() == 1);
    BOOST_CHECK(top_node->headers_sent() == 1);
    BOOST_CHECK(data_recv->data_received() == 1);

    /* Check message content */
    msg_header sent_header(helloworld_mg.from_address(), helloworld_mg.group_address(), 0, hellomars.size(), 1, true);
    BOOST_CHECK(sent_header == *top_node->sent_header());
    BOOST_CHECK(hellomars_mg.check_data(top_node->sent_data()));
    BOOST_CHECK(helloworld_mg.check_data(data_recv->received_data()));
}


/* Test receive with promise */
BOOST_AUTO_TEST_CASE( recv_with_promise_test )
{
    stack_accessor acc;
    std::future<std::istream *const> resp;
    delivery_node->send(acc, helloworld_mg.send_data(), helloworld_mg.header(1), &resp, ec);

    /* Check for an error */
    BOOST_CHECK(!ec);
    
    /* Check message delivery state */
    BOOST_CHECK(delivery_node->awaited_promises() == 1);

    /* Check number of messages */
    BOOST_CHECK(top_node->data_sent() == 1);
    BOOST_CHECK(top_node->headers_sent() == 1);
    BOOST_CHECK(data_recv->data_received() == 0);

    /* Check message content */
    BOOST_CHECK(*helloworld_mg.header(1) == *top_node->sent_header());
    BOOST_CHECK(helloworld_mg.check_data(top_node->sent_data()));

    std::shared_ptr<msg_header> resp_header(new msg_header(hellomars_mg.from_address(), hellomars_mg.group_address(), 6, hellomars.size(), 1, true));
    delivery_node->received(acc, hellomars_mg.receive_data(), resp_header);

    /* Check message delivery state */
    BOOST_CHECK(delivery_node->awaited_promises() == 0);

    /* Check number of messages */
    BOOST_CHECK(top_node->data_sent() == 0);
    BOOST_CHECK(top_node->headers_sent() == 0);
    BOOST_CHECK(data_recv->data_received() == 1);
    BOOST_CHECK(resp.wait_for(std::chrono::seconds(0)) == future_status::ready);

    /* Check message content */
    BOOST_CHECK(hellomars_mg.check_data(std::unique_ptr<std::istream>(resp.get())));
    BOOST_CHECK(hellomars_mg.check_data(data_recv->received_data()));
}

BOOST_AUTO_TEST_SUITE_END()
