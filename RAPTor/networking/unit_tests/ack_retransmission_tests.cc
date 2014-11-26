#ifdef STAND_ALONE
#define BOOST_TEST_MODULE ack_retransmission test
#endif /* #ifdef STAND_ALONE */


/* Boost headers */
#include "boost/test/unit_test.hpp"

/* Networking headers */
#include "retransmission.h"

/* Test headers */
#include "stack_component_fixture.h"
#include "message_generator.h"
#include "mock_stack_component.h"


using namespace raptor_networking;

/* Test data */
typedef ack_retransmission<stack_component, stack_component> uut;
struct ack_retransmission_fixture : public stack_component_fixture<uut>
{
    ack_retransmission_fixture()
        : mg0("Hello World"),
          mg1("Goodbye World"), 
          mg2("Hello Mars"),
          resend_ms(100)
    {
        link_uut(new uut(bottom_node, ioservice, ctrl, resend_ms));
    };

    const message_generator mg0;
    const message_generator mg1;
    const message_generator mg2;
    const unsigned int      resend_ms;
};

BOOST_FIXTURE_TEST_SUITE( ack_retransmission_tests, ack_retransmission_fixture );

/* Test constructors */
BOOST_AUTO_TEST_CASE( ctor_test )
{
    BOOST_CHECK(mid_node->pending_resends() == 0);
    BOOST_CHECK(mid_node->resend_timeout() == resend_ms);
    BOOST_CHECK(mid_node->current_resend_id() == 0);
}


/* Clone test */
BOOST_AUTO_TEST_CASE( clone_test )
{
    auto cloned = mid_node->clean_clone();
    auto *retran = dynamic_cast<uut*>(cloned.last());
    BOOST_CHECK(retran->pending_resends() == 0);
    BOOST_CHECK(retran->resend_timeout() == resend_ms);
    BOOST_CHECK(retran->current_resend_id() == 0);

    retran->build_up_links(new mock_stack_component());
    delete cloned.bottom();
}


/* Test send */
BOOST_AUTO_TEST_CASE( send_test )
{
    stack_accessor acc;
    mid_node->send(acc, mg0.send_data(), mg0.header(1), ec);

    /* Check for an error */
    BOOST_CHECK(!ec);

    /* Check retransmitter state */
    BOOST_CHECK(mid_node->pending_resends() == 1);
    BOOST_CHECK(mid_node->resend_timeout() == resend_ms);
    BOOST_CHECK(mid_node->current_resend_id() == 1);
    
    /* Check number of messages */
    BOOST_CHECK(bottom_node->data_sent() == 1);
    BOOST_CHECK(bottom_node->headers_sent() == 1);
    BOOST_CHECK(top_node->data_received() == 0);
    BOOST_CHECK(top_node->headers_received() == 0);

    /* Check message content */
    BOOST_CHECK(*mg0.header(1) == *bottom_node->sent_header());
    BOOST_CHECK(mg0.check_data(bottom_node->sent_data()));

    mid_node->send(acc, mg1.send_data(), mg1.header(3), ec);

    /* Check for an error */
    BOOST_CHECK(!ec);

    /* Check retransmitter state */
    BOOST_CHECK(mid_node->pending_resends() == 2);
    BOOST_CHECK(mid_node->resend_timeout() == resend_ms);
    BOOST_CHECK(mid_node->current_resend_id() == 2);
    
    /* Check number of messages */
    BOOST_CHECK(bottom_node->data_sent() == 1);
    BOOST_CHECK(bottom_node->headers_sent() == 1);
    BOOST_CHECK(top_node->data_received() == 0);
    BOOST_CHECK(top_node->headers_received() == 0);

    /* Check message content */
    const std::shared_ptr<msg_header> &recv_header = bottom_node->sent_header();
    BOOST_CHECK(recv_header->resend_id() == 1);
    recv_header->resend_id(0);
    BOOST_CHECK(*mg1.header(3) == *recv_header);
    BOOST_CHECK(mg1.check_data(bottom_node->sent_data()));
}


/* Test timeout and resend */
BOOST_AUTO_TEST_CASE( resend_test )
{
    stack_accessor acc;
    mid_node->send(acc, mg0.send_data(), mg0.header(1), ec);

    /* Check for an error */
    BOOST_CHECK(!ec);

    /* Check retransmitter state */
    BOOST_CHECK(mid_node->pending_resends() == 1);
    BOOST_CHECK(mid_node->resend_timeout() == resend_ms);
    BOOST_CHECK(mid_node->current_resend_id() == 1);
    
    /* Check number of messages */
    BOOST_CHECK(bottom_node->data_sent() == 1);
    BOOST_CHECK(bottom_node->headers_sent() == 1);
    BOOST_CHECK(top_node->data_received() == 0);
    BOOST_CHECK(top_node->headers_received() == 0);

    /* Check message content */
    BOOST_CHECK(*mg0.header(1) == *bottom_node->sent_header());
    BOOST_CHECK(mg0.check_data(bottom_node->sent_data()));

    /* Wait for timeout */
    std::thread thread(std::bind(
                    (std::size_t(boost::asio::io_service::*)())&boost::asio::io_service::run_one, &ioservice));
    thread.detach();
    std::this_thread::sleep_for(std::chrono::duration<unsigned int,std::milli>(resend_ms * 2));
    
    /* Check retransmitter state */
    BOOST_CHECK(mid_node->pending_resends() == 1);
    BOOST_CHECK(mid_node->resend_timeout() == resend_ms);
    BOOST_CHECK(mid_node->current_resend_id() == 1);
    
    /* Check number of messages */
    BOOST_CHECK(bottom_node->data_sent() == 1);
    BOOST_CHECK(bottom_node->headers_sent() == 1);
    BOOST_CHECK(top_node->data_received() == 0);
    BOOST_CHECK(top_node->headers_received() == 0);

    /* Check message content */
    BOOST_CHECK(*mg0.header(1) == *bottom_node->sent_header());
    BOOST_CHECK(mg0.check_data(bottom_node->sent_data()));
}


/* Test timeout and resend */
BOOST_AUTO_TEST_CASE( empty_resend_test )
{
    /* Wait for timeout */
    std::thread thread(std::bind(
                    (std::size_t(boost::asio::io_service::*)())&boost::asio::io_service::run_one, &ioservice));
    thread.detach();
    std::this_thread::sleep_for(std::chrono::duration<unsigned int,std::milli>(resend_ms * 2));
    
    /* Check retransmitter state */
    BOOST_CHECK(mid_node->pending_resends() == 0);
    BOOST_CHECK(mid_node->resend_timeout() == resend_ms);
    BOOST_CHECK(mid_node->current_resend_id() == 0);
    
    /* Check number of messages */
    BOOST_CHECK(bottom_node->data_sent() == 0);
    BOOST_CHECK(bottom_node->headers_sent() == 0);
    BOOST_CHECK(top_node->data_received() == 0);
    BOOST_CHECK(top_node->headers_received() == 0);
}


/* Test receive */
BOOST_AUTO_TEST_CASE( recv_test )
{
    stack_accessor acc;
    mid_node->received(acc, mg0.receive_data(), mg0.header(1));
    ioservice.run_one();

    /* Check retransmitter state */
    BOOST_CHECK(mid_node->pending_resends() == 0);
    BOOST_CHECK(mid_node->resend_timeout() == resend_ms);
    BOOST_CHECK(mid_node->current_resend_id() == 0);

    /* Check number of messages */
    BOOST_CHECK(bottom_node->data_sent() == 1);
    BOOST_CHECK(bottom_node->headers_sent() == 1);
    BOOST_CHECK(top_node->data_received() == 1);
    BOOST_CHECK(top_node->headers_received() == 1);

    /* Check message content */
    BOOST_CHECK(*mg0.header(1) == *top_node->received_header());
    BOOST_CHECK(mg0.check_data(top_node->received_data()));

    msg_header ack_header(mg0.from_address(), mg0.group_address(), 0, MSG_ACKNOWLEDGE.size(), 1, true);
    BOOST_CHECK(ack_header == *bottom_node->sent_header());
    BOOST_CHECK(std::equal(MSG_ACKNOWLEDGE.begin(), MSG_ACKNOWLEDGE.end(), bottom_node->sent_data()->begin()));
}


/* Test acknowledge recv */
BOOST_AUTO_TEST_CASE( recv_ack_test )
{
    stack_accessor acc;
    mid_node->send(acc, mg0.send_data(), mg0.header(1), ec);

    /* Check for an error */
    BOOST_CHECK(!ec);

    /* Check retransmitter state */
    BOOST_CHECK(mid_node->pending_resends() == 1);
    BOOST_CHECK(mid_node->resend_timeout() == resend_ms);
    BOOST_CHECK(mid_node->current_resend_id() == 1);
    
    /* Check number of messages */
    BOOST_CHECK(bottom_node->data_sent() == 1);
    BOOST_CHECK(bottom_node->headers_sent() == 1);
    BOOST_CHECK(top_node->data_received() == 0);
    BOOST_CHECK(top_node->headers_received() == 0);

    /* Check message content */
    BOOST_CHECK(*mg0.header(1) == *bottom_node->sent_header());
    BOOST_CHECK(mg0.check_data(bottom_node->sent_data()));

    std::unique_ptr<char []> ack_data(new char [MSG_ACKNOWLEDGE.size()]);
    std::copy(MSG_ACKNOWLEDGE.begin(), MSG_ACKNOWLEDGE.end(), &ack_data[0]);
    mid_node->received(acc, std::shared_ptr<msg_data>(new msg_data(ack_data.release(), MSG_ACKNOWLEDGE.size())), mg0.header(1));
    ioservice.run_one();

    /* Check retransmitter state */
    BOOST_CHECK(mid_node->pending_resends() == 0);
    BOOST_CHECK(mid_node->resend_timeout() == resend_ms);
    BOOST_CHECK(mid_node->current_resend_id() == 1);

    /* Check number of messages */
    BOOST_CHECK(bottom_node->data_sent() == 0);
    BOOST_CHECK(bottom_node->headers_sent() == 0);
    BOOST_CHECK(top_node->data_received() == 0);
    BOOST_CHECK(top_node->headers_received() == 0);
}

BOOST_AUTO_TEST_SUITE_END()
