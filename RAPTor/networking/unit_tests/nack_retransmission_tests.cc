#ifdef STAND_ALONE
#define BOOST_TEST_MODULE nack_retransmission test
#endif /* #ifdef STAND_ALONE */


/* Boost headers */
#include "boost/test/unit_test.hpp"

/* Networking headers */
#include "retransmission.h"

/* Test headers */
#include "stack_component_fixture.h"
#include "message_generator.h"
#include "mock_stack_component.h"


namespace raptor_networking
{
namespace test
{
/* Test data */
typedef nack_retransmission<stack_component, stack_component> uut;
struct nack_retransmission_fixture : public stack_component_fixture<uut>
{
    nack_retransmission_fixture()
        : mg0("Hello World"),
          mg1("Goodbye World"), 
          mg2("Hello Mars")
    {
        link_uut(new uut(bottom_node, ctrl));
    };

    const message_generator mg0;
    const message_generator mg1;
    const message_generator mg2;
};

BOOST_FIXTURE_TEST_SUITE( nack_retransmission_tests, nack_retransmission_fixture );

/* Test constructors */
BOOST_AUTO_TEST_CASE( ctor_test )
{
    BOOST_CHECK(mid_node->seen() == -1);
    BOOST_CHECK(mid_node->pending_resends() == 0);
    BOOST_CHECK(mid_node->current_resend_id() == 0);
}


/* Clone test */
BOOST_AUTO_TEST_CASE( clone_test )
{
    auto cloned = mid_node->clean_clone();
    auto *seq = dynamic_cast<uut*>(cloned.last());
    BOOST_CHECK(seq != nullptr);
    BOOST_CHECK(seq->seen() == -1);
    BOOST_CHECK(seq->pending_resends() == 0);
    BOOST_CHECK(seq->current_resend_id() == 0);

    seq->build_up_links(new mock_stack_component());
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
    BOOST_CHECK(mid_node->seen() == -1);
    BOOST_CHECK(mid_node->pending_resends() == 1);
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
    BOOST_CHECK(mid_node->seen() == -1);
    BOOST_CHECK(mid_node->pending_resends() == 2);
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


/* Test receive */
BOOST_AUTO_TEST_CASE( recv_test )
{
    stack_accessor acc;
    mid_node->received(acc, mg0.receive_data(), mg0.header(1, 0));

    /* Check retransmitter state */
    BOOST_CHECK(mid_node->seen() == 0);
    BOOST_CHECK(mid_node->pending_resends() == 0);
    BOOST_CHECK(mid_node->current_resend_id() == 0);

    /* Check number of messages */
    BOOST_CHECK(bottom_node->data_sent() == 0);
    BOOST_CHECK(bottom_node->headers_sent() == 0);
    BOOST_CHECK(top_node->data_received() == 1);
    BOOST_CHECK(top_node->headers_received() == 1);

    /* Check message content */
    BOOST_CHECK(*mg0.header(1, 0) == *top_node->received_header());
    BOOST_CHECK(mg0.check_data(top_node->received_data()));

    mid_node->received(acc, mg0.receive_data(), mg0.header(1, 1));

    /* Check retransmitter state */
    BOOST_CHECK(mid_node->seen() == 1);
    BOOST_CHECK(mid_node->pending_resends() == 0);
    BOOST_CHECK(mid_node->current_resend_id() == 0);

    /* Check number of messages */
    BOOST_CHECK(bottom_node->data_sent() == 0);
    BOOST_CHECK(bottom_node->headers_sent() == 0);
    BOOST_CHECK(top_node->data_received() == 1);
    BOOST_CHECK(top_node->headers_received() == 1);

    /* Check message content */
    BOOST_CHECK(*mg0.header(1, 1) == *top_node->received_header());
    BOOST_CHECK(mg0.check_data(top_node->received_data()));
}


/* Test receive out of order */
BOOST_AUTO_TEST_CASE( recv_out_of_order_test )
{
    stack_accessor acc;
    mid_node->received(acc, mg0.receive_data(), mg0.header(1, 1));

    /* Check retransmitter state */
    BOOST_CHECK(mid_node->seen() == -1);
    BOOST_CHECK(mid_node->pending_resends() == 0);
    BOOST_CHECK(mid_node->current_resend_id() == 0);

    /* Check number of messages */
    BOOST_CHECK(bottom_node->data_sent() == 0);
    BOOST_CHECK(bottom_node->headers_sent() == 0);
    BOOST_CHECK(top_node->data_received() == 1);
    BOOST_CHECK(top_node->headers_received() == 1);

    /* Check message content */
    BOOST_CHECK(*mg0.header(1, 1) == *top_node->received_header());
    BOOST_CHECK(mg0.check_data(top_node->received_data()));

    mid_node->received(acc, mg0.receive_data(), mg0.header(1, 2));

    /* Check retransmitter state */
    BOOST_CHECK(mid_node->seen() == -1);
    BOOST_CHECK(mid_node->pending_resends() == 0);
    BOOST_CHECK(mid_node->current_resend_id() == 0);

    /* Check number of messages */
    BOOST_CHECK(bottom_node->data_sent() == 0);
    BOOST_CHECK(bottom_node->headers_sent() == 0);
    BOOST_CHECK(top_node->data_received() == 1);
    BOOST_CHECK(top_node->headers_received() == 1);

    /* Check message content */
    BOOST_CHECK(*mg0.header(1, 2) == *top_node->received_header());
    BOOST_CHECK(mg0.check_data(top_node->received_data()));

    mid_node->received(acc, mg0.receive_data(), mg0.header(1, 0));

    /* Check retransmitter state */
    BOOST_CHECK(mid_node->seen() == 2);
    BOOST_CHECK(mid_node->pending_resends() == 0);
    BOOST_CHECK(mid_node->current_resend_id() == 0);

    /* Check number of messages */
    BOOST_CHECK(bottom_node->data_sent() == 0);
    BOOST_CHECK(bottom_node->headers_sent() == 0);
    BOOST_CHECK(top_node->data_received() == 1);
    BOOST_CHECK(top_node->headers_received() == 1);

    /* Check message content */
    BOOST_CHECK(*mg0.header(1, 0) == *top_node->received_header());
    BOOST_CHECK(mg0.check_data(top_node->received_data()));
}


/* Test request resend */
BOOST_AUTO_TEST_CASE( request_resend_test )
{
    stack_accessor acc;
    mid_node->received(acc, mg0.receive_data(), mg0.header(1, 1));
    ioservice.run_one();

    /* Check retransmitter state */
    BOOST_CHECK(mid_node->seen() == -1);
    BOOST_CHECK(mid_node->pending_resends() == 0);
    BOOST_CHECK(mid_node->current_resend_id() == 0);

    /* Check number of messages */
    BOOST_CHECK(bottom_node->data_sent() == 1);
    BOOST_CHECK(bottom_node->headers_sent() == 1);
    BOOST_CHECK(top_node->data_received() == 1);
    BOOST_CHECK(top_node->headers_received() == 1);

    /* Check message content */
    BOOST_CHECK(*mg0.header(1, 1) == *top_node->received_header());
    BOOST_CHECK(mg0.check_data(top_node->received_data()));

    const msg_header nack_header(mg0.from_address(), mg0.group_address(), 0, MSG_RESEND.size() + sizeof(std::uint32_t), 1, true);
    BOOST_CHECK(nack_header == *bottom_node->sent_header());

    const std::shared_ptr<std::vector<char>> &nack_data = bottom_node->sent_data();
    BOOST_CHECK(nack_data->size() == MSG_RESEND.size() + sizeof(std::uint32_t));
    BOOST_CHECK(std::equal(MSG_RESEND.begin(), MSG_RESEND.end(), nack_data->begin()));
    const std::uint32_t resend_id = from_big_endian_byte_array<char, std::uint32_t>(&nack_data->data()[MSG_RESEND.size()]);
    BOOST_CHECK(resend_id == 0);

    mid_node->received(acc, mg0.receive_data(), mg0.header(1, 0));

    /* Check retransmitter state */
    BOOST_CHECK(mid_node->seen() == 1);
    BOOST_CHECK(mid_node->pending_resends() == 0);
    BOOST_CHECK(mid_node->current_resend_id() == 0);

    /* Check number of messages */
    BOOST_CHECK(bottom_node->data_sent() == 0);
    BOOST_CHECK(bottom_node->headers_sent() == 0);
    BOOST_CHECK(top_node->data_received() == 1);
    BOOST_CHECK(top_node->headers_received() == 1);

    /* Check message content */
    BOOST_CHECK(*mg0.header(1, 0) == *top_node->received_header());
    BOOST_CHECK(mg0.check_data(top_node->received_data()));
}


/* Test receive stable message */
BOOST_AUTO_TEST_CASE( stable_test )
{
    stack_accessor acc;
    mid_node->send(acc, mg0.send_data(), mg0.header(1), ec);

    /* Check for an error */
    BOOST_CHECK(!ec);

    /* Check retransmitter state */
    BOOST_CHECK(mid_node->seen() == -1);
    BOOST_CHECK(mid_node->pending_resends() == 1);
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
    BOOST_CHECK(mid_node->seen() == -1);
    BOOST_CHECK(mid_node->pending_resends() == 2);
    BOOST_CHECK(mid_node->current_resend_id() == 2);

    /* Check number of messages */
    BOOST_CHECK(bottom_node->data_sent() == 1);
    BOOST_CHECK(bottom_node->headers_sent() == 1);
    BOOST_CHECK(top_node->data_received() == 0);
    BOOST_CHECK(top_node->headers_received() == 0);

    /* Check message content */
    const std::shared_ptr<msg_header> &recv_header0 = bottom_node->sent_header();
    BOOST_CHECK(recv_header0->resend_id() == 1);
    recv_header0->resend_id(0);
    BOOST_CHECK(*mg1.header(3) == *recv_header0);
    BOOST_CHECK(mg1.check_data(bottom_node->sent_data()));

    mid_node->send(acc, mg1.send_data(), mg1.header(5), ec);

    /* Check for an error */
    BOOST_CHECK(!ec);

    /* Check retransmitter state */
    BOOST_CHECK(mid_node->seen() == -1);
    BOOST_CHECK(mid_node->pending_resends() == 3);
    BOOST_CHECK(mid_node->current_resend_id() == 3);

    /* Check number of messages */
    BOOST_CHECK(bottom_node->data_sent() == 1);
    BOOST_CHECK(bottom_node->headers_sent() == 1);
    BOOST_CHECK(top_node->data_received() == 0);
    BOOST_CHECK(top_node->headers_received() == 0);

    /* Check message content */
    const std::shared_ptr<msg_header> &recv_header1 = bottom_node->sent_header();
    BOOST_CHECK(recv_header1->resend_id() == 2);
    recv_header1->resend_id(0);
    BOOST_CHECK(*mg1.header(5) == *recv_header1);
    BOOST_CHECK(mg1.check_data(bottom_node->sent_data()));

    mid_node->send(acc, mg1.send_data(), mg1.header(7), ec);

    /* Check for an error */
    BOOST_CHECK(!ec);

    /* Check retransmitter state */
    BOOST_CHECK(mid_node->seen() == -1);
    BOOST_CHECK(mid_node->pending_resends() == 4);
    BOOST_CHECK(mid_node->current_resend_id() == 4);

    /* Check number of messages */
    BOOST_CHECK(bottom_node->data_sent() == 1);
    BOOST_CHECK(bottom_node->headers_sent() == 1);
    BOOST_CHECK(top_node->data_received() == 0);
    BOOST_CHECK(top_node->headers_received() == 0);

    /* Check message content */
    const std::shared_ptr<msg_header> &recv_header2 = bottom_node->sent_header();
    BOOST_CHECK(recv_header2->resend_id() == 3);
    recv_header2->resend_id(0);
    BOOST_CHECK(*mg1.header(7) == *recv_header2);
    BOOST_CHECK(mg1.check_data(bottom_node->sent_data()));

    /* Prepare stable message */
    const unsigned int stable_size = MSG_STABLE.size() + sizeof(std::uint32_t);
    std::unique_ptr<char []> stable_data(new char[stable_size]);
    std::copy(MSG_STABLE.begin(), MSG_STABLE.end(), &stable_data.get()[0]);
    to_big_endian_byte_array<std::uint32_t, char>(1, &stable_data.get()[MSG_STABLE.size()]);
    std::shared_ptr<msg_data> stable(new msg_data(stable_data.release(), stable_size));

    mid_node->received(acc, stable, mg0.header(1));

    /* Check retransmitter state */
    BOOST_CHECK(mid_node->seen() == -1);
    BOOST_CHECK_MESSAGE(mid_node->pending_resends() == 2, mid_node->pending_resends());
    BOOST_CHECK(mid_node->current_resend_id() == 4);

    /* Check number of messages */
    BOOST_CHECK(bottom_node->data_sent() == 0);
    BOOST_CHECK(bottom_node->headers_sent() == 0);
    BOOST_CHECK(top_node->data_received() == 0);
    BOOST_CHECK(top_node->headers_received() == 0);
}


/* Test resend */
BOOST_AUTO_TEST_CASE( resend_test )
{
    stack_accessor acc;
    mid_node->send(acc, mg0.send_data(), mg0.header(1), ec);

    /* Check for an error */
    BOOST_CHECK(!ec);

    /* Check retransmitter state */
    BOOST_CHECK(mid_node->seen() == -1);
    BOOST_CHECK(mid_node->pending_resends() == 1);
    BOOST_CHECK(mid_node->current_resend_id() == 1);
    
    /* Check number of messages */
    BOOST_CHECK(bottom_node->data_sent() == 1);
    BOOST_CHECK(bottom_node->headers_sent() == 1);
    BOOST_CHECK(top_node->data_received() == 0);
    BOOST_CHECK(top_node->headers_received() == 0);

    /* Check message content */
    BOOST_CHECK(*mg0.header(1) == *bottom_node->sent_header());
    BOOST_CHECK(mg0.check_data(bottom_node->sent_data()));

    mid_node->received(acc, mg0.receive_data(), mg0.header(1, 2));
    ioservice.run_one();
    ioservice.reset();

    /* Check retransmitter state */
    BOOST_CHECK(mid_node->seen() == -1);
    BOOST_CHECK(mid_node->pending_resends() == 1);
    BOOST_CHECK(mid_node->current_resend_id() == 1);

    /* Check number of messages */
    BOOST_CHECK(bottom_node->data_sent() == 1);
    BOOST_CHECK(bottom_node->headers_sent() == 1);
    BOOST_CHECK(top_node->data_received() == 1);
    BOOST_CHECK(top_node->headers_received() == 1);

    /* Check message content */
    BOOST_CHECK(*mg0.header(1, 2) == *top_node->received_header());
    BOOST_CHECK(mg0.check_data(top_node->received_data()));

    /* Return the message for resending */
    bottom_node->loopback_sent_message(acc);
    ioservice.run_one();
    ioservice.reset();

    /* Check retransmitter state */
    BOOST_CHECK(mid_node->seen() == -1);
    BOOST_CHECK(mid_node->pending_resends() == 1);
    BOOST_CHECK(mid_node->current_resend_id() == 1);

    /* Check number of messages */
    BOOST_CHECK_MESSAGE(bottom_node->data_sent() == 1, bottom_node->data_sent());
    BOOST_CHECK(bottom_node->headers_sent() == 1);
    BOOST_CHECK(top_node->data_received() == 0);
    BOOST_CHECK(top_node->headers_received() == 0);

    /* Check message content */
    BOOST_CHECK(*mg0.header(1) == *bottom_node->sent_header());
    BOOST_CHECK(mg0.check_data(bottom_node->sent_data()));
}

BOOST_AUTO_TEST_SUITE_END()
}; /* namespace test */
}; /* namespace raptor_networking */
