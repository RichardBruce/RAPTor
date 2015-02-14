#ifdef STAND_ALONE
#define BOOST_TEST_MODULE reorder_sequencer test
#endif /* #ifdef STAND_ALONE */


/* Boost headers */
#include "boost/test/unit_test.hpp"

/* Networking headers */
#include "sequencer.h"

/* Test headers */
#include "stack_component_fixture.h"
#include "message_generator.h"
#include "mock_stack_component.h"


namespace raptor_networking
{
namespace test
{
/* Test data */
typedef reorder_sequencer<stack_component, stack_component> uut;
struct reorder_sequencer_fixture : public stack_component_fixture<uut>
{
    reorder_sequencer_fixture()
        : mg0("Hello World"),
          mg1("Goodbye World"), 
          mg2("Hello Mars")
    {
        link_uut(new uut(bottom_node));
    };

    const message_generator mg0;
    const message_generator mg1;
    const message_generator mg2;
};

BOOST_FIXTURE_TEST_SUITE( reorder_sequencer_tests, reorder_sequencer_fixture );


/* Test constructors */
BOOST_AUTO_TEST_CASE( ctor_test )
{
    BOOST_CHECK(mid_node->current_sequence() == 0);
    BOOST_CHECK(mid_node->pending_messages() == 0);
}


/* Clone test */
BOOST_AUTO_TEST_CASE( clone_test )
{
    auto cloned = mid_node->clean_clone();
    auto *seq = dynamic_cast<uut*>(cloned.last());
    BOOST_CHECK(seq != nullptr);
    BOOST_CHECK(seq->current_sequence() == 0);
    BOOST_CHECK(seq->pending_messages() == 0);

    seq->build_up_links(new mock_stack_component());
    delete cloned.bottom();
}


/* Test that send data is passed */
BOOST_AUTO_TEST_CASE( send_reorder_test )
{
    stack_accessor acc;
    mid_node->send(acc, mg0.send_data(), mg0.header(0), ec);

    /* Check for error */
    BOOST_CHECK(!ec);

    /* Check sequencer state */
    BOOST_CHECK(mid_node->current_sequence() == 0);
    BOOST_CHECK(mid_node->pending_messages() == 0);

    /* Check number of messages received */
    BOOST_CHECK(bottom_node->data_sent() == 1);
    BOOST_CHECK(bottom_node->headers_sent() == 1);
    BOOST_CHECK(top_node->data_received() == 0);
    BOOST_CHECK(top_node->headers_received() == 0);

    /* Check message content */
    BOOST_CHECK(mg0.check_data(bottom_node->sent_data()));
    BOOST_CHECK(*mg0.header(0) == *bottom_node->sent_header());
}


/* Test that message 0 can be received */
BOOST_AUTO_TEST_CASE( recv_init_reorder_test )
{
    stack_accessor acc;
    mid_node->received(acc, mg0.receive_data(), mg0.header(0));

    /* Check sequencer state */
    BOOST_CHECK(mid_node->current_sequence() == 1);
    BOOST_CHECK(mid_node->pending_messages() == 0);

    /* Check number of messages received */
    BOOST_CHECK(bottom_node->data_sent() == 0);
    BOOST_CHECK(bottom_node->headers_sent() == 0);
    BOOST_CHECK(top_node->data_received() == 1);
    BOOST_CHECK(top_node->headers_received() == 1);

    /* Check message content */
    BOOST_CHECK(mg0.check_data(top_node->received_data()));
    BOOST_CHECK(*mg0.header(0) == *top_node->received_header());
}


/* Test that multiple messages can be received */
BOOST_AUTO_TEST_CASE( recv_multi_reorder_test )
{
    stack_accessor acc;
    mid_node->received(acc, mg0.receive_data(), mg0.header(0));

    /* Check sequencer state */
    BOOST_CHECK(mid_node->current_sequence() == 1);
    BOOST_CHECK(mid_node->pending_messages() == 0);

    /* Check number of messages received */
    BOOST_CHECK(bottom_node->data_sent() == 0);
    BOOST_CHECK(bottom_node->headers_sent() == 0);
    BOOST_CHECK(top_node->data_received() == 1);
    BOOST_CHECK(top_node->headers_received() == 1);

    /* Check message content */
    BOOST_CHECK(mg0.check_data(top_node->received_data()));
    BOOST_CHECK(*mg0.header(0) == *top_node->received_header());

    mid_node->received(acc, mg1.receive_data(), mg1.header(1));

    /* Check sequencer state */
    BOOST_CHECK(mid_node->current_sequence() == 2);
    BOOST_CHECK(mid_node->pending_messages() == 0);

    /* Check number of messages received */
    BOOST_CHECK(bottom_node->data_sent() == 0);
    BOOST_CHECK(bottom_node->headers_sent() == 0);
    BOOST_CHECK(top_node->data_received() == 1);
    BOOST_CHECK(top_node->headers_received() == 1);

    /* Check message content */
    BOOST_CHECK(mg1.check_data(top_node->received_data()));
    BOOST_CHECK(*mg1.header(1) == *top_node->received_header());

    mid_node->received(acc, mg2.receive_data(), mg2.header(2));

    /* Check sequencer state */
    BOOST_CHECK(mid_node->current_sequence() == 3);
    BOOST_CHECK(mid_node->pending_messages() == 0);

    /* Check number of messages received */
    BOOST_CHECK(bottom_node->data_sent() == 0);
    BOOST_CHECK(bottom_node->headers_sent() == 0);
    BOOST_CHECK(top_node->data_received() == 1);
    BOOST_CHECK(top_node->headers_received() == 1);

    /* Check message content */
    BOOST_CHECK(mg2.check_data(top_node->received_data()));
    BOOST_CHECK(*mg2.header(2) == *top_node->received_header());
}


/* Test that duplicate messages are dropped */
BOOST_AUTO_TEST_CASE( drop_dupl_reorder_test )
{
    stack_accessor acc;
    mid_node->received(acc, mg0.receive_data(), mg0.header(0));

    /* Check sequencer state */
    BOOST_CHECK(mid_node->current_sequence() == 1);
    BOOST_CHECK(mid_node->pending_messages() == 0);

    /* Check number of messages received */
    BOOST_CHECK(bottom_node->data_sent() == 0);
    BOOST_CHECK(bottom_node->headers_sent() == 0);
    BOOST_CHECK(top_node->data_received() == 1);
    BOOST_CHECK(top_node->headers_received() == 1);

    /* Check message contents */
    BOOST_CHECK(mg0.check_data(top_node->received_data()));
    BOOST_CHECK(*mg0.header(0) == *top_node->received_header());

    mid_node->received(acc, mg1.receive_data(), mg1.header(0));

    /* Check sequencer state */
    BOOST_CHECK(mid_node->current_sequence() == 1);
    BOOST_CHECK(mid_node->pending_messages() == 0);
    
    /* Check number of messages received */
    BOOST_CHECK(bottom_node->data_sent() == 0);
    BOOST_CHECK(bottom_node->headers_sent() == 0);
    BOOST_CHECK(top_node->data_received() == 0);
    BOOST_CHECK(top_node->headers_received() == 0);
}


/* Test that simple reordering can be done */
BOOST_AUTO_TEST_CASE( reorder_simple_reorder_test )
{
    stack_accessor acc;
    mid_node->received(acc, mg0.receive_data(), mg0.header(1));

    /* Check sequencer state */
    BOOST_CHECK(mid_node->current_sequence() == 0);
    BOOST_CHECK(mid_node->pending_messages() == 1);

    /* Check number of messages received */
    BOOST_CHECK(bottom_node->data_sent() == 0);
    BOOST_CHECK(bottom_node->headers_sent() == 0);
    BOOST_CHECK(top_node->data_received() == 0);
    BOOST_CHECK(top_node->headers_received() == 0);

    mid_node->received(acc, mg1.receive_data(), mg1.header(0));

    /* Check sequencer state */
    BOOST_CHECK(mid_node->current_sequence() == 2);
    BOOST_CHECK(mid_node->pending_messages() == 0);

    /* Check number of messages received */
    BOOST_CHECK(bottom_node->data_sent() == 0);
    BOOST_CHECK(bottom_node->headers_sent() == 0);
    BOOST_CHECK(top_node->data_received() == 2);
    BOOST_CHECK(top_node->headers_received() == 2);

    /* Check message content */
    BOOST_CHECK(mg1.check_data(top_node->received_data()));
    BOOST_CHECK(*mg1.header(0) == *top_node->received_header());
    BOOST_CHECK(mg0.check_data(top_node->received_data()));
    BOOST_CHECK(*mg0.header(1) == *top_node->received_header());
}


/* Test that slightly more complicated ordering can be done */
BOOST_AUTO_TEST_CASE( recv_complex_reorder_test )
{
    stack_accessor acc;
    mid_node->received(acc, mg0.receive_data(), mg0.header(2));

    /* Check sequencer state */
    BOOST_CHECK(mid_node->current_sequence() == 0);
    BOOST_CHECK(mid_node->pending_messages() == 1);

    /* Check number of messages received */
    BOOST_CHECK(bottom_node->data_sent() == 0);
    BOOST_CHECK(bottom_node->headers_sent() == 0);
    BOOST_CHECK(top_node->data_received() == 0);
    BOOST_CHECK(top_node->headers_received() == 0);

    mid_node->received(acc, mg1.receive_data(), mg1.header(1));

    /* Check sequencer state */
    BOOST_CHECK(mid_node->current_sequence() == 0);
    BOOST_CHECK(mid_node->pending_messages() == 2);

    /* Check number of messages received */
    BOOST_CHECK(bottom_node->data_sent() == 0);
    BOOST_CHECK(bottom_node->headers_sent() == 0);
    BOOST_CHECK(top_node->data_received() == 0);
    BOOST_CHECK(top_node->headers_received() == 0);

    mid_node->received(acc, mg2.receive_data(), mg2.header(0));

    /* Check sequencer state */
    BOOST_CHECK(mid_node->current_sequence() == 3);
    BOOST_CHECK(mid_node->pending_messages() == 0);

    /* Check number of messages received */
    BOOST_CHECK(bottom_node->data_sent() == 0);
    BOOST_CHECK(bottom_node->headers_sent() == 0);
    BOOST_CHECK(top_node->data_received() == 3);
    BOOST_CHECK(top_node->headers_received() == 3);

    /* Check message content */
    BOOST_CHECK(mg2.check_data(top_node->received_data()));
    BOOST_CHECK(*mg2.header(0) == *top_node->received_header());
    BOOST_CHECK(mg1.check_data(top_node->received_data()));
    BOOST_CHECK(*mg1.header(1) == *top_node->received_header());
    BOOST_CHECK(mg0.check_data(top_node->received_data()));
    BOOST_CHECK(*mg0.header(2) == *top_node->received_header());

    mid_node->received(acc, mg2.receive_data(), mg2.header(3));

    /* Check sequencer state */
    BOOST_CHECK(mid_node->current_sequence() == 4);
    BOOST_CHECK(mid_node->pending_messages() == 0);

    /* Check number of messages received */
    BOOST_CHECK(bottom_node->data_sent() == 0);
    BOOST_CHECK(bottom_node->headers_sent() == 0);
    BOOST_CHECK(top_node->data_received() == 1);
    BOOST_CHECK(top_node->headers_received() == 1);

    /* Check message content */
    BOOST_CHECK(mg2.check_data(top_node->received_data()));
    BOOST_CHECK(*mg2.header(3) == *top_node->received_header());
}

BOOST_AUTO_TEST_SUITE_END()
}; /* namespace test */
}; /* namespace raptor_networking */
