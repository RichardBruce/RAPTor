#ifdef STAND_ALONE
#define BOOST_TEST_MODULE fragmenter test
#endif /* #ifdef STAND_ALONE */


/* Boost headers */
#include "boost/test/unit_test.hpp"

/* Networking headers */
#include "fragmenter.h"

/* Test headers */
#include "stack_component_fixture.h"
#include "message_generator.h"
#include "mock_stack_component.h"


using namespace raptor_networking;

/* Test data */
typedef fixed_size_fragmenter<stack_component, stack_component> uut;
struct fragmenter_fixture : public stack_component_fixture<uut>
{
    fragmenter_fixture()
    : helloworld_mg("Hello World"),
      ab_mg("ab"),
      fragment_size(4)
    {
        link_uut(new uut(bottom_node, fragment_size));
    };

    const message_generator helloworld_mg;
    const message_generator ab_mg;
    const std::uint32_t     fragment_size;
};


BOOST_FIXTURE_TEST_SUITE( fragmenter_tests, fragmenter_fixture )

/* Test constructors */
BOOST_AUTO_TEST_CASE( ctor_test )
{
    BOOST_CHECK(mid_node->pending_messages() == 0);
    BOOST_CHECK(mid_node->fragment_size() == fragment_size);
}


/* Clone test */
BOOST_AUTO_TEST_CASE( clone_test )
{
    auto cloned = mid_node->clean_clone();
    auto *frag = dynamic_cast<uut*>(cloned.last());
    BOOST_CHECK(frag != nullptr);
    BOOST_CHECK(frag->pending_messages() == 0);
    BOOST_CHECK(frag->fragment_size() == fragment_size);

    frag->build_up_links(new mock_stack_component());
    delete cloned.bottom();
}


/* Test send */
BOOST_AUTO_TEST_CASE( send_test_not_fragmented )
{
    stack_accessor acc;
    mid_node->send(acc, ab_mg.send_data(), ab_mg.header(1), ec);

    /* Check for an error */
    BOOST_CHECK(!ec);
    
    /* Check number of messages */
    const unsigned int fragments = 1;
    BOOST_CHECK(bottom_node->data_sent() == fragments);
    BOOST_CHECK(bottom_node->headers_sent() == fragments);
    BOOST_CHECK(top_node->data_received() == 0);
    BOOST_CHECK(top_node->headers_received() == 0);

    /* Check message content */
    BOOST_CHECK(*ab_mg.header(1) == *bottom_node->sent_header());
    BOOST_CHECK(ab_mg.check_data(bottom_node->sent_data()));
}


BOOST_AUTO_TEST_CASE( send_test_fragmented )
{
    stack_accessor acc;
    mid_node->send(acc, helloworld_mg.send_data(), helloworld_mg.header(1), ec);

    /* Check for an error */
    BOOST_CHECK(!ec);
    
    /* Check number of messages */
    const unsigned int fragments = std::ceil(helloworld_mg.message_size() / static_cast<float>(fragment_size));
    BOOST_CHECK(bottom_node->data_sent() == fragments);
    BOOST_CHECK(bottom_node->headers_sent() == fragments);
    BOOST_CHECK(top_node->data_received() == 0);
    BOOST_CHECK(top_node->headers_received() == 0);

    /* Check message content */
    for (unsigned int i = 0; i < fragments; ++i)
    {
        auto sent_header = bottom_node->sent_header();
        BOOST_CHECK(sent_header->fragment() == i);
        sent_header->fragment(0);
        sent_header->fragment_length(sent_header->length());
        BOOST_CHECK(*helloworld_mg.header(1) == *sent_header);

        const unsigned int fragment_end = std::min<size_t>((i + 1) * fragment_size, helloworld_mg.message_size());
        BOOST_CHECK(helloworld_mg.check_data(bottom_node->sent_data(), i * fragment_size, fragment_end));
    }
}


BOOST_AUTO_TEST_CASE( send_test_fragmented_error_out )
{
    stack_accessor acc;
    ec = make_error_code(boost::system::errc::errc_t::bad_message);
    mid_node->send(acc, helloworld_mg.send_data(), helloworld_mg.header(1), ec);

    /* Check for an error */
    BOOST_CHECK(ec == make_error_code(boost::system::errc::errc_t::bad_message));
    
    /* Check number of messages */
    BOOST_CHECK(bottom_node->data_sent() == 1);
    BOOST_CHECK(bottom_node->headers_sent() == 1);
    BOOST_CHECK(top_node->data_received() == 0);
    BOOST_CHECK(top_node->headers_received() == 0);

    /* Check message content */
    auto sent_header = bottom_node->sent_header();
    BOOST_CHECK(sent_header->fragment() == 0);
    BOOST_CHECK(sent_header->fragment_length() == fragment_size);
    sent_header->fragment(0);
    sent_header->fragment_length(sent_header->length());
    BOOST_CHECK(*helloworld_mg.header(1) == *sent_header);

    BOOST_CHECK(helloworld_mg.check_data(bottom_node->sent_data(), 0, fragment_size));
}


/* Test receive */
BOOST_AUTO_TEST_CASE( recv_test_not_fragmented )
{
    stack_accessor acc;
    mid_node->received(acc, ab_mg.receive_data(), ab_mg.header(1));

    /*  Check fragmenter state */
    BOOST_CHECK(mid_node->pending_messages() == 0);

    /* Check number of messages */
    const unsigned int fragments = 1;
    BOOST_CHECK(bottom_node->data_sent() == 0);
    BOOST_CHECK(bottom_node->headers_sent() == 0);
    BOOST_CHECK(top_node->data_received() == fragments);
    BOOST_CHECK(top_node->headers_received() == fragments);

    /* Check message content */
    BOOST_CHECK(*ab_mg.header(1) == *top_node->received_header());
    BOOST_CHECK(ab_mg.check_data(top_node->received_data()));
}


BOOST_AUTO_TEST_CASE( recv_test_buffer_and_clean )
{
    /* Send a message */
    stack_accessor acc;
    mid_node->send(acc, helloworld_mg.send_data(), helloworld_mg.header(1), ec);

    /* Receive it back again */
    const unsigned int fragments = std::ceil(helloworld_mg.message_size() / static_cast<float>(fragment_size));
    for (unsigned int i = 0; i < fragments - 1; ++i)
    {
        bottom_node->loopback_sent_message(acc);
        BOOST_CHECK(mid_node->pending_messages() == ((i + 1) == fragments) ? 0 : 1);
    }

    /* Need memcheck to check this gets cleaned up */
}


BOOST_AUTO_TEST_CASE( recv_test_fragmented )
{
    /* Send a message */
    stack_accessor acc;
    mid_node->send(acc, helloworld_mg.send_data(), helloworld_mg.header(1), ec);

    /* Receive it back again */
    const unsigned int fragments = std::ceil(helloworld_mg.message_size() / static_cast<float>(fragment_size));
    for (unsigned int i = 0; i < fragments; ++i)
    {
        bottom_node->loopback_sent_message(acc);
        BOOST_CHECK(mid_node->pending_messages() == ((i + 1) == fragments) ? 0 : 1);
    }

    /* Check number of messages */
    const unsigned int messages = 1;
    BOOST_CHECK(bottom_node->data_sent() == 0);
    BOOST_CHECK(bottom_node->headers_sent() == 0);
    BOOST_CHECK(top_node->data_received() == messages);
    BOOST_CHECK(top_node->headers_received() == messages);

    /* Check header physics address then remove */
    auto header = top_node->received_header();
    BOOST_CHECK(header->has_physical_address());
    header->has_physical_address(false);
    
    /* Check message content */
    BOOST_CHECK(*helloworld_mg.header(1) == *header);
    BOOST_CHECK(helloworld_mg.check_data(top_node->received_data()));
}


BOOST_AUTO_TEST_CASE( recv_test_fragmented_out_of_order )
{
    /* Send a message */
    stack_accessor acc;
    mid_node->send(acc, helloworld_mg.send_data(), helloworld_mg.header(1), ec);

    /* Receive it back again */
    const unsigned int fragments = std::ceil(helloworld_mg.message_size() / static_cast<float>(fragment_size));
    for (unsigned int i = 0; i < fragments; ++i)
    {
        bottom_node->loopback_sent_message(acc);
        mid_node->received(acc, helloworld_mg.receive_data(), helloworld_mg.header(i + 10));
        BOOST_CHECK(mid_node->pending_messages() == ((i + 1) == fragments) ? (i + 1) : (i + 2));
    }

    /* Check number of messages */
    const unsigned int messages = 1;
    BOOST_CHECK(bottom_node->data_sent() == 0);
    BOOST_CHECK(bottom_node->headers_sent() == 0);
    BOOST_CHECK(top_node->data_received() == messages);
    BOOST_CHECK(top_node->headers_received() == messages);
    
    /* Check header physics address then remove */
    auto header = top_node->received_header();
    BOOST_CHECK(header->has_physical_address());
    header->has_physical_address(false);
    
    /* Check message content */
    BOOST_CHECK(*helloworld_mg.header(1) == *header);
    BOOST_CHECK(helloworld_mg.check_data(top_node->received_data()));
}

BOOST_AUTO_TEST_SUITE_END()
