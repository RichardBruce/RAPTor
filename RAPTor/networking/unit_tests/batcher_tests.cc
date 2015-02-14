#ifdef STAND_ALONE
#define BOOST_TEST_MODULE batcher test
#endif /* #ifdef STAND_ALONE */

/* Boost headers */
#include "boost/uuid/uuid.hpp"
#include "boost/test/unit_test.hpp"

/* Networking headers */
#include "batcher.h"

/* Test headers */
#include "stack_component_fixture.h"
#include "message_generator.h"
#include "mock_stack_component.h"


using boost::uuids::uuid;
namespace raptor_networking
{
namespace test
{
/* Test data */
typedef batcher<stack_component, stack_component> uut;
struct batcher_fixture : public stack_component_fixture<uut>
{
    batcher_fixture()
    : short_mg(std::string(3, 'a')),
      mid_mg(std::string(6 + (2 * msg_header::size()), 'a')),
      just_mg(std::string(10 + (2 * msg_header::size()), 'a')),
      long_mg(std::string(20 + (2 * msg_header::size()), 'a')),
      max_delay_ms(100),
      max_size_bytes(10 + (4 * msg_header::size()) + MSG_BATCH.size())
    {
        link_uut(new uut(bottom_node, ioservice, ctrl, max_delay_ms, max_size_bytes));
    };

    const message_generator short_mg;
    const message_generator mid_mg;
    const message_generator just_mg;
    const message_generator long_mg;
    const int               max_delay_ms;
    const unsigned short    max_size_bytes;
};


BOOST_FIXTURE_TEST_SUITE( batcher_tests, batcher_fixture );

/* Test constructors */
BOOST_AUTO_TEST_CASE( ctor_test )
{
    BOOST_CHECK(mid_node->pending_bytes() == MSG_BATCH.size());
    BOOST_CHECK(mid_node->max_size() == max_size_bytes);
    BOOST_CHECK(mid_node->max_delay() == max_delay_ms);
}


/* Clone test */
BOOST_AUTO_TEST_CASE( clone_test )
{
    auto cloned = mid_node->clean_clone();
    auto *bat = dynamic_cast<uut*>(cloned.last());
    BOOST_CHECK(bat != nullptr);
    BOOST_CHECK(bat->pending_bytes() == MSG_BATCH.size());
    BOOST_CHECK(bat->max_size() == max_size_bytes);
    BOOST_CHECK(bat->max_delay() == max_delay_ms);

    bat->build_up_links(new mock_stack_component());
    delete cloned.bottom();
}


/* Test that large messages are just passed */
BOOST_AUTO_TEST_CASE( send_large_batcher_test )
{
    stack_accessor acc;
    ctrl->lock_stack(acc, boost::uuids::nil_uuid());
    mid_node->send(acc, long_mg.send_data(), long_mg.header(0), ec);
    
    /* Check for an error */
    BOOST_CHECK(!ec);
    
    /* Check number of messages */
    const unsigned int fragments = 1;
    BOOST_CHECK(bottom_node->data_sent() == fragments);
    BOOST_CHECK(bottom_node->headers_sent() == fragments);
    BOOST_CHECK(top_node->data_received() == 0);
    BOOST_CHECK(top_node->headers_received() == 0);

    /* Check message content */
    BOOST_CHECK(*long_mg.header(0) == *bottom_node->sent_header());
    BOOST_CHECK(long_mg.check_data(bottom_node->sent_data()));
}


/* Test that messages that would be bigger than max size when wrapped are just passed */
BOOST_AUTO_TEST_CASE( send_wrapped_size_batcher_test )
{
    stack_accessor acc;
    ctrl->lock_stack(acc, boost::uuids::nil_uuid());
    mid_node->send(acc, just_mg.send_data(), just_mg.header(1), ec);
    
    /* Check for an error */
    BOOST_CHECK(!ec);
    
    /* Check number of messages */
    const unsigned int fragments = 1;
    BOOST_CHECK(bottom_node->data_sent() == fragments);
    BOOST_CHECK(bottom_node->headers_sent() == fragments);
    BOOST_CHECK(top_node->data_received() == 0);
    BOOST_CHECK(top_node->headers_received() == 0);

    /* Check message content */
    BOOST_CHECK(*just_mg.header(1) == *bottom_node->sent_header());
    BOOST_CHECK(just_mg.check_data(bottom_node->sent_data()));
}


/* Test receiving not a batch */
BOOST_AUTO_TEST_CASE( recv_not_batched_batcher_test )
{
    stack_accessor acc;
    mid_node->received(acc, long_mg.receive_data(), long_mg.header(1));

    /* Check number of messages */
    const unsigned int messages = 1;
    BOOST_CHECK(bottom_node->data_sent() == 0);
    BOOST_CHECK(bottom_node->headers_sent() == 0);
    BOOST_CHECK(top_node->data_received() == messages);
    BOOST_CHECK(top_node->headers_received() == messages);

    /* Check message content */
    BOOST_CHECK(*long_mg.header(1) == *top_node->received_header());
    BOOST_CHECK(long_mg.check_data(top_node->received_data()));
}


/* Test that medium size messages clear the buffer */
BOOST_AUTO_TEST_CASE( send_medium_batcher_test )
{
    stack_accessor acc;
    ctrl->lock_stack(acc, boost::uuids::nil_uuid());
    mid_node->send(acc, mid_mg.send_data(), mid_mg.header(1), ec);

    /* Check for an error */
    BOOST_CHECK(!ec);
    
    /* Check number of messages */
    BOOST_CHECK(bottom_node->data_sent() == 0);
    BOOST_CHECK(bottom_node->headers_sent() == 0);
    BOOST_CHECK(top_node->data_received() == 0);
    BOOST_CHECK(top_node->headers_received() == 0);

    mid_node->send(acc, mid_mg.send_data(), mid_mg.header(2), ec);
    
    /* Check for an error */
    BOOST_CHECK(!ec);
    
    /* Check number of messages */
    const unsigned int fragments = 1;
    BOOST_CHECK(bottom_node->data_sent() == fragments);
    BOOST_CHECK(bottom_node->headers_sent() == fragments);
    BOOST_CHECK(top_node->data_received() == 0);
    BOOST_CHECK(top_node->headers_received() == 0);

    /* Return the message for unbatching */
    bottom_node->loopback_sent_message(acc);

    /* Check number of messages */
    const unsigned int messages = 1;
    BOOST_CHECK(bottom_node->data_sent() == 0);
    BOOST_CHECK(bottom_node->headers_sent() == 0);
    BOOST_CHECK(top_node->data_received() == messages);
    BOOST_CHECK(top_node->headers_received() == messages);

    /* Check message content */
    const std::shared_ptr<msg_header> &recv_header = top_node->received_header();
    BOOST_CHECK(recv_header->has_physical_address());
    recv_header->has_physical_address(false);
    BOOST_CHECK(*mid_mg.header(1) == *recv_header);
    BOOST_CHECK(mid_mg.check_data(top_node->received_data()));
}


/* Test that small messages are batched */
BOOST_AUTO_TEST_CASE( send_small_batcher_test )
{
    stack_accessor acc;
    ctrl->lock_stack(acc, boost::uuids::nil_uuid());
    mid_node->send(acc, short_mg.send_data(), short_mg.header(1), ec);
    
    /* Check for an error */
    BOOST_CHECK(!ec);
    
    /* Check number of messages */
    BOOST_CHECK(bottom_node->data_sent() == 0);
    BOOST_CHECK(bottom_node->headers_sent() == 0);
    BOOST_CHECK(top_node->data_received() == 0);
    BOOST_CHECK(top_node->headers_received() == 0);

    mid_node->send(acc, short_mg.send_data(), short_mg.header(2), ec);
    
    /* Check for an error */
    BOOST_CHECK(!ec);
    
    /* Check number of messages */
    BOOST_CHECK(bottom_node->data_sent() == 0);
    BOOST_CHECK(bottom_node->headers_sent() == 0);
    BOOST_CHECK(top_node->data_received() == 0);
    BOOST_CHECK(top_node->headers_received() == 0);

    mid_node->send(acc, short_mg.send_data(), short_mg.header(3), ec);

    /* Check for an error */
    BOOST_CHECK(!ec);
    
    /* Check number of messages */
    BOOST_CHECK(bottom_node->data_sent() == 0);
    BOOST_CHECK(bottom_node->headers_sent() == 0);
    BOOST_CHECK(top_node->data_received() == 0);
    BOOST_CHECK(top_node->headers_received() == 0);

    mid_node->send(acc, short_mg.send_data(), short_mg.header(0), ec);

    /* Check for an error */
    BOOST_CHECK(!ec);
    
    /* Check number of messages */
    const unsigned int fragments = 1;
    BOOST_CHECK(bottom_node->data_sent() == fragments);
    BOOST_CHECK(bottom_node->headers_sent() == fragments);
    BOOST_CHECK(top_node->data_received() == 0);
    BOOST_CHECK(top_node->headers_received() == 0);

    /* Return the message for unbatching */
    bottom_node->loopback_sent_message(acc);

    /* Check number of messages */
    const unsigned int messages = 3;
    BOOST_CHECK(bottom_node->data_sent() == 0);
    BOOST_CHECK(bottom_node->headers_sent() == 0);
    BOOST_CHECK(top_node->data_received() == messages);
    BOOST_CHECK(top_node->headers_received() == messages);

    /* Check message content */
    for (unsigned int i = 0; i < messages; ++i)
    {
        const std::shared_ptr<msg_header> &recv_header = top_node->received_header();
        BOOST_CHECK(recv_header->has_physical_address());
        recv_header->has_physical_address(false);
        BOOST_CHECK(*short_mg.header(i + 1) == *recv_header);
        BOOST_CHECK(short_mg.check_data(top_node->received_data()));
    }
}


/* Test that small messages are batched and time out */
BOOST_AUTO_TEST_CASE( send_small_timeout_batcher_test )
{
    stack_accessor acc;
    ctrl->lock_stack(acc, boost::uuids::nil_uuid());
    mid_node->send(acc, short_mg.send_data(), short_mg.header(1), ec);
    
    /* Check for an error */
    BOOST_CHECK(!ec);

    /* Check nothing was sent */
    BOOST_CHECK(bottom_node->data_sent() == 0);
    BOOST_CHECK(bottom_node->headers_sent() == 0);
    BOOST_CHECK(top_node->data_received() == 0);
    BOOST_CHECK(top_node->headers_received() == 0);

    /* Wait for timeout */
    std::thread thread(std::bind(
                    (std::size_t(boost::asio::io_service::*)())&boost::asio::io_service::run, &ioservice));
    thread.detach();
    std::this_thread::sleep_for(std::chrono::duration<int,std::milli>(max_delay_ms * 2));
    
    /* Check number of messages */
    const unsigned int fragments = 1;
    BOOST_CHECK(bottom_node->data_sent() == fragments);
    BOOST_CHECK(bottom_node->headers_sent() == fragments);
    BOOST_CHECK(top_node->data_received() == 0);
    BOOST_CHECK(top_node->headers_received() == 0);

    /* Return the message for unbatching */
    bottom_node->loopback_sent_message(acc);

    /* Check number of messages */
    const unsigned int messages = 1;
    BOOST_CHECK(bottom_node->data_sent() == 0);
    BOOST_CHECK(bottom_node->headers_sent() == 0);
    BOOST_CHECK(top_node->data_received() == messages);
    BOOST_CHECK(top_node->headers_received() == messages);

    /* Check message content */
    const std::shared_ptr<msg_header> &recv_header = top_node->received_header();
    BOOST_CHECK(recv_header->has_physical_address());
    recv_header->has_physical_address(false);
    BOOST_CHECK(*short_mg.header(1) == *recv_header);
    BOOST_CHECK(short_mg.check_data(top_node->received_data()));
}


BOOST_AUTO_TEST_SUITE_END()
}; /* namespace test */
}; /* namespace raptor_networking */
