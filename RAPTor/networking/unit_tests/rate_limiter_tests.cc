#ifdef STAND_ALONE
#define BOOST_TEST_MODULE rate_limiter test
#endif /* #ifdef STAND_ALONE */


/* Boost headers */
#include "boost/test/unit_test.hpp"

/* Networking headers */
#include "rate_limiter.h"

/* Test headers */
#include "stack_component_fixture.h"
#include "message_generator.h"
#include "mock_stack_component.h"


namespace raptor_networking
{
namespace test
{
/* Test data */
typedef rate_limiter<stack_component, stack_component> uut;
struct rate_limiter_fixture : public stack_component_fixture<uut>
{
    rate_limiter_fixture()
    : ax1024(std::string(1024, 'a')),
      ab_mg("ab"),
      bytes_per_milli(1.0f)
    {
        link_uut(new uut(bottom_node, ioservice, bytes_per_milli));
    };

    const message_generator ax1024;
    const message_generator ab_mg;
    const float             bytes_per_milli;
};


BOOST_FIXTURE_TEST_SUITE( rate_limiter_tests, rate_limiter_fixture );

/* Test constructors */
BOOST_AUTO_TEST_CASE( ctor_test )
{
    BOOST_CHECK(mid_node->rate() == bytes_per_milli);
}


/* Clone test */
BOOST_AUTO_TEST_CASE( clone_test )
{
    auto cloned = mid_node->clean_clone();
    auto *limit = dynamic_cast<uut*>(cloned.last());
    BOOST_CHECK(limit != nullptr);
    BOOST_CHECK(limit->rate() == bytes_per_milli);

    limit->build_up_links(new mock_stack_component());
    delete cloned.bottom();
}


/* Test send */
BOOST_AUTO_TEST_CASE( send_test )
{
    stack_accessor acc;
    mid_node->send(acc, ab_mg.send_data(), ab_mg.header(1), ec);

    /* Check for an error */
    BOOST_CHECK(!ec);
    
    /* Check number of messages */
    BOOST_CHECK(bottom_node->data_sent() == 1);
    BOOST_CHECK(bottom_node->headers_sent() == 1);
    BOOST_CHECK(top_node->data_received() == 0);
    BOOST_CHECK(top_node->headers_received() == 0);

    /* Check message content */
    BOOST_CHECK(*ab_mg.header(1) == *bottom_node->sent_header());
    BOOST_CHECK(ab_mg.check_data(bottom_node->sent_data()));
}


/* Test limited send */
BOOST_AUTO_TEST_CASE( send_limited_test )
{
    stack_accessor acc;
    auto start_time(std::chrono::system_clock::now());
    mid_node->send(acc, ax1024.send_data(), ax1024.header(1), ec);

    /* Check for an error */
    BOOST_CHECK(!ec);
    
    mid_node->send(acc, ab_mg.send_data(), ab_mg.header(1), ec);

    /* Check timing */
    auto end_time(std::chrono::system_clock::now());
    auto run_time(std::chrono::duration_cast<std::chrono::duration<int, std::milli>>(end_time - start_time));
    BOOST_CHECK_CLOSE(run_time.count(), (1024 / bytes_per_milli), 2);

    /* Check for an error */
    BOOST_CHECK(!ec);
    
    /* Check number of messages */
    BOOST_CHECK(bottom_node->data_sent() == 2);
    BOOST_CHECK(bottom_node->headers_sent() == 2);
    BOOST_CHECK(top_node->data_received() == 0);
    BOOST_CHECK(top_node->headers_received() == 0);

    /* Check message content */
    BOOST_CHECK(*ax1024.header(1) == *bottom_node->sent_header());
    BOOST_CHECK(ax1024.check_data(bottom_node->sent_data()));
    BOOST_CHECK(*ab_mg.header(1) == *bottom_node->sent_header());
    BOOST_CHECK(ab_mg.check_data(bottom_node->sent_data()));
}


/* Test receive */
BOOST_AUTO_TEST_CASE( recv_test )
{
    stack_accessor acc;
    mid_node->received(acc, ab_mg.receive_data(), ab_mg.header(1));

    /* Check number of messages */
    BOOST_CHECK(bottom_node->data_sent() == 0);
    BOOST_CHECK(bottom_node->headers_sent() == 0);
    BOOST_CHECK(top_node->data_received() == 1);
    BOOST_CHECK(top_node->headers_received() == 1);

    /* Check message content */
    BOOST_CHECK(*ab_mg.header(1) == *top_node->received_header());
    BOOST_CHECK(ab_mg.check_data(top_node->received_data()));
}

BOOST_AUTO_TEST_SUITE_END()
}; /* namespace test */
}; /* namespace raptor_networking */
