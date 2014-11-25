#ifdef STAND_ALONE
#define BOOST_TEST_MODULE group test
#endif /* #ifdef STAND_ALONE */


/* Boost headers */
#include "boost/test/unit_test.hpp"

/* Networking headers */
#include "group.h"

/* Test headers */
#include "stack_component_fixture.h"
#include "message_generator.h"
#include "mock_stack_component.h"


using namespace raptor_networking;

/* Test data */
typedef group uut;
struct group_fixture : public stack_component_fixture<uut>
{
    group_fixture()
    : subscribe_mg(std::string(MSG_SUBSCRIBE.begin(), MSG_SUBSCRIBE.end())),
      unsubscribe_mg(std::string(MSG_UNSUBSCRIBE.begin(), MSG_UNSUBSCRIBE.end())),
      ab_mg("ab"),
      group_address("255.239.0.1"),
      me_address("192.168.0.100"),
      from_address("192.168.0.200"),
      group_as_address(address::from_string(group_address)),
      me_as_address(address::from_string(me_address)),
      group_port(7000),
      port_offset(3)
    {
        uut *grp = new uut(ctrl, group_address, me_address, group_port, port_offset);
        grp->link_down_node(bottom_node);
        link_uut(grp);
    };

    const message_generator subscribe_mg;
    const message_generator unsubscribe_mg;
    const message_generator ab_mg;
    const std::string       group_address;
    const std::string       me_address;
    const std::string       from_address;
    const address           group_as_address;
    const address           me_as_address;
    const std::uint16_t     group_port;
    const std::uint16_t     port_offset;
};


BOOST_FIXTURE_TEST_SUITE( group_tests, group_fixture )

/* Test constructors */
BOOST_AUTO_TEST_CASE( ctor0_test )
{
    BOOST_CHECK(mid_node->group_physical_address() == group_as_address);
    BOOST_CHECK(mid_node->group_address() != boost::uuids::nil_uuid());
    BOOST_CHECK(mid_node->group_port() == group_port);
    BOOST_CHECK(mid_node->size_of_group() == 1);
    auto addr = mid_node->my_address();
    BOOST_CHECK(*mid_node->physical_address(addr) == me_as_address);
    BOOST_CHECK(mid_node->port_offset(addr) == port_offset);
}


BOOST_AUTO_TEST_CASE( ctor1_test )
{
    const std::unique_ptr<uut> test(new uut(ctrl, me_address, port_offset));
    test->link_down_node(bottom_node);
    BOOST_CHECK(test->group_physical_address() == address());
    BOOST_CHECK(test->group_address() != boost::uuids::nil_uuid());
    BOOST_CHECK(test->group_port() == 0);
    BOOST_CHECK(test->size_of_group() == 1);
    auto addr = test->my_address();
    BOOST_CHECK(*test->physical_address(addr) == me_as_address);
    BOOST_CHECK(test->port_offset(addr) == port_offset);
}


/* Clone test */
BOOST_AUTO_TEST_CASE( clone_test )
{
    auto cloned = mid_node->clean_clone();
    auto *grp = dynamic_cast<uut*>(cloned.last());
    BOOST_CHECK(grp != nullptr);
    BOOST_CHECK(grp->group_physical_address() == group_as_address);
    BOOST_CHECK(grp->group_address() != boost::uuids::nil_uuid());
    BOOST_CHECK(grp->group_port() == group_port);
    BOOST_CHECK(grp->size_of_group() == 1);
    auto addr = grp->my_address();
    BOOST_CHECK(*grp->physical_address(addr) == me_as_address);
    BOOST_CHECK(grp->port_offset(addr) == port_offset);

    grp->build_up_links(new mock_stack_component());
    delete cloned.bottom();
}


/* Test access functions */
BOOST_AUTO_TEST_CASE( physical_address_test )
{
    BOOST_CHECK(mid_node->physical_address(boost::uuids::nil_uuid()) == nullptr);
}


BOOST_AUTO_TEST_CASE( port_offset_test )
{
    BOOST_CHECK(mid_node->port_offset(boost::uuids::nil_uuid()) == 0);
}


BOOST_AUTO_TEST_CASE( add_to_group_test )
{
    const auto id(subscribe_mg.from_address());
    const auto addr(address::from_string("192.168.0.255"));
    mid_node->add_to_group(id, addr, 7);

    // Checks
    BOOST_CHECK( mid_node->port_offset(id)      == 7);
    BOOST_CHECK(*mid_node->physical_address(id) == addr);
}


/* Test send */
BOOST_AUTO_TEST_CASE( send_test )
{
    stack_accessor acc;
    mid_node->send(acc, ab_mg.send_data(), ab_mg.header(1), ec);

    /* Check for an error */
    BOOST_CHECK(!ec);

    /* Check group state */
    BOOST_CHECK(mid_node->group_physical_address() == group_as_address);
    BOOST_CHECK(mid_node->group_address() != boost::uuids::nil_uuid());
    BOOST_CHECK(mid_node->group_port() == group_port);
    BOOST_CHECK(mid_node->size_of_group() == 1);
    auto addr = mid_node->my_address();
    BOOST_CHECK(*mid_node->physical_address(addr) == me_as_address);
    BOOST_CHECK(mid_node->port_offset(addr) == port_offset);
    
    /* Check number of messages */
    BOOST_CHECK(bottom_node->data_sent() == 1);
    BOOST_CHECK(bottom_node->headers_sent() == 1);
    BOOST_CHECK(top_node->data_received() == 0);
    BOOST_CHECK(top_node->headers_received() == 0);

    /* Check message content */
    BOOST_CHECK(*ab_mg.header(1) == *bottom_node->sent_header());
    BOOST_CHECK(ab_mg.check_data(bottom_node->sent_data()));
}


/* Test receive */
BOOST_AUTO_TEST_CASE( recv_test )
{
    stack_accessor acc;
    mid_node->received(acc, ab_mg.receive_data(), ab_mg.header(1));

    /* Check group state */
    BOOST_CHECK(mid_node->group_physical_address() == group_as_address);
    BOOST_CHECK(mid_node->group_address() != boost::uuids::nil_uuid());
    BOOST_CHECK(mid_node->group_port() == group_port);
    BOOST_CHECK(mid_node->size_of_group() == 1);
    auto addr = mid_node->my_address();
    BOOST_CHECK(*mid_node->physical_address(addr) == me_as_address);
    BOOST_CHECK(mid_node->port_offset(addr) == port_offset);

    /* Check number of messages */
    BOOST_CHECK(bottom_node->data_sent() == 0);
    BOOST_CHECK(bottom_node->headers_sent() == 0);
    BOOST_CHECK(top_node->data_received() == 1);
    BOOST_CHECK(top_node->headers_received() == 1);

    /* Check message content */
    BOOST_CHECK(*ab_mg.header(1) == *top_node->received_header());
    BOOST_CHECK(ab_mg.check_data(top_node->received_data()));
}


BOOST_AUTO_TEST_CASE( recv_sub_test )
{
    stack_accessor acc;
    ctrl->lock_stack(acc, boost::uuids::nil_uuid());
    mid_node->received(acc, subscribe_mg.receive_data(), subscribe_mg.header(from_address, 1));

    /* Check group state */
    BOOST_CHECK(mid_node->group_physical_address() == group_as_address);
    BOOST_CHECK(mid_node->group_address() != boost::uuids::nil_uuid());
    BOOST_CHECK(mid_node->group_port() == group_port);
    BOOST_CHECK(mid_node->size_of_group() == 2);
    auto addr = mid_node->my_address();
    BOOST_CHECK(*mid_node->physical_address(addr) == me_as_address);
    BOOST_CHECK(mid_node->port_offset(addr) == port_offset);

    /* Check number of messages */
    BOOST_CHECK(bottom_node->data_sent() == 1);
    BOOST_CHECK(bottom_node->headers_sent() == 1);
    BOOST_CHECK(top_node->data_received() == 0);
    BOOST_CHECK(top_node->headers_received() == 0);

    /* Check message content */
    const msg_header acc_header(subscribe_mg.from_address(), 0, bottom_node->sent_data()->size(), 1, true);
    BOOST_CHECK(acc_header == *bottom_node->sent_header());
    /* Dont even attempt to check the message, it's well complicated */
}


BOOST_AUTO_TEST_CASE( recv_acc_test )
{
    stack_accessor acc;
    ctrl->lock_stack(acc, boost::uuids::nil_uuid());
    mid_node->received(acc, subscribe_mg.receive_data(), subscribe_mg.header(from_address, 1));

    /* Check group state */
    BOOST_CHECK(mid_node->group_physical_address() == group_as_address);
    BOOST_CHECK(mid_node->group_address() != boost::uuids::nil_uuid());
    BOOST_CHECK(mid_node->group_port() == group_port);
    BOOST_CHECK(mid_node->size_of_group() == 2);
    const auto addr0 = mid_node->my_address();
    BOOST_CHECK(*mid_node->physical_address(addr0) == me_as_address);
    BOOST_CHECK(mid_node->port_offset(addr0) == port_offset);

    /* Check number of messages */
    BOOST_CHECK(bottom_node->data_sent() == 1);
    BOOST_CHECK(bottom_node->headers_sent() == 1);
    BOOST_CHECK(top_node->data_received() == 0);
    BOOST_CHECK(top_node->headers_received() == 0);

    /* Return the message for deserialising */
    const auto group_addr = mid_node->group_address();
    bottom_node->loopback_sent_message(acc);

    /* Check group state */
    BOOST_CHECK(mid_node->group_physical_address() == group_as_address);
    BOOST_CHECK(mid_node->group_address() == group_addr);
    BOOST_CHECK(mid_node->group_port() == group_port);
    BOOST_CHECK(mid_node->size_of_group() == 2);
    const auto addr1 = mid_node->my_address();
    BOOST_CHECK(*mid_node->physical_address(addr1) == me_as_address);
    BOOST_CHECK(mid_node->port_offset(addr1) == port_offset);

    /* Check number of messages */
    BOOST_CHECK(bottom_node->data_sent() == 0);
    BOOST_CHECK(bottom_node->headers_sent() == 0);
    BOOST_CHECK(top_node->data_received() == 0);
    BOOST_CHECK(top_node->headers_received() == 0);
}


BOOST_AUTO_TEST_CASE( recv_unsub_test )
{
    stack_accessor acc;
    ctrl->lock_stack(acc, boost::uuids::nil_uuid());
    mid_node->received(acc, subscribe_mg.receive_data(), subscribe_mg.header(from_address, 1));

    /* Check group state */
    BOOST_CHECK(mid_node->group_physical_address() == group_as_address);
    BOOST_CHECK(mid_node->group_address() != boost::uuids::nil_uuid());
    BOOST_CHECK(mid_node->group_port() == group_port);
    BOOST_CHECK(mid_node->size_of_group() == 2);
    const auto addr0 = mid_node->my_address();
    BOOST_CHECK(*mid_node->physical_address(addr0) == me_as_address);
    BOOST_CHECK(mid_node->port_offset(addr0) == port_offset);

    /* Check number of messages */
    BOOST_CHECK(bottom_node->data_sent() == 1);
    BOOST_CHECK(bottom_node->headers_sent() == 1);
    BOOST_CHECK(top_node->data_received() == 0);
    BOOST_CHECK(top_node->headers_received() == 0);

    /* Check message content */
    const msg_header acc0_header(subscribe_mg.from_address(), 0, bottom_node->sent_data()->size(), 1, true);
    BOOST_CHECK(acc0_header == *bottom_node->sent_header());
    /* Dont even attempt to check the message, it's well complicated */

    mid_node->received(acc, unsubscribe_mg.receive_data(), subscribe_mg.header(from_address, 2));

    /* Check group state */
    BOOST_CHECK(mid_node->group_physical_address() == group_as_address);
    BOOST_CHECK(mid_node->group_address() != boost::uuids::nil_uuid());
    BOOST_CHECK(mid_node->group_port() == group_port);
    BOOST_CHECK(mid_node->size_of_group() == 1);
    const auto addr1 = mid_node->my_address();
    BOOST_CHECK(*mid_node->physical_address(addr1) == me_as_address);
    BOOST_CHECK(mid_node->port_offset(addr1) == port_offset);

    /* Check number of messages */
    BOOST_CHECK(bottom_node->data_sent() == 1);
    BOOST_CHECK(bottom_node->headers_sent() == 1);
    BOOST_CHECK(top_node->data_received() == 0);
    BOOST_CHECK(top_node->headers_received() == 0);

    /* Check message content */
    const msg_header acc1_header(subscribe_mg.from_address(), 1, bottom_node->sent_data()->size(), 2, true);
    BOOST_CHECK(acc1_header == *bottom_node->sent_header());
    /* Dont even attempt to check the message, it's well complicated */
}


/* Test connect */
BOOST_AUTO_TEST_CASE( send_connect_test )
{
    stack_accessor acc;
    std::future<bool> fut = mid_node->connect();

    /* Check group state */
    BOOST_CHECK(mid_node->group_physical_address() == group_as_address);
    BOOST_CHECK(mid_node->group_address() != boost::uuids::nil_uuid());
    BOOST_CHECK(mid_node->group_port() == group_port);
    BOOST_CHECK(mid_node->size_of_group() == 1);
    auto addr = mid_node->my_address();
    BOOST_CHECK(*mid_node->physical_address(addr) == me_as_address);
    BOOST_CHECK(mid_node->port_offset(addr) == port_offset);

    /* Check number of messages */
    BOOST_CHECK(bottom_node->data_sent() == 1);
    BOOST_CHECK(bottom_node->headers_sent() == 1);
    BOOST_CHECK(top_node->data_received() == 0);
    BOOST_CHECK(top_node->headers_received() == 0);

    /* Check message content */
    const msg_header sub_header(boost::uuids::nil_uuid(), mid_node->group_address(), 0, MSG_SUBSCRIBE.size());
    BOOST_CHECK(sub_header == *bottom_node->sent_header());
    BOOST_CHECK(std::equal(MSG_SUBSCRIBE.begin(), MSG_SUBSCRIBE.end(), bottom_node->sent_data()->begin()));
}


BOOST_AUTO_TEST_CASE( connect_resp_test )
{
    stack_accessor acc;
    ctrl->lock_stack(acc, boost::uuids::nil_uuid());
    std::future<bool> fut = mid_node->connect();

    /* Check group state */
    BOOST_CHECK(mid_node->group_physical_address() == group_as_address);
    BOOST_CHECK(mid_node->group_address() != boost::uuids::nil_uuid());
    BOOST_CHECK(mid_node->group_port() == group_port);
    BOOST_CHECK(mid_node->size_of_group() == 1);
    auto addr = mid_node->my_address();
    BOOST_CHECK(*mid_node->physical_address(addr) == me_as_address);
    BOOST_CHECK(mid_node->port_offset(addr) == port_offset);

    /* Check number of messages */
    BOOST_CHECK(bottom_node->data_sent() == 1);
    BOOST_CHECK(bottom_node->headers_sent() == 1);
    BOOST_CHECK(top_node->data_received() == 0);
    BOOST_CHECK(top_node->headers_received() == 0);

    /* Check message content */
    bottom_node->loopback_sent_message(acc);

    /* Check group state */
    BOOST_CHECK(mid_node->group_physical_address() == group_as_address);
    BOOST_CHECK(mid_node->group_address() != boost::uuids::nil_uuid());
    BOOST_CHECK(mid_node->group_port() == group_port);
    BOOST_CHECK(mid_node->size_of_group() == 2);
    const auto addr0 = mid_node->my_address();
    BOOST_CHECK(*mid_node->physical_address(addr0) == me_as_address);
    BOOST_CHECK(mid_node->port_offset(addr0) == port_offset);

    /* Check number of messages */
    BOOST_CHECK(bottom_node->data_sent() == 1);
    BOOST_CHECK(bottom_node->headers_sent() == 1);
    BOOST_CHECK(top_node->data_received() == 0);
    BOOST_CHECK(top_node->headers_received() == 0);

    /* Return the message for deserialising */
    const auto group_addr = mid_node->group_address();
    bottom_node->loopback_sent_message(acc);

    /* Check group state */
    BOOST_CHECK(mid_node->group_physical_address() == group_as_address);
    BOOST_CHECK(mid_node->group_address() == group_addr);
    BOOST_CHECK(mid_node->group_port() == group_port);
    BOOST_CHECK(mid_node->size_of_group() == 2);
    const auto addr1 = mid_node->my_address();
    BOOST_CHECK(*mid_node->physical_address(addr1) == me_as_address);
    BOOST_CHECK(mid_node->port_offset(addr1) == port_offset);

    /* Check number of messages */
    BOOST_CHECK(bottom_node->data_sent() == 0);
    BOOST_CHECK(bottom_node->headers_sent() == 0);
    BOOST_CHECK(top_node->data_received() == 0);
    BOOST_CHECK(top_node->headers_received() == 0);

    /* Check the future completed */
    BOOST_REQUIRE(fut.wait_for(std::chrono::seconds(0)) == future_status::ready);
    BOOST_CHECK(fut.get() == true);
}


/* Test connect */
BOOST_AUTO_TEST_CASE( send_disconnect_test )
{
    stack_accessor acc;
    BOOST_CHECK(mid_node->disconnect());

    /* Check group state */
    BOOST_CHECK(mid_node->group_physical_address() == group_as_address);
    BOOST_CHECK(mid_node->group_address() != boost::uuids::nil_uuid());
    BOOST_CHECK(mid_node->group_port() == group_port);
    BOOST_CHECK(mid_node->size_of_group() == 1);
    auto addr = mid_node->my_address();
    BOOST_CHECK(*mid_node->physical_address(addr) == me_as_address);
    BOOST_CHECK(mid_node->port_offset(addr) == port_offset);

    /* Check number of messages */
    BOOST_CHECK(bottom_node->data_sent() == 1);
    BOOST_CHECK(bottom_node->headers_sent() == 1);
    BOOST_CHECK(top_node->data_received() == 0);
    BOOST_CHECK(top_node->headers_received() == 0);

    /* Check message content */
    const msg_header sub_header(boost::uuids::nil_uuid(), mid_node->group_address(), 0, MSG_UNSUBSCRIBE.size());
    BOOST_CHECK(sub_header == *bottom_node->sent_header());
    BOOST_CHECK(std::equal(MSG_UNSUBSCRIBE.begin(), MSG_UNSUBSCRIBE.end(), bottom_node->sent_data()->begin()));
}


BOOST_AUTO_TEST_SUITE_END()
