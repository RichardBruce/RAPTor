#ifdef STAND_ALONE
#define BOOST_TEST_MODULE tcp_connection test
#endif /* #ifdef STAND_ALONE */


/* Boost headers */
#include "boost/test/unit_test.hpp"

/* Networking headers */
#include "group.h"
#include "tcp_connection.h"

/* Test headers */
#include "stack_component_fixture.h"
#include "message_generator.h"
#include "mock_stack_component.h"


using namespace raptor_networking;

/* Test data */
// typedef tcp_connection<stack_component> uut;
// struct tcp_connection_fixture : public stack_component_fixture<uut>
// {
//     tcp_connection_fixture()
//     : group_node(new uut(bottom_node, ctrl, "239.255.0.1", "127.0.0.1", 9000, 10))
//       helloworld_mg("Hello World"),
//       ab_mg("ab"),
//       send_port(7000),
//       recv_port(8000)
//     {
//         link_uut(new uut(ctrl, group_node, send_port, recv_port));
//     };

//     group *const            group_node;
//     const message_generator helloworld_mg;
//     const message_generator ab_mg;
//     const std::uint32_t     send_port;
//     const std::uint32_t     recv_port;
// };


// BOOST_FIXTURE_TEST_SUITE( tcp_connection_tests, tcp_connection_fixture )

// /* Test constructors */
// BOOST_AUTO_TEST_CASE( ctor_test )
// {
//     BOOST_CHECK(mid_node->pending_messages() == 0);
//     BOOST_CHECK(mid_node->fragment_size() == fragment_size);
// }


// /* Clone test */
// BOOST_AUTO_TEST_CASE( clone_test )
// {
//     auto cloned = mid_node->clean_clone();
//     auto *frag = dynamic_cast<uut*>(cloned.last());
//     BOOST_CHECK(frag != nullptr);
//     BOOST_CHECK(frag->pending_messages() == 0);
//     BOOST_CHECK(frag->fragment_size() == fragment_size);

//     frag->build_up_links(new mock_stack_component());
//     delete cloned.bottom();
// }

// BOOST_AUTO_TEST_SUITE_END()
