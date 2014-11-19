#ifdef STAND_ALONE
#define BOOST_TEST_MODULE udp_connection test
#endif /* #ifdef STAND_ALONE */


/* Boost headers */
#include "boost/test/unit_test.hpp"

/* Networking headers */
#include "group.h"
#include "udp_connection.h"

/* Test headers */
#include "stack_component_fixture.h"
#include "message_generator.h"
#include "mock_stack_component.h"


using namespace raptor_networking;

/* Test data */
// typedef udp_connection<stack_component> uut;
// struct udp_connection_fixture : public stack_component_fixture<uut>
// {
//     udp_connection_fixture()
//     : grp(new group(ctrl, "127.0.0.1", 10)),
//       helloworld_mg("Hello World"),
//       ab_mg("ab")
//     {
//         link_uut(new uut(ctrl, grp, 25000, 24000));
//     };

//     group *const            grp;
//     const message_generator helloworld_mg;
//     const message_generator ab_mg;
// };
// /* Open for multi cast */
//         // udp_connection(stack_controller *const ctrl, group *const grp, const boost::asio::ip::address &addr, 
//         //     const boost::asio::ip::address &multi_addr, const short port) 

// BOOST_FIXTURE_TEST_SUITE( udp_connection_tests, udp_connection_fixture )

// /* Test constructors */
// BOOST_AUTO_TEST_CASE( ctor_test )
// {
//     BOOST_CHECK(mid_node->recv_port()   == 24000);
//     BOOST_CHECK(mid_node->send_port()   == 25000);
// }


// /* Clone test */
// BOOST_AUTO_TEST_CASE( clone_test )
// {
//     auto cloned = mid_node->clean_clone();
//     auto *frag = dynamic_cast<uut*>(cloned.last());
//     BOOST_CHECK(frag != nullptr);
//     BOOST_CHECK(frag->recv_port()   == 24000);
//     BOOST_CHECK(frag->send_port()   == 25000);

//     frag->build_up_links(new mock_stack_component());
//     delete cloned.bottom();
// }

// BOOST_AUTO_TEST_SUITE_END()
