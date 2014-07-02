#ifdef STAND_ALONE
#define BOOST_TEST_MODULE udp_connection test
#endif /* #ifdef STAND_ALONE */


/* Boost headers */
#include "boost/test/unit_test.hpp"

/* Networking headers */
#include "udp_connection.h"

/* Test headers */
#include "stack_component_fixture.h"
#include "message_generator.h"
#include "mock_stack_component.h"


using namespace raptor_networking;

/* Test data */
typedef udp_connection<stack_component> uut;
struct udp_connection_fixture : public stack_component_fixture<uut>
{
    udp_connection_fixture()
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


BOOST_FIXTURE_TEST_SUITE( udp_connection_tests, udp_connection_fixture )

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

BOOST_AUTO_TEST_SUITE_END()
