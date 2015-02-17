#ifdef STAND_ALONE
#define BOOST_TEST_MODULE msg_data test
#endif /* #ifdef STAND_ALONE */


/* Standard headers */
#include <vector>

/* Boost headers */
#include "boost/test/unit_test.hpp"

/* Networking headers */
#include "msg_data.h"


namespace raptor_networking
{
namespace test
{
/* Test data */
struct msg_data_fixture
{
    msg_data_fixture()
        : helloworld({'h', 'e', 'l', 'l', 'o', ' ', 'w', 'o', 'r', 'l', 'd'}),
          hellomars({'h', 'e', 'l', 'l', 'o', ' ', 'm', 'a', 'r', 's'}),
          padded_hellomars({'h', 'e', 'l', 'l', 'o', ' ', 'm', 'a', 'r', 's', '!'})
        {  };

    char * new_helloworld() const
    {
        char *ret = new char [helloworld.size()];
        std::copy(helloworld.begin(), helloworld.end(), &ret[0]);
        return ret;
    }

    char * new_hellomars() const
    {
        char *ret = new char [hellomars.size()];
        std::copy(hellomars.begin(), hellomars.end(), &ret[0]);
        return ret;
    }

    char * new_padded_hellomars() const
    {
        char *ret = new char [padded_hellomars.size()];
        std::copy(padded_hellomars.begin(), padded_hellomars.end(), &ret[0]);
        return ret;
    }

    const std::vector<char> helloworld;
    const std::vector<char> hellomars;
    const std::vector<char> padded_hellomars;
};

BOOST_FIXTURE_TEST_SUITE( msg_data_tests, msg_data_fixture );

/* Test constructors */
BOOST_AUTO_TEST_CASE( ctor_test )
{
    msg_data data(new_helloworld(), helloworld.size());
    BOOST_CHECK(data.complete());
    BOOST_CHECK(data.can_peek(helloworld.size()));

    const char *const peek_data = data.peek();
    BOOST_CHECK(std::equal(helloworld.begin(), helloworld.end(), &peek_data[0]));
}


/* Check basic access for checking */
BOOST_AUTO_TEST_CASE( complete_test )
{
    msg_data data0(new_helloworld(), helloworld.size());
    BOOST_CHECK(data0.complete());

    data0.expand_slots(3, helloworld.size() * 3);
    BOOST_CHECK(!data0.complete());

    std::shared_ptr<msg_data> data1(new msg_data(new_helloworld(), helloworld.size()));
    data0.move(data1, 1);
    BOOST_CHECK(!data0.complete());

    std::shared_ptr<msg_data> data2(new msg_data(new_helloworld(), helloworld.size()));
    data0.move(data2, 2);
    BOOST_CHECK(data0.complete());
}


BOOST_AUTO_TEST_CASE( can_peek_test )
{
    /* Build with one fragment  */
    msg_data data0(new_helloworld(), helloworld.size());
    BOOST_CHECK(data0.complete());

    /* Check */
    BOOST_CHECK(data0.can_peek(helloworld.size()));
    BOOST_CHECK(!data0.can_peek(helloworld.size() + 1));

    /* Add another fragment we cant peek into it */
    std::shared_ptr<msg_data> data1(new msg_data(new_helloworld(), helloworld.size()));
    data0.expand_slots(2, helloworld.size() * 2);
    BOOST_CHECK(!data0.complete());
    data0.move(data1, 1);
    BOOST_CHECK(data0.complete());

    /* Check */
    BOOST_CHECK(data0.can_peek(helloworld.size()));
    BOOST_CHECK(!data0.can_peek(helloworld.size() + 1));
}


BOOST_AUTO_TEST_CASE( expand_slots_test )
{
    msg_data data0(new_helloworld(), helloworld.size());
    BOOST_CHECK(data0.complete());

    data0.expand_slots(2, helloworld.size() + hellomars.size());
    std::shared_ptr<msg_data> data1(new msg_data(new_hellomars(), hellomars.size()));
    data0.move(data1, 1);
    std::unique_ptr<std::istream> stream(data0.as_stream());

    std::vector<char> read0(helloworld.size());
    stream->read(read0.data(), helloworld.size());
    BOOST_CHECK(std::equal(helloworld.begin(), helloworld.end(), read0.begin()));

    std::vector<char> read1(hellomars.size());
    stream->read(read1.data(), hellomars.size());
    BOOST_CHECK(std::equal(hellomars.begin(), hellomars.end(), read1.begin()));
}


BOOST_AUTO_TEST_CASE( self_move_test )
{
    /* Build with one fragment  */
    std::shared_ptr<msg_data> data0(new msg_data(new_helloworld(), helloworld.size()));
    BOOST_CHECK(data0->complete());

    data0->expand_slots(2, helloworld.size() * 2);
    BOOST_CHECK(!data0->complete());
    data0->move(data0, 1);
    BOOST_CHECK(!data0->complete());
    BOOST_CHECK(!data0->can_peek(1));

    std::shared_ptr<msg_data> data1(new msg_data(new_padded_hellomars(), padded_hellomars.size()));
    data0->move(data1, 0);
    BOOST_CHECK(data0->can_peek(1));

    const char *const peek_data = data0->peek();
    BOOST_CHECK(std::equal(padded_hellomars.begin(), padded_hellomars.end(), &peek_data[0]));
}

BOOST_AUTO_TEST_SUITE_END()
}; /* namespace test */
}; /* namespace raptor_networking */
