#ifdef STAND_ALONE
#define BOOST_TEST_MODULE msg_header test
#endif /* #ifdef STAND_ALONE */


/* Standard headers */
#include <string>

/* Boost headers */
#include "boost/test/unit_test.hpp"
#include "boost/asio/ip/address.hpp"
#include "boost/uuid/uuid.hpp"
#include "boost/uuid/uuid_io.hpp"
#include "boost/uuid/uuid_generators.hpp"

/* Networking headers */
#include "msg_header.h"


using boost::uuids::uuid;
using boost::asio::ip::address;
namespace raptor_networking
{
namespace test
{
/* Test data */
struct msg_header_fixture
{
    msg_header_fixture()
        : _ran_gen(), 
          _uuid_gen(&_ran_gen), 
          nil_log_addr(boost::uuids::nil_uuid()),
          phy_addr0(address::from_string("0.0.0.0")),
          phy_addr1(address::from_string("255.255.255.255")),
          phy_addr2(address::from_string("123.123.123.123"))
    {
        _ran_gen.seed(time(NULL));
        log_addr0  = _uuid_gen();
        log_addr1  = _uuid_gen();
        log_addr2  = _uuid_gen();
    };

    typedef boost::uuids::basic_random_generator<boost::mt19937> uuid_gen;

    boost::mt19937  _ran_gen;   /* Random number generator to drive the uuid generator  */
    uuid_gen        _uuid_gen;  /* Uuid generator for process ids                       */
    uuid            log_addr0;
    uuid            log_addr1;
    uuid            log_addr2;
    uuid            nil_log_addr;
    const address   phy_addr0;
    const address   phy_addr1;
    const address   phy_addr2;
};

BOOST_FIXTURE_TEST_SUITE( msg_header_tests, msg_header_fixture );

/* Test constructors */
BOOST_AUTO_TEST_CASE( ctor_0_test )
{
    msg_header header(phy_addr0, 98, 765, 345, false);

    BOOST_CHECK(*header.physical_address()      == phy_addr0);
    BOOST_CHECK( header.to_address()            == nil_log_addr);
    BOOST_CHECK( header.group_address()         == nil_log_addr);
    BOOST_CHECK( header.from_address()          == nil_log_addr);
    BOOST_CHECK( header.sequence()              == 98);
    BOOST_CHECK( header.length()                == 765);
    BOOST_CHECK( header.fragment()              == 0);
    BOOST_CHECK( header.fragment_length()       == 765);
    BOOST_CHECK( header.resend_id()             == 0);
    BOOST_CHECK( header.response_to()           == 345);
    BOOST_CHECK( header.is_resposne()           == false);
    BOOST_CHECK( header.has_physical_address()  == true);
}


BOOST_AUTO_TEST_CASE( ctor_1_test )
{
    msg_header header(log_addr0, 123, 456, 876, true);

    BOOST_CHECK( header.physical_address()      == nullptr);
    BOOST_CHECK( header.to_address()            == log_addr0);
    BOOST_CHECK( header.group_address()         == nil_log_addr);
    BOOST_CHECK( header.from_address()          == nil_log_addr);
    BOOST_CHECK( header.sequence()              == 123);
    BOOST_CHECK( header.length()                == 456);
    BOOST_CHECK( header.fragment()              == 0);
    BOOST_CHECK( header.fragment_length()       == 456);
    BOOST_CHECK( header.resend_id()             == 0);
    BOOST_CHECK( header.response_to()           == 876);
    BOOST_CHECK( header.is_resposne()           == true);
    BOOST_CHECK( header.has_physical_address()  == false);
}


BOOST_AUTO_TEST_CASE( ctor_2_test )
{
    msg_header header(log_addr1, log_addr2, 654, 12, 9, true);

    BOOST_CHECK( header.physical_address()      == nullptr);
    BOOST_CHECK( header.to_address()            == log_addr1);
    BOOST_CHECK( header.group_address()         == log_addr2);
    BOOST_CHECK( header.from_address()          == nil_log_addr);
    BOOST_CHECK( header.sequence()              == 654);
    BOOST_CHECK( header.length()                == 12);
    BOOST_CHECK( header.fragment()              == 0);
    BOOST_CHECK( header.fragment_length()       == 12);
    BOOST_CHECK( header.resend_id()             == 0);
    BOOST_CHECK( header.response_to()           == 9);
    BOOST_CHECK( header.is_resposne()           == true);
    BOOST_CHECK( header.has_physical_address()  == false);
}


BOOST_AUTO_TEST_CASE( ctor_3_test )
{
    msg_header header(phy_addr0, 98, 765);

    BOOST_CHECK(*header.physical_address()      == phy_addr0);
    BOOST_CHECK( header.to_address()            == nil_log_addr);
    BOOST_CHECK( header.group_address()         == nil_log_addr);
    BOOST_CHECK( header.from_address()          == nil_log_addr);
    BOOST_CHECK( header.sequence()              == 98);
    BOOST_CHECK( header.length()                == 765);
    BOOST_CHECK( header.fragment()              == 0);
    BOOST_CHECK( header.fragment_length()       == 765);
    BOOST_CHECK( header.resend_id()             == 0);
    BOOST_CHECK( header.response_to()           == 0);
    BOOST_CHECK( header.is_resposne()           == false);
    BOOST_CHECK( header.has_physical_address()  == true);

}


BOOST_AUTO_TEST_CASE( ctor_4_test )
{
    msg_header header(log_addr0, 123, 456);

    BOOST_CHECK( header.physical_address()      == nullptr);
    BOOST_CHECK( header.to_address()            == log_addr0);
    BOOST_CHECK( header.group_address()         == nil_log_addr);
    BOOST_CHECK( header.from_address()          == nil_log_addr);
    BOOST_CHECK( header.sequence()              == 123);
    BOOST_CHECK( header.length()                == 456);
    BOOST_CHECK( header.fragment()              == 0);
    BOOST_CHECK( header.fragment_length()       == 456);
    BOOST_CHECK( header.resend_id()             == 0);
    BOOST_CHECK( header.response_to()           == 0);
    BOOST_CHECK( header.is_resposne()           == false);
    BOOST_CHECK( header.has_physical_address()  == false);
}


BOOST_AUTO_TEST_CASE( ctor_5_test )
{
    msg_header header(log_addr1, log_addr2, 654, 12);

    BOOST_CHECK( header.physical_address()      == nullptr);
    BOOST_CHECK( header.to_address()            == log_addr1);
    BOOST_CHECK( header.group_address()         == log_addr2);
    BOOST_CHECK( header.from_address()          == nil_log_addr);
    BOOST_CHECK( header.sequence()              == 654);
    BOOST_CHECK( header.length()                == 12);
    BOOST_CHECK( header.fragment()              == 0);
    BOOST_CHECK( header.fragment_length()       == 12);
    BOOST_CHECK( header.resend_id()             == 0);
    BOOST_CHECK( header.response_to()           == 0);
    BOOST_CHECK( header.is_resposne()           == false);
    BOOST_CHECK( header.has_physical_address()  == false);
}


/* Test copy constructors */
BOOST_AUTO_TEST_CASE( copy_ctor_0_test )
{
    msg_header orig_header(phy_addr0, 98, 765, 345, false);
    msg_header header(orig_header);

    BOOST_CHECK(*header.physical_address()      == phy_addr0);
    BOOST_CHECK( header.to_address()            == nil_log_addr);
    BOOST_CHECK( header.group_address()         == nil_log_addr);
    BOOST_CHECK( header.from_address()          == nil_log_addr);
    BOOST_CHECK( header.sequence()              == 98);
    BOOST_CHECK( header.length()                == 765);
    BOOST_CHECK( header.fragment()              == 0);
    BOOST_CHECK( header.fragment_length()       == 765);
    BOOST_CHECK( header.resend_id()             == 0);
    BOOST_CHECK( header.response_to()           == 345);
    BOOST_CHECK( header.is_resposne()           == false);
    BOOST_CHECK( header.has_physical_address()  == true);
}


BOOST_AUTO_TEST_CASE( copy_ctor_1_test )
{
    msg_header orig_header(log_addr0, 123, 456, 876, true);
    msg_header header(orig_header);

    BOOST_CHECK( header.physical_address()      == nullptr);
    BOOST_CHECK( header.to_address()            == log_addr0);
    BOOST_CHECK( header.group_address()         == nil_log_addr);
    BOOST_CHECK( header.from_address()          == nil_log_addr);
    BOOST_CHECK( header.sequence()              == 123);
    BOOST_CHECK( header.length()                == 456);
    BOOST_CHECK( header.fragment()              == 0);
    BOOST_CHECK( header.fragment_length()       == 456);
    BOOST_CHECK( header.resend_id()             == 0);
    BOOST_CHECK( header.response_to()           == 876);
    BOOST_CHECK( header.is_resposne()           == true);
    BOOST_CHECK( header.has_physical_address()  == false);
}


BOOST_AUTO_TEST_CASE( copy_ctor_2_test )
{
    msg_header orig_header(log_addr1, log_addr2, 654, 12, 9, true);
    msg_header header(orig_header);

    BOOST_CHECK( header.physical_address()      == nullptr);
    BOOST_CHECK( header.to_address()            == log_addr1);
    BOOST_CHECK( header.group_address()         == log_addr2);
    BOOST_CHECK( header.from_address()          == nil_log_addr);
    BOOST_CHECK( header.sequence()              == 654);
    BOOST_CHECK( header.length()                == 12);
    BOOST_CHECK( header.fragment()              == 0);
    BOOST_CHECK( header.fragment_length()       == 12);
    BOOST_CHECK( header.resend_id()             == 0);
    BOOST_CHECK( header.response_to()           == 9);
    BOOST_CHECK( header.is_resposne()           == true);
    BOOST_CHECK( header.has_physical_address()  == false);
}


/* Test serialisation and deserialisation */
BOOST_AUTO_TEST_CASE( serialise_deserialise_test )
{
    msg_header orig_header(log_addr1, log_addr2, 654, 12, 9, true);

    /* Serialise */
    char serial[msg_header::size()];
    orig_header.serialise(serial);

    /* Deserialise */
    msg_header header(serial);

    BOOST_CHECK(orig_header == header);
}


BOOST_AUTO_TEST_CASE( serialise_deserialise_with_phy_addr_test )
{
    msg_header orig_header(phy_addr1, 654, 12);

    /* Serialise */
    char serial[msg_header::size()];
    orig_header.serialise(serial);

    /* Deserialise */
    msg_header header(serial, phy_addr1);

    BOOST_CHECK(orig_header == header);
}


/* Test basic getters and setters */        
BOOST_AUTO_TEST_CASE( from_addr_test )
{
    msg_header header(log_addr1, log_addr2, 654, 12);
    header.from_address(log_addr0);
    BOOST_CHECK(header.from_address() == log_addr0);
}


BOOST_AUTO_TEST_CASE( length_test )
{
    msg_header header(log_addr1, log_addr2, 654, 12);
    header.length(5);
    BOOST_CHECK(header.length() == 5);
}


BOOST_AUTO_TEST_CASE( fragment_test )
{
    msg_header header(log_addr1, log_addr2, 654, 12);
    header.fragment(15);
    BOOST_CHECK(header.fragment() == 15);
}


BOOST_AUTO_TEST_CASE( fragment_length_test )
{
    msg_header header(log_addr1, log_addr2, 654, 12);
    BOOST_CHECK(header.fragment_length() == header.length());

    header.fragment_length(7);
    BOOST_CHECK(header.fragment_length() == 7);
    BOOST_CHECK(header.length() == 12);
}


BOOST_AUTO_TEST_CASE( resend_id_test )
{
    msg_header header(log_addr1, log_addr2, 654, 12);
    header.resend_id(98);
    BOOST_CHECK(header.resend_id() == 98);
}


BOOST_AUTO_TEST_CASE( has_physical_address_test )
{
    msg_header header(phy_addr0, 98, 765, 345, false);
    header.has_physical_address(false);
    BOOST_CHECK(header.has_physical_address() == false);
    BOOST_CHECK(header.physical_address() == nullptr);
}


/* Stack determination functions */
BOOST_AUTO_TEST_CASE( group_to_stack_test )
{
    msg_header header(log_addr1, log_addr2, 654, 12);
    BOOST_CHECK(header.to_stack() == log_addr2);
}

        
BOOST_AUTO_TEST_CASE( no_group_to_stack_test )
{
    msg_header header(log_addr1, 654, 12);
    BOOST_CHECK(header.to_stack() == log_addr1);
}


BOOST_AUTO_TEST_CASE( group_reply_stack_test )
{
    msg_header header(log_addr1, log_addr2, 654, 12);
    header.from_address(log_addr0);
    BOOST_CHECK(header.reply_stack() == log_addr2);
}

        
BOOST_AUTO_TEST_CASE( no_group_reply_stack_test )
{
    msg_header header(log_addr1, 654, 12);
    header.from_address(log_addr0);
    BOOST_CHECK(header.reply_stack() == log_addr0);
}


BOOST_AUTO_TEST_CASE( group_from_stack_test )
{
    msg_header header(log_addr1, log_addr2, 654, 12);
    char serial[msg_header::size()];
    header.serialise(serial);

    BOOST_CHECK(msg_header::from_stack(serial) == log_addr2);
}


BOOST_AUTO_TEST_CASE( no_group_from_stack_test )
{
    msg_header header(log_addr1, 654, 12);
    header.from_address(log_addr2);
    
    char serial[msg_header::size()];
    header.serialise(serial);

    BOOST_CHECK(msg_header::from_stack(serial) == log_addr2);
}


/* Test comparison */
BOOST_AUTO_TEST_CASE( comparison_test )
{
    msg_header header0(log_addr1, 654, 12);
    msg_header header1(log_addr0, 654, 12);
    BOOST_CHECK((header0 == header1) == false);

    msg_header header2(log_addr1, log_addr0, 654, 12);
    BOOST_CHECK((header0 == header2) == false);

    msg_header header3(log_addr1, 654, 12);
    header3.from_address(log_addr2);
    BOOST_CHECK((header0 == header3) == false);

    msg_header header4(log_addr1, 653, 12);
    BOOST_CHECK((header0 == header4) == false);

    msg_header header5(log_addr1, 654, 10);
    BOOST_CHECK((header0 == header5) == false);

    msg_header header6(log_addr1, 654, 12);
    header6.fragment(2);
    BOOST_CHECK((header0 == header6) == false);

    msg_header header7(log_addr1, 654, 12);
    header7.fragment_length(86);
    BOOST_CHECK((header0 == header7) == false);

    msg_header header8(log_addr1, 654, 12);
    header8.resend_id(39);
    BOOST_CHECK((header0 == header8) == false);

    msg_header header9(log_addr1, 654, 12, 1, true);
    BOOST_CHECK((header0 == header9) == false);

    msg_header header10(log_addr1, 654, 12, 0, true);
    BOOST_CHECK((header0 == header10) == false);

    msg_header header11(phy_addr0, 654, 12);
    msg_header header12(nil_log_addr, 654, 12);
    BOOST_CHECK((header11 == header12) == false);

    msg_header header13(phy_addr1, 654, 12);
    BOOST_CHECK((header11 == header13) == false);

    msg_header header14(log_addr1, 654, 12);
    BOOST_CHECK((header0 == header14) == true);
}

BOOST_AUTO_TEST_SUITE_END()
}; /* namespace test */
}; /* namespace raptor_networking */
