#ifdef STAND_ALONE
#define BOOST_TEST_MODULE udp_connection test
#endif /* #ifdef STAND_ALONE */

/* Standard headers */
#include <array>

/* Boost headers */
#include "boost/asio.hpp"
#include "boost/test/unit_test.hpp"

/* Networking headers */
#include "group.h"
#include "udp_connection.h"

/* Test headers */
#include "stack_component_fixture.h"
#include "message_generator.h"
#include "mock_stack_component.h"


using namespace raptor_networking;

const std::string local_host("0.0.0.0");
const std::string multi_cast("239.255.0.1");

/* Test data */
typedef udp_connection<stack_component> uut;
struct udp_fixture : public stack_component_fixture<uut>
{
    udp_fixture()
    : group_node(new group(ctrl, local_host, 10)),
      socket(ioservice),
      helloworld_mg("Hello World"),
      ab_mg("ab")
    {
        socket.open(udp::v4());
    }

    ~udp_fixture()
    {
        socket.close();

        /* The base fixture only deletes the connection, which in this case doesnt clean the stack above it so do it here */
        delete mid_node;
        delete top_node;
        delete group_node;
    }

    bool send(const message_generator &mg, const boost::asio::ip::address &addr, const int seq)
    {
        /* Build the message */
        auto header(mg.header(seq));
        auto data(mg.send_data());
        std::array<char, msg_header::size()> serial_header;
        header->serialise(serial_header.data());
        std::array<boost::asio::const_buffer, 2> send_buf =
        {{
            boost::asio::buffer(serial_header), 
            boost::asio::buffer(*data, header->fragment_length())
        }};

        /* Send */
        boost::system::error_code error;
        udp::endpoint send_endpoint(addr, 4000);
        socket.send_to(send_buf, send_endpoint, 0, error);

        return !error;
    }

    bool receive(std::shared_ptr<msg_header> *const recv_header, std::shared_ptr<msg_data> *const recv_data)
    {
        /* Prepare receive buffer */
        char *data_buf = new char [MAX_UDP_SIZE];
        char *head_buf = new char [msg_header::size()];
        std::array<boost::asio::mutable_buffer, 2> recv_buf =
        {
            boost::asio::buffer(head_buf, msg_header::size()),
            boost::asio::buffer(data_buf, MAX_UDP_SIZE)
        };

        /* Wait for data */
        socket.async_receive(recv_buf,
            [this, head_buf, data_buf, recv_header, recv_data](const boost::system::error_code &ec, size_t bytes)
            {
                if (ec)
                {
                    std::cout << "Error: Testbench failed to receive" << std::endl;
                    delete [] head_buf;
                    return;
                }

                std::lock_guard<std::mutex> lock(recv_mutex);

                recv_header->reset(new msg_header(head_buf));
                delete [] head_buf;

                const int frag_len = (*recv_header)->fragment_length();
                recv_data->reset(new msg_data(data_buf, frag_len));
            });

        return true;
    }

    group *const                    group_node;
    boost::asio::ip::udp::socket    socket;
    const message_generator         helloworld_mg;
    const message_generator         ab_mg;
    std::mutex                      recv_mutex;
};

struct udp_uni_cast_fixture : public udp_fixture
{
    udp_uni_cast_fixture()
    : udp_fixture(), recv_endpoint(udp::v4(), 5000)
    {
        link_uut(new uut(ctrl, group_node, 5000, 4000));
    }

    bool send(const message_generator &mg, const int seq)
    {
        return udp_fixture::send(mg, boost::asio::ip::address::from_string(local_host), seq);
    }

    bool receive(std::shared_ptr<msg_header> *const recv_header, std::shared_ptr<msg_data> *const recv_data, const bool bind = false)
    {
        /* Bind to the socket */
        if (bind)
        {
            boost::system::error_code ec;
            socket.bind(recv_endpoint, ec);
            if (ec)
            {
                std::cout << "Error: Testbench failed to bind socket" << std::endl;
                return false;
            }
        }

        return udp_fixture::receive(recv_header, recv_data);
    }

    udp::endpoint   recv_endpoint;
};

struct udp_multi_cast_fixture : public udp_fixture
{
    udp_multi_cast_fixture()
    : udp_fixture(), recv_endpoint(udp::v4(), 4000)
    {
        link_uut(new uut(ctrl, group_node, boost::asio::ip::address::from_string(local_host), boost::asio::ip::address::from_string(multi_cast), 4000));
    }

    bool send(const message_generator &mg, const int seq)
    {
        return udp_fixture::send(mg, boost::asio::ip::address::from_string(multi_cast), seq);
    }

    bool receive(std::shared_ptr<msg_header> *const recv_header, std::shared_ptr<msg_data> *const recv_data, const bool bind = false)
    {
        if (bind)
        {
            /* Create a socket that can share an address, for multicast receive */
            boost::system::error_code ec;
            socket.set_option(udp::socket::reuse_address(true));
            socket.bind(recv_endpoint, ec);
            if (ec)
            {
                std::cout << "Error: Testbench failed to bind socket" << std::endl;
                return false;
            }

            /* Join the multi cast group */
            socket.set_option(boost::asio::ip::multicast::join_group(boost::asio::ip::address::from_string(multi_cast)));
        }

        return udp_fixture::receive(recv_header, recv_data);
    }
    
    udp::endpoint   recv_endpoint;
};

BOOST_AUTO_TEST_SUITE( udp_connection_tests );

/* Test constructors */
BOOST_FIXTURE_TEST_CASE( uni_cast_ctor_test, udp_uni_cast_fixture )
{
    BOOST_CHECK(mid_node->recv_port() == 4000);
    BOOST_CHECK(mid_node->send_port() == 5000);
}

BOOST_FIXTURE_TEST_CASE( multi_cast_ctor_test, udp_uni_cast_fixture )
{
    BOOST_CHECK(mid_node->recv_port() == 4000);
    BOOST_CHECK(mid_node->send_port() == 5000);
}


/* Clone test */
BOOST_FIXTURE_TEST_CASE( uni_cast_clone_test, udp_uni_cast_fixture )
{
    auto cloned = mid_node->clean_clone();
    auto *conn = dynamic_cast<uut*>(cloned.last());
    BOOST_CHECK(conn != nullptr);
    BOOST_CHECK(conn->recv_port() == 4000);
    BOOST_CHECK(conn->send_port() == 5000);

    delete cloned.bottom();
}

BOOST_FIXTURE_TEST_CASE( multi_cast_clone_test, udp_uni_cast_fixture )
{
    auto cloned = mid_node->clean_clone();
    auto *conn = dynamic_cast<uut*>(cloned.last());
    BOOST_CHECK(conn != nullptr);
    BOOST_CHECK(conn->recv_port() == 4000);
    BOOST_CHECK(conn->send_port() == 5000);

    delete cloned.bottom();
}

/* Received test */
BOOST_FIXTURE_TEST_CASE( received_test, udp_uni_cast_fixture )
{
    /* Get a header and serialise it */
    auto ab_header(ab_mg.header(5));
    char serial_header[msg_header::size()];
    ab_header->serialise(serial_header);

    /* Send the message up the stack */
    ctrl->post(msg_header::from_stack(serial_header), [this, &ab_header](const stack_accessor &acc)
        {
            this->mid_node->received(acc, this->ab_mg.receive_data(), ab_header);
        });
    BOOST_CHECK( ioservice.poll_one());
    BOOST_CHECK(!ioservice.poll_one());

    /* Checks */
    ctrl->run(msg_header::from_stack(serial_header), [this](const stack_accessor &acc)
        {
            auto top_node = dynamic_cast<mock_stack_component*>(acc.stack_bottom());
            BOOST_REQUIRE(top_node->data_received()     == 1);
            BOOST_REQUIRE(top_node->headers_received()  == 1);
            
            /* Check message content */
            BOOST_CHECK(this->ab_mg.check_data(top_node->received_data()));
            BOOST_CHECK(*this->ab_mg.header(5) == *top_node->received_header());
        });

    /* Get a header and serialise it */
    auto helloworld_header(helloworld_mg.header(2));
    helloworld_header->serialise(serial_header);

    /* Send the message up the stack */
    ctrl->run(msg_header::from_stack(serial_header), [this, &helloworld_header](const stack_accessor &acc)
        {
            this->mid_node->received(acc, this->helloworld_mg.receive_data(), helloworld_header);
        });

    /* Checks */
    ctrl->run(msg_header::from_stack(serial_header), [this](const stack_accessor &acc)
        {
            auto top_node = dynamic_cast<mock_stack_component*>(acc.stack_bottom());
            BOOST_REQUIRE(top_node->data_received()     == 1);
            BOOST_REQUIRE(top_node->headers_received()  == 1);
            
            /* Check message content */
            BOOST_CHECK(this->helloworld_mg.check_data(top_node->received_data()));
            BOOST_CHECK(*this->helloworld_mg.header(2) == *top_node->received_header());
        });
}

/* Start receiving tests */
BOOST_FIXTURE_TEST_CASE( start_receiving_uni_cast_test, udp_uni_cast_fixture )
{
    /* Start the receiver */
    ioservice.post(std::bind(&connection<stack_component>::start_receiving, mid_node));
    ctrl->start(1);

    /* Send some data */
    BOOST_CHECK(send(ab_mg, 3));

    /* Pass the data up the stack */
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    /* Checks */
    ctrl->run(ab_mg.from_address(), [this](const stack_accessor &acc)
        {
            auto top_node = dynamic_cast<mock_stack_component*>(acc.stack_bottom());
            BOOST_REQUIRE(top_node->data_received()     == 1);
            BOOST_REQUIRE(top_node->headers_received()  == 1);
            
            /* Check message content */
            BOOST_CHECK(this->ab_mg.check_data(top_node->received_data()));

            auto recved_header(top_node->received_header());
            BOOST_CHECK(recved_header->has_physical_address());

            recved_header->has_physical_address(false);
            BOOST_CHECK(*this->ab_mg.header(3) == *recved_header);
        });

    /* Send some data */
    BOOST_CHECK(send(helloworld_mg, 12));

    /* Pass the data up the stack */
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    /* Checks */
    ctrl->run(helloworld_mg.from_address(), [this](const stack_accessor &acc)
        {
            auto top_node = dynamic_cast<mock_stack_component*>(acc.stack_bottom());
            BOOST_REQUIRE(top_node->data_received()     == 1);
            BOOST_REQUIRE(top_node->headers_received()  == 1);
            
            /* Check message content */
            BOOST_CHECK(this->helloworld_mg.check_data(top_node->received_data()));

            auto recved_header(top_node->received_header());
            BOOST_CHECK(recved_header->has_physical_address());

            recved_header->has_physical_address(false);
            BOOST_CHECK(*this->helloworld_mg.header(12) == *recved_header);
        });
}

BOOST_FIXTURE_TEST_CASE( start_receiving_multi_cast_test, udp_multi_cast_fixture )
{
    /* Start the receiver */
    ioservice.post(std::bind(&connection<stack_component>::start_receiving, mid_node));
    ctrl->start(1);

    /* Send some data */
    BOOST_CHECK(send(ab_mg, 3));

    /* Pass the data up the stack */
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    /* Checks */
    ctrl->run(ab_mg.from_address(), [this](const stack_accessor &acc)
        {
            auto top_node = dynamic_cast<mock_stack_component*>(acc.stack_bottom());
            BOOST_REQUIRE(top_node->data_received()     == 1);
            BOOST_REQUIRE(top_node->headers_received()  == 1);
            
            /* Check message content */
            BOOST_CHECK(this->ab_mg.check_data(top_node->received_data()));

            auto recved_header(top_node->received_header());
            BOOST_CHECK(recved_header->has_physical_address());

            recved_header->has_physical_address(false);
            BOOST_CHECK(*this->ab_mg.header(3) == *recved_header);
        });

    /* Send some data */
    BOOST_CHECK(send(helloworld_mg, 12));

    /* Pass the data up the stack */
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    /* Checks */
    ctrl->run(helloworld_mg.from_address(), [this](const stack_accessor &acc)
        {
            auto top_node = dynamic_cast<mock_stack_component*>(acc.stack_bottom());
            BOOST_REQUIRE(top_node->data_received()     == 1);
            BOOST_REQUIRE(top_node->headers_received()  == 1);
            
            /* Check message content */
            BOOST_CHECK(this->helloworld_mg.check_data(top_node->received_data()));

            auto recved_header(top_node->received_header());
            BOOST_CHECK(recved_header->has_physical_address());

            recved_header->has_physical_address(false);
            BOOST_CHECK(*this->helloworld_mg.header(12) == *recved_header);
        });
}

/* Send tests */
BOOST_FIXTURE_TEST_CASE( send_uni_cast_test, udp_uni_cast_fixture )
{
    /* Start the receiver */
    std::shared_ptr<msg_data>   recv_data(nullptr);
    std::shared_ptr<msg_header> recv_header(nullptr);
    BOOST_CHECK(receive(&recv_header, &recv_data, true));

    /* Start the receiver so the io service has work to keep it from finishing */
    ioservice.post(std::bind(&connection<stack_component>::start_receiving, mid_node));
    ctrl->start(1);

    /* Get header with physical address */
    auto ab_header(ab_mg.header(4));
    group_node->add_to_group(ab_header->to_address(), boost::asio::ip::address::from_string(local_host), 7);

    /* Send */
    const stack_accessor acc;
    boost::system::error_code ec;
    this->mid_node->send(acc, ab_mg.send_data(), ab_header, ec);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    {
        std::lock_guard<std::mutex> lock(recv_mutex);

        BOOST_REQUIRE(recv_data     != nullptr);
        BOOST_REQUIRE(recv_header   != nullptr);

        /* Check message content */
        BOOST_CHECK(ab_mg.check_data(recv_data));
        BOOST_CHECK(*ab_header == *recv_header);

        recv_data.reset();
        recv_header.reset();
    }

    /* Start the receiver */
    BOOST_CHECK(receive(&recv_header, &recv_data));

    /* Get header with physical address */
    auto helloworld_header(helloworld_mg.header(14));
    group_node->add_to_group(helloworld_header->to_address(), boost::asio::ip::address::from_string(local_host), 7);

    /* Send */
    this->mid_node->send(acc, helloworld_mg.send_data(), helloworld_header, ec);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    {
        std::lock_guard<std::mutex> lock(recv_mutex);

        BOOST_REQUIRE(recv_data     != nullptr);
        BOOST_REQUIRE(recv_header   != nullptr);

        /* Check message content */
        BOOST_CHECK(helloworld_mg.check_data(recv_data));
        BOOST_CHECK(*helloworld_header == *recv_header);

        recv_data.reset();
        recv_header.reset();
    }
}

BOOST_FIXTURE_TEST_CASE( send_uni_cast_physical_header_test, udp_uni_cast_fixture )
{
    /* Start the receiver */
    std::shared_ptr<msg_data>   recv_data(nullptr);
    std::shared_ptr<msg_header> recv_header(nullptr);
    BOOST_CHECK(receive(&recv_header, &recv_data, true));

    /* Start the receiver so the io service has work to keep it from finishing */
    ioservice.post(std::bind(&connection<stack_component>::start_receiving, mid_node));
    ctrl->start(1);

    /* Get header with physical address */
    auto ab_header(ab_mg.header(local_host, 4));

    /* Send */
    const stack_accessor acc;
    boost::system::error_code ec;
    this->mid_node->send(acc, ab_mg.send_data(), ab_header, ec);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    {
        std::lock_guard<std::mutex> lock(recv_mutex);

        BOOST_REQUIRE(recv_data     != nullptr);
        BOOST_REQUIRE(recv_header   != nullptr);

        /* Check message content */
        ab_header->has_physical_address(false);
        BOOST_CHECK(ab_mg.check_data(recv_data));
        BOOST_CHECK(*ab_header == *recv_header);

        recv_data.reset();
        recv_header.reset();
    }

    /* Start the receiver */
    BOOST_CHECK(receive(&recv_header, &recv_data));

    /* Get header with physical address */
    auto helloworld_header(helloworld_mg.header(local_host, 14));

    /* Send */
    this->mid_node->send(acc, helloworld_mg.send_data(), helloworld_header, ec);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    {
        std::lock_guard<std::mutex> lock(recv_mutex);

        BOOST_REQUIRE(recv_data     != nullptr);
        BOOST_REQUIRE(recv_header   != nullptr);

        /* Check message content */
        helloworld_header->has_physical_address(false);
        BOOST_CHECK(helloworld_mg.check_data(recv_data));
        BOOST_CHECK(*helloworld_header == *recv_header);

        recv_data.reset();
        recv_header.reset();
    }
}

BOOST_FIXTURE_TEST_CASE( send_multi_cast_test, udp_multi_cast_fixture )
{
    /* Start the receiver */
    std::shared_ptr<msg_data>   recv_data(nullptr);
    std::shared_ptr<msg_header> recv_header(nullptr);
    BOOST_CHECK(receive(&recv_header, &recv_data, true));

    /* Start the receiver so the io service has work to keep it from finishing */
    ioservice.post(std::bind(&connection<stack_component>::start_receiving, mid_node));
    ctrl->start(1);

    /* Get header with physical address */
    auto ab_header(ab_mg.header(4));
    group_node->add_to_group(ab_header->to_address(), boost::asio::ip::address::from_string(multi_cast), 7);

    /* Send */
    const stack_accessor acc;
    boost::system::error_code ec;
    this->mid_node->send(acc, ab_mg.send_data(), ab_header, ec);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    {
        std::lock_guard<std::mutex> lock(recv_mutex);

        BOOST_REQUIRE(recv_data     != nullptr);
        BOOST_REQUIRE(recv_header   != nullptr);

        /* Check message content */
        BOOST_CHECK(ab_mg.check_data(recv_data));
        BOOST_CHECK(*ab_header == *recv_header);

        recv_data.reset();
        recv_header.reset();
    }

    /* Start the receiver */
    BOOST_CHECK(receive(&recv_header, &recv_data));

    /* Get header with physical address */
    auto helloworld_header(helloworld_mg.header(14));
    group_node->add_to_group(helloworld_header->to_address(), boost::asio::ip::address::from_string(multi_cast), 7);

    /* Send */
    this->mid_node->send(acc, helloworld_mg.send_data(), helloworld_header, ec);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    {
        std::lock_guard<std::mutex> lock(recv_mutex);

        BOOST_REQUIRE(recv_data     != nullptr);
        BOOST_REQUIRE(recv_header   != nullptr);

        /* Check message content */
        BOOST_CHECK(helloworld_mg.check_data(recv_data));
        BOOST_CHECK(*helloworld_header == *recv_header);

        recv_data.reset();
        recv_header.reset();
    }
}

BOOST_FIXTURE_TEST_CASE( send_multi_cast_physical_header_test, udp_multi_cast_fixture )
{
    /* Start the receiver */
    std::shared_ptr<msg_data>   recv_data(nullptr);
    std::shared_ptr<msg_header> recv_header(nullptr);
    BOOST_CHECK(receive(&recv_header, &recv_data, true));

    /* Start the receiver so the io service has work to keep it from finishing */
    ioservice.post(std::bind(&connection<stack_component>::start_receiving, mid_node));
    ctrl->start(1);

    /* Get header with physical address */
    auto ab_header(ab_mg.header(multi_cast, 4));

    /* Send */
    const stack_accessor acc;
    boost::system::error_code ec;
    this->mid_node->send(acc, ab_mg.send_data(), ab_header, ec);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    {
        std::lock_guard<std::mutex> lock(recv_mutex);

        BOOST_REQUIRE(recv_data     != nullptr);
        BOOST_REQUIRE(recv_header   != nullptr);

        /* Check message content */
        ab_header->has_physical_address(false);
        BOOST_CHECK(ab_mg.check_data(recv_data));
        BOOST_CHECK(*ab_header == *recv_header);

        recv_data.reset();
        recv_header.reset();
    }

    /* Start the receiver */
    BOOST_CHECK(receive(&recv_header, &recv_data));

    /* Get header with physical address */
    auto helloworld_header(helloworld_mg.header(multi_cast, 14));

    /* Send */
    this->mid_node->send(acc, helloworld_mg.send_data(), helloworld_header, ec);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    {
        std::lock_guard<std::mutex> lock(recv_mutex);

        BOOST_REQUIRE(recv_data     != nullptr);
        BOOST_REQUIRE(recv_header   != nullptr);

        /* Check message content */
        helloworld_header->has_physical_address(false);
        BOOST_CHECK(helloworld_mg.check_data(recv_data));
        BOOST_CHECK(*helloworld_header == *recv_header);

        recv_data.reset();
        recv_header.reset();
    }
}

BOOST_AUTO_TEST_SUITE_END()
