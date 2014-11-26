#ifdef STAND_ALONE
#define BOOST_TEST_MODULE tcp_connection test
#endif /* #ifdef STAND_ALONE */


/* Standard headers */
#include <mutex>

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

const std::string local_host("127.0.0.1");

/* Test data */
typedef tcp_connection<stack_component> uut;
struct tcp_connection_fixture : public stack_component_fixture<uut>
{
    tcp_connection_fixture()
    : group_node(new group(ctrl, local_host, 10)),
      socket(ioservice),
      recv_endpoint(tcp::v4(), 7000),
      acceptor(ioservice, recv_endpoint), 
      helloworld_mg("Hello World"),
      ab_mg("ab"),
      send_port(7000),
      recv_port(8000)
    {
        link_uut(new uut(ctrl, group_node, send_port, recv_port));
    };

    ~tcp_connection_fixture()
    {
        socket.close();

        /* The base fixture only deletes the connection, which in this case doesnt clean the stack above it so do it here */
        delete mid_node;
        delete top_node;
        delete group_node;
    }

    bool connect(const short port)
    {
        /* Connect to the socket */
        boost::system::error_code error;
        socket.connect(boost::asio::ip::tcp::endpoint(boost::asio::ip::address::from_string(local_host), port), error);

        return !error;
    }

    void accept(std::shared_ptr<msg_header> *const recv_header, std::shared_ptr<msg_data> *const recv_data, const short port)
    {
        /* Start async connect */
        acceptor.async_accept(socket, recv_endpoint, [this, recv_header, recv_data](const boost::system::error_code &)
            {
                receive(recv_header, recv_data);
            });
    }

    bool send(const message_generator &mg, const int seq)
    {
        /* Build the message */
        auto data(mg.send_data());
        auto header(mg.header(seq));
        std::array<char, msg_header::size()> serial_header;
        header->serialise(serial_header.data());
        std::array<boost::asio::const_buffer, 2> send_buf =
        {
            boost::asio::buffer(serial_header), 
            boost::asio::buffer(*data, header->fragment_length())
        };

        /* Send the message */
        boost::system::error_code error;
        boost::asio::write(socket, send_buf, error);
        return !error;
    }

    bool receive(std::shared_ptr<msg_header> *const recv_header, std::shared_ptr<msg_data> *const recv_data)
    {
        /* Get the header */
        std::lock_guard<std::mutex> lock(recv_mutex);
        boost::system::error_code error;
        std::vector<char> head_buf(msg_header::size());
        boost::asio::read(socket, boost::asio::buffer(head_buf), error);
        if (error)
        {
            return false;
        }
        recv_header->reset(new msg_header(head_buf.data()));

        /* Get the data */
        const int frag_len = (*recv_header)->fragment_length();
        char * data_buf = new char [frag_len];
        boost::asio::read(socket, boost::asio::buffer(data_buf, frag_len), error);

        recv_data->reset(new msg_data(data_buf, frag_len));
        socket.close();
        return !error;
    }

    group *const                    group_node;
    boost::asio::ip::tcp::socket    socket;
    tcp::endpoint                   recv_endpoint;
    boost::asio::ip::tcp::acceptor  acceptor;
    const message_generator         helloworld_mg;
    const message_generator         ab_mg;
    std::mutex                      recv_mutex;
    const short                     send_port;
    const short                     recv_port;
};


BOOST_FIXTURE_TEST_SUITE( tcp_connection_tests, tcp_connection_fixture );

/* Test constructors */
BOOST_AUTO_TEST_CASE( ctor_test )
{
    BOOST_CHECK(mid_node->recv_port() == recv_port);
    BOOST_CHECK(mid_node->send_port() == send_port);
}


/* Clone test */
BOOST_AUTO_TEST_CASE( clone_test )
{
    auto cloned = mid_node->clean_clone();
    auto *conn = dynamic_cast<uut*>(cloned.last());
    BOOST_CHECK(conn != nullptr);
    BOOST_CHECK(conn->recv_port() == recv_port);
    BOOST_CHECK(conn->send_port() == send_port);

    delete cloned.bottom();
}

/* Received test */
BOOST_AUTO_TEST_CASE( received_test )
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
    ctrl->post(msg_header::from_stack(serial_header), [this, &helloworld_header](const stack_accessor &acc)
        {
            this->mid_node->received(acc, this->helloworld_mg.receive_data(), helloworld_header);
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
            BOOST_CHECK(this->helloworld_mg.check_data(top_node->received_data()));
            BOOST_CHECK(*this->helloworld_mg.header(2) == *top_node->received_header());
        });
}

/* Start receiving tests */
BOOST_AUTO_TEST_CASE( start_receiving_test )
{
    /* Start the receiver */
    ioservice.post(std::bind(&connection<stack_component>::start_receiving, mid_node));
    ctrl->start(1);

    /* Send some data */
    BOOST_CHECK(connect(recv_port));
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
    socket.close();
    BOOST_CHECK(connect(recv_port));
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
BOOST_AUTO_TEST_CASE( send_test )
{
    /* Connect receiver */
    std::shared_ptr<msg_data>   recv_data(nullptr);
    std::shared_ptr<msg_header> recv_header(nullptr);
    accept(&recv_header, &recv_data, send_port);

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
    BOOST_CHECK(!ec);

    /* Checks */
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

    /* Connect receiver */
    accept(&recv_header, &recv_data, send_port);

    /* Get header with physical address */
    auto helloworld_header(helloworld_mg.header(15));
    group_node->add_to_group(helloworld_header->to_address(), boost::asio::ip::address::from_string(local_host), 17);

    /* Send */
    this->mid_node->send(acc, helloworld_mg.send_data(), helloworld_header, ec);
    BOOST_CHECK(!ec);

    /* Checks */
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

BOOST_AUTO_TEST_CASE( send_physical_address_test )
{
    /* Connect receiver */
    std::shared_ptr<msg_data>   recv_data(nullptr);
    std::shared_ptr<msg_header> recv_header(nullptr);
    accept(&recv_header, &recv_data, send_port);

    /* Start the receiver so the io service has work to keep it from finishing */
    ioservice.post(std::bind(&connection<stack_component>::start_receiving, mid_node));
    ctrl->start(1);

    /* Get header with physical address */
    auto ab_header(ab_mg.header(local_host, 3));

    /* Send */
    const stack_accessor acc;
    boost::system::error_code ec;
    this->mid_node->send(acc, ab_mg.send_data(), ab_header, ec);
    BOOST_CHECK(!ec);

    /* Checks */
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

    /* Connect receiver */
    accept(&recv_header, &recv_data, send_port);

    /* Get header with physical address */
    auto helloworld_header(helloworld_mg.header(local_host, 12));

    /* Send */
    this->mid_node->send(acc, helloworld_mg.send_data(), helloworld_header, ec);
    BOOST_CHECK(!ec);

    /* Checks */
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
