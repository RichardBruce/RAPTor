#ifdef STAND_ALONE
#define BOOST_TEST_MODULE vector_stream test
#endif /* #ifdef STAND_ALONE */


/* Standard headers */
#include <algorithm>
#include <vector>

/* Boost headers */
#include "boost/test/unit_test.hpp"

/* Networking headers */
#include "vector_stream.h"


namespace raptor_networking
{
namespace test
{
/* Test data */
struct vector_stream_fixture
{
    public :
        vector_stream_fixture()
            : long_message_size(5000),
              helloworld({'h', 'e', 'l', 'l', 'o', ' ', 'w', 'o', 'r', 'l', 'd'}),
              hellomars({'h', 'e', 'l', 'l', 'o', ' ', 'm', 'a', 'r', 's'}),
              long_message(long_message_size, 'r'),
              helloworld_ptr(new std::vector<std::unique_ptr<const char[]>>()),
              hellomars_ptr(new std::vector<std::unique_ptr<const char[]>>()),
              long_message_ptr(new std::vector<std::unique_ptr<const char[]>>()),
              segmented_data_ptr(new std::vector<std::unique_ptr<const char[]>>())
            {
                helloworld_ptr->emplace_back(to_vector_buffer(helloworld));
                hellomars_ptr->emplace_back(to_vector_buffer(hellomars));
                long_message_ptr->emplace_back(to_vector_buffer(long_message));

                segmented_data_ptr->emplace_back(to_vector_buffer(long_message));
                segmented_data_ptr->emplace_back(to_vector_buffer(helloworld));
                segmented_data_ptr->emplace_back(to_vector_buffer(hellomars));
            };

        typedef std::shared_ptr<std::vector<std::unique_ptr<const char[]>>> vector_ptr;
        const int               long_message_size;
        const std::vector<char> helloworld;
        const std::vector<char> hellomars;
        const std::vector<char> long_message;
        vector_ptr              helloworld_ptr;
        vector_ptr              hellomars_ptr;
        vector_ptr              long_message_ptr;
        vector_ptr              segmented_data_ptr;

    private :
        const char* to_vector_buffer(const std::vector<char> &data)
        {
            char *char_buf = new char[data.size()];
            std::copy(&data[0], &data[data.size()], char_buf);
            return char_buf;
        } 
};

BOOST_FIXTURE_TEST_SUITE( vector_stream_tests, vector_stream_fixture );

/* Test ivector_stream */
/* Test constructors */
BOOST_AUTO_TEST_CASE( istream_ctor_test )
{
    ivector_stream stream(&helloworld);

    std::vector<char> read(helloworld.size());
    BOOST_CHECK(stream.read(read.data(), helloworld.size()) == static_cast<std::streamsize>(helloworld.size()));
    BOOST_CHECK(stream.read(read.data(), 1) == -1);
    BOOST_CHECK(std::equal(helloworld.begin(), helloworld.end(), read.begin()));
}


BOOST_AUTO_TEST_CASE( istream_move_ctor_0_test )
{
    ivector_stream stream0(&helloworld);
    ivector_stream stream1(std::move(stream0));

    std::vector<char> read0(helloworld.size());
    BOOST_CHECK(stream0.read(read0.data(), helloworld.size()) == static_cast<std::streamsize>(helloworld.size()));
    BOOST_CHECK(stream0.read(read0.data(), 1) == -1);
    BOOST_CHECK(std::equal(helloworld.begin(), helloworld.end(), read0.begin()));

    std::vector<char> read1(helloworld.size());
    BOOST_CHECK(stream1.read(read1.data(), helloworld.size()) == static_cast<std::streamsize>(helloworld.size()));
    BOOST_CHECK(stream1.read(read1.data(), 1) == -1);
    BOOST_CHECK(std::equal(helloworld.begin(), helloworld.end(), read1.begin()));
}


BOOST_AUTO_TEST_CASE( istream_move_ctor_1_test )
{
    /* Build the stream and read the first 8 bytes */
    const int front_size = 8;
    char read_front[front_size];
    ivector_stream stream0(&helloworld);
    stream0.read(read_front, front_size);

    /* Move */
    ivector_stream stream1(std::move(stream0));

    /* Check */
    std::vector<char> read0(helloworld.size() - front_size);
    BOOST_CHECK(stream0.read(read0.data(), helloworld.size() - front_size) == static_cast<std::streamsize>(helloworld.size() - front_size));
    BOOST_CHECK(stream0.read(read0.data(), 1) == -1);
    BOOST_CHECK(std::equal(&helloworld[front_size], &helloworld[helloworld.size()], &read0[0]));

    std::vector<char> read1(helloworld.size() - front_size);
    BOOST_CHECK(stream1.read(read1.data(), helloworld.size() - front_size) == static_cast<std::streamsize>(helloworld.size() - front_size));
    BOOST_CHECK(stream1.read(read1.data(), 1) == -1);
    BOOST_CHECK(std::equal(&helloworld[front_size], &helloworld[helloworld.size()], &read1[0]));
}


BOOST_AUTO_TEST_CASE( istream_copy_ctor_0_test )
{
    ivector_stream stream0(&helloworld);
    ivector_stream stream1(stream0);

    std::vector<char> read0(helloworld.size());
    BOOST_CHECK(stream0.read(read0.data(), helloworld.size()) == static_cast<std::streamsize>(helloworld.size()));
    BOOST_CHECK(stream0.read(read0.data(), 1) == -1);
    BOOST_CHECK(std::equal(helloworld.begin(), helloworld.end(), read0.begin()));

    std::vector<char> read1(helloworld.size());
    BOOST_CHECK(stream1.read(read1.data(), helloworld.size()) == static_cast<std::streamsize>(helloworld.size()));
    BOOST_CHECK(stream1.read(read1.data(), 1) == -1);
    BOOST_CHECK(std::equal(helloworld.begin(), helloworld.end(), read1.begin()));
}


BOOST_AUTO_TEST_CASE( istream_copy_ctor_1_test )
{
    /* Build the stream and read the first 8 bytes */
    const int front_size = 8;
    char read_front[front_size];
    ivector_stream stream0(&helloworld);
    stream0.read(read_front, front_size);

    /* Copy */
    ivector_stream stream1(stream0);

    /* Check */
    std::vector<char> read0(helloworld.size() - front_size);
    BOOST_CHECK(stream0.read(read0.data(), helloworld.size() - front_size) == static_cast<std::streamsize>(helloworld.size() - front_size));
    BOOST_CHECK(stream0.read(read0.data(), 1) == -1);
    BOOST_CHECK(std::equal(&helloworld[front_size], &helloworld[helloworld.size()], &read0[0]));

    std::vector<char> read1(helloworld.size() - front_size);
    BOOST_CHECK(stream1.read(read1.data(), helloworld.size() - front_size) == static_cast<std::streamsize>(helloworld.size() - front_size));
    BOOST_CHECK(stream1.read(read1.data(), 1) == -1);
    BOOST_CHECK(std::equal(&helloworld[front_size], &helloworld[helloworld.size()], &read1[0]));
}


/* Test reading data */
BOOST_AUTO_TEST_CASE( istream_read_test )
{
    const int leave_size = 5;
    const int extra_size = 6;
    ivector_stream stream0(&long_message);

    std::vector<char> read0(long_message_size - leave_size);
    BOOST_CHECK(stream0.read(read0.data(), read0.size()) == static_cast<std::streamsize>(read0.size()));
    BOOST_CHECK(std::equal(&long_message[0], &long_message[long_message_size - leave_size], &read0[0]));

    std::vector<char> read1(leave_size);
    BOOST_CHECK(stream0.read(read1.data(), leave_size + extra_size) == static_cast<std::streamsize>(leave_size));
    BOOST_CHECK(stream0.read(read1.data(), 1) == -1);
    BOOST_CHECK(std::equal(&long_message[long_message_size - leave_size], &long_message[long_message_size], &read1[0]));
}


/* Test ovector_stream */
/* Test constructor */
BOOST_AUTO_TEST_CASE( ostream_ctor_test )
{
    std::vector<char> write_buf;
    ovector_stream stream0(&write_buf);

    stream0.write(helloworld.data(), helloworld.size());
    BOOST_CHECK(std::equal(write_buf.begin(), write_buf.end(), helloworld.begin()));
}


/* Test writing */
BOOST_AUTO_TEST_CASE( ostream_write_test )
{
    std::vector<char> write_buf;
    ovector_stream stream0(&write_buf);

    stream0.write(helloworld.data(), helloworld.size());
    BOOST_CHECK(std::equal(write_buf.begin(), write_buf.end(), helloworld.begin()));

    stream0.write(hellomars.data(), hellomars.size());
    BOOST_CHECK(std::equal(&write_buf[0], &write_buf[helloworld.size()], &helloworld[0]));
    BOOST_CHECK(std::equal(&write_buf[helloworld.size()], &write_buf[write_buf.size()], &hellomars[0]));
}


/* Test ivectorbuf_stream */
/* Test constructors */
BOOST_AUTO_TEST_CASE( ibuf_stream_ctor_test )
{
    ivectorbuf_stream stream(helloworld_ptr, helloworld.size(), helloworld.size());

    std::vector<char> read(helloworld.size());
    BOOST_CHECK(stream.read(read.data(), helloworld.size()) == static_cast<std::streamsize>(helloworld.size()));
    BOOST_CHECK(stream.read(read.data(), 1) == -1);
    BOOST_CHECK(std::equal(helloworld.begin(), helloworld.end(), read.begin()));
}


BOOST_AUTO_TEST_CASE( ibuf_stream_move_ctor_0_test )
{
    ivectorbuf_stream stream0(helloworld_ptr, helloworld.size(), helloworld.size());
    ivectorbuf_stream stream1(std::move(stream0));

    std::vector<char> read0(helloworld.size());
    BOOST_CHECK(stream0.read(read0.data(), helloworld.size()) == static_cast<std::streamsize>(helloworld.size()));
    BOOST_CHECK(stream0.read(read0.data(), 1) == -1);
    BOOST_CHECK(std::equal(helloworld.begin(), helloworld.end(), read0.begin()));

    std::vector<char> read1(helloworld.size());
    BOOST_CHECK(stream1.read(read1.data(), helloworld.size()) == static_cast<std::streamsize>(helloworld.size()));
    BOOST_CHECK(stream1.read(read1.data(), 1) == -1);
    BOOST_CHECK(std::equal(helloworld.begin(), helloworld.end(), read1.begin()));
}


BOOST_AUTO_TEST_CASE( ibuf_stream_move_ctor_1_test )
{
    /* Build the stream and read the first 8 bytes */
    const int front_size = 8;
    char read_front[front_size];
    ivectorbuf_stream stream0(helloworld_ptr, helloworld.size(), helloworld.size());
    stream0.read(read_front, front_size);

    /* Move */
    ivectorbuf_stream stream1(std::move(stream0));

    /* Check */
    std::vector<char> read0(helloworld.size() - front_size);
    BOOST_CHECK(stream0.read(read0.data(), helloworld.size() - front_size) == static_cast<std::streamsize>(helloworld.size() - front_size));
    BOOST_CHECK(stream0.read(read0.data(), 1) == -1);
    BOOST_CHECK(std::equal(&helloworld[front_size], &helloworld[helloworld.size()], &read0[0]));

    std::vector<char> read1(helloworld.size() - front_size);
    BOOST_CHECK(stream1.read(read1.data(), helloworld.size() - front_size) == static_cast<std::streamsize>(helloworld.size() - front_size));
    BOOST_CHECK(stream1.read(read1.data(), 1) == -1);
    BOOST_CHECK(std::equal(&helloworld[front_size], &helloworld[helloworld.size()], &read1[0]));
}


BOOST_AUTO_TEST_CASE( ibuf_stream_copy_ctor_0_test )
{
    ivectorbuf_stream stream0(helloworld_ptr, helloworld.size(), helloworld.size());
    ivectorbuf_stream stream1(stream0);

    std::vector<char> read0(helloworld.size());
    BOOST_CHECK(stream0.read(read0.data(), helloworld.size()) == static_cast<std::streamsize>(helloworld.size()));
    BOOST_CHECK(stream0.read(read0.data(), 1) == -1);
    BOOST_CHECK(std::equal(helloworld.begin(), helloworld.end(), read0.begin()));

    std::vector<char> read1(helloworld.size());
    BOOST_CHECK(stream1.read(read1.data(), helloworld.size()) == static_cast<std::streamsize>(helloworld.size()));
    BOOST_CHECK(stream1.read(read1.data(), 1) == -1);
    BOOST_CHECK(std::equal(helloworld.begin(), helloworld.end(), read1.begin()));
}


BOOST_AUTO_TEST_CASE( ibuf_stream_copy_ctor_1_test )
{
    /* Build the stream and read the first 8 bytes */
    const int front_size = 8;
    char read_front[front_size];
    ivectorbuf_stream stream0(helloworld_ptr, helloworld.size(), helloworld.size());
    stream0.read(read_front, front_size);

    /* Copy */
    ivectorbuf_stream stream1(stream0);

    /* Check */
    std::vector<char> read0(helloworld.size() - front_size);
    BOOST_CHECK(stream0.read(read0.data(), helloworld.size() - front_size) == static_cast<std::streamsize>(helloworld.size() - front_size));
    BOOST_CHECK(stream0.read(read0.data(), 1) == -1);
    BOOST_CHECK(std::equal(&helloworld[front_size], &helloworld[helloworld.size()], &read0[0]));

    std::vector<char> read1(helloworld.size() - front_size);
    BOOST_CHECK(stream1.read(read1.data(), helloworld.size() - front_size) == static_cast<std::streamsize>(helloworld.size() - front_size));
    BOOST_CHECK(stream1.read(read1.data(), 1) == -1);
    BOOST_CHECK(std::equal(&helloworld[front_size], &helloworld[helloworld.size()], &read1[0]));
}


BOOST_AUTO_TEST_CASE( ibuf_stream_read_test )
{
    const int frag_size = std::min(std::min(helloworld.size(), hellomars.size()), long_message.size());
    const int total_size = segmented_data_ptr->size() * frag_size;
    ivectorbuf_stream stream0(segmented_data_ptr, total_size, frag_size);
    
    std::vector<char> read0(total_size);
    BOOST_CHECK(stream0.read(read0.data(), total_size) == total_size);
    BOOST_CHECK(stream0.read(read0.data(), 1) == -1);
    BOOST_CHECK(std::equal(&read0[            0], &read0[    frag_size], &long_message[0]));
    BOOST_CHECK(std::equal(&read0[    frag_size], &read0[2 * frag_size], &helloworld[0]));
    BOOST_CHECK(std::equal(&read0[2 * frag_size], &read0[3 * frag_size], &hellomars[0]));
}


BOOST_AUTO_TEST_CASE( ibuf_stream_read_small_chunks_test )
{
    const int frag_size = std::min(std::min(helloworld.size(), hellomars.size()), long_message.size());
    const int total_size = segmented_data_ptr->size() * frag_size;
    ivectorbuf_stream stream0(segmented_data_ptr, total_size, frag_size);
    
    const int chunk_size = 4;
    std::vector<char> read0(total_size);
    for (int i = 0; i < total_size; i += chunk_size)
    {
        BOOST_CHECK(stream0.read(&read0[i], chunk_size) == std::min(chunk_size, total_size - i));
    }
    BOOST_CHECK(stream0.read(read0.data(), 1) == -1);
    BOOST_CHECK(std::equal(&read0[            0], &read0[    frag_size], &long_message[0]));
    BOOST_CHECK(std::equal(&read0[    frag_size], &read0[2 * frag_size], &helloworld[0]));
    BOOST_CHECK(std::equal(&read0[2 * frag_size], &read0[3 * frag_size], &hellomars[0]));
}


BOOST_AUTO_TEST_CASE( ibuf_stream_read_small_last_fragment_test )
{
    const int frag_size = helloworld.size();
    const int total_size = (2 * frag_size) + hellomars.size();
    ivectorbuf_stream stream0(segmented_data_ptr, total_size, frag_size);
    
    std::vector<char> read0(total_size);
    BOOST_CHECK(stream0.read(read0.data(), total_size) == total_size);
    BOOST_CHECK(stream0.read(read0.data(), 1) == -1);
    BOOST_CHECK(std::equal(&read0[            0], &read0[    frag_size], &long_message[0]));
    BOOST_CHECK(std::equal(&read0[    frag_size], &read0[2 * frag_size], &helloworld[0]));
    BOOST_CHECK(std::equal(&read0[2 * frag_size], &read0[2 * frag_size + hellomars.size()], &hellomars[0]));
}

BOOST_AUTO_TEST_SUITE_END()
}; /* namespace test */
}; /* namespace raptor_networking */
