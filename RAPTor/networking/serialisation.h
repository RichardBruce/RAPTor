#ifndef __SERIALISATION_H__
#define __SERIALISATION_H__

/* Standard headers */
#include <memory>
#include <sstream>
#include <string>
#include <vector>

/* Boost headers */
#include "boost/iostreams/stream.hpp"
#include "boost/iostreams/device/back_inserter.hpp"

#include "boost/archive/binary_iarchive.hpp"
#include "boost/archive/binary_oarchive.hpp"
#include "boost/serialization/split_free.hpp"
#include "boost/serialization/base_object.hpp"
#include "boost/serialization/list.hpp"
#include "boost/serialization/string.hpp"
#include "boost/serialization/vector.hpp"

#include "boost/asio/ip/address.hpp"
#include "boost/asio/ip/address_v4.hpp"
#include "boost/asio/ip/address_v6.hpp"

/* Networking headers */
#include "stack_component.h"
#include "vector_stream.h"


namespace raptor_networking
{
/* Serialisation */
/* Template method to deserialise and append */
template<class T>
std::vector<char> *const serialise(std::vector<char> *const serial_msg, const T &msg)
{
    /* Build  back inserter */
    boost::iostreams::back_insert_device<std::vector<char>> inserter(*serial_msg);
    boost::iostreams::stream<boost::iostreams::back_insert_device<std::vector<char>>> s(inserter);
    boost::archive::binary_oarchive oa(s);

    /* Serialise and flush the stream */
    oa << msg;
    s.flush();

    return serial_msg;
}

/* Specialisation for datatypes not requiring serialisation */
template<> std::vector<char> *const serialise<std::string>(std::vector<char> *const serial_msg, const std::string &msg)
{
    std::cout << "sending string" << std::endl;
    serial_msg->insert(serial_msg->end(), msg.begin(), msg.end());
    return serial_msg;
}

template<> std::vector<char> *const serialise<std::vector<char>>(std::vector<char> *const serial_msg, const std::vector<char> &msg)
{
    std::cout << "sending vector" << std::endl;
    serial_msg->insert(serial_msg->end(), msg.begin(), msg.end());
    return serial_msg;
}

/* Template method to deserialise */
template<class T>
std::vector<char> serialise(const T &msg)
{
    std::vector<char> serial_msg;
    serialise(&serial_msg, msg);
    return serial_msg;
}

/* Specialisation for datatypes not requiring serialisation */
template<> std::vector<char> serialise<std::string>(const std::string &msg)
{
    return std::vector<char>(msg.begin(), msg.end());
}

template<> std::vector<char> serialise<std::vector<char>>(const std::vector<char> &msg)
{
    std::cout << "sending vector" << std::endl;
    return msg;
}


/* Deserialisation */
/* Template method to deserialise */
template<class T>
void deserialise(T *const recv, std::istream &recv_buf)
{
    boost::archive::binary_iarchive archive(recv_buf);
    archive >> *recv;
}

/* Specialisation for datatypes not requiring deserialisation */
template<> void deserialise<std::string>(std::string *const recv, std::istream &recv_buf)
{
    std::cout << "receiving string" << std::endl;
    *recv = std::string(std::istream_iterator<char>(recv_buf), std::istream_iterator<char>());
}

template<> void deserialise<std::vector<char>>(std::vector<char> *const recv, std::istream &recv_buf)
{
    std::cout << "receiving vector" << std::endl;
    *recv = std::vector<char>(std::istream_iterator<char>(recv_buf), std::istream_iterator<char>());
}

}; /* namespace raptor_networking */

/* Comon class serialisation */
BOOST_SERIALIZATION_SPLIT_FREE(boost::asio::ip::address);
BOOST_SERIALIZATION_SPLIT_FREE(boost::asio::ip::address_v4);

namespace boost {
namespace serialization {
template<class Archive>
void save(Archive &ar, const boost::asio::ip::address_v4 &addr, const unsigned int version)
{
    const unsigned long bytes = addr.to_ulong();
    ar << bytes;
}


template<class Archive>
void load(Archive &ar, boost::asio::ip::address_v4 &addr, const unsigned int version)
{
    unsigned long bytes;
    ar >> bytes;
    addr = boost::asio::ip::address_v4(bytes);
}


template<class Archive>
void save(Archive &ar, const boost::asio::ip::address &addr, const unsigned int version)
{
    auto addr_v4 = addr.to_v4();
    save(ar, addr_v4, version);
}


template<class Archive>
void load(Archive &ar, boost::asio::ip::address &addr, const unsigned int version)
{
    boost::asio::ip::address_v4 addr_v4;
    load(ar, addr_v4, version);
    addr = boost::asio::ip::address(addr_v4);
}
} /* namespace serialization */
} /* namespace boost */

#endif /* #ifndef __SERIALISATION_H__ */
