#ifndef __MESSAGE_GENERATOR_H__
#define __MESSAGE_GENERATOR_H__

/* Standard headers */
#include <algorithm>
#include <memory>
#include <string>

/* Boost headers */
#include "boost/uuid/uuid.hpp"
#include "boost/uuid/uuid_generators.hpp"

/* Networking headers */
#include "msg_data.h"
#include "msg_header.h"


namespace raptor_networking
{
using boost::uuids::uuid;

class message_generator
{
    public :
        message_generator(const std::string &msg)
            : _ran_gen(),
              _uuid_gen(&_ran_gen),
              _msg(msg)
            {
                _ran_gen.seed(time(NULL));
                _to     = _uuid_gen();
                _group  = _uuid_gen();
                _from   = _uuid_gen();
            };

        size_t message_size() const { return _msg.size(); }

        std::shared_ptr<msg_header> header(const int seq, const int resend = 0) const
        {
            std::shared_ptr<msg_header> header(new msg_header(_to, _group, seq, _msg.size()));
            header->resend_id(resend);      /* Set resend id normally done by retransmission    */
            header->from_address(_from);    /* Set from address normally done by the conection  */
            return header;
        }

        std::shared_ptr<msg_header> header(const std::string &phy_addr, const int seq, const int resend = 0) const
        {
            std::shared_ptr<msg_header> header(new msg_header(address::from_string(phy_addr), seq, _msg.size()));
            header->resend_id(resend);      /* Set resend id normally done by retransmission    */
            header->from_address(_from);    /* Set from address normally done by the conection  */
            return header;
        }

        std::shared_ptr<msg_data> receive_data() const
        {
            char *data = new char [_msg.size()];
            std::copy(_msg.begin(), _msg.end(), data);
            return std::shared_ptr<msg_data>(new msg_data(data, _msg.size()));
        }

        std::shared_ptr<std::vector<char>> send_data() const
        {
            return std::shared_ptr<std::vector<char>>(new std::vector<char>(_msg.begin(), _msg.end()));
        }

        /* Functions to check data */
        bool check_data(const std::shared_ptr<msg_data> &data) const
        {
            /* Convert to stream and read its contents into an array */
            std::unique_ptr<std::istream> stream(data->as_stream());
            std::unique_ptr<char []> read_data(new char [_msg.size()]);
            stream->read(read_data.get(), _msg.size());

            return check_data(read_data.get());
        }

        bool check_data(const std::unique_ptr<std::istream> &data) const
        {
            std::unique_ptr<char []> raw_data(new char [_msg.size()]);
            data->read(raw_data.get(), _msg.size());
            return check_data(raw_data.get());
        }

        bool check_data(const std::shared_ptr<std::vector<char>> &data) const
        {
            return check_data(data->data());
        }

        bool check_data(const std::shared_ptr<std::vector<char>> &data, const unsigned int start, const unsigned int end) const
        {
            return check_data(data->data(), start, end);
        }

        bool check_data(const char *const data) const
        {
            return strncmp(data, _msg.data(), _msg.size()) == 0;
        }

        bool check_data(const char *const data, const unsigned int start, const unsigned int end) const
        {
            return strncmp(data, &_msg.data()[start], end - start) == 0;
        }

        /* Expected data */
        std::string expected() const
        {
            return expected(0, _msg.size());
        }

        std::string expected(const unsigned int start, const unsigned int end) const
        {
            return std::string(&_msg.data()[start], end - start);
        }

        const uuid & to_stack() const
        {
            return _group.is_nil() ? _to : _group;
        }

        const uuid & from_address()     const { return _from;   }
        const uuid & group_address()    const { return _group;  }

    private :
        typedef boost::uuids::basic_random_generator<boost::mt19937> uuid_gen;

        boost::mt19937      _ran_gen;   /* Random number generator to drive the uuid generator  */
        uuid_gen            _uuid_gen;  /* Uuid generator for process ids                       */
        uuid                _from;
        uuid                _to;
        uuid                _group;
        const std::string   _msg;
};
}; /* namespace raptor_networking */

#endif /* #ifndef __MESSAGE_GENERATOR_H__ */
