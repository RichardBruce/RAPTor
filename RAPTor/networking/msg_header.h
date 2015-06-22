#pragma once

/* Standard headers */
#include <array>

/* Boost headers */
#include "boost/asio/ip/address.hpp"
#include "boost/uuid/uuid.hpp"
#include "boost/uuid/uuid_io.hpp"
#include "boost/uuid/uuid_generators.hpp"

/* Networking Headers */
#include "conversion.h"
#include "networking_constants.h"


namespace raptor_networking
{
using boost::uuids::uuid;
using boost::asio::ip::address;

/* Class the represent the messages header */
class msg_header
{
    public :
        /* CTORs */
        explicit msg_header(const address &to_addr, const std::uint32_t seq_id, const std::uint32_t length, 
            const std::uint32_t resp_to = 0, const bool is_resp = false)
            : msg_header(to_addr, boost::uuids::nil_uuid(), boost::uuids::nil_uuid(), seq_id, length, resp_to, is_resp, true) {  };

        explicit msg_header(const uuid &to_addr, const std::uint32_t seq_id, const std::uint32_t length, 
            const std::uint32_t resp_to = 0, const bool is_resp = false)
            : msg_header(address(), to_addr, boost::uuids::nil_uuid(), seq_id, length, resp_to, is_resp, false) {  };

        explicit msg_header(const uuid &to_addr, const uuid &group_addr, const std::uint32_t seq_id, const std::uint32_t length, 
            const std::uint32_t resp_to = 0, const bool is_resp = false)
            : msg_header(address(), to_addr, group_addr, seq_id, length, resp_to, is_resp, false) {  };

        /* Deserialising CTORs */
        explicit msg_header(const char *const header)
            : msg_header(header, address(), false) {  };

        explicit msg_header(const char *const header, const address &phy_addr)
            : msg_header(header, phy_addr, true) {  };

        /* Copy CTOR */
        explicit msg_header(const msg_header &header)
            :   _phy_addr(header._phy_addr),
                _to_addr(header._to_addr),
                _group_addr(header._group_addr),
                _from_addr(header._from_addr),
                _seq_id(header._seq_id),
                _length(header._length),
                _frag_id(header._frag_id),
                _frag_len(header._frag_len),
                _resend_id(header._resend_id),
                _resp_to(header._resp_to),
                _is_resp(header._is_resp),
                _has_phy(header._has_phy) {  };

        /* Getters */
        const address * physical_address()      const { return _has_phy ? &_phy_addr : nullptr; }
        const uuid &    to_address()            const { return _to_addr;                        }
        const uuid &    group_address()         const { return _group_addr;                     }
        const uuid &    from_address()          const { return _from_addr;                      }
        std::uint32_t   sequence()              const { return _seq_id;                         }
        std::uint32_t   length()                const { return _length;                         }
        std::uint32_t   fragment()              const { return _frag_id;                        }
        std::uint32_t   fragment_length()       const { return _frag_len;                       }
        std::uint32_t   resend_id()             const { return _resend_id;                      }
        std::uint32_t   response_to()           const { return _resp_to;                        }
        bool            is_resposne()           const { return _is_resp;                        }
        bool            has_physical_address()  const { return _has_phy;                        }

        /* Setters */
        msg_header& from_address(const uuid &from_addr)
        {
            _from_addr = from_addr;
            return *this;
        }

        msg_header& length(const std::uint32_t length)
        {
            _length = length;
            return *this;
        }

        msg_header& fragment(const std::uint32_t frag_id)
        {
            _frag_id = frag_id;
            return *this;
        }

        msg_header& fragment_length(const std::uint32_t frag_len)
        {
            _frag_len = frag_len;
            return *this;
        }

        msg_header& resend_id(const std::uint32_t resend_id)
        {
            _resend_id = resend_id;
            return *this;
        }

        /* Stack determination */
        const uuid & reply_stack() const
        {
            return _group_addr.is_nil() ? _from_addr : _group_addr;
        }

        const uuid & to_stack() const
        {
            return _group_addr.is_nil() ? _to_addr : _group_addr;
        }

        /* Apply the from stack rules to a serialised header */
        static uuid from_stack(const char *const header)
        {
            /* Try to get the group id, if it is null use the individual id */
            const uuid id = parse_uuid(&header[16]);
            return id.is_nil() ? parse_uuid(&header[32]) : id;
        }

        msg_header& has_physical_address(const bool has_phy)
        {
            _has_phy = has_phy;
            return *this;
        }

        /* Get the size of the serialised header */
        static constexpr size_t size() { return HEADER_SIZE; }

        /* Convert a header to a byte stream */
        const msg_header& serialise(char *const header) const
        {
            std::copy(_to_addr.begin(), _to_addr.end(), &header[0]);
            std::copy(_group_addr.begin(), _group_addr.end(), &header[16]);
            std::copy(_from_addr.begin(), _from_addr.end(), &header[32]);
            to_big_endian_byte_array<std::uint32_t, char>(_seq_id, &header[48]);
            to_big_endian_byte_array<std::uint32_t, char>(_length, &header[52]);
            to_big_endian_byte_array<std::uint32_t, char>(_frag_id, &header[56]);
            to_big_endian_byte_array<std::uint32_t, char>(_frag_len, &header[60]);
            to_big_endian_byte_array<std::uint32_t, char>(_resend_id, &header[64]);
            to_big_endian_byte_array<std::uint32_t, char>(_resp_to, &header[68]);
            to_big_endian_byte_array<bool, char>(_is_resp, &header[72]);

            return *this;
        }

        /* Comparison operator */
        bool operator==(const msg_header &rhs) const
        {
          return (_to_addr    == rhs._to_addr)    &&
                 (_group_addr == rhs._group_addr) &&
                 (_from_addr  == rhs._from_addr)  &&
                 (_seq_id     == rhs._seq_id)     &&
                 (_length     == rhs._length)     &&
                 (_frag_id    == rhs._frag_id)    &&
                 (_frag_len   == rhs._frag_len)   &&
                 (_resend_id  == rhs._resend_id)  &&
                 (_resp_to    == rhs._resp_to)    &&
                 (_is_resp    == rhs._is_resp)    &&
                 (_has_phy    == rhs._has_phy)    &&
                 (!_has_phy || (_phy_addr   == rhs._phy_addr));
        }

        void dump() const
        {
            std::cout << "To Address: " << _to_addr << std::endl;
            std::cout << "Group Address: " << _group_addr << std::endl;
            std::cout << "From Address: " << _from_addr << std::endl;
            std::cout << "Sequence: " << _seq_id << std::endl;
            std::cout << "Length: " << _length << std::endl;
            std::cout << "Fragment Id: " << _frag_id << std::endl;
            std::cout << "Fragment Length: " << _frag_len << std::endl;
            std::cout << "Resend Id: " << _resend_id << std::endl;
            std::cout << "Resend to: " << _resp_to << std::endl;
            std::cout << "Is Response: " << _is_resp << std::endl;
            std::cout << "Has Physical Address: " << _has_phy << std::endl;
            if (_has_phy)
            {
                std::cout << "Physical Address: " <<  _phy_addr.to_string() << std::endl;
            }
        }

    private :
        /* Common CTOR for all non serialised construction */
        explicit msg_header(const address &phy_addr, const uuid &to_addr, const uuid &group_addr, const std::uint32_t seq_id, 
            const std::uint32_t length, const std::uint32_t resp_to, const bool is_resp, const bool has_phy)
            : _phy_addr(phy_addr),
              _to_addr(to_addr),
              _group_addr(group_addr),
              _from_addr(boost::uuids::nil_uuid()),
              _seq_id(seq_id), 
              _length(length), 
              _frag_id(0), 
              _frag_len(length), 
              _resend_id(0),
              _resp_to(resp_to), 
              _is_resp(is_resp),
              _has_phy(has_phy)
            {  };

        /* Common CTOR for all serialised contruction */
        explicit msg_header(const char *const header, const address &phy_addr, const bool has_phy)
            : _phy_addr(phy_addr),
              _to_addr(msg_header::parse_uuid(&header[0])),
              _group_addr(msg_header::parse_uuid(&header[16])),
              _from_addr(msg_header::parse_uuid(&header[32])),
              _seq_id(from_big_endian_byte_array<char, std::uint32_t>(&header[48])),
              _length(from_big_endian_byte_array<char, std::uint32_t>(&header[52])),
              _frag_id(from_big_endian_byte_array<char, std::uint32_t>(&header[56])),
              _frag_len(from_big_endian_byte_array<char, std::uint32_t>(&header[60])),
              _resend_id(from_big_endian_byte_array<char, std::uint32_t>(&header[64])),
              _resp_to(from_big_endian_byte_array<char, std::uint32_t>(&header[68])),
              _is_resp(from_big_endian_byte_array<char, bool>(&header[72])),
              _has_phy(has_phy)
            {  };

        /* Utility function to parse a uuid */
        uuid static parse_uuid(const char *const header)
        {
            uuid id;
            std::copy(&header[0], &header[uuid::static_size()], id.begin());
            return id;
        }

        const address       _phy_addr;      /* The physical address the message was received from, not serialised                   */
        const uuid          _to_addr;       /* The logical address the message is being sent to                                     */
        const uuid          _group_addr;    /* The logical address the message is being sent to                                     */
        uuid                _from_addr;     /* The logical address the message is being sent from, to be filled by the connection   */
        const std::uint32_t _seq_id;        /* The unique, for this address and across parts, id of this message                    */
        std::uint32_t       _length;        /* The length in bytes of this message excluding the header                             */
        std::uint32_t       _frag_id;       /* A unique, for this sequence id, id of this part of the message                       */
        std::uint32_t       _frag_len;      /* The length of this part of the message in bytes excluding the header                 */
        std::uint32_t       _resend_id;     /* The unique, for this address, id of this message                                     */
        const std::uint32_t _resp_to;       /* The sequence id that this is message is a response to                                */
        const bool          _is_resp;       /* If this message is a response                                                        */
        bool                _has_phy;       /* Weather a physical address is available                                              */
};
}; /* namespace raptor_networking */
