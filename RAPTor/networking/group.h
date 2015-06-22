#pragma once

/* Standard headers */
#include <algorithm>
#include <cstdint>
#include <map>
#include <string>
#include <sstream>
#include <thread>
#include <vector>

/* Boost */
#include "boost/archive/text_oarchive.hpp"
#include "boost/archive/text_iarchive.hpp"
#include "boost/serialization/map.hpp"
#include "boost/asio/ip/address.hpp"
#include "boost/uuid/uuid.hpp"
#include "boost/uuid/uuid_generators.hpp"
#include "boost/uuid/uuid_serialize.hpp"


/* Networking headers */
#include "conversion.h"
#include "serialisation.h"
#include "stack_controller.h"


namespace raptor_networking
{
using boost::uuids::uuid;
using boost::asio::ip::address;

/* Class to represent a member of a group */
class group_member
{
    public :
        group_member() = default;

        group_member(const address &addr, const std::uint16_t port_offset, const bool me)
            : _addr(addr), _port_offset(port_offset), _me(me) { };

        /* Access functions */
        const address * physical_address()  const { return &_addr;          }
        std::uint16_t   port_offset()       const { return _port_offset;    }
        bool            is_me()             const { return _me;             }

        /* Serialisation left blank and left to save_construct_data and load_construct_data */
        template<class Archive>
        void serialize(Archive &ar, const unsigned int version)
        {
            ar & _addr;
            ar & _port_offset;
            ar & _me;
        }

    private :
        friend class boost::serialization::access;

        address       _addr;          /* The ip address of the process        */
        std::uint16_t _port_offset;   /* Offset for all uni cast ports        */
        bool          _me;            /* Is this group member this process    */
};

/* Class to represent multiple processes that communicate */
class group : public stack_component_impl<stack_component, stack_component>
{
    public :
        group(stack_controller *const ctrl, const std::string &grp_addr, 
            const std::string &me_addr, const std::uint16_t grp_port, const std::uint16_t port_offset)
            : stack_component_impl<stack_component, stack_component>(nullptr),
              _ran_gen(),
              _uuid_gen(&_ran_gen),
              _ctrl(ctrl), 
              _pimpl(new group_impl(group_member(address::from_string(me_addr), port_offset, true), grp_addr, _uuid_gen(), _uuid_gen(), grp_port))
            {
                _ran_gen.seed(time(NULL));
            };

        group(stack_controller *const ctrl, const std::string &me_addr, 
            const std::uint16_t port_offset)
            : stack_component_impl<stack_component, stack_component>(nullptr), 
              _ran_gen(),
              _uuid_gen(&_ran_gen),
              _ctrl(ctrl), 
              _pimpl(new group_impl(group_member(address::from_string(me_addr), port_offset, true), _uuid_gen(), _uuid_gen()))
            {
                _ran_gen.seed(time(NULL));
            };

        /* Group control functions */
        std::future<bool> connect()
        {
            _ctrl->run(_pimpl->_id, [this](const stack_accessor &acc)
            {
                /* Build header */
                auto seq = acc.next_sequence();
                std::shared_ptr<msg_header> header(new msg_header(boost::uuids::nil_uuid(), _pimpl->_id, seq, MSG_SUBSCRIBE.size()));
                _conn.first = seq;

                /* Build message */
                std::shared_ptr<std::vector<char>> data(new std::vector<char>(MSG_SUBSCRIBE.begin(), MSG_SUBSCRIBE.end()));

                /* Pass message down the stack */
                boost::system::error_code ignored;
                this->_dn_node->send(acc, data, header, ignored);
            });

            return _conn.second.get_future();
        }

        bool disconnect()
        {
            boost::system::error_code ec;
            _ctrl->run(_pimpl->_id, [this, &ec](const stack_accessor &acc)
            {
                /* Build header */
                std::shared_ptr<msg_header> header(new msg_header(boost::uuids::nil_uuid(), _pimpl->_id, acc.next_sequence(), MSG_UNSUBSCRIBE.size()));

                /* Build message */
                std::shared_ptr<std::vector<char>> data(new std::vector<char>(MSG_UNSUBSCRIBE.begin(), MSG_UNSUBSCRIBE.end()));

                /* Pass message down the stack */
                this->_dn_node->send(acc, data, header, ec);
            });

            return !ec;
        }

        /* Access functions */
        const address & group_physical_address()    const { return _pimpl->_addr;           }
        const uuid&     group_address()             const { return _pimpl->_id;             }
        std::uint16_t   group_port()                const { return _pimpl->_port;           }
        size_t          size_of_group()             const { return _pimpl->_group.size();   }

        const uuid & my_address() const
        {
            auto me = std::find_if(_pimpl->_group.begin(), _pimpl->_group.end(), [](const std::pair<uuid, group_member> &p)
            {
                return p.second.is_me();
            });

            return me->first;
        }

        const address * physical_address(const uuid &id) const
        {
            auto group_iter = _pimpl->_group.find(id);
            if (group_iter == _pimpl->_group.end())
            {
                return nullptr;
            }

            return group_iter->second.physical_address();
        }

        std::uint16_t port_offset(const uuid &id) const
        {
            auto group_iter = _pimpl->_group.find(id);
            if (group_iter == _pimpl->_group.end())
            {
                return 0;
            }

            return group_iter->second.port_offset();
        }

        group& add_to_group(const uuid &id, const address &addr, const std::uint16_t port_offset)
        {
            _pimpl->_group.insert({ id, group_member(addr, port_offset, false) });
            _pimpl->_ages.push_back(id);
            return *this;
        }

        /* Virtual functions for sending and receiving messages */
        virtual void received(const stack_accessor &acc, const std::shared_ptr<msg_data> &data, const std::shared_ptr<msg_header> &header) override
        {
            /* If not interested in the message, pass it up stream */
            if (!(data->first_fragment_begins(MSG_ACCEPT.data(),      MSG_ACCEPT.size()))    &&
                !(data->first_fragment_begins(MSG_SUBSCRIBE.data(),   MSG_SUBSCRIBE.size())) &&
                !(data->first_fragment_begins(MSG_UNSUBSCRIBE.data(), MSG_UNSUBSCRIBE.size())))
            {
                this->_up_node->received(acc, data, header);
                return;
            }

            /* If someone was accepted then need to update the group */
            if (data->first_fragment_begins(MSG_ACCEPT.data(), MSG_ACCEPT.size()))
            {
                /* Update internal state */
                std::unique_ptr<std::istream> data_stream(data->as_stream());

                char type_buf [MSG_ACCEPT.size()];
                data_stream->read(type_buf, MSG_ACCEPT.size());

                deserialise(_pimpl.get(), *data_stream);

                /* Update any pending promises */
                if (_conn.first == header->sequence())
                {
                    _conn.second.set_value(true);
                }

                return;
            }

            /* Otherwise the message is only relavent to the group coordinator (the oldest member of the group) */
            const auto oldest_iter = _pimpl->_group.find(_pimpl->_ages.front());
            if ((oldest_iter != _pimpl->_group.end()) && oldest_iter->second.is_me())
            {
                if (data->first_fragment_begins(MSG_SUBSCRIBE.data(), MSG_SUBSCRIBE.size()))
                {
                    /* Find the group member */
                    auto group_iter = _pimpl->_group.find(header->from_address());
                    if (group_iter == _pimpl->_group.end())
                    {
                        /* TODO -- get group and phy addr from the data */
                        _pimpl->_group.insert({ header->from_address(), group_member(*header->physical_address(), 0, false) });
                        _pimpl->_ages.push_back(header->from_address());
                    }

                }
                else if (data->first_fragment_begins(MSG_UNSUBSCRIBE.data(), MSG_UNSUBSCRIBE.size()))
                {
                    /* Find the group member */
                    auto group_iter = _pimpl->_group.find(header->from_address());
                    if (group_iter != _pimpl->_group.end())
                    {
                        /* Remove it */
                        _pimpl->_group.erase(group_iter);
                        _pimpl->_ages.erase(std::remove(_pimpl->_ages.begin(), _pimpl->_ages.end(), header->from_address()), _pimpl->_ages.end()); 
                    }
                }

                /* Send a response */
                std::shared_ptr<std::vector<char>> resp_data(new std::vector<char>(MSG_ACCEPT));
                serialise(resp_data.get(), *_pimpl);

                std::shared_ptr<msg_header> resp_header(new msg_header(header->from_address(), acc.next_sequence(), resp_data->size(), header->sequence(), true));

                boost::system::error_code ignored;
                this->_dn_node->send(acc, resp_data, resp_header, ignored);
            }
        }

        virtual void send(const stack_accessor &acc, const std::shared_ptr<std::vector<char>> &data, const std::shared_ptr<msg_header> &header, boost::system::error_code &ec) override
        {
            /* Pass down the stack */
            this->_dn_node->send(acc, data, header, ec);
        }

        /* Virtual function to clone as was constructed */
        virtual stack_component::copied_pair clean_clone() override
        {
            /* Clone this and down nodes */
            auto dn_pair = this->_dn_node->clean_clone();
            auto *cloned = shared_clone();
            cloned->link_down_node(dn_pair.last());

            /* Update return pair */
            dn_pair.update(cloned);
            return dn_pair;
        }

        group* shared_clone()
        {
            return new group(_ctrl, _pimpl);
        }

    private :
        class group_impl;
        typedef boost::uuids::basic_random_generator<boost::mt19937> uuid_gen;

        group(stack_controller *const ctrl, std::shared_ptr<group_impl> pimpl)
            : stack_component_impl<stack_component, stack_component>(nullptr), 
              _ran_gen(),
              _uuid_gen(&_ran_gen),
              _ctrl(ctrl), 
              _pimpl(pimpl)
            {
                _ran_gen.seed(time(NULL));
            };

        struct group_impl
        {
            group_impl(const group_member &mem, const std::string &grp_addr, const uuid &grp_id, const uuid &id, const std::uint16_t grp_port)
                : _group({{id, mem}}), _ages(1, id), _addr(address::from_string(grp_addr)), _id(grp_id), _port(grp_port) {  };

            group_impl(const group_member &mem, const uuid &grp_id, const uuid &id)
                : _group({{id, mem}}), _ages(1, id), _addr(address()), _id(grp_id), _port(0) {  };

            /* Serialise */
            template<class Archive>
            void serialize(Archive &ar, const unsigned int version)
            {
                ar & _group;
                ar & _ages;
                ar & _addr;
                ar & _id;
                ar & _port;
            }

            friend class boost::serialization::access;
            typedef std::map<uuid, group_member> group_map;
            group_map           _group; /* All the group members                        */
            std::vector<uuid>   _ages;  /* Age order of the group members               */
            address             _addr;  /* The physical multicast address of the group  */
            uuid                _id;    /* The logical address of the group             */
            std::uint16_t       _port;  /* Port the multicast group operates on         */
        };

        boost::mt19937                                  _ran_gen;   /* Random number generator to drive the uuid generator  */
        uuid_gen                                        _uuid_gen;  /* Uuid generator for process ids                       */
        stack_controller  *const                        _ctrl;      /* Stack to join or leave groups with                   */
        std::shared_ptr<group_impl>                     _pimpl;     /* Pointer to shared implementation details             */
        std::pair<std::uint32_t, std::promise<bool>>    _conn;      /* State of a pending connect                           */
};
}; /* namespace raptor_networking */
