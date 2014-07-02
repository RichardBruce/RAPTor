#ifndef __RETRANSMISSION_H__
#define __RETRANSMISSION_H__

/* Standard headers */
#include <chrono>
#include <list>
#include <map>
#include <vector>

/* Boost headers */
#include "boost/uuid/uuid.hpp"
#include "boost/asio/system_timer.hpp"

/* Networking headers */
#include "stack_accessor.h"
#include "stack_controller.h"


/* TODO -- Could data be const in send? The stable information could be queried from another component */
/*         that filled the message all by itself */
namespace raptor_networking
{
using std::chrono::system_clock;

/* Class to hold per context resend information */
class timestamped_resend_info
{
    public :
        /* Inner class to hold all the information needed for a resend */
        class msg_info
        {
            public :
                msg_info(const system_clock::time_point &touched, const std::shared_ptr<std::vector<char>> &data, 
                    const std::shared_ptr<msg_header> &header)
                : _touched(touched), 
                  _data(data), 
                  _header(header)
                  { };

                /* Access functions */
                system_clock::time_point                    touched()   const { return _touched;    }
                const std::shared_ptr<std::vector<char>> &  data()      const { return _data;       }
                const std::shared_ptr<msg_header> &         header()    const { return _header;     }

                msg_info& touched(const system_clock::time_point &touched)
                {
                    _touched = touched;
                    return *this;
                }

            private :
                system_clock::time_point                    _touched;
                const std::shared_ptr<std::vector<char>>    _data;
                const std::shared_ptr<msg_header>           _header;
        };

        /* Clean up outstanding messages */
        ~timestamped_resend_info()
        {
            for (auto &p : _resend)
            {
                delete p.second;
            }
        }

        /* Remove message with sequence id 'seq' because it was acknowledged */
        timestamped_resend_info& erase(const msg_header &header)
        {
            /* Find the element, that must be there */
            auto element = _resend.find(header.resend_id());
            assert(element != _resend.end());

            /* Remove the element */
            delete element->second;
            _resend.erase(element);
            
            return *this;
        }

        timestamped_resend_info& insert(const std::shared_ptr<std::vector<char>> &data, const std::shared_ptr<msg_header> &header)
        {
            _resend.insert({header->resend_id(), new msg_info(system_clock::now(), data, header)});
            return *this;
        }

        msg_info* find_if_untouched_since(const system_clock::time_point &time)
        {
            /* Search for an element with a timestamp before time */
            auto found_iter = std::find_if(_resend.begin(), _resend.end(),
                [time](const std::pair<std::uint32_t, msg_info*> &p)
                {
                    return p.second->touched() <= time;
                });

            /* If such an element exists */
            if (found_iter != _resend.end())
            {
                /* Retouch its timestamp and return it */
                found_iter->second->touched(system_clock::now());
                return found_iter->second;
            }
            else
            {
                /* Otherwise return nothing found */
                return nullptr;
            }
        }

        /* Access functions */
        size_t size() const { return _resend.size(); }

    private :
        typedef std::map<std::uint32_t, msg_info*> msg_map;
        msg_map _resend;    /* Map of messages per sequence id that might need resending    */
};


/* Reliable transmission by acknowledging each packet and resending unacknowldged packets */
template<class UpNode, class DnNode>
class ack_retransmission : public stack_component_impl<UpNode, DnNode>
{
    public :
        ack_retransmission(DnNode *const dn_node, boost::asio::io_service &io_service, 
                stack_controller *const ctrl, const unsigned int resend_ms)
            : stack_component_impl<UpNode, DnNode>(dn_node), 
              _ctrl(ctrl), 
              _timeout(io_service), 
              _resend_dur(resend_ms),
              _resend_id(0)
            {
                _timeout.expires_from_now(_resend_dur);
                _timeout.async_wait(std::bind(&ack_retransmission<UpNode,DnNode>::handle_timer, this));
            };

        /* Virtual functions for sending and receiving messages */
        virtual void received(const stack_accessor &acc, const std::shared_ptr<msg_data> &data, const std::shared_ptr<msg_header> &header) override
        {
            /* If the message is an acknowledge remove a message from the resend buffer */
            if (data->first_fragment_begins(MSG_ACKNOWLEDGE.data(), MSG_ACKNOWLEDGE.size()))
            {
                _resend.erase(*header);
            }
            /* Else, acknowledge it */
            else
            {
                /* Send acknowledge */
                const uuid from_addr        = header->from_address();
                const uuid group_addr       = header->group_address();
                const std::uint32_t seq     = header->sequence();
                const std::uint32_t re_id   = header->resend_id();
                _ctrl->post(header->reply_stack(), [this, from_addr, group_addr, seq, re_id](const stack_accessor &acc)
                    {
                        /* Build header */
                        std::shared_ptr<msg_header> ack_header(new msg_header(from_addr, group_addr, acc.next_sequence(), MSG_ACKNOWLEDGE.size(), seq, true));
                        ack_header->resend_id(re_id);            

                        /* Pass message down the stack */
                        boost::system::error_code ignored;
                        this->_dn_node->send(acc, std::shared_ptr<std::vector<char>>(new std::vector<char>(MSG_ACKNOWLEDGE)), ack_header, ignored);
                    });

                /* Pass message up the stack */
                this->_up_node->received(acc, data, header);
            }
        }


        virtual void send(const stack_accessor &acc, const std::shared_ptr<std::vector<char>> &data, const std::shared_ptr<msg_header> &header, boost::system::error_code &ec) override
        {
            /* Set the resend if to track resends */
            header->resend_id(_resend_id++);

            /* Store the message for resending */
            _resend.insert(data, header);

            /* Pass message down the stack */
            this->_dn_node->send(acc, data, header, ec);

            return;
        }

        /* Virtual function to clone as was constructed */
        virtual stack_component::copied_pair clean_clone() override
        {
            /* Clone this and down nodes */
            auto dn_pair = this->_dn_node->clean_clone();
            auto *cloned = new ack_retransmission<UpNode, DnNode>(dn_pair.last(), _timeout.get_io_service(), _ctrl, _resend_dur.count());

            /* Update return pair */
            dn_pair.update(cloned);
            return dn_pair;
        }

        /* Access functions */
        size_t          pending_resends()   const { return _resend.size();      }
        unsigned int    resend_timeout()    const { return _resend_dur.count(); }
        std::uint32_t   current_resend_id() const { return _resend_id;          }

    private :
        void handle_timer()
        {
            /* Check if there is anything to resend */
            auto since = system_clock::now() - _resend_dur;
            auto *to_send = _resend.find_if_untouched_since(since);
            if (to_send == nullptr)
            {
                return;
            }
            
            /* Dispatch so the lambda can run in this thread, post would start lots of tasks */
            /* The resend must be atomic to stop acks arriving at the same time */
            /* TODO -- Does this end up locking the stack for a long time? */
            _ctrl->run(to_send->header()->to_stack(), [this, since, to_send](const stack_accessor &acc)
            {
                /* Find and resend all messages */
                auto *send_data = to_send;
                boost::system::error_code ignored;
                do
                {
                    this->_dn_node->send(acc, send_data->data(), send_data->header(), ignored);
                    send_data = _resend.find_if_untouched_since(since);
                } while (send_data != nullptr);

                /* Schedule the next time out */
                _timeout.expires_from_now(_resend_dur);
                _timeout.async_wait(std::bind(&ack_retransmission<UpNode,DnNode>::handle_timer, this));
            });
        }

        stack_controller *const                         _ctrl;          /* Safe access to the protocol stack            */
        boost::asio::system_timer                       _timeout;       /* Timer for resend task                        */
        std::chrono::duration<unsigned int,std::milli>  _resend_dur;    /* Period at which to resend messages           */
        timestamped_resend_info                         _resend;        /* Information needed for resend per context    */
        std::uint32_t                                   _resend_id;     /* The message id to be used for resending      */
};


/* Class to hold per context resend information */
class resend_info
{
    public :
        typedef std::pair<const std::shared_ptr<std::vector<char>>, const std::shared_ptr<msg_header>> msg_map_pair;

        /* Remove messages up to and including seen */
        resend_info& erase_until(const std::uint32_t seen)
        {
            auto iter = _resend.find(seen);
            assert(iter != _resend.end());

            _resend.erase(_resend.begin(), ++iter);

            return *this;
        }

        /* Add a message that is about to be send */
        resend_info& insert(const std::shared_ptr<std::vector<char>> &data, const std::shared_ptr<msg_header> &header)
        {
            _resend.insert({header->resend_id(), {data, header}});
            return *this;
        }

        /* Get a message for resend */
        msg_map_pair* at(const std::uint32_t re_id)
        {
            const auto &resend_iter = _resend.find(re_id);
            if (resend_iter == _resend.end())
            {
                return nullptr;
            }
            else
            {
                return &(resend_iter->second);
            }
        }

        /* Access functions */
        size_t size() const { return _resend.size(); }

    private :
        typedef std::map<std::uint32_t, msg_map_pair> msg_map;
        msg_map _resend;    /* Map of messages per sequence id that might need resending    */
};


/* Reliable transmission by requesting resends */
template<class UpNode, class DnNode>
class nack_retransmission : public stack_component_impl<UpNode, DnNode>
{
    public :
        nack_retransmission(DnNode *const dn_node, stack_controller *const ctrl)
            : stack_component_impl<UpNode, DnNode>(dn_node), 
              _ctrl(ctrl),
              _seen(1, -1),
              _resend_id(0) {  };

        /* Virtual functions for sending and receiving messages */
        virtual void received(const stack_accessor &acc, const std::shared_ptr<msg_data> &data, const std::shared_ptr<msg_header> &header) override
        {
            /* If the message is a stable remove messages from the resend buffer */
            if (data->first_fragment_begins(MSG_STABLE.data(), MSG_STABLE.size()))
            {
                const std::uint32_t seen = from_big_endian_byte_array<char, std::uint32_t>(&data->peek()[MSG_STABLE.size()]);
                _resend.erase_until(seen);                 
                return;
            }

            /* If the message is a resend then resend messages from the resend buffer */
            if (data->first_fragment_begins(MSG_RESEND.data(), MSG_RESEND.size()))
            {
                const std::uint32_t len = header->fragment_length();
                _ctrl->post(header->reply_stack(), [this, len, data](const stack_accessor &acc)
                {
                    assert(data->can_peek(len) || !"Error: Resend message got fragmented");

                    /* Get all the messages that need resending */
                    boost::system::error_code ignored;
                    const char *const data_chars = data->peek();
                    for (std::uint32_t i = 0; i < len - MSG_RESEND.size(); i += 4)
                    {
                        const std::uint32_t re_id = from_big_endian_byte_array<char, std::uint32_t>(&data_chars[MSG_RESEND.size() + i]);
                        auto *resend_pair = _resend.at(re_id);
    
                        /* Pass message down the stack */
                        if (resend_pair != nullptr)
                        {
                            this->_dn_node->send(acc, resend_pair->first, resend_pair->second, ignored);
                        }
                    }
                });

                return;
            }


            /* Place resend id into order */
            const std::int32_t re_id = header->resend_id();
            auto insert_iter = std::find_if(_seen.begin(), _seen.end(), [re_id](int e)
            {
                return e > re_id;
            });

            _seen.insert(insert_iter, re_id);

            /* Retire leading sequential entries */
            while (*(++_seen.begin()) == (_seen.front() + 1))
            {
                _seen.pop_front();
            }

            /* Check if everything is ok */
            if (_seen.front() < re_id)
            {
                const uuid from_addr    = header->from_address();
                const uuid group_addr   = header->group_address();
                const std::uint32_t seq = header->sequence();
                _ctrl->post(header->reply_stack(), [this, from_addr, group_addr, seq, re_id](const stack_accessor &acc)
                {
                    /* Build response */
                    std::shared_ptr<std::vector<char>> resend(new std::vector<char>(MSG_RESEND));
                    resend->resize(MSG_RESEND.size() + ((re_id - _seen.front() - 1) << 2));

                    auto seen = _seen.begin();
                    unsigned int end_idx = MSG_RESEND.size();
                    for (int i = _seen.front(); i < re_id; ++i)
                    {
                        if ((*seen) == i)
                        {
                            ++seen;
                        }
                        else
                        {
                            to_big_endian_byte_array<std::uint32_t, char>(i, &resend->data()[end_idx]);
                            end_idx += sizeof(std::uint32_t);
                        }
                    }

                    /* Correctly size */
                    resend->resize(end_idx);

                    /* Build header */
                    std::shared_ptr<msg_header> nack_header(new msg_header(from_addr, group_addr, acc.next_sequence(), resend->size(), seq, true));

                    /* Pass message down the stack */
                    boost::system::error_code ignored;
                    this->_dn_node->send(acc, resend, nack_header, ignored);
                });
            }

            /* Pass message up the stack */
            this->_up_node->received(acc, data, header);
        }


        virtual void send(const stack_accessor &acc, const std::shared_ptr<std::vector<char>> &data, const std::shared_ptr<msg_header> &header, boost::system::error_code &ec) override
        {
            /* Set the resend if to track resends */
            header->resend_id(_resend_id++);

            /* Store the message for resending */
            _resend.insert(data, header);

            /* Pass message down the stack */
            this->_dn_node->send(acc, data, header, ec);
        }

        /* Virtual function to clone as was constructed */
        virtual stack_component::copied_pair clean_clone() override
        {
            /* Clone this and down nodes */
            auto dn_pair = this->_dn_node->clean_clone();
            auto *cloned = new nack_retransmission<UpNode, DnNode>(dn_pair.last(), _ctrl);

            /* Update return pair */
            dn_pair.update(cloned);
            return dn_pair;
        }

        /* Access functions */
        int             seen()              const { return _seen.front();   }
        size_t          pending_resends()   const { return _resend.size();  }
        std::uint32_t   current_resend_id() const { return _resend_id;      }

    private :
        typedef std::list<int> seen_list;
        stack_controller *const _ctrl;      /* Safe access to the protocol stack            */
        seen_list               _seen;      /* Map of sequence ids seen                     */
        resend_info             _resend;    /* Information needed for resend per context    */
        std::uint32_t           _resend_id;     /* The message id to be used for resending      */
};
}; /* namespace raptor_networking */

#endif /* #ifndef __RETRANSMISSION_H__ */
