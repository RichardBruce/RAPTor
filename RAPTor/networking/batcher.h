#pragma once

/* Standard headers */
#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

/* Boost headers */
#include "boost/uuid/uuid.hpp"
#include "boost/asio/io_service.hpp"
#include "boost/asio/system_timer.hpp"
#include "boost/asio/basic_waitable_timer.hpp"

/* Networking headers */
#include "stack_component.h"
#include "stack_accessor.h"
#include "stack_controller.h"


namespace raptor_networking
{
using boost::uuids::uuid;

/* Class to batch a number of small messages together into one big one */
template<class UpNode, class DnNode>
class batcher : public stack_component_impl<UpNode, DnNode>
{
    public :
        batcher(DnNode *const dn_node, boost::asio::io_service &io_service, stack_controller *const ctrl,
            const int max_delay_ms, const unsigned short max_size_bytes)
            : stack_component_impl<UpNode, DnNode>(dn_node),
              _ctrl(ctrl), 
              _timeout(io_service), 
              _max_delay(max_delay_ms), 
              _buffer(new std::vector<char>(MSG_BATCH)),
              _stack(),
              _to_addr(),
              _group_addr(),
              _max_size(max_size_bytes)
              {  };

        /* Virtual functions for sending and receiving messages */
        virtual void received(const stack_accessor &acc, const std::shared_ptr<msg_data> &data, const std::shared_ptr<msg_header> &header) override
        {
            /* If the message was a batch split it up and pass it up the stack */
            /* Assumes the message wasnt later fragmented */
            if (data->first_fragment_begins(MSG_BATCH.data(), MSG_BATCH.size()))
            {
                /* While this is enough data for another message header */
                unsigned int idx = MSG_BATCH.size();
                const char *const raw_data = data->peek();
                while (data->can_peek(idx + msg_header::size()))
                {
                    /* Copy out the header */
                    std::shared_ptr<msg_header> split_header(new msg_header(&raw_data[idx], *header->physical_address()));
                    idx += msg_header::size();

                    /* There must be enough data for the message body now */
                    const std::uint32_t frag = split_header->fragment_length();
                    assert(data->can_peek(idx + frag) || !"Error: Batched message without enough data for body, this probably means a match message was fragmented");

                    /* Copy out the data */
                    char * split_data = new char [frag];
                    std::copy(&raw_data[idx], &raw_data[idx + frag], split_data);
                    idx += frag;

                    /* Call up the stack */
                    this->_up_node->received(acc, std::shared_ptr<msg_data>(new msg_data(split_data, frag)), split_header);
                }
            }
            /* Else just pass it up the stack */
            else
            {
                this->_up_node->received(acc, data, header);
            }
        }

        virtual void send(const stack_accessor &acc, const std::shared_ptr<std::vector<char>> &data, const std::shared_ptr<msg_header> &header, boost::system::error_code &ec) override
        {
            /* If the message is big enough (or would be when batched) just send it */
            const size_t msg_size = data->size() + msg_header::size();
            if (msg_size >= (_max_size - msg_header::size() - MSG_BATCH.size()))
            {
                this->_dn_node->send(acc, data, header, ec);
                return;
            }

            /* If the batch would overflow then send and begin again */
            const int buf_size = _buffer->size() + msg_header::size();
            if ((msg_size + buf_size) >= _max_size)
            {
                std::shared_ptr<msg_header> batch_header(new msg_header(header->to_address(), acc.next_sequence(), _buffer->size()));
                this->_dn_node->send(acc, _buffer, batch_header, ec);
                _buffer.reset(new std::vector<char>(MSG_BATCH));
            }

            /* Reize the buffer */
            const size_t cur_size = _buffer->size();
            _buffer->resize(cur_size + msg_size);

            /* Copy the header */
            header->serialise(&_buffer->data()[cur_size]);

            /* Copy the data */
            std::copy(data->begin(), data->end(), &_buffer->data()[cur_size + msg_header::size()]);
            
            /* Set the timeout for the data just added */
            if (cur_size == MSG_BATCH.size())
            {
                _stack = header->to_stack();        /* TODO -- Can be moved to CTOR as one stack only serves one address */
                _to_addr = header->to_address();    /* TODO -- Can be moved to CTOR as one stack only serves one address */
                _group_addr = header->group_address();   /* TODO -- Can be moved to CTOR as one stack only serves one address */

                /* Schedule the next time out */
                _timeout.expires_from_now(_max_delay);
                _timeout.async_wait(std::bind(&batcher<UpNode, DnNode>::handle_timer, this));
            }
        }

        /* Virtual function to clone as was constructed */
        virtual stack_component::copied_pair clean_clone() override
        {
            /* Clone this and down nodes */
            auto dn_pair = this->_dn_node->clean_clone();
            auto *cloned = new batcher<UpNode, DnNode>(dn_pair.last(), _timeout.get_io_service(), _ctrl, _max_delay.count(), _max_size);

            /* Update return pair */
            dn_pair.update(cloned);
            return dn_pair;
        }

        /* Access functions */
        size_t  pending_bytes() const { return _buffer->size();     }
        size_t  max_size()      const { return _max_size;           }
        int     max_delay()     const { return _max_delay.count();  }

    private :
        void handle_timer()
        {
            /* Check if the data is still here to sent and send if needed */
            _ctrl->run(_stack, [this](const stack_accessor &acc)
                {
                    if ((_buffer->size() > MSG_BATCH.size()) && (_timeout.expires_at() <= std::chrono::system_clock::now()))
                    {
                        /* Prepare and send */
                        boost::system::error_code ignored;
                        std::shared_ptr<msg_header> header(new msg_header(_to_addr, _group_addr, acc.next_sequence(), _buffer->size()));
                        this->_dn_node->send(acc, _buffer, header, ignored);

                        /* Create new buffer */
                        _buffer.reset(new std::vector<char>(MSG_BATCH));
                    }
                });
        }

        stack_controller                                        *const  _ctrl;      /* Safe access to the stack                 */
        boost::asio::basic_waitable_timer<std::chrono::system_clock>    _timeout;   /* Timer to implement send after max_delay  */
        const std::chrono::duration<int,std::milli>                     _max_delay; /* The maximum time to hold a packet for    */
        std::shared_ptr<std::vector<char>>                              _buffer;    /* Buffer to bacth in                       */
        uuid                                                            _stack;
        uuid                                                            _to_addr;   /* The logical address served               */
        uuid                                                            _group_addr;
        const unsigned short                                            _max_size;  /* The maximum size to batch up to in bytes */
};
}; /* namespace raptor_networking */
