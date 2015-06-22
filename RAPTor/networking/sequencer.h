#pragma once

/* Standard headers */
#include <cstdint>
#include <map>

/* Networking headers */
#include "stack_component.h"
#include "stack_accessor.h"
#include "msg_header.h"
#include "msg_data.h"


namespace raptor_networking
{
/* Class to handle out of order messages by putting them back into order */
template<class UpNode, class DnNode>
class reorder_sequencer : public stack_component_impl<UpNode, DnNode>
{
    public :
        reorder_sequencer(DnNode *const dn_node)
            : stack_component_impl<UpNode, DnNode>(dn_node), _seq(0) { };

        ~reorder_sequencer()
        {
            /* Delete all the messages waiting to be reordered */
            _reorder.clear();
        }

        /* Virtual functions for sending and receiving messages */
        virtual void received(const stack_accessor &acc, const std::shared_ptr<msg_data> &data, const std::shared_ptr<msg_header> &header) override
        {
            /* If the sequence id is equal to the next expected */
            if (header->sequence() == _seq)
            {
                /* Pass the message up the stack */
                this->_up_node->received(acc, data, header);

                /* Check for held entries that are now in order */
                auto reorder_iter = _reorder.begin();
                while (reorder_iter->first == ++_seq)
                {
                    /* Pass the message up the stack */
                    this->_up_node->received(acc, reorder_iter->second.second, reorder_iter->second.first);

                    /* Erase this entry and move to the next */
                    _reorder.erase(reorder_iter++);
                }
            }
            /* If the message is early hold it for later */
            else if (header->sequence() > _seq)
            {
                _reorder.insert({header->sequence(), {header, data}});
            }
        }
        
        virtual void send(const stack_accessor &acc, const std::shared_ptr<std::vector<char>> &data, const std::shared_ptr<msg_header> &header, boost::system::error_code &ec) override
        {
            /* Pass the message down the stack */
            this->_dn_node->send(acc, data, header, ec);
        }

        /* Virtual function to clone as was constructed */
        virtual stack_component::copied_pair clean_clone() override
        {
            /* Clone this and down nodes */
            auto dn_pair = this->_dn_node->clean_clone();
            auto *cloned = new reorder_sequencer<UpNode, DnNode>(dn_pair.last());

            /* Update return pair */
            dn_pair.update(cloned);
            return dn_pair;
        }

        /* Access functions */
        std::uint32_t   current_sequence() const { return _seq;             }
        size_t          pending_messages() const { return _reorder.size();  }

    private :
        typedef std::map<std::uint32_t, std::pair<const std::shared_ptr<msg_header>, const std::shared_ptr<msg_data>>> reorder_map;

        reorder_map     _reorder;   /* Map of messages waiting to be passed on  */
        std::uint32_t   _seq;       /* The next expected sequence id            */
};


/* Class to handle out of order messages by dropping late ones */
template<class UpNode, class DnNode>
class drop_late_sequencer : public stack_component_impl<UpNode, DnNode>
{
    public :
        drop_late_sequencer(DnNode *const dn_node)
            : stack_component_impl<UpNode, DnNode>(dn_node), _seq(0) { };

        /* Virtual functions for sending and receiving messages */
        virtual void received(const stack_accessor &acc, const std::shared_ptr<msg_data> &data, const std::shared_ptr<msg_header> &header) override
        {
            /* If the sequence id is equal to the next expected */
            if (header->sequence() >= _seq)
            {
                /* Pass the message up the stack */
                _seq = header->sequence() + 1;
                this->_up_node->received(acc, data, header);
            }
        }
        
        virtual void send(const stack_accessor &acc, const std::shared_ptr<std::vector<char>> &data, const std::shared_ptr<msg_header> &header, boost::system::error_code &ec) override
        {
            /* Pass the message down the stack */
            this->_dn_node->send(acc, data, header, ec);
        }

        /* Virtual function to clone as was constructed */
        virtual stack_component::copied_pair clean_clone() override
        {
            /* Clone this and down nodes */
            auto dn_pair = this->_dn_node->clean_clone();
            auto *cloned = new drop_late_sequencer<UpNode, DnNode>(dn_pair.last());

            /* Update return pair */
            dn_pair.update(cloned);
            return dn_pair;
        }

        /* Access functions */
        std::uint32_t current_sequence() const { return _seq; }

    private :
        std::uint32_t   _seq;   /* The next expected sequence id */
};
}; /* namespace raptor_networking */
