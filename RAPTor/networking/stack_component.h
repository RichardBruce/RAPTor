#ifndef __STACK_COMPONENT_H__
#define __STACK_COMPONENT_H__

/* Standard headers */
#include <string>
#include <vector>

/* Boost headers */
#include "boost/asio/ip/address.hpp"

/* Networking headers */
#include "msg_data.h"
#include "msg_header.h"


namespace raptor_networking
{
/* Forward declarations */
class stack_accessor;
class message_delivery;

/* Pure virtual class to represent any component on the send/receive stack */
class stack_component
{
    public :
        class copied_pair
        {
            public :
                copied_pair(stack_component *last)
                    : _last(last), _bot(nullptr), _top(nullptr) {  };

                /* Access functions */
                stack_component *   last()      const { return _last;   }
                stack_component *   bottom()    const { return _bot;    }
                message_delivery *  top()       const { return _top;    }

                copied_pair& update(stack_component *const last)
                {
                    /* Conditional update of bottom */
                    if (_bot == nullptr)
                    {
                        _bot = last;
                    }

                    _last = last;

                    return *this;
                }

                copied_pair& top(message_delivery *const top)
                {
                    _top = top;
                    return *this;
                }

            private :
                stack_component *   _last;
                stack_component *   _bot;
                message_delivery *  _top;
        };

        /* Virtual DTOR for inheritance */
        virtual ~stack_component() { };

        /* Virtual functions for sending and receiving messages */
        virtual void received(const stack_accessor &acc, const std::shared_ptr<msg_data> &data, const std::shared_ptr<msg_header> &header) = 0;
        virtual void send(const stack_accessor &acc, const std::shared_ptr<std::vector<char>> &data, const std::shared_ptr<msg_header> &header, boost::system::error_code &ec) = 0;

        /* Virtual function to clone as was constructed, must be threadsafe */
        virtual copied_pair clean_clone() = 0;

        /* Virtual function to connect the upward links of the stack */
        virtual stack_component& build_up_links(void *const up_node) = 0;
};


/* Class to provide common implementation details of stack_component */
template<class UpNode, class DnNode>
class stack_component_impl : public stack_component
{
    public :
        /* Stack linking members */
        stack_component& link_down_node(DnNode *const dn_node)
        {
            _dn_node = dn_node;
            return *this;
        }

        stack_component& link_up_node(UpNode *const up_node)
        {
            _up_node = up_node;
            return *this;
        }

        /* Virtual function to connect the upward links of the stack */
        virtual stack_component& build_up_links(void *const up_node) override
        {
            /* This cast will always be ok */
            _up_node = static_cast<UpNode *>(up_node);
            _dn_node->build_up_links(this);
            return *this;
        }

    protected :
        /* CTOR */
        stack_component_impl(DnNode *const dn_node)
            : _up_node(nullptr), _dn_node(dn_node) { };

        /* Virtual DTOR for inheriting classes */
        virtual ~stack_component_impl()
        {
            if (_up_node != nullptr)
            {
                delete _up_node;
            }
        };

        UpNode * _up_node;
        DnNode * _dn_node;
};
}; /* namespace raptor_networking */

#endif /* #ifndef __STACK_COMPONENT_H__ */
