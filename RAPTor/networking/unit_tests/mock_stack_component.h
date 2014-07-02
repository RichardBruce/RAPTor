#ifndef __MOCK_STACK_COMPONENT_H__
#define __MOCK_STACK_COMPONENT_H__

/* Standard headers */
#include <deque>

/* Networking headers */
#include "stack_component.h"
#include "msg_header.h"
#include "msg_data.h"

namespace raptor_networking
{
class mock_stack_component : public stack_component_impl<stack_component, stack_component>
{
    public :
        mock_stack_component(stack_component *const dn_node = nullptr)
            : stack_component_impl<stack_component, stack_component>(dn_node), _conn(false) {  };

        mock_stack_component(const bool conn)
            : stack_component_impl<stack_component, stack_component>(nullptr), _conn(conn) {  };

        /* Access functions */
        size_t headers_sent()       const { return _send_header.size(); }
        size_t headers_received()   const { return _recv_header.size(); }
        size_t data_sent()          const { return _send_data.size();   }
        size_t data_received()      const { return _recv_data.size();   }

        std::shared_ptr<msg_header> received_header()
        {
            if (_recv_header.empty())
            {
                return nullptr;
            }
            else
            {
                auto ret = _recv_header.front();
                _recv_header.pop_front();
                return ret;
            }
        }

        std::shared_ptr<msg_header> sent_header()
        {
            if (_send_header.empty())
            {
                return nullptr;
            }
            else
            {
                auto ret = _send_header.front();
                _send_header.pop_front();
                return ret;
            }
        }

        std::shared_ptr<msg_data> received_data()
        {
            if (_recv_data.empty())
            {
                return nullptr;
            }
            else
            {
                auto ret = _recv_data.front();
                _recv_data.pop_front();
                return ret;
            }
        }

        std::shared_ptr<std::vector<char>> sent_data()
        {
            if (_send_data.empty())
            {
                return nullptr;
            }
            else
            {
                auto ret = _send_data.front();
                _send_data.pop_front();
                return ret;
            }
        }

        void loopback_sent_message(stack_accessor &acc, const std::string &phy_addr = std::string("0.0.0.0"))
        {
            auto data   = sent_data();
            auto header = sent_header();

            /* Reformat header */
            char serial_header[msg_header::size()];
            header->serialise(serial_header);
            std::shared_ptr<msg_header> resp_header(new msg_header(serial_header, boost::asio::ip::address::from_string(phy_addr)));

            /* Reformat data */
            auto *resp_data = new char [header->fragment_length()];
            std::copy(data->begin(), data->end(), &resp_data[0]);

            _up_node->received(acc, std::shared_ptr<msg_data>(new msg_data(resp_data, header->fragment_length())), resp_header);
        }

        /* Virtual functions for sending and receiving messages */
        virtual void received(const stack_accessor &acc, const std::shared_ptr<msg_data> &data, const std::shared_ptr<msg_header> &header)
        {
            _recv_header.push_back(header);
            _recv_data.push_back(data);
        }

        virtual void send(const stack_accessor &acc, const std::shared_ptr<std::vector<char>> &data, const std::shared_ptr<msg_header> &header, boost::system::error_code &ec)
        {
            _send_header.push_back(header);
            _send_data.push_back(data);
        }

        /* Virtual function to clone as was constructed, must be threadsafe */
        virtual copied_pair clean_clone()
        {
            /* If this is the bottom of the stack emmulate a connection and return this */
            if (this->_dn_node == nullptr)
            {
                return copied_pair(this);
            }

            /* Clone this and down nodes */
            auto dn_pair = this->_dn_node->clean_clone();
            auto *cloned = new mock_stack_component(dn_pair.last());

            /* Update return pair */
            dn_pair.update(cloned);
            return dn_pair;
        }

        /* Virtual function to connect the upward links of the stack */
        virtual stack_component& build_up_links(void *const up_node) override
        {
            /* Conenctions dont up link directly */
            if (_conn)
            {
                return *this;
            }

            /* This cast will always be ok */
            this->_up_node = static_cast<stack_component *>(up_node);
            if (this->_dn_node != nullptr)
            {
                this->_dn_node->build_up_links(this);
            }

            return *this;
        }

        int size() { return _recv_header.size(); }

    private :
        std::deque<std::shared_ptr<msg_header>>         _recv_header;
        std::deque<std::shared_ptr<msg_header>>         _send_header;
        std::deque<std::shared_ptr<msg_data>>           _recv_data;
        std::deque<std::shared_ptr<std::vector<char>>>  _send_data;
        const bool                                      _conn;
};
}; /* namespace raptor_networking */

#endif /* #ifndef __MOCK_STACK_COMPONENT_H__ */
