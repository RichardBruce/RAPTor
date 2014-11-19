#ifndef __RATE_LIMITER_H__
#define __RATE_LIMITER_H__

/* Standard headers */
#include <chrono>

/* Boost headers */
#include "boost/asio/system_timer.hpp"

/* Networking headers */
#include "stack_component.h"


namespace raptor_networking
{
template<class UpNode, class DnNode>
class rate_limiter : public stack_component_impl<UpNode, DnNode>
{
    public :
        rate_limiter(DnNode *const dn_node, boost::asio::io_service &io_service, const float bytes_per_milli)
            : stack_component_impl<UpNode, DnNode>(dn_node), _timeout(io_service), _bytes_per_milli_inv(1.0f / bytes_per_milli)
              { };

        ~rate_limiter()
        {
            /* Free any threads waiting on the timer */
            _timeout.cancel();
        }

        /* Virtual functions for sending and receiving messages */
        virtual void received(const stack_accessor &acc, const std::shared_ptr<msg_data> &data, const std::shared_ptr<msg_header> &header) override
        {
            /* Pass the data up the stack */
            this->_up_node->received(acc, data, header);
        }

        virtual void send(const stack_accessor &acc, const std::shared_ptr<std::vector<char>> &data, const std::shared_ptr<msg_header> &header, boost::system::error_code &ec) override
        {
            /* Wait until rate has been met from last message */
            if (_timeout.expires_at() > std::chrono::system_clock::now())
            {
                _timeout.wait();
            }

            /* Increase the timer by the time required for this message */
            const float bytes   = header->fragment_length();
            const float millis  = bytes * _bytes_per_milli_inv;
            _timeout.expires_at(std::chrono::system_clock::now() + std::chrono::milliseconds(static_cast<int>(millis)));

            /* Pass the data down the stack */
            this->_dn_node->send(acc, data, header, ec);
        }

        /* Virtual function to clone as was constructed */
        virtual stack_component::copied_pair clean_clone() override
        {
            /* Clone this and down nodes */
            auto dn_pair = this->_dn_node->clean_clone();
            auto *cloned = new rate_limiter<UpNode, DnNode>(dn_pair.last(), _timeout.get_io_service(), 1.0f / _bytes_per_milli_inv);

            /* Update return pair */
            dn_pair.update(cloned);
            return dn_pair;
        }

        /* Access fucntions */
        float rate() const { return 1.0f / _bytes_per_milli_inv; }

    private :
        boost::asio::system_timer   _timeout;
        const float                 _bytes_per_milli_inv;
};
}; /* namespace raptor_networking */

#endif /* #ifndef __RATE_LIMITER_H__ */
