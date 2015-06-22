#pragma once

/* Standard headers */
#include <future>
#include <map>
#include <thread>

/* Networking headers */
#include "stack_component.h"
#include "stack_accessor.h"
#include "data_receiver.h"


namespace raptor_networking
{
/* Class to delivery the message into user code, this must be at the top of every stack */
class message_delivery : public stack_component
{
    public :
        message_delivery(stack_component *const dn_node, data_receiver *const receiver)
            : _dn_node(dn_node), _receiver(receiver) {  };

        ~message_delivery()
        {
            delete _receiver;
        }

        /* Virtual functions for sending and receiving messages */
        virtual void received(const stack_accessor &acc, const std::shared_ptr<msg_data> &data, const std::shared_ptr<msg_header> &header) override
        {
            /* Check if this result was promised anywhere */
            const std::uint32_t resp_to = header->response_to();
            auto awaited_iter = _awaited_resp.find(resp_to);
            if (awaited_iter != _awaited_resp.end())
            {
                awaited_iter->second.set_value(data->as_stream());
                _awaited_resp.erase(awaited_iter);
            }
            
            /* Let the application process the data */
            std::shared_ptr<std::vector<char>> resp(_receiver->received(std::unique_ptr<std::istream>(data->as_stream())));

            /* Send any response */
            if (resp != nullptr)
            {
                std::shared_ptr<msg_header> resp_header(new msg_header(header->from_address(), header->group_address(), acc.next_sequence(), resp->size(), header->sequence(), true));

                boost::system::error_code ignored;
                this->_dn_node->send(acc, resp, resp_header, ignored);
            }
        }

        virtual void send(const stack_accessor &acc, const std::shared_ptr<std::vector<char>> &data, const std::shared_ptr<msg_header> &header, boost::system::error_code &ec) override
        {
            /* Just pass down the stack */
            this->_dn_node->send(acc, data, header, ec);
        }

        /* Send overload that permits waiting for a response */
        void send(const stack_accessor &acc, const std::shared_ptr<std::vector<char>> &data, const std::shared_ptr<msg_header> &header, std::future<std::istream *const> *resp, boost::system::error_code &ec)
        {
            /* Just pass down the stack */
            const std::uint32_t seq = header->sequence();
            this->_dn_node->send(acc, data, header, ec);

            /* Save a promise for the response */
            if (!ec)
            {
                auto awaited_iter = _awaited_resp.insert(std::pair<std::uint32_t, std::promise<std::istream *const>>(seq, std::promise<std::istream *const>())).first;
                *resp = awaited_iter->second.get_future();
            }
        }

        /* Virtual function to clone as was constructed */
        virtual stack_component::copied_pair clean_clone() override
        {
            /* Clone this and down nodes */
            auto dn_pair = this->_dn_node->clean_clone();
            auto *cloned = new message_delivery(dn_pair.last(), _receiver->clean_clone());

            /* Update the pair */
            dn_pair.update(cloned);
            dn_pair.top(cloned);

            /* Link the cloned up links */
            cloned->build_up_links();

            return dn_pair;
        }

        /* Virtual function to connect the upward links of the stack */
        virtual stack_component& build_up_links(void *const sc) override
        {
            this->_dn_node->build_up_links(this);
            return *this;
        }

        /* build up links overload that doesnt require an input */
        stack_component& build_up_links()
        {
            return build_up_links(nullptr);
        }

        /* Access functions */
        size_t awaited_promises() const { return _awaited_resp.size(); }

    private :
        typedef std::map<std::uint32_t, std::promise<std::istream *const>> resp_map;

        /* NOTE -- One data receiver is held per stack. */
        /*         This user defined class can just be a pointer to another object if this behaviour isnt desirable. */
        /*         The data receiver will be called single threadly per stack. */
        /*         Any object pointed to may need synchronisation because there can be multiple stacks. */
        stack_component *const  _dn_node;       /* The next node down in the stack          */
        resp_map                _awaited_resp;  /* Map of sequence id to promised data      */
        data_receiver  *const   _receiver;      /* User defined class to handle a message   */
};
}; /* namespace raptor_networking */
