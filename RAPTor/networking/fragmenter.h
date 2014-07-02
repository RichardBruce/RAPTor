#ifndef __FRAGMENTER_H__
#define __FRAGMENTER_H__

/* Standard headers */
#include <unordered_map>

/* Networking headers */
#include "stack_component.h"


namespace raptor_networking
{
template<class UpNode, class DnNode>
class fixed_size_fragmenter : public stack_component_impl<UpNode, DnNode>
{
    public :
        fixed_size_fragmenter(DnNode *const dn_node, const size_t frag_size)
            : stack_component_impl<UpNode, DnNode>(dn_node),
              _frag_size(frag_size) { };

        ~fixed_size_fragmenter()
        {
            for (auto& msg : _msgs)
            {
                delete msg.second;
            }
        }

        /* Virtual functions for sending and receiving messages */
        virtual void received(const stack_accessor &acc, const std::shared_ptr<msg_data> &data, const std::shared_ptr<msg_header> &header) override
        {
            /* Check if there is any defragmentation to do */
            const std::uint32_t len = header->length();
            const size_t nr_frags   = static_cast<size_t>(std::ceil(len / static_cast<fp_t>(_frag_size)));
            if (nr_frags == 1)
            {
                this->_up_node->received(acc, data, header);
                return;
            }

            /* Attempt to insert new message */
            /* If the message is already being processed updated it */
            const std::uint32_t frag = header->fragment();
            auto msg_iter = _msgs.find(header->sequence());
            if (msg_iter != _msgs.end())
            {
                /* If complete pass the data up the stack and clean up */
                if (msg_iter->second->set_fragment(data, header, frag))
                {
                    /* Release locks in the message map before calling something else */
                    auto *frag_store = msg_iter->second;
                    _msgs.erase(msg_iter);

                    /* Fix up the header */
                    auto &defrag_header = frag_store->header();
                    defrag_header->fragment_length(defrag_header->length());

                    /* Pass up the stack */
                    this->_up_node->received(acc, frag_store->message(), defrag_header);

                    /* Clean  up */
                    delete frag_store;
                }
                /* Else accessor will be release by leaving scope */
            }
            /* Else store the message, we need to wait for other fragments */
            else
            {
                data->expand_slots(nr_frags, len);
                auto insert_iter = _msgs.insert({header->sequence(), new fragment_store(data)}).first;
                insert_iter->second->set_fragment(data, header, frag);
            }
        }
        
        virtual void send(const stack_accessor &acc, const std::shared_ptr<std::vector<char>> &data, const std::shared_ptr<msg_header> &header, boost::system::error_code &ec) override
        {
            /* Work out the number of fragments to send */
            const std::uint32_t size = header->length();
            const std::uint32_t nr_frags = static_cast<std::uint32_t>(std::ceil(size / static_cast<fp_t>(_frag_size)));
            if (nr_frags == 1)
            {
                this->_dn_node->send(acc, data, header, ec);
                return;
            }

            /* Fragment */
            for (std::uint32_t i = 0; i < nr_frags; ++i)
            {
                /* Write fragment into the header */
                const std::uint32_t pos = i * _frag_size;
                const std::uint32_t remaining_size = std::min(_frag_size, size - pos);

                /* Create new header */
                const std::shared_ptr<msg_header> frag_header(new msg_header(*header));
                frag_header->fragment_length(remaining_size);
                frag_header->fragment(i);

                /* Create new data */
                std::shared_ptr<std::vector<char>> frag_data(new std::vector<char>(&data->data()[pos], &data->data()[pos + remaining_size]));

                /* Call down the stack with each fragment */
                this->_dn_node->send(acc, frag_data, frag_header, ec);
                if (ec)
                {
                    return;
                }
            }
        }

        /* Virtual function to clone as was constructed */
        virtual stack_component::copied_pair clean_clone() override
        {
            /* Clone this and down nodes */
            auto dn_pair = this->_dn_node->clean_clone();
            auto *cloned = new fixed_size_fragmenter<UpNode, DnNode>(dn_pair.last(), _frag_size);

            /* Update return pair */
            dn_pair.update(cloned);
            return dn_pair;
        }

        /* Access functions */
        size_t          pending_messages()  const { return _msgs.size();    }
        std::uint32_t   fragment_size()     const { return _frag_size;      }

    private :
        /* Private class to hold a message as it is recomposed */
        /* Not thread safe */
        class fragment_store
        {
            public :
                fragment_store(const std::shared_ptr<msg_data> &data) : _msg(data), _header() {  };

                /* Add another fragment into the store. Returns true if the message has been completed, else false */
                bool set_fragment(const std::shared_ptr<msg_data> &data, const std::shared_ptr<msg_header> &header, const std::uint32_t frag)
                {
                    /* Set header when the correct fragment is found */
                    if (frag == 0)
                    {
                        _header = header;
                    }

                    /* Insert the data */
                    _msg->move(data, frag);

                    /* Check if the message has been completed */
                    return _msg->complete();
                }

                /* Convert the message fragments beack into a message */
                const std::shared_ptr<msg_data>& message()
                {
                    return _msg;
                }

                /* Get the header of the first fragment */
                const std::shared_ptr<msg_header>& header()
                {
                    return _header;
                }

            private :
                const std::shared_ptr<msg_data> _msg;       /* The fragments of the message     */
                std::shared_ptr<msg_header>     _header;    /* The header of the first fragment */
        };

        typedef std::unordered_map<std::uint32_t, fragment_store*> msg_map;
        msg_map             _msgs;      /* Received message buffers */
        const std::uint32_t _frag_size; /* The size of the fragment */
};
}; /* namespace raptor_networking */

#endif /* #ifndef __FRAGMENTER_H__ */
