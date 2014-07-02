#ifndef __MOCK_DATA_RECEIVER_H__
#define __MOCK_DATA_RECEIVER_H__

/* Standard headers */
#include <deque>
#include <iostream>

/* Networking headers */
#include "data_receiver.h"
#include "msg_header.h"
#include "msg_data.h"


namespace raptor_networking
{
class mock_data_receiver : public data_receiver
{
    public :
        mock_data_receiver() : _resp_data(nullptr) {  };
        mock_data_receiver(const std::string *const resp_data) : _resp_data(resp_data) {  };

        std::unique_ptr<std::istream> received_data()
        {
            if (_recv_data.empty())
            {
                return nullptr;
            }
            else
            {
                auto ret(std::move(_recv_data.front()));
                _recv_data.pop_front();
                return ret;
            }
        }

        /* Virtual function for receiving messages */
        virtual std::vector<char>* received(std::unique_ptr<std::istream> &&data) override
        {
            /* Save data */
            _recv_data.emplace_back(std::move(data));

            /* Make automated response */
            if (_resp_data == nullptr)
            {
                return nullptr;
            }
            else
            {
                return new std::vector<char>(_resp_data->begin(), _resp_data->end());
            }
        }

        /* Virtual function to clone as was constructed, must be threadsafe */
        virtual data_receiver* clean_clone() override
        {
            /* Mock not used */
            return new mock_data_receiver();
        }

        /* Access functions */
        size_t data_received() const { return _recv_data.size(); }

        mock_data_receiver& set_auto_response(const std::string *const resp_data)
        {
            _resp_data.reset(resp_data);
            return *this;
        }

    private :
        std::deque<std::unique_ptr<std::istream>>   _recv_data;
        std::unique_ptr<const std::string>          _resp_data;
};
}; /* namespace raptor_networking */

#endif /* #ifndef __MOCK_DATA_RECEIVER_H__ */
