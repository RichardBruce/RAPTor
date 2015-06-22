#pragma once

namespace raptor_networking
{
/* Pure virtual class to represent an application level component */
class data_receiver
{
    public :
        /* Virtual DTOR for inheritors */
        virtual ~data_receiver() { };

        /* Virtual function for receiving messages */
        virtual std::vector<char>* received(std::unique_ptr<std::istream> &&data) = 0;

        /* Virtual function to clone as was constructed, must be threadsafe */
        virtual data_receiver* clean_clone() = 0;
};
}; /* namespace raptor_networking */
